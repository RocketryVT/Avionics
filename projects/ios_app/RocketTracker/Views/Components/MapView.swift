//
//  MapViews.swift
//  RocketTracker
//
//  Created by Gregory Wainer on 5/14/25.
//

import SwiftUI
import MapKit
import MapboxMaps

func getDeviceColor(for deviceID: UInt32) -> Color {
    let colors: [Color] = [.red, .blue, .green, .orange, .purple, .brown, .cyan]
    return colors[Int(deviceID) % colors.count]
}

func getUIDeviceColor(for deviceID: UInt32) -> UIColor {
    let colors: [UIColor] = [.red, .blue, .green, .orange, .purple, .brown, .cyan]
    return colors[Int(deviceID) % colors.count]
}

@MainActor
func MapView(for userLocation: CLLocationCoordinate2D?, for telemetry: TelemetryData?, presenter: MainPresenter) -> some View {
    
    /// Return Map of first rocket location
    if let telemetry {
        let mapConfig = MapConfig(
            center: CLLocationCoordinate2D(latitude: telemetry.gps.lat, longitude: telemetry.gps.lon),
            zoomLevel: 15,
            showsUserLocation: true // Always show user location regardless of tracking mode
        )
        return MapboxMapView(
            config: mapConfig,
            presenter: presenter
        )
    }
    
    /// If Rocket not connected then use the users current location
    if let userLocation {
        let mapConfig = MapConfig(
            center: CLLocationCoordinate2D(latitude: userLocation.latitude, longitude: userLocation.longitude),
            zoomLevel: 15,
            showsUserLocation: true // Always show user location regardless of tracking mode
        )
        return MapboxMapView(
            config: mapConfig,
            presenter: presenter
        )
    }
    
    print("Defaulting to USA overview map")
    /// Return overview map of the USA
    let mapConfig = MapConfig(
        center: CLLocationCoordinate2D(latitude: 39.5, longitude: -98.0),
        zoomLevel: 2,
        showsUserLocation: true // Always show user location regardless of tracking mode
    )
    return MapboxMapView(
        config: mapConfig,
        presenter: presenter
    )
    
}

struct MapConfig {
    var center: CLLocationCoordinate2D
    var zoomLevel: Double
    var showsUserLocation: Bool = true
}

struct MapboxMapView: UIViewRepresentable {
    var config: MapConfig
    var tileStore: TileStore?
    @ObservedObject var presenter: MainPresenter
    
    func makeUIView(context: Context) -> MapboxMaps.MapView {
        let styleURI = StyleURI(rawValue: "mapbox://styles/gregoryw3/cmblpdzrd00gn01s2cnjre4bj") ?? StyleURI.satelliteStreets
        let options = MapInitOptions(
            mapOptions: MapOptions(
                constrainMode: .none,
                viewportMode: .default
            ),
            cameraOptions: CameraOptions(
                center: config.center, 
                zoom: config.zoomLevel,
            ),
            styleURI: styleURI
        )
        
        let mapView = MapboxMaps.MapView(frame: .zero, mapInitOptions: options)
        var puckConfiguration = Puck2DConfiguration.makeDefault(showBearing: true)

        let rectangleCoords = [
            CLLocationCoordinate2D(latitude: 31.0, longitude: -103.6),
            CLLocationCoordinate2D(latitude: 31.0, longitude: -103.5),
            CLLocationCoordinate2D(latitude: 31.1, longitude: -103.5),
            CLLocationCoordinate2D(latitude: 31.1, longitude: -103.6),
            CLLocationCoordinate2D(latitude: 31.0, longitude: -103.6)
        ]
        let polygon = Polygon([rectangleCoords])
        let geometry = Geometry(polygon)

        let offlineManager = OfflineManager()
        let tilesetDescriptorOptions = TilesetDescriptorOptions(
            styleURI: styleURI,
            zoomRange: 0...16,
            tilesets: ["gregoryw3.cmbmg8u9c7kfc1nup1yceq85z-128d1"]
        )
        let tilesetDescriptor = offlineManager.createTilesetDescriptor(for: tilesetDescriptorOptions)

        let tileRegionLoadOptions = TileRegionLoadOptions(
            geometry: geometry,
            descriptors: [tilesetDescriptor],
            acceptExpired: true
        )
        
        // Always show location indicator (blue dot) regardless of tracking mode
        puckConfiguration.layerPosition = .default
        mapView.location.options.puckType = .puck2D(puckConfiguration)
        mapView.location.options.puckBearing = .heading
        mapView.location.options.puckBearingEnabled = true

        // self.tileStore?.loadTileRegion(
        //     forId: "my-offline-region",
        //     loadOptions: tileRegionLoadOptions,
        //     progress: { progress in
        //         print("Offline region progress: \(progress.completedResourceCount)/\(progress.requiredResourceCount)")
        //     },
        //     completion: { result in
        //         switch result {
        //         case .success(let region):
        //             print("Successfully loaded tile region: \(region)")
        //         case .failure(let error):
        //             print("Failed to load tile region: \(error)")
        //         }
        //     }
        // )
        
        // mapView.mapboxMap.loadStyle(.outdoors)
        // mapView.mapboxMap.loadStyleURI(StyleURI(url: "mapbox://styles/gregoryw3/cmblpdzrd00gn01s2cnjre4bj"))
        
        context.coordinator.mapView = mapView
        
        return mapView
    }
    
    func updateUIView(_ uiView: MapboxMaps.MapView, context: Context) {
        // Update device paths, markers, etc.
        for deviceID in presenter.getAvailableDeviceIDs() {
            if let telemetry = presenter.getTelemetryData(for: deviceID) {
                // Update or add marker for this device
                context.coordinator.updateDeviceMarker(deviceID: deviceID, 
                                                     telemetry: telemetry, 
                                                     on: uiView)
                
                // Update or add path for this device
                let coordinates = presenter.getPathCoordinates(for: deviceID)
                context.coordinator.updateDevicePath(deviceID: deviceID, 
                                                   coordinates: coordinates,
                                                   on: uiView)
            }
        }
    }

    func makeCoordinator() -> Coordinator {
        Coordinator(self)
    }

    class Coordinator: NSObject {
        var parent: MapboxMapView
        var mapView: MapboxMaps.MapView?
        var pointAnnotationManagers: [UInt32: PointAnnotationManager] = [:] // Track managers by device ID
        
        init(_ parent: MapboxMapView) {
            self.parent = parent
        }
        
        func updateDeviceMarker(deviceID: UInt32, telemetry: TelemetryData, on mapView: MapboxMaps.MapView) {
            let pointAnnotationManager: PointAnnotationManager
            if let existingManager = pointAnnotationManagers[deviceID] {
                pointAnnotationManager = existingManager
            } else {
                pointAnnotationManager = mapView.annotations.makePointAnnotationManager()
                pointAnnotationManagers[deviceID] = pointAnnotationManager
            }
            
            // Create a unique ID for this device's marker
            let id = "device-\(deviceID)"

            // Clear existing annotations for this manager
            pointAnnotationManager.annotations.removeAll()

            // Create new marker
            var point = PointAnnotation(id: id, coordinate: CLLocationCoordinate2D(
                latitude: telemetry.gps.lat,
                longitude: telemetry.gps.lon
            ))
            
            // Style the marker with device-specific color
            let color = getUIDeviceColor(for: deviceID)
            point.iconImage = "rocket" // Make sure you add this image to your assets
            point.iconColor = StyleColor(color)
            point.iconSize = 1.5
            point.textField = "Device \(deviceID)"
            point.textColor = StyleColor(.white)
            point.textHaloColor = StyleColor(.black)
            point.textHaloWidth = 1.0
            
            // Add to map
            pointAnnotationManager.annotations.append(point)
        }

        func updateDevicePath(deviceID: UInt32, coordinates: [CLLocationCoordinate2D], on mapView: MapboxMaps.MapView) {
            if coordinates.count < 2 { return }
            
            // Source ID for this device's path
            let sourceID = "path-source-\(deviceID)"
            let layerID = "path-layer-\(deviceID)"
            
            // Remove existing source and layer if they exist
            if mapView.mapboxMap.layerExists(withId: layerID) {
                try? mapView.mapboxMap.removeLayer(withId: layerID)
            }
            if mapView.mapboxMap.sourceExists(withId: sourceID) {
                try? mapView.mapboxMap.removeSource(withId: sourceID)
            }
            
            // Create a feature for the path
            let feature = Feature(geometry: .lineString(LineString(coordinates)))
            
            // Create a source with the feature
            var source = GeoJSONSource(id: sourceID)
            source.data = .feature(feature)
            
            // Add the source to the map
            try? mapView.mapboxMap.addSource(source)
            
            // Create a line layer
            var lineLayer = LineLayer(id: layerID, source: sourceID)
            // Add the source ID
            lineLayer.source = sourceID
            
            // Style based on the device color
            let color = getUIDeviceColor(for: deviceID)
            lineLayer.lineColor = Value.constant(StyleColor(color))
            lineLayer.lineWidth = Value.constant(3.0)
            
            // Add the layer
            try? mapView.mapboxMap.addLayer(lineLayer)
        }
    }
}

// Custom polyline with device ID information
class DevicePolyline: MKPolyline {
    var deviceID: UInt32 = 0
    var colorIndex: Int = 0
    var isDashed: Bool = false
}

// Custom annotation with device ID information
class DeviceAnnotation: NSObject, MKAnnotation {
    var coordinate: CLLocationCoordinate2D
    var deviceID: UInt32
    var colorIndex: Int
    
    init(coordinate: CLLocationCoordinate2D, deviceID: UInt32, colorIndex: Int) {
        self.coordinate = coordinate
        self.deviceID = deviceID
        self.colorIndex = colorIndex
    }
    
    var title: String? {
        return "Device \(deviceID)"
    }
}

// Placeholder map when not connected
var placeholderMapView: some View {
    Group {
        if #available(iOS 17.0, *) {
            // iOS 17+ implementation
            ZStack {
                Color(Color.secondary.opacity(0.1))
                VStack {
                    Image(systemName: "map")
                        .font(.system(size: 50))
                        .foregroundColor(.gray)
                    Text("Connect to Rocket Tracker to view location")
                        .foregroundColor(.gray)
                }
            }
            .frame(height: 300)
        } else {
            // iOS 15-16 implementation
            PlaceholderMapView()
        }
    }
}

// iOS 15-16 placeholder view implementation
struct PlaceholderMapView: View {
    var body: some View {
        ZStack {
            Color.secondary.opacity(0.1)
            VStack {
                Image(systemName: "map")
                    .font(.system(size: 50))
                    .foregroundColor(.gray)
                Text("Connect to Rocket Tracker to view location")
                    .foregroundColor(.gray)
            }
        }
        .frame(height: 300)
    }
}
