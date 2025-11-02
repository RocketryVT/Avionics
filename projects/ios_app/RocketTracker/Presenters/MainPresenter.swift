import Foundation
import Combine
import CoreBluetooth
import MapKit

class MainPresenter: NSObject, ObservableObject {
    // Published properties for the view
    @Published var isConnected = false
    @Published var telemetryDataByDevice: [UInt32: TelemetryData] = [:]
    @Published var receivedMessages: [ReceivedMessage] = []
    @Published var isSending = false
    @Published var pathCoordinatesByDevice: [UInt32: [CLLocationCoordinate2D]] = [:]
    @Published var mapRegion = MKCoordinateRegion(
        center: CLLocationCoordinate2D(latitude: 37.0, longitude: -80.0),
        span: MKCoordinateSpan(latitudeDelta: 0.01, longitudeDelta: 0.01)
    )
    @Published var selectedDeviceID: UInt32? = nil // Track which device is selected
    
    @Published var userLocation: CLLocationCoordinate2D?
    @Published var headingToRocket: Double?
    @Published var deviceHeading: Double = 0
    @Published var relativeHeadingToRocket: Double?
    @Published var selectedDate: Date? = nil

    var telemetryData: TelemetryData? {
        if let selected = selectedDeviceID {
            return telemetryDataByDevice[selected]
        }
        return telemetryDataByDevice.values.first
    }

    var pathCoordinates: [CLLocationCoordinate2D] {
        if let selected = selectedDeviceID {
            return pathCoordinatesByDevice[selected] ?? []
        }
        return pathCoordinatesByDevice.values.first ?? []
    }
    
    // Services
    private let bluetoothService: BluetoothServiceProtocol
    private let locationService: LocationServiceProtocol
    private let dataManager = TelemetryDataManager()
    private var cancellables = Set<AnyCancellable>()
    
    init(bluetoothService: BluetoothServiceProtocol, locationService: LocationServiceProtocol) {
        self.bluetoothService = bluetoothService
        self.locationService = locationService

        super.init()
        
        dataManager.verifyDataStore()

        setupBindings()
    }
    
    // Get telemetry records for specific device
    func getAllTelemetryRecords() -> [TelemetryRecord] {
        return dataManager.getTelemetryRecords()
    }
    
    func getCurrentLocation() -> CLLocationCoordinate2D? {
        return locationService.userLocation
    }
    
    // Get available dates for specific device
    func getAvailableDates(forDeviceID deviceID: UInt32? = nil) -> [Date] {
        return dataManager.getAvailableDates(forDeviceID: deviceID)
    }
    
    // Delete records for specific device and date
    func deleteRecordsForDate(_ date: Date, deviceID: UInt32? = nil) {
        dataManager.deleteRecordsForDate(date, deviceID: deviceID)
    }
    
    private func setupBindings() {
        // Subscribe to service publishers
        bluetoothService.connectionStatusPublisher
            .assign(to: &$isConnected)
        
        bluetoothService.telemetryPublisher
            .sink { [weak self] telemetry in
                guard let self = self, let data = telemetry else { return }
                
                // Store data for this device
                self.telemetryDataByDevice[data.deviceID] = telemetry
                
                // Log the telemetry data
                self.dataManager.logTelemetry(data)
                
                // If this is a new device, select it
                if self.selectedDeviceID == nil {
                    self.selectedDeviceID = data.deviceID
                }
                
                // Update path coordinates for this device
                let coordinate = CLLocationCoordinate2D(
                    latitude: data.gps.lat,
                    longitude: data.gps.lon
                )
                
                if self.pathCoordinatesByDevice[data.deviceID] == nil {
                    self.pathCoordinatesByDevice[data.deviceID] = []
                }
                
                // Add the coordinate to the path
                self.pathCoordinatesByDevice[data.deviceID]?.append(coordinate)
                
                // Limit path length to prevent memory issues
                if let pathCount = self.pathCoordinatesByDevice[data.deviceID]?.count,
                pathCount > 1000 {
                    self.pathCoordinatesByDevice[data.deviceID]?.removeFirst()
                }
                
                // Update headings
                // self.updateHeadingsForAllDevices()
                
                // Center map on selected device
                if let selectedID = self.selectedDeviceID,
                let selectedTelemetry = self.telemetryDataByDevice[selectedID] {
                    self.mapRegion.center = CLLocationCoordinate2D(
                        latitude: selectedTelemetry.gps.lat,
                        longitude: selectedTelemetry.gps.lon
                    )
                }
            }
            .store(in: &cancellables)
        
        bluetoothService.messagesPublisher
            .assign(to: &$receivedMessages)

        locationService.userLocationPublisher
            .sink { [weak self] location in
                guard let self = self else { return }
                
                // Update our published property
                self.userLocation = location

//                print("User location updated: \(String(describing: location))")
                
                // If we're connected to a device, send the location
                if self.bluetoothService.isConnected, let location = location {
//                    print("Sending location update: \(location.latitude), \(location.longitude)")
                    self.bluetoothService.sendUserLocation(location)
                }
            }
            .store(in: &cancellables)
    }

    // Select a specific device to focus on
    func selectDevice(_ deviceID: UInt32?) {
        self.selectedDeviceID = deviceID
        
        // Update map region if we have data for this device
        if let deviceID = deviceID, 
        let telemetry = telemetryDataByDevice[deviceID] {
            mapRegion.center = CLLocationCoordinate2D(
                latitude: telemetry.gps.lat,
                longitude: telemetry.gps.lon
            )
        }
    }

    // Get all device IDs with telemetry data
    func getAvailableDeviceIDs() -> [UInt32] {
        // Combine current devices and historical devices
        var deviceIDs = Array(telemetryDataByDevice.keys)
        
        // Add historical devices from database
        let historicalDeviceIDs = dataManager.getAllDeviceIDs()
        for deviceID in historicalDeviceIDs {
            if !deviceIDs.contains(deviceID) {
                deviceIDs.append(deviceID)
            }
        }
        
        return deviceIDs.sorted()
    }

    func getAllDeviceIDs() -> [UInt32] {
        return dataManager.getAllDeviceIDs()
    }

    // Access methods for telemetry by device
    func getTelemetryRecords(deviceID: UInt32? = nil, from startDate: Date? = nil, to endDate: Date? = nil) -> [TelemetryRecord] {
        return dataManager.getTelemetryRecords(deviceID: deviceID, from: startDate, to: endDate)
    }

    func getPathCoordinates(for deviceID: UInt32) -> [CLLocationCoordinate2D] {
        return pathCoordinatesByDevice[deviceID] ?? []
    }
    
    // Public methods for the view
    func startScanning() {
        bluetoothService.startScanning()
    }
    
    func stopScanning() {
        bluetoothService.stopScanning()
    }
    
    func connect(to peripheral: CBPeripheral) {
        bluetoothService.connect(to: peripheral)
    }
    
    func disconnect() {
        bluetoothService.disconnect()
        // Force any pending saves
        self.dataManager.forceSave()
    }
    
    func getDiscoveredDevices() -> [(peripheral: CBPeripheral, rssi: NSNumber)] {
        // This could come from a published property as well
        return (bluetoothService as? BluetoothService)?.discoveredDevices ?? []
    }

    func getTelemetryData() -> TelemetryData? {
        return self.telemetryData
    }
    
    func getTelemetryData(for deviceID: UInt32) -> TelemetryData? {
        return self.telemetryDataByDevice[deviceID]
    }

    func getRecordsForSelectedDate() -> [TelemetryRecord] {
        guard let selectedDate = selectedDate else {
            return []
        }
        
        let deviceID = selectedDeviceID
        
        let calendar = Calendar.current
        let startOfDay = calendar.startOfDay(for: selectedDate)
        let endOfDay = calendar.date(byAdding: .day, value: 1, to: startOfDay)!
        
        return dataManager.getTelemetryRecords(deviceID: deviceID, 
                                            from: startOfDay, 
                                            to: endOfDay)
    }

    func getAvailableDates() -> [Date] {
        let dates = dataManager.getAvailableDates()
        print("Retrieved \(dates.count) available dates")
        return dates
    }

    func deleteOldRecords(olderThan date: Date) {
        dataManager.deleteRecords(olderThan: date)
    }

    func deleteRecordsForDate(_ date: Date) {
        print("MainPresenter: Deleting records for \(date)")
        dataManager.deleteRecordsForDate(date)
    }
}

extension Double {
    func toRadians() -> Double {
        return self * .pi / 180
    }
    
    func toDegrees() -> Double {
        return self * 180 / .pi
    }
}
