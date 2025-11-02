//
//  ContentView.swift
//  RocketryAtVT
//
//  Created by Gregory Wainer on 2/4/25.
//

import SwiftUI
import MapKit

struct ContentView: View {
    @StateObject private var presenter: MainPresenter
    @State private var showDeviceSelector: Bool = false

    init() {
        // Create the service and presenter
        let bluetoothService = BluetoothService()
        let locationService = LocationService()
        _presenter = StateObject(wrappedValue: MainPresenter(bluetoothService: bluetoothService, locationService: locationService))
    }

    var body: some View {
        if #available(iOS 16.0, *) {
            // iOS 16+ implementation using NavigationStack
            NavigationStack {
                contentView
            }
        } else {
            // iOS 15 implementation using NavigationView
            NavigationView {
                contentView
            }
            .navigationViewStyle(.stack)
        }
    }

    private var contentView: some View {
        GeometryReader { geometry in
            VStack(spacing: 0) {
                // Header with connection status
                connectionHeader
                mapSection(geometry)
                TelemetryDataView(presenter: presenter)
                
                // // Adapt layout based on size class (horizontal compact vs regular)
                // if geometry.size.width > 500 && geometry.size.height > 500 {
                //     // Wide layout for iPad/large screens - side by side
                //     HStack(alignment: .top) {
                //         mapSection(geometry)
                //             .frame(width: geometry.size.width * 0.6)
                
                //         TelemetryDataView(presenter: presenter)
                //             .frame(width: geometry.size.width * 0.4)
                //     }
                // } else {
                //     // Vertical layout for smaller screens or split view
                //     mapSection(geometry)
                //     TelemetryDataView(presenter: presenter)
                // }
            }
            .navigationTitle("Rocket Tracker")
            .toolbar {
                ToolbarItem(placement: .automatic) {
                    Button(action: { showDeviceSelector = true }) {
                        Label("Connect", systemImage: "antenna.radiowaves.left.and.right")
                    }
                }
                
                ToolbarItem(placement: .automatic) {
                    if presenter.isConnected {
                        Button(action: { presenter.disconnect() }) {
                            Label("Disconnect", systemImage: "xmark.circle")
                        }
                    }
                }
            }
            .sheet(isPresented: $showDeviceSelector) {
                DeviceSelectorView(
                    presenter: presenter,
                    isPresented: $showDeviceSelector
                )
            }
        }
    }

    private func getDeviceColor(for deviceID: UInt32) -> Color {
        let colors: [Color] = [.red, .blue, .green, .orange, .purple, .brown, .cyan]
        return colors[Int(deviceID) % colors.count]
    }
    
    // Connection status header
    private var connectionHeader: some View {
        HStack {
            Image(systemName: presenter.isConnected ? "checkmark.circle.fill" : "xmark.circle.fill")
                .foregroundColor(presenter.isConnected ? .green : .red)
            
            Text(presenter.isConnected ? "Connected to RocketryAtVT Tracker" : "Not Connected")
                .font(.headline)
            
            // Spacer()
            
            if presenter.isConnected, presenter.isSending {
                ProgressView()
                    .progressViewStyle(CircularProgressViewStyle())
                    .padding(.trailing)
            }
        }
        .padding(4)
        .background(Color.secondary.opacity(0.0))
    }

    private func mapSection(_ geometry: GeometryProxy) -> some View {
        Group {
            if let telemetry = presenter.telemetryData {
                ZStack(alignment: .topTrailing) {
                    MapView(for: nil, for: telemetry, presenter: presenter)
                         .frame(height: min(300, geometry.size.height * 0.5))
                         .padding(8)
                }
            } else if let userLocation = presenter.userLocation {
                ZStack(alignment: .topTrailing) {
                    MapView(for: userLocation, for: nil, presenter: presenter)
                         .frame(height: min(300, geometry.size.height * 0.5))
                         .padding(8)
                }
            } else {
                ZStack(alignment: .topTrailing) {
                    MapView(for: nil, for: nil, presenter: presenter)
                        .frame(height: min(300, geometry.size.height * 0.5))
                        .padding(8)
                }
            }
        }
    }

    private func calculateDistance(from userLocation: CLLocationCoordinate2D?, to rocketLocation: CLLocationCoordinate2D) -> Double? {
        guard let userLocation = userLocation else { return nil }
        
        let userCLLocation = CLLocation(latitude: userLocation.latitude, longitude: userLocation.longitude)
        let rocketCLLocation = CLLocation(latitude: rocketLocation.latitude, longitude: rocketLocation.longitude)
        
        return userCLLocation.distance(from: rocketCLLocation) // Returns distance in meters
    }
}

#Preview {
    ContentView()
}
