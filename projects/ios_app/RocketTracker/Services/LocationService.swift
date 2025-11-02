//
//  LocationService.swift
//  RocketTracker
//
//  Created by Gregory Wainer on 5/22/25.
//

import CoreLocation
import Combine

protocol LocationServiceProtocol {
    var userLocation: CLLocationCoordinate2D? { get }
    var userLocationPublisher: AnyPublisher<CLLocationCoordinate2D?, Never> { get }

    func setupLocationServices()
    func startUpdatingLocation()
    func stopUpdatingLocation()
}

class LocationService: NSObject, LocationServiceProtocol, ObservableObject, CLLocationManagerDelegate {
    private var locationManager: CLLocationManager
    
    @Published var userLocation: CLLocationCoordinate2D?
    @Published var deviceHeading: Double = 0
    
    func setupLocationServices() {
        locationManager.delegate = self
        locationManager.desiredAccuracy = kCLLocationAccuracyBestForNavigation
        locationManager.requestWhenInUseAuthorization()
        locationManager.allowsBackgroundLocationUpdates = false
        locationManager.activityType = .otherNavigation
        locationManager.pausesLocationUpdatesAutomatically = true
    }

    var userLocationPublisher: AnyPublisher<CLLocationCoordinate2D?, Never> {
        return $userLocation.eraseToAnyPublisher()
    }
    
    func startUpdatingLocation() {
//        locationManager.startMonitoringSignificantLocationChanges()
        locationManager.startUpdatingLocation()
        locationManager.startUpdatingHeading()
    }
    
    func stopUpdatingLocation() {
        locationManager.stopUpdatingLocation()
        locationManager.stopUpdatingHeading()
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        if let location = locations.last?.coordinate {
            userLocation = location
        }
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        // Get magnetic heading (direction device is pointing)
        deviceHeading = newHeading.magneticHeading
    }
    
    override init() {
        locationManager = CLLocationManager()
        super.init()
        setupLocationServices()
        startUpdatingLocation()
    }
    
}
