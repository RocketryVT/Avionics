//
//  BluetoothService.swift
//  RocketTracker
//
//  Created by Gregory Wainer on 5/14/25.
//

import CoreBluetooth
import Combine
import SwiftProtobuf
import MapKit

struct PacketKey: Hashable {
    let deviceID: UInt32
    let msgNum: UInt32
}

class PartialTelemetryData {
    let deviceID: UInt32
    let msgNum: UInt32
    let timeSinceBoot: UInt64
    var receivedTime = Date()
    
    // Track which packet types we've received
    var hasGps = false
    var hasIsmPrimary = false
    var hasIsmSecondary = false
    var hasLsm = false
    var hasAccel = false
    var hasBaro = false
    
    // Data storage
    var gpsData: RKTGpsData?
    var ismPrimaryData: RKTImuData?
    var ismSecondaryData: RKTImuData?
    var lsmData: RKTImuData?
    var accelData: RKTAccelData?
    var baroData: RKTBaroData?
    
    init(deviceID: UInt32, msgNum: UInt32, timeSinceBoot: UInt64) {
        self.deviceID = deviceID
        self.msgNum = msgNum
        self.timeSinceBoot = timeSinceBoot
    }
    
    var isComplete: Bool {
        // For now, wait longer (3 seconds) to allow all sensors to arrive
        let hasTimeout = Date().timeIntervalSince(receivedTime) > 3.0
        
        // Either we have all data, or we have GPS and waited long enough
        return (hasGps && hasIsmPrimary && hasIsmSecondary && hasLsm && hasAccel && hasBaro) || 
            (hasTimeout)
    }
    
    // Convert partial data to a complete TelemetryData object

    func toTelemetryData() -> TelemetryData {
        return TelemetryData(
            deviceID: deviceID,
            time_since_boot: Int(timeSinceBoot),
            msg_num: Int(msgNum),
            gps: GPSData(
                alt: gpsData?.alt ?? 0,
                fix: gpsFix(from: gpsData?.fixType ?? .unknownGpsfix), // Use your helper function to convert enum to String
                lat: gpsData?.lat ?? 0,
                lon: gpsData?.lon ?? 0,
                num_sats: Int(gpsData?.numSats ?? 0), // Convert to Int
                time: UTCTime(
                    itow: Int(gpsData?.itow ?? 0),
                    time_accuracy_estimate_ns: Int(gpsData?.timeAccuracyEstimateNs ?? 0),
                    year: Int(gpsData?.year ?? 0),  // Changed order to match expected parameters
                    month: Int(gpsData?.month ?? 0),
                    day: Int(gpsData?.day ?? 0),
                    hour: Int(gpsData?.hour ?? 0),
                    min: Int(gpsData?.min ?? 0),
                    sec: Int(gpsData?.sec ?? 0),
                    nanos: Int(gpsData?.nanos ?? 0), // Changed position in parameter list
                    valid: Int(gpsData?.valid ?? 0)
                )
            ),
            ism_primary: AccelGyroData( // Changed from AccelerometerData to AccelGyroData
                accelerometer: AccelerometerData(
                    x: ismPrimaryData?.accelX ?? 0,
                    y: ismPrimaryData?.accelY ?? 0,
                    z: ismPrimaryData?.accelZ ?? 0
                ),
                gyroscope: GyroscopeData(
                    x: ismPrimaryData?.gyroX ?? 0,
                    y: ismPrimaryData?.gyroY ?? 0,
                    z: ismPrimaryData?.gyroZ ?? 0
                )
            ),
            ism_secondary: AccelGyroData(
                accelerometer: AccelerometerData(
                    x: ismSecondaryData?.accelX ?? 0,
                    y: ismSecondaryData?.accelY ?? 0,
                    z: ismSecondaryData?.accelZ ?? 0
                ),
                gyroscope: GyroscopeData(
                    x: ismSecondaryData?.gyroX ?? 0,
                    y: ismSecondaryData?.gyroY ?? 0,
                    z: ismSecondaryData?.gyroZ ?? 0
                )
            ),
            lsm: AccelGyroData(
                accelerometer: AccelerometerData(
                    x: lsmData?.accelX ?? 0,
                    y: lsmData?.accelY ?? 0,
                    z: lsmData?.accelZ ?? 0
                ),
                gyroscope: GyroscopeData(
                    x: lsmData?.gyroX ?? 0,
                    y: lsmData?.gyroY ?? 0,
                    z: lsmData?.gyroZ ?? 0
                )
            ),
            adxl: AccelerometerData( // Changed from AccelGyroData to AccelerometerData
                x: accelData?.accelX ?? 0,
                y: accelData?.accelY ?? 0,
                z: accelData?.accelZ ?? 0
            ),
            barometer: BarometerData(
                altitude: baroData?.altitude ?? 0
            )
        )
    }
    
    // Helper function to convert protobuf GPS fix type to app's format
    private func gpsFix(from fixType: RKTGpsFix) -> String {
        switch fixType {
        case .unknownGpsfix:
            return "Unknown"
        case .noFix:
            return "No Fix"
        case .deadReckoningOnly:
            return "Dead Reckoning"
        case .fix2D:
            return "2D Fix"
        case .fix3D:
            return "3D Fix"
        case .gpsPlusDeadReckoning:
            return "GPS+DR"
        case .timeOnlyFix:
            return "Time Only"
        case .UNRECOGNIZED(_):
            return "Unrecognized Fix"
        @unknown default:
            return "Unknown"
        }
    }
}

protocol BluetoothServiceProtocol {
    var discoveredDevicesPublisher: Published<[(peripheral: CBPeripheral, rssi: NSNumber)]>.Publisher { get }
    var connectionStatusPublisher: Published<Bool>.Publisher { get }
    var telemetryPublisher: Published<TelemetryData?>.Publisher { get }
    var messagesPublisher: Published<[ReceivedMessage]>.Publisher { get }

    var isConnected: Bool { get }
    
    func startScanning()
    func stopScanning()
    func connect(to peripheral: CBPeripheral)
    func disconnect()
    func sendUserLocation(_ location: CLLocationCoordinate2D)
}

class BluetoothService: NSObject, BluetoothServiceProtocol, ObservableObject {
    @Published private(set) var discoveredDevices: [(peripheral: CBPeripheral, rssi: NSNumber)] = []
    @Published private(set) var telemetryDataByDevice: [UInt32: TelemetryData] = [:]
    @Published private(set) var isConnected = false
    @Published private(set) var latestTelemetry: TelemetryData?
    @Published private(set) var receivedMessages: [ReceivedMessage] = []
    @Published var isSending = false
    private var isLoggingEnabled = false
    
    var discoveredDevicesPublisher: Published<[(peripheral: CBPeripheral, rssi: NSNumber)]>.Publisher { $discoveredDevices }
    var connectionStatusPublisher: Published<Bool>.Publisher { $isConnected }
    var telemetryPublisher: Published<TelemetryData?>.Publisher { $latestTelemetry }
    var messagesPublisher: Published<[ReceivedMessage]>.Publisher { $receivedMessages }
    
    // MARK: - Private Properties
    private var centralManager: CBCentralManager!
    private var connectedDevice: CBPeripheral?
    
    private var trackerPacketsCharacteristic: CBCharacteristic?
    private var userLocationCharacteristic: CBCharacteristic?

    private var partialDataCache = [PacketKey: PartialTelemetryData]()
    private let partialDataTimeout: TimeInterval = 2.0
    private var cleanupTimer: Timer?
    private var telemetryLogger: DataLogger?


    // UUIDs
    private let serviceUUID = CBUUID(string: "0000FB34-9B5F-8000-0080-001000003412")
    private let trackerPacketsUUID = CBUUID(string: "00000000-0000-0000-0000-000000000001")
    private let userLocationUUID = CBUUID(string: "00000000-0000-0000-0000-000000000002")

    private struct TelemetryPacketBuffer {
        var trackerPacket: RKTTrackerPacket?
        var ismPrime: RKTImuData?
        var ismSec: RKTImuData?
        var lsm: RKTImuData?
        var adxl: RKTAccelData?
        var barometer: RKTBaroData?
        var lastUpdate: Date = Date()
    }

    private var telemetryBuffers: [UInt32: TelemetryPacketBuffer] = [:] // Keyed by msg_num
    
    // MARK: - Initialization
    override init() {
        super.init()
        centralManager = CBCentralManager(delegate: self, queue: nil)
        setupCleanupTimer()
    }
    
    // MARK: - Public Methods
    
    /// Start scanning for Bluetooth devices
    func startScanning() {
        if centralManager.state == .poweredOn {
            discoveredDevices = []
            centralManager.scanForPeripherals(withServices: nil, options: nil)
            print("Started scanning for devices")
        } else {
            print("Bluetooth is not powered on")
        }
    }
    
    /// Stop scanning for Bluetooth devices
    func stopScanning() {
        centralManager.stopScan()
        print("Stopped scanning for devices")
    }
    
    /// Connect to a discovered peripheral
    func connect(to peripheral: CBPeripheral) {
        print("Connecting to \(peripheral.name ?? "Unknown Device")...")
        centralManager.connect(peripheral, options: nil)
    }
    
    /// Disconnect from the connected peripheral
    func disconnect() {
        if let peripheral = connectedDevice {
            centralManager.cancelPeripheralConnection(peripheral)
        }
    }

    func sendUserLocation(_ location: CLLocationCoordinate2D) {
        guard isConnected, 
              let peripheral = connectedDevice,
              let characteristic = userLocationCharacteristic else {
            print("Cannot send location: not connected or characteristic not found")
            return
        }
        
        var userLocationProto = RKTUserLocation()
        userLocationProto.lat = location.latitude
        userLocationProto.lon = location.longitude
        
        do {
            let data = try userLocationProto.serializedData()

            isSending = true

            peripheral.writeValue(data, for: characteristic, type: .withResponse)
        } catch {
            print("Failed to serialize protocol buffer: \(error)")
        }
    }
}

extension BluetoothService: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .poweredOn:
            print("Bluetooth is powered on")
            startScanning()
        case .poweredOff:
            print("Bluetooth is powered off")
            isConnected = false
        case .unsupported:
            print("Bluetooth is not supported on this device")
        case .unauthorized:
            print("Bluetooth use is not authorized")
        case .resetting:
            print("Bluetooth is resetting")
        case .unknown:
            print("Bluetooth state is unknown")
        @unknown default:
            print("Unknown Bluetooth state")
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        // Only add devices with names to our list
        if let name = peripheral.name, !name.isEmpty {
            if !discoveredDevices.contains(where: { $0.peripheral.identifier == peripheral.identifier }) {
                discoveredDevices.append((peripheral, RSSI))
            } else if let index = discoveredDevices.firstIndex(where: { $0.peripheral.identifier == peripheral.identifier }) {
                discoveredDevices[index].rssi = RSSI
            }
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        connectedDevice = peripheral
        peripheral.delegate = self
        print("Connected to \(peripheral.name ?? "Unknown Device")")
        
        // Discover services after a brief delay to let connection stabilize
        DispatchQueue.main.asyncAfter(deadline: .now() + 0.5) {
            peripheral.discoverServices(nil)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didDisconnectPeripheral peripheral: CBPeripheral, error: Error?) {
        print("Disconnected from \(peripheral.name ?? "Unknown Device")")
        connectedDevice = nil
        trackerPacketsCharacteristic = nil
        userLocationCharacteristic = nil
        
        DispatchQueue.main.async {
            self.isConnected = false
        }
    }
    
    func centralManager(_ central: CBCentralManager, didFailToConnect peripheral: CBPeripheral, error: Error?) {
        print("Failed to connect: \(error?.localizedDescription ?? "Unknown error")")
    }
}

// MARK: - CBPeripheralDelegate
extension BluetoothService: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let error = error {
            print("Error discovering services: \(error.localizedDescription)")
            return
        }
        
        guard let services = peripheral.services else { return }
        
        for service in services {
            print("Discovered service: \(service.uuid)")
            peripheral.discoverCharacteristics(nil, for: service)
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if let error = error {
            print("Error discovering characteristics: \(error.localizedDescription)")
            return
        }
        
        guard let characteristics = service.characteristics else { return }
        
        for characteristic in characteristics {
            print("Found characteristic: \(characteristic.uuid)")
            
            // Look for our data characteristic
            if characteristic.uuid.uuidString == trackerPacketsUUID.uuidString {
                print("Found data characteristic")
                trackerPacketsCharacteristic = characteristic
                
                if characteristic.properties.contains(.notify) {
                    peripheral.setNotifyValue(true, for: characteristic)
                    print("Enabled notifications for data characteristic")
                }
                
                if characteristic.properties.contains(.read) {
                    peripheral.readValue(for: characteristic)
                }
            }

            if characteristic.uuid.uuidString == userLocationUUID.uuidString {
                print("Found user location characteristic")
                userLocationCharacteristic = characteristic

                if characteristic.properties.contains(.read) {
                    print("User location characteristic is readable")
                }
                
                if characteristic.properties.contains(.write) {
                    print("User location characteristic is writable")
                }
            }
        }
        
        // If we found our data characteristic, we're connected
        if trackerPacketsCharacteristic != nil {
            DispatchQueue.main.async {
                self.isConnected = true
            }
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard error == nil else {
            print("Error reading characteristic: \(error!.localizedDescription)")
            return
        }
        
        guard let data = characteristic.value else {
            print("No data received from characteristic")
            return
        }

        print("Received data for characteristic: \(characteristic.uuid)")
        print("Received data length: \(data.count) bytes")
        print("Raw bytes: \(data.map { String(format: "%02x", $0) }.joined())")
        
        if characteristic.uuid == trackerPacketsUUID {
            // Extract the protocol buffer size from the first two bytes (little endian)
            guard data.count >= 2 else {
                print("Data too short, need at least 2 bytes for length")
                return
            }
            
            // Read length prefix (little endian)
            let length = UInt16(data[0]) | (UInt16(data[1]) << 8)
            print("Protobuf message length: \(length) bytes")
            
            // Validate length
            guard length > 0, data.count >= Int(length) + 2 else {
                print("Invalid length or insufficient data: length=\(length), data size=\(data.count)")
                return
            }
            
            // Extract just the protobuf message (skip the length prefix)
            let protobufData = data.subdata(in: 2..<(Int(length) + 2))
            print("Extracted protobuf data (\(protobufData.count) bytes)")
            
            do {
                // Parse the extracted protobuf data
                let packet = try RKTTrackerPacket(serializedBytes: protobufData)
                print("Successfully parsed packet: type=\(packet.packetType), device=\(packet.deviceID), msg=\(packet.msgNum)")
                
                // Process the packet
                handleTrackerPacket(packet)
                
            } catch {
                print("Error decoding protobuf data: \(error)")
                
                // Debug: Print the first few bytes of the extracted data
                if protobufData.count > 0 {
                    let bytesToPrint = min(16, protobufData.count)
                    print("First \(bytesToPrint) bytes of protobuf: \(protobufData.prefix(bytesToPrint).map { String(format: "%02x", $0) }.joined(separator: " "))")
                }
            }
        }
    }

    private func handleTrackerPacket(_ packet: RKTTrackerPacket) {
        let key = PacketKey(deviceID: packet.deviceID, msgNum: packet.msgNum)
        
        // Get or create partial data object
        var partialData: PartialTelemetryData
        if let existing = partialDataCache[key] {
            partialData = existing
            partialData.receivedTime = Date()
        } else {
            partialData = PartialTelemetryData(
                deviceID: packet.deviceID,
                msgNum: packet.msgNum,
                timeSinceBoot: packet.timeSinceBoot
            )
            partialDataCache[key] = partialData
        }
        
        // Update partial data based on packet type
        switch packet.packetType {
            case .gps:
                if case .gps(let gpsData)? = packet.payload {
                    partialData.gpsData = gpsData
                    partialData.hasGps = true
                }
                
            case .ismPrimary:
                if case .imu(let imuData)? = packet.payload {
                    partialData.ismPrimaryData = imuData
                    partialData.hasIsmPrimary = true
                }
                
            case .ismSecondary:
                if case .imu(let imuData)? = packet.payload {
                    partialData.ismSecondaryData = imuData
                    partialData.hasIsmSecondary = true
                }
                
            case .lsm:
                if case .imu(let imuData)? = packet.payload {
                    partialData.lsmData = imuData
                    partialData.hasLsm = true
                }
                
            case .accel:
                if case .accel(let accelData)? = packet.payload {
                    partialData.accelData = accelData
                    partialData.hasAccel = true
                }
                
            case .baro:
                if case .baro(let baroData)? = packet.payload {
                    partialData.baroData = baroData
                    partialData.hasBaro = true
                }
                
            case .userLocation, .unknown, .UNRECOGNIZED(_):
                // Handle user location if needed
                break
            @unknown default:
                print("Unknown packet type: \(packet.packetType)")
        }

        let status = [
            partialData.hasGps ? "GPS" : nil,
            partialData.hasIsmPrimary ? "ISM1" : nil, 
            partialData.hasIsmSecondary ? "ISM2" : nil,
            partialData.hasLsm ? "LSM" : nil,
            partialData.hasAccel ? "ADXL" : nil,
            partialData.hasBaro ? "BARO" : nil
        ].compactMap { $0 }.joined(separator: ", ")

        // Check if we have all required data to create a complete telemetry record
        if partialData.isComplete {
            // Convert to TelemetryData
            let telemetryData = partialData.toTelemetryData()
            
            // Publish the telemetry data
            publishTelemetryUpdate(telemetryData)
            
            // Remove this entry from the cache
            partialDataCache.removeValue(forKey: key)
        }
    }

    private func publishTelemetryUpdate(_ telemetryData: TelemetryData) {
        DispatchQueue.main.async {
            // Publish the telemetry update
            self.latestTelemetry = telemetryData
            
            // Update by device ID
            self.telemetryDataByDevice[telemetryData.deviceID] = telemetryData
            
            // Notify listeners 
            NotificationCenter.default.post(
                name: .telemetryDataUpdated,
                object: nil,
                userInfo: ["telemetryData": telemetryData]
            )
            
            // Log the data if logging is enabled
            if let logger = self.telemetryLogger, self.isLoggingEnabled {
                logger.logTelemetry(telemetryData)
            }
        }
    }

    // To be called when the app initializes
    func setupCleanupTimer() {
        // Cancel any existing timer
        cleanupTimer?.invalidate()
        
        // Create a new timer that runs every second to clean up stale partial data
        cleanupTimer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
            guard let self = self else { return }
            self.cleanupStaleData()
        }
    }

    private func cleanupStaleData() {
        let now = Date()
        
        // Find keys for stale partial data
        let staleKeys = partialDataCache.filter { _, partialData in
            return now.timeIntervalSince(partialData.receivedTime) > partialDataTimeout
        }.keys
        
        // Remove stale entries
        for key in staleKeys {
            partialDataCache.removeValue(forKey: key)
        }
        
        if !staleKeys.isEmpty {
            print("Cleaned up \(staleKeys.count) stale partial data entries")
        }
    }
        

    func peripheral(_ peripheral: CBPeripheral, didWriteValueFor characteristic: CBCharacteristic, error: Error?) {
        // Reset sending flag
        isSending = false
        
        if let error = error {
            print("Error writing user location: \(error.localizedDescription)")
        } else if characteristic.uuid.uuidString == userLocationUUID.uuidString {
//            print("User location successfully sent")
        }
    }
    
    private func processTelemetryData(_ data: Data) {
        do {
            let packet = try parseProtobufWithLengthPrefix(data: data)
            handleTrackerPacket(packet)
        } catch {
            print("Not a valid protobuf: \(error)")
        }
    }

    private func parseProtobufWithLengthPrefix(data: Data) throws -> RKTTrackerPacket {
        guard data.count >= 2 else {
            throw CustomError.invalidLength
        }
        
        // Extract length (first two bytes, little endian)
        let length = UInt16(data[0]) + (UInt16(data[1]) << 8)
        
        // Ensure length is valid
        guard length > 0, length <= data.count - 2 else {
            throw CustomError.invalidLength
        }
        
        // Extract actual protobuf data using the length
        let protobufData = data.subdata(in: 2..<(Int(length) + 2))
        
        // Parse the protobuf data
        return try RKTTrackerPacket(serializedBytes: protobufData)
    }
    

    enum CustomError: Error {
        case invalidLength
        case invalidData
        case parsingFailure
    }
    
    private func addMessage(_ message: String) {
        DispatchQueue.main.async {
            self.receivedMessages.append(ReceivedMessage(timestamp: Date(), message: message))
            
            // Keep only last 10 messages
            if self.receivedMessages.count > 10 {
                self.receivedMessages.removeFirst()
            }
        }
    }
}

// Helper function to convert RKTGpsFix enum to a String
private func gpsFix(from fix: RKTGpsFix) -> String {
    switch fix {
    case .unknownGpsfix:
        return "Unknown GPS FIX Type"
    case .noFix:
        return "No Fix"
    case .deadReckoningOnly:
        return "Dead Reckoning Only"
    case .fix2D:
        return "2D Fix"
    case .fix3D:
        return "3D Fix"
    case .gpsPlusDeadReckoning:
        return "GPS + Dead Reckoning"
    case .timeOnlyFix:
        return "Time Only Fix"
    case .UNRECOGNIZED(let value):
        return "Unknown (\(value))"
    }
}

protocol DataLogger {
    func logTelemetry(_ data: TelemetryData)
}

extension Notification.Name {
    static let telemetryDataUpdated = Notification.Name("telemetryDataUpdated")
}