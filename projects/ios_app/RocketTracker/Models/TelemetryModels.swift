import Foundation

struct TelemetryData: Codable {
    let deviceID: UInt32
    let time_since_boot: Int
    let msg_num: Int
    let gps: GPSData
    let ism_primary: AccelGyroData
    let ism_secondary: AccelGyroData
    let lsm: AccelGyroData
    let adxl: AccelerometerData
    let barometer: BarometerData
}

struct GPSData : Codable {
    let alt: Double
    let fix: String
    let lat: Double
    let lon: Double
    let num_sats: Int
    let time: UTCTime
}

struct UTCTime: Codable {
    let itow: Int
    let time_accuracy_estimate_ns: Int
    let year: Int
    let month: Int
    let day: Int
    let hour: Int
    let min: Int
    let sec: Int
    let nanos: Int
    let valid: Int
}

struct AccelGyroData: Codable {
    let accelerometer: AccelerometerData
    let gyroscope: GyroscopeData
}

struct AccelerometerData: Codable {
    let x: Double
    let y: Double
    let z: Double
}

struct GyroscopeData: Codable {
    let x: Double
    let y: Double
    let z: Double
}

struct BarometerData: Codable {
    let altitude: Double
}

struct ReceivedMessage: Identifiable, Hashable {
    let id = UUID()
    let timestamp: Date
    let message: String
    
    func hash(into hasher: inout Hasher) {
        hasher.combine(id)
    }
    
    static func == (lhs: ReceivedMessage, rhs: ReceivedMessage) -> Bool {
        return lhs.id == rhs.id
    }
}
