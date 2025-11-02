import Foundation
import GRDB

struct TelemetryRecord: Codable, FetchableRecord, PersistableRecord, Identifiable {
    var id: Int64?
    var deviceID: Int32
    var msgNum: Int32
    var timeSinceBoot: Int32
    var timestamp: Date?
    var adxl_accel_x: Double
    var adxl_accel_y: Double
    var adxl_accel_z: Double
    var ism_primary_accel_x: Double
    var ism_primary_accel_y: Double
    var ism_primary_accel_z: Double
    var ism_primary_gyro_x: Double
    var ism_primary_gyro_y: Double
    var ism_primary_gyro_z: Double
    var ism_secondary_accel_x: Double
    var ism_secondary_accel_y: Double
    var ism_secondary_accel_z: Double
    var ism_secondary_gyro_x: Double
    var ism_secondary_gyro_y: Double
    var ism_secondary_gyro_z: Double
    var lsm_accel_x: Double
    var lsm_accel_y: Double
    var lsm_accel_z: Double
    var lsm_gyro_x: Double
    var lsm_gyro_y: Double
    var lsm_gyro_z: Double
    var baro_alt: Double
    var gps_alt: Double
    var gps_fix: String?
    var gps_lat: Double
    var gps_lon: Double
    var gps_num_sats: Int16
    var gps_utc_day: Int16
    var gps_utc_hour: Int16
    var gps_utc_itow: Double
    var gps_utc_min: Int16
    var gps_utc_month: Int16
    var gps_utc_nanos: Double
    var gps_utc_sec: Int16
    var gps_utc_time_accuracy_estimate_ns: Double
    var gps_utc_valid: Int32
    var gps_utc_year: Int16
    
    enum Columns {
        static let id = Column(CodingKeys.id)
        static let deviceID = Column(CodingKeys.deviceID)
        static let msgNum = Column(CodingKeys.msgNum)
        static let timeSinceBoot = Column(CodingKeys.timeSinceBoot)
        static let timestamp = Column(CodingKeys.timestamp)
        static let adxl_accel_x = Column(CodingKeys.adxl_accel_x)
        static let adxl_accel_y = Column(CodingKeys.adxl_accel_y)
        static let adxl_accel_z = Column(CodingKeys.adxl_accel_z)
        static let ism_primary_accel_x = Column(CodingKeys.ism_primary_accel_x)
        static let ism_primary_accel_y = Column(CodingKeys.ism_primary_accel_y)
        static let ism_primary_accel_z = Column(CodingKeys.ism_primary_accel_z)
        static let ism_primary_gyro_x = Column(CodingKeys.ism_primary_gyro_x)
        static let ism_primary_gyro_y = Column(CodingKeys.ism_primary_gyro_y)
        static let ism_primary_gyro_z = Column(CodingKeys.ism_primary_gyro_z)
        static let ism_secondary_accel_x = Column(CodingKeys.ism_secondary_accel_x)
        static let ism_secondary_accel_y = Column(CodingKeys.ism_secondary_accel_y)
        static let ism_secondary_accel_z = Column(CodingKeys.ism_secondary_accel_z)
        static let ism_secondary_gyro_x = Column(CodingKeys.ism_secondary_gyro_x)
        static let ism_secondary_gyro_y = Column(CodingKeys.ism_secondary_gyro_y)
        static let ism_secondary_gyro_z = Column(CodingKeys.ism_secondary_gyro_z)
        static let lsm_accel_x = Column(CodingKeys.lsm_accel_x)
        static let lsm_accel_y = Column(CodingKeys.lsm_accel_y)
        static let lsm_accel_z = Column(CodingKeys.lsm_accel_z)
        static let lsm_gyro_x = Column(CodingKeys.lsm_gyro_x)
        static let lsm_gyro_y = Column(CodingKeys.lsm_gyro_y)
        static let lsm_gyro_z = Column(CodingKeys.lsm_gyro_z)
        static let baro_alt = Column(CodingKeys.baro_alt)
        static let gps_alt = Column(CodingKeys.gps_alt)
        static let gps_fix = Column(CodingKeys.gps_fix)
        static let gps_lat = Column(CodingKeys.gps_lat)
        static let gps_lon = Column(CodingKeys.gps_lon)
        static let gps_num_sats = Column(CodingKeys.gps_num_sats)
        static let gps_utc_day = Column(CodingKeys.gps_utc_day)
        static let gps_utc_hour = Column(CodingKeys.gps_utc_hour)
        static let gps_utc_itow = Column(CodingKeys.gps_utc_itow)
        static let gps_utc_min = Column(CodingKeys.gps_utc_min)
        static let gps_utc_month = Column(CodingKeys.gps_utc_month)
        static let gps_utc_nanos = Column(CodingKeys.gps_utc_nanos)
        static let gps_utc_sec = Column(CodingKeys.gps_utc_sec)
        static let gps_utc_time_accuracy_estimate_ns = Column(CodingKeys.gps_utc_time_accuracy_estimate_ns)
        static let gps_utc_valid = Column(CodingKeys.gps_utc_valid)
        static let gps_utc_year = Column(CodingKeys.gps_utc_year)
    }
}