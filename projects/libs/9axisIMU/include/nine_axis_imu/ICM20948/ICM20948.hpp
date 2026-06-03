#pragma once

#include <cstddef>
#include <cstdint>

namespace nine_axis_imu {

using WriteRegFn = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t val);
using ReadRegsFn = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);

struct Transport {
    void*      ctx{};
    WriteRegFn write_reg{};
    ReadRegsFn read_regs{};
};

enum class AccelRange : uint8_t {
    g2  = 0,
    g4  = 1,
    g8  = 2,
    g16 = 3,
};

enum class GyroRange : uint8_t {
    dps_250  = 0,
    dps_500  = 1,
    dps_1000 = 2,
    dps_2000 = 3,
};

enum class DlpfBandwidth : uint8_t {
    hz_196_6 = 0,
    hz_151_8 = 1,
    hz_119_5 = 2,
    hz_51_2  = 3,
    hz_23_9  = 4,
    hz_11_6  = 5,
    hz_5_7   = 6,
    hz_361_4 = 7,
};

enum class MagMode : uint8_t {
    power_down = 0x00,
    single     = 0x01,
    hz_10      = 0x02,
    hz_20      = 0x04,
    hz_50      = 0x06,
    hz_100     = 0x08,
};

struct Config {
    AccelRange   accel_range = AccelRange::g16;
    GyroRange    gyro_range  = GyroRange::dps_2000;
    uint16_t     accel_rate_hz = 100;
    uint16_t     gyro_rate_hz  = 100;
    bool         accel_dlpf_enable = true;
    bool         gyro_dlpf_enable  = true;
    DlpfBandwidth accel_dlpf = DlpfBandwidth::hz_23_9;
    DlpfBandwidth gyro_dlpf  = DlpfBandwidth::hz_23_9;
    MagMode      mag_mode = MagMode::hz_100;
    void (*delay_ms)(uint32_t) = nullptr;
};

struct DataReady {
    bool accel{};
    bool gyro{};
    bool mag{};
    bool mag_overflow{};
};

struct Sample {
    float accel_x{};
    float accel_y{};
    float accel_z{};
    float gyro_x{};
    float gyro_y{};
    float gyro_z{};
    float mag_x_ut{};
    float mag_y_ut{};
    float mag_z_ut{};
    float temp_c{};
    bool  mag_valid{};
    bool  mag_overflow{};
};

class ICM20948 {
public:
    static constexpr uint8_t kDefaultAddress = 0x69;
    static constexpr uint8_t kAltAddress = 0x68;
    static constexpr uint8_t kWhoAmIExpected = 0xEA;
    static constexpr uint16_t kAk09916WhoAmIExpected = 0x4809;

    explicit ICM20948(Transport transport, uint8_t device_address = kDefaultAddress) noexcept;

    [[nodiscard]] bool initialize(Config const& cfg = {});
    [[nodiscard]] bool reset(void (*delay_ms)(uint32_t) = nullptr);

    [[nodiscard]] bool set_accel_range(AccelRange range);
    [[nodiscard]] bool set_gyro_range(GyroRange range);
    [[nodiscard]] bool set_accel_rate(uint16_t hz);
    [[nodiscard]] bool set_gyro_rate(uint16_t hz);
    [[nodiscard]] bool set_accel_dlpf(bool enable, DlpfBandwidth bandwidth);
    [[nodiscard]] bool set_gyro_dlpf(bool enable, DlpfBandwidth bandwidth);
    [[nodiscard]] bool set_mag_mode(MagMode mode);

    [[nodiscard]] DataReady data_ready() const;
    [[nodiscard]] bool read_sample(Sample& sample) const;

    [[nodiscard]] bool read_who_am_i(uint8_t& value) const;
    [[nodiscard]] bool read_mag_who_am_i(uint16_t& value) const;

    [[nodiscard]] AccelRange current_accel_range() const noexcept { return m_accel_range; }
    [[nodiscard]] GyroRange current_gyro_range() const noexcept { return m_gyro_range; }
    [[nodiscard]] uint16_t current_accel_rate_hz() const noexcept { return m_accel_rate_hz; }
    [[nodiscard]] uint16_t current_gyro_rate_hz() const noexcept { return m_gyro_rate_hz; }

private:
    enum class Bank : uint8_t {
        bank0 = 0,
        bank1 = 1,
        bank2 = 2,
        bank3 = 3,
    };

    [[nodiscard]] bool set_bank(Bank bank) const;
    [[nodiscard]] bool write_bank(Bank bank, uint8_t reg, uint8_t val) const;
    [[nodiscard]] bool read_bank(Bank bank, uint8_t reg, uint8_t* buf, size_t len) const;
    [[nodiscard]] bool rmw_bank(Bank bank, uint8_t reg, uint8_t mask, uint8_t bits) const;
    [[nodiscard]] bool write_reg(uint8_t reg, uint8_t val) const;
    [[nodiscard]] bool read_regs(uint8_t reg, uint8_t* buf, size_t len) const;

    [[nodiscard]] bool enable_i2c_master();
    [[nodiscard]] bool reset_i2c_master();
    [[nodiscard]] bool configure_mag_readout();
    [[nodiscard]] bool configure_aux_peripheral(uint8_t slot, uint8_t addr, uint8_t reg,
                                                uint8_t len, bool read, bool enable,
                                                uint8_t data_out = 0) const;
    [[nodiscard]] bool aux_read(uint8_t addr, uint8_t reg, uint8_t& data) const;
    [[nodiscard]] bool aux_write(uint8_t addr, uint8_t reg, uint8_t data) const;

    Transport m_transport;
    uint8_t m_device_address;
    mutable Bank m_current_bank = Bank::bank0;
    mutable bool m_bank_known = false;

    AccelRange m_accel_range = AccelRange::g16;
    GyroRange m_gyro_range = GyroRange::dps_2000;
    uint16_t m_accel_rate_hz = 100;
    uint16_t m_gyro_rate_hz = 100;
};

} // namespace nine_axis_imu
