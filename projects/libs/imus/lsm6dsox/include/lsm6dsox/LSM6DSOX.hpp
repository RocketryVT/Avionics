#pragma once

#include <cstddef>
#include <cstdint>

namespace lsm6dsox {

using WriteRegFn = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t val);
using ReadRegsFn = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);

struct Transport {
    void*       ctx{};
    WriteRegFn write_reg{};
    ReadRegsFn read_regs{};
};

enum class AccelODR : uint8_t {
    power_down = 0x00,
    hz_12_5    = 0x10,
    hz_26      = 0x20,
    hz_52      = 0x30,
    hz_104     = 0x40,
    hz_208     = 0x50,
    hz_416     = 0x60,
    hz_833     = 0x70,
    hz_1667    = 0x80,
    hz_3333    = 0x90,
    hz_6667    = 0xA0,
};

enum class GyroODR : uint8_t {
    power_down = 0x00,
    hz_12_5    = 0x10,
    hz_26      = 0x20,
    hz_52      = 0x30,
    hz_104     = 0x40,
    hz_208     = 0x50,
    hz_416     = 0x60,
    hz_833     = 0x70,
    hz_1667    = 0x80,
    hz_3333    = 0x90,
    hz_6667    = 0xA0,
};

enum class AccelRange : uint8_t {
    g2  = 0x00,
    g16 = 0x04,
    g4  = 0x08,
    g8  = 0x0C,
};

enum class GyroRange : uint8_t {
    dps_125  = 0x02,
    dps_250  = 0x00,
    dps_500  = 0x04,
    dps_1000 = 0x08,
    dps_2000 = 0x0C,
};

struct Config {
    AccelRange accel_range = AccelRange::g4;
    GyroRange  gyro_range  = GyroRange::dps_2000;
    AccelODR   accel_odr   = AccelODR::hz_104;
    GyroODR    gyro_odr    = GyroODR::hz_104;
    bool       block_data_update = true;
    bool       auto_increment = true;
    bool       disable_i3c = true;
    bool       accel_lpf2_odr_div4 = true;
    void (*delay_ms)(uint32_t) = nullptr;
};

struct DataReady {
    bool accel{};
    bool gyro{};
    bool temp{};
};

struct Sample {
    float accel_x{};
    float accel_y{};
    float accel_z{};
    float gyro_x{};
    float gyro_y{};
    float gyro_z{};
    float temp_c{};
};

class Device {
public:
    static constexpr uint8_t kDefaultAddress = 0x6A;
    static constexpr uint8_t kAltAddress = 0x6B;
    static constexpr uint8_t kWhoAmIExpected = 0x6C;
    static constexpr uint8_t kWhoAmICompatible = 0x69;

    explicit Device(Transport transport, uint8_t device_address = kDefaultAddress) noexcept;

    [[nodiscard]] bool initialize(Config const& cfg = {});
    [[nodiscard]] bool reset(void (*delay_ms)(uint32_t) = nullptr);

    [[nodiscard]] bool set_accel_odr(AccelODR odr);
    [[nodiscard]] bool set_gyro_odr(GyroODR odr);
    [[nodiscard]] bool set_accel_range(AccelRange range);
    [[nodiscard]] bool set_gyro_range(GyroRange range);
    [[nodiscard]] bool set_accel_self_test(bool enable);
    [[nodiscard]] bool set_gyro_self_test(bool enable);

    [[nodiscard]] bool enable_accel();
    [[nodiscard]] bool disable_accel();
    [[nodiscard]] bool enable_gyro();
    [[nodiscard]] bool disable_gyro();

    [[nodiscard]] DataReady data_ready() const;
    [[nodiscard]] bool read_sample(Sample& sample) const;
    [[nodiscard]] bool read_accelerometer(float& x, float& y, float& z) const;
    [[nodiscard]] bool read_gyroscope(float& x, float& y, float& z) const;
    [[nodiscard]] bool read_temperature(float& temp_c) const;
    [[nodiscard]] bool read_who_am_i(uint8_t& value) const;

    [[nodiscard]] AccelODR   current_accel_odr()   const noexcept { return m_accel_odr; }
    [[nodiscard]] GyroODR    current_gyro_odr()    const noexcept { return m_gyro_odr; }
    [[nodiscard]] AccelRange current_accel_range() const noexcept { return m_accel_range; }
    [[nodiscard]] GyroRange  current_gyro_range()  const noexcept { return m_gyro_range; }

private:
    [[nodiscard]] bool write_reg(uint8_t reg, uint8_t val) const;
    [[nodiscard]] bool read_regs(uint8_t reg, uint8_t* buf, size_t len) const;
    [[nodiscard]] bool rmw_reg(uint8_t reg, uint8_t mask, uint8_t bits) const;

    Transport m_transport;
    uint8_t   m_device_address;
    AccelRange m_accel_range = AccelRange::g2;
    GyroRange  m_gyro_range  = GyroRange::dps_250;
    AccelODR   m_accel_odr   = AccelODR::power_down;
    GyroODR    m_gyro_odr    = GyroODR::power_down;
    AccelODR   m_accel_last_odr = AccelODR::hz_104;
    GyroODR    m_gyro_last_odr  = GyroODR::hz_104;
};

} // namespace lsm6dsox
