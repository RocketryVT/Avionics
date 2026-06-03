#pragma once

#include "lis3mdl/LIS3MDL.hpp"
#include "lsm6dsox/LSM6DSOX.hpp"

#include <cstddef>
#include <cstdint>

namespace nine_axis_imu::lsm6dsox_lis3mdl {

using WriteRegFn = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t val);
using ReadRegsFn = bool (*)(void* ctx, uint8_t addr, uint8_t reg, uint8_t* buf, size_t len);

struct Transport {
    void*       ctx{};
    WriteRegFn write_reg{};
    ReadRegsFn read_regs{};
};

struct Config {
    lsm6dsox::Config imu{};
    lis3mdl::Config  mag{};
};

struct DataReady {
    lsm6dsox::DataReady imu{};
    bool mag{};
};

struct Sample {
    float accel[3]{};
    float gyro[3]{};
    float mag_gauss[3]{};
    float temp_c{};
    bool mag_valid{};
};

class Device {
public:
    explicit Device(Transport transport,
                    uint8_t imu_address = lsm6dsox::Device::kDefaultAddress,
                    uint8_t mag_address = lis3mdl::Device::kDefaultAddress) noexcept;

    [[nodiscard]] bool initialize(Config const& cfg = {});
    [[nodiscard]] DataReady data_ready() const;
    [[nodiscard]] bool read_sample(Sample& sample) const;

    [[nodiscard]] lsm6dsox::Device& imu() noexcept { return m_imu; }
    [[nodiscard]] lis3mdl::Device& mag() noexcept { return m_mag; }
    [[nodiscard]] lsm6dsox::Device const& imu() const noexcept { return m_imu; }
    [[nodiscard]] lis3mdl::Device const& mag() const noexcept { return m_mag; }

private:
    static lsm6dsox::Transport imu_transport(Transport transport) noexcept;
    static lis3mdl::Transport mag_transport(Transport transport) noexcept;

    lsm6dsox::Device m_imu;
    lis3mdl::Device  m_mag;
};

} // namespace nine_axis_imu::lsm6dsox_lis3mdl
