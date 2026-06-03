#include "nine_axis_imu/LSM6DSOX_LIS3MDL/LSM6DSOX_LIS3MDL.hpp"

namespace nine_axis_imu::lsm6dsox_lis3mdl {

lsm6dsox::Transport Device::imu_transport(Transport transport) noexcept
{
    return lsm6dsox::Transport{
        .ctx = transport.ctx,
        .write_reg = transport.write_reg,
        .read_regs = transport.read_regs,
    };
}

lis3mdl::Transport Device::mag_transport(Transport transport) noexcept
{
    return lis3mdl::Transport{
        .ctx = transport.ctx,
        .write_reg = transport.write_reg,
        .read_regs = transport.read_regs,
    };
}

Device::Device(Transport transport, uint8_t imu_address, uint8_t mag_address) noexcept
    : m_imu(imu_transport(transport), imu_address)
    , m_mag(mag_transport(transport), mag_address)
{}

bool Device::initialize(Config const& cfg)
{
    return m_imu.initialize(cfg.imu) && m_mag.initialize(cfg.mag);
}

DataReady Device::data_ready() const
{
    return DataReady{
        .imu = m_imu.data_ready(),
        .mag = m_mag.data_ready(),
    };
}

bool Device::read_sample(Sample& sample) const
{
    lsm6dsox::Sample imu_sample = {};
    if (!m_imu.read_sample(imu_sample)) return false;

    sample.accel[0] = imu_sample.accel_x;
    sample.accel[1] = imu_sample.accel_y;
    sample.accel[2] = imu_sample.accel_z;
    sample.gyro[0] = imu_sample.gyro_x;
    sample.gyro[1] = imu_sample.gyro_y;
    sample.gyro[2] = imu_sample.gyro_z;
    sample.temp_c = imu_sample.temp_c;

    lis3mdl::Sample mag_sample = {};
    sample.mag_valid = m_mag.read_sample(mag_sample);
    if (sample.mag_valid) {
        sample.mag_gauss[0] = mag_sample.x_gauss;
        sample.mag_gauss[1] = mag_sample.y_gauss;
        sample.mag_gauss[2] = mag_sample.z_gauss;
    }

    return true;
}

} // namespace nine_axis_imu::lsm6dsox_lis3mdl
