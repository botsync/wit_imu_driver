/*
 * Copyright(c) 2019, strv
 * All rights reserved.
 */
#ifndef WIT_IMU_DRIVER_WIT_IMU_H
#define WIT_IMU_DRIVER_WIT_IMU_H

#include <vector>
#include <queue>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>

namespace wit_imu_driver
{
enum PRODUCT
{
    WT901C
};

class WitImu
{
public:
    WitImu(const double co_gravity, const size_t msg_buffer_size = 100)
    : buf_(1024)
    , co_gravity_(co_gravity)
    , msg_buf_max_(msg_buffer_size)
    {
    }

    virtual void pushBytes( const std::vector<uint8_t>& bytes,
                            const size_t size,
                            const ros::Time& stamp){};
    bool popImuData(sensor_msgs::Imu* const p_msg)
    {
        if (imu_buf_.empty())
        {
            return false;
        }
        *p_msg = imu_buf_.front();
        imu_buf_.pop();
        return true;
    };

    size_t sizeImuData()
    {
        return imu_buf_.size();
    };

    bool popTempData(sensor_msgs::Temperature* const p_msg)
    {
        if (temp_buf_.empty())
        {
            return false;
        }
        *p_msg = temp_buf_.front();
        temp_buf_.pop();
        return true;
    };

    size_t sizeTempData()
    {
        return temp_buf_.size();
    };

    bool popMagData(sensor_msgs::MagneticField* const p_msg)
    {
        if (mag_buf_.empty())
        {
            return false;
        }
        *p_msg = mag_buf_.front();
        mag_buf_.pop();
        return true;
    };

    size_t sizeMagData()
    {
        return mag_buf_.size();
    };

protected:
    const double co_gravity_;
    const size_t msg_buf_max_;
    std::vector<uint8_t> buf_;
    std::queue<sensor_msgs::Imu> imu_buf_;
    std::queue<sensor_msgs::Temperature> temp_buf_;
    std::queue<sensor_msgs::MagneticField> mag_buf_;

    static int bytes2int(const uint8_t h, const uint8_t l)
    {
        return static_cast<int16_t>(
                ((static_cast<uint16_t>(h) << 8) & 0xFF00)
                | (static_cast<uint16_t>(l) & 0x00FF));
    }
};
}   // wit_imu_driver

#endif // WIT_IMU_DRIVER_WIT_IMU_H