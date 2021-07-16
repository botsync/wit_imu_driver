/*
 * Copyright(c) 2019, strv
 * All rights reserved.
 */

#include <mutex>
#include <thread>
#include <vector>
#include <chrono>
#include <queue>
#include <numeric>
#include <string>
#include <linux/serial.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include "wit_imu_driver/wit_imu.h"
#include "wit_imu_driver/wt901c.h"

namespace wit_imu_driver
{
namespace ba = boost::asio;
class WitImuDriver 
{
public:
    std::shared_ptr<rclcpp::Node>node; 
    bool first_launch = true;

    WitImuDriver()
    : node {rclcpp::Node::make_shared("wit_imu_driver")}
    , clock_{node->get_clock()}
    , port_io_()
    , port_(port_io_)
    , rx_buf_(1024)
    {
        node->declare_parameter("gravity", 9.797673);
        co_gravity_=node->get_parameter("gravity").as_double();

        node->declare_parameter("frame_id", "imu_link");
        frame_id_=node->get_parameter("frame_id").as_string();

        //WitImu::PRODUCT::WT901C
        node->declare_parameter("product", 0);
        product_=node->get_parameter("product").as_int();
    }

    bool open()
    {
        std::string dev;
        int baud;

        node->declare_parameter("device", "/dev/ttyUSB0");
        dev=node->get_parameter("device").as_string();

        node->declare_parameter("baud", 115200);
        baud=node->get_parameter("baud").as_int();

        ptr_imu_ = boost::make_shared<Wt901c>(Wt901c(co_gravity_));

        boost::system::error_code ec;
        port_.open(dev, ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to open %s. Error code : %d",
                    dev.c_str(),
                    ec.value());
            return false;
        }
        port_.set_option(ba::serial_port_base::baud_rate(baud), ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set baudrate. Error code : %d",
                    ec.value());
            return false;
        }
        port_.set_option(ba::serial_port_base::character_size(8), ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set options. Error code : %d",
                    ec.value());
            return false;
        }
        port_.set_option(ba::serial_port_base::flow_control(
                            ba::serial_port_base::flow_control::none), ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set options. Error code : %d",
                    ec.value());
            return false;
        }
        port_.set_option(ba::serial_port_base::parity(
                            ba::serial_port_base::parity::none), ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set options. Error code : %d",
                    ec.value());
            return false;
        }
        port_.set_option(ba::serial_port_base::stop_bits(
                            ba::serial_port_base::stop_bits::one), ec);
        if (ec.value() != 0)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to set options. Error code : %d",
                    ec.value());
            return false;
        }
        // FTDI USB-serial device has 16 msec latency in default.
        // It makes large latency and jitter for high rate measurement.
        const int fd = port_.native_handle();
        serial_struct port_info;
        ioctl(fd, TIOCGSERIAL, &port_info);
        port_info.flags |= ASYNC_LOW_LATENCY;
        ioctl(fd, TIOCSSERIAL, &port_info);
        
        return true;
    }

    bool spin()
    {
        switch (product_)
        {
            //WitImu::PRODUCT::WT901C
            case 0:
            {
                pub_imu_=node->create_publisher<sensor_msgs::msg::Imu>("data_raw", 10);
                pub_temp_=node->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
                pub_mag_=node->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
                cmd_vel_subcriber=node->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1000, 
                [this](std::shared_ptr<geometry_msgs::msg::Twist> msg){WitImuDriver::cmd_vel_callback(msg);});
                ptr_imu_ = boost::make_shared<Wt901c>(Wt901c(co_gravity_));

                srv_trg_yaw_clr_ = node->create_service<std_srvs::srv::Trigger>(
                                        "trigger_yaw_clear", [this](std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
                                        {WitImuDriver::cbSrvTrgWriteCommand(req, res, ptr_imu_->genYawClr());});

                srv_trg_acc_cal_ = node->create_service<std_srvs::srv::Trigger>(
                                        "trigger_acc_calibration", [this](std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
                                        {WitImuDriver::cbSrvTrgWriteCommand(req, res, ptr_imu_->genAccCal());});

                srv_trg_mag_cal_ = node->create_service<std_srvs::srv::Trigger>(
                                        "trigger_mag_calibration", [this](std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
                                        {WitImuDriver::cbSrvTrgWriteCommand(req, res, ptr_imu_->genMagCal());});

                srv_trg_exit_cal_ = node->create_service<std_srvs::srv::Trigger>(
                                        "trigger_exit_calibration", [this](std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
                                        {WitImuDriver::cbSrvTrgWriteCommand(req, res, ptr_imu_->genExitCal());});

                srv_trg_enable_gyro_cali_ = node->create_service<std_srvs::srv::Trigger>(
                                        "trigger_enable_gyro_auto_calibration", [this](std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
                                        {WitImuDriver::cbSrvTrgWriteCommand(req, res, ptr_imu_->enableAutoGyroCali());});

                srv_trg_disable_gyro_cali_ = node->create_service<std_srvs::srv::Trigger>(
                                        "trigger_disable_gyro_auto_calibration", [this](std::shared_ptr<std_srvs::srv::Trigger::Request> req, 
                                        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
                                        {WitImuDriver::cbSrvTrgWriteCommand(req, res, ptr_imu_->diableAutoGyroCali());});

                if(first_launch){
                    bool ret = sendBytes(ptr_imu_->enableAutoGyroCali());
                    if (ret)
                    {
                        RCLCPP_ERROR(node->get_logger(), "Successfully ENABLED Gyro Auto-calibration");
                    }
                    else
                    {
                        RCLCPP_ERROR(node->get_logger(), "Failed to ENABLED Gyro Auto-calibration");
                    }

                    sleep(10);

                    ret = sendBytes(ptr_imu_->diableAutoGyroCali());
                    if (ret)
                    {
                        RCLCPP_ERROR(node->get_logger(), "Successfully DISABLED Gyro Auto-calibration");
                    }
                    else
                    {
                        RCLCPP_ERROR(node->get_logger(), "Failed ENABLED Gyro Auto-calibration");
                    }
                    first_launch = false;
                }
            }
            break;

            default:
            RCLCPP_ERROR(node->get_logger(), "Product is not provided");
            return false;
        }
        startRead();
        auto io_run = [this]()
        {
            boost::system::error_code ec;
            port_io_.run(ec);
        };
        io_thread_ = std::thread(io_run);

        wdg_=node->create_wall_timer(std::chrono::milliseconds(500), [this](){WitImuDriver::cbWdg();});

        rclcpp::spin(node);
        close();
        return true;
    }

private:

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trg_yaw_clr_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trg_height_clr_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trg_acc_cal_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trg_mag_cal_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trg_exit_cal_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trg_enable_gyro_cali_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_trg_disable_gyro_cali_;

    rclcpp::TimerBase::SharedPtr wdg_;;

    // Create the clock object 
    rclcpp::Clock::SharedPtr clock_;

    std::string frame_id_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subcriber;
    // WitImuDriver listener;

    ba::io_service port_io_;
    ba::serial_port port_;
    std::thread io_thread_;

    double co_gravity_;
    int product_;
    std::vector<uint8_t> rx_buf_;

    boost::shared_ptr<WitImu> ptr_imu_;
    
    bool cmd_vel_input = false;
    bool enabled_auto_cali_status = false;
    rclcpp::Time prev_time = clock_->now();
    rclcpp::Time current_time = clock_->now();

    void close()
    {
        if (port_.is_open())
        {
            port_.cancel();
            port_.close();
        }
        if (!port_io_.stopped())
        {
            port_io_.stop();
        }
        if (io_thread_.joinable())
        {
            io_thread_.join();
        }
    }

    void startRead()
    {
        ba::async_read(port_,
                    ba::buffer(rx_buf_),
                    ba::transfer_at_least(1),
                    boost::bind(&WitImuDriver::cbPort, this, ba::placeholders::error, ba::placeholders::bytes_transferred));
    }

    void cbPort(const boost::system::error_code& ec,
                std::size_t size)
    {

        if (!ec)
        {
            
            ptr_imu_->pushBytes(rx_buf_, size, clock_->now());
            cmd_vel_check();
            while (ptr_imu_->sizeImuData() != 0)
            {
                sensor_msgs::msg::Imu::SharedPtr msg;
                ptr_imu_->popImuData(msg);
                msg->header.frame_id = frame_id_;
                pub_imu_->publish(*msg);
            }
            while (ptr_imu_->sizeTempData() != 0)
            {
                sensor_msgs::msg::Temperature::SharedPtr msg;
                ptr_imu_->popTempData(msg);
                msg->header.frame_id = frame_id_;
                pub_temp_->publish(*msg);
            }
            while (ptr_imu_->sizeMagData() != 0)
            {
                sensor_msgs::msg::MagneticField::SharedPtr msg;
                ptr_imu_->popMagData(msg);
                msg->header.frame_id = frame_id_;
                pub_mag_->publish(*msg);
            }
            startRead();
            resetWdg();
        }
        else if (ec == boost::system::errc::operation_canceled)
        {
            // Enter to this state when stop a connection
        }
        else
        {
            // Unknown state
            RCLCPP_ERROR(node->get_logger(), "[wit_imu_driver] serial error : %s", ec.message().c_str());
            rclcpp::shutdown();
        }
    }



    bool cbSrvTrgWriteCommand(std::shared_ptr<std_srvs::srv::Trigger::Request> req
                                , std::shared_ptr<std_srvs::srv::Trigger::Response> res        // NOLINT
                                , const std::vector<uint8_t>& bytes)
    {
        bool ret = sendBytes(bytes);
        if (ret)
        {
            res->message = "Success";
            res->success = true;
        }
        else
        {
            res->message = "Failed";
            res->success = false;
        }
        return true;
    }

    void cbWdg()
    {
        if (port_.is_open())
        {
            RCLCPP_ERROR(node->get_logger(), "Timeouted. No data received from IMU.");
            rclcpp::shutdown();
        }
    }

    void resetWdg()
    {
        wdg_->cancel();
        wdg_->reset();
    }

    bool sendBytes(const std::vector<uint8_t>& bytes)
    {
        boost::system::error_code ec;
        const size_t w_len = ba::write(port_,
                                    ba::buffer(bytes),
                                    ec);
        if (w_len != bytes.size())
        {
            RCLCPP_WARN(node->get_logger(), "Could not send full length of packet.");
            return false;
        }
        else if (ec.value() != 0)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to write. Error code : %d",
                    ec.value());
            return false;
        }
        return true;
    }
    
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg){
        current_time = clock_->now();
        prev_time = current_time;
    }
    
    void cmd_vel_check(){
        current_time = clock_->now();

        if(current_time - prev_time > rclcpp::Duration(60) && !enabled_auto_cali_status){
            bool ret = sendBytes(ptr_imu_->enableAutoGyroCali());
            if (ret)
            {
                RCLCPP_ERROR(node->get_logger(), "Successfully ENABLED Gyro Auto-calibration");
                enabled_auto_cali_status = true;
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to ENABLED Gyro Auto-calibration");
            }
        } else if (current_time - prev_time <= rclcpp::Duration(60) && enabled_auto_cali_status) {
            bool ret = sendBytes(ptr_imu_->diableAutoGyroCali());
            if (ret)
            {
                RCLCPP_ERROR(node->get_logger(), "Successfully DISABLED Gyro Auto-calibration");
                enabled_auto_cali_status = false;
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed ENABLED Gyro Auto-calibration");
            }
        }
    }
};
}   // namespace wit_imu_driver

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<wit_imu_driver::WitImuDriver> imu_obj= std::make_shared<wit_imu_driver::WitImuDriver>();   
    RCLCPP_INFO(imu_obj->node->get_logger(), "Start wit_imu_driver");

    if (!imu_obj->open())
    {
        RCLCPP_ERROR(imu_obj->node->get_logger(), "[wit_imu_driver] Failed to open");
        return 1;
    }

    if (!imu_obj->spin())
    {
        RCLCPP_ERROR(imu_obj->node->get_logger(), "[wit_imu_driver] Exit by error");
        return 1;
    }
    
    return 0;
}
