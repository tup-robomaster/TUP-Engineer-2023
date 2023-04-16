#include "../include/serialport_node.hpp"

using namespace std::placeholders;
namespace serialport
{
    SerialPortNode::SerialPortNode(const rclcpp::NodeOptions &options)
        : Node("serial_port", options), device_name_("ttyACM0"), baud_(115200)
    {
        RCLCPP_WARN(this->get_logger(), "Serialport node...");
        try
        {
            serial_port_ = initSerialPort();
            data_transform_ = initDataTransform();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while initializing serial port: %s", e.what());
        }

        // QoS
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        qos.durability_volatile();

        // detector_information订阅
        RCLCPP_WARN(this->get_logger(), "Detect!!!");
        target_info_sub_ = this->create_subscription<TargetMsg>(
            "/target_pub",
            qos,
            std::bind(&SerialPortNode::TargetMsgSub, this, _1));

        // stone_msg订阅
        stone_info_sub_ = this->create_subscription<StoneMsg>(
            "/stone_msg",
            qos,
            std::bind(&SerialPortNode::StoneMsgSub, this, _1));

        // 创建发送数据定时器
        //  timer_ = this->create_wall_timer(5ms, std::bind(&SerialPortNode::sendData, this));
        timer_ = rclcpp::create_timer(this, this->get_clock(), 500ms, std::bind(&SerialPortNode::serialWatcher, this));

        // if (using_port_)
        // { // Use serial port.
        //     if (serial_port_->openPort())
        //     {
        //         serial_msg_pub_ = this->create_publisher<SerialMsg>("/serial_msg", qos);
        //         // joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
        //         receive_thread_ = std::thread(&SerialPortNode::receiveData, this);
        //     }
        // }
    }

    SerialPortNode::~SerialPortNode()
    {
        if (receive_thread_.joinable())
            receive_thread_.join();
    }

    void SerialPortNode::serialWatcher()
    {
        if (access(serial_port_->serial_data_.device.path.c_str(), F_OK) == -1 || !serial_port_->serial_data_.is_initialized)
        {
            serial_port_->serial_data_.is_initialized = true;
            if (!serial_port_->openPort())
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Port open failed!!!");
            }
        }
    }

    // void SerialPortNode::receiveData()
    // {
    //     while (1)
    //     {
    //         // 若串口离线则跳过数据发送
    //         if (!serial_port_->serial_data_.is_initialized)
    //         {
    //             RCLCPP_INFO_THROTTLE(this->get_logger(), this->serial_port_->steady_clock_, 1000, "Serial port offline!!!");
    //             usleep(5000);
    //             continue;
    //         }

    //         // 数据读取不成功进行循环
    //         // while (!serial_port_->receiveData())
    //         // {
    //             // RCLCPP_INFO_THROTTLE(this->get_logger(), this->serial_port_->steady_clock_, 1000, "CHECKSUM FAILED OR NO DATA RECVIED!!!");
    //             // usleep(5000);
    //         // }

    //         // uchar mode = serial_port_->serial_data_.rdata[1];
    //         uchar mode = 1;
    //         mode_ = mode;
    //         RCLCPP_INFO_THROTTLE(this->get_logger(), this->serial_port_->steady_clock_, 1000, "mode:%d", mode);
    //         // RCLCPP_INFO(this->get_logger(), "mode:%d", mode);

    //         // if (mode)
    //         // {
    //         //     // RCLCPP_INFO_THROTTLE(this->get_logger(), this->serial_port_->steady_clock_, 1000, "mode:%d", mode);

    //         //     if (print_serial_info_)
    //         //     {
    //         //         RCLCPP_INFO(this->get_logger(), "mode:%d", mode);
    //         //     }

    //         //     SerialMsg serial_msg;
    //         //     serial_msg.imu.header.frame_id = "imu_link";
    //         //     serial_msg.imu.header.stamp = this->get_clock()->now();
    //         //     serial_msg.mode = mode;
    //         //     serial_msg_pub_->publish(std::move(serial_msg));
    //         // }
    //     }
    // }

    void SerialPortNode::TargetMsgSub(TargetMsg::SharedPtr target_info)
    {
        // int mode = mode_;
        int mode = 1;
        std::cout<<target_info->is_target<<std::endl;
        if (this->using_port_)
        {
            VisionData vision_data;
            if (mode == STONE_STATION_DETECT)
            {
                RCLCPP_WARN(this->get_logger(), "Sub stone station msg!!!");
                mutex_.lock();
                vision_data =
                    {
                        (serial_port_->steady_clock_.now().nanoseconds() / 1e6),
                        (float)target_info->pitch,
                        (float)target_info->yaw,
                        (float)target_info->roll,
                        (float)target_info->x_dis,
                        (float)target_info->y_dis,
                        (float)target_info->z_dis,
                        target_info->is_target,
                        // 1
                    };

                // 根据不同mode进行对应的数据转换
                data_transform_->transformData(mode, vision_data, serial_port_->Tdata);
                // 数据发送
                serial_port_->sendData();
                mutex_.unlock();
                flag_ = true;
            }
        }
        else
        { // Debug without com.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "No sub target_info msg...");
        }
    }

    void SerialPortNode::StoneMsgSub(StoneMsg::SharedPtr stone_info)
    {
        int mode = mode_;
        RCLCPP_WARN(this->get_logger(), "Mode:%d", mode);
        if (this->using_port_)
        {
            VisionData vision_data;
            if (mode == STONE_DETECT)
            {
                mutex_.lock();
                vision_data =
                    {
                        (serial_port_->steady_clock_.now().nanoseconds() / 1e6),
                        (float)stone_info->up,
                        (float)stone_info->down,
                        (float)stone_info->left,
                        (float)stone_info->right,
                    };

                // 根据不同mode进行对应的数据转换
                data_transform_->transformData(mode, vision_data, serial_port_->Tdata);
                // 数据发送
                serial_port_->sendData();
                mutex_.unlock();
                flag_ = true;
            }
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "No sub stnoe_info msg...");
        }
    }

    bool SerialPortNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = params_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            this->using_port_ = param.as_bool();
            break;
        case 1:
            this->baud_ = param.as_int();
            break;
        case 3:
            this->print_serial_info_ = param.as_bool();
            break;
        default:
            break;
        }
        return true;
    }

    rcl_interfaces::msg::SetParametersResult SerialPortNode::paramsCallback(const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        for (const auto &param : params)
        {
            result.successful = setParam(param);
        }
        return result;
    }

    std::unique_ptr<SerialPort> SerialPortNode::initSerialPort()
    {
        params_map_ =
            {
                {"using_port", 0},
                {"baud", 1},
                {"print_serial_info", 3}};

        this->declare_parameter<std::string>("port_id", "483/5740/200");
        this->get_parameter("port_id", id_);

        this->declare_parameter<int>("baud", 115200);
        this->get_parameter("baud", baud_);

        this->declare_parameter<bool>("using_port", false);
        this->get_parameter("using_port", using_port_);

        this->declare_parameter("print_serial_info", false);
        this->get_parameter("print_serial_info", this->print_serial_info_);

        return std::make_unique<SerialPort>(id_, baud_, using_port_);
    }

    std::unique_ptr<DataTransform> SerialPortNode::initDataTransform()
    {
        return std::make_unique<DataTransform>();
    }
} // namespace serialport

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<serialport::SerialPortNode>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serialport::SerialPortNode)