// #include "serialport.hpp"
// #include "../include/serialport_node.hpp"

// using std::placeholders::_1;

// namespace serialport
// {
//   serial_node::serial_node(const rclcpp::NodeOptions& options)
//   : Node("serial_port", options), device_name_("ttyACM0"), baud_(115200)
//   {
    
//     RCLCPP_WARN(this->get_logger(), "Serialport node... ");
//     try
//     {
//       serial_port_ = init_serial_port();
//     }
//     catch(const std::exception& e)
//     {
//       RCLCPP_ERROR(this->get_logger(), "Error while initializing serial port: %s", e.what());
//     }

//     station_info_sub_ = this->create_subscription<TargetMsg>("/station_processor/target_info", 10, std::bind(&serial_node::send_station_data, this, _1) );
//     // stone_info_sub_ = this->create_subscription<StoneMsg>("/stone_processor/stone_info", 10, std::bind(&seriaport::param_callback, this, _1) );
    
//   }

//   serial_node::~serial_node()
//   {

//   }

//   void serial_node::receive_data()
//   {
//     while(1)
//     {
//       if (!serial_port_->serial_data_.is_initialized)
//       {
//           std::cout << "Offline..." << std::endl;
//           usleep(5000);
//           continue;
//       }
//       while (!serial_port_->get_Mode(bytes_num_))
//         ;
           
//     }
//   }

//   void serial_node::send_station_data(TargetMsg::SharedPtr target_info)
//   {
//     if(!this->deubug_without_port)
//     {
//       if(serial_port_->mode == 1 || serial_port_->mode == 2)
//       {
//         RCLCPP_INFO(this->get_logger(), "current mode is %d", serial_port_->mode);
//         VisionData transmition_data = {target_info->pitch, target_info->yaw, target_info->roll, target_info->x_dis, target_info->y_dis, target_info->z_dis ,1};
//         seria_port_->transformData(transmition_data);
//         serial_port_->send();
//       }
//       else
//       {   // Debug without com.
//         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Sub station msg...");
//       }
//     }
//   }

// }

// int main(int argc, char* argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<serialport::serial_node>());
//     rclcpp::shutdown();
//     return 0;
// }

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(serialport::serial_node)


#include "../include/serialport_node.hpp"

// #define SENTRY_RECV_NORMAL 0x05
using namespace std::placeholders;
namespace serialport
{
    SerialPortNode::SerialPortNode(const rclcpp::NodeOptions& options)
    : Node("serial_port", options), device_name_("ttyACM0"), baud_(115200)
    {
        RCLCPP_WARN(this->get_logger(), "Serialport node...");
        try
        {
            serial_port_ = initSerialPort();
            data_transform_ = initDataTransform();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while initializing serial port: %s", e.what());
        }

        // QoS
        rclcpp::QoS qos(0);
        qos.keep_last(10);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        qos.durability_volatile();
        
        //自瞄msg订阅
        target_info_sub_ = this->create_subscription<TargetMsg>(
            "/station_info", 
            qos,
            std::bind(&SerialPortNode::TargetMsgSub, this, _1)
        );
        
        //能量机关msg订阅
        // stone_info_sub_ = this->create_subscription<GimbalMsg>(
        //     "/buff_processor/gimbal_info",
        //     qos,
        //     std::bind(&SerialPortNode::buffMsgSub, this, _1)
        // );
        
        auto publish_message = [this]() -> void
        {
            RCLCPP_INFO(this->get_logger(), "Callback function...");        
        };
        
        //创建发送数据定时器
        // timer_ = this->create_wall_timer(5ms, std::bind(&SerialPortNode::sendData, this));
        timer_ = rclcpp::create_timer(this, this->get_clock(), 5ms, std::bind(&SerialPortNode::sendData, this));

        if(!debug_without_port_)
        {   // Use serial port.
            if(serial_port_->openPort())
            {
                // serial_msg_pub_ = this->create_publisher<SerialMsg>("/serial_msg", qos);
                // joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
                receive_thread_ = std::thread(&SerialPortNode::receiveData, this);
            }
        }
    }

    SerialPortNode::~SerialPortNode()
    {
        if(receive_thread_.joinable())
            receive_thread_.join();
    }
    
    // void SerialPortNode::receiveData()
    // {
    //     while(1)
    //     {
    //         // 若串口离线则跳过数据发送
    //         if (!serial_port_->serial_data_.is_initialized)
    //         {
    //             RCLCPP_WARN(this->get_logger(), "Serial port offline!!!");
    //             usleep(5000);
    //             continue;
    //         }

    //         // 数据读取不成功进行循环
    //         while (!serial_port_->receiveData())
    //         {
    //             RCLCPP_WARN(this->get_logger(),"CHECKSUM FAILED OR NO DATA RECVIED!!!");
    //             usleep(2000);
    //         }
            
    //         uchar mode = serial_port_->serial_data_.rdata[1];
    //         mode_ = mode;

    //         std::vector<float> quat(4);
    //         std::vector<float> gyro(3);
    //         std::vector<float> acc(3);
    //         float bullet_speed;

    //         //Process IMU Datas
    //         data_transform_->getQuatData(&serial_port_->serial_data_.rdata[3], quat);
    //         data_transform_->getGyroData(&serial_port_->serial_data_.rdata[19], gyro);
    //         data_transform_->getAccData(&serial_port_->serial_data_.rdata[31], acc);
    //         data_transform_->getBulletSpeed(&serial_port_->serial_data_.rdata[43], bullet_speed);

    //         SerialMsg serial_msg;
    //         serial_msg.imu.header.frame_id = "imu_link";
    //         serial_msg.imu.header.stamp = this->get_clock()->now();
    //         serial_msg.mode = mode;
    //         serial_msg.bullet_speed = bullet_speed;
    //         serial_msg.imu.orientation.w = quat[0];
    //         serial_msg.imu.orientation.x = quat[1];
    //         serial_msg.imu.orientation.y = quat[2];
    //         serial_msg.imu.orientation.z = quat[3];
    //         serial_msg.imu.angular_velocity.x = gyro[0];
    //         serial_msg.imu.angular_velocity.y = gyro[1];
    //         serial_msg.imu.angular_velocity.z = gyro[2];
    //         serial_msg.imu.linear_acceleration.x = acc[0];
    //         serial_msg.imu.linear_acceleration.y = acc[1];
    //         serial_msg.imu.linear_acceleration.z = acc[2];
    //         serial_msg_pub_->publish(std::move(serial_msg));

    //         if (mode == SENTRY_RECV_NORMAL)
    //         {
    //             float theta;
    //             data_transform_->getThetaAngle(&serial_port_->serial_data_.rdata[47], theta);

    //             sensor_msgs::msg::JointState joint_state;
    //             joint_state.header.stamp = this->get_clock()->now();
    //             joint_state.name.push_back("gimbal_yaw_joint");
    //             joint_state.name.push_back("gimbal_pitch_joint");
    //             joint_state.position.push_back(theta);
    //             joint_state.position.push_back(0);
    //             joint_state_pub_->publish(joint_state);
    //         }
    //     }
    // }

    /**
     * @brief 数据发送回调函数
     * 
     */
    void SerialPortNode::sendData()
    {
        VisionData vision_data = {0.0, (float)0.0, (float)0.0, (float)0.0, (float)0.0, (float)0.0, (float)0.0, 1};
        if(flag_)
        {
            auto now = (serial_port_->steady_clock_.now().nanoseconds() / 1e6);
            mutex_.lock();
            if(abs(now - vision_data_.timestamp) < 30) //(ms)，若时间差过大，则忽略此帧数据
            {
                vision_data = vision_data_;
            }
            mutex_.unlock();
            flag_ = false;
        }
        //根据不同mode进行对应的数据转换
        data_transform_->transformData(mode_, vision_data, serial_port_->Tdata);
        //数据发送
        serial_port_->sendData();

        return;
    }

    void SerialPortNode::TargetMsgSub(TargetMsg::SharedPtr target_info) 
    {
        if(!this->debug_without_port_)
        {
            if(mode_ == STONE_STATION_DETACTE)
            {
                mutex_.lock();
                vision_data_ = 
                {
                    (serial_port_->steady_clock_.now().nanoseconds() / 1e6),
                    (float)target_info->pitch, 
                    (float)target_info->yaw,
                    (float)target_info->roll,
                    (float)target_info->x_dis,
                    (float)target_info->y_dis, 
                    (float)target_info->z_dis, 
                    1
                };
                mutex_.unlock();
                flag_ = true;
            }
        }
        else
        {   // Debug without com.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Sub autoaim msg...");
        }
    }

    // void SerialPortNode::buffMsgSub(GimbalMsg::SharedPtr target_info) 
    // {
    //     if(!this->debug_without_port_)
    //     {
    //         if(mode_ == SMALL_BUFF || mode_ == BIG_BUFF)
    //         {
    //             mutex_.lock();
    //             vision_data_ = 
    //             {
    //                 (serial_port_->steady_clock_.now().nanoseconds() / 1e6),
    //                 (float)target_info->pitch, 
    //                 (float)target_info->yaw, 
    //                 (float)target_info->distance, 
    //                 target_info->is_switched, 
    //                 1, 
    //                 target_info->is_spinning, 
    //                 0
    //             };
    //             mutex_.unlock();
    //             flag_ = true;
    //         }
    //     }
    //     else
    //     {
    //         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Sub buff msg...");
    //     }
    // }

    bool SerialPortNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = params_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            this->debug_without_port_ = param.as_bool();
            break;
        case 1:
            this->baud_ = param.as_int();
            break;
        default:
            break;
        }
        return true;
    }
    
    rcl_interfaces::msg::SetParametersResult SerialPortNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        for(const auto& param : params)
        {
            result.successful = setParam(param);
        }
        return result;
    }

    std::unique_ptr<SerialPort> SerialPortNode::initSerialPort()
    {
        params_map_ =
        {
            {"debug_without_com", 0},
            {"baud", 1}
        };

        this->declare_parameter<std::string>("port_id", "483/5740/200");
        this->get_parameter("port_id", id_);

        this->declare_parameter<int>("baud", 115200);
        this->get_parameter("baud", baud_);

        this->declare_parameter<bool>("debug_without_com", false);
        this->get_parameter("debug_without_com", debug_without_port_);

        return std::make_unique<SerialPort>(id_, baud_, debug_without_port_);
    }

    std::unique_ptr<DataTransform> SerialPortNode::initDataTransform()
    {
        return std::make_unique<DataTransform>();
    }
} //namespace serialport

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<serialport::SerialPortNode>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serialport::SerialPortNode)