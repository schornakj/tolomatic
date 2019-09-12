/**
 * @file servo_node.cpp
 * @brief Tolomatic ACSI servo interface using Ethernet/IP - ROS node to publish
 *data & handle service calls.
 *
 * @author Bill McCormick <wmccormick@swri.org>
 * @date Feb 13, 2019
 * @version 0.1
 * @bug Implicit messaging not functional
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <rclcpp/rclcpp.hpp>
#include <boost/shared_ptr.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"

#include "servo.h"

using std::cout;
using std::endl;
using boost::shared_ptr;
using eip::socket::TCPSocket;
using eip::socket::UDPSocket;

using namespace acsi_eip_driver;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::sleep_for(std::chrono::seconds(3));
  auto node = rclcpp::Node::make_shared("servo");

  //ros::Time::init();

  double throttle_time;
  node->declare_parameter("throttle");
  if(!node->get_parameter<double>("throttle", throttle_time))
  {
    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'throttle', using default instead.");
    throttle_time = 10.0;
  }
  rclcpp::Rate throttle(throttle_time);

  // get sensor config from params
  string host;
  node->declare_parameter("host");  
  if(!node->get_parameter<std::string>("host", host))
  {
    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'host', using default instead.");
    host = "192.168.100.10";
  }
  RCLCPP_INFO(node->get_logger(), "Host is: %s", host.c_str());

  //This will be needed for implicit messaging
  string local_ip;
  node->declare_parameter("local_ip");    
  if(!node->get_parameter<std::string>("local_ip", local_ip))
  {
    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'local_ip', using default instead.");
    local_ip = "0.0.0.0";
  }
  
  // optionally publish ROS joint_state messages
  bool publish_joint_state;
  string joint_name = "drive1";
  string joint_states_topic = "joint_states";
  node->declare_parameter("publish_joint_state");      
  node->declare_parameter("joint_name");      
  node->declare_parameter("joint_states_topic");        
  if(!node->get_parameter<bool>("publish_joint_state", publish_joint_state))
  {
    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'publish_joint_state', using default instead.");
    publish_joint_state = false;
  }
  if (publish_joint_state)
  {
    if(!node->get_parameter<std::string>("joint_name", joint_name))
    {
      RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'joint_name', using default instead.");
      joint_name = "drive1";
    }  if(!node->get_parameter<std::string>("joint_states_topic", joint_states_topic))
    {
      RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'joint_states_topic', using default instead.");
      joint_states_topic = "joint_states";
    }
  }

  boost::asio::io_service io_service;
  shared_ptr<TCPSocket> socket = shared_ptr<TCPSocket>(new TCPSocket(io_service));
  shared_ptr<UDPSocket> io_socket = shared_ptr<UDPSocket>(new UDPSocket(io_service, 0, local_ip));

  auto servo = std::make_shared<ACSI>(socket, io_socket);

  RCLCPP_INFO(node->get_logger(), "Socket created");

  try
  {
    servo->open(host);
    RCLCPP_INFO(node->get_logger(), "Host is open");
    InputAssembly test = servo->getDriveData();  
    servo->updateDriveStatus(test); 
  }
  catch (std::runtime_error& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "Exception caught opening session: %s", ex.what());
    return -1;
  }
  
  InputAssembly test2 = servo->getDriveData();  
  servo->updateDriveStatus(test2);

  try
  {
    // get status
  }
  catch (std::invalid_argument& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "Invalid arguments in sensor configuration: %s", ex.what());
    return -1;
  }

  try
  {
    // TODO: Setup implicit messaging here
    // stepper.startUDPIO();
    // ROS_INFO_STREAM("UDP Started");
  }
  catch (std::logic_error& ex)
  {
    RCLCPP_FATAL(node->get_logger(), "Could not start UDP IO: %s", ex.what());
    return -1;
  }

  float default_accel;
  node->declare_parameter("default_accel");      
  if(!node->get_parameter<float>("default_accel", default_accel))
  {
    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'default_accel', using default instead.");
    default_accel = 100.0;
  }
  servo->so.accel = default_accel;

  float default_decel;
  node->declare_parameter("default_decel");      
  if(!node->get_parameter<float>("default_decel", default_decel))
  {
    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'default_decel', using default instead.");
    default_decel = 100.0;
  }
  servo->so.decel = default_decel;

  float default_force;
  node->declare_parameter("default_force");      
  if(!node->get_parameter<float>("default_force", default_force))
  {
    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'default_force', using default instead.");
    default_force = 30.0;
  }
  servo->so.force = default_force;

  float default_velocity;
  node->declare_parameter("default_velocity");      
  if(!node->get_parameter<float>("default_velocity", default_velocity))
  {
    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'default_velocity', using default instead.");
    default_velocity = 10.0;
  }
  servo->so.velocity = default_velocity;

  // publisher for stepper status
  auto servo_pub = node->create_publisher<tolomatic_msgs::msg::AcsiInputs>("inputs", 1);
  auto status_pub = node->create_publisher<tolomatic_msgs::msg::AcsiStatus>("status", 1);
  
  // publisher and message for joint state
  sensor_msgs::msg::JointState joint_state;
  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 1);


  // services
  auto enable_service = node->create_service<std_srvs::srv::SetBool>("enable", std::bind(&ACSI::enable, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  auto estop_service = node->create_service<std_srvs::srv::SetBool>("estop", std::bind(&ACSI::estop, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  auto moveSelect_service = node->create_service<tolomatic_msgs::srv::AcsiMoveSelect>("moveSelect", std::bind(&ACSI::moveSelect, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  auto moveHome_service = node->create_service<std_srvs::srv::Trigger>("moveHome", std::bind(&ACSI::moveHome, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  auto moveStop_service = node->create_service<std_srvs::srv::Trigger>("moveStop", std::bind(&ACSI::moveStop, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  auto moveVelocity_service = node->create_service<tolomatic_msgs::srv::AcsiMoveVelocity>("moveVelocity", std::bind(&ACSI::moveVelocity, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  auto moveAbsolute_service = node->create_service<tolomatic_msgs::srv::AcsiMoveAbsolute>("moveAbsolute", std::bind(&ACSI::moveAbsolute, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  auto moveIncremental_service = node->create_service<tolomatic_msgs::srv::AcsiMoveIncremental>("moveIncremental", std::bind(&ACSI::moveIncremental, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  auto moveRotary_service = node->create_service<tolomatic_msgs::srv::AcsiMoveRotary>("moveRotary", std::bind(&ACSI::moveRotary, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  auto setHome_service = node->create_service<std_srvs::srv::Trigger>("setHome", std::bind(&ACSI::setHome, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  auto setProfile_service = node->create_service<tolomatic_msgs::srv::AcsiSetProfile>("setProfile", std::bind(&ACSI::setProfile, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  // not implimented
  // ros::ServiceServer estop_service = nh.advertiseService("estop",
  // &STEPPER::estop, &stepper);

  while (rclcpp::ok())
  {
    try
    {
      // Collect status from controller, convert to ROS message format.
      InputAssembly test = servo->getDriveData();
      servo->updateDriveStatus(test);
      //servo.updateDriveStatus(servo.getDriveData());
      

      if (publish_joint_state)
      {
        joint_state.header.stamp = rclcpp::Clock().now();
        joint_state.name.resize(1);
        joint_state.position.resize(1);
        joint_state.name[0] = joint_name;
        // TODO: See issue #2
        joint_state.position[0] = double(servo->ss.current_position) / 1000.0;
        joint_state_pub->publish(joint_state);
      }

      // publish stepper inputs
      servo_pub->publish(servo->si);

      // publish stepper status
      status_pub->publish(servo->ss);

      // set outputs to stepper drive controller
      servo->setDriveData();
    }
    catch (std::runtime_error& ex)
    {
      RCLCPP_ERROR(node->get_logger(), "Exception caught requesting scan data: %s", ex.what());
    }
    catch (std::logic_error& ex)
    {
      RCLCPP_ERROR(node->get_logger(), "Problem parsing return data: ", ex.what());
    }

    rclcpp::spin_some(node);
    throttle.sleep();
  }

  servo->closeConnection(0);
  servo->close();
  return 0;
}
