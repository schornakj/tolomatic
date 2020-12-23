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

#include <tolomatic_msgs/srv/acsi_set_profile.hpp>
#include <tolomatic_msgs/srv/acsi_move_absolute.hpp>
#include <tolomatic_msgs/srv/acsi_move_incremental.hpp>
#include <tolomatic_msgs/srv/acsi_move_rotary.hpp>
#include <tolomatic_msgs/srv/acsi_move_select.hpp>
#include <tolomatic_msgs/srv/acsi_move_velocity.hpp>

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "odva_ethernetip/socket/tcp_socket.h"
#include "odva_ethernetip/socket/udp_socket.h"

#include "servo.h"

static const std::string SERVICE_NAME_ENABLE = "enable";
static const std::string SERVICE_NAME_ESTOP = "estop";
static const std::string SERVICE_NAME_MOVE_SELECT = "moveSelect";
static const std::string SERVICE_NAME_MOVE_HOME = "moveHome";
static const std::string SERVICE_NAME_MOVE_STOP = "moveStop";
static const std::string SERVICE_NAME_MOVE_VELOCITY = "moveVelocity";
static const std::string SERVICE_NAME_MOVE_ABSOLUTE = "moveAbsolute";
static const std::string SERVICE_NAME_MOVE_INCREMENTAL = "moveIncremental";
static const std::string SERVICE_NAME_MOVE_ROTARY = "moveRotary";
static const std::string SERVICE_NAME_SET_HOME = "setHome";
static const std::string SERVICE_NAME_SET_PROFILE = "setProfile";

static const std::string TOPIC_NAME_JOINT_STATES = "joint_states";
static const std::string TOPIC_NAME_STATUS = "status";

static const std::string PARAM_NAME_PUBLISH_JOINT_STATE = "publish_joint_state";
static const std::string PARAM_NAME_JOINT_NAME = "joint_name";
static const std::string PARAM_NAME_THROTTLE = "throttle";
static const std::string PARAM_NAME_HOST = "host";
static const std::string PARAM_NAME_LOCAL_IP = "local_ip";
static const std::string PARAM_NAME_DEFAULT_ACCEL = "default_accel";
static const std::string PARAM_NAME_DEFAULT_DECEL = "default_decel";
static const std::string PARAM_NAME_DEFAULT_FORCE = "default_force";
static const std::string PARAM_NAME_DEFAULT_VELOCITY = "default_velocity";

using std::cout;
using std::endl;
using boost::shared_ptr;
using eip::socket::TCPSocket;
using eip::socket::UDPSocket;

using namespace acsi_eip_driver;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class ServoNode : public rclcpp::Node
{
public:
  ServoNode()
    : Node("servo_node")
    , cb_group_reentrant_(this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant))
    , enable_service_(this->create_service<std_srvs::srv::SetBool>(SERVICE_NAME_ENABLE, std::bind(&ServoNode::enable_cb, this, _1, _2, _3), rmw_qos_profile_services_default, cb_group_reentrant_))
//    , estop_service_(this->create_service<std_srvs::srv::SetBool>(SERVICE_NAME_ESTOP, std::bind(&ACSI::estop, this, _1, _2, _3)))
//    , move_select_service_(this->create_service<tolomatic_msgs::srv::AcsiMoveSelect>(SERVICE_NAME_ENABLE, std::bind(&ACSI::moveSelect, this, _1, _2, _3)))
    , move_home_service_(this->create_service<std_srvs::srv::Trigger>(SERVICE_NAME_MOVE_HOME, std::bind(&ServoNode::move_home_cb, this, _1, _2, _3), rmw_qos_profile_services_default, cb_group_reentrant_))
//    , move_stop_service_(this->create_service<std_srvs::srv::Trigger>(SERVICE_NAME_MOVE_STOP, std::bind(&ACSI::moveStop, this, _1, _2, _3)))
//    , move_vel_service_(this->create_service<tolomatic_msgs::srv::AcsiMoveVelocity>(SERVICE_NAME_MOVE_VELOCITY, std::bind(&ACSI::moveVelocity, this, _1, _2, _3)))
    , move_abs_service_(this->create_service<tolomatic_msgs::srv::AcsiMoveAbsolute>(SERVICE_NAME_MOVE_ABSOLUTE, std::bind(&ServoNode::move_absolute_cb, this, _1, _2, _3), rmw_qos_profile_services_default, cb_group_reentrant_))
//    , move_incr_service_(this->create_service<tolomatic_msgs::srv::AcsiMoveIncremental>(SERVICE_NAME_MOVE_INCREMENTAL, std::bind(&ACSI::moveIncremental, this, _1, _2, _3)))
//    , move_rot_service_(this->create_service<tolomatic_msgs::srv::AcsiMoveRotary>(SERVICE_NAME_MOVE_ROTARY, std::bind(&ACSI::moveRotary, this, _1, _2, _3)))
//    , set_home_service_(this->create_service<std_srvs::srv::Trigger>(SERVICE_NAME_SET_HOME, std::bind(&ACSI::setHome, this, _1, _2, _3)))
//    , set_profile_service_(this->create_service<tolomatic_msgs::srv::AcsiSetProfile>(SERVICE_NAME_SET_PROFILE, std::bind(&ACSI::setProfile, this, _1, _2, _3)))
//    , servo_status_pub_(this->create_publisher<tolomatic_msgs::msg::AcsiStatus>(TOPIC_NAME_STATUS, rclcpp::QoS(1)))
//    , joint_state_pub_(this->create_publisher<sensor_msgs::msg::JointState>(TOPIC_NAME_JOINT_STATES, rclcpp::QoS(1)))

  {
    this->declare_parameter(PARAM_NAME_THROTTLE, 0.1);
    this->declare_parameter(PARAM_NAME_HOST);
    this->declare_parameter(PARAM_NAME_LOCAL_IP);

    this->declare_parameter(PARAM_NAME_DEFAULT_ACCEL, 100.0);
    this->declare_parameter(PARAM_NAME_DEFAULT_DECEL, 100.0);
    this->declare_parameter(PARAM_NAME_DEFAULT_FORCE, 30.0);
    this->declare_parameter(PARAM_NAME_DEFAULT_VELOCITY, 10.0);

    // optionally publish ROS joint_state messages
    this->declare_parameter(PARAM_NAME_PUBLISH_JOINT_STATE, false);
    this->declare_parameter(PARAM_NAME_JOINT_NAME, "drive1");

    rclcpp::PublisherOptions options;
    options.callback_group = cb_group_reentrant_;

    servo_status_pub_ = this->create_publisher<tolomatic_msgs::msg::AcsiStatus>(TOPIC_NAME_STATUS, rclcpp::QoS(1), options);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(TOPIC_NAME_JOINT_STATES, rclcpp::QoS(1), options);
  }

  bool init()
  {
    joint_state_.name.resize(1);
    joint_state_.position.resize(1);
    joint_state_.name[0] = this->get_parameter(PARAM_NAME_JOINT_NAME).as_string();
    joint_state_.position[0] = 0.0;

    std::string host;
    if(!this->get_parameter<std::string>(PARAM_NAME_HOST, host))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to lookup required parameter 'host'.");
      return false;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Host IP is: " << host.c_str());

    string local_ip;
    if(!this->get_parameter<std::string>(PARAM_NAME_LOCAL_IP, local_ip))
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Failed to lookup parameter 'local_ip', using default instead.");
      local_ip = "0.0.0.0";
    }

    boost::asio::io_service io_service;
    shared_ptr<TCPSocket> socket = shared_ptr<TCPSocket>(new TCPSocket(io_service));
    shared_ptr<UDPSocket> io_socket = shared_ptr<UDPSocket>(new UDPSocket(io_service, 0, local_ip));

    std::scoped_lock servo_lock(servo_guard_);

    servo_ = std::make_shared<ACSI>(socket, io_socket);

    RCLCPP_INFO_STREAM(this->get_logger(), "Socket created");

    try
    {
      servo_->open(host);
      RCLCPP_INFO_STREAM(this->get_logger(), "Host is open");
      InputAssembly test = servo_->getDriveData();
      servo_->updateDriveStatus(test);
    }
    catch (const std::runtime_error& e)
    {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Exception caught opening session: " << e.what());
      return false;
    }

    servo_->so.accel = static_cast<float>(this->get_parameter(PARAM_NAME_DEFAULT_ACCEL).as_double());
    servo_->so.decel = static_cast<float>(this->get_parameter(PARAM_NAME_DEFAULT_DECEL).as_double());
    servo_->so.force = static_cast<float>(this->get_parameter(PARAM_NAME_DEFAULT_FORCE).as_double());
    servo_->so.velocity = static_cast<float>(this->get_parameter(PARAM_NAME_DEFAULT_VELOCITY).as_double());

    update_timer_ = this->create_wall_timer(std::chrono::duration<std::double_t>(this->get_parameter(PARAM_NAME_THROTTLE).as_double()), std::bind(&ServoNode::on_update_timer, this), cb_group_reentrant_);
    joint_state_pub_timer_ = this->create_wall_timer(std::chrono::duration<std::double_t>(this->get_parameter(PARAM_NAME_THROTTLE).as_double()), std::bind(&ServoNode::on_joint_state_timer, this), cb_group_reentrant_);
    return true;
  }

  void cleanup()
  {
    std::scoped_lock servo_lock(servo_guard_);
    update_timer_->cancel();
    servo_->closeConnection(0);
    servo_->close();
  }

private:

  void on_update_timer()
  {
    try
    {
      std::scoped_lock servo_lock(servo_guard_);
      // Collect status from controller, convert to ROS message format.
      InputAssembly test = servo_->getDriveData();
      servo_->updateDriveStatus(test);

      {
        std::scoped_lock js_lock(js_guard_);
        // TODO: Convert from user-set units on servo driver to meters.
        // Currently assume that the servo driver reports positions in millimeters.
        // See issue #2.
        joint_state_.name[0] = this->get_parameter(PARAM_NAME_JOINT_NAME).as_string();
        joint_state_.header.stamp = this->now();
        joint_state_.position[0] = double(servo_->ss.current_position) / 1000.0;
      }

      // publish stepper inputs
//      servo_pub->publish(servo->si);

      // publish stepper status
      servo_status_pub_->publish(servo_->ss);

      // set outputs to stepper drive controller
      servo_->setDriveData();
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Exception caught requesting scan data: " << e.what());
    }
    catch (std::logic_error& e)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Problem parsing return data: " << e.what());
    }
  }

  void on_joint_state_timer()
  {
    if (this->get_parameter(PARAM_NAME_PUBLISH_JOINT_STATE).as_bool())
    {
      std::cout << "JS" << std::endl;
      std::scoped_lock js_lock(js_guard_);
      joint_state_pub_->publish(joint_state_);
    }
  }

  void move_absolute_cb(const std::shared_ptr<rmw_request_id_t> request_header,
                        const std::shared_ptr<tolomatic_msgs::srv::AcsiMoveAbsolute::Request> req,
                        std::shared_ptr<tolomatic_msgs::srv::AcsiMoveAbsolute::Response> res)
  {
    (void) request_header;

    if (servo_ == nullptr)
    {
      res->message = "Connection to servo was not initialized";
      res->success = false;
      return;
    }

    try
    {
      servo_->moveAbsolute(req->position);
    }
    catch (const std::runtime_error &e)
    {
      res->message = std::string("Failed to move to absolute position: ") + e.what();
      res->success = false;
      return;
    }

    res->message = "Sent command to move to absolute position";
    res->success = true;
    return;
  }

  void enable_cb(const std::shared_ptr<rmw_request_id_t> request_header,
                 const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                 const std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    (void) request_header;

    if (servo_ == nullptr)
    {
      res->message = "Connection to servo was not initialized";
      res->success = false;
      return;
    }

    try
    {
      servo_->setEnable(req->data);
    }
    catch (const std::runtime_error &e)
    {
      res->message = std::string("Failed to set enable state: ") + e.what();
      res->success = false;
      return;
    }

    res->message = "Set enabled state";
    res->success = true;
    return;
  }

  void move_home_cb(const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    const std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    (void) request_header;
    (void) req;

    if (servo_ == nullptr)
    {
      res->message = "Connection to servo was not initialized";
      res->success = false;
      return;
    }

    try
    {
      servo_->moveHome();
    }
    catch (const std::runtime_error &e)
    {
      res->message = std::string("Failed to move home: ") + e.what();
      res->success = false;
      return;
    }

    res->message = "Sent command to move home";
    res->success = true;
    return;
  }

//  auto enable_service = node->create_service<std_srvs::srv::SetBool>("enable", std::bind(&ACSI::enable, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto estop_service = node->create_service<std_srvs::srv::SetBool>("estop", std::bind(&ACSI::estop, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveSelect_service = node->create_service<tolomatic_msgs::srv::AcsiMoveSelect>("moveSelect", std::bind(&ACSI::moveSelect, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveHome_service = node->create_service<std_srvs::srv::Trigger>("moveHome", std::bind(&ACSI::moveHome, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveStop_service = node->create_service<std_srvs::srv::Trigger>("moveStop", std::bind(&ACSI::moveStop, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveVelocity_service = node->create_service<tolomatic_msgs::srv::AcsiMoveVelocity>("moveVelocity", std::bind(&ACSI::moveVelocity, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveAbsolute_service = node->create_service<tolomatic_msgs::srv::AcsiMoveAbsolute>("moveAbsolute", std::bind(&ACSI::moveAbsolute, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveIncremental_service = node->create_service<tolomatic_msgs::srv::AcsiMoveIncremental>("moveIncremental", std::bind(&ACSI::moveIncremental, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveRotary_service = node->create_service<tolomatic_msgs::srv::AcsiMoveRotary>("moveRotary", std::bind(&ACSI::moveRotary, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto setHome_service = node->create_service<std_srvs::srv::Trigger>("setHome", std::bind(&ACSI::setHome, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto setProfile_service = node->create_service<tolomatic_msgs::srv::AcsiSetProfile>("setProfile", std::bind(&ACSI::setProfile, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));


  std::shared_ptr<ACSI> servo_;

  sensor_msgs::msg::JointState joint_state_;

  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr joint_state_pub_timer_;

  std::mutex js_guard_;
  std::mutex servo_guard_;

  rclcpp::callback_group::CallbackGroup::SharedPtr cb_group_reentrant_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_service_;
  rclcpp::Service<tolomatic_msgs::srv::AcsiMoveSelect>::SharedPtr move_select_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_home_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_stop_service_;
  rclcpp::Service<tolomatic_msgs::srv::AcsiMoveVelocity>::SharedPtr move_vel_service_;
  rclcpp::Service<tolomatic_msgs::srv::AcsiMoveAbsolute>::SharedPtr move_abs_service_;
  rclcpp::Service<tolomatic_msgs::srv::AcsiMoveIncremental>::SharedPtr move_incr_service_;
  rclcpp::Service<tolomatic_msgs::srv::AcsiMoveRotary>::SharedPtr move_rot_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_home_service_;
  rclcpp::Service<tolomatic_msgs::srv::AcsiSetProfile>::SharedPtr set_profile_service_;

  rclcpp::Publisher<tolomatic_msgs::msg::AcsiInputs>::SharedPtr servo_input_pub_; // = node->create_publisher<tolomatic_msgs::msg::AcsiInputs>("inputs", 1);
  rclcpp::Publisher<tolomatic_msgs::msg::AcsiStatus>::SharedPtr servo_status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

//  auto status_pub = node->create_publisher<tolomatic_msgs::msg::AcsiStatus>("status", 1);


};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ServoNode>();
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  if(!node->init())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to initialize servo node");
    return -1;
  }
  exec->add_node(node);
  exec->spin();
  exec->remove_node(node);
  node->cleanup();
  return 0;

//  rclcpp::sleep_for(std::chrono::seconds(3));
//  auto node = rclcpp::Node::make_shared("servo");

//  //ros::Time::init();

//  double throttle_time;
//  node->declare_parameter("throttle");
//  if(!node->get_parameter<double>("throttle", throttle_time))
//  {
//    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'throttle', using default instead.");
//    throttle_time = 10.0;
//  }
//  rclcpp::Rate throttle(throttle_time);

//  // get sensor config from params
//  string host;
//  node->declare_parameter("host");
//  if(!node->get_parameter<std::string>("host", host))
//  {
//    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'host', using default instead.");
//    host = "192.168.100.10";
//  }
//  RCLCPP_INFO(node->get_logger(), "Host is: %s", host.c_str());

//  //This will be needed for implicit messaging
//  string local_ip;
//  node->declare_parameter("local_ip");
//  if(!node->get_parameter<std::string>("local_ip", local_ip))
//  {
//    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'local_ip', using default instead.");
//    local_ip = "0.0.0.0";
//  }
  
//  // optionally publish ROS joint_state messages
//  bool publish_joint_state;
//  string joint_name = "drive1";
//  string joint_states_topic = "joint_states";
//  node->declare_parameter("publish_joint_state");
//  node->declare_parameter("joint_name");
//  node->declare_parameter("joint_states_topic");
//  if(!node->get_parameter<bool>("publish_joint_state", publish_joint_state))
//  {
//    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'publish_joint_state', using default instead.");
//    publish_joint_state = false;
//  }
//  if (publish_joint_state)
//  {
//    if(!node->get_parameter<std::string>("joint_name", joint_name))
//    {
//      RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'joint_name', using default instead.");
//      joint_name = "drive1";
//    }

//    if(!node->get_parameter<std::string>("joint_states_topic", joint_states_topic))
//    {
//      RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'joint_states_topic', using default instead.");
//      joint_states_topic = "joint_states";
//    }
//  }

//  boost::asio::io_service io_service;
//  shared_ptr<TCPSocket> socket = shared_ptr<TCPSocket>(new TCPSocket(io_service));
//  shared_ptr<UDPSocket> io_socket = shared_ptr<UDPSocket>(new UDPSocket(io_service, 0, local_ip));

//  auto servo = std::make_shared<ACSI>(socket, io_socket);

//  RCLCPP_INFO(node->get_logger(), "Socket created");

//  try
//  {
//    servo->open(host);
//    RCLCPP_INFO(node->get_logger(), "Host is open");
//    InputAssembly test = servo->getDriveData();
//    servo->updateDriveStatus(test);
//  }
//  catch (std::runtime_error& ex)
//  {
//    RCLCPP_FATAL(node->get_logger(), "Exception caught opening session: %s", ex.what());
//    return -1;
//  }
  
//  InputAssembly test2 = servo->getDriveData();
//  servo->updateDriveStatus(test2);

//  try
//  {
//    // get status
//  }
//  catch (std::invalid_argument& ex)
//  {
//    RCLCPP_FATAL(node->get_logger(), "Invalid arguments in sensor configuration: %s", ex.what());
//    return -1;
//  }

//  try
//  {
//    // TODO: Setup implicit messaging here
//    // stepper.startUDPIO();
//    // ROS_INFO_STREAM("UDP Started");
//  }
//  catch (std::logic_error& ex)
//  {
//    RCLCPP_FATAL(node->get_logger(), "Could not start UDP IO: %s", ex.what());
//    return -1;
//  }

//  float default_accel;
//  node->declare_parameter("default_accel");
//  if(!node->get_parameter<float>("default_accel", default_accel))
//  {
//    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'default_accel', using default instead.");
//    default_accel = 100.0;
//  }
//  servo->so.accel = default_accel;

//  float default_decel;
//  node->declare_parameter("default_decel");
//  if(!node->get_parameter<float>("default_decel", default_decel))
//  {
//    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'default_decel', using default instead.");
//    default_decel = 100.0;
//  }
//  servo->so.decel = default_decel;

//  float default_force;
//  node->declare_parameter("default_force");
//  if(!node->get_parameter<float>("default_force", default_force))
//  {
//    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'default_force', using default instead.");
//    default_force = 30.0;
//  }
//  servo->so.force = default_force;

//  float default_velocity;
//  node->declare_parameter("default_velocity");
//  if(!node->get_parameter<float>("default_velocity", default_velocity))
//  {
//    RCLCPP_WARN(node->get_logger(), "Failed to lookup parameter 'default_velocity', using default instead.");
//    default_velocity = 10.0;
//  }
//  servo->so.velocity = default_velocity;

//  // publisher for stepper status
//  auto servo_pub = node->create_publisher<tolomatic_msgs::msg::AcsiInputs>("inputs", 1);
//  auto status_pub = node->create_publisher<tolomatic_msgs::msg::AcsiStatus>("status", 1);
  
//  // publisher and message for joint state
//  sensor_msgs::msg::JointState joint_state;
//  joint_state.name.resize(1);
//  joint_state.position.resize(1);
//  joint_state.name[0] = joint_name;
//  joint_state.position[0] = 0.0;

//  auto joint_state_pub = node->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic, 1);

//  // services
//  auto enable_service = node->create_service<std_srvs::srv::SetBool>("enable", std::bind(&ACSI::enable, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto estop_service = node->create_service<std_srvs::srv::SetBool>("estop", std::bind(&ACSI::estop, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveSelect_service = node->create_service<tolomatic_msgs::srv::AcsiMoveSelect>("moveSelect", std::bind(&ACSI::moveSelect, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveHome_service = node->create_service<std_srvs::srv::Trigger>("moveHome", std::bind(&ACSI::moveHome, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveStop_service = node->create_service<std_srvs::srv::Trigger>("moveStop", std::bind(&ACSI::moveStop, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveVelocity_service = node->create_service<tolomatic_msgs::srv::AcsiMoveVelocity>("moveVelocity", std::bind(&ACSI::moveVelocity, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveAbsolute_service = node->create_service<tolomatic_msgs::srv::AcsiMoveAbsolute>("moveAbsolute", std::bind(&ACSI::moveAbsolute, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveIncremental_service = node->create_service<tolomatic_msgs::srv::AcsiMoveIncremental>("moveIncremental", std::bind(&ACSI::moveIncremental, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto moveRotary_service = node->create_service<tolomatic_msgs::srv::AcsiMoveRotary>("moveRotary", std::bind(&ACSI::moveRotary, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto setHome_service = node->create_service<std_srvs::srv::Trigger>("setHome", std::bind(&ACSI::setHome, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
//  auto setProfile_service = node->create_service<tolomatic_msgs::srv::AcsiSetProfile>("setProfile", std::bind(&ACSI::setProfile, servo, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

//  // not implemented
//  // ros::ServiceServer estop_service = nh.advertiseService("estop",
//  // &STEPPER::estop, &stepper);

//  while (rclcpp::ok())
//  {
//    try
//    {
//      // Collect status from controller, convert to ROS message format.
//      InputAssembly test = servo->getDriveData();
//      servo->updateDriveStatus(test);

//      // Update joint state with new info from servo
//      joint_state.header.stamp = node->now();
//      // TODO: Convert from user-set units on servo driver to meters.
//      // Currently assume that the servo driver reports positions in millimeters.
//      // See issue #2.
//      joint_state.position[0] = double(servo->ss.current_position) / 1000.0;

//      // publish stepper inputs
//      servo_pub->publish(servo->si);

//      // publish stepper status
//      status_pub->publish(servo->ss);

//      // set outputs to stepper drive controller
//      servo->setDriveData();
//    }
//    catch (std::runtime_error& ex)
//    {
//      RCLCPP_ERROR(node->get_logger(), "Exception caught requesting scan data: %s", ex.what());
//    }
//    catch (std::logic_error& ex)
//    {
//      RCLCPP_ERROR(node->get_logger(), "Problem parsing return data: ", ex.what());
//    }

//    // Always publish last-available joint state data even if we couldn't communicate with the servo.
//    if (publish_joint_state)
//    {
//      joint_state_pub->publish(joint_state);
//    }

//    rclcpp::spin_some(node);
//    throttle.sleep();
//  }

//  servo->closeConnection(0);
//  servo->close();
//  return 0;
}
