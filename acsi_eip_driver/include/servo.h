/**
 * @file servo.h
 * @brief Tolomatic ACSI servo interface using Ethernet/IP - servo functions
 *class definition.
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

#ifndef ACSI_EIP_DRIVER_H
#define ACSI_EIP_DRIVER_H

#include <boost/shared_ptr.hpp>



#include "odva_ethernetip/session.h"
#include "odva_ethernetip/socket/socket.h"

#include "input_assembly.h"
#include "output_assembly.h"


#include <tolomatic_msgs/msg/acsi_inputs.hpp>
#include <tolomatic_msgs/msg/acsi_outputs.hpp>
#include <tolomatic_msgs/msg/acsi_status.hpp>



using boost::shared_ptr;
using eip::Session;
using eip::socket::Socket;

namespace acsi_eip_driver
{
const double IN_POSITION_TOLERANCE = 0.10;

/**
 * Main interface for the Tolomatic stepper controller.
 * Produces methods to access the stepper controller from a high level.
 */
class ACSI : public Session
{
public:
  /**
   * Construct a new instance.
   * @param socket Socket instance to use for communication with the stepper
   * controller
   */
  ACSI(shared_ptr<Socket> socket, shared_ptr<Socket> io_socket)
    : Session(socket, io_socket), connection_num_(-1), mrc_sequence_num_(1)
  {
  }

  // drive interface functions
  InputAssembly getDriveData();
  void setDriveData();
  void servoControlCallback(const tolomatic_msgs::msg::AcsiOutputs::ConstPtr& oa);
  void updateDriveStatus(InputAssembly ia);

  // ROS service callback handler functions
//  void enable(const std::shared_ptr<rmw_request_id_t> request_header,
//              const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
//              std::shared_ptr<std_srvs::srv::SetBool::Response> res);

//  void estop(const std::shared_ptr<rmw_request_id_t> request_header,
//             const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
//             std::shared_ptr<std_srvs::srv::SetBool::Response> res);

//  void setHome(const std::shared_ptr<rmw_request_id_t> request_header,
//               const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
//               std::shared_ptr<std_srvs::srv::Trigger::Response> res);

//  void moveHome(const std::shared_ptr<rmw_request_id_t> request_header,
//                const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
//                std::shared_ptr<std_srvs::srv::Trigger::Response> res);

//  void moveStop(const std::shared_ptr<rmw_request_id_t> request_header,
//                const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
//                std::shared_ptr<std_srvs::srv::Trigger::Response> res);

//  void setProfile(const std::shared_ptr<rmw_request_id_t> request_header,
//                  const std::shared_ptr<tolomatic_msgs::srv::AcsiSetProfile::Request> req,
//                  std::shared_ptr<tolomatic_msgs::srv::AcsiSetProfile::Response> res);

//  void moveAbsolute(const std::shared_ptr<rmw_request_id_t> request_header,
//                    const std::shared_ptr<tolomatic_msgs::srv::AcsiMoveAbsolute::Request> req,
//                    std::shared_ptr<tolomatic_msgs::srv::AcsiMoveAbsolute::Response> res);

//  void moveIncremental(const std::shared_ptr<rmw_request_id_t> request_header,
//                       const std::shared_ptr<tolomatic_msgs::srv::AcsiMoveIncremental::Request> req,
//                       std::shared_ptr<tolomatic_msgs::srv::AcsiMoveIncremental::Response> res);

//  void moveRotary(const std::shared_ptr<rmw_request_id_t> request_header,
//                  const std::shared_ptr<tolomatic_msgs::srv::AcsiMoveRotary::Request> req,
//                  std::shared_ptr<tolomatic_msgs::srv::AcsiMoveRotary::Response> res);

//  void moveSelect(const std::shared_ptr<rmw_request_id_t> request_header,
//                  const std::shared_ptr<tolomatic_msgs::srv::AcsiMoveSelect::Request> req,
//                  std::shared_ptr<tolomatic_msgs::srv::AcsiMoveSelect::Response> res);

//  void moveVelocity(const std::shared_ptr<rmw_request_id_t> request_header,
//                    const std::shared_ptr<tolomatic_msgs::srv::AcsiMoveVelocity::Request> req,
//                    std::shared_ptr<tolomatic_msgs::srv::AcsiMoveVelocity::Response> res);

  void setEnable(const bool& enable);

  void setHome();

  void moveHome();

  void moveStop();

  void moveAbsolute(const float& position);


  // TODO: Debug this
  // void startUDPIO(); //for implicit data; not working

  tolomatic_msgs::msg::AcsiInputs si;
  tolomatic_msgs::msg::AcsiOutputs so;
  tolomatic_msgs::msg::AcsiStatus ss;
  tolomatic_msgs::msg::AcsiStatus ss_last;

private:
  // data for sending to stepper controller
  int connection_num_;
  EIP_UDINT mrc_sequence_num_;
};

}  // namespace acsi_eip_driver

#endif  // ACSI_EIP_DRIVER_H
