/*****************************************************************/                                     
/*    NAME: Michael DeFilippo and Dr. Supun Randeni              */     
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */                                  
/*    FILE: protobuf_client_node.hpp                             */     
/*    DATE: 2023-04-28                                           */                                    
/*    NOTE: ROS 2 port of the protobuf_client package for ROS    */
/*          Node to connect to MOOS gateway and share data       */                                     
/*          via a generic protobuf message                       */                             
/*                                                               */                                     
/* This is unreleased BETA code. no permission is granted or     */                                     
/* implied to use, copy, modify, and distribute this software    */                                     
/* except by the author(s), or those designated by the author.   */                                     
/*****************************************************************/

#ifndef PROTOBUF_CLIENT__PROTOBUF_CLIENT_NODE_HPP_
#define PROTOBUF_CLIENT__PROTOBUF_CLIENT_NODE_HPP_

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

// Gateway Msg
#include "protobuf_client_interfaces/msg/gateway.hpp"
// Protobuf include                                                                                   
#include "gateway.pb.h"                                                                                 
// gateway include                                                                                      
#include "tcp_client.h"

/// \brief client node to pass data to iMOOSGateway
class ProtobufClientNode : public rclcpp::Node
{
public:
  ProtobufClientNode(rclcpp::NodeOptions options);

private:
  void on_timer();
  size_t count_;
  // pubs and subs
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;   // Test pubs
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;  // Test subs
  rclcpp::TimerBase::SharedPtr timer_;
  int gateway_port_;
  std::string gateway_ip_;
  std::string send_to_gateway_;
  boost::asio::io_service io_;
  std::shared_ptr<gateway::tcp_client> client_{gateway::tcp_client::create(io_)};

  void ToGatewayCallback(const protobuf_client_interfaces::msg::Gateway &msg);
  /** 
   * @brief callback to subscribe to client ros gateway msgs and post to MOOSDB
   * @param[in] msg
   **/

    void ConnectToGateway();
  /**
   * @brief Connect to the MOOSDB through TCP 
   **/

  void IngestGatewayMsg();
  /**
   * @brief Ingests msg from gateway and publishes as a custom Gateway ROS msg
   **/

  
  
};

#endif  // PROTOBUF_CLIENT__PROTOBUF_CLIENT_NODE_HPP_
