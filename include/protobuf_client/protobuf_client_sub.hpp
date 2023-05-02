/*****************************************************************/                                     
/*    NAME: Michael DeFilippo and Dr. Supun Randeni              */     
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */                                  
/*    FILE: protobuf_client_sub.hpp                              */     
/*    DATE: 2023-05-01                                           */                                    
/*    NOTE: ROS 2 sub node to connect to MOOS gateway and post   */                                     
/*            data to ROS DDS Bus                                */                                     
/*                                                               */                                     
/* This is unreleased BETA code. no permission is granted or     */                                     
/* implied to use, copy, modify, and distribute this software    */                                     
/* except by the author(s), or those designated by the author.   */                                     
/*****************************************************************/

#ifndef PROTOBUF_CLIENT__PROTOBUF_CLIENT_SUB_HPP_
#define PROTOBUF_CLIENT__PROTOBUF_CLIENT_SUB_HPP_

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/// \brief client node to pass data to iMOOSGateway
class protobuf_client : public rclcpp::Node
{
public:
  PROTOBUF_CLIENT_PUBLIC ProtobufClientNode(rclcpp::NodeOptions options);

private:
  void on_timer();
  size_t count_;
  // pubs and subs
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;   // Test pubs
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;  // Test subs
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // PROTOBUF_CLIENT__PROTOBUF_CLIENT_NODE_HPP_
