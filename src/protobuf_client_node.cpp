/*****************************************************************/                                     
/*    NAME: Michael DeFilippo and Dr. Supun Randeni              */                                     
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */                                     
/*    FILE: protobuf_client_node.cpp                             */                                     
/*    DATE: 2023-04-28                                           */                                     
/*    NOTE: ROS 2 Node to connect to MOOS gateway and share data */                                     
/*          via a generic protobuf message                       */                                     
/*                                                               */                                     
/* This is unreleased BETA code. no permission is granted or     */                                     
/* implied to use, copy, modify, and distribute this software    */                                     
/* except by the author(s), or those designated by the author.   */                                     
/*****************************************************************/

#include <chrono>

#include "protobuf_client/protobuf_client_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

ProtobufClientNode::ProtobufClientNode(rclcpp::NodeOptions options)
  : Node("protobuf_client_node", options), count_(0)
{
  // pub
  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = create_wall_timer(
    500ms, std::bind(&ProtobufClientNode::on_timer, this));

  // sub
  // Create a callback function for when messages are received.                                   
  auto callback =                                                                                       
    [this](std_msgs::msg::String::UniquePtr msg)                                                        
      {                                                                                                 
      RCLCPP_INFO(this->get_logger(), "Subscriber: '%s'", msg->data.c_str());                           
      std::flush(std::cout);                                                                            
    };                                                                                                  
                                                                                                        
  // Create a subscription to the "topic" topic which can be matched with one or more                   
  // compatible ROS publishers.                                                                         
  // Note that not all publishers on the same topic with the same type will be compatible:              
  // they must have compatible Quality of Service policies.                                             
  subscriber_ = create_subscription<std_msgs::msg::String>(                                           
    "topic",                                                                                            
    10, callback);

  // Gateway msg ( Don't have to include? )
  // auto msg = protobuf_client::msg::Gateway();
}

void ProtobufClientNode::on_timer()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publisher: '%s'", message.data.c_str());
  publisher_->publish(message);
}


RCLCPP_COMPONENTS_REGISTER_NODE(ProtobufClientNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ProtobufClientNode>(options);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
