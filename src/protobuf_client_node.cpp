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
  // Test params
  declare_parameter("gateway_port", 5001);
  declare_parameter("gateway_ip", "127.0.0.1");
  declare_parameter("send_to_gateway_topic", "/send_to_gateway");
  // pub
  publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
  pub_gateway_msg_ = create_publisher<protobuf_client_interfaces::msg::Gateway>("/gateway_msg", 100);
  
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
  /// Breaking the system!!! how to use sub callback properly? 
  sub_to_gateway_ = create_subscription<protobuf_client_interfaces::msg::Gateway>(
    send_to_gateway_topic_,
    10, std::bind(&ProtobufClientNode::to_gateway_cb, this));
}

void ProtobufClientNode::on_timer()
{
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publisher: '%s'", message.data.c_str());
  publisher_->publish(message);
}

void ProtobufClientNode::connect_to_gateway()
{ 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connecting to iMOOSGateway...");
  bool print_loop_error = false;
  client_->connect(gateway_ip_, gateway_port_);
  while (!client_->connected())
  {
    client_->connect(gateway_ip_, gateway_port_);
    usleep(10000);
    try
    {
      io_.poll();
    }
    catch(boost::system::error_code & ec)
    {
      if (!print_loop_error)
      {
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Looping until connected to gateway. Error ");
	std::cout << ec << std::endl;
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Connected to gateway");

  // Send kickoff message to gateway
  moos::gateway::ToGateway msg;
  //ros::Time time = ros::Time::now();
  rclcpp::Time time = ros_clock_.now();
  //msg.set_client_time(time.toSec());
  msg.set_client_time(time.seconds());
  msg.set_client_key("NAV_TEST");
  msg.set_client_double(0.0);
  msg.set_client_string(" ");

  if (client_->connected())
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending kickoff msg: %s", msg.ShortDebugString().c_str());
    client_->write(msg);
  }
}

void ProtobufClientNode::ingest_gateway_msg()
{
  client_->read_callback<moos::gateway::FromGateway>(
    [this](const moos::gateway::FromGateway& msg,
	   const boost::asio::ip::tcp::endpoint& ep)
      {
	// Create Gateway msg
	protobuf_client_interfaces::msg::Gateway gateway_msg;
	rclcpp::Time time = ros_clock_.now();
	gateway_msg.gateway_time = time;  // Update for actual moos time msg.gateway_time
	gateway_msg.header.stamp = time;
	gateway_msg.gateway_key = msg.gateway_key();
	gateway_msg.gateway_string = msg.gateway_string();
	gateway_msg.gateway_double = msg.gateway_double();
	// Publish Gateway msg
	pub_gateway_msg_->publish(gateway_msg);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
		    "Received Gateway Key: %s", gateway_msg.gateway_key.c_str());
      });
}

void ProtobufClientNode::init_ros_io()
{
  // Move to constructor?
  
  // init params
  gateway_port_ = get_parameter("gateway_port").as_int();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[%s] gateway_port: %ld", __APP_NAME__, gateway_port_);
  gateway_ip_ = get_parameter("gateway_ip").as_string();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "[%s] gateway IP: %s", __APP_NAME__, gateway_ip_.c_str());
  send_to_gateway_topic_ = get_parameter("send_to_gateway_topic").as_string();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
	      "[%s] subscription topic: %s", __APP_NAME__, send_to_gateway_topic_.c_str());
  
  // Connecting to iMOOSGateway
  connect_to_gateway();

  // Define handles for interface messages
  ingest_gateway_msg();
  
  // init subscriptions
  //sub_to_gateway_ = create_subscription<protbuf_client_interfaces::msg::Gateway>(       
  //  send_to_gateway_topic_,                                                                     
  //  10, to_gateway_cb);
  //nh_.subscribe(send_to_gateway_, 100, &ProtobufClient::ToGatewayCallback, this);

  // init publishers
  //pub_gateway_msg_ = nh_.advertise<protobuf_client::Gateway>("/gateway_msg", 1000);
  //  pub_gateway_msg_ = create_publisher<protobuf_client_interfaces::msg::Gateway>(
  //  "/gateway_msg", 100);
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
