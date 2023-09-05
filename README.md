# protobuf_client_ros2
ROS 2 client for the MIT AUV Lab's MOOS-ROS bridge

### protobuf_client_ros2 is intended to be used with the iMOOSGateway MOOS application. Please see the moos-ivp-gateway package (https://github.com/mikedef/moos-ivp-gateway) for more information.

`protobuf_client_ros2` is a ROS node that serves as a bridge to MOOS-IvP through a predefined Google Protobuf message type. It runs a TCP client using `lib_gateway_tcp` (adopted from https://github.com/GobySoft/ntsim/tree/master/src/lib/tcp), which allows this application to connect to a MOOS based TCP server, using the same library.                                                                             
                                                                                                        
When `protobuf_client_ros2` receives key-value pairs defined in the MOOS-IvP Gateway application. This data is posted as the rostopic `/gateway_msg` as Gateway.msg ROS type. The gateway_msg topic can then be subscribed to by an additional data parsing node created by the user.                 
                                                                                                        
The `protobuf_client_ros2` is set to automatically forward any messages posted to the topic specified by the `send_to_gateway_topic` parameter. This topics data type is of the custom Gateway.msg ROS type.   
                                                                                                        
* Please see Gateway.msg for more information into the `/gateway_msg` and `/send_to_gateway` topics custom data type.

## NOTE on the custom Gateway.msg
While it is provided in the protobuf_client_ros2/msg directory this message should be built into it's own custom package `protobuf_client_interfaces` to be used properly by `protobuf_client_ros2`

Please be aware of this failure during build time. The package will fail to build due to "protobuf_client_interfaces" missing. Please download the `protobuf_client_interfaces` ROS 2 package (https://github.com/mikedef/protobuf_client_interfaces)


## Dependencies
* Google protocol buffers
  * `sudo apt install protobuf-compiler`
* Base64 encode/decode
  * `sudo apt install libb64-dev`

                                                                                                        
## ROS API                                                                                              
                                                                                                        
### Subs                                                                                                
* `send_to_gateway_topic` (msg/Gateway)                                                                 
                                                                                                        
### Pubs                                                                                                
* `/gateway_msg` (msg/Gateway)                                                                          
Received data from MOOS Gateway is published to this topic                                              
                                                                                                        
### ROS Parameters                                                                                      
* `gateway_port`: default="9502"                                                                        
  * Port number of MOOS Gateway                                                                         
* `gateway_ip`:   default="127.0.0.1"                                                                   
  * IP address of MOOS Gateway                                                                          
* `send_to_gateway_topic`: default="/send_to_gateway_cora"                                              
  * Subscribe to this topic to send data to MOOS                                                        
                                                              
