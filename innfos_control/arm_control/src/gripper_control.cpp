#include "../include/WzSerialPort.h"
#include "std_msgs/Int32.h"
#include "ros/ros.h"

class Gripper{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    std::string port_name;
    int baudrate;
public:
    WzSerialPort w;
    Gripper(){
        nh_.param<std::string>("gripper_port", port_name, "/dev/ttyUSB0");
        nh_.param<int>("gripper_baudrate", baudrate, 1000000);
        if(!w.open( port_name.c_str(), baudrate, 'N', 8, 1)){
          ROS_ERROR("[gripper_control] open serial port failed, port: %s, baudrate: %d", port_name.c_str(), baudrate);
        }
        sub = nh_.subscribe("/gripper_control/int32",10,&Gripper::callback,this);
    }
    ~Gripper(){
      w.close();
    }
    void callback(const std_msgs::Int32::ConstPtr& msg){
      int len = 11;
      unsigned char data[len];
      data[0] = 0xFE;
      data[1] = 0xEF;
      data[2] = 0x0E;
      data[3] = 0x07;
      data[4] = 0x03;
      data[5] = 0xA6;
      data[6] = 0xE7;
      data[7] = 0x03;
      data[8] = 0x00;
      data[9] = 0x00;
      data[10] = 0x57;
      if(msg->data==1){
        data[6] = 0x01;
        data[7] = 0x00;
        data[10] = 0x40;
      }
      if(!w.send(data, len)){
          ROS_ERROR("[gripper_control] serial send failed, data: %d", msg->data);
      }
    }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_control");
  Gripper node;

  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::shutdown();
  return 0;
}