#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include <math.h>
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "rosppm/ppm_io.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#define loopRate 100

float joy_set_axes[9];
float read_axes[10];

int num_Channels;

float map(float v,float in_min,float in_max,float out_min,float out_max){
    //Check that the value is at least in_min
    if(v < in_min)
        v = in_min;
    // Check that the value is at most in_max
    if(v > in_max)
        v = in_max;
    //return constrain((v - in_min) * (out_max - out_min) / (in_max - in_min) + out_min,out_min,out_max);
    return std::min(out_max, std::max(out_min, ((v - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)));
}

void joy_set_channels_Callback(const sensor_msgs::Joy msg){
	
	for(int i=0; i<num_Channels; i++){
	 joy_set_axes[i] = msg.axes[i];
	}
}

void read_ppm_Callback(const rosppm::ppm_io msg){
	read_axes[0] = map(msg.a,1000,1900, 0,1);
	read_axes[1] = map(msg.b,1000,2000,-1,1);
	read_axes[2] = map(msg.c,1000,2000,-1,1);
	read_axes[3] = map(msg.d,1100,1900,-1,1);

	if(msg.e > 1600) read_axes[4] = 1;
	else read_axes[4] = 0;
	if(msg.f > 1600) read_axes[5] = 1;
	else read_axes[5] = 0;

}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "rosppm_node");
   ros::NodeHandle n;
   
   //rosppm interface using Joy message
   ros::Publisher pub_read_channels = n.advertise<sensor_msgs::Joy>("/rosppm_interface/joy_read_channels",loopRate);
   ros::Subscriber sub_set_channels = n.subscribe<sensor_msgs::Joy>("/rosppm_interface/joy_set_channels", loopRate, joy_set_channels_Callback);
   //PPM command message
   ros::Publisher pub_set_ppm = n.advertise<rosppm::ppm_io>("/rosppm_interface/set_ppm", loopRate);
   ros::Subscriber sub_read_ppm = n.subscribe<rosppm::ppm_io>("/rosppm_interface/read_ppm", loopRate, read_ppm_Callback);
   
   ros::Rate loop_rate(loopRate);
   ros::spinOnce();

   sensor_msgs::Joy joy_read_channels;
   sensor_msgs::Joy joy_set_channels;
   rosppm::ppm_io msg_ppm_set;
   rosppm::ppm_io msg_ppm_read;

   joy_read_channels.axes.resize(4);
   joy_read_channels.buttons.resize(6);

   int count = 1;

   n.param("rosppm/number_of_channels", num_Channels, 9);
   //n.param("rosppm/mode", mode, 0);
		
   while(ros::ok()){

		//Encode read channel message
		
		msg_ppm_set.a = joy_set_axes[0];
		msg_ppm_set.b = joy_set_axes[1];
		msg_ppm_set.c = joy_set_axes[2];
		msg_ppm_set.d = joy_set_axes[3];
		msg_ppm_set.e = joy_set_axes[4];
		msg_ppm_set.f = joy_set_axes[5];
		msg_ppm_set.g = joy_set_axes[6];
		msg_ppm_set.h = joy_set_axes[7];
		msg_ppm_set.i = joy_set_axes[8];

		joy_read_channels.header.frame_id = "rc_link";
		//joy_read_channels.header.stamp = Time.now();

		joy_read_channels.axes[0] = read_axes[0];
		joy_read_channels.axes[1] = read_axes[1];
		joy_read_channels.axes[2] = read_axes[2];
		joy_read_channels.axes[3] = read_axes[3];

		joy_read_channels.buttons[4] = read_axes[4];
		joy_read_channels.buttons[5] = read_axes[5];

		//Publish messages      
		pub_read_channels.publish(joy_read_channels); //Subscirbe set channel values
		pub_set_ppm.publish(msg_ppm_set);
		ros::spinOnce();
		count++;
		loop_rate.sleep();
   }  
   return 0;
}
