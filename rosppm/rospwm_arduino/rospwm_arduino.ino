
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <rosppm/ppm_io.h>

unsigned long now;                        // timing variables to update data at a regular interval                  
unsigned long rc_update;
const int channels = 6;                   // specify the number of receiver channels
float RC_in[channels];                    // an array to store the calibrated input from receiver 
float read_val[channels];
int count=0;//loopcount

ros::NodeHandle nh;

rosppm::ppm_io msg_read_ppm;
std_msgs::Float32 stat_msg;

ros::Publisher pub_read_ppm("/rosppm_interface/read_ppm", &msg_read_ppm);


void setup() {
  // ROS Configurations
  /// Initialize topics
  nh.initNode();
  nh.advertise(pub_read_ppm);
  
  while(!nh.connected()){    nh.spinOnce();  }
  // Hardware settings
  /// Set Pinmodes
  setup_pwmRead();
  pinMode(13, OUTPUT); //status LED                     
  //Serial.begin(9600);
}

void loop() {  
    
    now = millis();
    msg_read_ppm.header.frame_id = "rc_link";
    if(RC_avail() || now - rc_update > 25){   // if RC data is available or 25ms has passed since last update (adjust to be equal or greater than the frame rate of receiver)
      
      rc_update = now;                           
      
      print_RCpwm();                        // uncommment to print raw data from receiver to serial
      
      //Encode Read ppm values
      msg_read_ppm.a = read_val[0];
      msg_read_ppm.b = read_val[1];
      msg_read_ppm.c = read_val[2];
      msg_read_ppm.d = read_val[3];
      msg_read_ppm.e = read_val[4];
      msg_read_ppm.f = read_val[5];
      
      //Publish read ppm values
      pub_read_ppm.publish(&msg_read_ppm);
    }
    timer_loopcount(); //Counter for handshake
    nh.spinOnce();
    
    delay(10);
    
}

void timer_loopcount(){
    count++;
  if(count>100){
    count =0;
  }
}
