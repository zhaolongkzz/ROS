
#include<stdio.h>
#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<math.h>
#define pi 3.1415926
//global variables for callback functions to populate for use in main program 
std_msgs::Float64 g_vel_cmd;

// void myCallbackDti(const std_msgs::Float64& message_holder){
//     ROS_INFO("received dti command value is: %f", message_holder.data);
//     g_dti.data = message_holder.data;
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "minimal_commander");
    ros::NodeHandle nh;

    // ros::Subscriber my_subscriber_object = nh.subscribe("dti", 1, myCallbackDti);
    ros::Publisher my_publisher_object = nh.advertise<std_msgs::Float64>("vel_cmd", 1);

    double amplitude = 0.0;
    double frequency = 0.0;
    double w = 0.0; 
    double dt = 0.01; //10ms integration time step
    double sample_rate = 1.0 / dt;
    ros::Rate naptime(sample_rate); // compute the corresponding update frequency 
    double dti = 0.0;
    std::cout << "please input two numbers for amplitude and frequency:" << std::endl;
    std::cin >> amplitude; //input a number for amplitude
    std::cin >> frequency; //input a number for frequency
    w = 2 * pi * frequency; //use frequency to compute w

    while(ros::ok()){
        g_vel_cmd.data = amplitude * sin(w * dti); //use the formula "v=A*sin(w*t)"
        dti += 0.001; 
        // if(g_dti.data > 1){
        //     g_dti.data = 0.0;
        // }
        my_publisher_object.publish(g_vel_cmd);
        ROS_INFO("vel_cmd command = %f", g_vel_cmd.data);
        ros::spinOnce();
        naptime.sleep();
    }
    return 0;
}