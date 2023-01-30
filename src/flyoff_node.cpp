/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
//本版本是绿色杆子采集
// 引入类 这些东西可以在rostopic 里面找到
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/image_encodings.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string> 
#include <iostream>
#include<cmath>

 int num_pic = 0;
cv::Mat imgCallback;
void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg){
    // 展示图片读取格式为bgr8 可以不显示，已经传入全局变量
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    imgCallback = cv_ptr->image;
    // cv::imshow("view" , imgCallback);
    //std::string path0="/home/uusei/img/1.jpg";
    //cv::imwrite(path0, imgCallback);
    // cv::waitKey(10);
}

void saveimg(){
    num_pic++;
    std::string string_pic = std::to_string(num_pic);
    std::string path0="/home/uusei/img/green"+string_pic+".jpg";
    cv::imwrite(path0, imgCallback);
    cv::waitKey(50);
}
// 飞行模式确认
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
//当前imu位姿态 精确度4位 调用方法举例:imp.px
struct imup{
    double px,py,pz,roll, pitch, yaw;
};
imup imp;

//结构体 绿色杆子位置
struct bar_g{
    double px=2;
    double py=1;
    double yaw=0;
};
bar_g bg;

//结构体 红色杆子位置
struct bar_r{
    double px=2;
    double py=1;
    double yaw=0;
};
bar_r br;

// 转换坐标姿态
void transpoi(geometry_msgs::PoseStamped current_posi){
    // 实例化角度
    double ox=current_posi.pose.orientation.x;
    double oy =current_posi.pose.orientation.y;
    double oz=current_posi.pose.orientation.z;
    double ow=current_posi.pose.orientation.w;
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (ow * ox + oy * oz);
    double cosr_cosp = 1 - 2 * (ox * ox + oy * oy);
    imp.roll = std::atan2(sinr_cosp, cosr_cosp);
 
    // pitch (y-axis rotation)
    double sinp = 2 * (ow * oy - oz * ox);
    if (std::abs(sinp) >= 1)
        imp.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        imp.pitch = std::asin(sinp);
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (ow * oz + ox * oy);
    double cosy_cosp = 1 - 2 * (oy * oy + oz * oz);
    imp.yaw = std::atan2(siny_cosp, cosy_cosp);

    // 6轴小数保留
    imp.px =  ceil(current_posi.pose.position.x * 10000)/10000;
    imp.py =  ceil(current_posi.pose.position.y * 10000)/10000;
    imp.pz =  ceil(current_posi.pose.position.z * 10000)/10000;
    // imupos.roll =  ceil(imupos.roll * 1000)/1000;
    // imupos.pitch =  ceil(imupos.pitch* 1000)/1000;
    // imupos.yaw =  ceil(imupos.yaw * 1000)/1000;
    //ROS_INFO("yaw:%f",imupos.yaw);
}
// 读取坐标位置
geometry_msgs::PoseStamped current_posi;
void posi_read(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_posi = *msg;
    /*
    坐标位置
    current_posi.pose.position.x
    current_posi.pose.position.y
    current_posi.pose.position.z
    四元数
    current_posi.pose.orientation.x
    current_posi.pose.orientation.y
    current_posi.pose.orientation.z
    current_posi.pose.orientation.w
    */
    transpoi(current_posi);
}
//定义标志位
int flag1,flag2,flag3,flag4,flag5;

int main(int argc, char **argv)
{
    flag1=1;
    flag2=0;
    flag3=0;
    flag4=0;
    flag5=0;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // 订阅图形话题
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::CompressedImage>
        ("/realsense_plugin/camera/color/image_raw/compressed", 1, imageCallback);
    // 订阅当前的位置信息
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, posi_read);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    // 新定义的发布
     ros::Publisher raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::PositionTarget raw_data;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;

    //必须要有设定点才能飞
    for(int i = 2; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    //模式设置
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    // 设置解索
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // 第一阶段解锁
    while(ros::ok()){
        //判断一下现在模式是否正确解锁
        if( current_state.mode != "OFFBOARD" &&
              (ros::Time::now() - last_request > ros::Duration(3.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(3.0))){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                     ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        if ( ros::Time::now()-last_request>ros::Duration(10))
            break;
        ros::spinOnce();
        rate.sleep();
    }
    // 第二阶段飞行路径
    while(ros::ok()){
        // x0m y0m z1.5m 按照坐标系来
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag1==1)
        {
            ROS_INFO("pos_init");           
            // raw_data.coordinate_frame = 8;  //flu坐标系
            raw_data.type_mask =  /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= 0;
            raw_data.position.y= 0;
            raw_data.position.z= 1.5;
            raw_data.yaw =0; 
            last_request = ros::Time::now();
            if((fabs(imp.px-raw_data.position.x)<0.03) && (fabs(imp.py-raw_data.position.y)<0.03)){
                ROS_INFO("init successed"); 
                flag1 = 0;
                flag2 = 1;
            }
        }
        // 飞到第2个点
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag2==1)
        {
            ROS_INFO("pos1");
            // raw_data.coordinate_frame = 8;  //flu坐标系
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= 0;
            raw_data.position.y= 1;
            raw_data.position.z= 1.5;
            raw_data.yaw =0; 
            last_request = ros::Time::now(); 
            //ROS_INFO("yaw:%f",imp.yaw);
            //ROS_INFO("x:%f",imp.px);
            // 闭环校正位置
            if(fabs(imp.px-raw_data.position.x)<0.03 && fabs(imp.py-raw_data.position.y)<0.03 && fabs(imp.yaw-raw_data.yaw)<0.02){
                saveimg();
                ROS_INFO("pos1 successed"); 
                flag2 = 0;
                flag3 = 1; 
            }
        }
        //飞到第3点
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag3==1)
        {
            ROS_INFO("pos2");
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= 1.4;
            raw_data.position.y= 1;
            raw_data.position.z= 1.5;
            raw_data.yaw =0; 
            last_request = ros::Time::now();
            //break;
            if(fabs(imp.px-raw_data.position.x)<0.03 && fabs(imp.py-raw_data.position.y)<0.03 && fabs(imp.yaw-raw_data.yaw)<0.02){
                saveimg();
                ROS_INFO("pos2 successed"); 
                flag3 = 0;
                flag4=  1; 
            }
        }
        // 旋转
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag4==1)
        {
            ROS_INFO("pos r");
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= bg.px-0.6*std::cos(bg.yaw);
            raw_data.position.y= bg.py-0.6*std::sin(bg.yaw);
            raw_data.position.z= 1.5;
            if (bg.yaw<=M_PI){raw_data.yaw = bg.yaw; }
            else{raw_data.yaw = bg.yaw-2*M_PI;}

            last_request = ros::Time::now();
            ROS_INFO("sinyaw:%f",bg.yaw);
            //break;
            if(fabs(imp.px-raw_data.position.x)<0.03 && fabs(imp.py-raw_data.position.y)<0.03 && fabs(imp.yaw-raw_data.yaw)<0.02){
                saveimg();
                ROS_INFO("pos r successed"); 
                if(bg.yaw>(M_PI*2)){
                    flag4=  0; 
                    flag1=  1;
                    bg.yaw=0;
                }else{
                    bg.yaw=bg.yaw+M_PI/4;
                }
            }
        }
        raw_local_pub.publish(raw_data);
        ros::spinOnce();
        rate.sleep();
    }
    // 降落步骤
    while(ros::ok()){
        if ( ros::Time::now()-last_request>ros::Duration(5) && flag3==1)
        {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("LAND");
                flag3 = 0;
            }
            last_request = ros::Time::now();
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}