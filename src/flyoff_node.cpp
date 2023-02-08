/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
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
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <string> 
#include <iostream>
#include<cmath>
#include <vector>
#include <ctime>

//定义标志位
int flag1,flag2,flag3,flag4,flag5,flag6,flag7,flag8;
int num_pic = 0;
//当前imu位姿态 精确度4位 调用方法举例:imp.px
struct imup{
    double px,py,pz,roll, pitch, yaw;
};
imup imp;
//结构体 绿色杆子位置
struct bar_g{
    double px=2.5;
    double py=1.5;
    double yaw=0;
};
bar_g bg;
//结构体 红色杆子位置
struct bar_r{
    double px=1;
    double py=1;
    double yaw=0;
};
bar_r br;
// 存储图像处理结果和图片信息
cv::Mat imgCallback;
struct info_img{
    int width;
    int height;
    double dist;
    double x_dist;
    double y_dist;
};
info_img ig;

// 路径规划 存储k值和相关信息
struct info_point{
    double kp;
    //这里true的话 就定义为x轴平行 后面直接简单避障
    bool is_x=false;
    //这里true的话 就定义为y轴平行 后面直接简单避障
    bool is_y=false;
    double dist_rb;
    double dist_gb;
    double dist_dd;
};
info_point ifp;
// cross_point.clear(); 清除向量内容
/*
    1.path_point 无人机路径点
    2.path_resist 障碍物点
    3.cross_point 待选择的距离
    通过障碍物点确定路径点
    比较选择最小距离
*/
std::vector<double> cross_point,path_line,line_resist;
std::vector<std::vector<double>> path_point,path_resist;
// 计算坐标距离 
double calculate_dist(double origin_x,double origin_y,double target_x,double target_y){
    double dist;
    double l_x,l_y;
    l_x = target_x-origin_x;
    l_y = target_y-origin_y;
    dist = sqrt(l_x*l_x+l_y*l_y);
    std::cout<<"cd output dist:"<<dist<<std::endl;
    return dist;
}
// 点到直线距离
double point2line(double y,double x,double k,double b){
    double xc,yc,dist;
    if(ifp.is_x){
        return fabs(y-b);
    }else if(ifp.is_y){
        return fabs(x-b);
    }else{
    xc = (x+k*(y-b))/(k*k+1);
    yc = k * xc + b;
    dist = calculate_dist(x,y,xc,yc);
    std::cout<<"p2l output dist:"<<dist<<std::endl;
    return dist;
    }
}
//线性规划验证 两种
double linear_dep(double y,double x,double k,double b){
    double arg;
    arg = y - k*x -b;
    return arg;
}
double linear_depx(double x,double b){
    double arg;
    arg = x -b;
    return arg;
}
// 路径规划 判断行进信息
int smooth_path(double target_x,double target_y){
    /*
    返回值说明: 最终使用switch case 语句进行下一步操作
    0：不能走
    1：直接走
    2:   需要避障
    */
    // 定义参数
    double k,b,p2l;
    std::vector<int> direction_d,direction_br,direction_bg,direction_rd,direction_rbr,direction_rbg;
    path_resist.clear();
    // y-kx-b 在 [-0.4,0.4]的范围内
    if (fabs(target_x-imp.px)<0.03){
        ifp.is_y = true;
        ifp.is_x = false;
        k=0;
        b=target_x;
    }else if(fabs(target_y-imp.py)<0.03){
        ifp.is_y = false;
        ifp.is_x = true;
        k=0;
        b=target_y;
    }else{
        ifp.is_y = false;
        ifp.is_x = false;
        k=(target_y-imp.py)/(target_x-imp.px);
        b=target_y-k*target_x;
        ifp.kp=(-1)/k;
        std::cout<<"kp:"<<ifp.kp<<std::endl;
    }
    // 距离测算
    ifp.dist_rb=calculate_dist(br.px, br.py, imp.px, imp.py);
    ifp.dist_gb=calculate_dist(bg.px, bg.py, imp.px, imp.py);
    ifp.dist_dd=calculate_dist(target_x, target_y, imp.px, imp.py);

    int dy=(target_y-imp.py)>-0.3;
    int dx=(target_x-imp.px)>-0.3;
    direction_d.push_back(dx);
    direction_d.push_back(dy);
    int aldy=(imp.py-target_y)>-0.3;
    int aldx=(imp.px-target_x)>-0.3;
    direction_rd.push_back(aldx);
    direction_rd.push_back(aldy);
    int ry=(br.py-imp.py)>-0.3;
    int rx=(br.px-imp.px)>-0.3;
    direction_br.push_back(rx);
    direction_br.push_back(ry);
    int alry=(imp.py-br.py)>-0.3;
    int alrx=(imp.px-br.px)>-0.3;
    direction_rbr.push_back(alrx);
    direction_rbr.push_back(alry);
    int gy=(bg.py-imp.py)>-0.3;
    int gx=(bg.px-imp.px)>-0.3;
    direction_bg.push_back(gx);
    direction_bg.push_back(gy);
    int algy=(imp.py-bg.py)>-0.3;
    int algx=(imp.px-bg.px)>-0.3;
    direction_rbg.push_back(algx);
    direction_rbg.push_back(algy);
    //三种情况  避障 直接走 不可走
    if((calculate_dist(br.px, br.py, target_x, target_y)<=0.45)||(calculate_dist(bg.px, bg.py, target_x, target_y)<=0.45)){
        // 不可以走 会撞到
        std::cout<<"there is no way to the point.it cause issue"<<std::endl;
        return 0;
    }
    if((direction_rd!=direction_rbg)&&(direction_d!=direction_bg)&&((direction_d==direction_br)||(direction_rd==direction_rbr))&&(point2line( br.py, br.px, k, b)<=0.45)){
        std::cout<<"classify now"<<std::endl;
        line_resist.clear();
        line_resist.push_back(br.px);
        line_resist.push_back(br.py);
        path_resist.push_back(line_resist);
    }else if((direction_rd!=direction_rbr)&&(direction_d!=direction_br)&&((direction_d==direction_bg)||(direction_rd==direction_rbg))&&(point2line( bg.py, bg.px, k, b)<=0.45)){
        std::cout<<"classify now"<<std::endl;
        line_resist.clear();
        line_resist.push_back(bg.px);
        line_resist.push_back(bg.py);
        path_resist.push_back(line_resist);
    }else if(((direction_d==direction_br)&&(direction_d==direction_bg))||((direction_rd==direction_rbr)&&(direction_rd==direction_rbg))){
        std::cout<<"classify now"<<std::endl;
        line_resist.clear();
        if(ifp.dist_rb>=ifp.dist_gb){
            if((point2line(bg.py, bg.px, k, b)<=0.45)&&(ifp.dist_dd>=ifp.dist_gb)){
                line_resist.push_back(bg.px);
                line_resist.push_back(bg.py);
                path_resist.push_back(line_resist);
            }
            line_resist.clear();
            if((point2line( br.py, br.px, k, b)<=0.45)&&(ifp.dist_dd>=ifp.dist_rb)){
                line_resist.push_back(br.px);
                line_resist.push_back(br.py);
                path_resist.push_back(line_resist);
            }
        }else{
            if((point2line( br.py, br.px, k, b)<=0.45)&&(ifp.dist_dd>=ifp.dist_rb)){
                line_resist.push_back(br.px);
                line_resist.push_back(br.py);
                path_resist.push_back(line_resist);
            }
            line_resist.clear();
            if((point2line( bg.py, bg.px, k, b)<=0.45)&&(ifp.dist_dd>=ifp.dist_gb)){
            line_resist.push_back(bg.px);
            line_resist.push_back(bg.py);
            path_resist.push_back(line_resist);
            }
        }
    }
    if(path_resist.empty()){
        std::cout<<"go through directly"<<std::endl;
        return 1;
    }else{
        std::cout<<"need calcaluted"<<std::endl;
        return 2;
    }
}

// 路径规划 规划途径障碍物的路径点  圆心为障碍物坐标
void set_path(double x_0,double y_0){
    /*  
    输入圆心坐标 k值在结构体里面
    直线与圆的交点，反求出两个路径点 比较使用前面的距离计算函数
    */
    double bp,delta,ao,bo,co,solve_x0,solve_x1,solve_y0,solve_y1;
    // 求解二次方程的解集
    bp = y_0-ifp.kp*x_0;
    std::cout<<"bp:"<<bp<<std::endl;
    ao=1+ ifp.kp*ifp.kp;//a 大于1 不用担心被除
    bo=2*ifp.kp*(bp-y_0)-2*x_0;
    co=x_0*x_0+(bp-y_0)*(bp-y_0)-0.36;
    delta = bo*bo-4*ao*co;
    std::cout<<"a0:"<<ao<<"b0:"<<bo<<"co:"<<co<<"delta"<<delta<<std::endl;
    solve_x0 = ((-1)*bo+sqrt(delta))/(2*ao);
    solve_y0 = ifp.kp*solve_x0+bp;
    solve_x1 = ((-1)*bo-sqrt(delta))/(2*ao);
    solve_y1 = ifp.kp*solve_x1+bp;
    std::cout<<"x0:"<<solve_x0<<"y0:"<<solve_y0<<"x1:"<<solve_x1<<"y1:"<<solve_y1<<std::endl;
    cross_point.clear();
    cross_point.push_back(solve_x0);
    cross_point.push_back(solve_y0);
    cross_point.push_back(solve_x1);
    cross_point.push_back(solve_y1);
}
//路径规划 最终封包
int select_path(double x,double y){
    path_line.clear();
    path_point.clear();
    switch(smooth_path(x,y)){
        case 0:                  
            path_line.push_back(imp.px);
            path_line.push_back(imp.py);
            path_point.push_back(path_line);
            break;
        case 1:
            path_line.push_back(x);
            path_line.push_back(y);
            path_point.push_back(path_line);
            break;
        case 2:
            if(ifp.is_y){
                for(int i=0;i<path_resist.size();i++){
                    double cd_p0= calculate_dist(path_resist[i][0]+0.5,path_resist[i][1],imp.px,imp.py);
                    double cd_p1= calculate_dist(path_resist[i][0]-0.5,path_resist[i][1],imp.px,imp.py);
                    if(cd_p0<=cd_p1){
                        path_line.clear();
                        path_line.push_back(path_resist[i][0]+0.5);
                        path_line.push_back(path_resist[i][1]);
                        path_point.push_back(path_line);
                    }else{
                        path_line.clear();
                        path_line.push_back(path_resist[i][0]);
                        path_line.push_back(path_resist[i][1]-0.5);
                        path_point.push_back(path_line);
                    }            
                }
            }else if(ifp.is_x){
                for(int i=0;i<path_resist.size();i++){
                    double cd_p0= calculate_dist(path_resist[i][0],path_resist[i][1]+0.5,imp.px,imp.py);
                    double cd_p1= calculate_dist(path_resist[i][0],path_resist[i][1]-0.5,imp.px,imp.py);
                    if(cd_p0<=cd_p1){
                        path_line.clear();
                        path_line.push_back(path_resist[i][0]);
                        path_line.push_back(path_resist[i][1]+0.5);
                        path_point.push_back(path_line);
                    }else{
                        path_line.clear();
                        path_line.push_back(path_resist[i][0]);
                        path_line.push_back(path_resist[i][1]-0.5);
                        path_point.push_back(path_line);
                    }            
                }
            }else{
                for(int i=0;i<path_resist.size();i++){
                    set_path(path_resist[i][0],path_resist[i][1]);
                    double cd_p0= calculate_dist(cross_point[0],cross_point[1],imp.px,imp.py);
                    double cd_p1= calculate_dist(cross_point[2],cross_point[3],imp.px,imp.py);
                    if(cd_p0<=cd_p1){
                        path_line.clear();
                        path_line.push_back(cross_point[0]);
                        path_line.push_back(cross_point[1]);
                        path_point.push_back(path_line);
                    }else{
                        path_line.clear();
                        path_line.push_back(cross_point[2]);
                        path_line.push_back(cross_point[3]);
                        path_point.push_back(path_line);
                    }            
                }                 
            }
            path_line.clear();
            path_line.push_back(x);
            path_line.push_back(y);
            path_point.push_back(path_line);
            std::cout<<"calcaluted fin"<<std::endl; 
            break;
    }
    return 0;
}

void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg){
    // 展示图片读取格式为bgr8 可以不显示，已经传入全局变量
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    imgCallback = cv_ptr->image;
}
// 图像处理
void img_process(){
    //clock_t begin, end;
    //begin = clock();
    cv::Mat kernel;
    cv::Mat read,hsv,hsv0,hsv1,mask,res,gray,opened;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> cnts;  
    std::vector<cv::Vec4i> hier; 

    // imgCallback = imgCallback(cv::Range(40,400),cv::Range::all());
    read = imgCallback;
    ig.width = read.cols;
    ig.height = read.rows;
    // mask
    cv::cvtColor(read, hsv, CV_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 160),cv::Scalar(20, 10, 255),hsv0);
    cv::bitwise_and(read, read, res, mask=hsv0);
    cv::blur(res, res,cv::Size(5, 5));
    //  二值化
    cv::threshold(res, res, 100, 255, CV_THRESH_BINARY);
    cv::cvtColor(res, gray, CV_BGR2GRAY);
    // 去噪点 这里编译不通过 直接代替源码数值
    kernel = cv::getStructuringElement(0, cv::Size(5, 5));
    cv::morphologyEx(gray,opened, 3, kernel);
    cv::morphologyEx(opened,opened, 3, kernel);
    cv::bitwise_not(opened,opened);
    // 边缘检测
    cv::findContours(opened,cnts,hier,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    std::cout<<cnts.size()<<std::endl;
    // contours.resize(cnts.size());
    cv::imwrite("/home/uusei/img/0.jpg", opened);
    for( int k = 0; k < cnts.size(); k++ )
    {
        cv::Rect  rect = cv::boundingRect(cnts.at(k));
        std::cout<<"w"<<rect.width<<std::endl;
        std::cout<<"h"<<rect.height<<std::endl;
        if ((cv::contourArea(cnts.at(k)) >= 3000) && (cv::contourArea(cnts.at(k)) <= 102400) && (rect.width >=100)&& (rect.width <= 320)&& (rect.height >=100)&& (rect.height <= 320)){
            contours.push_back(cnts.at(k));
            std::cout<<"get contours"<<std::endl;
        }
    }
    for( int k = 0; k < contours.size(); k++ )
    {
        cv::Rect  rect = cv::boundingRect(contours.at(k));
        int r_x = rect.x + rect.width/2;
        int r_y = rect.y + rect.height/2;
        ig.x_dist= (r_y - 240)*0.24*0.01*(-1)+imp.px;
        ig.y_dist= (r_x - 320)*0.24*0.01*(-1)+imp.py;
        std::cout<<"x"<<ig.x_dist<<std::endl;
        std::cout<<"y"<<ig.y_dist<<std::endl;

        // for( int i = 0; i < rect.width; i++ ){
        //     for( int j = 0; j < 10; j++ ){
        //         if ((read.at<cv::Vec3i>(r_y+j,i+rect.x)[0] <= 35) && (read.at<cv::Vec3i>(r_y+j,i+rect.x)[1]<= 35))
        //             wide += 1;
        //     }
        // }
    } 
    //end = clock();
    //std::cout << "所用时间" << double(end - begin) / CLOCKS_PER_SEC * 1000 << "ms" << std::endl;       
}
// 保存图片
void saveimg(){
    num_pic++;
    std::string string_pic = std::to_string(num_pic);
    std::string path0="/home/uusei/img/red"+string_pic+".jpg";
    cv::imwrite(path0, imgCallback);
    cv::waitKey(50);
}
// 飞行模式确认
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// 转换坐标姿态
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
    transpoi(current_posi);
}
int main(int argc, char **argv)
{
    flag1=1;
    flag2=0;
    flag3=0;
    flag4=0;
    flag5=0;
    flag6=0;
    flag7=0;
    flag8=0;
    int ppi = 0;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // 订阅图形话题
    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::CompressedImage>
        ("/image_raw/compressed", 1, imageCallback);
    // 订阅当前的位置信息
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, posi_read);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    // 新定义的发布
    ros::Publisher raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
    //发布 x y yaw 
    ros::Publisher xy_yaw_pub = nh.advertise<geometry_msgs::Pose2D>
            ("/mavros/flyoff/local", 10);
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

    geometry_msgs::Pose2D pose2d;
    
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
        pose2d.x=imp.px;
        pose2d.y=imp.py;
        pose2d.theta=imp.yaw;
        xy_yaw_pub.publish(pose2d);
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
            if((fabs(imp.px-raw_data.position.x)<0.04) && (fabs(imp.py-raw_data.position.y)<0.04)){
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
            if(fabs(imp.px-raw_data.position.x)<0.04 && fabs(imp.py-raw_data.position.y)<0.04 && fabs(imp.yaw-raw_data.yaw)<0.02){
                saveimg();
                ROS_INFO("pos1 successed"); 
                flag2 = 0;
                flag3 = 1;
            }
        }
        //路径规划
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag3==1)
        {
            ROS_INFO("pos planning"); 
            if(!path_point.empty()){
                ROS_INFO("pos planning fin");
                flag3=0;
                flag4=1;
            }else{
                select_path(1.7,1.5);
            } 
            last_request = ros::Time::now();          
        }
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag4==1)
        {
            ROS_INFO("pos 2");
            int pps= path_point.size();
            ROS_INFO("x:%f",path_point[ppi][0]);
            ROS_INFO("y:%f",path_point[ppi][1]);
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= path_point[ppi][0];
            raw_data.position.y= path_point[ppi][1];
            raw_data.position.z= 1.5;
            raw_data.yaw =0; 
            last_request = ros::Time::now();
            if(fabs(imp.px-raw_data.position.x)<0.04 && fabs(imp.py-raw_data.position.y)<0.04 && fabs(imp.yaw-raw_data.yaw)<0.02){
                ROS_INFO("pos 2 successed once");
                ppi++;
            }
            if(ppi>=pps){
                ROS_INFO("pos2 planning successed");
                flag4 = 0;
                flag5 = 1;
                ppi = 0;
                path_point.clear();
            }          
        }
        // 旋转
        if ( ros::Time::now()-last_request>ros::Duration(0.3) && flag5==1)
        {
            ROS_INFO("pos r");
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= bg.px-0.6*std::cos(bg.yaw);
            raw_data.position.y= bg.py-0.6*std::sin(bg.yaw);
            raw_data.position.z= 1.5;
            if (bg.yaw<=M_PI){raw_data.yaw = bg.yaw; }
            else{raw_data.yaw = bg.yaw-2*M_PI;}
            last_request = ros::Time::now();
            if(fabs(imp.px-raw_data.position.x)<0.03 && fabs(imp.py-raw_data.position.y)<0.03 && fabs(imp.yaw-raw_data.yaw)<0.02){
                // saveimg();
                ROS_INFO("pos r successed"); 
                ROS_INFO("dist:%f",ig.dist);
                if(bg.yaw>(M_PI*2)){
                    flag5=0;
                    flag6=1;
                    bg.yaw=0;
                }else{
                    bg.yaw=bg.yaw+M_PI/18;
                }
            }
        }
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag6==1)
        {
            ROS_INFO("pos planning");       
            if(!path_point.empty()){
                ROS_INFO("pos planning fin");
                flag6=0;
                flag7=1;
            }else{
                select_path(0.1,1.8);
            } 
            last_request = ros::Time::now();          
        }
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag7==1)
        {
            ROS_INFO("pos 3");
            int pps= path_point.size();
            ROS_INFO("x:%f",path_point[ppi][0]);
            ROS_INFO("y:%f",path_point[ppi][1]);
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= path_point[ppi][0];
            raw_data.position.y= path_point[ppi][1];
            raw_data.position.z= 1.5;
            raw_data.yaw =0; 
            last_request = ros::Time::now();
            if(fabs(imp.px-raw_data.position.x)<0.04 && fabs(imp.py-raw_data.position.y)<0.04 && fabs(imp.yaw-raw_data.yaw)<0.02){
                ROS_INFO("pos 3 successed once");
                ppi++;
            }
            if(ppi>=pps){
                ROS_INFO("pos3 planning successed");
                ppi=0;
                flag7=0;
                flag8=1;
                saveimg();
                img_process();
            }          
        }
        if ( ros::Time::now()-last_request>ros::Duration(2) && flag8==1)
        {
            ROS_INFO("pos1");
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= ig.x_dist;
            raw_data.position.y= ig.y_dist;
            raw_data.position.z= 0.5;
            raw_data.yaw =0; 
            if(fabs(imp.px-raw_data.position.x)<0.02 && fabs(imp.py-raw_data.position.y)<0.02 && fabs(imp.yaw-raw_data.yaw)<0.02){
                ROS_INFO("pos1 successed");
                break;
            }
            last_request = ros::Time::now();
            //ROS_INFO("yaw:%f",imp.yaw);
            //ROS_INFO("x:%f",imp.px);
        }
        pose2d.x=imp.px;
        pose2d.y=imp.py;
        pose2d.theta=imp.yaw;
        xy_yaw_pub.publish(pose2d);
        raw_local_pub.publish(raw_data);
        ros::spinOnce();
        rate.sleep();
    }
    // 降落步骤
    while(ros::ok()){

        if ( ros::Time::now()-last_request>ros::Duration(5) )
        {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("LAND");
            }    
            last_request = ros::Time::now();
            return 0;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}