/*
    节点名：gazebo 仿真节点
    节点描述：实现飞行器自主飞行控制
 */
// 引入类 这些东西可以在rostopic 里面找到
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <image_transport/image_transport.h>
#include "sensor_msgs/image_encodings.h"
// opencv 库
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
// C++ 标准库
#include <string> 
#include <iostream>
#include<cmath>
#include <vector>
#include <ctime>

/****************************************************************************/
/*
    功能区：变量定义和声明
    描述：定义障碍物位置
*/
/****************************************************************************/

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

/*************************************************************************/
/*
    功能区：类组
    描述：坐标图像相关类的定义
*/
/*************************************************************************/
// 坐标处理
class imup{
    public:
    /*
        变量说明：6 轴坐标
    */
        double px, py, pz, roll, pitch, yaw;
        //  内联函数
        // 4元数坐标转换
        void transpoi(geometry_msgs::PoseStamped current_posi);
};
// 四元素坐标轴转换
void imup::transpoi(geometry_msgs::PoseStamped current_posi){
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
    double ox=current_posi.pose.orientation.x;
    double oy =current_posi.pose.orientation.y;
    double oz=current_posi.pose.orientation.z;
    double ow=current_posi.pose.orientation.w;
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (ow * ox + oy * oz);
    double cosr_cosp = 1 - 2 * (ox * ox + oy * oy);
    roll = std::atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = 2 * (ow * oy - oz * ox);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);
 
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (ow * oz + ox * oy);
    double cosy_cosp = 1 - 2 * (oy * oy + oz * oz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    // 6轴小数保留
    px =  ceil(current_posi.pose.position.x * 10000)/10000;
    py =  ceil(current_posi.pose.position.y * 10000)/10000;
    pz =  ceil(current_posi.pose.position.z * 10000)/10000;
}
/*************************************************************************/
// 图像处理类 坐标信息继承自坐标处理
class info_img: public imup{
    public:
    /*
        变量说明:
            imgCallback -> 图片矩阵
            dist -> 和目标点的 x , y 距离
            num_pic -> 图片编号
    */
        cv::Mat imgCallback;
        int width, height, num_pic;
        double dist, x_dist, y_dist;
        //  构造函数
        info_img();
        //  内联函数
        // 图像处理:降落处理
        void img_process();
        // 图像保存
        void saveimg();
};
info_img ig;

info_img::info_img(){
    num_pic = 0;
}
void info_img::saveimg(){
    num_pic++;
    std::string string_pic = std::to_string(num_pic);
    std::string path0 = "/home/uusei/img/shoot"+string_pic+".jpg";
    cv::imwrite(path0, imgCallback);
    cv::waitKey(50);
}
void info_img::img_process(){
    cv::Mat read,hsv,hsv0,hsv1,mask,res,gray,opened,kernel;
    std::vector<std::vector<cv::Point>> contours, cnts;
    std::vector<cv::Point> approx;
    std::vector<cv::Vec4i> hier;

    double dpx = px;
    double dpy = py;
    read = imgCallback;
    cv::blur(read, res,cv::Size(5, 5));
    //  二值化
    cv::cvtColor(res, gray, CV_BGR2GRAY);
    cv::threshold(gray, gray, 15, 255, CV_THRESH_BINARY); 
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
        if ((cv::contourArea(cnts.at(k)) >= 3000) && (cv::contourArea(cnts.at(k)) <= 102400) && (rect.width >=150)&& (rect.width <= 270)&& (rect.height >=150)&& (rect.height <= 270)){
            contours.push_back(cnts.at(k));
            std::cout<<"get contours"<<std::endl;
        }
    }
    for( int k = 0; k < contours.size(); k++ )
    {
        cv::Rect  rect = cv::boundingRect(contours.at(k));
        approxPolyDP(contours[k], approx, arcLength(contours[k], true)*0.02, true);
        std::cout<<" the quantity of edge "<<approx.size()<<std::endl;
        if((approx.size()<2)||(approx.size()>7)){
            int r_x = rect.x + rect.width / 2;
            int r_y = rect.y + rect.height / 2;
            double para_kx = 60.00 / rect.width;
            double para_ky = 60.00 / rect.height;
            std::cout<<"para_kx: "<<para_kx<<std::endl;
            std::cout<<"para_ky: "<<para_ky<<std::endl;
            x_dist= (r_y - 240) * para_ky * 0.01 * (-1) + dpx;
            y_dist= (r_x - 320) * para_kx * 0.01 * (-1) + dpy;
            std::cout<<"x"<<x_dist<<std::endl;
            std::cout<<"y"<<y_dist<<std::endl;
        } 
    }
}
/*************************************************************************/
// 避障类 继承坐标
class info_point:public imup{
    public:
    /*
        变量说明:
        1.普通变量:
        is_x,is_y -> 是否和 x  y 轴平行
        k  , kp -> 和目标点的斜率 和垂直方向的斜率
        dist_rb,dist_gb,dist_dd -> 目标点，杆子与飞机距离距离
    */
        bool is_x,is_y;
        double k,kp,b,dist_rb,dist_gb,dist_dd;
    /*
        2.向量和矩阵:
        path_point -> 无人机路径点 
        path_resist -> 障碍物点 
        cross_point -> 待选择的路径 
        bar_point -> 插入中间安全点的距离 第三版避障特性
        通过障碍物点确定路径点比较选择最小距离
        包含 line 的向量是后面等待传入二维矩阵的向量
    */
        std::vector<double> cross_point, cross_line, path_line, line_resist, bar_line;
        std::vector<std::vector<double>> path_point, path_resist, bar_point;
        //  构造函数
        info_point(){
            is_x = false;
            is_y = false;
        }
        //  内联函数
        // 路径规划 判断行进信息
        int smooth_path(double target_x,double target_y);
        // 计算坐标之间两点距离 
        double calculate_dist(double origin_x,double origin_y,double target_x,double target_y);
        // 点到直线距离
        double point2line(double y,double x,double k,double b);
        //路径规划 封装全部功能 设置路径点
        void select_path(double x,double y);
        // 路径规划 规划途径障碍物的路径点  圆心为障碍物坐标
        void set_path(double x_0,double y_0);
        // 取中间点 提高避障安全性
        void inplug2point(double origin_x,double origin_y,double target_x,double target_y);
        // 决定先去哪个杆子 路径最短
        void bar_path();
};
info_point ifp;

int info_point::smooth_path(double target_x,double target_y){
    /*
    返回值说明: 最终使用switch case 语句进行下一步操作
    0：不能走
    1：直接走
    2:   需要避障
    */
    // 定义参数
    std::vector<int> direction_d,direction_br,direction_bg,direction_rd,direction_rbr,direction_rbg;
    path_resist.clear();
    // y-kx-b 在 [-0.4,0.4]的范围内
    if (fabs(target_x-px)<0.03){
        is_y = true;
        is_x = false;
        k=0;
        b=target_x;
    }else if(fabs(target_y-py)<0.03){
        is_y = false;
        is_x = true;
        k=0;
        b=target_y;
    }else{
        is_y = false;
        is_x = false;
        k=(target_y-py)/(target_x-px);
        b=target_y-k*target_x;
        kp=(-1)/k;
        std::cout<<"kp:"<<kp<<std::endl;
    }
    // 距离测算
    dist_dd=calculate_dist(target_x, target_y, px, py);
    dist_rb=calculate_dist(px,py,br.px, br.py);
    dist_gb=calculate_dist(px,py,bg.px, bg.py);
    // 杆子位于的坐标系方向
    int dy=(target_y-py)>-0.3;
    int dx=(target_x-px)>-0.3;
    direction_d.push_back(dx);
    direction_d.push_back(dy);
    int aldy=(py-target_y)>-0.3;
    int aldx=(px-target_x)>-0.3;
    direction_rd.push_back(aldx);
    direction_rd.push_back(aldy);
    int ry=(br.py-py)>-0.3;
    int rx=(br.px-px)>-0.3;
    direction_br.push_back(rx);
    direction_br.push_back(ry);
    int alry=(py-br.py)>-0.3;
    int alrx=(px-br.px)>-0.3;
    direction_rbr.push_back(alrx);
    direction_rbr.push_back(alry);
    int gy=(bg.py-py)>-0.3;
    int gx=(bg.px-px)>-0.3;
    direction_bg.push_back(gx);
    direction_bg.push_back(gy);
    int algy=(py-bg.py)>-0.3;
    int algx=(px-bg.px)>-0.3;
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
        if(dist_rb>=dist_gb){
            if((point2line(bg.py, bg.px, k, b)<=0.45)&&(dist_dd>=dist_gb)){
                line_resist.push_back(bg.px);
                line_resist.push_back(bg.py);
                path_resist.push_back(line_resist);
            }
            line_resist.clear();
            if((point2line( br.py, br.px, k, b)<=0.45)&&(dist_dd>=dist_rb)){
                line_resist.push_back(br.px);
                line_resist.push_back(br.py);
                path_resist.push_back(line_resist);
            }
        }else{
            if((point2line( br.py, br.px, k, b)<=0.45)&&(dist_dd>=dist_rb)){
                line_resist.push_back(br.px);
                line_resist.push_back(br.py);
                path_resist.push_back(line_resist);
            }
            line_resist.clear();
            if((point2line( bg.py, bg.px, k, b)<=0.45)&&(dist_dd>=dist_gb)){
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
void info_point::select_path(double x,double y){
    std::vector<double>::iterator it;
    path_line.clear();
    path_point.clear();
    switch(smooth_path(x,y)){
        case 0:                  
            path_line.push_back(px);
            path_line.push_back(py);
            path_point.push_back(path_line);
            break;
        case 1:
            path_line.push_back(x);
            path_line.push_back(y);
            path_point.push_back(path_line);
            break;
        case 2:
            if(is_y){
                for(int i=0;i<path_resist.size();i++){
                    double cd_p0= calculate_dist(path_resist[i][0]+0.6,path_resist[i][1],px,py);
                    double cd_p1= calculate_dist(path_resist[i][0]-0.6,path_resist[i][1],px,py);
                    if(cd_p0<=cd_p1){
                        path_line.clear();
                        path_line.push_back(path_resist[i][0]+0.6);
                        path_line.push_back((path_resist[i][1]+py)/2);
                        path_point.push_back(path_line);
                        path_line.clear();
                        path_line.push_back(path_resist[i][0]+0.6);
                        path_line.push_back((path_resist[i][1]+y)/2);
                        path_point.push_back(path_line);
                    }else{
                        path_line.clear();
                        path_line.push_back(path_resist[i][0]-0.6);
                        path_line.push_back((path_resist[i][1]+py)/2);
                        path_point.push_back(path_line);
                        path_line.clear();
                        path_line.push_back(path_resist[i][0]-0.6);
                        path_line.push_back((path_resist[i][1]+y)/2);
                        path_point.push_back(path_line);
                    }            
                }
            }else if(is_x){
                for(int i=0;i<path_resist.size();i++){
                    double cd_p0= calculate_dist(path_resist[i][0],path_resist[i][1]+0.6,px,py);
                    double cd_p1= calculate_dist(path_resist[i][0],path_resist[i][1]-0.6,px,py);
                    if(cd_p0<=cd_p1){
                        path_line.clear();
                        path_line.push_back((path_resist[i][0]+px)/2);
                        path_line.push_back(path_resist[i][1]+0.6);
                        path_point.push_back(path_line);
                        path_line.clear();
                        path_line.push_back((path_resist[i][0]+x)/2);
                        path_line.push_back(path_resist[i][1]+0.6);
                        path_point.push_back(path_line);
                    }else{
                        path_line.clear();
                        path_line.push_back((path_resist[i][0]+px)/2);
                        path_line.push_back(path_resist[i][1]-0.6);
                        path_point.push_back(path_line);
                        path_line.clear();
                        path_line.push_back((path_resist[i][0]+x)/2);
                        path_line.push_back(path_resist[i][1]-0.6);
                        path_point.push_back(path_line);
                    }            
                }
            }else{
                for(int i=0;i<path_resist.size();i++){
                    set_path(path_resist[i][0],path_resist[i][1]);
                    double cd_p0= calculate_dist(cross_point[0],cross_point[1],px,py);
                    double cd_p1= calculate_dist(cross_point[2],cross_point[3],px,py);
                    if(cd_p0<=cd_p1){
                        inplug2point(px,py,cross_point[0],cross_point[1]);
                        it = cross_line.begin();
                        path_line.clear();
                        path_line.push_back(*it);
                        path_line.push_back(*(it + 1));
                        path_point.push_back(path_line);
                        path_line.clear();
                        path_line.push_back(cross_point[0]);
                        path_line.push_back(cross_point[1]);
                        path_point.push_back(path_line);
                    }else{
                        inplug2point(px,py,cross_point[2],cross_point[3]);
                        it = cross_line.begin();
                        path_line.clear();
                        path_line.push_back(*it);
                        path_line.push_back(*(it + 1));
                        path_point.push_back(path_line);
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
}
double info_point::calculate_dist(double origin_x,double origin_y,double target_x,double target_y){
    double dist;
    double l_x,l_y;
    l_x = target_x-origin_x;
    l_y = target_y-origin_y;
    dist = sqrt(l_x*l_x+l_y*l_y);
    std::cout<<"cd output dist:"<<dist<<std::endl;
    return dist;
}
double info_point::point2line(double y,double x,double k,double b){
    double xc,yc,dist;
    if(is_x){
        return fabs(y-b);
    }else if(is_y){
        return fabs(x-b);
    }else{
    xc = (x+k*(y-b))/(k*k+1);
    yc = k * xc + b;
    dist = calculate_dist(x,y,xc,yc);
    std::cout<<"p2l output dist:"<<dist<<std::endl;
    return dist;
    }
}
void info_point::set_path(double x_0,double y_0){
    /*  
    直线与圆的交点，反求出两个路径点 比较使用前面的距离计算函数
    */
    double bp,delta,ao,bo,co,solve_x0,solve_x1,solve_y0,solve_y1;
    // 求解二次方程的解集
    bp = y_0 - kp * x_0;
    std::cout<<"bp:"<<bp<<std::endl;
    ao = 1+ kp * kp;//a 大于1 不用担心被除
    bo = 2 * kp * (bp - y_0)- 2 * x_0;
    co = x_0 * x_0 + (bp - y_0) * (bp - y_0) - 0.49;
    delta = bo * bo-4 * ao * co;
    std::cout<<"a0:"<<ao<<"b0:"<<bo<<"co:"<<co<<"delta"<<delta<<std::endl;
    solve_x0 = ((-1)*bo + sqrt(delta)) / (2*ao);
    solve_y0 = kp * solve_x0 + bp;
    solve_x1 = ((-1) * bo - sqrt(delta)) / (2*ao);
    solve_y1 = kp * solve_x1 + bp;
    std::cout<<"x0:"<<solve_x0<<"y0:"<<solve_y0<<"x1:"<<solve_x1<<"y1:"<<solve_y1<<std::endl;
    cross_point.clear();
    cross_point.push_back(solve_x0);
    cross_point.push_back(solve_y0);
    cross_point.push_back(solve_x1);
    cross_point.push_back(solve_y1);
}
void info_point::inplug2point(double origin_x,double origin_y,double target_x,double target_y){
    /*
        目标点是之前圆心的交点 x0 , y0 ->target_x,target_y
        垂直外侧点是 x1 ，y1 -> set_x0,set_y0
    */
    cross_line.clear();
    std::cout<<"i2p output k:"<< k <<std::endl;
    double set_x0 = (target_x + origin_x) / 2;
    double set_y0 = (target_y + origin_y) / 2;
    double fi_x = (k * k * target_x - k * target_y + k * set_y0 + set_x0) / ( k * k + 1);
    double fi_y = k * (fi_x - target_x) + target_y;
    cross_line.push_back(fi_x);
    cross_line.push_back(fi_y);
    std::cout<<"i2p output x:"<< fi_x <<std::endl;
    std::cout<<"i2p output y:"<< fi_y <<std::endl;
}
void info_point::bar_path(){
    dist_rb=calculate_dist(br.px, br.py, px, py);
    dist_gb=calculate_dist(bg.px, bg.py, px, py);
    bar_line.clear();
    if(dist_gb>=dist_rb){
        bar_line.push_back(br.px);
        bar_line.push_back(br.py);
        bar_point.push_back(bar_line);
        bar_line.clear();
        bar_line.push_back(bg.px);
        bar_line.push_back(bg.py);
        bar_point.push_back(bar_line);
    }else{
        bar_line.push_back(bg.px);
        bar_line.push_back(bg.py);
        bar_point.push_back(bar_line);
        bar_line.clear();
        bar_line.push_back(br.px);
        bar_line.push_back(br.py);
        bar_point.push_back(bar_line);
    }
}
/*************************************************************************/
/*
    功能区：回调函数组
    描述：ros的订阅回调函数
*/
/*************************************************************************/
// 回调函数:读取图片
void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg){
    // 展示图片读取格式为bgr8 可以不显示，已经传入全局变量
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    ig.imgCallback = cv_ptr->image;
}
// 回调函数:状态
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
// 回调函数:位置
geometry_msgs::PoseStamped current_posi;
void posi_read(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_posi = *msg;
    ig.transpoi(current_posi);
    ifp.transpoi(current_posi);
}
/*******************************************************************************************************/
/*
    功能区介绍：主函数
    功能描述：
        1 执行飞行 降落功能
        2 接受话题 发布飞行话题
        /realsense_plugin/camera/color/image_raw/compressed
        /image_raw/compressed
*/
/*******************************************************************************************************/

int main(int argc, char **argv)
{
    int flag=0;
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

    // 第一阶段 解锁 与 起飞
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
        pose2d.x=ifp.px;
        pose2d.y=ifp.py;
        pose2d.theta=ifp.yaw;
        xy_yaw_pub.publish(pose2d);
        local_pos_pub.publish(pose);
        if ( ros::Time::now()-last_request>ros::Duration(10))
            break;
        ros::spinOnce();
        rate.sleep();
    }
    // 第二阶段 飞行路径计算与设置
    while(ros::ok()){
        // x0m y0m z1.5m 按照坐标系来
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag==0)
        {
            ROS_INFO("pos_init");           
            // raw_data.coordinate_frame = 8;  //flu坐标系
            raw_data.type_mask =  /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= 0;
            raw_data.position.y= 0;
            raw_data.position.z= 1.5;
            raw_data.yaw =0; 
            last_request = ros::Time::now();
            if((fabs(ifp.px-raw_data.position.x)<0.04) && (fabs(ifp.py-raw_data.position.y)<0.04)){
                ROS_INFO("init successed");
                flag++;
            }
        }
        // 飞到第2个点
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag==1)
        {
            ROS_INFO("pos1");
            // raw_data.coordinate_frame = 8;  //flu坐标系
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= 0;
            raw_data.position.y= 1;
            raw_data.position.z= 1.5;
            raw_data.yaw =0; 
            last_request = ros::Time::now(); 
            //ROS_INFO("yaw:%f",ifp.yaw);
            //ROS_INFO("x:%f",ifp.px);
            // 闭环校正位置
            if(fabs(ifp.px-raw_data.position.x)<0.04 && fabs(ifp.py-raw_data.position.y)<0.04 && fabs(ifp.yaw-raw_data.yaw)<0.02){
                ig.saveimg();
                ROS_INFO("pos1 successed");
                ifp.bar_path();
                flag++;
            }
        }
        //路径规划
        if ( ros::Time::now()-last_request>ros::Duration(0.6) && flag==2)
        {
            ROS_INFO("pos planning"); 
            if(!ifp.path_point.empty()){
                ROS_INFO("pos planning fin");
                flag++;
            }else{
                ifp.select_path(ifp.bar_point[0][0]-0.6,ifp.bar_point[0][1]);
            } 
            last_request = ros::Time::now();          
        }
        if ( ros::Time::now()-last_request>ros::Duration(0.6) && flag==3)
        {
            ROS_INFO("pos 2");
            int pps= ifp.path_point.size();
            ROS_INFO("x:%f",ifp.path_point[ppi][0]);
            ROS_INFO("y:%f",ifp.path_point[ppi][1]);
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= ifp.path_point[ppi][0];
            raw_data.position.y= ifp.path_point[ppi][1];
            raw_data.position.z= 1.5;
            raw_data.yaw =0; 
            last_request = ros::Time::now();
            if(fabs(ifp.px-raw_data.position.x)<0.04 && fabs(ifp.py-raw_data.position.y)<0.04 && fabs(ifp.yaw-raw_data.yaw)<0.02){
                ROS_INFO("pos 2 successed once");
                ppi++;
            }
            if(ppi>=pps){
                ROS_INFO("pos2 planning successed");
                flag++;
                ppi = 0;
                ifp.path_point.clear();
            }          
        }

        // 旋转
        if ( ros::Time::now()-last_request>ros::Duration(0.3) && flag==4)
        {
            ROS_INFO("pos r");
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= ifp.bar_point[0][0]-0.6*std::cos(br.yaw);
            raw_data.position.y= ifp.bar_point[0][1]-0.6*std::sin(br.yaw);
            raw_data.position.z= 1.5;
            if (br.yaw<=M_PI){raw_data.yaw = br.yaw; }
            else{raw_data.yaw = br.yaw-2*M_PI;}
            last_request = ros::Time::now();
            if(fabs(ifp.px-raw_data.position.x)<0.03 && fabs(ifp.py-raw_data.position.y)<0.03 && fabs(ifp.yaw-raw_data.yaw)<0.02){
                // saveimg();
                ROS_INFO("pos r successed"); 
                if(br.yaw>(M_PI*2)){
                    flag++;
                    br.yaw=0;
                }else{
                    br.yaw=br.yaw+M_PI/18;
                }
            }
        }
        if ( ros::Time::now()-last_request>ros::Duration(0.6) && flag==5)
        {
            ROS_INFO("pos1");
            // raw_data.coordinate_frame = 8;  //flu坐标系
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= ifp.bar_point[0][0]-0.6;
            raw_data.position.y= ifp.bar_point[0][1];
            raw_data.position.z= 1.5;
            raw_data.yaw =0; 
            last_request = ros::Time::now(); 
            // 闭环校正位置
            if(fabs(ifp.px-raw_data.position.x)<0.04 && fabs(ifp.py-raw_data.position.y)<0.04 && fabs(ifp.yaw-raw_data.yaw)<0.02 && fabs(ifp.pz-raw_data.position.z)<0.05){
                ig.saveimg();
                ROS_INFO("pos1 successed");
                ifp.bar_path();
                flag++;
            }
        }
        if ( ros::Time::now()-last_request>ros::Duration(0.6) && flag==6)
        {
            ROS_INFO("pos planning"); 
            if(!ifp.path_point.empty()){
                ROS_INFO("pos planning fin");
                flag++;
            }else{
                ifp.select_path(ifp.bar_point[1][0] - 0.6, ifp.bar_point[1][1]);
            } 
            last_request = ros::Time::now();          
        }
        if ( ros::Time::now()-last_request>ros::Duration(0.6) && flag==7)
        {
            ROS_INFO("pos 3");
            int pps= ifp.path_point.size();
            ROS_INFO("x:%f",ifp.path_point[ppi][0]);
            ROS_INFO("y:%f",ifp.path_point[ppi][1]);
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= ifp.path_point[ppi][0];
            raw_data.position.y= ifp.path_point[ppi][1];
            raw_data.position.z= 1.5;
            raw_data.yaw =0; 
            last_request = ros::Time::now();
            if(fabs(ifp.px-raw_data.position.x)<0.04 && fabs(ifp.py-raw_data.position.y)<0.04 && fabs(ifp.yaw-raw_data.yaw)<0.02){
                ROS_INFO("pos 3 successed once");
                ppi++;
            }
            if(ppi>=pps){
                ROS_INFO("pos3 planning successed");
                flag++;
                ppi = 0;
                ifp.path_point.clear();
            }          
        }
        // 旋转
        if ( ros::Time::now()-last_request>ros::Duration(0.3) && flag==8)
        {
            ROS_INFO("pos r2");
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= ifp.bar_point[1][0]-0.6*std::cos(bg.yaw);
            raw_data.position.y= ifp.bar_point[1][1]-0.6*std::sin(bg.yaw);
            raw_data.position.z= 1.5;
            if (bg.yaw<=M_PI){raw_data.yaw = bg.yaw; }
            else{raw_data.yaw = bg.yaw-2*M_PI;}
            last_request = ros::Time::now();
            if(fabs(ifp.px-raw_data.position.x)<0.03 && fabs(ifp.py-raw_data.position.y)<0.03 && fabs(ifp.yaw-raw_data.yaw)<0.02){
                // saveimg();
                ROS_INFO("pos r2 successed"); 
                if(bg.yaw>(M_PI*2)){
                    flag++;
                    bg.yaw=0;
                }else{
                    bg.yaw=bg.yaw+M_PI/18;
                }
            }
        }
        if ( ros::Time::now()-last_request>ros::Duration(0.6) && flag==9)
        {
            ROS_INFO("pos planning");       
            if(!ifp.path_point.empty()){
                ROS_INFO("pos planning fin");
                flag++;
            }else{
                ifp.select_path(0.1,1.8);
            } 
            last_request = ros::Time::now();          
        }
        if ( ros::Time::now()-last_request>ros::Duration(0.5) && flag==10)
        {
            ROS_INFO("pos 4");
            int pps= ifp.path_point.size();
            ROS_INFO("x:%f",ifp.path_point[ppi][0]);
            ROS_INFO("y:%f",ifp.path_point[ppi][1]);
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x= ifp.path_point[ppi][0];
            raw_data.position.y= ifp.path_point[ppi][1];
            raw_data.position.z= 1.5;
            raw_data.yaw =0;
            last_request = ros::Time::now();
            if(fabs(ifp.px-raw_data.position.x)<0.04 && fabs(ifp.py-raw_data.position.y)<0.04  && fabs(ifp.yaw-raw_data.yaw)<0.02 && fabs(ifp.pz-raw_data.position.z)<0.04){
                ROS_INFO("pos 4 successed once");
                ppi++;
            }
            if(ppi>=pps){
                ROS_INFO("pos4 planning successed");
                ppi=0;
                flag++;
            }          
        }
         if ( ros::Time::now()-last_request>ros::Duration(0.6) && flag==11)
        {
            ROS_INFO("find land");
            if((ig.x_dist!=0)&&(ig.y_dist!=0)){
                ROS_INFO("find land fin");
                flag++;
            }else{
                ig.saveimg();
                ig.img_process();
            } 
            last_request = ros::Time::now();          
        }
        if ( ros::Time::now()-last_request>ros::Duration(0.5) && flag==12)
        {
            ROS_INFO("land pos");
            raw_data.type_mask = /* 1 +2 + 4 + 8 +16 + 32 + 64 + 128 + 256 + */512  /*+1024*/ + 2048;
            raw_data.position.x = ig.x_dist;
            raw_data.position.y = ig.y_dist;
            raw_data.position.z = 0.25;
            raw_data.yaw = 0;
            if(fabs(ifp.px-raw_data.position.x)<0.04 && fabs(ifp.py-raw_data.position.y)<0.04 && fabs(ifp.pz-raw_data.position.z)<0.03){
                ROS_INFO("pos land successed");
                ig.saveimg();
                break;
            }
            last_request = ros::Time::now();
            //ROS_INFO("yaw:%f",ifp.yaw);
            //ROS_INFO("x:%f",ifp.px);
        }
        pose2d.x=ifp.px;
        pose2d.y=ifp.py;
        pose2d.theta=ifp.yaw;
        xy_yaw_pub.publish(pose2d);
        raw_local_pub.publish(raw_data);
        ros::spinOnce();
        rate.sleep();
    }
    // 第三阶段 飞机降落
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