/*
    节点名：gazebo 仿真 图像处理
    节点描述： 测试 仿真
 */
// 引入类 这些东西可以在rostopic 里面找到
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include<nav_msgs/Odometry.h>
// C++ 标准库
#include <string> 
#include <iostream>
#include<cmath>
#include <vector>
#include<numeric>
#include <ctime>

//结构体 障碍物1位置
struct bar_g{
    double px = 1.375;
    double py = -0.875;
    double yaw = 0;
};
bar_g bg;

//结构体 障碍物2位置
struct bar_r{
    double px = 0.625;
    double py = -0.875;
    double yaw = 0;
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
        void transpoi(nav_msgs::Odometry current_posi);
};
// 四元素坐标轴转换
void imup::transpoi(nav_msgs::Odometry current_posi){
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
    double ox=current_posi.pose.pose.orientation.x;
    double oy =current_posi.pose.pose.orientation.y;
    double oz=current_posi.pose.pose.orientation.z;
    double ow=current_posi.pose.pose.orientation.w;
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
    px =  ceil(current_posi.pose.pose.position.x * 10000)/10000;
    py =  ceil(current_posi.pose.pose.position.y * 10000)/10000;
    pz =  ceil(current_posi.pose.pose.position.z * 10000)/10000;
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
        dist_rb,dist_gb,dist_dd -> 目标点，杆子与飞机距离距离x1
    */
        bool is_x, is_y;
        double k, kp, b, dist_rb, dist_gb, dist_dd, angle;
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
        // 方向设置
        void direction2point(double target_x,double target_y);
        // 坐标换算
        std::vector<float> convert_pos(double angle,double dist);
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
void info_point::direction2point(double target_x,double target_y){
    angle = std::atan2(target_y - py, target_x - px);
    std::cout<<"output angle"<<angle<<std::endl;
}
std::vector<float> info_point::convert_pos(double angle,double dist){
    std::vector<float> target;
    std::cout<<"angle:"<<angle<<std::endl;
    std::cout<<"dist:"<<dist<<std::endl;
    float x_dist = std::cos(angle)*dist;
    float y_dist = std::sin(angle)*dist;
    float fin_x = x_dist+px;
    float fin_y = y_dist+py;
    std::cout<<"fin_x:"<<fin_x<<std::endl;
    std::cout<<"fin_y:"<<fin_y<<std::endl;
    target.push_back(fin_x);
    target.push_back(fin_y);
    return target;
}
/*************************************************************************/
/*
    功能区：回调函数组
    描述：ros的订阅回调函数
*/
/*************************************************************************/

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
nav_msgs::Odometry current_posi;
void posi_read(const nav_msgs::Odometry::ConstPtr& msg){
    current_posi = *msg;
    ifp.transpoi(current_posi);
}
mavros_msgs::RCIn rc_info;
void rc_read(const mavros_msgs::RCIn::ConstPtr& msg){
    rc_info = *msg;
}
/*************************************************************************/
/*
    功能区：主函数
    描述：offboard主函数
    /realsense_plugin/camera/color/image_raw/compressed
    /image_raw/compressed

*/
/*************************************************************************/
int main(int argc, char **argv)
{
    int flag=0;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    // 订阅话题
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    // 订阅当前的位置信息
    ros::Subscriber position_sub = nh.subscribe<nav_msgs::Odometry>
            ("/camera/odom/sample", 10, posi_read);
    // 订阅遥控通道
    ros::Subscriber rcout_sub = nh.subscribe<mavros_msgs::RCIn>
            ("/mavros/rc/in", 10, rc_read);

    // 发布话题
    // ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
    //         ("mavros/setpoint_position/local", 10);
    // 新定义的发布
    ros::Publisher raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
    //发布 x y yaw 
    ros::Publisher xy_yaw_pub = nh.advertise<geometry_msgs::Pose2D>
            ("/flyoff/local", 10);
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

    raw_data.coordinate_frame = 8;  //flu坐标系
    raw_data.type_mask =  /* 1 +2 + 4 +*/ 8 +16 + 32 + 64 + 128 + 256 + 512  +1024 + 2048;
    raw_data.position.x= 0;
    raw_data.position.y= 0;
    raw_data.position.z= 1;
    
    //必须要有设定点才能飞
    for(int i = 100; ros::ok() && i > 0; --i){
        raw_local_pub.publish(raw_data);
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
              (ros::Time::now() - last_request > ros::Duration(1.0))
              && (rc_info.channels[4] > 1600)){
            if(set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(1.0))
                &&(rc_info.channels[4] > 1600)){
                if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                     ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        ROS_INFO("rc5_out:%d", rc_info.channels[4]);
        pose2d.x=ifp.px;
        pose2d.y=ifp.py;
        pose2d.theta=ifp.yaw;
        xy_yaw_pub.publish(pose2d);
        raw_local_pub.publish(raw_data);
        if (ros::Time::now()-last_request>ros::Duration(4)&& current_state.armed)
            break;
        ros::spinOnce();
        rate.sleep();
    }
    //第二阶段 飞行路径计算与设置
    while(ros::ok()){
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag==0)
        {
            ROS_INFO("pos_init");          
            raw_data.coordinate_frame = 8;  //flu坐标系
            raw_data.type_mask =  /* 1 +2 + 4 +*/ 8 +16 + 32 + 64 + 128 + 256 + 512  /*+1024*/ + 2048;
            raw_data.position.x= 0;
            raw_data.position.y= 0;
            raw_data.position.z= 1;
            raw_data.yaw =0;
            last_request = ros::Time::now();
            if((fabs(ifp.yaw - raw_data.yaw)<0.02)&&(fabs(ifp.px - raw_data.position.x)<0.02) && (fabs(ifp.py - raw_data.position.y)<0.02) && (fabs(ifp.pz - raw_data.position.z)<0.03)){
                ROS_INFO("init successed");
                flag++;
                ifp.direction2point(1,1);
            }
        }
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag==1)
        {
            ROS_INFO("pos_0");           
            raw_data.coordinate_frame = 8;  //flu坐标系
            raw_data.type_mask =  /* 1 +2 + 4 +*/ 8 +16 + 32 + 64 + 128 + 256 + 512  /*+1024*/ + 2048;
            raw_data.position.x= 0;
            raw_data.position.y= 0;
            raw_data.position.z= 1;
            raw_data.yaw = ifp.angle;
            last_request = ros::Time::now();
            if((fabs(ifp.px - raw_data.position.x)<0.02) && (fabs(ifp.py - raw_data.position.y)<0.02) && (fabs(ifp.pz - raw_data.position.z)<0.03)&& (fabs(ifp.yaw - raw_data.yaw)<0.02) ){
                ROS_INFO("pos_0 successed");
                flag++;
            }
        }
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag==2)
        {
            ROS_INFO("pos_1");           
            raw_data.coordinate_frame = 8;  //flu坐标系
            raw_data.type_mask =  /* 1 +2 + 4 +*/ 8 +16 + 32 + 64 + 128 + 256 + 512  /*+1024*/ + 2048;
            raw_data.position.x= 1;
            raw_data.position.y= 1;
            raw_data.position.z= 1;
            raw_data.yaw = ifp.angle;
            last_request = ros::Time::now();
            if((fabs(ifp.px - raw_data.position.x)<0.02) && (fabs(ifp.py - raw_data.position.y)<0.02) && (fabs(ifp.pz - raw_data.position.z)<0.03)&& (fabs(ifp.yaw - raw_data.yaw)<0.02)){
                ROS_INFO("pos_1 successed");
                ifp.angle=0;
                break;
            }
        }
        if ( ros::Time::now()-last_request>ros::Duration(1) && flag==3)
        {
            ROS_INFO("pos_2");           
            raw_data.coordinate_frame = 8;  //flu坐标系
            raw_data.type_mask =  /* 1 +2 + 4 +*/ 8 +16 + 32 + 64 + 128 + 256 + 512  /*+1024*/ + 2048;
            raw_data.position.x= 1;
            raw_data.position.y= 1;
            raw_data.position.z= 0.5;
            raw_data.yaw =0;
            last_request = ros::Time::now();
            if((fabs(ifp.px - raw_data.position.x)<0.02) && (fabs(ifp.py - raw_data.position.y)<0.02) && (fabs(ifp.pz - raw_data.position.z)<0.03)&&(fabs(ifp.yaw - raw_data.yaw)<0.02)){
                ROS_INFO("pos_2 successed");
                break;
            }
        }
        // 退出控制
        if (rc_info.channels[4] < 1600)
        {
            ROS_INFO("stop now");           
            raw_data.coordinate_frame = 8;  //flu坐标系
            raw_data.type_mask =  /* 1 +2 + 4 +*/ 8 +16 + 32 + 64 + 128 + 256 + 512  /*+1024*/ + 2048;
            raw_data.position.z= 0;
            raw_data.yaw = 0;
            last_request = ros::Time::now();
            if(ifp.pz<0.1){
                ROS_INFO("stop successed");
                break;
            }
        }
        pose2d.x=ifp.px;
        pose2d.y=ifp.py;
        pose2d.theta=ifp.yaw;
        xy_yaw_pub.publish(pose2d);
        raw_local_pub.publish(raw_data);
        ros::spinOnce();
        rate.sleep();
    }
    while(ros::ok()){
         if ( ros::Time::now()-last_request>ros::Duration(5) )
        {
            offb_set_mode.request.custom_mode = "AUTO.LAND";
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("LAND");
            }    
            last_request = ros::Time::now();
            if(!current_state.armed){
                ROS_INFO("LAND_ST successed");
                return 0;
            }
        }
        raw_local_pub.publish(raw_data);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}