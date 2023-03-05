 <h1>mavros无人机offboard节点</h1>
<h2>目前实现的功能：</h2></br>
·基础飞行 </br>
·飞行校准</br>
·xyz坐标读取</br>
·pry坐标轴读取</br>
·图像读取保存</br>
·避障算法实现</br>
·图像识别降落</br>
·激光雷达信息转换</br>
</br>
<h2>待实现的功能：</h2>
·舵机控制实现
</br>


# source 环境变量
	source ~/catkin_ws/devel/setup.bash

# px4 gazebo sitl 编译 但是不建议使用该方法运行
	DONT_RUN=1 make px4_sitl gazebo

# px4 的直接启动 launch文件可以搜索教程自己改

<h2>launch文件 启动参数 roslaunch {包名} {文件名}</h2>

	roslaunch px4 posix_sitl.launch
	roslaunch simulation rtabmap_stereo_mapping_demo_px4.launch

# ros 启动节点 建议调试时单独启动 
	rosrun flyoff_pkg flyoff_node

# 使用一个遥控开关去开启关闭程序运行
目前方案是使用ros bridge


## 	ros的环境配置对初学者较为困难，但大部分问题都集中在网络上，解决过程中其实也能提升对于ubuntu这类linux系统的理解，对于px4无人机，从仿真到实践，主要经历几个阶段：

</br>
<h2>·熟悉ros话题发布和订阅</h2>
<h2>·熟悉ros节点启动文件编写和编译</h2>
<h2>·熟悉ros-gazebo仿真的自定义搭建</h2>
<h2>·能创建节点完整开发图像处理，路径规划等功能</h2></br>
<h3>上述内容是比较宽泛的，根据开发对象不同，涉及的具体建模，绑定骨骼，计算转动惯量，添加插件等等需要具体问题具体分析.</h3></br>