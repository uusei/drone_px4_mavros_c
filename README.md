 <h1>mavros无人机offboard节点</h1>
<h2>目前实现的功能：</h2></br>
·基础飞行 </br>·飞行校准</br>·xyz坐标读取</br>·pry坐标轴读取</br>·图像读取保存</br>
·避障算法实现</br>
<h2>待实现的功能：</h2>
·图像识别</br>
·距离测量</br>

# source 环境变量
	source ~/catkin_ws/devel/setup.bash

# px4 sitl 编译
	make px4_sitl gazebo

# px4 的直接启动
	roslaunch px4 posix_sitl.launch
	roslaunch simulation rtabmap_stereo_mapping_demo_px4.launch

# ros 启动
	rosrun flyoff_pkg flyoff_node
