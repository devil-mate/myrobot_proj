[toc]
# 总体框架
说明：
myrobot_proj作为功能包集，下面包括各个功能包：
mygazebo: 启动gazebo仿真环境（加载world和robot）;包括了world模型，要能加载自己的机器人和其他ros机器人。
ros_robot包： 存放多种机器人文件，包括ros下载的机器人（自己写的urdf模型在mygazebo中），用于mygazebo加载。{或者应该把world模型放在这里来}
myrobot_simulator: 只是个文件夹，用作文件组织结构，包括各种仿真包（和这个平级的可以是用在实际机器人上的代码）
	仿真功能：激光slam、vslam、导航功能;
	另外加一些辅助功能：如按键控制包、gui、以及gazebo插件（应该放在mygazebo包里面）
# 
1.利用已有模块，快速搭建demo，实现turtlebot的定位与导航仿真，
2. 直接利用turtlebot_description包，启动turtlebot机器人模型。
myrobot_proj 的文件组织结构：
	a.myrobot_gazebo  在gazebo启动一个机器人模型，可以是自己的模型，如mygazebo包中的小车，也可以是turtlebot或其他git上的模型。所以，都是已有的模型，这个包里只需要启动文件，或者再加一些worlds，或者把自己建立的机器人模型和world可以都放在一起，不用到处找，只需要再launch中改变机器人名字就可以了，比如mygazebo中，或者就像turtlebot一样，做一个turtlebot_description包。
	b.navigation功能包集合的使用http://wiki.ros.org/cn/navigation/Tutorials/RobotSetup,也是一些launch文件及配置文件，因为源程序已有包。


# 工程下各个文件夹说明
 * **myrobot_simulator**： 利用已有的源程序/包搭建应用；只是启动文件（利用已有的功能包），没有具体的功能实现

* **myrobot_apps**: 
    * 只是一个文件夹，组织文件，和myrobot_simulator一样，自己实现各种包：如slam，navigation{名字叫myrobot,因为启动不同的robot的话是通过mygazebo启动不同的机器人，然后myrobot包就负责处理数据以及控制};
    * fundation_pra 包作为练习/test ，减少myROS_Proj下的代码。
    * myrobot_apps 下面实现自己的导航，slam功能等；其中，导航功能包作为功能包集合，下面有各种自己的全局/局部规划器。
* **mygazebo**
    包括 urdf 各个模块的模型；
	gazebo 模型需要的gazebo标签；
	robots 各个部分组合成的机器人，由urdf 和gazebo组成；
	launch 启动/加载到gazebo或者rviz中。

	
# myrobot_proj 包使用
* 基本步骤：
	1.  启动自己的世界和机器人模型：
	mygazebo/ myrobot_world.launch
		* 在该启动文件中更改世界模型和机器人模型
		* 或者，在一些已有的包中启动已有的模型，如turtlebot3_gazebo/turtlebot3_world.launch,(参数为自己的世界模型)

	2. 启动nvigation/SLAM包节点：
		moveBase_gmapping.launch

# 工程项目： tracked_vechile_ros
## 文件组织结构
* tracked_vehicle_ros: 文件夹，工程项目根目录
    * tracked_vehicle  ： ROS metapackage ，
        * tv_bringup: 启动包，模型解析/静态坐标变换，用于显示；系统启动launch
        * tv_tasks: 完成主要任务以及串口通信（负责和MCU交互）； {或者应该放到tv_bringup中？} 
        * tv_description： 模型描述包
        * tv_msgs: 自定义消息
        * common: 包,
            * TODO--把日志管理、错误管理放到common包，节点管理不纳入，需要根据不同工程调整不同的节点。
        * tv_gui:
        * tv_slam:
        * tv_navigation:
        * 其他包：日志管理、节点管理、错误管理等
    * rosPackageDepend: ros依赖
    * rosPackageAmmend： ros修改
    * tracked_vechicle_simulations:  仿真 metapackage
        * tv_gazebo: 主要就是gazebo
        * 
* 
## 启动与仿真
*  仿真的时候启动gazebo，不仿真的时候启动实际程序，由底盘给出各种数据。
* 
    * tv_bringup_base.launch 机器人底层启动（解析模型、得到基础数据等，比如启动gazebo）；然后再启动slam、navigation等,可以用来单独测试slam、navigation等; 启动多个文件配合
        * tv_navigation_own.launch 单独测试
    * tv_bringup_base_all.launch 
* 对于仿真，先不同的机器人使用不同的文件，基本步骤：
	1. 启动模型的rviz和gazebo
	2. 启动slam、navigation等 
	* 对于summit_xl 在lidar_3D_proj里面更改即可，对不同的项目，比如tracked_vechile项目，要验证时，只需要在slam或者navigation中启动summit_xl的模型即可（或者两次分开启动就行）。



# 项目：lidar_3D_proj
## 启动仿真
* 原工程：
	* summit_xl_complete.launch 是一个完整的amcl定位 的导航功能仿真。
    * summit_xl_gazebo.launch  除了启动模型（以及控制器和imu）外，后面的比如slam、navigation都是if条件的，所以，就是把多个仿真统一在一个文件里了{也就是启动模型，即只需要启动前面部分即可，后面的仿真使用自己的slam等进行仿真}{具体的参数使能，在该launch调用的下一级summit_xl_one_gazebo.launch}
* 一个基础的launch： summit_xl_sim_base.launch 
	* 启动rviz和gazebo
	
* 先不管，include中存放自己需要的launch