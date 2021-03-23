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

	

