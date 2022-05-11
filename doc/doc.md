[toc]
# 总体框架用途
* 

* 用途：
    * 所有的机器人相关的练习，包括pcl，opencv等都在这里；ros只是作为通框架，比如可以ros相关的单独作为一个类。

    * 利用已有模块，快速搭建demo，实现turtlebot的定位与导航仿真，
# 基本组织结构：
* myrobot_proj 功能包集
    * mygazebo: 
        * 启动gazebo仿真环境（加载world和robot）;包括了world模型，要能加载自己的机器人和其他ros机器人。
    * ros_robot_descriptions： 
        * 存放多种机器人文件，包括ros下载的机器人（自己写的urdf模型在mygazebo中），用于mygazebo加载。{或者应该把world模型放在这里来}
    * myrobot_simulator: 只是个文件夹，用作文件组织结构，包括各种仿真包（和这个平级的可以是用在实际机器人上的代码）
	仿真功能：激光slam、vslam、导航功能;
	另外加一些辅助功能：如按键控制包、gui、以及gazebo插件（应该放在mygazebo包里面）
        *********
    * > 新增功能包
    * 