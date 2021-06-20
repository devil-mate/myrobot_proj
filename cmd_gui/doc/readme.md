<!--
 * @Author: your name
 * @Date: 2021-03-11 16:57:17
 * @LastEditTime: 2021-03-11 17:25:08
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /pabi_linux/pabi_gui/doc/readme.md
-->
[toc]
# 使用说明
* 磁吸附爬壁机器人，AP通信，ip:192.168.12.2
   笔记本控制，IP：192.168.12.100
# 操作步骤：
1. AP上电，连接好网线，配置笔记本IP，
2. 打开mobaxterm中 ，连接WSL（WSL中主从机配置） 
3. 启动gui节点 roslaunch pabi_gui  pabi_gui.launch
4. 参数界面中设置PID参数，
5. 界面中的【openControlBox】按钮，打开控制界面，鼠标操作机器人运动。