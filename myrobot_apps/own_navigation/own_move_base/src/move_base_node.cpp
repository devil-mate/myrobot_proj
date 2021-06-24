/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <move_base/move_base.h>
#include <tf2_ros/transform_listener.h>
#include <glog/logging.h>
bool glogInit(){
    // 日志等级分为INFO, WARNING, ERROR, FATAL,如果是FATAL级别这直接运行报错
    google::InitGoogleLogging("move_base_node.log");
    FLAGS_logtostderr = false;// 是否将日志输出到stderr而非文件。
    FLAGS_alsologtostderr = false; //是否将日志输出到文件和stderr，如果：true，忽略FLAGS_stderrthreshold的限制，所有信息打印到终端。
    FLAGS_stderrthreshold = google::INFO;    //INFO, WARNING, ERROR都输出，若为google::WARNING，则只输出WARNING, ERROR
    FLAGS_log_prefix = true; //设置日志前缀是否应该添加到每行输出。
    FLAGS_logbufsecs = 0; //设置可以缓冲日志的最大秒数，0指实时输出。
    FLAGS_max_log_size = 10; //设置最大日志文件大小（以MB为单位）。
    FLAGS_stop_logging_if_full_disk = true; //设置是否在磁盘已满时避免日志记录到磁盘。
    FLAGS_colorlogtostderr = true;  //log为彩色

    google::SetStderrLogging(google::GLOG_INFO); //大于指定级别的日志都输出到标准错误(包括自己)。注意：这个函数有时会失效，原因没有查到。
    google::SetLogDestination(google::GLOG_INFO, "/home/log/testLog/prefix_"); //日志的输出目录和前缀。
    google::SetLogFilenameExtension(".log"); //在日志文件名中级别后添加一个扩展名，适用于所有严重级别。
    // //会输出导致程序结束的信号,和google::InstallFailureWriter(&FatalMessageDump); 配合使用，可以在程序出现严重错误时将详细的错误信息打印出来
    // // google::InstallFailureSignalHandler(); //注册一下即可。默认是打印到stderr中，可以通过InstallFailureWriter更改输出目标。
    // // google::InstallFailureWriter(&FatalMessageDump);

}
bool glogInfoTest(){
   //基本用法：INFO，WAINING，ERROR
    LOG(INFO) << "Hello GLOG";                // << " cookies";
    LOG(WARNING) << "warning test";    // 会输出一个Warning日志
    LOG(ERROR) << "error test";        // 会输出一个Error日志
    // LOG(FATAL) << "fatal";  //严重错误程序退出， Logging a FATAL message terminates the program (after the message is logged)!

    //常见用法2：glog提供VLOG宏来提供自定义打印等级的功能.Verbose
    //可以通过命令行"--v=n",来控制VLOG的输出,VLOG(x),x<=n的情况,VLOG会输出,否则不输出.v默认为0,所以默认情况下VLOG(-1),VLOG(0)能够输出.
    //通常把调试的LOG设为1,这样在默认情况下不输出
    VLOG(-1) << "VLOG(-1) message.";
    VLOG(0) << "VLOG(0) message.";
    VLOG(1) << "VLOG(1) message.";
    VLOG(2) << "VLOG(2) message.";
    VLOG(3) << "VLOG(3) message."; 
    //常见用法3：
    // 如果工程为Debug模式,下面信息会输出,如果是release模式,下面信息不会输出.
    DLOG(INFO) << "DLOG:Debug Mode Message!";
    //用法4 条件输出
    LOG_IF(INFO, 89 > 4) << "89 > 4";
    // 用法5 频率输出，本质上是通过google::COUNTER这个计数机实现的
    LOG_EVERY_N(ERROR, 5) << "每隔5次输出一次 " ;
    LOG_FIRST_N(ERROR, 2) << "前两次输出 " ;
    LOG_IF_EVERY_N(WARNING, 1 < 2, 2) << "条件输出+频率" << google::COUNTER;
    //关闭log服务
    // google::ShutdownGoogleLogging();  
    return 0;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "move_base_node");
//   tf2_ros::Buffer buffer(ros::Duration(10));
//   tf2_ros::TransformListener tf(buffer);

//   move_base::MoveBase move_base( buffer );

  //ros::MultiThreadedSpinner s;
  ROS_INFO_STREAM("START");
  glogInit();
  glogInfoTest();
//   ros::spin();

  return(0);
}
