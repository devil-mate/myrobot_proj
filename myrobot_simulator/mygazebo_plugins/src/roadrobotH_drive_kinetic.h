/*
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.

 *      * Redistributions in joint_state_binary form must reproduce the above copyright


 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_diff_drive.h
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */

#ifndef ROADROBOTH_DRIVE_KINETIC_H
#define ROADROBOTH_DRIVE_KINETIC_H

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#define PI 3.1415926
typedef struct {
    double len;
    double angle;
}vec_s; //向量(r,theta)
typedef struct {
    double x;
    double y;
    double theta;
}vector_s; //向量(x,y,theta)
namespace gazebo {

  class Joint;
  class Entity;

  class RoadrobotHDrive : public ModelPlugin {

    enum OdomSource
    {
        ENCODER = 0,
        WORLD = 1,
    };
    public:
      RoadrobotHDrive();
      ~RoadrobotHDrive();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
      void Reset();

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishOdometry(double step_time);
      void getWheelVelocities();
      void publishWheelTF(); /// publishes the wheel tf's
      void publishWheelJointState();
      void UpdateOdometryEncoder();


      GazeboRosPtr gazebo_ros_;
      physics::ModelPtr parent;
      event::ConnectionPtr update_connection_;

      double wheel_separation_;
      double wheel_diameter_;
      double wheel_torque;
      double leg_torque;
      double wheel_speed_[4];
//      double leg_angle[4];
      vec_s Vss[4];//Vc 每个轮子的最终和向量（大小，方向）。
	  double wheel_accel;
      double wheel_speed_instr_[4];

      std::vector<physics::JointPtr> joints_;
      std::vector<physics::JointPtr> jointsLeg_;

      // ROS STUFF
      ros::Publisher odometry_publisher_;
      ros::Subscriber cmd_vel_subscriber_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
      boost::shared_ptr<tf::TransformBroadcaster> transform_broadcasterLeg_;
      sensor_msgs::JointState joint_state_;
      sensor_msgs::JointState jointleg_state_;
      ros::Publisher joint_state_publisher_;
      nav_msgs::Odometry odom_;
      std::string tf_prefix_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string command_topic_;
      std::string odometry_topic_;
      std::string odometry_frame_;
      std::string robot_base_frame_;
      bool publish_tf_;
      bool legacy_mode_;
      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      // DiffDrive stuff
      void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

      double x_; //给定线速度Vx
      double y_; //Vy
      double rot_;
      bool alive_;

      // Update Rate
      double update_rate_,wheelBase_x,wheelBase_y;
      double update_period_;
      common::Time last_update_time_;

      OdomSource odom_source_;
      geometry_msgs::Pose2D pose_encoder_;
      common::Time last_odom_update_;

    // Flags
    bool publishWheelTF_;
    bool publishOdomTF_;
    bool publishWheelJointState_;

  };

}

#endif
