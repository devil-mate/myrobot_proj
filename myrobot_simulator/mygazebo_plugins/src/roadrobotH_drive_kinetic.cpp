/*
    Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
 * \file  gazebo_ros_diff_drive.cpp
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */


/*
 *
 * The support of acceleration limit was added by
 * \author   George Todoran <todorangrg@gmail.com>
 * \author   Markus Bader <markus.bader@tuwien.ac.at>
 * \date 22th of May 2014
 */

#include <algorithm>
#include <assert.h>

#include "roadrobotH_drive_kinetic.h"

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>

namespace gazebo
{

enum {
    LEFT_FRONT,
    RIGHT_FRONT,
    RIGHT_BACK,
    LEFT_BACK,

};

RoadrobotHDrive::RoadrobotHDrive():ModelPlugin() {
    ROS_INFO_STREAM("---------------hello mygazebo_plugin--------");
}

// Destructor
RoadrobotHDrive::~RoadrobotHDrive() 
{
	FiniChild();
}

// Load the controller
void RoadrobotHDrive::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "DiffDrive" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint" );
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishOdomTF_, "publishOdomTF", true);
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );
    gazebo_ros_->getParameterBoolean ( legacy_mode_, "legacyMode", true );

    if (!_sdf->HasElement("legacyMode"))
    {
      ROS_ERROR_NAMED("diff_drive", "RoadrobotHDrive Plugin missing <legacyMode>, defaults to true\n"
	       "This setting assumes you have a old package, where the right and left wheel are changed to fix a former code issue\n"
	       "To get rid of this error just set <legacyMode> to false if you just created a new package.\n"
	       "To fix an old package you have to exchange left wheel by the right wheel.\n"
	       "If you do not want to fix this issue in an old package or your z axis points down instead of the ROS standard defined in REP 103\n"
	       "just set <legacyMode> to true.\n"
      );
    }

    gazebo_ros_->getParameter<double> ( wheel_separation_, "wheelSeparation", 0.34 );
    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.3 );
    gazebo_ros_->getParameter<double> ( wheel_accel, "wheelAcceleration", 0.2 );
    gazebo_ros_->getParameter<double> ( wheel_torque, "wheelTorque", 500.0 );
    gazebo_ros_->getParameter<double> ( leg_torque, "legTorque", 100.0 );
    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 100.0 );
    gazebo_ros_->getParameter<double> ( wheelBase_x, "wheelBase_x", 0.4 );
    gazebo_ros_->getParameter<double> ( wheelBase_y, "wheelBase_y", 0.7 );

    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );

 
    joints_.resize ( 4 );
 
    joints_[LEFT_FRONT] = gazebo_ros_->getJoint ( parent, "leftFrontJoint", "left_FrontJoint" );

    joints_[RIGHT_FRONT] = gazebo_ros_->getJoint ( parent, "rightFrontJoint", "rightFrontJoint" );

    joints_[LEFT_BACK] = gazebo_ros_->getJoint ( parent, "leftBackJoint", "left_BackJoint" );
    
    joints_[RIGHT_BACK] = gazebo_ros_->getJoint ( parent, "rightBackJoint", "right_BackJoint" );

    joints_[LEFT_FRONT]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT_FRONT]->SetParam ( "fmax", 0, wheel_torque );
    joints_[LEFT_BACK]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT_BACK]->SetParam ( "fmax", 0, wheel_torque );

    jointsLeg_.resize(4);
    jointsLeg_[0] = gazebo_ros_->getJoint ( parent, "leftFrontLegJoint", "left_FrontLegJoint" );
    jointsLeg_[1] = gazebo_ros_->getJoint ( parent, "rightFrontLegJoint", "right_FrontLegJoint" );
    jointsLeg_[2] = gazebo_ros_->getJoint ( parent, "rightBackLegJoint", "right_backLegJoint" );
    jointsLeg_[3] = gazebo_ros_->getJoint ( parent, "leftBackLegJoint", "left_BackLegJoint" );

    jointsLeg_[0]->SetParam ( "fmax", 0, leg_torque );
    jointsLeg_[1]->SetParam ( "fmax", 0, leg_torque );
    jointsLeg_[2]->SetParam ( "fmax", 0, leg_torque );
    jointsLeg_[3]->SetParam ( "fmax", 0, leg_torque );

   

    this->publish_tf_ = true;
    if (!_sdf->HasElement("publishTf")) {
      ROS_WARN_NAMED("diff_drive", "RoadrobotHDrive Plugin (ns = %s) missing <publishTf>, defaults to %d",
          this->robot_namespace_.c_str(), this->publish_tf_);
    } else {
      this->publish_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
#if GAZEBO_MAJOR_VERSION >= 8
    last_update_time_ = parent->GetWorld()->SimTime();
#else
    last_update_time_ = parent->GetWorld()->GetSimTime();
#endif

    // Initialize velocity stuff
    wheel_speed_[RIGHT_FRONT] = 0;
    wheel_speed_[LEFT_FRONT] = 0;
    wheel_speed_[RIGHT_BACK] = 0;
    wheel_speed_[LEFT_BACK] = 0;
    // leg_angle={0,0,0,0};

    // Initialize velocity support stuff
    wheel_speed_instr_[RIGHT_FRONT] = 0;
    wheel_speed_instr_[LEFT_FRONT] = 0;
    wheel_speed_instr_[RIGHT_FRONT] = 0;
    wheel_speed_instr_[LEFT_FRONT] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;


    if (this->publishWheelJointState_)
    {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO_NAMED("diff_drive", "%s: Advertise joint_states", gazebo_ros_->info());
    }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO_NAMED("diff_drive", "%s: Try to subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                boost::bind(&RoadrobotHDrive::cmdVelCallback, this, _1),
                ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("diff_drive", "%s: Subscribe to %s", gazebo_ros_->info(), command_topic_.c_str());

    if (this->publish_tf_)
    {
      odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
      ROS_INFO_NAMED("diff_drive", "%s: Advertise odom on %s ", gazebo_ros_->info(), odometry_topic_.c_str());
    }

    // start custom queue for diff drive
    this->callback_queue_thread_ =
        boost::thread ( boost::bind ( &RoadrobotHDrive::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin ( boost::bind ( &RoadrobotHDrive::UpdateChild, this ) );

}

void RoadrobotHDrive::Reset()
{
#if GAZEBO_MAJOR_VERSION >= 8
  last_update_time_ = parent->GetWorld()->SimTime();
#else
  last_update_time_ = parent->GetWorld()->GetSimTime();
#endif
  pose_encoder_.x = 0;
  pose_encoder_.y = 0;
  pose_encoder_.theta = 0;
  x_ = 0;
  rot_ = 0;
    joints_[LEFT_FRONT]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT_FRONT]->SetParam ( "fmax", 0, wheel_torque );
    joints_[LEFT_BACK]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT_BACK]->SetParam ( "fmax", 0, wheel_torque );

    jointsLeg_[LEFT_FRONT]->SetParam ( "fmax", 0, leg_torque );
    jointsLeg_[RIGHT_FRONT]->SetParam ( "fmax", 0, leg_torque );
    jointsLeg_[LEFT_BACK]->SetParam ( "fmax", 0, leg_torque );
    jointsLeg_[RIGHT_BACK]->SetParam ( "fmax", 0, leg_torque );
}

void RoadrobotHDrive::publishWheelJointState()
{
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( 8 );
    joint_state_.position.resize ( 8);
    joint_state_.velocity.resize ( 8);
//0-3  wheel;4-7 leg
    for ( int i = 0; i < 4; i++ ) {
        physics::JointPtr joint = joints_[i];
#if GAZEBO_MAJOR_VERSION >= 8
            double position = joint->Position(0);
            double velocity = joint->GetVelocity(0);
#else
            double position = joint->GetAngle ( 0 ).Radian();
#endif
            joint_state_.name[i] = joint->GetName();
            joint_state_.position[i] = position;
            joint_state_.velocity[i] = velocity;
    }
    for ( int i = 0; i < 4; i++ ) {
        physics::JointPtr joint = jointsLeg_[i];
#if GAZEBO_MAJOR_VERSION >= 8
        double position = joint->Position ( 0 );
#else
        double position = joint->GetAngle ( 0 ).Radian();
#endif
        joint_state_.name[i+4] = joint->GetName();
        joint_state_.position[i+4] = position;
    }
    joint_state_publisher_.publish ( joint_state_ );
}

void RoadrobotHDrive::publishWheelTF()
{
    ros::Time current_time = ros::Time::now();
    for ( int i = 0; i < 4; i++ ) {

        std::string wheel_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName ());
        std::string wheel_parent_frame = gazebo_ros_->resolveTF(jointsLeg_[i]->GetParent()->GetName ());


//        std::string leg_frame = gazebo_ros_->resolveTF(jointsLeg_[i]->GetChild()->GetName ());
//        std::string leg_parent_frame = gazebo_ros_->resolveTF(jointsLeg_[i]->GetParent()->GetName ());
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->RelativePose();
//        ignition::math::Pose3d poseLeg = jointsLeg_[i]->GetChild()->RelativePose();
#else
        ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->GetRelativePose().Ign();
//        ignition::math::Pose3d poseLeg = joints_[i]->GetChild()->GetRelativePose().Ign();
#endif

        tf::Quaternion qt ( poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W() );
        tf::Vector3 vt ( poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z() );

        tf::Transform tfWheel ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );

//        tf::Quaternion qt_leg ( poseLeg.Rot().X(), poseLeg.Rot().Y(), poseLeg.Rot().Z(), poseLeg.Rot().W() );
//        tf::Vector3 vt_leg ( poseLeg.Pos().X(), poseLeg.Pos().Y(), poseLeg.Pos().Z() );
//
//        tf::Transform tf_leg ( qt_leg, vt_leg );
//        transform_broadcaster_->sendTransform (
//                tf::StampedTransform ( tf_leg, current_time, leg_parent_frame, leg_frame ) );
    }
    for ( int i = 0; i < 4; i++ ) {

//        std::string wheel_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName ());
//        std::string wheel_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName ());

        std::string leg_frame = gazebo_ros_->resolveTF(jointsLeg_[i]->GetChild()->GetName ());
        std::string leg_parent_frame = gazebo_ros_->resolveTF(jointsLeg_[i]->GetParent()->GetName ());

#if GAZEBO_MAJOR_VERSION >= 8
//        ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->RelativePose();
        ignition::math::Pose3d poseLeg = jointsLeg_[i]->GetChild()->RelativePose();
#else
        ignition::math::Pose3d poseWheel = joints_[i]->GetChild()->GetRelativePose().Ign();
        ignition::math::Pose3d poseLeg = joints_[i]->GetChild()->GetRelativePose().Ign();
#endif

//        tf::Quaternion qt ( poseWheel.Rot().X(), poseWheel.Rot().Y(), poseWheel.Rot().Z(), poseWheel.Rot().W() );
//        tf::Vector3 vt ( poseWheel.Pos().X(), poseWheel.Pos().Y(), poseWheel.Pos().Z() );
//
//        tf::Transform tfWheel ( qt, vt );
//        transform_broadcaster_->sendTransform (
//                tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );

        tf::Quaternion qt_leg ( poseLeg.Rot().X(), poseLeg.Rot().Y(), poseLeg.Rot().Z(), poseLeg.Rot().W() );
        tf::Vector3 vt_leg ( poseLeg.Pos().X(), poseLeg.Pos().Y(), poseLeg.Pos().Z() );

        tf::Transform tf_leg ( qt_leg, vt_leg );
        transform_broadcaster_->sendTransform (
                tf::StampedTransform ( tf_leg, current_time, leg_parent_frame, leg_frame ) );
    }
}

// Update the controller
void RoadrobotHDrive::UpdateChild()
{

    /* force reset SetParam("fmax") since Joint::Reset reset MaxForce to zero at
       https://bitbucket.org/osrf/gazebo/src/8091da8b3c529a362f39b042095e12c94656a5d1/gazebo/physics/Joint.cc?at=gazebo2_2.2.5#cl-331
       (this has been solved in https://bitbucket.org/osrf/gazebo/diff/gazebo/physics/Joint.cc?diff2=b64ff1b7b6ff&at=issue_964 )
       and Joint::Reset is called after ModelPlugin::Reset, so we need to set maxForce to wheel_torque other than RoadrobotHDrive::Reset
       (this seems to be solved in https://bitbucket.org/osrf/gazebo/commits/ec8801d8683160eccae22c74bf865d59fac81f1e)
    */
    for ( int i = 0; i < 4; i++ ) {
      if ( fabs(wheel_torque -joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
        joints_[i]->SetParam ( "fmax", 0, wheel_torque );
      }
    }
    for ( int i = 0; i < 4; i++ ) {
        if ( fabs(leg_torque -jointsLeg_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
            jointsLeg_[i]->SetParam ( "fmax", 0, leg_torque );
        }
    }


    if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

    if ( seconds_since_last_update > update_period_ ) {
        if (this->publish_tf_) publishOdometry ( seconds_since_last_update );

        if ( publishWheelTF_ ) {
            publishWheelTF();
        }

        if ( publishWheelJointState_ ){
            publishWheelJointState();

        }

        // Update robot in case new velocities have been requested
        getWheelVelocities();
        // ROS_ERROR_STREAM("---------------------get vel");

        double current_speed[4];
        double setWheelSpeed[4],setLegPosition[4];

        current_speed[LEFT_FRONT] = joints_[LEFT_FRONT]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
        current_speed[RIGHT_FRONT] = joints_[RIGHT_FRONT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
        current_speed[LEFT_BACK] = joints_[LEFT_BACK]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
        current_speed[RIGHT_BACK] = joints_[RIGHT_BACK]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
//        if(0){
        if ( ( fabs ( wheel_speed_[LEFT_FRONT] - current_speed[LEFT_FRONT] ) < 0.01 ) ||
                ( fabs ( wheel_speed_[RIGHT_FRONT] - current_speed[RIGHT_FRONT] ) < 0.01 ) ) {
            //if max_accel == 0, or target speed is reached
//            joints_[LEFT_FRONT]->SetParam ( "vel", 0, wheel_speed_[LEFT_FRONT]/ ( wheel_diameter_ / 2.0 ) );
//            joints_[RIGHT_FRONT]->SetParam ( "vel", 0, wheel_speed_[RIGHT_FRONT]/ ( wheel_diameter_ / 2.0 ) );
//            joints_[LEFT_BACK]->SetParam ( "vel", 0, wheel_speed_[LEFT_FRONT]/ ( wheel_diameter_ / 2.0 ) );
//            joints_[LEFT_BACK]->SetParam ( "vel", 0, wheel_speed_[RIGHT_FRONT]/ ( wheel_diameter_ / 2.0 ) );

        } else {
            for (int i = 0; i < 4; i++) {
                if (wheel_speed_[i] >= current_speed[i])
                    wheel_speed_instr_[i] += fmin(wheel_speed_[i] - current_speed[i],
                                                  wheel_accel * seconds_since_last_update);
                else
                    wheel_speed_instr_[i] += fmax(wheel_speed_[i] - current_speed[i],
                                                  -wheel_accel * seconds_since_last_update);
            }
        }

            for(int i=0;i<4;i++){
////                joints_[i]->SetParam ( "vel", 0, wheel_speed_instr_[i] / ( wheel_diameter_ / 2.0 ) );
                joints_[i]->SetParam ( "vel", 0, Vss[i].len / ( wheel_diameter_ / 2.0 ) );
                jointsLeg_[i]->SetPosition (  0, Vss[i].angle);

                ROS_INFO_STREAM("======set velocity "
                                     <<"Vss[i].len"<<Vss[i].len <<"  "<<Vss[i].len / ( wheel_diameter_ / 2.0 )
                                     <<"..."<<Vss[i].angle/3.1415926*180
                                     <<"current_speed"<<current_speed[i]);

            }
//        jointsLeg_[0]->SetPosition (  0, 3.1415926/6);
//        jointsLeg_[1]->SetPosition (  0, 3.1415926/6);
//        jointsLeg_[2]->SetPosition (  0, -3.1415926/3);
//        jointsLeg_[3]->SetPosition (  0, -3.1415926/6);
//        joints_[0]->SetParam ( "vel", 0, Vss[0].len / ( wheel_diameter_ / 2.0 ) );
//        joints_[1]->SetParam ( "vel", 0, Vss[1].len / ( wheel_diameter_ / 2.0 ) );
//        joints_[3]->SetParam ( "vel", 0, Vss[1].len / ( wheel_diameter_ / 2.0 ) );
//        ROS_INFO_STREAM("set v:" <<Vss[1].len );

        last_update_time_+= common::Time ( update_period_ );
    }
}

// Finalize the controller
void RoadrobotHDrive::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    gazebo_ros_->node()->shutdown();
    callback_queue_thread_.join();
}

void RoadrobotHDrive::getWheelVelocities()
{
    boost::mutex::scoped_lock scoped_lock ( lock );

    double V_x = 0;
    double V_y = 0;
    double va = 0;
    V_x = x_;
    V_y = y_;
    va = rot_;
    //TODO===参数
    ROS_INFO_STREAM("01===receive V_x "<<V_x<<"V_y "<<V_y<<"va "<<va<<"==============");

//////////
    double radius=  sqrt(pow(wheelBase_x,2)+pow(wheelBase_y,2));
    double alpha = atan(wheelBase_x/wheelBase_y);
//    double radius=0.645;
//    double alpha = 0.5555;
    vector_s wheelVector[4];
    double velocityOfOmega =0;
    double walkingVelocity[4]={0};
    double wheelAngleTemp[4]={0};

    try{
        if(fabs(V_x)<0.001 && fabs(V_y)<0.01 &&fabs(va)<0.01){
            for(int i=0;i<4;i++){
                Vss[i].len=walkingVelocity[i]=0;
//                wheelAngleTemp[i]=0;
            }
        }else {
            velocityOfOmega = fabs(va * radius);// w部分 速度大小
            double wheelVectorAngle[4] = {0, 0, 0, 0};

//            wheelVectorAngle[0] = va < 0 ?  alpha: PI+ alpha;
//            wheelVectorAngle[1] = va < 0 ? -(PI/2+alpha) : (PI - alpha);
//            wheelVectorAngle[2] = va < 0 ? PI+ alpha :  alpha;
//            wheelVectorAngle[3] = va < 0 ? (PI - alpha) :  - alpha;

            wheelVectorAngle[0] = va < 0 ? alpha : PI+ alpha;
            wheelVectorAngle[1] = va < 0 ? - alpha : (PI - alpha);
            wheelVectorAngle[2] = va < 0 ? PI+ alpha :  alpha;
            wheelVectorAngle[3] = va < 0 ? (PI - alpha) :  - alpha;

            for (int i = 0; i < 4; i++) {
                wheelVector[i].x = V_x + velocityOfOmega * cos(wheelVectorAngle[i]);
                wheelVector[i].y = V_y + velocityOfOmega * sin(wheelVectorAngle[i]);
                walkingVelocity[i] = sqrt(pow(wheelVector[i].x, 2) + pow(wheelVector[i].y, 2));
                if (walkingVelocity[i] > 0.0001) {

                    wheelAngleTemp[i] = acos(wheelVector[i].y / walkingVelocity[i]);//与y夹角
                    if (wheelVector[i].x > 0) {
                        wheelAngleTemp[i] = -wheelAngleTemp[i];
                    }
                    Vss[i].len = walkingVelocity[i];
                    Vss[i].angle = wheelAngleTemp[i];

                }
// ------------------test------------------
//                    Vss[i].angle=va*180/3.1415926;
//                    Vss[i].angle=PI/4;
//                Vss[i].len=0.8;
//                Vss[0].angle = 3.1415926/6;
//                Vss[3].angle = -4*3.1415926/6;
// ------------------test------------------
                ROS_INFO_STREAM("cal:: "<<"Vss["<<i<<"].len "<<Vss[i].len<<"..."<<Vss[i].angle
                <<"radius "<<radius <<"    alpha "<<alpha);
            }
        }






/***********************************
   //    vec_s Vc;//和线速度
   //    vec_s Wc[4]={0}; //每个轮子的角速度部分（wr，还是线速度）
   //
   //    double  inclination[4]={0};//两向量间的夹角
   //    ROS_INFO_STREAM("01===receive V_x "<<V_x<<"V_y "<<V_y<<"va "<<va);
   //
   //    try {
   //        Vc.len= sqrt(pow(V_x,2)+pow(V_y,2));
   //        if(V_x){
   //            Vc.angle = atan(V_y/V_x);
   //        }
   //        else{
   //    //        Vc.angle=acos(−1)/2; //反三角 PI
   //            Vc.angle=0; //反三角 PI
   //        }



           ROS_INFO_STREAM("02===  Vc.len "<<Vc.len<<"Vc.angle"<<Vc.angle);

           Wc[LEFT_FRONT].len=Wc[RIGHT_FRONT].len=Wc[LEFT_BACK].len=Wc[RIGHT_BACK].len= va*RR;
           if(va){
               Wc[RIGHT_FRONT].angle=atan(length/with)+PI/2;
               Wc[LEFT_FRONT].angle=-(Wc[RIGHT_FRONT].angle);
               Wc[LEFT_BACK].angle=atan(length/with)-PI/2;
               Wc[RIGHT_BACK].angle=-(Wc[LEFT_BACK].angle);
           }else{
               Wc[RIGHT_FRONT].angle=0;
               Wc[LEFT_FRONT].angle=0;
               Wc[LEFT_BACK].angle=0;
               Wc[RIGHT_BACK].angle=0;
           }
           ROS_INFO_STREAM("03===  Wc[LEFT_FRONT].len "<<Wc[LEFT_FRONT].len<<"Wc[LEFT_FRONT].angle"<<Wc[LEFT_FRONT].angle);
           ROS_INFO_STREAM("03===  Wc[RIGHT_FRONT].len "<<Wc[LEFT_FRONT].len<<"Wc[RIGHT_FRONT].angle"<<Wc[LEFT_FRONT].angle);
           ROS_INFO_STREAM("03===  Wc[LEFT_FRONT].len "<<Wc[LEFT_FRONT].len<<"Wc[LEFT_FRONT].angle"<<Wc[LEFT_FRONT].angle);
           ROS_INFO_STREAM("03===  Wc[RIGHT_FRONT].len "<<Wc[LEFT_FRONT].len<<"Wc[RIGHT_FRONT].angle"<<Wc[LEFT_FRONT].angle);
           if(Vc.len && Wc[RIGHT_FRONT].len){
               Vss[LEFT_FRONT].len=Vss[RIGHT_FRONT].len=Vss[LEFT_BACK].len=Vss[RIGHT_BACK].len=0;
               Wc[LEFT_FRONT].len=Wc[RIGHT_FRONT].len=Wc[LEFT_BACK].len=Wc[RIGHT_BACK].len=0;
           } else {

   //            ///////-----RIGHT_FRONT
   //            //线速度和角速度部分向量夹角
   //            inclination[RIGHT_FRONT] = fabs(Wc[RIGHT_FRONT].angle - Vc.angle);
   //            ROS_INFO_STREAM("Vc.len-- " << inclination[RIGHT_FRONT]);
   //            //和向量大小 c=sqrt(a^2+b^2-2abcos(180-theta)) ；theta两向量间夹角。
   //            Vss[RIGHT_FRONT].len = sqrt(pow(Vc.len, 2) + pow(Wc[RIGHT_FRONT].len, 2) -
   //                                        2 * Vc.len * Wc[RIGHT_FRONT].len * cos(PI - inclination[RIGHT_FRONT]));
   //            if((Vc.len + Wc[RIGHT_FRONT].len * cos(inclination[RIGHT_FRONT]))){
   //            //和向量的方向 phi= arctg(b*sin(theta) /(a+b*cos(theta))); phi 是合向量和向量a 的夹角，然后这个里右前轮再加 速度向量的方向。
   //            Vss[RIGHT_FRONT].angle = atan(Wc[RIGHT_FRONT].len * sin(inclination[RIGHT_FRONT]) /
   //                                          (Vc.len + Wc[RIGHT_FRONT].len * cos(inclination[RIGHT_FRONT]))) + Vc.angle;
   //
   //            }else{
   //                Vss[RIGHT_FRONT].angle=0;
   //            }
   //            ROS_INFO_STREAM("04=== Vss[RIGHT_FRONT].angle " << Vss[RIGHT_FRONT].angle);
   ////            ROS_INFO_STREAM("-Vss[RIGHT_FRONT].len: " << Vss[RIGHT_FRONT].len << "   Vc.angle:" << Vc.angle);
   //            ROS_INFO_STREAM("Vss[RIGHT_FRONT].len: " << Vss[RIGHT_FRONT].len << " Vss[RIGHT_FRONT].angle " << Vss[RIGHT_FRONT].angle);
   //
   //            ///////-----LEFT_FRONT
   //            //线速度和角速度部分向量夹角
   //            inclination[LEFT_FRONT] = fabs(Wc[LEFT_FRONT].angle + Vc.angle);
   //            //和向量大小 c=sqrt(a^2+b^2-2abcos(180-theta)) ；theta两向量间夹角。
   //            Vss[LEFT_FRONT].len = sqrt(pow(Vc.len, 2) + pow(Wc[LEFT_FRONT].len, 2) -
   //                                       2 * Vc.len * Wc[LEFT_FRONT].len * cos(PI - inclination[LEFT_FRONT]));
   //
   //            if((Vc.len +Wc[LEFT_FRONT].len *cos(inclination[LEFT_FRONT]))) {
   //                //和向量的方向 phi= arctg(b*sin(theta) /(a+b*cos(theta))); phi 是合向量和向量a 的夹角，然后这个里右前轮再加 速度向量的方向。
   //                Vss[LEFT_FRONT].angle = atan(Wc[LEFT_FRONT].len * sin(inclination[LEFT_FRONT]) /
   //                                               (Vc.len +Wc[LEFT_FRONT].len *cos(inclination[LEFT_FRONT]))) -Vc.angle;
   //
   //            }
   //            else{
   //                Vss[LEFT_FRONT].angle=0;
   //            }
   //            ROS_INFO_STREAM("05=== Vss[RIGHT_FRONT].angle " << Vss[LEFT_FRONT].angle);
   ////            ROS_INFO_STREAM("Vss[LEFT_FRONT].len: " << Vss[LEFT_FRONT].len << " Vss[LEFT_FRONT].angle " << Vss[LEFT_FRONT].angle);
   //
   //            ///////-----LEFT_BACK
   //            //线速度和角速度部分向量夹角
   //            inclination[LEFT_BACK] = fabs(Wc[LEFT_BACK].angle + Vc.angle);
   //            //和向量大小 c=sqrt(a^2+b^2-2abcos(180-theta)) ；theta两向量间夹角。
   //            Vss[LEFT_BACK].len = sqrt(pow(Vc.len, 2) + pow(Wc[LEFT_BACK].len, 2) -
   //                                      2 * Vc.len * Wc[LEFT_BACK].len * cos(PI - inclination[LEFT_BACK]));
   //            if((Vc.len +Wc[LEFT_BACK].len *cos(inclination[LEFT_BACK]))) {
   //                //和向量的方向 phi= arctg(b*sin(theta) /(a+b*cos(theta))); phi 是合向量和向量a 的夹角，然后这个里右前轮再加 速度向量的方向。
   //                Vss[LEFT_BACK].angle =
   //                        inclination[LEFT_BACK] - atan(Wc[LEFT_BACK].len * sin(inclination[LEFT_BACK]) / (Vc.len +
   //                                                                                                         Wc[LEFT_BACK].len *
   //                                                                                                         cos(inclination[LEFT_BACK]))) -
   //                        Vc.angle;
   //            }
   //            ROS_INFO_STREAM("06=== Vss[LEFT_BACK].angle " << Vss[LEFT_BACK].angle);
   //            ///////-----RIGHT_BACK
   //            //线速度和角速度部分向量夹角
   //            inclination[RIGHT_BACK] = fabs(Wc[RIGHT_BACK].angle - Vc.angle);
   //            //和向量大小 c=sqrt(a^2+b^2-2abcos(180-theta)) ；theta两向量间夹角。
   //            Vss[RIGHT_BACK].len = sqrt(pow(Vc.len, 2) + pow(Wc[RIGHT_BACK].len, 2) -
   //                                       2 * Vc.len * Wc[RIGHT_BACK].len * cos(PI - inclination[RIGHT_BACK]));
   //            if((Vc.len +Wc[RIGHT_BACK].len *cos(inclination[RIGHT_BACK]))) {
   //                //和向量的方向 phi= arctg(b*sin(theta) /(a+b*cos(theta))); phi 是合向量和向量a 的夹角，然后这个里右前轮再加 速度向量的方向。
   //                Vss[RIGHT_BACK].angle = atan(Wc[RIGHT_BACK].len * sin(inclination[RIGHT_BACK]) /
   //                                             (Vc.len + Wc[RIGHT_BACK].len * cos(inclination[RIGHT_BACK]))) + Vc.angle;
   //
   ////            ROS_INFO_STREAM("velocity " << Vss[RIGHT_FRONT].len << "..." << Vss[RIGHT_FRONT].angle);
   //            }
   //            ROS_INFO_STREAM("07=== Vss[RIGHT_BACK].angle " << Vss[RIGHT_BACK].angle);
    **************************************************/





        }


        catch(...){
        ROS_ERROR_STREAM("caculate error");
        return;
    }


//    if(legacy_mode_)
//    {
//    //   wheel_speed_[LEFT_FRONT] = vr + va * wheel_separation_ / 2.0;
//    //   wheel_speed_[RIGHT_FRONT] = vr - va * wheel_separation_ / 2.0;
//        wheel_speed_[RIGHT_FRONT]=wheel_speed_[LEFT_FRONT]=wheel_speed_[RIGHT_BACK]=wheel_speed_[LEFT_BACK]=vr;
//
//    }
//    else
//    {
//    //   wheel_speed_[LEFT_FRONT] = vr - va * wheel_separation_ / 2.0;
//    //   wheel_speed_[RIGHT_FRONT] = vr + va * wheel_separation_ / 2.0;
//    wheel_speed_[RIGHT_FRONT]=wheel_speed_[LEFT_FRONT]=wheel_speed_[RIGHT_BACK]=wheel_speed_[LEFT_BACK]=vr;
//    }
}

void RoadrobotHDrive::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
{
    boost::mutex::scoped_lock scoped_lock ( lock );
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;

}

void RoadrobotHDrive::QueueThread()
{
    static const double timeout = 0.01;

    while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
    }
}

void RoadrobotHDrive::UpdateOdometryEncoder()
{
    double vl = joints_[LEFT_FRONT]->GetVelocity ( 0 );
    double vr = joints_[RIGHT_FRONT]->GetVelocity ( 0 );
#if GAZEBO_MAJOR_VERSION >= 8
    common::Time current_time = parent->GetWorld()->SimTime();
#else
    common::Time current_time = parent->GetWorld()->GetSimTime();
#endif
    double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
    last_odom_update_ = current_time;

    double b = wheel_separation_;

    // Book: Sigwart 2011 Autonompus Mobile Robots page:337
    double sl = vl * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double sr = vr * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
    double ssum = sl + sr;

    double sdiff;
    if(legacy_mode_)
    {
      sdiff = sl - sr;
    }
    else
    {

      sdiff = sr - sl;
    }

    double dx = ( ssum ) /2.0 * cos ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
    double dy = ( ssum ) /2.0 * sin ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
    double dtheta = ( sdiff ) /b;

    pose_encoder_.x += dx;
    pose_encoder_.y += dy;
    pose_encoder_.theta += dtheta;

    double w = dtheta/seconds_since_last_update;
    double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

    tf::Quaternion qt;
    tf::Vector3 vt;
    qt.setRPY ( 0,0,pose_encoder_.theta );
    vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

    odom_.pose.pose.position.x = vt.x();
    odom_.pose.pose.position.y = vt.y();
    odom_.pose.pose.position.z = vt.z();

    odom_.pose.pose.orientation.x = qt.x();
    odom_.pose.pose.orientation.y = qt.y();
    odom_.pose.pose.orientation.z = qt.z();
    odom_.pose.pose.orientation.w = qt.w();

    odom_.twist.twist.angular.z = w;
    odom_.twist.twist.linear.x = dx/seconds_since_last_update;
    odom_.twist.twist.linear.y = dy/seconds_since_last_update;
}

void RoadrobotHDrive::publishOdometry ( double step_time )
{

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
    std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

    tf::Quaternion qt;
    tf::Vector3 vt;

    if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

    }
    if ( odom_source_ == WORLD ) {
        // getting data from gazebo world
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent->WorldPose();
#else
        ignition::math::Pose3d pose = parent->GetWorldPose().Ign();
#endif
        qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
        vt = tf::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        ignition::math::Vector3d linear;
#if GAZEBO_MAJOR_VERSION >= 8
        linear = parent->WorldLinearVel();
        odom_.twist.twist.angular.z = parent->WorldAngularVel().Z();
#else
        linear = parent->GetWorldLinearVel().Ign();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().Ign().Z();
#endif

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.Rot().Yaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();
    }

    if (publishOdomTF_ == true){
        tf::Transform base_footprint_to_odom ( qt, vt );
        transform_broadcaster_->sendTransform (
            tf::StampedTransform ( base_footprint_to_odom, current_time,
                                   odom_frame, base_footprint_frame ) );
    }


    // set covariance
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    // set header
    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    odometry_publisher_.publish ( odom_ );
}

GZ_REGISTER_MODEL_PLUGIN ( RoadrobotHDrive );
}
