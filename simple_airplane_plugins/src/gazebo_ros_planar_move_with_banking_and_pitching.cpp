/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: Simple model controller that uses a twist message to move an S-UAV in 3D.
 *       (Modified from Gazebo_ROS_Planar plugin by Piyush Khandelwal)
 * Author: Vengatesan Govindaraju
 */

#include "../include/simple_airplane_plugins/gazebo_ros_planar_move_with_banking_and_pitching.h"
#include <chrono>
#include <thread>
#include <string>

using namespace std;

namespace gazebo 
{

  GazeboRosPlanarMoveWithBankingAndPitching1::GazeboRosPlanarMoveWithBankingAndPitching1() {}

  GazeboRosPlanarMoveWithBankingAndPitching1::~GazeboRosPlanarMoveWithBankingAndPitching1() {}

  // Load the controller
  void GazeboRosPlanarMoveWithBankingAndPitching1::Load(physics::ModelPtr parent, 
      sdf::ElementPtr sdf) 
  {

    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("PlanarMovePlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), odometry_topic_.c_str());
    } 
    else 
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else 
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    robot_base_frame_ = "base_footprint";
    if (!sdf->HasElement("robotBaseFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <robotBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_frame_.c_str());
    } 
    else 
    {
      robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    } 

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    } 
    else 
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    } 
 
    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
    last_odom_pose_ = parent_->GetWorldPose();
    x_ = 0;
    y_ = 0;
    z_ = 0;
    rot_ = 0;
    roll_ = 0;
    pitch_ = 0;
    alive_ = true;

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("PlanarMovePlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG("OCPlugin (%s) has started!", 
        robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_.reset(new tf::TransformBroadcaster());

    // subscribe to the odometry topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosPlanarMoveWithBankingAndPitching1::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);
    odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRosPlanarMoveWithBankingAndPitching1::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosPlanarMoveWithBankingAndPitching1::UpdateChild, this));

  }

  // Update the controller
  void GazeboRosPlanarMoveWithBankingAndPitching1::UpdateChild() 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    math::Pose pose = parent_->GetWorldPose();
    float yaw = pose.rot.GetYaw();
    //float pitch = pose.rot.GetPitch();

    math::Vector3 X_axis, Y_axis, Z_axis;
    X_axis = pose.rot.GetXAxis();
    Y_axis = pose.rot.GetYAxis();
    Z_axis = pose.rot.GetZAxis();

    math::Quaternion current_orientation = pose.rot;
    
    // math::Vector3 g_vec = parent_->GetWorld()->GetPhysicsEngine ()->GetGravity();
    // double gravity = abs(g_vec.GetSum());
    double gravity = 1;   //// Simulations scaled to 1/10 th of the real values (HARD CODING)
    roll_ = atan2( -(x_*rot_), gravity );
    pitch_ = -atan2( z_, x_ );

    //roll_ =0;

    // ROS_WARN("Vel X: %f", x_);
    // ROS_WARN("Vel Y: %f", y_);
    // ROS_WARN("Yaw: %f", rot_);
    // ROS_WARN("Roll: %f", roll_);
    float c_yaw = cosf(yaw);
    float s_yaw = sinf(yaw);
    float c_pitch = cosf(pitch_);
    float s_pitch = sinf(pitch_);


    float c_yaw_2 = cosf(yaw/2);
    float s_yaw_2 = sinf(yaw/2);
    float c_roll_2 = cosf(roll_/2);
    float s_roll_2 = sinf(roll_/2);
    float c_pitch_2 = cosf(pitch_/2);
    float s_pitch_2 = sinf(pitch_/2); 

    math::Quaternion q1,q2,q3, resultant, temp;

    q1.w = c_roll_2;
    q1.x = s_roll_2 * c_yaw;
    q1.y = s_roll_2 * s_yaw;
    q1.z = s_roll_2 * 0;

    q2.w = c_pitch_2;
    q2.x = s_pitch_2 * -s_yaw;
    q2.y = s_pitch_2 * c_yaw;
    q2.z = s_pitch_2 * 0;

    q3.w = c_yaw_2;
    q3.x = s_yaw_2 * 0;
    q3.y = s_yaw_2 * 0;
    q3.z = s_yaw_2 * 1;

    double w1,x1,y1,z1,w2,x2,y2,z2;

    w1 = q2.w; 
    x1 = q2.x; 
    y1 = q2.y; 
    z1 = q2.z;
    w2 = q1.w; 
    x2 = q1.x; 
    y2 = q1.y; 
    z2 = q1.z;

    temp.w = (w1*w2 - x1*x2 - y1*y2 - z1*z2);
    temp.x = (w1*x2 + x1*w2 + y1*z2 - z1*y2);
    temp.y = (w1*y2 - x1*z2 + y1*w2 + z1*x2);
    temp.z = (w1*z2 + x1*y2 - y1*x2 + z1*w2);

    w1 = temp.w; 
    x1 = temp.x; 
    y1 = temp.y; 
    z1 = temp.z;
    w2 = q3.w; 
    x2 = q3.x; 
    y2 = q3.y; 
    z2 = q3.z;

    resultant.w = (w1*w2 - x1*x2 - y1*y2 - z1*z2);
    resultant.x = (w1*x2 + x1*w2 + y1*z2 - z1*y2);
    resultant.y = (w1*y2 - x1*z2 + y1*w2 + z1*x2);
    resultant.z = (w1*z2 + x1*y2 - y1*x2 + z1*w2);

    pose.rot = resultant;
    pose.rot.Normalize();

    double diff_x = x_;
    double diff_y = 0;
    double diff_z = 0;

    double quat_w = resultant.w;
    double quat_x = resultant.x;
    double quat_y = resultant.y;
    double quat_z = resultant.z;

    double new_x = (pow(quat_w,2) +pow(quat_x,2) -pow(quat_y,2) -pow(quat_z,2))*diff_x +
                   (2*quat_x*quat_y - 2*quat_w*quat_z)*diff_y +
                   (2*quat_x*quat_z + 2*quat_w*quat_y)* diff_z;

    double new_y = (2*quat_x*quat_y + 2*quat_w*quat_z)*diff_x +
                   (pow(quat_w,2) -pow(quat_x,2) +pow(quat_y,2) -pow(quat_z,2))*diff_y +
                   (2*quat_z*quat_y - 2*quat_w*quat_x)*diff_z;

    double new_z = (2*quat_x*quat_z - 2*quat_w*quat_y)*diff_x +
                   (2*quat_z*quat_y + 2*quat_w*quat_x)*diff_y +
                   (pow(quat_w,2) -pow(quat_x,2) -pow(quat_y,2) +pow(quat_z,2))*diff_z;

    parent_->SetLinearVel(math::Vector3( new_x, new_y, z_));

    parent_->SetAngularVel(math::Vector3(0, 0, rot_));

    parent_->SetWorldPose((const math::Pose)(pose));

    if (odometry_rate_ > 0.0) {
      common::Time current_time = parent_->GetWorld()->GetSimTime();
      double seconds_since_last_update = 
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometry(seconds_since_last_update);
        last_odom_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void GazeboRosPlanarMoveWithBankingAndPitching1::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosPlanarMoveWithBankingAndPitching1::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    z_ = cmd_msg->linear.z;
    rot_ = cmd_msg->angular.z;
  }

  void GazeboRosPlanarMoveWithBankingAndPitching1::QueueThread() 
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosPlanarMoveWithBankingAndPitching1::publishOdometry(double step_time) 
  {

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = 
      tf::resolve(tf_prefix_, robot_base_frame_);

    // getting data for base_footprint to odom transform
    math::Pose pose = this->parent_->GetWorldPose();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_->sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time, odom_frame,
            base_footprint_frame));

    // publish odom topic
    odom_.pose.pose.position.x = pose.pos.x;
    odom_.pose.pose.position.y = pose.pos.y;
    odom_.pose.pose.position.z = pose.pos.z;

    odom_.pose.pose.orientation.x = pose.rot.x;
    odom_.pose.pose.orientation.y = pose.rot.y;
    odom_.pose.pose.orientation.z = pose.rot.z;
    odom_.pose.pose.orientation.w = pose.rot.w;
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;


    // set parameter suav_altitude
    std::string suav_altitude_string = robot_namespace_+"suav_altitude";
    ros::param::set(suav_altitude_string, double(pose.pos.z) );

    // get velocity in /odom frame
    math::Vector3 linear;
    linear.x = (pose.pos.x - last_odom_pose_.pos.x) / step_time;
    linear.y = (pose.pos.y - last_odom_pose_.pos.y) / step_time;
    linear.z = (pose.pos.z - last_odom_pose_.pos.z) / step_time;
    if (rot_ > M_PI / step_time) 
    { 
      // we cannot calculate the angular velocity correctly
      odom_.twist.twist.angular.z = rot_;
    } 
    else 
    {
      float last_yaw = last_odom_pose_.rot.GetYaw();
      float current_yaw = pose.rot.GetYaw();
      while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
      while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
      float angular_diff = current_yaw - last_yaw;
      odom_.twist.twist.angular.z = angular_diff / step_time;
    }

    last_odom_pose_ = pose;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
    odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;
    odom_.twist.twist.linear.z = linear.z;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    odometry_pub_.publish(odom_);
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosPlanarMoveWithBankingAndPitching1)
}