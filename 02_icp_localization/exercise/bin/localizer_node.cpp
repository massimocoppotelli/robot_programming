#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "constants.h"
#include "icp/eigen_kdtree.h"
#include "localizer2d.h"
#include "map.h"
#include "ros_bridge.h"

// Map callback definition
void callback_map(const nav_msgs::OccupancyGridConstPtr&);
// Initial pose callback definition
void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
// Scan callback definition
void callback_scan(const sensor_msgs::LaserScanConstPtr&);

std::shared_ptr<Map> map_ptr = nullptr;
ros::Publisher pub_scan, pub_odom;

Localizer2D localizer;

int main(int argc, char** argv) {
  // Initialize ROS system
  // TODO 1
  ros::init(argc, argv, "icp_localization");


  // Create a NodeHandle to manage the node.
  // The namespace of the node is set to global
  ros::NodeHandle nh("/");

  // Create shared pointer for the Map object
  // TODO 2
  map_ptr = std::make_shared<Map>();

  //
  /**
   * Subscribe to the topics:
   * /map [nav_msgs::OccupancyGrid]
   * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
   * /base_scan [sensor_msgs::LaserScan]
   * and assign the correct callbacks
   *
   * Advertise the following topic:
   * /odom_out [nav_msgs::Odometry]
   */
  // TODO 3
  ros::Subscriber sub_map =
      nh.subscribe("/map", 1, callback_map); // Subscribe to /map
  ros::Subscriber sub_initialpose =
      nh.subscribe("/initialpose", 1, callback_initialpose); // Subscribe to /initialpose
  ros::Subscriber sub_scan =
      nh.subscribe("/base_scan", 1, callback_scan); // Subscribe to /base_scan

  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_out", 10);

  // Scan advertiser for visualization purposes
  pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_out", 10);

  ROS_INFO("Node started. Waiting for input data");

  // Spin the node
  ros::spin();

  return 0;
}

void callback_map(const nav_msgs::OccupancyGridConstPtr& msg_) {
  // If the internal map is not initialized, load the incoming occupancyGrid and
  // set the localizer map accordingly
  // Remember to load the map only once during the execution of the map.

  // TODO 4

  if (map_ptr && !map_ptr->initialized()) {
    map_ptr->loadOccupancyGrid(msg_);
    localizer.setMap(map_ptr);
    ROS_INFO("Map loaded and set successfully.");
  } else {
    ROS_WARN("Map is already initialized. Ignoring subsequent map messages.");
  }
}

void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_) {
  /**
   * Convert the PoseWithCovarianceStamped message to an Eigen Isometry and
   * inform the localizer.
   * You can check ros_bridge.h for helps :)
   */

  // TODO 5

  // Extract the pose from the message
  geometry_msgs::Pose pose = msg_->pose.pose;

  // Log the received pose
  ROS_INFO("Received InitialPose: Position (x=%.2f, y=%.2f), Orientation Quaternion (w=%.2f, x=%.2f, y=%.2f, z=%.2f)",
           pose.position.x, pose.position.y,
           pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

  // Convert the pose to Eigen::Isometry2f
  Eigen::Isometry2f initial_pose;
  pose2isometry(pose, initial_pose);

  // Log the converted isometry
  ROS_INFO("Converted Initial Pose: Translation (x=%.2f, y=%.2f), Rotation Angle (theta=%.2f)",
           initial_pose.translation().x(),
           initial_pose.translation().y(),
           Eigen::Rotation2Df(initial_pose.linear()).angle());

  // Set the initial pose in the localizer
  localizer.setInitialPose(initial_pose);
  
}

void callback_scan(const sensor_msgs::LaserScanConstPtr& msg_) {
  
  // Log the received LaserScan message details
  ROS_INFO("SCAN RECEIVED: time=%.2f, range_min=%.2f, range_max=%.2f, angle_increment=%.2f, angle_min=%.2f, angle_max=%.2f",
           msg_->header.stamp.toSec(), msg_->range_min, msg_->range_max,
           msg_->angle_increment, msg_->angle_min, msg_->angle_max);
  
  /**
   * Convert the LaserScan message into a Localizer2D::ContainerType
   * [std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>]
   */
  // TODO 6
  
  Localizer2D::ContainerType scan_points;
  scan2eigen(msg_, scan_points);
  

  /**
   * Set the laser parameters and process the incoming scan through the
   * localizer
   */
  // TODO 7

  localizer.setLaserParams(msg_->range_min, msg_->range_max,
                           msg_->angle_min, msg_->angle_max, msg_->angle_increment);
  localizer.process(scan_points);

  /**
   * Send a transform message between FRAME_WORLD and FRAME_LASER.
   * The transform should contain the pose of the laser with respect to the map
   * frame.
   * You can use the isometry2transformStamped function to convert the isometry
   * to the right message type.
   * Look at include/constants.h for frame names
   *
   * The timestamp of the message should be equal to the timestamp of the
   * received message (msg_->header.stamp)
   */
  // TODO 8
  // Get the pose of the laser in the world frame
  Eigen::Isometry2f laser_in_world = localizer.X();
  ROS_INFO("Laser pose in world: Translation (x=%.2f, y=%.2f), Rotation (theta=%.2f)",
           laser_in_world.translation().x(),
           laser_in_world.translation().y(),
           Eigen::Rotation2Df(laser_in_world.linear()).angle());

  // Convert the pose to TransformStamped
  geometry_msgs::TransformStamped transform_stamped;
  isometry2transformStamped(laser_in_world, transform_stamped, FRAME_WORLD, FRAME_LASER, msg_->header.stamp);

  // Broadcast the transform
  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transform_stamped);
  ROS_INFO("Broadcasted transform: %s -> %s", FRAME_WORLD.c_str(), FRAME_LASER.c_str());
  
  /**
   * Send a nav_msgs::Odometry message containing the current laser_in_world
   * transform.
   * You can use transformStamped2odometry to convert the previously computed
   * TransformStamped message to a nav_msgs::Odometry message.
   */
  // TODO 9

  nav_msgs::Odometry odom_msg;
  transformStamped2odometry(transform_stamped, odom_msg);
  pub_odom.publish(odom_msg);
  ROS_INFO("Published odometry: x=%.2f, y=%.2f, theta=%.2f",
           odom_msg.pose.pose.position.x,
           odom_msg.pose.pose.position.y,
           tf2::getYaw(odom_msg.pose.pose.orientation));

  // Sends a copy of msg_ with FRAME_LASER set as frame_id
  // Used to visualize the scan attached to the current laser estimate.
  sensor_msgs::LaserScan out_scan = *msg_;
  out_scan.header.frame_id = FRAME_LASER;
  pub_scan.publish(out_scan);
  ROS_INFO("Published LaserScan with frame_id: %s", FRAME_LASER.c_str());
}