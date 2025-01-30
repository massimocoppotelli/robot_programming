#include "localizer2d.h"

#include "icp/eigen_icp_2d.h"

#include <ros/ros.h>  // Added for ROS logging

Localizer2D::Localizer2D()
    : _map(nullptr),
      _laser_in_world(Eigen::Isometry2f::Identity()),
      _obst_tree_ptr(nullptr) {}

/**
 * @brief Set the internal map reference and constructs the KD-Tree containing
 * obstacles coordinates for fast access.
 *
 * @param map_
 */
void Localizer2D::setMap(std::shared_ptr<Map> map_) {
  // Set the internal map pointer
  _map = map_;
  /**
   * If the map is initialized, fill the _obst_vect vector with world
   * coordinates of all cells representing obstacles.
   * Finally instantiate the KD-Tree (obst_tree_ptr) on the vector.
   */
  // TODO

  if (!_map || !_map->initialized()) {
    ROS_ERROR("Map is not initialized. Cannot build KD-Tree.");
    return;
  }

  ROS_INFO("Initializing KD-Tree from map...");

  // Clear existing obstacle vector
  _obst_vect.clear();

  int rows = _map->rows();
  int cols = _map->cols();

  ROS_INFO("Map size: (%d, %d)", rows, cols);


  // Iterate over all grid cells
  for (int x = 0; x < rows; ++x) {
    for (int y = 0; y < cols; ++y) {
      if ((*_map)(x, y) == CellType::Occupied) {
        // Convert from grid coordinates to world coordinates
        Eigen::Vector2f world_point = _map->grid2world(cv::Point2i(x, y));
        _obst_vect.push_back(world_point);
      }
    }
  }



  // Create KD-Tree
  // TODO

  // Create the KD-Tree
  _obst_tree_ptr = std::make_shared<TreeType>(_obst_vect.begin(), _obst_vect.end());

  if (!_obst_tree_ptr) {
    ROS_ERROR("Failed to construct KD-Tree of obstacles.");
  } else {
    ROS_INFO("KD-Tree successfully created with %zu obstacle points.", _obst_vect.size());
  }

}

/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */
void Localizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {
  // TODO

  _laser_in_world = initial_pose_;

}

/**
 * @brief Process the input scan.
 * First creates a prediction using the current laser_in_world estimate
 *
 * @param scan_
 */
void Localizer2D::process(const ContainerType& scan_) {
  // Use initial pose to get a synthetic scan to compare with scan_
  // TODO

  // Generate a predicted scan based on the current estimate of laser_in_world
  ContainerType predicted_scan;
  getPrediction(predicted_scan);

  /**
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */
  // TODO

  // Configure ICP with the predicted scan and the actual scan
  static const int min_points_leaf = 8;
  ICP icp(predicted_scan, scan_, min_points_leaf);

  // Set the current laser_in_world as the initial guess for ICP
  icp.X() = _laser_in_world;

  // Run ICP with a different number of iterations
  static const int max_iterations = 30;
  icp.run(max_iterations);

  /**
   * Store the solver result (X) as the new laser_in_world estimate
   *
   */
  // TODO

  // Store the updated transformation from ICP
  _laser_in_world = icp.X();

}

/**
 * @brief Set the parameters of the laser scanner. Used to predict
 * measurements.
 * These parameters should be taken from the incoming sensor_msgs::LaserScan
 * message
 *
 * For further documentation, refer to:
 * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
 *
 *
 * @param range_min_
 * @param range_max_
 * @param angle_min_
 * @param angle_max_
 * @param angle_increment_
 */
void Localizer2D::setLaserParams(float range_min_, float range_max_,
                                 float angle_min_, float angle_max_,
                                 float angle_increment_) {
  _range_min = range_min_;
  _range_max = range_max_;
  _angle_min = angle_min_;
  _angle_max = angle_max_;
  _angle_increment = angle_increment_;
}

/**
 * @brief Computes the predicted scan at the current laser_in_world pose
 * estimate.
 *
 * @param dest_ Output predicted scan
 */
void Localizer2D::getPrediction(ContainerType& prediction_) {
  prediction_.clear();
  /**
   * To compute the prediction, query the KD-Tree and search for all points
   * around the current laser_in_world estimate.
   * You may use additional sensor's informations to refine the prediction.
   */
  // TODO

  // Search radius for KD-Tree
  static const float ball_radius = 15;

  // Store neighbors found in the KD-Tree search
  std::vector<PointType*> neighbors;
  _obst_tree_ptr->fullSearch(neighbors, _laser_in_world.translation(), ball_radius);

  // Extract laser position
  const Eigen::Vector2f laser_pos = _laser_in_world.translation();

  // Iterate through found neighbors
  for (const auto& neighbor : neighbors) {
    // Compute displacement vector and distance
    Eigen::Vector2f displacement = laser_pos - *neighbor;
    float distance = displacement.norm();

    // Compute angle using atan2
    float angle = std::atan2(displacement.y(), displacement.x());

    // Ensure the point falls within the laser's valid range and angle
    if (distance >= _range_min && distance <= _range_max &&
        angle >= _angle_min && angle <= _angle_max) {
      
      prediction_.push_back(*neighbor);
    }
  }

  // Log the number of matched points
  ROS_INFO("Prediction computed: %lu points found within range [%.2f, %.2f] meters.",
           prediction_.size(), _range_min, _range_max);

}