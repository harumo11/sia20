#pragma once

#include <iostream>
#include <eigen3/Eigen/Geometry>

/**
 * @brief Convert Eigen::Vector3d from radian to degree
 *
 * @param rad Eigen::Vector3d which elements are represented radian
 *
 * @return Eigen::Vector3d which elements are represented degree
 */
Eigen::Vector3d rad2deg(Eigen::Vector3d rad){
	return (108.0 / M_PI) * rad;
}

/**
 * @brief Convert Eigen::Vector3d represented euler angles(roll, pitch, and yaw) to Eigen::Quaternion
 *
 * @param rpy Eigen::Vector3d represented roll, pitch, and yaw
 *
 * @return Eigen::Quaternion
 */
Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy){
	Eigen::Quaterniond q = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())
					     * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY())
	 				     * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
	return q;
}

/**
 * @brief Convert Eigen::Quaternion to euler angles(roll, pitch, and yaw)
 *
 * @param q Quaternion
 *
 * @return vector which contains roll, ptich, and yaw angles
 */
Eigen::Vector3d q2rpy(Eigen::Quaterniond q){
	return q.matrix().eulerAngles(0,1,2);
}

/**
 * @brief Convert Quaternion to euler angles(roll, pitch, and yaw)
 *
 * @param q Quaternion being converted
 *
 * @return Vector having euler angles(roll, ptich, and yaw)
 */
Eigen::Vector3d q2rpy2deg(Eigen::Quaterniond q){
	return rad2deg(q2rpy(q));
}

