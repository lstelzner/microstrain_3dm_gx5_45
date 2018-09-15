/*

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the microstrain_3dm_gx5_45 package.

microstrain_3dm_gx5_45 is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

microstrain_3dm_gx5_45 is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

*/

/**
 * @file    microstrain_3dm_gx5_25.cpp
 * @author  Laura Stelzner
 * @version 0.1
 *
 * @brief ROS Node for Microstrain
 *
 */

#include <ros/ros.h>
#include "3dm_gx5_25.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "microstrain_3dm_gx5_25_node");
  Microstrain::Microstrain_25 ustrain;
  ustrain.setup_node();
  ustrain.run();
  ros::shutdown();
  return 0;
}
