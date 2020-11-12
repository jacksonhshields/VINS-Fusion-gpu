/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

extern int USE_GPS;
extern int USE_COMPASS;
extern int USE_PRESSURE;
extern std::string GPS_TOPIC;
extern std::string PRESSURE_TOPIC;
extern std::string COMPASS_TOPIC;

