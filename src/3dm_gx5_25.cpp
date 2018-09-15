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


Note:Used gx5_45 as base and editted out un-supported portions
*/

#include "3dm_gx5_25.h"
#include <tf2/LinearMath/Transform.h>
#include <algorithm>
#include <string>

namespace Microstrain {
Microstrain_25::Microstrain_25()
    :  // Initialization list
      filter_valid_packet_count_(0),
      ahrs_valid_packet_count_(0),
      filter_timeout_packet_count_(0),
      ahrs_timeout_packet_count_(0),
      filter_checksum_error_packet_count_(0),
      ahrs_checksum_error_packet_count_(0),
      imu_frame_id_("imu_frame"),
      publish_imu_(true),
      device_setup_(false),
      readback_settings_(true),
      save_settings_(true),
      auto_init_(true),
      auto_init_u8_(1),
      readback_headingsource_(0),
      readback_auto_init_(0),
      dynamics_mode_(0),
      readback_dynamics_mode_(0) {
  // pass
}
Microstrain_25::~Microstrain_25() {
  // pass
}
void Microstrain_25::setup_node() {
  // ROS setup
  ros::Time::init();
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  ROS_INFO("setting up microstrain 25");
  // ROS Parameters
  // Comms Parameters
  std::string port;
  int baud, pdyn_mode;
  private_nh.param("port", port, std::string("/dev/ttyACM0"));
  private_nh.param("baudrate", baud, 115200);
  baudrate_ = (u32)baud;
  // Configuration Parameters
  private_nh.param("device_setup", device_setup_, false);
  private_nh.param("readback_settings", readback_settings_, true);
  private_nh.param("save_settings", save_settings_, true);

  private_nh.param("auto_init", auto_init_, true);
  private_nh.param("imu_rate", imu_rate_, 10);
  private_nh.param("dynamics_mode", pdyn_mode, 1);
  dynamics_mode_ = (u8)pdyn_mode;
  if (dynamics_mode_ < 1 || dynamics_mode_ > 3) {
    ROS_WARN("dynamics_mode can't be %#04X, must be 1, 2 or 3.  Setting to 1.",
             dynamics_mode_);
    dynamics_mode_ = 1;
  }
  private_nh.param("declination_source", declination_source, 2);
  if (declination_source < 1 || declination_source > 3) {
    ROS_WARN(
        "declination_source can't be %#04X, must be 1, 2 or 3.  Setting to 2.",
        declination_source);
    declination_source = 2;
  }
  declination_source_u8 = (u8)declination_source;
  // declination_source_command=(u8)declination_source;
  private_nh.param("declination", declination, 0.23);

  private_nh.param("imu_frame_id", imu_frame_id_, std::string("base_link"));

  private_nh.param("publish_imu", publish_imu_, true);

  // ROS publishers and subscribers
  if (publish_imu_)
    imu_pub_ = node.advertise<sensor_msgs::Imu>("imu/data", 100);

  ros::ServiceServer service =
      node.advertiseService("reset_kf", &Microstrain_25::reset_callback, this);

  // Initialize the serial interface to the device
  ROS_INFO("Attempting to open serial port <%s> at <%d> \n", port.c_str(),
           baudrate_);
  if (mip_interface_init(port.c_str(), baudrate_, &device_interface_,
                         DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK) {
    ROS_FATAL("Couldn't open serial port!  Is it plugged in?");
  }

  // Setup device callbacks
  if (mip_interface_add_descriptor_set_callback(
          &device_interface_, MIP_FILTER_DATA_SET, this,
          &filter_packet_callback_wrapper) != MIP_INTERFACE_OK) {
    ROS_FATAL("Can't setup filter callback!");
    return;
  }
  ROS_INFO("Node successfully configured");
}  // namespace Microstrain

void Microstrain_25::setup_device() {
  tf2::Quaternion quat;
  base_device_info_field device_info;
  u8 temp_string[20] = {0};
  u32 bit_result;
  u8 enable = 1;
  u8 data_stream_format_descriptors[10];
  u16 data_stream_format_decimation[10];
  u8 data_stream_format_num_entries = 0;
  u8 readback_data_stream_format_descriptors[10] = {0};
  u16 readback_data_stream_format_decimation[10] = {0};
  u8 readback_data_stream_format_num_entries = 0;
  u16 base_rate = 0;
  u16 device_descriptors[128] = {0};
  u16 device_descriptors_size = 128 * 2;
  s16 i;
  u16 j;
  u8 com_mode = 0;
  u8 readback_com_mode = 0;
  float angles[3] = {0};
  float readback_angles[3] = {0};
  float offset[3] = {0};
  float readback_offset[3] = {0};
  float hard_iron[3] = {0};
  float hard_iron_readback[3] = {0};
  float soft_iron[9] = {0};
  float soft_iron_readback[9] = {0};
  u16 estimation_control = 0, estimation_control_readback = 0;
  u8 heading_source = 0x1;
  float noise[3] = {0};
  float readback_noise[3] = {0};
  float beta[3] = {0};
  float readback_beta[3] = {0};
  mip_low_pass_filter_settings filter_settings;
  float bias_vector[3] = {0};
  u16 duration = 0;
  gx4_imu_diagnostic_device_status_field imu_diagnostic_field;
  gx4_imu_basic_status_field imu_basic_field;
  gx4_45_diagnostic_device_status_field diagnostic_field;
  gx4_45_basic_status_field basic_field;
  mip_filter_external_gps_update_command external_gps_update;
  mip_filter_external_heading_update_command external_heading_update;
  mip_filter_zero_update_command zero_update_control, zero_update_readback;
  mip_filter_external_heading_with_time_command external_heading_with_time;
  mip_complementary_filter_settings comp_filter_command, comp_filter_readback;

  mip_filter_accel_magnitude_error_adaptive_measurement_command
      accel_magnitude_error_command,
      accel_magnitude_error_readback;
  mip_filter_magnetometer_magnitude_error_adaptive_measurement_command
      mag_magnitude_error_command,
      mag_magnitude_error_readback;
  mip_filter_magnetometer_dip_angle_error_adaptive_measurement_command
      mag_dip_angle_error_command,
      mag_dip_angle_error_readback;

  ////////////////////////////////////////
  // Device setup
  float dT = 0.5;  // common sleep time after setup communications
  if (device_setup_) {
    // Put device into standard mode - we never really use "direct mode"
    ROS_INFO("Putting device communications into 'standard mode'");
    device_descriptors_size = 128 * 2;
    com_mode = MIP_SDK_GX4_25_IMU_STANDARD_MODE;
    while (mip_system_com_mode(&device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                               &com_mode) != MIP_INTERFACE_OK) {
    }
    // Verify device mode setting
    ROS_INFO("Verify comm's mode");
    while (mip_system_com_mode(&device_interface_, MIP_FUNCTION_SELECTOR_READ,
                               &com_mode) != MIP_INTERFACE_OK) {
    }
    ROS_INFO("Sleep for a second...");
    ros::Duration(dT).sleep();
    ROS_INFO("Right mode?");
    if (com_mode != MIP_SDK_GX4_25_IMU_STANDARD_MODE) {
      ROS_ERROR("Appears we didn't get into standard mode!");
    }

    // Put into idle mode
    ROS_INFO("Idling Device: Stopping data streams and/or waking from sleep");
    while (mip_base_cmd_idle(&device_interface_) != MIP_INTERFACE_OK) {
    }
    ros::Duration(dT).sleep();

    // IMU Filter Setup
    // Get base rate
    if (publish_imu_) {
      while (mip_3dm_cmd_get_filter_base_rate(&device_interface_, &base_rate) !=
             MIP_INTERFACE_OK) {
      }
      ROS_INFO("FILTER Base Rate => %d Hz", base_rate);
      u8 imu_decimation = (u8)((float)base_rate / (float)imu_rate_);
      ros::Duration(dT).sleep();

      ////////// Filter Message Format
      // Set
      ROS_INFO("Setting Filter stream format");
      // Note: -25 does not support all the filter settings that -45 does
      data_stream_format_descriptors[0] = MIP_FILTER_DATA_ATT_EULER_ANGLES;
      data_stream_format_descriptors[1] = MIP_FILTER_DATA_ATT_QUATERNION;
      data_stream_format_descriptors[2] =
          MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE;
      data_stream_format_descriptors[3] = MIP_FILTER_DATA_LINEAR_ACCELERATION;
      data_stream_format_descriptors[4] = MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER;
      data_stream_format_descriptors[5] = MIP_FILTER_DATA_GYRO_BIAS_UNCERTAINTY;
      data_stream_format_descriptors[6] = MIP_FILTER_DATA_FILTER_STATUS;

      data_stream_format_decimation[0] = imu_decimation;  // 0x32;
      data_stream_format_decimation[1] = imu_decimation;  // 0x32;
      data_stream_format_decimation[2] = imu_decimation;  // 0x32;
      data_stream_format_decimation[3] = imu_decimation;  // 0x32;
      data_stream_format_decimation[4] = imu_decimation;  // 0x32;
      data_stream_format_decimation[5] = imu_decimation;  // 0x32;
      data_stream_format_decimation[6] = imu_decimation;  // 0x32;
      data_stream_format_num_entries = 7;
      while (mip_3dm_cmd_filter_message_format(
                 &device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                 &data_stream_format_num_entries,
                 data_stream_format_descriptors,
                 data_stream_format_decimation) != MIP_INTERFACE_OK) {
      }
      ros::Duration(dT).sleep();
      // Poll to verify
      ROS_INFO("Poll filter data to test stream");
      while (mip_3dm_cmd_poll_filter(
                 &device_interface_, MIP_3DM_POLLING_ENABLE_ACK_NACK,
                 data_stream_format_num_entries,
                 data_stream_format_descriptors) != MIP_INTERFACE_OK) {
      }
      ros::Duration(dT).sleep();
      // Save
      if (save_settings_) {
        ROS_INFO("Saving Filter data settings");
        while (mip_3dm_cmd_filter_message_format(
                   &device_interface_, MIP_FUNCTION_SELECTOR_STORE_EEPROM, 0,
                   NULL, NULL) != MIP_INTERFACE_OK) {
        }
        ros::Duration(dT).sleep();
      }

      // Heading Source
      ROS_INFO("Set heading source to internal mag.");
      heading_source = 0x1;
      ROS_INFO("Setting heading source to %#04X", heading_source);
      while (mip_filter_heading_source(&device_interface_,
                                       MIP_FUNCTION_SELECTOR_WRITE,
                                       &heading_source) != MIP_INTERFACE_OK) {
      }
      ros::Duration(dT).sleep();

      ROS_INFO("Read back heading source...");
      while (mip_filter_heading_source(
                 &device_interface_, MIP_FUNCTION_SELECTOR_READ,
                 &readback_headingsource_) != MIP_INTERFACE_OK) {
      }
      ROS_INFO("Heading source = %#04X", readback_headingsource_);
      ros::Duration(dT).sleep();

      if (save_settings_) {
        ROS_INFO("Saving heading source to EEPROM");
        while (mip_filter_heading_source(&device_interface_,
                                         MIP_FUNCTION_SELECTOR_STORE_EEPROM,
                                         NULL) != MIP_INTERFACE_OK) {
        }
        ros::Duration(dT).sleep();
      }
    }  // end of Filter setup

    // I believe the auto-init pertains to the kalman filter for the -45
    // OR for the complementary filter for the -25  - need to test
    // Auto Initialization
    // Set auto-initialization based on ROS parameter
    ROS_INFO("Setting auto-initinitalization to: %#04X", auto_init_);
    auto_init_u8_ = auto_init_;  // convert bool to u8
    while (mip_filter_auto_initialization(&device_interface_,
                                          MIP_FUNCTION_SELECTOR_WRITE,
                                          &auto_init_u8_) != MIP_INTERFACE_OK) {
    }
    ros::Duration(dT).sleep();

    if (readback_settings_) {
      // Read the settings back
      ROS_INFO("Reading back auto-initialization value");
      while (mip_filter_auto_initialization(
                 &device_interface_, MIP_FUNCTION_SELECTOR_READ,
                 &readback_auto_init_) != MIP_INTERFACE_OK) {
      }
      ros::Duration(dT).sleep();
      if (auto_init_ == readback_auto_init_)
        ROS_INFO("Success: Auto init. setting is: %#04X", readback_auto_init_);
      else
        ROS_ERROR(
            "Failure: Auto init. setting set to be %#04X, but reads as %#04X",
            auto_init_, readback_auto_init_);
    }
    if (save_settings_) {
      ROS_INFO("Saving auto init. settings to EEPROM");
      while (mip_filter_auto_initialization(&device_interface_,
                                            MIP_FUNCTION_SELECTOR_STORE_EEPROM,
                                            NULL) != MIP_INTERFACE_OK) {
      }
      ros::Duration(dT).sleep();
    }

    // Enable Data streams
    dT = 0.25;
    if (publish_imu_) {
      ROS_INFO("Enabling Filter stream");
      enable = 0x01;
      // TODO verify
      while (mip_3dm_cmd_continuous_data_stream(
                 &device_interface_, MIP_FUNCTION_SELECTOR_WRITE,
                 MIP_3DM_INS_DATASTREAM, &enable) != MIP_INTERFACE_OK) {
      }
      ros::Duration(dT).sleep();
    }

    ROS_INFO("End of device setup - starting streaming");
  } else {
    ROS_INFO("Skipping device setup and listing for existing streams");
  }  // end of device_setup

  // Reset filter - should be for either the KF or CF
  ROS_INFO("Reset filter");
  while (mip_filter_reset_filter(&device_interface_) != MIP_INTERFACE_OK) {
  }
  ros::Duration(dT).sleep();
}
void Microstrain_25::run() {
  setup_device();
  int max_rate = 1;
  if (publish_imu_) {
    max_rate = std::max(max_rate, imu_rate_);
  }
  int spin_rate = std::min(3 * max_rate, 1000);
  ROS_INFO("Setting spin rate to <%d>", spin_rate);
  ros::Rate r(spin_rate);  // Rate in Hz
  while (ros::ok()) {
    // Update the parser (this function reads the port and parses the bytes
    mip_interface_update(&device_interface_);
    ros::spinOnce();  // take care of service requests.
    r.sleep();

  }  // end loop

  // close serial port
  mip_sdk_port_close(device_interface_.port_handle);
}

bool Microstrain_25::reset_callback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &resp) {
  ROS_INFO("Reseting the filter");
  while (mip_filter_reset_filter(&device_interface_) != MIP_INTERFACE_OK) {
  }

  return true;
}

void Microstrain_25::print_packet_stats() {
  ROS_DEBUG_THROTTLE(
      1.0, "%u FILTER (%u errors)  Packets", filter_valid_packet_count_,
      filter_timeout_packet_count_ + filter_checksum_error_packet_count_);
}  // print_packet_stats

void Microstrain_25::filter_packet_callback(void *user_ptr, u8 *packet,
                                            u16 packet_size, u8 callback_type) {
  mip_field_header *field_header;
  u8 *field_data;
  u16 field_offset = 0;

  // If we aren't publishing, then return
  if (!publish_imu_ && false) return;
  // The packet callback can have several types, process them all
  switch (callback_type) {
      ///
      // Handle valid packets
      ///

    case MIP_INTERFACE_CALLBACK_VALID_PACKET: {
      filter_valid_packet_count_++;

      ///
      // Loop through all of the data fields
      ///

      while (mip_get_next_field(packet, &field_header, &field_data,
                                &field_offset) == MIP_OK) {
        ///
        // Decode the field
        ///

        switch (field_header->descriptor) {
            ///
            // Estimated Attitude, Euler Angles
            ///

          case MIP_FILTER_DATA_ATT_EULER_ANGLES: {
            memcpy(&curr_filter_angles_, field_data,
                   sizeof(mip_filter_attitude_euler_angles));

            // For little-endian targets, byteswap the data field
            mip_filter_attitude_euler_angles_byteswap(&curr_filter_angles_);
            imu_msg_.header.frame_id = imu_frame_id_;
            imu_msg_.header.stamp = ros::Time::now();
          } break;

            // Quaternion
          case MIP_FILTER_DATA_ATT_QUATERNION: {
            memcpy(&curr_filter_quaternion_, field_data,
                   sizeof(mip_filter_attitude_quaternion));

            // For little-endian targets, byteswap the data field
            mip_filter_attitude_quaternion_byteswap(&curr_filter_quaternion_);

            // put into ENU - swap X/Y, invert Z
            imu_msg_.orientation.x = curr_filter_quaternion_.q[2];
            imu_msg_.orientation.y = curr_filter_quaternion_.q[1];
            imu_msg_.orientation.z = -1.0 * curr_filter_quaternion_.q[3];
            imu_msg_.orientation.w = curr_filter_quaternion_.q[0];

          } break;

            // Angular Rates
          case MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE: {
            memcpy(&curr_filter_angular_rate_, field_data,
                   sizeof(mip_filter_compensated_angular_rate));

            // For little-endian targets, byteswap the data field
            mip_filter_compensated_angular_rate_byteswap(
                &curr_filter_angular_rate_);

            imu_msg_.angular_velocity.x = curr_filter_angular_rate_.x;
            imu_msg_.angular_velocity.y = curr_filter_angular_rate_.y;
            imu_msg_.angular_velocity.z = curr_filter_angular_rate_.z;
            ROS_INFO("ang");
          } break;

          case MIP_FILTER_DATA_LINEAR_ACCELERATION: {
            memcpy(&curr_filter_accel_, field_data,
                   sizeof(mip_filter_linear_acceleration));

            mip_filter_linear_acceleration_byteswap(&curr_filter_accel_);
            imu_msg_.linear_acceleration.x = curr_filter_accel_.x;
            imu_msg_.linear_acceleration.y = curr_filter_accel_.y;
            imu_msg_.linear_acceleration.z = curr_filter_accel_.z;
          } break;

            // Attitude Uncertainty
          case MIP_FILTER_DATA_ATT_UNCERTAINTY_EULER: {
            memcpy(&curr_filter_att_uncertainty_, field_data,
                   sizeof(mip_filter_euler_attitude_uncertainty));

            // For little-endian targets, byteswap the data field
            mip_filter_euler_attitude_uncertainty_byteswap(
                &curr_filter_att_uncertainty_);
            imu_msg_.orientation_covariance[0] =
                curr_filter_att_uncertainty_.roll *
                curr_filter_att_uncertainty_.roll;
            imu_msg_.orientation_covariance[4] =
                curr_filter_att_uncertainty_.pitch *
                curr_filter_att_uncertainty_.pitch;
            imu_msg_.orientation_covariance[8] =
                curr_filter_att_uncertainty_.yaw *
                curr_filter_att_uncertainty_.yaw;

          } break;

          case MIP_FILTER_DATA_GYRO_BIAS_UNCERTAINTY: {
            memcpy(&curr_filter_gyro_bias_uncertainty_, field_data,
                   sizeof(mip_filter_gyro_bias_uncertainty));

            // For little-endian targets, byteswap the data field
            mip_filter_gyro_bias_uncertainty_byteswap(
                &curr_filter_gyro_bias_uncertainty_);
            imu_msg_.angular_velocity_covariance[0] =
                curr_filter_gyro_bias_uncertainty_.x *
                curr_filter_gyro_bias_uncertainty_.x;
            imu_msg_.angular_velocity_covariance[4] =
                curr_filter_gyro_bias_uncertainty_.y *
                curr_filter_gyro_bias_uncertainty_.y;
            imu_msg_.angular_velocity_covariance[8] =
                curr_filter_gyro_bias_uncertainty_.z *
                curr_filter_gyro_bias_uncertainty_.z;

            // set nominal covariance for accel since none is given
            imu_msg_.linear_acceleration_covariance[0] = 0.1;
            imu_msg_.linear_acceleration_covariance[4] = 0.1;
            imu_msg_.linear_acceleration_covariance[8] = 0.1;
          } break;

            // Filter Status
          case MIP_FILTER_DATA_FILTER_STATUS: {
            memcpy(&curr_filter_status_, field_data, sizeof(mip_filter_status));

            // For little-endian targets, byteswap the data field
            mip_filter_status_byteswap(&curr_filter_status_);

            // imu_status_msg_.data.clear();
            ROS_DEBUG_THROTTLE(
                1.0,
                "Filter Status: %#06X, Dyn. Mode: %#06X, Filter State: %#06X",
                curr_filter_status_.filter_state,
                curr_filter_status_.dynamics_mode,
                curr_filter_status_.status_flags);
            // imu_status_msg_.data.push_back(curr_filter_status_.filter_state);
            // imu_status_msg_.data.push_back(curr_filter_status_.dynamics_mode);
            // imu_status_msg_.data.push_back(curr_filter_status_.status_flags);
            // imu_status_pub_.publish(imu_status_msg_);

          } break;

          default:
            break;
        }
      }

      // Publish
      imu_pub_.publish(imu_msg_);
    } break;

      ///
      // Handle checksum error packets
      ///

    case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR: {
      filter_checksum_error_packet_count_++;
    } break;

      ///
      // Handle timeout packets
      ///

    case MIP_INTERFACE_CALLBACK_TIMEOUT: {
      filter_timeout_packet_count_++;
    } break;
    default:
      break;
  }

  print_packet_stats();
}  // filter_packet_callback

void Microstrain_25::ahrs_packet_callback(void *user_ptr, u8 *packet,
                                          u16 packet_size, u8 callback_type) {
  ROS_WARN("AHRS not implemented for 25");
}

void Microstrain_25::gps_packet_callback(void *user_ptr, u8 *packet,
                                         u16 packet_size, u8 callback_type) {
  ROS_WARN("GPS not implemented for 25");
}

}  // namespace Microstrain
