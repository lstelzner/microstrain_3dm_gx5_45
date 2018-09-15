#ifndef _MICROSTRAIN_DEVICE_H
#define _MICROSTRAIN_DEVICE_H

extern "C" {
#include "byteswap_utilities.h"
#include "mip_gx4_45.h"
#include "mip_gx4_imu.h"
#include "mip_sdk.h"
}

#include <unistd.h>

namespace Microstrain {

class MicrostrainDevice {
 public:
  // MicrostrainDevice();
  //   ~MicrostrainDevice();

  //! Nav estimate callback
  virtual void filter_packet_callback(void *user_ptr, u8 *packet,
                                      u16 packet_size, u8 callback_type) {}
  //! @brief AHRS callback
  virtual void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size,
                                    u8 callback_type) {}
  //! @brief GPS callback
  virtual void gps_packet_callback(void *user_ptr, u8 *packet, u16 packet_size,
                                   u8 callback_type) {}
};

// // Define wrapper functions that call the Microstrain member functions
#ifdef __cplusplus
extern "C" {
#endif

/**
 * Callback for KF estimate packets from sensor.
 */
void filter_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size,
                                    u8 callback_type);
/**
 * Callback for AHRS packets from sensor.
 */
void ahrs_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size,
                                  u8 callback_type);
/**
 * Callback for GPS packets from sensor.
 */
void gps_packet_callback_wrapper(void *user_ptr, u8 *packet, u16 packet_size,
                                 u8 callback_type);
#ifdef __cplusplus
}
#endif

}  // namespace Microstrain

#endif
