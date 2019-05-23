/**
 * @file matlab_board.cpp
 * @brief ROSflight board specialization for MATLAB
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 22 May 2019
 */

#include "rosflight_matlab/matlab_board.h"

namespace rosflight_matlab
{

MATLABBoard::MATLABBoard()
: rosflight_firmware::Board()
{}

// ----------------------------------------------------------------------------

void MATLABBoard::setTime(uint32_t secs, uint64_t nsecs)
{
  double t = secs + nsecs*1e-9;

  // capture "power on" time so that ROSflight clock is secs from start up
  if (time_init_ == 0) time_init_ = t;

  time_ = t - time_init_;
}

// ----------------------------------------------------------------------------

void MATLABBoard::setIMU(const float gyro[3], const float accel[3])
{
  std::memcpy(gyro_, gyro, sizeof(float)*3);
  std::memcpy(accel_, accel, sizeof(float)*3);
  has_new_imu_data_ = true;
}

// ----------------------------------------------------------------------------
// Overrides (required by rosflight firmware)
// ----------------------------------------------------------------------------

uint32_t MATLABBoard::clock_millis()
{
  return static_cast<uint32_t>(time_ * 1e3);
}

// ----------------------------------------------------------------------------

uint64_t MATLABBoard::clock_micros()
{
  return static_cast<uint64_t>(time_ * 1e6);
}

// ----------------------------------------------------------------------------

bool MATLABBoard::new_imu_data()
{
  return has_new_imu_data_;
}

// ----------------------------------------------------------------------------

bool MATLABBoard::imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
{
  has_new_imu_data_ = false;

  *temperature = 27.0f;
  *time_us = clock_micros();
  return true;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

} // ns rosflight_matlab
