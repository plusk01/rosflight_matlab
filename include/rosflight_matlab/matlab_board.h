/**
 * @file matlab_board.h
 * @brief ROSflight board specialization for MATLAB
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 22 May 2019
 */

#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <chrono>
#include <cstring>

#include <board.h>

namespace rosflight_matlab
{

  class MATLABBoard : public rosflight_firmware::Board
  {
  public:
    MATLABBoard();
    ~MATLABBoard() = default;
    
    /**
     * @brief      Sets the time based on an external clock
     *
     * @param[in]  secs  The seconds
     */
    void setTime(double secs);

    /**
     * @brief      Sets gyro and accel values from external source
     *
     * @param[in]  gyro   The gyro
     * @param[in]  accel  The accel
     */
    void setIMU(const float gyro[3], const float accel[3]);

    //
    // ROSflight overrides
    //

    // setup
    void init_board() override {}
    void board_reset(bool bootloader) override {}

    // clock
    uint32_t clock_millis() override;
    uint64_t clock_micros() override;
    void clock_delay(uint32_t milliseconds) override {}

    // serial
    void serial_init(uint32_t baud_rate, uint32_t dev) override {}
    void serial_write(const uint8_t *src, size_t len) override {}
    uint16_t serial_bytes_available() override { return 0; }
    uint8_t serial_read() override { return 0; }
    void serial_flush() override {}

    // sensors
    void sensors_init() override {}
    uint16_t num_sensor_errors() override { return 0; }

    bool new_imu_data() override;
    bool imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us) override;
    void imu_not_responding_error() override {}

    bool mag_present() override { return false; }
    void mag_read(float mag[3]) override {}
    void mag_update() override {}

    bool baro_present() override { return false; }
    void baro_read(float *pressure, float *temperature) override {}
    void baro_update() override {}

    bool diff_pressure_present() override { return false; }
    void diff_pressure_read(float *diff_pressure, float *temperature) override {}
    void diff_pressure_update() override {}

    bool sonar_present() override { return false; }
    float sonar_read() override { return 0.0f; }
    void sonar_update() override {}

    bool gnss_present() override { return false; }
    void gnss_update() override {}

    // GNSS
    rosflight_firmware::GNSSData gnss_read() override { return {}; };
    bool gnss_has_new_data() override { return false; }
    rosflight_firmware::GNSSRaw gnss_raw_read() override { return {}; }

    // PWM
    // TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
    void pwm_init(uint32_t refresh_rate, uint16_t idle_pwm) override {}
    void pwm_write(uint8_t channel, float value) override {}
    void pwm_disable() override {}

    // RC
    void rc_init(rc_type_t rc_type) override {}
    float rc_read(uint8_t channel) override { return 0.0f; }
    bool rc_lost() override { return false; }

    // non-volatile memory
    void memory_init() override {}
    bool memory_read(void * dest, size_t len) override { return false; }
    bool memory_write(const void * src, size_t len) override { return false; }

    // LEDs
    void led0_on() override {}
    void led0_off() override {}
    void led0_toggle() override {}

    void led1_on() override {}
    void led1_off() override {}
    void led1_toggle() override {}

    // Backup memory
    bool has_backup_data() override { return false; }
    rosflight_firmware::BackupData get_backup_data() override { return {}; }

  private:
    double time_init_ = 0; ///< initial time from external source
    double time_ = 0; ///< time calculated from external source

    float gyro_[3], accel_[3];
    bool has_new_imu_data_ = false;

  };

} // ns rosflight_matlab
