#pragma once

#include <curses.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include "utils.h"

using namespace std;



class IMU {
public:
    IMU() = default;

    IMU(int g_scale, int a_scale, int g_dlpf, int a_dlpf) {
        wiringPiSetup();
        imu_ = wiringPiI2CSetup(0x68); // accel / gyro address

        if (imu_ == -1) {
            // Cannot connect to IMU, throw exception (fail constructor)
            throw runtime_error("Cannot connect to I2C device (IMU)");
        }
        // If we make it here, IMU was connected

        printf("connected to i2c device %d\n", imu_);
        printf("imu who am i is %d \n", wiringPiI2CReadReg8(imu_, 0x75));

        uint8_t accel_scale = a_scale; // AFS_2G, AFS_4G, AFS_8G, AFS_16G
        uint8_t gyro_scale = g_scale;  // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS

        // Get min/max values of the scale according to primary value * multiplier
        accel_scale_ = 2.0 * pow(2.0, static_cast<int>(accel_scale));
        gyro_scale_ = 250.0 * pow(2.0, static_cast<int>(gyro_scale));

        // Init imu
        wiringPiI2CWriteReg8(imu_, PWR_MGMT_1, 0x00);
        printf("                    \n\r");
        wiringPiI2CWriteReg8(imu_, PWR_MGMT_1, 0x01);
        wiringPiI2CWriteReg8(imu_, CONFIG, 0x00);
        wiringPiI2CWriteReg8(imu_, CONFIG, g_dlpf);
        wiringPiI2CWriteReg8(imu_, SMPLRT_DIV, 0x00); // 0x04
        int c = wiringPiI2CReadReg8(imu_, GYRO_CONFIG);
        wiringPiI2CWriteReg8(imu_, GYRO_CONFIG, c & ~0xE0);
        wiringPiI2CWriteReg8(imu_, GYRO_CONFIG, c & ~0x18);
        wiringPiI2CWriteReg8(imu_, GYRO_CONFIG, c | gyro_scale << 3);
        c = wiringPiI2CReadReg8(imu_, ACCEL_CONFIG);
        wiringPiI2CWriteReg8(imu_, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
        wiringPiI2CWriteReg8(imu_, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
        wiringPiI2CWriteReg8(imu_, ACCEL_CONFIG, c | accel_scale << 3);
        c = wiringPiI2CReadReg8(imu_, ACCEL_CONFIG2);
        wiringPiI2CWriteReg8(imu_, ACCEL_CONFIG2, c & ~0x0F);
        wiringPiI2CWriteReg8(imu_, ACCEL_CONFIG2, a_dlpf);
        c = wiringPiI2CReadReg8(imu_, ACCEL_CONFIG2);
    }

    ~IMU() = default;

    void calibrate_imu(size_t iterations = 1000) {
        if (calibrated_) {
            calibrated_ = false;
            gyro_x_calibration_ = 0.0;
            gyro_y_calibration_ = 0.0;
            gyro_z_calibration_ = 0.0;
            accel_z_calibration_ = 0.0;
            roll_calibration_ = 0.0;
            pitch_calibration_ = 0.0;
        }

        auto temp_gx = 0.0;
        auto temp_gy = 0.0;
        auto temp_gz = 0.0;
        auto temp_ax = 0.0;
        auto temp_ay = 0.0;
        auto temp_az = 0.0;

        // Sum values over given iterations
        for (size_t i = 0; i < iterations; ++i) {
            const auto imu_values = read_imu();
            temp_gx += imu_values.at(0);
            temp_gy += imu_values.at(1);
            temp_gz += imu_values.at(2);
            temp_ax += imu_values.at(3);
            temp_ay += imu_values.at(4);
            temp_az += imu_values.at(5);
        }

        // Average the negative to subtract.
        gyro_x_calibration_ = (temp_gx / (-1.0 * iterations));
        gyro_y_calibration_ = (temp_gy / (-1.0 * iterations));
        gyro_z_calibration_ = (temp_gz / (-1.0 * iterations));
        acc_x_calibration_ = temp_ax / iterations;
        acc_y_calibration_ = temp_ay / iterations;
        gravity = temp_az / iterations;

        roll_calibration_ = -1.0 * atan2(temp_az, temp_ax);
        pitch_calibration_ = -1.0 * atan2(temp_az, temp_ay);

        calibrated_ = true;
    }

    vector<double> get_roll_pitch() const {
        auto roll = (atan2(imu_values_.at(5), imu_values_.at(3)) + roll_calibration_) * 180.0f / PI;
        auto pitch = (atan2(imu_values_.at(5), imu_values_.at(4)) + pitch_calibration_) * 180.0f / PI;
        return {roll, pitch}; // degree
    }

    pair <vector<double>, vector<double>> update_and_fetch_values(double dt) {
        update_imu_values(dt);
        return pair < vector < double > , vector < double >> {imu_values_, rpy_};
    }

    vector<double> read_imu() const {
        vector<double> imu_values{}; // Gx, Gy, Gz, Ax, Ay, Az

        auto gx = read_raw(g_x_address_); // This one is negative
        imu_values.push_back(gx * (gyro_scale_ / -32768.0) + gyro_x_calibration_);

        auto gy = read_raw(g_y_address_);
        imu_values.push_back(gy * (gyro_scale_ / 32768.0) + gyro_y_calibration_);

        auto gz = read_raw(g_z_address_);
        imu_values.push_back(gz * (gyro_scale_ / 32768.0) + gyro_z_calibration_);

        auto ax = read_raw(a_x_address_);
        imu_values.push_back(ax * (accel_scale_ / 32768.0));

        auto ay = read_raw(a_y_address_);
        imu_values.push_back(ay * (accel_scale_ / 32768.0));

        auto az = read_raw(a_z_address_);
        imu_values.push_back(az * (accel_scale_ / 32768.0));

        return imu_values;
    }

    void imu_safety_check() const {
        if (abs(imu_values_.at(0)) > OVERSPEED_LIMIT ||
            abs(imu_values_.at(1)) > OVERSPEED_LIMIT ||
            abs(imu_values_.at(2)) > OVERSPEED_LIMIT) {
            throw runtime_error("Gyro overspeed detected. Shutting down.\n");
        }

        if (abs(rpy_.at(0)) > EULER_LIMIT ||
            abs(rpy_.at(1)) > EULER_LIMIT) {
            throw runtime_error("Angle overturn detected. Shutting down.\n");
        }
    }

    double gravity = 0.0;

private:
    int imu_;

    // Gx, Gy, Gz, Ax, Ay, Az
    vector<double> last_imu_values_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    vector<double> imu_values_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    vector<double> last_rpy_{0.0, 0.0};
    vector<double> rpy_{0.0, 0.0};

    double alpha_ = 0.02; // complementary filter

    double gyro_scale_;
    double accel_scale_;

    static constexpr int g_x_address_ = 67;
    static constexpr int g_y_address_ = 69;
    static constexpr int g_z_address_ = 71;

    static constexpr int a_x_address_ = 59;
    static constexpr int a_y_address_ = 61;
    static constexpr int a_z_address_ = 63;

    bool calibrated_ = false;
    double gyro_x_calibration_ = 0.0;
    double gyro_y_calibration_ = 0.0;
    double gyro_z_calibration_ = 0.0;
    double accel_z_calibration_ = 0.0;
    double roll_calibration_ = 0.0;
    double pitch_calibration_ = 0.0;
    double acc_x_calibration_ = 0.0;
    double acc_y_calibration_ = 0.0;

    void update_imu_values(double dt) {
        last_imu_values_ = imu_values_;
        imu_values_ = read_imu();
        last_rpy_ = rpy_;
        rpy_ = get_filtered_rpy(dt);
    }

    vector<double> get_filtered_rpy(double dt) const {
        const auto raw_rpy = get_roll_pitch();
        /// \note: The 0 and 1 are NOT flipped on the imu_values_.
        const auto filter_roll = raw_rpy.at(0) * alpha_ +
                                 (1.0f - alpha_) * (imu_values_.at(1) * dt + last_rpy_.at(0));
        const auto filtr_pitch = raw_rpy.at(1) * alpha_ +
                                 (1.0f - alpha_) * (imu_values_.at(0) * dt + last_rpy_.at(1));
        return {filter_roll, filtr_pitch};
    }

    /// \brief Read the raw return of the IMU
    /// \param address - The address to read
    /// \return The raw value
    int read_raw(const int address) const {
        int vh = wiringPiI2CReadReg8(imu_, address);
        int vl = wiringPiI2CReadReg8(imu_, address + 1);
        int vw = (((vh << 8) & 0xff00) | (vl & 0x00ff)) & 0xffff;
        if (vw > 0x8000) {
            vw = vw ^ 0xffff;
            vw = -vw - 1;
        }
        return vw;
    }
};
