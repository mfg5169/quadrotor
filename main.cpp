#include <curses.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <string>
#include <deque>

#include "clock.h"
#include "controller.h"
#include "vive.h"
#include "imu.h"

// g++ -o main main.cpp -lwiringPi -lncurses -lm

using namespace std;

string file_name = "./csv_files/pid_data.csv";
Joy *shared_memory;
Position local_p;
bool program_running = true;
int pwm;

void setup_joy() {
    struct shmid_ds shmbuffer;
    const int shared_segment_size = 0x6400;
    int smhkey = 33222;
    int segment_id = shmget(smhkey, shared_segment_size, IPC_CREAT | 0666);
    shared_memory = (Joy *) shmat(segment_id, 0, 0);
    printf("shared memory attached at address %p\n", shared_memory);
    shmctl(segment_id, IPC_STAT, &shmbuffer);
    int segment_size = shmbuffer.shm_segsz;
    printf("segment size: %d\n", segment_size);
}

void trap(int signal) {
    printf("ending program\n\r");
    program_running = false;
}

void joy_safety_check(Joy joy, double heartbeat_time_diff) {
    if (heartbeat_time_diff > 0.250) {
        throw runtime_error("Joystick heartbeat lost. Shutting down.\n");
    }
    if (joy.key0 == 1) {
        throw runtime_error("Shutdown requested. Shutting down.\n");
    }
}

void read_pid_params(double &kp_pitch, double &ki_pitch, double &kd_pitch,
                     double &kp_roll, double &ki_roll, double &kd_roll,
                     double &kp_yaw, double &ki_yaw, double &kd_yaw,
                     double &kp_y, double &ki_y, double &kd_y,
                     double &kp_x, double &ki_x, double &kd_x,
                     double &kp_z, double &ki_z, double &kd_z) {
    // cout << "Enter Kp_pitch: ";
    // cin >> kp_pitch;
    // cout << "Enter Ki_pitch: ";
    // cin >> ki_pitch;
    // cout << "Enter Kd_pitch: ";
    // cin >> kd_pitch;

    // cout << "Enter Kp_roll: ";
    // cin >> kp_roll;
    // cout << "Enter Ki_roll: ";
    // cin >> ki_roll;
    // cout << "Enter Kd_roll: ";
    // cin >> kd_roll;

    // cout << "Enter Kp_yaw: ";
    // cin >> kp_yaw;
    // cout << "Enter Ki_yaw: ";
    // cin >> ki_yaw;
    // cout << "Enter Kd_yaw: ";
    // cin >> kd_yaw;
    // kp_yaw = -kp_yaw;
    
    kp_pitch = 9.5;
    ki_pitch = 0.15;
    kd_pitch = 0.95;

    kp_roll = 8.3;
    ki_roll = 0.15;
    kd_roll = 0.67;

    kp_yaw = -5;
    ki_yaw = 0;
    kd_yaw = 0.5;

    cout << "Enter Kp_y: ";
    cin >> kp_y;
    cout << "Enter Ki_y: ";
    cin >> ki_y;
    cout << "Enter Kd_y: ";
    cin >> kd_y;
    kp_y = -kp_y;
    ki_y = -ki_y;
    kd_y = -kd_y;

    cout << "Enter Kp_x: ";
    cin >> kp_x;
    cout << "Enter Ki_x: ";
    cin >> ki_x;
    cout << "Enter Kd_x: ";
    cin >> kd_x;
    kp_x = -kp_x;
    ki_x = -ki_x;
    kd_x = -kd_x;

    cout << "Enter Kp_z: ";
    cin >> kp_z;
    cout << "Enter Ki_z: ";
    cin >> ki_z;
    cout << "Enter Kd_z: ";
    cin >> kd_z;
    kp_z = -kp_z;
    ki_z = -ki_z;
    kd_z = -kd_z;

}

void calibrate_vive(double &desired_x, double &desired_y, double &desired_z, double &desired_yaw) {
    desired_x = 0.0;
    desired_y = 0.0;
    for (int i = 0; i < 100; i++) {
        init_shared_memory();
        local_p = *position;
        desired_x += local_p.x;
        desired_y += local_p.y;
    }
    desired_x /= 100;
    desired_y /= 100;
}

void vive_safety_check(Position vive_pos, double &desired_x, double &desired_y, double &desired_z) {
    if (abs(vive_pos.x - desired_x) > 2000.0 || abs(vive_pos.y - desired_y) > 2000.0) {
        throw runtime_error("Vive sensor lost. Shutting down.\n");
    }
}

void vive_time_check(double heartbeat_time_diff) {
    if (heartbeat_time_diff > 0.5) {
        throw runtime_error("Vive sensor heartbeat lost. Shutting down.\n");
    }
}

void init_pwm() {
    pwm = wiringPiI2CSetup(0x40);
    if (pwm == -1) {
        printf("-----cant connect to I2C device %d --------\n", pwm);

    } else {
        double freq = 400.0 * .95;
        double prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        uint8_t prescale = floor(prescaleval + 0.5);
        int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
        int sleep = settings | 0x10;
        int wake = settings & 0xef;
        int restart = wake | 0x80;
        wiringPiI2CWriteReg8(pwm, 0x00, sleep);
        wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
        wiringPiI2CWriteReg8(pwm, 0x00, wake);
        delay(10);
        wiringPiI2CWriteReg8(pwm, 0x00, restart | 0x20);
    }
}

void init_motor(uint8_t channel) {
    int on_value = 0;

    int time_on_us = 900;
    uint16_t off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));

    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel,
                         on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel,
                         on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,
                         off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel,
                         off_value >> 8);
    delay(100);

    time_on_us = 1200;
    off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));

    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel,
                         on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel,
                         on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,
                         off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel,
                         off_value >> 8);
    delay(100);

    time_on_us = 1000;
    off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));

    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel,
                         on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel,
                         on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,
                         off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel,
                         off_value >> 8);
    delay(100);
}

void set_PWM(uint8_t channel, double time_on_us) {
    if (program_running == 1) {
        if (time_on_us > PWM_MAX) {
            time_on_us = PWM_MAX;
        } else if (time_on_us < 1000) {
            time_on_us = 1000;
        }
        uint16_t off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));
        wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,
                              off_value);
    } else {
        time_on_us = 1000;
        uint16_t off_value = round((time_on_us * 4096.f) / (1000000.f / 400.0));
        wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,
                              off_value);
    }
}


int main(int argc, char *argv[]) {
    // Initialize some variables
    vector<double> intg_rp{0.0, 0.0};
    bool pause = false;

    // Get PID parameters
    double kp_pitch, ki_pitch, kd_pitch;
    double kp_roll, ki_roll, kd_roll;
    double kp_yaw, ki_yaw, kd_yaw;
    double kp_y, ki_y, kd_y;
    double kp_x, ki_x, kd_x;
    double kp_z, ki_z, kd_z;
    std::cout << "Enter PID parameters for PID control" << std::endl;
    read_pid_params(kp_pitch, ki_pitch, kd_pitch,
                    kp_roll, ki_roll, kd_roll,
                    kp_yaw, ki_yaw, kd_yaw,
                    kp_y, ki_y, kd_y,
                    kp_x, ki_x, kd_x,
                    kp_z, ki_z, kd_z);
    vector<double> pid_params{kp_pitch, ki_pitch, kd_pitch, // 9.2, 0.15, 0.95,
                              kp_roll, ki_roll, kd_roll, // 8.5, 0.15, 0.7,
                              kp_yaw, ki_yaw, kd_yaw, // 5, 0, 0.5
                              kp_y, ki_y, kd_y, // 0.00202, 0, 0.00355
                              kp_x, ki_x, kd_x, // 0.0023, 0, 0.0041
                              kp_z, ki_z, kd_z};

    // Initialize controller
    Controller controller = Controller(pid_params);

    // Initialize motors
    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);
    delay(1000);

    // Initialize the CSV file
    ofstream my_csv;
    my_csv.open(file_name);
    my_csv << "motor_0_speed, motor_1_speed, motor_2_speed, motor_3_speed, "
              "des_roll, filter_roll, accel_roll, intg_roll, roll_rate, "
              "des_pitch, filter_pitch, accel_pitch, intg_pitch, pitch_rate, "
              "yaw_rate,"
              "z_acc_avg, z_vel_esti, "
              "x_err, y_err, time\n";

    // Initialize IMU
    IMU imu(GFS_500DPS, AFS_2G, GYRO_DLPF::G_HZ_184, ACCEL_DLPF::A_HZ_10);

    // Calibrate
    imu.calibrate_imu();
    cout << "First calibration done!" << endl;

    // Joystick
    setup_joy();
    signal(SIGINT, &trap);

    // Initialize motor speed
    // 0(CW)   F  2(CCW)    y --- X
    //      ---|---               |
    // 1(CCW)  T  3(CW)           x
    // F is the side of battery connector
    double motor_0_speed = 1000;
    double motor_1_speed = 1000;
    double motor_2_speed = 1000;
    double motor_3_speed = 1000;

    // For PID control
    double joy_thrust = 0.0;

    double joy_pitch_target = 0.0;
    double joy_roll_target = 0.0;
    double yaw_target = 0.0;

    double x_target = 0.0;
    double y_target = 0.0;
    double z_target = 6200.0;

    calibrate_vive(x_target, y_target, z_target, yaw_target);

    // Initialize clock
    Clock clock = Clock();
    double last_time = clock.get_time();
    double now_time = clock.get_time();

    // Initialize heartbeat signal
    double last_heartbeat_recv = now_time;
    int last_heartbeat = -1;
    double last_vive_recv = now_time;
    int last_vive_version = -1;

    double vive_dt = 0.0;
    bool got_first_vive_data = false;

    double x_esti = 0.0;
    double y_esti = 0.0;
    double z_esti = 0.0;
    vector<double> cur_pos;
    vector<double> prev_pos;
    double z_vel_esti = 0.0;

    // For Z velocity estimate
    double z_acc_sum = 0.0;
    double z_acc_count = 0.0;
    double z_acc_avg = 0.0;
    const double K = 50.0; // for z velocity estimate
    const double A = 0.9; // for z velocity estimate

    // Get data from vive sensor
    init_shared_memory();

    // Main control loop
    double loop_start_time = clock.get_time();
    
    while (program_running) {
        local_p = *position;
        // Update position and linear acceleration iff new vive data is received
        if (local_p.version != last_vive_version) { // if got new vive data
            if (!got_first_vive_data) {
                got_first_vive_data = true; // first data received

                x_esti = local_p.x; // initialize estimated x position
                y_esti = local_p.y; // initialize estimated y position
                z_esti = local_p.z; // initialize estimated z position

                prev_pos = {x_esti, y_esti, z_esti}; // initialize previous position
                cur_pos = {x_esti, y_esti, z_esti}; // initialize current position
            }
            prev_pos = cur_pos; // update previous position

            x_esti = 0.6 * x_esti + 0.4 * local_p.x; // update estimated x position
            y_esti = 0.6 * y_esti + 0.4 * local_p.y; // update estimated y position
            z_esti = 0.6 * z_esti + 0.4 * local_p.z; // update estimated z position
            cur_pos = {x_esti, y_esti, z_esti}; // update current position

            // Since we got new vive sensor data, we need to reset z acceleration sum and count
            z_acc_sum = 0.0; // reset z acceleration sum
            z_acc_count = 0.0; // reset z acceleration count

            last_vive_version = local_p.version; // update version
            double tmp = last_vive_recv;
            last_vive_recv = clock.get_time(); // update time
            vive_dt = last_vive_recv - tmp; // calculate vive dt
        } else { // if didn't get new vive data, check if sth is wrong
            try {
                vive_time_check(clock.get_time() - last_vive_recv);
            } catch (exception &e) {
                cout << e.what();
                program_running = false;
                break;
            }
        }

        // Get joystick signal from shared memory
        Joy joy_data = *shared_memory;

        // Kill program if [A] is pressed
        if (joy_data.key0) {
            break;
        }

        // Pause if [B] is pressed, unpause if [X] is pressed
        if (joy_data.key1 == 1) {
            pause = true;
        }
        if (joy_data.key2 == 1) {
            pause = false;
        }

        // Adjust joy_thrust
        joy_thrust = -1.2 * (joy_data.thrust - 128) / 1.28; // -120~120
        if (abs(joy_thrust) < 4) joy_thrust = 0.0;

        // Get dt
        now_time = clock.get_time();
        double dt = now_time - last_time;

        // Update heartbeat signal
        if (joy_data.sequence_num != last_heartbeat) {
            last_heartbeat = joy_data.sequence_num;
            last_heartbeat_recv = now_time;
        }

        // Read IMU
        double get_imu_data_time = clock.get_time() - loop_start_time;
        const auto result = imu.update_and_fetch_values(dt);
        const auto imu_values = result.first;        // raw IMU values
        const auto roll_pitch = result.second;       // filtered roll and pitch
        const auto accel_rp = imu.get_roll_pitch();  // roll and pitch from accelerometer
        intg_rp.at(0) += (imu_values.at(1) * dt);    // roll from angular velocity integration
        intg_rp.at(1) += (imu_values.at(0) * dt);    // pitch from angular velocity integration
        z_acc_sum += imu_values.at(5) - imu.gravity; // sum z acceleration, need to subtract gravity
        z_acc_count += 1.0;                          // increment z acceleration count
        z_acc_avg = z_acc_sum / z_acc_count;         // calculate average z acceleration

        vector<double> cur_euler{roll_pitch[1], roll_pitch[0], local_p.yaw * 180.0 / PI}; // pitch, roll, yaw
        vector<double> cur_ang_vel{imu_values[0], imu_values[1], imu_values[2]};          // pitch, roll, yaw rates

        // Safety checks
        try {
            imu.imu_safety_check();
            joy_safety_check(joy_data, now_time - last_heartbeat_recv);
            vive_safety_check(local_p, x_target, y_target, z_target);
        } catch (exception &e) {
            cout << e.what();
            program_running = false;
            break;
        }

        // Calibrate IMU and position if [Y] is pressed
        if (joy_data.key3 == 1) {
            set_PWM(0, 1000);
            set_PWM(1, 1000);
            set_PWM(2, 1000);
            set_PWM(3, 1000);
            imu.calibrate_imu();
            cout << "IMU is calibrated!" << endl;
            calibrate_vive(x_target, y_target, z_target, yaw_target);
            cout << "Vive sensor is calibrated!" << endl;
        }

        // Get desired pitch and roll from joystick
        joy_pitch_target = -0.5 * (joy_data.pitch - 128) / 12.8; // -5~5 degree
        joy_roll_target = -0.5 * (joy_data.roll - 128) / 12.8; // -5~5 degree

        if (abs(joy_pitch_target) < 0.1) joy_pitch_target = 0.0;
        if (abs(joy_roll_target) < 0.1) joy_roll_target = 0.0;

        // Set control target
        vector<double> joy_euler_target{joy_pitch_target, joy_roll_target, yaw_target}; // [pitch, roll, yaw]
        vector<double> pos_target{x_target, y_target, z_target}; // [x, y, z]
        controller.set_target(joy_euler_target, pos_target, cur_pos, prev_pos, vive_dt);

        // Estimate Z velocity
        double vive_z_vel_esti = (cur_pos[2] - prev_pos[2]) / vive_dt;
        z_vel_esti = (z_vel_esti + z_acc_avg * K) * A
                     + vive_z_vel_esti * (1 - A);
        // The z axis of IMU points downward, so the acceleration should times -1

        // Get PWM
        vector<double> motors_pwm = controller.get_pwm(cur_euler, cur_ang_vel, z_esti, z_vel_esti, joy_thrust);

        // Set PWM
        if (pause) {
            set_PWM(0, 1000);
            set_PWM(1, 1000);
            set_PWM(2, 1000);
            set_PWM(3, 1000);
        } else {
            set_PWM(0, motors_pwm[0]);
            set_PWM(1, motors_pwm[1]);
            set_PWM(2, motors_pwm[2]);
            set_PWM(3, motors_pwm[3]);
            // set_PWM(0, 1000);
            // set_PWM(1, 1000);
            // set_PWM(2, 1000);
            // set_PWM(3, 1000);

            // cout << local_p.x << ", " << local_p.y << ", " << local_p.z << ", "
            //      << local_p.yaw << ", desired yaw: " << yaw_target << endl;

            // Write to CSV
            my_csv << motors_pwm[0] << ", " << motors_pwm[1] << ", " << motors_pwm[2] << ", " << motors_pwm[3] << ", "               // motor speeds
                   << controller.roll_target << ", " << roll_pitch.at(0) << ", " << accel_rp.at(0) << ", "  << intg_rp.at(0) << ", " // roll
                   << imu_values.at(1) << ", "                                                                                       // roll rate
                   << controller.pitch_target << ", " << roll_pitch.at(1) << ", " << accel_rp.at(1) << ", " << intg_rp.at(1) << ", " // pitch                   
                   << imu_values.at(0) << ", "                                                                                       // pitch rate
                   << imu_values.at(2) << ", "                                                                                       // yaw rate
                   << z_acc_avg << ", " << z_vel_esti << ", "                                                                        // z acceleration and velocity
                   << cur_pos[0] - pos_target[0] << ", "                                                                             // x error
                   << cur_pos[1] - pos_target[1] << ", "                                                                             // y error
                   << get_imu_data_time << ", "
                   << vive_z_vel_esti << ", " << z_esti << endl;
        }

        // Update clock
        last_time = now_time;
    }

    // Stop motors
    set_PWM(0, 1000);
    set_PWM(1, 1000);
    set_PWM(2, 1000);
    set_PWM(3, 1000);

    return 0;
}
