#pragma once

#include <vector>

#include "utils.h"

using namespace std;

class PIDController {
public:
    ~PIDController() = default;

    PIDController() = default;

    void set_params(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0, double i_term_max = 100.0) {
        Kp_ = Kp;
        Ki_ = Ki;
        Kd_ = Kd;
        i_term_max_ = i_term_max;
    }

    void set_target(double q_target, double dq_target) {
        q_target_ = q_target;
        dq_target_ = dq_target;
    }

    double get_control(double cur_q, double cur_dq) {
        const auto q_err = q_target_ - cur_q;
        const auto dq_err = dq_target_ - cur_dq;

        const auto p_term = Kp_ * q_err;
        const auto d_term = Kd_ * dq_err;

        i_term_ += q_err;
        const auto i_term = min(max(Ki_ * i_term_, -1.0f * i_term_max_), i_term_max_);

        return p_term + i_term + d_term;
    }

private:
    double Kp_ = 0.0;
    double Ki_ = 0.0;
    double Kd_ = 0.0;

    double q_target_ = 0.0;
    double dq_target_ = 0.0;

    double i_term_max_ = 100.0;
    double i_term_ = 0.0;
};

class Controller {
public:
    ~Controller() = default;

    double pitch_target;
    double roll_target;

    Controller(vector<double> pid_params) {
        // Pitch controller
        Kp_pitch_ = pid_params[0];
        Ki_pitch_ = pid_params[1];
        Kd_pitch_ = pid_params[2];

        // Roll controller
        Kp_roll_ = pid_params[3];
        Ki_roll_ = pid_params[4];
        Kd_roll_ = pid_params[5];

        // Yaw controller
        Kp_yaw_ = pid_params[6];
        Ki_yaw_ = pid_params[7];
        Kd_yaw_ = pid_params[8];

        // Pitch target controller (based on Y position)
        Kp_y_ = pid_params[9];
        Ki_y_ = pid_params[10];
        Kd_y_ = pid_params[11];

        // Roll target controller (based on X position)
        Kp_x_ = pid_params[12];
        Ki_x_ = pid_params[13];
        Kd_x_ = pid_params[14];

        // Thrust controller (based on Z position)
        Kp_z_ = pid_params[15];
        Ki_z_ = pid_params[16];
        Kd_z_ = pid_params[17];

        // Initialize controllers
        pitch_controller.set_params(Kp_pitch_, Ki_pitch_, Kd_pitch_, 0.2 * (PWM_MAX - 1000));
        roll_controller.set_params(Kp_roll_, Ki_roll_, Kd_roll_, 0.2 * (PWM_MAX - 1000));
        yaw_controller.set_params(Kp_yaw_, Ki_yaw_, Kd_yaw_, 0.2 * (PWM_MAX - 1000));

        pitch_target_controller.set_params(Kp_y_, Ki_y_, Kd_y_, 10.0);
        roll_target_controller.set_params(Kp_x_, Ki_x_, Kd_x_, 10.0);
        thrust_controller.set_params(Kp_z_, Ki_z_, Kd_z_, 300.0);
    }

    void set_target(vector<double> joy_euler_target, vector<double> pos_target,
                    vector<double> cur_pos, vector<double> prev_pos, double dt) {
        // Command from joystick
        joy_pitch_target = joy_euler_target[0];
        joy_roll_target = joy_euler_target[1];
        yaw_target = joy_euler_target[2];

        // Target position
        x_target = pos_target[0];
        y_target = pos_target[1];
        z_target = pos_target[2];

        // Adjust desired pitch and roll based on target position
        pitch_target_controller.set_target(y_target, 0.0); // adjust pitch based on Y position
        roll_target_controller.set_target(x_target, 0.0); // adjust roll based on X position

        // Mixed autonomy
        pitch_target = 0.5 * pitch_target_controller.get_control(cur_pos[1], (cur_pos[1] - prev_pos[1]) / dt)
                       + 0.5 * joy_pitch_target;
        roll_target = 0.5 * roll_target_controller.get_control(cur_pos[0], (cur_pos[0] - prev_pos[0]) / dt)
                      + 0.5 * joy_roll_target;

        // Set target for Euler PID controllers
        pitch_controller.set_target(pitch_target, 0.0);
        roll_controller.set_target(roll_target, 0.0);
        yaw_controller.set_target(yaw_target, 0.0);

        // Set target for thrust PID controller
        thrust_controller.set_target(z_target, 0.0); // adjust thrust based on Z position
    }

    vector<double> get_pwm(vector<double> cur_euler, vector<double> cur_ang_vel,
                           double cur_z, double cur_z_vel, double joy_thrust) {
        const auto pitch_pwm = pitch_controller.get_control(cur_euler[0], cur_ang_vel[0]);
        const auto roll_pwm = roll_controller.get_control(cur_euler[1], cur_ang_vel[1]);
        const auto yaw_pwm = yaw_controller.get_control(cur_euler[2], cur_ang_vel[2]);
        const auto thrust = thrust_controller.get_control(cur_z, cur_z_vel);

        const auto all_motor_power = NEU_PWM + 0.5 * joy_thrust + 0.5 * thrust;
        const auto front_motor_power = pitch_pwm;
        const auto back_motor_power = -pitch_pwm;
        const auto left_motor_power = roll_pwm;
        const auto right_motor_power = -roll_pwm;
        const auto ccw_motor_power = yaw_pwm;
        const auto cw_motor_power = -yaw_pwm;

        motors_pwm.clear();
        motors_pwm.push_back(all_motor_power + back_motor_power + right_motor_power + cw_motor_power);
        motors_pwm.push_back(all_motor_power + front_motor_power + right_motor_power + ccw_motor_power);
        motors_pwm.push_back(all_motor_power + back_motor_power + left_motor_power + ccw_motor_power);
        motors_pwm.push_back(all_motor_power + front_motor_power + left_motor_power + cw_motor_power);

        return motors_pwm;
    }

private:
    double Kp_pitch_ = 0.0;
    double Ki_pitch_ = 0.0;
    double Kd_pitch_ = 0.0;

    double Kp_roll_ = 0.0;
    double Ki_roll_ = 0.0;
    double Kd_roll_ = 0.0;

    double Kp_yaw_ = 0.0;
    double Ki_yaw_ = 0.0;
    double Kd_yaw_ = 0.0;

    double Kp_y_ = 0.0;
    double Ki_y_ = 0.0;
    double Kd_y_ = 0.0;

    double Kp_x_ = 0.0;
    double Ki_x_ = 0.0;
    double Kd_x_ = 0.0;

    double Kp_z_ = 0.0;
    double Ki_z_ = 0.0;
    double Kd_z_ = 0.0;

    PIDController pitch_controller;
    PIDController roll_controller;
    PIDController yaw_controller;

    PIDController pitch_target_controller;
    PIDController roll_target_controller;
    PIDController thrust_controller;

    double joy_pitch_target;
    double joy_roll_target;
    double joy_yaw_target;
    
    double yaw_target;

    double x_target;
    double y_target;
    double z_target;

    vector<double> motors_pwm;
};
