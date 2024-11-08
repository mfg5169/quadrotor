#pragma once

#define CONFIG 0x1A
#define SMPLRT_DIV 0x19
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG2 0x1D
#define PWR_MGMT_1 0x6B  // device defaults to the SLEEP mode
#define PWM_MAX 1900.0
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define LED_MULTIPLYER 4
#define OVERSPEED_LIMIT 600.0
#define EULER_LIMIT 45.0
#define NEU_PWM 1000.0

enum AScale {
    AFS_2G = 0, AFS_4G, AFS_8G, AFS_16G
};

/// \brief The possible gyro scales
enum GScale {
    GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
};

enum GYRO_DLPF {
    G_HZ_250 = 0x00,
    G_HZ_184 = 0x01,
    G_HZ_92 = 0x02,
    G_HZ_41 = 0x03,
    G_HZ_20 = 0x04,
    G_HZ_10 = 0x05,
    G_HZ_5 = 0x06

};

enum ACCEL_DLPF {
    A_HZ_5 = 0x06,
    A_HZ_10 = 0x05,
    A_HZ_20 = 0x04,
    A_HZ_41 = 0x03,
    A_HZ_92 = 0x02,
    A_HZ_184 = 0x01,
    A_HZ_460 = 0x00
};

enum Motor {
    BACK_RIGHT = 0,
    FRONT_RIGHT = 1,
    BACK_LEFT = 2,
    FRONT_LEFT = 3
};

/// \brief The struct for the keyboard data
struct Keyboard {
    char key_press;
    int heartbeat;
    int version;
};

struct Joy {
    int key0;
    int key1;
    int key2;
    int key3;
    int pitch;
    int roll;
    int yaw;
    int thrust;
    int sequence_num;
};

static constexpr double PI = 3.14159265;
