#ifndef __WIT_IMU__H
#define __WIT_IMU__H

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"
#include "math.h"
#include "tf/tf.h"

#define PI 3.1415926

struct Imu_data{
    float roll; 
    float pitch;
    float yaw;

    float wx;
    float wy;
    float wz;

    float ax;
    float ay;
    float az;

    float T;
};
enum packetFinderState
{
    waitingForHead,
    waitingForPayloadSize,
    waitingForPayload,
    waitingForCheckSum
};

class IDHeader{
public:
enum PayloadType 
{
    angle_acc = 0x51, angle_speed = 0x52, angle = 0x53
};
};

Imu_data imu_data;
uint8_t ready_get = 1;
packetFinderState state = waitingForHead;
bool findHeader = false;
uint8_t checksum = 0x0;
uint8_t dataheader = 0x55;
uint8_t payload_size = 10;
double RPY[3] = {0};


struct SAcc
{
    short a[3];
    short T;
};
struct SGyro
{
    short w[3];
    short T;
};
struct SAngle
{
    short Angle[3];
    short T;
};

struct SAcc         stcAcc;
struct SGyro        stcGyro;
struct SAngle       stcAngle;



#endif