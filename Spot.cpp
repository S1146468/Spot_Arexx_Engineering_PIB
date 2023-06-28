#include "pxt.h"
#include <cstdint>
#include <math.h>

#ifndef MIN_MAX
#define MIN_MAX(input, min_value, max_value) MAX(MIN(input, min_value), max_value)
#define MAX(input, max_value) ((input) < (max_value) ? (input) : (max_value))
#define MIN(input, min_value) ((input) > (min_value) ? (input) : (min_value))
#endif

#ifndef INVERT_ANGLE
#define INVERT_ANGLE(angle, invert) (invert ? INVERT(angle) : angle)
#define INVERT(angle) (180.0f - angle)
#endif

#define P_LF_1 12
#define P_LF_2 13
#define P_LF_3 14
#define P_RF_1 8
#define P_RF_2 9
#define P_RF_3 10
#define P_LR_1 1
#define P_LR_2 11
#define P_LR_3 0
#define P_RR_1 4
#define P_RR_2 7
#define P_RR_3 6
#define P_H_1 15
#define P_RES_1 3
#define P_RES_2 5
#define P_RES_3 2

using namespace pxt;

#if MICROBIT_CODAL
#define BUFFER_TYPE uint8_t*
#else
#define BUFFER_TYPE char*
#endif

enum LegID {
    FrontLeftID = 0x10,
    FrontRightID = 0x20,
    BackLeftID = 0x30,
    BackRightID = 0x40
};

enum LegJointID {
    ShoulderX = 0xA0,
    ShoulderY = 0xB0,
    Knee = 0xC0
};

enum directions {
    forward = 0x01,
    backward = 0x02,
    left = 0x03,
    right = 0x04
};

class Servo
{
private:
    static bool PCA9685_initialized;
    const static int I2C_address = 0x40;	            // i2c address of PCA9685 servo controller
    const static int I2C_Starting_address = 0x06;    // first servo address for start byte low
    const static int Absolute_Min_Servo_Angle = 0;
    const static int Absolute_Max_Servo_Angle = 180;

    bool Servo_initialized = false;
    int _ServoID = 0;

    float _minPosition = this->Absolute_Min_Servo_Angle;
    float _maxPosition = this->Absolute_Max_Servo_Angle;

    float _zeroPointOffset = 0.0f;
    float _currentPosition = 90.0f;

    void writeI2C(uint8_t I2C_register, uint8_t I2C_value){
        char DATA[2] = {};

        DATA[0] = (char)I2C_register;
        DATA[1] = (char)I2C_value;

        if (uBit.i2c.write((uint8_t)this->I2C_address << 1, (BUFFER_TYPE)DATA, sizeof(DATA), false) == 0) {}
    }
public:
    void initialize(int id, float zeroPointOffset, float minPosition, float maxPosition ) {
        if (!this->PCA9685_initialized) {
            // Mode 1 register
            // Set sleep
            this->writeI2C(0x00, 0x10);

            // Prescale register
            // Set to 50 Hz
            this->writeI2C(0xFE, 0x75);

            // Mode 1 register
            // Wake up
            this->writeI2C(0x00, 0x81);

            this->PCA9685_initialized = true;
        }

        if (!this->Servo_initialized) {
            this->_ServoID = id;
            // Servo register
            // Low-byte start - always 0
            this->writeI2C(this->I2C_Starting_address + this->_ServoID * 4 + 0, 0x00);

            // Servo register
            // High-byte start - always 0
            this->writeI2C(this->I2C_Starting_address + this->_ServoID * 4 + 1, 0x00);
            this->Servo_initialized = true;
        }

        this->_zeroPointOffset = zeroPointOffset;
        this->_minPosition = minPosition;
        this->_maxPosition = maxPosition;

        this->moveServo(this->_currentPosition);
    }

    void moveServo(float angle) {
        this->_currentPosition = MIN_MAX(angle, this->_minPosition, this->_maxPosition);

        angle = MIN_MAX((angle + this->_zeroPointOffset), this->_minPosition, this->_maxPosition);

        int stop = (int)((102.0f + (angle) * ((512.0f - 102.0f) / 180.0f)));

        // Servo register
        // Low byte stop
        this->writeI2C(this->I2C_Starting_address + this->_ServoID * 4 + 2, stop & 0xff);

        // Servo register
        // High byte stop
        this->writeI2C(this->I2C_Starting_address + this->_ServoID * 4 + 3, stop >> 8);
    }

    void setMinMax(float min, float max) {
        this->_minPosition = MIN_MAX(min, Absolute_Min_Servo_Angle, Absolute_Max_Servo_Angle);
        this->_maxPosition = MIN_MAX(max, Absolute_Min_Servo_Angle, Absolute_Max_Servo_Angle);
    }

    int get_id() { return this->_ServoID; }
    float get_pos() { return this->_currentPosition; }
    float get_min_pos() { return this->_minPosition; }
    float get_max_pos() { return this->_maxPosition; }
    float get_offset() { return this->_zeroPointOffset; }
};

bool Servo::PCA9685_initialized = false;

class Leg
{
private:
    int _id = 0;

    float _currentX = 0.0f;
    float _currentY = 0.0f;
    float _currentZ = 0.0f;
    float _currentPitch = 0.0f;
    float _currentRoll = 0.0f;
    float _currentYaw = 0.0f;
public:
    Servo _ShoulderX = {};
    Servo _ShoulderY = {};
    Servo _Knee = {};

public:
    Leg(int id) {
        this->_id = id;
    }

    void init_servo(int joint_id, int servo_id, float servo_calibrate, float servo_min_value, float servo_max_value) {
        if (joint_id == LegJointID::ShoulderX)  this->_ShoulderX.initialize(servo_id, servo_calibrate, servo_min_value, servo_max_value);
        if (joint_id == LegJointID::ShoulderY) this->_ShoulderY.initialize(servo_id, servo_calibrate, servo_min_value, servo_max_value);
        if (joint_id == LegJointID::Knee) this->_Knee.initialize(servo_id, servo_calibrate, servo_min_value, servo_max_value);
    }

    void setNewPosition(float* inputArray) {
        this->_currentX = inputArray[0];
        this->_currentY = inputArray[1];
        this->_currentZ = inputArray[2];
        this->_currentPitch = inputArray[3];
        this->_currentRoll = inputArray[4];
        this->_currentYaw = inputArray[5];
    }

    void moveLeg(float* inputArray) {
        this->_ShoulderX.moveServo(inputArray[0]);
        this->_ShoulderY.moveServo(inputArray[1]);
        this->_Knee.moveServo(inputArray[2]);
    }

    void getServoCurrentAngles(float* outputArray) {
        outputArray[0] = this->_ShoulderX.get_pos();
        outputArray[1] = this->_ShoulderY.get_pos();
        outputArray[2] = this->_Knee.get_pos();
    }
};

Leg FrontLeft(LegID::FrontLeftID);
Leg FrontRight(LegID::FrontRightID);
Leg BackLeft(LegID::BackLeftID);
Leg BackRight(LegID::BackRightID);

//%
namespace Spot {
    void Walk_cpp(directions direction, int speed, int distance);
    void Turn_cpp(directions direction, int speed, int angle);
    void SetDataForKinematics(float X, float Y, float Z, float Pitch, float Roll, float Yaw, int leg_ID);
	void DataProcessorKinematics(int speed);
	void InverseKinematicModel(float* outputArray, float* inputArray, int leg_ID);

    /**
     * Initialize Spot
     */
    //%
    void Init_Spot_cpp() {
        FrontLeft.init_servo(LegJointID::ShoulderX, P_LF_3, 9.0f, 55.0f, 125.0f);
        FrontLeft.init_servo(LegJointID::ShoulderY, P_LF_1, -11.0f, 45.0f, 135.0f);
        FrontLeft.init_servo(LegJointID::Knee, P_LF_2, -5.0f, 35.0f, 120.0f);

        FrontRight.init_servo(LegJointID::ShoulderX, P_RF_3, 6.0f, 55.0f, 125.0f);
        FrontRight.init_servo(LegJointID::ShoulderY, P_RF_1, -6.0f, 45.0f, 135.0f);
        FrontRight.init_servo(LegJointID::Knee, P_RF_2, 0.0f, 60.0f, 145.0f);

        BackLeft.init_servo(LegJointID::ShoulderX, P_LR_3, 8.0f, 55.0f, 125.0f);
        BackLeft.init_servo(LegJointID::ShoulderY, P_LR_2, -3.0f, 45.0f, 135.0f);
        BackLeft.init_servo(LegJointID::Knee, P_LR_1, -9.0f, 35.0f, 120.0f);

        BackRight.init_servo(LegJointID::ShoulderX, P_RR_2, -8.0f, 55.0f, 125.0f);
        BackRight.init_servo(LegJointID::ShoulderY, P_RR_3, -11.0f, 45.0f, 135.0f);
        BackRight.init_servo(LegJointID::Knee, P_RR_1, 7.0f, 60.0f, 145.0f);

        sleep_us(1000000);
    }
    

    /**
     * Demo function
     */
    //%
    void Demo_cpp() {
        int speed = 5;
        int length = 5;
        float distance = 20.0f;
        SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
        SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
        SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
        SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
        DataProcessorKinematics(speed);
        for (int i = 0; i < length; i++)
        {
            SetDataForKinematics(0.0f, 0.0f, distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, -distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, -distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, -distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, -distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
        }
        sleep_us(1000000);
        
        for (int i = 0; i < length; i++)
        {
            SetDataForKinematics(0.0f, -distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, -distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, -distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, -distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
        }
        sleep_us(1000000);

        for (int i = 0; i < length; i++)
        {
            SetDataForKinematics(-distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(-distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(-distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(-distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
        }
        sleep_us(1000000);

        for (int i = 0; i < length; i++)
        {
            SetDataForKinematics(0.0f, 0.0f, 0.0f, distance, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, distance, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, distance, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, distance, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, -distance, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, -distance, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, -distance, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, -distance, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
        }
        sleep_us(1000000);

        for (int i = 0; i < length; i++)
        {
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, distance, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, distance, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, distance, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, distance, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, -distance, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, -distance, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, -distance, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, -distance, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
        }
        sleep_us(1000000);

        for (int i = 0; i < length; i++)
        {
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, distance, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, distance, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, distance, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, distance, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -distance, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -distance, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -distance, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -distance, LegID::BackRightID);
            DataProcessorKinematics(speed);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);
        }

        sleep_us(1000000);
        Walk_cpp(directions::forward, 5, 6);
        sleep_us(1000000);
        Walk_cpp(directions::backward, 5, 6);        
        sleep_us(1000000);
        Walk_cpp(directions::left, 5, 6);
        sleep_us(1000000);
        Walk_cpp(directions::right, 5, 6);
        sleep_us(1000000);
        Turn_cpp(directions::left, 5, 90);
        sleep_us(1000000);
        Turn_cpp(directions::right, 5, 90);
        sleep_us(1000000);
        Turn_cpp(directions::left, 5, 180);
        sleep_us(1000000);
        Turn_cpp(directions::right, 5, 360);
        sleep_us(1000000);
        Turn_cpp(directions::left, 5, 180);
        sleep_us(1000000);

        SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
        SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
        SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
        SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
        DataProcessorKinematics(speed);
    }
    

    /**
     * C++ function for walking
     * Speed unit is in %
     * Distance unit is in cm
     */
    //%
    void Walk_cpp(directions direction, int speed, int distance) {
        distance = MIN_MAX(distance, 1, 500);
        speed = MIN_MAX(speed, 1, 10);

        distance *= 10;
        
        int amount_of_steps = 0;

        float remaining_distance = 0.0f;
        float X_movement_distance = 20.0f;
        float Y_movement_distance = 20.0f;
        float Z_movement_distance = 20.0f;
        
        bool flip = false;

        if ((direction == directions::forward) || (direction == directions::backward)) { // Forward and backward
            float _distance = 0.0f;

            if  (Y_movement_distance < 0.0f){
                _distance = Y_movement_distance * -1.0f;
                flip = true;
            } else {
                _distance = Y_movement_distance;
                flip = false;
            }

            if (distance > _distance) {
                amount_of_steps = distance / _distance;
                remaining_distance = distance - (_distance * amount_of_steps);
                Y_movement_distance = _distance + remaining_distance / amount_of_steps;
            } else {
                Y_movement_distance = _distance;
            }
            
            if (flip){
                Y_movement_distance *= -1.0f;
            }
        }
        if ((direction == directions::left) || (direction == directions::right)) { // Left and right
            float _distance = 0.0f;
            
            if  (X_movement_distance < 0.0f){
                _distance = X_movement_distance * -1.0f;
                flip = true;
            } else {
                _distance = X_movement_distance;
                flip = false;
            }

            if (distance > _distance) {
                amount_of_steps = distance / _distance;
                remaining_distance = distance - (_distance * amount_of_steps);
                X_movement_distance = _distance + remaining_distance / amount_of_steps;
            } else {
                X_movement_distance = _distance;
            }
            
            if (flip){
                X_movement_distance *= -1.0f;
            }
        }

        if (direction == directions::forward)  // Forward
        {
            /*---------------------------------Start sequence step forward begin-----------------------------------*/
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);  
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID); 
            DataProcessorKinematics(speed);     // Front left and back right leg move up
            SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg move forward
            SetDataForKinematics(0.0f, Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg move down
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Move forward
            /*----------------------------------Start sequence step forward end------------------------------------*/


            /*----------------------------------Full step forward sequence begin-----------------------------------*/
            for (int i = 0; i < amount_of_steps; i++) {
                SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);   
                SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID); 
                DataProcessorKinematics(speed);     // Front right and back left leg move up
                SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg move forward
                SetDataForKinematics(0.0f, Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(0.0f, Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg move down

                SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);   
                SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID); 
                DataProcessorKinematics(speed);     // Front left and back right leg move up
                SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg move forward
                SetDataForKinematics(0.0f, Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg move down
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(0.0f, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Move forward
            }
            /*-----------------------------------Full step forward sequence end------------------------------------*/


            /*----------------------------------End sequence step forward begin------------------------------------*/
            SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);   
            SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID); 
            DataProcessorKinematics(speed);     // Front right and back left leg move up
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg move forward
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg move down
            /*-----------------------------------End sequence step forward end-------------------------------------*/
        }

        if (direction == directions::backward)  // Backward
        {
            /*---------------------------------Start sequence step backward begin----------------------------------*/
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);  
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID); 
            DataProcessorKinematics(speed);     // Front left and back right leg move up
            SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg move backward
            SetDataForKinematics(0.0f, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg move down
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Move back
            /*----------------------------------Start sequence step backward end-----------------------------------*/


            /*---------------------------------Full step backward sequence begin-----------------------------------*/
            for (int i = 0; i < amount_of_steps; i++) {
                SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);   
                SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID); 
                DataProcessorKinematics(speed);     // Front right and back left leg move up
                SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg move back
                SetDataForKinematics(0.0f, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(0.0f, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg move down

                SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);   
                SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID); 
                DataProcessorKinematics(speed);     // Front left and back right leg move up
                SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, -Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg move back
                SetDataForKinematics(0.0f, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg move down
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(0.0f, Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Move back
            } 
            /*----------------------------------Full step backward sequence end------------------------------------*/


            /*----------------------------------End sequence step backward begin-----------------------------------*/
            SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);   
            SetDataForKinematics(0.0f, Y_movement_distance, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID); 
            DataProcessorKinematics(speed);     // Front right and back left leg move up
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg move back
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg move down
            /*-----------------------------------End sequence step backward end------------------------------------*/
        }

        if (direction == directions::left)  // Left
        {
            /*---------------------------------Start sequence step backward begin----------------------------------*/
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);  
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID); 
            DataProcessorKinematics(speed);     // Front left and back right leg move up
            SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg move left
            SetDataForKinematics(X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg move down
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(-X_movement_distance, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(-X_movement_distance, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Move left
            /*----------------------------------Start sequence step backward end-----------------------------------*/


            /*---------------------------------Full step backward sequence begin-----------------------------------*/
            for (int i = 0; i < amount_of_steps; i++) {
                SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);   
                SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID); 
                DataProcessorKinematics(speed);     // Front right and back left leg move up
                SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg move left
                SetDataForKinematics(X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg move down

                SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);   
                SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID); 
                DataProcessorKinematics(speed);     // Front left and back right leg move up
                SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg move left
                SetDataForKinematics(X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg move down
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(-X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(-X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Move left
            }
            /*----------------------------------Full step backward sequence end------------------------------------*/


            /*----------------------------------End sequence step backward begin-----------------------------------*/
            SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);   
            SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID); 
            DataProcessorKinematics(speed);     // Front right and back left leg move up
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg move left
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg move down
            /*-----------------------------------End sequence step backward end------------------------------------*/
        }


        if (direction == directions::right)  // Right
        {
            /*---------------------------------Start sequence step backward begin----------------------------------*/
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);  
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID); 
            DataProcessorKinematics(speed);     // Front left and back right leg move up
            SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg move right
            SetDataForKinematics(-X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(-X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg move down
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(X_movement_distance, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(X_movement_distance, -Y_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Move left
            /*----------------------------------Start sequence step backward end-----------------------------------*/

        
            /*---------------------------------Full step backward sequence begin-----------------------------------*/
            for (int i = 0; i < amount_of_steps; i++) {
                SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);   
                SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID); 
                DataProcessorKinematics(speed);     // Front right and back left leg move up
                SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg move left
                SetDataForKinematics(-X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(-X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg move down

                SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);   
                SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID); 
                DataProcessorKinematics(speed);     // Front left and back right leg move up
                SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(-X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg move right
                SetDataForKinematics(-X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(-X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg move down
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
                SetDataForKinematics(X_movement_distance, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Move left
            }
            /*----------------------------------Full step backward sequence end------------------------------------*/


            /*----------------------------------End sequence step backward begin-----------------------------------*/
            SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);   
            SetDataForKinematics(X_movement_distance, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID); 
            DataProcessorKinematics(speed);     // Front right and back left leg move up
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg move right
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg move down
            /*-----------------------------------End sequence step backward end------------------------------------*/
        }
    }
    

    /**
     * C++ function for turning
     * Speed unit is in %
     * Angle unit is in degrees
     */
    //%
    void Turn_cpp(directions direction, int speed, int angle) {
        speed = MIN_MAX(speed, 1, 10);
        angle = MIN_MAX(angle, 5, 360);

        int full_turns = 0;

        float remaining_degrees = 0.0f;
        float YAW_movement_angle = 10.0f; // degrees
        float Z_movement_distance = 20.0f;

        if ((direction != directions::left) && (direction != directions::right))
            return;

        if (angle > YAW_movement_angle) {
            full_turns = angle / YAW_movement_angle;
            remaining_degrees = angle - (YAW_movement_angle * full_turns);
            YAW_movement_angle = YAW_movement_angle + remaining_degrees / full_turns;
        } else {
            YAW_movement_angle = angle;
        }

        if (direction == directions::left)      // left
        {
            /*-----------------------------------Start sequence turn left begin------------------------------------*/
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);  
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID); 
            DataProcessorKinematics(speed);     // Front left and back right leg move up
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg turn left
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, YAW_movement_angle, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, YAW_movement_angle, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg move down
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -YAW_movement_angle, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -YAW_movement_angle, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Turn left
            /*------------------------------------Start sequence turn left end-------------------------------------*/


            /*-----------------------------------Full left turn sequence begin-------------------------------------*/
            for (int i = 0; i < (full_turns - 2); i++) {
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::FrontRightID);   
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::BackLeftID); 
                DataProcessorKinematics(speed);     // Front right and back left leg move up
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::FrontRightID);
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg turn left
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, YAW_movement_angle, LegID::FrontRightID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, YAW_movement_angle, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg move down
                
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::FrontLeftID);   
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::BackRightID); 
                DataProcessorKinematics(speed);     // Front left and back right leg move up
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg turn left
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, YAW_movement_angle, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, YAW_movement_angle, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg move down
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -YAW_movement_angle, LegID::FrontRightID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -YAW_movement_angle, LegID::BackLeftID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Turn left
            }
            /*------------------------------------Full left turn sequence end--------------------------------------*/


            /*------------------------------------End sequence turn left begin-------------------------------------*/
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::FrontRightID);   
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::BackLeftID); 
            DataProcessorKinematics(speed);     // Front right and back left leg move up
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg turn left
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg move down
            /*-------------------------------------End sequence turn left end--------------------------------------*/
        }
       

        if (direction == directions::right)     // right
        {
            /*----------------------------------Start sequence turn right begin------------------------------------*/
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);  
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackRightID); 
            DataProcessorKinematics(speed);     // Front left and back right leg move up
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg turn left
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -YAW_movement_angle, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -YAW_movement_angle, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Front left and back right leg move down
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, YAW_movement_angle, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, YAW_movement_angle, LegID::BackLeftID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
            DataProcessorKinematics(speed);     // Turn left
            /*-----------------------------------Start sequence turn right end-------------------------------------*/


            /*-----------------------------------Full right turn sequence begin------------------------------------*/
            for (int i = 0; i < (full_turns -2) ; i++) {
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::FrontRightID);   
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::BackLeftID); 
                DataProcessorKinematics(speed);     // Front right and back left leg move up
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::FrontRightID);
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg turn left
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -YAW_movement_angle, LegID::FrontRightID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -YAW_movement_angle, LegID::BackLeftID);
                DataProcessorKinematics(speed);     // Front right and back left leg move down
                
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::FrontLeftID);   
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::BackRightID); 
                DataProcessorKinematics(speed);     // Front left and back right leg move up
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, -YAW_movement_angle, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg turn left
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -YAW_movement_angle, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -YAW_movement_angle, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Front left and back right leg move down
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontLeftID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, YAW_movement_angle, LegID::FrontRightID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, YAW_movement_angle, LegID::BackLeftID);
                SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackRightID);
                DataProcessorKinematics(speed);     // Turn left
            }
            /*------------------------------------Full right turn sequence end-------------------------------------*/


            /*------------------------------------End sequence turn left begin-------------------------------------*/
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::FrontRightID);   
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, YAW_movement_angle, LegID::BackLeftID); 
            DataProcessorKinematics(speed);     // Front right and back left leg move up
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, -Z_movement_distance, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg turn left
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::FrontRightID);
            SetDataForKinematics(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, LegID::BackLeftID);
            DataProcessorKinematics(speed);     // Front right and back left leg move down
            /*-------------------------------------End sequence turn left end--------------------------------------*/
        }
    }

    bool executeFrontLeft = false;
    bool executeFrontRight = false;
    bool executeBackLeft = false;
    bool executeBackRight = false;

    float DataFrontLeft[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    float DataFrontRight[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    float DataBackLeft[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    float DataBackRight[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

    
    void SetDataForKinematics(float X, float Y, float Z, float Pitch, float Roll, float Yaw, int leg_ID) {
        if (leg_ID == LegID::FrontLeftID) {
            DataFrontLeft[0] = X;
            DataFrontLeft[1] = Y;
            DataFrontLeft[2] = Z;
            DataFrontLeft[3] = Pitch;
            DataFrontLeft[4] = Roll;
            DataFrontLeft[5] = Yaw * -1.0f;
            executeFrontLeft = true;
        }
        if (leg_ID == LegID::FrontRightID) {
            DataFrontRight[0] = X;
            DataFrontRight[1] = Y;
            DataFrontRight[2] = Z;
            DataFrontRight[3] = Pitch;
            DataFrontRight[4] = Roll;
            DataFrontRight[5] = Yaw * -1.0f;
            executeFrontRight = true;
        }
        if (leg_ID == LegID::BackLeftID) {
            DataBackLeft[0] = X;
            DataBackLeft[1] = Y;
            DataBackLeft[2] = Z;
            DataBackLeft[3] = Pitch;
            DataBackLeft[4] = Roll;
            DataBackLeft[5] = Yaw * -1.0f;
            executeBackLeft = true;
        }
        if (leg_ID == LegID::BackRightID) {
            DataBackRight[0] = X;
            DataBackRight[1] = Y;
            DataBackRight[2] = Z;
            DataBackRight[3] = Pitch;
            DataBackRight[4] = Roll;
            DataBackRight[5] = Yaw * -1.0f;
            executeBackRight = true;
        }
    }
    
    void DataProcessorKinematics(int speed) {
        const int filter_steps = 50;
        const int wait_time_us = (10-speed) * 200;

        /*----------------------Varibles and pointers for front left calculations begin------------------------*/
        float* FrontLeft_ptr = DataFrontLeft;
        float _startFrontLeft[3] = {};
        float _targetFrontLeft[3] = {};
        float _currentFrontLeft[3] = {};
        /*-----------------------Varibles and pointers for front left calculations end-------------------------*/


        /*----------------------Varibles and pointers for front right calculations begin-----------------------*/
        float* FrontRight_ptr = DataFrontRight;
        float _startFrontRight[3] = {};
        float _targetFrontRight[3] = {};
        float _currentFrontRight[3] = {};
        /*-----------------------Varibles and pointers for front right calculations end------------------------*/
        

        /*-----------------------Varibles and pointers for back left calculations begin------------------------*/
        float* BackLeft_ptr = DataBackLeft;
        float _startBackLeft[3] = {};
        float _targetBackLeft[3] = {};
        float _currentBackLeft[3] = {};
        /*------------------------Varibles and pointers for back left calculations end-------------------------*/


        /*----------------------Varibles and pointers for back right calculations begin------------------------*/
        float* BackRight_ptr = DataBackRight;
        float _startBackRight[3] = {};
        float _targetBackRight[3] = {};
        float _currentBackRight[3] = {};
        /*-----------------------Varibles and pointers for back right calculations end-------------------------*/


        /*-------------------------Initializations of the varibles and pointers begin--------------------------*/
        if (executeFrontLeft) {
            InverseKinematicModel(_targetFrontLeft, FrontLeft_ptr, LegID::FrontLeftID);     // calculate kinematics
            FrontLeft.getServoCurrentAngles(_startFrontLeft);                             // getting start values from servo's
        }
        if (executeFrontRight) {
            InverseKinematicModel(_targetFrontRight, FrontRight_ptr, LegID::FrontRightID);  // calculate kinematics
            FrontRight.getServoCurrentAngles(_startFrontRight);                           // getting start values from servo's
        }
        if (executeBackLeft) {
            InverseKinematicModel(_targetBackLeft, BackLeft_ptr, LegID::BackLeftID);        // calculate kinematics
            BackLeft.getServoCurrentAngles(_startBackLeft);                               // getting start values from servo's
        }
        if (executeBackRight) {
            InverseKinematicModel(_targetBackRight, BackRight_ptr, LegID::BackRightID);     // calculate kinematics
            BackRight.getServoCurrentAngles(_startBackRight);                             // getting start values from servo's
        }
        /*--------------------------Initializations of the varibles and pointers end---------------------------*/


        /*-------------------------------Filer all values to the servo's begin---------------------------------*/
        for (int i = 0; i < filter_steps; i++) {
            if (executeFrontLeft) {
                _currentFrontLeft[0] = i * (_targetFrontLeft[0] - _startFrontLeft[0]) / (filter_steps - 1) + _startFrontLeft[0];
                _currentFrontLeft[1] = i * (_targetFrontLeft[1] - _startFrontLeft[1]) / (filter_steps - 1) + _startFrontLeft[1];
                _currentFrontLeft[2] = i * (_targetFrontLeft[2] - _startFrontLeft[2]) / (filter_steps - 1) + _startFrontLeft[2];
                FrontLeft.moveLeg(_currentFrontLeft);
            }
            if (executeFrontRight) {
                _currentFrontRight[0] = i * (_targetFrontRight[0] - _startFrontRight[0]) / (filter_steps - 1) + _startFrontRight[0];
                _currentFrontRight[1] = i * (_targetFrontRight[1] - _startFrontRight[1]) / (filter_steps - 1) + _startFrontRight[1];
                _currentFrontRight[2] = i * (_targetFrontRight[2] - _startFrontRight[2]) / (filter_steps - 1) + _startFrontRight[2];
                FrontRight.moveLeg(_currentFrontRight);
            }
            if (executeBackLeft) {
                _currentBackLeft[0] = i * (_targetBackLeft[0] - _startBackLeft[0]) / (filter_steps - 1) + _startBackLeft[0];
                _currentBackLeft[1] = i * (_targetBackLeft[1] - _startBackLeft[1]) / (filter_steps - 1) + _startBackLeft[1];
                _currentBackLeft[2] = i * (_targetBackLeft[2] - _startBackLeft[2]) / (filter_steps - 1) + _startBackLeft[2];
                BackLeft.moveLeg(_currentBackLeft);
            }
            if (executeBackRight) {
                _currentBackRight[0] = i * (_targetBackRight[0] - _startBackRight[0]) / (filter_steps - 1) + _startBackRight[0];
                _currentBackRight[1] = i * (_targetBackRight[1] - _startBackRight[1]) / (filter_steps - 1) + _startBackRight[1];
                _currentBackRight[2] = i * (_targetBackRight[2] - _startBackRight[2]) / (filter_steps - 1) + _startBackRight[2];
                BackRight.moveLeg(_currentBackRight);
            }
            // Wait command
            if (!!wait_time_us) sleep_us(wait_time_us);
        }
        /*--------------------------------Filer all values to the servo's end----------------------------------*/

        if (executeFrontLeft) {
            FrontLeft.setNewPosition(FrontLeft_ptr);                                // Set new position for front left leg
        }
        if (executeFrontRight) {
            FrontRight.setNewPosition(FrontRight_ptr);                              // Set new position for front right leg
        }
        if (executeBackLeft) {
            BackLeft.setNewPosition(BackLeft_ptr);                                  // Set new position for back left leg
        }
        if (executeBackRight) {
            BackRight.setNewPosition(BackRight_ptr);                                // Set new position for back right leg
        }

        /*--------------------------------------Clear input array begin----------------------------------------*/
        for (int i = 0; i < 6; i++) {
            DataFrontLeft[i] = 0.0f;
            DataFrontRight[i] = 0.0f;
            DataBackLeft[i] = 0.0f;
            DataBackRight[i] = 0.0f;
        }
        executeFrontLeft = false;
        executeFrontRight = false;
        executeBackLeft = false;
        executeBackRight = false;
        FrontLeft_ptr = {};
        FrontRight_ptr = {};
        BackLeft_ptr = {};
        BackRight_ptr = {};
        /*---------------------------------------Clear input array end-----------------------------------------*/
    }
        
    void InverseKinematicModel(float* outputArray, float* inputArray, int leg_ID) {
        /*------------------------------------------Constants begin--------------------------------------------*/
        const float _PI = 3.14159265359f;

        const float LENGTH = 137.5f;
        const float WIDTH = 51.0f;
        const float HEIGHT_0 = 100.0f;

        const float LOWERLEGLENGTH = 70.0f;
        const float UPPERLEGLENGTH = 70.0f;
        const float SHOULDERXOFFSET = 38.3f;
        const float SHOULDERZOFFSET = 17.5f;

        const float CALC_BREAK_PROTECTION = 0.000001f;
        /*-------------------------------------------Constants end---------------------------------------------*/


        /*----------------------------------------Input varibles begin-----------------------------------------*/
        float _x = 0.0f;    // XYZ
        float _y = 0.0f;    // XYZ
        float _z = 0.0f;    // XYZ

        float _alpha_1 = 0.0f;  // Pitch
        float _alpha_2 = 0.0f;  // Pitch
        float _alpha_3 = 0.0f;  // Pitch

        float _beta_1 = 0.0f;  // Roll
        float _beta_2 = 0.0f;  // Roll
        float _beta_3 = 0.0f;  // Roll

        float _gamma_1 = 0.0f;  // Yaw
        float _gamma_2 = 0.0f;  // Yaw
        float _gamma_3 = 0.0f;  // Yaw
        /*-----------------------------------------Input varibles end------------------------------------------*/


        /*----------------------------Protection to non defined calculations begin-----------------------------*/
        if (inputArray[0] == 0.0f) _x = CALC_BREAK_PROTECTION; else _x = inputArray[0];
        if (inputArray[1] == 0.0f) _y = CALC_BREAK_PROTECTION; else _y = inputArray[1];
        if (inputArray[2] == 0.0f) _z = CALC_BREAK_PROTECTION; else _z = inputArray[2];
        if (inputArray[3] == 0.0f) _alpha_1 = CALC_BREAK_PROTECTION; else _alpha_1 = inputArray[3];
        if (inputArray[4] == 0.0f) _beta_1 = CALC_BREAK_PROTECTION; else _beta_1 = inputArray[4];
        if (inputArray[5] == 0.0f) _gamma_1 = CALC_BREAK_PROTECTION; else _gamma_1 = inputArray[5];
        /*-----------------------------Protection to non defined calculations end------------------------------*/


        /*--------------------------------Set front and side from led ID begin---------------------------------*/
        bool front = false;
        bool side = false;

        if (leg_ID == LegID::FrontLeftID) { front = true; side = false; }   // Front left
        if (leg_ID == LegID::FrontRightID) { front = true; side = true; }   // Front right
        if (leg_ID == LegID::BackLeftID) { front = false; side = false; }   // Back left
        if (leg_ID == LegID::BackRightID) { front = false; side = true; }   // Back right
        /*---------------------------------Set front and side from led ID end----------------------------------*/


        /*----------------------------------Convert degrees to radians begin-----------------------------------*/
        _alpha_1 *= (71.0f / 4068.0f);  // Pitch
        _beta_1 *= (71.0f / 4068.0f);  // Roll
        _gamma_1 *= (71.0f / 4068.0f);  // Yaw
        /*-----------------------------------Convert degrees to radians end------------------------------------*/


        /*-------------------------------------Set standard height begin---------------------------------------*/
        _z += HEIGHT_0 - SHOULDERZOFFSET;
        /*--------------------------------------Set standard height end----------------------------------------*/


        /*-----------------------------------------Yaw varibles begin------------------------------------------*/
        float YawOffsetY = 0.0f;
        float YawOffsetX = 0.0f;
        float YawDistanceToCenter = 0.0f;
        /*------------------------------------------Yaw varibles end-------------------------------------------*/


        /*---------------------------------------Yaw calculations begin----------------------------------------*/
        YawOffsetX = WIDTH / 2.0f + SHOULDERXOFFSET;
        YawOffsetY = LENGTH / 2.0f;

        if (front) _y = _y + YawOffsetY;   // add Y offsets for the front
        if (!front) _y = _y - YawOffsetY;   // add Y offsets for the front

        if (side) _x = _x + YawOffsetX;    // add X offsets for the side
        if (!side) _x = _x - YawOffsetX;    // add X offsets for the side

        _gamma_2 = atan(_x / _y);

        YawDistanceToCenter = _x / sin(_gamma_2);

        _gamma_3 = _gamma_1 + _gamma_2;

        _x = YawDistanceToCenter * sin(_gamma_3);
        _y = YawDistanceToCenter * cos(_gamma_3);

        if (front) _y = _y - YawOffsetY;   // remove Y offsets for the front
        if (!front) _y = _y + YawOffsetY;   // remove Y offsets for the front

        if (side) _x = _x - YawOffsetX;    // remove X offsets for the side
        if (!side) _x = _x + YawOffsetX;    // remove X offsets for the side
        /*----------------------------------------Yaw calculations end-----------------------------------------*/


        /*----------------------------Protection to non defined calculations begin-----------------------------*/
        if (_x == 0.0f) _x = CALC_BREAK_PROTECTION;
        if (_y == 0.0f) _y = CALC_BREAK_PROTECTION;
        if (_z == 0.0f) _z = CALC_BREAK_PROTECTION;
        /*-----------------------------Protection to non defined calculations end------------------------------*/


        /*----------------------------------------Add X Offsets begin------------------------------------------*/
        if (side) _x = SHOULDERXOFFSET + _x;
        if (!side) _x = SHOULDERXOFFSET - _x;
        /*-----------------------------------------Add X Offsets end-------------------------------------------*/


        /*----------------------------------------Pitch varibles begin-----------------------------------------*/
        float PitchShoulderOffsetY = 0.0f;
        float PitchShoulderOffsetZ = 0.0f;
        float PitchNewShoulderOffsetY = 0.0f;
        float PitchNewShoulderHeight = 0.0f;
        float PitchNewLegLength = 0.0f;
        /*-----------------------------------------Pitch varibles end------------------------------------------*/


        /*--------------------------------------Pitch calculations begin---------------------------------------*/
        if (front) _alpha_1 *= -1.0f;     // front legs
        if (!front) _alpha_1 *= 1.0f;      // back legs

        PitchShoulderOffsetZ = sin(_alpha_1) * (LENGTH / 2.0f);
        PitchShoulderOffsetY = cos(_alpha_1) * (LENGTH / 2.0f);

        PitchNewShoulderHeight = _z + PitchShoulderOffsetZ;
        PitchNewShoulderOffsetY = _y + (LENGTH / 2.0f - PitchShoulderOffsetY);

        if (front) PitchNewShoulderOffsetY *= -1.0f;

        _alpha_2 = atan(PitchNewShoulderOffsetY / PitchNewShoulderHeight);
        PitchNewLegLength = PitchNewShoulderHeight / cos(_alpha_2);

        _alpha_3 = _alpha_1 + _alpha_2;

        if (front) _alpha_3 *= -1.0f;

        _y = PitchNewLegLength * sin(_alpha_3);
        _z = PitchNewLegLength * cos(_alpha_3);
        /*---------------------------------------Pitch calculations end----------------------------------------*/


        /*----------------------------Protection to non defined calculations begin-----------------------------*/
        if (_x == 0.0f) _x = CALC_BREAK_PROTECTION;
        if (_y == 0.0f) _y = CALC_BREAK_PROTECTION;
        if (_z == 0.0f) _z = CALC_BREAK_PROTECTION;
        /*-----------------------------Protection to non defined calculations end------------------------------*/


        /*----------------------------------------Roll varibles begin------------------------------------------*/
        float RollShoulderOffsetX = 0.0f;
        float RollShoulderOffsetZ = 0.0f;
        float RollNewShoulderOffsetX = 0.0f;
        float RollNewShoulderHeight = 0.0f;
        float RollNewLegLength = 0.0f;
        /*-----------------------------------------Roll varibles end-------------------------------------------*/


        /*--------------------------------------Roll calculations begin----------------------------------------*/
        _beta_1 *= -1.0f;

        RollShoulderOffsetZ = sin(_beta_1) * (WIDTH / 2.0f);
        RollShoulderOffsetX = cos(_beta_1) * (WIDTH / 2.0f);

        if (side) RollNewShoulderHeight = _z + RollShoulderOffsetZ;
        if (!side) RollNewShoulderHeight = _z - RollShoulderOffsetZ;

        RollNewShoulderOffsetX = _x + (WIDTH / 2.0f - RollShoulderOffsetX);

        if (side) RollNewShoulderOffsetX *= -1.0f;

        _beta_2 = atan(RollNewShoulderOffsetX / RollNewShoulderHeight);

        RollNewLegLength = RollNewShoulderHeight / cos(_beta_2);

        _beta_3 = _beta_1 + _beta_2;

        if (side) _beta_3 *= -1.0f;

        _x = RollNewLegLength * sin(_beta_3);
        _z = RollNewLegLength * cos(_beta_3);
        /*---------------------------------------Roll calculations end-----------------------------------------*/


        /*----------------------------Protection to non defined calculations begin-----------------------------*/
        if (_x == 0.0f) _x = CALC_BREAK_PROTECTION;
        if (_y == 0.0f) _y = CALC_BREAK_PROTECTION;
        if (_z == 0.0f) _z = CALC_BREAK_PROTECTION;
        /*-----------------------------Protection to non defined calculations end------------------------------*/


        /*-----------------------------------------XYZ varibles begin------------------------------------------*/
        float Knee = 0.0f;
        float ShoulderX = 0.0f;
        float ShoulderY = 0.0f;

        float _delta_1 = 0.0f;
        float _delta_2 = 0.0f;
        float _delta_3 = 0.0f;
        float _delta_4 = 0.0f;

        float _epsilon_1 = 0.0f;
        float _epsilon_2 = 0.0f;
        float _epsilon_3 = 0.0f;

        float _zeta_1 = 0.0f;
        /*------------------------------------------XYZ varibles end-------------------------------------------*/


        /*---------------------------------------XYZ calculations begin----------------------------------------*/
        _delta_1 = atan(_x / _z);

        _delta_2 = _x / sin(_delta_1);

        _delta_3 = asin(SHOULDERXOFFSET / _delta_2);

        _z = SHOULDERXOFFSET / tan(_delta_3);

        _delta_4 = (_PI - _PI / 2.0f - _delta_3) - (_PI - _PI / 2.0f - _delta_1);

        _z += SHOULDERZOFFSET;

        _epsilon_1 = atan(_y / _z);

        _z = sqrt(pow(_z, 2) + pow(_y, 2));

        _zeta_1 = acos((pow(LOWERLEGLENGTH, 2) +
            pow(UPPERLEGLENGTH, 2) -
            pow(_z, 2)) /
            (2.0 * LOWERLEGLENGTH * UPPERLEGLENGTH));

        _epsilon_2 = _zeta_1 / 2.0f;
        _epsilon_3 = _epsilon_2 + _epsilon_1;

        ShoulderX = _delta_4;
        ShoulderY = _epsilon_3;
        Knee = _zeta_1;
        /*----------------------------------------XYZ calculations end-----------------------------------------*/


        /*----------------------------------Convert radians to degrees begin-----------------------------------*/
        Knee *= (4068.0f / 71.0f);
        ShoulderY *= (4068.0f / 71.0f);
        ShoulderX *= (4068.0f / 71.0f);
        /*-----------------------------------Convert radians to degrees end------------------------------------*/


        /*--------------------------------------Add Servo Offsets begin----------------------------------------*/
        Knee += 0.0f;
        ShoulderY += 45.0f;
        ShoulderX += 90.0f;
        /*---------------------------------------Add Servo Offsets end-----------------------------------------*/


        /*------------------------------------------Set sides begin--------------------------------------------*/
        if (front && !side)  // Front Left leg
        {
            ShoulderX = INVERT_ANGLE(ShoulderX, false);
            ShoulderY = INVERT_ANGLE(ShoulderY, true);
            Knee = INVERT_ANGLE(Knee, true);
        }
        if (front && side)   // Front Right leg
        {
            ShoulderX = INVERT_ANGLE(ShoulderX, true);
            ShoulderY = INVERT_ANGLE(ShoulderY, false);
            Knee = INVERT_ANGLE(Knee, false);
        }
        if (!front && !side) // Back Left leg
        {
            ShoulderX = INVERT_ANGLE(ShoulderX, true);
            ShoulderY = INVERT_ANGLE(ShoulderY, true);
            Knee = INVERT_ANGLE(Knee, true);
        }
        if (!front && side)  // Back Right leg
        {
            ShoulderX = INVERT_ANGLE(ShoulderX, false);
            ShoulderY = INVERT_ANGLE(ShoulderY, false);
            Knee = INVERT_ANGLE(Knee, false);
        }
        /*-------------------------------------------Set sides end---------------------------------------------*/


        /*--------------------------------------Output data to leg begin---------------------------------------*/
        outputArray[0] = ShoulderX;
        outputArray[1] = ShoulderY;
        outputArray[2] = Knee;
        /*---------------------------------------Output data to leg end----------------------------------------*/
    }
}