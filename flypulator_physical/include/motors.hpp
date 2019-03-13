#ifndef __MOTORS_HPP__
#define __MOTORS_HPP__

#include "ros/ros.h"
#include <ft260_driver.h>
#include <algorithm>

class Motors
{

  public:
    // definition for motors' state
    typedef enum
    {
        MOTOR_ARMED = 0,
        MOTOR_DISARMED
    } MOTORS_STATE_T;
    // definition for I2C master write state
    typedef enum
    {
        WRITE_OK = 0,
        WRITE_ERROR
    } WRITE_STATE_T;

  private:
    // number of motors
    static const int N_MOTORS = 6;
    // motors' I2C address
    const uint8_t motor_i2c_addr[N_MOTORS] = {0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e};
    // relationship between throttle and rpm
    // rpm = fractor_k * throttle + fractor_b
    const float fractor_k = 30.922;
    const float fractor_b = 122.22;
    // maximal/minimal rotational velocity(command) of the motors
    const float MAX_MOTOR_VELOCITY = 6000;
    const float MIN_MOTOR_VELOCITY = fractor_b + 1;
    // maximal throttle of the ESC see: HERKULES_3_User_Manual_v040.pdf
    const float MAX_THROTTLE = 245;
    const float MIN_THROTTLE = 0;



    // current motors state armed or disarmed
    MOTORS_STATE_T motors_state;
    // velocity command to the motors [rpm]
    float velocity[N_MOTORS];
    // throttle of the motors that will be written to I2C bus
    // 8 bit resolution see: HERKULES_3_User_Manual_v040.pdf
    int throttle[N_MOTORS];
    // handle for the I2C master
    FT260_DEVICE_T *i2c_master;
    // initialization flag of the I2C master
    bool flag_i2c_init = false;

  public:
    /**
   * @name Motors()
   * @brief constructor open ft260 device setup i2c master, arm motors.
   */
    Motors()
    {
        // initialize FT260_DEVICE_T structure,
        // set vendor/product/interface IDs and all function pointers.
        i2c_master = New_FT260(0x0403, 0x6030, 0);

        if (FT260_OK != i2c_master->Open(i2c_master))
        {
            printf(PRED "I2C Open error!" PRST);
            return;
        }

        if (FT260_OK != i2c_master->I2C_Setup(i2c_master))
        {
            printf(PRED "I2C Setup error!" PRST);
            return;
        }

        flag_i2c_init = true;

        disarmMotors();
        
        // keep ~1s send init signal to ESC
        for (int i = 0; i < 200; i++)
            writeMotors();
    }

    /**
     * @name setMotorsVel
     * @brief set motors' velocity.
     * @param vel motors' velocity in rpm.
     */
    void setMotorsVel(float *vel)
    {
        if (!flag_i2c_init)
            return;
        if (motors_state != MOTOR_ARMED)
            return;

        for (int i = 0; i < N_MOTORS; ++i)
            velocity[i] = std::max(MIN_MOTOR_VELOCITY,std::min(vel[i],MAX_MOTOR_VELOCITY));
    }

    /**
     * @name armMotors
     * @brief allow setting velocity
     */
    void armMotors()
    {
        if (!flag_i2c_init)
            return;

        motors_state = MOTOR_ARMED;
    }

    /**
    * @name  disarmMotors
    * @brief set all motors to zero velocity.
    */
    void disarmMotors()
    {
        if (!flag_i2c_init)
            return;

        for (int i = 0; i < N_MOTORS; ++i)
        {
            velocity[i] = 0;
        }

        motors_state = MOTOR_DISARMED;
    }

    /**
     * @name getMotorsState
     * @brief get current motors' state armed/disarmed
     */
    MOTORS_STATE_T getMotorsState()
    {
        return motors_state;
    }

    /**
     * @name writeMotors
     * @brief write throttle to ESC via I2C master.
     */
    WRITE_STATE_T writeMotors()
    {
        uint8_t send_buf[2];
        FT260_STATUS ftStatus;
        int32_t writeLength = 0;
        int writeLength_sum = 0;

        normVelocity();

        send_buf[0] = throttle[0] & (0xff); // HB
        send_buf[1] = 0;                    // LB(no use)
        ftStatus = i2c_master->I2C_Write(i2c_master, motor_i2c_addr[0], FT260_I2C_REPEATED_START, send_buf, 2, &writeLength);
        writeLength_sum += writeLength;

        send_buf[0] = throttle[1] & (0xff); // HB
        send_buf[1] = 0;                    // LB(no use)
        ftStatus = i2c_master->I2C_Write(i2c_master, motor_i2c_addr[1], FT260_I2C_REPEATED_START, send_buf, 2, &writeLength);
        writeLength_sum += writeLength;

        send_buf[0] = throttle[2] & (0xff); // HB
        send_buf[1] = 0;                    // LB(no use)
        ftStatus = i2c_master->I2C_Write(i2c_master, motor_i2c_addr[2], FT260_I2C_REPEATED_START, send_buf, 2, &writeLength);
        writeLength_sum += writeLength;

        send_buf[0] = throttle[3] & (0xff); // HB
        send_buf[1] = 0;                    // LB(no use)
        ftStatus = i2c_master->I2C_Write(i2c_master, motor_i2c_addr[3], FT260_I2C_REPEATED_START, send_buf, 2, &writeLength);
        writeLength_sum += writeLength;

        send_buf[0] = throttle[4] & (0xff); // HB
        send_buf[1] = 0;                    // LB(no use)
        ftStatus = i2c_master->I2C_Write(i2c_master, motor_i2c_addr[4], FT260_I2C_REPEATED_START, send_buf, 2, &writeLength);
        writeLength_sum += writeLength;

        send_buf[0] = throttle[5] & (0xff); // HB
        send_buf[1] = 0;                    // LB(no use)
        ftStatus = i2c_master->I2C_Write(i2c_master, motor_i2c_addr[5], FT260_I2C_START_AND_STOP, send_buf, 2, &writeLength);
        writeLength_sum += writeLength;

        if (writeLength_sum == 12)
            return WRITE_OK;
        else
            return WRITE_ERROR;
    }

  private:
    /**
   * @name normVelocity
   * @brief normalize the velocity from rpm to "rounded" throttle(here 0-255 8-bit resolution)
   */
    void normVelocity()
    {
        for (int i = 0; i < N_MOTORS; ++i){
            if(velocity[i]<MIN_MOTOR_VELOCITY)
                throttle[i] = 0;
            else{
                throttle[i] = (velocity[i] - fractor_b)/fractor_k + 0.5;
                throttle[i] = std::max(MIN_THROTTLE,std::min(static_cast<float>(throttle[i]),MAX_THROTTLE));
            }
        }
    }
};

#endif /* __MOTORS_HPP__ */