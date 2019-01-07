/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if defined(HAL_IMUF_RESET_PIN) && defined(HAL_IMUF_READY_PIN)
#include "AP_InertialSensor_IMUF.h"
#include <stdio.h>
#include <utility>

#define IMUF_FIRMWARE_MIN_VERSION 1108
#define IMUF_RESET_ATTEMPTS 10
#define IMUF_COMM_MODE 48

#define IMUF_BL_ERASE_ALL         22
#define IMUF_BL_REPORT_INFO       24
#define IMUF_BL_WRITE_FIRMWARE    29
#define IMUF_BL_PREPARE_PROGRAM   30
#define IMUF_BL_END_PROGRAM       31
#define IMUF_BL_LISTENING         32
#define IMUF_COMMAND_NONE          0
#define IMUF_COMMAND_CALIBRATE    99
#define IMUF_COMMAND_LISTENING   108
#define IMUF_COMMAND_REPORT_INFO 121
#define IMUF_COMMAND_SETUP       122
#define IMUF_COMMAND_SETPOINT    126
#define IMUF_COMMAND_RESTART     127

extern const AP_HAL::HAL& hal;

AP_InertialSensor_IMUF::AP_InertialSensor_IMUF(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev,
                                                   enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_IMUF::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_IMUF(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_IMUF::start()
{
    printf("IMUF start\n");
    accel_instance = _imu.register_accel(1000, dev->get_bus_id_devtype(DEVTYPE_INS_IMUF));
    gyro_instance = _imu.register_gyro(1000,   dev->get_bus_id_devtype(DEVTYPE_INS_IMUF));
    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);
    
    // setup callback
    dev->register_periodic_callback(1000,
                                    FUNCTOR_BIND_MEMBER(&AP_InertialSensor_IMUF::read_sensor, void));
    printf("IMUF start done\n");
}

/*
  calculate a 32 bit CRC using the STM32 hardware CRC support. Uses
  the default polynomial of 0x04C11DB7
 */
static uint32_t crc_block(const uint32_t *data, uint8_t len)
{
    CRC->CR = CRC_CR_RESET;
    while (len--) {
        CRC->DR = *data++;
    }
    return CRC->DR;
}


static void imuf_reset(bool bootloaderMode)
{
    printf("RESET!\n");

    hal.gpio->pinMode(HAL_IMUF_RESET_PIN, 1);

    if(bootloaderMode) {
        hal.gpio->pinMode(HAL_IMUF_READY_PIN, 1);
        hal.gpio->write(HAL_IMUF_READY_PIN, 1);
    }

    hal.gpio->write(HAL_IMUF_RESET_PIN, 0);
    hal.scheduler->delay(200);
    hal.gpio->write(HAL_IMUF_RESET_PIN, 1);

    printf("RESET COMPLETE!\n");
    hal.scheduler->delay(70);
    if(bootloaderMode) {
        hal.gpio->write(HAL_IMUF_READY_PIN, 0);
        hal.gpio->pinMode(HAL_IMUF_READY_PIN, 0);
    }
}

bool AP_InertialSensor_IMUF::wait_ready(uint32_t timeout_ms)
{
    hal.scheduler->delay_microseconds(100);
    uint32_t start_ms = AP_HAL::millis();
    palSetLineMode(HAL_GPIO_PIN_IMUF_READY, PAL_MODE_INPUT);
    while (AP_HAL::millis() - start_ms < timeout_ms) {
        if (hal.gpio->read(HAL_IMUF_READY_PIN)) {
            return true;
        }
        hal.scheduler->delay_microseconds(100);
    }
    return hal.gpio->read(HAL_IMUF_READY_PIN) == 1;
}

bool AP_InertialSensor_IMUF::imuf_send_receive(IMUFCommand* cmd, IMUFCommand* reply)
{
    uint32_t crcCalc, commandToSend;

    memset(reply, 0, sizeof(IMUFCommand));

    printf("IMUF sending and receiving\n");

    cmd->crc = crc_block((const uint32_t *)cmd, 11);

    hal.gpio->pinMode(HAL_IMUF_READY_PIN, 0);
    if (!wait_ready(100)) {
        printf("not ready\n");
        return false;
    }

    bool ret = dev->transfer_fullduplex((const uint8_t *)cmd, (uint8_t *)reply, sizeof(IMUFCommand));
    if (!ret) {
        printf("transfer failed\n");
        return false;
    }

    crcCalc = crc_block((const uint32_t *)reply, 11);
    printf("reply1: cmd=%u cmdr=%u crc=0x%08x crc2=0x%08x crcr=0x%08x\n", cmd->command, reply->command, cmd->crc, crcCalc, reply->crc);

    if ((crcCalc == reply->crc) && ((reply->command == IMUF_COMMAND_LISTENING) || (reply->command == IMUF_BL_LISTENING))) {    //this tells us the IMU was listening for a command, else we need to reset sync

        if (!wait_ready(100)) {
            printf("transfer failed\n");
            return false;
        }

        commandToSend = cmd->command; //get old command for reference.
        cmd->command = IMUF_COMMAND_NONE; //reset command, just waiting for reply data now
        cmd->crc = crc_block((const uint32_t *)reply, 11); //set CRC

        if (commandToSend == IMUF_BL_ERASE_ALL) //give IMU-f time to erase flash
        {
            hal.scheduler->delay(600);
        }
        if (commandToSend == IMUF_BL_WRITE_FIRMWARE) //give IMU-f time to write firmware to flash
        {
            hal.scheduler->delay(10);
        }

        ret = dev->transfer_fullduplex((const uint8_t *)cmd, (uint8_t *)reply, sizeof(IMUFCommand));
        if (!ret) {
            printf("transfer failed\n");
            return false;
        }

        crcCalc = crc_block((const uint32_t *)reply, 11); //set CRC
        printf("reply2: cmd=%u cmdr=%u crc=0x%08x crc2=0x%08x crcr=0x%08x\n", cmd->command, reply->command, cmd->crc, crcCalc, reply->crc);
        printf("crcCalc=0x%08x, reply->crc=0x%08x, reply->command=%u, command sent=%u\n", crcCalc, reply->crc, reply->command, commandToSend);
        if ((crcCalc == reply->crc) && (reply->command == commandToSend)) //this tells us the IMU understood the last command
        {
            return true;
        } else {
            printf("IMU-f crc check on 2 failed\n");
            return false;
        }
    } else {
        printf("IMU-f crc check on 1 failed\n");
        return false;
    }
    
    return false;

}


void AP_InertialSensor_IMUF::setup_whoami_command(IMUFCommand* cmd)
{
    cmd->command = IMUF_COMMAND_REPORT_INFO; //set command to get firmware version info
    memset(cmd->param, 0, sizeof(uint32_t) * 10); //zero params, packed struct
    cmd->crc = crc_block((const uint32_t *)&cmd, 11); //set crc
}

void AP_InertialSensor_IMUF::setup_contract(IMUFCommand* cmd, uint32_t imufVersion)
{
    cmd->command  = IMUF_COMMAND_SETUP;
    cmd->param[0] = IMUF_COMM_MODE; // gyro+accel+temp+crc
    cmd->param[1] = (5<<16) | 300; // 2kHz, 300 window
    cmd->param[2] = (3000<<16) | 3000; // RollQ, PitchQ
    cmd->param[3] = (3000<<16) | 100; // YawQ, RollGyroLPF
    cmd->param[4] = (100<<16) | 100; // pitchGyroLPF, YawGyroLPF
    cmd->param[5] = 0; // unused
    cmd->param[6] = 0; // unused
    cmd->param[7] = 0; // RollOrient, Orient
    cmd->param[8] = 0; // YawOrient, PitchOrient
    cmd->param[9] = 0; // unknown
    cmd->crc      = crc_block((const uint32_t *)&cmd, 11);

    // data->param2 = ( (uint16_t)(gyroConfig()->imuf_rate+1) << 16)            | (uint16_t)gyroConfig()->imuf_w;
    // data->param3 = ( (uint16_t)gyroConfig()->imuf_roll_q << 16)              | (uint16_t)gyroConfig()->imuf_pitch_q;
    // data->param4 = ( (uint16_t)gyroConfig()->imuf_yaw_q << 16)               | (uint16_t)gyroConfig()->imuf_roll_lpf_cutoff_hz;
    // data->param5 = ( (uint16_t)gyroConfig()->imuf_pitch_lpf_cutoff_hz << 16) | (uint16_t)gyroConfig()->imuf_yaw_lpf_cutoff_hz;
    // data->param7 = ( (uint16_t)0 << 16)                                      | (uint16_t)0;
    // data->param8 = ( (int16_t)boardAlignment()->rollDegrees << 16 )          | imufGyroAlignment();
    // data->param9 = ( (int16_t)boardAlignment()->yawDegrees << 16 )           | (int16_t)boardAlignment()->pitchDegrees;
}

bool AP_InertialSensor_IMUF::init()
{
    struct IMUFCommand cmd {}, reply {};

    // for (uint8_t i=0; i<40; i++) {
    //     printf("waiting %u\n", i);
    //     hal.scheduler->delay(100);
    // }

    //enable CRC
    rccEnableCRC(FALSE);

    //set blocking
    dev->get_semaphore()->take_blocking();
    for (uint8_t attempt = 0; attempt < IMUF_RESET_ATTEMPTS; attempt++)
    {
        printf("IMU-f Read Attempt %d\n", attempt);

        imuf_reset(false);
        imuf_reset(false);
        if(attempt)
        {
            imuf_reset(false);
            hal.scheduler->delay(200 * attempt);
        }

        //SETUP A COMMAND
        setup_whoami_command(&cmd);
        if (imuf_send_receive(&cmd, &reply)) //imuf_send_receive returns true is reply is valid
        {
            if(reply.param[1] >= IMUF_FIRMWARE_MIN_VERSION) { //make sure version is compatible. 106 to current (110) is okay to use)
                printf("IMU-f version is safe. Version %d Found!\n", reply.param[1]);
                setup_contract(&cmd, reply.param[1]); //setup and start the IMU-f
                if (imuf_send_receive(&cmd, &reply))
                {
                    dev->get_semaphore()->give();
                    return true;
                }
            } else {
                printf("IMU-f needs to be updated. Version %d Found!\n", reply.param[1]);
            }
        }
    }

    dev->get_semaphore()->give();
    while(true) //handle failure here, this is temporary, return false?
    {
        printf("IMU-f problem\n");
        palToggleLine(HAL_GPIO_PIN_LED0);
        hal.scheduler->delay(500);
    }
    return false;
}

/*
  read accel fifo
 */
void AP_InertialSensor_IMUF::read_sensor(void)
{
    struct PACKED imuCommFrame
    {
        float gyro[3];
        float accel[3];
        float temp;
        float quat[4];
        uint32_t crc;
        uint32_t tail;
    };

    struct imuCommFrame data {};
    struct IMUFCommand command {};

    //TODO: This needs to be changed to send actual setpoint data in DPS
    command.command = IMUF_COMMAND_SETPOINT;
    command.param[0]=0; //commanded roll, 
    command.param[1]=0; //commanded pitch, 
    command.param[2]=0; //commanded yaw, 
    command.param[3]=1; //armed, 
    command.param[4]=crc_block((uint32_t *)&command, 5); 

    if (!dev->transfer_fullduplex((uint8_t *)&command, (uint8_t *)&data, 50)) {
        //printf("BAD TRANSFER OF IMUF!");
        return;
    }

    uint32_t calcedCRC = crc_block( (uint32_t *)(&data), (IMUF_COMM_MODE >> 2) - 1);

    if (calcedCRC != data.crc) {
        //printf("BAD CRC!!!!!!!!!!! crc2 0x%08x crc 0x%08x\n", calcedCRC, data.crc);
        //printf("IMUF accel %.2f %.2f %.2f\n", data.accel[0], data.accel[1], data.accel[2]);
        return;
    }

    //IMU-f outputs DPS, but we need RPS, convert here.
    Vector3f gyro(data.gyro[0]*0.0174533f, data.gyro[1]*0.0174533f, data.gyro[2]*0.0174533f);
    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    //IMU-f outputs Gees, but we need m/s^2, convert here.
    Vector3f accel(data.accel[0]*9.807f, data.accel[1]*9.807f, data.accel[2]*9.807f);
    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    _publish_temperature(accel_instance, data.temp);
}

bool AP_InertialSensor_IMUF::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

#endif
