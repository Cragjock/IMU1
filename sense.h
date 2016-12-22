
#ifndef SENSE_H
#define SENSE_H

#include <iostream>
#include <iomanip>
#include <cmath>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <time.h>

/// I2C slave address
#define I2C_SLAVE_ADDR_RTC      0x6f        /// TBD
#define I2C_SLAVE_ADDR_IMU      0x6a        /// MEMS LSM9DS1
#define I2C_SLAVE_ADDR_MAG      0x1c        /// MEMS LSM9DS1
#define I2C_SLAVE_ADDR_RHTEMP   0x5f        /// MEMS HTS221
#define I2C_SLAVE_ADDR_PRESSURE 0x5c        /// MEMS LP25H


/// MEMS LSM9DS1 Gyro, Accel Registers
#define IMU_ACT_THS             0x04
#define IMU_ACT_DUR             0x05
#define IMU_INT_GEN_CFG_XL      0x06
#define IMU_INT_GEN_THS_X_XL    0x07
#define IMU_INT_GEN_THS_Y_XL    0x08
#define IMU_INT_GEN_THS_Z_XL    0x09
#define IMU_GEN_DUR_XL          0x0a
#define IMU_REFERENCE_G         0x0b
#define IMU_INT1_CTRL           0x0c
#define IMU_INT2_CTRL           0x0d
/// register 0x0e RESERVED
#define IMU_WHO_AM_I            0x0f
#define IMU_CTRL_REG1_G         0x10




#define IMU_CTRL_REG2_G         0x11
#define IMU_CTRL_REG3_G         0x12
#define IMU_ORIENT_CFG_G        0x13
#define IMU_INT_GEN_SRC_G       0x14
#define IMU_OUT_TEMP_L          0x15
#define IMU_OUT_TEMP_H          0x16
#define IMU_STATUS_REG          0x17
#define IMU_OUT_X_L_G           0x18
#define IMU_OUT_X_H_G           0x19
#define IMU_OUT_Y_L_G           0x1a
#define IMU_OUT_Y_H_G           0x1b
#define IMU_OUT_Z_L_G           0x1c
#define IMU_OUT_Z_H_G           0x1d
#define IMU_CTRL_REG4           0x1e
#define IMU_CTRL_REG5_XL        0x1f
#define IMU_CTRL_REG6_XL        0x20



#define IMU_CTRL_REG7_XL        0x21
#define IMU_CTRL_REG8           0x22
#define IMU_CTRL_REG9           0x23
#define IMU_CTRL_REG10          0x24
/// register 0x25 RESERVED
#define IMU_GEN_SRC_XL          0x26
#define IMU_STATUS_REG1         0x27
#define IMU_OUT_X_L_XL          0x28
#define IMU_OUT_X_H_XL          0x29
#define IMU_OUT_Y_L_XL          0x2a
#define IMU_OUT_Y_H_XL          0x2b
#define IMU_OUT_Z_L_XL          0x2c
#define IMU_OUT_Z_H_XL          0x2d
#define IMU_FIFO_CTRL           0x2e
#define IMU_FIFO_SRC            0x2f
#define IMU_INT_GEN_CFG_G       0x30
#define IMU_INT_GEN_THS_XH_G    0x31
#define IMU_INT_GEN_THS_XL_G    0x32
#define IMU_INT_GEN_THS_YH_G    0x33
#define IMU_INT_GEN_THS_YL_G    0x34
#define IMU_INT_GEN_THS_ZH_G    0x35
#define IMU_INT_GEN_THS_ZL_G    0x36
#define IMU_INT_GEN_DUR_G       0x37


/// MEMS LSM9DS1 Magnetic Registers
/// register 00-04 RESERVED
#define MAG_OFFSET_X_REG_L_M    0x05
#define MAG_OFFSET_X_REG_H_M    0x06
#define MAG_OFFSET_Y_REG_L_M    0x07
#define MAG_OFFSET_Y_REG_H_M    0x08
#define MAG_OFFSET_Z_REG_L_M    0x09
#define MAG_OFFSET_Z_REG_H_M    0x0a
/// register 0b-0e RESERVED
#define MAG_WHO_AM_I            0x0f
/// register 10-1e RESERVED
#define MAG_CTRL_REG1_M         0x20
#define MAG_CTRL_REG2_M         0x21
#define MAG_CTRL_REG3_M         0x22
#define MAG_CTRL_REG4_M         0x23
#define MAG_CTRL_REG5_M         0x24
/// register 25-26 RESERVED
#define MAG_STATUS_REG_M        0x27
#define MAG_OUT_X_L_M           0x28
#define MAG_OUT_X_H_M           0x29
#define MAG_OUT_Y_L_M           0x2a
#define MAG_OUT_Y_H_M           0x2b
#define MAG_OUT_Z_L_M           0x2c
#define MAG_OUT_Z_H_M           0x2d
/// register 2e-2f RESERVED
#define MAG_INT_CFG_M           0x30
#define MAG_INT_SRC_M           0x31
#define MAG_INT_THS_L_M         0x32
#define MAG_INT_THS_H_M         0x22

#define READ_BUF_G              0x00
#define READ_BUF_XL             0x08

/// 2-complement macro
#define TC_CONVERT(x)   ((~x)+1)



#define G_FS_245    0x00
#define G_FS_500    0x10
#define G_FS_2000    0x11

/// FIFO modes
#define FIFO_bypass     0x00    /// reg 0x2e
#define FIFO_mode       0x01    /// reg 0x2e
#define FIFO_enable_bit 1       /// reg 0x23 bit 1


/// RTC
#define SRAM_SIZE       0x40    // 64 bytes 0-3f
#define SRAM_START      0x20
#define RTCSEC          0x00
#define RTCMIN          0x01
#define RTCHR           0x02
#define RTCWKDAY        0x03
#define RTCDATE         0x04
#define RTCMTH          0x05
#define RTCYR           0x06
#define RTC_CONTROL     0x07
#define RTC_OSCTRIM     0x08
#define PWRDNMIN        0x18

/// RH-Temp HTS221 MEMS
#define RHT_WHO_AM_I    0x0f
#define RHT_AV_CONF     0x10
#define RHT_CTRL_REG1   0x20
#define RHT_CTRL_REG2   0x21
#define RHT_CTRL_REG3   0x22
#define RHT_STATUS_REG  0x27
#define RHT_HUMID_OUT_L 0x28
#define RHT_HUMID_OUT_H 0x29
#define RHT_TEMP_OUT_L  0x2a
#define RHT_TEMP_OUT_H  0x2b
/// Temp calibration interporlation
#define RHT_T0_degC_x8  0x32
#define RHT_T1_degC_x8  0x33
#define RHT_T1_T0_MSB   0x35
#define RHT_T0_OUT_L    0x3c
#define RHT_T0_OUT_H    0x3d
#define RHT_T1_OUT_L    0x3e
#define RHT_T1_OUT_H    0x3f
/// needed for multibyte I2C reads
#define RHT_BUS_READ    0x80


/// bit order
///ODR2 ODR1 ODR0 FS1 FS0 0 BW1 BW0
enum ODR_G {G_power_down=0, ODR_G_14_9=0x20, ODR_G_59_5=0x40, ODR_G_119=0x60, ODR_G_238=0x80, ODR_G_476=0xA0, ODR_G_952=0xC0};
enum FS_G {FS_245=0, FS_500=0x08, FS_NA=0x10, FS_1200=0x18};
enum BW_G {G_low, G_med, G_high};

/// bit order
/// ODR2 ODR1 ODR0 FS1 FS0 BWS BWX1 BWX0
enum ODR_XL { XL_power_down=0, ODR_XL_10=0x20, ODR_XL_50=0x40, ODR_XL_119=0x60, ODR_XL_238=0x80, ODR_XL_476=0xA0, ODR_XL_952=0xC0};
enum FS_XL {FS_2=0, FS_16=0x08, FS_4=0x10, FS_8=0x18 };
enum BW_XL {BW_408, BW_211, BW_105, BW_50 };

enum LAFS { XL_FS2, XL_FS4, XL_FS8, XL_FS16};
enum GFS { G_FS245, G_FS500, G_FS2000};
enum MFS { M_FS4, M_FS8, M_FS12, M_FS16};
enum LAGODR {PwrDown, ODR_149, ODR_595, ODR_119, ODR_238, ODR_476, ODR_952};
enum MAGODR {ODR_625, ODR_125, ODR_25, ODR_5, ODR_10, ODR_20, ODR_40, ODR_80};

const float LA_So[] = { 0.000061, 0.000122, 0.000244, 0.000732};
const float G_So[] = {.00875, .01750, 0.70};
const float M_GN[] = {0.00014, 0.00029, 0.00043, 0.00058 };
const float LA_G_ODR[] = {0.0, 14.9, 59.5, 119.0, 238.0, 476.0, 952.0};
const float Mag_ODR[] = {0.625, 1.25, 5.0, 10.0, 20.0, 40.0, 80.0};

const int arr_FS_XL[] = {0, 0x08, 0x10, 0x18 };
const int arr_ODR_XLG[]= {0, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0};

int I2C_Init(const char* devname, int sadrress);
int myI2C_read(int whoaddress,int RTC_reg);
int myI2C_write(int whpaddress, int RTC_reg, int data);
const char * devName = "/dev/i2c-1";
// static int myFP;        //  for the file pointer

enum I2C_bus {IMU, MAG, TEMP, HUMID, RTC};
const int myI2C_address[5]= {I2C_SLAVE_ADDR_IMU, 0x1c, 0x5c, 0x5f, 0x6f};
int I2C_FP[5];


uint8_t funcs;
int16_t testdata;

int okornot;
int i;
uint8_t timestamp[7]={0};
//char timestamp[7];


typedef struct tag_config
{
    short op_ODR;
    short op_FS;
    short op_BW;

} op_config;


typedef struct tag_XYZ
{
    short x_LB;
    short x_UB;
    short x;
    short y_LB;
    short y_UB;
    short y;
    short z_LB;
    short z_UB;
    short z;
} XYZ;

struct entry
{
    int reg_address;
    uint8_t value;
};

/// i2c bus 0x6a
struct entry xl_mylist[10]={
                            {IMU_CTRL_REG1_G, 0b01101001},      /// 0x10, 0x69
                            {IMU_CTRL_REG2_G, 0b00000010},      /// 0x11, 0x02
                            {IMU_CTRL_REG3_G, 0b01000100},      /// 0x12, 0x44
                            {IMU_CTRL_REG5_XL, 0b00111000},     /// 0x1f, 0x38
                            {IMU_CTRL_REG6_XL, 0b01100011},     /// 0x20, 0x66
                            {IMU_CTRL_REG7_XL, 0b00000000},     /// 0x21, 0x00
                            {IMU_CTRL_REG8, 0b00000100},        /// 0x22, 0x04
                            {IMU_CTRL_REG9, 0b00010000},        /// 0x23, 0x02
                            {IMU_CTRL_REG10, 0b00000000},       /// 0x24, 0x00
                            {IMU_FIFO_CTRL, 0b00001111}         /// 0x2e, 0x2f 0x0f is off
                        };
/// i2c bus 0x1c
struct entry m_mylist[5]={
                            {MAG_CTRL_REG1_M, 0b10110000},          /// 0x20, 0xb0
                            {MAG_CTRL_REG2_M, 0b00000000},          /// 0x21, 0x00
                            {MAG_CTRL_REG3_M, 0b00000000},          /// 0x22, 0x83
                            {MAG_CTRL_REG4_M, 0b00000100},          /// 0x23, 0x00
                            {MAG_CTRL_REG5_M, 0b00000000}           /// 0x24, 0x00

                        };


int j;

    enum buf_offset
    {
        temp_LB=0,
        temp_HB,
        gx_LB,
        gx_HB,
        gy_LB,
        gy_HB,
        gz_LB,
        gz_HB,
        xx_LB,
        xx_HB,
        xy_LB,
        xy_HB,
        xz_LB,
        xz_HB,
        mx_LB=32,
        mx_HB,
        my_LB,
        my_HB,
        mz_LB,
        mz_HB
    };


int register_set_bit(int fp, int RTC_reg, int bit_number);
int register_clear_bit(int fp, int RTC_reg, int bit_number);

#endif // SENSE_H
