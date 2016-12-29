
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
#include <poll.h>
#include <signal.h>

/// I2C slave address
#define I2C_SLAVE_ADDR_RTC      0x6f        /// TBD
#define I2C_SLAVE_ADDR_IMU      0x6a        /// MEMS LSM9DS1
#define I2C_SLAVE_ADDR_MAG      0x1c        /// MEMS LSM9DS1
#define I2C_SLAVE_ADDR_RHTEMP   0x5f        /// MEMS HTS221
#define I2C_SLAVE_ADDR_PRESSURE 0x5c        /// MEMS LP25H


/// MEMS LSM9DS1 Gyro, Accel Registers
#define ACT_THS             0x04
#define ACT_DUR             0x05
#define INT_GEN_CFG_XL      0x06
#define INT_GEN_THS_X_XL    0x07
#define INT_GEN_THS_Y_XL    0x08
#define INT_GEN_THS_Z_XL    0x09
#define INT_GEN_DUR_XL      0x0a
#define REFERENCE_G         0x0b
#define INT1_CTRL           0x0c
#define INT2_CTRL           0x0d
#define WHO_AM_I_XLG        0x0f
#define CTRL_REG1_G         0x10
#define CTRL_REG2_G         0x11
#define CTRL_REG3_G         0x12
#define ORIENT_CFG_G        0x13
#define INT_GEN_SRC_G       0x14
#define OUT_TEMP_L          0x15
#define OUT_TEMP_H          0x16
#define STATUS_REG          0x17
#define OUT_X_L_G           0x18
#define OUT_X_H_G           0x19
#define OUT_Y_L_G           0x1a
#define OUT_Y_H_G           0x1b
#define OUT_Z_L_G           0x1c
#define OUT_Z_H_G           0x1d
#define CTRL_REG4           0x1e
#define CTRL_REG5_XL        0x1f
#define CTRL_REG6_XL        0x20
#define CTRL_REG7_XL        0x21
#define CTRL_REG8           0x22
#define CTRL_REG9           0x23
#define CTRL_REG10          0x24
#define INT_GEN_SRC_XL      0x26
#define STATUS_REG1         0x27
#define OUT_X_L_XL          0x28
#define OUT_X_H_XL          0x29
#define OUT_Y_L_XL          0x2a
#define OUT_Y_H_XL          0x2b
#define OUT_Z_L_XL          0x2c
#define OUT_Z_H_XL          0x2d
#define FIFO_CTRL           0x2e
#define FIFO_SRC            0x2f
#define INT_GEN_CFG_G       0x30
#define INT_GEN_THS_XH_G    0x31
#define INT_GEN_THS_XL_G    0x32
#define INT_GEN_THS_YH_G    0x33
#define INT_GEN_THS_YL_G    0x34
#define INT_GEN_THS_ZH_G    0x35
#define INT_GEN_THS_ZL_G    0x36
#define INT_GEN_DUR_G       0x37


///  Gyro ODR
#define GYRO_ODR_PD      0
#define GYRO_ODR_14_9    1
#define GYRO_ODR_59_5    2
#define GYRO_ODR_119     3
#define GYRO_ODR_238     4
#define GYRO_ODR_476     5
#define GYRO_ODR_952     6

///  Gyro banwidth
#define GYRO_BW_0    0
#define GYRO_BW_1    1
#define GYRO_BW_2    2
#define GYRO_BW_3    3

///  Gyro FSR
#define GYRO_FSR_250        0
#define GYRO_FSR_500        1
#define GYRO_FSR_2000       2

///  Gyro HPF
#define GYRO_HPF_0          0
#define GYRO_HPF_1          1
#define GYRO_HPF_2          2
#define GYRO_HPF_3          3
#define GYRO_HPF_4          4
#define GYRO_HPF_5          5
#define GYRO_HPF_6          6
#define GYRO_HPF_7          7
#define GYRO_HPF_8          8
#define GYRO_HPF_9          9

///  XL ODR
#define XL_ODR_14_9    1
#define XL_ODR_59_5    2
#define XL_ODR_119     3
#define XL_ODR_238     4
#define XL_ODR_476     5
#define XL_ODR_952     6

///  XL FSR
#define XL_FSR_2     0
#define XL_FSR_16    1
#define XL_FSR_4     2
#define XL_FSR_8     3

///  XL BW for anti-alias LPF
#define XL_LPF_408   0
#define XL_LPF_211   1
#define XL_LPF_105   2
#define XL_LPF_50    3

///  XL digital filter
#define XL_DCF_50   0
#define XL_DCF_100  1
#define XL_DCF_9   2
#define XL_DCF_400   3




/// MEMS LSM9DS1 Magnetic Registers
#define OFFSET_X_REG_L_M    0x05
#define OFFSET_X_REG_H_M    0x06
#define OFFSET_Y_REG_L_M    0x07
#define OFFSET_Y_REG_H_M    0x08
#define OFFSET_Z_REG_L_M    0x09
#define OFFSET_Z_REG_H_M    0x0a
#define WHO_AM_I_M          0x0f
#define CTRL_REG1_M         0x20
#define CTRL_REG2_M         0x21
#define CTRL_REG3_M         0x22
#define CTRL_REG4_M         0x23
#define CTRL_REG5_M         0x24
#define STATUS_REG_M        0x27
#define OUT_X_L_M           0x28
#define OUT_X_H_M           0x29
#define OUT_Y_L_M           0x2a
#define OUT_Y_H_M           0x2b
#define OUT_Z_L_M           0x2c
#define OUT_Z_H_M           0x2d
#define INT_CFG_M           0x30
#define INT_SRC_M           0x31
#define INT_THS_L_M         0x32
#define INT_THS_H_M         0x22

///  Mag ODR
#define Mag_ODR_0_625    0
#define Mag_ODR_1_25     1
#define Mag_ODR_2_5      2
#define Mag_ODR_5        3
#define Mag_ODR_10       4
#define Mag_ODR_20       5
#define Mag_ODR_40       6
#define Mag_ODR_80       7

///  Mag FSR
#define Mag_FSR_4       0
#define Mag_FSR_8       1
#define Mag_FSR_12      2
#define Mag_FSR_16      3


///  Mag MD
#define Mag_CONT        0
#define Mag_SINGLE      1
#define Mag_OM3         2
#define Mag_OM4         3



#define READ_BUF_G         0x00
#define READ_BUF_XL        0x08

/// 2-complement macro
#define TC_CONVERT(x)   ((~x)+1)

/// FIFO modes
#define FIFO_bypass     0x00    /// reg 0x2e
#define FIFO_mode       0x01    /// reg 0x2e
#define FIFO_enable_bit 1       /// reg 0x23 bit 1

const float ALPHA = 0.4;
#define YL_Declination 12.0


/// bit order
///ODR2 ODR1 ODR0 FS1 FS0 0 BW1 BW0
enum ODR_G {G_power_down=0, ODR_G_14_9=0x20, ODR_G_59_5=0x40, ODR_G_119=0x60, ODR_G_238=0x80, ODR_G_476=0xA0, ODR_G_952=0xC0};
enum FS_G {FS_245=0, FS_500=0x08, FS_NA=0x10, FS_1200=0x18};
enum BW_G {G_low, G_med, G_high};

/// bit order
/// ODR2 ODR1 ODR0 FS1 FS0 BWS BWX1 BWX0
enum ODR_XL {XL_power_down=0, ODR_XL_10=0x20, ODR_XL_50=0x40, ODR_XL_119=0x60, ODR_XL_238=0x80, ODR_XL_476=0xA0, ODR_XL_952=0xC0};
enum FS_XL {FS_2=0, FS_16=0x08, FS_4=0x10, FS_8=0x18 };
enum BW_XL {BW_408, BW_211, BW_105, BW_50 };

const int arr_FS_XL[]   = {0, 0x08, 0x10, 0x18 };
const int arr_ODR_XLG[] = {0, 0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0};


enum XLFS { XL_FS2, XL_FS4, XL_FS8, XL_FS16};
enum GFS { G_FS245, G_FS500, G_FS2000};
enum MFS { M_FS4, M_FS8, M_FS12, M_FS16};

enum LAGODR {PwrDown, ODR_149, ODR_595, ODR_119, ODR_238, ODR_476, ODR_952};
enum MAGODR {ODR_625, ODR_125, ODR_25, ODR_5, ODR_10, ODR_20, ODR_40, ODR_80};

/// sensitvities per spec, need access to these items
const float XL_So[]     = { 0.000061, 0.000732, 0.000122, 0.000244};
const float G_So[]      = {.00875, .01750, 0.70};
const float M_GN[]      = {0.00014, 0.00029, 0.00043, 0.00058 };


//const float LA_G_ODR[]  = {0.0, 14.9, 59.5, 119.0, 238.0, 476.0, 952.0};
//const float Mag_ODR[]   = {0.625, 1.25, 5.0, 10.0, 20.0, 40.0, 80.0};
//const int G_BW[]        = {0 ,1, 2, 3};
//const int XL_BW[]       = {0, 1, 2, 3};


/************************************/
/// I2C items
int I2C_Init(const char* devname, int sadrress);
int myI2C_read(int whoaddress,int RTC_reg);
int myI2C_write(int whpaddress, int RTC_reg, int data);
int myI2C_read_block(int myFP, int reg_request, int rd_size, unsigned char* readbuffer);
const char * devName = "/dev/i2c-1";
enum I2C_bus {IMU, MAG, TEMP, HUMID, RTC};
const int myI2C_address[5]= {I2C_SLAVE_ADDR_IMU, 0x1c, 0x5c, 0x5f, 0x6f};
int I2C_FP[5];
/************************************/


uint8_t funcs;
int16_t testdata;

int okornot;
int i;
uint8_t timestamp[7]={0};
//char timestamp[7];


typedef struct tag_config
{
    short x_enable;
    short y_enable;
    short z_enable;

    int mag_ODR;    ///M_CR 0x20
    int mag_FS;     ///M_CR 0x21
    int mag_DM;     ///M_CR 0x22

    int gyro_ODR;   ///G_CR 0x10
    int gyro_FS;    ///G_CR 0x10
    int gyro_BW;    ///G_CR 0x10
    int gyro_HPF;   ///G_CR 0x12

    int XL_ODR;     /// XL_CR 0x20
    int XL_FS;      /// XL_CR 0x20
    int XL_BW;      /// XL_CR 0x20
    int XL_DCF;     /// XL_CR 0x21

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
    int dynMax;
    int dynMin;
    int dynAvg;
} XYZ;

typedef struct tag_DCM
{
    float x;    /// x component in, x angle out
    float y;    /// y component in, y angle out
    float z;    /// z component in, z angle out
    float r;    /// magnitude of x, y, z out
} DCM;



struct entry
{
    int reg_address;
    uint8_t value;
};

/// i2c bus 0x6a
struct entry xl_mylist[11]={
                            {CTRL_REG1_G, 0b01101001},      /// 0x10, 0x69
                            {CTRL_REG2_G, 0b00000010},      /// 0x11, 0x02
                            {CTRL_REG3_G, 0b01000100},      /// 0x12, 0x44
                            {ORIENT_CFG_G, 0b00000000},      /// 0x13,
                            {CTRL_REG5_XL, 0b00111000},     /// 0x1f, 0x38
                            {CTRL_REG6_XL, 0b01100011},     /// 0x20, 0x66
                            {CTRL_REG7_XL, 0b00000000},     /// 0x21, 0x00
                            {CTRL_REG8, 0b00000100},        /// 0x22, 0x04
                            {CTRL_REG9, 0b00010000},        /// 0x23, 0x02
                            {CTRL_REG10, 0b00000000},       /// 0x24, 0x00
                            {FIFO_CTRL, 0b00001111}         /// 0x2e, 0x2f 0x0f is off
                        };
/// i2c bus 0x1c
struct entry m_mylist[5]={
                            {CTRL_REG1_M, 0b10110000},          /// 0x20, 0xb0
                            {CTRL_REG2_M, 0b00000000},          /// 0x21, 0x00
                            {CTRL_REG3_M, 0b00000000},          /// 0x22, 0x83
                            {CTRL_REG4_M, 0b00000100},          /// 0x23, 0x00
                            {CTRL_REG5_M, 0b00000000}           /// 0x24, 0x00

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

void compute_DCM(DCM* pdcm);

void sig_handler(int sig);
bool ctrl_c_pressed = false;

#endif // SENSE_H
