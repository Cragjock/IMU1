
#include <ctime>
#include "sense.h"



using namespace std;

int main(int argc, char *argv[])
{
    cout<<setbase(16);
    //cout << "Hello world!" << endl;
    //cout<<"test enum "<< myI2C_address[IMU] <<endl;

    unsigned char read_buffer[288];
    unsigned char temp_buffer[4];
    temp_buffer[0]=0;
    temp_buffer[1]=1;
    temp_buffer[2]=2;
    temp_buffer[3]=3;

    short temp_reg = 0;
    //temp_reg = (ODR_G_119<<5) | (FS_500 << 3) | G_med;
    temp_reg = (ODR_G_119) | (FS_500) | G_med;


    uint8_t readb[80];
    for (int sc=0; sc<80; sc++)
        {
        read_buffer[sc]=0xae;
        readb[sc]=0xae;
        }


/// set the file pointer for I2C ======================
    I2C_FP[IMU]=    I2C_Init(devName,I2C_SLAVE_ADDR_IMU);
    I2C_FP[MAG]=    I2C_Init(devName,I2C_SLAVE_ADDR_MAG);
    I2C_FP[TEMP]=   I2C_Init(devName,I2C_SLAVE_ADDR_RHTEMP);

    XYZ GYRO;
    XYZ ACCEL;
    XYZ COMPASS;
    op_config op_gyro;
    op_config op_accel;

    op_accel.op_FS = XL_FS2;
    op_accel.op_ODR = ODR_119;
    op_accel.op_BW = G_med;


    temp_reg = arr_FS_XL[op_accel.op_FS] | arr_ODR_XLG[op_accel.op_ODR] | op_accel.op_BW;



/// init XL and gyro ===============
    for(j=0; j<10;j++)
    {
        int check_it = myI2C_write(I2C_FP[IMU], xl_mylist[j].reg_address, xl_mylist[j].value);
        if(check_it < 0)
        {
            perror("Failed to write G XL init");
            exit(1);
        }
    }

/// init mag ===========================
    for(j=0; j<5;j++)
    {
        int check_it = myI2C_write(I2C_FP[MAG], m_mylist[j].reg_address, m_mylist[j].value);
        if(check_it < 0)
        {
            perror("Failed to write to M init");
            exit(1);
        }
    }

/**
        set-up for do while continous loop; need to add
        cntr-c action to stop
*/
    clock_t goal;
   clock_t wait=(clock_t)2 * CLOCKS_PER_SEC;       // change the 2 for update rate, 2= about 2 seconds
//    do
 //{


/*********************************************************
    Read a buffer of size N for gyro and accel
    do the read per page 21 on burst read + temperature
*********************************************************/

/// read the 26 registers of gyro data, registers 0x18 - 0x1d IMU_OUT_TEMP_L IMU_OUT_X_L_G
    int result = i2c_smbus_read_i2c_block_data(I2C_FP[IMU], IMU_OUT_TEMP_L, 26, read_buffer+READ_BUF_G);
    if(result < 0)
        {
            perror("Failed to read from the i2c bus");
            exit(1);
        }
/// read the 6 registers of mag data, registers 0x28 - 0x2d
    result = i2c_smbus_read_i2c_block_data(I2C_FP[MAG], MAG_OUT_X_L_M, 6, read_buffer+32);
    if(result < 0)
        {
            perror("Failed to read from the i2c bus");
            exit(1);
        }

//    cout<<"\nbytes read in: "<<result<<endl;
//    for(j=0; j<11; j++)
//        cout<<"buffer at "<<j<<"== "<<(short int)read_buffer[j]<<endl;
//
//
//    for(j=0; j<6;j=j+2)
//        cout<<(short int)read_buffer[j+1]<<(short int)read_buffer[j]<<endl;

    short buftemp = short((read_buffer[temp_LB] | (read_buffer[temp_HB]<<8)));
    buftemp = buftemp/16;
    buftemp = buftemp+25;

    printf("is this the temp? %i degress C\n", buftemp);
    cout<<"temp from buffer directly in hex "<<buftemp<<endl;

    GYRO.x_LB = read_buffer[gx_LB];
    GYRO.x_UB = read_buffer[gx_HB];
    GYRO.y_LB = read_buffer[gy_LB];
    GYRO.y_UB = read_buffer[gy_HB];
    GYRO.z_LB = read_buffer[gz_LB];
    GYRO.z_UB = read_buffer[gz_HB];

    GYRO.x = (GYRO.x_UB<<8) | GYRO.x_LB;
    GYRO.y = (GYRO.y_UB<<8) | GYRO.y_LB;
    GYRO.z = (GYRO.z_UB<<8) | GYRO.z_LB;

    float myGX = (float)GYRO.x * G_So[G_FS500];
    float myGy = (float)GYRO.y * G_So[G_FS500];
    float myGz = (float)GYRO.z * G_So[G_FS500];


    float r_gyro = myGX*myGX +myGy*myGy + myGz*myGz;
    r_gyro =pow(r_gyro, 0.5);




//    cout<<"\t"<<"x\t"<<"y\t"<<"z\t"<<endl;
    cout<<setw(15)<<"x "<<setw(15)<<"y"<<setw(15)<<"z"<<setw(15)<<"r"<<endl;
    //cout<<"Gyro:\t"<<GYRO.x<<"\t"<<GYRO.y<<"\t"<<GYRO.z<<endl;

    //cout<<"Gyro:\t "<<myGX<<"\t"<<myGy<<"\t"<<myGz<<endl;
    cout<<"Gyro: "<<setw(15)<<myGX<<setw(15)<<myGy<<setw(15)<<myGz<<setw(15)<<r_gyro<<endl;
//    cout<<"Gyro: "<<
//        setw(10)<<GYRO.x<<
//        setw(10)<<GYRO.y<<
//        setw(10)<<GYRO.z<<endl;

    ACCEL.x_LB = read_buffer[xx_LB];
    ACCEL.x_UB = read_buffer[xx_HB];
    ACCEL.y_LB = read_buffer[xy_LB];
    ACCEL.y_UB = read_buffer[xy_HB];
    ACCEL.z_LB = read_buffer[xz_LB];
    ACCEL.z_UB = read_buffer[xz_HB];
// format word from 2 bytes
    ACCEL.x = (ACCEL.x_UB<<8) | ACCEL.x_LB;
    ACCEL.y = (ACCEL.y_UB<<8) | ACCEL.y_LB;
    ACCEL.z = (ACCEL.z_UB<<8) | ACCEL.z_LB;

   // cout<<"1 TEST ACCEL.y "<<ACCEL.y<<endl;


    float x_mytest= ACCEL.x;
    float y_mytest= ACCEL.y;
    float z_mytest= ACCEL.z;


    //x_mytest = x_mytest*LA_So_2;
    //y_mytest = y_mytest*LA_So_2;
    //z_mytest = z_mytest*LA_So_2;
    //x_mytest = x_mytest * LA_So[XL_FS2];
    x_mytest = x_mytest * LA_So[op_accel.op_FS];
    y_mytest = y_mytest * LA_So[XL_FS2];
    z_mytest = z_mytest * LA_So[XL_FS2];

    float r_accel = x_mytest*x_mytest +y_mytest*y_mytest + z_mytest*z_mytest;

    r_accel =pow(r_accel, 0.5);
    //cout<<"r_accel: "<<r_accel<<endl;


    //cout<<"mytest accel.y "<<mytest<<endl;

    //cout<<"2 complement "<< short(TC_CONVERT(ACCEL.y))<<endl;


//    cout<<"\t"<<"x\t"<<"y\t"<<"z\t"<<endl;
    //cout<<"Accel:\t"<<ACCEL.x<<"\t"<<ACCEL.y<<"\t"<<ACCEL.z<<endl;
 //   cout<<"Accel:\t"<<x_mytest<<"\t"<<y_mytest<<"\t"<<z_mytest<<endl;
    cout<<"Accel:"<<setw(15)<<x_mytest<<setw(15)<<y_mytest<<setw(15)<<z_mytest<<setw(15)<<r_accel<<endl;
//    cout<<"Accel: "<<
//        setw(10)<<ACCEL.x<<
//        setw(10)<<ACCEL.y<<
//        setw(10)<<ACCEL.z<<endl;




    COMPASS.x_LB = read_buffer[mx_LB];
    COMPASS.x_UB = read_buffer[mx_HB];
    COMPASS.y_LB = read_buffer[my_LB];
    COMPASS.y_UB = read_buffer[my_HB];
    COMPASS.z_LB = read_buffer[mz_LB];
    COMPASS.z_UB = read_buffer[mz_HB];
// format word from 2 bytes
    COMPASS.x = (COMPASS.x_UB<<8) | COMPASS.x_LB;
    COMPASS.y = (COMPASS.y_UB<<8) | COMPASS.y_LB;
    COMPASS.z = (COMPASS.z_UB<<8) | COMPASS.z_LB;

    float myCx = (float)COMPASS.x * M_GN[M_FS4];
    float myCy = (float)COMPASS.y * M_GN[M_FS4];
    float myCz = (float)COMPASS.z * M_GN[M_FS4];

    float r_comp = myCx*myCx +myCy*myCy + myCz*myCz;

    r_comp =pow(r_comp, 0.5);

    float dircosx = atan(myCx/r_comp);      //radians
    float dircosy = atan(myCy/r_comp);
    float dircosz = atan(myCz/r_comp);

    cout<<"dircos x "<<(180.0/3.141)*dircosx
        <<"\ndircos y "<<(180.0/3.141)*dircosy
        <<"\ndircos z "<<(180.0/3.141)*dircosz<<endl;



    //cout<<"COMP:\t"<<COMPASS.x<<"\t"<<COMPASS.y<<"\t"<<COMPASS.z<<endl;
 //   cout<<"COMP:\t"<<myCx<<"\t"<<myCy<<"\t"<<myCz<<endl;
    cout<<"COMP: "<<setw(15)<<myCx<<setw(15)<<myCy<<setw(15)<<myCz<<setw(15)<<r_comp<<endl;




    result= myI2C_read(I2C_FP[IMU], IMU_STATUS_REG);
    cout<<"status reg x17 data "<<result<<endl;

//    int templ= myI2C_read(I2C_FP[IMU], IMU_OUT_TEMP_L);
//    int temph= myI2C_read(I2C_FP[IMU], IMU_OUT_TEMP_H);
//    short tempdata = templ | (temph<<8);
//    cout<<"temp data "<<(~(tempdata)+1)<<endl;


    for(i=0; i< 12; i++)
    {
        //okornot= myI2C_read(I2C_FP[IMU], IMU_OUT_X_L_G+i);
        okornot= myI2C_read(I2C_FP[IMU], 0x15+i);
        if(okornot < 0)
        {
            perror("Failed to read from the i2c bus");
            exit(1);
        }
//        cout<<"single buffer at: "
//        <<(0x15+i)                 //  <<(IMU_OUT_X_L_G+i)
//        <<" is: "
//        <<okornot<<endl;
    }



/// stuff for the HTS221 temp-humidity device =======
    //result = myI2C_write(I2C_FP[TEMP], RHT_CTRL_REG1, 0x23);
    result = myI2C_write(I2C_FP[TEMP], RHT_CTRL_REG1, 0x81);    // was x87
    result = myI2C_write(I2C_FP[TEMP], RHT_AV_CONF, 0x23);  //was 1b

   result = i2c_smbus_read_i2c_block_data(I2C_FP[TEMP], (0x30 | RHT_BUS_READ), 18, readb);
    if(result < 0)
        {
            perror("Failed to read from the i2c bus");
            exit(1);
        }

   result = i2c_smbus_read_i2c_block_data(I2C_FP[TEMP], (RHT_TEMP_OUT_L | RHT_BUS_READ), 2, temp_buffer);
    if(result < 0)
        {
            perror("Failed to read 2a 2b from the i2c bus");
            exit(1);
        }

    int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
    int16_t T0_degC, T1_degC;
    unsigned char t_buffer[4];
    uint8_t t_tmp;
    uint32_t tmp32;
    float slope =0.0;

    uint8_t t_result = i2c_smbus_read_i2c_block_data(I2C_FP[TEMP], (RHT_T0_degC_x8  | RHT_BUS_READ), 2, t_buffer);
    if(t_result < 0)
        {
            perror("Failed to read 32 and 33 from the i2c bus");
            exit(1);
        }


    t_result = myI2C_read(I2C_FP[TEMP], RHT_T1_T0_MSB);
    if(t_result < 0)
        {
            perror("Failed to read 35 from the i2c bus");
            exit(1);
        }


    T0_degC_x8_u16 = (((uint16_t)(t_result & 0x03))<<8) | ((uint16_t)t_buffer[0]);
    T1_degC_x8_u16 = (((uint16_t)(t_result & 0x0c))<<6) | ((uint16_t)t_buffer[1]);
    T0_degC = T0_degC_x8_u16 >>3;
    T1_degC = T1_degC_x8_u16 >>3;
//    cout<<"T0_degC= "<<T0_degC<<endl;
//    cout<<"T1_degC= "<<T1_degC<<endl;

    float rise = (float)T1_degC - (float)T0_degC;
//    cout<<"rise = "<<rise<<endl;


    t_result = i2c_smbus_read_i2c_block_data(I2C_FP[TEMP], (RHT_T0_OUT_L | RHT_BUS_READ), 4, t_buffer);
    if(t_result < 0)
        {
            perror("Failed to read 3c 3d 3e 3f from the i2c bus");
            exit(1);
        }

    T0_out = (((uint16_t)t_buffer[1])<<8) | (uint16_t)t_buffer[0];
    T1_out = (((uint16_t)t_buffer[3])<<8) | (uint16_t)t_buffer[2];
//    cout<<"T0_out= "<<T0_out<<endl;
 //   cout<<"T1_out= "<<T1_out<<endl;

    float run = (float)T1_out - (float)T0_out;
//    cout<<"run = "<<run<<endl;

    slope = rise/run;
//    cout<<"slope =  "<<slope<<endl;



    T_out = (((uint16_t)temp_buffer[1])<<8) | (uint16_t)temp_buffer[0];
//    cout<<"T_out from 2a 2b= "<<T_out<<endl;

    tmp32 =((uint32_t)(T_out - T0_out)) * ((uint32_t)(T1_degC - T0_degC)*10);

    float intercept = (float)T0_degC - slope*(float)T0_out;
//    cout<<"intercept= "<<intercept<<endl;

    int16_t value = tmp32/(T1_out - T0_out) + T0_degC*10;
    cout<<"temp is deg C: "<<dec<<(value/10)<<endl;
    //cout<<"temp is deg C: "<<(value/10)<<endl;



    float t_temp = T_out*slope + intercept;
    cout<<"slope temp= "<<t_temp<<endl;

// reset to default for control registers
    //myI2C_write(I2C_FP[IMU], IMU_CTRL_REG3_G, 0x00);
    //myI2C_write(I2C_FP[IMU], IMU_CTRL_REG1_G, 0x00);
    //myI2C_write(I2C_FP[TEMP], RHT_CTRL_REG1, 0x00);

    result = myI2C_read(I2C_FP[IMU], IMU_FIFO_SRC);
    cout<<"fifo status "<<result<<endl;

    /// TEST
    //register_set_bit(I2C_FP[IMU], IMU_CTRL_REG9, FIFO_enable_bit);
    //register_clear_bit(I2C_FP[IMU], IMU_CTRL_REG9, FIFO_enable_bit);


//   goal = wait + clock();
 //   while( goal > clock() );

//}
//while(1);

    close(I2C_FP[IMU]);
    close(I2C_FP[MAG]);
    close(I2C_FP[TEMP]);
    return 0;
}

int I2C_Init(const char* devname, int slaveaddress)
{
    int myFP = open(devname, O_RDWR);       // device name= "/dev/i2c-1" how auto detect?
    if (myFP == -1)
        {
            perror(devname);
            exit(1);
        }
    if (ioctl(myFP, I2C_SLAVE, slaveaddress) < 0)
    //if (ioctl(myFP, I2C_SLAVE, I2C_SLAVE_ADDR_MAG) < 0)
    //if (ioctl(myFP, I2C_SLAVE_FORCE, I2C_SLAVE_ADDR_MAG) < 0)
    {
        perror("Failed to acquire bus access and/or talk to slave");
        exit(1);
    }
    return myFP;
}


int myI2C_read(int myFP, int reg_request)
{
    int result=i2c_smbus_read_byte_data(myFP, reg_request);
    if (result < 0)
        {
            perror("Failed to read from the i2c bus");
            exit(1);
        }
    return result;
}

int myI2C_write(int myFP, int reg_request, int data)
{
    int result = i2c_smbus_write_byte_data(myFP, reg_request, data);
    if (result < 0)
        {
            perror("Failed to write from the i2c bus");
            exit(1);
        }
    return result;
}



/// set a register bit X to 1
int register_set_bit(int fp, int RTC_reg, int bit_number)
{
    int read_result = myI2C_read(fp, RTC_reg);
    int set_mask = 0;
    set_mask |= (1<<bit_number);
    read_result |=set_mask;
    myI2C_write(fp, RTC_reg, read_result);

    return 0;
}


/// clear a register bit X to 0
int register_clear_bit(int fp, int RTC_reg, int bit_number)
{
    int read_result = myI2C_read(fp,RTC_reg);

    int set_mask = 0;
    set_mask |= ~(1<<bit_number);
    read_result &=set_mask;
    myI2C_write(fp, RTC_reg, read_result);

    return 0;
}



    //int checkresult = ioctl(I2C_FP[IMU], I2C_FUNCS, &funcs  );

//    if (!(funcs & I2C_FUNC_SMBUS_READ_BLOCK_DATA))
//    {
        /* Oops, the needed functionality (SMBus write_quick function) is not available! */
//    exit(1);
//    }
//    int result = i2c_smbus_read_block_data(I2C_FP[IMU], IMU_WHO_AM_I, read_buffer); NO GOOD  HERE



 /************************
    reg_data = myI2C_read(RTCMIN);
    cout<<"register minutes= "<<(0x7f & reg_data)<<endl;

    int writeok = myI2C_write(PWRDNMIN, reg_data);


    reg_data = myI2C_read(RTCSEC);
    cout<<"register seconds= "<<(0x7f & reg_data)<<endl;

    reg_data = myI2C_read(RTCMTH);
    cout<<"register month= "<<(0x1f & reg_data)<<endl;

    reg_data = myI2C_read(RTCDATE);
    cout<<"register day= "<<(0x2f & reg_data)<<endl;

    reg_data = myI2C_read(RTCWKDAY);
    cout<<"register week day= "<<(0x3 & reg_data)<<endl;

    reg_data = myI2C_read(RTCYR);
    cout<<"register year= "<<(reg_data)<<endl;

//   __s32 i2c_smbus_write_block_data(int file, __u8 command, __u8 length, __u8 *values);
    beter to use i2c_smbus_write_i2c_block_data
    same for read
    okornot= i2c_smbus_write_block_data(myFP, SRAM_START, 8, timestamp);
    /// NOTE:: this sends the length down to a buffer too. Maybe unique on this device ?
    okornot= myI2C_write(SRAM_START, &timestamp);
        if(okornot < 0)
        {
            perror("Failed to read from the i2c bus");
            exit(1);
        }

        cout<<"time stamp buffer "<<okornot<<endl;
    for(i=0; i< SRAM_SIZE; i++)
    {
        okornot= myI2C_write(SRAM_START+i, 0x0+i);
        if(okornot < 0)
        {
            perror("Failed to read from the i2c bus");
            exit(1);
        }
    }
********************************************/

