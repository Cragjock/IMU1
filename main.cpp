
#include <ctime>
#include "sense.h"



using namespace std;

int main(int argc, char *argv[])
{
    cout.precision(3);
    cout<<setbase(16);

/**************************************************/
/// signal handler

    #ifdef __linux__
 	struct sigaction sig_struct;
	sig_struct.sa_handler = sig_handler;
	sig_struct.sa_flags = 0;
	sigemptyset(&sig_struct.sa_mask);

	if (sigaction(SIGINT, &sig_struct, NULL) == -1)
    {
		cout << "Problem with sigaction" << endl;
		exit(1);
	}
	#endif // __linux__
/***************************************************/

    unsigned char read_buffer[288];

    short temp_reg = 0;
    //temp_reg = (ODR_G_119<<5) | (FS_500 << 3) | G_med;
    temp_reg = (ODR_G_119) | (FS_500) | G_med;


    uint8_t readb[80];
    for (int sc=0; sc<80; sc++)
        {
        read_buffer[sc]=0xae;
        readb[sc]=0xae;
        }

/*******************************/
/// Init I2C
/// set the file pointer for I2C
    I2C_FP[IMU]=    I2C_Init(devName,I2C_SLAVE_ADDR_IMU);
    I2C_FP[MAG]=    I2C_Init(devName,I2C_SLAVE_ADDR_MAG);

/*******************************/
/// init XL and gyro ===========
    for(j=0; j<11;j++)
    {
        int check_it = myI2C_write(I2C_FP[IMU], xl_mylist[j].reg_address, xl_mylist[j].value);
        if(check_it < 0)
        {
            perror("Failed to write G XL init");
            exit(1);
        }
    }
/*******************************/
/// init mag =================
    for(j=0; j<5;j++)
    {
        int check_it = myI2C_write(I2C_FP[MAG], m_mylist[j].reg_address, m_mylist[j].value);
        if(check_it < 0)
        {
            perror("Failed to write to M init");
            exit(1);
        }
    }


/***********************************/
/// do the loop forever
/// set-up for do while continous loop;
/// signal handler breaks the loop

#ifdef LOOP
    clock_t goal;
    clock_t wait=(clock_t)2 * CLOCKS_PER_SEC;       // change the 2 for update rate, 2= about 2 seconds
    do
{
#endif // LOOP

    cout<<"~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;

/******************************/
/// set the device configurations
    XYZ GYRO;
    XYZ ACCEL;
    XYZ COMPASS;

    op_config myConfig = {1,1,1,
                            Mag_ODR_10,     /// CRMAG 0x20
                            Mag_FSR_4,      /// CRMAG 0x21
                            Mag_CONT,       /// CRMAG 0x22
                            GYRO_ODR_119,   /// CRG 0x10
                            GYRO_FSR_250,   /// CRG 0x10
                            GYRO_BW_1,      /// CRG 0x10
                            GYRO_HPF_3,     /// CRG 0x12
                            XL_ODR_119,     /// CRXL 0x20
                            XL_FSR_8,       /// CRXL 0x20
                            XL_LPF_50,     /// CRXL 0x20
                            XL_DCF_100};    /// CRXL 0x21
/************************************/

    int c_register =0;
    int c1_register = 0;


    /// Gyro items
    c1_register = arr_ODR_XLG[myConfig.gyro_ODR] | arr_FS_XL[myConfig.gyro_FS];
    myI2C_write(I2C_FP[IMU], CTRL_REG1_G, c1_register);

    /// XL items
    c1_register = arr_ODR_XLG[myConfig.XL_ODR] | arr_FS_XL[myConfig.XL_FS] | myConfig.XL_BW;
    myI2C_write(I2C_FP[IMU], CTRL_REG6_XL, c1_register);








/******************************************************
    Read a buffer of size N for gyro and accel
    do the read per page 21 on burst read + temperature
*******************************************************/
    int result = -1;
    result= myI2C_read_block(I2C_FP[IMU], OUT_TEMP_L, 26, read_buffer);
    result= myI2C_read_block(I2C_FP[MAG], OUT_X_L_M, 6, read_buffer+32);

/***************************/
/// temperature register
    short buftemp = short((read_buffer[temp_LB] | (read_buffer[temp_HB]<<8)));
    buftemp = buftemp/16;
    buftemp = buftemp+25;
    printf("is this the temp? %i degress C\n", buftemp);
    cout<<"temp from buffer directly in hex "<<buftemp<<endl;


/********************************************/
///  Gyro register read and convert
    GYRO.x_LB = read_buffer[gx_LB];
    GYRO.x_UB = read_buffer[gx_HB];
    GYRO.y_LB = read_buffer[gy_LB];
    GYRO.y_UB = read_buffer[gy_HB];
    GYRO.z_LB = read_buffer[gz_LB];
    GYRO.z_UB = read_buffer[gz_HB];

    GYRO.x = (GYRO.x_UB<<8) | GYRO.x_LB;
    GYRO.y = (GYRO.y_UB<<8) | GYRO.y_LB;
    GYRO.z = (GYRO.z_UB<<8) | GYRO.z_LB;

    float G_offset = 0.00;

    float myGX = (float)GYRO.x * G_So[myConfig.gyro_FS] + G_offset;
    float myGy = (float)GYRO.y * G_So[myConfig.gyro_FS] + G_offset;
    float myGz = (float)GYRO.z * G_So[myConfig.gyro_FS] + G_offset;
    //float myGz = (float)GYRO.z * G_So[G_FS500] + G_offset;
    float myGX_1 = .02;

    DCM dcm_Gyro ={myGX, myGy,myGz, 0.0};
    compute_DCM(&dcm_Gyro);

    float r_gyro = myGX*myGX +myGy*myGy + myGz*myGz;
    r_gyro =pow(r_gyro, 0.5);

    cout<<setw(15)<<"x "<<setw(15)<<"y"<<setw(15)<<"z"<<setw(15)<<"r"<<endl;
    cout<<"Gyro: "<<setw(15)<<myGX<<setw(15)<<myGy<<setw(15)<<myGz<<setw(15)<<r_gyro<<endl<<endl;;

    //myGX_1 = ((1-ALPHA)*myGX)+(ALPHA*myGX_1);
    //cout<<"myGX: "<<myGX<<endl<<"myGX_1: "<<myGX_1<<endl;

/********************************************/
///  Accel register read and convert
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
    float XL_offset = 0.005;     // was .02
    float x1_mytest;

    DCM dcm_Accel ={x_mytest, y_mytest,z_mytest, 0.0};
    compute_DCM(&dcm_Accel);


    cout<<"XL dircos "<<setw(15)<<(180.0/M_PI)*dcm_Accel.x
        <<setw(15)<<(180.0/M_PI)*dcm_Accel.y
        <<setw(15)<<(180.0/M_PI)*dcm_Accel.z
        <<setw(15)<<dcm_Accel.r<<endl;




    //x_mytest = x_mytest * XL_So[XL_FS2];
    x_mytest = x_mytest * XL_So[myConfig.XL_FS] + XL_offset;
    y_mytest = y_mytest * XL_So[myConfig.XL_FS] + XL_offset;
    z_mytest = z_mytest * XL_So[myConfig.XL_FS] + XL_offset;

    float r_accel = x_mytest*x_mytest +y_mytest*y_mytest + z_mytest*z_mytest;
    r_accel =pow(r_accel, 0.5);

    cout<<"Accel:"<<setw(15)<<x_mytest<<setw(15)<<y_mytest<<setw(15)<<z_mytest<<setw(15)<<r_accel<<endl<<endl;

/********************************************/
///  Magnetometer register read and convert
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

    float myCx = (float)COMPASS.x * M_GN[myConfig.mag_FS];    // gauss
    float myCy = (float)COMPASS.y * M_GN[myConfig.mag_FS];
    float myCz = (float)COMPASS.z * M_GN[myConfig.mag_FS];

    //float myCx = (float)COMPASS.x * M_GN[M_FS4];    // gauss
    //float myCx = (float)COMPASS.x * .00014;    // gauss
    //float myCy = (float)COMPASS.y * M_GN[M_FS4];
    //float myCz = (float)COMPASS.z * M_GN[M_FS4];

    float r_comp = myCx*myCx +myCy*myCy + myCz*myCz;
    r_comp =pow(r_comp, 0.5);

/// direction cosines
    float dircosx = acos(myCx/r_comp);      //radians
    float dircosy = acos(myCy/r_comp);
    float dircosz = acos(myCz/r_comp);

/// heading
    float heading = 0.0;
    //heading = atan(myCx/myCy);  /// or is this y/x?
    heading = atan(myCy/myCx);
    heading = heading *(180/M_PI);

    if(myCy > 0)
        heading = 90 - heading;
    if(myCy < 0)
        heading = 270 - heading;


    //cout<<"COMP:\t"<<COMPASS.x<<"\t"<<COMPASS.y<<"\t"<<COMPASS.z<<endl;
 //   cout<<"COMP:\t"<<myCx<<"\t"<<myCy<<"\t"<<myCz<<endl;
    cout<<"COMP: "<<setw(15)<<myCx<<setw(15)<<myCy<<setw(15)<<myCz<<setw(15)<<r_comp<<endl;
    cout<<"C RAW: "<<dec<<setw(15)<<COMPASS.x<<setw(15)<<COMPASS.y<<setw(15)<<COMPASS.z<<endl;

    cout<<"dircos "<<setw(15)<<(180.0/M_PI)*dircosx
        <<setw(15)<<(180.0/M_PI)*dircosy
        <<setw(15)<<(180.0/M_PI)*dircosz<<endl;

    cout<<"true heading = "<<heading<<endl;


    result= myI2C_read(I2C_FP[IMU], STATUS_REG);
    cout<<"status reg x17 data "<<result<<endl;



    result = myI2C_read(I2C_FP[IMU], FIFO_SRC);
    cout<<"fifo status "<<result<<endl;



/*******************************************/
/// signal handler to break the do while(1) loop
#ifdef LOOP
    if(ctrl_c_pressed)
    {
        cout << "Ctrl^C Pressed" << endl;
        cout << "unexporting pins" << endl;
        //gpio26->unexport_gpio();
        //gpio16->unexport_gpio();
        cout << "deallocating GPIO Objects" << endl;
        //delete gpio26;
        //gpio26 = 0;
        break;
    }
/*****************************************/

    goal = wait + clock();
    while( goal > clock() );
}

/***********************************/
/// do the loop forever
    while(1);
#endif // LOOP

    /// cleanup and close
    close(I2C_FP[IMU]);
    close(I2C_FP[MAG]);
    cout<<"good bye"<<endl;

    return 0;
}



/**********************************/
/// DCM
void compute_DCM(DCM* p_DCM)
{
    float rmag = (p_DCM->x *p_DCM->x) + (p_DCM->y *p_DCM->y) + (p_DCM->z *p_DCM->z);
    p_DCM->r = pow(rmag, 0.5);
    p_DCM->x = acos(p_DCM->x/p_DCM->r);
    p_DCM->y = acos(p_DCM->y/p_DCM->r);
    p_DCM->z = acos(p_DCM->z/p_DCM->r);


/// direction cosines
//    float dircosx = acos(myCx/r_comp);      //radians
//    float dircosy = acos(myCy/r_comp);
//    float dircosz = acos(myCz/r_comp);

}



/**********************************/
/// signal handler
void sig_handler(int sig)
{
	write(0,"\nCtrl^C pressed in sig handler\n",32);
	ctrl_c_pressed = true;
}


/************************************/
/// I2C stuff
///

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



int myI2C_read_block(int myFP, int reg_request, int rd_size, unsigned char* readbuffer)
{

/// read the 26 registers of gyro data, registers 0x18 - 0x1d IMU_OUT_TEMP_L IMU_OUT_X_L_G
    //int result = i2c_smbus_read_i2c_block_data(I2C_FP[IMU], OUT_TEMP_L, 26, read_buffer+READ_BUF_G);
    int result = i2c_smbus_read_i2c_block_data(myFP, reg_request, rd_size, readbuffer);
    if(result < 0)
        {
            perror("Failed to read from the i2c bus");
            exit(1);
        }

    return result;
}
/************************************/



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





/// read the 26 registers of gyro data, registers 0x18 - 0x1d IMU_OUT_TEMP_L IMU_OUT_X_L_G
    //int result = i2c_smbus_read_i2c_block_data(I2C_FP[IMU], OUT_TEMP_L, 26, read_buffer+READ_BUF_G);
    //if(result < 0)
    //    {
    //        perror("Failed to read from the i2c bus");
    //        exit(1);
    //    }
/// read the 6 registers of mag data, registers 0x28 - 0x2d
    //int result = i2c_smbus_read_i2c_block_data(I2C_FP[MAG], OUT_X_L_M, 6, read_buffer+32);
    //if(result < 0)
    //    {
    //        perror("Failed to read from the i2c bus");
    //        exit(1);
    //    }

//    cout<<"\nbytes read in: "<<result<<endl;
//    for(j=0; j<11; j++)
//        cout<<"buffer at "<<j<<"== "<<(short int)read_buffer[j]<<endl;
//
//
//    for(j=0; j<6;j=j+2)
//        cout<<(short int)read_buffer[j+1]<<(short int)read_buffer[j]<<endl;





/// update control registers for config settings


//        short temp_value = arr_FS_XL[op_gyro.op_FS] | arr_ODR_XLG[op_gyro.op_ODR] | op_gyro.op_BW ;

//        int check_it = myI2C_write(I2C_FP[IMU], CTRL_REG1_G, temp_value);
//        if(check_it < 0)
//        {
//            perror("Failed to write to Accel init");
//            exit(1);
//        }

//        temp_value = arr_FS_XL[op_accel.op_FS] | arr_ODR_XLG[op_accel.op_ODR] | op_accel.op_BW ;
//        check_it = myI2C_write(I2C_FP[IMU],CTRL_REG6_XL, temp_value);
//        if(check_it < 0)
//        {
//            perror("Failed to write to M init");
//            exit(1);
//        }



 //   switch(myConfig.gyro_ODR)
 //   {
 //       case GYRO_ODR_14_9:
 //           c_register = ODR_G_14_9;
//            break;
//        case GYRO_ODR_59_5:
//            c_register = ODR_G_59_5;
//            break;
//        case GYRO_ODR_119:
//            c_register = ODR_G_119;
//            break;
//        case GYRO_ODR_238:
//            c_register = ODR_G_238;
//            break;
//        case GYRO_ODR_476:
//            c_register = ODR_G_476;
//            break;
//        case GYRO_ODR_952:
//            c_register = ODR_G_952;
//            break;
//        default:
//            c_register = 0x00;
//    }



