/**
 *   @defgroup  eMPL
 *   @brief     Embedded Motion Processing Library
 *
 *   @{
 *       @file      motion_driver_test.c
 */
#include "stm32f1xx.h"
//#include "msp430_clock.h"
#include "i2c.h"
//#include "msp430_interrupt.h"

#include "dmp_user.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "math.h"
#include "motor.h"

/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

short gyro[3], accel[3], sensors;
unsigned char more;
long quat[4];
float q[4];
const float q30_inv = 9.313225746154785e-10;
float pitch, roll, yaw;
float q_cali[3][4] = {0};
float q_cali_inv[3][4] = {0};
int32_t calibration_device[3] = {1, 1, 1}, calibration_cnt[3] = {0};

/*
enum packet_type_e {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC
};
*/

/* Send data to the Python client application.
 * Data is formatted as follows:
 * packet[0]    = $
 * packet[1]    = packet type (see packet_type_e)
 * packet[2+]   = data
 */
/*
void send_packet(char packet_type, void *data)
{
#define MAX_BUF_LENGTH  (18)
    char buf[MAX_BUF_LENGTH], length;

    memset(buf, 0, MAX_BUF_LENGTH);
    buf[0] = '$';
    buf[1] = packet_type;

    if (packet_type == PACKET_TYPE_ACCEL || packet_type == PACKET_TYPE_GYRO) {
        short *sdata = (short*)data;
        buf[2] = (char)(sdata[0] >> 8);
        buf[3] = (char)sdata[0];
        buf[4] = (char)(sdata[1] >> 8);
        buf[5] = (char)sdata[1];
        buf[6] = (char)(sdata[2] >> 8);
        buf[7] = (char)sdata[2];
        length = 8;
    } else if (packet_type == PACKET_TYPE_QUAT) {
        long *ldata = (long*)data;
        buf[2] = (char)(ldata[0] >> 24);
        buf[3] = (char)(ldata[0] >> 16);
        buf[4] = (char)(ldata[0] >> 8);
        buf[5] = (char)ldata[0];
        buf[6] = (char)(ldata[1] >> 24);
        buf[7] = (char)(ldata[1] >> 16);
        buf[8] = (char)(ldata[1] >> 8);
        buf[9] = (char)ldata[1];
        buf[10] = (char)(ldata[2] >> 24);
        buf[11] = (char)(ldata[2] >> 16);
        buf[12] = (char)(ldata[2] >> 8);
        buf[13] = (char)ldata[2];
        buf[14] = (char)(ldata[3] >> 24);
        buf[15] = (char)(ldata[3] >> 16);
        buf[16] = (char)(ldata[3] >> 8);
        buf[17] = (char)ldata[3];
        length = 18;
    } else if (packet_type == PACKET_TYPE_TAP) {
        buf[2] = ((char*)data)[0];
        buf[3] = ((char*)data)[1];
        length = 4;
    } else if (packet_type == PACKET_TYPE_ANDROID_ORIENT) {
        buf[2] = ((char*)data)[0];
        length = 3;
    } else if (packet_type == PACKET_TYPE_PEDO) {
        long *ldata = (long*)data;
        buf[2] = (char)(ldata[0] >> 24);
        buf[3] = (char)(ldata[0] >> 16);
        buf[4] = (char)(ldata[0] >> 8);
        buf[5] = (char)ldata[0];
        buf[6] = (char)(ldata[1] >> 24);
        buf[7] = (char)(ldata[1] >> 16);
        buf[8] = (char)(ldata[1] >> 8);
        buf[9] = (char)ldata[1];
        length = 10;
    } else if (packet_type == PACKET_TYPE_MISC) {
        buf[2] = ((char*)data)[0];
        buf[3] = ((char*)data)[1];
        buf[4] = ((char*)data)[2];
        buf[5] = ((char*)data)[3];
        length = 6;
    }
    cdcSendDataWaitTilDone((BYTE*)buf, length, CDC0_INTFNUM, 100);
}
*/

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

/* Handle sensor on/off combinations. */
static void setup_gyro(void)
{
    unsigned char mask = 0;
    if (hal.sensors & ACCEL_ON)
        mask |= INV_XYZ_ACCEL;
    if (hal.sensors & GYRO_ON)
        mask |= INV_XYZ_GYRO;
    /* If you need a power transition, this function should be called with a
     * mask of the sensors still enabled. The driver turns off any sensors
     * excluded from this mask.
     */
    mpu_set_sensors(mask);
    if (!hal.dmp_on)
        mpu_configure_fifo(mask);
}

/*
static void tap_cb(unsigned char direction, unsigned char count)
{
    char data[2];
    data[0] = (char)direction;
    data[1] = (char)count;
    send_packet(PACKET_TYPE_TAP, data);
}
*/

/*
static void android_orient_cb(unsigned char orientation)
{
    send_packet(PACKET_TYPE_ANDROID_ORIENT, &orientation);
}
*/


/*
static inline void msp430_reset(void)
{
    PMMCTL0 |= PMMSWPOR;
}
*/


static inline void run_self_test(void)
{
    int result;
    //char test_packet[4] = {0};
    long gyro[3], accel[3];
    unsigned char i = 0;

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x7) {
        printf("DMP self test passed\n");
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        for(i = 0; i<3; i++) {
        	gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
        	accel[i] *= 4096.f; //convert to +-8G
        	accel[i] = accel[i] >> 16;
        	gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
    }
    printf("DMP self test didn't passed\n");

    /* Report results. */
    //test_packet[0] = 't';
    //test_packet[1] = result;
    //send_packet(PACKET_TYPE_MISC, test_packet);
}

/*
static void handle_input(void)
{
    char c;
    const unsigned char header[3] = "inv";
    unsigned long pedo_packet[2];

    rx_new = 0;
    cdcReceiveDataInBuffer((BYTE*)&c, 1, CDC0_INTFNUM);
    if (hal.rx.header[0] == header[0]) {
        if (hal.rx.header[1] == header[1]) {
            if (hal.rx.header[2] == header[2]) {
                memset(&hal.rx.header, 0, sizeof(hal.rx.header));
                hal.rx.cmd = c;
            } else if (c == header[2])
                hal.rx.header[2] = c;
            else
                memset(&hal.rx.header, 0, sizeof(hal.rx.header));
        } else if (c == header[1])
            hal.rx.header[1] = c;
        else
            memset(&hal.rx.header, 0, sizeof(hal.rx.header));
    } else if (c == header[0])
        hal.rx.header[0] = header[0];
    if (!hal.rx.cmd)
        return;

    switch (hal.rx.cmd) {
    case '8':
        if (!hal.dmp_on) {
            hal.sensors ^= ACCEL_ON;
            setup_gyro();
        }
        break;
    case '9':
        if (!hal.dmp_on) {
            hal.sensors ^= GYRO_ON;
            setup_gyro();
        }
        break;
    case 'a':
        hal.report ^= PRINT_ACCEL;
        break;
    case 'g':
        hal.report ^= PRINT_GYRO;
        break;
    case 'q':
        hal.report ^= PRINT_QUAT;
        break;
    case 't':
        run_self_test();
        break;
    case '1':
        if (hal.dmp_on)
            dmp_set_fifo_rate(10);
        else
            mpu_set_sample_rate(10);
        break;
    case '2':
        if (hal.dmp_on)
            dmp_set_fifo_rate(20);
        else
            mpu_set_sample_rate(20);
        break;
    case '3':
        if (hal.dmp_on)
            dmp_set_fifo_rate(40);
        else
            mpu_set_sample_rate(40);
        break;
    case '4':
        if (hal.dmp_on)
            dmp_set_fifo_rate(50);
        else
            mpu_set_sample_rate(50);
        break;
    case '5':
        if (hal.dmp_on)
            dmp_set_fifo_rate(100);
        else
            mpu_set_sample_rate(100);
        break;
    case '6':
        if (hal.dmp_on)
            dmp_set_fifo_rate(200);
        else
            mpu_set_sample_rate(200);
        break;
	case ',':
        dmp_set_interrupt_mode(DMP_INT_GESTURE);
        break;
    case '.':
        dmp_set_interrupt_mode(DMP_INT_CONTINUOUS);
        break;
    case '7':
        dmp_set_pedometer_step_count(0);
        dmp_set_pedometer_walk_time(0);
        break;
    case 'f':
        if (hal.dmp_on) {
            unsigned short dmp_rate;
            hal.dmp_on = 0;
            mpu_set_dmp_state(0);
            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            dmp_get_fifo_rate(&dmp_rate);
            mpu_set_sample_rate(dmp_rate);
        } else {
            unsigned short sample_rate;
            hal.dmp_on = 1;
            hal.sensors |= ACCEL_ON | GYRO_ON;
            mpu_set_sensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            mpu_configure_fifo(INV_XYZ_ACCEL | INV_XYZ_GYRO);
            mpu_get_sample_rate(&sample_rate);
            dmp_set_fifo_rate(sample_rate);
            mpu_set_dmp_state(1);
        }
        break;
    case 'm':
		#ifndef MPU6050 // not enabled for 6050 product
		hal.motion_int_mode = 1;
		#endif 
        break;
    case 'p':
        dmp_get_pedometer_step_count(pedo_packet);
        dmp_get_pedometer_walk_time(pedo_packet + 1);
        send_packet(PACKET_TYPE_PEDO, pedo_packet);
        break;
    case 'x':
        msp430_reset();
        break;
    case 'v':
        hal.dmp_features ^= DMP_FEATURE_6X_LP_QUAT;
        dmp_enable_feature(hal.dmp_features);
        break;
    default:
        break;
    }
    hal.rx.cmd = 0;
}
*/

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
static void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}

/*
static inline void platform_init(void)
{
	WDTCTL = WDTPW | WDTHOLD;
    SetVCore(2);
    msp430_clock_init(12000000L, 2);
    if (USB_init() != kUSB_succeed)
        msp430_reset();
    msp430_i2c_enable();
    msp430_int_init();

    USB_setEnabledEvents(kUSB_allUsbEvents);
    if (USB_connectionInfo() & kUSB_vbusPresent){
        if (USB_enable() == kUSB_succeed){
            USB_reset();
            USB_connect();
        } else
            msp430_reset();
    }
}
*/

uint8_t mpu_dmp_init(void){
    I2cInit();
    MotorInit();

    int32_t i;
    for(i = 0; i < 3; ++i){
        switch(i){
            case 0:
                SetI2CTarget(I2C1);
                SetHwAddr(0x68);
                break;
            case 1:
                SetI2CTarget(I2C1);
                SetHwAddr(0x69);
                break;
            case 2:
                SetI2CTarget(I2C2);
                SetHwAddr(0x68);
                break;
            default:
                break;
        }
        printf("MPU %d initialization start.\n", i);
        int result;
        /*
        unsigned char accel_fsr;
        unsigned short gyro_rate, gyro_fsr;
        unsigned long timestamp;
        struct int_param_s int_param;

        platform_init();
        */

        /* Set up gyro.
        * Every function preceded by mpu_ is a driver function and can be found
        * in inv_mpu.h.
        */
        /*
        int_param.cb = gyro_data_ready_cb;
        int_param.pin = INT_PIN_P20;
        int_param.lp_exit = INT_EXIT_LPM0;
        int_param.active_low = 1;
        */
        result = mpu_init(0);
        if (result)NVIC_SystemReset();

        /* If you're not using an MPU9150 AND you're not using DMP features, this
        * function will place all slaves on the primary bus.
        * mpu_set_bypass(1);
        */

        /* Get/set hardware configuration. Start gyro. */
        /* Wake up all sensors. */
        printf("Wake up accel and gyro sensors\n");
        if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL))return 2;
        /* Push both gyro and accel data into the FIFO. */
        printf("Configure accel and gyro data into fifo\n");
        if(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))return 3;
        printf("Set sample rate\n");
        if(mpu_set_sample_rate(25))return 4;
        /* Read back configuration in case it was set improperly. */
        /*
        mpu_get_sample_rate(&gyro_rate);
        mpu_get_gyro_fsr(&gyro_fsr);
        mpu_get_accel_fsr(&accel_fsr);
        */

        /* Initialize HAL state variables. */
        memset(&hal, 0, sizeof(hal));
        hal.sensors = ACCEL_ON | GYRO_ON;
        //hal.report = PRINT_QUAT;

        /* To initialize the DMP:
        * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
        *    inv_mpu_dmp_motion_driver.h into the MPU memory.
        * 2. Push the gyro and accel orientation matrix to the DMP.
        * 3. Register gesture callbacks. Don't worry, these callbacks won't be
        *    executed unless the corresponding feature is enabled.
        * 4. Call dmp_enable_feature(mask) to enable different features.
        * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
        * 6. Call any feature-specific control functions.
        *
        * To enable the DMP, just call mpu_set_dmp_state(1). This function can
        * be called repeatedly to enable and disable the DMP at runtime.
        *
        * The following is a short summary of the features supported in the DMP
        * image provided in inv_mpu_dmp_motion_driver.c:
        * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
        * 200Hz. Integrating the gyro data at higher rates reduces numerical
        * errors (compared to integration on the MCU at a lower sampling rate).
        * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
        * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
        * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
        * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
        * an event at the four orientations where the screen should rotate.
        * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
        * no motion.
        * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
        * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
        * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
        * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
        */
        printf("Load driver firmawre\n");
        if(dmp_load_motion_driver_firmware())return 5;
        printf("Driver firmware load success\n");

        printf("Set mpu orientation\n");
        if(dmp_set_orientation(
            inv_orientation_matrix_to_scalar(gyro_orientation)))return 6;
        //dmp_register_tap_cb(tap_cb);
        //dmp_register_android_orient_cb(android_orient_cb);
        /*
        * Known Bug -
        * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
        * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
        * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
        * there will be a 25Hz interrupt from the MPU device.
        *
        * There is a known issue in which if you do not enable DMP_FEATURE_TAP
        * then the interrupts will be at 200Hz even if fifo rate
        * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
        */
        printf("Enable MPU features\n");
        hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL;
        if(dmp_enable_feature(hal.dmp_features))return 7;
        printf("Set fifo rate\n");
        if(dmp_set_fifo_rate(25))return 8;
        printf("Enable dmp\n");
        mpu_set_dmp_state(1);
        hal.dmp_on = 1;
        
        printf("Run self test\n");
        run_self_test();

        //close dmp temporary
        mpu_set_dmp_state(0);
        hal.dmp_on = 0;
    }

    for(i = 0; i < 3; ++i){
        switch(i){
            case 0:
                SetI2CTarget(I2C1);
                SetHwAddr(0x68);
                mpu_set_dmp_state(1);
                break;
            case 1:
                SetI2CTarget(I2C1);
                SetHwAddr(0x69);
                mpu_set_dmp_state(1);
                break;
            case 2:
                SetI2CTarget(I2C1);
                SetHwAddr(0x68);
                mpu_set_dmp_state(1);
                break;
            default:
                break;
        }
    }
    hal.dmp_on = 1;

    //__enable_interrupt();

    /* Wait for enumeration. */
    //while (USB_connectionState() != ST_ENUM_ACTIVE);
    return 0;
}

uint8_t get_dmp_data(){
    //unsigned long sensor_timestamp;
        /* A byte has been received via USB. See handle_input for a list of
            * valid commands.
            */
    //if (rx_new)handle_input();
    //msp430_get_clock_ms(&timestamp);

    /*
    if (hal.motion_int_mode) {
        mpu_lp_motion_interrupt(500, 1, 5);
        hal.new_gyro = 0;
        while (!hal.new_gyro)
            __bis_SR_register(LPM0_bits + GIE);
        mpu_lp_motion_interrupt(0, 0, 0);
        hal.motion_int_mode = 0;
    }
    */

    /*
    if (!hal.sensors || !hal.new_gyro) {
        __bis_SR_register(LPM0_bits + GIE);
        continue;
    }
    */


    int32_t mpuIndex;
    for(mpuIndex = 0; mpuIndex < 3; ++mpuIndex){
        switch(mpuIndex){
            case 0:
                SetI2CTarget(I2C1);
                SetHwAddr(0x68);
                break;
            case 1:
                SetI2CTarget(I2C1);
                SetHwAddr(0x69);
                break;
            case 2:
                SetI2CTarget(I2C2);
                SetHwAddr(0x68);
                break;
            default:
                break;
        }

        gyro_data_ready_cb();

        if (hal.new_gyro && hal.dmp_on) {
            /* This function gets new data from the FIFO when the DMP is in
                * use. The FIFO can contain any combination of gyro, accel,
                * quaternion, and gesture data. The sensors parameter tells the
                * caller which data fields were actually populated with new data.
                * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
                * the FIFO isn't being filled with accel data.
                * The driver parses the gesture data to determine if a gesture
                * event has occurred; on an event, the application will be notified
                * via a callback (assuming that a callback function was properly
                * registered). The more parameter is non-zero if there are
                * leftover packets in the FIFO.
                */
            unsigned long sensor_timestamp;
            if(dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))continue;
            if (!more)
                hal.new_gyro = 0;
            /* Gyro and accel data are written to the FIFO by the DMP in chip
                * frame and hardware units. This behavior is convenient because it
                * keeps the gyro and accel outputs of dmp_read_fifo and
                * mpu_read_fifo consistent.
                */
            /*
            if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
                send_packet(PACKET_TYPE_GYRO, gyro);
            */
            /*
            if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
                send_packet(PACKET_TYPE_ACCEL, accel);
            */
            /* Unlike gyro and accel, quaternions are written to the FIFO in
                * the body frame, q30. The orientation is set by the scalar passed
                * to dmp_set_orientation during initialization.
                */
            /*
            if (sensors & INV_WXYZ_QUAT && hal.report & PRINT_QUAT)
                send_packet(PACKET_TYPE_QUAT, quat);
            */
        }
        /*
        else if (hal.new_gyro) {
            short gyro[3], accel[3];
            unsigned char sensors, more;
            mpu_read_fifo(gyro, accel, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO && hal.report & PRINT_GYRO)
                send_packet(PACKET_TYPE_GYRO, gyro);
            if (sensors & INV_XYZ_ACCEL && hal.report & PRINT_ACCEL)
                send_packet(PACKET_TYPE_ACCEL, accel);
        }
        */
        //update pitch, roll and yaw
        if(sensors & INV_XYZ_GYRO){
            int32_t i;
            for(i = 0; i < 4; ++i){
                q[i] = quat[i] * q30_inv;
            }
            if(calibration_device[mpuIndex]){
                printf("MPU %d in calibration\n", mpuIndex);
                if(calibration_cnt[mpuIndex] == 0){
                    q_cali[mpuIndex][0] = q[0];
                    q_cali[mpuIndex][1] = q[1];
                    q_cali[mpuIndex][2] = q[2];
                    q_cali[mpuIndex][3] = q[3];
                }
                int32_t j;
                int32_t stable = 1;
                for(j = 0; j < 4; ++j){
                    if(fabs(q[j] - q_cali[mpuIndex][j]) > 0.1)stable = 0; 
                }
                if(stable)calibration_cnt[mpuIndex] += 1;
                else calibration_cnt[mpuIndex] = 0;

                if(calibration_cnt[mpuIndex] > 100){
                    calibration_device[mpuIndex] = 0;
                    q_cali_inv[mpuIndex][0] = q_cali[mpuIndex][0];
                    q_cali_inv[mpuIndex][1] = -q_cali[mpuIndex][1];
                    q_cali_inv[mpuIndex][2] = -q_cali[mpuIndex][2];
                    q_cali_inv[mpuIndex][3] = -q_cali[mpuIndex][3];

                    MotorTurnOff(mpuIndex);
                }
            }
            if(!calibration_device[mpuIndex]){

                float p[4]; 
                p[0] = q[0] * q_cali_inv[mpuIndex][0] - q[1] * q_cali_inv[mpuIndex][1] - q[2] * q_cali_inv[mpuIndex][2] - q[3] * q_cali_inv[mpuIndex][3];
                p[1] = q[1] * q_cali_inv[mpuIndex][0] + q[0] * q_cali_inv[mpuIndex][1] + q[2] * q_cali_inv[mpuIndex][3] - q[3] * q_cali_inv[mpuIndex][2];
                p[2] = q[2] * q_cali_inv[mpuIndex][0] + q[0] * q_cali_inv[mpuIndex][2] + q[3] * q_cali_inv[mpuIndex][1] - q[1] * q_cali_inv[mpuIndex][3];
                p[3] = q[3] * q_cali_inv[mpuIndex][0] + q[0] * q_cali_inv[mpuIndex][3] + q[1] * q_cali_inv[mpuIndex][2] - q[2] * q_cali_inv[mpuIndex][1];

                pitch= asin(2 * p[0] * p[2] - 2 * p[1] * p[3]) * 57.3;
                roll= atan2(2 * p[2] * p[3] + 2 * p[0] * p[1], - 2 * p[1] * p[1] - 2 * p[2] * p[2] + 1) * 57.3;
                yaw= atan2(2 * p[1] * p[2] + 2 * p[0] * p[3], p[0] * p[0] + p[1] * p[1] - p[2] * p[2] - p[3] * p[3]) * 57.3;
                printf("Mpu %d:\n", mpuIndex);
                if(pitch > 15 || pitch < -15 || roll > 15 || roll < -15)MotorTurnOn(mpuIndex);
                else MotorTurnOff(mpuIndex);
                printf("Pitch: %d, Roll: %d, Yaw: %d\n", (int32_t)pitch, (int32_t)roll, (int32_t)yaw);
            }
        } 
    }

    return 0;
}
