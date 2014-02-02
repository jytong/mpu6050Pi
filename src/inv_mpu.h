/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu.h
 *      @brief      An I2C-based driver for Invensense gyroscopes.
 *      @details    This driver currently works for the following devices:
 *                  MPU6050
 *                  MPU6500
 *                  MPU9150 (or MPU6050 w/ AK8975 on the auxiliary bus)
 *                  MPU9250 (or MPU6500 w/ AK8963 on the auxiliary bus)
 */

/****************************
 *	for Raspberry Pi v2
 *	by Jiang YuTong
 ****************************/
#ifdef __cplusplus
extern "C" {
#endif

#ifndef PIEXTERN
#define PIEXTERN extern
#endif

#define __MOTION_DRIVER_TARGET_RASPBERRY_PI
/*
	need library i2c-dev

	$sudo apt-get update
	$sudo apt-get upgrade
	$sudo apt-get install libi2c-dev
	$sudo apt-get install i2c-tools
*/
#ifdef __MOTION_DRIVER_TARGET_RASPBERRY_PI
#define MPU6050
// #define MPU9150	// for MPU9150
#endif
/*********  end  ************/

#ifndef _INV_MPU_H_
#define _INV_MPU_H_

#define INV_X_GYRO      (0x40)
#define INV_Y_GYRO      (0x20)
#define INV_Z_GYRO      (0x10)
#define INV_XYZ_GYRO    (INV_X_GYRO | INV_Y_GYRO | INV_Z_GYRO)
#define INV_XYZ_ACCEL   (0x08)
#define INV_XYZ_COMPASS (0x01)

struct int_param_s {
#if defined EMPL_TARGET_MSP430 || defined MOTION_DRIVER_TARGET_MSP430
    void (*cb)(void);
    unsigned short pin;
    unsigned char lp_exit;
    unsigned char active_low;
#elif defined EMPL_TARGET_UC3L0
    unsigned long pin;
    void (*cb)(volatile void*);
    void *arg;
#endif
};

#define MPU_INT_STATUS_DATA_READY       (0x0001)
#define MPU_INT_STATUS_DMP              (0x0002)
#define MPU_INT_STATUS_PLL_READY        (0x0004)
#define MPU_INT_STATUS_I2C_MST          (0x0008)
#define MPU_INT_STATUS_FIFO_OVERFLOW    (0x0010)
#define MPU_INT_STATUS_ZMOT             (0x0020)
#define MPU_INT_STATUS_MOT              (0x0040)
#define MPU_INT_STATUS_FREE_FALL        (0x0080)
#define MPU_INT_STATUS_DMP_0            (0x0100)
#define MPU_INT_STATUS_DMP_1            (0x0200)
#define MPU_INT_STATUS_DMP_2            (0x0400)
#define MPU_INT_STATUS_DMP_3            (0x0800)
#define MPU_INT_STATUS_DMP_4            (0x1000)
#define MPU_INT_STATUS_DMP_5            (0x2000)

/* Set up APIs */
#ifdef __MOTION_DRIVER_TARGET_RASPBERRY_PI
int PIEXTERN mpu_init();
#else
int PIEXTERN mpu_init(struct int_param_s *int_param);
#endif
int PIEXTERN mpu_init_slave(void);
int PIEXTERN mpu_set_bypass(unsigned char bypass_on);

/* Configuration APIs */
int PIEXTERN mpu_lp_accel_mode(unsigned char rate);
int PIEXTERN mpu_lp_motion_interrupt(unsigned short thresh, unsigned char time, unsigned char lpa_freq);
int PIEXTERN mpu_set_int_level(unsigned char active_low);
int PIEXTERN mpu_set_int_latched(unsigned char enable);

int PIEXTERN mpu_set_dmp_state(unsigned char enable);
int PIEXTERN mpu_get_dmp_state(unsigned char *enabled);

int PIEXTERN mpu_get_lpf(unsigned short *lpf);
int PIEXTERN mpu_set_lpf(unsigned short lpf);

int PIEXTERN mpu_get_gyro_fsr(unsigned short *fsr);
int PIEXTERN mpu_set_gyro_fsr(unsigned short fsr);

int PIEXTERN mpu_get_accel_fsr(unsigned char *fsr);
int PIEXTERN mpu_set_accel_fsr(unsigned char fsr);

int PIEXTERN mpu_get_compass_fsr(unsigned short *fsr);

int PIEXTERN mpu_get_gyro_sens(float *sens);
int PIEXTERN mpu_get_accel_sens(unsigned short *sens);

int PIEXTERN mpu_get_sample_rate(unsigned short *rate);
int PIEXTERN mpu_set_sample_rate(unsigned short rate);
int PIEXTERN mpu_get_compass_sample_rate(unsigned short *rate);
int PIEXTERN mpu_set_compass_sample_rate(unsigned short rate);

int PIEXTERN mpu_get_fifo_config(unsigned char *sensors);
int PIEXTERN mpu_configure_fifo(unsigned char sensors);

int PIEXTERN mpu_get_power_state(unsigned char *power_on);
int PIEXTERN mpu_set_sensors(unsigned char sensors);

int PIEXTERN mpu_set_accel_bias(const long *accel_bias);

/* Data getter/setter APIs */
int PIEXTERN mpu_get_gyro_reg(short *data, unsigned long *timestamp);
int PIEXTERN mpu_get_accel_reg(short *data, unsigned long *timestamp);
int PIEXTERN mpu_get_compass_reg(short *data, unsigned long *timestamp);
int PIEXTERN mpu_get_temperature(long *data, unsigned long *timestamp);

int PIEXTERN mpu_get_int_status(short *status);
int PIEXTERN mpu_read_fifo(short *gyro, short *accel, unsigned long *timestamp, unsigned char *sensors, unsigned char *more);
int PIEXTERN mpu_read_fifo_stream(unsigned short length, unsigned char *data, unsigned char *more);
int PIEXTERN mpu_reset_fifo(void);

int PIEXTERN mpu_write_mem(unsigned short mem_addr, unsigned short length, unsigned char *data);
int PIEXTERN mpu_read_mem(unsigned short mem_addr, unsigned short length, unsigned char *data);
int PIEXTERN mpu_load_firmware(unsigned short length, const unsigned char *firmware,
            unsigned short start_addr, unsigned short sample_rate);

int PIEXTERN mpu_reg_dump(void);
int PIEXTERN mpu_read_reg(unsigned char reg, unsigned char *data);
int PIEXTERN mpu_run_self_test(long *gyro, long *accel);
int PIEXTERN mpu_register_tap_cb(void (*func)(unsigned char, unsigned char));

#ifdef __MOTION_DRIVER_TARGET_RASPBERRY_PI
int PIEXTERN __mpu_get_device_addr( unsigned char *p_pData );
int PIEXTERN __mpu_get_temperature( float *p_fTemperature );
int PIEXTERN __mpu_get_clock_sel( unsigned char *p_pData );
int PIEXTERN __mpu_get_sleep_mode( unsigned char *p_pData );
int PIEXTERN __mpu_set_sleep_mode( unsigned char p_ucVal );
int PIEXTERN __mpu_get_fifo_bytes_count( unsigned short *p_usCount );
int PIEXTERN __mpu_drop_fifo_data( int len );
// p_piSize in/out  in: buffer length; out: got data len
// return: 0 success; <0 fail
int PIEXTERN __mpu_reg_dump( unsigned char *p_pBuff, int *p_piSize );
int PIEXTERN __mpu_set_user_x_gyro_offset( short p_sVal );
int PIEXTERN __mpu_set_user_y_gyro_offset( short p_sVal );
int PIEXTERN __mpu_set_user_z_gyro_offset( short p_sVal );
int PIEXTERN __mpu_set_x_accel_offset( short p_sVal );
int PIEXTERN __mpu_set_y_accel_offset( short p_sVal );
int PIEXTERN __mpu_set_z_accel_offset( short p_sVal );
#endif	// __MOTION_DRIVER_TARGET_RASPBERRY_PI

#ifdef __cplusplus
}
#endif

#endif  /* #ifndef _INV_MPU_H_ */

