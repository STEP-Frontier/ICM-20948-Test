#ifndef ICM
#define ICM

#include "main.h"

typedef enum {
    ub_0 = 0 << 4,
	ub_1 = 1 << 4,
	ub_2 = 2 << 4,
	ub_3 = 3 << 4
} userbank; 

typedef enum {
	_2g,
	_4g,
	_8g,
	_16g
} accel_full_scale;

typedef enum {
	_250dps,
	_500dps,
	_1000dps,
	_2000dps
} gyro_full_scale;

typedef struct {
    float x; 
    float y; 
    float z; 
} vector;


/* Functions */
void ICM_init(); 
void accel_calibration(); 
void set_accel_full_scale(accel_full_scale fs); 
void get_raw_accel(vector* data); 
void get_accel(vector* data); 

void gyro_calibration(); 
void set_gyro_full_scale(gyro_full_scale fs);
void get_raw_gyro(vector* data); 
void get_gyro(vector* data); 


/* Register Map */
#define WHO_AM_I 0x00
#define REG_BANK_SEL 0x7F

// User Bank 0
#define USER_CTRL 0x03
#define PWR_MGMT_1 0x06
#define ACCEL_XOUT_H 0x2D
#define GYRO_XOUT_H 0x33

// User Bank1
#define XA_OFFS_H 0x14

// User Bank 2
#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define XG_OFFS_USRH 0x03
#define ODR_ALIGN_EN 0x09
#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_CONFIG 0x14

#endif