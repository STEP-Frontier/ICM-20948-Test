#include "icm.h"

extern SPI_HandleTypeDef hspi1; 

/* Define private functions and variables*/
static void cs_low(); 
static void cs_high(); 
static uint8_t readdata(userbank ub, uint8_t adr); 
static uint8_t* readdatas(userbank ub, uint8_t adr, uint8_t len); 
static void writedata(userbank ub, uint8_t adr, uint8_t data); 
static void writedatas(userbank ub, uint8_t adr, uint8_t* data, uint8_t len); 
static void select_user_bank(userbank ub); 

static float accel_scale_factor; 
static float gyro_scale_factor; 


void ICM_init() {
    uint8_t temp; 

    while(!(readdata(ub_0, WHO_AM_I) == 0xEA));  // p36

     // Reset p37
    writedata(ub_0, PWR_MGMT_1, 0b11000001); 
    HAL_Delay(100); 

    // Wakeup and set clock source
    temp = readdata(ub_0, PWR_MGMT_1); 
    temp &= 0b10111111; 
    writedata(ub_0, PWR_MGMT_1, temp); 
    HAL_Delay(100);
    temp = readdata(ub_0, PWR_MGMT_1); 
    temp |= 0b00000001; 
    writedata(ub_0, PWR_MGMT_1, temp); 

    // Enable ODR alignment p63
    writedata(ub_0, ODR_ALIGN_EN, 0b00000001); 

    // Enable SPI p36
    temp = readdata(ub_0, USER_CTRL); 
    temp |= 0b00010000; 
    writedata(ub_0, USER_CTRL, temp); 

    // Set accel low pass filter p64
    temp = readdata(ub_2, ACCEL_CONFIG); 
    temp |= 0 << 3;  // Should be changeable
    writedata(ub_2, ACCEL_CONFIG, temp); 

    // Set gyro low pass filter p59
    temp = readdata(ub_2, GYRO_CONFIG_1); 
    temp |= 0 << 3;  // Should be changeable
    writedata(ub_2, GYRO_CONFIG_1, temp); 

    // Set accel sample rate divider p63
    writedata(ub_2, ACCEL_SMPLRT_DIV_1, 0);  // Should be changeable
	writedata(ub_2, ACCEL_SMPLRT_DIV_2, 0);
    
    // Set gyro sample rate divider p59
    writedata(ub_2, GYRO_SMPLRT_DIV, 0);  // Should be changeable

    // Calibration
    accel_calibration(); 
    gyro_calibration(); 

    // Set full scale
    set_accel_full_scale(_16g); 
    set_gyro_full_scale(_2000dps); 

}

void accel_calibration() {
    vector temp;
    int32_t accel_bias[3] = {0}; 
    uint8_t cur_offset[6] = {0}; 

    // Get 100 data
    for(int i = 0; i < 100; i++) {
        get_raw_accel(&temp);
        accel_bias[0] += temp.x;
        accel_bias[1] += temp.y;
        accel_bias[2] += temp.z;
    }

    for (int i = 0; i < 3; i++) {
        // Get average bias
        accel_bias[i] /= 100;

        // Read old bias
        uint8_t* prev_offset = readdatas(ub_1, XA_OFFS_H + i * 2, 2); 
        int32_t prev_accel_bias = (int32_t)(prev_offset[0] << 8 | prev_offset[1]);
        uint8_t mask_bit = prev_offset[1] & 0x01; 

        // Calculate new bias
        int32_t cur_accel_bias = prev_accel_bias -(accel_bias[i] / 8); 
        cur_offset[i * 2] = (cur_accel_bias >> 8) & 0xFF; 
        cur_offset[i * 2 + 1] = (cur_accel_bias & 0xFE) | mask_bit; 

        // Save new bias
        writedatas(ub_1, XA_OFFS_H + i * 2, &cur_offset[i * 2], 2); 
    }
}

void set_accel_full_scale(accel_full_scale fs) {
    uint8_t val = readdata(ub_2, ACCEL_CONFIG);
	
	switch(fs) {
		case _2g :
			val |= 0x00;
			accel_scale_factor = 16384;
			break;
		case _4g :
			val |= 0x02;
			accel_scale_factor = 8192;
			break;
		case _8g :
			val |= 0x04;
			accel_scale_factor = 4096;
			break;
		case _16g :
			val |= 0x06;
			accel_scale_factor = 2048;
			break;
	}

	writedata(ub_2, ACCEL_CONFIG, val); 
}

void get_raw_accel(vector* data) { 
    uint8_t* buffer = readdatas(ub_0, ACCEL_XOUT_H, 6); 
    data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->z = (int16_t)((buffer[4] << 8) | buffer[5]);
}

void get_accel(vector* data) {
    get_raw_accel(data); 
    data->x /= accel_scale_factor;
	data->y /= accel_scale_factor;
	data->z /= accel_scale_factor;
}



void gyro_calibration() {
    vector temp;
    int32_t gyro_bias[3] = {0}; 
    uint8_t cur_offset[6] = {0}; 

    // Get 100 data
    for(int i = 0; i < 100; i++) {
        get_raw_accel(&temp);
        gyro_bias[0] += temp.x;
        gyro_bias[1] += temp.y;
        gyro_bias[2] += temp.z;
    }

    for (int i = 0; i < 3; i++) {
        // Get average bias
        gyro_bias[i] /= 100;

        // Calculate new bias
        cur_offset[i * 2] = (-gyro_bias[i] / 4 >> 8) & 0xFF; 
        cur_offset[i * 2 + 1] = (-gyro_bias[i] / 4) & 0xFF; 
    }

    // Save new bias
    writedatas(ub_2, XG_OFFS_USRH, cur_offset, 6); 
}

void set_gyro_full_scale(gyro_full_scale full_scale) {
	uint8_t val = readdata(ub_2, GYRO_CONFIG_1);
	
	switch(full_scale) {
		case _250dps :
			val |= 0x00;
			gyro_scale_factor = 131.0;
			break;
		case _500dps :
			val |= 0x02;
			gyro_scale_factor = 65.5;
			break;
		case _1000dps :
			val |= 0x04;
			gyro_scale_factor = 32.8;
			break;
		case _2000dps :
			val |= 0x06;
			gyro_scale_factor = 16.4;
			break;
	}

	writedata(ub_2, GYRO_CONFIG_1, val);
}

void get_raw_gyro(vector* data) { 
    uint8_t* buffer = readdatas(ub_0, GYRO_XOUT_H, 6); 
    data->x = (int16_t)((buffer[0] << 8) | buffer[1]);
    data->y = (int16_t)((buffer[2] << 8) | buffer[3]);
    data->z = (int16_t)((buffer[4] << 8) | buffer[5]);
}

void get_gyro(vector* data) {
    get_raw_gyro(data); 
    data->x /= gyro_scale_factor;
	data->y /= gyro_scale_factor;
	data->z /= gyro_scale_factor;
}



/* Static functions */
static void cs_low() {
    HAL_GPIO_WritePin(GPIOB, ICM_CS, GPIO_PIN_RESET); 
}

static void cs_high() {
     HAL_GPIO_WritePin(GPIOB, ICM_CS, GPIO_PIN_SET); 
}

static void select_user_bank(userbank ub) {
    uint8_t reg = REG_BANK_SEL & 0x7F; 
    
    cs_low(); 

    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, &ub, 1, HAL_MAX_DELAY);

    cs_high(); 
}

// p31
static uint8_t readdata(userbank ub, uint8_t adr) {
    select_user_bank(ub); 

    uint8_t reg = adr | 0x80; 
    uint8_t val;

    cs_low(); 

    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY); 
    HAL_SPI_Receive(&hspi1, &val, 1, HAL_MAX_DELAY);

    cs_high();

    return val; 
}

static uint8_t* readdatas(userbank ub, uint8_t adr, uint8_t len) {
    select_user_bank(ub); 

    uint8_t reg = adr | 0x80; 
    static uint8_t val[6];

    cs_low(); 

    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY); 
    HAL_SPI_Receive(&hspi1, val, len, HAL_MAX_DELAY);

    cs_high(); 

    return val; 
}

static void writedata(userbank ub, uint8_t adr, uint8_t data) {
    select_user_bank(ub); 

    uint8_t reg = adr & 0x7F; 
    
    cs_low(); 

    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);

    cs_high(); 
}

static void writedatas(userbank ub, uint8_t adr, uint8_t* data, uint8_t len) {
    select_user_bank(ub); 

    uint8_t reg = adr & 0x7F; 
    
    cs_low(); 

    HAL_SPI_Transmit(&hspi1, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, data, len, HAL_MAX_DELAY);

    cs_high(); 
}
