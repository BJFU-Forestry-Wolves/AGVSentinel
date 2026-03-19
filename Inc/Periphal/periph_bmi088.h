/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_bmi088.h
 *  Description  : This file contains the functions of 
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-02-11 00:31:08
 */


#ifndef BMI088_PERIPH_H
#define BMI088_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "util_spi.h"
#include "util_gpio.h"
#include "reg_bmi088.h"

#define BMI088_ACCEL_WRITE_SINGLE_REG(reg, data)                        \
    {                                                                   \
        GPIO_Reset(CS_ACCEL);                                           \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, reg);               \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, data);              \
        GPIO_Set(CS_ACCEL);                                             \
    }
#define BMI088_ACCEL_READ_SINGLE_REG(reg, data)                         \
    {                                                                   \
        GPIO_Reset(CS_ACCEL);                                           \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, ((reg) | 0x80));    \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, 0x55);              \
        (data) = Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, 0x55);     \
        GPIO_Set(CS_ACCEL);                                             \
    }
#define BMI088_ACCEL_READ_MULI_REG(reg, data, len)                      \
    {                                                                   \
        GPIO_Reset(CS_ACCEL);                                           \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, ((reg) | 0x80));    \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, ((reg) | 0x80));    \
        Spi_ReadMuliReg(Const_BMI088_SPI_HANDLER, data, len);           \
        GPIO_Set(CS_ACCEL);                                             \
    }
#define BMI088_GYRO_WRITE_SINGLE_REG(reg, data)                         \
    {                                                                   \
        GPIO_Reset(CS_GYRO);                                            \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, reg);               \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, data);              \
        GPIO_Set(CS_GYRO);                                              \
    }
#define BMI088_GYRO_READ_SINGLE_REG(reg, data)                          \
    {                                                                   \
        GPIO_Reset(CS_GYRO);                                            \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, ((reg) | 0x80));    \
        (data) = Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, 0x55);     \
        GPIO_Set(CS_GYRO);                                              \
    }
#define BMI088_GYRO_READ_MULI_REG(reg, data, len)                       \
    {                                                                   \
        GPIO_Reset(CS_GYRO);                                            \
        Spi_SwapAbyteData(Const_BMI088_SPI_HANDLER, ((reg) | 0x80));    \
        Spi_ReadMuliReg(Const_BMI088_SPI_HANDLER, data, len);           \
        GPIO_Set(CS_GYRO);                                              \
    }

typedef enum {
    BMI088_STATE_NULL      = 0,
    BMI088_STATE_CONNECTED = 1,
    BMI088_STATE_LOST      = 2,
    BMI088_STATE_ERROR     = 3,
    BMI088_STATE_PENDING   = 4
} BMI088_BMI088StateEnum;

typedef struct {
    float x;
    float y;
    float z;
} BMI088_BMI088AccelTypeDef;

typedef struct {
    float yaw;
    float pitch;
    float row;
} BMI088_BMI088SpeedTypeDef;

typedef struct {
    BMI088_BMI088StateEnum state;
    
    BMI088_BMI088AccelTypeDef accel;        //  real data
    BMI088_BMI088SpeedTypeDef gyro;        //  real data

    float d_accel[3];
    float d_gyro[3];
    
    float gyro_offset[3];
    float accel_offset[3];
    float gNorm;
    float accelScale;
    float sensor_time;
    float temperature;

    uint8_t caliOffset;
    
    uint32_t last_update_time;
} BMI088_BMI088DataTypeDef;

BMI088_BMI088DataTypeDef* BMI088_GetBMI088DataPtr(void);
uint8_t BMI088_Init(uint8_t calibrate);
static uint8_t BMI088_BMI088AccelInit(void);
static uint8_t BMI088_BMI088GyroInit(void);
static uint8_t BMI088_BMI088AccelTest(void);
static uint8_t BMI088_BMI088GyroTest(void);
uint8_t BMI088_IsBMI088Offline(void);
void BMI088_SetOffset(void);
void BMI088_GetOffset(void);
void BMI088_BMI088DecodeData(void);
void BMI088_ResetBMI088Data(void);
void BMI088_BMI088TempReadOver(uint8_t *rx_buff);
void BMI088_BMI088AccelReadOver(uint8_t *rx_buff);
void BMI088_BMI088GyroReadOver(uint8_t *rx_buff);


#endif

#ifdef __cplusplus
}
#endif
