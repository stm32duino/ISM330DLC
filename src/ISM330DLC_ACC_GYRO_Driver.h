/**
 ******************************************************************************
 * @file    ISM330DLC_ACC_GYRO_driver.h
 * @author  MEMS Application Team
 * @version V1.0.0
 * @date    13-January-2020
 * @brief   ISM330DLC header driver file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ISM330DLC_ACC_GYRO_DRIVER__H
#define __ISM330DLC_ACC_GYRO_DRIVER__H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
/* Exported types ------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

//these could change accordingly with the architecture

#ifndef __ARCHDEP__TYPES
#define __ARCHDEP__TYPES

typedef unsigned char u8_t;
typedef unsigned short int u16_t;
typedef unsigned int u32_t;
typedef int i32_t;
typedef short int i16_t;
typedef signed char i8_t;

#endif /*__ARCHDEP__TYPES*/

/* Exported common structure --------------------------------------------------------*/

#ifndef __SHARED__TYPES
#define __SHARED__TYPES

typedef union{
	i16_t i16bit[3];
	u8_t u8bit[6];
} Type3Axis16bit_U;

typedef union{
	i16_t i16bit;
	u8_t u8bit[2];
} Type1Axis16bit_U;

typedef union{
	i32_t i32bit;
	u8_t u8bit[4];
} Type1Axis32bit_U;

typedef enum {
  MEMS_SUCCESS				=		0x01,
  MEMS_ERROR				=		0x00
} mems_status_t;

#endif /*__SHARED__TYPES*/

/* Exported macro ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/************** I2C Address *****************/

#define ISM330DLC_ACC_GYRO_I2C_ADDRESS_LOW   0xD4  // SAD[0] = 0
#define ISM330DLC_ACC_GYRO_I2C_ADDRESS_HIGH  0xD6  // SAD[0] = 1

/************** Who am I  *******************/

#define ISM330DLC_ACC_GYRO_WHO_AM_I         0x6A

/************** Device Register  *******************/

#define ISM330DLC_ACC_GYRO_FUNC_CFG_ACCESS  	0X01

#define ISM330DLC_ACC_GYRO_SENSOR_SYNC_TIME  	0X04
#define ISM330DLC_ACC_GYRO_SENSOR_RES_RATIO  	0X05

#define ISM330DLC_ACC_GYRO_FIFO_CTRL1  	0X06
#define ISM330DLC_ACC_GYRO_FIFO_CTRL2  	0X07
#define ISM330DLC_ACC_GYRO_FIFO_CTRL3  	0X08
#define ISM330DLC_ACC_GYRO_FIFO_CTRL4  	0X09
#define ISM330DLC_ACC_GYRO_FIFO_CTRL5  	0X0A

#define ISM330DLC_ACC_GYRO_DRDY_PULSE_CFG_G  	0X0B
#define ISM330DLC_ACC_GYRO_INT1_CTRL  	0X0D
#define ISM330DLC_ACC_GYRO_INT2_CTRL  	0X0E
#define ISM330DLC_ACC_GYRO_WHO_AM_I_REG  	0X0F
#define ISM330DLC_ACC_GYRO_CTRL1_XL  	0X10
#define ISM330DLC_ACC_GYRO_CTRL2_G  	0X11
#define ISM330DLC_ACC_GYRO_CTRL3_C  	0X12
#define ISM330DLC_ACC_GYRO_CTRL4_C  	0X13
#define ISM330DLC_ACC_GYRO_CTRL5_C  	0X14
#define ISM330DLC_ACC_GYRO_CTRL6_G  	0X15
#define ISM330DLC_ACC_GYRO_CTRL7_G  	0X16
#define ISM330DLC_ACC_GYRO_CTRL8_XL  	0X17
#define ISM330DLC_ACC_GYRO_CTRL9_XL  	0X18
#define ISM330DLC_ACC_GYRO_CTRL10_C  	0X19

#define ISM330DLC_ACC_GYRO_MASTER_CONFIG  	0X1A
#define ISM330DLC_ACC_GYRO_WAKE_UP_SRC  	0X1B
#define ISM330DLC_ACC_GYRO_TAP_SRC  	0X1C
#define ISM330DLC_ACC_GYRO_D6D_SRC  	0X1D
#define ISM330DLC_ACC_GYRO_STATUS_REG  	0X1E

#define ISM330DLC_ACC_GYRO_OUT_TEMP_L  	0X20
#define ISM330DLC_ACC_GYRO_OUT_TEMP_H  	0X21
#define ISM330DLC_ACC_GYRO_OUTX_L_G  	0X22
#define ISM330DLC_ACC_GYRO_OUTX_H_G  	0X23
#define ISM330DLC_ACC_GYRO_OUTY_L_G  	0X24
#define ISM330DLC_ACC_GYRO_OUTY_H_G  	0X25
#define ISM330DLC_ACC_GYRO_OUTZ_L_G  	0X26
#define ISM330DLC_ACC_GYRO_OUTZ_H_G  	0X27
#define ISM330DLC_ACC_GYRO_OUTX_L_XL  	0X28
#define ISM330DLC_ACC_GYRO_OUTX_H_XL  	0X29
#define ISM330DLC_ACC_GYRO_OUTY_L_XL  	0X2A
#define ISM330DLC_ACC_GYRO_OUTY_H_XL  	0X2B
#define ISM330DLC_ACC_GYRO_OUTZ_L_XL  	0X2C
#define ISM330DLC_ACC_GYRO_OUTZ_H_XL  	0X2D
#define ISM330DLC_ACC_GYRO_SENSORHUB1_REG  	0X2E
#define ISM330DLC_ACC_GYRO_SENSORHUB2_REG  	0X2F
#define ISM330DLC_ACC_GYRO_SENSORHUB3_REG  	0X30
#define ISM330DLC_ACC_GYRO_SENSORHUB4_REG  	0X31
#define ISM330DLC_ACC_GYRO_SENSORHUB5_REG  	0X32
#define ISM330DLC_ACC_GYRO_SENSORHUB6_REG  	0X33
#define ISM330DLC_ACC_GYRO_SENSORHUB7_REG  	0X34
#define ISM330DLC_ACC_GYRO_SENSORHUB8_REG  	0X35
#define ISM330DLC_ACC_GYRO_SENSORHUB9_REG  	0X36
#define ISM330DLC_ACC_GYRO_SENSORHUB10_REG  	0X37
#define ISM330DLC_ACC_GYRO_SENSORHUB11_REG  	0X38
#define ISM330DLC_ACC_GYRO_SENSORHUB12_REG  	0X39
#define ISM330DLC_ACC_GYRO_FIFO_STATUS1  	0X3A
#define ISM330DLC_ACC_GYRO_FIFO_STATUS2  	0X3B
#define ISM330DLC_ACC_GYRO_FIFO_STATUS3  	0X3C
#define ISM330DLC_ACC_GYRO_FIFO_STATUS4  	0X3D
#define ISM330DLC_ACC_GYRO_FIFO_DATA_OUT_L  	0X3E
#define ISM330DLC_ACC_GYRO_FIFO_DATA_OUT_H  	0X3F
#define ISM330DLC_ACC_GYRO_TIMESTAMP0_REG  	0X40
#define ISM330DLC_ACC_GYRO_TIMESTAMP1_REG  	0X41
#define ISM330DLC_ACC_GYRO_TIMESTAMP2_REG  	0X42

#define ISM330DLC_ACC_GYRO_SENSORHUB13_REG  	0X4D
#define ISM330DLC_ACC_GYRO_SENSORHUB14_REG  	0X4E
#define ISM330DLC_ACC_GYRO_SENSORHUB15_REG  	0X4F
#define ISM330DLC_ACC_GYRO_SENSORHUB16_REG  	0X50
#define ISM330DLC_ACC_GYRO_SENSORHUB17_REG  	0X51
#define ISM330DLC_ACC_GYRO_SENSORHUB18_REG  	0X52

#define ISM330DLC_ACC_GYRO_FUNC_SRC1  	0X53
#define ISM330DLC_ACC_GYRO_FUNC_SRC2  	0X54
#define ISM330DLC_ACC_GYRO_TAP_CFG  	0X58
#define ISM330DLC_ACC_GYRO_TAP_THS_6D  	0X59
#define ISM330DLC_ACC_GYRO_INT_DUR2  	0X5A
#define ISM330DLC_ACC_GYRO_WAKE_UP_THS  	0X5B
#define ISM330DLC_ACC_GYRO_WAKE_UP_DUR  	0X5C
#define ISM330DLC_ACC_GYRO_FREE_FALL  	0X5D
#define ISM330DLC_ACC_GYRO_MD1_CFG  	0X5E
#define ISM330DLC_ACC_GYRO_MD2_CFG  	0X5F

#define ISM330DLC_ACC_GYRO_MASTER_CMD_CODE  		0X60
#define ISM330DLC_ACC_GYRO_SENS_SYNC_SPI_ERR_CODE  	0X61

#define ISM330DLC_ACC_GYRO_OUT_MAG_RAW_X_L  	0X66
#define ISM330DLC_ACC_GYRO_OUT_MAG_RAW_X_H  	0X67
#define ISM330DLC_ACC_GYRO_OUT_MAG_RAW_Y_L  	0X68
#define ISM330DLC_ACC_GYRO_OUT_MAG_RAW_Y_H  	0X69
#define ISM330DLC_ACC_GYRO_OUT_MAG_RAW_Z_L  	0X6A
#define ISM330DLC_ACC_GYRO_OUT_MAG_RAW_Z_H  	0X6B

#define ISM330DLC_ACC_GYRO_INT_OIS  	0X6F
#define ISM330DLC_ACC_GYRO_CTRL1_OIS  	0X70
#define ISM330DLC_ACC_GYRO_CTRL2_OIS  	0X71
#define ISM330DLC_ACC_GYRO_CTRL3_OIS  	0X72

#define ISM330DLC_ACC_GYRO_X_OFS_USR  	0X73
#define ISM330DLC_ACC_GYRO_Y_OFS_USR  	0X74
#define ISM330DLC_ACC_GYRO_Z_OFS_USR  	0X75

/************** Embedded functions register mapping  *******************/
#define ISM330DLC_ACC_GYRO_SLV0_ADD                     0x02
#define ISM330DLC_ACC_GYRO_SLV0_SUBADD                  0x03
#define ISM330DLC_ACC_GYRO_SLAVE0_CONFIG                0x04
#define ISM330DLC_ACC_GYRO_SLV1_ADD                     0x05
#define ISM330DLC_ACC_GYRO_SLV1_SUBADD                  0x06
#define ISM330DLC_ACC_GYRO_SLAVE1_CONFIG                0x07
#define ISM330DLC_ACC_GYRO_SLV2_ADD                     0x08
#define ISM330DLC_ACC_GYRO_SLV2_SUBADD                  0x09
#define ISM330DLC_ACC_GYRO_SLAVE2_CONFIG                0x0A
#define ISM330DLC_ACC_GYRO_SLV3_ADD                     0x0B
#define ISM330DLC_ACC_GYRO_SLV3_SUBADD                  0x0C
#define ISM330DLC_ACC_GYRO_SLAVE3_CONFIG                0x0D
#define ISM330DLC_ACC_GYRO_DATAWRITE_SRC_MODE_SUB_SLV0  0x0E
#define ISM330DLC_ACC_GYRO_CONFIG_PEDO_THS_MIN          0x0F

#define ISM330DLC_ACC_GYRO_SM_STEP_THS                  0x13
#define ISM330DLC_ACC_GYRO_PEDO_DEB_REG                0x14
#define ISM330DLC_ACC_GYRO_STEP_COUNT_DELTA            0x15

#define ISM330DLC_ACC_GYRO_MAG_SI_XX                    0x24
#define ISM330DLC_ACC_GYRO_MAG_SI_XY                    0x25
#define ISM330DLC_ACC_GYRO_MAG_SI_XZ                    0x26
#define ISM330DLC_ACC_GYRO_MAG_SI_YX                    0x27
#define ISM330DLC_ACC_GYRO_MAG_SI_YY                    0x28
#define ISM330DLC_ACC_GYRO_MAG_SI_YZ                    0x29
#define ISM330DLC_ACC_GYRO_MAG_SI_ZX                    0x2A
#define ISM330DLC_ACC_GYRO_MAG_SI_ZY                    0x2B
#define ISM330DLC_ACC_GYRO_MAG_SI_ZZ                    0x2C
#define ISM330DLC_ACC_GYRO_MAG_OFFX_L                   0x2D
#define ISM330DLC_ACC_GYRO_MAG_OFFX_H                   0x2E
#define ISM330DLC_ACC_GYRO_MAG_OFFY_L                   0x2F
#define ISM330DLC_ACC_GYRO_MAG_OFFY_H                   0x30
#define ISM330DLC_ACC_GYRO_MAG_OFFZ_L                   0x31
#define ISM330DLC_ACC_GYRO_MAG_OFFZ_H                   0x32

/************** Generic Function  *******************/

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : W
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_WriteReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/*******************************************************************************
* Register      : Generic - All
* Address       : Generic - All
* Bit Group Name: None
* Permission    : R
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_ReadReg( void *handle, u8_t Reg, u8_t *Bufp, u16_t len );

/**************** Base Function  *******************/

/*******************************************************************************
* Register      : WHO_AM_I
* Address       : 0X0F
* Bit Group Name: WHO_AM_I_BIT
* Permission    : RO
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_WHO_AM_I_BIT_MASK  	0xFF
#define  	ISM330DLC_ACC_GYRO_WHO_AM_I_BIT_POSITION  	0
mems_status_t ISM330DLC_ACC_GYRO_R_WHO_AM_I(void *handle, u8_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: BDU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_BDU_CONTINUOS 			 =0x00,
  	ISM330DLC_ACC_GYRO_BDU_BLOCK_UPDATE 		 =0x40,
} ISM330DLC_ACC_GYRO_BDU_t;

#define  	ISM330DLC_ACC_GYRO_BDU_MASK  	0x40
mems_status_t ISM330DLC_ACC_GYRO_W_BDU(void *handle, ISM330DLC_ACC_GYRO_BDU_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_BDU(void *handle, ISM330DLC_ACC_GYRO_BDU_t *value);

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: FS_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_FS_XL_2g 		 =0x00,
  	ISM330DLC_ACC_GYRO_FS_XL_16g 		 =0x04,
  	ISM330DLC_ACC_GYRO_FS_XL_4g 		 =0x08,
  	ISM330DLC_ACC_GYRO_FS_XL_8g 		 =0x0C,
} ISM330DLC_ACC_GYRO_FS_XL_t;

#define  	ISM330DLC_ACC_GYRO_FS_XL_MASK  	0x0C
mems_status_t ISM330DLC_ACC_GYRO_W_FS_XL(void *handle, ISM330DLC_ACC_GYRO_FS_XL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FS_XL(void *handle, ISM330DLC_ACC_GYRO_FS_XL_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetAccData
* Permission    : RO
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_GetRawAccData(void *handle, u8_t *buff);
mems_status_t ISM330DLC_ACC_Get_Acceleration(void *handle, int *buff, u8_t from_fifo);

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: ODR_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_ODR_XL_POWER_DOWN 	 =0x00,
  	ISM330DLC_ACC_GYRO_ODR_XL_1Hz6 			 =0xB0,
  	ISM330DLC_ACC_GYRO_ODR_XL_12Hz5 		 =0x10,
  	ISM330DLC_ACC_GYRO_ODR_XL_26Hz 			 =0x20,
  	ISM330DLC_ACC_GYRO_ODR_XL_52Hz 			 =0x30,
  	ISM330DLC_ACC_GYRO_ODR_XL_104Hz 		 =0x40,
  	ISM330DLC_ACC_GYRO_ODR_XL_208Hz 		 =0x50,
  	ISM330DLC_ACC_GYRO_ODR_XL_416Hz 		 =0x60,
  	ISM330DLC_ACC_GYRO_ODR_XL_833Hz 		 =0x70,
  	ISM330DLC_ACC_GYRO_ODR_XL_1660Hz 		 =0x80,
  	ISM330DLC_ACC_GYRO_ODR_XL_3330Hz 		 =0x90,
  	ISM330DLC_ACC_GYRO_ODR_XL_6660Hz 		 =0xA0,
} ISM330DLC_ACC_GYRO_ODR_XL_t;

#define  	ISM330DLC_ACC_GYRO_ODR_XL_MASK  	0xF0
mems_status_t ISM330DLC_ACC_GYRO_W_ODR_XL(void *handle, ISM330DLC_ACC_GYRO_ODR_XL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_ODR_XL(void *handle, ISM330DLC_ACC_GYRO_ODR_XL_t *value);
mems_status_t ISM330DLC_ACC_GYRO_translate_ODR_XL(ISM330DLC_ACC_GYRO_ODR_XL_t value, u16_t *odr_hz_val);

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: FS_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_FS_G_250dps 			 =0x00,
  	ISM330DLC_ACC_GYRO_FS_G_500dps 			 =0x04,
  	ISM330DLC_ACC_GYRO_FS_G_1000dps 		 =0x08,
  	ISM330DLC_ACC_GYRO_FS_G_2000dps 		 =0x0C,
} ISM330DLC_ACC_GYRO_FS_G_t;

#define  	ISM330DLC_ACC_GYRO_FS_G_MASK  	0x0C
mems_status_t ISM330DLC_ACC_GYRO_W_FS_G(void *handle, ISM330DLC_ACC_GYRO_FS_G_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FS_G(void *handle, ISM330DLC_ACC_GYRO_FS_G_t *value);

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: ODR_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_ODR_G_POWER_DOWN 	 =0x00,
  	ISM330DLC_ACC_GYRO_ODR_G_12Hz5 			 =0x10,
  	ISM330DLC_ACC_GYRO_ODR_G_26Hz 			 =0x20,
  	ISM330DLC_ACC_GYRO_ODR_G_52Hz 			 =0x30,
  	ISM330DLC_ACC_GYRO_ODR_G_104Hz 			 =0x40,
  	ISM330DLC_ACC_GYRO_ODR_G_208Hz 			 =0x50,
  	ISM330DLC_ACC_GYRO_ODR_G_416Hz 			 =0x60,
  	ISM330DLC_ACC_GYRO_ODR_G_833Hz 			 =0x70,
  	ISM330DLC_ACC_GYRO_ODR_G_1660Hz 		 =0x80,
  	ISM330DLC_ACC_GYRO_ODR_G_3330Hz 		 =0x90,
  	ISM330DLC_ACC_GYRO_ODR_G_6660Hz 		 =0xA0,
} ISM330DLC_ACC_GYRO_ODR_G_t;

#define  	ISM330DLC_ACC_GYRO_ODR_G_MASK  	0xF0
mems_status_t ISM330DLC_ACC_GYRO_W_ODR_G(void *handle, ISM330DLC_ACC_GYRO_ODR_G_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_ODR_G(void *handle, ISM330DLC_ACC_GYRO_ODR_G_t *value);
mems_status_t ISM330DLC_ACC_GYRO_translate_ODR_G(ISM330DLC_ACC_GYRO_ODR_G_t value, u16_t *odr_hz_val);

/*******************************************************************************
* Register      : OUT_TEMP_L & OUT_TEMP_H
* Address       : 0X20 & 0x21
* Permission    : RO
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_GetRawTempData(void *handle, u16_t *buff);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetGyroData
* Permission    : RO
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_GetRawGyroData(void *handle, u8_t *buff);
mems_status_t ISM330DLC_ACC_Get_AngularRate(void *handle, int *buff, u8_t from_fifo);

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: BW0_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_BW0_XL_1_5KHZ 		 =0x00,
  	ISM330DLC_ACC_GYRO_BW0_XL_400HZ 		 =0x01,
} ISM330DLC_ACC_GYRO_BW0_XL_t;

#define  	ISM330DLC_ACC_GYRO_BW0_XL_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_W_BW0_XL(void *handle, ISM330DLC_ACC_GYRO_BW0_XL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_BW0_XL(void *handle, ISM330DLC_ACC_GYRO_BW0_XL_t *value);

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: BW_SEL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_BW_SEL_ODR2 		 =0x00,
  	ISM330DLC_ACC_GYRO_BW_SEL_ODR4 		 =0x02,
} ISM330DLC_ACC_GYRO_BW_SEL_t;

#define  	ISM330DLC_ACC_GYRO_BW_SEL_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_W_BW_SEL(void *handle, ISM330DLC_ACC_GYRO_BW_SEL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_BW_SEL(void *handle, ISM330DLC_ACC_GYRO_BW_SEL_t *value);

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: FS_125
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_FS_125_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_FS_125_ENABLED 		 =0x02,
} ISM330DLC_ACC_GYRO_FS_125_t;

#define  	ISM330DLC_ACC_GYRO_FS_125_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_W_FS_125(void *handle, ISM330DLC_ACC_GYRO_FS_125_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FS_125(void *handle, ISM330DLC_ACC_GYRO_FS_125_t *value);

/**************** Advanced Function  *******************/

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: BLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_BLE_LSB 		 =0x00,
  	ISM330DLC_ACC_GYRO_BLE_MSB 		 =0x02,
} ISM330DLC_ACC_GYRO_BLE_t;

#define  	ISM330DLC_ACC_GYRO_BLE_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_W_BLE(void *handle, ISM330DLC_ACC_GYRO_BLE_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_BLE(void *handle, ISM330DLC_ACC_GYRO_BLE_t *value);

/*******************************************************************************
* Register      : FUNC_CFG_ACCESS
* Address       : 0X01
* Bit Group Name: EMB_ACC
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_EMBEDDED_ACCESS_DISABLED 	 =0x00,
  	ISM330DLC_ACC_GYRO_EMBEDDED_ACCESS_ENABLED 		 =0x80,
} ISM330DLC_ACC_GYRO_EMB_ACC_t;

#define  	ISM330DLC_ACC_GYRO_EMB_ACC_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_EmbeddedAccess(void *handle, ISM330DLC_ACC_GYRO_EMB_ACC_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_EmbeddedAccess(void *handle, ISM330DLC_ACC_GYRO_EMB_ACC_t *value);

/*******************************************************************************
* Register      : SENSOR_SYNC_TIME
* Address       : 0X04
* Bit Group Name: TPH
* Permission    : RW
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_TPH_MASK  	0x0F
#define  	ISM330DLC_ACC_GYRO_TPH_POSITION  	0
mems_status_t ISM330DLC_ACC_GYRO_W_Stamping_Time_Frame(void *handle, u8_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_Stamping_Time_Frame(void *handle, u8_t *value);

/*******************************************************************************
* Register      : SENSOR_SYNC_RES_RATIO
* Address       : 0X05
* Bit Group Name: RR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_TIM_RATIO_2_11 		 =0x00,
  	ISM330DLC_ACC_GYRO_TIM_RATIO_2_12 		 =0x01,
  	ISM330DLC_ACC_GYRO_TIM_RATIO_2_13 		 =0x02,
  	ISM330DLC_ACC_GYRO_TIM_RATIO_2_14 		 =0x03,
} ISM330DLC_ACC_GYRO_SYNC_RES_RATIO_t;

#define  	ISM330DLC_ACC_GYRO_SYNC_RES_RATIO_MASK  	0x03
mems_status_t ISM330DLC_ACC_GYRO_W_SYNC_RES_RATIO(void *handle, ISM330DLC_ACC_GYRO_SYNC_RES_RATIO_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SYNC_RES_RATIO(void *handle, ISM330DLC_ACC_GYRO_SYNC_RES_RATIO_t *value);


/*******************************************************************************
* Register      : FIFO_CTRL1
* Address       : 0X06
* Bit Group Name: WTM_FIFO
* Permission    : RW
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL1_MASK  	0xFF
#define  	ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL1_POSITION  	0
#define  	ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL2_MASK  	0x07
#define  	ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL2_POSITION  	0
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_Watermark(void *handle, u16_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_Watermark(void *handle, u16_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL2
* Address       : 0X07
* Bit Group Name: FIFO_TEMP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_FIFO_TEMP_DISABLE 		 =0x00,
  	ISM330DLC_ACC_GYRO_FIFO_TEMP_ENABLE 		 =0x08,
} ISM330DLC_ACC_GYRO_FIFO_TEMP_t;

#define  	ISM330DLC_ACC_GYRO_FIFO_TEMP_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_TEMP(void *handle, ISM330DLC_ACC_GYRO_FIFO_TEMP_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_TEMP(void *handle, ISM330DLC_ACC_GYRO_FIFO_TEMP_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL2
* Address       : 0X07
* Bit Group Name: FIFO_TIMER_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_ENABLED 		 =0x80,
} ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_t;

#define  	ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_TIMER_En(void *handle, ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_TIMER_En(void *handle, ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL3
* Address       : 0X08
* Bit Group Name: DEC_FIFO_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DATA_NOT_IN_FIFO 	 =0x00,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION 		 =0x01,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_2 		 =0x02,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_3 		 =0x03,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_4 		 =0x04,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_8 		 =0x05,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_16 	 =0x06,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_32 	 =0x07,
} ISM330DLC_ACC_GYRO_DEC_FIFO_XL_t;

#define  	ISM330DLC_ACC_GYRO_DEC_FIFO_XL_MASK  	0x07
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL_val(void *handle, u16_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DEC_FIFO_XL(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL3
* Address       : 0X08
* Bit Group Name: DEC_FIFO_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEC_FIFO_G_DATA_NOT_IN_FIFO 		 =0x00,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_G_NO_DECIMATION 		 =0x08,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_2 		 =0x10,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_3 		 =0x18,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_4 		 =0x20,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_8 		 =0x28,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_16 		 =0x30,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_32 		 =0x38,
} ISM330DLC_ACC_GYRO_DEC_FIFO_G_t;

#define  	ISM330DLC_ACC_GYRO_DEC_FIFO_G_MASK  	0x38
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_G(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_G_val(void *handle, u16_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DEC_FIFO_G(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0X09
* Bit Group Name: DEC_DS3_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_DATA_NOT_IN_FIFO 		 =0x00,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_NO_DECIMATION 			 =0x01,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_DECIMATION_BY_2 		 =0x02,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_DECIMATION_BY_3 		 =0x03,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_DECIMATION_BY_4 		 =0x04,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_DECIMATION_BY_8 		 =0x05,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_DECIMATION_BY_16 		 =0x06,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_DECIMATION_BY_32 		 =0x07,
} ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_t;

#define  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_MASK  	0x07
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_DS3(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DEC_FIFO_DS3(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0X09
* Bit Group Name: DEC_DS4_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_DATA_NOT_IN_FIFO 		 =0x00,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_NO_DECIMATION 			 =0x08,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_DECIMATION_BY_2 		 =0x10,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_DECIMATION_BY_3 		 =0x18,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_DECIMATION_BY_4 		 =0x20,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_DECIMATION_BY_8 		 =0x28,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_DECIMATION_BY_16 		 =0x30,
  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_DECIMATION_BY_32 		 =0x38,
} ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_t;

#define  	ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_MASK  	0x38
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_DS4(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DEC_FIFO_DS4(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0X09
* Bit Group Name: HI_DATA_ONLY
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_HI_DATA_ONLY_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_HI_DATA_ONLY_ENABLED 		 =0x40,
} ISM330DLC_ACC_GYRO_HI_DATA_ONLY_t;

#define  	ISM330DLC_ACC_GYRO_HI_DATA_ONLY_MASK  	0x40
mems_status_t ISM330DLC_ACC_GYRO_W_HI_DATA_ONLY(void *handle, ISM330DLC_ACC_GYRO_HI_DATA_ONLY_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_HI_DATA_ONLY(void *handle, ISM330DLC_ACC_GYRO_HI_DATA_ONLY_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL4
* Address       : 0X09
* Bit Group Name: STOP_ON_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_STOP_ON_FTH_DISABLED 	 =0x00,
  	ISM330DLC_ACC_GYRO_STOP_ON_FTH_ENABLED 		 =0x80,
} ISM330DLC_ACC_GYRO_STOP_ON_FTH_t;

#define  	ISM330DLC_ACC_GYRO_STOP_ON_FTH_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_STOP_ON_FTH(void *handle, ISM330DLC_ACC_GYRO_STOP_ON_FTH_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_STOP_ON_FTH(void *handle, ISM330DLC_ACC_GYRO_STOP_ON_FTH_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL5
* Address       : 0X0A
* Bit Group Name: FIFO_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_FIFO_MODE_BYPASS 		 =0x00,
  	ISM330DLC_ACC_GYRO_FIFO_MODE_FIFO 			 =0x01,
  	ISM330DLC_ACC_GYRO_FIFO_MODE_RESERVED1 		 =0x02,
  	ISM330DLC_ACC_GYRO_FIFO_MODE_STREAM 		 =0x03,
  	ISM330DLC_ACC_GYRO_FIFO_MODE_BTS 			 =0x04,
  	ISM330DLC_ACC_GYRO_FIFO_MODE_RESERVED2 		 =0x05,
  	ISM330DLC_ACC_GYRO_FIFO_MODE_STREAM_OVER 	 =0x06,
  	ISM330DLC_ACC_GYRO_FIFO_MODE_RESERVED3 		 =0x07,
} ISM330DLC_ACC_GYRO_FIFO_MODE_t;

#define  	ISM330DLC_ACC_GYRO_FIFO_MODE_MASK  	0x07
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_MODE(void *handle, ISM330DLC_ACC_GYRO_FIFO_MODE_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_MODE(void *handle, ISM330DLC_ACC_GYRO_FIFO_MODE_t *value);

/*******************************************************************************
* Register      : FIFO_CTRL5
* Address       : 0X0A
* Bit Group Name: ODR_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_ODR_FIFO_DISABLE 	 =0x00,
  	ISM330DLC_ACC_GYRO_ODR_FIFO_12Hz5 		 =0x08,
  	ISM330DLC_ACC_GYRO_ODR_FIFO_26Hz 		 =0x10,
  	ISM330DLC_ACC_GYRO_ODR_FIFO_52Hz 		 =0x18,
  	ISM330DLC_ACC_GYRO_ODR_FIFO_104Hz 		 =0x20,
  	ISM330DLC_ACC_GYRO_ODR_FIFO_208Hz 		 =0x28,
  	ISM330DLC_ACC_GYRO_ODR_FIFO_416Hz 		 =0x30,
  	ISM330DLC_ACC_GYRO_ODR_FIFO_833Hz 		 =0x38,
  	ISM330DLC_ACC_GYRO_ODR_FIFO_1600Hz 		 =0x40,
  	ISM330DLC_ACC_GYRO_ODR_FIFO_3300Hz 		 =0x48,
  	ISM330DLC_ACC_GYRO_ODR_FIFO_6600Hz 		 =0x50,
} ISM330DLC_ACC_GYRO_ODR_FIFO_t;

#define  	ISM330DLC_ACC_GYRO_ODR_FIFO_MASK  	0x78
mems_status_t ISM330DLC_ACC_GYRO_W_ODR_FIFO(void *handle, ISM330DLC_ACC_GYRO_ODR_FIFO_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_ODR_FIFO(void *handle, ISM330DLC_ACC_GYRO_ODR_FIFO_t *value);

/*******************************************************************************
* Register      : DRDY_PULSE_CFG_G
* Address       : 0X0B
* Bit Group Name: DRDY_PULSE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DRDY_LATCH 		 =0x00,
  	ISM330DLC_ACC_GYRO_DRDY_PULSE 		 =0x80,
} ISM330DLC_ACC_GYRO_DRDY_PULSE_t;

#define  	ISM330DLC_ACC_GYRO_DRDY_PULSE_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_PULSE(void *handle, ISM330DLC_ACC_GYRO_DRDY_PULSE_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_PULSE(void *handle, ISM330DLC_ACC_GYRO_DRDY_PULSE_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_DRDY_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_DRDY_XL_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_DRDY_XL_ENABLED 		 =0x01,
} ISM330DLC_ACC_GYRO_INT1_DRDY_XL_t;

#define  	ISM330DLC_ACC_GYRO_INT1_DRDY_XL_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_XL_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_DRDY_XL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_XL_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_DRDY_XL_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_DRDY_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_DRDY_G_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_DRDY_G_ENABLED 			 =0x02,
} ISM330DLC_ACC_GYRO_INT1_DRDY_G_t;

#define  	ISM330DLC_ACC_GYRO_INT1_DRDY_G_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_G_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_DRDY_G_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_G_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_DRDY_G_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_BOOT_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_BOOT_ENABLED 		 =0x04,
} ISM330DLC_ACC_GYRO_INT1_BOOT_t;

#define  	ISM330DLC_ACC_GYRO_INT1_BOOT_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_BOOT_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_BOOT_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_BOOT_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_BOOT_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_FTH_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_FTH_ENABLED 		 =0x08,
} ISM330DLC_ACC_GYRO_INT1_FTH_t;

#define  	ISM330DLC_ACC_GYRO_INT1_FTH_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_TSHLD_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_FTH_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_TSHLD_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_FTH_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_OVR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_OVR_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_OVR_ENABLED 		 =0x10,
} ISM330DLC_ACC_GYRO_INT1_OVR_t;

#define  	ISM330DLC_ACC_GYRO_INT1_OVR_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_OVERRUN_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_OVR_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_OVERRUN_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_OVR_t *value);

/*******************************************************************************
* Register      : INT1_CTRL
* Address       : 0X0D
* Bit Group Name: INT1_FULL_FLAG
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_ENABLED 		 =0x20,
} ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_t;

#define  	ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_W_FULL_FLAG_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FULL_FLAG_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_DRDY_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_DRDY_XL_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_DRDY_XL_ENABLED 		 =0x01,
} ISM330DLC_ACC_GYRO_INT2_DRDY_XL_t;

#define  	ISM330DLC_ACC_GYRO_INT2_DRDY_XL_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_XL_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_XL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_XL_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_XL_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_DRDY_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_DRDY_G_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_DRDY_G_ENABLED 			 =0x02,
} ISM330DLC_ACC_GYRO_INT2_DRDY_G_t;

#define  	ISM330DLC_ACC_GYRO_INT2_DRDY_G_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_G_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_G_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_G_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_G_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_DRDY_TEMP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_ENABLED 		 =0x04,
} ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_t;

#define  	ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_TEMP_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_TEMP_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_t *value);


/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_FTH
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_FTH_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_FTH_ENABLED 		 =0x08,
} ISM330DLC_ACC_GYRO_INT2_FTH_t;

#define  	ISM330DLC_ACC_GYRO_INT2_FTH_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_TSHLD_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_FTH_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_TSHLD_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_FTH_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_OVR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_OVR_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_OVR_ENABLED 		 =0x10,
} ISM330DLC_ACC_GYRO_INT2_OVR_t;

#define  	ISM330DLC_ACC_GYRO_INT2_OVR_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_OVERRUN_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_OVR_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_OVERRUN_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_OVR_t *value);

/*******************************************************************************
* Register      : INT2_CTRL
* Address       : 0X0E
* Bit Group Name: INT2_FULL_FLAG
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_ENABLED 		 =0x20,
} ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_t;

#define  	ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_W_FULL_FLAG_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FULL_FLAG_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: SW_RESET
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_SW_RESET_NORMAL_MODE 		 =0x00,
  	ISM330DLC_ACC_GYRO_SW_RESET_RESET_DEVICE 		 =0x01,
} ISM330DLC_ACC_GYRO_SW_RESET_t;

#define  	ISM330DLC_ACC_GYRO_SW_RESET_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_W_SW_RESET(void *handle, ISM330DLC_ACC_GYRO_SW_RESET_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SW_RESET(void *handle, ISM330DLC_ACC_GYRO_SW_RESET_t *value);


/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: IF_INC
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_IF_INC_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_IF_INC_ENABLED 		 =0x04,
} ISM330DLC_ACC_GYRO_IF_INC_t;

#define  	ISM330DLC_ACC_GYRO_IF_INC_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_IF_Addr_Incr(void *handle, ISM330DLC_ACC_GYRO_IF_INC_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_IF_Addr_Incr(void *handle, ISM330DLC_ACC_GYRO_IF_INC_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: SIM
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_SIM_4_WIRE 		 =0x00,
  	ISM330DLC_ACC_GYRO_SIM_3_WIRE 		 =0x08,
} ISM330DLC_ACC_GYRO_SIM_t;

#define  	ISM330DLC_ACC_GYRO_SIM_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_W_SPI_Mode(void *handle, ISM330DLC_ACC_GYRO_SIM_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SPI_Mode(void *handle, ISM330DLC_ACC_GYRO_SIM_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: PP_OD
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_PP_OD_PUSH_PULL 		 =0x00,
  	ISM330DLC_ACC_GYRO_PP_OD_OPEN_DRAIN 	 =0x10,
} ISM330DLC_ACC_GYRO_PP_OD_t;

#define  	ISM330DLC_ACC_GYRO_PP_OD_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_PadSel(void *handle, ISM330DLC_ACC_GYRO_PP_OD_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_PadSel(void *handle, ISM330DLC_ACC_GYRO_PP_OD_t *value);

/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: H_LACTIVE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_ACTIVE_HI 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_ACTIVE_LO 		 =0x20,
} ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_t;

#define  	ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_W_INT_ACT_LEVEL(void *handle, ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_INT_ACT_LEVEL(void *handle, ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_t *value);


/*******************************************************************************
* Register      : CTRL3_C
* Address       : 0X12
* Bit Group Name: BOOT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_BOOT_NORMAL_MODE 		 =0x00,
  	ISM330DLC_ACC_GYRO_BOOT_REBOOT_MODE 		 =0x80,
} ISM330DLC_ACC_GYRO_BOOT_t;

#define  	ISM330DLC_ACC_GYRO_BOOT_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_BOOT(void *handle, ISM330DLC_ACC_GYRO_BOOT_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_BOOT(void *handle, ISM330DLC_ACC_GYRO_BOOT_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: LPF1_SEL_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_MODE3_LPF1_G_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_MODE3_LPF1_G_ENABLED 		 =0x02,
} ISM330DLC_ACC_GYRO_LPF1_SEL_G_t;

#define  	ISM330DLC_ACC_GYRO_LPF1_SEL_G_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_W_LPF1_SEL_G(void *handle, ISM330DLC_ACC_GYRO_LPF1_SEL_G_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_LPF1_SEL_G(void *handle, ISM330DLC_ACC_GYRO_LPF1_SEL_G_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: I2C_DISABLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_I2C_DISABLE_I2C_AND_SPI 		 =0x00,
  	ISM330DLC_ACC_GYRO_I2C_DISABLE_SPI_ONLY 		 =0x04,
} ISM330DLC_ACC_GYRO_I2C_DISABLE_t;

#define  	ISM330DLC_ACC_GYRO_I2C_DISABLE_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_I2C_DISABLE(void *handle, ISM330DLC_ACC_GYRO_I2C_DISABLE_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_I2C_DISABLE(void *handle, ISM330DLC_ACC_GYRO_I2C_DISABLE_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: DRDY_MSK
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DRDY_MSK_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_DRDY_MSK_ENABLED 		 =0x08,
} ISM330DLC_ACC_GYRO_DRDY_MSK_t;

#define  	ISM330DLC_ACC_GYRO_DRDY_MSK_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_MSK(void *handle, ISM330DLC_ACC_GYRO_DRDY_MSK_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_MSK(void *handle, ISM330DLC_ACC_GYRO_DRDY_MSK_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: DEN_DRDY_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_ENABLED 		 =0x10,
} ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_t;

#define  	ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_DEN_DRDY_INT1(void *handle, ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DEN_DRDY_INT1(void *handle, ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: INT2_ON_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_ON_INT1_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_ON_INT1_ENABLED 		 =0x20,
} ISM330DLC_ACC_GYRO_INT2_ON_INT1_t;

#define  	ISM330DLC_ACC_GYRO_INT2_ON_INT1_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_W_INT2_ON_INT1(void *handle, ISM330DLC_ACC_GYRO_INT2_ON_INT1_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_INT2_ON_INT1(void *handle, ISM330DLC_ACC_GYRO_INT2_ON_INT1_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: SLEEP_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_SLEEP_G_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_SLEEP_G_ENABLED 			 =0x40,
} ISM330DLC_ACC_GYRO_SLEEP_G_t;

#define  	ISM330DLC_ACC_GYRO_SLEEP_G_MASK  	0x40
mems_status_t ISM330DLC_ACC_GYRO_W_SleepMode_G(void *handle, ISM330DLC_ACC_GYRO_SLEEP_G_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SleepMode_G(void *handle, ISM330DLC_ACC_GYRO_SLEEP_G_t *value);

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: DEN_XL_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEN_XL_EN_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_DEN_XL_EN_ENABLED 		 =0x80,
} ISM330DLC_ACC_GYRO_DEN_XL_EN_t;

#define  	ISM330DLC_ACC_GYRO_DEN_XL_EN_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_DEN_XL_EN(void *handle, ISM330DLC_ACC_GYRO_DEN_XL_EN_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DEN_XL_EN(void *handle, ISM330DLC_ACC_GYRO_DEN_XL_EN_t *value);

/*******************************************************************************
* Register      : CTRL5_C
* Address       : 0X14
* Bit Group Name: ST_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_ST_XL_NORMAL_MODE 		 =0x00,
  	ISM330DLC_ACC_GYRO_ST_XL_POS_SIGN_TEST 		 =0x01,
  	ISM330DLC_ACC_GYRO_ST_XL_NEG_SIGN_TEST 		 =0x02,
  	ISM330DLC_ACC_GYRO_ST_XL_NA 				 =0x03,
} ISM330DLC_ACC_GYRO_ST_XL_t;

#define  	ISM330DLC_ACC_GYRO_ST_XL_MASK  	0x03
mems_status_t ISM330DLC_ACC_GYRO_W_SelfTest_XL(void *handle, ISM330DLC_ACC_GYRO_ST_XL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SelfTest_XL(void *handle, ISM330DLC_ACC_GYRO_ST_XL_t *value);

/*******************************************************************************
* Register      : CTRL5_C
* Address       : 0X14
* Bit Group Name: ST_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_ST_G_NORMAL_MODE 		 =0x00,
  	ISM330DLC_ACC_GYRO_ST_G_POS_SIGN_TEST 		 =0x04,
  	ISM330DLC_ACC_GYRO_ST_G_NA 					 =0x08,
  	ISM330DLC_ACC_GYRO_ST_G_NEG_SIGN_TEST 		 =0x0C,
} ISM330DLC_ACC_GYRO_ST_G_t;

#define  	ISM330DLC_ACC_GYRO_ST_G_MASK  	0x0C
mems_status_t ISM330DLC_ACC_GYRO_W_SelfTest_G(void *handle, ISM330DLC_ACC_GYRO_ST_G_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SelfTest_G(void *handle, ISM330DLC_ACC_GYRO_ST_G_t *value);

/*******************************************************************************
* Register      : CTRL5_C
* Address       : 0X14
* Bit Group Name: DEN_LH
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEN_LOW 			 =0x00,
  	ISM330DLC_ACC_GYRO_DEN_HIGH 		 =0x10,
} ISM330DLC_ACC_GYRO_DEN_LH_t;

#define  	ISM330DLC_ACC_GYRO_DEN_LH_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_DEN_Polarity(void *handle, ISM330DLC_ACC_GYRO_DEN_LH_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DEN_Polarity(void *handle, ISM330DLC_ACC_GYRO_DEN_LH_t *value);

/*******************************************************************************
* Register      : CTRL5_C
* Address       : 0X14
* Bit Group Name: ST_ROUNDING
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_NO_ROUNDING 					 =0x00,
  	ISM330DLC_ACC_GYRO_ACC_ONLY 					 =0x20,
  	ISM330DLC_ACC_GYRO_GYRO_ONLY 					 =0x40,
	ISM330DLC_ACC_GYRO_ACC_GYRO 					 =0x60,
  	ISM330DLC_ACC_GYRO_SH1_SH6 						 =0x80,
	ISM330DLC_ACC_GYRO_ACC_SH1_SH6 					 =0xA0,
	ISM330DLC_ACC_GYRO_ACC_GYRO_SH1_SH6_SH7_SH12 	 =0xC0,
	ISM330DLC_ACC_GYRO_ACC_GYRO_SH1_SH6				 =0xE0,
} ISM330DLC_ACC_GYRO_ROUNDING_t;

#define  	ISM330DLC_ACC_GYRO_ISM330DLC_ACC_GYRO_ROUNDING_t_MASK  	0xE0
mems_status_t ISM330DLC_ACC_GYRO_W_CircularBurstMode(void *handle, ISM330DLC_ACC_GYRO_ROUNDING_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_CircularBurstMode(void *handle, ISM330DLC_ACC_GYRO_ROUNDING_t *value);

/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0X15
* Bit Group Name: FTYPE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_LP_G_NORMAL 			 =0x00,
  	ISM330DLC_ACC_GYRO_LP_G_NARROW 			 =0x01,
  	ISM330DLC_ACC_GYRO_LP_G_VERY_NARROW 	 =0x02,
  	ISM330DLC_ACC_GYRO_LP_G_WIDE 			 =0x03,
} ISM330DLC_ACC_GYRO_FTYPE_t;

#define  	ISM330DLC_ACC_GYRO_FTYPE_MASK  	0x03
mems_status_t ISM330DLC_ACC_GYRO_W_LP_BW_G(void *handle, ISM330DLC_ACC_GYRO_FTYPE_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_LP_BW_G(void *handle, ISM330DLC_ACC_GYRO_FTYPE_t *value);

/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0X15
* Bit Group Name: USR_OFF_W
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_2Emin10 		 =0x00,
  	ISM330DLC_ACC_GYRO_2Emin6 		 =0x08,
} ISM330DLC_ACC_GYRO_USR_OFF_W_t;

#define  	ISM330DLC_ACC_GYRO_USR_OFF_W_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_W_UserOffsetWeight(void *handle, ISM330DLC_ACC_GYRO_USR_OFF_W_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_UserOffsetWeight(void *handle, ISM330DLC_ACC_GYRO_USR_OFF_W_t *value);


/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0X15
* Bit Group Name: XL_HM_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_XL_HM_MODE_ENABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_XL_HM_MODE_DISABLED 		 =0x10,
} ISM330DLC_ACC_GYRO_XL_HM_MODE_t;

#define  	ISM330DLC_ACC_GYRO_XL_HM_MODE_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_HighPerform(void *handle, ISM330DLC_ACC_GYRO_XL_HM_MODE_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_HighPerform(void *handle, ISM330DLC_ACC_GYRO_XL_HM_MODE_t *value);

/*******************************************************************************
* Register      : CTRL6_G
* Address       : 0X15
* Bit Group Name: TRIGER_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_EDGE_SENSITIVE_TRIGER_MODE 		 =0x80,
  	ISM330DLC_ACC_GYRO_LEVEL_SENSITIVE_TRIGER_MODE 		 =0x40,
  	ISM330DLC_ACC_GYRO_LEVEL_SENSITIVE_LATCHED_MODE 	 =0x60,
  	ISM330DLC_ACC_GYRO_LEVEL_SENSITIVE_FIFO_MODE 		 =0xC0,
} ISM330DLC_ACC_GYRO_TRIGER_MODE_t;

#define  	ISM330DLC_ACC_GYRO_TRIGER_MODE_MASK  	0xE0
mems_status_t ISM330DLC_ACC_GYRO_W_TrigerMode(void *handle, ISM330DLC_ACC_GYRO_TRIGER_MODE_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TrigerMode(void *handle, ISM330DLC_ACC_GYRO_TRIGER_MODE_t *value);

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0X16
* Bit Group Name: ROUNDING_STATUS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_ROUNDING_STATUS_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_ROUNDING_STATUS_ENABLED 			 =0x04,
} ISM330DLC_ACC_GYRO_ROUNDING_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_ROUNDING_STATUS_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_ROUNDING_STATUS(void *handle, ISM330DLC_ACC_GYRO_ROUNDING_STATUS_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_ROUNDING_STATUS(void *handle, ISM330DLC_ACC_GYRO_ROUNDING_STATUS_t *value);


/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0X16
* Bit Group Name: HPM_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_HPM_G_0Hz016 		 =0x00,
  	ISM330DLC_ACC_GYRO_HPM_G_0Hz065  		 =0x10,
  	ISM330DLC_ACC_GYRO_HPM_G_0Hz260 		 =0x20,
  	ISM330DLC_ACC_GYRO_HPM_G_1Hz04 			 =0x30,
} ISM330DLC_ACC_GYRO_HPM_G_t;

#define  	ISM330DLC_ACC_GYRO_HPM_G_MASK  	0x30
mems_status_t ISM330DLC_ACC_GYRO_W_HPM_G(void *handle, ISM330DLC_ACC_GYRO_HPM_G_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_HPM_G(void *handle, ISM330DLC_ACC_GYRO_HPM_G_t *value);

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0X16
* Bit Group Name: HP_EN_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_HP_EN_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_HP_EN_ENABLED 		 =0x40,
} ISM330DLC_ACC_GYRO_HP_EN_t;

#define  	ISM330DLC_ACC_GYRO_HP_EN_MASK  	0x40
mems_status_t ISM330DLC_ACC_GYRO_W_HPFilter_En(void *handle, ISM330DLC_ACC_GYRO_HP_EN_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_HPFilter_En(void *handle, ISM330DLC_ACC_GYRO_HP_EN_t *value);

/*******************************************************************************
* Register      : CTRL7_G
* Address       : 0X16
* Bit Group Name: G_HM_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_G_HM_MODE_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_G_HM_MODE_ENABLED 		 =0x80,
} ISM330DLC_ACC_GYRO_G_HM_MODE_t;

#define  	ISM330DLC_ACC_GYRO_G_HM_MODE_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_HM_Mode(void *handle, ISM330DLC_ACC_GYRO_G_HM_MODE_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_HM_Mode(void *handle, ISM330DLC_ACC_GYRO_G_HM_MODE_t *value);

/*******************************************************************************
* Register      : CTRL8_XL
* Address       : 0X17
* Bit Group Name: LOW_PASS_ON_6D
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_OFF 		 =0x00,
  	ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_ON 		 =0x01,
} ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_t;

#define  	ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_W_LOW_PASS_ON_6D(void *handle, ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_LOW_PASS_ON_6D(void *handle, ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_t *value);

/*******************************************************************************
* Register      : CTRL8_XL
* Address       : 0X17
* Bit Group Name: HP_SLOPE_XL_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_HP_SLOPE_XL_EN 		 =0x00,
  	ISM330DLC_ACC_GYRO_HP_SLOPE_XL_DIS 		 =0x04,
} ISM330DLC_ACC_GYRO_HP_SLOPE_XL_t;

#define  	ISM330DLC_ACC_GYRO_HP_SLOPE_XL_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_HP_SLOPE_XL(void *handle, ISM330DLC_ACC_GYRO_HP_SLOPE_XL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_HP_SLOPE_XL(void *handle, ISM330DLC_ACC_GYRO_HP_SLOPE_XL_t *value);

/*******************************************************************************
* Register      : CTRL8_XL
* Address       : 0X17
* Bit Group Name: INPUT_COMPOSITE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_IN_ODR_DIV_2 		 =0x00,
  	ISM330DLC_ACC_GYRO_IN_ODR_DIV_4 		 =0x80,
} ISM330DLC_ACC_GYRO_IN_COMP_t;

#define  	ISM330DLC_ACC_GYRO_IN_COMP_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_InComposit(void *handle, ISM330DLC_ACC_GYRO_IN_COMP_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_InComposit(void *handle, ISM330DLC_ACC_GYRO_IN_COMP_t *value);

/*******************************************************************************
* Register      : CTRL8_XL
* Address       : 0X17
* Bit Group Name: HP_REF_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_HP_REF_DISABLE 		 =0x00,
  	ISM330DLC_ACC_GYRO_HP_REF_ENABLE 		 =0x10,
} ISM330DLC_ACC_GYRO_HP_REF_MODE_t;

#define  	ISM330DLC_ACC_GYRO_HP_REF_MODE_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_HPfilterReference(void *handle, ISM330DLC_ACC_GYRO_HP_REF_MODE_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_HPfilterReference(void *handle, ISM330DLC_ACC_GYRO_HP_REF_MODE_t *value);

/*******************************************************************************
* Register      : CTRL8_XL
* Address       : 0X17
* Bit Group Name: HPCF_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_HPCF_XL_DIV4 		 =0x00,
  	ISM330DLC_ACC_GYRO_HPCF_XL_DIV100 		 =0x20,
  	ISM330DLC_ACC_GYRO_HPCF_XL_DIV9 		 =0x40,
  	ISM330DLC_ACC_GYRO_HPCF_XL_DIV400 		 =0x60,
} ISM330DLC_ACC_GYRO_HPCF_XL_t;

#define  	ISM330DLC_ACC_GYRO_HPCF_XL_MASK  	0x60
mems_status_t ISM330DLC_ACC_GYRO_W_HPCF_XL(void *handle, ISM330DLC_ACC_GYRO_HPCF_XL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_HPCF_XL(void *handle, ISM330DLC_ACC_GYRO_HPCF_XL_t *value);

/*******************************************************************************
* Register      : CTRL8_XL
* Address       : 0X17
* Bit Group Name: LPF2_XL_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_LPF2_XL_DISABLE 		 =0x00,
  	ISM330DLC_ACC_GYRO_LPF2_XL_ENABLE 		 =0x80,
} ISM330DLC_ACC_GYRO_LPF2_XL_t;

#define  	ISM330DLC_ACC_GYRO_LPF2_XL_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_LowPassFiltSel_XL(void *handle, ISM330DLC_ACC_GYRO_LPF2_XL_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_LowPassFiltSel_XL(void *handle, ISM330DLC_ACC_GYRO_LPF2_XL_t *value);


/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0X18
* Bit Group Name: SOFT_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_SOFT_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_SOFT_ENABLE 			 =0x04,
} ISM330DLC_ACC_GYRO_SOFT_t;

#define  	ISM330DLC_ACC_GYRO_SOFT_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_SOFT(void *handle, ISM330DLC_ACC_GYRO_SOFT_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SOFT(void *handle, ISM330DLC_ACC_GYRO_SOFT_t *value);

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0X18
* Bit Group Name: DEN_XL_G
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEN_STAMP_GYRO 		 =0x00,
  	ISM330DLC_ACC_GYRO_DEN_STAMP_ACC 		 =0x10,
} ISM330DLC_ACC_GYRO_DEN_XL_G_t;

#define  	ISM330DLC_ACC_GYRO_DEN_XL_G_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_DEN_XL_G(void *handle, ISM330DLC_ACC_GYRO_DEN_XL_G_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DEN_XL_G(void *handle, ISM330DLC_ACC_GYRO_DEN_XL_G_t *value);

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0X18
* Bit Group Name: DEN_Z
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEN_NOT_STORED_Z 	 =0x00,
  	ISM330DLC_ACC_GYRO_DEN_STORED_Z 		 =0x20,
} ISM330DLC_ACC_GYRO_DEN_Z_t;

#define  	ISM330DLC_ACC_GYRO_DEN_Z_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_W_DEN_Z(void *handle, ISM330DLC_ACC_GYRO_DEN_Z_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DEN_Z(void *handle, ISM330DLC_ACC_GYRO_DEN_Z_t *value);

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0X18
* Bit Group Name: DEN_Y
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEN_NOT_STORED_Y 	 =0x00,
  	ISM330DLC_ACC_GYRO_DEN_STORED_Y 		 =0x40,
} ISM330DLC_ACC_GYRO_DEN_Y_t;

#define  	ISM330DLC_ACC_GYRO_DEN_Y_MASK  	0x40
mems_status_t ISM330DLC_ACC_GYRO_W_DEN_Y(void *handle, ISM330DLC_ACC_GYRO_DEN_Y_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DEN_Y(void *handle, ISM330DLC_ACC_GYRO_DEN_Y_t *value);

/*******************************************************************************
* Register      : CTRL9_XL
* Address       : 0X18
* Bit Group Name: DEN_X
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEN_NOT_STORED_X 	 =0x00,
  	ISM330DLC_ACC_GYRO_DEN_STORED_X 		 =0x80,
} ISM330DLC_ACC_GYRO_DEN_X_t;

#define  	ISM330DLC_ACC_GYRO_DEN_X_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_DEN_X(void *handle, ISM330DLC_ACC_GYRO_DEN_X_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DEN_X(void *handle, ISM330DLC_ACC_GYRO_DEN_X_t *value);

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0X19
* Bit Group Name: FUNC_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_FUNC_EN_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_FUNC_EN_ENABLED 			 =0x04,
} ISM330DLC_ACC_GYRO_FUNC_EN_t;

#define  	ISM330DLC_ACC_GYRO_FUNC_EN_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_FUNC_EN(void *handle, ISM330DLC_ACC_GYRO_FUNC_EN_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FUNC_EN(void *handle, ISM330DLC_ACC_GYRO_FUNC_EN_t *value);

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0X19
* Bit Group Name: TILT_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_TILT_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_TILT_ENABLED 		 =0x08,
} ISM330DLC_ACC_GYRO_TILT_G_t;

#define  	ISM330DLC_ACC_GYRO_TILT_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_W_TILT(void *handle, ISM330DLC_ACC_GYRO_TILT_G_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TILT(void *handle, ISM330DLC_ACC_GYRO_TILT_G_t *value);

/*******************************************************************************
* Register      : CTRL10_C
* Address       : 0X19
* Bit Group Name: TIMER_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_TIMER_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_TIMER_ENABLED 		 =0x20,
} ISM330DLC_ACC_GYRO_TIMER_t;

#define  	ISM330DLC_ACC_GYRO_TIMER_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_W_TIMER(void *handle, ISM330DLC_ACC_GYRO_TIMER_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TIMER(void *handle, ISM330DLC_ACC_GYRO_TIMER_t *value);


/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: MASTER_ON
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_MASTER_ON_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_MASTER_ON_ENABLED 		 =0x01,
} ISM330DLC_ACC_GYRO_MASTER_ON_t;

#define  	ISM330DLC_ACC_GYRO_MASTER_ON_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_W_I2C_MASTER_Enable(void *handle, ISM330DLC_ACC_GYRO_MASTER_ON_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_I2C_MASTER_Enable(void *handle, ISM330DLC_ACC_GYRO_MASTER_ON_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: IRON_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_IRON_EN_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_IRON_EN_ENABLED 			 =0x02,
} ISM330DLC_ACC_GYRO_IRON_EN_t;

#define  	ISM330DLC_ACC_GYRO_IRON_EN_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_W_IronCorrection_EN(void *handle, ISM330DLC_ACC_GYRO_IRON_EN_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_IronCorrection_EN(void *handle, ISM330DLC_ACC_GYRO_IRON_EN_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: PASS_THRU_MODE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_PASS_THRU_MODE_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_PASS_THRU_MODE_ENABLED 		 =0x04,
} ISM330DLC_ACC_GYRO_PASS_THRU_MODE_t;

#define  	ISM330DLC_ACC_GYRO_PASS_THRU_MODE_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_PASS_THRU_MODE(void *handle, ISM330DLC_ACC_GYRO_PASS_THRU_MODE_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_PASS_THRU_MODE(void *handle, ISM330DLC_ACC_GYRO_PASS_THRU_MODE_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: PULL_UP_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_PULL_UP_EN_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_PULL_UP_EN_ENABLED 		 =0x08,
} ISM330DLC_ACC_GYRO_PULL_UP_EN_t;

#define  	ISM330DLC_ACC_GYRO_PULL_UP_EN_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_W_PULL_UP_EN(void *handle, ISM330DLC_ACC_GYRO_PULL_UP_EN_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_PULL_UP_EN(void *handle, ISM330DLC_ACC_GYRO_PULL_UP_EN_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: START_CONFIG
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_START_CONFIG_XL_G_DRDY 		 =0x00,
  	ISM330DLC_ACC_GYRO_START_CONFIG_EXT_INT2 		 =0x10,
} ISM330DLC_ACC_GYRO_START_CONFIG_t;

#define  	ISM330DLC_ACC_GYRO_START_CONFIG_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_SensorHUB_Trigger_Sel(void *handle, ISM330DLC_ACC_GYRO_START_CONFIG_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SensorHUB_Trigger_Sel(void *handle, ISM330DLC_ACC_GYRO_START_CONFIG_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: DATA_VAL_SEL_FIFO
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_XL_G_DRDY 		 =0x00,
  	ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_SHUB_DRDY 		 =0x40,
} ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_t;

#define  	ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_MASK  	0x40
mems_status_t ISM330DLC_ACC_GYRO_W_DATA_VAL_SEL_FIFO(void *handle, ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DATA_VAL_SEL_FIFO(void *handle, ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_t *value);

/*******************************************************************************
* Register      : MASTER_CONFIG
* Address       : 0X1A
* Bit Group Name: DRDY_ON_INT1
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DRDY_ON_INT1_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_DRDY_ON_INT1_ENABLED 		 =0x80,
} ISM330DLC_ACC_GYRO_DRDY_ON_INT1_t;

#define  	ISM330DLC_ACC_GYRO_DRDY_ON_INT1_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_ON_INT1(void *handle, ISM330DLC_ACC_GYRO_DRDY_ON_INT1_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_ON_INT1(void *handle, ISM330DLC_ACC_GYRO_DRDY_ON_INT1_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: Z_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_Z_WU_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_Z_WU_DETECTED 			 =0x01,
} ISM330DLC_ACC_GYRO_Z_WU_t;

#define  	ISM330DLC_ACC_GYRO_Z_WU_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_R_Z_WU(void *handle, ISM330DLC_ACC_GYRO_Z_WU_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: Y_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_Y_WU_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_Y_WU_DETECTED 			 =0x02,
} ISM330DLC_ACC_GYRO_Y_WU_t;

#define  	ISM330DLC_ACC_GYRO_Y_WU_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_R_Y_WU(void *handle, ISM330DLC_ACC_GYRO_Y_WU_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: X_WU
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_X_WU_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_X_WU_DETECTED 			 =0x04,
} ISM330DLC_ACC_GYRO_X_WU_t;

#define  	ISM330DLC_ACC_GYRO_X_WU_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_R_X_WU(void *handle, ISM330DLC_ACC_GYRO_X_WU_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: WU_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_WU_EV_STATUS_NOT_DETECTED 	 =0x00,
  	ISM330DLC_ACC_GYRO_WU_EV_STATUS_DETECTED 		 =0x08,
} ISM330DLC_ACC_GYRO_WU_EV_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_WU_EV_STATUS_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_R_WU_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_WU_EV_STATUS_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: SLEEP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_SLEEP_EV_STATUS_NOT_DETECTED 	 =0x00,
  	ISM330DLC_ACC_GYRO_SLEEP_EV_STATUS_DETECTED 		 =0x10,
} ISM330DLC_ACC_GYRO_SLEEP_EV_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_SLEEP_EV_STATUS_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_R_SLEEP_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_SLEEP_EV_STATUS_t *value);

/*******************************************************************************
* Register      : WAKE_UP_SRC
* Address       : 0X1B
* Bit Group Name: FF_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_FF_EV_STATUS_NOT_DETECTED 	 =0x00,
  	ISM330DLC_ACC_GYRO_FF_EV_STATUS_DETECTED 		 =0x20,
} ISM330DLC_ACC_GYRO_FF_EV_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_FF_EV_STATUS_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_R_FF_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_FF_EV_STATUS_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: Z_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_Z_TAP_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_Z_TAP_DETECTED 			 =0x01,
} ISM330DLC_ACC_GYRO_Z_TAP_t;

#define  	ISM330DLC_ACC_GYRO_Z_TAP_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_R_Z_TAP(void *handle, ISM330DLC_ACC_GYRO_Z_TAP_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: Y_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_Y_TAP_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_Y_TAP_DETECTED 			 =0x02,
} ISM330DLC_ACC_GYRO_Y_TAP_t;

#define  	ISM330DLC_ACC_GYRO_Y_TAP_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_R_Y_TAP(void *handle, ISM330DLC_ACC_GYRO_Y_TAP_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: X_TAP
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_X_TAP_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_X_TAP_DETECTED 			 =0x04,
} ISM330DLC_ACC_GYRO_X_TAP_t;

#define  	ISM330DLC_ACC_GYRO_X_TAP_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_R_X_TAP(void *handle, ISM330DLC_ACC_GYRO_X_TAP_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: TAP_SIGN
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_TAP_SIGN_POS_SIGN 		 =0x00,
  	ISM330DLC_ACC_GYRO_TAP_SIGN_NEG_SIGN 		 =0x08,
} ISM330DLC_ACC_GYRO_TAP_SIGN_t;

#define  	ISM330DLC_ACC_GYRO_TAP_SIGN_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_SIGN(void *handle, ISM330DLC_ACC_GYRO_TAP_SIGN_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: DOUBLE_TAP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DOUBLE_TAP_EV_STATUS_NOT_DETECTED 	 =0x00,
  	ISM330DLC_ACC_GYRO_DOUBLE_TAP_EV_STATUS_DETECTED 		 =0x10,
} ISM330DLC_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_DOUBLE_TAP_EV_STATUS_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_R_DOUBLE_TAP_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: SINGLE_TAP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_SINGLE_TAP_EV_STATUS_NOT_DETECTED 	 =0x00,
  	ISM330DLC_ACC_GYRO_SINGLE_TAP_EV_STATUS_DETECTED 		 =0x20,
} ISM330DLC_ACC_GYRO_SINGLE_TAP_EV_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_SINGLE_TAP_EV_STATUS_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_R_SINGLE_TAP_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_SINGLE_TAP_EV_STATUS_t *value);

/*******************************************************************************
* Register      : TAP_SRC
* Address       : 0X1C
* Bit Group Name: TAP_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_TAP_EV_STATUS_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_TAP_EV_STATUS_DETECTED 			 =0x40,
} ISM330DLC_ACC_GYRO_TAP_EV_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_TAP_EV_STATUS_MASK  	0x40
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_TAP_EV_STATUS_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_XL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DSD_XL_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_DSD_XL_DETECTED 			 =0x01,
} ISM330DLC_ACC_GYRO_DSD_XL_t;

#define  	ISM330DLC_ACC_GYRO_DSD_XL_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_XL(void *handle, ISM330DLC_ACC_GYRO_DSD_XL_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_XH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DSD_XH_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_DSD_XH_DETECTED 			 =0x02,
} ISM330DLC_ACC_GYRO_DSD_XH_t;

#define  	ISM330DLC_ACC_GYRO_DSD_XH_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_XH(void *handle, ISM330DLC_ACC_GYRO_DSD_XH_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_YL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DSD_YL_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_DSD_YL_DETECTED 			 =0x04,
} ISM330DLC_ACC_GYRO_DSD_YL_t;

#define  	ISM330DLC_ACC_GYRO_DSD_YL_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_YL(void *handle, ISM330DLC_ACC_GYRO_DSD_YL_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_YH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DSD_YH_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_DSD_YH_DETECTED 			 =0x08,
} ISM330DLC_ACC_GYRO_DSD_YH_t;

#define  	ISM330DLC_ACC_GYRO_DSD_YH_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_YH(void *handle, ISM330DLC_ACC_GYRO_DSD_YH_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_ZL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DSD_ZL_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_DSD_ZL_DETECTED 			 =0x10,
} ISM330DLC_ACC_GYRO_DSD_ZL_t;

#define  	ISM330DLC_ACC_GYRO_DSD_ZL_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_ZL(void *handle, ISM330DLC_ACC_GYRO_DSD_ZL_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DSD_ZH
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DSD_ZH_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_DSD_ZH_DETECTED 			 =0x20,
} ISM330DLC_ACC_GYRO_DSD_ZH_t;

#define  	ISM330DLC_ACC_GYRO_DSD_ZH_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_ZH(void *handle, ISM330DLC_ACC_GYRO_DSD_ZH_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: D6D_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_D6D_EV_STATUS_NOT_DETECTED 	 =0x00,
  	ISM330DLC_ACC_GYRO_D6D_EV_STATUS_DETECTED 		 =0x40,
} ISM330DLC_ACC_GYRO_D6D_EV_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_D6D_EV_STATUS_MASK  	0x40
mems_status_t ISM330DLC_ACC_GYRO_R_D6D_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_D6D_EV_STATUS_t *value);

/*******************************************************************************
* Register      : D6D_SRC
* Address       : 0X1D
* Bit Group Name: DEN_DRDY
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_DEN_DRDY_NOT_ACTIVE 		 =0x00,
  	ISM330DLC_ACC_GYRO_DEN_DRDY_ACTIVE 			 =0x80,
} ISM330DLC_ACC_GYRO_DEN_DRDY_t;

#define  	ISM330DLC_ACC_GYRO_DEN_DRDY_STATUS_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_R_DEN_DRDY_STATUS(void *handle, ISM330DLC_ACC_GYRO_DEN_DRDY_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X1E
* Bit Group Name: XLDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_XLDA_NO_DATA_AVAIL 	 =0x00,
  	ISM330DLC_ACC_GYRO_XLDA_DATA_AVAIL 		 =0x01,
} ISM330DLC_ACC_GYRO_XLDA_t;

#define  	ISM330DLC_ACC_GYRO_XLDA_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_R_XLDA(void *handle, ISM330DLC_ACC_GYRO_XLDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X1E
* Bit Group Name: GDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_GDA_NO_DATA_AVAIL 	 =0x00,
  	ISM330DLC_ACC_GYRO_GDA_DATA_AVAIL 		 =0x02,
} ISM330DLC_ACC_GYRO_GDA_t;

#define  	ISM330DLC_ACC_GYRO_GDA_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_R_GDA(void *handle, ISM330DLC_ACC_GYRO_GDA_t *value);

/*******************************************************************************
* Register      : STATUS_REG
* Address       : 0X1E
* Bit Group Name: TDA
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_TDA_NO_DATA_AVAIL 	 =0x00,
  	ISM330DLC_ACC_GYRO_TDA_DATA_AVAIL 		 =0x04,
} ISM330DLC_ACC_GYRO_TDA_t;

#define  	ISM330DLC_ACC_GYRO_TDA_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_R_TDA(void *handle, ISM330DLC_ACC_GYRO_TDA_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS1
* Address       : 0X3A
* Bit Group Name: DIFF_FIFO
* Permission    : RO
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_DIFF_FIFO_STATUS1_MASK  		0xFF
#define  	ISM330DLC_ACC_GYRO_DIFF_FIFO_STATUS1_POSITION  	0
#define  	ISM330DLC_ACC_GYRO_DIFF_FIFO_STATUS2_MASK   	0x07
#define  	ISM330DLC_ACC_GYRO_DIFF_FIFO_STATUS2_POSITION  	0
mems_status_t ISM330DLC_ACC_GYRO_R_FIFONumOfEntries(void *handle, u16_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0X3B
* Bit Group Name: FIFO_EMPTY
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_FIFO_EMPTY_FIFO_NOT_EMPTY 	 =0x00,
  	ISM330DLC_ACC_GYRO_FIFO_EMPTY_FIFO_EMPTY 		 =0x10,
} ISM330DLC_ACC_GYRO_FIFO_EMPTY_t;

#define  	ISM330DLC_ACC_GYRO_FIFO_EMPTY_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_R_FIFOEmpty(void *handle, ISM330DLC_ACC_GYRO_FIFO_EMPTY_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0X3B
* Bit Group Name: FIFO_FULL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_FIFO_FULL_FIFO_NOT_FULL 		 =0x00,
  	ISM330DLC_ACC_GYRO_FIFO_FULL_FIFO_FULL 			 =0x20,
} ISM330DLC_ACC_GYRO_FIFO_FULL_t;

#define  	ISM330DLC_ACC_GYRO_FIFO_FULL_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_R_FIFOFull(void *handle, ISM330DLC_ACC_GYRO_FIFO_FULL_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0X3B
* Bit Group Name: OVERRUN
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_OVERRUN_NO_OVERRUN 		 =0x00,
  	ISM330DLC_ACC_GYRO_OVERRUN_OVERRUN 			 =0x40,
} ISM330DLC_ACC_GYRO_OVERRUN_t;

#define  	ISM330DLC_ACC_GYRO_OVERRUN_MASK  	0x40
mems_status_t ISM330DLC_ACC_GYRO_R_OVERRUN(void *handle, ISM330DLC_ACC_GYRO_OVERRUN_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS2
* Address       : 0X3B
* Bit Group Name: WTM
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_WTM_BELOW_WTM 			 =0x00,
  	ISM330DLC_ACC_GYRO_WTM_ABOVE_OR_EQUAL_WTM 	 =0x80,
} ISM330DLC_ACC_GYRO_WTM_t;

#define  	ISM330DLC_ACC_GYRO_WTM_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_R_WaterMark(void *handle, ISM330DLC_ACC_GYRO_WTM_t *value);

/*******************************************************************************
* Register      : FIFO_STATUS3
* Address       : 0X3C
* Bit Group Name: FIFO_PATTERN
* Permission    : RO
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_FIFO_STATUS3_PATTERN_MASK  		0xFF
#define  	ISM330DLC_ACC_GYRO_FIFO_STATUS3_PATTERN_POSITION  	0
#define  	ISM330DLC_ACC_GYRO_FIFO_STATUS4_PATTERN_MASK  		0x03
#define  	ISM330DLC_ACC_GYRO_FIFO_STATUS4_PATTERN_POSITION  	0
mems_status_t ISM330DLC_ACC_GYRO_R_FIFOPattern(void *handle, u16_t *value);

/*******************************************************************************
* Register      : FUNC_SRC1
* Address       : 0X53
* Bit Group Name: SENS_HUB_END
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_SENS_HUB_END_STILL_ONGOING 		 =0x00,
  	ISM330DLC_ACC_GYRO_SENS_HUB_END_OP_COMPLETED 		 =0x01,
} ISM330DLC_ACC_GYRO_SENS_HUB_END_t;

#define  	ISM330DLC_ACC_GYRO_SENS_HUB_END_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_R_SENS_HUB_END(void *handle, ISM330DLC_ACC_GYRO_SENS_HUB_END_t *value);

/*******************************************************************************
* Register      : FUNC_SRC1
* Address       : 0X53
* Bit Group Name: SOFT_IRON_END
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_SOFT_IRON_END_NOT_COMPLETED 		 =0x00,
  	ISM330DLC_ACC_GYRO_SOFT_IRON_END_COMPLETED 			 =0x02,
} ISM330DLC_ACC_GYRO_SOFT_IRON_END_t;

#define  	ISM330DLC_ACC_GYRO_SOFT_IRON_END_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_R_SOFT_IRON_END(void *handle, ISM330DLC_ACC_GYRO_SOFT_IRON_END_t *value);

/*******************************************************************************
* Register      : FUNC_SRC1
* Address       : 0X53
* Bit Group Name: HI_FAIL
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_HARD_IRON_NORMAL 		 =0x00,
  	ISM330DLC_ACC_GYRO_HARD_IRON_FAIL 			 =0x04,
} ISM330DLC_ACC_GYRO_SOFT_HARD_IRON_STAT_t;

#define  	ISM330DLC_ACC_GYRO_HARD_IRON_STAT_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_R_HardIron(void *handle, ISM330DLC_ACC_GYRO_SOFT_HARD_IRON_STAT_t *value);

/*******************************************************************************
* Register      : FUNC_SRC1
* Address       : 0X53
* Bit Group Name: TILT_EV_STATUS
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_TILT_EV_STATUS_NOT_DETECTED 		 =0x00,
  	ISM330DLC_ACC_GYRO_TILT_EV_STATUS_DETECTED 			 =0x20,
} ISM330DLC_ACC_GYRO_TILT_EV_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_TILT_EV_STATUS_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_R_TILT_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_TILT_EV_STATUS_t *value);

/*******************************************************************************
* Register      : FUNC_SRC2
* Address       : 0X54
* Bit Group Name: SLAVE0_NACK
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_NACK_OCCUR_SLAVE0 			 =0x00,
  	ISM330DLC_ACC_GYRO_NACK_NOT_OCCUR_SLAVE0 		 =0x08,
} ISM330DLC_ACC_GYRO_SLAVE0_NACK_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_SLAVE0_NACK_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_R_SLAVE0_NACK_STATUS(void *handle, ISM330DLC_ACC_GYRO_SLAVE0_NACK_STATUS_t *value);

/*******************************************************************************
* Register      : FUNC_SRC2
* Address       : 0X54
* Bit Group Name: SLAVE1_NACK
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_NACK_OCCUR_SLAVE1 			 =0x00,
  	ISM330DLC_ACC_GYRO_NACK_NOT_OCCUR_SLAVE1 		 =0x10,
} ISM330DLC_ACC_GYRO_SLAVE1_NACK_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_SLAVE1_NACK_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_R_SLAVE1_NACK_STATUS(void *handle, ISM330DLC_ACC_GYRO_SLAVE1_NACK_STATUS_t *value);


/*******************************************************************************
* Register      : FUNC_SRC2
* Address       : 0X54
* Bit Group Name: SLAVE2_NACK
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_NACK_OCCUR_SLAVE2 			 =0x00,
  	ISM330DLC_ACC_GYRO_NACK_NOT_OCCUR_SLAVE2 		 =0x10,
} ISM330DLC_ACC_GYRO_SLAVE2_NACK_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_SLAVE2_NACK_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_R_SLAVE2_NACK_STATUS(void *handle, ISM330DLC_ACC_GYRO_SLAVE2_NACK_STATUS_t *value);

/*******************************************************************************
* Register      : FUNC_SRC2
* Address       : 0X54
* Bit Group Name: SLAVE3_NACK
* Permission    : RO
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_NACK_OCCUR_SLAVE3 			 =0x00,
  	ISM330DLC_ACC_GYRO_NACK_NOT_OCCUR_SLAVE3 		 =0x10,
} ISM330DLC_ACC_GYRO_SLAVE3_NACK_STATUS_t;

#define  	ISM330DLC_ACC_GYRO_SLAVE3_NACK_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_R_SLAVE3_NACK_STATUS(void *handle, ISM330DLC_ACC_GYRO_SLAVE3_NACK_STATUS_t *value);

/*******************************************************************************
* Register      : TAP_CFG
* Address       : 0X58
* Bit Group Name: LIR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_LIR_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_LIR_ENABLED 			 =0x01,
} ISM330DLC_ACC_GYRO_LIR_t;

#define  	ISM330DLC_ACC_GYRO_LIR_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_W_LIR(void *handle, ISM330DLC_ACC_GYRO_LIR_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_LIR(void *handle, ISM330DLC_ACC_GYRO_LIR_t *value);

/*******************************************************************************
* Register      : TAP_CFG
* Address       : 0X58
* Bit Group Name: TAP_Z_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_TAP_Z_EN_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_TAP_Z_EN_ENABLED 		 =0x02,
} ISM330DLC_ACC_GYRO_TAP_Z_EN_t;

#define  	ISM330DLC_ACC_GYRO_TAP_Z_EN_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_W_TAP_Z_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_Z_EN_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_Z_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_Z_EN_t *value);

/*******************************************************************************
* Register      : TAP_CFG
* Address       : 0X58
* Bit Group Name: TAP_Y_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_TAP_Y_EN_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_TAP_Y_EN_ENABLED 		 =0x04,
} ISM330DLC_ACC_GYRO_TAP_Y_EN_t;

#define  	ISM330DLC_ACC_GYRO_TAP_Y_EN_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_TAP_Y_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_Y_EN_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_Y_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_Y_EN_t *value);

/*******************************************************************************
* Register      : TAP_CFG
* Address       : 0X58
* Bit Group Name: TAP_X_EN
* Permission    : RW
*******************************************************************************/

typedef enum {
  	ISM330DLC_ACC_GYRO_TAP_X_EN_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_TAP_X_EN_ENABLED 		 =0x08,
} ISM330DLC_ACC_GYRO_TAP_X_EN_t;

#define  	ISM330DLC_ACC_GYRO_TAP_X_EN_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_W_TAP_X_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_X_EN_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_X_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_X_EN_t *value);

/*******************************************************************************
* Register      : TAP_CFG
* Address       : 0X58
* Bit Group Name: SLOPE_FDS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_SLOPE_FILTER_APPLIED 		 =0x00,
  	ISM330DLC_ACC_GYRO_SLOPE_HPF_APPLIED 			 =0x10,
} ISM330DLC_ACC_GYRO_SLOPE_FDS_t;

#define  	ISM330DLC_ACC_GYRO_SLOPE_FDS_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_SLOPE_FDS(void *handle, ISM330DLC_ACC_GYRO_SLOPE_FDS_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SLOPE_FDS(void *handle, ISM330DLC_ACC_GYRO_SLOPE_FDS_t *value);

/*******************************************************************************
* Register      : TAP_CFG
* Address       : 0X58
* Bit Group Name: INACT_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INACT_DISABLE 					 =0x00,
  	ISM330DLC_ACC_GYRO_ACC_ODR_12HZ5_GYRO_NOT_CHANGE 	 =0x20,
  	ISM330DLC_ACC_GYRO_ACC_ODR_12HZ5_GYRO_SLEEP 		 =0x40,
  	ISM330DLC_ACC_GYRO_ACC_ODR_12HZ5_GYRO_POWER_DOWN 	 =0x60,
} ISM330DLC_ACC_GYRO_INACT_EN_t;

#define  	ISM330DLC_ACC_GYRO_INACT_ENS_MASK  	0x60
mems_status_t ISM330DLC_ACC_GYRO_W_INACT_EN(void *handle, ISM330DLC_ACC_GYRO_INACT_EN_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_INACT_EN(void *handle, ISM330DLC_ACC_GYRO_INACT_EN_t *value);

/*******************************************************************************
* Register      : TAP_CFG
* Address       : 0X58
* Bit Group Name: INTERRUPTS_ENABLE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_BASIC_INT_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_BASIC_INT_ENABLED 		 =0x80,
} ISM330DLC_ACC_GYRO_INT_EN_t;

#define  	ISM330DLC_ACC_GYRO_INT_EN_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_BASIC_INT(void *handle, ISM330DLC_ACC_GYRO_INT_EN_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_BASIC_INT(void *handle, ISM330DLC_ACC_GYRO_INT_EN_t *value);

/*******************************************************************************
* Register      : TAP_THS_6D
* Address       : 0X59
* Bit Group Name: TAP_THS
* Permission    : RW
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_TAP_THS_MASK  		0x1F
#define  	ISM330DLC_ACC_GYRO_TAP_THS_POSITION  	0
mems_status_t ISM330DLC_ACC_GYRO_W_TAP_THS(void *handle, u8_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_THS(void *handle, u8_t *value);

/*******************************************************************************
* Register      : TAP_THS_6D
* Address       : 0X59
* Bit Group Name: SIXD_THS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_SIXD_THS_80_degree 		 =0x00,
  	ISM330DLC_ACC_GYRO_SIXD_THS_70_degree 		 =0x20,
  	ISM330DLC_ACC_GYRO_SIXD_THS_60_degree 		 =0x40,
  	ISM330DLC_ACC_GYRO_SIXD_THS_50_degree 		 =0x60,
} ISM330DLC_ACC_GYRO_SIXD_THS_t;

#define  	ISM330DLC_ACC_GYRO_SIXD_THS_MASK  	0x60
mems_status_t ISM330DLC_ACC_GYRO_W_SIXD_THS(void *handle, ISM330DLC_ACC_GYRO_SIXD_THS_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SIXD_THS(void *handle, ISM330DLC_ACC_GYRO_SIXD_THS_t *value);

/*******************************************************************************
* Register      : TAP_THS_6D
* Address       : 0X59
* Bit Group Name: D4D_EN
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_D4D_DIS 		 =0x00,
  	ISM330DLC_ACC_GYRO_D4D_EN 		 =0x80,
} ISM330DLC_ACC_GYRO_D4D_t;

#define  	ISM330DLC_ACC_GYRO_D4D_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_D4D(void *handle, ISM330DLC_ACC_GYRO_D4D_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_D4D(void *handle, ISM330DLC_ACC_GYRO_D4D_t *value);

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0X5A
* Bit Group Name: SHOCK
* Permission    : RW
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_SHOCK_MASK  	0x03
#define  	ISM330DLC_ACC_GYRO_SHOCK_POSITION  	0
mems_status_t ISM330DLC_ACC_GYRO_W_SHOCK_Duration(void *handle, u8_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SHOCK_Duration(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0X5A
* Bit Group Name: QUIET
* Permission    : RW
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_QUIET_MASK  	0x0C
#define  	ISM330DLC_ACC_GYRO_QUIET_POSITION  	2
mems_status_t ISM330DLC_ACC_GYRO_W_QUIET_Duration(void *handle, u8_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_QUIET_Duration(void *handle, u8_t *value);

/*******************************************************************************
* Register      : INT_DUR2
* Address       : 0X5A
* Bit Group Name: DUR
* Permission    : RW
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_DUR_MASK  	0xF0
#define  	ISM330DLC_ACC_GYRO_DUR_POSITION  	4
mems_status_t ISM330DLC_ACC_GYRO_W_DUR(void *handle, u8_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_DUR(void *handle, u8_t *value);

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0X5B
* Bit Group Name: WK_THS
* Permission    : RW
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_WK_THS_MASK  	0x3F
#define  	ISM330DLC_ACC_GYRO_WK_THS_POSITION  	0
mems_status_t ISM330DLC_ACC_GYRO_W_WK_THS(void *handle, u8_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_WK_THS(void *handle, u8_t *value);

/*******************************************************************************
* Register      : WAKE_UP_THS
* Address       : 0X5B
* Bit Group Name: SINGLE_DOUBLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_SINGLE_TAP 		 =0x00,
  	ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP 		 =0x80,
} ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_t;

#define  	ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV(void *handle, ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SINGLE_DOUBLE_TAP_EV(void *handle, ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_t *value);

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0X5C
* Bit Group Name: SLEEP_DUR
* Permission    : RW
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_SLEEP_DUR_MASK  	0x0F
#define  	ISM330DLC_ACC_GYRO_SLEEP_DUR_POSITION  	0
mems_status_t ISM330DLC_ACC_GYRO_W_SLEEP_DUR(void *handle, u8_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SLEEP_DUR(void *handle, u8_t *value);

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0X5C
* Bit Group Name: TIMER_HR
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_TIMER_HR_6_4ms 		 =0x00,
  	ISM330DLC_ACC_GYRO_TIMER_HR_25us 		 =0x10,
} ISM330DLC_ACC_GYRO_TIMER_HR_t;

#define  	ISM330DLC_ACC_GYRO_TIMER_HR_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_TIMER_HR(void *handle, ISM330DLC_ACC_GYRO_TIMER_HR_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TIMER_HR(void *handle, ISM330DLC_ACC_GYRO_TIMER_HR_t *value);

/*******************************************************************************
* Register      : WAKE_UP_DUR
* Address       : 0X5C
* Bit Group Name: WAKE_DUR
* Permission    : RW
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_WAKE_DUR_MASK  	0x60
#define  	ISM330DLC_ACC_GYRO_WAKE_DUR_POSITION  	5
mems_status_t ISM330DLC_ACC_GYRO_W_WAKE_DUR(void *handle, u8_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_WAKE_DUR(void *handle, u8_t *value);

/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0X5D
* Bit Group Name: FF_DUR
* Permission    : RW
*******************************************************************************/
#define  	ISM330DLC_ACC_GYRO_FF_FREE_FALL_DUR_MASK  	0xF8
#define  	ISM330DLC_ACC_GYRO_FF_FREE_FALL_DUR_POSITION  	3
#define  	ISM330DLC_ACC_GYRO_FF_WAKE_UP_DUR_MASK  	0x80
#define  	ISM330DLC_ACC_GYRO_FF_WAKE_UP_DUR_POSITION  	7
mems_status_t ISM330DLC_ACC_GYRO_W_FF_Duration(void *handle, u8_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FF_Duration(void *handle, u8_t *value);


/*******************************************************************************
* Register      : FREE_FALL
* Address       : 0X5D
* Bit Group Name: FF_THS
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_FF_THS_156mg 		 =0x00,
  	ISM330DLC_ACC_GYRO_FF_THS_219mg 		 =0x01,
  	ISM330DLC_ACC_GYRO_FF_THS_250mg 		 =0x02,
  	ISM330DLC_ACC_GYRO_FF_THS_312mg 		 =0x03,
  	ISM330DLC_ACC_GYRO_FF_THS_344mg 		 =0x04,
  	ISM330DLC_ACC_GYRO_FF_THS_406mg 		 =0x05,
  	ISM330DLC_ACC_GYRO_FF_THS_469mg 		 =0x06,
  	ISM330DLC_ACC_GYRO_FF_THS_500mg 		 =0x07,
} ISM330DLC_ACC_GYRO_FF_THS_t;

#define  	ISM330DLC_ACC_GYRO_FF_THS_MASK  	0x07
mems_status_t ISM330DLC_ACC_GYRO_W_FF_THS(void *handle, ISM330DLC_ACC_GYRO_FF_THS_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FF_THS(void *handle, ISM330DLC_ACC_GYRO_FF_THS_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_TIMER
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_TIMER_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_TIMER_ENABLED 		 =0x01,
} ISM330DLC_ACC_GYRO_INT1_TIMER_t;

#define  	ISM330DLC_ACC_GYRO_INT1_TIMER_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_W_TimerEvRouteInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_TIMER_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TimerEvRouteInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_TIMER_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_TILT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_TILT_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_TILT_ENABLED 		 =0x02,
} ISM330DLC_ACC_GYRO_INT1_TILT_t;

#define  	ISM330DLC_ACC_GYRO_INT1_TILT_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_W_TiltEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_TILT_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TiltEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_TILT_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_6D
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_6D_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_6D_ENABLED 			 =0x04,
} ISM330DLC_ACC_GYRO_INT1_6D_t;

#define  	ISM330DLC_ACC_GYRO_INT1_6D_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_6DEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_6D_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_6DEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_6D_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_DOUBLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_DOUBLE_TAP_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_DOUBLE_TAP_ENABLED 			 =0x08,
} ISM330DLC_ACC_GYRO_INT1_DOUBLE_TAP_t;

#define  	ISM330DLC_ACC_GYRO_INT1_DOUBLE_TAP_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_W_TapEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_DOUBLE_TAP_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TapEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_DOUBLE_TAP_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_FF
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_FF_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_FF_ENABLED 			 =0x10,
} ISM330DLC_ACC_GYRO_INT1_FF_t;

#define  	ISM330DLC_ACC_GYRO_INT1_FF_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_FFEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_FF_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FFEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_FF_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_WU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_WU_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_WU_ENABLED 			 =0x20,
} ISM330DLC_ACC_GYRO_INT1_WU_t;

#define  	ISM330DLC_ACC_GYRO_INT1_WU_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_W_WUEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_WU_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_WUEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_WU_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_SINGLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_DISABLED 	 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_ENABLED 		 =0x40,
} ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_t;

#define  	ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_MASK  	0x40
mems_status_t ISM330DLC_ACC_GYRO_W_SingleTapOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SingleTapOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_t *value);

/*******************************************************************************
* Register      : MD1_CFG
* Address       : 0X5E
* Bit Group Name: INT1_INACT_STATE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT1_SLEEP_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT1_SLEEP_ENABLED 		 =0x80,
} ISM330DLC_ACC_GYRO_INT1_SLEEP_t;

#define  	ISM330DLC_ACC_GYRO_INT1_SLEEP_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_SleepEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_SLEEP_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SleepEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_SLEEP_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_IRON
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_IRON_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_IRON_ENABLED 		 =0x01,
} ISM330DLC_ACC_GYRO_INT2_IRON_t;

#define  	ISM330DLC_ACC_GYRO_INT2_IRON_MASK  	0x01
mems_status_t ISM330DLC_ACC_GYRO_W_MagCorrection_Int2(void *handle, ISM330DLC_ACC_GYRO_INT2_IRON_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_MagCorrection_Int2(void *handle, ISM330DLC_ACC_GYRO_INT2_IRON_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_TILT
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_TILT_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_TILT_ENABLED 		 =0x02,
} ISM330DLC_ACC_GYRO_INT2_TILT_t;

#define  	ISM330DLC_ACC_GYRO_INT2_TILT_MASK  	0x02
mems_status_t ISM330DLC_ACC_GYRO_W_TiltEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_TILT_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TiltEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_TILT_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_6D
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_6D_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_6D_ENABLED 			 =0x04,
} ISM330DLC_ACC_GYRO_INT2_6D_t;

#define  	ISM330DLC_ACC_GYRO_INT2_6D_MASK  	0x04
mems_status_t ISM330DLC_ACC_GYRO_W_6DEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_6D_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_6DEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_6D_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_DOUBLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_DOUBLE_TAP_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_DOUBLE_TAP_ENABLED 			 =0x08,
} ISM330DLC_ACC_GYRO_INT2_DOUBLE_TAP_t;

#define  	ISM330DLC_ACC_GYRO_INT2_DOUBLE_TAP_MASK  	0x08
mems_status_t ISM330DLC_ACC_GYRO_W_TapEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_DOUBLE_TAP_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_TapEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_DOUBLE_TAP_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_FF
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_FF_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_FF_ENABLED 			 =0x10,
} ISM330DLC_ACC_GYRO_INT2_FF_t;

#define  	ISM330DLC_ACC_GYRO_INT2_FF_MASK  	0x10
mems_status_t ISM330DLC_ACC_GYRO_W_FFEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_FF_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_FFEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_FF_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_WU
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_WU_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_WU_ENABLED 			 =0x20,
} ISM330DLC_ACC_GYRO_INT2_WU_t;

#define  	ISM330DLC_ACC_GYRO_INT2_WU_MASK  	0x20
mems_status_t ISM330DLC_ACC_GYRO_W_WUEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_WU_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_WUEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_WU_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_SINGLE_TAP
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_DISABLED 	 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_ENABLED 		 =0x40,
} ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_t;

#define  	ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_MASK  	0x40
mems_status_t ISM330DLC_ACC_GYRO_W_SingleTapOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SingleTapOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_t *value);

/*******************************************************************************
* Register      : MD2_CFG
* Address       : 0X5F
* Bit Group Name: INT2_INACT_STATE
* Permission    : RW
*******************************************************************************/
typedef enum {
  	ISM330DLC_ACC_GYRO_INT2_SLEEP_DISABLED 		 =0x00,
  	ISM330DLC_ACC_GYRO_INT2_SLEEP_ENABLED 		 =0x80,
} ISM330DLC_ACC_GYRO_INT2_SLEEP_t;

#define  	ISM330DLC_ACC_GYRO_INT2_SLEEP_MASK  	0x80
mems_status_t ISM330DLC_ACC_GYRO_W_SleepEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_SLEEP_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SleepEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_SLEEP_t *value);

/*******************************************************************************
* Register      : MASTER_CMD_CODE
* Address       : 0X60
* Bit Group Name: MASTER_CMD_CODE
* Permission    : RW
*******************************************************************************/

#define  	ISM330DLC_ACC_GYRO_MASTER_CMD_CODE_MASK  	0xFF
mems_status_t ISM330DLC_ACC_GYRO_W_MASTER_CMD_CODE(void *handle, u8_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_MASTER_CMD_CODE(void *handle, u8_t *value);

/*******************************************************************************
* Register      : SENS_SYNC_SPI_ERROR_CODE
* Address       : 0X60
* Bit Group Name: SENS_SYNC_SPI_ERROR_CODE
* Permission    : RW
*******************************************************************************/

#define  	ISM330DLC_ACC_GYRO_SENS_SYNC_SPI_ERROR_CODE_MASK  	0xFF
mems_status_t ISM330DLC_ACC_GYRO_W_SENS_SYNC_SPI_ERROR_CODE(void *handle, u8_t newValue);
mems_status_t ISM330DLC_ACC_GYRO_R_SENS_SYNC_SPI_ERROR_CODE(void *handle, u8_t *value);

/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetFIFOData
* Permission    : RO
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_Get_GetFIFOData(void *handle, u8_t *buff);
/*******************************************************************************
* Register      : <REGISTER_L> - <REGISTER_H>
* Output Type   : GetTimestamp
* Permission    : RO
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_Get_GetTimestamp(void *handle, u8_t *buff);

/************** Use Sensor Hub  *******************/

/* program to .... */
mems_status_t ISM330DLC_ACC_GYRO_SH0_Program(void *handle, u8_t SlvAddr, u8_t Reg, u8_t len);

/* Program the six Soft Iron Matrix coefficients. */
mems_status_t ISM330DLC_ACC_GYRO_SH_init_SI_Matrix(void *handle, u8_t *SI_matrix);

/* Read a remote device through I2C Sensor Hub Slave 0 */
mems_status_t ISM330DLC_ACC_GYRO_SH0_ReadMem(void *handle, u8_t SlvAddr, u8_t Reg, u8_t *Bufp, u8_t len, u8_t stop);

/* Write a remote device through I2C Sensor Hub Slave 0 */
mems_status_t ISM330DLC_ACC_GYRO_SH0_WriteByte(void *handle, u8_t SlvAddr, u8_t Reg, u8_t Bufp);

#ifdef __cplusplus
}
#endif

#endif
