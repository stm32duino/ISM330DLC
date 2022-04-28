/**
 ******************************************************************************
 * @file    ISM330DLC_ACC_GYRO_driver.c
 * @author  MEMS Application Team
 * @version V1.0.0
 * @date    13-January-2020
 * @brief   ISM330DLC driver file
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

/* Includes ------------------------------------------------------------------*/
#include "ISM330DLC_ACC_GYRO_Driver.h"   

/* Imported function prototypes ----------------------------------------------*/
extern uint8_t ISM330DLC_IO_Write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
extern uint8_t ISM330DLC_IO_Read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/

/************** Generic Function  *******************/

/*******************************************************************************
* Function Name		: ISM330DLC_ACC_GYRO_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either
*					: I2C or SPI reading functions
* Input				: Register Address, length of buffer
* Output			: Data REad
* Return			: None
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_ReadReg(void *handle, u8_t Reg, u8_t* Data, u16_t len)
{
  if (ISM330DLC_IO_Read(handle, Reg, Data, len))
  {
    return MEMS_ERROR;
  }
  else
  {
    return MEMS_SUCCESS;
  }
}

/*******************************************************************************
* Function Name		: ISM330DLC_ACC_GYRO_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*					: I2C or SPI writing function
* Input				: Register Address, Data to be written, length of buffer
* Output			: None
* Return			: None
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_WriteReg(void *handle, u8_t Reg, u8_t *Data, u16_t len)
{
  if (ISM330DLC_IO_Write(handle, Reg, Data, len))
  {
    return MEMS_ERROR;
  }
  else
  {
    return MEMS_SUCCESS;
  }
}

/**************** Base Function  *******************/

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_WHO_AM_I
* Description    : Read WHO_AM_I_BIT
* Input          : Pointer to u8_t
* Output         : Status of WHO_AM_I_BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_WHO_AM_I(void *handle, u8_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WHO_AM_I_REG, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_WHO_AM_I_BIT_MASK; //coerce
  *value = *value >> ISM330DLC_ACC_GYRO_WHO_AM_I_BIT_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_BDU
* Description    : Write BDU
* Input          : ISM330DLC_ACC_GYRO_BDU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_BDU(void *handle, ISM330DLC_ACC_GYRO_BDU_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_BDU_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_BDU
* Description    : Read BDU
* Input          : Pointer to ISM330DLC_ACC_GYRO_BDU_t
* Output         : Status of BDU see ISM330DLC_ACC_GYRO_BDU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_BDU(void *handle, ISM330DLC_ACC_GYRO_BDU_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_BDU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FS_XL
* Description    : Write FS_XL
* Input          : ISM330DLC_ACC_GYRO_FS_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FS_XL(void *handle, ISM330DLC_ACC_GYRO_FS_XL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_FS_XL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FS_XL
* Description    : Read FS_XL
* Input          : Pointer to ISM330DLC_ACC_GYRO_FS_XL_t
* Output         : Status of FS_XL see ISM330DLC_ACC_GYRO_FS_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FS_XL(void *handle, ISM330DLC_ACC_GYRO_FS_XL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FS_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : mems_status_t ISM330DLC_ACC_GYRO_GetRawAccData(u8_t *buff)
* Description    : Read GetAccData output register
* Input          : pointer to [u8_t]
* Output         : GetAccData buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_GetRawAccData(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ )
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{
		if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_OUTX_L_XL+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;
	}
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : mems_status_t ISM330DLC_ACC_Get_Acceleration(void *handle, int *buff, u8_t from_fifo)
* Description    : Read GetAccData output register
* Input          : pointer to [u8_t]
* Output         : values are expressed in mg
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
/*
 * Following is the table of sensitivity values for each case.
 * Values are expressed in ug/digit.
 */
static const long long ISM330DLC_ACC_Sensitivity_List[4] = {
      61,	/* FS @2g */
      122,	/* FS @4g */
      244,	/* FS @8g */
      488,	/* FS @16g */
};
mems_status_t ISM330DLC_ACC_Get_Acceleration(void *handle, int *buff, u8_t from_fifo)
{
  ISM330DLC_ACC_GYRO_FS_XL_t fs;
  long long sensitivity = 0;
  Type3Axis16bit_U raw_data_tmp;

  /* Read out current odr, fs, hf setting */
  ISM330DLC_ACC_GYRO_R_FS_XL(handle, &fs);

  /* Determine the sensitivity according to fs */
  switch(fs) {
  case ISM330DLC_ACC_GYRO_FS_XL_2g:
    sensitivity = ISM330DLC_ACC_Sensitivity_List[0];
    break;

  case ISM330DLC_ACC_GYRO_FS_XL_4g:
    sensitivity = ISM330DLC_ACC_Sensitivity_List[1];
    break;

  case ISM330DLC_ACC_GYRO_FS_XL_8g:
    sensitivity = ISM330DLC_ACC_Sensitivity_List[2];
    break;

  case ISM330DLC_ACC_GYRO_FS_XL_16g:
    sensitivity = ISM330DLC_ACC_Sensitivity_List[3];
    break;
  }

  /* Read out raw accelerometer samples */
  if (from_fifo) {
    u8_t i;

    /* read all 3 axis from FIFO */
    for(i = 0; i < 3; i++)
      ISM330DLC_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp.u8bit + 2*i);
  } else
    ISM330DLC_ACC_GYRO_GetRawAccData(handle, raw_data_tmp.u8bit);

  /* Apply proper shift and sensitivity */
  buff[0] = (raw_data_tmp.i16bit[0] * sensitivity + 500)/1000;
  buff[1] = (raw_data_tmp.i16bit[1] * sensitivity + 500)/1000;
  buff[2] = (raw_data_tmp.i16bit[2] * sensitivity + 500)/1000;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_ODR_XL
* Description    : Write ODR_XL
* Input          : ISM330DLC_ACC_GYRO_ODR_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_ODR_XL(void *handle, ISM330DLC_ACC_GYRO_ODR_XL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_ODR_XL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_ODR_XL
* Description    : Read ODR_XL
* Input          : Pointer to ISM330DLC_ACC_GYRO_ODR_XL_t
* Output         : Status of ODR_XL see ISM330DLC_ACC_GYRO_ODR_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_ODR_XL(void *handle, ISM330DLC_ACC_GYRO_ODR_XL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_ODR_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_translate_ODR_XL
* Description    : Read ODR_XL
* Input          : ISM330DLC_ACC_GYRO_ODR_XL_t
* Output         : The ODR value in Hz
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_translate_ODR_XL(ISM330DLC_ACC_GYRO_ODR_XL_t value, u16_t *odr_hz_val)
{
  switch(value) {
  case ISM330DLC_ACC_GYRO_ODR_XL_POWER_DOWN:
    *odr_hz_val = 0;
    break;

  case ISM330DLC_ACC_GYRO_ODR_XL_1Hz6:
    *odr_hz_val = 2;
    break;

  case ISM330DLC_ACC_GYRO_ODR_XL_12Hz5:
    *odr_hz_val = 13;
    break;

  case ISM330DLC_ACC_GYRO_ODR_XL_26Hz:
    *odr_hz_val = 26;
    break;

  case ISM330DLC_ACC_GYRO_ODR_XL_52Hz:
    *odr_hz_val = 52;
    break;

  case ISM330DLC_ACC_GYRO_ODR_XL_104Hz:
    *odr_hz_val = 104;
    break;

  case ISM330DLC_ACC_GYRO_ODR_XL_208Hz:
    *odr_hz_val = 208;
    break;

  case ISM330DLC_ACC_GYRO_ODR_XL_416Hz:
    *odr_hz_val = 416;
    break;

  case ISM330DLC_ACC_GYRO_ODR_XL_833Hz:
    *odr_hz_val = 833;
    break;

  case ISM330DLC_ACC_GYRO_ODR_XL_1660Hz:
    *odr_hz_val = 1660;
    break;

  default:
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FS_G
* Description    : Write FS_G
* Input          : ISM330DLC_ACC_GYRO_FS_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FS_G(void *handle, ISM330DLC_ACC_GYRO_FS_G_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL2_G, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_FS_G_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL2_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FS_G
* Description    : Read FS_G
* Input          : Pointer to ISM330DLC_ACC_GYRO_FS_G_t
* Output         : Status of FS_G see ISM330DLC_ACC_GYRO_FS_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FS_G(void *handle, ISM330DLC_ACC_GYRO_FS_G_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL2_G, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FS_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : mems_status_t ISM330DLC_ACC_GYRO_GetRawGyroData(u8_t *buff)
* Description    : Read GetGyroData output register
* Input          : pointer to [u8_t]
* Output         : GetGyroData buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_GetRawGyroData(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ )
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{
		if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_OUTX_L_G+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;
	}
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : mems_status_t ISM330DLC_ACC_Get_AngularRate(u8_t *buff)
* Description    : Read GetGyroData output register
* Input          : pointer to [u8_t]
* Output         : Returned values are espressed in mdps
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
/*
 * Following is the table of sensitivity values for each case.
 * Values are espressed in udps/digit.
 */
static const long long ISM330DLC_GYRO_Sensitivity_List[5] = {
      4375,	/* FS @125 */
      8750,	/* FS @250 */
      17500,	/* FS @500 */
      35000,	/* FS @1000 */
      70000,	/* FS @2000 */
};
mems_status_t ISM330DLC_ACC_Get_AngularRate(void *handle, int *buff, u8_t from_fifo)
{
  ISM330DLC_ACC_GYRO_FS_125_t fs_125;
  ISM330DLC_ACC_GYRO_FS_G_t fs;
  long long sensitivity = 0;
  Type3Axis16bit_U raw_data_tmp;

  /* Read out current odr, fs, hf setting */
  ISM330DLC_ACC_GYRO_R_FS_125(handle, &fs_125);
  if (fs_125 == ISM330DLC_ACC_GYRO_FS_125_ENABLED) {
    sensitivity = ISM330DLC_GYRO_Sensitivity_List[0];
  } else {
    ISM330DLC_ACC_GYRO_R_FS_G(handle, &fs);

    /* Determine the sensitivity according to fs */
    switch(fs) {
    case ISM330DLC_ACC_GYRO_FS_G_250dps:
      sensitivity = ISM330DLC_GYRO_Sensitivity_List[1];
      break;

    case ISM330DLC_ACC_GYRO_FS_G_500dps:
      sensitivity = ISM330DLC_GYRO_Sensitivity_List[2];
      break;

    case ISM330DLC_ACC_GYRO_FS_G_1000dps:
      sensitivity = ISM330DLC_GYRO_Sensitivity_List[3];
      break;

    case ISM330DLC_ACC_GYRO_FS_G_2000dps:
      sensitivity = ISM330DLC_GYRO_Sensitivity_List[4];
      break;
    }
  }

  /* Read out raw accelerometer samples */
  if (from_fifo) {
    u8_t i;

    /* read all 3 axis from FIFO */
    for(i = 0; i < 3; i++)
      ISM330DLC_ACC_GYRO_Get_GetFIFOData(handle, raw_data_tmp.u8bit + 2*i);
  } else
    ISM330DLC_ACC_GYRO_GetRawGyroData(handle, raw_data_tmp.u8bit);

  /* Apply proper shift and sensitivity */
  buff[0] = (raw_data_tmp.i16bit[0] * sensitivity + 500)/1000;
  buff[1] = (raw_data_tmp.i16bit[1] * sensitivity + 500)/1000;
  buff[2] = (raw_data_tmp.i16bit[2] * sensitivity + 500)/1000;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_ODR_G
* Description    : Write ODR_G
* Input          : ISM330DLC_ACC_GYRO_ODR_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_ODR_G(void *handle, ISM330DLC_ACC_GYRO_ODR_G_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL2_G, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_ODR_G_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL2_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_ODR_G
* Description    : Read ODR_G
* Input          : Pointer to ISM330DLC_ACC_GYRO_ODR_G_t
* Output         : Status of ODR_G see ISM330DLC_ACC_GYRO_ODR_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_ODR_G(void *handle, ISM330DLC_ACC_GYRO_ODR_G_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL2_G, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_ODR_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_translate_ODR_G
* Description    : Read ODR_G
* Input          : ISM330DLC_ACC_GYRO_ODR_G_t
* Output         : The ODR value in Hz
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_translate_ODR_G(ISM330DLC_ACC_GYRO_ODR_G_t value, u16_t *odr_hz_val)
{
  switch(value) {
  case ISM330DLC_ACC_GYRO_ODR_G_POWER_DOWN:
    *odr_hz_val = 0;
    break;

  case ISM330DLC_ACC_GYRO_ODR_G_12Hz5:
    *odr_hz_val = 13;
    break;

  case ISM330DLC_ACC_GYRO_ODR_G_26Hz:
    *odr_hz_val = 26;
    break;

  case ISM330DLC_ACC_GYRO_ODR_G_52Hz:
    *odr_hz_val = 52;
    break;

  case ISM330DLC_ACC_GYRO_ODR_G_104Hz:
    *odr_hz_val = 104;
    break;

  case ISM330DLC_ACC_GYRO_ODR_G_208Hz:
    *odr_hz_val = 208;
    break;

  case ISM330DLC_ACC_GYRO_ODR_G_416Hz:
    *odr_hz_val = 416;
    break;

  case ISM330DLC_ACC_GYRO_ODR_G_833Hz:
    *odr_hz_val = 833;
    break;

  case ISM330DLC_ACC_GYRO_ODR_G_1660Hz:
    *odr_hz_val = 1660;
    break;

  default:
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FS_125
* Description    : Write FS_125
* Input          : ISM330DLC_ACC_GYRO_FS_125_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FS_125(void *handle, ISM330DLC_ACC_GYRO_FS_125_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL2_G, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_FS_125_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL2_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FS_125
* Description    : Read FS_125
* Input          : Pointer to ISM330DLC_ACC_GYRO_FS_125_t
* Output         : Status of FS_125 see ISM330DLC_ACC_GYRO_FS_125_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FS_125(void *handle, ISM330DLC_ACC_GYRO_FS_125_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL2_G, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FS_125_MASK; //mask

  return MEMS_SUCCESS;
}

/**************** Advanced Function  *******************/

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_BW0_XL
* Description    : Write BW0_XL
* Input          : ISM330DLC_ACC_GYRO_BW0_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_BW0_XL(void *handle, ISM330DLC_ACC_GYRO_BW0_XL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_BW0_XL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_BW0_XL
* Description    : Read BW0_XL
* Input          : Pointer to ISM330DLC_ACC_GYRO_BW0_XL_t
* Output         : Status of BW0_XL see ISM330DLC_ACC_GYRO_BW0_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_BW0_XL(void *handle, ISM330DLC_ACC_GYRO_BW0_XL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_BW0_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_BW_SEL
* Description    : Write BW_SEL
* Input          : ISM330DLC_ACC_GYRO_BW_SEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_BW_SEL(void *handle, ISM330DLC_ACC_GYRO_BW_SEL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_BW_SEL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_BW_SEL
* Description    : Read BW_SEL
* Input          : Pointer to ISM330DLC_ACC_GYRO_BW_SEL_t
* Output         : Status of BW_SEL see ISM330DLC_ACC_GYRO_BW_SEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_BW_SEL(void *handle, ISM330DLC_ACC_GYRO_BW_SEL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL1_XL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_BW_SEL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_BLE
* Description    : Write BLE
* Input          : ISM330DLC_ACC_GYRO_BLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_BLE(void *handle, ISM330DLC_ACC_GYRO_BLE_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_BLE_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_BLE
* Description    : Read BLE
* Input          : Pointer to ISM330DLC_ACC_GYRO_BLE_t
* Output         : Status of BLE see ISM330DLC_ACC_GYRO_BLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_BLE(void *handle, ISM330DLC_ACC_GYRO_BLE_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_BLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_EmbeddedAccess
* Description    : Write EMB_ACC
* Input          : ISM330DLC_ACC_GYRO_EMB_ACC_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_EmbeddedAccess(void *handle, ISM330DLC_ACC_GYRO_EMB_ACC_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FUNC_CFG_ACCESS, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_EMB_ACC_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FUNC_CFG_ACCESS, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_EmbeddedAccess
* Description    : Read EMB_ACC
* Input          : Pointer to ISM330DLC_ACC_GYRO_EMB_ACC_t
* Output         : Status of EMB_ACC see ISM330DLC_ACC_GYRO_EMB_ACC_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_EmbeddedAccess(void *handle, ISM330DLC_ACC_GYRO_EMB_ACC_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FUNC_CFG_ACCESS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_EMB_ACC_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SYNC_RES_RATIO
* Description    : Write RR
* Input          : ISM330DLC_ACC_GYRO_SYNC_RES_RATIO_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SYNC_RES_RATIO(void *handle, ISM330DLC_ACC_GYRO_SYNC_RES_RATIO_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_SENSOR_RES_RATIO, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_SYNC_RES_RATIO_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_SENSOR_RES_RATIO, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SYNC_RES_RATIO
* Description    : Read RR
* Input          : Pointer to ISM330DLC_ACC_GYRO_SYNC_RES_RATIO_t
* Output         : Status of RR see ISM330DLC_ACC_GYRO_SYNC_RES_RATIO_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SYNC_RES_RATIO(void *handle, ISM330DLC_ACC_GYRO_SYNC_RES_RATIO_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_SENSOR_RES_RATIO, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SYNC_RES_RATIO_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_Stamping_Time_Frame
* Description    : Write TPH
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_Stamping_Time_Frame(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << ISM330DLC_ACC_GYRO_TPH_POSITION; //mask
  newValue &= ISM330DLC_ACC_GYRO_TPH_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_SENSOR_SYNC_TIME, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~ISM330DLC_ACC_GYRO_TPH_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_SENSOR_SYNC_TIME, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_Stamping_Time_Frame
* Description    : Read TPH
* Input          : Pointer to u8_t
* Output         : Status of TPH
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_Stamping_Time_Frame(void *handle, u8_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_SENSOR_SYNC_TIME, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TPH_MASK; //coerce
  *value = *value >> ISM330DLC_ACC_GYRO_TPH_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FIFO_Watermark
* Description    : Write WTM_FIFO
* Input          : u16_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_Watermark(void *handle, u16_t newValue)
{
  u8_t valueH, valueL;
  u8_t value;

  valueL = newValue & 0xFF;
  valueH = (newValue >> 8) & 0xFF;

  /* Low part goes in FIFO_CTRL1 */
  valueL = valueL << ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL1_POSITION; //mask
  valueL &= ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL1_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL1, &value, 1) )
    return MEMS_ERROR;

  value &= (u8_t)~ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL1_MASK;
  value |= valueL;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL1, &value, 1) )
    return MEMS_ERROR;

  /* High part goes in FIFO_CTRL2 */
  valueH = valueH << ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL2_POSITION; //mask
  valueH &= ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL2_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL2_MASK;
  value |= valueH;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FIFO_Watermark
* Description    : Read WTM_FIFO
* Input          : Pointer to u16_t
* Output         : Status of WTM_FIFO
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_Watermark(void *handle, u16_t *value)
{
  u8_t valueH, valueL;

  /* Low part from FIFO_CTRL1 */
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL1, (u8_t *)&valueL, 1) )
    return MEMS_ERROR;

  valueL &= ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL1_MASK; //coerce
  valueL = valueL >> ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL1_POSITION; //mask

  /* High part from FIFO_CTRL2 */
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL2, (u8_t *)&valueH, 1) )
    return MEMS_ERROR;

  valueH &= ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL2_MASK; //coerce
  valueH = valueH >> ISM330DLC_ACC_GYRO_WTM_FIFO_CTRL2_POSITION; //mask

  *value = ((valueH << 8) & 0xFF00) | valueL;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FIFO_TEMP
* Description    : Write FIFO_TEMP_EN
* Input          : ISM330DLC_ACC_GYRO_FIFO_TEMP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_TEMP(void *handle, ISM330DLC_ACC_GYRO_FIFO_TEMP_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_FIFO_TEMP_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FIFO_TEMP
* Description    : Read FIFO_TEMP_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_FIFO_TEMP_t
* Output         : Status of FIFO_TEMP_EN see ISM330DLC_ACC_GYRO_FIFO_TEMP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_TEMP(void *handle, ISM330DLC_ACC_GYRO_FIFO_TEMP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FIFO_TEMP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FIFO_TIMER_En
* Description    : Write FIFO_TIMER_EN
* Input          : ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_TIMER_En(void *handle, ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FIFO_TIMER_En
* Description    : Read FIFO_TIMER_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_t
* Output         : Status of FIFO_TIMER_EN see ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_TIMER_En(void *handle, ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FIFO_TIMER_EN_MASK; //mask

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL
* Description    : Write DEC_FIFO_XL
* Input          : ISM330DLC_ACC_GYRO_DEC_FIFO_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL3, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DEC_FIFO_XL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL_val
* Description    : Write DEC_FIFO_XL
* Input          : u16_t
* Output         : Program XL decimation value from unsigned short
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL_val(void *handle, u16_t newValue)
{
  switch(newValue) {
  case 0:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DATA_NOT_IN_FIFO);
    break;

  case 1:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_NO_DECIMATION);
    break;

  case 2:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_2);
    break;

  case 3:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_3);
    break;

  case 4:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_4);
    break;

  case 8:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_8);
    break;

  case 16:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_16);
    break;

  case 32:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_XL(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_DECIMATION_BY_32);
    break;

  default:
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DEC_FIFO_XL
* Description    : Read DEC_FIFO_XL
* Input          : Pointer to ISM330DLC_ACC_GYRO_DEC_FIFO_XL_t
* Output         : Status of DEC_FIFO_XL see ISM330DLC_ACC_GYRO_DEC_FIFO_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DEC_FIFO_XL(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_XL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL3, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DEC_FIFO_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DEC_FIFO_G
* Description    : Write DEC_FIFO_G
* Input          : ISM330DLC_ACC_GYRO_DEC_FIFO_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_G(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL3, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DEC_FIFO_G_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL3, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DEC_FIFO_G_val
* Description    : Write DEC_FIFO_G
* Input          : u16_t
* Output         : Program G decimation value from unsigned short
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_G_val(void *handle, u16_t newValue)
{
  switch(newValue) {
  case 0:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_G(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_DATA_NOT_IN_FIFO);
    break;

  case 1:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_G(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_NO_DECIMATION);
    break;

  case 2:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_G(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_2);
    break;

  case 3:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_G(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_3);
    break;

  case 4:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_G(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_4);
    break;

  case 8:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_G(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_8);
    break;

  case 16:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_G(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_16);
    break;

  case 32:
    ISM330DLC_ACC_GYRO_W_DEC_FIFO_G(handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_DECIMATION_BY_32);
    break;

  default:
    return MEMS_ERROR;
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DEC_FIFO_G
* Description    : Read DEC_FIFO_G
* Input          : Pointer to ISM330DLC_ACC_GYRO_DEC_FIFO_G_t
* Output         : Status of DEC_FIFO_G see ISM330DLC_ACC_GYRO_DEC_FIFO_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DEC_FIFO_G(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_G_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL3, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DEC_FIFO_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DEC_FIFO_DS3
* Description    : Write DEC_DS3_FIFO
* Input          : ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_DS3(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DEC_FIFO_DS3
* Description    : Read DEC_DS3_FIFO
* Input          : Pointer to ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_t
* Output         : Status of DEC_DS3_FIFO see ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DEC_FIFO_DS3(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DEC_FIFO_DS3_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DEC_FIFO_DS4
* Description    : Write DEC_DS4_FIFO
* Input          : ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DEC_FIFO_DS4(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DEC_FIFO_DS4
* Description    : Read DEC_DS4_FIFO
* Input          : Pointer to ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_t
* Output         : Status of DEC_DS4_FIFO see ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DEC_FIFO_DS4(void *handle, ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DEC_FIFO_DS4_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_HI_DATA_ONLY
* Description    : Write HI_DATA_ONLY
* Input          : ISM330DLC_ACC_GYRO_HI_DATA_ONLY_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_HI_DATA_ONLY(void *handle, ISM330DLC_ACC_GYRO_HI_DATA_ONLY_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_HI_DATA_ONLY_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_HI_DATA_ONLY
* Description    : Read HI_DATA_ONLY
* Input          : Pointer to ISM330DLC_ACC_GYRO_HI_DATA_ONLY_t
* Output         : Status of HI_DATA_ONLY see ISM330DLC_ACC_GYRO_HI_DATA_ONLY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_HI_DATA_ONLY(void *handle, ISM330DLC_ACC_GYRO_HI_DATA_ONLY_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_HI_DATA_ONLY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_STOP_ON_FTH
* Description    : Write STOP_ON_FTH
* Input          : ISM330DLC_ACC_GYRO_STOP_ON_FTH_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_STOP_ON_FTH(void *handle, ISM330DLC_ACC_GYRO_STOP_ON_FTH_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_STOP_ON_FTH_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_STOP_ON_FTH
* Description    : Read STOP_ON_FTH
* Input          : Pointer to ISM330DLC_ACC_GYRO_STOP_ON_FTH_t
* Output         : Status of STOP_ON_FTH see ISM330DLC_ACC_GYRO_STOP_ON_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_STOP_ON_FTH(void *handle, ISM330DLC_ACC_GYRO_STOP_ON_FTH_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL4, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_STOP_ON_FTH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FIFO_MODE
* Description    : Write FIFO_MODE
* Input          : ISM330DLC_ACC_GYRO_FIFO_MODE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_MODE(void *handle, ISM330DLC_ACC_GYRO_FIFO_MODE_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL5, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_FIFO_MODE_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FIFO_MODE
* Description    : Read FIFO_MODE
* Input          : Pointer to ISM330DLC_ACC_GYRO_FIFO_MODE_t
* Output         : Status of FIFO_MODE see ISM330DLC_ACC_GYRO_FIFO_MODE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_MODE(void *handle, ISM330DLC_ACC_GYRO_FIFO_MODE_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL5, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FIFO_MODE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_ODR_FIFO
* Description    : Write ODR_FIFO
* Input          : ISM330DLC_ACC_GYRO_ODR_FIFO_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_ODR_FIFO(void *handle, ISM330DLC_ACC_GYRO_ODR_FIFO_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL5, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_ODR_FIFO_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL5, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_ODR_FIFO
* Description    : Read ODR_FIFO
* Input          : Pointer to ISM330DLC_ACC_GYRO_ODR_FIFO_t
* Output         : Status of ODR_FIFO see ISM330DLC_ACC_GYRO_ODR_FIFO_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_ODR_FIFO(void *handle, ISM330DLC_ACC_GYRO_ODR_FIFO_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_CTRL5, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_ODR_FIFO_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DRDY_PULSE
* Description    : Write DRDY_PULSE
* Input          : ISM330DLC_ACC_GYRO_DRDY_PULSE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_PULSE(void *handle, ISM330DLC_ACC_GYRO_DRDY_PULSE_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_DRDY_PULSE_CFG_G, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DRDY_PULSE_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_DRDY_PULSE_CFG_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DRDY_PULSE
* Description    : Read DRDY_PULSE
* Input          : Pointer to ISM330DLC_ACC_GYRO_DRDY_PULSE_t
* Output         : Status of DRDY_PULSE see ISM330DLC_ACC_GYRO_DRDY_PULSE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_PULSE(void *handle, ISM330DLC_ACC_GYRO_DRDY_PULSE_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_DRDY_PULSE_CFG_G, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DRDY_PULSE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DRDY_XL_on_INT1
* Description    : Write INT1_DRDY_XL
* Input          : ISM330DLC_ACC_GYRO_INT1_DRDY_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_XL_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_DRDY_XL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_DRDY_XL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DRDY_XL_on_INT1
* Description    : Read INT1_DRDY_XL
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_DRDY_XL_t
* Output         : Status of INT1_DRDY_XL see ISM330DLC_ACC_GYRO_INT1_DRDY_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_XL_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_DRDY_XL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_DRDY_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DRDY_G_on_INT1
* Description    : Write INT1_DRDY_G
* Input          : ISM330DLC_ACC_GYRO_INT1_DRDY_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_G_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_DRDY_G_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_DRDY_G_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DRDY_G_on_INT1
* Description    : Read INT1_DRDY_G
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_DRDY_G_t
* Output         : Status of INT1_DRDY_G see ISM330DLC_ACC_GYRO_INT1_DRDY_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_G_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_DRDY_G_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_DRDY_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_BOOT_on_INT1
* Description    : Write INT1_BOOT
* Input          : ISM330DLC_ACC_GYRO_INT1_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_BOOT_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_BOOT_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_BOOT_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_BOOT_on_INT1
* Description    : Read INT1_BOOT
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_BOOT_t
* Output         : Status of INT1_BOOT see ISM330DLC_ACC_GYRO_INT1_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_BOOT_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_BOOT_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FIFO_TSHLD_on_INT1
* Description    : Write INT1_FTH
* Input          : ISM330DLC_ACC_GYRO_INT1_FTH_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_TSHLD_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_FTH_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_FTH_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FIFO_TSHLD_on_INT1
* Description    : Read INT1_FTH
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_FTH_t
* Output         : Status of INT1_FTH see ISM330DLC_ACC_GYRO_INT1_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_TSHLD_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_FTH_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_FTH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_OVERRUN_on_INT1
* Description    : Write INT1_OVR
* Input          : ISM330DLC_ACC_GYRO_INT1_OVR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_OVERRUN_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_OVR_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_OVR_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1))
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_OVERRUN_on_INT1
* Description    : Read INT1_OVR
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_OVR_t
* Output         : Status of INT1_OVR see ISM330DLC_ACC_GYRO_INT1_OVR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_OVERRUN_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_OVR_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_OVR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FULL_FLAG_on_INT1
* Description    : Write INT1_FULL_FLAG
* Input          : ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FULL_FLAG_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FULL_FLAG_on_INT1
* Description    : Read INT1_FULL_FLAG
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_t
* Output         : Status of INT1_FULL_FLAG see ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FULL_FLAG_on_INT1(void *handle, ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT1_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_FULL_FLAG_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DRDY_XL_on_INT2
* Description    : Write INT2_DRDY_XL
* Input          : ISM330DLC_ACC_GYRO_INT2_DRDY_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_XL_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_XL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_DRDY_XL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DRDY_XL_on_INT2
* Description    : Read INT2_DRDY_XL
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_DRDY_XL_t
* Output         : Status of INT2_DRDY_XL see ISM330DLC_ACC_GYRO_INT2_DRDY_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_XL_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_XL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_DRDY_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DRDY_G_on_INT2
* Description    : Write INT2_DRDY_G
* Input          : ISM330DLC_ACC_GYRO_INT2_DRDY_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_G_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_G_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_DRDY_G_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DRDY_G_on_INT2
* Description    : Read INT2_DRDY_G
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_DRDY_G_t
* Output         : Status of INT2_DRDY_G see ISM330DLC_ACC_GYRO_INT2_DRDY_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_G_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_G_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_DRDY_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DRDY_TEMP_on_INT2
* Description    : Write INT2_DRDY_TEMP
* Input          : ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_TEMP_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DRDY_TEMP_on_INT2
* Description    : Read INT2_DRDY_TEMP
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_t
* Output         : Status of INT2_DRDY_TEMP see ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_TEMP_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_DRDY_TEMP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FIFO_TSHLD_on_INT2
* Description    : Write INT2_FTH
* Input          : ISM330DLC_ACC_GYRO_INT2_FTH_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FIFO_TSHLD_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_FTH_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_FTH_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FIFO_TSHLD_on_INT2
* Description    : Read INT2_FTH
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_FTH_t
* Output         : Status of INT2_FTH see ISM330DLC_ACC_GYRO_INT2_FTH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FIFO_TSHLD_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_FTH_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_FTH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_OVERRUN_on_INT2
* Description    : Write INT2_OVR
* Input          : ISM330DLC_ACC_GYRO_INT2_OVR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_OVERRUN_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_OVR_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_OVR_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_OVERRUN_on_INT2
* Description    : Read INT2_OVR
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_OVR_t
* Output         : Status of INT2_OVR see ISM330DLC_ACC_GYRO_INT2_OVR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_OVERRUN_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_OVR_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_OVR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FULL_FLAG_on_INT2
* Description    : Write INT2_FULL_FLAG
* Input          : ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FULL_FLAG_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FULL_FLAG_on_INT2
* Description    : Read INT2_FULL_FLAG
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_t
* Output         : Status of INT2_FULL_FLAG see ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FULL_FLAG_on_INT2(void *handle, ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT2_CTRL, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_FULL_FLAG_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SW_RESET
* Description    : Write SW_RESET
* Input          : ISM330DLC_ACC_GYRO_SW_RESET_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SW_RESET(void *handle, ISM330DLC_ACC_GYRO_SW_RESET_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_SW_RESET_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SW_RESET
* Description    : Read SW_RESET
* Input          : Pointer to ISM330DLC_ACC_GYRO_SW_RESET_t
* Output         : Status of SW_RESET see ISM330DLC_ACC_GYRO_SW_RESET_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SW_RESET(void *handle, ISM330DLC_ACC_GYRO_SW_RESET_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SW_RESET_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_IF_Addr_Incr
* Description    : Write IF_INC
* Input          : ISM330DLC_ACC_GYRO_IF_INC_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_IF_Addr_Incr(void *handle, ISM330DLC_ACC_GYRO_IF_INC_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_IF_INC_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_IF_Addr_Incr
* Description    : Read IF_INC
* Input          : Pointer to ISM330DLC_ACC_GYRO_IF_INC_t
* Output         : Status of IF_INC see ISM330DLC_ACC_GYRO_IF_INC_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_IF_Addr_Incr(void *handle, ISM330DLC_ACC_GYRO_IF_INC_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_IF_INC_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SPI_Mode
* Description    : Write SIM
* Input          : ISM330DLC_ACC_GYRO_SIM_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SPI_Mode(void *handle, ISM330DLC_ACC_GYRO_SIM_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_SIM_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SPI_Mode
* Description    : Read SIM
* Input          : Pointer to ISM330DLC_ACC_GYRO_SIM_t
* Output         : Status of SIM see ISM330DLC_ACC_GYRO_SIM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SPI_Mode(void *handle, ISM330DLC_ACC_GYRO_SIM_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SIM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_PadSel
* Description    : Write PP_OD
* Input          : ISM330DLC_ACC_GYRO_PP_OD_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_PadSel(void *handle, ISM330DLC_ACC_GYRO_PP_OD_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_PP_OD_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_PadSel
* Description    : Read PP_OD
* Input          : Pointer to ISM330DLC_ACC_GYRO_PP_OD_t
* Output         : Status of PP_OD see ISM330DLC_ACC_GYRO_PP_OD_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_PadSel(void *handle, ISM330DLC_ACC_GYRO_PP_OD_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_PP_OD_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_INT_ACT_LEVEL
* Description    : Write INT_ACT_LEVEL
* Input          : ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_INT_ACT_LEVEL(void *handle, ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_INT_ACT_LEVEL
* Description    : Read INT_ACT_LEVEL
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_t
* Output         : Status of INT_ACT_LEVEL see ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_INT_ACT_LEVEL(void *handle, ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT_ACT_LEVEL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_BOOT
* Description    : Write BOOT
* Input          : ISM330DLC_ACC_GYRO_BOOT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_BOOT(void *handle, ISM330DLC_ACC_GYRO_BOOT_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_BOOT_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_BOOT
* Description    : Read BOOT
* Input          : Pointer to ISM330DLC_ACC_GYRO_BOOT_t
* Output         : Status of BOOT see ISM330DLC_ACC_GYRO_BOOT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_BOOT(void *handle, ISM330DLC_ACC_GYRO_BOOT_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL3_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_BOOT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_LPF1_SEL_G
* Description    : Write LPF1_SEL_G
* Input          : ISM330DLC_ACC_GYRO_LPF1_SEL_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_LPF1_SEL_G(void *handle, ISM330DLC_ACC_GYRO_LPF1_SEL_G_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_LPF1_SEL_G_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_LPF1_SEL_G
* Description    : Read LPF1_SEL_G
* Input          : Pointer to ISM330DLC_ACC_GYRO_LPF1_SEL_G_t
* Output         : Status of LPF1_SEL_G see ISM330DLC_ACC_GYRO_LPF1_SEL_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_LPF1_SEL_G(void *handle, ISM330DLC_ACC_GYRO_LPF1_SEL_G_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_LPF1_SEL_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_I2C_DISABLE
* Description    : Write I2C_DISABLE
* Input          : ISM330DLC_ACC_GYRO_I2C_DISABLE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_I2C_DISABLE(void *handle, ISM330DLC_ACC_GYRO_I2C_DISABLE_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_I2C_DISABLE_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_I2C_DISABLE
* Description    : Read I2C_DISABLE
* Input          : Pointer to ISM330DLC_ACC_GYRO_I2C_DISABLE_t
* Output         : Status of I2C_DISABLE see ISM330DLC_ACC_GYRO_I2C_DISABLE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_I2C_DISABLE(void *handle, ISM330DLC_ACC_GYRO_I2C_DISABLE_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_I2C_DISABLE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DRDY_MSK
* Description    : Write DRDY_MSK
* Input          : ISM330DLC_ACC_GYRO_DRDY_MSK_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_MSK(void *handle, ISM330DLC_ACC_GYRO_DRDY_MSK_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DRDY_MSK_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DRDY_MSK
* Description    : Read DRDY_MSK
* Input          : Pointer to ISM330DLC_ACC_GYRO_DRDY_MSK_t
* Output         : Status of DRDY_MSK see ISM330DLC_ACC_GYRO_DRDY_MSK_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_MSK(void *handle, ISM330DLC_ACC_GYRO_DRDY_MSK_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DRDY_MSK_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DEN_DRDY_INT1
* Description    : Write DEN_DRDY_INT1
* Input          : ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DEN_DRDY_INT1(void *handle, ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DEN_DRDY_INT1
* Description    : Read DEN_DRDY_INT1
* Input          : Pointer to ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_t
* Output         : Status of DEN_DRDY_INT1 see ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DEN_DRDY_INT1(void *handle, ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DEN_DRDY_INT1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_INT2_ON_INT1
* Description    : Write INT2_ON_INT1
* Input          : ISM330DLC_ACC_GYRO_INT2_ON_INT1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_INT2_ON_INT1(void *handle, ISM330DLC_ACC_GYRO_INT2_ON_INT1_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_ON_INT1_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_INT2_ON_INT1
* Description    : Read INT2_ON_INT1
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_ON_INT1_t
* Output         : Status of INT2_ON_INT1 see ISM330DLC_ACC_GYRO_INT2_ON_INT1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_INT2_ON_INT1(void *handle, ISM330DLC_ACC_GYRO_INT2_ON_INT1_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_ON_INT1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SleepMode_G
* Description    : Write SLEEP_G
* Input          : ISM330DLC_ACC_GYRO_SLEEP_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SleepMode_G(void *handle, ISM330DLC_ACC_GYRO_SLEEP_G_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_SLEEP_G_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SleepMode_G
* Description    : Read SLEEP_G
* Input          : Pointer to ISM330DLC_ACC_GYRO_SLEEP_G_t
* Output         : Status of SLEEP_G see ISM330DLC_ACC_GYRO_SLEEP_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SleepMode_G(void *handle, ISM330DLC_ACC_GYRO_SLEEP_G_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SLEEP_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DEN_XL_EN
* Description    : Write DEN_XL_EN
* Input          : ISM330DLC_ACC_GYRO_DEN_XL_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DEN_XL_EN(void *handle, ISM330DLC_ACC_GYRO_DEN_XL_EN_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DEN_XL_EN_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DEN_XL_EN
* Description    : Read DEN_XL_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_DEN_XL_EN_t
* Output         : Status of DEN_XL_EN see ISM330DLC_ACC_GYRO_DEN_XL_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DEN_XL_EN(void *handle, ISM330DLC_ACC_GYRO_DEN_XL_EN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL4_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DEN_XL_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SelfTest_XL
* Description    : Write ST_XL
* Input          : ISM330DLC_ACC_GYRO_ST_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SelfTest_XL(void *handle, ISM330DLC_ACC_GYRO_ST_XL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_ST_XL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SelfTest_XL
* Description    : Read ST_XL
* Input          : Pointer to ISM330DLC_ACC_GYRO_ST_XL_t
* Output         : Status of ST_XL see ISM330DLC_ACC_GYRO_ST_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SelfTest_XL(void *handle, ISM330DLC_ACC_GYRO_ST_XL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_ST_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SelfTest_G
* Description    : Write ST_G
* Input          : ISM330DLC_ACC_GYRO_ST_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SelfTest_G(void *handle, ISM330DLC_ACC_GYRO_ST_G_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_ST_G_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SelfTest_G
* Description    : Read ST_G
* Input          : Pointer to ISM330DLC_ACC_GYRO_ST_G_t
* Output         : Status of ST_G see ISM330DLC_ACC_GYRO_ST_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SelfTest_G(void *handle, ISM330DLC_ACC_GYRO_ST_G_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_ST_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DEN_Polarity
* Description    : Write DEN_LH
* Input          : ISM330DLC_ACC_GYRO_DEN_LH_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DEN_Polarity(void *handle, ISM330DLC_ACC_GYRO_DEN_LH_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DEN_LH_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DEN_Polarity
* Description    : Read DEN_LH
* Input          : Pointer to ISM330DLC_ACC_GYRO_DEN_LH_t
* Output         : Status of DEN_LH see ISM330DLC_ACC_GYRO_DEN_LH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DEN_Polarity(void *handle, ISM330DLC_ACC_GYRO_DEN_LH_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DEN_LH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_CircularBurstMode
* Description    : Write ST_ROUNDING
* Input          : ISM330DLC_ACC_GYRO_ROUNDING_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_CircularBurstMode(void *handle, ISM330DLC_ACC_GYRO_ROUNDING_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_ISM330DLC_ACC_GYRO_ROUNDING_t_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_CircularBurstMode
* Description    : Read ST_ROUNDING
* Input          : Pointer to ISM330DLC_ACC_GYRO_ROUNDING_t
* Output         : Status of ST_ROUNDING see ISM330DLC_ACC_GYRO_ROUNDING_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_CircularBurstMode(void *handle, ISM330DLC_ACC_GYRO_ROUNDING_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL5_C, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_ISM330DLC_ACC_GYRO_ROUNDING_t_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_LP_BW_G
* Description    : Write FTYPE
* Input          : ISM330DLC_ACC_GYRO_FTYPE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_LP_BW_G(void *handle, ISM330DLC_ACC_GYRO_FTYPE_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_FTYPE_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_LP_BW_G
* Description    : Read FTYPE
* Input          : Pointer to ISM330DLC_ACC_GYRO_FTYPE_t
* Output         : Status of FTYPE see ISM330DLC_ACC_GYRO_FTYPE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_LP_BW_G(void *handle, ISM330DLC_ACC_GYRO_FTYPE_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FTYPE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_UserOffsetWeight
* Description    : Write USR_OFF_W
* Input          : ISM330DLC_ACC_GYRO_USR_OFF_W_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_UserOffsetWeight(void *handle, ISM330DLC_ACC_GYRO_USR_OFF_W_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_USR_OFF_W_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_UserOffsetWeight
* Description    : Read USR_OFF_W
* Input          : Pointer to ISM330DLC_ACC_GYRO_USR_OFF_W_t
* Output         : Status of USR_OFF_W see ISM330DLC_ACC_GYRO_USR_OFF_W_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_UserOffsetWeight(void *handle, ISM330DLC_ACC_GYRO_USR_OFF_W_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_USR_OFF_W_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_HighPerform
* Description    : Write XL_HM_MODE
* Input          : ISM330DLC_ACC_GYRO_XL_HM_MODE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_HighPerform(void *handle, ISM330DLC_ACC_GYRO_XL_HM_MODE_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_XL_HM_MODE_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_HighPerform
* Description    : Read XL_HM_MODE
* Input          : Pointer to ISM330DLC_ACC_GYRO_XL_HM_MODE_t
* Output         : Status of XL_HM_MODE see ISM330DLC_ACC_GYRO_XL_HM_MODE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_HighPerform(void *handle, ISM330DLC_ACC_GYRO_XL_HM_MODE_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_XL_HM_MODE_MASK; //mask

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TrigerMode
* Description    : Write TRIGER_MODE
* Input          : ISM330DLC_ACC_GYRO_TRIGER_MODE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TrigerMode(void *handle, ISM330DLC_ACC_GYRO_TRIGER_MODE_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, &value, 1))
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_TRIGER_MODE_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TrigerMode
* Description    : Read TRIGER_MODE
* Input          : Pointer to ISM330DLC_ACC_GYRO_TRIGER_MODE_t
* Output         : Status of TRIGER_MODE see ISM330DLC_ACC_GYRO_TRIGER_MODE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TrigerMode(void *handle, ISM330DLC_ACC_GYRO_TRIGER_MODE_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL6_G, (u8_t *)value, 1))
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TRIGER_MODE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_HPM_G
* Description    : Write HPM_G
* Input          : ISM330DLC_ACC_GYRO_HPM_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_HPM_G(void *handle, ISM330DLC_ACC_GYRO_HPM_G_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_HPM_G_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_HPM_G
* Description    : Read HPM_G
* Input          : Pointer to ISM330DLC_ACC_GYRO_HPM_G_t
* Output         : Status of HPM_G see ISM330DLC_ACC_GYRO_HPM_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_HPM_G(void *handle, ISM330DLC_ACC_GYRO_HPM_G_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_HPM_G_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_ROUNDING_STATUS
* Description    : Write ROUNDING_STATUS
* Input          : ISM330DLC_ACC_GYRO_ROUNDING_STATUS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_ROUNDING_STATUS(void *handle, ISM330DLC_ACC_GYRO_ROUNDING_STATUS_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_ROUNDING_STATUS_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_ROUNDING_STATUS
* Description    : Read ROUNDING_STATUS
* Input          : Pointer to ISM330DLC_ACC_GYRO_ROUNDING_STATUS_t
* Output         : Status of ROUNDING_STATUS see ISM330DLC_ACC_GYRO_ROUNDING_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_ROUNDING_STATUS(void *handle, ISM330DLC_ACC_GYRO_ROUNDING_STATUS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_ROUNDING_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_HPFilter_En
* Description    : Write HP_EN
* Input          : ISM330DLC_ACC_GYRO_HP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_HPFilter_En(void *handle, ISM330DLC_ACC_GYRO_HP_EN_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_HP_EN_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_HPFilter_En
* Description    : Read HP_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_HP_EN_t
* Output         : Status of HP_EN see ISM330DLC_ACC_GYRO_HP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_HPFilter_En(void *handle, ISM330DLC_ACC_GYRO_HP_EN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_HP_EN_MASK; //mask

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_HM_Mode
* Description    : Write G_HM_MODE
* Input          : ISM330DLC_ACC_GYRO_G_HM_MODE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_HM_Mode(void *handle, ISM330DLC_ACC_GYRO_G_HM_MODE_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_G_HM_MODE_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_HM_Mode
* Description    : Read G_HM_MODE
* Input          : Pointer to ISM330DLC_ACC_GYRO_G_HM_MODE_t
* Output         : Status of G_HM_MODE see ISM330DLC_ACC_GYRO_G_HM_MODE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_HM_Mode(void *handle, ISM330DLC_ACC_GYRO_G_HM_MODE_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL7_G, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_G_HM_MODE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_InComposit
* Description    : Write INPUT_COMPOSITE
* Input          : ISM330DLC_ACC_GYRO_IN_COMP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_InComposit(void *handle, ISM330DLC_ACC_GYRO_IN_COMP_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_IN_COMP_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_InComposit
* Description    : Read INPUT_COMPOSITE
* Input          : Pointer to ISM330DLC_ACC_GYRO_IN_COMP_t
* Output         : Status of INPUT_COMPOSITE see ISM330DLC_ACC_GYRO_IN_COMP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_InComposit(void *handle, ISM330DLC_ACC_GYRO_IN_COMP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_IN_COMP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_HPfilterReference
* Description    : Write HP_REF_MODE
* Input          : ISM330DLC_ACC_GYRO_HP_REF_MODE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_HPfilterReference(void *handle, ISM330DLC_ACC_GYRO_HP_REF_MODE_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_HP_REF_MODE_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_HPfilterReference
* Description    : Read HP_REF_MODE
* Input          : Pointer to ISM330DLC_ACC_GYRO_HP_REF_MODE_t
* Output         : Status of HP_REF_MODE see ISM330DLC_ACC_GYRO_HP_REF_MODE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_HPfilterReference(void *handle, ISM330DLC_ACC_GYRO_HP_REF_MODE_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_HP_REF_MODE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_HPCF_XL
* Description    : Write HPCF_XL
* Input          : ISM330DLC_ACC_GYRO_HPCF_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_HPCF_XL(void *handle, ISM330DLC_ACC_GYRO_HPCF_XL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_HPCF_XL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_HPCF_XL
* Description    : Read HPCF_XL
* Input          : Pointer to ISM330DLC_ACC_GYRO_HPCF_XL_t
* Output         : Status of HPCF_XL see ISM330DLC_ACC_GYRO_HPCF_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_HPCF_XL(void *handle, ISM330DLC_ACC_GYRO_HPCF_XL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_HPCF_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_LowPassFiltSel_XL
* Description    : Write LPF2_XL_EN
* Input          : ISM330DLC_ACC_GYRO_LPF2_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_LowPassFiltSel_XL(void *handle, ISM330DLC_ACC_GYRO_LPF2_XL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_LPF2_XL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_LowPassFiltSel_XL
* Description    : Read LPF2_XL_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_LPF2_XL_t
* Output         : Status of LPF2_XL_EN see ISM330DLC_ACC_GYRO_LPF2_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_LowPassFiltSel_XL(void *handle, ISM330DLC_ACC_GYRO_LPF2_XL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_LPF2_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_LOW_PASS_ON_6D
* Description    : Write LOW_PASS_ON_6D
* Input          : ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_LOW_PASS_ON_6D(void *handle, ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_LOW_PASS_ON_6D
* Description    : Read LOW_PASS_ON_6D
* Input          : Pointer to ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_t
* Output         : Status of LOW_PASS_ON_6D see ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_LOW_PASS_ON_6D(void *handle, ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_LOW_PASS_ON_6D_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_HP_SLOPE_XL
* Description    : Write HP_SLOPE_XL_EN
* Input          : ISM330DLC_ACC_GYRO_HP_SLOPE_XL_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_HP_SLOPE_XL(void *handle, ISM330DLC_ACC_GYRO_HP_SLOPE_XL_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_HP_SLOPE_XL_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_HP_SLOPE_XL
* Description    : Read HP_SLOPE_XL_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_HP_SLOPE_XL_t
* Output         : Status of HP_SLOPE_XL_EN see ISM330DLC_ACC_GYRO_HP_SLOPE_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_HP_SLOPE_XL(void *handle, ISM330DLC_ACC_GYRO_HP_SLOPE_XL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL8_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_HP_SLOPE_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SOFT
* Description    : Write SOFT_EN
* Input          : ISM330DLC_ACC_GYRO_SOFT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SOFT(void *handle, ISM330DLC_ACC_GYRO_SOFT_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL9_XL, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_SOFT_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL9_XL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SOFT
* Description    : Read SOFT_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_SOFT_t
* Output         : Status of SOFT_EN see ISM330DLC_ACC_GYRO_SOFT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SOFT(void *handle, ISM330DLC_ACC_GYRO_SOFT_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL9_XL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SOFT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TILT
* Description    : Write XEN_G
* Input          : ISM330DLC_ACC_GYRO_TILT_G_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TILT(void *handle, ISM330DLC_ACC_GYRO_TILT_G_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_TILT_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TILT
* Description    : Read XEN_G
* Input          : Pointer to ISM330DLC_ACC_GYRO_TILT_G_t
* Output         : Status of XEN_G see ISM330DLC_ACC_GYRO_TILT_G_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TILT(void *handle, ISM330DLC_ACC_GYRO_TILT_G_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL10_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TILT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TIMER
* Description    : Write TIMER_EN
* Input          : ISM330DLC_ACC_GYRO_TIMER_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TIMER(void *handle, ISM330DLC_ACC_GYRO_TIMER_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_TIMER_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TIMER
* Description    : Read TIMER_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_TIMER_t
* Output         : Status of TIMER_EN see ISM330DLC_ACC_GYRO_TIMER_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TIMER(void *handle, ISM330DLC_ACC_GYRO_TIMER_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL10_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TIMER_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FUNC_EN
* Description    : Write FUNC_EN
* Input          : ISM330DLC_ACC_GYRO_FUNC_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FUNC_EN(void *handle, ISM330DLC_ACC_GYRO_FUNC_EN_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_FUNC_EN_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_CTRL10_C, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FUNC_EN
* Description    : Read FUNC_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_FUNC_EN_t
* Output         : Status of FUNC_EN see ISM330DLC_ACC_GYRO_FUNC_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FUNC_EN(void *handle, ISM330DLC_ACC_GYRO_FUNC_EN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_CTRL10_C, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FUNC_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_I2C_MASTER_Enable
* Description    : Write MASTER_ON
* Input          : ISM330DLC_ACC_GYRO_MASTER_ON_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_I2C_MASTER_Enable(void *handle, ISM330DLC_ACC_GYRO_MASTER_ON_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_MASTER_ON_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_I2C_MASTER_Enable
* Description    : Read MASTER_ON
* Input          : Pointer to ISM330DLC_ACC_GYRO_MASTER_ON_t
* Output         : Status of MASTER_ON see ISM330DLC_ACC_GYRO_MASTER_ON_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_I2C_MASTER_Enable(void *handle, ISM330DLC_ACC_GYRO_MASTER_ON_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_MASTER_ON_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_IronCorrection_EN
* Description    : Write IRON_EN
* Input          : ISM330DLC_ACC_GYRO_IRON_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_IronCorrection_EN(void *handle, ISM330DLC_ACC_GYRO_IRON_EN_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_IRON_EN_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_IronCorrection_EN
* Description    : Read IRON_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_IRON_EN_t
* Output         : Status of IRON_EN see ISM330DLC_ACC_GYRO_IRON_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_IronCorrection_EN(void *handle, ISM330DLC_ACC_GYRO_IRON_EN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_IRON_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_PASS_THRU_MODE
* Description    : Write PASS_THRU_MODE
* Input          : ISM330DLC_ACC_GYRO_PASS_THRU_MODE_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_PASS_THRU_MODE(void *handle, ISM330DLC_ACC_GYRO_PASS_THRU_MODE_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_PASS_THRU_MODE_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_PASS_THRU_MODE
* Description    : Read PASS_THRU_MODE
* Input          : Pointer to ISM330DLC_ACC_GYRO_PASS_THRU_MODE_t
* Output         : Status of PASS_THRU_MODE see ISM330DLC_ACC_GYRO_PASS_THRU_MODE_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_PASS_THRU_MODE(void *handle, ISM330DLC_ACC_GYRO_PASS_THRU_MODE_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_PASS_THRU_MODE_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_PULL_UP_EN
* Description    : Write PULL_UP_EN
* Input          : ISM330DLC_ACC_GYRO_PULL_UP_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_PULL_UP_EN(void *handle, ISM330DLC_ACC_GYRO_PULL_UP_EN_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_PULL_UP_EN_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_PULL_UP_EN
* Description    : Read PULL_UP_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_PULL_UP_EN_t
* Output         : Status of PULL_UP_EN see ISM330DLC_ACC_GYRO_PULL_UP_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_PULL_UP_EN(void *handle, ISM330DLC_ACC_GYRO_PULL_UP_EN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_PULL_UP_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SensorHUB_Trigger_Sel
* Description    : Write START_CONFIG
* Input          : ISM330DLC_ACC_GYRO_START_CONFIG_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SensorHUB_Trigger_Sel(void *handle, ISM330DLC_ACC_GYRO_START_CONFIG_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_START_CONFIG_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SensorHUB_Trigger_Sel
* Description    : Read START_CONFIG
* Input          : Pointer to ISM330DLC_ACC_GYRO_START_CONFIG_t
* Output         : Status of START_CONFIG see ISM330DLC_ACC_GYRO_START_CONFIG_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SensorHUB_Trigger_Sel(void *handle, ISM330DLC_ACC_GYRO_START_CONFIG_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_START_CONFIG_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DATA_VAL_SEL_FIFO
* Description    : Write DATA_VAL_SEL_FIFO
* Input          : ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DATA_VAL_SEL_FIFO(void *handle, ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DATA_VAL_SEL_FIFO
* Description    : Read DATA_VAL_SEL_FIFO
* Input          : Pointer to ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_t
* Output         : Status of DATA_VAL_SEL_FIFO see ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DATA_VAL_SEL_FIFO(void *handle, ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DATA_VAL_SEL_FIFO_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DRDY_ON_INT1
* Description    : Write DRDY_ON_INT1
* Input          : ISM330DLC_ACC_GYRO_DRDY_ON_INT1_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DRDY_ON_INT1(void *handle, ISM330DLC_ACC_GYRO_DRDY_ON_INT1_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DRDY_ON_INT1_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DRDY_ON_INT1
* Description    : Read DRDY_ON_INT1
* Input          : Pointer to ISM330DLC_ACC_GYRO_DRDY_ON_INT1_t
* Output         : Status of DRDY_ON_INT1 see ISM330DLC_ACC_GYRO_DRDY_ON_INT1_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DRDY_ON_INT1(void *handle, ISM330DLC_ACC_GYRO_DRDY_ON_INT1_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MASTER_CONFIG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DRDY_ON_INT1_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_Z_WU
* Description    : Read Z_WU
* Input          : Pointer to ISM330DLC_ACC_GYRO_Z_WU_t
* Output         : Status of Z_WU see ISM330DLC_ACC_GYRO_Z_WU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_Z_WU(void *handle, ISM330DLC_ACC_GYRO_Z_WU_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_Z_WU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_Y_WU
* Description    : Read Y_WU
* Input          : Pointer to ISM330DLC_ACC_GYRO_Y_WU_t
* Output         : Status of Y_WU see ISM330DLC_ACC_GYRO_Y_WU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_Y_WU(void *handle, ISM330DLC_ACC_GYRO_Y_WU_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_Y_WU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_X_WU
* Description    : Read X_WU
* Input          : Pointer to ISM330DLC_ACC_GYRO_X_WU_t
* Output         : Status of X_WU see ISM330DLC_ACC_GYRO_X_WU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_X_WU(void *handle, ISM330DLC_ACC_GYRO_X_WU_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_X_WU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_WU_EV_STATUS
* Description    : Read WU_EV_STATUS
* Input          : Pointer to ISM330DLC_ACC_GYRO_WU_EV_STATUS_t
* Output         : Status of WU_EV_STATUS see ISM330DLC_ACC_GYRO_WU_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_WU_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_WU_EV_STATUS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_WU_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SLEEP_EV_STATUS
* Description    : Read SLEEP_EV_STATUS
* Input          : Pointer to ISM330DLC_ACC_GYRO_SLEEP_EV_STATUS_t
* Output         : Status of SLEEP_EV_STATUS see ISM330DLC_ACC_GYRO_SLEEP_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SLEEP_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_SLEEP_EV_STATUS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SLEEP_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FF_EV_STATUS
* Description    : Read FF_EV_STATUS
* Input          : Pointer to ISM330DLC_ACC_GYRO_FF_EV_STATUS_t
* Output         : Status of FF_EV_STATUS see ISM330DLC_ACC_GYRO_FF_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FF_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_FF_EV_STATUS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FF_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_Z_TAP
* Description    : Read Z_TAP
* Input          : Pointer to ISM330DLC_ACC_GYRO_Z_TAP_t
* Output         : Status of Z_TAP see ISM330DLC_ACC_GYRO_Z_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_Z_TAP(void *handle, ISM330DLC_ACC_GYRO_Z_TAP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_Z_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_Y_TAP
* Description    : Read Y_TAP
* Input          : Pointer to ISM330DLC_ACC_GYRO_Y_TAP_t
* Output         : Status of Y_TAP see ISM330DLC_ACC_GYRO_Y_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_Y_TAP(void *handle, ISM330DLC_ACC_GYRO_Y_TAP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_Y_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_X_TAP
* Description    : Read X_TAP
* Input          : Pointer to ISM330DLC_ACC_GYRO_X_TAP_t
* Output         : Status of X_TAP see ISM330DLC_ACC_GYRO_X_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_X_TAP(void *handle, ISM330DLC_ACC_GYRO_X_TAP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_X_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TAP_SIGN
* Description    : Read TAP_SIGN
* Input          : Pointer to ISM330DLC_ACC_GYRO_TAP_SIGN_t
* Output         : Status of TAP_SIGN see ISM330DLC_ACC_GYRO_TAP_SIGN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_SIGN(void *handle, ISM330DLC_ACC_GYRO_TAP_SIGN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TAP_SIGN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DOUBLE_TAP_EV_STATUS
* Description    : Read DOUBLE_TAP_EV_STATUS
* Input          : Pointer to ISM330DLC_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t
* Output         : Status of DOUBLE_TAP_EV_STATUS see ISM330DLC_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DOUBLE_TAP_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_DOUBLE_TAP_EV_STATUS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DOUBLE_TAP_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SINGLE_TAP_EV_STATUS
* Description    : Read SINGLE_TAP_EV_STATUS
* Input          : Pointer to ISM330DLC_ACC_GYRO_SINGLE_TAP_EV_STATUS_t
* Output         : Status of SINGLE_TAP_EV_STATUS see ISM330DLC_ACC_GYRO_SINGLE_TAP_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SINGLE_TAP_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_SINGLE_TAP_EV_STATUS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SINGLE_TAP_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TAP_EV_STATUS
* Description    : Read TAP_EV_STATUS
* Input          : Pointer to ISM330DLC_ACC_GYRO_TAP_EV_STATUS_t
* Output         : Status of TAP_EV_STATUS see ISM330DLC_ACC_GYRO_TAP_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_TAP_EV_STATUS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TAP_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DSD_XL
* Description    : Read DSD_XL
* Input          : Pointer to ISM330DLC_ACC_GYRO_DSD_XL_t
* Output         : Status of DSD_XL see ISM330DLC_ACC_GYRO_DSD_XL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_XL(void *handle, ISM330DLC_ACC_GYRO_DSD_XL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DSD_XL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DSD_XH
* Description    : Read DSD_XH
* Input          : Pointer to ISM330DLC_ACC_GYRO_DSD_XH_t
* Output         : Status of DSD_XH see ISM330DLC_ACC_GYRO_DSD_XH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_XH(void *handle, ISM330DLC_ACC_GYRO_DSD_XH_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DSD_XH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DSD_YL
* Description    : Read DSD_YL
* Input          : Pointer to ISM330DLC_ACC_GYRO_DSD_YL_t
* Output         : Status of DSD_YL see ISM330DLC_ACC_GYRO_DSD_YL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_YL(void *handle, ISM330DLC_ACC_GYRO_DSD_YL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DSD_YL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DSD_YH
* Description    : Read DSD_YH
* Input          : Pointer to ISM330DLC_ACC_GYRO_DSD_YH_t
* Output         : Status of DSD_YH see ISM330DLC_ACC_GYRO_DSD_YH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_YH(void *handle, ISM330DLC_ACC_GYRO_DSD_YH_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DSD_YH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DSD_ZL
* Description    : Read DSD_ZL
* Input          : Pointer to ISM330DLC_ACC_GYRO_DSD_ZL_t
* Output         : Status of DSD_ZL see ISM330DLC_ACC_GYRO_DSD_ZL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_ZL(void *handle, ISM330DLC_ACC_GYRO_DSD_ZL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DSD_ZL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DSD_ZH
* Description    : Read DSD_ZH
* Input          : Pointer to ISM330DLC_ACC_GYRO_DSD_ZH_t
* Output         : Status of DSD_ZH see ISM330DLC_ACC_GYRO_DSD_ZH_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DSD_ZH(void *handle, ISM330DLC_ACC_GYRO_DSD_ZH_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DSD_ZH_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_D6D_EV_STATUS
* Description    : Read D6D_EV_STATUS
* Input          : Pointer to ISM330DLC_ACC_GYRO_D6D_EV_STATUS_t
* Output         : Status of D6D_EV_STATUS see ISM330DLC_ACC_GYRO_D6D_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_D6D_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_D6D_EV_STATUS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_D6D_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_D6D_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_XLDA
* Description    : Read XLDA
* Input          : Pointer to ISM330DLC_ACC_GYRO_XLDA_t
* Output         : Status of XLDA see ISM330DLC_ACC_GYRO_XLDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_XLDA(void *handle, ISM330DLC_ACC_GYRO_XLDA_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_XLDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_GDA
* Description    : Read GDA
* Input          : Pointer to ISM330DLC_ACC_GYRO_GDA_t
* Output         : Status of GDA see ISM330DLC_ACC_GYRO_GDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_GDA(void *handle, ISM330DLC_ACC_GYRO_GDA_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_GDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TDA
* Description    : Read GDA
* Input          : Pointer to ISM330DLC_ACC_GYRO_TDA_t
* Output         : Status of GDA see ISM330DLC_ACC_GYRO_TDA_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TDA(void *handle, ISM330DLC_ACC_GYRO_TDA_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_STATUS_REG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TDA_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FIFONumOfEntries
* Description    : Read DIFF_FIFO
* Input          : Pointer to u16_t
* Output         : Status of DIFF_FIFO
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FIFONumOfEntries(void *handle, u16_t *value)
{
  u8_t valueH, valueL;

  /* Low part from FIFO_STATUS1 */
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_STATUS1, (u8_t *)&valueL, 1) )
    return MEMS_ERROR;

  valueL &= ISM330DLC_ACC_GYRO_DIFF_FIFO_STATUS1_MASK; //coerce
  valueL = valueL >> ISM330DLC_ACC_GYRO_DIFF_FIFO_STATUS1_POSITION; //mask

  /* High part from FIFO_STATUS2 */
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_STATUS2, (u8_t *)&valueH, 1) )
    return MEMS_ERROR;

  valueH &= ISM330DLC_ACC_GYRO_DIFF_FIFO_STATUS2_MASK; //coerce
  valueH = valueH >> ISM330DLC_ACC_GYRO_DIFF_FIFO_STATUS2_POSITION; //mask

  *value = ((valueH << 8) & 0xFF00) | valueL;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FIFOEmpty
* Description    : Read FIFO_EMPTY
* Input          : Pointer to ISM330DLC_ACC_GYRO_FIFO_EMPTY_t
* Output         : Status of FIFO_EMPTY see ISM330DLC_ACC_GYRO_FIFO_EMPTY_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FIFOEmpty(void *handle, ISM330DLC_ACC_GYRO_FIFO_EMPTY_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_STATUS2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FIFO_EMPTY_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FIFOFull
* Description    : Read FIFO_FULL
* Input          : Pointer to ISM330DLC_ACC_GYRO_FIFO_FULL_t
* Output         : Status of FIFO_FULL see ISM330DLC_ACC_GYRO_FIFO_FULL_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FIFOFull(void *handle, ISM330DLC_ACC_GYRO_FIFO_FULL_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_STATUS2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FIFO_FULL_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_OVERRUN
* Description    : Read OVERRUN
* Input          : Pointer to ISM330DLC_ACC_GYRO_OVERRUN_t
* Output         : Status of OVERRUN see ISM330DLC_ACC_GYRO_OVERRUN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_OVERRUN(void *handle, ISM330DLC_ACC_GYRO_OVERRUN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_STATUS2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_OVERRUN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_WaterMark
* Description    : Read WTM
* Input          : Pointer to ISM330DLC_ACC_GYRO_WTM_t
* Output         : Status of WTM see ISM330DLC_ACC_GYRO_WTM_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_WaterMark(void *handle, ISM330DLC_ACC_GYRO_WTM_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_STATUS2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_WTM_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FIFOPattern
* Description    : Read FIFO_PATTERN
* Input          : Pointer to u16_t
* Output         : Status of FIFO_PATTERN
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FIFOPattern(void *handle, u16_t *value)
{
  u8_t valueH, valueL;

  /* Low part from FIFO_STATUS3 */
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_STATUS3, (u8_t *)&valueL, 1) )
    return MEMS_ERROR;

  valueL &= ISM330DLC_ACC_GYRO_FIFO_STATUS3_PATTERN_MASK; //coerce
  valueL = valueL >> ISM330DLC_ACC_GYRO_FIFO_STATUS3_PATTERN_POSITION; //mask

  /* High part from FIFO_STATUS4 */
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_STATUS4, (u8_t *)&valueH, 1) )
    return MEMS_ERROR;

  valueH &= ISM330DLC_ACC_GYRO_FIFO_STATUS4_PATTERN_MASK; //coerce
  valueH = valueH >> ISM330DLC_ACC_GYRO_FIFO_STATUS4_PATTERN_POSITION; //mask

  *value = ((valueH << 8) & 0xFF00) | valueL;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SENS_HUB_END
* Description    : Read SENS_HUB_END
* Input          : Pointer to ISM330DLC_ACC_GYRO_SENS_HUB_END_t
* Output         : Status of SENS_HUB_END see ISM330DLC_ACC_GYRO_SENS_HUB_END_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SENS_HUB_END(void *handle, ISM330DLC_ACC_GYRO_SENS_HUB_END_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FUNC_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SENS_HUB_END_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SOFT_IRON_END
* Description    : Read SOFT_IRON_END
* Input          : Pointer to ISM330DLC_ACC_GYRO_SOFT_IRON_END_t
* Output         : Status of SOFT_IRON_END see ISM330DLC_ACC_GYRO_SOFT_IRON_END_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SOFT_IRON_END(void *handle, ISM330DLC_ACC_GYRO_SOFT_IRON_END_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FUNC_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SOFT_IRON_END_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_HardIron
* Description    : Read HI_FAIL
* Input          : Pointer to ISM330DLC_ACC_GYRO_SOFT_HARD_IRON_STAT_t
* Output         : Status of HI_FAIL see ISM330DLC_ACC_GYRO_SOFT_HARD_IRON_STAT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_HardIron(void *handle, ISM330DLC_ACC_GYRO_SOFT_HARD_IRON_STAT_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FUNC_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_HARD_IRON_STAT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TILT_EV_STATUS
* Description    : Read TILT_EV_STATUS
* Input          : Pointer to ISM330DLC_ACC_GYRO_TILT_EV_STATUS_t
* Output         : Status of TILT_EV_STATUS see ISM330DLC_ACC_GYRO_TILT_EV_STATUS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TILT_EV_STATUS(void *handle, ISM330DLC_ACC_GYRO_TILT_EV_STATUS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FUNC_SRC, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TILT_EV_STATUS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_LIR
* Description    : Write LIR
* Input          : ISM330DLC_ACC_GYRO_LIR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_LIR(void *handle, ISM330DLC_ACC_GYRO_LIR_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_LIR_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_LIR
* Description    : Read LIR
* Input          : Pointer to ISM330DLC_ACC_GYRO_LIR_t
* Output         : Status of LIR see ISM330DLC_ACC_GYRO_LIR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_LIR(void *handle, ISM330DLC_ACC_GYRO_LIR_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_LIR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TAP_Z_EN
* Description    : Write TAP_Z_EN
* Input          : ISM330DLC_ACC_GYRO_TAP_Z_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TAP_Z_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_Z_EN_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_TAP_Z_EN_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TAP_Z_EN
* Description    : Read TAP_Z_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_TAP_Z_EN_t
* Output         : Status of TAP_Z_EN see ISM330DLC_ACC_GYRO_TAP_Z_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_Z_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_Z_EN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TAP_Z_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TAP_Y_EN
* Description    : Write TAP_Y_EN
* Input          : ISM330DLC_ACC_GYRO_TAP_Y_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TAP_Y_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_Y_EN_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_TAP_Y_EN_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TAP_Y_EN
* Description    : Read TAP_Y_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_TAP_Y_EN_t
* Output         : Status of TAP_Y_EN see ISM330DLC_ACC_GYRO_TAP_Y_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_Y_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_Y_EN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TAP_Y_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TAP_X_EN
* Description    : Write TAP_X_EN
* Input          : ISM330DLC_ACC_GYRO_TAP_X_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TAP_X_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_X_EN_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_TAP_X_EN_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TAP_X_EN
* Description    : Read TAP_X_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_TAP_X_EN_t
* Output         : Status of TAP_X_EN see ISM330DLC_ACC_GYRO_TAP_X_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_X_EN(void *handle, ISM330DLC_ACC_GYRO_TAP_X_EN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TAP_X_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SLOPE_FDS
* Description    : Write SLOPE_FDS
* Input          : ISM330DLC_ACC_GYRO_SLOPE_FDS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SLOPE_FDS(void *handle, ISM330DLC_ACC_GYRO_SLOPE_FDS_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_SLOPE_FDS_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SLOPE_FDS
* Description    : Read SLOPE_FDS
* Input          : Pointer to ISM330DLC_ACC_GYRO_SLOPE_FDS_t
* Output         : Status of SLOPE_FDS see ISM330DLC_ACC_GYRO_SLOPE_FDS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SLOPE_FDS(void *handle, ISM330DLC_ACC_GYRO_SLOPE_FDS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SLOPE_FDS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_BASIC_INT
* Description    : Write INTERRUPTS_ENABLE
* Input          : ISM330DLC_ACC_GYRO_INT_EN_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_BASIC_INT(void *handle, ISM330DLC_ACC_GYRO_INT_EN_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT_EN_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_BASIC_INT
* Description    : Read INTERRUPTS_ENABLE
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT_EN_t
* Output         : Status of INTERRUPTS_ENABLE see ISM330DLC_ACC_GYRO_INT_EN_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_BASIC_INT(void *handle, ISM330DLC_ACC_GYRO_INT_EN_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_CFG1, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT_EN_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TAP_THS
* Description    : Write TAP_THS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TAP_THS(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << ISM330DLC_ACC_GYRO_TAP_THS_POSITION; //mask
  newValue &= ISM330DLC_ACC_GYRO_TAP_THS_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_TAP_THS_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TAP_THS
* Description    : Read TAP_THS
* Input          : Pointer to u8_t
* Output         : Status of TAP_THS
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TAP_THS(void *handle, u8_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_THS_6D, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TAP_THS_MASK; //coerce
  *value = *value >> ISM330DLC_ACC_GYRO_TAP_THS_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SIXD_THS
* Description    : Write SIXD_THS
* Input          : ISM330DLC_ACC_GYRO_SIXD_THS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SIXD_THS(void *handle, ISM330DLC_ACC_GYRO_SIXD_THS_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_SIXD_THS_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SIXD_THS
* Description    : Read SIXD_THS
* Input          : Pointer to ISM330DLC_ACC_GYRO_SIXD_THS_t
* Output         : Status of SIXD_THS see ISM330DLC_ACC_GYRO_SIXD_THS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SIXD_THS(void *handle, ISM330DLC_ACC_GYRO_SIXD_THS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_THS_6D, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SIXD_THS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_D4D
* Description    : Write D4D_EN
* Input          : ISM330DLC_ACC_GYRO_D4D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_D4D(void *handle, ISM330DLC_ACC_GYRO_D4D_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_D4D_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_TAP_THS_6D, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_D4D
* Description    : Read D4D_EN
* Input          : Pointer to ISM330DLC_ACC_GYRO_D4D_t
* Output         : Status of D4D_EN see ISM330DLC_ACC_GYRO_D4D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_D4D(void *handle, ISM330DLC_ACC_GYRO_D4D_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TAP_THS_6D, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_D4D_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SHOCK_Duration
* Description    : Write SHOCK
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SHOCK_Duration(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << ISM330DLC_ACC_GYRO_SHOCK_POSITION; //mask
  newValue &= ISM330DLC_ACC_GYRO_SHOCK_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_SHOCK_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SHOCK_Duration
* Description    : Read SHOCK
* Input          : Pointer to u8_t
* Output         : Status of SHOCK
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SHOCK_Duration(void *handle, u8_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT_DUR2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SHOCK_MASK; //coerce
  *value = *value >> ISM330DLC_ACC_GYRO_SHOCK_POSITION; //mask

  return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_QUIET_Duration
* Description    : Write QUIET
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_QUIET_Duration(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << ISM330DLC_ACC_GYRO_QUIET_POSITION; //mask
  newValue &= ISM330DLC_ACC_GYRO_QUIET_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_QUIET_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_QUIET_Duration
* Description    : Read QUIET
* Input          : Pointer to u8_t
* Output         : Status of QUIET
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_QUIET_Duration(void *handle, u8_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT_DUR2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_QUIET_MASK; //coerce
  *value = *value >> ISM330DLC_ACC_GYRO_QUIET_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_DUR
* Description    : Write DUR
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_DUR(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << ISM330DLC_ACC_GYRO_DUR_POSITION; //mask
  newValue &= ISM330DLC_ACC_GYRO_DUR_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_DUR_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_INT_DUR2, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_DUR
* Description    : Read DUR
* Input          : Pointer to u8_t
* Output         : Status of DUR
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_DUR(void *handle, u8_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_INT_DUR2, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_DUR_MASK; //coerce
  *value = *value >> ISM330DLC_ACC_GYRO_DUR_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_WK_THS
* Description    : Write WK_THS
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_WK_THS(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << ISM330DLC_ACC_GYRO_WK_THS_POSITION; //mask
  newValue &= ISM330DLC_ACC_GYRO_WK_THS_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_WK_THS_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_WK_THS
* Description    : Read WK_THS
* Input          : Pointer to u8_t
* Output         : Status of WK_THS
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_WK_THS(void *handle, u8_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_THS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_WK_THS_MASK; //coerce
  *value = *value >> ISM330DLC_ACC_GYRO_WK_THS_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV
* Description    : Write SINGLE_DOUBLE_TAP
* Input          : ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV(void *handle, ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_THS, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SINGLE_DOUBLE_TAP_EV
* Description    : Read SINGLE_DOUBLE_TAP
* Input          : Pointer to ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_t
* Output         : Status of SINGLE_DOUBLE_TAP see ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SINGLE_DOUBLE_TAP_EV(void *handle, ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_THS, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SLEEP_DUR
* Description    : Write SLEEP_DUR
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SLEEP_DUR(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << ISM330DLC_ACC_GYRO_SLEEP_DUR_POSITION; //mask
  newValue &= ISM330DLC_ACC_GYRO_SLEEP_DUR_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_SLEEP_DUR_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SLEEP_DUR
* Description    : Read SLEEP_DUR
* Input          : Pointer to u8_t
* Output         : Status of SLEEP_DUR
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SLEEP_DUR(void *handle, u8_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_SLEEP_DUR_MASK; //coerce
  *value = *value >> ISM330DLC_ACC_GYRO_SLEEP_DUR_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TIMER_HR
* Description    : Write TIMER_HR
* Input          : ISM330DLC_ACC_GYRO_TIMER_HR_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TIMER_HR(void *handle, ISM330DLC_ACC_GYRO_TIMER_HR_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_TIMER_HR_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TIMER_HR
* Description    : Read TIMER_HR
* Input          : Pointer to ISM330DLC_ACC_GYRO_TIMER_HR_t
* Output         : Status of TIMER_HR see ISM330DLC_ACC_GYRO_TIMER_HR_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TIMER_HR(void *handle, ISM330DLC_ACC_GYRO_TIMER_HR_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_TIMER_HR_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_WAKE_DUR
* Description    : Write WAKE_DUR
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_WAKE_DUR(void *handle, u8_t newValue)
{
  u8_t value;

  newValue = newValue << ISM330DLC_ACC_GYRO_WAKE_DUR_POSITION; //mask
  newValue &= ISM330DLC_ACC_GYRO_WAKE_DUR_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_WAKE_DUR_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_WAKE_DUR
* Description    : Read WAKE_DUR
* Input          : Pointer to u8_t
* Output         : Status of WAKE_DUR
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_WAKE_DUR(void *handle, u8_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_WAKE_DUR_MASK; //coerce
  *value = *value >> ISM330DLC_ACC_GYRO_WAKE_DUR_POSITION; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FF_THS
* Description    : Write FF_THS
* Input          : ISM330DLC_ACC_GYRO_FF_THS_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FF_THS(void *handle, ISM330DLC_ACC_GYRO_FF_THS_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FREE_FALL, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_FF_THS_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FREE_FALL, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FF_THS
* Description    : Read FF_THS
* Input          : Pointer to ISM330DLC_ACC_GYRO_FF_THS_t
* Output         : Status of FF_THS see ISM330DLC_ACC_GYRO_FF_THS_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FF_THS(void *handle, ISM330DLC_ACC_GYRO_FF_THS_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FREE_FALL, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_FF_THS_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FF_Duration
* Description    : Write FF_DUR
* Input          : u8_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FF_Duration(void *handle, u8_t newValue)
{
  u8_t valueH, valueL;
  u8_t value;

  valueL = newValue & 0x1F;
  valueH = (newValue >> 5) & 0x1;

  /* Low part in FREE_FALL reg */
  valueL = valueL << ISM330DLC_ACC_GYRO_FF_FREE_FALL_DUR_POSITION; //mask
  valueL &= ISM330DLC_ACC_GYRO_FF_FREE_FALL_DUR_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FREE_FALL, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_FF_FREE_FALL_DUR_MASK;
  value |= valueL;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_FREE_FALL, &value, 1) )
    return MEMS_ERROR;

  /* High part in WAKE_UP_DUR reg */
  valueH = valueH << ISM330DLC_ACC_GYRO_FF_WAKE_UP_DUR_POSITION; //mask
  valueH &= ISM330DLC_ACC_GYRO_FF_WAKE_UP_DUR_MASK; //coerce

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_FF_WAKE_UP_DUR_MASK;
  value |= valueH;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FF_Duration
* Description    : Read FF_DUR
* Input          : Pointer to u8_t
* Output         : Status of FF_DUR
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FF_Duration(void *handle, u8_t *value)
{
  u8_t valueH, valueL;

  /* Low part from FREE_FALL reg */
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FREE_FALL, (u8_t *)&valueL, 1) )
    return MEMS_ERROR;

  valueL &= ISM330DLC_ACC_GYRO_FF_FREE_FALL_DUR_MASK; //coerce
  valueL = valueL >> ISM330DLC_ACC_GYRO_FF_FREE_FALL_DUR_POSITION; //mask

  /* High part from WAKE_UP_DUR reg */
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_WAKE_UP_DUR, (u8_t *)&valueH, 1) )
    return MEMS_ERROR;

  valueH &= ISM330DLC_ACC_GYRO_FF_WAKE_UP_DUR_MASK; //coerce
  valueH = valueH >> ISM330DLC_ACC_GYRO_FF_WAKE_UP_DUR_POSITION; //mask

  *value = ((valueH << 5) & 0x20) | valueL;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TimerEvRouteInt1
* Description    : Write INT1_TIMER
* Input          : ISM330DLC_ACC_GYRO_INT1_TIMER_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TimerEvRouteInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_TIMER_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_TIMER_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TimerEvRouteInt1
* Description    : Read INT1_TIMER
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_TIMER_t
* Output         : Status of INT1_TIMER see ISM330DLC_ACC_GYRO_INT1_TIMER_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TimerEvRouteInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_TIMER_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_TIMER_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TiltEvOnInt1
* Description    : Write INT1_TILT
* Input          : ISM330DLC_ACC_GYRO_INT1_TILT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TiltEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_TILT_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_TILT_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TiltEvOnInt1
* Description    : Read INT1_TILT
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_TILT_t
* Output         : Status of INT1_TILT see ISM330DLC_ACC_GYRO_INT1_TILT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TiltEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_TILT_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_TILT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_6DEvOnInt1
* Description    : Write INT1_6D
* Input          : ISM330DLC_ACC_GYRO_INT1_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_6DEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_6D_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_6D_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_6DEvOnInt1
* Description    : Read INT1_6D
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_6D_t
* Output         : Status of INT1_6D see ISM330DLC_ACC_GYRO_INT1_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_6DEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_6D_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_6D_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TapEvOnInt1
* Description    : Write INT1_TAP
* Input          : ISM330DLC_ACC_GYRO_INT1_TAP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TapEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_TAP_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_TAP_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TapEvOnInt1
* Description    : Read INT1_TAP
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_TAP_t
* Output         : Status of INT1_TAP see ISM330DLC_ACC_GYRO_INT1_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TapEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_TAP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FFEvOnInt1
* Description    : Write INT1_FF
* Input          : ISM330DLC_ACC_GYRO_INT1_FF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FFEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_FF_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_FF_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FFEvOnInt1
* Description    : Read INT1_FF
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_FF_t
* Output         : Status of INT1_FF see ISM330DLC_ACC_GYRO_INT1_FF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FFEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_FF_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_FF_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_WUEvOnInt1
* Description    : Write INT1_WU
* Input          : ISM330DLC_ACC_GYRO_INT1_WU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_WUEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_WU_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_WU_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_WUEvOnInt1
* Description    : Read INT1_WU
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_WU_t
* Output         : Status of INT1_WU see ISM330DLC_ACC_GYRO_INT1_WU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_WUEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_WU_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_WU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SingleTapOnInt1
* Description    : Write INT1_SINGLE_TAP
* Input          : ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SingleTapOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SingleTapOnInt1
* Description    : Read INT1_SINGLE_TAP
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_t
* Output         : Status of INT1_SINGLE_TAP see ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SingleTapOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SleepEvOnInt1
* Description    : Write INT1_SLEEP
* Input          : ISM330DLC_ACC_GYRO_INT1_SLEEP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SleepEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_SLEEP_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT1_SLEEP_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SleepEvOnInt1
* Description    : Read INT1_SLEEP
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT1_SLEEP_t
* Output         : Status of INT1_SLEEP see ISM330DLC_ACC_GYRO_INT1_SLEEP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SleepEvOnInt1(void *handle, ISM330DLC_ACC_GYRO_INT1_SLEEP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD1_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT1_SLEEP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_MagCorrection_Int2
* Description    : Write INT2_IRON
* Input          : ISM330DLC_ACC_GYRO_INT2_IRON_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_MagCorrection_Int2(void *handle, ISM330DLC_ACC_GYRO_INT2_IRON_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_IRON_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_MagCorrection_Int2
* Description    : Read INT2_IRON
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_IRON_t
* Output         : Status of INT2_IRON see ISM330DLC_ACC_GYRO_INT2_IRON_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_MagCorrection_Int2(void *handle, ISM330DLC_ACC_GYRO_INT2_IRON_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_IRON_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TiltEvOnInt2
* Description    : Write INT2_TILT
* Input          : ISM330DLC_ACC_GYRO_INT2_TILT_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TiltEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_TILT_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_TILT_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TiltEvOnInt2
* Description    : Read INT2_TILT
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_TILT_t
* Output         : Status of INT2_TILT see ISM330DLC_ACC_GYRO_INT2_TILT_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TiltEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_TILT_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_TILT_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_6DEvOnInt2
* Description    : Write INT2_6D
* Input          : ISM330DLC_ACC_GYRO_INT2_6D_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_6DEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_6D_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_6D_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_6DEvOnInt2
* Description    : Read INT2_6D
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_6D_t
* Output         : Status of INT2_6D see ISM330DLC_ACC_GYRO_INT2_6D_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_6DEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_6D_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_6D_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_TapEvOnInt2
* Description    : Write INT2_TAP
* Input          : ISM330DLC_ACC_GYRO_INT2_TAP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_TapEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_TAP_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_TAP_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_TapEvOnInt2
* Description    : Read INT2_TAP
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_TAP_t
* Output         : Status of INT2_TAP see ISM330DLC_ACC_GYRO_INT2_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_TapEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_TAP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_FFEvOnInt2
* Description    : Write INT2_FF
* Input          : ISM330DLC_ACC_GYRO_INT2_FF_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_FFEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_FF_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_FF_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_FFEvOnInt2
* Description    : Read INT2_FF
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_FF_t
* Output         : Status of INT2_FF see ISM330DLC_ACC_GYRO_INT2_FF_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_FFEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_FF_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_FF_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_WUEvOnInt2
* Description    : Write INT2_WU
* Input          : ISM330DLC_ACC_GYRO_INT2_WU_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_WUEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_WU_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_WU_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_WUEvOnInt2
* Description    : Read INT2_WU
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_WU_t
* Output         : Status of INT2_WU see ISM330DLC_ACC_GYRO_INT2_WU_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_WUEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_WU_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_WU_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SingleTapOnInt2
* Description    : Write INT2_SINGLE_TAP
* Input          : ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SingleTapOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SingleTapOnInt2
* Description    : Read INT2_SINGLE_TAP
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_t
* Output         : Status of INT2_SINGLE_TAP see ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SingleTapOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_W_SleepEvOnInt2
* Description    : Write INT2_SLEEP
* Input          : ISM330DLC_ACC_GYRO_INT2_SLEEP_t
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_W_SleepEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_SLEEP_t newValue)
{
  u8_t value;

  if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  value &= ~ISM330DLC_ACC_GYRO_INT2_SLEEP_MASK;
  value |= newValue;

  if( !ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, &value, 1) )
    return MEMS_ERROR;

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : ISM330DLC_ACC_GYRO_R_SleepEvOnInt2
* Description    : Read INT2_SLEEP
* Input          : Pointer to ISM330DLC_ACC_GYRO_INT2_SLEEP_t
* Output         : Status of INT2_SLEEP see ISM330DLC_ACC_GYRO_INT2_SLEEP_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_R_SleepEvOnInt2(void *handle, ISM330DLC_ACC_GYRO_INT2_SLEEP_t *value)
{
 if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_MD2_CFG, (u8_t *)value, 1) )
    return MEMS_ERROR;

  *value &= ISM330DLC_ACC_GYRO_INT2_SLEEP_MASK; //mask

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : mems_status_t ISM330DLC_ACC_GYRO_Get_GetFIFOData(u8_t *buff)
* Description    : Read GetFIFOData output register
* Input          : pointer to [u8_t]
* Output         : GetFIFOData buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_Get_GetFIFOData(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension=2/1;

  k=0;
  for (i=0; i<1;i++ )
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{
		if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_FIFO_DATA_OUT_L+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;
	}
  }

  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : mems_status_t ISM330DLC_ACC_GYRO_Get_GetTimestamp(u8_t *buff)
* Description    : Read GetTimestamp output register
* Input          : pointer to [u8_t]
* Output         : GetTimestamp buffer u8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
mems_status_t ISM330DLC_ACC_GYRO_Get_GetTimestamp(void *handle, u8_t *buff)
{
  u8_t i, j, k;
  u8_t numberOfByteForDimension;

  numberOfByteForDimension=3/1;

  k=0;
  for (i=0; i<1;i++ )
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{
		if( !ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_TIMESTAMP0_REG+k, &buff[k], 1))
		  return MEMS_ERROR;
		k++;
	}
  }

  return MEMS_SUCCESS;
}

/************** Use Sensor Hub  *******************/
/*
 * Program the nine Soft Iron Matrix coefficients.
 * The SI_Matrix buffer must provide coefficients
 * in xx, xy, xz, yx, yy, yz, zx, zy, zz order.
 */
mems_status_t ISM330DLC_ACC_GYRO_SH_init_SI_Matrix(void *handle, u8_t *SI_matrix)
{
  /* Open Embedded Function Register page*/
  ISM330DLC_ACC_GYRO_W_EmbeddedAccess(handle, ISM330DLC_ACC_GYRO_EMBEDDED_ACCESS_ENABLED);

  /* Write the Soft Iron Matrix coefficients */
  ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_MAG_SI_XX, SI_matrix, 9);

  /* Close Embedded Function Register page*/
  ISM330DLC_ACC_GYRO_W_EmbeddedAccess(handle, ISM330DLC_ACC_GYRO_EMBEDDED_ACCESS_DISABLED);

  return MEMS_SUCCESS;
}

/* Read a remote device through I2C Sensor Hub Slave 0 */
mems_status_t ISM330DLC_ACC_GYRO_SH0_Program(void *handle, u8_t SlvAddr, u8_t Reg, u8_t len)
{
  /* Open Embedded Function Register page*/
  ISM330DLC_ACC_GYRO_W_EmbeddedAccess(handle, ISM330DLC_ACC_GYRO_EMBEDDED_ACCESS_ENABLED);

  /* Write remote device I2C slave address */
  SlvAddr |= 0x1; /* Raise the read op bit */
  ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_SLV0_ADD, &SlvAddr, 1);

  /* Write remote device I2C subaddress */
  ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_SLV0_SUBADD, &Reg, 1);

  /* Write number of bytes to read [SLAVE0_CONFIG - 04h ]*/
  u8_t sl0_cfg = 0;
  sl0_cfg |= 0x00;       //00 bit [7-6] : no decimation
  sl0_cfg |= 0x00;       //00 bit [5-4] : one sensor
  sl0_cfg |= 0x00;       // 0 bit [3] : source mode read disabled
  sl0_cfg |= len & 0x07; // bit [2-0] : number of bytes

  ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_SLAVE0_CONFIG, &sl0_cfg, 1);

  /* Close Embedded Function Register page*/
  ISM330DLC_ACC_GYRO_W_EmbeddedAccess(handle, ISM330DLC_ACC_GYRO_EMBEDDED_ACCESS_DISABLED);

  /* Enable FUNC */
  ISM330DLC_ACC_GYRO_W_FUNC_EN(handle, ISM330DLC_ACC_GYRO_FUNC_EN_ENABLED);

  /* MASTER_EN */
  ISM330DLC_ACC_GYRO_W_I2C_MASTER_Enable(handle, ISM330DLC_ACC_GYRO_MASTER_ON_ENABLED);

  return MEMS_SUCCESS;
}

/* Read a remote device through I2C Sensor Hub Slave 0 */
mems_status_t ISM330DLC_ACC_GYRO_SH0_ReadMem(void *handle, u8_t SlvAddr, u8_t Reg, u8_t *Bufp, u8_t len, u8_t stop)
{
  ISM330DLC_ACC_GYRO_SENS_HUB_END_t op_cmpl = ISM330DLC_ACC_GYRO_SENS_HUB_END_STILL_ONGOING;
  ISM330DLC_ACC_GYRO_XLDA_t op_update = ISM330DLC_ACC_GYRO_XLDA_NO_DATA_AVAIL;
  u8_t dummy[6];

  ISM330DLC_ACC_GYRO_W_ODR_XL(handle, ISM330DLC_ACC_GYRO_ODR_XL_POWER_DOWN);

  ISM330DLC_ACC_GYRO_SH0_Program(handle, SlvAddr, Reg, len);

  /* Syncronize the SH with internal trigger (xl) */
  ISM330DLC_ACC_GYRO_W_ODR_XL(handle, ISM330DLC_ACC_GYRO_ODR_XL_104Hz);

  /* Wait until operation is not completed */
  ISM330DLC_ACC_GYRO_GetRawAccData(handle, dummy);
  do {
    ISM330DLC_ACC_GYRO_R_XLDA(handle, &op_update);
  } while(op_update != ISM330DLC_ACC_GYRO_XLDA_DATA_AVAIL);
  do {
    ISM330DLC_ACC_GYRO_R_SENS_HUB_END(handle, &op_cmpl);
  } while(op_cmpl != ISM330DLC_ACC_GYRO_SENS_HUB_END_OP_COMPLETED);


  /* Read the result */
  ISM330DLC_ACC_GYRO_ReadReg(handle, ISM330DLC_ACC_GYRO_SENSORHUB1_REG, Bufp, len);

  ISM330DLC_ACC_GYRO_W_ODR_XL(handle, ISM330DLC_ACC_GYRO_ODR_XL_POWER_DOWN);

  if (stop) {
    /* Stop everything */
    ISM330DLC_ACC_GYRO_W_FUNC_EN(handle, ISM330DLC_ACC_GYRO_FUNC_EN_DISABLED);
    ISM330DLC_ACC_GYRO_W_I2C_MASTER_Enable(handle, ISM330DLC_ACC_GYRO_MASTER_ON_DISABLED);
  }

  return MEMS_SUCCESS;
}

/* Write a remote device through I2C Sensor Hub Slave 0 */
mems_status_t ISM330DLC_ACC_GYRO_SH0_WriteByte(void *handle, u8_t SlvAddr, u8_t Reg, u8_t Bufp)
{
  ISM330DLC_ACC_GYRO_SENS_HUB_END_t op_cmpl = ISM330DLC_ACC_GYRO_SENS_HUB_END_STILL_ONGOING;
  ISM330DLC_ACC_GYRO_XLDA_t op_update = ISM330DLC_ACC_GYRO_XLDA_NO_DATA_AVAIL;
  u8_t dummy[6];

  /* Open Embedded Function Register page*/
  ISM330DLC_ACC_GYRO_W_EmbeddedAccess(handle, ISM330DLC_ACC_GYRO_EMBEDDED_ACCESS_ENABLED);

  /* Write remote device I2C slave address */
  ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_SLV0_ADD, &SlvAddr, 1);

  /* Write remote device I2C subaddress */
  ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_SLV0_SUBADD, &Reg, 1);

  /* Write the data */
  ISM330DLC_ACC_GYRO_WriteReg(handle, ISM330DLC_ACC_GYRO_DATAWRITE_SRC_MODE_SUB_SLV0, &Bufp, 1);

  /* Close Embedded Function Register page*/
  ISM330DLC_ACC_GYRO_W_EmbeddedAccess(handle, ISM330DLC_ACC_GYRO_EMBEDDED_ACCESS_DISABLED);

  /* Enable FUNC */
  ISM330DLC_ACC_GYRO_W_FUNC_EN(handle, ISM330DLC_ACC_GYRO_FUNC_EN_ENABLED);

    /* Enable PULL_UP_EN and MASTER_EN */
  //ISM330DLC_ACC_GYRO_W_PULL_UP_EN(handle, ISM330DLC_ACC_GYRO_PULL_UP_EN_ENABLED);
  ISM330DLC_ACC_GYRO_W_I2C_MASTER_Enable(handle, ISM330DLC_ACC_GYRO_MASTER_ON_ENABLED);

  /* Syncronize the SH with internal trigger (xl) */
  ISM330DLC_ACC_GYRO_W_ODR_XL(handle, ISM330DLC_ACC_GYRO_ODR_XL_104Hz);

  /* Wait until operation is not completed */
  ISM330DLC_ACC_GYRO_GetRawAccData(handle, dummy);
  do {
    ISM330DLC_ACC_GYRO_R_XLDA(handle, &op_update);
  } while(op_update != ISM330DLC_ACC_GYRO_XLDA_DATA_AVAIL);
  do {
    ISM330DLC_ACC_GYRO_R_SENS_HUB_END(handle, &op_cmpl);
  } while(op_cmpl != ISM330DLC_ACC_GYRO_SENS_HUB_END_OP_COMPLETED);

  ISM330DLC_ACC_GYRO_W_ODR_XL(handle, ISM330DLC_ACC_GYRO_ODR_XL_POWER_DOWN);

  /* Stop everything */
  ISM330DLC_ACC_GYRO_W_FUNC_EN(handle, ISM330DLC_ACC_GYRO_FUNC_EN_DISABLED);
  ISM330DLC_ACC_GYRO_W_I2C_MASTER_Enable(handle, ISM330DLC_ACC_GYRO_MASTER_ON_DISABLED);


  return MEMS_SUCCESS;
}
