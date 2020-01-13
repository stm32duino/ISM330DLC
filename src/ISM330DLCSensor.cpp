/**
 ******************************************************************************
 * @file    ISM330DLCSensor.cpp
 * @author  AST
 * @version V1.0.0
 * @date    13-January-2020
 * @brief   Implementation of an ISM330DLC Inertial Measurement Unit (IMU) 6 axes
 *          sensor.
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

#include "Arduino.h"
#include "Wire.h"
#include "ISM330DLCSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
ISM330DLCSensor::ISM330DLCSensor(TwoWire *i2c) : dev_i2c(i2c)
{
  address = ISM330DLC_ACC_GYRO_I2C_ADDRESS_HIGH;

  /* Enable register address automatically incremented during a multiple byte
     access with a serial interface. */
  if ( ISM330DLC_ACC_GYRO_W_IF_Addr_Incr( (void *)this, ISM330DLC_ACC_GYRO_IF_INC_ENABLED ) == MEMS_ERROR )
  {
    return;
  }

  /* Enable BDU */
  if ( ISM330DLC_ACC_GYRO_W_BDU( (void *)this, ISM330DLC_ACC_GYRO_BDU_BLOCK_UPDATE ) == MEMS_ERROR )
  {
    return;
  }

  /* FIFO mode selection */
  if ( ISM330DLC_ACC_GYRO_W_FIFO_MODE( (void *)this, ISM330DLC_ACC_GYRO_FIFO_MODE_BYPASS ) == MEMS_ERROR )
  {
    return;
  }

  /* Output data rate selection - power down. */
  if ( ISM330DLC_ACC_GYRO_W_ODR_XL( (void *)this, ISM330DLC_ACC_GYRO_ODR_XL_POWER_DOWN ) == MEMS_ERROR )
  {
    return;
  }

  /* Full scale selection. */
  if ( Set_X_FS( 2.0f ) == ISM330DLC_STATUS_ERROR )
  {
    return;
  }

  /* Output data rate selection - power down */
  if ( ISM330DLC_ACC_GYRO_W_ODR_G( (void *)this, ISM330DLC_ACC_GYRO_ODR_G_POWER_DOWN ) == MEMS_ERROR )
  {
    return;
  }

  /* Full scale selection. */
  if ( Set_G_FS( 2000.0f ) == ISM330DLC_STATUS_ERROR )
  {
    return;
  }

  X_Last_ODR = 104.0f;

  X_isEnabled = 0;

  G_Last_ODR = 104.0f;

  G_isEnabled = 0;
};

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
ISM330DLCSensor::ISM330DLCSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  /* Enable register address automatically incremented during a multiple byte
     access with a serial interface. */
  if ( ISM330DLC_ACC_GYRO_W_IF_Addr_Incr( (void *)this, ISM330DLC_ACC_GYRO_IF_INC_ENABLED ) == MEMS_ERROR )
  {
    return;
  }

  /* Enable BDU */
  if ( ISM330DLC_ACC_GYRO_W_BDU( (void *)this, ISM330DLC_ACC_GYRO_BDU_BLOCK_UPDATE ) == MEMS_ERROR )
  {
    return;
  }

  /* FIFO mode selection */
  if ( ISM330DLC_ACC_GYRO_W_FIFO_MODE( (void *)this, ISM330DLC_ACC_GYRO_FIFO_MODE_BYPASS ) == MEMS_ERROR )
  {
    return;
  }

  /* Output data rate selection - power down. */
  if ( ISM330DLC_ACC_GYRO_W_ODR_XL( (void *)this, ISM330DLC_ACC_GYRO_ODR_XL_POWER_DOWN ) == MEMS_ERROR )
  {
    return;
  }

  /* Full scale selection. */
  if ( Set_X_FS( 2.0f ) == ISM330DLC_STATUS_ERROR )
  {
    return;
  }

  /* Output data rate selection - power down */
  if ( ISM330DLC_ACC_GYRO_W_ODR_G( (void *)this, ISM330DLC_ACC_GYRO_ODR_G_POWER_DOWN ) == MEMS_ERROR )
  {
    return;
  }

  /* Full scale selection. */
  if ( Set_G_FS( 2000.0f ) == ISM330DLC_STATUS_ERROR )
  {
    return;
  }

  X_Last_ODR = 104.0f;

  X_isEnabled = 0;

  G_Last_ODR = 104.0f;

  G_isEnabled = 0;
};

/**
 * @brief  Enable ISM330DLC Accelerator
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_X(void)
{
  /* Check if the component is already enabled */
  if ( X_isEnabled == 1 )
  {
    return ISM330DLC_STATUS_OK;
  }

  /* Output data rate selection. */
  if ( Set_X_ODR_When_Enabled( X_Last_ODR ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  X_isEnabled = 1;

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Enable ISM330DLC Gyroscope
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_G(void)
{
  /* Check if the component is already enabled */
  if ( G_isEnabled == 1 )
  {
    return ISM330DLC_STATUS_OK;
  }

  /* Output data rate selection. */
  if ( Set_G_ODR_When_Enabled( G_Last_ODR ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  G_isEnabled = 1;

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Disable ISM330DLC Accelerator
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Disable_X(void)
{
  /* Check if the component is already disabled */
  if ( X_isEnabled == 0 )
  {
    return ISM330DLC_STATUS_OK;
  }

  /* Store actual output data rate. */
  if ( Get_X_ODR( &X_Last_ODR ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Output data rate selection - power down. */
  if ( ISM330DLC_ACC_GYRO_W_ODR_XL( (void *)this, ISM330DLC_ACC_GYRO_ODR_XL_POWER_DOWN ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  X_isEnabled = 0;

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Disable ISM330DLC Gyroscope
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Disable_G(void)
{
  /* Check if the component is already disabled */
  if ( G_isEnabled == 0 )
  {
    return ISM330DLC_STATUS_OK;
  }

  /* Store actual output data rate. */
  if ( Get_G_ODR( &G_Last_ODR ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Output data rate selection - power down */
  if ( ISM330DLC_ACC_GYRO_W_ODR_G( (void *)this, ISM330DLC_ACC_GYRO_ODR_G_POWER_DOWN ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  G_isEnabled = 0;

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Read ID of ISM330DLC Accelerometer and Gyroscope
 * @param  p_id the pointer where the ID of the device is stored
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::ReadID(uint8_t *p_id)
{
  if(!p_id)
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Read WHO AM I register */
  if ( ISM330DLC_ACC_GYRO_R_WHO_AM_I( (void *)this, p_id ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Read data from ISM330DLC Accelerometer
 * @param  pData the pointer where the accelerometer data are stored
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_X_Axes(int32_t *pData)
{
  int16_t dataRaw[3];
  float sensitivity = 0;

  /* Read raw data from ISM330DLC output register. */
  if ( Get_X_AxesRaw( dataRaw ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Get ISM330DLC actual sensitivity. */
  if ( Get_X_Sensitivity( &sensitivity ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Calculate the data. */
  pData[0] = ( int32_t )( dataRaw[0] * sensitivity );
  pData[1] = ( int32_t )( dataRaw[1] * sensitivity );
  pData[2] = ( int32_t )( dataRaw[2] * sensitivity );

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Read data from ISM330DLC Gyroscope
 * @param  pData the pointer where the gyroscope data are stored
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_G_Axes(int32_t *pData)
{
  int16_t dataRaw[3];
  float sensitivity = 0;

  /* Read raw data from ISM330DLC output register. */
  if ( Get_G_AxesRaw( dataRaw ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Get ISM330DLC actual sensitivity. */
  if ( Get_G_Sensitivity( &sensitivity ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Calculate the data. */
  pData[0] = ( int32_t )( dataRaw[0] * sensitivity );
  pData[1] = ( int32_t )( dataRaw[1] * sensitivity );
  pData[2] = ( int32_t )( dataRaw[2] * sensitivity );

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Read Accelerometer Sensitivity
 * @param  pfData the pointer where the accelerometer sensitivity is stored
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_X_Sensitivity(float *pfData)
{
  ISM330DLC_ACC_GYRO_FS_XL_t fullScale;

  /* Read actual full scale selection from sensor. */
  if ( ISM330DLC_ACC_GYRO_R_FS_XL( (void *)this, &fullScale ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch( fullScale )
  {
    case ISM330DLC_ACC_GYRO_FS_XL_2g:
      *pfData = ( float )ISM330DLC_ACC_SENSITIVITY_FOR_FS_2G;
      break;
    case ISM330DLC_ACC_GYRO_FS_XL_4g:
      *pfData = ( float )ISM330DLC_ACC_SENSITIVITY_FOR_FS_4G;
      break;
    case ISM330DLC_ACC_GYRO_FS_XL_8g:
      *pfData = ( float )ISM330DLC_ACC_SENSITIVITY_FOR_FS_8G;
      break;
    case ISM330DLC_ACC_GYRO_FS_XL_16g:
      *pfData = ( float )ISM330DLC_ACC_SENSITIVITY_FOR_FS_16G;
      break;
    default:
      *pfData = -1.0f;
      return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Read Gyroscope Sensitivity
 * @param  pfData the pointer where the gyroscope sensitivity is stored
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_G_Sensitivity(float *pfData)
{
  ISM330DLC_ACC_GYRO_FS_125_t fullScale125;
  ISM330DLC_ACC_GYRO_FS_G_t   fullScale;

  /* Read full scale 125 selection from sensor. */
  if ( ISM330DLC_ACC_GYRO_R_FS_125( (void *)this, &fullScale125 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  if ( fullScale125 == ISM330DLC_ACC_GYRO_FS_125_ENABLED )
  {
    *pfData = ( float )ISM330DLC_GYRO_SENSITIVITY_FOR_FS_125DPS;
  }

  else
  {

    /* Read actual full scale selection from sensor. */
    if ( ISM330DLC_ACC_GYRO_R_FS_G( (void *)this, &fullScale ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }

    /* Store the sensitivity based on actual full scale. */
    switch( fullScale )
    {
      case ISM330DLC_ACC_GYRO_FS_G_245dps:
        *pfData = ( float )ISM330DLC_GYRO_SENSITIVITY_FOR_FS_245DPS;
        break;
      case ISM330DLC_ACC_GYRO_FS_G_500dps:
        *pfData = ( float )ISM330DLC_GYRO_SENSITIVITY_FOR_FS_500DPS;
        break;
      case ISM330DLC_ACC_GYRO_FS_G_1000dps:
        *pfData = ( float )ISM330DLC_GYRO_SENSITIVITY_FOR_FS_1000DPS;
        break;
      case ISM330DLC_ACC_GYRO_FS_G_2000dps:
        *pfData = ( float )ISM330DLC_GYRO_SENSITIVITY_FOR_FS_2000DPS;
        break;
      default:
        *pfData = -1.0f;
        return ISM330DLC_STATUS_ERROR;
    }
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Read raw data from ISM330DLC Accelerometer
 * @param  pData the pointer where the accelerometer raw data are stored
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_X_AxesRaw(int16_t *pData)
{
  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};

  /* Read output registers from ISM330DLC_ACC_GYRO_OUTX_L_XL to ISM330DLC_ACC_GYRO_OUTZ_H_XL. */
  if ( ISM330DLC_ACC_GYRO_GetRawAccData( (void *)this, regValue ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Format the data. */
  pData[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
  pData[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
  pData[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Read raw data from ISM330DLC Gyroscope
 * @param  pData the pointer where the gyroscope raw data are stored
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_G_AxesRaw(int16_t *pData)
{
  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};

  /* Read output registers from ISM330DLC_ACC_GYRO_OUTX_L_G to ISM330DLC_ACC_GYRO_OUTZ_H_G. */
  if ( ISM330DLC_ACC_GYRO_GetRawGyroData( (void *)this, regValue ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Format the data. */
  pData[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
  pData[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
  pData[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Read ISM330DLC Accelerometer output data rate
 * @param  odr the pointer to the output data rate
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_X_ODR(float* odr)
{
  ISM330DLC_ACC_GYRO_ODR_XL_t odr_low_level;

  if ( ISM330DLC_ACC_GYRO_R_ODR_XL( (void *)this, &odr_low_level ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  switch( odr_low_level )
  {
    case ISM330DLC_ACC_GYRO_ODR_XL_POWER_DOWN:
      *odr = 0.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_XL_13Hz:
      *odr = 13.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_XL_26Hz:
      *odr = 26.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_XL_52Hz:
      *odr = 52.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_XL_104Hz:
      *odr = 104.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_XL_208Hz:
      *odr = 208.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_XL_416Hz:
      *odr = 416.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_XL_833Hz:
      *odr = 833.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_XL_1660Hz:
      *odr = 1660.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_XL_3330Hz:
      *odr = 3330.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_XL_6660Hz:
      *odr = 6660.0f;
      break;
    default:
      *odr = -1.0f;
      return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Read ISM330DLC Gyroscope output data rate
 * @param  odr the pointer to the output data rate
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_G_ODR(float* odr)
{
  ISM330DLC_ACC_GYRO_ODR_G_t odr_low_level;

  if ( ISM330DLC_ACC_GYRO_R_ODR_G( (void *)this, &odr_low_level ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  switch( odr_low_level )
  {
    case ISM330DLC_ACC_GYRO_ODR_G_POWER_DOWN:
      *odr = 0.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_G_13Hz:
      *odr = 13.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_G_26Hz:
      *odr = 26.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_G_52Hz:
      *odr = 52.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_G_104Hz:
      *odr = 104.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_G_208Hz:
      *odr = 208.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_G_416Hz:
      *odr = 416.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_G_833Hz:
      *odr = 833.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_G_1660Hz:
      *odr = 1660.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_G_3330Hz:
      *odr = 3330.0f;
      break;
    case ISM330DLC_ACC_GYRO_ODR_G_6660Hz:
      *odr = 6660.0f;
      break;
    default:
      *odr = -1.0f;
      return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Set ISM330DLC Accelerometer output data rate
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_X_ODR(float odr)
{
  if(X_isEnabled == 1)
  {
    if(Set_X_ODR_When_Enabled(odr) == ISM330DLC_STATUS_ERROR)
    {
      return ISM330DLC_STATUS_ERROR;
    }
  }
  else
  {
    if(Set_X_ODR_When_Disabled(odr) == ISM330DLC_STATUS_ERROR)
    {
      return ISM330DLC_STATUS_ERROR;
    }
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Set ISM330DLC Accelerometer output data rate when enabled
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_X_ODR_When_Enabled(float odr)
{
  ISM330DLC_ACC_GYRO_ODR_XL_t new_odr;

  new_odr = ( odr <=   13.0f ) ? ISM330DLC_ACC_GYRO_ODR_XL_13Hz
          : ( odr <=   26.0f ) ? ISM330DLC_ACC_GYRO_ODR_XL_26Hz
          : ( odr <=   52.0f ) ? ISM330DLC_ACC_GYRO_ODR_XL_52Hz
          : ( odr <=  104.0f ) ? ISM330DLC_ACC_GYRO_ODR_XL_104Hz
          : ( odr <=  208.0f ) ? ISM330DLC_ACC_GYRO_ODR_XL_208Hz
          : ( odr <=  416.0f ) ? ISM330DLC_ACC_GYRO_ODR_XL_416Hz
          : ( odr <=  833.0f ) ? ISM330DLC_ACC_GYRO_ODR_XL_833Hz
          : ( odr <= 1660.0f ) ? ISM330DLC_ACC_GYRO_ODR_XL_1660Hz
          : ( odr <= 3330.0f ) ? ISM330DLC_ACC_GYRO_ODR_XL_3330Hz
          :                      ISM330DLC_ACC_GYRO_ODR_XL_6660Hz;

  if ( ISM330DLC_ACC_GYRO_W_ODR_XL( (void *)this, new_odr ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Set ISM330DLC Accelerometer output data rate when disabled
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_X_ODR_When_Disabled(float odr)
{
  X_Last_ODR = ( odr <=   13.0f ) ? 13.0f
             : ( odr <=   26.0f ) ? 26.0f
             : ( odr <=   52.0f ) ? 52.0f
             : ( odr <=  104.0f ) ? 104.0f
             : ( odr <=  208.0f ) ? 208.0f
             : ( odr <=  416.0f ) ? 416.0f
             : ( odr <=  833.0f ) ? 833.0f
             : ( odr <= 1660.0f ) ? 1660.0f
             : ( odr <= 3330.0f ) ? 3330.0f
             :                      6660.0f;

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Set ISM330DLC Gyroscope output data rate
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_G_ODR(float odr)
{
  if(G_isEnabled == 1)
  {
    if(Set_G_ODR_When_Enabled(odr) == ISM330DLC_STATUS_ERROR)
    {
      return ISM330DLC_STATUS_ERROR;
    }
  }
  else
  {
    if(Set_G_ODR_When_Disabled(odr) == ISM330DLC_STATUS_ERROR)
    {
      return ISM330DLC_STATUS_ERROR;
    }
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Set ISM330DLC Gyroscope output data rate when enabled
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_G_ODR_When_Enabled(float odr)
{
  ISM330DLC_ACC_GYRO_ODR_G_t new_odr;

  new_odr = ( odr <=  13.0f )  ? ISM330DLC_ACC_GYRO_ODR_G_13Hz
          : ( odr <=  26.0f )  ? ISM330DLC_ACC_GYRO_ODR_G_26Hz
          : ( odr <=  52.0f )  ? ISM330DLC_ACC_GYRO_ODR_G_52Hz
          : ( odr <= 104.0f )  ? ISM330DLC_ACC_GYRO_ODR_G_104Hz
          : ( odr <= 208.0f )  ? ISM330DLC_ACC_GYRO_ODR_G_208Hz
          : ( odr <= 416.0f )  ? ISM330DLC_ACC_GYRO_ODR_G_416Hz
          : ( odr <= 833.0f )  ? ISM330DLC_ACC_GYRO_ODR_G_833Hz
          : ( odr <= 1660.0f ) ? ISM330DLC_ACC_GYRO_ODR_G_1660Hz
          : ( odr <= 3330.0f ) ? ISM330DLC_ACC_GYRO_ODR_G_3330Hz
          :                      ISM330DLC_ACC_GYRO_ODR_G_6660Hz;

  if ( ISM330DLC_ACC_GYRO_W_ODR_G( (void *)this, new_odr ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Set ISM330DLC Gyroscope output data rate when disabled
 * @param  odr the output data rate to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_G_ODR_When_Disabled(float odr)
{
  G_Last_ODR = ( odr <=  13.0f )  ? 13.0f
             : ( odr <=  26.0f )  ? 26.0f
             : ( odr <=  52.0f )  ? 52.0f
             : ( odr <= 104.0f )  ? 104.0f
             : ( odr <= 208.0f )  ? 208.0f
             : ( odr <= 416.0f )  ? 416.0f
             : ( odr <= 833.0f )  ? 833.0f
             : ( odr <= 1660.0f ) ? 1660.0f
             : ( odr <= 3330.0f ) ? 3330.0f
             :                      6660.0f;

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Read ISM330DLC Accelerometer full scale
 * @param  fullScale the pointer to the full scale
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_X_FS(float* fullScale)
{
  ISM330DLC_ACC_GYRO_FS_XL_t fs_low_level;

  if ( ISM330DLC_ACC_GYRO_R_FS_XL( (void *)this, &fs_low_level ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  switch( fs_low_level )
  {
    case ISM330DLC_ACC_GYRO_FS_XL_2g:
      *fullScale = 2.0f;
      break;
    case ISM330DLC_ACC_GYRO_FS_XL_4g:
      *fullScale = 4.0f;
      break;
    case ISM330DLC_ACC_GYRO_FS_XL_8g:
      *fullScale = 8.0f;
      break;
    case ISM330DLC_ACC_GYRO_FS_XL_16g:
      *fullScale = 16.0f;
      break;
    default:
      *fullScale = -1.0f;
      return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Read ISM330DLC Gyroscope full scale
 * @param  fullScale the pointer to the full scale
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_G_FS(float* fullScale)
{
  ISM330DLC_ACC_GYRO_FS_G_t fs_low_level;
  ISM330DLC_ACC_GYRO_FS_125_t fs_125;

  if ( ISM330DLC_ACC_GYRO_R_FS_125( (void *)this, &fs_125 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }
  if ( ISM330DLC_ACC_GYRO_R_FS_G( (void *)this, &fs_low_level ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  if ( fs_125 == ISM330DLC_ACC_GYRO_FS_125_ENABLED )
  {
    *fullScale = 125.0f;
  }

  else
  {
    switch( fs_low_level )
    {
      case ISM330DLC_ACC_GYRO_FS_G_245dps:
        *fullScale = 245.0f;
        break;
      case ISM330DLC_ACC_GYRO_FS_G_500dps:
        *fullScale = 500.0f;
        break;
      case ISM330DLC_ACC_GYRO_FS_G_1000dps:
        *fullScale = 1000.0f;
        break;
      case ISM330DLC_ACC_GYRO_FS_G_2000dps:
        *fullScale = 2000.0f;
        break;
      default:
        *fullScale = -1.0f;
        return ISM330DLC_STATUS_ERROR;
    }
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Set ISM330DLC Accelerometer full scale
 * @param  fullScale the full scale to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_X_FS(float fullScale)
{
  ISM330DLC_ACC_GYRO_FS_XL_t new_fs;

  new_fs = ( fullScale <= 2.0f ) ? ISM330DLC_ACC_GYRO_FS_XL_2g
         : ( fullScale <= 4.0f ) ? ISM330DLC_ACC_GYRO_FS_XL_4g
         : ( fullScale <= 8.0f ) ? ISM330DLC_ACC_GYRO_FS_XL_8g
         :                         ISM330DLC_ACC_GYRO_FS_XL_16g;

  if ( ISM330DLC_ACC_GYRO_W_FS_XL( (void *)this, new_fs ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief  Set ISM330DLC Gyroscope full scale
 * @param  fullScale the full scale to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_G_FS(float fullScale)
{
  ISM330DLC_ACC_GYRO_FS_G_t new_fs;

  if ( fullScale <= 125.0f )
  {
    if ( ISM330DLC_ACC_GYRO_W_FS_125( (void *)this, ISM330DLC_ACC_GYRO_FS_125_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
  }
  else
  {
    new_fs = ( fullScale <=  245.0f ) ? ISM330DLC_ACC_GYRO_FS_G_245dps
           : ( fullScale <=  500.0f ) ? ISM330DLC_ACC_GYRO_FS_G_500dps
           : ( fullScale <= 1000.0f ) ? ISM330DLC_ACC_GYRO_FS_G_1000dps
           :                            ISM330DLC_ACC_GYRO_FS_G_2000dps;

    if ( ISM330DLC_ACC_GYRO_W_FS_125( (void *)this, ISM330DLC_ACC_GYRO_FS_125_DISABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    if ( ISM330DLC_ACC_GYRO_W_FS_G( (void *)this, new_fs ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Enable free fall detection
 * @note This function sets the ISM330DLC accelerometer ODR to 416Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
*/
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_Free_Fall_Detection(void)
{
  return Enable_Free_Fall_Detection(ISM330DLC_INT1_PIN);
}

/**
 * @brief Enable free fall detection
 * @param int_pin the interrupt pin to be used
 * @note This function sets the ISM330DLC accelerometer ODR to 416Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
*/
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_Free_Fall_Detection(ISM330DLC_Interrupt_Pin_t int_pin)
{
  /* Output Data Rate selection */
  if(Set_X_ODR(416.0f) == ISM330DLC_STATUS_ERROR)
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Full scale selection */
  if ( ISM330DLC_ACC_GYRO_W_FS_XL( (void *)this, ISM330DLC_ACC_GYRO_FS_XL_2g ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* FF_DUR setting */
  if ( ISM330DLC_ACC_GYRO_W_FF_Duration( (void *)this, 0x06 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* WAKE_DUR setting */
  if ( ISM330DLC_ACC_GYRO_W_WAKE_DUR( (void *)this, 0x00 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* TIMER_HR setting */
  if ( ISM330DLC_ACC_GYRO_W_TIMER_HR( (void *)this, ISM330DLC_ACC_GYRO_TIMER_HR_6_4ms ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* SLEEP_DUR setting */
  if ( ISM330DLC_ACC_GYRO_W_SLEEP_DUR( (void *)this, 0x00 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* FF_THS setting */
  if ( ISM330DLC_ACC_GYRO_W_FF_THS( (void *)this, ISM330DLC_ACC_GYRO_FF_THS_312mg ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable basic Interrupts */
  if ( ISM330DLC_ACC_GYRO_W_BASIC_INT( (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable free fall event on either INT1 or INT2 pin */
  switch (int_pin)
  {
  case ISM330DLC_INT1_PIN:
    if ( ISM330DLC_ACC_GYRO_W_FFEvOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_FF_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if ( ISM330DLC_ACC_GYRO_W_FFEvOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_FF_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  default:
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Disable free fall detection
 * @param None
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
*/
ISM330DLCStatusTypeDef ISM330DLCSensor::Disable_Free_Fall_Detection(void)
{
  /* Disable free fall event on INT1 pin */
  if ( ISM330DLC_ACC_GYRO_W_FFEvOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_FF_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable free fall event on INT2 pin */
  if ( ISM330DLC_ACC_GYRO_W_FFEvOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_FF_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable basic Interrupts */
  if ( ISM330DLC_ACC_GYRO_W_BASIC_INT( (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* FF_DUR setting */
  if ( ISM330DLC_ACC_GYRO_W_FF_Duration( (void *)this, 0x00 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* FF_THS setting */
  if ( ISM330DLC_ACC_GYRO_W_FF_THS( (void *)this, ISM330DLC_ACC_GYRO_FF_THS_156mg ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Set the free fall detection threshold for ISM330DLC accelerometer sensor
 * @param thr the threshold to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_Free_Fall_Threshold(uint8_t thr)
{

  if ( ISM330DLC_ACC_GYRO_W_FF_THS( (void *)this, (ISM330DLC_ACC_GYRO_FF_THS_t)thr ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Enable the pedometer feature for ISM330DLC accelerometer sensor
 * @note This function sets the ISM330DLC accelerometer ODR to 26Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_Pedometer(void)
{
  /* Output Data Rate selection */
  if( Set_X_ODR(26.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Full scale selection. */
  if( Set_X_FS(2.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Set pedometer threshold. */
  if ( Set_Pedometer_Threshold(ISM330DLC_PEDOMETER_THRESHOLD_MID_HIGH) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable embedded functionalities. */
  if ( ISM330DLC_ACC_GYRO_W_FUNC_EN( (void *)this, ISM330DLC_ACC_GYRO_FUNC_EN_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable pedometer algorithm. */
  if ( ISM330DLC_ACC_GYRO_W_PEDO( (void *)this, ISM330DLC_ACC_GYRO_PEDO_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable pedometer on INT1. */
  if ( ISM330DLC_ACC_GYRO_W_STEP_DET_on_INT1( (void *)this, ISM330DLC_ACC_GYRO_INT1_PEDO_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Disable the pedometer feature for ISM330DLC accelerometer sensor
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Disable_Pedometer(void)
{
  /* Disable pedometer on INT1. */
  if ( ISM330DLC_ACC_GYRO_W_STEP_DET_on_INT1( (void *)this, ISM330DLC_ACC_GYRO_INT1_PEDO_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable pedometer algorithm. */
  if ( ISM330DLC_ACC_GYRO_W_PEDO( (void *)this, ISM330DLC_ACC_GYRO_PEDO_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable embedded functionalities. */
  if ( ISM330DLC_ACC_GYRO_W_FUNC_EN( (void *)this, ISM330DLC_ACC_GYRO_FUNC_EN_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Reset pedometer threshold. */
  if ( Set_Pedometer_Threshold(0x0) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Get the step counter for ISM330DLC accelerometer sensor
 * @param step_count the pointer to the step counter
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_Step_Counter(uint16_t *step_count)
{
  if ( ISM330DLC_ACC_GYRO_Get_GetStepCounter( (void *)this, ( uint8_t* )step_count ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Reset of the step counter for ISM330DLC accelerometer sensor
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Reset_Step_Counter(void)
{
  if ( ISM330DLC_ACC_GYRO_W_PedoStepReset( (void *)this, ISM330DLC_ACC_GYRO_PEDO_RST_STEP_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  delay(10);

  if ( ISM330DLC_ACC_GYRO_W_PedoStepReset( (void *)this, ISM330DLC_ACC_GYRO_PEDO_RST_STEP_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Set the pedometer threshold for ISM330DLC accelerometer sensor
 * @param thr the threshold to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_Pedometer_Threshold(uint8_t thr)
{
  if ( ISM330DLC_ACC_GYRO_W_PedoThreshold( (void *)this, thr ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Enable the tilt detection for ISM330DLC accelerometer sensor
 * @note This function sets the ISM330DLC accelerometer ODR to 26Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_Tilt_Detection(void)
{
  return Enable_Tilt_Detection(ISM330DLC_INT1_PIN);
}

/**
 * @brief Enable the tilt detection for ISM330DLC accelerometer sensor
 * @param int_pin the interrupt pin to be used
 * @note This function sets the ISM330DLC accelerometer ODR to 26Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_Tilt_Detection(ISM330DLC_Interrupt_Pin_t int_pin)
{
  /* Output Data Rate selection */
  if( Set_X_ODR(26.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Full scale selection. */
  if( Set_X_FS(2.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable embedded functionalities */
  if ( ISM330DLC_ACC_GYRO_W_FUNC_EN( (void *)this, ISM330DLC_ACC_GYRO_FUNC_EN_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable tilt calculation. */
  if ( ISM330DLC_ACC_GYRO_W_TILT( (void *)this, ISM330DLC_ACC_GYRO_TILT_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable tilt detection on either INT1 or INT2 pin */
  switch (int_pin)
  {
  case ISM330DLC_INT1_PIN:
    if ( ISM330DLC_ACC_GYRO_W_TiltEvOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_TILT_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if ( ISM330DLC_ACC_GYRO_W_TiltEvOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_TILT_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  default:
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Disable the tilt detection for ISM330DLC accelerometer sensor
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Disable_Tilt_Detection(void)
{
  /* Disable tilt event on INT1. */
  if ( ISM330DLC_ACC_GYRO_W_TiltEvOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_TILT_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable tilt event on INT2. */
  if ( ISM330DLC_ACC_GYRO_W_TiltEvOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_TILT_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable tilt calculation. */
  if ( ISM330DLC_ACC_GYRO_W_TILT( (void *)this, ISM330DLC_ACC_GYRO_TILT_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable embedded functionalities */
  if ( ISM330DLC_ACC_GYRO_W_FUNC_EN( (void *)this, ISM330DLC_ACC_GYRO_FUNC_EN_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Enable the wake up detection for ISM330DLC accelerometer sensor
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_Wake_Up_Detection(void)
{
  return Enable_Wake_Up_Detection(ISM330DLC_INT2_PIN);
}

/**
 * @brief Enable the wake up detection for ISM330DLC accelerometer sensor
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_Wake_Up_Detection(ISM330DLC_Interrupt_Pin_t int_pin)
{
  /* Output Data Rate selection */
  if( Set_X_ODR(416.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Full scale selection. */
  if( Set_X_FS(2.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* WAKE_DUR setting */
  if ( ISM330DLC_ACC_GYRO_W_WAKE_DUR( (void *)this, 0x00 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Set wake up threshold. */
  if ( ISM330DLC_ACC_GYRO_W_WK_THS( (void *)this, 0x02 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable basic Interrupts */
  if ( ISM330DLC_ACC_GYRO_W_BASIC_INT( (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable wake up detection on either INT1 or INT2 pin */
  switch (int_pin)
  {
  case ISM330DLC_INT1_PIN:
    if ( ISM330DLC_ACC_GYRO_W_WUEvOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_WU_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if ( ISM330DLC_ACC_GYRO_W_WUEvOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_WU_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  default:
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Disable the wake up detection for ISM330DLC accelerometer sensor
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Disable_Wake_Up_Detection(void)
{
  /* Disable wake up event on INT1 */
  if ( ISM330DLC_ACC_GYRO_W_WUEvOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_WU_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable wake up event on INT2 */
  if ( ISM330DLC_ACC_GYRO_W_WUEvOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_WU_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable basic Interrupts */
  if ( ISM330DLC_ACC_GYRO_W_BASIC_INT( (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* WU_DUR setting */
  if ( ISM330DLC_ACC_GYRO_W_WAKE_DUR( (void *)this, 0x00 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* WU_THS setting */
  if ( ISM330DLC_ACC_GYRO_W_WK_THS( (void *)this, 0x00 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Set the wake up threshold for ISM330DLC accelerometer sensor
 * @param thr the threshold to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_Wake_Up_Threshold(uint8_t thr)
{
  if ( ISM330DLC_ACC_GYRO_W_WK_THS( (void *)this, thr ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Enable the single tap detection for ISM330DLC accelerometer sensor
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_Single_Tap_Detection(void)
{
  return Enable_Single_Tap_Detection(ISM330DLC_INT1_PIN);
}

/**
 * @brief Enable the single tap detection for ISM330DLC accelerometer sensor
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_Single_Tap_Detection(ISM330DLC_Interrupt_Pin_t int_pin)
{
  /* Output Data Rate selection */
  if( Set_X_ODR(416.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Full scale selection. */
  if( Set_X_FS(2.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_X_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_X_EN_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_Y_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_Y_EN_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_Z_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_Z_EN_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Set tap threshold. */
  if ( Set_Tap_Threshold( ISM330DLC_TAP_THRESHOLD_MID_LOW ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Set tap shock time window. */
  if ( Set_Tap_Shock_Time( ISM330DLC_TAP_SHOCK_TIME_MID_HIGH ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Set tap quiet time window. */
  if ( Set_Tap_Quiet_Time( ISM330DLC_TAP_QUIET_TIME_MID_LOW ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* _NOTE_: Tap duration time window - don't care for single tap. */

  /* _NOTE_: Single/Double Tap event - don't care of this flag for single tap. */

  /* Enable basic Interrupts */
  if ( ISM330DLC_ACC_GYRO_W_BASIC_INT( (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable single tap on either INT1 or INT2 pin */
  switch (int_pin)
  {
  case ISM330DLC_INT1_PIN:
    if ( ISM330DLC_ACC_GYRO_W_SingleTapOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if ( ISM330DLC_ACC_GYRO_W_SingleTapOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  default:
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Disable the single tap detection for ISM330DLC accelerometer sensor
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Disable_Single_Tap_Detection(void)
{
  /* Disable single tap interrupt on INT1 pin. */
  if ( ISM330DLC_ACC_GYRO_W_SingleTapOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable single tap interrupt on INT2 pin. */
  if ( ISM330DLC_ACC_GYRO_W_SingleTapOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable basic Interrupts */
  if ( ISM330DLC_ACC_GYRO_W_BASIC_INT( (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Reset tap threshold. */
  if ( Set_Tap_Threshold( 0x0 ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Reset tap shock time window. */
  if ( Set_Tap_Shock_Time( 0x0 ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Reset tap quiet time window. */
  if ( Set_Tap_Quiet_Time( 0x0 ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* _NOTE_: Tap duration time window - don't care for single tap. */

  /* _NOTE_: Single/Double Tap event - don't care of this flag for single tap. */

  /* Disable Z direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_Z_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_Z_EN_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_Y_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_Y_EN_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_X_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_X_EN_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Enable the double tap detection for ISM330DLC accelerometer sensor
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_Double_Tap_Detection(void)
{
  return Enable_Double_Tap_Detection(ISM330DLC_INT1_PIN);
}

/**
 * @brief Enable the double tap detection for ISM330DLC accelerometer sensor
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_Double_Tap_Detection(ISM330DLC_Interrupt_Pin_t int_pin)
{
  /* Output Data Rate selection */
  if( Set_X_ODR(416.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Full scale selection. */
  if( Set_X_FS(2.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable X direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_X_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_X_EN_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable Y direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_Y_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_Y_EN_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable Z direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_Z_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_Z_EN_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Set tap threshold. */
  if ( Set_Tap_Threshold( ISM330DLC_TAP_THRESHOLD_MID_LOW ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Set tap shock time window. */
  if ( Set_Tap_Shock_Time( ISM330DLC_TAP_SHOCK_TIME_HIGH ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Set tap quiet time window. */
  if ( Set_Tap_Quiet_Time( ISM330DLC_TAP_QUIET_TIME_HIGH ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Set tap duration time window. */
  if ( Set_Tap_Duration_Time( ISM330DLC_TAP_DURATION_TIME_MID ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Single and double tap enabled. */
  if ( ISM330DLC_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV( (void *)this, ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_DOUBLE_TAP ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable basic Interrupts */
  if ( ISM330DLC_ACC_GYRO_W_BASIC_INT( (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable double tap on either INT1 or INT2 pin */
  switch (int_pin)
  {
  case ISM330DLC_INT1_PIN:
    if ( ISM330DLC_ACC_GYRO_W_TapEvOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_TAP_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if ( ISM330DLC_ACC_GYRO_W_TapEvOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_TAP_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  default:
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Disable the double tap detection for ISM330DLC accelerometer sensor
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Disable_Double_Tap_Detection(void)
{
  /* Disable double tap interrupt on INT1 pin. */
  if ( ISM330DLC_ACC_GYRO_W_TapEvOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_TAP_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable double tap interrupt on INT2 pin. */
  if ( ISM330DLC_ACC_GYRO_W_TapEvOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_TAP_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable basic Interrupts */
  if ( ISM330DLC_ACC_GYRO_W_BASIC_INT( (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Reset tap threshold. */
  if ( Set_Tap_Threshold( 0x0 ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Reset tap shock time window. */
  if ( Set_Tap_Shock_Time( 0x0 ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Reset tap quiet time window. */
  if ( Set_Tap_Quiet_Time( 0x0 ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Reset tap duration time window. */
  if ( Set_Tap_Duration_Time( 0x0 ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Only single tap enabled. */
  if ( ISM330DLC_ACC_GYRO_W_SINGLE_DOUBLE_TAP_EV( (void *)this, ISM330DLC_ACC_GYRO_SINGLE_DOUBLE_TAP_SINGLE_TAP ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable Z direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_Z_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_Z_EN_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable Y direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_Y_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_Y_EN_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable X direction in tap recognition. */
  if ( ISM330DLC_ACC_GYRO_W_TAP_X_EN( (void *)this, ISM330DLC_ACC_GYRO_TAP_X_EN_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Set the tap threshold for ISM330DLC accelerometer sensor
 * @param thr the threshold to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_Tap_Threshold(uint8_t thr)
{
  if ( ISM330DLC_ACC_GYRO_W_TAP_THS( (void *)this, thr ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Set the tap shock time window for ISM330DLC accelerometer sensor
 * @param time the shock time window to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_Tap_Shock_Time(uint8_t time)
{
  if ( ISM330DLC_ACC_GYRO_W_SHOCK_Duration( (void *)this, time ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Set the tap quiet time window for ISM330DLC accelerometer sensor
 * @param time the quiet time window to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_Tap_Quiet_Time(uint8_t time)
{
  if ( ISM330DLC_ACC_GYRO_W_QUIET_Duration( (void *)this, time ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Set the tap duration of the time window for ISM330DLC accelerometer sensor
 * @param time the duration of the time window to be set
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Set_Tap_Duration_Time(uint8_t time)
{
  if ( ISM330DLC_ACC_GYRO_W_DUR( (void *)this, time ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Enable the 6D orientation detection for ISM330DLC accelerometer sensor
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_6D_Orientation(void)
{
  return Enable_6D_Orientation(ISM330DLC_INT1_PIN);
}

/**
 * @brief Enable the 6D orientation detection for ISM330DLC accelerometer sensor
 * @param int_pin the interrupt pin to be used
 * @note  This function sets the ISM330DLC accelerometer ODR to 416Hz and the ISM330DLC accelerometer full scale to 2g
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Enable_6D_Orientation(ISM330DLC_Interrupt_Pin_t int_pin)
{
  /* Output Data Rate selection */
  if( Set_X_ODR(416.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Full scale selection. */
  if( Set_X_FS(2.0f) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Set 6D threshold. */
  if ( ISM330DLC_ACC_GYRO_W_SIXD_THS( (void *)this, ISM330DLC_ACC_GYRO_SIXD_THS_60_degree ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable basic Interrupts */
  if ( ISM330DLC_ACC_GYRO_W_BASIC_INT( (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_ENABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Enable 6D orientation on either INT1 or INT2 pin */
  switch (int_pin)
  {
  case ISM330DLC_INT1_PIN:
    if ( ISM330DLC_ACC_GYRO_W_6DEvOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_6D_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  case ISM330DLC_INT2_PIN:
    if ( ISM330DLC_ACC_GYRO_W_6DEvOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_6D_ENABLED ) == MEMS_ERROR )
    {
      return ISM330DLC_STATUS_ERROR;
    }
    break;

  default:
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Disable the 6D orientation detection for ISM330DLC accelerometer sensor
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Disable_6D_Orientation(void)
{
  /* Disable 6D orientation interrupt on INT1 pin. */
  if ( ISM330DLC_ACC_GYRO_W_6DEvOnInt1( (void *)this, ISM330DLC_ACC_GYRO_INT1_6D_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable 6D orientation interrupt on INT2 pin. */
  if ( ISM330DLC_ACC_GYRO_W_6DEvOnInt2( (void *)this, ISM330DLC_ACC_GYRO_INT2_6D_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Disable basic Interrupts */
  if ( ISM330DLC_ACC_GYRO_W_BASIC_INT( (void *)this, ISM330DLC_ACC_GYRO_BASIC_INT_DISABLED ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  /* Reset 6D threshold. */
  if ( ISM330DLC_ACC_GYRO_W_SIXD_THS( (void *)this, ISM330DLC_ACC_GYRO_SIXD_THS_80_degree ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Get the 6D orientation XL axis for ISM330DLC accelerometer sensor
 * @param xl the pointer to the 6D orientation XL axis
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_6D_Orientation_XL(uint8_t *xl)
{
  ISM330DLC_ACC_GYRO_DSD_XL_t xl_raw;

  if ( ISM330DLC_ACC_GYRO_R_DSD_XL( (void *)this, &xl_raw ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  switch( xl_raw )
  {
    case ISM330DLC_ACC_GYRO_DSD_XL_DETECTED:
      *xl = 1;
      break;
    case ISM330DLC_ACC_GYRO_DSD_XL_NOT_DETECTED:
      *xl = 0;
      break;
    default:
      return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Get the 6D orientation XH axis for ISM330DLC accelerometer sensor
 * @param xh the pointer to the 6D orientation XH axis
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_6D_Orientation_XH(uint8_t *xh)
{
  ISM330DLC_ACC_GYRO_DSD_XH_t xh_raw;

  if ( ISM330DLC_ACC_GYRO_R_DSD_XH( (void *)this, &xh_raw ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  switch( xh_raw )
  {
    case ISM330DLC_ACC_GYRO_DSD_XH_DETECTED:
      *xh = 1;
      break;
    case ISM330DLC_ACC_GYRO_DSD_XH_NOT_DETECTED:
      *xh = 0;
      break;
    default:
      return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Get the 6D orientation YL axis for ISM330DLC accelerometer sensor
 * @param yl the pointer to the 6D orientation YL axis
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_6D_Orientation_YL(uint8_t *yl)
{
  ISM330DLC_ACC_GYRO_DSD_YL_t yl_raw;

  if ( ISM330DLC_ACC_GYRO_R_DSD_YL( (void *)this, &yl_raw ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  switch( yl_raw )
  {
    case ISM330DLC_ACC_GYRO_DSD_YL_DETECTED:
      *yl = 1;
      break;
    case ISM330DLC_ACC_GYRO_DSD_YL_NOT_DETECTED:
      *yl = 0;
      break;
    default:
      return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Get the 6D orientation YH axis for ISM330DLC accelerometer sensor
 * @param yh the pointer to the 6D orientation YH axis
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_6D_Orientation_YH(uint8_t *yh)
{
  ISM330DLC_ACC_GYRO_DSD_YH_t yh_raw;

  if ( ISM330DLC_ACC_GYRO_R_DSD_YH( (void *)this, &yh_raw ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  switch( yh_raw )
  {
    case ISM330DLC_ACC_GYRO_DSD_YH_DETECTED:
      *yh = 1;
      break;
    case ISM330DLC_ACC_GYRO_DSD_YH_NOT_DETECTED:
      *yh = 0;
      break;
    default:
      return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Get the 6D orientation ZL axis for ISM330DLC accelerometer sensor
 * @param zl the pointer to the 6D orientation ZL axis
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_6D_Orientation_ZL(uint8_t *zl)
{
  ISM330DLC_ACC_GYRO_DSD_ZL_t zl_raw;

  if ( ISM330DLC_ACC_GYRO_R_DSD_ZL( (void *)this, &zl_raw ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  switch( zl_raw )
  {
    case ISM330DLC_ACC_GYRO_DSD_ZL_DETECTED:
      *zl = 1;
      break;
    case ISM330DLC_ACC_GYRO_DSD_ZL_NOT_DETECTED:
      *zl = 0;
      break;
    default:
      return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Get the 6D orientation ZH axis for ISM330DLC accelerometer sensor
 * @param zh the pointer to the 6D orientation ZH axis
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_6D_Orientation_ZH(uint8_t *zh)
{
  ISM330DLC_ACC_GYRO_DSD_ZH_t zh_raw;

  if ( ISM330DLC_ACC_GYRO_R_DSD_ZH( (void *)this, &zh_raw ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  switch( zh_raw )
  {
    case ISM330DLC_ACC_GYRO_DSD_ZH_DETECTED:
      *zh = 1;
      break;
    case ISM330DLC_ACC_GYRO_DSD_ZH_NOT_DETECTED:
      *zh = 0;
      break;
    default:
      return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Get the status of all hardware events for ISM330DLC accelerometer sensor
 * @param status the pointer to the status of all hardware events
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::Get_Event_Status( ISM330DLC_Event_Status_t *status )
{
  uint8_t Wake_Up_Src = 0, Tap_Src = 0, D6D_Src = 0, Func_Src = 0, Md1_Cfg = 0, Md2_Cfg = 0, Int1_Ctrl = 0;

  memset((void *)status, 0x0, sizeof(ISM330DLC_Event_Status_t));

  if(ReadReg(ISM330DLC_ACC_GYRO_WAKE_UP_SRC, &Wake_Up_Src ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  if(ReadReg(ISM330DLC_ACC_GYRO_TAP_SRC, &Tap_Src ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  if(ReadReg(ISM330DLC_ACC_GYRO_D6D_SRC, &D6D_Src ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  if(ReadReg(ISM330DLC_ACC_GYRO_FUNC_SRC, &Func_Src ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  if(ReadReg(ISM330DLC_ACC_GYRO_MD1_CFG, &Md1_Cfg ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  if(ReadReg(ISM330DLC_ACC_GYRO_MD2_CFG, &Md2_Cfg ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  if(ReadReg(ISM330DLC_ACC_GYRO_INT1_CTRL, &Int1_Ctrl ) == ISM330DLC_STATUS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  if((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_FF_MASK) || (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_FF_MASK))
  {
    if((Wake_Up_Src & ISM330DLC_ACC_GYRO_FF_EV_STATUS_MASK))
    {
      status->FreeFallStatus = 1;
    }
  }

  if((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_WU_MASK) || (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_WU_MASK))
  {
    if((Wake_Up_Src & ISM330DLC_ACC_GYRO_WU_EV_STATUS_MASK))
    {
      status->WakeUpStatus = 1;
    }
  }

  if((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_SINGLE_TAP_MASK) || (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_SINGLE_TAP_MASK))
  {
    if((Tap_Src & ISM330DLC_ACC_GYRO_SINGLE_TAP_EV_STATUS_MASK))
    {
      status->TapStatus = 1;
    }
  }

  if((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_TAP_MASK) || (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_TAP_MASK))
  {
    if((Tap_Src & ISM330DLC_ACC_GYRO_DOUBLE_TAP_EV_STATUS_MASK))
    {
      status->DoubleTapStatus = 1;
    }
  }

  if((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_6D_MASK) || (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_6D_MASK))
  {
    if((D6D_Src & ISM330DLC_ACC_GYRO_D6D_EV_STATUS_MASK))
    {
      status->D6DOrientationStatus = 1;
    }
  }

  if((Int1_Ctrl & ISM330DLC_ACC_GYRO_INT1_PEDO_MASK))
  {
    if((Func_Src & ISM330DLC_ACC_GYRO_PEDO_EV_STATUS_MASK))
    {
      status->StepStatus = 1;
    }
  }

  if((Md1_Cfg & ISM330DLC_ACC_GYRO_INT1_TILT_MASK) || (Md2_Cfg & ISM330DLC_ACC_GYRO_INT2_TILT_MASK))
  {
    if((Func_Src & ISM330DLC_ACC_GYRO_TILT_EV_STATUS_MASK))
    {
      status->TiltStatus = 1;
    }
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Read the data from register
 * @param reg register address
 * @param data register data
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::ReadReg( uint8_t reg, uint8_t *data )
{

  if ( ISM330DLC_ACC_GYRO_ReadReg( (void *)this, reg, data, 1 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}

/**
 * @brief Write the data to register
 * @param reg register address
 * @param data register data
 * @retval ISM330DLC_STATUS_OK in case of success, an error code otherwise
 */
ISM330DLCStatusTypeDef ISM330DLCSensor::WriteReg( uint8_t reg, uint8_t data )
{

  if ( ISM330DLC_ACC_GYRO_WriteReg( (void *)this, reg, &data, 1 ) == MEMS_ERROR )
  {
    return ISM330DLC_STATUS_ERROR;
  }

  return ISM330DLC_STATUS_OK;
}


uint8_t ISM330DLC_IO_Write( void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite )
{
  return ((ISM330DLCSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

uint8_t ISM330DLC_IO_Read( void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead )
{
  return ((ISM330DLCSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
