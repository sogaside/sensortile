/**
 ******************************************************************************
 * @file    MotionPM_Manager.c
 * @author  Central LAB
 * @version V3.2.0
 * @date    24-November-2016
 * @brief   This file includes Pedometer interface functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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
#include "TargetFeatures.h"
#include "../../Middlewares/ST/STM32_OSX_MotionPM_Library/osx_license.h"
#include "MotionPM_Manager.h"
/* Imported Variable -------------------------------------------------------------*/
extern float sensitivity_Mul;

/* exported Variable -------------------------------------------------------------*/
osx_MPM_output_t PM_DataOUT;


/* Private defines -----------------------------------------------------------*/

/** @addtogroup  Drv_Sensor      Drv_Sensor
  * @{
  */

/** @addtogroup Drv_MotionGR    Drv_MotionPM
  * @{
  */   

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run gesture recognition algorithm. This function collects and scale data 
* from accelerometer and calls the Pedometer Algo
* @param  SensorAxesRaw_t ACC_Value_Raw Acceleration values (x/y/z)
* @retval None
*/
void MotionPM_manager_run(SensorAxesRaw_t ACC_Value_Raw)
{
  osx_MPM_input_t iDataIN;

  iDataIN.AccX = ((float) ACC_Value_Raw.AXIS_X) * sensitivity_Mul;
  iDataIN.AccY = ((float) ACC_Value_Raw.AXIS_Y) * sensitivity_Mul;
  iDataIN.AccZ = ((float) ACC_Value_Raw.AXIS_Z) * sensitivity_Mul;

  osx_MotionPM_Update(&PM_DataOUT, &iDataIN);
  
}

/**
  * @brief  Initialize MotionPM License
  * @param  MDM_PayLoadLic_t *PayLoad Pointer to the osx License MetaData
  * @retval None
  */
void MotionPM_License_init(MDM_PayLoadLic_t *PayLoad)
{
  MCR_OSX_COPY_LICENSE_TO_MDM(osx_mpm_license,PayLoad->osxLicense);

  if(!osx_MotionPM_Initialize()) {
    OSX_BMS_PRINTF("Error MotionPM License authentication \n\r");
    while(1) {
      ;
    }
  } else {
    osx_MotionPM_GetLibVersion(PayLoad->osxLibVersion);
    OSX_BMS_PRINTF("Enabled %s\n\r",PayLoad->osxLibVersion);
    if(PayLoad->osxLicenseInitialized==0) {
      NecessityToSaveMetaDataManager=1;
      PayLoad->osxLicenseInitialized=1;
    }
  }
}

/**
* @brief  Initialises MotionPM algorithm
* @param  None
* @retval None
*/
void MotionPM_manager_init(void)
{
  TargetBoardFeatures.osxMotionPMIsInitalized=1;
  OSX_BMS_PRINTF("Initialized osxMotionPM\n\r");
}

/**
 * @}
 */ /* end of group  Drv_MotionGR        Drv_MotionPM*/

/**
 * @}
 */ /* end of group Drv_Sensor          Drv_Sensor*/

/************************ (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
