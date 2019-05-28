/**
  ******************************************************************************
  * File Name          :  stmicroelectronics_x-cube-mems1_5_2_1.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.5.2.1 instances.
  ******************************************************************************
  *
  * COPYRIGHT 2019 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_mems-library.h"
#include "main.h"

/*THIS IS NOAH EDITS TO THE CODE*/
#include "lsm6dsl.h"
#include "lsm6dsl_reg.h"
#include "bsp_motion_sensors.h"
#include "bsp_motion_sensors_ex.h"
#include "LSM6DS3_ACC_GYRO_driver.h"
// Needed to call for an interrupt. May need to add Header later.
//#include "app_mems_int_pin_a_interface.h" 
/*THIS IS NOAH EDITS TO THE CODE*/
uint8_t get_ID;
BSP_MOTION_SENSOR_Axes_t AccelData;
BSP_MOTION_SENSOR_Axes_t MyData;
int32_t x;
int32_t y;
int32_t z; 
float Sens;

void MX_MEMS_Init(void)
{
  /* USER CODE BEGIN SV */ 

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Library_Init_PreTreatment */
  
  /* USER CODE END MEMS_Library_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  /* USER CODE BEGIN SV */ 

  /* USER CODE END SV */
  
  /* USER CODE BEGIN MEMS_Library_Init_PostTreatment */
  
  /* USER CODE END MEMS_Library_Init_PostTreatment */
}
/*
 * LM background task
 */
void MX_MEMS_Process(void)
{
  /* USER CODE BEGIN MEMS_Library_Process */
	BSP_MOTION_SENSOR_Enable(LSM6DSL_0,MOTION_ACCELERO);
	//BSP_MOTION_SENSOR_GetFullScale(LSM6DSL_0, MOTION_ACCELERO, &what);
	BSP_MOTION_SENSOR_GetAxes(LSM6DSL_0,MOTION_ACCELERO, &AccelData);
	MyData.x = (float)AccelData.x * Sens;
	MyData.y = (float)AccelData.y * Sens;
	MyData.z = (float)AccelData.z * Sens;
	
	
	//if (BSP_MOTION_SENSOR_Init(LSM6DSL_0,MOTION_GYRO | MOTION_ACCELERO | MOTION_MAGNETO) != BSP_ERROR_NONE) {
		//BSP_MOTION_SENSOR_AxesRaw_t myData;
		//BSP_MOTION_SENSOR_GetAxesRaw(LSM6DSL_0,MOTION_GYRO | MOTION_ACCELERO | MOTION_MAGNETO, &myData);
	//}

  /* USER CODE END MEMS_Library_Process */
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
