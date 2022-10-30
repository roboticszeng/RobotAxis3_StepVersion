/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

#include "stdio.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
//#define RXBUFFERSIZE  256     //最大接收字节数

char Rx1_Buffer[RXBUFFERSIZE];  //接收数据
uint8_t Rx1_tmpBuffer;			      //接收中断缓冲
uint8_t Uart1_Rx_Cnt = 0;     //接收缓冲计数

char Rx2_Buffer[RXBUFFERSIZE];  //接收数据
uint8_t Rx2_tmpBuffer;			      //接收中断缓冲
uint8_t Uart2_Rx_Cnt = 0;     //接收缓冲计数

void HAL_UART_Rx1Callback(void);
void HAL_UART_Rx2Callback(void);
uint8_t *protocol_analysis(char cmdRes[]);
uint8_t *setPositionControlCmd(int actPosJoint, int targPosJoint, int j);
int deg2pulseSingleJoint(double theta, int i);
double pulse2radSingleJoint(int pulse, int i);
double *getInverseKinematics(double X, double Y);
uint8_t *setJogControlCmd(char cmdRes[]);

extern int flagGetEncPulse;

const int zeroPos[3] = {42287, 53530, 27074};
const int directionFlag[3] = {1, -1, 1};

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
    
    if (huart == &huart1){
        HAL_UART_Rx1Callback();
    }
    else if(huart == &huart2){
        HAL_UART_Rx2Callback();
    }

}

void HAL_UART_Rx1Callback(void){
    
    static uint8_t *cmdSend;
    Rx1_Buffer[Uart1_Rx_Cnt++] = Rx1_tmpBuffer;   //
    if(Rx1_Buffer[Uart1_Rx_Cnt-1] == 0x0d){
        cmdSend = protocol_analysis(Rx1_Buffer);
//        HAL_UART_Transmit(&huart2, cmdSend, 5, 0xFFFF);
        while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);//检测UART发送结束
        flagGetEncPulse = 0;
        Uart1_Rx_Cnt = 0;
        memset(Rx1_Buffer, 0x00, sizeof(Rx1_Buffer)); //清空数组
    }
	
	HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx1_tmpBuffer, 1);   //因为接收中断使用了一次即关闭，所以在最后加入这行代码即可实现无限使用
} 


extern int jointIndex;
int actualPosition[3];

void HAL_UART_Rx2Callback(void){
    
    static int flag = 0;
    static int U_CNT_MAX = 3;
    
//    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    
    
    if(flagGetEncPulse == 0){
        U_CNT_MAX = 2;
    }
    else{
        U_CNT_MAX = 3;
    }
    
    
    if (Rx2_tmpBuffer == 0xe0 || Rx2_tmpBuffer == 0xe1 || Rx2_tmpBuffer == 0xe2){
        Rx2_Buffer[0] = Rx2_tmpBuffer;
        flag = 1;
    }
    if (flag == 1 && Rx2_tmpBuffer != 0){
        Rx2_Buffer[Uart2_Rx_Cnt++] = Rx2_tmpBuffer;
    }
    
    if(Uart2_Rx_Cnt == U_CNT_MAX){
        if(flagGetEncPulse == 1){
            actualPosition[Rx2_Buffer[0] - 0xe0] = Rx2_Buffer[1] * 256 + Rx2_Buffer[2];
        }
        Uart2_Rx_Cnt = 0;
        memset(Rx2_Buffer, 0x00, sizeof(Rx2_Buffer)); //清空数组
        flag = 0;
    }
	
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&Rx2_tmpBuffer, 1);   //因为接收中断使用了一次即关闭，所以在最后加入这行代码即可实现无限使用

}

uint8_t *protocol_analysis(char cmdRes[]){
    
    uint8_t cmdSend[15];
    uint8_t *cmdReturn, *cmdJog;
    double theta, X, Y;
    double *thetaInv;
    static int targetPosition[3];
    int i, j;
    
    
//    cmdSend[0] = 0xe1;
//    cmdSend[1] = 0x30;
    
    
    switch (cmdRes[0]){
        case 0x11:
            printf("forward kinematics mode.\r\n");
            for(i = 0; i < DOF; i++){
                theta = (cmdRes[2*i+1] == 0)? cmdRes[2*i+2] : -cmdRes[2*i+2];
                targetPosition[i] = deg2pulseSingleJoint(theta, i);
                cmdReturn = setPositionControlCmd(actualPosition[i], targetPosition[i], i);
                HAL_UART_Transmit(&huart2, cmdReturn, 5, 0xFFFF);
                while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);
            }
            break;
        case 0x12:
            printf("inverse kinematics mode.\r\n");
//            theta = (cmdRes[1] == 0)? cmdRes[2] : -cmdRes[2];
            X = (cmdRes[1] == 0)? cmdRes[2] : -cmdRes[2];
            Y = (cmdRes[3] == 0)? cmdRes[4] : -cmdRes[4];
//            targetPosition[0] = deg2pulseSingleJoint(theta, 0);
            thetaInv = getInverseKinematics(X, Y);
            thetaInv[0] = (cmdRes[5] == 0)? cmdRes[6] : -cmdRes[6];
            thetaInv[0] = thetaInv[0] * PI / 180;
            for(i = 0; i < DOF; i++){
                targetPosition[i] = deg2pulseSingleJoint(thetaInv[i] * 180 / PI, i);
                cmdReturn = setPositionControlCmd(actualPosition[i], targetPosition[i], i);
                HAL_UART_Transmit(&huart2, cmdReturn, 5, 0xFFFF);
                while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);
            }
//            printf("tg = %d %d %d \r\n", targetPosition[0], targetPosition[1], targetPosition[2]);
            break;
        case 0x13:
            printf("jog mode.\r\n");
            cmdReturn = setJogControlCmd(cmdRes);
            HAL_UART_Transmit(&huart2, cmdReturn, 5, 0xFFFF);
            while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX);
        
            break;
        case 'T':
            printf("Drag-teach mode.\r\n");
            break;
        default:
            printf("error mode.\r\n");
    }
    
    return cmdReturn;
    
}

uint8_t *setPositionControlCmd(int actPosJoint, int targPosJoint, int j){
    static uint8_t cmd[5];
    static int targIncJoint;
    targIncJoint = targPosJoint - actPosJoint;
    cmd[0] = 0xe0 + j;
    cmd[1] = 0xfd;
    cmd[2] = (targIncJoint>=0) ? 0x80 : 0x00;
    cmd[3] = abs(targIncJoint) / 256;
    cmd[4] = abs(targIncJoint) % 256;
    return cmd;
}

uint8_t *setJogControlCmd(char cmdRes[]){
    static uint8_t cmd[5];
    cmd[0] = cmdRes[1] + 0xe0;
    cmd[1] = 0xfd;
    cmd[2] = (cmdRes[2] == 0) ? 0x00 : 0x80;
    cmd[3] = 0x01;
    cmd[4] = 0x80;
    
    return cmd;
}

//int *deg2pulse(double theta[]){
//    // double actualPosition[3];
//    static int pulse[DOF];
//    for (int i = 0; i < DOF; i++){
//        pulse[i] = (int)(theta[i] * directionFlag[i] * DIV_NUM_DEG) + zeroPos[i];
//    }
//    return pulse;
//}

int deg2pulseSingleJoint(double theta, int i){
    // double actualPosition[3];
    return (int)(theta * directionFlag[i] * DIV_NUM_DEG) + zeroPos[i];
}

double pulse2radSingleJoint(int pulse, int i){
    // double theta[3];
    double theta;
    return (double)((pulse - zeroPos[i]) * directionFlag[i]) / DIV_NUM_RAD ;
    // return theta;    
}

double *getInverseKinematics(double X, double Y){
    static double theta[3];
    const double L[DOF] = {50, 130, 168};
    double a, b, c, t0;
    double t[2];
    double theta1Tmp[2], theta2Tmp[2];
    int setValueFlag = 0;
    int i;
    a = X * X + Y * Y + L[2] * L[2] - L[1] * L[1] - 2 * L[2] * Y;
    b = -4 * X * L[2];
    c = X * X + Y * Y + L[2] * L[2] - L[1] * L[1] + 2 * L[2] * Y;

    if(a == 0){
        t0 = -c / b;
        theta[1] = atan2(t0, 1) * 2;
        theta[2] = atan2((Y + L[2] * cos(theta[1])) / (X - L[2] * sin(theta[1])), 1);
    }
    else{
        t[0] = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
        t[1] = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
        for (i = 0; i < 2; i++){
          theta1Tmp[i] = atan2(t[i], 1) * 2;
          theta2Tmp[i] = atan2((Y + L[2] * cos(theta1Tmp[i])) / (X - L[2] * sin(theta1Tmp[i])), 1);
        }

        for(i = 0; i < 2; i++){
          if (theta1Tmp[i] >= -PI/2 && theta1Tmp[i] <= PI/2 && theta2Tmp[i] >= 0 && theta2Tmp[i] <= PI/2){
              theta[1] = theta1Tmp[i];
              theta[2] = theta2Tmp[i];
              setValueFlag = 1;
          }
        }
        if (setValueFlag == 0){
            for(i = 1; i < DOF; i++){
                theta[i] = pulse2radSingleJoint(actualPosition[i], i);
            }
        }
    }
    return theta;
}


/* USER CODE END 1 */
