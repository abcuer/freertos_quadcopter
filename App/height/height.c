#include "headfile.h"

// 定义缓冲区大小，建议为一包数据的几倍，比如 64 字节
StreamBufferHandle_t xFlowStreamBuffer = NULL;
const size_t xStreamBufferSizeBytes = 64; 
const size_t xTriggerLevel = 1; // 只要有 1 个字节进入就唤醒任务
volatile uint8_t rx_data = 0;

void Flow_Buffer_Init(void) 
{
    xFlowStreamBuffer = xStreamBufferCreate(xStreamBufferSizeBytes, xTriggerLevel);

    if (xFlowStreamBuffer != NULL) {
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // 将接收到的 1 字节丢入流缓冲区
        if (xFlowStreamBuffer != NULL) {
            xStreamBufferSendFromISR(xFlowStreamBuffer, &rx_data, 1, &xHigherPriorityTaskWoken);
        }

        // 继续开启中断接收
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);

        // 如果有高优先级任务就绪，进行上下文切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
// 光流数据解析
void StartFlowTask(void const * argument)
{
    uint8_t recv_data;
    for(;;)
    {
        // 阻塞等待缓冲区数据，portMAX_DELAY 表示无限等待
        size_t xReceivedBytes = xStreamBufferReceive(xFlowStreamBuffer, 
                                                     &recv_data, 
                                                     1, 
                                                     portMAX_DELAY);

        if(xReceivedBytes > 0)
        {
            // 浮点运算和解析
            Flow_Parse_Data(recv_data); 
        }
    }
}