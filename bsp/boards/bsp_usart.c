#include "bsp_usart.h"
#include "main.h"
#include "autoaim.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;  
extern DMA_HandleTypeDef hdma_usart1_rx;  //自己配置
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

extern char Sendtosight[Sendtosight_len];

//extern uint8_t ChariotRecognition_data[2][ChariotRecognition_data_dma_buf_len];


/*串口1DMA发送初始化*/
void usart1_tx_dma_init(void)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }
      
      hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
		
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;
		
//		  hdma_usart1_tx.Instance->M0AR = (uint32_t)(&Sendtosight[0]);  //发送给视觉的数据
//      hdma_usart1_tx.Instance->NDTR = Sendtosight_len;
		
//		  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//      __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);
//		  __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, sizeof(Sendtosight));
//		
//	  SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);  
		
//		  __HAL_DMA_ENABLE(&hdma_usart1_tx);
		
}

/*串口1DMA接收初始化*/
void usart1_rx_dma_init(void)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
		 __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);   //串口1空闲中断使能，少了这句不能进串口1的中断服务函数
//		hdma_usart1_rx.Instance->M0AR =  (uint32_t)(NULL);
	  __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_HISR_TCIF5);
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(&ChariotRecognition_data[0][0]);   //自瞄接收数据数组地址1   双缓冲区
		hdma_usart1_rx.Instance->M1AR = (uint32_t)(&ChariotRecognition_data[1][0]);		//自瞄接收数据数组地址2
    hdma_usart1_rx.Instance->NDTR = (uint16_t)ChariotRecognition_data_dma_buf_len;  //接收数据长度
	  //__HAL_DMA_SET_COUNTER(&hdma_usart1_rx, sizeof(ChariotRecognition_data)/2);    //数组长度，二维数组，取一半长度

    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);  //使能DMA双缓冲区
	  __HAL_DMA_ENABLE(&hdma_usart1_rx);
		
}

void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void usart1_rx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);

    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

//    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx, DMA_HISR_TCIF7);

    hdma_usart1_rx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_rx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}

void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);



    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);


    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}


