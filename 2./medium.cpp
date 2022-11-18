#include "main.h"                  

#define BOOT_FLAG_ADDRESS           0x08004000U
#define APP_ADDRESS                 0x08008000U
#define TIMEOUT_VALUE               SystemCoreClock/4

#define ACK     0x06U
#define NACK    0x16U

static UART_HandleTypeDef huart;
static uint8_t RX_Buffer[32];
typedef enum
{
    ERASE = 0x43,
    WRITE = 0x31,
    CHECK = 0x51,
    JUMP  = 0xA1,
} COMMANDS;

static void Jump2App(void);
static void Boot_Init(void);
static void Transmit_ACK(UART_HandleTypeDef *huart);
static void Transmit_NACK(UART_HandleTypeDef *huart);
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len);
static void Erase(void);
static void Write(void);
static void Check(void);

int main(void)
{
    Clk_Update();
    Boot_Init();//this function activates the uart interface
    
    Transmit_ACK(&huart);//sends ack signal
    //waiting for 2 byte data If it does not receive data during this timeout, it sends a nack signal and goes to the application.
//If it receives data and the checksum result of the data it receives is not 1 and the first byte is not ack, it sends a nack signal and goes to the application.    if(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT)
    {
        Transmit_NACK(&huart);
        Jump2App();
    }
    if(Check_Checksum(RX_Buffer, 2) != 1 || RX_Buffer[0] != ACK)
    {
        Transmit_NACK(&huart);
        Jump2App();
    }
  //It listens in a way that it constantly receives data from the uart. if the received data checksum the received 2 bytes of data . 
  //If the result of the checksum operation is negative, it sends a nack signal, if it is positive, it performs deletion, write, check jump and operations according to the 0th byte of the data it receives.
	for(;;)
	{
        while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
        
        if(Check_Checksum(RX_Buffer, 2) != 1)
        {
            Transmit_NACK(&huart);
        }
        else
        {
            switch(RX_Buffer[0])
            {
                case ERASE:
                    Transmit_ACK(&huart);
                    Erase();
                    break;
                case WRITE:
                    Transmit_ACK(&huart);
                    Write();
                    break;8
                case CHECK:
                    Transmit_ACK(&huart);
                    Check();
                    break;
                case JUMP:
                    Transmit_ACK(&huart);
                    Jump2App();
                    break;
                default: 
                    Transmit_NACK(&huart);
                    break;
            }
        }
	}
    
    for(;;);
	return 0;
}
//this function executes the function at app_address.
static void Jump2App(void)
{
    if (((*(__IO uint32_t*)APP_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
    {
        __disable_irq();
        uint32_t jump_address = *(__IO uint32_t *)(APP_ADDRESS + 4);
        __set_MSP(*(__IO uint32_t *)APP_ADDRESS);
        void (*pmain_app)(void) = (void (*)(void))(jump_address);
        pmain_app();
    }
    
}

/*this function activates the usart2 interface*/
static void Boot_Init(void)
{
    GPIO_InitTypeDef gpio_uart;
    
    gpio_uart.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio_uart.Mode = GPIO_MODE_AF_PP;
    gpio_uart.Pull = GPIO_PULL_NONE;
    gpio_uart.Speed = GPIO_SPEED_LOW;
    gpio_uart.Alternate = GPIO_AF7_USART2;
    
    HAL_RCC_GPIOA_CLK_ENABLE();
    HAL_GPIO_Init(GPIOA, &gpio_uart);
    
    huart.Init.BaudRate = 115200;
    huart.Init.Mode = HAL_UART_MODE_TX_RX;
    huart.Init.OverSampling = HAL_UART_OVERSAMPLING_16;
    huart.Init.Parity = HAL_UART_PARITY_NONE;
    huart.Init.StopBits = HAL_UART_STOP_1;
    huart.Init.WordLength = HAL_UART_WORD8;
    huart.Instance = USART2;
    
    HAL_RCC_USART2_CLK_ENABLE();
    HAL_UART_Init(&huart);
}

/*this function sends 2 bytes of data with ack value from uart*/
static void Transmit_ACK(UART_HandleTypeDef *handle)
{
    uint8_t msg[2] = {ACK, ACK};
    
    HAL_UART_Tx(handle, msg, 2);
}

/*this function sends 2 bytes of data with nack value from uart*/
static void Transmit_NACK(UART_HandleTypeDef *handle)
{
    uint8_t msg[2] = {NACK, NACK};
    
    HAL_UART_Tx(handle, msg, 2);
}

/*this function xorifies all data from the previous data, returns 1 if the result is zero, otherwise returns 0.*/
static uint8_t Check_Checksum(uint8_t *pBuffer, uint32_t len)
{
    uint8_t initial = 0xFF;
    uint8_t result = 0x7F; 
    
    result = initial ^ *pBuffer++;
    len--;
    while(len--)
    {
        result ^= *pBuffer++;
    }
    
    result ^= 0xFF;
    
    if(result == 0x00)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
/*This function listens to the uart to get 3 bytes of data, passes the data it receives through checksum, if the result is negative, 
it sends a nack signal and exits the function. If the first byte of the data it receives is 0xFF, it sends a nack signal and exits the function, 
then deletes the sector number indicated by the zeroth byte and the sector indicated by the first byte.*/
static void Erase(void)
{
    Flash_EraseInitTypeDef flashEraseConfig;
    uint32_t sectorError;
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 3, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    if(Check_Checksum(RX_Buffer, 3) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    
    if(RX_Buffer[0] == 0xFF)
    {
        Transmit_NACK(&huart);
    }
    else
    {

        flashEraseConfig.TypeErase = HAL_FLASH_TYPEERASE_SECTOR;
        flashEraseConfig.NbSectors = RX_Buffer[0];
        flashEraseConfig.Sector = RX_Buffer[1];

        HAL_Flash_Unlock();
        HAL_Flash_Erase(&flashEraseConfig, &sectorError);
        HAL_Flash_Lock();
        
        Transmit_ACK(&huart);
    }
}
/*This function listens to the uart interface, puts the incoming data into the checksum control, if the result is negative, it sends a nack signal and exits the function.
 if it is positive it sends ack signal. After that, uart listens to take 2 bytes and looks at the first byte of the incoming data and finds the total data length in bytes. 
 It listens to the uart channel again until it receives data up to the bytes of the length it finds, passes the data it receives through checksum, if the result is negative, 
 it exits the function, if the result is positive, it prints the data to the flash.*/
static void Write(void)
{
    uint8_t numBytes;
    uint32_t startingAddress = 0;
    uint8_t i;

    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }

    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 2, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    numBytes = RX_Buffer[0];
    
    while(HAL_UART_Rx(&huart, RX_Buffer, numBytes+1, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    
    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }

    i = 0;
    HAL_Flash_Unlock();
    while(numBytes--)
    {
        HAL_Flash_Program(FLASH_TYPEPROGRAM_BYTE, startingAddress, RX_Buffer[i]);
        startingAddress++;
        i++; 
    }
    HAL_Flash_Lock();
    Transmit_ACK(&huart);
}
/*If the checksum result of the incoming uart data is not suitable, it sends a nack signal and exits the function. 
If the incoming function checksum result is appropriate, it sends an ack signal. and checks according to the crc algorithm.
 if the check result is positive it sends ack signal, if negative it sends nack signal and jamp2app executes the function.*/
static void Check(void)
{
    uint32_t startingAddress = 0;
    uint32_t endingAddress = 0;
    uint32_t address;
    uint32_t *data;
    uint32_t crcResult;
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);
    
    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }
    
    startingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);
    
    while(HAL_UART_Rx(&huart, RX_Buffer, 5, TIMEOUT_VALUE) == HAL_UART_TIMEOUT);

    if(Check_Checksum(RX_Buffer, 5) != 1)
    {
        Transmit_NACK(&huart);
        return;
    }
    else
    {
        Transmit_ACK(&huart);
    }
    
    endingAddress = RX_Buffer[0] + (RX_Buffer[1] << 8) 
                    + (RX_Buffer[2] << 16) + (RX_Buffer[3] << 24);
    
    HAL_RCC_CRC_CLK_ENABLE();
    data = (uint32_t *)((__IO uint32_t*) startingAddress);
    for(address = startingAddress; address < endingAddress; address += 4)
    {
        data = (uint32_t *)((__IO uint32_t*) address);
        crcResult = HAL_CRC_Accumulate(data, 1);
    }
    
    HAL_RCC_CRC_CLK_DISABLE();
    if(crcResult == 0x00)
    {
        Transmit_ACK(&huart);
    }
    else
    {
        Transmit_NACK(&huart);
    }
    
    Jump2App();
}
