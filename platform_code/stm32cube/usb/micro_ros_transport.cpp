#include "stm32u5xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>

extern "C" {
#ifdef MICRO_ROS_TRANSPORT_USB_CDC 


//Initialization Handler 
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;


// USB CDC Transmission complete callback 
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
// USB CDC Receive function (rewriting this function to copy received data to rxBuffer)
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len);
// USB CDC Control function 
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);

// Line coding: Rate: 115200bps; CharFormat: 1 Stop bit; Parity: None; Data: 8 bits
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

#define USB_BUFFER_SIZE 2048
#define WRITE_TIMEOUT_MS 100U

// Static buffer to hold received data
volatile uint8_t rxBuffer[USB_BUFFER_SIZE] = {0};
//Index where data is read from 
volatile size_t headIndex = 0 ; 
//Index where new data is written to 
volatile size_t tailIndex = 0 ;
//Transmission complete indicator 
volatile bool USBCDC_Transmit_Cplt = false;
//USb initialization indicator 
bool initialized = false ; 


static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  switch(cmd)
  {
    case CDC_SET_LINE_CODING:
      memcpy(line_coding, pbuf, sizeof(line_coding));
      break;

    case CDC_GET_LINE_CODING:
      memcpy(pbuf, line_coding, sizeof(line_coding));
      break;

    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
    case CDC_SET_CONTROL_LINE_STATE:
    case CDC_SEND_BREAK:
    default:
      break;
  }

return USBD_OK;
}


static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  (void) Buf ; 
  (void) Len ; 
  (void) epnum ; 

  USBCDC_Transmit_Cplt = true ; 
  return USBD_OK ;   
}
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  //Set where to store incoming data 
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  // Circular Buffer Management 
  // managing if incoming data length is higher than USB buffer Size 
  if((tailIndex + * Len) > USB_BUFFER_SIZE)
  {
    size_t firstSecionSize = USB_BUFFER_SIZE - tailIndex ; 
    size_t secondSectionSize = *Len - firstSecionSize ;
    // copy first section in the receive buffer  
    memcpy((void*)&rxBuffer[tailIndex],Buf,firstSecionSize);
    // append second section to receive buffer 
    memcpy((void*)&rxBuffer[0],Buf,secondSectionSize);
    // where next data will be stored 
    tailIndex = secondSectionSize ; 
  }
  else 
  {
    // copy data to the receive buffer 
    memcpy((void*)&rxBuffer[tailIndex],Buf,*Len);
    // where next data will be stored 
    tailIndex += *Len ; 
  }
  USBD_CDC_ReceivePacket(&hUsbDeviceFS); 
  return USBD_OK ; 

}

bool platformio_transport_open(struct uxrCustomTransport * transport)
{
  if(!initialized)
  {
    USBD_Interface_fops_FS.TransmitCplt = CDC_TransmitCplt_FS;
    USBD_Interface_fops_FS.Control = CDC_Control_FS;
    USBD_Interface_fops_FS.Receive = CDC_Receive_FS;
    initialized = true;    
  }
  return true;
}

bool platformio_transport_close(struct uxrCustomTransport * transport)
{
  return true;
}

size_t platformio_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode)
{
  //Transmit data
  uint8_t USBstatus = CDC_Transmit_FS((uint8_t *) buf , len);
  //Transmission error 
  if(USBstatus != USBD_OK )
  {
    return 0 ;  
  }
  //Start time of transmission 
  int64_t start = uxr_millis();
  // wiat while transmission is not completed and write timeout is not exceeded 
  while(!USBCDC_Transmit_Cplt && (uxr_millis()-start) < WRITE_TIMEOUT_MS)
  {
    HAL_Delay(1);  
  }
  //if transmission completed return length else return 0 
  size_t writed = USBCDC_Transmit_Cplt ? len : 0 ; 
  USBCDC_Transmit_Cplt = false ; 
  return writed ; 


}

size_t platformio_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
  //Start time 
  int64_t start = uxr_millis();
  size_t readed = 0 ; 
  // do while the timeout  is not exceeded 
  do
  { // check if there is data avialable in the buffer 
    if ( headIndex!=tailIndex )
    {
      // copy data until the buffer is empty or copied the requested length 
      while(headIndex!=tailIndex && readed < len)
      { 
        // copies received data in buf 
        buf[readed]=rxBuffer[headIndex] ; 
        // increment head index making sure it does not exceed the USB buffer size 
        headIndex = (headIndex+1) % USB_BUFFER_SIZE ; 
        // increment readed bytes 
        readed++;
      }

      break ; 
    }  
    
    HAL_Delay(1);
    
  } while ((uxr_millis()-start)<timeout);
  return readed ; 
 
}
#endif

}
