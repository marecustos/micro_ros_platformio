#include "stm32u5xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

static inline void set_microros_usb_transports(PCD_HandleTypeDef *handle){
	rmw_uros_set_custom_transport(
		true,
		handle,
		platformio_transport_open,
		platformio_transport_close,
		platformio_transport_write,
		platformio_transport_read
	);
}