#include <hardware/flash.h>
#include <tusb.h>

#define DESC_STR_MAX 31

#define USBD_VID 0x2E8A /* Raspberry Pi */
#define USBD_PID 0x000A /* Raspberry Pi Pico SDK CDC */

#define USBD_DESC_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN * CFG_TUD_CDC)
#define USBD_MAX_POWER_MA 500

enum {
  ITF_NUM_CDC_COMM = 0,
  ITF_NUM_CDC_DATA,
  ITF_NUM_TOTAL
};

#define USBD_CDC_EP_CMD 0x81

#define USBD_CDC_EP_OUT 0x01

#define USBD_CDC_EP_IN 0x82

#define USBD_CDC_CMD_MAX_SIZE 8
#define USBD_CDC_IN_OUT_MAX_SIZE 64

#define USBD_STR_0 0x00
#define USBD_STR_MANUF 0x01
#define USBD_STR_PRODUCT 0x02
#define USBD_STR_SERIAL 0x03
#define USBD_STR_SERIAL_LEN 17
#define USBD_STR_CDC 0x04

static const tusb_desc_device_t usbd_desc_device = {
	.bLength = sizeof(tusb_desc_device_t),
	.bDescriptorType = TUSB_DESC_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = TUSB_CLASS_MISC,
	.bDeviceSubClass = MISC_SUBCLASS_COMMON,
	.bDeviceProtocol = MISC_PROTOCOL_IAD,
	.bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
	.idVendor = USBD_VID,
	.idProduct = USBD_PID,
	.bcdDevice = 0x0100,
	.iManufacturer = USBD_STR_MANUF,
	.iProduct = USBD_STR_PRODUCT,
	.iSerialNumber = USBD_STR_SERIAL,
	.bNumConfigurations = 1,
};

static const uint8_t usbd_desc_cfg[USBD_DESC_LEN] = {
	TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, USBD_STR_0, USBD_DESC_LEN,
		TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, USBD_MAX_POWER_MA),

	TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_COMM, USBD_STR_CDC, USBD_CDC_EP_CMD,
		USBD_CDC_CMD_MAX_SIZE, USBD_CDC_EP_OUT, USBD_CDC_EP_IN,
		USBD_CDC_IN_OUT_MAX_SIZE),

};

static char usbd_serial[USBD_STR_SERIAL_LEN];

static const char *const usbd_desc_str[] = {
	[USBD_STR_MANUF] = "KNfLrPn",
	[USBD_STR_PRODUCT] = "2wiCC COM",
	[USBD_STR_SERIAL] = usbd_serial,
	[USBD_STR_CDC] = "2wiCC COM",
};

const uint8_t *tud_descriptor_device_cb(void)
{
	return (const uint8_t *) &usbd_desc_device;
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
	return usbd_desc_cfg;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
	static uint16_t desc_str[DESC_STR_MAX];
	uint8_t len;

	if (index == 0) {
		desc_str[1] = 0x0409;
		len = 1;
	} else {
		const char *str;
		char serial[USBD_STR_SERIAL_LEN];

		if (index >= sizeof(usbd_desc_str) / sizeof(usbd_desc_str[0]))
			return NULL;

		str = usbd_desc_str[index];
		len = strlen(str);
		if (len > 31)
			len = 31;

		for (uint8_t i = 0; i < len; i++)
		{
			desc_str[1 + i] = str[i];
		}

	}

	desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * len + 2);

	return desc_str;
}

void usbd_serial_init(void)
{
    static const char hexchars[] = "0123456789ABCDEF";
    uint8_t id[8];

    flash_get_unique_id(id);

    /* USBD_STR_SERIAL_LEN must be at least 17 to hold 16 hex chars + NUL */
    for (uint8_t i = 0; i < 8; ++i) {
        usbd_serial[2*i + 0] = hexchars[id[i] >> 4];
        usbd_serial[2*i + 1] = hexchars[id[i] & 0x0F];
    }
    usbd_serial[16] = '\0';
}
