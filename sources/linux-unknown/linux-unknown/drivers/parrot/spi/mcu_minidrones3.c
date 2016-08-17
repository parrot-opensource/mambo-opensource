#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/firmware.h>
#include <linux/limits.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/platform_device.h>

#include <mach/gpio.h>
#include <mach/gpio_parrot.h>

#include "mcu_minidrones3.h"

#define DRIVER_NAME "mcu_minidrones3"
#define DRIVER_VERSION 0x1

struct mcu_minidrones3_data {
	struct p6_spi_config *client;
	struct input_dev  *input;
	struct spi_device *spi;
	struct mutex iomutex;
	unsigned long spi_read_delay_ms;
	char *firmware_name;
	int gpio_rst;
	int needs_resync;
	int keys_nb;
	struct mcu_minidrones3_key *keys;
};

#define FLAG_MORE_DATA  0x01

typedef enum cmd {
	CMD_NO_DATA = 0,
	CMD_VERSION = 1,
	CMD_READ_GPIO = 2,
	CMD_CLEAR_IT = 3,
	CMD_POWER_OFF = 4,
	CMD_READ_VBAT = 5,
	CMD_US_HEATER_PWM = 6,
	CMD_BARO_HEATER_PWM = 7,
} eCmd;

#define SPI_BYTE_DELAY_US (20)
#define SYNC_PREAMBLE_LENGTH (4)
#define SYNC_RETRIES (10)
#define BUSY_RETRIES (10)
#define MAX_READ_GPIO_LOOPS (32)

/*
 * Serial Programming Instruction Set
 * Instruction/Operation
 * Instruction Format
 *                                  Byte 1       Byte 2       Byte 3       Byte4
 * Programming Enable                 $AC          $53          $00         $00
 * Chip Erase (Program Memory/EEPROM) $AC          $80          $00         $00
 * Poll RDY/BSY                       $F0          $00          $00     data out

 * Load Instructions
 * Extended Address byte              $4D          $00 Extended adr         $00
 * Load Program Memory Page H byte    $48      adr_MSB      adr_LSB     Hdata in
 * Load Program Memory Page, L byte   $40      adr_MSB      adr_LSB     Ldata in
 * Load EEPROM Memory Page (access)   $C1          $00   0000 000aa      data in

 * Read Instructions
 * Read Program Memory, High byte     $28      adr_MSB      adr_LSB    Hdata out
 * Read Program Memory, Low byte      $20      adr_MSB      adr_LSB    Ldata out
 * Read EEPROM Memory                 $A0          $00    00aa aaaa     data out
 * Read Lock bits                     $58          $00          $00     data out
 * Read Signature Byte                $30          $00   0000 000aa     data out
 * Read Fuse bits                     $50          $00          $00     data out
 * Read Fuse High bits                $58          $08          $00     data out
 * Read Extended Fuse Bits            $50          $08          $00     data out
 * Read Calibration Byte              $38          $00          $00     data out

 * Write Instructions
 * Write Program Memory Page          $4C      adr_MSB      adr_LSB          $00
 * Write EEPROM Memory                $C0          $00    00aa aaaa      data in
 * Write EEPROM Memory Page (access)  $C2          $00    00aa aa00          $00
 * Write Lock bits                    $AC          $E0          $00      data in
 * Write Fuse bits                    $AC          $A0          $00      data in
 * Write Fuse High bits               $AC          $A8          $00      data in
 * Write Extended Fuse Bits           $AC          $A4          $00      data in
 */

#define CMD_WRITE_PROG_MEM 0b01001100
#define CMD_READ_PROG_MEM  0b00100000
#define CMD_PROG_EN_0      0b10101100
#define CMD_PROG_EN_1      0b01010011
#define CMD_POLL_RDY_BSY   0b11110000
#define CMD_LOAD_PROG_MEM  0b01000000
#define CMD_CHIP_ERASE_0   0b10101100
#define CMD_CHIP_ERASE_1   0b10000000
#define CMD_READ_FUSE_LOW_BITS_0 0b01010000
#define CMD_WRITE_FUSE_LOW_BITS_0 0b10101100
#define CMD_WRITE_FUSE_LOW_BITS_1 0b10100000
#define CMD_READ_SIGNATURE 0b00110000

#define LOW_BYTE           0b00000000
#define HIGH_BYTE          0b00001000

#define FUSE_LOW_BITS_DEFAULT 0x62

/* Fuse low byte value: Run at full 8 MHz */
#define FUSE_LOW_BITS_VALUE 0xe2

static void mcu_reset_and_resume(struct mcu_minidrones3_data *drv_data);

static int send_bytes(struct mcu_minidrones3_data *drv_data, unsigned char *cmd,
		      unsigned char *data_buf, int data_buf_size, int usedelay)
{
	int ret = 0;
	int i;
	struct spi_message msg;
	for (i = 0; i < data_buf_size; i ++) {
		uint8_t tx = cmd[i];
		uint8_t rx = 0;
		struct spi_transfer xfer = {
			.tx_buf = (void *)&tx,
			.rx_buf = (void *)&rx,
			.len = 1,
			.delay_usecs = 0,
			.bits_per_word = 8,
			.cs_change = 0,
		};
		spi_message_init(&msg);
		spi_message_add_tail(&xfer, &msg);
		ret = spi_sync(drv_data->spi, &msg);
		if (ret != 0)
			dev_warn(&drv_data->spi->dev, "Failed\n");
		if (data_buf)
			data_buf[i] = rx;
		dev_dbg(&drv_data->spi->dev,
			"Tx[%d]=0x%02X\tRx[%d]=0x%02X\n",
			i, cmd[i], i, rx);
		if (usedelay)
			udelay(SPI_BYTE_DELAY_US);
	}
	return ret;
}

static int mcu_send_query(struct mcu_minidrones3_data *drv_data, enum cmd cmd,
		unsigned char param1, unsigned char param2)
{
	unsigned char query[4] = { (unsigned char)cmd, param1, param2, 0x84 };
	return send_bytes(drv_data, query, NULL, sizeof(query), 1);
}

/* Read response from MCU into resp (a four bytes buffer).
 * Buffer is filled if it is not NULL and the function returns 0.
 * In case of error, the buffer is left untouched and an error code is
 * returned.
 */
static int mcu_read_response(struct mcu_minidrones3_data *drv_data,
		unsigned char* resp)
{
	unsigned char dummy[4] = { 0, 0, 0, 0 };
	unsigned char myresp[4];
	int res = send_bytes(drv_data, dummy, myresp, sizeof(dummy), 1);
	if (res)
		return res;
	/* We check that the 2nd byte isn't 0, it can happen if the MCU went
	 * back to unsynced state without us noticing. In this case, it is
	 * repeating our bytes, producing invalid responses starting with
	 * byte 0x84, so we check that command byte isn't zero.
	 */
	if (myresp[0] == 0x84 && myresp[1] != 0x00) {
		if (resp)
			memcpy(resp, myresp, sizeof(myresp));
		return 0;
	}
	if (myresp[0] == 0x00) {
		dev_warn(&drv_data->spi->dev,
			"MCU returned invalid request: %.2x %.2x %.2x\n",
			myresp[1], myresp[2], myresp[3]);
		return -EINVAL;
	}
	if (myresp[0] == 0x7b)
		return -EAGAIN;
	return -ECOMM;
}

/* Perform a MCU query (send query and read response).
 * resp is a 4 bytes buffer which is filled if it is not NULL and the
 * function returns 0 or -EINVAL.
 * If the MCU is busy, retry BUSY_RETRIES times.
 */
static int mcu_perform(struct mcu_minidrones3_data *drv_data,
		enum cmd cmd, unsigned char param1, unsigned char  param2,
		unsigned char *resp)
{
	unsigned char myresp[4];
	int res;
	int retries = 0;

	res = mcu_send_query(drv_data, cmd, param1, param2);
	if (res)
		return res;
	while (retries < BUSY_RETRIES) {
		res = mcu_read_response(drv_data, myresp);
		retries ++;
		if (res == -EAGAIN)
			continue;
		if (res == 0) {
			if (resp)
				memcpy(resp, myresp, sizeof(myresp));
		} else {
			/* Force resync */
			drv_data->needs_resync = 1;
		}
		return res;
	}
	return -ETIMEDOUT;
}

static int mcu_sync_send_preamble(struct mcu_minidrones3_data *drv_data)
{
	unsigned char sync_preamble[4] = {
		0xff,
		0xff,
		0xff,
		0xff
	};
	unsigned char feedback[4];
	int i;
	for (i = 0; i < SYNC_PREAMBLE_LENGTH; i ++) {
		int res = send_bytes(drv_data, sync_preamble, feedback,
				sizeof(sync_preamble), 1);
		if (res)
			return res;
	}
	return 0;
}

static int mcu_sync_single(struct mcu_minidrones3_data *drv_data)
{
	int res;
	unsigned char dummy[4] = { 0, 0, 0, 0 };
	unsigned char sync_query[4] = {
		0x00, 0x00, 0x00, 0x80
	};
	unsigned char sync_resp[4];
	/* Send sync preamble */
	res = mcu_sync_send_preamble(drv_data);
	if (res)
		return res;
	/* Send sync query */
	res = send_bytes(drv_data, sync_query, sync_resp,
			sizeof(sync_query), 1);
	if (res)
		return res;
	/* Read ack */
	res = send_bytes(drv_data, dummy, sync_resp,
			sizeof(sync_resp), 1);
	if (res)
		return res;
	/* Check sync response. */
	if (sync_resp[0] == 0x55 && sync_resp[1] == 0xaa &&
		sync_resp[2] == 0x55 && sync_resp[3] == 0xaa) {
		return 0;
	}
	return -ECOMM;
}

static int mcu_sync(struct mcu_minidrones3_data *drv_data)
{
	int res;
	int retries = 0;
	while (retries < SYNC_RETRIES) {
		res = mcu_sync_single(drv_data);
		if (res == 0)
			break;
		retries ++;
	}
	if (res) {
		dev_err(&drv_data->spi->dev, "Sync failed after %i "
				"retries! Resetting...\n", retries);
		mcu_reset_and_resume(drv_data);
		return -ECONNRESET;
	}
	drv_data->needs_resync = 0;
	return 0;
}

static int mcu_sync_and_perform(struct mcu_minidrones3_data *drv_data,
		enum cmd cmd, unsigned char param1, unsigned char param2,
		unsigned char *resp)
{
	int res;
	if (drv_data->needs_resync) {
		res = mcu_sync(drv_data);
		if (res)
			return res;
	}
	res = mcu_perform(drv_data, cmd, param1, param2, resp);
	if (res == -ETIMEDOUT)
		drv_data->needs_resync = 1;
	return res;
}

static void get_messages(struct mcu_minidrones3_data *drv_data)
{
	int loopcount = 0;
	int res;
	unsigned char flags;

	/* Process messages */
	do {
		int i;
		unsigned char resp[4];
		unsigned char gpio_data;
		res = mcu_sync_and_perform(drv_data,
				CMD_READ_GPIO, 0, 0, resp);
		if (res) {
			dev_err(&drv_data->spi->dev, "GPIO read failed: %d\n",
					res);
			goto failsafe;
		}
		gpio_data = resp[2];
		flags = resp[3];

		for (i = 0; i < drv_data->keys_nb; i++) {
			dev_info(&drv_data->spi->dev, "Report key %d :  %d\n",
				 drv_data->keys[i].input_key,
				 !!(gpio_data & drv_data->keys[i].bmask));
			input_report_key(drv_data->input,
					 drv_data->keys[i].input_key,
					 !!(gpio_data & drv_data->keys[i].bmask));
		}
		input_sync(drv_data->input);
		loopcount ++;
		if (loopcount >= MAX_READ_GPIO_LOOPS) {
			dev_err(&drv_data->spi->dev,
					"Too many GPIO read iterations! "
					"Resetting...\n");
			mcu_reset_and_resume(drv_data);
			break;
		}
	} while (flags & FLAG_MORE_DATA);
	return;
failsafe:
	res = mcu_sync_and_perform(drv_data, CMD_CLEAR_IT, 0, 0, NULL);
	if (res) {
		dev_err(&drv_data->spi->dev, "Failed to clear IT: %d!\n", res);
	}
}

static int get_version(struct mcu_minidrones3_data *drv_data)
{
	unsigned char resp[4];
	int res;

	res = mcu_sync_and_perform(drv_data, CMD_VERSION, 0, 0, resp);
	if (res)
		return res;
	return resp[2];
}

static int get_vbat(struct mcu_minidrones3_data *drv_data)
{
	unsigned char resp[4];
	int res;

	res = mcu_sync_and_perform(drv_data, CMD_READ_VBAT, 0, 0, resp);
	if (res)
		return res;
	res = resp[3];
	res <<= 8;
	res |= resp[2];
	return res;
}

static int set_us_heater_pwm(struct mcu_minidrones3_data *drv_data,
		uint8_t pwm_ratio)
{
	return mcu_sync_and_perform(drv_data, CMD_US_HEATER_PWM, pwm_ratio, 0,
			NULL);
}

static int set_baro_heater_pwm(struct mcu_minidrones3_data *drv_data,
		uint8_t pwm_ratio)
{
	return mcu_sync_and_perform(drv_data, CMD_BARO_HEATER_PWM, pwm_ratio,
			0, NULL);
}

static char power_off(struct mcu_minidrones3_data *drv_data)
{
	return mcu_sync_and_perform(drv_data, CMD_POWER_OFF, 0, 0, NULL);
}


static unsigned char read_fw_byte(struct mcu_minidrones3_data *drv_data, int byte)
{
	int ret;
	char cmd[4] = { 0, 0, 0, 0 };
	char data_buf[4] = { 0, 0, 0, 0 };

	cmd[0] = CMD_READ_PROG_MEM | ((byte%2) ? HIGH_BYTE : LOW_BYTE);
	cmd[1] = (byte >> 9);
	cmd[2] = (byte >> 1) & 0xFF;
	cmd[3] = 0x00;
	ret = send_bytes(drv_data, cmd, data_buf, 4, 0);
	if (ret) {
		dev_warn(&drv_data->spi->dev, "Failed to get byte %d\n",
			 byte);
		return '\0';
	}
	return data_buf[3];
}

static unsigned int read_fuses_low(struct mcu_minidrones3_data *drv_data,
		uint8_t *value)
{
	int ret;
	char cmd[4] = { CMD_READ_FUSE_LOW_BITS_0, 0, 0, 0 };
	char data_buf[4] = { 0, 0, 0, 0 };

	ret = send_bytes(drv_data, cmd, data_buf, 4, 0);
	if (ret) {
		dev_warn(&drv_data->spi->dev,
			"Failed to get lower fuse bits: %d\n", ret);
		return ret;
	}
	*value = data_buf[3];
	return 0;
}

static unsigned int program_fuses_low(struct mcu_minidrones3_data *drv_data,
		uint8_t value)
{
	int ret;
	char cmd[4] = {
		CMD_WRITE_FUSE_LOW_BITS_0,
		CMD_WRITE_FUSE_LOW_BITS_1,
		0x00,
		value
	};

	dev_info(&drv_data->spi->dev, "Fw: program fuses low byte 0x%.2x\n",
		value);
	ret = send_bytes(drv_data, cmd, NULL, 4, 0);

	/* Wait for tWD_FUSE 4.5ms min */
	msleep(5);

	return ret;
}

static unsigned int program_page(struct mcu_minidrones3_data *drv_data,
				 int  page_address, int total_pages)
{
	int ret;
	char cmd[4] = { 0, 0, 0, 0 };
	char data_buf[4] = { 0, 0, 0, 0 };

	dev_info(&drv_data->spi->dev, "Fw: program page %d/%d\n",
		 page_address, total_pages);
	/* program the page */
	cmd[0] = CMD_WRITE_PROG_MEM;
	cmd[1] = (page_address >> 4);
	cmd[2] = (page_address & 0x0F) << 4;
	cmd[3] = 0x00;
	ret = send_bytes(drv_data, cmd, NULL, 4, 0);
	if (ret)
		return -1;

	/* Wait no more busy */
	cmd[0] = CMD_POLL_RDY_BSY;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	cmd[3] = 0x00;
	do {
		msleep(6); /* tWD_FLASH 4.5ms min */
		ret = send_bytes(drv_data, cmd, data_buf, 4, 0);
		if (ret)
			return -2;
	} while (data_buf[3] & 0x01);

	return 0;
}

static unsigned int write_fw_byte(struct mcu_minidrones3_data *drv_data, int byte,
				  unsigned char value)
{
	int ret;
	unsigned char cmd[4] = { 0, 0, 0, 0 };

	/* load Program memory page */
	cmd[0] = CMD_LOAD_PROG_MEM | ((byte%2) ? HIGH_BYTE : LOW_BYTE);
	cmd[1] = 0x00;
	cmd[2] = (byte >> 1) & 0x0F;
	cmd[3] = value;
	ret = send_bytes(drv_data, cmd, NULL, 4, 0);
	if (ret)
		return -1;

	return 0;
}

static int chip_erase(struct mcu_minidrones3_data *drv_data)
{
	int ret = 0;
	char cmd[4] = { CMD_CHIP_ERASE_0, CMD_CHIP_ERASE_1, 0, 0 };
	char data_buf[4] = { 0, 0, 0, 0 };
	ret = send_bytes(drv_data, cmd, NULL, 4, 0);
	if (ret)
		return ret;
	cmd[0] = CMD_POLL_RDY_BSY;
	cmd[1] = 0x00;
	do {
		msleep(6);
		ret = send_bytes(drv_data, cmd, data_buf, 4, 0);
		if (ret)
			return ret;
	} while (data_buf[3] & 0x01);
	return ret;
}

static void reset_chip(struct mcu_minidrones3_data *drv_data)
{
	/* "Power-up sequence:
	 * Apply power between Vcc and GND while /RESET and SCK are set to “0”.
	 * In some systems, the programmer can not guarantee that SCK is held
	 * low during powerup.
	 * In this case, /RESET must be given a positive pulse after SCK has
	 * been set to “0”. The pulse duration must be at least t_RST (miniumum
	 *  pulse width of /RESET pin, see Table 18-4 on page 120 and Figure
	 * 19-58 on page 153) plus two CPU clock cycles" */
	if (!gpio_is_valid(drv_data->gpio_rst)) {
		dev_info(&drv_data->spi->dev,
			 "Bad gpio %d\n", drv_data->gpio_rst);
		return;
	}
	dev_info(&drv_data->spi->dev, "Unreset MCU chip\n");
	gpio_set_value(drv_data->gpio_rst, 0);
	/* 2.5 µs min + 2 cpu clock */
	udelay(5);
	dev_info(&drv_data->spi->dev, "Reset MCU chip\n");
	gpio_set_value(drv_data->gpio_rst, 1);
	drv_data->needs_resync = 1;
}

static void mcu_reset_and_resume(struct mcu_minidrones3_data *drv_data)
{
	reset_chip(drv_data);

	/* Reset */
	dev_info(&drv_data->spi->dev, "Unreset MCU chip\n");
	gpio_set_value(drv_data->gpio_rst, 0);

	msleep(10);
}

static unsigned char prog_enable(struct mcu_minidrones3_data *drv_data)
{
	int ret = 0;
	int retries = 10;
	char cmd[4] = { CMD_PROG_EN_0, CMD_PROG_EN_1, 0, 0 };
	char data_buf[4] = { 0, 0, 0, 0 };
	/* send programming enable serial instruction
	 * "The serial programming instructions will not work if the
	 * communication is out of synchronization. When in sync. the second
	 * byte (0x53), will echo back when issuing the third byte of the
	 * Programming Enable instruction. Whether the echo is correct or not,
	 * all four bytes of the instruction must be transmitted. If the 0x53
	 * did not echo back, give RESET a positive pulse and issue a new
	 * Programming Enable command. */
	do {
		reset_chip(drv_data);
		/* "Wait for at least 20 ms and enable serial programming by
		 * sending the Programming Enable serial instruction to pin MOSI
		 * ->let's make it 30 */
		msleep(30);

		ret = send_bytes(drv_data, cmd, data_buf, 4, 0);
		if (ret)
			dev_warn(&drv_data->spi->dev,
				 "Failed to enable first serial instruction\n");
		else if (data_buf[2] == 0x53)
			break;
		dev_info(&drv_data->spi->dev,
			 "Resend programming enable serial instruction\n");
	} while (retries-- > 0);
	return (retries > 0) ? ret : -1;
}

static int send_firmware(struct mcu_minidrones3_data *drv_data,
			  u8 *fw_data, size_t fw_size)
{
	int ret = 0;
	unsigned char current_value;
	int bytes_diffs = 0;
	int byte;
	int written_bytes;
	int current_page;
	uint8_t fuses = FUSE_LOW_BITS_DEFAULT;

	ret = prog_enable(drv_data);
	if (ret) {
		dev_warn(&drv_data->spi->dev,
			 "Failed to initialize MCU chip\n");
		goto error;
	}

	dev_info(&drv_data->spi->dev,
		 "Check all bytes (please wait) ...\n");
	for (byte = 0; byte < fw_size && bytes_diffs == 0; byte++) {
		if ((byte+1)%32 == 0)
			dev_info(&drv_data->spi->dev, "Fw: checked page %d/%d\n",
				 (byte+1)/32-1, fw_size/32);
		current_value = read_fw_byte(drv_data, byte);
		if (current_value != fw_data[byte]) {
			dev_warn(&drv_data->spi->dev,
				 "Byte %d : %02X instead of %02X\n",
				 byte, current_value, fw_data[byte]);
			bytes_diffs++;
		}
	}

	dev_info(&drv_data->spi->dev, "Checking fuse bits...\n");
	if (read_fuses_low(drv_data, &fuses)) {
		dev_err(&drv_data->spi->dev,
			"Failed to read fuses. Aborting FW update.\n");
		ret = -EIO;
	} else if (fuses != FUSE_LOW_BITS_VALUE) {
		dev_warn(&drv_data->spi->dev,
			"Fuses byte is 0x%.2x instead of 0x%.2x.\n",
			fuses, FUSE_LOW_BITS_VALUE);
		bytes_diffs++;
	}

	if (bytes_diffs == 0) {
		dev_info(&drv_data->spi->dev, "Firmware is up to date\n");
		goto error;
	}
	dev_info(&drv_data->spi->dev, "Firmware is different\n");

	dev_info(&drv_data->spi->dev, "Chip erase\n");
	chip_erase(drv_data);

	dev_info(&drv_data->spi->dev, "Update firmware, size %d\n", fw_size);
	written_bytes = 0;
	current_page = 0;
	for (byte = 0; byte < fw_size; byte++) {
		ret = write_fw_byte(drv_data, byte, fw_data[byte]);
		if (ret) {
			dev_info(&drv_data->spi->dev, "Error writing byte %d\n",
				 byte);
			break;
		}
		written_bytes++;

		/* Check if it was last byte of page -> programm current */
		if (written_bytes == 32) {
			program_page(drv_data, current_page, fw_size/32);
			current_page++;
			written_bytes = 0;
		}
	}

	/* If odd size, add a 0 */
	if (written_bytes % 2 != 0) {
		write_fw_byte(drv_data, byte, '\0');
		written_bytes++;
	}

	/* Current page not finished ? */
	if (written_bytes > 0)
		program_page(drv_data, current_page, fw_size/32);

	/* Program fuse bits */
	ret = program_fuses_low(drv_data, FUSE_LOW_BITS_VALUE);
	if (ret) {
		dev_err(&drv_data->spi->dev,
				"Error programming fuse bits: %d\n", ret);
		ret = -EIO;
		goto error;
	}

error :
	/* Reset */
	dev_info(&drv_data->spi->dev, "Unreset MCU chip\n");
	gpio_set_value(drv_data->gpio_rst, 0);
	return ret;
}

static int fw_update(struct mcu_minidrones3_data *drv_data)
{
	int status;
	const struct firmware *fw;

	dev_info(&drv_data->spi->dev, "Search firmware %s\n",
		 drv_data->firmware_name);
	status = request_firmware(&fw, drv_data->firmware_name,
				  &drv_data->spi->dev);
	if (status < 0) {
		dev_err(&drv_data->spi->dev,
			"Failed to request firmware : %d\n", status);
		return status;
	}
	dev_info(&drv_data->spi->dev, "Found firmware, size %d\n",
		 fw->size);
	status = send_firmware(drv_data, (u8 *)fw->data, fw->size);
	return status;
}

static ssize_t dev_fw_update_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Write here to firmware update\n");
}
static ssize_t dev_fw_update_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct platform_device *plat_dev = to_platform_device(dev);
	struct mcu_minidrones3_data *drv_data = platform_get_drvdata(plat_dev);
	ret = mutex_lock_interruptible(&drv_data->iomutex);
	if (ret)
		return ret;
	ret = fw_update(drv_data);
	mutex_unlock(&drv_data->iomutex);
	return (ret == 0) ? count : -EIO;
}
static DEVICE_ATTR(fw_update, S_IRUGO | S_IWUSR,
		   dev_fw_update_show, dev_fw_update_store);

static ssize_t dev_version_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *plat_dev = to_platform_device(dev);
	struct mcu_minidrones3_data *drv_data = platform_get_drvdata(plat_dev);
	int res;
	int ver;
	res = mutex_lock_interruptible(&drv_data->iomutex);
	if (res)
		return res;
	ver = get_version(drv_data);
	mutex_unlock(&drv_data->iomutex);
	if (ver < 0)
		return ver;
	return sprintf(buf, "%d\n", ver);
}
static DEVICE_ATTR(version, S_IRUGO,
		   dev_version_show, NULL);

static ssize_t dev_vbat_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *plat_dev = to_platform_device(dev);
	struct mcu_minidrones3_data *drv_data = platform_get_drvdata(plat_dev);
	int res;
	int vbat;
	res = mutex_lock_interruptible(&drv_data->iomutex);
	if (res)
		return res;
	vbat = get_vbat(drv_data);
	mutex_unlock(&drv_data->iomutex);
	if (vbat < 0)
		return vbat;
	return sprintf(buf, "%d\n", vbat);
}
static ssize_t dev_vbat_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	return -EIO;
}
static DEVICE_ATTR(vbat, S_IRUGO | S_IWUSR,
		   dev_vbat_show, dev_vbat_store);

/* US heater PWM */
static ssize_t dev_us_heater_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *plat_dev = to_platform_device(dev);
	struct mcu_minidrones3_data *drv_data = platform_get_drvdata(plat_dev);
	ssize_t ret;
	int res;
	char *after;
	unsigned long ratio = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count != size)
		return -EINVAL;

	if (ratio > 255)
		return -EINVAL;

	res = mutex_lock_interruptible(&drv_data->iomutex);
	if (res)
		return res;
	ret = set_us_heater_pwm(drv_data, (uint8_t)ratio);
	mutex_unlock(&drv_data->iomutex);
	if (ret)
		return ret;
	return count;
}
static DEVICE_ATTR(us_heater_pwm, 0200, NULL, dev_us_heater_store);

/* Barometer heater PWM */
static ssize_t dev_baro_heater_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *plat_dev = to_platform_device(dev);
	struct mcu_minidrones3_data *drv_data = platform_get_drvdata(plat_dev);
	ssize_t ret;
	int res;
	char *after;
	unsigned long ratio = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (isspace(*after))
		count++;

	if (count != size)
		return -EINVAL;

	if (ratio > 255)
		return -EINVAL;

	res = mutex_lock_interruptible(&drv_data->iomutex);
	if (res)
		return res;
	ret = set_baro_heater_pwm(drv_data, (uint8_t)ratio);
	mutex_unlock(&drv_data->iomutex);
	if (ret)
		return ret;
	return count;
}
static DEVICE_ATTR(baro_heater_pwm, 0200, NULL, dev_baro_heater_store);

/* MCU reset */
static ssize_t dev_reset_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t size)
{
	struct platform_device *plat_dev = to_platform_device(dev);
	struct mcu_minidrones3_data *drv_data = platform_get_drvdata(plat_dev);

	mutex_lock(&drv_data->iomutex);
	mcu_reset_and_resume(drv_data);
	mutex_unlock(&drv_data->iomutex);

	return (ssize_t)size;
}
static DEVICE_ATTR(reset, 0200, NULL, dev_reset_store);

static struct mcu_minidrones3_data *pw_off_drv_data;
static void mcu_minidrones3_power_off(void)
{
	mutex_lock(&pw_off_drv_data->iomutex);
	power_off(pw_off_drv_data);
	mutex_unlock(&pw_off_drv_data->iomutex);
}

static irqreturn_t mcu_minidrones3_interrupt(int irq, void *dev_id)
{
	struct mcu_minidrones3_data *drv_data = dev_id;

	mutex_lock(&drv_data->iomutex);
	get_messages(drv_data);
	mutex_unlock(&drv_data->iomutex);

	return IRQ_HANDLED;
}

static int __devinit mcu_minidrones3_probe(struct spi_device *spi)
{
	int i;
	struct mcu_minidrones3_platform_data *mcu_minidrones3_pdata;
	struct mcu_minidrones3_data *drv_data;
	struct input_dev  *input_dev;
	int error = 0;

	printk(KERN_INFO "starting mcu_minidrones3\n");

	mcu_minidrones3_pdata = spi->dev.platform_data;
	if (mcu_minidrones3_pdata == NULL) {
		printk(KERN_ERR "no platform datas\n");
		goto error_ret;
	}
	drv_data = kzalloc(sizeof(*drv_data), GFP_KERNEL);
	if (drv_data == NULL) {
		error = -ENOMEM;
		goto error_ret;
	}
	drv_data->spi = spi;
	spi_set_drvdata(spi, drv_data);
	drv_data->gpio_rst = mcu_minidrones3_pdata->gpio_rst;
	drv_data->spi_read_delay_ms = mcu_minidrones3_pdata->spi_read_delay_ms;
	drv_data->keys_nb = mcu_minidrones3_pdata->keys_nb;
	drv_data->keys = mcu_minidrones3_pdata->keys;
	drv_data->needs_resync = 1;

	drv_data->firmware_name = mcu_minidrones3_pdata->firmware_name;
	if (drv_data->firmware_name == NULL)
		drv_data->firmware_name = "attiny_fw.bin";
	error = device_create_file(&spi->dev, &dev_attr_fw_update);
	if (error)
		goto error_free;
	error = device_create_file(&spi->dev, &dev_attr_version);
	if (error)
		goto error_free_fwfile;
	error = device_create_file(&spi->dev, &dev_attr_vbat);
	if (error)
		goto error_free_verfile;
	error = device_create_file(&spi->dev, &dev_attr_us_heater_pwm);
	if (error)
		goto error_free_vbat;
	error = device_create_file(&spi->dev, &dev_attr_baro_heater_pwm);
	if (error)
		goto error_free_uspwmfile;
	error = device_create_file(&spi->dev, &dev_attr_reset);
	if (error)
		goto error_free_baropwmfile;

	input_dev = input_allocate_device();
	if (!input_dev) {
		error = -ENOMEM;
		goto error_free_resetfile;
	}

	/* input layer init */
	drv_data->input = input_dev;
	input_dev->name = DRIVER_NAME;
	input_dev->phys = DRIVER_NAME"/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x19cf;
	input_dev->id.product = 0x0001;
	input_dev->id.version = DRIVER_VERSION;
	input_dev->dev.parent = &spi->dev;
	input_set_drvdata(input_dev, drv_data);
	for (i = 0; i < drv_data->keys_nb; i++)
		input_set_capability(drv_data->input, EV_KEY,
				     drv_data->keys[i].input_key);
	error = input_register_device(drv_data->input);
	if (error)
		goto error_free_input;

	mutex_init(&drv_data->iomutex);

	dev_info(&drv_data->spi->dev, "Firmware version %d\n",
		 get_version(drv_data));

	/* Read status at start */
	get_messages(drv_data);

	error = request_threaded_irq(drv_data->spi->irq,
				     NULL,
				     mcu_minidrones3_interrupt,
				     IRQF_ONESHOT |
				     IRQF_TRIGGER_HIGH,
				     DRIVER_NAME,
				     drv_data);
	if (error)
		goto error_unregister;

	pw_off_drv_data = drv_data;
	pm_power_off = mcu_minidrones3_power_off;
	return 0;

error_unregister:
	mutex_destroy(&drv_data->iomutex);
	input_unregister_device(drv_data->input);
error_free_input:
	input_free_device(input_dev);
error_free_resetfile:
	device_remove_file(&spi->dev, &dev_attr_reset);
error_free_baropwmfile:
	device_remove_file(&spi->dev, &dev_attr_baro_heater_pwm);
error_free_uspwmfile:
	device_remove_file(&spi->dev, &dev_attr_us_heater_pwm);
error_free_vbat:
	device_remove_file(&spi->dev, &dev_attr_vbat);
error_free_verfile:
	device_remove_file(&spi->dev, &dev_attr_version);
error_free_fwfile:
	device_remove_file(&spi->dev, &dev_attr_fw_update);
error_free:
	kfree(drv_data);
error_ret:
	return error;
}

static int __devexit mcu_minidrones3_remove(struct spi_device *spi)
{
	struct mcu_minidrones3_data *drv_data = spi_get_drvdata(spi);

	free_irq(drv_data->spi->irq, drv_data);
	mutex_destroy(&drv_data->iomutex);
	input_unregister_device(drv_data->input);
	input_free_device(drv_data->input);
	device_remove_file(&spi->dev, &dev_attr_fw_update);
	device_remove_file(&spi->dev, &dev_attr_version);
	device_remove_file(&spi->dev, &dev_attr_vbat);
	device_remove_file(&spi->dev, &dev_attr_us_heater_pwm);
	device_remove_file(&spi->dev, &dev_attr_baro_heater_pwm);
	device_remove_file(&spi->dev, &dev_attr_reset);
	kfree(drv_data);

	return 0;
}

static struct spi_driver mcu_minidrones3_driver = {
	.driver = {
		.name = "mcu_minidrones3",
		.owner = THIS_MODULE,
	},
	.probe = mcu_minidrones3_probe,
	.remove = __devexit_p(mcu_minidrones3_remove),
};

static __init int mcu_minidrones3_init(void)
{
	return spi_register_driver(&mcu_minidrones3_driver);
}
module_init(mcu_minidrones3_init);

static __exit void mcu_minidrones3_exit(void)
{
	spi_unregister_driver(&mcu_minidrones3_driver);
}
module_exit(mcu_minidrones3_exit);

MODULE_DESCRIPTION("Parrot MiniDrones 3 MCU Driver");
MODULE_ALIAS("mcu_minidrones3");
MODULE_AUTHOR("Thomas Poussevin <thomas.poussevin@parrot.com>");
MODULE_LICENSE("GPL");
