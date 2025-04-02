/********************************************************
	locoboard.c
	
	This module contains code that relates to the board hardware
	There are four main parts
	
	Buttons - this scans the buttons and rotary encoder and communicates with the ui using addEvent
	
	DAC - Audio related functions
	
		locoAudioInit initialises the DAC and I2S
		setVolume
		startAudioThread creates a thread to fetch audio samples using getAdfSamples
		and send them to the DAC
	
	SD Card
	
		initSDDetect 
		sdPoll - mounts the SD when inserted
		isMounted

	TFT
		lcdInit - this initialises the display and lvgl
		set_backlight_brightness
		
		NB lvgl changes a lot so a specific version has been included as 
		a component in this project. There are some minor edits to
		components/lvgl/src/lv_conf_internal.h (LV_COLOR_16_SWAP and fonts)
		components/lvgl/src/misc/lv_mem.c to allocate memory from SPIRAM

	
*********************************************************/


#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <pthread.h>

#include <loco.h>
#include "audio.h"

#include "driver/i2c.h"
#include "driver/i2s_std.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_spiffs.h"

#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include <stdio.h>
#include "esp_timer.h"
//#include "lvgl.h"
#include "driver/ledc.h"

#include <math.h>


uint64_t millis (){
	return esp_timer_get_time() / 1000;
}	

/****************************************************************
 DAC
*****************************************************************/ 


int i2c_port = 0;

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ	/*!< I2C master read */
#define ACK_CHECK_EN 0x1        	/*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0       	/*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0             	/*!< I2C ack value */
#define NACK_VAL 0x1            	/*!< I2C nack value */

static esp_err_t i2c_master_driver_initialize(void)
{
	
//	printf ("i2c_master_driver_initialize()\n");
	
	i2c_config_t conf = {
    	.mode = I2C_MODE_MASTER,
    	.sda_io_num = 39,
    	.sda_pullup_en = GPIO_PULLUP_ENABLE,
    	.scl_io_num = 1,
    	.scl_pullup_en = GPIO_PULLUP_ENABLE,   	

    	.master.clk_speed = 100000,
    	// .clk_flags = 0,      	/*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
	};
	return i2c_param_config(i2c_port, &conf);
}


int do_i2cdetect_cmd()
{
	i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	i2c_master_driver_initialize();
    
    
	uint8_t address;
    
    
	printf("\n 	0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
	for (int i = 0; i < 128; i += 16) {
    	printf("%02x: ", i);
    	for (int j = 0; j < 16; j++) {
        	fflush(stdout);
        	address = i + j;
        	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        	i2c_master_start(cmd);
        	i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
        	i2c_master_stop(cmd);
        	esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_PERIOD_MS);
        	i2c_cmd_link_delete(cmd);
        	if (ret == ESP_OK) {
            	printf("%02x ", address);
        	} else if (ret == ESP_ERR_TIMEOUT) {
            	printf("UU ");
        	} else {
            	printf("-- ");
        	}
    	}
    	printf("\r\n");
	}
//  printf ("do_i2cdetect_cmd - leaving driver installed\n");
	i2c_driver_delete(i2c_port);
	return 0;
}

typedef enum {
    BIT_LENGTH_MIN = -1,
    BIT_LENGTH_16BITS = 0x03,
    BIT_LENGTH_18BITS = 0x02,
    BIT_LENGTH_20BITS = 0x01,
    BIT_LENGTH_24BITS = 0x00,
    BIT_LENGTH_32BITS = 0x04,
    BIT_LENGTH_MAX,
} es_bits_length_t;

typedef enum {
    DAC_OUTPUT_MIN = -1,
    DAC_OUTPUT_LOUT1 = 0x04,
    DAC_OUTPUT_LOUT2 = 0x08,
    DAC_OUTPUT_SPK   = 0x09,
    DAC_OUTPUT_ROUT1 = 0x10,
    DAC_OUTPUT_ROUT2 = 0x20,
    DAC_OUTPUT_ALL = 0x3c,
    DAC_OUTPUT_MAX,
} es_dac_output_t;

typedef enum {
    ADC_INPUT_MIN = -1,
    ADC_INPUT_LINPUT1_RINPUT1 = 0x00,
    ADC_INPUT_MIC1  = 0x05,
    ADC_INPUT_MIC2  = 0x06,
    ADC_INPUT_LINPUT2_RINPUT2 = 0x50,
    ADC_INPUT_DIFFERENCE = 0xf0,
    ADC_INPUT_MAX,
} es_adc_input_t;

typedef enum {
    ES_I2S_MIN = -1,
    ES_I2S_NORMAL = 0,
    ES_I2S_LEFT = 1,
    ES_I2S_RIGHT = 2,
    ES_I2S_DSP = 3,
    ES_I2S_MAX
} es_i2s_fmt_t;

typedef enum {
    ES_MODULE_MIN = -1,
    ES_MODULE_ADC = 0x01,
    ES_MODULE_DAC = 0x02,
    ES_MODULE_ADC_DAC = 0x03,
    ES_MODULE_LINE = 0x04,
    ES_MODULE_MAX
} es_module_t;

typedef enum {
    I2S_NORMAL = 0,  /*!< set normal I2S format */
    I2S_LEFT,        /*!< set all left format */
    I2S_RIGHT,       /*!< set all right format */
    I2S_DSP,         /*!< set dsp/pcm format */
} es_format_t;

typedef enum {
    ES_MODE_MIN = -1,
    ES_MODE_SLAVE = 0x00,
    ES_MODE_MASTER = 0x01,
    ES_MODE_MAX,
} es_mode_t;

/* ES8388 register */
#define ES8388_CONTROL1         0x00
#define ES8388_CONTROL2         0x01

#define ES8388_CHIPPOWER        0x02

#define ES8388_ADCPOWER         0x03
#define ES8388_DACPOWER         0x04

#define ES8388_CHIPLOPOW1       0x05
#define ES8388_CHIPLOPOW2       0x06

#define ES8388_ANAVOLMANAG      0x07

#define ES8388_MASTERMODE       0x08
/* ADC */
#define ES8388_ADCCONTROL1      0x09
#define ES8388_ADCCONTROL2      0x0a
#define ES8388_ADCCONTROL3      0x0b
#define ES8388_ADCCONTROL4      0x0c
#define ES8388_ADCCONTROL5      0x0d
#define ES8388_ADCCONTROL6      0x0e
#define ES8388_ADCCONTROL7      0x0f
#define ES8388_ADCCONTROL8      0x10
#define ES8388_ADCCONTROL9      0x11
#define ES8388_ADCCONTROL10     0x12
#define ES8388_ADCCONTROL11     0x13
#define ES8388_ADCCONTROL12     0x14
#define ES8388_ADCCONTROL13     0x15
#define ES8388_ADCCONTROL14     0x16
/* DAC */
#define ES8388_DACCONTROL1      0x17
#define ES8388_DACCONTROL2      0x18
#define ES8388_DACCONTROL3      0x19
#define ES8388_DACCONTROL4      0x1a
#define ES8388_DACCONTROL5      0x1b
#define ES8388_DACCONTROL6      0x1c
#define ES8388_DACCONTROL7      0x1d
#define ES8388_DACCONTROL8      0x1e
#define ES8388_DACCONTROL9      0x1f
#define ES8388_DACCONTROL10     0x20
#define ES8388_DACCONTROL11     0x21
#define ES8388_DACCONTROL12     0x22
#define ES8388_DACCONTROL13     0x23
#define ES8388_DACCONTROL14     0x24
#define ES8388_DACCONTROL15     0x25
#define ES8388_DACCONTROL16     0x26
#define ES8388_DACCONTROL17     0x27
#define ES8388_DACCONTROL18     0x28
#define ES8388_DACCONTROL19     0x29
#define ES8388_DACCONTROL20     0x2a
#define ES8388_DACCONTROL21     0x2b
#define ES8388_DACCONTROL22     0x2c
#define ES8388_DACCONTROL23     0x2d
#define ES8388_DACCONTROL24     0x2e
#define ES8388_DACCONTROL25     0x2f
#define ES8388_DACCONTROL26     0x30
#define ES8388_DACCONTROL27     0x31
#define ES8388_DACCONTROL28     0x32
#define ES8388_DACCONTROL29     0x33
#define ES8388_DACCONTROL30     0x34

#define ES8388_ADDR 0x10 // Replace this with the actual I2C address of the ES8388

uint8_t read_register(uint8_t reg_addr) {
    uint8_t data = 0; // Data to store the register value

    // Use I2C master write-read function to send the register address and read data
    i2c_master_write_read_device(
        i2c_port,
        ES8388_ADDR,
        &reg_addr,
        1,      // Write size (1 byte for register address)
        &data,
        1,      // Read size (1 byte for register data)
        pdMS_TO_TICKS(1000) // Timeout
    );

    return data;
}

void write_register(uint8_t reg_addr, uint8_t data) {
    uint8_t write_data[2] = { reg_addr, data }; // First byte is the register address, second byte is the data

    // Write register address and data
    i2c_master_write_to_device(
        i2c_port,
        ES8388_ADDR,
        write_data,
        sizeof(write_data), // Write 2 bytes (register address + data)
        pdMS_TO_TICKS(1000) // Timeout
    );
}

void es8388_read_all()
{
	uint8_t regs[50];
	
    for (int i = 0; i < 50; i++) {
//        es_read_reg(i, &regs[i]);
        regs[i] = read_register (i);
    }
    cdump (regs,50);
}


/**
 * @brief Configure ES8388 ADC and DAC volume. Basicly you can consider this as ADC and DAC gain
 *
 * @param mode:             set ADC or DAC or all
 * @param volume:           -96 ~ 0              for example Es8388SetAdcDacVolume(ES_MODULE_ADC, 30, 6); means set ADC volume -30.5db
 * @param dot:              whether include 0.5. for example Es8388SetAdcDacVolume(ES_MODULE_ADC, 30, 4); means set ADC volume -30db
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
static void es8388_set_adc_dac_volume(int mode, int volume, int dot)
{
    if ( volume < -96 || volume > 0 ) {
        if (volume < -96)
            volume = -96;
        else
            volume = 0;
    }
    dot = (dot >= 5 ? 1 : 0);
    volume = (-volume << 1) + dot;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        write_register (ES8388_ADCCONTROL8, volume);
        write_register (ES8388_ADCCONTROL9, volume);  //ADC Right Volume=0db
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        write_register (ES8388_DACCONTROL5, volume);
        write_register (ES8388_DACCONTROL4, volume);
    }
}

void xes8388_set_voice_mute(bool enable)
{
    uint8_t reg = 0;
    reg = read_register (ES8388_DACCONTROL3);
    reg = reg & 0xFB;
    write_register (ES8388_DACCONTROL3, reg | (((int)enable) << 2));
}

void xes8388_start(es_module_t mode)
{
    uint8_t prev_data = 0, data = 0;
    prev_data = read_register (ES8388_DACCONTROL21);
    if (mode == ES_MODULE_LINE) {
        write_register (ES8388_DACCONTROL16, 0x09); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2 by pass enable
        write_register (ES8388_DACCONTROL17, 0x50); // left DAC to left mixer enable  and  LIN signal to left mixer enable 0db  : bupass enable
        write_register (ES8388_DACCONTROL20, 0x50); // right DAC to right mixer enable  and  LIN signal to right mixer enable 0db : bupass enable
        write_register (ES8388_DACCONTROL21, 0xC0); //enable adc
    } else {
        write_register (ES8388_DACCONTROL21, 0x80);   //enable dac
    }
    data = read_register (ES8388_DACCONTROL21);
    if (prev_data != data) {
    	printf( "Resetting State Machine\n");
      write_register (ES8388_CHIPPOWER, 0xF0);   //start state machine
      write_register (ES8388_CHIPPOWER, 0x00);   //start state machine
    }
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	printf( "Powering up ADC\n");
      write_register (ES8388_ADCPOWER, 0x00);   //power up adc and line in
    }
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC || mode == ES_MODULE_LINE) {
    	printf( "Powering up DAC\n");
      write_register (ES8388_DACPOWER, 0x3c);   //power up dac and line out
      xes8388_set_voice_mute(false);
    }
}

// This is internal assumes I2C driver installed

void xes8388_set_voice_volume(int volume)
{
    if (volume < 0)
        volume = 0;
    else if (volume > 100)
        volume = 100;
    volume /= 3;
    write_register (ES8388_DACCONTROL24, volume);
    write_register (ES8388_DACCONTROL25, volume);
    write_register (ES8388_DACCONTROL26, volume);
    write_register (ES8388_DACCONTROL27, volume);
}

// This variant installs and deletes the I2C driver

void setVolume (int volume){
	i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	i2c_master_driver_initialize();	
	xes8388_set_voice_volume(volume);
	i2c_driver_delete(i2c_port);		
}

// This function sets the I2S format which can be one of
//		I2S_NORMAL
//		I2S_LEFT		Left Justified
//		I2S_RIGHT,      Right Justified
//		I2S_DSP,        dsp/pcm format
//
// and the bits per sample which must be one of
//		BIT_LENGTH_16BITS
//		BIT_LENGTH_18BITS
//		BIT_LENGTH_20BITS
//		BIT_LENGTH_24BITS
//		BIT_LENGTH_32BITS
//
// Note the above must match the ESP-IDF I2S configuration which is set separately

void xes8388_config_i2s( es_bits_length_t bits_length, es_module_t mode, es_format_t fmt )
{
    esp_err_t res = ESP_OK;
    uint8_t reg = 0;

    // Set the Format
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S ADC Format\n");
        reg = read_register (ES8388_ADCCONTROL4);
        reg = reg & 0xfc;
        write_register (ES8388_ADCCONTROL4, reg | fmt);
    }
//    printf ("xes8388_config_i2s hacked - do not touch DACCONTROL1\n");
    
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
		fmt = 1;
        printf( "Setting I2S DAC Format %d\n",fmt);
        reg = read_register (ES8388_DACCONTROL1);
        reg = reg & 0xf9;
        write_register (ES8388_DACCONTROL1, reg | (fmt << 1));
    }


    // Set the Sample bits length
    int bits = (int)bits_length;
    if (mode == ES_MODULE_ADC || mode == ES_MODULE_ADC_DAC) {
        printf( "Setting I2S ADC Bits: %d\n", bits);
        reg = read_register (ES8388_ADCCONTROL4);
        reg = reg & 0xe3;
        write_register (ES8388_ADCCONTROL4, reg | (bits << 2));
    }
    
    if (mode == ES_MODULE_DAC || mode == ES_MODULE_ADC_DAC) {
        printf ("Setting I2S DAC Bits: %d\n", bits);
        reg = read_register (ES8388_DACCONTROL1);
        reg = reg & 0xc7;
        write_register (ES8388_DACCONTROL1, reg | (bits << 3));
    }

    reg = read_register (ES8388_DACCONTROL1);
    printf ("xes_config_i2s() final value %02x\n",reg);
}



/*
MJB 14/3/25
This variant appears to fix occasional bad initialisation
There is one remaining difference with the Helix registers
15 40 00 09 3c 00 00 7c
00 00 00 06 0c 04 30 20
00 00 38 b0 32 06 00 1a
04 00 00 00 08 00 1f f7
fd ff 1f f7 fd ff 00 90
28 28 90 80 00 00 10 10
10 10
Register 3 ADCPOWER
Helix writes 0x9 as the last step.
If the problem resurfaces another thing to try is making sure there are clocks
running during setup. Currently only MCLK runs
I2S is not setup or paused (during a codec restart
*/


void xes8388_init2( es_dac_output_t output, es_adc_input_t input )
{
	printf ("NEW xes8388_init2()\n");

  write_register (ES8388_MASTERMODE, ES_MODE_SLAVE ); //CODEC IN I2S SLAVE MODE
  write_register (ES8388_CHIPPOWER, 0xF3);
  write_register (ES8388_DACCONTROL21, 0x80); //set internal ADC and DAC use the same LRCK clock, ADC LRCK as internal LRCK
  write_register (ES8388_CONTROL1, 0x15);  //Enfr=0,Play&Record Mode,(0x17-both of mic&paly) MJB
  write_register (ES8388_CONTROL2, 0x40);		// MJB 
  
    
  write_register (ES8388_DACCONTROL3, 0x04);  // 0x04 mute/0x00 unmute&ramp;DAC unmute and  disabled digital volume control soft ramp
  write_register (ES8388_DACPOWER, 0xC0);  //disable DAC and disable Lout/Rout/1/2


  write_register (ES8388_DACCONTROL1, 0x18);//1a 0x18:16bit iis , 0x00:24
  write_register (ES8388_DACCONTROL2, 0x04);  //DACFsMode,SINGLE SPEED; DACFsRatio,512
  write_register (ES8388_DACCONTROL16, 0x00); // 0x00 audio on LIN1&RIN1,  0x09 LIN2&RIN2
  write_register (ES8388_DACCONTROL17, 0x90); // only left DAC to left mixer enable 0db
  write_register (ES8388_DACCONTROL20, 0x90); // only right DAC to right mixer enable 0db

  write_register (ES8388_DACCONTROL23, 0x00);   //vroi=0
  es8388_set_adc_dac_volume(ES_MODULE_DAC, 0, 0);          // 0db

  write_register (ES8388_DACPOWER, output );
  write_register (ES8388_ADCPOWER, 0xFF);
  write_register (ES8388_ADCCONTROL1, 0x0); // MIC Left and Right channel PGA gain MJB

  write_register (ES8388_ADCCONTROL2, input);

  write_register (ES8388_ADCCONTROL3, 0x06); // MJB
  write_register (ES8388_ADCCONTROL4, 0x0d); // Left/Right data, Left/Right justified mode, Bits length, I2S format
  write_register (ES8388_ADCCONTROL5, 0x04);  //ADCFsMode,singel SPEED,RATIO=512 MJB

  es8388_set_adc_dac_volume(ES_MODULE_ADC, 0, 0);      // 0db
//  write_register (ES8388_ADCPOWER, 0x09); //Power on ADC, Enable LIN&RIN, Power off MICBIAS, set int1lp to low power mode
  write_register (ES8388_ADCPOWER, 0xF0); //Power on ADC, Enable LIN&RIN, Power off MICBIAS, set int1lp to low power mode

  write_register (ES8388_CHIPPOWER, 0x00);
    
}


void xes8388_init()
{
    // Input/Output Modes
    //
    //	es_dac_output_t output = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
    //	es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;
    // 	es_adc_input_t input = ADC_INPUT_LINPUT2_RINPUT2;

    es_dac_output_t output = DAC_OUTPUT_LOUT1 | DAC_OUTPUT_LOUT2 | DAC_OUTPUT_ROUT1 | DAC_OUTPUT_ROUT2;
	//es_dac_output_t output = DAC_OUTPUT_LOUT1  | DAC_OUTPUT_ROUT1;
	//es_dac_output_t output = DAC_OUTPUT_LOUT2  | DAC_OUTPUT_ROUT2;

    //es_dac_output_t output = 0;
	es_adc_input_t input = ADC_INPUT_LINPUT1_RINPUT1;

    xes8388_init2( output, input );

    // Modes Available
    //
    //	es_mode_t  = ES_MODULE_ADC;
    //	es_mode_t  = ES_MODULE_LINE;
    //	es_mode_t  = ES_MODULE_DAC;
    //	es_mode_t  = ES_MODULE_ADC_DAC;

    es_bits_length_t bits_length = BIT_LENGTH_16BITS;
    es_module_t module = ES_MODULE_DAC;
    es_format_t fmt = I2S_NORMAL;

    xes8388_config_i2s( bits_length, ES_MODULE_ADC_DAC, fmt );
    xes8388_set_voice_volume( 50 );
    xes8388_start( module );

	es8388_read_all();

}

void xes8388_deinit(void)
{

//    res = es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0xFF);  //reset and stop es8388
    write_register(ES8388_CHIPPOWER, 0xFF);  //reset and stop es8388
}

// base addresses p131 S3 Technical Reference Manual

#define I2S0BASE 0x6000F000
#define I2S1BASE 0x6002D000
#define GPIOBASE 0x60004000
#define IOMUXBASE 0x60009000
#define mI2S_RX_CONF_REG 0x20
#define mI2S_RX_CONF1_REG 0x28
#define mI2S_TX_CONF1_REG 0x2c
#define mI2S_RX_CLKM_CONF_REG 0x30
#define mI2S_TX_CLKM_CONF_REG 0x34
#define mI2S_RX_CLKM_DIV_CONF_REG 0x38
#define mI2S_TX_CLKM_DIV_CONF_REG 0x3C

#define mGPIO_ENABLE_REG 0x20
#define mGPIO_W1TS_REG 0x24
#define mGPIO_ENABLE_W1TS_REG 0x24

uint32_t readReg(uint32_t base, uint32_t offset) {
  uint32_t r = *(volatile uint32_t *)(base + offset);
  return r;
}

void writeReg(uint32_t base, uint32_t offset, uint32_t value) {
  volatile uint32_t *a = (volatile uint32_t *)(base + offset);
  *a = value;
}

// table of peripherals p206 S3 Technical Reference Manual

void setPeripheralOutput(unsigned peripheral, unsigned gpio) {
  uint32_t ai = (GPIOBASE + 0x0554 + (4 * gpio));
  volatile uint32_t *a = (volatile uint32_t *)ai;
  *a = peripheral + 0x400;
}

uint32_t readPeripheralOutput(unsigned gpio) {
  uint32_t ai = (GPIOBASE + 0x0554 + (4 * gpio));
  volatile uint32_t *a = (volatile uint32_t *)ai;
  return *a;
}

void setPeripheralInput(unsigned peripheral, unsigned gpio) {
  uint32_t ai = (GPIOBASE + 0x0154 + (4 * peripheral));
  volatile uint32_t *a = (volatile uint32_t *)ai;
  *a = gpio + 0x80;
}

uint32_t readPeripheralInput(unsigned peripheral) {
  uint32_t ai = (GPIOBASE + 0x0154 + (4 * peripheral));
  volatile uint32_t *a = (volatile uint32_t *)ai;
  return *a;
}

uint32_t readIOMux(unsigned gpio) {
  uint32_t ai = (IOMUXBASE + 0x0010 + (4 * gpio));
  volatile uint32_t *a = (volatile uint32_t *)ai;
  return *a;
}

void setIOMux(unsigned gpio, uint32_t value) {
  uint32_t ai = (IOMUXBASE + 0x0010 + (4 * gpio));
  volatile uint32_t *a = (volatile uint32_t *)ai;
  *a = value;
}

void i2sUpdate() {
  uint32_t v = readReg(I2S0BASE, mI2S_RX_CONF_REG);
  v = v | 0x100;
  writeReg(I2S0BASE, mI2S_RX_CONF_REG, v);

  v = readReg(I2S1BASE, mI2S_RX_CONF_REG);
  v = v | 0x100;
  writeReg(I2S1BASE, mI2S_RX_CONF_REG, v);
}

// This function sets up the I2S clock to use the external 22MHz crystal

void setupTXDivider(int rate) {

  printf("before setupTXDivider\n");

  writeReg(IOMUXBASE, 0, 0xFF00); // PIN_CTRL / IO_MUX_PIB_CTRL_REG p 236

  setPeripheralInput(23, 3); // I2S0_MCLK_In from GPIO20 - page 207 TRM

  writeReg(I2S0BASE, mI2S_TX_CLKM_DIV_CONF_REG,
       	0x0); // turn off fractional (1/N+b/a) divider

  i2sUpdate();

  int divider;
  if (rate == 44100)
	divider = 2;
  else
	divider = 4;

  uint32_t v;

  v = readReg(I2S0BASE, mI2S_TX_CLKM_CONF_REG); // p747 p771 TRM
  v = v & 0xE7FFFFFF; // set I2S0_TX_CLK_SEL to MCLK_in
  v |= 0x18000000;
  v = v & 0xFFFFFF00; // set I2S0_TX_CLKM_DIV_NUM to 2
  v |= divider;
  writeReg(I2S0BASE, mI2S_TX_CLKM_CONF_REG, v);

  i2sUpdate();

  gpio_set_direction(20, GPIO_MODE_INPUT);

  writeReg(GPIOBASE, 0x28,
       	0x100000); // GPIO_ENABLE_W1TC -clear enable on GPIO20
  writeReg(GPIOBASE, 0x554 + 4 * 20, 0x500);

  gpio_set_direction(20, GPIO_MODE_INPUT);
}


int i2sUnderrun = 0;

static IRAM_ATTR bool i2s_tx_queue_overflow_callback(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
	i2sUnderrun = 1;
	return false;
}


i2s_chan_handle_t tx_handle;

i2s_event_callbacks_t cbs = {
	.on_recv = NULL,
	.on_recv_q_ovf = NULL,
	.on_sent = NULL,
	.on_send_q_ovf = i2s_tx_queue_overflow_callback,
};


void locoAudioInit(void) {


	i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	i2c_master_driver_initialize();
 
  xes8388_init();


  /******************* This is the new v5.0 I2S system */

  /* Get the default channel configuration by helper macro.
   * This helper macro is defined in 'i2s_common.h' and shared by all the i2s
   * communication mode. It can help to specify the I2S role, and port id */
  i2s_chan_config_t chan_cfg =
  	I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

//  chan_cfg.dma_frame_num = 1000;        	// was 240 - 1000 breaks the system

  /* Allocate a new tx channel and get the handle of this channel */  
  i2s_new_channel(&chan_cfg, &tx_handle, NULL);

  i2s_channel_register_event_callback(tx_handle, &cbs, NULL);

  /* Setting the configurations, the slot configuration and clock configuration
   * can be generated by the macros These two helper macros is defined in
   * 'i2s_std.h' which can only be used in STD mode. They can help to specify
   * the slot and clock configurations for initialization or updating */
  i2s_std_config_t std_cfg = {
  	.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
  	.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                              	I2S_SLOT_MODE_STEREO),
  	.gpio_cfg =
      	{
         	
          	.mclk = I2S_GPIO_UNUSED,


          	.bclk = GPIO_NUM_38,
          	.ws = GPIO_NUM_17,
          	.dout = GPIO_NUM_18,
          	.din = GPIO_NUM_7,         	          	
          	.invert_flags =
              	{
                  	.mclk_inv = false,
                  	.bclk_inv = false,
                  	.ws_inv = false,
              	},
      	},
  };
  /* Initialize the channel */
  i2s_channel_init_std_mode(tx_handle, &std_cfg);

  /* Before read data, start the tx channel first */
  i2s_channel_enable(tx_handle);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  //************************** Use external crystal

#if 1
  //	setupDivider (rate); 
  setupTXDivider(44100);
#endif

  //****************************



  // Disable CLK_OUT2 from RXD pin

  gpio_set_direction(44, GPIO_MODE_INPUT);

  setPeripheralInput(
  	12, 44); // RXD0 from GPIO44 - pages 207 & 230 technical reference manual



	i2c_driver_delete(i2c_port);

}

pthread_t audioThread;
int testToneEnable = 0;
int stereoToneEnable = 1;
int rightTestTone = 0;
int i2sRestart = 0;

#define AUDIOBUFFERSIZE 4096
int16_t audioBuffer[AUDIOBUFFERSIZE];
#define PI (3.14159265)

// MJB 28/2/24 alternate 3f on right side an 1f on left if stereo test

void testWaveform() {

  int frequency = 20; // multiple of 44100/AUDIOBUFFERSIZE (21.5Hz)
  double sin_float, cos_float;
  int amplitude = 23170;    	// -3dB
//  int amplitude = 16000;
  int16_t *s = audioBuffer;
  for (int n = 0; n < AUDIOBUFFERSIZE / 2; n++) {
	sin_float = amplitude * sin(3*frequency * n * 2 * PI / (AUDIOBUFFERSIZE / 2));
	cos_float = amplitude * cos(frequency * n * 2 * PI / (AUDIOBUFFERSIZE / 2));
	int val = sin_float;
	int val2 = cos_float;
	//    	*s++ = 0;
	if (stereoToneEnable){
    	if (rightTestTone){
        	*s++ = val & 0xFFFF;
        	*s++ = n & 0x3;            	// "noise" to stop mute kicking in
    	}
    	else {
        	*s++ = n & 0x3;
        	*s++ = val2 & 0xFFFF;
    	}
	}
	else {   	 
    	*s++ = val2 & 0xFFFF;	// right
    	*s++ = val2 & 0xFFFF;           	 
	}
  }
}

// void *audioThreadCode(void *param) {
void audioThreadCode(void *param) {

  int16_t *s = audioBuffer;
  int count = 0;

  long int stereoCount = 0;

  testWaveform();

  while (true) {

    size_t len;
    s = audioBuffer;

    //	count = audioThreadEnable ? getAdfSamples ((unsigned char
    //*)s,AUDIOBUFFERSIZE*2) : 0;
    count = getAdfSamples((unsigned char *)s, AUDIOBUFFERSIZE * 2);

    //	printf ("audioThreadCode audioThreadEnable=%d\n",audioThreadEnable);

    if (!count) {

      if (testToneEnable) {
        stereoCount += AUDIOBUFFERSIZE / 2;
        if (stereoCount >= 44100 * 3) { // three seconds
          stereoCount = 0;
          rightTestTone = !rightTestTone;
          testWaveform(); // rebuild the waveform
        }
      } else
        memset(audioBuffer, 0, AUDIOBUFFERSIZE * 2);

      count = AUDIOBUFFERSIZE * 2;
      s = audioBuffer;
    }

    while (count) {
      /*
          if (i2sRestart)
            vTaskDelay(10 / portTICK_PERIOD_MS);
              else {
                      int r = i2s_channel_write(tx_handle, s, count, &len, 100 /
         portTICK_PERIOD_MS); if (r) printf ("i2s_channel_write () failed\n");
              }
      */
      int r = i2s_channel_write(tx_handle, s, count, &len,
                                500 / portTICK_PERIOD_MS);
      if (r)
        printf("i2s_channel_write () failed err=%d count %d len %d\n",r,count, len);

      count -= len;
      s += len / 2;
    }
  }
}

void startAudioThread() {
    
  printf ("startAudioThread ()\n");    
    
//  pthread_mutex_init(&queueMutex, NULL);

  
#if 0
	static StaticTask_t audioTaskBuffer;
	uint8_t *audioStack = 	heap_caps_malloc (STACKSIZE,MALLOC_CAP_SPIRAM);	
  xTaskCreateStatic(&audioThreadCode, "audioThread", STACKSIZE, NULL, 7, audioStack, &audioTaskBuffer);
#else
  xTaskCreate(&audioThreadCode, "audioThread", STACKSIZE, NULL, 7, NULL);
#endif  
  
}


void toggleTestTone() {
	stereoToneEnable = 0;
  testToneEnable = !testToneEnable;
  if (testToneEnable) testWaveform();
  printf("toggleTestTone = %d\n", testToneEnable);
}

void toggleStereoTestTone() {
	stereoToneEnable = 1;
  testToneEnable = !testToneEnable;
  if (testToneEnable) testWaveform();
  printf("toggleStereoTestTone = %d\n", testToneEnable);
}

void testToneOff() { testToneEnable = 0; }

void codecRestart (int volume){
	
	
	printf ("codecRestart (%d)\n",volume);
	i2sRestart = 1;	

	i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
	i2c_master_driver_initialize();

	i2s_channel_disable(tx_handle);
	xes8388_deinit();

    vTaskDelay(100 / portTICK_PERIOD_MS);	  
//		vTaskDelay (pdMS_TO_TICKS(100));

	xes8388_init();
	xes8388_set_voice_volume(volume);
		
	i2s_channel_enable(tx_handle);

	i2c_driver_delete(i2c_port);	
	i2sRestart = 0;
}	



/****************************************************************
 SD Card
*****************************************************************/ 



#define MOUNT_POINT "/sdcard"

sdmmc_card_t *sdCard;
int sdMounted = 0;


int isMounted (){
	return sdMounted;
}    

sdmmc_card_t sdCardStructure;


int mountSd(int format) {

	if (sdMounted){
    	printf ("Already mounted\n");
    	return 0;
	}    

#if (1)
  printf ("WARNING mountSD NIY for LOCO2\n");
  printf ("uses too much memory\n");
  return 0;
#else  
    
  sdCard = &sdCardStructure; 

  esp_err_t ret;

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
  	.format_if_mount_failed = format,
  	.max_files = 5,
  	.allocation_unit_size = 16 * 1024};

  const char mount_point[] = MOUNT_POINT;

  sdmmc_host_t host = SDMMC_HOST_DEFAULT();



  // This initializes the slot without card detect (CD) and write protect (WP)
  // signals. Modify slot_config.gpio_cd and slot_config.gpio_wp if your board
  // has these signals.
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

  slot_config.width = 1;
  slot_config.clk = 41;    
  slot_config.cmd = 42;
  slot_config.d0 = 40;
  slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

  ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config,
                            	&sdCard);
  printf ("mountSd result %d\n", ret);

  if (ret) return 0;    

  sdMounted = 1;
 
  uint64_t total;
  uint64_t free;
  int rf = esp_vfs_fat_info(mount_point, &total, &free);
  if (rf == ESP_OK) {
	uint64_t temp = total / 1000000;
	printf ("Total %dMb\n", (int)temp);
	temp = free / 1000000;
	printf ("Free %dMb\n", (int)temp);
  } else {
	printf ("esp_vfs_fat_info didnt work error %s\n", esp_err_to_name(rf));
  }

  return !ret;
 
#endif  
  
}

// I found that if I did this at the same time as initButtons
// GPIO 12 reverted to an output - so something else is setting this pin by mistake


void initSDDetect (){
  gpio_set_direction(12, GPIO_MODE_INPUT);			// SD Detect	
}

int getSDDetect (){
	return gpio_get_level(12);
}


int sdTimer = 0;
int sdMountFail = 0;

int unMountSd (){

    const char mount_point[] = MOUNT_POINT;
	int ret = esp_vfs_fat_sdcard_unmount (mount_point,sdCard);

    printf ("Un mountSd result %d\n",ret);
    
    return !ret;
	
}

void sdPoll() {

  if (sdTimer > 0)
    sdTimer--;
  if (sdTimer)
    return;
  sdTimer = 10;

  if (sdMounted || sdMountFail) {
    if (!getSDDetect()) {
      printf ("SD card removed\n");
      if (sdMounted)
        unMountSd();
      sdMounted = 0;
      sdMountFail = 0;
    }
  } else {
    if (getSDDetect()) {
      printf ("SD card inserted\n");
      if (mountSd(0)) {
        sdMounted = 1;
        sdMountFail = 0;

      } else {
        sdMounted = 0;
        sdMountFail = 1;
      }
    }
  }
}

void initSpiffs() {

  printf("Initializing SPIFFS\n");
	
  esp_vfs_spiffs_conf_t conf = {.base_path = "/spiffs",
                                .partition_label = NULL,
                                .max_files = 5,
                                .format_if_mount_failed = true};

  // Use settings defined above to initialize and mount SPIFFS filesystem.
  // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
  esp_err_t ret = esp_vfs_spiffs_register(&conf);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      printf("Failed to mount or format filesystem\n");
    } else if (ret == ESP_ERR_NOT_FOUND) {
      printf("Failed to find SPIFFS partition\n");
    } else {
      printf("Failed to initialize SPIFFS (%s)\n", esp_err_to_name(ret));
    }
  } else {
    
        size_t size;
        size_t used;
        esp_spiffs_info(NULL, &size, &used);
        printf("SPIFFS OK %d/%d\n", used, size);
    
    printf("SPIFFS OK\n");
  }
}

