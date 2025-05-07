#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_heap_alloc_caps.h"
#include "driver/ledc.h"

#include "sdkconfig.h"

////////////////////////////////////////////////////////////Pin defination starts/////////////////////////////////////////////////////////

/*preserve pin_config [defined by pebri]
#define PIN_NUM_MOSI 25
#define PIN_NUM_CLK  23
#define PIN_NUM_CS   19
#define PIN_NUM_DC   22
#define PIN_NUM_RST  16
#define PIN_NUM_BCKL 5
*/

/*testing pin_config [defined by me]*/
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   19
#define PIN_NUM_DC   2
#define PIN_NUM_RST  4
#define PIN_NUM_BCKL 5

////////////////////////////////////////////////////////////Pin defination ends/////////////////////////////////////////////////////////



#define MADCTL_MY    0x80
#define MADCTL_MX    0x40
#define MADCTL_MV    0x20
#define MADCTL_ML    0x10
#define MADCTL_RGB   0x00
#define MADCTL_BGR   0x08
#define MADCTL_MH    0x04

#define TFT_CMD_SWRESET 0x01
#define TFT_CMD_SLEEP 0x11
#define TFT_CMD_DISPLAY_OFF 0x28

static const int DUTY_MAX = 0x1fff;
static const int LCD_BACKLIGHT_ON_VALUE = 1;
static spi_device_handle_t spi;

/*
 The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} tft_init_cmd_t;

static const tft_init_cmd_t tft_sleep_cmds[] = {
    {TFT_CMD_SWRESET, {0}, 0x80},
    {TFT_CMD_DISPLAY_OFF, {0}, 0x80},
    {TFT_CMD_SLEEP, {0}, 0x80},
    {0, {0}, 0xff}
};

static const tft_init_cmd_t tft_init_cmds[]={

    /* 
    //ST7735r
    {0x01, {0}, 0x80}, //SWRESET
    //   {0x00, {0}, 0x80}, //NOP delay?
    {0x11, {0}, 0x80}, //SLEEPOUT
    {0xB1, {0x01, 0x2C, 0x2D}, 3}, //FRMCTL1 porch padding
    {0xB2, {0x01, 0x2C, 0x2D}, 3}, //FRMCTL2 porch padding
    {0xB3, {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D}, 6}, //FRMCTL3 porch padding
    {0xB4, {0x07}, 1}, //INVCTR
    {0xC0, {0xA2, 0x02, 0x84}, 3}, //PWCTR1
    {0xC1, {0xC5}, 1}, //PWCTR2
    {0xC2, {0x0A, 0x00}, 2}, //PWCTR3
    {0xC3, {0x8A, 0x2A}, 2}, //PWCTR4
    {0xC4, {0x8A, 0xEE}, 2}, //PWCTR5
    {0xC5, {0x0E}, 1}, //VMCTR1
    {0x20, {0}, 0}, //INVOFF
    {0x36, {(MADCTL_MV)|(MADCTL_MX)}, 1}, // rotated 90 CW
    {0x3A, {0x05}, 1|0x80}, //COLMOD
    {0x2A, {0x00, 0x02, 0x00, 0x81}, 4}, //CASET
    {0x2B, {0x00, 0x01, 0x00, 0xA0}, 4}, //RASET
    //commented values are for blacktab, good luck!
    //{0xE0, {0x09, 0x16, 0x09, 0x20, 0x21, 0x1B, 0x13, 0x19, 0x17, 0x15, 0x1E, 0x2B, 0x04, 0x05, 0x02, 0x0E}, 16},
    //{0xE1, {0x0B, 0x14, 0x08, 0x1E, 0x22, 0x1D, 0x18, 0x1E, 0x1B, 0x1A, 0x24, 0x2B, 0x06, 0x06, 0x02, 0x0F}, 16},
    {0xE0, {0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10}, 16},
    {0xE1, {0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10}, 16},
    {0x13, {0}, 0x0}, //NORON
    {0x29, {0}, 0x80}, //DISPON
    {0, {0}, 0xff} 
    
    */

    /*
    //ST7789
    {0x01, {0}, 0x80}, //SWRESET
    //{0x00, {0}, 0x80}, //NOP delay?
    {0x11, {0}, 0x80}, //SLEEPOUT
    // {0x3A, {0x55}, 1|0x80}, //COLMOD
    //{0x36, {(MADCTL_MV)|(MADCTL_MX)}, 1}, // rotated 90 CW
    //{0x36, {(MADCTL_MX)}, 1}, // Set to Portrait Mode
    {0x21, {0}, 0}, //INVON
    {0x36, {MADCTL_MX | MADCTL_MY | TFT_RGB_BGR}, 1},
    {0x3A, {0x55}, 1|0x80}, //COLMOD
    {0x2A, {0x00, 0x00, 0x00, 0x7F}, 4}, //CASET
    {0x2B, {0x00, 0x00, 0x00, 0x7F}, 4}, //RASET
    // {0x21, {0}, 0}, //INVON
    {0x13, {0}, 0x0}, //NORON
    // {0x11, {0}, 0x80},
    {0x29, {0}, 0x80}, //DISPON
    {0, {0}, 0xff}

    */ 

    //GC9107 128*128
    {0x01, {0}, 0x80}, //SWRESET
    //   {0x00, {0}, 0x80}, //NOP delay?
    {0x11, {0}, 0x80}, //SLEEPOUT
    {0x3A, {0x55}, 1|0x80}, //COLMOD
    {0x36, {MADCTL_MX | MADCTL_MY | MADCTL_BGR}, 1},
    // {0x36, {MADCTL_MV | MADCTL_MY | TFT_RGB_BGR}, 1},
    {0x2A, {0x00, 0x00, 0x00, 0xF0}, 4}, //CASET
    {0x2B, {0x00, 0x00, 0x00, 0xF0}, 4}, //RASET
    {0x21, {0}, 0}, //INVON
    {0x13, {0}, 0x0}, //NORON
    {0x29, {0}, 0x80}, //DISPON
    {0, {0}, 0xff}


};

//Send a command to the ST7735R. Uses spi_device_transmit, which waits until the transfer is complete.
static void tft_cmd(spi_device_handle_t spi, const uint8_t cmd) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//Send data to the ST7735R. Uses spi_device_transmit, which waits until the transfer is complete.
static void tft_data(spi_device_handle_t spi, const uint8_t *data, int len) 
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
static void tft_spi_pre_transfer_callback(spi_transaction_t *t) 
{
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

static void backlight_init()
{
    //configure timer0
    ledc_timer_config_t ledc_timer;
    memset(&ledc_timer, 0, sizeof(ledc_timer));

    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT; //set timer counter bit number
    ledc_timer.freq_hz = 5000;              //set frequency of pwm
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;   //timer mode,
    ledc_timer.timer_num = LEDC_TIMER_0;    //timer index

    ledc_timer_config(&ledc_timer);

    //set the configuration
    ledc_channel_config_t ledc_channel;
    memset(&ledc_channel, 0, sizeof(ledc_channel));

    //set LEDC channel 0
    ledc_channel.channel = LEDC_CHANNEL_0;
    //set the duty for initialization.(duty range is 0 ~ ((2**duty_resolution)-1)
    ledc_channel.duty = (LCD_BACKLIGHT_ON_VALUE) ? 0 : DUTY_MAX;
    //GPIO number
    ledc_channel.gpio_num = PIN_NUM_BCKL;
    //GPIO INTR TYPE, as an example, we enable fade_end interrupt here.
    ledc_channel.intr_type = LEDC_INTR_FADE_END;
    //set LEDC mode, from ledc_mode_t
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    //set LEDC timer source, if different channel use one timer,
    //the frequency and duty_resolution of these channels should be the same
    ledc_channel.timer_sel = LEDC_TIMER_0;

    ledc_channel_config(&ledc_channel);

    //initialize fade service.
    ledc_fade_func_install(0);

    // duty range is 0 ~ ((2**duty_resolution)-1)
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (LCD_BACKLIGHT_ON_VALUE) ? DUTY_MAX : 0, 500);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);

    printf("Backlight initialization done.\n");
}

static void send_header_start(spi_device_handle_t spi, int xpos, int ypos, int w, int h)
{
    esp_err_t ret;
    int x;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[5];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. 
    for (x=0; x<5; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0]=0x2A;           //Column Address Set
    trans[1].tx_data[0]=xpos>>8;              //Start Col High
    trans[1].tx_data[1]=xpos;              //Start Col Low
    trans[1].tx_data[2]=(xpos+w-1)>>8;       //End Col High
    trans[1].tx_data[3]=(xpos+w-1)&0xff;     //End Col Low
    trans[2].tx_data[0]=0x2B;           //Page address set
    trans[3].tx_data[0]=ypos>>8;        //Start page high
    trans[3].tx_data[1]=ypos&0xff;      //start page low
    trans[3].tx_data[2]=(ypos+h-1)>>8;    //end page high
    trans[3].tx_data[3]=(ypos+h-1)&0xff;  //end page low
    trans[4].tx_data[0]=0x2C;           //memory write

    //Queue all transactions.
    for (x=0; x<5; x++) {
        ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret==ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}

static void send_header_cleanup(spi_device_handle_t spi) 
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    //Wait for all 5 transactions to be done and get back the results.
    for (int x=0; x<5; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        //We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}

//Initialize the display
static void tft_init(spi_device_handle_t spi) 
{
    int cmd=0;
    //Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    //Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //Send all the commands
    while (tft_init_cmds[cmd].databytes!=0xff) {
        ESP_LOGI("GC9107", "Sending command 0x%02X", tft_init_cmds[cmd].cmd);  // Debug log
        uint8_t dmdata[16];
        tft_cmd(spi, tft_init_cmds[cmd].cmd);
        //Need to copy from flash to DMA'able memory
        memcpy(dmdata, tft_init_cmds[cmd].data, 16);
        tft_data(spi, dmdata, tft_init_cmds[cmd].databytes&0x1F);
        if (tft_init_cmds[cmd].databytes&0x80) {
            ESP_LOGI("GC9107", "Waiting for delay...");
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    backlight_init();
}

void st7735rSetBrightness(int value)
{
    int duty = DUTY_MAX * (value * 0.01f);

    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty, 500);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
}

void st7735rSendFB(const uint16_t *fb, int xs, int ys, int w, int h) {
    esp_err_t ret;
    spi_transaction_t trans={0};
    spi_transaction_t *rtrans;
	send_header_start(spi, xs, ys, w, h);
	trans.length=w*h*16;
	trans.user=(void*)1;
	trans.tx_buffer=fb;
    ret=spi_device_queue_trans(spi, &trans, portMAX_DELAY);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
	send_header_cleanup(spi);
    ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
    assert(ret==ESP_OK);
}

void st7735rPowerDown()
{
    esp_err_t err = ESP_OK;
    
    // fade off backlight
    ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (LCD_BACKLIGHT_ON_VALUE) ? 0 : DUTY_MAX, 100);
    ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_WAIT_DONE);

    // Disable LCD panel
    int cmd = 0;
    while (tft_sleep_cmds[cmd].databytes != 0xff)
    {
        tft_cmd(spi, tft_sleep_cmds[cmd].cmd);
        tft_data(spi, tft_sleep_cmds[cmd].data, tft_sleep_cmds[cmd].databytes & 0x7f);
        if (tft_sleep_cmds[cmd].databytes & 0x80)
        {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }
}

void st7735rInit() {
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=-1,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=(160*128*2)+16
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=22000000,               //Clock out at 26 MHz. Yes, that's heavily overclocked.
        .mode=0,                                //SPI mode 0
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .queue_size=10,               //We want to be able to queue this many transfers
        .pre_cb=tft_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
    //Initialize the LCD
    tft_init(spi);
    printf("st7735r initialize done. \n");
}
