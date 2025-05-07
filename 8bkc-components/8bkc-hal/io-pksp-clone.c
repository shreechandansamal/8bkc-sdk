#include <string.h>
#include <stdio.h>
#include <sdkconfig.h>
#include "rom/ets_sys.h"
#include "soc/gpio_reg.h"
#include "soc/dport_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "io-pksp-clone.h"
#include "st7735r.h"
#include "esp_deep_sleep.h"
#include "driver/rtc_io.h"
#include "8bkc-hal.h" //for button codes


////////////////////////////////////////////////////////////Pin defination starts/////////////////////////////////////////////////////////

/*preserve pin_config for rest  [defined by pebri] 
//Buttons. Pressing a button pulls down the associated GPIO
#define GPIO_BTN_RIGHT (1<<12)
#define GPIO_BTN_LEFT ((uint64_t)1<<39)
#define GPIO_BTN_UP ((uint64_t)1<<34)
#define GPIO_BTN_DOWN ((uint64_t)1<<35)
#define GPIO_BTN_B (1<<27)
#define GPIO_BTN_A (1<<18)
#define GPIO_BTN_SELECT (1<<14)
#define GPIO_BTN_START (1<<13)
#define GPIO_BTN_PWR_PIN 32
#define GPIO_BTN_PWR ((uint64_t)1<<GPIO_BTN_PWR_PIN)
#define GPIO_SOUND_EN (1<<17)

#define GPIO_DAC (1<<26)
#define VBAT_ADC_CHAN ADC1_CHANNEL_0
#define GPIO_CHGSTDBY ((uint64_t)1UL<<33UL) //micro-usb plugged
*/

/*testing pin_config for rest [defined by me]*/
//Buttons. Pressing a button pulls down the associated GPIO
#define GPIO_BTN_RIGHT (1<<25)
#define GPIO_BTN_LEFT ((uint64_t)1<<33)   //needs ext_pull-up
#define GPIO_BTN_UP ((uint64_t)1<<34)     //needs ext_pull-up
#define GPIO_BTN_DOWN ((uint64_t)1<<35)   //needs ext_pull-up
#define GPIO_BTN_B ((uint64_t)1<<39)
#define GPIO_BTN_A (1<<0)
#define GPIO_BTN_SELECT (1<<12)
#define GPIO_BTN_START (1<<14)
#define GPIO_BTN_PWR_PIN 32
#define GPIO_BTN_PWR ((uint64_t)1<<GPIO_BTN_PWR_PIN)
// #define GPIO_SOUND_EN (1<<27)

//spaker
#define GPIO_DAC (1<<26)

#define VBAT_ADC_CHAN ADC1_CHANNEL_0//ADC1 channel << 36UL or VP 

#define GPIO_CHGDET (1<<15) //battery is charging, low-active
#define GPIO_CHGSTDBY ((uint64_t)1UL<<13UL) //micro-usb plugged


////////////////////////////////////////////////////////////Pin defination ends/////////////////////////////////////////////////////////




#define VBATMEAS_HISTCT 16

int vbatHist[VBATMEAS_HISTCT]={0};
int vbatHistPos=0;

int ioGetVbatAdcVal() {
	int sum=0;
	for (int i=0; i<VBATMEAS_HISTCT; i++) {
		if (vbatHist[i]==0) {
			//Measured less than VBATMEAS_HISTCT values
			return (i==0)?0:sum/i;
		}
		sum+=vbatHist[i];
	}
	return sum/VBATMEAS_HISTCT;
}

void ioVbatForceMeasure() {	
	for (int i=0; i<VBATMEAS_HISTCT; i++) {
		vTaskDelay(1);
		vbatHist[i]=adc1_get_raw(VBAT_ADC_CHAN);
	}
}

int ioGetChgStatus() {
	uint64_t io=((uint64_t)GPIO.in1.data<<32)|GPIO.in;
	if ((io&GPIO_CHGSTDBY)==0) {
		return IO_CHG_NOCHARGER;
	} else {
		//if ((io&GPIO_CHGDET)==0) {
			return IO_CHG_CHARGING;
		//} else {
		//	return IO_CHG_FULL;
		//}
	}
}

int ioJoyReadInput() {
	int i=0;
	static int initial=1;
	static int powerWasPressed=0;
	static uint32_t powerPressedTime;
	uint64_t io=((uint64_t)GPIO.in1.data<<32)|GPIO.in;
	static uint64_t last=0xffffffff;

	//There's some weirdness with the select button... I see dips of about 10uS in the signal. May be caused by 
	//some other hardware EMC'ing power into that line... need to research a bit more.
	//For now, here's a quick and dirty hack to fix it: if the select button was not pressed but is now, wait 12uS
	//(long enough for the glitch to pass) and re-sample.
	if (last&GPIO_BTN_SELECT) {
		if (!(io&GPIO_BTN_SELECT)) {
			ets_delay_us(12);
			io=((uint64_t)GPIO.in1.data<<32)|GPIO.in;
		}
	}
	last=io;

	//Ignore remnants from 1st power press
	if ((io&GPIO_BTN_PWR)) {
		if (!initial) {
			i|=KC_BTN_POWER;
			if (!powerWasPressed) powerPressedTime=xTaskGetTickCount();
			if ((xTaskGetTickCount()-powerPressedTime)>(1500/portTICK_PERIOD_MS)) {
				i|=KC_BTN_POWER_LONG;
			}
			powerWasPressed=1;
		}
	} else {
		initial=0;
		powerWasPressed=0;
	}
	if (!(io&GPIO_BTN_RIGHT)) i|=KC_BTN_RIGHT;
	if (!(io&GPIO_BTN_LEFT)) i|=KC_BTN_LEFT;
	if (!(io&GPIO_BTN_UP)) i|=KC_BTN_UP;
	if (!(io&GPIO_BTN_DOWN)) i|=KC_BTN_DOWN;
	if (!(io&GPIO_BTN_SELECT)) i|=KC_BTN_SELECT;
	if (!(io&GPIO_BTN_START)) i|=KC_BTN_START;
	if (!(io&GPIO_BTN_A)) i|=KC_BTN_A;
	if (!(io&GPIO_BTN_B)) i|=KC_BTN_B;
//	printf("%x\n", i);
	return i;
}

void ioOledPowerDown() {
	st7735rPowerDown();
}

void ioPowerDown() {
	printf("PowerDown: wait till power btn is released...\n");
	while(1) {
		uint64_t io=((uint64_t)GPIO.in1.data<<32)|GPIO.in;
		vTaskDelay(50/portTICK_PERIOD_MS);
		if (!(io&GPIO_BTN_PWR)) break;
	}
	//debounce
	vTaskDelay(200/portTICK_PERIOD_MS);

//	esp_deep_sleep_enable_ext1_wakeup(GPIO_BTN_PWR|GPIO_CHGSTDBY, ESP_EXT1_WAKEUP_ANY_HIGH);
	esp_deep_sleep_enable_ext0_wakeup(GPIO_BTN_PWR_PIN, 1);
//	esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
	esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);

	printf("PowerDown: esp_deep_sleep_start.\n");
	esp_deep_sleep_start();
	printf("PowerDown: after deep_sleep_start, huh?\n");
	while(1);
}


/*in my current circuit no Amplifier is available so disabled
  if in future you want enable them the uncomment all the ioAmplifierOn and ioAmplifierOff */
// void ioAmplifierOn(){
// 	WRITE_PERI_REG(GPIO_OUT_W1TS_REG, GPIO_SOUND_EN);
// 	vTaskDelay(20 / portTICK_PERIOD_MS);
// }

// void ioAmplifierOff(){
// 	WRITE_PERI_REG(GPIO_OUT_W1TC_REG, GPIO_SOUND_EN);
// 	vTaskDelay(20 / portTICK_PERIOD_MS);
// }

void ioInit() {
	gpio_config_t io_conf[]={
	{
		.intr_type=GPIO_INTR_DISABLE,
		.mode=GPIO_MODE_INPUT,
		.pull_up_en=1,
		.pin_bit_mask=GPIO_BTN_RIGHT|GPIO_BTN_LEFT|GPIO_BTN_UP|GPIO_BTN_DOWN|GPIO_BTN_B|GPIO_BTN_A|GPIO_BTN_SELECT|GPIO_BTN_START|GPIO_CHGSTDBY
	},
	// {
	// 	.intr_type=GPIO_INTR_DISABLE,
	// 	.mode=GPIO_MODE_OUTPUT,
	// 	.pin_bit_mask=GPIO_SOUND_EN
	// },
	{
		.intr_type=GPIO_INTR_DISABLE,
		.mode=GPIO_MODE_INPUT,
		.pull_down_en=1,
		.pin_bit_mask=GPIO_BTN_PWR|GPIO_CHGSTDBY
	}
	};
	WRITE_PERI_REG(RTC_IO_XTAL_32K_PAD_REG, 0);

	for (int x=0; x<3; x++) {
		gpio_config(&io_conf[x]);
	}

	//Initialize battery voltage ADC
	//We double-use /CS as the voltage measurement pin.
	adc1_config_width(ADC_WIDTH_12Bit);
	adc1_config_channel_atten(VBAT_ADC_CHAN, ADC_ATTEN_11db);

	GPIO.func_out_sel_cfg[GPIO_NUM_36].oen_inv_sel=1;
	SET_PERI_REG_MASK(rtc_gpio_desc[GPIO_NUM_36].reg, (rtc_gpio_desc[GPIO_NUM_36].mux));

	st7735rInit();
}