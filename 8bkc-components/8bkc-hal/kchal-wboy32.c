#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2s.h"
#include "esp_deep_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "soc/rtc_cntl_reg.h"
#include "8bkc-hal.h"
#include "spi_lcd.h"
#include "appfs.h"
#include "8bkc-ugui.h"
#include "ugui.h"
#include "psxcontroller.h"

SemaphoreHandle_t oledMux;
SemaphoreHandle_t configMux;

//Buttons. Pressing a button pulls down the associated GPIO
#define GPIO_BTN_RIGHT ((uint64_t)1<<32)
#define GPIO_BTN_LEFT (1<<17)
#define GPIO_BTN_UP ((uint64_t)1<<33)
#define GPIO_BTN_DOWN (1<<27)
#define GPIO_BTN_B ((uint64_t)1<<35)
#define GPIO_BTN_A ((uint64_t)1<<34)
#define GPIO_BTN_SELECT ((uint64_t)1<<39)
#define GPIO_BTN_START (1<<23)

//The hardware size of the display.
#define OLED_REAL_H 320
#define OLED_REAL_W 240

//Bit used as pocketsprite screen
#define OLED_FAKE_XOFF ((OLED_REAL_W-KC_SCREEN_W)/2)
#define OLED_FAKE_W KC_SCREEN_W
#define OLED_FAKE_YOFF ((OLED_REAL_H-KC_SCREEN_H)/2)
#define OLED_FAKE_H KC_SCREEN_H

typedef struct {
	uint8_t volume;
	uint8_t brightness;
} ConfVars;
#define VOLUME_KEY "vol"
#define BRIGHTNESS_KEY "con" //backwards compatible; used to be misnamed 'contrast'
#define BATFULLADC_KEY "batadc"

static ConfVars config, savedConfig;
static QueueHandle_t soundQueue;
static int soundRunning=0;
static nvs_handle nvsHandle=NULL, nvsAppHandle=NULL;

int kchal_get_hw_ver() {
	return -1; //fake
}

static void flushConfigToNvs() {
	//Check if anything changed
	if (memcmp(&config, &savedConfig, sizeof(config))==0) return;
	if (config.volume!=savedConfig.volume) nvs_set_u8(nvsHandle, VOLUME_KEY, config.volume);
	if (config.brightness!=savedConfig.brightness) nvs_set_u8(nvsHandle, BRIGHTNESS_KEY, config.brightness);
	//Okay, we're up to date again
	memcpy(&savedConfig, &config, sizeof(config));
	nvs_commit(nvsHandle);
}


void kchal_cal_adc() {
	//stub
}

int kchal_get_bat_mv() {
	return 3600; //stub
}

int kchal_get_bat_pct() {
	return 100;
}

static uint32_t orig_store0_reg=0xFFFFFFFF;

uint32_t kchal_rtc_reg_bootup_val() {
	if (orig_store0_reg==0xFFFFFFFF) {
		return REG_READ(RTC_CNTL_STORE0_REG);
	} else {
		return orig_store0_reg;
	}
}

static volatile uint16_t buttons=0xff;

void fakeCustomKeyInit(){
	printf("Using custom buttons for input.\n");
	gpio_config_t io_conf={
		.intr_type=GPIO_INTR_DISABLE,
		.mode=GPIO_MODE_INPUT,
		.pull_up_en=1,
		// .pin_bit_mask=((1<<CONFIG_HW_RIGHT_BUTTON)|(1<<CONFIG_HW_LEFT_BUTTON)|(1<<CONFIG_HW_UP_BUTTON)|(1<<CONFIG_HW_DOWN_BUTTON)|(1<<CONFIG_HW_B_BUTTON)|(1<<CONFIG_HW_A_BUTTON)|(1<<CONFIG_HW_SELECT_BUTTON)|(1<<CONFIG_HW_START_BUTTON))
		.pin_bit_mask=(GPIO_BTN_RIGHT
			| GPIO_BTN_LEFT
			| GPIO_BTN_UP
			| GPIO_BTN_DOWN
			| GPIO_BTN_B
			| GPIO_BTN_A
			| GPIO_BTN_SELECT
			| GPIO_BTN_START)
	};
	gpio_config(&io_conf);
}


//User Custom buttons as input
static void kchal_mgmt_task(void *args) {
	fakeCustomKeyInit();
	printf("Using custom buttons for input.\n");
	int b;
	while(1) {
		b=0;
		uint64_t io=((uint64_t)GPIO.in1.data<<32)|GPIO.in;

		if (!(io&GPIO_BTN_RIGHT)) b|=KC_BTN_RIGHT;
		if (!(io&GPIO_BTN_LEFT))  b|=KC_BTN_LEFT;
		if (!(io&GPIO_BTN_UP))    b|=KC_BTN_UP;
		if (!(io&GPIO_BTN_DOWN))  b|=KC_BTN_DOWN;
		
		if (!(io&GPIO_BTN_SELECT)) b|=KC_BTN_SELECT;
		if (!(io&GPIO_BTN_START))  b|=KC_BTN_START;
		if (!(io&GPIO_BTN_A))      b|=KC_BTN_A;
		if (!(io&GPIO_BTN_B))      b|=KC_BTN_B;
		
	    printf("%02x\n",b);
		buttons=b;
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}

#define INIT_HW_DONE 1
#define INIT_SDK_DONE 2
#define INIT_COMMON_DONE 4
static int initstate=0;

static void kchal_init_common() {
	initstate|=INIT_COMMON_DONE;
}

void kchal_init_hw(int flags) {
	if (initstate&INIT_HW_DONE) return; //already did this
	oledMux=xSemaphoreCreateMutex();
	configMux=xSemaphoreCreateMutex();
	//Route DAC
	i2s_set_pin(0, NULL);
	i2s_set_dac_mode(I2S_DAC_CHANNEL_LEFT_EN);
	//I2S enables *both* DAC channels; we only need DAC2. Do some Deeper Magic to make this into
	//an essentially uninitialized GPIO pin again.
	CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_DAC_XPD_FORCE_M);
	CLEAR_PERI_REG_MASK(RTC_IO_PAD_DAC1_REG, RTC_IO_PDAC1_XPD_DAC_M);
	gpio_config_t io_conf={
		.intr_type=GPIO_INTR_DISABLE,
		.mode=GPIO_MODE_INPUT,
		.pull_up_en=1,
		.pin_bit_mask=(1<<25)
	};
	gpio_config(&io_conf);

	//Initialize display
	spi_lcd_init();
	//Clear entire OLED screen
	uint16_t *fb=malloc(OLED_REAL_W*2);
	assert(fb);
	memset(fb, 0, OLED_REAL_W*2);
	for (int y=0; y<OLED_REAL_H; y++) {
		spi_lcd_send(0, y, OLED_REAL_W, 1, fb);
	}
	free(fb);
	initstate|=INIT_HW_DONE;
	if (initstate==(INIT_HW_DONE|INIT_SDK_DONE)) kchal_init_common();
}

void kchal_init_sdk(int flags) {
	esp_err_t r;
	if (initstate&INIT_SDK_DONE) return; //already did this
	//Hack: This initializes a bunch of locks etc; that process uses a bunch of locks. If we do not
	//do it here, it happens in the mgmt task, which is somewhat stack-starved.
	esp_get_deep_sleep_wake_stub();

#if 0
 //no appfs for fake pocketsprite for now?
	//Init appfs
	esp_err_t r=appfsInit(0x43, 3);
	assert(r==ESP_OK);
	printf("Appfs inited.\n");
#endif
	//Grab relevant nvram variables
	r=nvs_flash_init();
	if (r!=ESP_OK) {
		printf("Warning: NVS init failed!\n");
	}
	printf("NVS inited\n");

	//Default values
	config.volume=128;
	config.brightness=192;
	r=nvs_open("8bkc", NVS_READWRITE, &nvsHandle);
	if (r==ESP_OK) {
		nvs_get_u8(nvsHandle, VOLUME_KEY, &config.volume);
		nvs_get_u8(nvsHandle, BRIGHTNESS_KEY, &config.brightness);
		memcpy(&savedConfig, &config, sizeof(config));
	}

	//We don't have appfs, so we can't deduce the app name. Just assume a generic appname for nvs.
	char *name="MyApp";
	printf("Opening NVS storage for app %s\n", name);
	r=nvs_open(name, NVS_READWRITE, &nvsAppHandle);
	if (r!=ESP_OK) {
		printf("Opening app NVS storage failed!\n");
	}

	xTaskCreatePinnedToCore(&kchal_mgmt_task, "kchal", 1024*4, NULL, 5, NULL, 0);
	initstate|=INIT_SDK_DONE;
	if (initstate==(INIT_HW_DONE|INIT_SDK_DONE)) kchal_init_common();
}

void kchal_init() {
	kchal_init_hw(0);
	kchal_init_sdk(0);
}

uint32_t kchal_get_keys() {
	return buttons;
}

void kchal_wait_keys_released() {
	uint32_t keys=kchal_get_keys();
	while (kchal_get_keys() & keys) {
		vTaskDelay(10);
	}
}

void kchal_send_fb(const uint16_t *fb) {
	xSemaphoreTake(oledMux, portMAX_DELAY);
	spi_lcd_send(OLED_FAKE_XOFF, OLED_FAKE_YOFF, OLED_FAKE_W, OLED_FAKE_H, fb);
	xSemaphoreGive(oledMux);
}

void kchal_send_fb_partial(const uint16_t *fb, int x, int y, int w, int h) {
	if (w<=0 || h<=0) return;
	if (x<0 || x+w>OLED_FAKE_W) return;
	if (y<0 || y+h>OLED_FAKE_H) return;
	xSemaphoreTake(oledMux, portMAX_DELAY);
	spi_lcd_send(x+OLED_FAKE_XOFF, y+OLED_FAKE_YOFF, w, h, fb);
	xSemaphoreGive(oledMux);
}


void kchal_set_volume(uint8_t new_volume) {
	xSemaphoreTake(configMux, portMAX_DELAY);
	config.volume=new_volume;
	xSemaphoreGive(configMux);
}

uint8_t kchal_get_volume() {
	return config.volume;
}

void kchal_set_brightness(int brightness) {
	xSemaphoreTake(configMux, portMAX_DELAY);
	config.brightness=brightness;
	xSemaphoreGive(configMux);
}

uint8_t kchal_get_brightness(int brightness) {
	return config.brightness;
}

void kchal_sound_start(int rate, int buffsize) {
	i2s_config_t cfg={
		.mode=I2S_MODE_DAC_BUILT_IN|I2S_MODE_TX|I2S_MODE_MASTER,
		.sample_rate=rate,
		.bits_per_sample=16,
		.channel_format=I2S_CHANNEL_FMT_RIGHT_LEFT,
		.communication_format=I2S_COMM_FORMAT_I2S_MSB,
		.intr_alloc_flags=0,
		.dma_buf_count=4,
		.dma_buf_len=buffsize/4
	};
	i2s_driver_install(0, &cfg, 4, &soundQueue);
	i2s_set_sample_rates(0, cfg.sample_rate);
	soundRunning=1;
}

void kchal_sound_mute(int doMute) {
	if (doMute) {
		dac_i2s_disable();
	} else {
		dac_i2s_enable();
	}
}

void kchal_sound_stop() {
	i2s_driver_uninstall(0);
}

#define SND_CHUNKSZ 32
void kchal_sound_push(uint8_t *buf, int len) {
	uint32_t tmpb[SND_CHUNKSZ];
	int i=0;
	while (i<len) {
		int plen=len-i;
		if (plen>SND_CHUNKSZ) plen=SND_CHUNKSZ;
		for (int j=0; j<plen; j++) {
			int s=((((int)buf[i+j])-128)*config.volume); //Make [-128,127], multiply with volume
			s=(s>>8)+128; //divide off volume max, get back to [0-255]
			if (s>255) s=255;
			if (s<0) s=0;
			tmpb[j]=((s)<<8)+((s)<<24);
		}
		i2s_write_bytes(0, (char*)tmpb, plen*4, portMAX_DELAY);
		i+=plen;
	}
}

void kchal_power_down() {
	printf("Powerdown not implemented on fake hardware. Aborting!\n");
	abort();
}

void kchal_exit_to_chooser() {
	printf("Exit to chooser not implemented on fake hardware. Aborting!\n");
	abort();
}

int kchal_get_chg_status() {
	return KC_CHG_NOCHARGER;
}

void kchal_set_new_app(int fd) {
	//Stub: no chooser
}

int kchal_get_new_app() {
	//Stub: no chooser
	return -1;
}

void kchal_boot_into_new_app() {
	printf("ERROR: No chooser in fake hardware!\n");
	abort();
}

nvs_handle kchal_get_app_nvsh() {
	return nvsAppHandle;
}

