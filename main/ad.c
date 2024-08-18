/*
 * ad.c
 *
 *  Created on: Oct 31, 2023
 *      Author: Ouail
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/adc.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "ad.h"

#define ESP_LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

#define TAG "ad"

#define MAX_CHANNELS (2)

#define STACK_SIZE 4096
#define TASK_DELAY_MS 100
#define MIN_TEMP 0
#define MAX_TEMP 100
#define MOVING_AVG_SIZE 10
#define MOVING_HYST_DELTA 50

#define K_D0 "d0"
#define K_D1 "d1"
#define K_T0 "t0"
#define K_T1 "t1"
#define K_TGA "tga"
#define DEF_D0 (0)
#define DEF_D1 (4096)
#define DEF_T0 (10)  //Cannot be float!
#define DEF_T1 (100) //Cannot be float!

#if DEF_D1-DEF_D0==0
#error DEF_D1-DEF_D0 is 0!
#endif

const char *KEYD0[MAX_CHANNELS]={"d00","d01","d02"};
const char *KEYD1[MAX_CHANNELS]={"d10","d11","d12"};
const char *KEYT0[MAX_CHANNELS]={"t00","t01","t02"};
const char *KEYT1[MAX_CHANNELS]={"t10","t11","t12"};
const char *KEYTGA[MAX_CHANNELS]={"TGA0","TGA1","TGA2"};


const float DEF_TGA=((float)(((float)DEF_T1)-((float)DEF_T0))/(((float)DEF_D1)-((float)DEF_D0)));

#if DEF_D1 <= DEF_D0
#error DEF_D1 is less or equal than DEF_D0
#endif

#if DEF_T1 <= DEF_T0
#error DEF_T1 is less or equal than DEF_T0
#endif

typedef struct {
	uint16_t hyst_min;
	uint16_t hyst_max;
	uint16_t hyst_delta;
} moving_hyst_t;

typedef struct {
	uint16_t queue[MOVING_AVG_SIZE];
	uint8_t idx;
} moving_avg_t;

typedef struct {
	uint16_t d0;
	uint16_t d1;
	float t0;
	float t1;
	float tga;
	uint8_t calibrated;
} calibration_t;

typedef struct {
	 uint8_t running;
	moving_hyst_t moving_hyst;
	moving_avg_t moving_avg;
	calibration_t calibration;
	uint16_t raw;
	uint16_t normalized;
	float temperature;
} ad_struct;

#define MAX(a,b) ((a)>(b)?(a):(b))

#define MIN(a,b) ((a)<(b)?(a):(b))

//!!! convert ad_channel[MAX_CHANNELS] to array of ad_channel[MAX_CHANNELS]s
static ad_struct ad_channel[MAX_CHANNELS];
static TaskHandle_t ad_tsk;
static SemaphoreHandle_t ad_sem;


//!!! Create an array contains de ad channel names for each channels
const adc_channel_t array_channels[MAX_CHANNELS]={
	
ADC1_CHANNEL_7,ADC1_CHANNEL_6 // 0 1 

};


static void register_cmd();
BaseType_t check_channel(uint8_t ch){

if(ch>MAX_CHANNELS){
ESP_LOGE(TAG,"more than allowed channels");
return pdFAIL;

}
return pdPASS;
}

//!!! add channel index to as parameter
static uint16_t get_hyst(uint16_t raw, uint8_t ch , uint16_t *res) {
	if(check_channel(ch)!=pdPASS) return pdFAIL;
	//!!! Use channel index
	if (raw>ad_channel[ch].moving_hyst.hyst_max) { // out of range in top direction
		ad_channel[ch].moving_hyst.hyst_max = MIN(AD_MAX,
						ad_channel[ch].moving_hyst.hyst_max+ad_channel[ch].moving_hyst.hyst_delta);
		ad_channel[ch].moving_hyst.hyst_min = MIN(AD_MAX,
						ad_channel[ch].moving_hyst.hyst_min+ad_channel[ch].moving_hyst.hyst_delta);
		return ad_channel[ch].moving_hyst.hyst_max;
	}
	if (raw<ad_channel[ch].moving_hyst.hyst_min) {  //out of range in bottom direction
		ad_channel[ch].moving_hyst.hyst_min = MAX(AD_MIN,
						ad_channel[ch].moving_hyst.hyst_min-ad_channel[ch].moving_hyst.hyst_delta);
		ad_channel[ch].moving_hyst.hyst_max = MAX(AD_MIN,
						ad_channel[ch].moving_hyst.hyst_max-ad_channel[ch].moving_hyst.hyst_delta);
		return ad_channel[ch].moving_hyst.hyst_min;
	}
	*res=raw;
	return *res ;
}
static uint16_t get_avg(uint16_t input, uint8_t ch, uint16_t *res) {
	if(check_channel(ch)!=pdPASS) return pdFAIL;
	ad_channel[ch].moving_avg.queue[ad_channel[ch].moving_avg.idx]=input;
	ad_channel[ch].moving_avg.idx=(ad_channel[ch].moving_avg.idx+1)%MOVING_AVG_SIZE;
	uint32_t sum=0;
	for(uint8_t i=0; i<MOVING_AVG_SIZE;++i)
		sum+=ad_channel[ch].moving_avg.queue[i];

	*res=sum/MOVING_AVG_SIZE;
return *res;
}

static void fn_ad(void *p) {
	ESP_LOGI(TAG,"ADC task started");
	for(;;) {
		for(int i=0;i<MAX_CHANNELS;i++){
		if (ad_channel[i].running) {
			
			ad_channel[i].raw=adc1_get_raw(array_channels[i]);
			if (xSemaphoreTake(ad_sem, pdMS_TO_TICKS(5))) {
				
				ad_channel[i].normalized=get_avg(get_hyst(ad_channel[i].raw,i,&ad_channel[i].normalized),i,&ad_channel[i].normalized);
				ad_channel[i].temperature=ad_channel[i].normalized*ad_channel[i].calibration.tga;
				xSemaphoreGive(ad_sem);
			}
			ESP_LOGI(TAG,"raw:%d, normalized:%d ", ad_channel[i].raw, ad_channel[i].normalized);
		}}
		vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
	}
}



//!!! pass channel index to function & use it
static inline int validate_calibration(uint8_t ch) {
	if(check_channel(ch)!=pdPASS) return 0;
	int res=1;
	if (ad_channel[ch].calibration.d0>=ad_channel[ch].calibration.d1) {
		ad_channel[ch].calibration.d0=DEF_D0;
		ad_channel[ch].calibration.d1=DEF_D1;
		res=0;
	}

	if (ad_channel[ch].calibration.t0>=ad_channel[ch].calibration.t1) {
		ad_channel[ch].calibration.t0=DEF_T0;
		ad_channel[ch].calibration.t1=DEF_T1;
		res=0;
	}

	float tga=(ad_channel[ch].calibration.t1-ad_channel[ch].calibration.t0)/
			  (ad_channel[ch].calibration.d1-ad_channel[ch].calibration.d0);


if (abs((tga-ad_channel[ch].calibration.tga))>0.01) {
		ad_channel[ch].calibration.tga=DEF_TGA;
		res=0;
	}



	

	return res;
}

//!!! pass channel index to function & use it
static BaseType_t restore_cal_from_flash(uint8_t ch) {
if(check_channel(ch)!=pdPASS) return pdFAIL;
	ESP_LOGI(TAG, "Enter restore_cal_from_flash");
	nvs_handle_t handle;
	esp_err_t res=nvs_open(TAG, NVS_READWRITE, &handle);
	if(res!=ESP_OK)
		return pdFAIL;

	if (ad_channel[ch].calibration.calibrated)
		return pdTRUE;

	res=nvs_get_u16(handle, KEYD0[ch], &ad_channel[ch].calibration.d0);
	if (res!=ESP_OK)
		ad_channel[ch].calibration.d0=DEF_D0;

	res=nvs_get_u16(handle, KEYD1[ch], &ad_channel[ch].calibration.d1);
	if (res!=ESP_OK)
		ad_channel[ch].calibration.d1=DEF_D1;

	uint16_t t;
	res=nvs_get_u16(handle,KEYT0[ch], &t);
	ad_channel[ch].calibration.t0=res==ESP_OK ? t/100.0 : DEF_T0;

	res=nvs_get_u16(handle, KEYT1[ch], &t);
	ad_channel[ch].calibration.t1=res==ESP_OK ? t/100.0 : DEF_T1;


	res=nvs_get_u16(handle, KEYTGA[ch], &t);
	ad_channel[ch].calibration.tga=res==ESP_OK ? t/100.0 : DEF_TGA;

	ad_channel[ch].calibration.calibrated=validate_calibration(ch);

	nvs_close(handle);
	return pdTRUE;
}

#define CHECK_RES(res,msg)     	\
	do {				   		\
		if (res!=ESP_OK) { 		\
			ESP_LOGE(TAG, msg); \
			return pdFAIL;		\
		}						\
	} while(0)


//!!! pass channel index to function & use it
static BaseType_t save_cal_to_flash(uint8_t ch) {
	
	if(check_channel(ch)!=pdPASS) return pdFAIL;
	ESP_LOGI(TAG, "Enter save_cal_to_flash");
	nvs_handle_t handle;
	esp_err_t res=nvs_open(TAG, NVS_READWRITE, &handle);
	if(res!=ESP_OK) {
		ESP_LOGE(TAG, "Cannot open flash for write");
		return pdFAIL;
	}

	res=nvs_set_u16(handle, KEYD0[ch], ad_channel[ch].calibration.d0);
	CHECK_RES(res, "Cannot write d0");

	res=nvs_set_u16(handle, KEYD1[ch], ad_channel[ch].calibration.d1);
	CHECK_RES(res, "Cannot write d1");

	res=nvs_set_u16(handle, KEYT0[ch], ad_channel[ch].calibration.t0*100);
	CHECK_RES(res, "Cannot write t0");

	res=nvs_set_u16(handle, KEYT1[ch], ad_channel[ch].calibration.t1*100);
	CHECK_RES(res, "Cannot write t1");

	res=nvs_set_u16(handle, KEYTGA[ch], ad_channel[ch].calibration.tga*100);
	CHECK_RES(res, "Cannot write tga");

	nvs_close(handle);
	return pdTRUE;
}

BaseType_t ad_init(){
	esp_log_level_set(TAG, ESP_LOG_LOCAL_LEVEL);
	ad_sem=xSemaphoreCreateBinary();
	configASSERT(ad_sem);
	xSemaphoreGive(ad_sem);

	bzero(&ad_channel, sizeof(ad_struct));

	//!!! Init every channels
	adc1_config_width(ADC_WIDTH_BIT_12);
	for(int ch=0;ch<MAX_CHANNELS;ch++){
	adc1_config_channel_atten(array_channels[ch], ADC_ATTEN_DB_6);
	uint16_t ad=adc1_get_raw(array_channels[ch]);
	ad_channel[ch].moving_hyst.hyst_max=ad+MOVING_HYST_DELTA;
	ad_channel[ch].moving_hyst.hyst_min=ad-MOVING_HYST_DELTA;
	ad_channel[ch].moving_hyst.hyst_delta=MOVING_HYST_DELTA;
	for(int i=0;i<MOVING_AVG_SIZE;++i)
		ad_channel[ch].moving_avg.queue[i]=ad;

	ad_channel[ch].running=0;
	restore_cal_from_flash(ch);
	}
	

	register_cmd();
	configASSERT(xTaskCreate(fn_ad, "adc", STACK_SIZE, NULL, uxTaskPriorityGet(NULL), &ad_tsk));
	return pdPASS;
}

//!!! pass channel index to function & use it
BaseType_t ad_get(uint16_t *value, TickType_t ticks,uint8_t ch){
	if(check_channel(ch)!=pdPASS) return pdFAIL;
	if(!value || !ad_channel[ch].running)
		return pdFAIL;

	if (xSemaphoreTake(ad_sem, ticks)) {
		*value=ad_channel[ch].normalized;
		xSemaphoreGive(ad_sem);
		return pdPASS;
	}
	return pdFAIL;
}

//!!! pass channel index to function & use it
BaseType_t ad_get_temperature(float *value, TickType_t ticks,uint8_t ch) {
if(check_channel(ch)!=pdPASS) return pdFAIL;
	if(!value || !ad_channel[ch].running)
		return pdFAIL;

	if (xSemaphoreTake(ad_sem, ticks)) {
		*value=ad_channel[ch].temperature;
		xSemaphoreGive(ad_sem);
		return pdPASS;
	}
	return pdFAIL;
}

BaseType_t ad_deinit(uint8_t ch){
	if(check_channel(ch)!=pdPASS) return pdFAIL;
	ad_channel[ch].running=0;
	vTaskDelete(ad_tsk);
	vSemaphoreDelete(ad_sem);
	ESP_LOGI(TAG, "ADC deinit");
	return pdTRUE;
}

/*
 * !!! add channel index to every commands like --channel or -l
 ad --help
 ad --cal -0 <float>
 ad --cal -1 <float>
 ad --save
 ad --restore
 ad --state
 ad --start
 ad --stop
 ad --ch <ch_num
 */

#define CMD "ad"

static void help() {
	printf("help\r\n");
}

//!!! pass channel index to function & use it
static void status(uint8_t ch ) {
if(check_channel(ch)!=pdPASS) return ;
	printf("ad_channel[ch].running:%s raw value:%d, normalized:%d\r\n",
			ad_channel[ch].running?"true":"false", ad_channel[ch].raw, ad_channel[ch].normalized);
	printf("hysteresis min:%d, max:%d, delta:%d\r\n",
			ad_channel[ch].moving_hyst.hyst_min,
			ad_channel[ch].moving_hyst.hyst_max,
			ad_channel[ch].moving_hyst.hyst_delta);
	printf("moving average: index:%d, queue\r\n", ad_channel[ch].moving_avg.idx);
	for(int i=0; i<MOVING_AVG_SIZE; i++)
		printf("%d:%d\r\n", i, ad_channel[ch].moving_avg.queue[i]);

	printf("calibration: calibrated:%s, d0:%d, d1:%d, t0:%f, t1:%f, tga:%f\r\n",
			ad_channel[ch].calibration.calibrated?"true":"false",
			ad_channel[ch].calibration.d0, ad_channel[ch].calibration.d1,
			ad_channel[ch].calibration.t0, ad_channel[ch].calibration.t1, ad_channel[ch].calibration.tga);

			


}

//!!! pass channel index to function & use it
static void calibrate(int tempidx, float value,uint8_t ch) {
if(check_channel(ch)!=pdPASS) return;
	if (tempidx<0||tempidx>1||value<MIN_TEMP||value>MAX_TEMP) {
		printf("calibrate param error tempidx:%d, value:%f\r\n", tempidx, value);
		return;
	}
	if (tempidx==0) {
		ad_channel[ch].calibration.t0=value;
		ad_channel[ch].calibration.d0=adc1_get_raw(array_channels[ch]);
	}
	else {
		ad_channel[ch].calibration.t1=value;
		ad_channel[ch].calibration.d1=adc1_get_raw(array_channels[ch]);
	}
}

static struct {
	struct arg_lit *help;
	struct arg_lit *cal;
	struct arg_dbl *t0;
	struct arg_dbl *t1;
	struct arg_lit *save;
	struct arg_lit *restore;
	struct arg_lit *state;
	struct arg_lit *start;
	struct arg_lit *stop;
	struct arg_int *channel;
	struct arg_end *end;
	
} ad_args;

//!!!get&use channel index
static int cmd_ad(int argc, char **argv) {
 	 
	 
	int errors=arg_parse(argc, argv, (void **) &ad_args);
	if (errors) {
		help();
		return 1;
	}

static	uint8_t ch=0;

if(ad_args.channel->count){

	 ch=ad_args.channel->ival[0];
	if(ch>=MAX_CHANNELS) 
	{
		printf("ERROR in CHANNLS");
	return 1;
	}
	return 0;
	}

	if (ad_args.state->count) {
		status(ch);
		return 0;
	}

	if (ad_args.start->count) {
		ad_channel[ch].running=1;
		printf("AD conversion starting...");

		return 0;
	}


	
	if (ad_args.stop->count) {
		
		ad_channel[ch].running=0;
		printf("AD conversion stopping...");
		return 0;
	}

	if (ad_args.save->count) {
		return save_cal_to_flash(ch)==pdTRUE?0:1;
	}

	if (ad_args.restore->count) {
		return restore_cal_from_flash(ch)==pdTRUE?0:1;
	}

	if (ad_args.cal->count) {
		if (ad_args.t0->count) {
			calibrate(0, ad_args.t0->dval[0],ch);
			return 0;
		}
		if (ad_args.t1->count) {
			calibrate(1, ad_args.t1->dval[0],ch);
			return 0;
		}
		printf("Unknown index, 0 or 1 accepted only\r\n");
		return 1;
	}

	help();
	return 1;
}

static void register_cmd() {
	ad_args.help=arg_lit0("hH", "help", "help for ad");
	ad_args.cal=arg_lit0("cC", "cal", "Calibration");
	ad_args.t0=arg_dbl1("0", "t0", "<n>", "t0 temp value");
	ad_args.t1=arg_dbl1("1", "t1", "<n>", "t1 temp value");
	ad_args.save=arg_lit0("vV", "save", "save calibration to flash");
	ad_args.restore=arg_lit0("eE", "restore", "restore calibration from flash");
	ad_args.state=arg_lit0("sS", "stat", "print statistics");
	ad_args.start=arg_lit0("tT", "start", "start ad conversion");
	ad_args.stop=arg_lit0("oO", "stop", "stop ad conversion");
	ad_args.channel=arg_int0("lL","channel","<n>","channel index");
	ad_args.end=arg_end(0);
 
	esp_console_cmd_t cmd = {
		.command=CMD,
		.help="AD conversion commands",
		.hint="Ad hints",
		.func=cmd_ad
	};

	ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}
