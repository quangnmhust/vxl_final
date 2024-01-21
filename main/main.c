#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc_cal.h"
#include <esp_system.h>
#include <esp_err.h>
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "freertos/queue.h"
#include <freertos/semphr.h>

#include <inttypes.h>
#include "esp_heap_caps.h"
#include "esp_chip_info.h"

#include "soc/rtc.h"
#include "esp_pm.h"
#include "esp_partition.h"

#include <sht3x.h>
#include <sht4x.h>
#include "ssd1306.h"
#include "common.h"
#include "i2cdev.h"

#define PERIOD CONFIG_PERIOD
#define HTTP_PERIOD CONFIG_HTTP_PERIOD
#define PROB_WARNING CONFIG_PROB_WARNING

#define SDA_PIN 21
#define SCL_PIN 22

static esp_adc_cal_characteristics_t *adc_chars;
#define DEFAULT_VREF    5000
#define NO_OF_SAMPLES 	100

#define MQ_ADC_CHANNEL  ADC1_CHANNEL_6
#define FL_ADC_CHANNEL  ADC1_CHANNEL_7

#define RED_GPIO_PIN 	GPIO_NUM_33
#define GREEN_GPIO_PIN 	GPIO_NUM_32
#define FAN_GPIO_PIN 	GPIO_NUM_25
#define BUZ_GPIO_PIN 	GPIO_NUM_26
#define BUT_GPIO_PIN 	GPIO_NUM_27

#define SSID CONFIG_SSID
#define PASS CONFIG_PASSWORD
#define API_KEY CONFIG_API_KEY
#define WEB_SERVER  CONFIG_WEB_SERVER
#define WEB_PORT  CONFIG_WEB_PORT

#define WIFI_CONNECTED_BIT BIT0
#define HTTP_CONNECTED_BIT BIT0

const char *TAG_UART = "VXL";

TaskHandle_t p_task_display = NULL;
TaskHandle_t test_sht41 = NULL;
TaskHandle_t test_adc = NULL;
TaskHandle_t get_data = NULL;

TaskHandle_t update_gpio_task = NULL;
TaskHandle_t send_data_http_task = NULL;
TaskHandle_t led_status_task = NULL;

QueueHandle_t dataSensor_queue;
QueueHandle_t data_http_queue;
QueueHandle_t qClick = NULL;

SemaphoreHandle_t I2C_mutex;
SemaphoreHandle_t Sem_send_http;

sht4x_t dev;
SSD1306_t ssd;
MQdata_st MQ_Init;

static EventGroupHandle_t wifi_event_group;
static EventGroupHandle_t http_event_group;

volatile status led_status;
volatile uint32_t count_restart = 0;
volatile data_type data;

#define TAG_HTTP        "Esp_HTTP"
#define WIFI_TAG        "Esp_Wifi"

const struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
struct addrinfo *res;
struct in_addr *addr;

int s, r;

char REQUEST[512];
char recv_buf[512];

esp_chip_info_t chip_info;

const char* get_chip_model(esp_chip_model_t model);
void print_chip_info(void);
void print_chip_features(void);
void print_freq_info(void);
void print_memory_sizes(void);

static void IRAM_ATTR gpio_ISR_handler(void *arg){
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(qClick, &gpio_num, NULL);
}

////////////--------------ADC---------------///////////////
float MQ2_a_LPG = 574.25;
float MQ2_b_LPG = -2.222;
float MQ2_a_CO = 36974;
float MQ2_b_CO = -3.109;

static void update_vol(){
	uint32_t adc_reading = 0;
	uint32_t fladc_reading = 0;

	//Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++) {
		adc_reading += adc1_get_raw((adc1_channel_t)MQ_ADC_CHANNEL);// ADC_CHANNEL_6
		fladc_reading += adc1_get_raw((adc1_channel_t)FL_ADC_CHANNEL); //ADC_CHANNEL_7
	}
	adc_reading /= NO_OF_SAMPLES;
	fladc_reading /= NO_OF_SAMPLES;

	//Convert adc_reading to voltage in mV
	uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
	MQ_Init._adc = (float)adc_reading;
	MQ_Init._sensor_volt = (float)voltage-70;
	data.flame_data = (float)fladc_reading;
}

static float MQ_calib(float ratioInCleanAir){
	float RS_air; //Define variable for sensor resistance
	float R0; //Define variable for R0
	RS_air = ((MQ_Init._VOL_RESOLUTION*MQ_Init._RL)/MQ_Init._sensor_volt)-MQ_Init._RL; //Calculate RS in fresh air
	if(RS_air < 0)  RS_air = 0; //No negative values accepted.
	R0 = RS_air/ratioInCleanAir; //Calculate R0
	if(R0 < 0)  R0 = 0; //No negative values accepted.
	return R0;
}

static void MQ_readSensor(float correctionFactor){

	ESP_LOGI(__func__,"RL: %f\tVOL_%f\t%f", MQ_Init._RL, MQ_Init._VOL_RESOLUTION, MQ_Init._sensor_volt );

	MQ_Init._Rs_calc = ((MQ_Init._VOL_RESOLUTION*MQ_Init._RL)/MQ_Init._sensor_volt) - MQ_Init._RL; //Get value of RS in a gas
	ESP_LOGI(__func__,"Rs_calc: %f", MQ_Init._Rs_calc);

	if(MQ_Init._Rs_calc < 0)  MQ_Init._Rs_calc = 0; //No negative values accepted.

	MQ_Init._ratio = MQ_Init._Rs_calc / MQ_Init._R0;   // Get ratio RS_gas/RS_air
	printf("ratio %.2f", MQ_Init._ratio);
	MQ_Init._ratio += correctionFactor;

	if(MQ_Init._ratio <= 0)  MQ_Init._ratio = 0;

	if(MQ_Init.MQ_regressMethod == EXPON){
		ESP_LOGI(__func__,"Exponential");
		MQ_Init._PPM_LPG= MQ_Init._a_LPG*pow(MQ_Init._ratio, MQ_Init._b_LPG);
		MQ_Init._PPM_CO= MQ_Init._a_CO*pow(MQ_Init._ratio, MQ_Init._b_CO)/10.0;
	}

	if(MQ_Init._PPM_LPG < 0)  MQ_Init._PPM_LPG = 0;
	if(MQ_Init._PPM_CO < 0)  MQ_Init._PPM_CO = 0;

}



/////////////----------adc check-------------////////////////////

static void initialize_nvs(void)
{
	esp_err_t error = nvs_flash_init();
	if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
		error = nvs_flash_init();
	}
	ESP_ERROR_CHECK_WITHOUT_ABORT(error);
}

static void check_efuse(void)
    {
        //Check if TP is burned into eFuse
        if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
          //  printf("eFuse Two Point: Supported\n");
        } else {
           // printf("eFuse Two Point: NOT supported\n");
        }
        //Check Vref is burned into eFuse
        if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
           // printf("eFuse Vref: Supported\n");
        } else {
           // printf("eFuse Vref: NOT supported\n");
        }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
       // printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        //printf("Characterized using eFuse Vref\n");
    } else {
       // printf("Characterized using Default Vref\n");
    }
}

/*______________________________________________________________*/


float ds_evidence(float temp, float humi, float CO){
	float DS_fire;
	float temperatureProbability = (temp - 20) / 65.0;
	if (temperatureProbability < 0)
	{
		temperatureProbability = 0;
	}
	ESP_LOGI(__func__, "Xac Suat temperature = %.2f ", temperatureProbability);
	float humidityProbability = (-humi / 100) + 1;
	ESP_LOGI(__func__, "Xac Suat humidity = %.2f ", humidityProbability);
	float coProbability;
	coProbability = (float)(CO - 500) / (10000 - 300);
	ESP_LOGI(__func__, "Nong Do co = %.2f ", CO);
	ESP_LOGI(__func__, "Xac Suat co = %.2f ", coProbability);
	if (coProbability < 0)
	{
		coProbability = 0;
		ESP_LOGI(__func__, "bi gan bang 0");
	}
	ESP_LOGI(__func__, "Xac Suat co = %.2f ", coProbability);
	DS_fire = (temperatureProbability * humidityProbability) / (1 - (1 - temperatureProbability) * humidityProbability - temperatureProbability * (1 - humidityProbability));
	// DS_noFire = 1 - DS_fire;
	ESP_LOGI(__func__,"DS lan 1 = %.2f", DS_fire);
	DS_fire = (DS_fire * coProbability) / (1 - (1-DS_fire) * coProbability - DS_fire * (1 - coProbability)); //
	// DS_noFire = 1 - DS_fire;
	ESP_LOGI(__func__,"DS lan 2 = %.2f", DS_fire);
	return DS_fire;
}


//////////////////////////////////////////////////////////////////////////

void i2c_cfig(){
	ESP_ERROR_CHECK( i2cdev_init());
	I2C_mutex = xSemaphoreCreateMutex();

	memset(&dev, 0, sizeof(sht4x_t));
	memset(&ssd, 0, sizeof(SSD1306_t));

	ESP_ERROR_CHECK(sht4x_init_desc(&dev, 0, SDA_PIN, SCL_PIN));
	ESP_ERROR_CHECK(sht4x_init(&dev));

	ssd1306_init(&ssd, 128, 64);
	ssd1306_clear_screen(&ssd, false);
}

/////////////////////////////////////////////////////////////////////////

void display_task(void *pvParameter){
	while (1)
	{
		TickType_t task_lastWakeTime;
		task_lastWakeTime = xTaskGetTickCount();
		data_type result = {};

			if (xSemaphoreTake(I2C_mutex, portMAX_DELAY) == pdTRUE)
			{	
				ESP_LOGI(__func__, "SSD1306 take semaphore");

				if(xQueueReceive(dataSensor_queue, &result, 500/portTICK_PERIOD_MS))
					ESP_LOGI(__func__, "SSD1306 pass receive sensor data");

				char str[16];
				ssd1306_contrast(&ssd, 0xff);
				sprintf(str, "Temp: %.2f  ", result.temp);
				ssd1306_display_text(&ssd, 1, str, strlen(str), false);
				sprintf(str, "Humi: %.2f  ", result.humi);
				ssd1306_display_text(&ssd, 2, str, strlen(str), false);
				sprintf(str, "CO:   %.2f  ", result.CO);
				ssd1306_display_text(&ssd, 3, str, strlen(str), false);
				sprintf(str, "LPG:  %.2f  ", result.LPG);
				ssd1306_display_text(&ssd, 4, str, strlen(str), false);
				sprintf(str, "PROB: %.2f  ", result.prob);
				ssd1306_display_text(&ssd, 5, str, strlen(str), false);
				sprintf(str, "FL:   %.2f  ", result.flame_data);
				ssd1306_display_text(&ssd, 6, str, strlen(str), false);

				sprintf(str, "Count:  %d  ", count_restart);
				ssd1306_display_text(&ssd, 7, str, strlen(str), false);

				ESP_LOGI(__func__, "SSD1306 give semaphore\n");
				xSemaphoreGive(I2C_mutex);

				xQueueSend(data_http_queue,&result, 500/portMAX_DELAY);
				ESP_LOGI(__func__, "Data waiting to read %d, Available space %d", uxQueueMessagesWaiting(data_http_queue), uxQueueSpacesAvailable(data_http_queue));
			}
		vTaskDelayUntil(&task_lastWakeTime, PERIOD/portTICK_RATE_MS);
 	}
}


void adc_cfig(){
	check_efuse();

	//Configure ADC
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(MQ_ADC_CHANNEL, ADC_ATTEN_DB_0);
	adc1_config_channel_atten(FL_ADC_CHANNEL, ADC_ATTEN_DB_0);

	//Characterize ADC
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
	print_char_val_type(val_type);

	MQ_Init.ratioInCleanAir = 9.83;
	MQ_Init.MQ_regressMethod = EXPON;
	MQ_Init._RL = 10;
	MQ_Init._VOL_RESOLUTION = 5000.0;
	MQ_Init._a_LPG = MQ2_a_LPG;
	MQ_Init._b_LPG = MQ2_b_LPG;
	MQ_Init._a_CO = MQ2_a_CO;
	MQ_Init._b_CO = MQ2_b_CO;
	MQ_Init.MQ_correctionFactor = 0.0;
	float calcR0 = 0;
	for(int i = 1; i<=10; i ++)
	  {
		update_vol();
		calcR0 += MQ_calib(MQ_Init.ratioInCleanAir);
	  }
	MQ_Init._R0 = calcR0/10;
	ESP_LOGI(__func__,"R0: %f\n", MQ_Init._R0);
	ESP_LOGI(__func__,"Calibrate done\n");
}

void gpio_cfig(){
	// config LED, FAN, BUZZER with output mode
	gpio_pad_select_gpio(RED_GPIO_PIN);
	gpio_set_direction(RED_GPIO_PIN, GPIO_MODE_OUTPUT);

	gpio_pad_select_gpio(GREEN_GPIO_PIN);
	gpio_set_direction(GREEN_GPIO_PIN, GPIO_MODE_OUTPUT);

	gpio_pad_select_gpio(FAN_GPIO_PIN);
	gpio_set_direction(FAN_GPIO_PIN, GPIO_MODE_OUTPUT);

	gpio_pad_select_gpio(BUZ_GPIO_PIN);
	gpio_set_direction(BUZ_GPIO_PIN, GPIO_MODE_OUTPUT);

	// config Button with input mode
	gpio_pad_select_gpio(BUT_GPIO_PIN);
	gpio_set_direction(BUT_GPIO_PIN, GPIO_MODE_INPUT);
	gpio_pulldown_en(BUT_GPIO_PIN);
	gpio_pullup_dis(BUT_GPIO_PIN);
	gpio_set_intr_type(BUT_GPIO_PIN, GPIO_INTR_POSEDGE); //kieu ngat
	gpio_install_isr_service(0);
	gpio_isr_handler_add(BUT_GPIO_PIN, gpio_ISR_handler, (void *)BUT_GPIO_PIN);

	// init default level gpio
	gpio_set_level(RED_GPIO_PIN, 0);
	gpio_set_level(FAN_GPIO_PIN, 0);
	gpio_set_level(BUZ_GPIO_PIN, 0);
	gpio_set_level(GREEN_GPIO_PIN, 1);
}

SemaphoreHandle_t button_semaphore;
//task bắt sự kiện ngắt
void button_status_task(void * param){
	printf("button\n");
	int gpio_num;
	// TickType_t l_time = 0;
	while(1){
		if(xQueueReceive(qClick,&gpio_num,50) ){
			printf("gpio_num %d\n",gpio_num);
			if(gpio_num == BUT_GPIO_PIN && gpio_get_level(gpio_num) == 1){
				// l_time = xTaskGetTickCount();
				vTaskDelay(1000/portTICK_RATE_MS);
				if(gpio_num == BUT_GPIO_PIN && gpio_get_level(gpio_num) == 1){
					esp_restart();
				} else {
				led_status = !led_status;
				printf("Button Pass %d\r\n", led_status);}
			}
		}
	}
}

void Task_Led_status(void *arg)
{
    while(1)
    {
        if(led_status == SAFE)
        {
			ESP_LOGI(__func__, "Safe");
        	gpio_set_level(RED_GPIO_PIN, 0);
			gpio_set_level(FAN_GPIO_PIN, 0);
			gpio_set_level(BUZ_GPIO_PIN, 0);
			gpio_set_level(GREEN_GPIO_PIN, 1);
			vTaskDelay(1000/portTICK_RATE_MS);
        }
        else if(led_status == DANGER)
        {
			ESP_LOGI(__func__, "Danger");
        	gpio_set_level(RED_GPIO_PIN, 1);
			gpio_set_level(FAN_GPIO_PIN, 1);
			gpio_set_level(BUZ_GPIO_PIN, 1);
			gpio_set_level(GREEN_GPIO_PIN, 0);
			vTaskDelay(1000/portTICK_RATE_MS);
        }

        
    }
}

void sht41_task(void *pvParameters)
{
	data_type result={};
	while(1){
		TickType_t task_lastWakeTime;
		task_lastWakeTime = xTaskGetTickCount();
		float temperature;
		float humidity;
		if (xSemaphoreTake(I2C_mutex, portMAX_DELAY) == pdTRUE)
		{
			ESP_LOGI(__func__, "SHT41 take semaphore");

			ESP_ERROR_CHECK(sht4x_measure(&dev, &temperature, &humidity));
			ESP_LOGI(__func__,"SHT4x Sensor: %.2f °C, %.2f %%", temperature, humidity);
			result.humi = humidity;
			result.temp = temperature;
			data.humi = humidity;
			data.temp = temperature;

			update_vol();
			MQ_readSensor(MQ_Init.MQ_correctionFactor);

			result.CO = MQ_Init._PPM_CO;
			printf("a %.2f", MQ_Init._PPM_CO);
			result.LPG = MQ_Init._PPM_LPG;

			data.CO = MQ_Init._PPM_CO;
			data.LPG = MQ_Init._PPM_LPG;

			result.flame_data = (data.flame_data < 4096.0) ? 1 : 0;

			result.prob = ds_evidence(result.temp, result.humi, result.CO);
			ESP_LOGI(__func__,"LPG: %.2fppm\tCO: %.2fppm\tFL:%.2f\tProb: %.2f\n", result.LPG, result.CO, result.flame_data, result.prob);

			ESP_LOGI(__func__, "SHT41 give semaphore");
			xSemaphoreGive(I2C_mutex);

		}
		if(result.prob > CONFIG_PROB_WARNING || result.flame_data == 0.0){
			led_status = DANGER;
		} else {
			led_status = SAFE;
			}
		xQueueSendToBack(dataSensor_queue, (void *)&result, 1000/portTICK_RATE_MS);
		ESP_LOGI(__func__, "Data waiting to read %d, Available space %d", uxQueueMessagesWaiting(dataSensor_queue), uxQueueSpacesAvailable(dataSensor_queue));
		vTaskDelayUntil(&task_lastWakeTime, PERIOD/portTICK_RATE_MS);
	}
}


//wifi
static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
	printf("id %d\n", event_id);
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(WIFI_TAG,"WiFi connecting ... \n");
        esp_wifi_connect();
        break;
    case WIFI_EVENT_STA_CONNECTED:
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
		ESP_LOGI(WIFI_TAG,"Wi-Fi connected HTTP start\n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(WIFI_TAG,"Try to WiFi connection ... \n");
        esp_wifi_connect();
        break;
    case IP_EVENT_STA_GOT_IP:
        ESP_LOGI(WIFI_TAG,"WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}

void wifi_connection()
{
    wifi_event_group = xEventGroupCreate();
	http_event_group = xEventGroupCreate();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta(); 
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation); 
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = SSID,
            .password = PASS}};
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);
    esp_wifi_start();
    esp_wifi_connect();
}

//htttp

void send_data_http(void*arg)
{
    data_type HTTP_data = {};
    while(1)
    {
		if(xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY) && WIFI_CONNECTED_BIT)
        {
			int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);
			if(err != 0 || res == NULL) {
				ESP_LOGE(__func__, "DNS lookup failed err=%d res=%p", err, res);
				// vTaskDelay((TickType_t)(1000 / portTICK_RATE_MS));
				continue;
			}

			addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
			ESP_LOGI(__func__, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

			s = socket(res->ai_family, res->ai_socktype, 0);
			if(s < 0) {
				ESP_LOGE(__func__, "... Failed to allocate socket.");
				freeaddrinfo(res);
				// vTaskDelay((TickType_t)(1000 / portTICK_RATE_MS));
				continue;
			}
			ESP_LOGI(__func__, "... allocated socket");

			if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
				ESP_LOGE(__func__, "... socket connect failed errno=%d", errno);
				close(s);
				freeaddrinfo(res);
				// vTaskDelay((TickType_t)(1000 / portTICK_RATE_MS));
				continue;
			}

			ESP_LOGI(__func__, "... connected");
			xEventGroupSetBits(http_event_group, HTTP_CONNECTED_BIT);
			freeaddrinfo(res);

			ESP_LOGI(TAG_HTTP,"Send data HTTP");
			if(xEventGroupWaitBits(http_event_group, HTTP_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY) && HTTP_CONNECTED_BIT)
        	{
				if(uxQueueMessagesWaiting(data_http_queue) != 0){
					if (xQueueReceive(data_http_queue, &HTTP_data, portMAX_DELAY) == pdPASS) {
						if (xSemaphoreTake(Sem_send_http, portMAX_DELAY) == pdTRUE)
						{
							memset(REQUEST, 0, 512);
							sprintf(REQUEST, "GET https://api.thingspeak.com/update?api_key=%s&field1=%.2f&field2=%.2f&field3=%.2f&field4=%.2f&field5=%.2f\n\n\n\n\n", API_KEY, HTTP_data.temp, HTTP_data.humi, HTTP_data.LPG, HTTP_data.CO, HTTP_data.prob);
							ESP_LOGI(__func__, "HTTP data waiting to read %d, Available space %d \n", uxQueueMessagesWaiting(data_http_queue), uxQueueSpacesAvailable(data_http_queue));
							xSemaphoreGive(Sem_send_http);
							int erro = 0;
							erro = write(s, REQUEST, strlen(REQUEST));
							if ( erro < 0) {
								ESP_LOGE(__func__, "HTTP client publish message failed");
							} else {
								ESP_LOGI(__func__, "HTTP client publish message success\n");
								count_restart++;
							}	
						}
					}
				}
			}
			close(s);
		}
		
		if(count_restart > 150) esp_restart();
        vTaskDelay(HTTP_PERIOD/portTICK_PERIOD_MS);
    }
}

void print_chip_info(void) {
    esp_chip_info(&chip_info);

    ESP_LOGI("Chip Info", "Chip Model: %s", get_chip_model(chip_info.model));
    ESP_LOGI("Chip Info", "Cores: %d", chip_info.cores);
    ESP_LOGI("Chip Info", "Revision number: %d", chip_info.revision);
}

void print_freq_info(void) {
    rtc_cpu_freq_config_t freq_config;
    rtc_clk_cpu_freq_get_config(&freq_config);

    // Reporting the CPU clock source
    const char* clk_source_str;
    switch (freq_config.source) {
        case RTC_CPU_FREQ_SRC_PLL:
            clk_source_str = "PLL";
            break;
        case RTC_CPU_FREQ_SRC_APLL:
            clk_source_str = "APLL";
            break;
        case RTC_CPU_FREQ_SRC_8M:
        	clk_source_str = "8M";
        	break;
        case RTC_CPU_FREQ_SRC_XTAL:
            clk_source_str = "XTAL";
            break;
        default:
            clk_source_str = "Unknown";
            break;
    }
    ESP_LOGI("Chip Info", "CPU Clock Source: %s", clk_source_str);
    ESP_LOGI("Chip Info", "Source Clock Frequency: %" PRIu32 " MHz", freq_config.source_freq_mhz);
    ESP_LOGI("Chip Info", "Divider: %" PRIu32, freq_config.div);
    ESP_LOGI("Chip Info", "Effective CPU Frequency: %" PRIu32 " MHz", freq_config.freq_mhz);
}

// determined based on the constants defined in esp_chip_info.h
void print_chip_features(void) {
    uint32_t features = chip_info.features;

    char binary_str[33]; // 32 bits + null terminator
    for (int i = 31; i >= 0; i--) {
        binary_str[31 - i] = (features & (1U << i)) ? '1' : '0';
    }
    binary_str[32] = '\0'; // Null terminate the string
    ESP_LOGI("Chip Info", "Features Bitmap: %s", binary_str);

    ESP_LOGI("Chip Info", "Embedded Flash: %s", (features & CHIP_FEATURE_EMB_FLASH) ? "Yes" : "No");
    ESP_LOGI("Chip Info", "Embedded PSRAM: %s", (features & CHIP_FEATURE_EMB_PSRAM) ? "Yes" : "No");
    ESP_LOGI("Chip Info", "Wi-Fi 2.4GHz support: %s", (features & CHIP_FEATURE_WIFI_BGN) ? "Yes" : "No");
    ESP_LOGI("Chip Info", "IEEE 802.15.4 support: %s", (features & CHIP_FEATURE_IEEE802154) ? "Yes" : "No");
    ESP_LOGI("Chip Info", "Bluetooth Classic support: %s", (features & CHIP_FEATURE_BT) ? "Yes" : "No");
    ESP_LOGI("Chip Info", "Bluetooth LE (BLE) support: %s", (features & CHIP_FEATURE_BLE) ? "Yes" : "No");
}

void print_memory_sizes(void) {

	// uint32_t flash_size = ESP.getFlashChipSize();
	// printf("--------> Flash size: %PRIu32 bytes\n", flash_size);

    // Flash Size
    const esp_partition_t* partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
    if (partition) {
        ESP_LOGI("Memory Info", "Found App partition");
        ESP_LOGI("Memory Info", "Partition Label: %s", partition->label);
        ESP_LOGI("Memory Info", "Partition Type: %d", partition->type);
        ESP_LOGI("Memory Info", "Partition Subtype: %d", partition->subtype);
        ESP_LOGI("Memory Info", "Partition Size: %" PRIu32 " bytes", partition->size);
    } else {
        ESP_LOGE("Memory Info", "Failed to get the App partition");
    }

    // Total SPIRAM (PSRAM) Size
    size_t spiram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if (spiram_size) {
        ESP_LOGI("Memory Info", "PSRAM Size: %zu bytes", spiram_size);
    } else {
        ESP_LOGI("Memory Info", "No PSRAM detected");
    }

    uint32_t total_internal_memory = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    uint32_t free_internal_memory = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    uint32_t largest_contig_internal_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    ESP_LOGI("Memory Info", "Total DRAM (internal memory): %"PRIu32" bytes", total_internal_memory);
    ESP_LOGI("Memory Info", "Free DRAM (internal memory): %"PRIu32" bytes", free_internal_memory);
    ESP_LOGI("Memory Info", "Largest free contiguous DRAM block: %"PRIu32" bytes", largest_contig_internal_block);

}

// determined based on the constants defined in esp_chip_info.h
const char* get_chip_model(esp_chip_model_t model) {
    switch (model) {
        case CHIP_ESP32:
            return "ESP32";
        case CHIP_ESP32S2:
            return "ESP32-S2";
        case CHIP_ESP32S3:
            return "ESP32-S3";
        case CHIP_ESP32C3:
            return "ESP32-C3";
        default:
            return "Unknown Model";
    }
}

 void app_main()
 {
	/* Print chip information */
    ESP_LOGE("ESP32 MCU Info", "---------------------------------------------------------");

    print_chip_info();

    print_freq_info();

	print_chip_features();

    print_memory_sizes();

    ESP_LOGE("ESP32 MCU Info", "---------------------------------------------------------");

	// Initialize nvs partition
	ESP_LOGE(__func__, "Initialize nvs partition.");
	initialize_nvs();
	//wifi connect
	wifi_connection();

	qClick = xQueueCreate(1, sizeof(uint32_t));
	dataSensor_queue = xQueueCreate(20, sizeof(data_type));
	data_http_queue = xQueueCreate(20,sizeof(data_type));
	ESP_LOGI(__func__, "Create Queue success.");

	I2C_mutex = xSemaphoreCreateMutex();
	Sem_send_http = xSemaphoreCreateMutex();

	i2c_cfig();
	gpio_cfig();
	adc_cfig();

	xTaskCreatePinnedToCore(sht41_task, "sht41_task", 2048 * 2, NULL, 3, &test_sht41, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(send_data_http, "send_data_http", 2048 * 2, NULL, 4, &send_data_http_task, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(button_status_task, "button_status_task", 2048 * 2, NULL, 1, &update_gpio_task, tskNO_AFFINITY); //ngat
	xTaskCreatePinnedToCore(Task_Led_status, "Task_Led_status", 2048 * 2, NULL, 2, &led_status_task, tskNO_AFFINITY);
	xTaskCreatePinnedToCore(display_task, "display_task", 2048 * 2, NULL, 2, &p_task_display, tskNO_AFFINITY);


 }
