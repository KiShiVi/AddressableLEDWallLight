#include "freertos/FreeRTOS.h"      ///< Подключение FreeRTOS
#include "freertos/task.h"          ///< Подключение задач FreeRTOS
#include "driver/gpio.h"            ///< Подключение драйвера GPIO
#include "esp_log.h"                ///< Подключение логирования ESP32
#include "nvs_flash.h"              ///< Подключение библиотеки nvs_flash
#include "esp_wifi.h"               ///< Подключение библиотеки WiFi
#include <string.h>                 ///< Для работы со строками
#include <stdio.h>                  ///< Стандартная библиотека ввода/вывода
#include "esp_event.h"              ///< Отлов callback'ов из HTTP сервера
#include "esp_netif.h"              ///< Работа с HTTP
#include "lwip/ip_addr.h"           ///< Работа с LwIP
#include "lwip/netif.h"             ///< Работа с LwIP
#include "esp_http_server.h"        ///< Работа с HTTP
#include "html.h"                   ///< HTML страница
#include "led_strip.h"              ///< Библиотека для работы со светодиодной лентой
#include "math.h"                   ///< Математика
#include "esp_random.h"             ///< Рандомайзер
#include "esp_timer.h"              ///< Для получения текущего времени

#define MAX_MACRO(a, b) (a > b ? a : b)     ///< Макрос для опредления максимального числа
#define MIN_MACRO(a, b) (a < b ? a : b)     ///< Макрос для определения минимального числа

#define SCALE_8(i, sc)                  ((i * sc) / 256)                                    ///< Макрос для скалирование числа от 0 до 255 к диапазону от 0 до sc
#define RANDOM_8()                      (esp_random() % 256)                                ///< Макрос для случайного числа от 0 до 255
#define RANDOM_TO(n)                    (esp_random() % n)                                  ///< Макрос для случайного числа от 0 до (n - 1)
#define RANDOM_FROM_TO(n, m)            ((esp_random() % (m - n)) + n)                      ///< Макрос для случайного числа от n до до (m - 1)
#define U_SUB(a, b)                     ( a > b ? a - b : 0)                                ///< Макрос для вычистания unsigned переменных (min = 0)
#define U_ADD(a, b)                     ( (uint16_t)a + (uint16_t)b < 256 ? a + b : 255)    ///< Макрос для сложения unsigned переменных (max = 255) 

#define NUM_LEDS 284    ///< Количество светодиодов на ленте

#define WIFI_SSID ""      ///< SSID WiFi точки
#define WIFI_PASSWORD ""    ///< Pass Wifi точки
#define MAXIMUM_RETRY 20            ///< Максимальное количество переподключений к точке
static int s_retry_num = 0;         // Переменная для подсчета попыток переподключений

const char* tag = "USER_DEBUG";                             // Пользовательский тег для логирования ESP32

// --- Структуры и определения ---
///< Режим работы устройства
typedef enum
{
    STATIC_LED_STRIP_MODE                   = 0,            ///< Статический свет
    SMOOTH_COLOR_CHANGE_LED_STRIP_MODE      = 1,            ///< Плавная смена цвета
    FIRE_ON_THE_EDGES_LED_STRIP_MODE        = 2,            ///< Языки пламени от краев
    FIRE_ON_THE_CENTER_LED_STRIP_MODE       = 3,            ///< Языки пламени от центра
    CONFETTI_LED_STRIP_MODE                 = 4,            ///< Конфетти
    CONFETTI_RGBY_LED_STRIP_MODE            = 5,            ///< Конфетти RGBY
    RAINBOW_LED_STRIP_MODE                  = 6,            ///< Радуга

    DEVICE_MODE_COUNT                                       ///< Количество режимов работы
}Device_mode;

///< Режим выбора энкодером
typedef enum
{
    ENCODER_CHANGE_MODE                     = 0,            ///< Энкодер в данный момент настраивает режим работы
    ENCODER_CHANGE_H                        = 1,            ///< Энкодер в данный момент настраивает Hue
    ENCODER_CHANGE_S                        = 2,            ///< Энкодер в данный момент настраивает Saturation
    ENCODER_CHANGE_V                        = 3,            ///< Энкодер в данный момент настраивает Value
    ENCODER_CHANGE_SPEED                    = 4,            ///< Энкодер в данный момент настраивает Скорость режима

    ENCODER_CHANGE_MODE_COUNT                               ///< Количество режимов энкодера
}Encoder_mode;

///< Структура для определения цвета в RGB888
typedef struct
{
    uint8_t r;      ///< Значение красного
    uint8_t g;      ///< Значение зеленого
    uint8_t b;      ///< Значение синего
}Device_color;

///< Структура устройства
typedef struct 
{
    uint8_t             is_On;                              ///< Включена ли лента? (0 - выкл, 1 - вкл)
    Device_mode         device_mode;                        ///< Режим работы ленты
    Device_color        device_color;                       ///< Цвет, выбранный пользователем (RGB)
    uint8_t             speed;                              ///< Скорость анимации (в % от 1 до 100)
    Encoder_mode        encoderMode;                        ///< На каком меню настройки находится энкодер
    uint64_t            lastChangeTime;                     ///< Время последнего физического взаимодействия с лентой
}Device;

static led_strip_handle_t led_strip_handle;                 ///< Хэндл светодиодной ленты
//  -------------------------------

//  --- Переменные ---
///< Переменная устройства
Device device = {
    .is_On                          = 0,
    .device_mode                    = STATIC_LED_STRIP_MODE,
    .device_color                   = {0, 255, 255},
    .speed                          = 50,
    .encoderMode                    = ENCODER_CHANGE_MODE,
    .lastChangeTime                 = 0
};
uint8_t mode_changed = 0;   ///< Переменная, определяющая момент смены режима работы устройства.
//  ------------------

//  --- Вспомогательные функции ---
void getColorByFireTemp(uint8_t temp, uint8_t baseR, uint8_t baseG, uint8_t baseB, uint8_t *outR, uint8_t *outG, uint8_t *outB);    ///< Функция получения цвета по температуре огня
void HSV_to_RGB(uint16_t h, uint8_t s, uint8_t v, uint8_t* r, uint8_t* g, uint8_t* b);                                              ///< Функция перевода HSV в RGB
void RGB_to_HSV(uint8_t r, uint8_t g, uint8_t b, uint16_t* h, uint8_t* s, uint8_t* v);                                              ///< Функция перевода RGB в HSV
//  -------------------------------

//  --- Функции инициализации ---
void gpioInit();                            // Инициализация GPIO
void init_wifi();                           // Инициализация WIFI
void init_http_server();                    // Инициализация HTTP сервера
static void configure_led(void);            // Инициализация RMT для светодиодной ленты
//  -----------------------------

//  --- Хэндлеры ---
void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);   // Хэндлер WIFI
esp_err_t index_handler(httpd_req_t *req);              // Хэндлер, когда пользователь заходит на HTTP страницу
esp_err_t onOff_changed_handler(httpd_req_t *req);      // Хэндлер на включение лампы
esp_err_t mode_changed_handler(httpd_req_t *req);       // Хэндлер изменения режима работы
esp_err_t color_changed_handler(httpd_req_t *req);      // Хэндлер на смену цвета
esp_err_t speed_changed_handler(httpd_req_t *req);      // Хэндлер изменения скорости анимации
esp_err_t get_status_handler(httpd_req_t *req);         // Хэндлер обновления информации для сайта
//  ----------------

/// --- Задачи ---
void vTestingLedTask(void* pvParameters);       // Задача мигания светодиодом на отладочной плате
void vLedStripTask(void* pvParameters);         // Задача управления светодиодной лентой
void vEncoderTask(void* pvParameters);          // Задача обработки энкодера
/// --------------

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();               // Инициализация flash памяти
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }                              

    //  --- GPIO ---
    gpioInit();                                     // Инициализация GPIO
    //  ------------

    //  --- WiFi ---
    init_wifi();                                    // Инициализация WiFi
    //  ------------

    //  --- HTTP ---
    init_http_server();                             // Инициализация HTTP сервера
    //  ------------

    //  --- Создание задач ---
    xTaskCreatePinnedToCore (vTestingLedTask,            "vTestingLedTask",          2048,          NULL,   1,      NULL,      0);
    xTaskCreatePinnedToCore (vLedStripTask,              "vLedStripTask",            65536,         NULL,   10,     NULL,      1);
    xTaskCreatePinnedToCore (vEncoderTask,               "vEncoderTask",             8192,          NULL,   5,      NULL,      0);
    //  ----------------------

}



void vTestingLedTask(void* pvParameters)
{
    ESP_LOGI(tag, "vTestingLedTask task started!");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        //  ---------------------------------------------------------------------
        //  В случае выключенного устрйоства - ничего не горит          
        //  ---------------------------------------------------------------------
        if (device.is_On == 0)
        {
            gpio_set_level(GPIO_NUM_2, 0);

            gpio_set_level(GPIO_NUM_14, 0);
            gpio_set_level(GPIO_NUM_23, 0);
            gpio_set_level(GPIO_NUM_25, 0);
            gpio_set_level(GPIO_NUM_26, 0);
            gpio_set_level(GPIO_NUM_27, 0);

            vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
            continue;
        }
        //  ---------------------------------------------------------------------
        //  Если устрйоство включено - горит светодиод на плате ESP32,
        //  а также зеленый светодиод отвечающий за выбор текущего режима смены
        //  энкодером (если не прошел таймаут ожидания)          
        //  ---------------------------------------------------------------------
        else
        {
            gpio_set_level(GPIO_NUM_2, 1);
            if ((esp_timer_get_time() / 1000) - device.lastChangeTime > 10000)
            {
                gpio_set_level(GPIO_NUM_14, 0);
                gpio_set_level(GPIO_NUM_23, 0);
                gpio_set_level(GPIO_NUM_25, 0);
                gpio_set_level(GPIO_NUM_26, 0);
                gpio_set_level(GPIO_NUM_27, 0);
            }
            else if (device.encoderMode == 0)
            {
                gpio_set_level(GPIO_NUM_14, 0);
                gpio_set_level(GPIO_NUM_23, 0);
                gpio_set_level(GPIO_NUM_25, 1);
                gpio_set_level(GPIO_NUM_26, 0);
                gpio_set_level(GPIO_NUM_27, 0);
            }
            else if(device.encoderMode == 1)
            {
                gpio_set_level(GPIO_NUM_14, 0);
                gpio_set_level(GPIO_NUM_23, 0);
                gpio_set_level(GPIO_NUM_25, 0);
                gpio_set_level(GPIO_NUM_26, 1);
                gpio_set_level(GPIO_NUM_27, 0);
            }
            else if(device.encoderMode == 2)
            {
                gpio_set_level(GPIO_NUM_14, 0);
                gpio_set_level(GPIO_NUM_23, 0);
                gpio_set_level(GPIO_NUM_25, 0);
                gpio_set_level(GPIO_NUM_26, 0);
                gpio_set_level(GPIO_NUM_27, 1);
            }
            else if(device.encoderMode == 3)
            {
                gpio_set_level(GPIO_NUM_14, 1);
                gpio_set_level(GPIO_NUM_23, 0);
                gpio_set_level(GPIO_NUM_25, 0);
                gpio_set_level(GPIO_NUM_26, 0);
                gpio_set_level(GPIO_NUM_27, 0);
            }
            else if(device.encoderMode == 4)
            {
                gpio_set_level(GPIO_NUM_14, 0);
                gpio_set_level(GPIO_NUM_23, 1);
                gpio_set_level(GPIO_NUM_25, 0);
                gpio_set_level(GPIO_NUM_26, 0);
                gpio_set_level(GPIO_NUM_27, 0);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    }
}



void vLedStripTask(void* pvParameters)
{
    ESP_LOGI(tag, "vLedStripTask task started!");

    configure_led();                                                        // Инициализация светодиодной ленты

    uint8_t currentR;                                                       // Текущий цвет красного канала (не для всех режимов)
    uint8_t currentG;                                                       // Текущий цвет зеленого канала (не для всех режимов)
    uint8_t currentB;                                                       // Текущий цвет синего канала (не для всех режимов)

    uint8_t fire_temp[NUM_LEDS];                                            // Температура каждого пикселя огня

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        if (device.is_On == 0)                                              // Если лента выключена
        {
            const float blackOutK = 0.1;                                    // Коэф. скорости затемнения
            for (uint16_t i = 0; i < NUM_LEDS; ++i)
            {
                    led_strip_set_pixel_goto_color(
                    led_strip_handle, 
                    i, 
                    0, 
                    0, 
                    0,
                    blackOutK);

                    fire_temp[i] = 0;
            }

            currentR    = 0;
            currentG    = 0;
            currentB    = 0;

            led_strip_refresh(led_strip_handle);

            vTaskDelayUntil(&xLastWakeTime, 33 / portTICK_PERIOD_MS);
            // TODO добавить режим усыпления!

            continue;
        }

        switch (device.device_mode)
        {
        //  ------------------------
        //  --- Статический цвет ---
        //  ------------------------
        case STATIC_LED_STRIP_MODE:
        {   
            if (mode_changed)
            {
                mode_changed = 0;
                device.device_color.r = 0;
                device.device_color.g = 238;
                device.device_color.b = 255;
            } 
            const float transitionK = 0.05;

            currentR = (uint8_t)(currentR * (1 - transitionK) + (float)device.device_color.r * transitionK);
            currentG = (uint8_t)(currentG * (1 - transitionK) + (float)device.device_color.g * transitionK);
            currentB = (uint8_t)(currentB * (1 - transitionK) + (float)device.device_color.b * transitionK);

            for (uint16_t i = 0; i < NUM_LEDS; ++i)
            {
                led_strip_set_pixel_goto_color(
                    led_strip_handle, 
                    i, 
                    currentR, 
                    currentG, 
                    currentB,
                    0.1);
            }
            led_strip_refresh(led_strip_handle);

            vTaskDelayUntil(&xLastWakeTime, 33 / portTICK_PERIOD_MS);
            break;
        }



        //  ---------------------------
        //  --- Плавная смена цвета ---
        //  ---------------------------
        case SMOOTH_COLOR_CHANGE_LED_STRIP_MODE:
        {
            if (mode_changed)
            {
                mode_changed = 0;
                device.device_color.r = 0;
                device.device_color.g = 238;
                device.device_color.b = 255;
                device.speed = 5;
            } 
            static float HueIdx = 0;        // Бегунок HUE

            uint16_t        currentH;
            uint8_t         currentS;
            uint8_t         currentV;

            uint16_t        necessaryH;
            uint8_t         necessaryS;
            uint8_t         necessaryV;

            RGB_to_HSV(device.device_color.r, device.device_color.g, device.device_color.b, &necessaryH, &necessaryS, &necessaryV);

            HueIdx = HueIdx + 0.04f * device.speed;
            if (HueIdx > 360) HueIdx = 0;

            HSV_to_RGB(
                (uint16_t)HueIdx, 
                necessaryS, 
                necessaryV, 
                &currentR, &currentG, &currentB);
            
            for (uint16_t i = 0; i < NUM_LEDS; ++i)
            {
                led_strip_set_pixel_goto_color(
                    led_strip_handle, 
                    i, 
                    currentR, 
                    currentG, 
                    currentB,
                    0.1);
            }
            led_strip_refresh(led_strip_handle);

            vTaskDelayUntil(&xLastWakeTime, 33 / portTICK_PERIOD_MS);
            break;
        }



        //  ------------------------------
        //  --- Языки пламени от краев ---
        //  ------------------------------
        case FIRE_ON_THE_EDGES_LED_STRIP_MODE:
        {
            if (mode_changed)
            {
                for(uint16_t i = 0; i < NUM_LEDS; ++i)
                {
                    fire_temp[i] = 0;
                }
                mode_changed = 0;

                device.device_color.r = 255;
                device.device_color.g = 68;
                device.device_color.b = 0;
                device.speed = 35;
            }
            static uint8_t COOLING  = 100;                  // Коэф. охлождения. Рекомендован 20-100
            static uint8_t SPARKING = 90;                   // Вероятность появления искры. Рекомендован 50-200

            // Шаг 1. Охлаждаем все пиксели
            for ( int i = 0; i < NUM_LEDS; i++) {
                fire_temp[i] = U_SUB(fire_temp[i], RANDOM_FROM_TO(0, ((COOLING * 10) / NUM_LEDS / 1) + 2));
            }

            
            // Шаг 2. Тепло от каждой ячейки поднимается и немного рассеивается
            // for ( int k = NUM_LEDS - 1; k >= (NUM_LEDS / 2) + 2; k--) {
            //     fire_temp[k] = (fire_temp[k - 1] + fire_temp[k - 2] + fire_temp[k - 2] ) / 3;
            // }
            // for ( int k = 0; k < (NUM_LEDS / 2) - 2; ++k) {
            //     fire_temp[k] = (fire_temp[k + 1] + fire_temp[k + 2] + fire_temp[k + 2] ) / 3;
            // }
            for ( int k = (NUM_LEDS / 2) - 15; k >= 2; k--) {
                fire_temp[k] = (fire_temp[k - 1] + fire_temp[k - 2] + fire_temp[k - 2] ) / 3;
            }
            for ( int k = (NUM_LEDS / 2) + 15; k < (NUM_LEDS - 3); ++k) {
                fire_temp[k] = (fire_temp[k + 1] + fire_temp[k + 2] + fire_temp[k + 2] ) / 3;
            }

            // Шаг 3.  Случайным образом зажигаем новые искры тепла около очага
            if ( RANDOM_8() < SPARKING ) {
                int y = RANDOM_FROM_TO(0, 7);
                fire_temp[y] = U_ADD( fire_temp[y], RANDOM_FROM_TO(200, 255) );
            }
            if ( RANDOM_8() < SPARKING ) {
                int y = RANDOM_FROM_TO((NUM_LEDS - 7), NUM_LEDS);
                fire_temp[y] = U_ADD( fire_temp[y], RANDOM_FROM_TO(200, 255) );
            }
            // if ( RANDOM_8() < (uint16_t)SPARKING * 2 ) {
            //     int y = RANDOM_FROM_TO((NUM_LEDS / 2 - 5), (NUM_LEDS / 2 + 5));
            //     fire_temp[y] = U_ADD( fire_temp[y], RANDOM_FROM_TO(200, 255) );
            // }

              // Step 4.  Map from heat cells to LED colors
            for ( int j = 0; j < NUM_LEDS; j++) {
                // fire_temp[j] = SCALE_8(fire_temp[j], 240);
                // // Scale the heat value from 0-255 down to 0-240
                // // for best results with color palettes.
                // if (fire_temp[j] <= 85) {
                //     // От черного к красному
                //     led_strip_set_pixel(
                //     led_strip_handle, 
                //     j, 
                //     fire_temp[j] * 3, 
                //     0, 
                //     0);
                // } else if (fire_temp[j] <= 170) {
                //     // От красного к оранжевому
                //     led_strip_set_pixel(
                //     led_strip_handle, 
                //     j, 
                //     255, 
                //     (fire_temp[j] - 85) * 3, 
                //     0);
                // } else {
                //     // От оранжевого к желтому и белому
                //     led_strip_set_pixel(
                //     led_strip_handle, 
                //     j, 
                //     255, 
                //     255, 
                //     (fire_temp[j] - 170) * 3);
                // }
                getColorByFireTemp(fire_temp[j], device.device_color.r, device.device_color.g, device.device_color.b, &currentR, &currentG, &currentB);
                led_strip_set_pixel(
                    led_strip_handle, 
                    j, 
                    currentR, 
                    currentG, 
                    currentB);
            }

            led_strip_refresh(led_strip_handle);
            vTaskDelayUntil(&xLastWakeTime, 16 + ((100 - device.speed) / 10) / portTICK_PERIOD_MS);
            break;
        }



        //  -------------------------------
        //  --- Языки пламени от центра ---
        //  -------------------------------
        case FIRE_ON_THE_CENTER_LED_STRIP_MODE:
        {
            if (mode_changed)
            {
                for(uint16_t i = 0; i < NUM_LEDS; ++i)
                {
                    fire_temp[i] = 0;
                }
                mode_changed = 0;

                device.device_color.r = 255;
                device.device_color.g = 68;
                device.device_color.b = 0;
                device.speed = 35;
            }
            static uint8_t COOLING  = 100;                  // Коэф. охлождения. Рекомендован 20-100
            static uint8_t SPARKING = 90;                   // Вероятность появления искры. Рекомендован 50-200

            // Шаг 1. Охлаждаем все пиксели
            for ( int i = 0; i < NUM_LEDS; i++) {
                fire_temp[i] = U_SUB(fire_temp[i], RANDOM_FROM_TO(0, ((COOLING * 10) / NUM_LEDS / 1) + 2));
            }

            
            // Шаг 2. Тепло от каждой ячейки поднимается и немного рассеивается
            for ( int k = NUM_LEDS - 16; k >= (NUM_LEDS / 2) + 2; --k) {
                fire_temp[k] = (fire_temp[k - 1] + fire_temp[k - 2] + fire_temp[k - 2] ) / 3;
            }
            for ( int k = 15; k < (NUM_LEDS / 2) - 2; ++k) {
                fire_temp[k] = (fire_temp[k + 1] + fire_temp[k + 2] + fire_temp[k + 2] ) / 3;
            }

            // Шаг 3.  Случайным образом зажигаем новые искры тепла около очага
            if ( RANDOM_8() < SPARKING ) {
                int y = RANDOM_FROM_TO(((NUM_LEDS / 2) - 7), (NUM_LEDS / 2));
                fire_temp[y] = U_ADD( fire_temp[y], RANDOM_FROM_TO(200, 255) );
            }
            if ( RANDOM_8() < SPARKING ) {
                int y = RANDOM_FROM_TO((NUM_LEDS / 2), ((NUM_LEDS / 2) + 7));
                fire_temp[y] = U_ADD( fire_temp[y], RANDOM_FROM_TO(200, 255) );
            }

            // Step 4.  Map from heat cells to LED colors
            for ( int j = 0; j < NUM_LEDS; j++) {
                getColorByFireTemp(fire_temp[j], device.device_color.r, device.device_color.g, device.device_color.b, &currentR, &currentG, &currentB);
                led_strip_set_pixel(
                    led_strip_handle, 
                    j, 
                    currentR, 
                    currentG, 
                    currentB);
            }

            led_strip_refresh(led_strip_handle);
            vTaskDelayUntil(&xLastWakeTime, 16 + ((100 - device.speed) / 10) / portTICK_PERIOD_MS);
            break;
        }



        //  ----------------
        //  --- Конфетти ---
        //  ----------------
        case CONFETTI_LED_STRIP_MODE:
        {
            if (mode_changed)
            { 
                mode_changed = 0;
                device.device_color.r = 0;
                device.device_color.g = 0;
                device.device_color.b = 0;

                device.speed = 5;
            }

            const uint8_t confetti_spawn_probability = 100;     // Вероятность появления конфетти (0 - 0%; 255 - 100%)

            const float transitionK = 0.5;

            currentR = (uint8_t)(currentR * (1 - transitionK) + (float)device.device_color.r * transitionK);
            currentG = (uint8_t)(currentG * (1 - transitionK) + (float)device.device_color.g * transitionK);
            currentB = (uint8_t)(currentB * (1 - transitionK) + (float)device.device_color.b * transitionK);

            for (uint16_t i = 0; i < NUM_LEDS; ++i)
            {
                led_strip_set_pixel_goto_color(
                    led_strip_handle, 
                    i, 
                    currentR, 
                    currentG, 
                    currentB,
                    0.02);
            }

            if (RANDOM_8() < confetti_spawn_probability)                    // Прокаем вероятность
            {
                uint16_t confetti_led_idx   = RANDOM_FROM_TO(0, NUM_LEDS);      // Загадываем расположение новой вспышки
                uint16_t h                  = RANDOM_FROM_TO(0, 361);           // Загадываем любой оттенок
                uint8_t s                   = RANDOM_FROM_TO(80, 101);          // Насыщенность повыше
                uint8_t v                   = RANDOM_FROM_TO(95, 101);          // Яркость повыше
                uint8_t width               = RANDOM_FROM_TO(1, 6);             // Загадываем ширину новой вспышки

                uint8_t confetti_r;
                uint8_t confetti_g;
                uint8_t confetti_b;

                HSV_to_RGB(h, s, v, &confetti_r, &confetti_g, &confetti_b);

                for (int8_t i = 0; i < width; ++i)
                {
                    led_strip_set_pixel(
                        led_strip_handle, 
                        ((confetti_led_idx + i) % NUM_LEDS), 
                        confetti_r, 
                        confetti_g, 
                        confetti_b
                    );
                }
            }

            led_strip_refresh(led_strip_handle);

            vTaskDelayUntil(&xLastWakeTime, 16 + ((100 - device.speed) / 10) / portTICK_PERIOD_MS);
            break;
        }



        //  --------------------
        //  --- RGBY Конфетти ---
        //  --------------------
        case CONFETTI_RGBY_LED_STRIP_MODE:
        {
            if (mode_changed)
            { 
                mode_changed = 0;
                device.device_color.r = 0;
                device.device_color.g = 0;
                device.device_color.b = 0;

                device.speed = 3;
            }

            const uint8_t confetti_spawn_probability = 50;     // Вероятность появления конфетти (0 - 0%; 255 - 100%)

            const float transitionK = 0.5;

            currentR = (uint8_t)(currentR * (1 - transitionK) + (float)device.device_color.r * transitionK);
            currentG = (uint8_t)(currentG * (1 - transitionK) + (float)device.device_color.g * transitionK);
            currentB = (uint8_t)(currentB * (1 - transitionK) + (float)device.device_color.b * transitionK);

            for (uint16_t i = 0; i < NUM_LEDS; ++i)
            {
                led_strip_set_pixel_goto_color(
                    led_strip_handle, 
                    i, 
                    currentR, 
                    currentG, 
                    currentB,
                    0.02);
            }

            if (RANDOM_8() < confetti_spawn_probability)                    // Прокаем вероятность
            {
                uint16_t confetti_led_idx   = RANDOM_FROM_TO(0, NUM_LEDS);      // Загадываем расположение новой вспышки
                uint8_t h                  = RANDOM_FROM_TO(0, 4);              // Загадываем любой RGBY
                uint8_t width               = RANDOM_FROM_TO(1, 6);             // Загадываем ширину новой вспышки

                uint8_t confetti_r = 0;
                uint8_t confetti_g = 0;
                uint8_t confetti_b = 0;

                switch (h)
                {
                case 0:
                confetti_r = 255;
                confetti_g = 0;
                confetti_b = 0;
                    break;
                case 1:
                confetti_r = 0;
                confetti_g = 255;
                confetti_b = 0;
                    break;
                case 2:
                confetti_r = 0;
                confetti_g = 0;
                confetti_b = 255;
                    break;
                case 3:
                confetti_r = 255;
                confetti_g = 255;
                confetti_b = 0;
                    break;
                
                default:
                    break;
                }

                for (int8_t i = 0; i < width; ++i)
                {
                    led_strip_set_pixel(
                        led_strip_handle, 
                        ((confetti_led_idx + i) % NUM_LEDS), 
                        confetti_r, 
                        confetti_g, 
                        confetti_b
                    );
                }
            }

            led_strip_refresh(led_strip_handle);

            vTaskDelayUntil(&xLastWakeTime, 16 + ((100 - device.speed) / 10) / portTICK_PERIOD_MS);
            break;
        }



        //  ----------
        //  Радуга ---
        //  ----------
        case RAINBOW_LED_STRIP_MODE:
        {
            static int16_t HueShift = 0;
            if (mode_changed)
            {
                HueShift = 0;
                mode_changed = 0;
                device.device_color.r = 80;
                device.device_color.g = 0;
                device.device_color.b = 0;
                device.speed = 50;
            } 

            uint16_t currentH;
            uint8_t currentS;
            uint8_t currentV;

            RGB_to_HSV(device.device_color.r, device.device_color.g, device.device_color.b, &currentH, &currentS, &currentV);

            for (uint16_t i = 0; i < NUM_LEDS; ++i)
            {
                currentH = ((i * 360 / NUM_LEDS) + HueShift) % 360;

                HSV_to_RGB(currentH, currentS, currentV, &currentR, &currentG, &currentB);

                led_strip_set_pixel(
                    led_strip_handle, 
                    i, 
                    currentR, 
                    currentG, 
                    currentB
                );
            }

            led_strip_refresh(led_strip_handle);

            HueShift -= 2;
            if (HueShift < 0) HueShift = 360;
            vTaskDelayUntil(&xLastWakeTime, 16 + ((100 - device.speed) / 5) / portTICK_PERIOD_MS);
            break;
        }
        default:
            break;
        continue;
        }
    }
}



void vEncoderTask(void* pvParameters)
{
    // Переменные для кнопки
    static bool button_pressed = false;
    static bool button_held = false;
    static uint64_t button_press_time = 0;

    #define BUTTON_HOLD_TIME_MS 1000    // Время для удержвния кнопки

    // Переменные для поворота ручки
    static int8_t last_state = 0;
    static int8_t full_cycle = 0;

    static uint16_t curH = 0;
    static uint8_t curS = 0;
    static uint8_t curV = 0;
    for(;;)
    {
        // Обработка кнопки
        if (!gpio_get_level(GPIO_NUM_19)) { // Кнопка нажата
            if (!button_pressed) {
                button_pressed = true;
                button_press_time = esp_timer_get_time() / 1000; // Текущее время в мс
            } else if (!button_held && ((esp_timer_get_time() / 1000) - button_press_time > BUTTON_HOLD_TIME_MS)) {
                button_held = true;
                //printf("Button held\n");
                device.is_On = !(device.is_On);
                device.lastChangeTime = esp_timer_get_time() / 1000;
            }
        } else if (button_pressed) { // Кнопка отпущена
            if (!button_held) {
                //printf("Button pressed\n");
                device.encoderMode = ((uint8_t)device.encoderMode + 1) % ENCODER_CHANGE_MODE_COUNT;
                device.lastChangeTime = esp_timer_get_time() / 1000;
            }
            button_pressed = false;
            button_held = false;
        }

        // Обработка поворота ручки
        int8_t state = (gpio_get_level(GPIO_NUM_32) << 0) | (gpio_get_level(GPIO_NUM_33) << 1);
        //printf("%u\n", state);
        if (last_state != state) {
            if ((last_state == 0b00 && state == 0b01) ||
                (last_state == 0b01 && state == 0b11) ||
                (last_state == 0b11 && state == 0b10) ||
                (last_state == 0b10 && state == 0b00)) {
                full_cycle++;
            } else if ((last_state == 0b00 && state == 0b10) ||
                       (last_state == 0b10 && state == 0b11) ||
                       (last_state == 0b11 && state == 0b01) ||
                       (last_state == 0b01 && state == 0b00)) {
                full_cycle--;
            }
            last_state = state;

            if (full_cycle == 4) {
                //printf("Encoder increment\n");
                device.lastChangeTime = esp_timer_get_time() / 1000;
                switch (device.encoderMode)
                {
                case ENCODER_CHANGE_MODE:
                    device.device_mode = ((uint8_t)device.device_mode + 1) % DEVICE_MODE_COUNT;
                    mode_changed = 1;
                    break;
                case ENCODER_CHANGE_H:

                    RGB_to_HSV(device.device_color.r, device.device_color.g, device.device_color.b, &curH, &curS, &curV);
                    curH = (curH > 349) ? 359 : curH + 10;
                    HSV_to_RGB(curH, curS, curV, &device.device_color.r, &device.device_color.g, &device.device_color.b);
                    break;
                case ENCODER_CHANGE_S:
                    RGB_to_HSV(device.device_color.r, device.device_color.g, device.device_color.b, &curH, &curS, &curV);
                    curS = (curS > 245 ? 255 : curS + 10);
                    HSV_to_RGB(curH, curS, curV, &device.device_color.r, &device.device_color.g, &device.device_color.b);
                    break;
                case ENCODER_CHANGE_V:
                    RGB_to_HSV(device.device_color.r, device.device_color.g, device.device_color.b, &curH, &curS, &curV);
                    curV = (curV > 245 ? 255 : curV + 10);
                    HSV_to_RGB(curH, curS, curV, &device.device_color.r, &device.device_color.g, &device.device_color.b);
                    break;
                case ENCODER_CHANGE_SPEED:
                    device.speed = (device.speed > 90 ? 100 : device.speed + 10);
                    break;
                default:
                    break;
                }
                full_cycle = 0;  // Сброс цикла
            } else if (full_cycle == -4) {
                //printf("Encoder decrement\n");
                device.lastChangeTime = esp_timer_get_time() / 1000;
                switch (device.encoderMode)
                {
                case ENCODER_CHANGE_MODE:
                    device.device_mode = (device.device_mode == 0) ? DEVICE_MODE_COUNT - 1 : device.device_mode - 1;
                    mode_changed = 1;
                    break;
                case ENCODER_CHANGE_H:
                    RGB_to_HSV(device.device_color.r, device.device_color.g, device.device_color.b, &curH, &curS, &curV);
                    curH = (curH < 10) ? 0 : curH - 10;
                    HSV_to_RGB(curH, curS, curV, &device.device_color.r, &device.device_color.g, &device.device_color.b);
                    break;
                case ENCODER_CHANGE_S:
                    RGB_to_HSV(device.device_color.r, device.device_color.g, device.device_color.b, &curH, &curS, &curV);
                    curS = (curS < 10) ? 0 : curS - 10;
                    HSV_to_RGB(curH, curS, curV, &device.device_color.r, &device.device_color.g, &device.device_color.b);
                    break;
                case ENCODER_CHANGE_V:
                    RGB_to_HSV(device.device_color.r, device.device_color.g, device.device_color.b, &curH, &curS, &curV);
                    curV = (curV < 10) ? 0 : curV - 10;
                    HSV_to_RGB(curH, curS, curV, &device.device_color.r, &device.device_color.g, &device.device_color.b);
                    break;
                case ENCODER_CHANGE_SPEED:
                    device.speed = (device.speed < 10) ? 0 : device.speed - 10;
                    break;
                default:
                    break;
                }
                full_cycle = 0;  // Сброс цикла
            }
        }

        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}



#pragma region InitFunctions
void gpioInit()
{
    esp_err_t ret;
    gpio_config_t gpio_config_arg = {
        .pin_bit_mask = (1 << GPIO_NUM_2) | (1 << GPIO_NUM_14) | (1 << GPIO_NUM_27) | (1 << GPIO_NUM_26) | (1 << GPIO_NUM_25)| (1 << GPIO_NUM_23),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ret = gpio_config(&gpio_config_arg);
    if (ret != ESP_OK) {
        printf("GPIO config error: %d\n", ret);
    }

    gpio_config_arg.pin_bit_mask = (1ULL << GPIO_NUM_19) | (1ULL << GPIO_NUM_32) | (1ULL << GPIO_NUM_33);
    gpio_config_arg.mode = GPIO_MODE_INPUT;
    gpio_config_arg.pull_up_en = 1;
    gpio_config_arg.pull_down_en = 0;
    gpio_config_arg.intr_type = GPIO_INTR_DISABLE;

    ret = gpio_config(&gpio_config_arg);
    if (ret != ESP_OK) {
        printf("GPIO config error: %d\n", ret);
    }
}



void init_wifi() {
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    assert(netif);

    esp_wifi_init(&(wifi_init_config_t)WIFI_INIT_CONFIG_DEFAULT());

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password));
    wifi_config.sta.bssid_set = false;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.channel = 0;
    wifi_config.sta.listen_interval = 0;

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifi_config.sta.failure_retry_cnt = 255;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(netif));

    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 0, 230);        // Задайте ваш статический IP
    IP4_ADDR(&ip_info.gw, 192, 168, 0, 1);          // Шлюз
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);   // Маска сети
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip_info));

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));  // Отключить режим энергосбережения

    ESP_ERROR_CHECK(esp_wifi_start());
    //vTaskDelay(pdMS_TO_TICKS(100)); // Задержка 100 мс
    //ESP_ERROR_CHECK(esp_wifi_connect());
    //ESP_LOGI(tag, "BUU!!!!!!");
}



void init_http_server()
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Регистрация маршрутов
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL
    };

    httpd_uri_t color_changed_uri = {
        .uri = "/color_changed",
        .method = HTTP_GET,
        .handler = color_changed_handler,
        .user_ctx = NULL
    };

    httpd_uri_t onOff_changed_uri = {
        .uri = "/toggle_changed",
        .method = HTTP_GET,
        .handler = onOff_changed_handler,
        .user_ctx = NULL
    };

    httpd_uri_t mode_changed_uri = {
        .uri = "/mode_changed",
        .method = HTTP_GET,
        .handler = mode_changed_handler,
        .user_ctx = NULL
    };

    httpd_uri_t speed_changed_uri = {
        .uri = "/speed_changed",
        .method = HTTP_GET,
        .handler = speed_changed_handler,
        .user_ctx = NULL
    };

    httpd_uri_t get_status_uri = {
        .uri = "/get_status",
        .method = HTTP_GET,
        .handler = get_status_handler,
        .user_ctx = NULL
    };

    config.server_port = 8081;

    // Запуск сервера
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &index_uri);
        httpd_register_uri_handler(server, &color_changed_uri);
        httpd_register_uri_handler(server, &onOff_changed_uri);
        httpd_register_uri_handler(server, &mode_changed_uri);
        httpd_register_uri_handler(server, &speed_changed_uri);
        httpd_register_uri_handler(server, &get_status_uri);
    }
}



static void configure_led(void)
{
  led_strip_config_t strip_config = {
    .flags = {0},
    .strip_gpio_num = 18,
    .max_leds = NUM_LEDS,
    .led_model = LED_MODEL_WS2812,
    .led_pixel_format = LED_PIXEL_FORMAT_GRB
  };
  led_strip_rmt_config_t rmt_config = {
    .mem_block_symbols = 512,
    .resolution_hz = 10 * 1000 * 1000,      //10MHz
  };
  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_handle));
  led_strip_clear(led_strip_handle);
}

#pragma endregion


#pragma region Handlers
// Обработчик запроса на главную страницу
esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

esp_err_t color_changed_handler(httpd_req_t *req)
{
    uint8_t buf[256];
    size_t buf_len;
    buf_len = httpd_req_get_url_query_len(req) + 1;

    httpd_req_get_url_query_str(req, (char*)buf, buf_len);
    char value[7];
    if (httpd_query_key_value((char*)buf, "color", (char *)&value, sizeof(value)) == ESP_OK) {
        ESP_LOGI(tag, "Color value: %d", atoi(value));
    }

    char rHex[3] = {value[0], value[1], '\0'};
    char gHex[3] = {value[2], value[3], '\0'};
    char bHex[3] = {value[4], value[5], '\0'};

    uint8_t rInt = (uint8_t)strtol(rHex, NULL, 16);
    uint8_t gInt = (uint8_t)strtol(gHex, NULL, 16);
    uint8_t bInt = (uint8_t)strtol(bHex, NULL, 16);

    device.device_color.r       = rInt;
    device.device_color.g       = gInt;
    device.device_color.b       = bInt;

    httpd_resp_set_type(req, "application/json");  // Устанавливаем тип контента как JSON
    httpd_resp_send(req, "{\"status\":\"OK\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t onOff_changed_handler(httpd_req_t *req)
{
    uint8_t buf[256];
    size_t buf_len;
    buf_len = httpd_req_get_url_query_len(req) + 1;

    httpd_req_get_url_query_str(req, (char*)buf, buf_len);
    char value[2];
    if (httpd_query_key_value((char*)buf, "on", (char *)&value, sizeof(value)) == ESP_OK) {
        ESP_LOGI(tag, "ON/OFF value: %d", atoi(value));
    }

    device.is_On = (uint8_t)atoi(&value[0]);

    httpd_resp_set_type(req, "application/json");  // Устанавливаем тип контента как JSON
    httpd_resp_send(req, "{\"status\":\"OK\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t mode_changed_handler(httpd_req_t *req)
{
    uint8_t buf[256];
    size_t buf_len;
    buf_len = httpd_req_get_url_query_len(req) + 1;

    httpd_req_get_url_query_str(req, (char*)buf, buf_len);
    char value[3];
    if (httpd_query_key_value((char*)buf, "mode", (char *)&value, sizeof(value)) == ESP_OK) {
        ESP_LOGI(tag, "MODE value: %d", atoi(value));
    }

    device.device_mode = (uint8_t)atoi(&value[0]);

    mode_changed = 1;

    httpd_resp_set_type(req, "application/json");  // Устанавливаем тип контента как JSON
    httpd_resp_send(req, "{\"status\":\"OK\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t speed_changed_handler(httpd_req_t *req)
{
    uint8_t buf[256];
    size_t buf_len;
    buf_len = httpd_req_get_url_query_len(req) + 1;

    httpd_req_get_url_query_str(req, (char*)buf, buf_len);
    char value[4];
    if (httpd_query_key_value((char*)buf, "speed", (char *)&value, sizeof(value)) == ESP_OK) {
        ESP_LOGI(tag, "MODE value: %d", atoi(value));
    }

    device.speed = (uint8_t)atoi(value);

    httpd_resp_set_type(req, "application/json");  // Устанавливаем тип контента как JSON
    httpd_resp_send(req, "{\"status\":\"OK\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t get_status_handler(httpd_req_t *req)
{
    // Создаем JSON строку для ответа
    char response[256];
    uint16_t currentH;
    uint8_t currentS;
    uint8_t currentV;

    RGB_to_HSV(device.device_color.r, device.device_color.g, device.device_color.b, &currentH, &currentS, &currentV);

    //ESP_LOGI("123", "%d, %d, %d, %d, %d, %d", device.device_color.r, device.device_color.g, device.device_color.b, currentH, currentS, currentV);

    snprintf(response, sizeof(response),
             "{\"power\": \"%s\", \"mode\": %d, \"hue\": %d, \"saturation\": %d, \"brightness\": %d, \"speed\": %d}",
             device.is_On ? "1" : "0",
             device.device_mode,
             currentH,
             (uint8_t)((float)currentS / 255 * 100),
             (uint8_t)((float)currentV / 255 * 100),
             device.speed);

    // Устанавливаем тип ответа как JSON
    httpd_resp_set_type(req, "application/json");
    // Отправляем JSON строку как ответ
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}



// // Обработчик запроса для получения значения датчика
// esp_err_t sensor_handler(httpd_req_t *req) {
//     char response[12];
//     snprintf(response, sizeof(response), "%u", (uint16_t)xTaskGetTickCount());
//     httpd_resp_send(req, response, strlen(response));
//     return ESP_OK;
// }

void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(tag, "WiFi started, connecting to AP...");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            s_retry_num++;
            ESP_LOGI(tag, "Retrying to connect to the AP (attempt %d/%d)...", s_retry_num, MAXIMUM_RETRY);
            esp_wifi_connect();
        } else {
            ESP_LOGE(tag, "Failed to connect after %d attempts. Restarting WiFi stack...", MAXIMUM_RETRY);
            s_retry_num = 0; // Сброс счетчика
            esp_wifi_stop();
            vTaskDelay(pdMS_TO_TICKS(500)); // Задержка перед перезапуском
            esp_wifi_start();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(tag, "Got IP: %s", ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0; // Сброс счетчика при успешном подключении
    }
}
#pragma endregion


#pragma region HelpfulFunc
void getColorByFireTemp(uint8_t temp, uint8_t baseR, uint8_t baseG, uint8_t baseB, uint8_t *outR, uint8_t *outG, uint8_t *outB) 
{
    // Масштабируем значение от 0-255 до 0-240
    temp = (temp * 240) / 255;

    // Определяем яркость базового цвета как max(R, G, B)
    uint8_t baseBrightness = MAX_MACRO(baseR, MAX_MACRO(baseG, baseB));
    float brightnessScale = baseBrightness / 255.0f;

    // Вычисляем цвет X, делая его более "тёплым" по сравнению с базовым цветом
    uint8_t xR = baseR > baseG ? 255 : baseR; // Усиливаем красный компонент
    uint8_t xG = baseG / 2; // Уменьшаем зелёный компонент
    uint8_t xB = baseB / 2; // Уменьшаем синий компонент

    // Масштабируем цвета X с учётом яркости
    xR = (uint8_t)(xR * brightnessScale);
    xG = (uint8_t)(xG * brightnessScale);
    xB = (uint8_t)(xB * brightnessScale);

    if (temp <= 110) {
        // Переход от чёрного к цвету X
        uint8_t scale = (temp * 255) / 80;
        *outR = (xR * scale) / 255;
        *outG = (xG * scale) / 255;
        *outB = (xB * scale) / 255;
    } else if (temp <= 180) {
        // Переход от цвета X к базовому цвету
        uint8_t scale = ((temp - 80) * 255) / 80;
        *outR = xR + ((baseR - xR) * scale / 255);
        *outG = xG + ((baseG - xG) * scale / 255);
        *outB = xB + ((baseB - xB) * scale / 255);
    } else {
        // Переход от базового цвета к белому
        uint8_t scale = ((temp - 160) * 255) / 80;
        *outR = baseR + ((255 - baseR) * scale / 255);
        *outG = baseG + ((255 - baseG) * scale / 255);
        *outB = baseB + ((255 - baseB) * scale / 255);
    }

    // Масштабируем конечные цвета с учётом яркости
    *outR = (uint8_t)(*outR * brightnessScale);
    *outG = (uint8_t)(*outG * brightnessScale);
    *outB = (uint8_t)(*outB * brightnessScale);
}



void HSV_to_RGB(uint16_t h, uint8_t s, uint8_t v, uint8_t* r, uint8_t* g, uint8_t* b)
{
    uint8_t region, remainder, p, q, t;

    if (s == 0) {
        // Если насыщенность равна 0, то цвет — это оттенок серого
        *r = v;
        *g = v;
        *b = v;
        return;
    }

    // Разделим угол (h) на 60 для определения региона (сектора цветового круга)
    region = h / 60;
    remainder = (h % 60) * 255 / 60;

    // Вычислим промежуточные значения
    p = (v * (255 - s)) / 255;
    q = (v * (255 - (s * remainder) / 255)) / 255;
    t = (v * (255 - (s * (255 - remainder)) / 255)) / 255;

    // Определим RGB в зависимости от региона
    switch (region) {
        case 0:
            *r = v;
            *g = t;
            *b = p;
            break;
        case 1:
            *r = q;
            *g = v;
            *b = p;
            break;
        case 2:
            *r = p;
            *g = v;
            *b = t;
            break;
        case 3:
            *r = p;
            *g = q;
            *b = v;
            break;
        case 4:
            *r = t;
            *g = p;
            *b = v;
            break;
        default:
            *r = v;
            *g = p;
            *b = q;
            break;
    }
}



void RGB_to_HSV(uint8_t r, uint8_t g, uint8_t b, uint16_t* h, uint8_t* s, uint8_t* v)
{
    uint8_t rgbMin, rgbMax, delta;
    float h_local;

    rgbMin = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
    rgbMax = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    delta = rgbMax - rgbMin;

    *v = rgbMax; // Яркость

    if (rgbMax == 0) {
        // Если максимальное значение равно 0, то цвет черный, насыщенность также равна 0
        *s = 0;
        *h = 0;
        return;
    }

    // Насыщенность
    *s = (delta * 255) / rgbMax;

    // Определение оттенка
    if (delta == 0) {
        h_local = 0;
    } else if (rgbMax == r) {
        h_local = 60.0 * ((float)(g - b) / delta);
    } else if (rgbMax == g) {
        h_local = 60.0 * (2.0 + (float)(b - r) / delta);
    } else {
        h_local = 60.0 * (4.0 + (float)(r - g) / delta);
    }

    if (h_local < 0) {
        h_local += 360.0;
    }

    *h = (uint16_t)h_local % 360;
}
#pragma endregion