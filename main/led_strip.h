#ifndef MAIN_LED_STRIP_H_
#define MAIN_LED_STRIP_H_
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include <sys/cdefs.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/rmt_types.h"
#include "driver/rmt_tx.h"
//-----------------------------------------------------------
typedef struct led_strip_t *led_strip_handle_t;
//-----------------------------------------------------------
typedef struct led_strip_t led_strip_t;
//-----------------------------------------------------------
struct led_strip_t {
  esp_err_t (*set_pixel)(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue);
  esp_err_t (*set_pixel_rgbw)(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue, uint32_t white);
  esp_err_t (*set_pixel_goto_color)(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue, float k);
  esp_err_t (*refresh)(led_strip_t *strip);
  esp_err_t (*clear)(led_strip_t *strip);
  esp_err_t (*del)(led_strip_t *strip);
};
//-----------------------------------------------------------
typedef struct {
  rmt_clock_source_t clk_src;             //RMT clock source
  uint32_t resolution_hz;                 //RMT tick resolution, if set to zero, a default resolution (10MHz) will be applied
  size_t mem_block_symbols;               //How many RMT symbols can one RMT channel hold at one time. Set to 0 will fallback to use the default size.
  struct {
    uint32_t with_dma: 1;                 //Use DMA to transmit data */
  } flags;
} led_strip_rmt_config_t;
//-----------------------------------------------------------
typedef enum {
  LED_PIXEL_FORMAT_GRB,                   //Pixel format: GRB
  LED_PIXEL_FORMAT_GRBW,                  //Pixel format: GRBW
  LED_PIXEL_FORMAT_INVALID                //Invalid pixel format
} led_pixel_format_t;
//-----------------------------------------------------------
typedef enum {
  LED_MODEL_WS2812,                       //LED strip model: WS2812
  LED_MODEL_SK6812,                       //LED strip model: SK6812
  LED_MODEL_INVALID                       //Invalid LED strip model
} led_model_t;
//-----------------------------------------------------------
typedef struct {
    uint32_t resolution;                  //Encoder resolution, in Hz
    led_model_t led_model;                //LED model
} led_strip_encoder_config_t;
//-----------------------------------------------------------
typedef struct {
  int strip_gpio_num;                     //GPIO number that used by LED strip
  uint32_t max_leds;                      //Maximum LEDs in a single strip
  led_pixel_format_t led_pixel_format;    //LED pixel format
  led_model_t led_model;                  //LED model
  struct {
      uint32_t invert_out: 1;             //Invert output signal
  } flags;
} led_strip_config_t;
//-----------------------------------------------------------
esp_err_t led_strip_set_pixel(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue);
esp_err_t led_strip_set_pixel_rgbw(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue, uint32_t white);
esp_err_t led_strip_set_pixel_goto_color(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue, float k);
esp_err_t led_strip_refresh(led_strip_handle_t strip);
esp_err_t led_strip_clear(led_strip_handle_t strip);
esp_err_t led_strip_del(led_strip_handle_t strip);
esp_err_t led_strip_new_rmt_device(const led_strip_config_t *led_config, const led_strip_rmt_config_t *rmt_config, led_strip_handle_t *ret_strip);
//-----------------------------------------------------------
#endif /* MAIN_LED_STRIP_H_ */
