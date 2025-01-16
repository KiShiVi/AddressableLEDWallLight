#include "led_strip.h"
//-----------------------------------------------------------
#define LED_STRIP_RMT_DEFAULT_RESOLUTION 10000000 // 10MHz resolution
#define LED_STRIP_RMT_DEFAULT_TRANS_QUEUE_SIZE 4
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define LED_STRIP_RMT_DEFAULT_MEM_BLOCK_SYMBOLS 64
#else
#define LED_STRIP_RMT_DEFAULT_MEM_BLOCK_SYMBOLS 48
#endif
//-----------------------------------------------------------
static const char *TAG = "led_strip";
//-----------------------------------------------------------
typedef struct {
  rmt_encoder_t base;
  rmt_encoder_t *bytes_encoder;
  rmt_encoder_t *copy_encoder;
  int state;
  rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;
//-----------------------------------------------------------
typedef struct {
  led_strip_t base;
  rmt_channel_handle_t rmt_chan;
  rmt_encoder_handle_t strip_encoder;
  uint32_t strip_len;
  uint8_t bytes_per_pixel;
  uint8_t pixel_buf[];
} led_strip_rmt_obj;
//-----------------------------------------------------------
static esp_err_t led_strip_rmt_set_pixel(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
  led_strip_rmt_obj *rmt_strip = __containerof(strip, led_strip_rmt_obj, base);
  ESP_RETURN_ON_FALSE(index < rmt_strip->strip_len, ESP_ERR_INVALID_ARG, TAG, "index out of maximum number of LEDs");
  uint32_t start = index * rmt_strip->bytes_per_pixel;
  //In thr order of GRB, as LED strip like WS2812 sends out pixels in this order
  rmt_strip->pixel_buf[start + 1] = green & 0xFF;
  rmt_strip->pixel_buf[start + 0] = red & 0xFF;
  rmt_strip->pixel_buf[start + 2] = blue & 0xFF;
  if (rmt_strip->bytes_per_pixel > 3) {
    rmt_strip->pixel_buf[start + 3] = 0;
  }
  return ESP_OK;
}
//-----------------------------------------------------------
static esp_err_t led_strip_rmt_set_pixel_rgbw(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue, uint32_t white)
{
  led_strip_rmt_obj *rmt_strip = __containerof(strip, led_strip_rmt_obj, base);
  ESP_RETURN_ON_FALSE(index < rmt_strip->strip_len, ESP_ERR_INVALID_ARG, TAG, "index out of maximum number of LEDs");
  ESP_RETURN_ON_FALSE(rmt_strip->bytes_per_pixel == 4, ESP_ERR_INVALID_ARG, TAG, "wrong LED pixel format, expected 4 bytes per pixel");
  uint8_t *buf_start = rmt_strip->pixel_buf + index * 4;
  //SK6812 component order is GRBW
  *buf_start = red & 0xFF;
  *++buf_start = green & 0xFF;
  *++buf_start = blue & 0xFF;
  *++buf_start = white & 0xFF;
  return ESP_OK;
}
//-----------------------------------------------------------
static esp_err_t led_strip_rmt_set_pixel_goto_color(led_strip_t *strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue, float k)
{
  led_strip_rmt_obj *rmt_strip = __containerof(strip, led_strip_rmt_obj, base);
  ESP_RETURN_ON_FALSE(index < rmt_strip->strip_len, ESP_ERR_INVALID_ARG, TAG, "index out of maximum number of LEDs");
  uint32_t start = index * rmt_strip->bytes_per_pixel;
  //In thr order of GRB, as LED strip like WS2812 sends out pixels in this order
  rmt_strip->pixel_buf[start + 1] = rmt_strip->pixel_buf[start + 1] * (1 - k) + (green & 0xFF) * k;
  rmt_strip->pixel_buf[start + 0] = rmt_strip->pixel_buf[start + 0] * (1 - k) + (red & 0xFF) * k;
  rmt_strip->pixel_buf[start + 2] = rmt_strip->pixel_buf[start + 2] * (1 - k) + (blue & 0xFF) * k;
  if (rmt_strip->bytes_per_pixel > 3) {
    rmt_strip->pixel_buf[start + 3] = 0;
  }
  return ESP_OK;
}
//-----------------------------------------------------------
static esp_err_t led_strip_rmt_refresh(led_strip_t *strip)
{
  led_strip_rmt_obj *rmt_strip = __containerof(strip, led_strip_rmt_obj, base);
  rmt_transmit_config_t tx_conf = {
    .loop_count = 0,
  };
  ESP_RETURN_ON_ERROR(rmt_enable(rmt_strip->rmt_chan), TAG, "enable RMT channel failed");
  ESP_RETURN_ON_ERROR(rmt_transmit(rmt_strip->rmt_chan, rmt_strip->strip_encoder, rmt_strip->pixel_buf,
                                   rmt_strip->strip_len * rmt_strip->bytes_per_pixel, &tx_conf), TAG, "transmit pixels by RMT failed");
  ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(rmt_strip->rmt_chan, -1), TAG, "flush RMT channel failed");
  ESP_RETURN_ON_ERROR(rmt_disable(rmt_strip->rmt_chan), TAG, "disable RMT channel failed");
  return ESP_OK;
}
//-----------------------------------------------------------
static esp_err_t led_strip_rmt_clear(led_strip_t *strip)
{
  led_strip_rmt_obj *rmt_strip = __containerof(strip, led_strip_rmt_obj, base);
  //Write zero to turn off all leds
  memset((void *)rmt_strip->pixel_buf, 0, (size_t)(rmt_strip->strip_len * rmt_strip->bytes_per_pixel));
  return led_strip_rmt_refresh(strip);
}
//-----------------------------------------------------------
static esp_err_t led_strip_rmt_del(led_strip_t *strip)
{
  led_strip_rmt_obj *rmt_strip = __containerof(strip, led_strip_rmt_obj, base);
  ESP_RETURN_ON_ERROR(rmt_del_channel(rmt_strip->rmt_chan), TAG, "delete RMT channel failed");
  ESP_RETURN_ON_ERROR(rmt_del_encoder(rmt_strip->strip_encoder), TAG, "delete strip encoder failed");
  free(rmt_strip);
  return ESP_OK;
}
//-----------------------------------------------------------
static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
  rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
  rmt_encoder_handle_t bytes_encoder = led_encoder->bytes_encoder;
  rmt_encoder_handle_t copy_encoder = led_encoder->copy_encoder;
  rmt_encode_state_t session_state = 0;
  rmt_encode_state_t state = 0;
  size_t encoded_symbols = 0;
  switch (led_encoder->state) {
  case 0: //send RGB data
    encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
    if (session_state & RMT_ENCODING_COMPLETE) {
      led_encoder->state = 1; //switch to next state when current encoding session finished
    }
    if (session_state & RMT_ENCODING_MEM_FULL) {
      state |= RMT_ENCODING_MEM_FULL;
      goto out; //yield if there's no free space for encoding artifacts
    }
  // fall-through
  case 1: //send reset code
    encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_encoder->reset_code,
                                            sizeof(led_encoder->reset_code), &session_state);
    if (session_state & RMT_ENCODING_COMPLETE) {
      led_encoder->state = 0; //back to the initial encoding session
      state |= RMT_ENCODING_COMPLETE;
    }
    if (session_state & RMT_ENCODING_MEM_FULL) {
      state |= RMT_ENCODING_MEM_FULL;
      goto out; //yield if there's no free space for encoding artifacts
    }
  }
out:
  *ret_state = state;
  return encoded_symbols;
}
//-----------------------------------------------------------
static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
  rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
  rmt_del_encoder(led_encoder->bytes_encoder);
  rmt_del_encoder(led_encoder->copy_encoder);
  free(led_encoder);
  return ESP_OK;
}
//-----------------------------------------------------------
static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
{
  rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
  rmt_encoder_reset(led_encoder->bytes_encoder);
  rmt_encoder_reset(led_encoder->copy_encoder);
  led_encoder->state = 0;
  return ESP_OK;
}
//-----------------------------------------------------------
esp_err_t rmt_new_led_strip_encoder(const led_strip_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
  esp_err_t ret = ESP_OK;
  rmt_led_strip_encoder_t *led_encoder = NULL;
  ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
  ESP_GOTO_ON_FALSE(config->led_model < LED_MODEL_INVALID, ESP_ERR_INVALID_ARG, err, TAG, "invalid led model");
  led_encoder = calloc(1, sizeof(rmt_led_strip_encoder_t));
  ESP_GOTO_ON_FALSE(led_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for led strip encoder");
  led_encoder->base.encode = rmt_encode_led_strip;
  led_encoder->base.del = rmt_del_led_strip_encoder;
  led_encoder->base.reset = rmt_led_strip_encoder_reset;
  rmt_bytes_encoder_config_t bytes_encoder_config;
  if (config->led_model == LED_MODEL_SK6812) {
    bytes_encoder_config = (rmt_bytes_encoder_config_t) {
      .bit0 = {
        .level0 = 1,
        .duration0 = 0.3 * config->resolution / 1000000, //T0H=0.3us
        .level1 = 0,
        .duration1 = 0.9 * config->resolution / 1000000, //T0L=0.9us
      },
      .bit1 = {
        .level0 = 1,
        .duration0 = 0.6 * config->resolution / 1000000, //T1H=0.6us
        .level1 = 0,
        .duration1 = 0.6 * config->resolution / 1000000, //T1L=0.6us
      },
    .flags.msb_first = 1 // SK6812 transfer bit order: G7...G0R7...R0B7...B0(W7...W0)
    };
  } else if (config->led_model == LED_MODEL_WS2812) {
    //different led strip might have its own timing requirements, following parameter is for WS2812
    bytes_encoder_config = (rmt_bytes_encoder_config_t) {
      .bit0 = {
        .level0 = 1,
        .duration0 = 0.3 * config->resolution / 1000000, //T0H=0.3us
        .level1 = 0,
        .duration1 = 0.9 * config->resolution / 1000000, //T0L=0.9us
      },
      .bit1 = {
        .level0 = 1,
        .duration0 = 0.9 * config->resolution / 1000000, //T1H=0.9us
        .level1 = 0,
        .duration1 = 0.3 * config->resolution / 1000000, //T1L=0.3us
      },
      .flags.msb_first = 1 //WS2812 transfer bit order: G7...G0R7...R0B7...B0
    };
  } else {
      assert(false);
  }
  ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");
  rmt_copy_encoder_config_t copy_encoder_config = {};
  ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder), err, TAG, "create copy encoder failed");
  uint32_t reset_ticks = config->resolution / 1000000 * 50 / 2; // reset code duration defaults to 50us
  led_encoder->reset_code = (rmt_symbol_word_t) {
    .level0 = 0,
    .duration0 = reset_ticks,
    .level1 = 0,
    .duration1 = reset_ticks,
  };
  *ret_encoder = &led_encoder->base;
  return ESP_OK;
  err:
    if (led_encoder) {
      if (led_encoder->bytes_encoder) {
        rmt_del_encoder(led_encoder->bytes_encoder);
      }
      if (led_encoder->copy_encoder) {
        rmt_del_encoder(led_encoder->copy_encoder);
      }
      free(led_encoder);
    }
  return ret;
}
//-----------------------------------------------------------
esp_err_t led_strip_set_pixel(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue)
{
  ESP_RETURN_ON_FALSE(strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  return strip->set_pixel(strip, index, red, green, blue);
}
//-----------------------------------------------------------
esp_err_t led_strip_set_pixel_rgbw(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue, uint32_t white)
{
  ESP_RETURN_ON_FALSE(strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  return strip->set_pixel_rgbw(strip, index, red, green, blue, white);
}
//-----------------------------------------------------------
esp_err_t led_strip_set_pixel_goto_color(led_strip_handle_t strip, uint32_t index, uint32_t red, uint32_t green, uint32_t blue, float k)
{
  ESP_RETURN_ON_FALSE(strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  return strip->set_pixel_goto_color(strip, index, red, green, blue, k);
}
//-----------------------------------------------------------
esp_err_t led_strip_refresh(led_strip_handle_t strip)
{
  ESP_RETURN_ON_FALSE(strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  return strip->refresh(strip);
}
//-----------------------------------------------------------
esp_err_t led_strip_clear(led_strip_handle_t strip)
{
  ESP_RETURN_ON_FALSE(strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  return strip->clear(strip);
}
//-----------------------------------------------------------
esp_err_t led_strip_del(led_strip_handle_t strip)
{
  ESP_RETURN_ON_FALSE(strip, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  return strip->del(strip);
}
//-----------------------------------------------------------
esp_err_t led_strip_new_rmt_device(const led_strip_config_t *led_config, const led_strip_rmt_config_t *rmt_config, led_strip_handle_t *ret_strip)
{
  led_strip_rmt_obj *rmt_strip = NULL;
  esp_err_t ret = ESP_OK;
  ESP_GOTO_ON_FALSE(led_config && rmt_config && ret_strip, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
  ESP_GOTO_ON_FALSE(led_config->led_pixel_format < LED_PIXEL_FORMAT_INVALID, ESP_ERR_INVALID_ARG, err, TAG, "invalid led_pixel_format");
  uint8_t bytes_per_pixel = 3;
  if (led_config->led_pixel_format == LED_PIXEL_FORMAT_GRBW) {
    bytes_per_pixel = 4;
  } else if (led_config->led_pixel_format == LED_PIXEL_FORMAT_GRB) {
    bytes_per_pixel = 3;
  } else {
    assert(false);
  }
  rmt_strip = calloc(1, sizeof(led_strip_rmt_obj) + led_config->max_leds * bytes_per_pixel);
  ESP_GOTO_ON_FALSE(rmt_strip, ESP_ERR_NO_MEM, err, TAG, "no mem for rmt strip");
  uint32_t resolution = rmt_config->resolution_hz ? rmt_config->resolution_hz : LED_STRIP_RMT_DEFAULT_RESOLUTION;
  rmt_clock_source_t clk_src = RMT_CLK_SRC_DEFAULT;
  if (rmt_config->clk_src) {
    clk_src = rmt_config->clk_src;
  }
  size_t mem_block_symbols = LED_STRIP_RMT_DEFAULT_MEM_BLOCK_SYMBOLS;
  //override the default value if the user sets it
  if (rmt_config->mem_block_symbols) {
    mem_block_symbols = rmt_config->mem_block_symbols;
  }
  rmt_tx_channel_config_t rmt_chan_config = {
    .clk_src = clk_src,
    .gpio_num = led_config->strip_gpio_num,
    .mem_block_symbols = mem_block_symbols,
    .resolution_hz = resolution,
    .trans_queue_depth = LED_STRIP_RMT_DEFAULT_TRANS_QUEUE_SIZE,
    .flags.with_dma = rmt_config->flags.with_dma,
    .flags.invert_out = led_config->flags.invert_out,
  };
  ESP_GOTO_ON_ERROR(rmt_new_tx_channel(&rmt_chan_config, &rmt_strip->rmt_chan), err, TAG, "create RMT TX channel failed");
  led_strip_encoder_config_t strip_encoder_conf = {
    .resolution = resolution,
    .led_model = led_config->led_model
  };
  ESP_GOTO_ON_ERROR(rmt_new_led_strip_encoder(&strip_encoder_conf, &rmt_strip->strip_encoder), err, TAG, "create LED strip encoder failed");
  rmt_strip->bytes_per_pixel = bytes_per_pixel;
  rmt_strip->strip_len = led_config->max_leds;
  rmt_strip->base.set_pixel = led_strip_rmt_set_pixel;
  rmt_strip->base.set_pixel_rgbw = led_strip_rmt_set_pixel_rgbw;
  rmt_strip->base.set_pixel_goto_color = led_strip_rmt_set_pixel_goto_color;
  rmt_strip->base.refresh = led_strip_rmt_refresh;
  rmt_strip->base.clear = led_strip_rmt_clear;
  rmt_strip->base.del = led_strip_rmt_del;
  *ret_strip = &rmt_strip->base;
  return ESP_OK;
  err:
    if (rmt_strip) {
      if (rmt_strip->rmt_chan) {
        rmt_del_channel(rmt_strip->rmt_chan);
      }
      if (rmt_strip->strip_encoder) {
        rmt_del_encoder(rmt_strip->strip_encoder);
      }
      free(rmt_strip);
    }
  return ret;
}
//-----------------------------------------------------------
