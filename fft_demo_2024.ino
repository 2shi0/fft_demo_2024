/*
 Addisional Boards Manager URLs:
   - https://static-cdn.m5stack.com/resource/arduino/package_m5stack_index.json

 liblary versions:
   - arduinoFFT 2.0.2
   - M5StickCPlus 0.1.0
  and more
*/

//=====================================
// fft program
// referenced from
// https://github.com/kosme/arduinoFFT/wiki
// https://homemadegarbage.com/m5stickc02
//=====================================
#include "arduinoFFT.h"

const int sampling_frequency = 44100;
const uint16_t samples = 1024;  // サンプル数は2のべき乗

float v_real[samples];  // v_real[]にサンプリングしたデーターを入れる
float v_imag[samples];
unsigned int sampling_period_us = round(1000000 * (1.0 / sampling_frequency));
uint16_t *adc_buffer = NULL;

ArduinoFFT<float> FFT = ArduinoFFT<float>(v_real, v_imag, samples, sampling_frequency); /* Create FFT object */

void fft() {
  for (int i = 0; i < samples; i++) {
    unsigned long t = micros();
    v_real[i] = adc_buffer[i];
    v_imag[i] = 0;
    while ((micros() - t) < sampling_period_us)
      ;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */
  FFT.compute(FFTDirection::Forward);                       /* Compute FFT */
  FFT.complexToMagnitude();                                 /* Compute magnitudes */
  float x = FFT.majorPeak();
  Serial.print(x);
  Serial.print(" Hz | ");
  Serial.print(get_loudness());
  Serial.println(" dB?");

  /*
  for (int band = 0; band < nsamples; band++) {
    float d = v_real[band] / dmax;
    Serial.print(band);
    Serial.print(" : ");
    Serial.print((band * 1.0 * SAMPLING_FREQUENCY) / samples / 1000);
    Serial.print("kHz : ");
    Serial.println(d);
  }
  */
}

//=====================================
// i2s microphone
// referenced from Examples (M5StickCPlus/Basic/Microphone)
//=====================================
#include <M5StickCPlus.h>
#include <driver/i2s.h>

#define PIN_CLK 0
#define PIN_DATA 34

const int read_len = 2 * samples;
uint8_t buffer[read_len] = { 0 };

void i2sInit() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = 44100,
    .bits_per_sample =
      I2S_BITS_PER_SAMPLE_16BIT,  // is fixed at 12bit, stereo, MSB
    .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
#else
    .communication_format = I2S_COMM_FORMAT_I2S,
#endif
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 128,
  };

  i2s_pin_config_t pin_config;

#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
  pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif

  pin_config.bck_io_num = I2S_PIN_NO_CHANGE;
  pin_config.ws_io_num = PIN_CLK;
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num = PIN_DATA;

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}

void mic_record_task(void *arg) {
  size_t bytesread;
  while (1) {
    i2s_read(I2S_NUM_0, (char *)buffer, read_len, &bytesread, (100 / portTICK_RATE_MS));
    adc_buffer = (uint16_t *)buffer;
    fft();
    vTaskDelay(100 / portTICK_RATE_MS);
  }
}

//=====================================
// Measuring Sound Loudness
// (WIP)
//=====================================
float get_loudness() {
  int sum = 0;
  for (int i = 0; i < read_len; i++) {
    sum += buffer[i];
  }
  return sum / read_len;
}

//=====================================
// tft screen
// referenced from https://lawn-tech.jp/m5stickc_sprite.html
//=====================================

//=====================================
// main code
//=====================================
void setup() {

  Serial.begin(115200);

  i2sInit();
  xTaskCreatePinnedToCore(mic_record_task, "mic_record_task", 2048, NULL, 1, NULL, 1);
}

void loop() {
  printf("loop cycling\n");

  vTaskDelay(1000 / portTICK_RATE_MS);  // otherwise the main task wastes half
                                        // of the cpu cycles
}