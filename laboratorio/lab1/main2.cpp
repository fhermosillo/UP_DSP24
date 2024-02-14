/* Private Includes --------------------------*/
#include <Arduino.h>
#include "AudioKitHAL.h"

/* Private Defines ---------------------------*/
#define DMA_BUFFER_SIZE 32

#define LED_4  GPIO_NUM_22
#define LED_5  GPIO_NUM_19
#define KEY_1  GPIO_NUM_36
#define KEY_2  GPIO_NUM_13
#define KEY_3  GPIO_NUM_19
#define KEY_4  GPIO_NUM_23
#define KEY_5  GPIO_NUM_18
#define KEY_6  GPIO_NUM_5

#define I2S_BCK_IO  GPIO_NUM_27
#define I2S_WS_IO   GPIO_NUM_25
#define I2S_DOUT_IO GPIO_NUM_26
#define I2S_DIN_IO  GPIO_NUM_35
#define I2S_MCLK_IO GPIO_NUM_0

#define GPIO_LOW    0
#define GPIO_HIGH   1

/* Private Macros ------------------------*/
#define TOFLOAT(X)  ((float)(X)/32767.0F)
#define TOINT16(X)  ((X)*32767.0F)
#define MAX(X,Y)    ((X) > (Y) ? (X) : (Y))
#define MIN(X,Y)    ((X) < (Y) ? (X) : (Y))

/* Private Structures ------------------------*/


/* Private Variables -------------------------*/
AudioKit kit;						//!< Codec driver
int16_t AudioBuffer[DMA_BUFFER_SIZE];	//!< Buffer that stores the data to process

/* Private functions --------------------------*/
void gpio_setup(void);
void i2s_setup(void);

/* Interrupts ---------------------------------*/

/* Setup --------------------------------------*/
void setup() {
  Serial.begin(115200);

  // Setup Codec chip only w/o I2S
  auto cfg = kit.defaultConfig(KitOutput);
  cfg.i2s_active = false;	// Disable I2S autoconfig
  cfg.sample_rate = AUDIO_HAL_08K_SAMPLES;
  kit.begin(cfg);
  kit.setVolume(100);
  
  // I2S setup (user)
  i2s_setup();
  
  // GPIO setup
  gpio_setup();
}

/* Main loop ----------------------------------*/
void loop() {
  /* Signal Sampling */
  // Suspend main thread until the number of bytes determined by the 
  // buffer size is read (this task is usually supended. An interrupt
  // is generated when when the number of bytes are read, and after that
  // the RTOS yield the CPU usage to this task.
  //
  // The "AudioBuffer" variable have both L and R samples alternated, i.e.
  // x_left(0),x_right(0),x_left(1),x_right(1),...,x_left(N-1),x_right(N-1)
  size_t bytesRead = kit.read((uint8_t *)AudioBuffer, sizeof(AudioBuffer));


  /* DSP processing goes here */
  // "AudioBuffer" variable has both Left and Right audio samples
  // n+=2 ensures we take only the Left audio samples
  for(int n = 0; n < bytesRead/2; n++) {
    // int16 to float
    float sample = TOFLOAT(AudioBuffer[n]);
	
	// Process sample by sample
	// BEGIN
	
	// END
	
    // Audio clipping Keeps audio between [-1,1]
    sample = MAX(sample,-1.0F);
    sample = MIN(sample,1.0F);
	
    // float to int16
    AudioBuffer[n] = TOINT16(sample);
  }
  
  /* Signal Interpolation */
  // Suspend main thread until buffer size is read (yield from interrupt)
  // 1. It is better to have a reader and a writter task separately to 
  // avoid suspending audio sampling when sample data is reconstructed
  // 2. A queue can be used to communicate each task that new data has arrived
  // 3. Also consider to use double buffering.
  i2s_write(I2S_NUM_0, AudioBuffer, bytesRead, NULL, portMAX_DELAY);
}








/* Reference functions -----------------------------------------*/
void gpio_setup(void) {
  // Set GPIO to reset state
  gpio_reset_pin(LED_4);
  gpio_reset_pin(LED_5);
  gpio_reset_pin(KEY_1);
  gpio_reset_pin(KEY_2);
  gpio_reset_pin(KEY_3);
  gpio_reset_pin(KEY_4);
  gpio_reset_pin(KEY_5);
  gpio_reset_pin(KEY_6);

  // Set gpio direction
  gpio_set_direction(LED_4, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED_5, GPIO_MODE_OUTPUT);
  gpio_set_direction(KEY_1, GPIO_MODE_INPUT);
  gpio_set_direction(KEY_2, GPIO_MODE_INPUT);
  gpio_set_direction(KEY_3, GPIO_MODE_INPUT);
  gpio_set_direction(KEY_4, GPIO_MODE_INPUT);
  gpio_set_direction(KEY_5, GPIO_MODE_INPUT);
  gpio_set_direction(KEY_6, GPIO_MODE_INPUT);
  
  // Set gpio output level
  gpio_set_level(LED_4, GPIO_HIGH);  // 1: LOW
  gpio_set_level(LED_5, GPIO_HIGH);  // 1: LOW
}

void i2s_setup(void) {
	const i2s_config_t i2s_config = {
		.mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX), // ADC & DAC
		.sample_rate = 8000, // Sampling frequency
		.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // Bits per sample
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Stereo signal (L&R)
		.communication_format = I2S_COMM_FORMAT_STAND_I2S,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
		.dma_buf_count = DMA_NUM_BUFFERS,
		.dma_buf_len = DMA_BUFFER_SIZE,
		.use_apll = true,
		.tx_desc_auto_clear = true,
	};
	
	i2s_pin_config_t pin_config = {
	  .mck_io_num = I2S_MCLK_IO,
	  .bck_io_num = I2S_BCK_IO,
	  .ws_io_num = I2S_WS_IO,
	  .data_out_num = I2S_DOUT_IO,
	  .data_in_num = I2S_DIN_IO
	};
	
	i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
	i2s_set_pin(I2S_NUM_0, &pin_config);
	
	// set MCLK
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
	SET_PERI_REG_BITS(PIN_CTRL, CLK_OUT1, 0, CLK_OUT1_S);
}
