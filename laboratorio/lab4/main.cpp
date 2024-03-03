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

#define GPIO_LOW    0
#define GPIO_HIGH   1

#define DTMF_THRESHOLD	0.8

/* Private Macros ------------------------*/
#define TOFLOAT(X)  ((float)(X)/32767.0F)
#define TOINT16(X)  ((X)*32767.0F)
#define MAX(X,Y)    ((X) > (Y) ? (X) : (Y))
#define MIN(X,Y)    ((X) < (Y) ? (X) : (Y))

/* Private Structures ------------------------*/
typedef struct {
  float re;	//!< Real term of complex number
  float im;	//!< Imaginary term of complex number
} complex_t;

typedef struct {
  float cosw0;		//!< cos(w0) constant term
  complex_t expw0;	//!< e^(-i*w0) constant term
  float sdelay[2];	//!< s(n) delay
  uint16_t N;		//!< Number of samples to process
} goertzel_t;


/* Private Variables -------------------------*/
AudioKit kit;						//!< Audio codec driver
int16_t AudioBuffer[BUFFER_SIZE];	//!< Buffer that stores the data to process
goertzel_t gfilt;					//!< Goertzel filter for a specific frequency
uint32_t nn = 0;					//!< To track time for the Goertzel filter

/* Private functions --------------------------*/
void audiokit_gpio_init(void);

void goertzel_init(goertzel_t *gf, int N, int k);
void goertzel_reset(goertzel_t *gf);
float goertzel_process(goertzel_t *gf, float x);


/* Interrupts ---------------------------------*/


/* Setup --------------------------------------*/
void setup() {
  Serial.begin(115200);

  // Goertzel filter init
  // Fs = 8000 Hz
  // N = 256
  // Frequency resolution = Fs/N = 31.25 Hz
  // Frequency to detect Fd = 770
  // Fd = k*Fs/N -> k = Fd*N/Fs = 770*256/8000 = 24.64
  // k should be an integer, therefore, we set k as
  // the nearest integer k = 25
  //
  // Using this k value, the actual frequency will be 
  //	Fd' = k*Fs/N = 25*8000/256 = 781.25 Hz
  // It is very close to the desired frequency of 770
  goertzel_init(&gfilt, 256, 25);

  // I2S Config
  auto cfg = kit.defaultConfig(KitInputOutput);
  cfg.adc_input = AUDIO_HAL_ADC_INPUT_LINE2;	// MICROPHONE/AUXIN audio input
  cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL;	// SPEAKER/HEADPHONE audio output
  cfg.sample_rate = AUDIO_HAL_08K_SAMPLES;		// Sampling frequency (Fs)
  cfg.bits_per_sample = AUDIO_HAL_BIT_LENGTH_16BITS;	// Number of bits per sample
  cfg.buffer_size=32;	// DMA buffer size (Each entry stores a variable of bits_per_sample)
  kit.begin(cfg);		// Initialize ES8388 audio codec
  kit.setVolume(100);	// Set audio codec volume to 100

  // BSP audiokit gpio initialization
  audiokit_gpio_init();
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
  for(int n = 0; n < bytesRead/2; n+=2) {
    // int16 to float
    float sample = TOFLOAT(AudioBuffer[n]);
	
	// DSP BEGIN
    // Perform energy computation for the actual frequency
	float energy_fx = goertzel_process(&gfilt, sample);	// Compute the Goertzel algorithm
	
	// Check if a valid energy is obtained
	if(++nn == gfilt.N)
	{
		nn = 0; // Reset time
		// Check if E[y(n)]^2 > Threshold
		if(energy_fx >= 0.8)
		{
			gpio_set_level(LED_4, GPIO_HIGH);
		}
		else
		{
			gpio_set_level(LED_4, GPIO_LOW);
		}
	}
	
    // audio clipping
    sample = MAX(sample,-1.0F);
    sample = MIN(sample,1.0F);

    // float to int16
    AudioBuffer[n] = TOINT16(sample);
  }
  // DSP END
  
  /* Signal Interpolation */
  // Suspend main thread until buffer size is read (yield from interrupt)
  // 1. It is better to have a reader and a writter task separately to 
  // avoid suspending audio sampling when sample data is reconstructed
  // 2. A queue can be used to communicate each task that new data has arrived
  // 3. Also consider to use double buffering.
  // 4. Although, data reconstruction is not necessary in this example
  //    and you can comment this line
  kit.write((uint8_t *)AudioBuffer, bytesRead);
}








/* Reference functions -----------------------------------------*/
void audiokit_gpio_init(void) {
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

  gpio_reset_pin(GPIO_NUM_12);
  gpio_set_direction(GPIO_NUM_12, GPIO_MODE_INPUT);
  gpio_pullup_en(GPIO_NUM_12);
  gpio_set_pull_mode(GPIO_NUM_12, GPIO_PULLUP_ONLY);
}



void goertzel_init(goertzel_t *gf, float Fs, float freq, int N) {
	gf->N = N;
	gf->cos_wk = cosf(2*M_PI*(float)k/(float)N);
	gf->exp_mwk.re = gf->cos_wk;
	gf->exp_mwk.im = -sinf(2*M_PI*(float)k/(float)N); // exp(-j*wk)

	// Condiciones iniciales
	goertzel_reset(gf);
}

void goertzel_reset(goertzel_t *gf) {
	// Condiciones iniciales
	gf->sprev[0] = 0.0F;
	gf->sprev[1] = 0.0F;
}

float goertzel_process(goertzel_t *gf, float x) {
	// s(n) = x(n) + 2*cos(w0)*s(n - 1) - s(n - 2)
	float sn = x + 2.0F*g->cosw0*g->sdelay[0] - g->sdelay[1];
	
	// y(n) = s(n) - exp(-j*w0)*s(n - 1) = s(n) - [cos(w0) - jsin(w0)]s(n - 1)
	//      = s(n) - cos(w0)*s(n - 1) - j*sin(w0)*s(n - 1)
	complex_t y;
	y.re = sn - g->expw0.re*g->sdelay[0];	// Re{y(n)} = s(n) - cos(w0)*s(n - 1)
	y.im = - g->expw0.im*g->sdelay[0];		// Im{y(n)} = -sin(w0)*s(n - 1)
	
	// Mag^2 = Re{y(n)}^2 + Im{y(n)}^2
	float mag_sqr = y.re*y.re + y.im*y.im;
	
	// Shift samples
	g->sdelay[1] = g->sdelay[0];
	g->sdelay[0] = sn;
	
	return mag_sqr;
}
