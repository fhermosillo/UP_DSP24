/*
Library:					STM32F4 Audio Codec - CS43L22
Written by:				Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Date Written:			29/01/2016
Last modified:			29/12/2018
Description:			This is an STM32 device driver library for the CS43L22 Audio Codec, using STM HAL libraries
References:
			1) Cirrus Logic CS43L22 datasheet
				 https://www.mouser.com/ds/2/76/CS43L22_F2-1142121.pdf
			2) ST opensource CS43L22 Audio Codec dsp drivers.

* Copyright (C) 2018 - M. Yaqoob
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.

   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

#include "cs43l22.h"

/* Private define ------------------------------------------------------------*/
#define POWER_CONTROL1						0x02
#define POWER_CONTROL2						0x04
#define CLOCKING_CONTROL 	  				0x05
#define INTERFACE_CONTROL1					0x06
#define INTERFACE_CONTROL2					0x07
#define PASSTHROUGH_A						0x08
#define PASSTHROUGH_B						0x09
#define MISCELLANEOUS_CONTRLS				0x0E
#define PLAYBACK_CONTROL					0x0F
#define PASSTHROUGH_VOLUME_A				0x14
#define PASSTHROUGH_VOLUME_B				0x15
#define PCM_VOLUME_A						0x1A
#define PCM_VOLUME_B						0x1B
#define CONFIG_00							0x00
#define CONFIG_47							0x47
#define CONFIG_32							0x32

#define CS43L22_REG_MASTER_A_VOL        	0x20
#define CS43L22_REG_MASTER_B_VOL        	0x21

#define CS43L22_I2C_ADDR 					0x94

/* Private macro -------------------------------------------------------------*/
#define VOLUME_CONVERT_A(Volume)    (((Volume) > 100)? 255:((uint8_t)(((Volume) * 255) / 100)))
#define VOLUME_CONVERT_D(Volume)    (((Volume) > 100)? 24:((uint8_t)((((Volume) * 48) / 100) - 24)))

/* Private variables ---------------------------------------------------------*/
static uint8_t ucI2CBuffer[2];
static I2C_HandleTypeDef *hi2c = NULL;
static I2S_HandleTypeDef *hi2s = NULL;

/* Private function reference ---------------------------------------------------------*/
// Write to register
static void cs43l22_write_register(uint8_t reg, uint8_t *data)
{
	ucI2CBuffer[0] = reg;
	ucI2CBuffer[1] = data[0];
	HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, ucI2CBuffer, 2, 100);
}
// Read from register
static void cs43l22_read_register(uint8_t reg, uint8_t *data)
{
	ucI2CBuffer[0] = reg;
	HAL_I2C_Master_Transmit(hi2c, CS43L22_I2C_ADDR, ucI2CBuffer, 1, 100);
	HAL_I2C_Master_Receive(hi2c,  CS43L22_I2C_ADDR,  data, 1, 100);
}

/* Public function reference ---------------------------------------------------------*/
// Initialisation
void MX_CS43L22_Init(I2S_HandleTypeDef *hi2sx, I2C_HandleTypeDef *hi2cx, cs43l22_mode_t mode)
{
	if(hi2sx == NULL || hi2cx == NULL) return;

	// Get handlers
	hi2c = hi2cx;
	hi2s = hi2sx;

	// Enable I2S
	__HAL_UNLOCK(hi2s);     // THIS IS EXTREMELY IMPORTANT FOR I2S3 TO WORK!!
	__HAL_I2S_ENABLE(hi2s); // THIS IS EXTREMELY IMPORTANT FOR I2S3 TO WORK!!

	// Reset CS43L22 driver
	HAL_GPIO_WritePin(Audio_RST_GPIO_Port, Audio_RST_Pin, GPIO_PIN_SET);

	// Power down
	ucI2CBuffer[1] = 0x01;
	cs43l22_write_register(POWER_CONTROL1, ucI2CBuffer);

	// Enable Right and Left headphones
	ucI2CBuffer[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always on)
	ucI2CBuffer[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
	ucI2CBuffer[1] |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
	ucI2CBuffer[1] |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
	cs43l22_write_register(POWER_CONTROL2, &ucI2CBuffer[1]);

	// Automatic clock detection
	ucI2CBuffer[1] = (1 << 7);
	cs43l22_write_register(CLOCKING_CONTROL, &ucI2CBuffer[1]);

	// Interface control 1
	cs43l22_read_register(INTERFACE_CONTROL1, ucI2CBuffer);
	ucI2CBuffer[1] &= (1 << 5); 	// Clear all bits except bit 5 which is reserved
	ucI2CBuffer[1] &= ~(1 << 7);  	// Slave
	ucI2CBuffer[1] &= ~(1 << 6);  	// Clock polarity: Not inverted
	ucI2CBuffer[1] &= ~(1 << 4);  	// No DSP mode
	ucI2CBuffer[1] &= ~(1 << 2);  	// Left justified, up to 24 bit (default)
	ucI2CBuffer[1] |=  (1 << 2);
	ucI2CBuffer[1] |=  (3 << 0);  // 16-bit audio word length for I2S interface
	cs43l22_write_register(INTERFACE_CONTROL1,&ucI2CBuffer[1]);

	// Passthrough A settings
	cs43l22_read_register(PASSTHROUGH_A, &ucI2CBuffer[1]);
	ucI2CBuffer[1] &= 0xF0;      // Bits [4-7] are reserved
	ucI2CBuffer[1] |=  (1 << 0); // Use AIN1A as source for passthrough
	cs43l22_write_register(PASSTHROUGH_A,&ucI2CBuffer[1]);

	// Passthrough B settings
	cs43l22_read_register(PASSTHROUGH_B, &ucI2CBuffer[1]);
	ucI2CBuffer[1] &= 0xF0;      // Bits [4-7] are reserved
	ucI2CBuffer[1] |=  (1 << 0); // Use AIN1B as source for passthrough
	cs43l22_write_register(PASSTHROUGH_B,&ucI2CBuffer[1]);

	// Miscellaneous register settings
	cs43l22_read_register(MISCELLANEOUS_CONTRLS, &ucI2CBuffer[1]);
	if(mode == CS43L22_MODE_ANALOG)
	{
		ucI2CBuffer[1] |=  (1 << 7);   // Enable passthrough for AIN-A
		ucI2CBuffer[1] |=  (1 << 6);   // Enable passthrough for AIN-B
		ucI2CBuffer[1] &= ~(1 << 5);   // Unmute passthrough on AIN-A
		ucI2CBuffer[1] &= ~(1 << 4);   // Unmute passthrough on AIN-B
		ucI2CBuffer[1] &= ~(1 << 3);   // Changed settings take affect immediately
	}
	else if(mode == CS43L22_MODE_I2S)
	{
		ucI2CBuffer[1] = 0x02;
	}
	cs43l22_write_register(MISCELLANEOUS_CONTRLS,&ucI2CBuffer[1]);

	// Unmute headphone and speaker
	cs43l22_read_register(PLAYBACK_CONTROL, &ucI2CBuffer[1]);
	ucI2CBuffer[1] = 0x00;
	cs43l22_write_register(PLAYBACK_CONTROL,&ucI2CBuffer[1]);

	// Set volume to default (0dB)
	ucI2CBuffer[1] = 0x00;
	cs43l22_write_register(PASSTHROUGH_VOLUME_A,&ucI2CBuffer[1]);
	cs43l22_write_register(PASSTHROUGH_VOLUME_B,&ucI2CBuffer[1]);
	cs43l22_write_register(PCM_VOLUME_A,&ucI2CBuffer[1]);
	cs43l22_write_register(PCM_VOLUME_B,&ucI2CBuffer[1]);
}

// Function(2): Enable Right and Left headphones
void HAL_CS43L22_Set_Channel(cs43l22_channel_t channel)
{
	switch (channel)
	{
		case CS43L22_CHANNEL_MUTE:
			ucI2CBuffer[1] =  (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			ucI2CBuffer[1] |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
		break;

		case CS43L22_CHANNEL_RIGHT:
			ucI2CBuffer[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			ucI2CBuffer[1] |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
		break;

		case CS43L22_CHANNEL_LEFT:
			ucI2CBuffer[1] =  (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			ucI2CBuffer[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
		break;

		case CS43L22_CHANNEL_RIGHT_LEFT:
			ucI2CBuffer[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			ucI2CBuffer[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
		break;

		default:
		break;
	}
	ucI2CBuffer[1] |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
	ucI2CBuffer[1] |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
	cs43l22_write_register(POWER_CONTROL2,&ucI2CBuffer[1]);
}

// Function(3): Set Volume Level
void HAL_CS43L22_Set_Volume(uint8_t volume)
{
	// Convert volume to register
	int8_t tempVol = volume - 50;
	tempVol = tempVol*(127/50);
	uint8_t myVolume =  (uint8_t )tempVol;
	ucI2CBuffer[1] = myVolume;
	cs43l22_write_register(PASSTHROUGH_VOLUME_A,&ucI2CBuffer[1]);
	cs43l22_write_register(PASSTHROUGH_VOLUME_B,&ucI2CBuffer[1]);
	ucI2CBuffer[1] = VOLUME_CONVERT_D(volume);

	/* Set the Master volume */
	cs43l22_write_register(CS43L22_REG_MASTER_A_VOL,&ucI2CBuffer[1]);
	cs43l22_write_register(CS43L22_REG_MASTER_B_VOL,&ucI2CBuffer[1]);
}

// Set bits per sample
void HAL_CS43L22_Set_Bps(cs43l22_bps_t bit_depth)
{
	cs43l22_read_register(INTERFACE_CONTROL1, ucI2CBuffer);
	ucI2CBuffer[1] &= ~ 0x03; // 24-bits
	ucI2CBuffer[1] |= (uint8_t)bit_depth;
	cs43l22_write_register(CS43L22_REG_MASTER_A_VOL,&ucI2CBuffer[1]);
}

// Function(4): Start the Audio DAC
void HAL_CS43L22_Start(void)
{
	// Write 0x99 to register 0x00.
	ucI2CBuffer[1] = 0x99;
	cs43l22_write_register(CONFIG_00, &ucI2CBuffer[1]);

	// Write 0x80 to register 0x47.
	ucI2CBuffer[1] = 0x80;
	cs43l22_write_register(CONFIG_47, &ucI2CBuffer[1]);

	// Write '1'b to bit 7 in register 0x32.
	cs43l22_read_register(CONFIG_32, &ucI2CBuffer[1]);
	ucI2CBuffer[1] |= 0x80;
	cs43l22_write_register(CONFIG_32, &ucI2CBuffer[1]);

	// Write '0'b to bit 7 in register 0x32.
	cs43l22_read_register(CONFIG_32, &ucI2CBuffer[1]);
	ucI2CBuffer[1] &= ~(0x80);
	cs43l22_write_register(CONFIG_32, &ucI2CBuffer[1]);

	// Write 0x00 to register 0x00.
	ucI2CBuffer[1] = 0x00;
	cs43l22_write_register(CONFIG_00, &ucI2CBuffer[1]);

	//Set the "Power Ctl 1" register (0x02) to 0x9E
	ucI2CBuffer[1] = 0x9E;
	cs43l22_write_register(POWER_CONTROL1,&ucI2CBuffer[1]);
}

void HAL_CS43L22_Stop(void)
{
	ucI2CBuffer[1] = 0x01;
	cs43l22_write_register(POWER_CONTROL1, &ucI2CBuffer[1]);
}
