/**
  ******************************************************************************
  * @file       iirfilter.h
  * @author     Fernando Hermosillo Reynoso
  * @brief      This file contains the prototype & definition functions for the
  *             iir filter.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 Universidad Panamericana.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file in
  * the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IIRFILTER_H
#define __IIRFILTER_H


/* Includes ------------------------------------------------------------------*/
#include <cmath>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/* Exported classes ----------------------------------------------------------*/
class IIRFilter
{
  public:
    IIRFilter(size_t order, const float *a, const float *b);

    float filter(float sample);
    void reset(void);
	
    ~IIRFilter();
    
  private:
    size_t mOrder;
    const float *mCoefOut;
    const float *mCoefInp;
    float *mInputDelayed;
    float *mOutputDelayed;
};


IIRFilter::IIRFilter(size_t order, const float *a, const float *b) {
  mOrder = order;
  mCoefOut = a;
  mCoefInp = b;

  mInputDelayed = new float [mOrder];
  mOutputDelayed = new float [mOrder];
}

void IIRFilter::reset(void) {
	if(mInputDelayed)
	{
		for(int n = 0; n < mOrder; n++)
		{
			mInputDelayed[n] = 0.0F;
		}
	}

	if(mOutputDelayed)
	{
		for(int n = 0; n < mOrder-1; n++)
		{
			mOutputDelayed[n] = 0.0F;
		}
	}
}

float IIRFilter::filter(float x) {
	// Do convolution
	float y = x*mCoefInp[0]; // y(n) = b0 * x(n)
	for(int n = 0; n < mOrder; n++) {
		y += mInputDelayed[n] * mCoefInp[n+1];
		if(n < mOrder-1) y -= mOutputDelayed[n] * mCoefOut[n];
	}

	// Update next previous samples
	for(int n = mOrder-1; n; n--)
	{
		mInputDelayed[n] = mInputDelayed[n-1];
		if(n < mOrder-1) mOutputDelayed[n] = mOutputDelayed[n-1];
	}
	mInputDelayed[0] = x;
	mOutputDelayed[0] = y;
	
	// Return output
	return y;
}

IIRFilter::~IIRFilter() {
  
}

#endif /* __IIRFILTER_H */
