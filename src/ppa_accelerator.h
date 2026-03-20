#pragma once
#ifndef PPA_ACCELERATOR_H
#define PPA_ACCELERATOR_H

#include <Arduino.h>

extern "C" {
#include "driver/ppa.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
}

class PPAAccelerator {
private:
    // PPA Hardware Acceleration variables
    ppa_client_handle_t ppa_scaling_handle;
	ppa_client_handle_t ppa_fill_handle;
	ppa_client_handle_t ppa_blend_handle;
    int frmBuffWidth;
	int frmBuffHeight;
	uint8_t tred;
	uint8_t tgreen;
	uint8_t tblue;
	bool transEN;
	float alphaVal;
	bool alphaEN;
public: 
    PPAAccelerator();
    ~PPAAccelerator();
	void setTransAlpha(uint16_t tcol, bool tEN, uint8_t alphVal, bool aEN);
	bool blend(uint16_t* srcPixels, int32_t fbWidth, int32_t fbHeight, int fbx, int fby, int32_t srcWidth, int32_t srcHeight, uint16_t* srcPixels2, int32_t fbWidth2, int32_t fbHeight2, int fbx2, int fby2, int32_t srcWidth2, int32_t srcHeight2, uint16_t* dstPixels, int32_t dstWidth, int32_t dstHeight, bool bSwap, int x, int y, bool fmt24, bool argb, int agrb1 = 0);
	bool blockFill(int16_t fbWidth, int16_t fbHeight, int fbx, int fby, int16_t fbw, int16_t fbh, uint16_t* dstPixels, uint16_t color, int alphB, bool fmt24, bool argb1 = false);
	bool scaleRotateImageFB(uint16_t* srcPixels, int32_t fbWidth, int32_t fbHeight, int fbx, int fby, int32_t srcWidth, int32_t srcHeight,
                         uint16_t* dstPixels, int32_t dstWidth, int32_t dstHeight, 
                         bool bSwap, float rotation, bool mirX, bool mirY, int x, int y, int ofstw, int ofsth, bool fmt24, bool argb = false, int inrgb = 0);
	void setPPAframebufferDimension(int x, int y);
private:
    // Internal helper functions
    ppa_srm_rotation_angle_t convertRotationAngle(float rotation);
};

extern PPAAccelerator ppaAccelerator;

#endif // PPA_ACCELERATOR_H
