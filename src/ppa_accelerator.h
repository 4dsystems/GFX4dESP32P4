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

public: 
    PPAAccelerator();
    ~PPAAccelerator();
    bool scaleRotateImage(uint16_t* srcPixels, int16_t srcWidth, int16_t srcHeight,
                         uint16_t* dstPixels, int16_t dstWidth, int16_t dstHeight, 
                         bool bSwap, float rotation, bool mirX, bool mirY);
	bool scaleRotateImageFB(uint16_t* srcPixels, int16_t fbWidth, int16_t fbHeight, int fbx, int fby, int16_t srcWidth, int16_t srcHeight,
                         uint16_t* dstPixels, int16_t dstWidth, int16_t dstHeight, 
                         bool bSwap, float rotation, bool mirX, bool mirY);
    bool scaleImage(uint16_t* srcPixels, int16_t srcWidth, int16_t srcHeight,
                   uint16_t* dstPixels, int16_t dstWidth, int16_t dstHeight);
private:
    // Internal helper functions
    ppa_srm_rotation_angle_t convertRotationAngle(float rotation);
};

extern PPAAccelerator ppaAccelerator;

#endif // PPA_ACCELERATOR_H
