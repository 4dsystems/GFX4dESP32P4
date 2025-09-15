#include "ppa_accelerator.h"

// Global instance
PPAAccelerator ppaAccelerator;

PPAAccelerator::PPAAccelerator() :
    ppa_scaling_handle(nullptr)
{
}

PPAAccelerator::~PPAAccelerator() {
}

bool PPAAccelerator::scaleRotateImage(uint16_t* srcPixels, int16_t srcWidth, int16_t srcHeight,
                                     uint16_t* dstPixels, int16_t dstWidth, int16_t dstHeight, 
                                     bool bSwap, float rotation, bool mirX, bool mirY) {
	ppa_client_config_t ppa_client_config = {
        .oper_type = PPA_OPERATION_SRM,  // Scaling, Rotating, and Mirror operations
		.max_pending_trans_num = 10,
    };
	ppa_register_client(&ppa_client_config, &ppa_scaling_handle);
	if (!ppa_scaling_handle) {
        return false;
    }
    ppa_srm_rotation_angle_t ppa_rotation = convertRotationAngle(rotation);
    if (ppa_rotation == (ppa_srm_rotation_angle_t)-1) {
        ppa_unregister_client(ppa_scaling_handle);
		return false; // Invalid rotation angle
    }
    size_t srcSize = srcWidth * srcHeight * sizeof(uint16_t);
    size_t dstSize = dstWidth * dstHeight * sizeof(uint16_t);    
    ppa_srm_oper_config_t srm_oper_config = {};
    srm_oper_config.in.buffer = srcPixels;//ppa_src_buffer;
    srm_oper_config.in.pic_w = srcWidth;
    srm_oper_config.in.pic_h = srcHeight;
    srm_oper_config.in.block_w = srcWidth;
    srm_oper_config.in.block_h = srcHeight;
    srm_oper_config.in.block_offset_x = 0;
    srm_oper_config.in.block_offset_y = 0;
    srm_oper_config.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    srm_oper_config.out.buffer = dstPixels;//ppa_dst_buffer;
    srm_oper_config.out.buffer_size = 2048000;//ppa_dst_buffer_size;
    if (ppa_rotation == PPA_SRM_ROTATION_ANGLE_90 || ppa_rotation == PPA_SRM_ROTATION_ANGLE_270){
		srm_oper_config.out.pic_w = dstHeight;
		srm_oper_config.out.pic_h = dstWidth;
	} else {
		srm_oper_config.out.pic_w = dstWidth;
		srm_oper_config.out.pic_h = dstHeight;
	}
    srm_oper_config.out.block_offset_x = 0;
    srm_oper_config.out.block_offset_y = 0;
    srm_oper_config.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    //if (ppa_rotation == PPA_SRM_ROTATION_ANGLE_90 || ppa_rotation == PPA_SRM_ROTATION_ANGLE_270){
    //    srm_oper_config.scale_x = (float)dstWidth / srcHeight;
	//	srm_oper_config.scale_y = (float)dstHeight / srcWidth;
	//} else {		
		srm_oper_config.scale_x = (float)dstWidth / srcWidth;
		srm_oper_config.scale_y = (float)dstHeight / srcHeight;
    //}
    srm_oper_config.rotation_angle = ppa_rotation;
    srm_oper_config.mirror_x = mirX;
    srm_oper_config.mirror_y = mirY;
    srm_oper_config.rgb_swap = false;
    srm_oper_config.byte_swap = bSwap;
    srm_oper_config.alpha_update_mode = PPA_ALPHA_NO_CHANGE;
    srm_oper_config.mode = PPA_TRANS_MODE_BLOCKING;
    srm_oper_config.user_data = nullptr;
    esp_err_t ret = ppa_do_scale_rotate_mirror(ppa_scaling_handle, &srm_oper_config);
    if (ret != ESP_OK) {
        ppa_unregister_client(ppa_scaling_handle);
		return false;
    }
    ppa_unregister_client(ppa_scaling_handle);
	return true;
}

bool PPAAccelerator::scaleRotateImageFB(uint16_t* srcPixels, int16_t fbWidth, int16_t fbHeight, int fbx, int fby, int16_t srcWidth, int16_t srcHeight,
                                     uint16_t* dstPixels, int16_t dstWidth, int16_t dstHeight, 
                                     bool bSwap, float rotation, bool mirX, bool mirY) {
	ppa_client_config_t ppa_client_config = {
        .oper_type = PPA_OPERATION_SRM,  // Scaling, Rotating, and Mirror operations
		.max_pending_trans_num = 10,
    };
	ppa_register_client(&ppa_client_config, &ppa_scaling_handle);
	if (!ppa_scaling_handle) {
        return false;
    }
    ppa_srm_rotation_angle_t ppa_rotation = convertRotationAngle(rotation);
    if (ppa_rotation == (ppa_srm_rotation_angle_t)-1) {
        ppa_unregister_client(ppa_scaling_handle);
		return false; // Invalid rotation angle
    }   
    ppa_srm_oper_config_t srm_oper_config = {};
    srm_oper_config.in.buffer = srcPixels;//ppa_src_buffer;
    srm_oper_config.in.pic_w = fbWidth;
    srm_oper_config.in.pic_h = fbHeight;
    srm_oper_config.in.block_w = srcWidth;
    srm_oper_config.in.block_h = srcHeight;
    srm_oper_config.in.block_offset_x = fbx;
    srm_oper_config.in.block_offset_y = fby;
    srm_oper_config.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    srm_oper_config.out.buffer = dstPixels;//ppa_dst_buffer;
    srm_oper_config.out.buffer_size = 2048000;//ppa_dst_buffer_size;
    if (ppa_rotation == PPA_SRM_ROTATION_ANGLE_90 || ppa_rotation == PPA_SRM_ROTATION_ANGLE_270){
		srm_oper_config.out.pic_w = dstHeight;
		srm_oper_config.out.pic_h = dstWidth;
	} else {
		srm_oper_config.out.pic_w = dstWidth;
		srm_oper_config.out.pic_h = dstHeight;
	}
    srm_oper_config.out.block_offset_x = 0;
    srm_oper_config.out.block_offset_y = 0;
    srm_oper_config.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
	srm_oper_config.rotation_angle = ppa_rotation;
    //if (ppa_rotation == PPA_SRM_ROTATION_ANGLE_90 || ppa_rotation == PPA_SRM_ROTATION_ANGLE_270){
    //    srm_oper_config.scale_x = (float)dstWidth / srcHeight;
	//	srm_oper_config.scale_y = (float)dstHeight / srcWidth;
	//} else {		
		srm_oper_config.scale_x = (float)dstWidth / srcWidth;
		srm_oper_config.scale_y = (float)dstHeight / srcHeight;
    //}
    srm_oper_config.mirror_x = mirX;
    srm_oper_config.mirror_y = mirY;
    srm_oper_config.rgb_swap = false;
    srm_oper_config.byte_swap = bSwap;
    srm_oper_config.alpha_update_mode = PPA_ALPHA_NO_CHANGE;
    srm_oper_config.mode = PPA_TRANS_MODE_BLOCKING;
    srm_oper_config.user_data = nullptr;
    esp_err_t ret = ppa_do_scale_rotate_mirror(ppa_scaling_handle, &srm_oper_config);
    if (ret != ESP_OK) {
        ppa_unregister_client(ppa_scaling_handle);
		return false;
    }
    return true;
}

bool PPAAccelerator::scaleImage(uint16_t* srcPixels, int16_t srcWidth, int16_t srcHeight,
                               uint16_t* dstPixels, int16_t dstWidth, int16_t dstHeight) {
    return scaleRotateImage(srcPixels, srcWidth, srcHeight, dstPixels, dstWidth, dstHeight, false, 0.0, false, false);
}

ppa_srm_rotation_angle_t PPAAccelerator::convertRotationAngle(float rotation) {
    if (rotation == 0.0) {
        return PPA_SRM_ROTATION_ANGLE_0;
    } else if (rotation == 90.0) {
        return PPA_SRM_ROTATION_ANGLE_90;
    } else if (rotation == 180.0) {
        return PPA_SRM_ROTATION_ANGLE_180;
    } else if (rotation == 270.0) {
        return PPA_SRM_ROTATION_ANGLE_270;
    } else {
        return (ppa_srm_rotation_angle_t)-1; // Invalid
    }
}
