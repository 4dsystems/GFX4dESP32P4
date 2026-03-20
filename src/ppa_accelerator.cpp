#include "ppa_accelerator.h"

#define ppa_Swap(a, b)                                                              \
    {                                                                           \
        int32_t tab = a;                                                        \
        a = b;                                                                  \
        b = tab;                                                                \
    }

// Global instance
PPAAccelerator ppaAccelerator;

PPAAccelerator::PPAAccelerator() :
    ppa_scaling_handle(nullptr)
{
}

PPAAccelerator::~PPAAccelerator() {
}

bool PPAAccelerator::blend(uint16_t* srcPixels, int32_t fbWidth, int32_t fbHeight, int fbx, int fby, int32_t srcWidth, int32_t srcHeight,
                                     uint16_t* srcPixels2, int32_t fbWidth2, int32_t fbHeight2, int fbx2, int fby2, int32_t srcWidth2, int32_t srcHeight2,
									 uint16_t* dstPixels, int32_t dstWidth, int32_t dstHeight, 
                                     bool bSwap, int x, int y, bool fmt24, bool argb, int argb1){
	ppa_client_config_t ppa_blend_config = {
        .oper_type = PPA_OPERATION_BLEND,
        .max_pending_trans_num = 10,
    };
	if(!ppa_blend_handle) ppa_register_client(&ppa_blend_config, &ppa_blend_handle);
	if (!ppa_blend_handle) {
        return false;
    }
	 int mul = 2;
    if (argb){
		mul = 4;
	} else if (fmt24){
		mul = 3;
	}
    ppa_blend_oper_config_t blend_config = {};
		blend_config.in_bg.buffer = srcPixels;
        blend_config.in_bg.pic_w = fbWidth;
        blend_config.in_bg.pic_h = fbHeight;
        blend_config.in_bg.block_w = srcWidth;
        blend_config.in_bg.block_h = srcHeight;
        blend_config.in_bg.block_offset_x = fbx;
        blend_config.in_bg.block_offset_y = fby;
        blend_config.in_bg.blend_cm = PPA_BLEND_COLOR_MODE_RGB565;
        blend_config.in_fg.buffer = srcPixels2;
        blend_config.in_fg.pic_w = fbWidth2;
        blend_config.in_fg.pic_h = fbHeight2;
        blend_config.in_fg.block_w = srcWidth2;
        blend_config.in_fg.block_h = srcHeight2;
        blend_config.in_fg.block_offset_x = fbx2;
        blend_config.in_fg.block_offset_y = fby2;
        if (argb) {
			blend_config.in_fg.blend_cm = PPA_BLEND_COLOR_MODE_ARGB8888;
		} else {
			blend_config.in_fg.blend_cm = PPA_BLEND_COLOR_MODE_RGB565;
		}
        blend_config.out.buffer = dstPixels;
		blend_config.out.buffer_size = (frmBuffWidth * frmBuffHeight) * mul;
        blend_config.out.pic_w = frmBuffWidth;
        blend_config.out.pic_h = frmBuffHeight;
        blend_config.out.block_offset_x = x;
        blend_config.out.block_offset_y = y;
        switch (argb1){
		    case 2:
			blend_config.out.blend_cm = PPA_BLEND_COLOR_MODE_ARGB8888;
				break;
			case 1:
			blend_config.out.blend_cm = PPA_BLEND_COLOR_MODE_RGB888;
				break;
			case 0:
			blend_config.out.blend_cm = PPA_BLEND_COLOR_MODE_RGB565;
				break;
		}
		//blend_config.out.blend_cm = PPA_BLEND_COLOR_MODE_RGB565;
        blend_config.bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE;//PPA_ALPHA_SCALE;
        blend_config.bg_alpha_scale_ratio = 0.3;
        blend_config.fg_alpha_update_mode = PPA_ALPHA_SCALE;
        if (alphaEN){
			blend_config.fg_alpha_scale_ratio = alphaVal;
		} else {
			blend_config.fg_alpha_scale_ratio = 0.99;
		}
		blend_config.fg_fix_rgb_val.b = tblue;
		blend_config.fg_fix_rgb_val.g = tgreen;
		blend_config.fg_fix_rgb_val.r = tred;
        blend_config.bg_ck_en = alphaEN;
        blend_config.fg_ck_en = transEN;
        blend_config.mode = PPA_TRANS_MODE_BLOCKING;
    esp_err_t ret = ppa_do_blend(ppa_blend_handle, &blend_config);
	if (ret != ESP_OK) {
        ppa_unregister_client(ppa_fill_handle);
		return false;
    }
    return true;
}

void PPAAccelerator::setTransAlpha(uint16_t tcol, bool tEN, uint8_t alphVal, bool aEN){
	alphaEN = aEN;
	transEN = tEN;
	alphaVal = 1.0 / 255.0 * (float)alphVal;
	if (alphaVal >= 1) alphaVal = 0.99;
	if (alphaVal < 0.01) alphaVal = 0.01;
	tred = (tcol & 0xf800) >> 8;
	tgreen = (tcol & 0x7e0) >> 3;
	tblue = (tcol & 0x1f) << 3;
}

bool PPAAccelerator::blockFill(int16_t fbWidth, int16_t fbHeight, int fbx, int fby, int16_t fbw, int16_t fbh, uint16_t* dstPixels, uint16_t color, int alphB, bool fmt24, bool argb){
    ppa_client_config_t ppa_fill_config = {
        .oper_type = PPA_OPERATION_FILL,
        .max_pending_trans_num = 1,
    };
	if(!ppa_fill_handle) ppa_register_client(&ppa_fill_config, &ppa_fill_handle);
	if (!ppa_fill_handle) {
        return false;
    }
	ppa_fill_oper_config_t fill_config = {};
        fill_config.out.buffer = dstPixels;
        fill_config.out.pic_w = fbWidth;
        fill_config.out.pic_h = fbHeight;
        fill_config.out.block_offset_x = fbx;
        fill_config.out.block_offset_y = fby;
		if (argb){
			fill_config.out.buffer_size = (fbWidth * fbHeight) * 4;
		    fill_config.out.fill_cm = PPA_FILL_COLOR_MODE_ARGB8888; 
		} else if (fmt24){
			fill_config.out.buffer_size = (fbWidth * fbHeight) * 3;
			fill_config.out.fill_cm = PPA_FILL_COLOR_MODE_RGB888;
		} else {
			fill_config.out.buffer_size = (fbWidth * fbHeight) << 1;
			fill_config.out.fill_cm = PPA_FILL_COLOR_MODE_RGB565;
        }
		fill_config.fill_block_w = fbw;
        fill_config.fill_block_h = fbh;
        fill_config.fill_argb_color.a = alphB;//{
		fill_config.fill_argb_color.r = (color >> 8) & 0xf8;
		fill_config.fill_argb_color.g = (color >> 3) & 0xfc;
		fill_config.fill_argb_color.b = (color << 3) & 0xf8;
        fill_config.mode = PPA_TRANS_MODE_BLOCKING;
	esp_err_t ret = ppa_do_fill(ppa_fill_handle, &fill_config);
	if (ret != ESP_OK) {
        ppa_unregister_client(ppa_fill_handle);
		return false;
    }
    return true;
}

bool PPAAccelerator::scaleRotateImageFB(uint16_t* srcPixels, int32_t fbWidth, int32_t fbHeight, int fbx, int fby, int32_t srcWidth, int32_t srcHeight,
                                     uint16_t* dstPixels, int32_t dstWidth, int32_t dstHeight, 
                                     bool bSwap, float rotation, bool mirX, bool mirY, int x, int y, int ofstw, int ofsth, bool fmt24, bool argb, int inrgb) {
	ppa_client_config_t ppa_client_config = {
        .oper_type = PPA_OPERATION_SRM,  // Scaling, Rotating, and Mirror operations
		.max_pending_trans_num = 10,
    };
	if(!ppa_scaling_handle) ppa_register_client(&ppa_client_config, &ppa_scaling_handle);
	if (!ppa_scaling_handle) {
        return false;
    }
    ppa_srm_rotation_angle_t ppa_rotation = convertRotationAngle(rotation);
    if (ppa_rotation == (ppa_srm_rotation_angle_t)-1) {
        ppa_unregister_client(ppa_scaling_handle);
		return false; // Invalid rotation angle
    } 
    ppa_srm_oper_config_t srm_oper_config = {};
    srm_oper_config.in.buffer = srcPixels;
    srm_oper_config.in.pic_w = fbWidth + ofstw;
    srm_oper_config.in.pic_h = fbHeight + ofsth;
    srm_oper_config.in.block_w = srcWidth;
    srm_oper_config.in.block_h = srcHeight;
    srm_oper_config.in.block_offset_x = fbx;
    srm_oper_config.in.block_offset_y = fby;
    switch (inrgb){
		case 0:
			srm_oper_config.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
			break;
		case 1:
			srm_oper_config.in.srm_cm = PPA_SRM_COLOR_MODE_RGB888;
			break;
		case 2:
			srm_oper_config.in.srm_cm = PPA_SRM_COLOR_MODE_ARGB8888;
			break;
	}
    srm_oper_config.out.buffer = dstPixels;
    srm_oper_config.out.pic_w = frmBuffWidth;
	srm_oper_config.out.pic_h = frmBuffHeight;
	srm_oper_config.out.block_offset_x = x;
	srm_oper_config.out.block_offset_y = y;
	if (argb){
	  srm_oper_config.out.srm_cm = PPA_SRM_COLOR_MODE_ARGB8888;
	  srm_oper_config.out.buffer_size = (frmBuffWidth * frmBuffHeight) << 2;
	} else if (fmt24){
      srm_oper_config.out.srm_cm = PPA_SRM_COLOR_MODE_RGB888;
	  srm_oper_config.out.buffer_size = (frmBuffWidth * frmBuffHeight) * 3;
	} else {
	  srm_oper_config.out.buffer_size = (frmBuffWidth * frmBuffHeight) << 1;
	  srm_oper_config.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
	}
	srm_oper_config.rotation_angle = ppa_rotation;
		
	srm_oper_config.scale_x = (float)dstWidth / srcWidth;
	srm_oper_config.scale_y = (float)dstHeight / srcHeight;

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

void PPAAccelerator::setPPAframebufferDimension(int x, int y){
	frmBuffWidth = x;
	frmBuffHeight = y;
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

