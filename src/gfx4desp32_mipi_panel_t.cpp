#include "gfx4desp32_mipi_panel_t.h"
#include "GT911.h";
GT911 ts = GT911();
gfx4desp32_mipi_panel_t::gfx4desp32_mipi_panel_t(
    int * config, int hres, int vres, int bk_pin, int bk_on_level,
    int bk_off_level, int ttype)
    : gfx4desp32_mipi_panel(config, hres, vres, bk_pin, bk_on_level, bk_off_level, ttype),
    gfx4desp32P4_touch() {
    if((ttype & 0x01) == 1){
		touchYswap = true; //touchYinvert;
	} else {
		touchYswap = false;
	}
	if(((ttype & 0x0C) >> 2) != 0){
		touchXYswap = true; //touchXYswap;
		trotate = (ttype & 0x0C) >> 2;
	} else {
		touchXYswap = false;
	}
	t_controller = (ttype >> 4) & 0x07;
	h_res = hres;
	v_res = vres;
	if((ttype & 0x80) == 0x80) alt_int = true;
}

gfx4desp32_mipi_panel_t::~gfx4desp32_mipi_panel_t() {}

/****************************************************************************/
/*!
  @brief  Enable / disable touch functions
  @param  mode - TOUCH_ENABLE / TOUCH_DISABLE
  @note   experimental
*/
/****************************************************************************/
void gfx4desp32_mipi_panel_t::touch_Set(uint8_t mode, bool aint) {
    delay(100);
    if (aint) alt_int = true;
	if (mode == TOUCH_ENABLE || mode == TOUCH_FW_WRITE) {
        pinMode(GFX4d_TOUCH_INT, INPUT_PULLUP);
		pinMode(GFX4d_TOUCH_RESET, OUTPUT);
		_TouchEnable = true;
        if (touchFirstEnable) {
            touchFirstEnable = false;
        }
		Wire.beginTransmission(0x5d);
        int error = Wire.endTransmission();
		if (error == 0) alt_int = true;
		if (t_controller == 1 || t_controller == 3 || t_controller == 4){
			if(alt_int){
			  ts.begin(5, 4, 0x5d, 400000);	
			} else {
			  ts.begin(5, 4, 0x14, 400000);
			}
			if (t_controller == 4){
				preferences.begin("touchFWSet", false);
				bool tstFW = preferences.getBool("FWset", 0);
				if (tstFW != true){
				    ts.fwWrite(GT967_800_800, 226, t_controller);
					preferences.putBool("FWset", true);
				}
				preferences.end();
			}
			if (mode == TOUCH_FW_WRITE){
				if (t_controller == 3) ts.fwWrite(GT967_720_720, 226, t_controller);
				if (t_controller == 4) ts.fwWrite(GT967_800_800, 226, t_controller);
			} else {
				ts.fwResolution(v_res, h_res, t_controller);
			}
			if (touchXYswap){ 
				ts.SetRotation(trotate);
			}
		}
    }
    else {
        _TouchEnable = false;
    }

}

int gfx4desp32_mipi_panel_t::touch_GetTouchPoints(int* tpx, int* tpy){
	int num = 0;
	if (!(_TouchEnable)) return 0;
	if (t_controller == 0){
		int bytesReceived, tps;
        Wire.beginTransmission(0x38);
        Wire.write((byte)2);
        Wire.endTransmission();
        bytesReceived = Wire.requestFrom(0x38, CTP_DATA_LEN);
        Wire.readBytes(gCTPData, bytesReceived);
        Wire.endTransmission();
        if (tps != 0xff) {
			num = gCTPData[0] ;
			if (num > 5) num = 5 ;
			for (int i = 0; i < num; i++) {
				tpy[i] = (((gCTPData[1 + i * 6] & 0x0f) << 8) + gCTPData[2 + i * 6]) - 1;
				tpx[i] = ((gCTPData[3 + i * 6] & 0x0f) << 8) + gCTPData[4 + i * 6] ;
			}
		}
	}
	if (t_controller == 1 || t_controller == 3 || t_controller == 4){
		num = ts.touched(GT911_MODE_POLLING);
		GTPoint* tpp;
		if(num > 0){
			tpp = ts.getPoints();
			for(int t = 0; t < num; t++){
				switch(rotation){
					case 0:
						tpx[t] = tpp[t].x;
						tpy[t] = tpp[t].y;			
						break;
					case 1:
						tpx[t] = __width - tpp[t].x;
						tpy[t] = __height - tpp[t].y;
						break;
					case 2:
						tpx[t] = __width - tpp[t].y;
						tpy[t] = tpp[t].x;
						break;
					case 3:
						tpx[t] = tpp[t].y;
						tpy[t] = __height - tpp[t].x;
						break;
				}			
			}
		}
	}
	return num;
}

/****************************************************************************/
/*!
  @brief  Update touch controller
  @note   if a touch event has occurred pen, xpos, ypos and images touched
          will be updated.
*/
/****************************************************************************/
bool gfx4desp32_mipi_panel_t::touch_Update() {
	bool intStatus = digitalRead(GFX4d_TOUCH_INT);
    // *** if touch is not enabled and no touch int or touch status is not no touch
    // then just return ***
    // *** need to create touch release state before no touch ***
    if (!_TouchEnable || (intStatus && tPen == 0))
        return false;
    bool update = false;
    bool validTouch;

    int i = 0;
    int tps;
    uint8_t gTPData[10];
    uint8_t bytesReceived;
    if (t_controller == 0){
    // *** process capacitive touch ***
		Wire.beginTransmission(0x38);
		Wire.write((byte)2);
		Wire.endTransmission();
		bytesReceived = Wire.requestFrom(0x38, 6);
		Wire.readBytes(gTPData, bytesReceived);
		Wire.endTransmission();
		tps = gTPData[0];
		if (tps != 0xff) { // some FT chips return garbage before first valid read,
        // try and detect that
			tPen = (tps > 0);
			num_touch_points = tps;
			if (tPen == 1) {
				switch (rotation) {
				case 0:
					lasttouchYpos = touchYpos;
					lasttouchXpos = touchXpos;
					if (touchYswap) {
						touchYpos =
							__height -
							(((gTPData[1 + i * 6] & 0x0f) << 8) + gTPData[2 + i * 6]) - 1;
					}
					else {
						touchYpos = ((gTPData[1 + i * 6] & 0x0f) << 8) + gTPData[2 + i * 6];
					}
					touchXpos = ((gTPData[3 + i * 6] & 0x0f) << 8) + gTPData[4 + i * 6];
					break;
				case 1:
					lasttouchYpos = touchYpos;
					lasttouchXpos = touchXpos;
					if (touchYswap) {
						touchYpos = ((gTPData[1 + i * 6] & 0x0f) << 8) + gTPData[2 + i * 6];
					}
					else {
						touchYpos =
							__height -
							(((gTPData[1 + i * 6] & 0x0f) << 8) + gTPData[2 + i * 6]) - 1;
					}
					touchXpos = __width -
						(((gTPData[3 + i * 6] & 0x0f) << 8) + gTPData[4 + i * 6]) -
						1;
					break;
				case 3:
					lasttouchYpos = touchYpos;
					lasttouchXpos = touchXpos;
					if (touchYswap) {
						touchXpos = __width - (((gTPData[1 + i * 6] & 0x0f) << 8) + gTPData[2 + i * 6]) - 1;
					}
					else {
						touchXpos = ((gTPData[1 + i * 6] & 0x0f) << 8) + gTPData[2 + i * 6];
					}
					touchYpos = __height -
						(((gTPData[3 + i * 6] & 0x0f) << 8) + gTPData[4 + i * 6]) -
						1;
					break;
				case 2:
					lasttouchYpos = touchYpos;
					lasttouchXpos = touchXpos;
					if (touchYswap) {
						touchXpos = ((gTPData[1 + i * 6] & 0x0f) << 8) + gTPData[2 + i * 6];
					}
					else {
						touchXpos = __width -
							(((gTPData[1 + i * 6] & 0x0f) << 8) + gTPData[2 + i * 6]) -
							1;
					}
                //touchXpos = ((gTPData[1 + i * 6] & 0x0f) << 8) + gTPData[2 + i * 6];
					touchYpos = ((gTPData[3 + i * 6] & 0x0f) << 8) + gTPData[4 + i * 6];
					break;
				}
			}
			validTouch = true;
		}
	}
	if (t_controller == 1 || t_controller == 3 || t_controller == 4){
		tps = ts.touched(GT911_MODE_POLLING);
		if (tps != 0xff) { // some FT chips return garbage before first valid read,
        // try and detect that
			tPen = (tps > 0);
			num_touch_points = tps;
			GTPoint* tp;
			if (tps) tp = ts.getPoints();
			if (tPen >= 1) {
				//int temp = tp[0].y;
				//tp[0].y = tp[0].x;
				//tp[0].x = temp;
				switch (rotation) {
				case 0:
					lasttouchYpos = touchYpos;
					lasttouchXpos = touchXpos;
					if (touchYswap) {
						touchYpos = __height - tp[0].y;
					}
					else {
						touchYpos = tp[0].y;
					}
					touchXpos = tp[0].x;
					break;
				case 1:
					lasttouchYpos = touchYpos;
					lasttouchXpos = touchXpos;
					if (touchYswap) {
						touchYpos = tp[0].y;
					}
					else {
						touchYpos =
							__height - tp[0].y;
					}
					touchXpos = __width - tp[0].x;
					break;
				case 3:
					lasttouchYpos = touchYpos;
					lasttouchXpos = touchXpos;
					if (touchYswap) {
						touchXpos = __width - tp[0].y;
					}
					else {
						touchXpos = tp[0].y;
					}
					touchYpos = __height - tp[0].x;
					break;
				case 2:
					lasttouchYpos = touchYpos;
					lasttouchXpos = touchXpos;
					if (touchYswap) {
						touchXpos = tp[0].y;
					}
					else {
						touchXpos = __width - tp[0].y;
					}
                //touchXpos = ((gTPData[1 + i * 6] & 0x0f) << 8) + gTPData[2 + i * 6];
					touchYpos = tp[0].x;
					break;
				}
			}
			validTouch = true;
		}
	}
	
    if (validTouch) {
        if (tPen == 1 && (lasttouchXpos != touchXpos || lasttouchYpos != touchYpos))
            update = true;
        if (tPen != lasttPen) {
            update = true;
            if (lasttPen == 1 && tPen == 0){
                tPen = 2; // *** create touch release state ***
			    touchDeltaX = 0;
				touchDeltaY = 0;
			}
            lasttPen = tPen;
            if (tPen == 1) {
				int xofst =  __canvasXpos - __canvasSXpos; int yofst = __canvasYpos - __canvasSYpos;
				int xofstARGB =  __canvasXposARGB - __canvasSXposARGB; int yofstARGB = __canvasYposARGB - __canvasSYposARGB;
                gciobjtouched = -1;
                if (gciobjnum > 0) {
                    if (opgfx) {
						int tx1, ty1, tx2, ty2;
                        for (int n = 0; n < gciobjnum; n++) {
                            if ((gciobjtouchenable[n] & 0x01) == 1) {
								tx1 = tuix[n];
								tx2 = tuix[n] + tuiw[n] - 1;
								ty1 = tuiy[n];
								ty2 = tuiy[n] + tuih[n] - 1;
								if (tuiExtra2[n] & WIDGET_ON_CANVAS){
									tx1 -= xofst; tx2 -= xofst; ty1 -= yofst; ty2 -= yofst;
									if (tx1 < __canvasSXpos) tx1 = __canvasSXpos;
									if (ty1 < __canvasSYpos) ty1 = __canvasSYpos;
									if (tx2 > __canvasSXpos + __canvasSWidth - 1) tx2 = __canvasSXpos + __canvasSWidth - 1;
									if (ty2 > __canvasSYpos + __canvasSHeight - 1) ty2 = __canvasSYpos + __canvasSHeight - 1;
								} else if (tuiExtra2[n] & WIDGET_ON_CANVAS_ARGB){
									tx1 -= xofstARGB; tx2 -= xofstARGB; ty1 -= yofstARGB; ty2 -= yofstARGB;
									if (tx1 < __canvasSXposARGB) tx1 = __canvasSXposARGB;
									if (ty1 < __canvasSYposARGB) ty1 = __canvasSYposARGB;
									if (tx2 > __canvasSXposARGB + __canvasSWidthARGB - 1) tx2 = __canvasSXposARGB + __canvasSWidthARGB - 1;
									if (ty2 > __canvasSYposARGB + __canvasSHeightARGB - 1) ty2 = __canvasSYposARGB + __canvasSHeightARGB - 1;
								}
                                if (touchXpos >= tx1 && touchXpos <= tx2 && touchYpos >= ty1 && touchYpos <= ty2) {
                                    gciobjtouched = n;
                                    break;
                                }
                            }
                        }
                    }
                }
				if (animIndexCounter > 0){
					int aPos;
					for (int n = 0; n < animIndexCounter; n++) {
						aPos = n << 3;						
						if ((anims[aPos + 7] & 0x1) == 1){
							if (touchXpos >= (anims[aPos + 3] & 0xffff) &&
                                touchXpos <= (anims[aPos + 5] & 0xffff) &&
                                touchYpos >= (anims[aPos + 4] & 0xffff) &&
                                touchYpos <= (anims[aPos + 6] & 0xffff)) {
                                gciobjtouched = -32767 + n;
                                break;
                            }
						}
					}
				}
				if (touchXpos >= __canvasSXpos && touchXpos <= (__canvasSXpos + __canvasSWidth - 1) && touchYpos >= __canvasSYpos && touchYpos <= (__canvasSYpos + __canvasSHeight - 1)){
					_canvasTouched = CANVAS_BUFFER_TOUCHED;
				} else if (touchXpos >= __canvasSXposARGB && touchXpos <= (__canvasSXposARGB + __canvasSWidthARGB - 1) && touchYpos >= __canvasSYposARGB && touchYpos <= (__canvasSYposARGB + __canvasSHeightARGB - 1)){
					_canvasTouched = CANVAS_BUFFER_ARGB_TOUCHED;
				} else {
					_canvasTouched = 0;
				}
            }
        }
		if (lasttPen == 1 && tPen == 1){
			if (lasttPenD != 0 && lasttPenD != 2){
				touchDeltaX = 0 - (lasttouchXpos - touchXpos);
				touchDeltaY = 0 - (lasttouchYpos - touchYpos);
			}
			lasttPen = tPen;
		} else {
			touchDeltaX = 0;
			touchDeltaY = 0;
		}
		lasttPenD = tPen;
    }
    return update;
}


