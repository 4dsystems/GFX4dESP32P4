#include "gfx4desp32P4.h"
#include "esp_attr.h"

#define gfx_Swap(a, b)                                                              \
    {                                                                           \
        int32_t tab = a;                                                        \
        a = b;                                                                  \
        b = tab;                                                                \
    }

#define calcAlpha(a, b, c)                                                     \
    {                                                                          \
        __alphatemp = c >> 3;                                                  \
        uint32_t fgu = a;                                                      \
        uint32_t bgu = b;                                                      \
        uint32_t fg = (fgu | (fgu << 16)) & 0x07e0f81f;                        \
        uint32_t bg = (bgu | (bgu << 16)) & 0x07e0f81f;                        \
        bg += (fg - bg) * __alphatemp >> 5; bg &= 0x07e0f81f;                  \
        __colour = (uint16_t)(bg | bg >> 16);                                  \
    }
	
#define boundaryTransAlpha()													\
    (bool){																		\
	    (uix >= clipx1) &&														\
        (uiy >= clipy1) && (x1 <= clipx2) && (y1 <= clipy2) &&					\
        (!transalpha)															\
	}

gfx4desp32P4::gfx4desp32P4() {}

gfx4desp32P4::~gfx4desp32P4() {}

#ifndef USE_LITTLEFS_FILE_SYSTEM
#ifndef USE_SDMMC_FILE_SYSTEM
SdFat gfx4desp32P4::uSD;

SdFat& gfx4desp32P4::getSdFatInstance() { return uSD; }
#else
#include "SD_MMC.h"   
#endif
#endif
#include "esp_async_fbcpy.h"

#define MAX_TXT_BUF_SIZE        8192
char __printf_buf[MAX_TXT_BUF_SIZE + 1];

/****************************************************************************/
/*!
  @brief  Begin gfx4desp32P4.
  @param  ips - sets if ips display is being used IPS_DISPLAY or TN_DISPLAY if not ips
                Default is TN_DISPLAY
          pval - sets the value of display clock in Mhz eg 13 = 13000000
          backLightStartOn, true or false. sets startup behaviour of backlight.
  @note   sets global variables, orientation and clears the screen.
          uSD is mounted at this stage and called seperately.
              begin(); start with default settings
              begin(30); // set display clock to 30Mhz
              begin(false); // start without backlight on
              begin(20, false); // start at 20Mhz and backlight off
*/
/****************************************************************************/
//void gfx4desp32P4::begin(int pval) { begin(pval/*, true*/); }
//void gfx4desp32P4::begin(bool backLightStartOn) { begin(0, backLightStartOn); }

/****************************************************************************/
/*!
  @brief  Begin gfx4desp32P4.
  @note   sets global variables, orientation and clears the screen.
          uSD is mounted at this stage and called seperately.
*/
/****************************************************************************/
void gfx4desp32P4::begin(int pval, bool backLightStartOn) {
    if (pval >= 13) {
        changePCLK = true;
        PCLKval = pval;
    }
    if (backLightStartOn) bkStartOn = true;
    GCItype = GCI_SYSTEM_USD;
	__begin();            // start panel
    panelOrientation(2);  // set default orientation
	width = getWidth();   // retrieve Width in pixels from panel
    height = getHeight(); // retrieve Height in pixels from panel
	if (width > height){
		__canvasWidth = width;
		__canvasHeight = width;
	} else {
		__canvasWidth = height;
		__canvasHeight = height;
	}
    JPEGinputbufferSize = (width + 16) * (height + 16) * 2;
	if (JPEGoutBufferSize != 0){
		JPEGoutputbufferSize = JPEGoutBufferSize;
	} else {	
		JPEGoutputbufferSize = (width + 16) * (height + 16) * 2;
    }
	screenArea = (width * height) << 1;
    scrollAreaY0 = 0; // set initial scroll area to maximum height - used for auto scroll if enabled
    scrollAreaY1 = height - 1;
    textXmin = 0;
    textXmax = width - 1;
    _nlh = height; // set newline cut-off point for autoscroll to maximum height
    
	Cls();         // clear the screen
	rotation = 2;  // set rotation variable to match initial orientation
    cursor_y = cursor_x = 0; // set text cursor to 0, 0
    textsize = 1;            // set text multiplier to 1
    textcolor = 0xFFFF;      // set default text colour to white
    textbgcolor = 0x0000;    // set default text background to black
    wrap = true;             // turn on wrap
    fno = 1;                 // set system font 1
    fsh = 8;                 // set font height
    lastfsh = 8; // match last font height - used by autoscroll if font changes
    // and scroll is needed
    fsw = 5;     // set font width
    scrolled = false; // legacy variable used for hardware scroll
    sEnable = false;  // disable auto scroll
    nl = false;       // set newline needed flag to false
    ssSpeed = 0; // set default smooth scroll to 5 - maybe we should default to 0,
    // smooth scroll off
    twcolnum = 13; // set default text window column number
    tchen = true;
    twcurson = true; // set default text window cursor on
	TextSize(1);
    if (bkStartOn) {
        Contrast(BK_LIGHT_STARTUP_LEVEL);
    }
	pinMode(46, INPUT);
	if(digitalRead(46) == LOW){
		uSDmount();
	}
	JPEGsoftInpBuffSize = 512*1024;
	JPEGinp = (uint8_t*)heap_caps_aligned_alloc(64, JPEGsoftInpBuffSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
	
	config = ASYNC_MEMCPY_DEFAULT_CONFIG();
	config.backlog = 64;
    config.flags = 0;
	esp_async_memcpy_install_gdma_axi(&config, &driver); // install driver with default DMA engine
}

bool gfx4desp32P4::uSDchange(){
	if(sdok == false){
		if(digitalRead(46) == LOW){
			uSDmount();
		}
	} else {
		if(digitalRead(46) == HIGH){
			sdok = false;
		}
	}
	return sdok;
}

void gfx4desp32P4::MemCopy(void* to, void* from, uint32_t len){
	esp_async_memcpy(driver, to, from, len, NULL, NULL);
}

/****************************************************************************/
/*!
  @brief  uSD mounting function used by begin
  @param  none
  @note works in conjunction with gfx4desp32P4 begin
*/
/****************************************************************************/
void gfx4desp32P4::uSDmount(){
#ifdef USE_LITTLEFS_FILE_SYSTEM // resolve File system method
#ifdef LITTLEFS_FORMAT_ON_FAIL
    if (LittleFS.begin(true))         // init flash file system
#else
    if (LittleFS.begin())         // init flash file system
#endif
    {
        sdok = true;
    }
    else {
        sdok = false;
    }
#else
#ifdef USE_SDMMC_FILE_SYSTEM
#ifndef SDMMC_4BIT
    pinMode(sd_cs, OUTPUT);
    digitalWrite(sd_cs, HIGH);
    if (SD_MMC.begin("/sdcard", true, false, 40000))
#else
	SD_MMC.setPins(43, 44, 39, 40, 41, 42);	
	if (SD_MMC.begin("/sdcard"))
	//if (SD_MMC.begin("/sdcard", false, false, SDMMC_FREQ_SDR50/*, 5*/))
#endif
    {
        sdok |= true;
    }
    else {
        sdok |= false;
    }
#endif
#endif
    delay(200);
}

/****************************************************************************/
/*!
  @brief  Store the x & y position to the text cursor if it is to be changed
  @param  none
  @note works in conjunction RestoreCursPos
*/
/****************************************************************************/
void gfx4desp32P4::StoreCursPos(){
  int pos = 6 * xystore;
  xyCurPos[pos + 0] = cursor_x;
  xyCurPos[pos + 1] = cursor_y;
  xyCurPos[pos + 2] = lastfsh;
  xyCurPos[pos + 3] = lastfsw;
  xyCurPos[pos + 4] = lastsizeht;
  xyCurPos[pos + 5] = nl;
  xystore ++;
  if (xystore > 9) xystore = 9; 
}

/****************************************************************************/
/*!
  @brief  Restore the x & y position that was previously stored
  @param  none
  @note works in conjunction StoreCursPos
*/
/****************************************************************************/
void gfx4desp32P4::RestoreCursPos(){ 
  int pos = 6 * (xystore - 1);
  if (pos < 0) pos = 0;
  MoveTo(xyCurPos[pos + 0], xyCurPos[pos + 1]);
  lastfsh = xyCurPos[pos + 2];
  lastfsw = xyCurPos[pos + 3];
  lastsizeht = xyCurPos[pos + 4];
  nl = xyCurPos[pos + 5];
  xystore --;
  if (xystore < 0) xystore = 0;	
}

/****************************************************************************/
/*!
  @brief  Write a character to the display using chosen font.
  @param  c - character to be written.
  @note works in conjunction with newLine function and Scroll if enabled
*/
/****************************************************************************/
size_t gfx4desp32P4::write(uint8_t c) {
    if (nl)
        newLine(lastfsh, lastsizeht, textXmin);
    if (c == 10) {
        nl = true;
        lastfsh = fsh;
        lastfsw = fsw;
        lastsizeht = textsizeht;
    }
    if (c == 13) {
        cursor_x = textXmin;
    }
    int tw;
    int tempHeight = fsh * textsizeht;

    if ((cursor_y + tempHeight) > getScrollareaY1() && sEnable) {
        int tempScroll = (cursor_y + tempHeight) - getScrollareaY1();
        newLine(tempScroll, 1, textXmin);
        cursor_y -= tempScroll;
    }

    if (c != 10 && c != 13) {
        if (fno == 0 || fno == -1) {
            uint16_t u16chr;

            // First we build the Utf8 character
            if (utf8expLen) {
                // If we already started building the utf-8, we continue
                // we can check this by checking expected length
                if ((c & 0xC0) != 0x80) {
                    // Invalid UTF-8 sequence, handle error or ignore
                    utf8expLen = 0;
                    utf8codepoint = 0;
                    return 0; // Indicate failure
                }
                utf8codepoint = (utf8codepoint << 6) | (c & 0x3F);
                utf8expLen--;
                if (utf8expLen != 0) return 0; // not yet complete
                u16chr = static_cast<uint16_t>(utf8codepoint);
            }
            else {
                // Otherwise, let's figure out how many bytes to expect
                if ((c & 0x80) == 0) {
                    // If the character is ASCII, directly write its Unicode value
                    u16chr = static_cast<uint16_t>(c);
                }
                else if ((c & 0xE0) == 0xC0) {
                    utf8codepoint = c & 0x1F;
                    utf8expLen = 1;
                    return 0;
                }
                else if ((c & 0xF0) == 0xE0) {
                    utf8codepoint = c & 0x0F;
                    utf8expLen = 2;
                    return 0;
                }
                else if ((c & 0xF8) == 0xF0) {
                    utf8codepoint = c & 0x07;
                    utf8expLen = 3;
                    return 0;
                }
                else {
                    // Invalid UTF-8 sequence, handle error or ignore
                    return 0; // Indicate failure
                }
            }
            tw = __gciCharWidth(u16chr) + 1;
            if (wrap && (cursor_x > (textXmax - tw))) {
                newLine(fsh, textsizeht, textXmin);
            }
            // draw character here
            if ((fno == -1) && fntCmprs) {
                drawChar4Dcmp(cursor_x, cursor_y, u16chr, textcolor, textbgcolor, textsize,
                    textsizeht);
            }
            else {
                drawChar4D(cursor_x, cursor_y, u16chr, textcolor, textbgcolor, textsize,
                    textsizeht);
            }
        }
        else if (c > 31 && c < 128) {
            tw = charWidth(c) + 1;
            if (wrap && (cursor_x > (textXmax - tw))) {
                newLine(fsh, textsizeht, textXmin);
            }
            if (fno == 1) {
                drawChar1(cursor_x, cursor_y, c - 32, textcolor, textbgcolor, textsize,
                    textsizeht);
            }
            if (fno == 2) {
                drawChar2(cursor_x, cursor_y, c - 32, textcolor, textbgcolor, textsize,
                    textsizeht);
            }

            cursor_x += textsize * (fsw + 1);
        }

        //if (wrap && (cursor_x > (width - tw))) {
        //    newLine(fsh, textsizeht, textXmax);
        //}
    }
    return 1;
}

/****************************************************************************/
/*!
  @brief  initiate a new line when drawing text.
  @param  f1 - current font height
  @param  ts - current multiplier for selected font
  @param  ux -
  @note Used for moving cursor to newLine and avoid last line of text to be
    not written to when scrolling, scroll will only occur when a new
    character occupies the last line.
*/
/****************************************************************************/
void gfx4desp32P4::newLine(int f1, int ts, int ux) {
    fsh1 = f1;
    int ScrollDist;
    nl = false;
    int remspc = ts * fsh1;
    cursor_y += remspc;
    if (sEnable && scroll_Direction == 0) {
        if (cursor_y + remspc > getScrollareaY1()) {
            ScrollDist = (cursor_y + remspc) - getScrollareaY1();
            if (ScrollDist > 0)
                if(!noscroll) Scroll(ScrollDist);
            scrolled = true;
            cursor_y = getScrollareaY1() - remspc;
        }
    }
    cursor_x = ux;
    lastfsh = remspc;
}

/****************************************************************************/
/*!
  @brief  Clear screen to black screen.
  @note May need to consider resseting more functions eg clip, scroll,
  transparency
*/
/****************************************************************************/
void gfx4desp32P4::Cls() {
	uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
	if (frame_buffer >= CANVAS_BUFFER){
		bool cargb = (frame_buffer == CANVAS_BUFFER_ARGB);
		if (cargb){
			ppaAccelerator.blockFill(altst_hresARGB, altst_vresARGB, 0, 0, altst_hresARGB, altst_vresARGB, tpto, BLACK, 0, colorFMT24, cargb);
		} else {
			ppaAccelerator.blockFill(altst_hres, altst_vres, 0, 0, altst_hres, altst_vres, tpto, BLACK, 255, colorFMT24);
		}
	} else {
		ppaAccelerator.blockFill(st_hres, st_vres, 0, 0, st_hres, st_vres, tpto, BLACK, 255, colorFMT24 & (frame_buffer == 0));
	}
	cursor_x = textXmin;
    cursor_y = 0;
    scrolled = false;
    nl = false;
    _nlh = height;
}

/****************************************************************************/
/*!
  @brief  Clear screen to custom colour
  @param  color - RGB565 colour
  @note As Cls above
*/
/****************************************************************************/
void gfx4desp32P4::Cls(uint16_t color, int alpha) {
	uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
    if (frame_buffer >= CANVAS_BUFFER){
		bool cargb = (frame_buffer == CANVAS_BUFFER_ARGB);
		if (cargb){
			ppaAccelerator.blockFill(altst_hresARGB, altst_vresARGB, 0, 0, altst_hresARGB, altst_vresARGB, tpto, BLACK, 0, colorFMT24, cargb);
		} else {
			ppaAccelerator.blockFill(altst_hres, altst_vres, 0, 0, altst_hres, altst_vres, tpto, BLACK, 255, colorFMT24);
		}
	} else {
		ppaAccelerator.blockFill(st_hres, st_vres, 0, 0, st_hres, st_vres, tpto, color, alpha, colorFMT24 & (frame_buffer == 0));
    }
	cursor_x = textXmin;
    cursor_y = 0;
    scrolled = false;
    nl = false;
    _nlh = height;
}

/****************************************************************************/
/*!
  @brief  Enable / disable auto scrolling.
  @param  bool sEn
  @note sets global sEnable flag
*/
/****************************************************************************/
void gfx4desp32P4::ScrollEnable(bool sEn) {
    _ScrollEnable(sEn);
    sEnable = sEn;
}

/****************************************************************************/
/*!
  @brief  Get X text cursor position.
  @note returns cursor_x variable
*/
/****************************************************************************/
int16_t gfx4desp32P4::getX(void) { return cursor_x; }

/****************************************************************************/
/*!
  @brief  Get Y text cursor position.
  @note returns cursor_y variable
*/
/****************************************************************************/
int16_t gfx4desp32P4::getY(void) { return cursor_y; }

/****************************************************************************/
/*!
  @brief  Get Last angle used in Angular function.
  @note   returns lastAngle variable
*/
/****************************************************************************/
int16_t gfx4desp32P4::getAngle(void) { return lastAngle; }

/****************************************************************************/
/*!
  @brief  Get Last Orbit x & y used in Angular function.
  @note   Loads lOrbit array with x & y position.
*/
/****************************************************************************/
void gfx4desp32P4::getOrbit(int * lOrbit) {
	lOrbit[0] = lastOrbit[0];
	lOrbit[1] = lastOrbit[1];
}

/****************************************************************************/
/*!
  @brief  Get Last Orbit x & y used in Angular function.
  @note   Loads lOrbit array with x & y position.
*/
/****************************************************************************/
void gfx4desp32P4::getOrbit(float * lOrbit) {
	lOrbit[0] = flastOrbit[0];
	lOrbit[1] = flastOrbit[1];
}

/****************************************************************************/
/*!
  @brief  Get Last decoded JPEG width.
  @note   returns width.
*/
/****************************************************************************/
uint16_t gfx4desp32P4::getLastJPEGwidth(void) { return _jpegWidth; }

/****************************************************************************/
/*!
  @brief  Get Last decoded JPEG height.
  @note   returns height.
*/
/****************************************************************************/
uint16_t gfx4desp32P4::getLastJPEGheight(void) { return _jpegHeight; }

/****************************************************************************/
/*!
  @brief  Get Last decoded JPEG padding value.
  @note   returns width of padding.
*/
/****************************************************************************/
uint16_t gfx4desp32P4::getLastJPEGpaddingWidth(void) { return lastJPEGpadding; }

/****************************************************************************/
/*!
  @brief  Draw system font 1 character.
  @param  x - left X position in pixels
  @param  y - top Y position in pixels
  @param  c - character to be drawn
  @param  color - foreground colour of character
  @param  bg - background colour of character
  @param  sizew - multiplier for width
  @param  sizeht - multiplier for width
  @note text size multiplier affects width and height and is not supported
    seperately. Font 1 has selectable drawing styles using FontStyle
    function.
*/
/****************************************************************************/
void gfx4desp32P4::drawChar1(int16_t x, int16_t y, unsigned char c,
    uint16_t color, uint16_t bg, uint8_t sizew,
    uint8_t sizeht) {
    bool needsEndWrite = StartWrite();
    int crad = 0;
    int co = 0;
    if (fstyle == 1 || fstyle == 2 || fstyle == 3)
        crad = sizew >> 1;
    if (fstyle == DOTMATRIXLED)
        co = crad * 68 / 100;
    if (sizew > 3)
        crad--;
    for (int8_t i = 0; i < 6; i++) {
        uint8_t tcol;
        if (i == (fsw)) {
            tcol = 0x0;
        }
        else {
            tcol = font1[(c * 5) + i];
        }
        for (int8_t j = 0; j < 8; j++) {
            if (i == 5)
                tcol = 0;
            if (tcol & 0x1) {
                if (sizew == 1 && sizeht == 1) {
                    PutPixel(x + i, y + j, color);
                }
                else {
                    if (fstyle == 0)
						if (frame_buffer == 0){    
							RectangleFilled(x + (i * sizew), y + (j * sizeht),
								(sizew + x) + (i * sizew) - 1,
								(sizeht + y) + (j * sizeht) - 1, color);
						} else {
							RectangleFilled(x + (i * sizew), y + (j * sizeht),
								(sizew + x) + (i * sizew) - 1,
								(sizeht + y) + (j * sizeht) - 1, color);
						}
					if (fstyle == 2)
                        Circle(x + (i * sizew) + crad, y + (j * sizeht) + crad, crad,
                            color);
                    if (fstyle == 1)
                        CircleFilled(x + (i * sizew) + crad, y + (j * sizeht) + crad, crad,
                            color);
                    if (fstyle == 3) {
                        CircleFilled(x + (i * sizew) + crad, y + (j * sizeht) + crad, crad,
                            color);
                        CircleFilled(x + (i * sizew) + co, y + (j * sizeht) + co, crad / 3,
                            WHITE);
                    }
                    if (fstyle == 4)
                        if (frame_buffer == 0){
							RectangleFilled(x + (i * sizew), y + (j * sizeht),
								(sizew + x) + (i * sizew) - 2,
								(sizeht + y) + (j * sizeht) - 2, color);
						} else {
					        RectangleFilled(x + (i * sizew), y + (j * sizeht),
								(sizew + x) + (i * sizew) - 2,
								(sizeht + y) + (j * sizeht) - 2, color);
						}
                    if (fstyle == 5) {
                        uint16_t fadcol = color;
                        fadcol = HighlightColors(fadcol, 10) & 0xffff;
                        int step = 60 / (sizew / 2);
                        if (step < 1)
                            step = 1;
                        for (int n = sizew / 2; n > -1; n--) {
                            Rectangle(x + (i * sizew) + n, y + (j * sizeht) + n,
                                (sizew + x) + (i * sizew) - 1 - n,
                                (sizeht + y) + (j * sizeht) - 1 - n, fadcol);
                            fadcol = HighlightColors(fadcol, step) >> 16;
                        }
                    }
                }
            }
            else if (opacitystate == false) {
                if (sizew == 1 && sizeht == 1) {
                    PutPixel(x + i, y + j, bg);
                }
                else {
                    if (frame_buffer == 0){
						RectangleFilled(x + i * sizew, y + j * sizeht,
							(sizew + x) + i * sizew - 1,
							(sizeht + y) + j * sizeht - 1, bg);
					} else {
						RectangleFilled(x + i * sizew, y + j * sizeht,
							(sizew + x) + i * sizew - 1,
							(sizeht + y) + j * sizeht - 1, bg);
					}						
                }
            }
            tcol >>= 1;
        }
    }

    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw system font 2 character.
  @param  x - left X position in pixels
  @param  y - top Y position in pixels
  @param  c - character to be drawn
  @param  color - foreground colour of character
  @param  bg - background colour of character
  @param  sizew - multiplier for width
  @param  sizeht - multiplier for width
  @note text size multiplier affects width and height and is not supported
    seperately.
*/
/****************************************************************************/
void gfx4desp32P4::drawChar2(int16_t x, int16_t y, unsigned char c,
    uint16_t color, uint16_t bg, uint8_t sizew,
    uint8_t sizeht) {
    bool needsEndWrite = StartWrite();

    for (int8_t j = 0; j < 16; j++) {
        uint8_t trow;
        trow = font2[(c * 16) + j];
        for (int8_t i = 0; i < (fsw + 1); i++) {
            if (i == (fsw)) {
                trow = 0x00;
            }
            if (trow & 0x80) {
                if (sizew == 1 && sizeht == 1) {
                    PutPixel(x + i, y + j, color);
                }
                else {
                    if (frame_buffer == 0){
						RectangleFilledPPA(x + (i * sizew), y + (j * sizeht),
							(sizew + x) + (i * sizew) - 1,
							(sizeht + y) + (j * sizeht) - 1, color);
					} else {
						RectangleFilled(x + (i * sizew), y + (j * sizeht),
							(sizew + x) + (i * sizew) - 1,
							(sizeht + y) + (j * sizeht) - 1, color);
					}
                }
            }
            else if (opacitystate == false) {
                if (sizew == 1 && sizeht == 1) {
                    PutPixel(x + i, y + j, bg);
                }
                else {
                    if (frame_buffer == 0){
						RectangleFilledPPA(x + (i * sizew), y + (j * sizeht),
							(sizew + x) + (i * sizew) - 1,
							(sizeht + y) + (j * sizeht) - 1, bg);
					} else {
						RectangleFilled(x + (i * sizew), y + (j * sizeht),
							(sizew + x) + (i * sizew) - 1,
							(sizeht + y) + (j * sizeht) - 1, bg);
					}
                }
            }
            trow <<= 1;
        }
    }
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw GCI font character.
  @param  x - left X position in pixels
  @param  y - top Y position in pixels
  @param  c - character to be drawn
  @param  color - foreground colour of character
  @param  bg - background colour of character
  @param  sizew - multiplier for width
  @param  sizeht - multiplier for width
  @note text size multiplier affects width and height and is not supported
      seperately. Font 1 has selectable drawing styles using FontStyle
      function.
*/
/****************************************************************************/
void gfx4desp32P4::drawChar4D(int16_t x, int16_t y, uint16_t c,
    uint16_t color, uint16_t bg, uint8_t sizew,
    uint8_t sizeht) {

    if (!gciFont && !fontPtr)
        return;

    if (c == '\r') {
        cursor_x = textXmin;
        return;
    }

    if (c == '\n') {
        cursor_y += fsh;
        cursor_x = textXmin; // ensures that \n works the same as \r\n
        return;
    }

    if (c >= fsc || c < fso) {
        // Character is not included
        return;
    }

    uint8_t _width = fsw;
    uint8_t _data[fsb];

    const uint8_t* data = &_data[2];

    int offset = c * fsb + 8; // character offset (number of bytes per character *
    // character value) +  8-byte header

    if (fno == 0) {
        gciFont.seek(offset);
        gciFont.read(_data, fsb);
        // width is different for each character
        _width = _data[0] << 8 | _data[1];
    }
    else {
        // width is different for each character
        _width = fontPtr[offset] << 8 | fontPtr[offset + 1];
        data = &fontPtr[offset + 2];
    }

    if (wrap && cursor_x + _width > textXmax) {
        // if next character overflows, move to next line
        cursor_y += fsh;
        cursor_x = textXmin;
    }

    int bytePerRow = _width << 1;
    uint16_t actualSize = fsh * bytePerRow;

    bool needsEndWrite = StartWrite();
    int j = -1;
    for (int i = 0; i < actualSize; i += 2) {
        int h = i % bytePerRow;
        if (h == 0) {
            j++;
        }
        h >>= 1;
        if ((data[i] << 8 | data[i + 1]) != 0) {
            if (sizew == 1 && sizeht == 1) {
                PutPixel(cursor_x + h, cursor_y + j, color);
            }
            else {
                RectangleFilled(cursor_x + h, cursor_y + j, cursor_x + h + sizew - 1,
                    cursor_y + j + sizeht - 1, color);
            }
        }
        else if (opacitystate == OPAQUE) {
            if (sizew == 1 && sizeht == 1) {
                PutPixel(cursor_x + h, cursor_y + j, bg);
            }
            else {
                RectangleFilled(cursor_x + h, cursor_y + j, cursor_x + h + sizew - 1,
                    cursor_y + j + sizeht - 1, bg);
            }
        }
    }
    if (needsEndWrite)
        EndWrite();
    cursor_x += _width * sizew;
}

/****************************************************************************/
/*!
  @brief  Draw Diablo16's FONT4 formatted font character.
  @param  x - left X position in pixels
  @param  y - top Y position in pixels
  @param  c - character to be drawn
  @param  color - foreground colour of character
  @param  bg - background colour of character
  @param  sizew - multiplier for width
  @param  sizeht - multiplier for width
  @note text size multiplier affects width and height and is not supported
      seperately. Font 1 has selectable drawing styles using FontStyle
      function.
*/
/****************************************************************************/
void gfx4desp32P4::drawChar4Dcmp(int16_t x, int16_t y, uint16_t c,
    uint16_t color, uint16_t bg, uint8_t sizew,
    uint8_t sizeht) {

    if (!fontPtr)
        return;

    if (c == '\r') {
        cursor_x = textXmin;
        return;
    }

    if (c == '\n') {
        cursor_y += fsh;
        cursor_x = textXmin; // ensures that \n works the same as \r\n
        return;
    }

    if (c >= fsc || c < fso) {
        // Character is not included
        return;
    }

    uint8_t width = fsw;
    uint8_t bytes_per_row = (fsw + 7) >> 3;
    // +7 ensures the bits will be >= next bit count in multiple of 8
    // >> 3 divides it by 8

    const uint8_t* data = fntData;

    data +=
        (c - fso) * ((bytes_per_row * fsh) + ((fontPtr[FONT_TYPE] != 0) ? 1 : 0));

    if (fontPtr[FONT_TYPE] != 0) {
        // if not simple
        // width is different for each character
        width = data[0];
        data++;
    }

    if (wrap && cursor_x + width > textXmax) {
        // if next character overflows, move to next line
        cursor_y += fsh;
        cursor_x = textXmin;
    }

    bool needsEndWrite = StartWrite();

    int16_t _x, _y;

    // loop here
    for (int i = 0; i < fsh; i++) {
        int byteIndex = 0;
        int bitIndex = 7;
        int numBits = width;

        while (numBits-- > 0) {
            _x = cursor_x + (width - numBits) * sizew - 1;
            _y = cursor_y + i * sizeht;
            if ((data[byteIndex] >> bitIndex) & 0x01) {
                if (sizew == 1 && sizeht == 1) {
                    PutPixel(_x, _y, color);
                }
                else {
                    RectangleFilled(_x, _y, _x + sizew - 1, _y + sizeht - 1, color);
                }
            }
            else {
                if (opacitystate == OPAQUE) {
                    if (sizew == 1 && sizeht == 1) {
                        PutPixel(_x, _y, bg);
                    }
                    else {
                        RectangleFilled(_x, _y, _x + sizew - 1, _y + sizeht - 1, bg);
                    }
                }
            }
            if (--bitIndex < 0) {
                byteIndex++;
                bitIndex = 7;
            }
        }

        data += bytes_per_row;
    }

    if (needsEndWrite)
        EndWrite();
    cursor_x += width * sizew;
}

/****************************************************************************/
/*!
  @brief  Move origin for drawing operations.
  @param  x - left X position in pixels
  @param  y - top Y position in pixels
  @note text position is set using this function.
*/
/****************************************************************************/
void gfx4desp32P4::MoveTo(int16_t x, int16_t y) {
    cursor_x = x;
    cursor_y = y;
    /*
	if (wrap == true) {
        if (cursor_y > (height - 1))
            cursor_y = height - 1;
        if (cursor_y < 0)
            cursor_y = 0;
        if (cursor_x < 0)
            cursor_x = 0;
        if (cursor_x > (width - 1))
            cursor_x = width - 1;
    }
	*/
    nl = false;
}

/****************************************************************************/
/*!
  @brief  Get the current system font number.
  @note System font number
*/
/****************************************************************************/
int8_t gfx4desp32P4::Font(void) { return fno; }

/****************************************************************************/
/*!
  @brief  Set system font
  @param  f - 1 or 2
  @note sets font and sets width and height variables
*/
/****************************************************************************/
void gfx4desp32P4::Font(uint8_t f) {
    if (f < 1 || f > 2)
        return;
    fno = f;
    if (fno == 1) {
        fsw = 5;
        fsh = 8;
    }
    if (fno == 2) {
        fsw = 8;
        fsh = 16;
    }
}

/****************************************************************************/
/*!
  @brief  Set Font as GCI font (IFont)
  @param  f - font opened using Open4dFont()
  @note sets font and sets width and height variables
*/
/****************************************************************************/
void gfx4desp32P4::Font(gfx4d_font f) {
    if (!f)
        return;

    fno = 0;
    gciFont = f;

    f.seek(0);
    fsw = (f.read() << 8) | f.read();
    fsh = (f.read() << 8) | f.read();
    f.seek(6);
    fsc = (f.read() << 8) | f.read();
    fsb = ((fsw * fsh) + 1) << 1;
}

/****************************************************************************/
/*!
  @brief  Set Font as IFont array
  @param  f - font opened using Open4dFont()
  @note sets font and sets width and height variables
*/
/****************************************************************************/
void gfx4desp32P4::Font(const uint8_t* f, bool compressed) {
    if (!f)
        return;

    fno = -1;
    fontPtr = f;
    fntCmprs = compressed;

    if (compressed) {

        int i = 1;

        fntCharCount = f[i++];
        if (f[FONT_TYPE] == 3) {
            fntCharCount |= f[i++] << 8;
        }
        fso = f[i++];
        fsw = f[i++];
        fsh = f[i++];

        fsc = fntCharCount + fso;

        fsb = fsh * ((fsw + 7) >> 3);
        switch (f[FONT_TYPE]) {
        case 0:
            fntData = &f[i];
            break;
        case 1:
            fntData = &f[i];
            fsb++;
            break;
        case 2:
            fntWidths = &f[i];
            fntData = fntWidths + fntCharCount;
            fsb++;
            break;
        case 3:
            fntWidths = &f[i];
            fntData = fntWidths + fntCharCount;
            fsb++;
            break;
        default:
            break;
        }

    }
    else {

        fsw = (f[0] << 8) | f[1];
        fsh = (f[2] << 8) | f[3];
        fsc = (f[6] << 8) | f[7];
        fsb = ((fsw * fsh) + 1) << 1;
    }
}

void gfx4desp32P4::__tempFont(int8_t f) {
    fnoBkup = fno;
    gciFontBkup = gciFont;
    fontPtrBkup = fontPtr;
    Font(f);
}

void gfx4desp32P4::__tempFont(const uint8_t* f, bool compressed) {
    fnoBkup = fno;
    gciFontBkup = gciFont;
    fontPtrBkup = fontPtr;
    fntCmprsBkup = fntCmprs;
    Font(f, compressed);
}

void gfx4desp32P4::__tempFont(gfx4d_font f) {
    fnoBkup = fno;
    gciFontBkup = gciFont;
    fontPtrBkup = fontPtr;
    Font(f);
}

void gfx4desp32P4::__restoreFont() {
    switch (fnoBkup) {
    case 0:
        Font(gciFontBkup);
        break;
    case -1:
        Font(fontPtrBkup, fntCmprsBkup);
        break;
    default:
        Font(fnoBkup);
        break;
    }
}

/****************************************************************************/
/*!
  @brief  system font multiplier
  @param  s - multiplier
  @note default is 1
*/
/****************************************************************************/
void gfx4desp32P4::TextSize(uint8_t s) {
    if (s > 0) {
        lastsizeht = textsizeht;
        textsize = s;
        textsizeht = s;
    }
}
/****************************************************************************/
/*!
  @brief  Set text foreground colour (compatible wrapper)
  @param  c - RGB565 colour
  @note foreground colour is set and transparency is controlled with opacity
        Opacity command.
*/
/****************************************************************************/
void gfx4desp32P4::FGcolour(uint16_t c) { textcolor = c; }

/****************************************************************************/
/*!
  @brief  Set text background colour (compatible wrapper)
  @param  c - RGB565 colour
  @note background colour is set and transparency is controlled with opacity
        Opacity command.
*/
/****************************************************************************/
void gfx4desp32P4::BGcolour(uint16_t c) { textbgcolor = c; }

/****************************************************************************/
/*!
  @brief  Set text foreground & background colour
  @param  c - RGB565 colour
  @note foreground and background colour are set the same and character is
    drawn without background and Opacity is set to transparent.
*/
/****************************************************************************/
void gfx4desp32P4::TextColor(uint16_t c) {
    textcolor = c;
    textbgcolor = c;
    opacitystate = TRANSPARENT;
}

/****************************************************************************/
/*!
  @brief  Sett text foreground & background colour with different colours
  @param  c - RGB565 foreground colour
  @param  b - RGB565 background colour
  @note foreground and background colour set seperatley. Background will be
    drawn if it is different to foreground if Opacity is set to OPAQUE.
    If Colours are the same then it is assumed TRANSPARENT
    and Opacity is set TRANSPARENT.
*/
/****************************************************************************/
void gfx4desp32P4::TextColor(uint16_t c, uint16_t b) {
    textcolor = c;
    textbgcolor = b;
    if (c == b) {
        opacitystate = TRANSPARENT;
    }
    else {
        opacitystate = OPAQUE;
    }
}

/****************************************************************************/
/*!
  @brief  Set if newLine is called when tet reaches end of screen.
  @param  w enable / disable
*/
/****************************************************************************/
void gfx4desp32P4::TextWrap(boolean w) {
	wrap = w;
	if (w == false){
		textXminBAK = textXmin;
		textXmaxBAK = textXmax;
		textXmin = 0;
		textXmax = width - 1;
	} else {
		textXmin = textXminBAK;
		textXmax = textXmaxBAK;
	}
}

/****************************************************************************/
/*!
  @brief  Set text margins to scroll window X limits.
  @param  none
*/
/****************************************************************************/
void gfx4desp32P4::TextMarginsXfromScrollWindow() {
	textXmin = scroll_window_store_x1;
	textXmax = scroll_window_store_x2;
	if(wrap){
		textXmin = scroll_window_store_x1;
	    textXmax = scroll_window_store_x2;
		textXminBAK = scroll_window_store_x1;
	    textXmaxBAK = scroll_window_store_x2;
	} else {
		textXminBAK = scroll_window_store_x1;
	    textXmaxBAK = scroll_window_store_x2;
	}
}

/****************************************************************************/
/*!
  @brief  Set text curstor X & Y to Scroll Window.
  @param  none
*/
/****************************************************************************/
void gfx4desp32P4::TextCursorXYfromScrollWindow() {
	cursor_x = scroll_window_store_x1;
	cursor_y = scroll_window_store_y1;
}

/****************************************************************************/
/*!
  @brief  Set minimum cursor position for newline.
  @param  pixels - pixels from left.
*/
/****************************************************************************/
void gfx4desp32P4::TextMarginMinX(int pixels) {
	if(wrap){
		textXmin = pixels;
		textXminBAK = pixels;
	} else {
		textXminBAK = pixels;
	} 
}

/****************************************************************************/
/*!
  @brief  Set maximum cursor position for newline with textWrap.
  @param  pixels - maximum x pixels from left.
*/
/****************************************************************************/
void gfx4desp32P4::TextMarginMaxX(int pixels) {
	if(wrap){
		textXmax = pixels;
		textXmaxBAK = pixels;
	} else {
		textXmaxBAK = pixels;
	}  
}

/****************************************************************************/
/*!
  @brief  Return height of current selected font
*/
/****************************************************************************/
int gfx4desp32P4::FontHeight(void) { return fsh; }


/****************************************************************************/
/*! gfx4desp32P4::FontHeight(void) { return fsh; }

/*********
		 gfx4desp32P4::FontHeight(void) { return fsh; }

/*********
  @brief  Set screen orientation
  @param  r - orientation - LANDSCAPE, LANDSCAPE_R, PORTRAIT, PORTRAIT_R
  @note Calls orientation function in panel and sets width & height variables
*/
/****************************************************************************/
void gfx4desp32P4::Orientation(uint8_t r) {
	bool resestWrapX = false;
	if (textXmax == width -1 && textXmin == 0) resestWrapX = true;
	panelOrientation(r);	
    width = getWidth();
    height = getHeight();
	if (resestWrapX){
		textXmax = width - 1;
		textXmin = 0;
	}
    _nlh = height;
	if (frame_buffer < CANVAS_BUFFER){
		ppaAccelerator.setPPAframebufferDimension(st_hres, st_vres);
    } else if (frame_buffer < CANVAS_BUFFER_ARGB){
		ppaAccelerator.setPPAframebufferDimension(altst_hres, altst_vres);
	} else {
		ppaAccelerator.setPPAframebufferDimension(altst_hresARGB, altst_vresARGB);
	}
}

/****************************************************************************/
/*!
  @brief  Get current orientation
*/
/****************************************************************************/
uint8_t gfx4desp32P4::Orientation() { 
	uint8_t trot;
	switch (rotation){
		case 0:
		    trot = 2;
			break;
		case 1:
		    trot = 3;
			break;
		case 2:
			trot = 0;
			break;
		case 3:
		    trot = 1;
			break;
	}
	return trot; 
}

/****************************************************************************/
/*!
  @brief  Draw rectangle (outline)
  @param  x - left X position in pixels
  @param  y - top Y position in pixels
  @param  x1 - right x position in pixels
  @param  y1 - bottom y position in pixels
  @param  color - RGB565 colour
  @note Clipping, if set, is handled by Hline / Vline functions
*/
/****************************************************************************/
void gfx4desp32P4::Rectangle(int16_t x, int16_t y, int16_t x1, int16_t y1, uint16_t color) {
    bool needsEndWrite = StartWrite();
    if (x > x1)
        gfx_Swap(x, x1);
    if (y > y1)
        gfx_Swap(y, y1);
    int w = x1 - x + 1;
    int h = y1 - y + 1;
    Hline(x, y, w, color);
    Hline(x, y + h - 1, w, color);
    Vline(x, y, h, color);
    Vline(x + w - 1, y, h, color);
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!trotf  Draw rectangle with a line thickness (outline)
  @param  x - left X position in pixels
  @param  y - top Y position in pixels
  @param  x1 - right x position in pixels
  @param  y1 - bottom y position in pixels
  @param  thk - thickness of line in pixels
  @param  color - RGB565 colour
  @note Clipping, if set, is handled by Hline / Vline functions
*/
/****************************************************************************/
void gfx4desp32P4::Rectangle(int16_t x, int16_t y, int16_t x1, int16_t y1,
    int16_t thk, uint16_t color) {
    bool needsEndWrite = StartWrite();
    if (x > x1)
        gfx_Swap(x, x1);
    if (y > y1)
        gfx_Swap(y, y1);
    int w = x1 - x + 1;
    int h = y1 - y + 1;
    RectangleFilled(x, y, x1 - thk, y + thk, color);
	RectangleFilled(x + thk, y1 - thk, x1, y1, color);
	RectangleFilled(x, y + thk, x + thk, y1, color);
	RectangleFilled(x1 - thk, y, x1, y1 - thk, color);
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw anti-aliaised circle with a line thickness (outline)
  @param  x - circle centre X position in pixels
  @param  y - circle centre Y position in pixels
  @param  r - radius of circle in pixels
  @param  thk - line thickness in pixels
  @param  color - RGB565 colour
  @note Clipping, if set, is handled by Hline / Vline functions
*/
/****************************************************************************/
void gfx4desp32P4::CircleAA(int32_t x, int32_t y, int32_t r, int thk, int32_t fg_color){
  bool needsEndWrite = StartWrite();
  int32_t rt = r - thk;
  if (rt < 0) rt = 0;
  drawArc(x, y, r, rt, 0, 360, fg_color);//, 0, true);
  if(needsEndWrite) EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw anti-aliaised arc
  @param  x - arc centre X position in pixels
  @param  y - arc centre Y position in pixels
  @param  r1 - external radius of arc in pixels
  @param  r2 - internal radius of arc in pixels
  @param  sA - start angle of arc degrees. must be 0 to 359 and less than r2
  @param  eA - start angle of arc degrees. must be 0 to 359 and greater than r1
  @param  color - RGB565 colour of source
  @param  rounded - true if rounded ends or false for straight
  @note Clipping, if set, is handled by Hline / Vline functions
*/
/****************************************************************************/
void gfx4desp32P4::ArcAA(int32_t x, int32_t y, int32_t r1, int32_t r2, int32_t sA, int32_t eA, int32_t color, bool rounded)
{
  StoreCursPos();
  if (eA != sA && (sA != 0 || eA != 360))
  {
    float stxy1[2], stxy2[2];
    float enxy1[2], enxy2[2];
    float str[2], enr[2];
	float thk = abs(r2 - r1);
	MoveTo(x, y);
    if (!rounded)
    {
      Orbit(sA + 180, r2, stxy1);
      Orbit(sA + 180, r1, stxy2);
      Orbit(eA + 180, r2, enxy1);
      Orbit(eA + 180, r1, enxy2);
      LineAA(stxy1[0], stxy1[1], stxy2[0], stxy2[1], 0.3, 0.3, color);
      LineAA(enxy1[0], enxy1[1], enxy2[0], enxy2[1], 0.3, 0.3, color);
    } else {
      Orbit(sA + 180, r2 + (thk / 2.0)/*(r1 + r2)/2.0*/, str);
      Orbit(eA + 180, r2 + (thk / 2.0)/*(r1 + r2)/2.0*/, enr);
      CircleFilledAA(str[0], str[1], abs(r1 - r2) / 2, color);
      CircleFilledAA(enr[0], enr[1], abs(r1 - r2) / 2, color);
    }
  } else {
    sA = 0; eA = 360;
  }
  drawArc(x, y, r1, r2, sA, eA, color);//, 0, true);
  RestoreCursPos();
}

/****************************************************************************/
/*!
  @brief  Draw anti-aliaised arc - helper function for drawArcAA
  @param  x - arc centre X position in pixels
  @param  y - arc centre Y position in pixels
  @param  r1 - external radius of arc in pixels
  @param  r2 - internal radius of arc in pixels
  @param  sA - start angle of arc degrees. can be out of deg range.
  @param  eA - start angle of arc degrees. can be out of deg range
  @param  color - RGB565 colour of source
  @note Clipping, if set, is handled by Hline / Vline functions
*/
/****************************************************************************/
void gfx4desp32P4::drawArc(int32_t x, int32_t y, int32_t r, int32_t ir,
                       int32_t sA, int32_t eA,
                       int32_t color)					   
{
  if (sA > eA){
	  eA = (eA % 360) + 360;
  }
  if(sA < 0 && eA > 0){
	  drawArcAA(x, y, r, ir, (sA + 360) % 360, 360, color);
	  drawArcAA(x, y, r, ir, 0, eA % 360, color);
  }
  if(sA < 360 && eA < 360){
	  drawArcAA(x, y, r, ir, sA, eA, color);
	  return;
  }
  if(sA < 360 && eA > 359){
	  drawArcAA(x, y, r, ir, sA, 360, color);
	  drawArcAA(x, y, r, ir, 0, eA % 360/*- 360*/, color);
	  return;
  } 
   if(sA > 359 && eA > 359){
	   drawArcAA(x, y, r, ir, sA % 360/*- 360*/, eA % 360/*- 360*/, color);
  }  
}
      
/****************************************************************************/
/*!
  @brief  Draw anti-aliaised arc without ending round or straight
  @param  x - arc centre X position in pixels
  @param  y - arc centre Y position in pixels
  @param  r1 - external radius of arc in pixels
  @param  r2 - internal radius of arc in pixels
  @param  sA - start angle of arc degrees. must be 0 to 359 and less than r2
  @param  eA - start angle of arc degrees. must be 0 to 359 and greater than r1
  @param  color - RGB565 colour of source
  @note Clipping, if set, is handled by Hline / Vline functions
*/
/****************************************************************************/
void gfx4desp32P4::drawArcAA(int32_t x, int32_t y, int32_t r, int32_t ir,
                       int32_t sA, int32_t eA,
                       int32_t color)//, uint32_t bg_color,
                       //bool smooth)
{
  if (sA == eA) return;
  if (r < ir) gfx_Swap(r, ir);  // Required that r > ir
  if (r <= 0 || ir < 0) return;  // Invalid r, ir can be zero (circle sector)
  bool needsEndWrite = StartWrite();
  int32_t x0, x1, y0, y1;
  float al;
  uint16_t fgcol = color;
  int32_t xs = 0;        // x start position for quadrant scan
  uint8_t alpha = 0;     // alpha value for blending pixels
  uint32_t r2 = r * r;   // Outer arc radius^2
  r++;       // Outer AA zone radius
  uint32_t r1 = r * r;   // Outer AA radius^2
  int16_t w = r - ir;   // Width of arc (r - ir + 1)
  uint32_t r3 = ir * ir; // Inner arc radius^2
  ir--;      // Inner AA zone radius
  uint32_t r4 = ir * ir; // Inner AA radius^2
  uint32_t ss[4] = {0, 0, 0xFFFFFFFF, 0};
  uint32_t es[4] = {0, 0xFFFFFFFF, 0, 0};
  float mD = 1.0/32768;
  float fabscos = fabsf(cosf(sA * deg2rad));
  float fabssin = fabsf(sinf(sA * deg2rad));
  uint32_t slope = (fabscos/(fabssin + mD)) * (float)(1<<16);
  if (sA <= 90) {
    ss[0] = slope;
  }
  else if (sA <= 180) {
    ss[1] = slope;
  }
  else if (sA <= 270) {
    ss[1] = 0xFFFFFFFF;
    ss[2] = slope;
  }
  else {
    ss[1] = 0xFFFFFFFF;
    ss[2] = 0;
    ss[3] = slope;
  }
  fabscos = fabsf(cosf(eA * deg2rad));
  fabssin = fabsf(sinf(eA * deg2rad));
  slope = (uint32_t)((fabscos/(fabssin + mD)) * (float)(1<<16));
  if (eA <= 90) {
    es[0] = slope; es[1] = 0; ss[2] = 0;
  }
  else if (eA <= 180) {
    es[1] = slope; ss[2] = 0;
  }
  else if (eA <= 270) {
    es[2] = slope;
  }
  else {
    es[3] = slope;
  }
  for (int32_t cy = r - 1; cy > 0; cy--)
  {
    uint32_t len[4] = { 0,  0,  0,  0}; // Pixel run length
    int32_t  xst[4] = {-1, -1, -1, -1}; // Pixel run x start
    uint32_t dy2 = (r - cy) * (r - cy);
    while ((r - xs) * (r - xs) + dy2 >= r1) xs++;
    for (int32_t cx = xs; cx < r; cx++)
    {
      uint32_t hyp = (r - cx) * (r - cx) + dy2;
      if (hyp > r2) {
        al = sqrt(hyp);
        alpha = ~((int)((al - (int)al) * 255));
      }
      else if (hyp >= r3) {
        slope = ((r - cy) << 16)/(r - cx);
        if (slope <= ss[0] && slope >= es[0]) { // slope hi -> lo
          xst[0] = cx; len[0]++;
        }
        if (slope >= ss[1] && slope <= es[1]) { // slope lo -> hi
          xst[1] = cx; len[1]++;
        }
        if (slope <= ss[2] && slope >= es[2]) { // slope hi -> lo
          xst[2] = cx; len[2]++;
        }
        if (slope <= es[3] && slope >= ss[3]) { // slope lo -> hi
          xst[3] = cx; len[3]++;
        }
        continue; // Next x
      } else {
        if (hyp <= r4) break;  // Skip inner pixels
        al = sqrt(hyp);
        alpha = (int)((al - (int)al) * 255);
      }
      //if (alpha < 16) continue;  // Skip low alpha pixels
      slope = ((r - cy)<<16)/(r - cx);
      x0 = x + cx - r; y0 = y - cy + r; x1 = x - cx + r; y1 = y + cy - r;
      if (slope <= ss[0] && slope >= es[0]) PutPixelAlpha((int)x0, (int)y0, color, (uint8_t)alpha); //{ // BL
      if (slope >= ss[1] && slope <= es[1]) PutPixelAlpha((int)x0, (int)y1, color, (uint8_t)alpha); //{ // TL
      if (slope <= ss[2] && slope >= es[2]) PutPixelAlpha((int)x1, (int)y1, color, (uint8_t)alpha); //{ // TR
      if (slope <= es[3] && slope >= ss[3]) PutPixelAlpha((int)x1, (int)y0, color, (uint8_t)alpha); //{ // BR
      }
      if (len[0]) HlineX(x + xst[0] - len[0] + 1 - r, (int)y0, len[0], color); // BL
      if (len[1]) HlineX(x + xst[1] - len[1] + 1 - r, (int)y1, len[1], color); // TL
      if (len[2]) HlineX(x - xst[2] + r, (int)y1, len[2], color); // TR
      if (len[3]) HlineX(x - xst[3] + r, (int)y0, len[3], color); // BR
    }
  //if(!gfx.gradON){
      if (sA == 0 || eA == 360) VlineX(x, y + r - w, w, color); // Bottom
      if (sA <= 90 && eA >= 90) HlineX(x - r + 1, y, w, color); // Left
      if (sA <= 180 && eA >= 180) VlineX(x, y - r + 1, w, color); // Top
      if (sA <= 270 && eA >= 270) HlineX(x + r - w, y, w, color); // Right
  //}
  if(needsEndWrite) EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw circle (outline).
  @param  xc - left X position in pixels
  @param  yc - top Y position in pixels
  @param  r - radius of circle
  @param  color - RGB565 colour
  @note clipping is handled by the PutPixel function.
*/
/****************************************************************************/
void gfx4desp32P4::Circle(int16_t xc, int16_t yc, int16_t r, uint16_t color) {
    bool needsEndWrite = StartWrite();
    int16_t c = 1 - r;
    int16_t xx = 1;
    int16_t yy = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    PutPixel(xc, yc + r, color);
    PutPixel(xc, yc - r, color);
    PutPixel(xc + r, yc, color);
    PutPixel(xc - r, yc, color);
    while (x < y) {
        if (c >= 0) {
            y--;
            yy = yy + 2;
            c = c + yy;
        }
        x++;
        xx = xx + 2;
        c = c + xx;
        PutPixel(xc + x, yc + y, color);
        PutPixel(xc - x, yc + y, color);
        PutPixel(xc + x, yc - y, color);
        PutPixel(xc - x, yc - y, color);
        PutPixel(xc + y, yc + x, color);
        PutPixel(xc - y, yc + x, color);
        PutPixel(xc + y, yc - x, color);
        PutPixel(xc - y, yc - x, color);
    }
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw circle (filled).
  @param  xc - left X position in pixels
  @param  yc - top Y position in pixels
  @param  r - daius of circle
  @param  color - RGB565 colour
  @note clipping is handled by PutPixel and Vline functions.
*/
/****************************************************************************/
void gfx4desp32P4::CircleFilled(int16_t xc, int16_t yc, int16_t r,
    uint32_t color) {
    bool needsEndWrite = StartWrite();
    Vline(xc, yc - r, 2 * r + 1, color);
    ArcFilled(xc, yc, r, 3, 0, color);
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw Ellipse (outline).
  @param  xe - left X position in pixels
  @param  ye - top Y position in pixels
  @param  radx - radius in a horizontal direction
  @param  rady - radius in a vertical direction
  @param  color - RGB565 colour
  @note clipping is handled by PutPixel function
*/
/****************************************************************************/
void gfx4desp32P4::Ellipse(int16_t xe, int16_t ye, int16_t radx, int16_t rady,
    uint16_t color) {
    if (radx < 2)
        return;
    if (rady < 2)
        return;
    bool needsEndWrite = StartWrite();
    int16_t x, y;
    int32_t es;
    int32_t radxx = radx * radx;
    int32_t radyy = rady * rady;
    int32_t xr = 4 * radxx;
    int32_t yr = 4 * radyy;
    for (x = 0, y = rady, es = 2 * radyy + radxx * (1 - 2 * rady);
        radyy * x <= radxx * y; x++) {
        PutPixel(xe + x, ye + y, color);
        PutPixel(xe - x, ye + y, color);
        PutPixel(xe - x, ye - y, color);
        PutPixel(xe + x, ye - y, color);
        if (es >= 0) {
            es += xr * (1 - y);
            y--;
        }
        es += radyy * ((4 * x) + 6);
    }
    for (x = radx, y = 0, es = 2 * radxx + radyy * (1 - 2 * radx);
        radxx * y <= radyy * x; y++) {
        PutPixel(xe + x, ye + y, color);
        PutPixel(xe - x, ye + y, color);
        PutPixel(xe - x, ye - y, color);
        PutPixel(xe + x, ye - y, color);
        if (es >= 0) {
            es += yr * (1 - x);
            x--;
        }
        es += radxx * ((4 * y) + 6);
    }
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw Ellipse (filled).
  @param  xe - left X position in pixels
  @param  ye - top Y position in pixels
  @param  radx - radius in a horizontal direction
  @param  rady - radius in a vertical direction
  @param  color - RGB565 colour
  @note clipping is handled by Hline function
*/
/****************************************************************************/
void gfx4desp32P4::EllipseFilled(int16_t xe, int16_t ye, int16_t radx,
    int16_t rady, uint16_t color) {
    if (radx < 2)
        return;
    if (rady < 2)
        return;
    bool needsEndWrite = StartWrite();
    int16_t x, y;
    int32_t es;
    int32_t radxx = radx * radx;
    int32_t radyy = rady * rady;
    int32_t xr = 4 * radxx;
    int32_t yr = 4 * radyy;
    for (x = 0, y = rady, es = 2 * radyy + radxx * (1 - 2 * rady);
        radyy * x <= radxx * y; x++) {
        Hline(xe - x, ye - y, 1 + x + x, color);
        Hline(xe - x, ye + y, 1 + x + x, color);
        if (es >= 0) {
            es += xr * (1 - y);
            y--;
        }
        es += radyy * ((4 * x) + 6);
    }
    for (x = radx, y = 0, es = 2 * radxx + radyy * (1 - 2 * radx);
        radxx * y <= radyy * x; y++) {
        Hline(xe - x, ye - y, 1 + x + x, color);
        Hline(xe - x, ye + y, 1 + x + x, color);
        if (es >= 0) {
            es += yr * (1 - x);
            x--;
        }
        es += radxx * ((4 * y) + 6);
    }
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw Arc (filled to center).
  @param  xa - left X position in pixels
  @param  ya - top Y position in pixels
  @param  r - radius
  @param  sa - start angle
  @param  ea - end angle
  @param  color - RGB565 colour
  @note clipping is handled by Vline function
*/
/****************************************************************************/
void gfx4desp32P4::ArcFilled(int16_t xa, int16_t ya, int16_t r, int16_t sa,
    int16_t ea, uint32_t color) {
    int16_t c = 1 - r;
    int16_t x = 0;
    int16_t y = r;
    int16_t xx = 1;
    int16_t yy = -2 * r;
    while (x < y) {
        if (c >= 0) {
            y--;
            yy = yy + 2;
            c = c + yy;
        }
        x++;
        xx = xx + 2;
        c = c + xx;
        if (sa & 0x1) {
            Vline(xa + x, ya - y, 2 * y + 1 + ea, color);
            Vline(xa + y, ya - x, 2 * x + 1 + ea, color);
        }
        if (sa & 0x2) {
            Vline(xa - x, ya - y, 2 * y + 1 + ea, color);
            Vline(xa - y, ya - x, 2 * x + 1 + ea, color);
        }
    }
}

/****************************************************************************/
/*!
  @brief  Draw Arc (outline).
  @param  x0 - left X position in pixels
  @param  y0 - top Y position in pixels
  @param  r - radius
  @param  sa - start angle
  @param  color - RGB565 colour
  @note clipping is handled by PutPixel function
*/
/****************************************************************************/
void gfx4desp32P4::Arc(int16_t x0, int16_t y0, int16_t r, uint16_t sa,
    uint16_t color) {
    bool needsEndWrite = StartWrite();
	int16_t c = 1 - r;
    int16_t xx = 1;
    int16_t yy = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    while (x < y) {
        if (c >= 0) {
            y--;
            yy = yy + 2;
            c = c + yy;
        }
        x++;
        xx = xx + 2;
        c = c + xx;
        if (sa & 0x4) {
            PutPixel(x0 + x, y0 + y, color);
            PutPixel(x0 + y, y0 + x, color);
        }
        if (sa & 0x2) {
            PutPixel(x0 + x, y0 - y, color);
            PutPixel(x0 + y, y0 - x, color);
        }
        if (sa & 0x8) {
            PutPixel(x0 - y, y0 + x, color);
            PutPixel(x0 - x, y0 + y, color);
        }
        if (sa & 0x1) {
            PutPixel(x0 - y, y0 - x, color);
            PutPixel(x0 - x, y0 - y, color);
        }
    }
	if (needsEndWrite) EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw Rounded rectangle (filled).
  @param  x - left X position in pixels
  @param  y - top Y position in pixels
  @param  x1 - right X position in pixels
  @param  y1 - bottom Y position in pixels
  @param  r - arc radius
  @param  color - RGB565 colour
  @note clipping is handled by filled rectangle function and filled arc function
*/
/****************************************************************************/
void gfx4desp32P4::RoundRectFilled(int16_t x, int16_t y, int16_t x1, int16_t y1,
    int16_t r, uint16_t color) {
    bool needsEndWrite = StartWrite();
    if (x > x1) {
        gfx_Swap(x, x1);
    }
    if (y > y1) {
        gfx_Swap(y, y1);
    }
    int w = x1 - x + 1;
    int h = y1 - y + 1;
    int maxR = 0;
    if (w >= h)
        maxR = (h - 1) / 2;
    else if (h > w)
        maxR = (w - 1) / 2;
    if (r > maxR)
        r = maxR;
    RectangleFilled(x + r, y, x + (w - r) - 1, y + h - 1, color);
    ArcFilled(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
    ArcFilled(x + r, y + r, r, 2, h - 2 * r - 1, color);
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw Rounded rectangle (outline).
  @param  x - left X position in pixels
  @param  y - top Y position in pixels
  @param  x1 - right X position in pixels
  @param  y1 - bottom Y position in pixels
  @param  r - arc radius
  @param  color - RGB565 colour
  @note clipping is handled by Hline, Vline functions and arc function
*/
/****************************************************************************/
void gfx4desp32P4::RoundRect(int16_t x, int16_t y, int16_t x1, int16_t y1,
    int16_t r, uint16_t color) {
    bool needsEndWrite = StartWrite();
    int w = x1 - x + 1;
    int h = y1 - y + 1;
    Hline(x + r, y, w - 2 * r, color);
    Hline(x + r, y + h - 1, w - 2 * r, color);
    Vline(x, y + r, h - 2 * r, color);
    Vline(x + w - 1, y + r, h - 2 * r, color);
    Arc(x + r, y + r, r, 1, color);
    Arc(x + w - r - 1, y + r, r, 2, color);
    Arc(x + w - r - 1, y + h - r - 1, r, 4, color);
    Arc(x + r, y + h - r - 1, r, 8, color);
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw line between 2 points
  @param  x - left X position in pixels
  @param  y - top Y position in pixels
  @param  x1 - right X position in pixels
  @param  y1 - bottom Y position in pixels
  @param  color - RGB565 colour
  @note clipping is handled by PutPixel function
*/
/****************************************************************************/
void gfx4desp32P4::Line(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
    uint16_t color) {
    bool needsEndWrite = StartWrite();
    int16_t angH = abs(y1 - y0) > abs(x1 - x0);
    if (angH) {
        gfx_Swap(x0, y0);
        gfx_Swap(x1, y1);
    }
    if (x0 > x1) {
        gfx_Swap(x0, x1);
        gfx_Swap(y0, y1);
    }
    int16_t xx;
    int16_t yy;
    xx = x1 - x0;
    yy = abs(y1 - y0);
    int16_t edx = xx / 2;
    int16_t incy;
    if (y0 < y1) {
        incy = 1;
    }
    else {
        incy = -1;
    }
    for (; x0 <= x1; x0++) {
        if (angH) {
            PutPixel(y0, x0, color);
        }
        else {
            PutPixel(x0, y0, color);
        }
        edx = edx - yy;
        if (edx < 0) {
            y0 = y0 + incy;
            edx = edx + xx;
        }
    }
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw triangle.
  @param  x0 - first X position in pixels
  @param  y0 - first Y position in pixels
  @param  x1 - second X position in pixels
  @param  y1 - second Y position in pixels
  @param  x2 - third X position in pixels
  @param  y2 - third Y position in pixels
  @param  color - RGB565 colour
  @note clipping is handled by Line function
*/
/****************************************************************************/
void gfx4desp32P4::Triangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
    int16_t x2, int16_t y2, uint16_t color) {
    bool needsEndWrite = StartWrite();
    Line(x0, y0, x1, y1, color);
    Line(x1, y1, x2, y2, color);
    Line(x2, y2, x0, y0, color);
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw triangle (filled).
  @param  x0 - first X position in pixels
  @param  y0 - first Y position in pixels
  @param  x1 - second X position in pixels
  @param  y1 - second Y position in pixels
  @param  x2 - third X position in pixels
  @param  y2 - third Y position in pixels
  @param  color - RGB565 colour
  @note clipping is handled by Hline function
*/
/****************************************************************************/
void gfx4desp32P4::TriangleFilled(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
    int16_t x2, int16_t y2, uint32_t color) {
    int16_t p0, p1, y, last;
    if (y0 > y1) {
        gfx_Swap(y0, y1);
        gfx_Swap(x0, x1);
    }
    if (y1 > y2) {
        gfx_Swap(y2, y1);
        gfx_Swap(x2, x1);
    }
    if (y0 > y1) {
        gfx_Swap(y0, y1);
        gfx_Swap(x0, x1);
    }
    if (y0 == y2) {
        p0 = p1 = x0;
        if (x1 < p0)
            p0 = x1;
        else if (x1 > p1)
            p1 = x1;
        if (x2 < p0)
            p0 = x2;
        else if (x2 > p1)
            p1 = x2;
        Hline(p0, y0, p1 - p0 + 1, color);
        return;
    }
    bool needsEndWrite = StartWrite();
    int16_t xx01 = x1 - x0, yy01 = y1 - y0, xx02 = x2 - x0, yy02 = y2 - y0;
    int16_t xx12 = x2 - x1, yy12 = y2 - y1;
    int32_t z1 = 0, z2 = 0;
    if (y1 == y2) {
        last = y1;
    }
    else {
        last = y1 - 1;
    }
    for (y = y0; y <= last; y++) {
        p0 = x0 + z1 / yy01;
        p1 = x0 + z2 / yy02;
        z1 += xx01;
        z2 += xx02;
        if (p0 > p1) {
            gfx_Swap(p0, p1);
        }
        Hline(p0, y, p1 - p0 + 1, color);
    }
    z1 = xx12 * (y - y1);
    z2 = xx02 * (y - y0);
    for (; y <= y2; y++) {
        p0 = x1 + z1 / yy12;
        p1 = x0 + z2 / yy02;
        z1 += xx12;
        z2 += xx02;
        if (p0 > p1) {
            gfx_Swap(p0, p1);
        }
        Hline(p0, y, p1 - p0 + 1, color);
    }
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Set maximum allowed JPEG widgets.
  @param  mw - total number
  @note Not necessary on ESP32
*/
/****************************************************************************/
void gfx4desp32P4::SetMaxJPEGWidgets(int mw) { MAX_JPEG_WIDGETS = mw; }
void gfx4desp32P4::SetMaxWidgets(int mw) { MAX_WIDGETS = mw; }

/****************************************************************************/
/*!
  @brief  Allocate custom PSRAM space for gci & dat laoded into PSRAM.
  @param  datS - size of dat file
  @param  gciS - size of gci file
  @note Open4DGFXtoPSRAM will automatically allocate the correct size of PSRAM
        but if the gci & dat files are expected to change in the sketch progmatically
        then it would be necessary to create space for the expected largest
*/
/****************************************************************************/
void gfx4desp32P4::AllocatePSRAMgciSpace(uint32_t datS, uint32_t gciS) {
    DAT_PSRAM_allocated = datS;
    GCI_PSRAM_allocated = gciS;
}

/****************************************************************************/
/*!
  @brief  Open4dGFXtoPSRAM helper function to load uSD gci & dat to PSRAM
  @param  file4d - previously selected filename
  @note using this method for displaying uSD is very fast but care needs to taken
        that gci & dat does not exceed available PSRAM. The project can still be
        edited as normal in WS4 and graphics need to be saved to uSD. a 320 x 240
        display can have as much as 6mB free PSRAM space wheras an 800 x 480 display
        would have between 4 and 5 mB free.
*/
/****************************************************************************/
void gfx4desp32P4::Open4dGFXtoPSRAM(String file4d) {
    String fnTemp = file4d;
    fnTemp.toUpperCase();
    if (fnTemp.indexOf(".GCJ") == (fnTemp.length() - 4)) {
		userImag = SD_MMC.open("/" + file4d);
		uint64_t fSize = userImag.size();
		if (fSize > 0) cache_GCI = (uint8_t*)ps_malloc(fSize);
		userImag.read(cache_GCI, fSize);
		userImag.close();
		GCIarray = cache_GCI;
		gciIsInPSRAM = true;
		gciArraySize = fSize;
		_Open4dGFXjpeg("", GCI_SYSTEM_JPEG_FLASH);
		return;
	}
    dat4d = file4d + ".dat";
    gci4d = file4d + ".gci";
    userDat = SD_MMC.open("/" + dat4d);
    cache_DAT_size = userDat.size();
    if (cache_DAT_size > 0) {
        if (DAT_PSRAM_allocated > 0) {
            cache_DAT = (uint8_t*)ps_malloc(DAT_PSRAM_allocated); // Create PSRAM cache space
        }
        else {
            cache_DAT = (uint8_t*)ps_malloc(cache_DAT_size); // Create PSRAM cache space
        }
        userDat.read(cache_DAT, cache_DAT_size);
    }
    userDat.close();
    userImag = SD_MMC.open("/" + gci4d);
    cache_GCI_size = userImag.size();
    if (cache_GCI_size > 0) {
        if (GCI_PSRAM_allocated > 0) {
            cache_GCI = (uint8_t*)ps_malloc(GCI_PSRAM_allocated); // Create PSRAM cache space
        }
        else {
            cache_GCI = (uint8_t*)ps_malloc(cache_GCI_size); // Create PSRAM cache space
        }
        userImag.read(cache_GCI, cache_GCI_size);
    }
    userImag.close();
    Open4dGFX(cache_DAT, cache_DAT_size, cache_GCI, cache_GCI_size);
}

/****************************************************************************/
/*!
  @brief  Open4dGFX helper function
  @param  file4d - previously selected filename
  @param  alloxMAX - choose between allocating memory for amound in uSD of set
  MAX
  @note to save memory, allocMAX can be set to allocate enough for counted
  objects
*/
/****************************************************************************/
void gfx4desp32P4::Open4dGFX(String file4d) {
	String fnTemp = file4d;
    fnTemp.toUpperCase();
    if (fnTemp.indexOf(".GCJ") == (fnTemp.length() - 4)) {
	    _Open4dGFXjpeg(file4d, GCI_SYSTEM_JPEG_USD);
    } else {  
        if (!gciImagesUsed) {
            gciImagesUsed = true;
            cdv = (uint8_t*)malloc(MAX_WIDGETS);
            gciobjframes = (uint16_t*)malloc(MAX_WIDGETS << 1);
            tuix = (int16_t*)malloc(MAX_WIDGETS << 1);
            tuiy = (int16_t*)malloc(MAX_WIDGETS << 1);
            tuiw = (int16_t*)malloc(MAX_WIDGETS << 1);
            tuih = (int16_t*)malloc(MAX_WIDGETS << 1);
            tuiImageIndex = (uint16_t*)malloc(MAX_WIDGETS << 1);
            tuiIndex = (uint32_t*)malloc(MAX_WIDGETS << 2);
            tuiExtra1 = (uint16_t*)malloc(MAX_WIDGETS << 1);
            tuiExtra2 = (uint16_t*)malloc(MAX_WIDGETS << 1);
			tuiRM = (uint8_t*)malloc(MAX_WIDGETS);
			tuScaleXY = (uint32_t*)malloc(MAX_WIDGETS << 2);
			memset(tuiRM, 0, gciobjnum);
			memset(tuScaleXY, 0, gciobjnum << 2);
        }
        _Open4dGFX(file4d, false);
    }
}

/****************************************************************************/
/*!
  @brief  Open4dGFX helper function for JPEG gcj's
  @param  file4d - previously selected filename
  @param  jpType - from uSD or from Flash
  @note mulitple loading of GCJ's will cause a crash due to RAM allocations
*/
/****************************************************************************/
void gfx4desp32P4::_Open4dGFXjpeg(String file4d, uint8_t jpType){
	GCItype = jpType;
    uint64_t fSize;
    if (GCItype == GCI_SYSTEM_JPEG_USD){
        file4d = "/" + file4d;
        userImag = SD_MMC.open((char*)file4d.c_str());
        fSize = userImag.size();
    } else {
	    fSize = gciArraySize;
	    GCIseek(0);  
    }
    if (fSize > 0){
        uint8_t gcjdat[16];
	    GCIread(gcjdat, 12);
		uint32_t framesPos = 0x200;
		bool MJPnotSet = false;
        gciobjnum = (gcjdat[8] << 24) + (gcjdat[9] << 16) + (gcjdat[10] << 8) + gcjdat[11]; 
        if (MAX_JPEG_WIDGETS == 0){
			MAX_JPEG_WIDGETS = gciobjnum;
			MJPnotSet = true;
		}
        if (gciobjnum > 0){
             if (!WidgetAlloc){
				cdv = (uint8_t*)malloc(MAX_JPEG_WIDGETS);
				tuiIndex = (uint32_t*)malloc(MAX_JPEG_WIDGETS << 2);
				tuix = (int16_t*)malloc(MAX_JPEG_WIDGETS << 1);
				tuiy = (int16_t*)malloc(MAX_JPEG_WIDGETS << 1);
				tuiw = (int16_t*)malloc(MAX_JPEG_WIDGETS << 1);
				tuih = (int16_t*)malloc(MAX_JPEG_WIDGETS << 1);
				tuiImageIndex = (uint16_t*)malloc(MAX_JPEG_WIDGETS << 1);
				tuiExtra1 = (uint16_t*)malloc(MAX_JPEG_WIDGETS << 1);
				tuiExtra2 = (uint16_t*)malloc(MAX_JPEG_WIDGETS << 1);
				tuiRM = (uint8_t*)malloc(MAX_JPEG_WIDGETS);
				tuScaleXY = (uint32_t*)malloc(MAX_JPEG_WIDGETS << 2);
				gciobjframes = (uint16_t*)malloc(MAX_JPEG_WIDGETS << 1);
				jpegOFFSETSpos = (uint32_t*)malloc(MAX_JPEG_WIDGETS << 2);
			}
			memset(tuiRM, 0, gciobjnum);
			memset(tuScaleXY, 0, gciobjnum << 2);
			WidgetAlloc = true;
        }
        uint32_t totframes = 0;
        uint32_t framesCount = 0;
        for (int n = 0; n < gciobjnum; n++){
	        GCIread(gcjdat, 16);
            tuiIndex[n] = (gcjdat[0] << 24) + (gcjdat[1] << 16) + (gcjdat[2] << 8) + gcjdat[3]; 
            tuix[n] = (gcjdat[4] << 8) + gcjdat[5];
            tuiy[n] = (gcjdat[6] << 8) + gcjdat[7];
            tuiw[n] = (gcjdat[8] << 8) + gcjdat[9];
            tuih[n] = (gcjdat[10] << 8) + gcjdat[11];
            gciobjframes[n] = (gcjdat[12] << 8) + gcjdat[13];
            totframes += gciobjframes[n];
        }
        if (!jpegWidgetFramesAlloc){
			if(!MJPnotSet){
				jpegOFFSETS = (uint32_t*)malloc((MAX_JPEG_WIDGETS * 100) << 2);
				jpegSIZES = (uint32_t*)malloc((MAX_JPEG_WIDGETS * 100) << 2);
			} else {
				jpegOFFSETS = (uint32_t*)malloc(totframes << 2);
				jpegSIZES = (uint32_t*)malloc(totframes << 2);
			}
			jpegWidgetFramesAlloc = true;
		}
        uint32_t readAddr;
		if (GCItype == GCI_SYSTEM_JPEG_FLASH && !gciIsInPSRAM){
			framesPos = (16 * gciobjnum) + 16;
		} else {
			if (gciobjframes[0] > 1){
				framesPos = tuiIndex[0];
			} else {	
				while (framesPos < fSize){
					GCIseek(framesPos);
					GCIread(gcjdat, 4);
					readAddr = (gcjdat[0] << 24) + (gcjdat[1] << 16) + (gcjdat[2] << 8) + gcjdat[3]; 
					if (readAddr == tuiIndex[0]){
						break;
					}
					framesPos += 0x200;
				}
			}
		}
		jpegOFFSETS = (uint32_t*)malloc(totframes << 2);
        jpegSIZES = (uint32_t*)malloc(totframes << 2);
		GCIseek(framesPos); 
		uint32_t prev, next;
		GCIread(gcjdat, 4);
        if(totframes == 1){
			prev = tuiIndex[0];
		} else {
			prev = (gcjdat[0] << 24) + (gcjdat[1] << 16) + (gcjdat[2] << 8) + gcjdat[3];
		}
		int32_t af;
		if (totframes > 1){
			for (af = 0; af < totframes - 1; af++){
				GCIread(gcjdat, 4);
				next = (gcjdat[0] << 24) + (gcjdat[1] << 16) + (gcjdat[2] << 8) + gcjdat[3]; 
				jpegSIZES[af] = next - prev;
				prev = next;
			}
		} else {
			af = 0;
		}
		jpegSIZES[af] = fSize - prev;
		uint32_t jSize;
        GCIseek(framesPos);
		if (totframes == 1){
			jpegOFFSETS[framesCount] = tuiIndex[0];
			jpegOFFSETSpos[0] = 0;
		} else {
			for (int n = 0; n < gciobjnum; n++){
				jpegOFFSETSpos[n] = framesCount;
				if(gciobjframes[n] == 1){
					GCIread(gcjdat, 4);
					tuiIndex[n] = (gcjdat[0] << 24) + (gcjdat[1] << 16) + (gcjdat[2] << 8) + gcjdat[3];
					jpegOFFSETS[framesCount] = (gcjdat[0] << 24) + (gcjdat[1] << 16) + (gcjdat[2] << 8) + gcjdat[3]; 
					framesCount ++;
				} else {
					for (int i = 0; i < gciobjframes[n]; i++){
						GCIread(gcjdat, 4);
						if (i == 0) tuiIndex[n] = (gcjdat[0] << 24) + (gcjdat[1] << 16) + (gcjdat[2] << 8) + gcjdat[3];
						jpegOFFSETS[framesCount] = (gcjdat[0] << 24) + (gcjdat[1] << 16) + (gcjdat[2] << 8) + gcjdat[3]; 
						jSize = jpegOFFSETS[framesCount];
						framesCount ++;
					}
				}
			}
		}    
	    if (gciobjnum  > 0 && JPEGinit == false){ 
			decode_eng_cfg = {
                .timeout_ms = 40,
            };
            InitializeJPEG();
        }
    }
}

/****************************************************************************/
/*!
  @brief  Open4dGFX helper function
  @param  file4d - previously selected filename
  @param  scan - pre count number of objects for memory saving allocation
*/
/****************************************************************************/
void gfx4desp32P4::_Open4dGFX(String file4d, bool scan) {
    if (userImag)
        Close4dGFX();
    uint8_t strpos = 0;
    uint8_t gotchar = 0;
    uint8_t ofset = 0;
    gciobjnum = 0;
    if (file4d != "gfx4dDummy")
        GCItype = GCI_SYSTEM_USD;
    String inputString;
    dat4d = file4d + ".dat";
    gci4d = file4d + ".gci";
    if (GCItype == GCI_SYSTEM_USD) {
        dat4d = "/" + dat4d;
        gci4d = "/" + gci4d;
		userDat = SD_MMC.open(dat4d);
    }
    if (GCItype == GCI_SYSTEM_PROGMEM) {
        datArrayPos = 0;
        gciArrayPos = 0;
        gcidatArray = false;
        if (datArraySize > 0 && gciArraySize > 0) {
            gcidatArray = true;
        }
    }
    if (userDat || gcidatArray) {
        char c;
        char prevc = 0;
        if (GCItype == GCI_SYSTEM_USD) {
            while (userDat.available() > 0) {
                c = userDat.read();
                if (c != 13 && c != 10) {
                    strpos++;
                    if (c == 34) {
                        gotchar++;
                    }
                    if (gotchar == 2) {
                        ofset = strpos;
                        gotchar = 0;
                    }
                    inputString = inputString + char(c);
                }
                if (c == 13 || (c == 10 && prevc != 13)) {
                    strpos = 0;
                    String tempis = inputString;
                    uint32_t tuindex = getIndexfromString(tempis, (2 + ofset));
                    if (!(scan))
                        tuiIndex[gciobjnum] = tuindex;
                    getCoordfromString(tempis, (12 + ofset)); // dummy read required
                    if (!(scan))
                        tuix[gciobjnum] = xic;
                    if (!(scan))
                        tuiy[gciobjnum] = yic;
                    inputString = "";
                    gciobjnum++;
                }
                prevc = c;
            }
        }
        if (GCItype == GCI_SYSTEM_PROGMEM) {
            while (datArrayPos < datArraySize) {
                c = (char)DATarray[datArrayPos++];
                if (c != 13 && c != 10) {
                    strpos++;
                    if (c == 34) {
                        gotchar++;
                    }
                    if (gotchar == 2) {
                        ofset = strpos;
                        gotchar = 0;
                    }
                    inputString = inputString + char(c);
                }
                if (c == 13 || (c == 10 && prevc != 13)) {
                    strpos = 0;
                    String tempis = inputString;
                    uint32_t tuindex = getIndexfromString(tempis, (2 + ofset));
                    if (!(scan))
                        tuiIndex[gciobjnum] = tuindex;
                    getCoordfromString(tempis, (12 + ofset)); // dummy read required
                    if (!(scan))
                        tuix[gciobjnum] = xic;
                    if (!(scan))
                        tuiy[gciobjnum] = yic;
                    inputString = "";
                    gciobjnum++;
                }
                prevc = c;
            }
        }
    }
    if (GCItype == GCI_SYSTEM_USD) {
        userDat.close();
    }
    if (GCItype == GCI_SYSTEM_PROGMEM) {
        datArrayPos = 0;
    }
    if (scan)
        return;
    if (GCItype == GCI_SYSTEM_USD) {
        userImag = SD_MMC.open((char*)gci4d.c_str());
    }
    if (GCItype == GCI_SYSTEM_PROGMEM) {
        gciArrayPos = 0;
    }
    uint32_t tIndex;
    for (int n = 0; n < gciobjnum; n++) {
        tIndex = tuiIndex[n];
        GCIseek(tIndex);
        if (!(scan))
            tuiw[n] = (GCIread() << 8) + GCIread();
        if (!(scan))
            tuih[n] = (GCIread() << 8) + GCIread();
        if (!(scan))
            cdv[n] = GCIread();
        int frms = GCIread();
        if (!(scan)) gciobjframes[n] = 0;
        if (frms != 0 && !(scan)) {
            gciobjframes[n] = (GCIread() << 8) + GCIread();
        }
    }
}

gfx4d_font gfx4desp32P4::Open4dFont(String font4d) {
    return SD_MMC.open("/" + font4d);
}

/****************************************************************************/
/*!
  @brief  Get frames count for each widget
  @param  widget - widget ID
*/
/****************************************************************************/
uint16_t gfx4desp32P4::getWidgetNumFrames(int widget) {
    if (opgfx) {
        return gciobjframes[widget];
    }
    else {
        return 0;
    }
}

/****************************************************************************/
/*!
  @brief  Open4dGFX helper function
*/
/****************************************************************************/
uint32_t gfx4desp32P4::getIndexfromString(String strval, uint8_t indx) {
    String tempstrval;
    for (int n = 0; n < 4; n++) {
        tempstrval = tempstrval + strval.charAt(indx + n + 4);
    }
    for (int n = 0; n < 4; n++) {
        tempstrval = tempstrval + strval.charAt(indx + n - 1);
    }
    uint32_t tempaddr;
    tempaddr = strtol(&tempstrval[0], NULL, 16);
    return tempaddr;
}

/****************************************************************************/
/*!
  @brief  Open4dGFX helper function
*/
/****************************************************************************/
uint32_t gfx4desp32P4::getCoordfromString(String strval, uint8_t indx) {
    String tempstrval = "";
    char c;
    for (int n = 0; n < 18; n++) {
        c = strval.charAt(indx + n - 1);
        if (c != char(32) && c != char(0)) {
            tempstrval = tempstrval + char(c);
        }
        if (c == char(32)) {
            if (tempstrval.length() > 7) {
                String tbuild = "";
                for (int o = 0; o < 4; o++) {
                    tbuild = tbuild + tempstrval.charAt(o + 4);
                }
                tempstrval = tbuild;
            }
            xic = strtol(&tempstrval[0], NULL, 16);
            tempstrval = "";
        }
        if (c == char(0)) {
            if (tempstrval.length() > 7) {
                String tbuild = "";
                for (int o = 0; o < 4; o++) {
                    tbuild = tbuild + tempstrval.charAt(o + 4);
                }
                tempstrval = tbuild;
            }
            yic = strtol(&tempstrval[0], NULL, 16);
            break;
        }
    }
    uint32_t tempcoord;
    tempcoord = (xic << 16) & yic;
    return tempcoord;
}

/****************************************************************************/
/*!
  @brief  close opened gci/gcj file
*/
/****************************************************************************/
void gfx4desp32P4::Close4dGFX() {
    if (userImag) {
        if (GCItype == GCI_SYSTEM_USD || GCItype == GCI_SYSTEM_JPEG_USD)
            userImag.close();
        if (GCItype == GCI_SYSTEM_PROGMEM) {
            gcidatArray = false;
            gciArrayPos = 0;
        }
        gciobjnum = 0;
		
    }
}

/****************************************************************************/
/*!
  @brief  Draw JPEG image file from uSD to current frame buffer using PPA
  @param  fname - filename of jpeg image
  @param  x - x position of image
  @param  y - y position of image
  @param  w - width of source image
  @param  h - height of source image
  @param  altw - optional scaled width
  @param  alth - optional scaled height
  @note 
*/
/****************************************************************************/
void gfx4desp32P4::ShowJPEGimageFile(String fname, int x, int y, int w, int h){
	ShowJPEGimageFile(fname, x, y, w, h, -1, -1, false, false, 0);
}

void gfx4desp32P4::ShowJPEGimageFile(String fname, int x, int y, int w, int h, int altw, int alth){
	ShowJPEGimageFile(fname, x, y, w, h, altw, alth, false, false, 0);
}

void gfx4desp32P4::ShowJPEGimageFile(String fname, int x, int y, int w, int h, bool mirx, bool miry){
	ShowJPEGimageFile(fname, x, y, w, h, -1, -1, mirx, miry, 0);
}

void gfx4desp32P4::ShowJPEGimageFile(String fname, int x, int y, int w, int h, int deg){
	ShowJPEGimageFile(fname, x, y, w, h, -1, -1, false, false, deg);
}

void gfx4desp32P4::ShowJPEGimageFile(String fname, int x, int y, int w, int h, int altw, int alth, bool mirx, bool miry, int deg){
	File jpgFile;
	if (SD_MMC.exists("/" + fname)){
		jpgFile = SD_MMC.open("/" + fname);
	} else { 
		return;
	}
	bool swapped = false;
	int twO = DecodeJPEGfromFile(jpgFile, w, h);
	if (twO == 16) twO = 0;
	lasttwO = twO;
	if(transparency || alpha){
		uint32_t pos = 0;
		SetGRAM (x, y, x + w - 1, y + h - 1);
		while(h--){
			WrGRAMs((uint16_t*)rx_buf + pos, w);
			pos += (w + twO);
		}
	} else {
		if ((deg == 90 || deg == 270) && rotation < 2){
			gfx_Swap(w, h);
			gfx_Swap(altw, alth);
			swapped = true;
		}
		if (altw > -1 && alth > -1){
			CheckBoundaryJPEG(x, y, w, h, alth, altw);
		} else {
			CheckBoundaryJPEG(x, y, w, h);
		}
		if (swapped){
			gfx_Swap(w, h);
			gfx_Swap(altw, alth);
		}
		uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
		uint16_t* pto = (uint16_t*)rx_buf;
		if (deg == 90){
			deg = 270;
		} else if (deg == 270){
			deg = 90;
		}
		TranslateCoords(_bCoords[B_XPOS2], _bCoords[B_YPOS2], _bCoords[B_WIDTH2], _bCoords[B_HEIGHT2], deg, frame_buffer);
		if ((deg == 90 || deg == 270) && rotation < 2){
			ppaAccelerator.scaleRotateImageFB(pto, w, h, _bCoords[B_XPOS1], _bCoords[B_YPOS1], _bCoords[B_HEIGHT1], _bCoords[B_WIDTH1], tpto, _bCoords[B_HEIGHT2], _bCoords[B_WIDTH2], false, _translated[TRANS_DEG], miry, mirx, _translated[TRANS_X], _translated[TRANS_Y], twO, 0, colorFMT24 & (frame_buffer == 0), frame_buffer == CANVAS_BUFFER_ARGB);
		} else {
		    ppaAccelerator.scaleRotateImageFB(pto, w, h, _bCoords[B_XPOS1], _bCoords[B_YPOS1], _bCoords[B_WIDTH1], _bCoords[B_HEIGHT1], tpto, _bCoords[B_WIDTH2], _bCoords[B_HEIGHT2], false, _translated[TRANS_DEG], miry, mirx, _translated[TRANS_X], _translated[TRANS_Y], twO, 0, colorFMT24 & (frame_buffer == 0), frame_buffer == CANVAS_BUFFER_ARGB);
		}
	}
}

/****************************************************************************/
/*!
  @brief  Helper function. Decode single JPEG image file from uSD to rx_buf buffer
  @param  tfile - filename of jpeg image
  @param  w - width of JPEG
  @param  h - height of JPEG
  @note   returns padding size 
*/
/****************************************************************************/
int16_t gfx4desp32P4::DecodeJPEGfromFile(File tfile, int w, int h){
	if (!tfile) return -1;
	uint32_t out_size = 0;
	uint32_t jpSize = tfile.size();
	int ttwO;
	tfile.read(tx_buf, jpSize);
	if (!JPEGinit) InitializeJPEG();
	if (w == 0  && h == 0){
		jpeg_decoder_get_info(tx_buf, jpSize, &header_info);
		JPEGiSize[0] = header_info.width;
		JPEGiSize[1] = header_info.height;
		_jpegWidth = JPEGiSize[0];
		_jpegHeight = JPEGiSize[1];
	} else {
		JPEGiSize[0] = header_info.width;
		JPEGiSize[1] = header_info.height;
		_jpegWidth = JPEGiSize[0];
		_jpegHeight = JPEGiSize[1];
	}
	jpeg_decoder_process(jpgd_handle, &decode_cfg_rgb, tx_buf, jpSize, rx_buf, rx_buffer_size, &out_size);
	ttwO = 16 - (w % 16);
	lastJPEGpadding = ttwO;
	return ttwO;
}

/****************************************************************************/
/*!
  @brief   Helper function. Decode JPEG image from GCJ file to rx_buf buffer
  @param  ofst - position of image frame in GCJ file
  @param  jpSize - size of image data
  @param  w - width of JPEG
  @param  h - height of JPEG
  @note   returns padding size
*/
/****************************************************************************/
int16_t gfx4desp32P4::DecodeJPEGfromGCJ(uint32_t ofst, uint32_t jpSize, int w, int h){
	if (!userImag) return -1;
	uint32_t out_size = 0;
	int ttwO = 0;
	userImag.seek(ofst);
	if(w < 64 && w != 0){
		if (jpegConfig == false){
			if (!JPEGinit) InitializeJPEG();
			jpeg_cfg.outbuf = (uint8_t *)rx_buf;
			jpeg_cfg.outbuf_size = rx_buffer_size;
			jpeg_cfg.indata = (uint8_t *)JPEGinp;
			jpeg_cfg.out_format = JPEG_IMAGE_FORMAT_RGB565;
			jpeg_cfg.out_scale = JPEG_IMAGE_SCALE_0;
			jpeg_cfg.flags.swap_color_bytes = 0;
			jpegConfig = true;
		}
		jpeg_cfg.indata_size = jpSize;
		userImag.read(JPEGinp, jpSize);
		esp_jpeg_decode(&jpeg_cfg, &outimg);
		ttwO = outimg.width - w;
	} else {
		if (!JPEGinit) InitializeJPEG();
		userImag.read(tx_buf, jpSize);
		jpeg_decoder_process(jpgd_handle, &decode_cfg_rgb, tx_buf, jpSize, rx_buf, rx_buffer_size, &out_size);
		ttwO = 16 - (w % 16);
	}
	_jpegWidth = w;
	_jpegHeight = h;
	lastJPEGpadding = ttwO;
	return ttwO;
}

/****************************************************************************/
/*!
  @brief   Helper function. Decode JPEG image from GCJ in Flash to rx_buf buffer
  @param  ofst - position of image frame in GCJ array
  @param  jpSize - size of image data
  @param  w - width of JPEG
  @param  h - height of JPEG
  @note   returns padding size
*/
/****************************************************************************/
int16_t gfx4desp32P4::DecodeJPEGfromGCJinFlash(uint32_t ofst, uint32_t jpSize, int w, int h){
	uint32_t out_size = 0;
	int ttwO;
	if(w < 64 && w != 0){
		if (jpegConfig == false){
			if (!JPEGinit) InitializeJPEG();
			jpeg_cfg.outbuf = (uint8_t *)rx_buf;
			jpeg_cfg.outbuf_size = rx_buffer_size;
			jpeg_cfg.indata = (uint8_t *)JPEGinp;
			jpeg_cfg.out_format = JPEG_IMAGE_FORMAT_RGB565;
			jpeg_cfg.out_scale = JPEG_IMAGE_SCALE_0;
			jpeg_cfg.flags.swap_color_bytes = 0;
			jpegConfig = true;
		}
		jpeg_cfg.indata_size = jpSize;
		memcpy(JPEGinp, GCIarray + ofst, jpSize);
		esp_jpeg_decode(&jpeg_cfg, &outimg);
		ttwO = outimg.width - w;
	} else {
		memcpy(tx_buf, GCIarray + ofst, jpSize);
		jpeg_decoder_process(jpgd_handle, &decode_cfg_rgb, tx_buf, jpSize, rx_buf, rx_buffer_size, &out_size);
		ttwO = 16 - (w % 16);
	}
	_jpegWidth = w;
	_jpegHeight = h;
	lastJPEGpadding = ttwO;
	return ttwO;
}

/****************************************************************************/
/*!
  @brief   Helper function. Decode JPEG image from Array to rx_buf buffer
  @param  tArray - array containing JPEG image data
  @param  ofst - size of JPEG image data
  @param  w - width of JPEG
  @param  h - height of JPEG
  @param  forceSoft - force use of software JPEG decoder
  @note   returns padding size
*/
/****************************************************************************/
int16_t gfx4desp32P4::DecodeJPEGfromArray(uint8_t* tArray, uint32_t ofst, int w, int h, bool forceSoft){
	uint32_t jpSize = ofst;
	uint32_t out_size = 0;
	int ttwO;
	bool getDim = (w == 0 || h == 0);
	if((w < 64 && w != 0) || forceSoft){
		if (jpegConfig == false){
			if (!JPEGinit) InitializeJPEG();
			jpeg_cfg.outbuf = (uint8_t *)rx_buf;
			jpeg_cfg.outbuf_size = rx_buffer_size;
			jpeg_cfg.indata = tArray;//(uint8_t *)JPEGinp;
			jpeg_cfg.out_format = JPEG_IMAGE_FORMAT_RGB565;
			jpeg_cfg.out_scale = JPEG_IMAGE_SCALE_0;
			jpeg_cfg.flags.swap_color_bytes = 0;
			jpegConfig = true;
		}
		jpeg_cfg.indata_size = jpSize;
		jpeg_cfg.indata = tArray;//(uint8_t *)JPEGinp;
		esp_jpeg_decode(&jpeg_cfg, &outimg);
		if (getDim){
			_jpegWidth = outimg.width;
			_jpegHeight = outimg.height;
		} else {
			_jpegWidth = w;
			_jpegHeight = h;
		}
		ttwO = _jpegWidth - w;
	} else {
		if (!JPEGinit) InitializeJPEG();
		memcpy(tx_buf, tArray, jpSize);
		if (_videoGetDim || getDim){
			jpeg_decoder_get_info(tx_buf, jpSize, &header_info);
			_jpegWidth = header_info.width;
			_jpegHeight = header_info.height;
			_videoGetDim = false;
		} else {
			_jpegWidth = w;
			_jpegHeight = h;
		}
		jpeg_decoder_process(jpgd_handle, &decode_cfg_rgb, tx_buf, jpSize, rx_buf, rx_buffer_size, &out_size);
		ttwO = 16 - (w % 16);
	}
	lastJPEGpadding = ttwO;
	return ttwO;
}

int16_t gfx4desp32P4::GetJPEGinfo(int infoType, uint8_t* tArray, uint32_t jpSze){
	int ret;
	if (!JPEGinit) InitializeJPEG();
	memcpy(tx_buf, tArray, jpSze);
	jpeg_decoder_get_info(tx_buf, jpSze, &header_info);
	if (infoType == JPEG_SAMPLE_METHOD) ret = header_info.sample_method; // JPEG_DOWN_SAMPLING_YUV444, JPEG_DOWN_SAMPLING_YUV422, JPEG_DOWN_SAMPLING_YUV420
	if (infoType == JPEG_WIDTH) ret = header_info.width;
	if (infoType == JPEG_HEIGHT) ret = header_info.height;
	return ret;
}

void gfx4desp32P4::DrawJPEGarray(const uint8_t* image, uint32_t image_size, int x, int y, int w, int h, int altw, int alth){
	int twO;
	if (twO == 16) twO = 0;
	bool swapped = false;
	int deg = 0;
	if (altw == -1 || alth == -1){
		altw = w; alth = h;
	}
	twO = DecodeJPEGfromArray((uint8_t*)image, image_size, w, h, false);
	if (twO == 16) twO = 0;
	if((transparency || alpha) && frame_buffer != CANVAS_BUFFER_ARGB){
		uint32_t pos = 0;
		SetGRAM (x, y, w, h);
		while(h--){
			WrGRAMs((uint16_t*)rx_buf + pos, w);
			pos += (w + twO);
		}
	} else {
		uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
		uint16_t* pto = (uint16_t*)rx_buf;
		TranslateCoords(x, y, altw, alth, deg, frame_buffer);
		ppaAccelerator.scaleRotateImageFB(pto, w, h, 0, 0, w, h, tpto, altw, alth, false, _translated[TRANS_DEG], false, false, _translated[TRANS_X], _translated[TRANS_Y], twO, 0, colorFMT24 & (frame_buffer == 0), frame_buffer == CANVAS_BUFFER_ARGB);
	}
}

void gfx4desp32P4::DrawARGB888array(uint8_t* image, uint32_t image_size, int x, int y, int w, int h, int altw, int alth){
	if (altw == -1 || alth == -1){
		altw = w; alth = h;
	}
	int deg = 0;
	uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
	uint16_t* pto = (uint16_t*)image;
	TranslateCoords(x, y, altw, alth, deg, frame_buffer);
	ppaAccelerator.scaleRotateImageFB(pto, w, h, 0, 0, w, h, tpto, altw, alth, false, _translated[TRANS_DEG], false, false, _translated[TRANS_X], _translated[TRANS_Y], 0, 0, colorFMT24 & (frame_buffer == 0), frame_buffer == CANVAS_BUFFER_ARGB, PPA_SRM_COLOR_MODE_ARGB8888);
}

void gfx4desp32P4::DrawARGB888array(const uint8_t* image, uint32_t image_size, int x, int y, int w, int h, int altw, int alth){
	if (altw == -1 || alth == -1){
		altw = w; alth = h;
	}
	int deg = 0;
	uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
	uint16_t* pto = (uint16_t*)image;
	TranslateCoords(x, y, altw, alth, deg, frame_buffer);
	ppaAccelerator.scaleRotateImageFB(pto, w, h, 0, 0, w, h, tpto, altw, alth, false, _translated[TRANS_DEG], false, false, _translated[TRANS_X], _translated[TRANS_Y], 0, 0, colorFMT24 & (frame_buffer == 0), frame_buffer == CANVAS_BUFFER_ARGB, PPA_SRM_COLOR_MODE_ARGB8888);
}

void gfx4desp32P4::DrawJPEGarray(uint8_t* image, uint32_t image_size, int x, int y, int w, int h, int altw, int alth){
	int twO;
	if (twO == 16) twO = 0;
	bool swapped = false;
	int deg = 0;
	if (altw == -1 || alth == -1){
		altw = w; alth = h;
	}
	twO = DecodeJPEGfromArray(image, image_size, w, h, false);
	if (twO == 16) twO = 0;
	if((transparency || alpha) && frame_buffer != CANVAS_BUFFER_ARGB){
		uint32_t pos = 0;
		SetGRAM (x, y, w, h);
		while(h--){
			WrGRAMs((uint16_t*)rx_buf + pos, w);
			pos += (w + twO);
		}
	} else {
		uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
		uint16_t* pto = (uint16_t*)rx_buf;
		TranslateCoords(x, y, altw, alth, deg, frame_buffer);
		ppaAccelerator.scaleRotateImageFB(pto, w, h, 0, 0, w, h, tpto, altw, alth, false, _translated[TRANS_DEG], false, false, _translated[TRANS_X], _translated[TRANS_Y], twO, 0, colorFMT24 & (frame_buffer == 0), frame_buffer == CANVAS_BUFFER_ARGB);
	}
}

/****************************************************************************/
/*!
  @brief  Helper function for MJPEG video functions
  @note   not recommended for User code
*/
/****************************************************************************/
void gfx4desp32P4::DrawJPEGbuffer(int x, int y, int w, int h, int twO, int altw, int alth){
	if (twO == 16) twO = 0;
	bool swapped = false;
	int deg = 0;
	if(transparency || alpha){
		uint32_t pos = 0;
		SetGRAM (x, y, w, h);
		while(h--){
			WrGRAMs((uint16_t*)rx_buf + pos, w);
			pos += (w + twO);
		}
	} else {
		uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
		uint16_t* pto = (uint16_t*)rx_buf;
		TranslateCoords(x, y, altw, alth, deg, frame_buffer);
		ppaAccelerator.scaleRotateImageFB(pto, w, h, 0, 0, w, h, tpto, altw, alth, false, _translated[TRANS_DEG], false, false, _translated[TRANS_X], _translated[TRANS_Y], twO, 0, colorFMT24 & (frame_buffer == 0), frame_buffer == CANVAS_BUFFER_ARGB);
	}
}

/****************************************************************************/
/*!
  @brief  Draw widget from GCI file. UserImage / UserImages draw function
  @param  Index - position on uSD of image
  @param  uix - x position of widget
  @param  uiy - y position of widgte
  @param  uiw - width of widget
  @param  uih - height of widget
  @param  frame - number of frame in widget
  @param  bar - used in spectrum widgets
  @param  images - if single image or image set
  @param  cdv - not really used
  @note clipping and out of bounds now handled by WrGRAMs functions
*/
/****************************************************************************/
void gfx4desp32P4::DrawWidget(uint32_t Index, int16_t uix, int16_t uiy,
	int16_t uiw, int16_t uih, uint16_t frame,
    int16_t bar, bool images, byte cdv, uint16_t ui, bool decodeOnly) {
	if (ui > (gciobjnum - 1)) return;
	bool swapped = false;
	int altw = -1;
	int alth = -1;
	int deg = (tuiRM[ui] & 0x03) * 90;
	int x1 = uix + uiw - 1;
    int y1 = uiy + uih - 1;
	//bool softDEC = false;
	if (bar != 0)
        uix = uix + bar;
    if (((GCItype == GCI_SYSTEM_USD ||  GCItype == GCI_SYSTEM_JPEG_USD) && (!userImag)) ||
        (GCItype == GCI_SYSTEM_PROGMEM && (!gcidatArray)))
        return;
    if (GCItype == GCI_SYSTEM_JPEG_USD || GCItype == GCI_SYSTEM_JPEG_FLASH){
		if (JPEGinit == false) return;
		uint32_t jpegsize = jpegSIZES[jpegOFFSETSpos[ui] + frame];
		int twO;
		if (_lastObject != ui || _lastFrame != frame){
			if(GCItype == GCI_SYSTEM_JPEG_USD){
				twO = DecodeJPEGfromGCJ(jpegOFFSETS[jpegOFFSETSpos[ui] + frame], jpegsize, uiw, uih);
			} else {
				twO = DecodeJPEGfromGCJinFlash(jpegOFFSETS[jpegOFFSETSpos[ui] + frame], jpegsize, uiw, uih);
			}
		} else {
			twO = lasttwO;
		}
		if (twO == 16) twO = 0;
		lasttwO = twO;
		_lastFrame = frame; _lastObject = ui;
		if (decodeOnly) return;
		//if (transalpha){
		if(!boundaryTransAlpha() && frame_buffer < CANVAS_BUFFER){
		    uint32_t pos = 0;
			SetGRAM (uix, uiy, uix + uiw - 1, uiy + uih - 1);
			while(uih--){
				WrGRAMs((uint16_t*)rx_buf + pos, uiw);
				pos += (uiw + twO);
			}
		} else {
			if (tuScaleXY[ui] != 0){
				int xs = tuScaleXY[ui] & 0xffff;
				int ys = (tuScaleXY[ui] & 0xffff0000) >> 16;
				if (xs != 0) altw = ys;
				if (ys != 0) alth = xs;
			}
			if ((deg == 90 || deg == 270) && rotation < 2){
				gfx_Swap(uiw, uih);
				gfx_Swap(altw, alth);
				swapped = true;
			}
			if (altw > -1 && alth > -1){
				CheckBoundaryJPEG(uix, uiy, uiw, uih, alth, altw);
			} else {
				CheckBoundaryJPEG(uix, uiy, uiw, uih);
			}
			if (swapped){
				gfx_Swap(uiw, uih);
				gfx_Swap(altw, alth);
			}
			uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
			uint16_t* pto = (uint16_t*)rx_buf;
			if (deg == 90){
				deg = 270;
			} else if (deg == 270){
				deg = 90;
			}
			TranslateCoords(_bCoords[B_XPOS2], _bCoords[B_YPOS2], _bCoords[B_WIDTH2], _bCoords[B_HEIGHT2], deg, frame_buffer);
			bool opFMT = false;
			bool opFMT1 = (frame_buffer == CANVAS_BUFFER_ARGB);
			if (colorFMT24 && frame_buffer == 0) opFMT = true;
			//opFMT = false;
			//opFMT1 = true;
			if ((deg == 90 || deg == 270) && rotation < 2){
				ppaAccelerator.scaleRotateImageFB(pto, uiw/* + twO*/, uih, _bCoords[B_XPOS1], _bCoords[B_YPOS1], _bCoords[B_HEIGHT1], _bCoords[B_WIDTH1], tpto, _bCoords[B_HEIGHT2], _bCoords[B_WIDTH2], false, _translated[TRANS_DEG], (tuiRM[ui] & 0x20) >> 5, (tuiRM[ui] & 0x10) >> 4, _translated[TRANS_X], _translated[TRANS_Y], twO, 0, opFMT, opFMT1, 0);
			} else {
				ppaAccelerator.scaleRotateImageFB(pto, uiw/* + twO*/, uih, _bCoords[B_XPOS1], _bCoords[B_YPOS1], _bCoords[B_WIDTH1], _bCoords[B_HEIGHT1], tpto, _bCoords[B_WIDTH2], _bCoords[B_HEIGHT2], false, _translated[TRANS_DEG], (tuiRM[ui] & 0x20) >> 5, (tuiRM[ui] & 0x10) >> 4, _translated[TRANS_X], _translated[TRANS_Y], twO, 0, opFMT, opFMT1, 0);
			}
		}
		if (frame_buffer >= CANVAS_BUFFER){
			tuiExtra2[ui] = tuiExtra2[ui] & 0x3fff;
			if (frame_buffer == CANVAS_BUFFER) tuiExtra2[ui] = (tuiExtra2[ui] & 0x3fff) | 0x8000;
			if (frame_buffer == CANVAS_BUFFER_ARGB) tuiExtra2[ui] = (tuiExtra2[ui] & 0x3fff) | 0x4000;
		}
		return;
	}
    byte ofst = 6;
    if (images) ofst = 8;
    int mul = 2;
    uint32_t isize;
    uint32_t pos;
    if (cdv == 8) mul = 1;
    isize = (uiw * uih) << (mul - 1);
    usePushColors = boundaryTransAlpha();//(uix >= clipx1) &&
    pos = (isize * frame);    
	if (GCItype == GCI_SYSTEM_USD) {        
		GCIreadToBuff(Index + ofst + pos, isize);
        if (usePushColors) {
			SetGRAM(uix, uiy, x1, y1);
			pushColors(psRAMbuffer1, isize >> 1);
        } else {
            SetGRAM(uix, uiy, x1, y1);
			WrGRAMs(psRAMbuffer1, isize >> 1);
        }
    } else {
        if (usePushColors) {
			SetGRAM(uix, uiy, x1, y1);
			pushColors(GCIarray + Index + ofst + pos, isize >> 1);
        } else {
            SetGRAM(uix, uiy, x1, y1);
			WrGRAMs(GCIarray + Index + ofst + pos, isize >> 1);
        }
    }
	if (frame_buffer >= CANVAS_BUFFER){
		tuiExtra2[ui] = tuiExtra2[ui] & 0x3fff;
		if (frame_buffer == CANVAS_BUFFER) tuiExtra2[ui] = (tuiExtra2[ui] & 0x3fff) | 0x8000;
		if (frame_buffer == CANVAS_BUFFER_ARGB) tuiExtra2[ui] = (tuiExtra2[ui] & 0x3fff) | 0x4000;
	}
}

/****************************************************************************/
/*!
  @brief  Draw a widget made up of 2 GCI elements
  @param  ui - position on uSD of image, 2nd image is next GCI image
  @param  val - value of position of 2nd image (calculated by Workshop)
  @param  range - 0 to max range of widget (calculated by Workshop)
  @param  orientation - movement orientation (calculated by Workshop)
  @param  gap1 - distance from edge of 1st image that movement starts (calculated by Workshop)
  @param  gap2 - distance from end edge of 1st image that movement starts (calculated by Workshop)
  @note clipping and out of bounds now handled by WrGRAMs functions
*/
/****************************************************************************/
void gfx4desp32P4::UserImages2image(int16_t ui, int16_t val, int16_t range,
    bool orientation, int16_t gap1,
    int16_t gap2) {
    if ((GCItype == GCI_SYSTEM_USD && (!userImag)) ||
        (GCItype == GCI_SYSTEM_PROGMEM && (!gcidatArray)))
        return;
    if (val > range)
        val = range;
    if (val < 0)
        val = 0;

    int16_t pixelRange = 0;
    int16_t w2i = tuiw[ui];
    int16_t h2i = tuih[ui];
    int16_t changePoint = 0;
    float inc = 0;
    if (orientation == HORIZONTAL) {
        pixelRange = w2i - (gap1 + gap2);
        inc = (float)pixelRange / (float)range;
        changePoint = (int)(inc * (float)val) + gap1;
        LastLinearPointerPosition = (int16_t)changePoint;
        if (changePoint > 0)
            UserImagesDR(ui, 1, 0, 0, changePoint, h2i);
        if ((w2i - changePoint - 1) > 0)
            UserImagesDR(ui, 0, changePoint, 0, w2i - changePoint - 1, h2i);
    }
    if (orientation == VERTICAL) {
        pixelRange = h2i - (gap1 + gap2);
        val = range - val;
        inc = (float)pixelRange / (float)range;
        changePoint = (int)(inc * (float)val) + gap1;
        LastLinearPointerPosition = (int16_t)changePoint;
        if (changePoint > 0)
            UserImagesDR(ui, 0, 0, 0, w2i, changePoint);
        if ((h2i - changePoint - 1) > 0)
            UserImagesDR(ui, 1, 0, changePoint, w2i, h2i - changePoint - 1);
    }
}

/****************************************************************************/
/*!
  @brief  Draw a widget made up of 3 GCI elements
  @param  ui - position on uSD of image, 2nd and 3rd image are next GCI images
  @param  val - value of position of 2nd image (calculated by Workshop)
  @param  range - 0 to max range of widget (calculated by Workshop)
  @param  orientation - movement orientation (calculated by Workshop)
  @param  gap1 - distance from edge of 1st image that movement starts (calculated by Workshop)
  @param  gap2 - distance from end edge of 1st image that movement starts (calculated by Workshop)
  @param  tc - (calculated by Workshop)
  @note clipping and out of bounds now handled by WrGRAMs functions not to be used with JPEG gcj's
*/
/****************************************************************************/
void gfx4desp32P4::UserImages3image(int16_t ui, int16_t val, int16_t range,
    bool orientation, int16_t gap1, int16_t gap2,
    int32_t tc) {
    if ((GCItype == GCI_SYSTEM_USD && (!userImag)) ||
        (GCItype == GCI_SYSTEM_PROGMEM && (!gcidatArray)))
        return;
    if (val > range)
        val = range;
    if (val < 0)
        val = 0;
    int16_t ui2 = ui + 1;

    int pixelRange;
    uint32_t w = tuiw[ui];
    uint32_t h = tuih[ui];
    uint32_t oh = h;
    uint32_t kw = tuiw[ui2];
    uint32_t kh = tuih[ui2];
    uint32_t changePoint;
    uint32_t isize = w * h;
    uint32_t pos;
    uint32_t posk;
    uint8_t tc1 = tc >> 8;
    uint8_t tc2 = tc & 0xff;
    float inc;
    uint32_t tempos = (isize << 1);
    int x1 = tuix[ui] + w - 1;
    int y1 = tuiy[ui] + h - 1;
    usePushColors = (tuix[ui] >= clipx1) &&
        (tuiy[ui] >= clipy1) && (x1 <= clipx2) && (y1 <= clipy2) &&
        (!transalpha) && (!WriteFBonly) && (frame_buffer == visibleFB);
    setGRAM(tuix[ui], tuiy[ui], x1, y1);
    posk = tuiIndex[ui2] + 6;
    if (orientation == HORIZONTAL) {
        pixelRange = w - (gap1 + gap2);
        inc = (float)pixelRange / (float)range;
        changePoint = (int)(inc * (float)val) + gap1;
        LastLinearPointerPosition = (int16_t)changePoint;
        pos = tuiIndex[ui] + 8;
        uint32_t pc;
        uint32_t p = 0;
        pc = pos + (isize << 1);
        GCIseek(pc);
        while (h--) {
            GCIreadToBuf(p, changePoint << 1);
            GCIseek(pc - (isize << 1) + (changePoint << 1));
            GCIreadToBuf(p + (changePoint << 1), (w - changePoint) << 1);
            pc += (w << 1);
            p += (w << 1);
            GCIseek(pc);
        }
        GCIseek(posk);
        p = (((oh - kh) >> 1) * (w << 1)) + (changePoint << 1) - (kw & 0xfffe);
        while (kh--) {
            if (tc == -1) {
                GCIreadToBuf(p, kw << 1);
            }
            else {
                GCIreadToBuf(tempos, kw << 1);
                for (int n = 0; n < kw; n++) {
                    if (psRAMbuffer1[tempos + (n << 1)] != tc1 && psRAMbuffer1[tempos + (n << 1) + 1] != tc2) {
                        psRAMbuffer1[p + (n << 1)] = psRAMbuffer1[tempos + (n << 1)];
                        psRAMbuffer1[p + (n << 1) + 1] = psRAMbuffer1[tempos + (n << 1) + 1];
                    }
                }
            }
            p += (w << 1);
        }
        if (usePushColors && frame_buffer == visibleFB) {
            pushColors(psRAMbuffer1, isize);
        }
        else {
            WrGRAMs(psRAMbuffer1, isize);
        }
    }
    if (orientation == VERTICAL) {
        pixelRange = h - (gap1 + gap2);
        val = range - val;
        inc = (float)pixelRange / (float)range;
        changePoint = (inc * (float)val) + gap1;
        LastLinearPointerPosition = (int16_t)changePoint;
        pos = tuiIndex[ui] + 8;
        uint32_t pc;
        uint32_t p = 0;
        pc = pos;
        GCIseek(pc);
        GCIreadToBuf(p, changePoint * (w << 1));
        GCIseek(pc + (isize << 1) + (changePoint * (w << 1)));
        GCIreadToBuf(p + (changePoint * (w << 1)), ((oh - changePoint) * (w << 1)));
        GCIseek(posk);
        p = ((changePoint - (kh >> 1)) * (w << 1)) + ((w - kw) & 0xfffe);

        while (kh--) {
            if (tc == -1) {
                GCIreadToBuf(p, kw << 1);
            }
            else {
                GCIreadToBuf(tempos, kw << 1);
                for (int n = 0; n < kw; n++) {
                    if (psRAMbuffer1[tempos + (n << 1)] != tc1 &&
                        psRAMbuffer1[tempos + (n << 1) + 1] != tc2) {
                        psRAMbuffer1[p + (n << 1)] = psRAMbuffer1[tempos + (n << 1)];
                        psRAMbuffer1[p + (n << 1) + 1] =
                            psRAMbuffer1[tempos + (n << 1) + 1];
                    }
                }
            }
            p += (w << 1);
        }
        if (usePushColors && frame_buffer == visibleFB) {
            pushColors(psRAMbuffer1, isize);
        }
        else {
            WrGRAMs(psRAMbuffer1, isize);
        }
    }
}

/****************************************************************************/
/*!
  @brief  get the x or y value of the change position of 2 & 3 image widgets
  @param  none
  @note   returns x value if horizontal or y value if vertical.
*/
/****************************************************************************/
int16_t gfx4desp32P4::getLastPointerPos() { return LastLinearPointerPosition; }

/****************************************************************************/
/*!
  @brief  Draw widget from GCI file. UserImage / UserImages draw function
  @param  Fname - filename(String) of GCI format image file
  @note clipping and out of bounds now handled by WrGRAMs functions
*/
/****************************************************************************/
void gfx4desp32P4::DrawImageFile(String Fname) {
	String fnTemp = Fname;
    fnTemp.toUpperCase();
	Fname = "/" + Fname;
    dataFile = SD_MMC.open((char*)Fname.c_str());
   if (!dataFile)
        return;
    byte ofst = 6;
    int uix = cursor_x;
    int uiy = cursor_y;
	int uiw;
    int uih;
    uint32_t isize;
	bool jpegGC;
    if (fnTemp.indexOf(".JPG") == (fnTemp.length() - 4) || fnTemp.indexOf(".JPEG") == (fnTemp.length() - 5)) {
		int dmy = DecodeJPEGfromFile(dataFile, 0, 0); // may be padded out
		jpegGC = true;
		uiw = JPEGiSize[0];
		uih = JPEGiSize[1];
	} else {
		uiw = (DATAread() << 8) + DATAread();
		uih = (DATAread() << 8) + DATAread();
	}
    isize = (uiw * uih) << 1;
    SetGRAM(uix, uiy, uix + uiw - 1, uiy + uih - 1);
    if (jpegGC){
		WrGRAMs((uint16_t*)rx_buf, isize >> 1);
	} else {
		WrGRAMs(psRAMbuffer1, isize >> 1);
	}
    dataFile.close();
	_lastFrame = -1; _lastObject = -1;
}

/****************************************************************************/
/*!
  @brief  Draw widget from GCI file. UserImage / UserImages draw function
  @param  Fname - filename(const char) of GCI format image file
  @note clipping and out of bounds now handled by WrGRAMs functions
*/
/****************************************************************************/
void gfx4desp32P4::DrawImageFile(const char* Fname) {
	String fnTemp = Fname;
    DrawImageFile(fnTemp); 
}

/****************************************************************************/
/*!
  @brief  Draw widget from GCI array. (UserImage)
  @param  ImageArray array of GCI format image file
  @note clipping and out of bounds now handled by WrGRAMs functions
*/
/****************************************************************************/
void gfx4desp32P4::DrawImageArray(uint16_t* ImageArray) {
    int uix = cursor_x;
    int uiy = cursor_y;
    int uiw = (int)ImageArray[0];
    int uih = (int)ImageArray[1];
    uint32_t isize;
    isize = (uiw * uih);
    SetGRAM(uix, uiy, uix + uiw - 1, uiy + uih - 1);
    WrGRAMs(ImageArray + 3, isize);
}

/****************************************************************************/
/*!
  @brief  Draw widget from GCI array. (UserImage)
  @param  ImageArray array of GCI format image file
  @note clipping and out of bounds now handled by WrGRAMs functions
*/
/****************************************************************************/
void gfx4desp32P4::DrawImageArray(uint8_t* ImageArray) {
    int uix = cursor_x;
    int uiy = cursor_y;
    int uiw = ((int)ImageArray[0] << 8) + (int)ImageArray[1];
    int uih = ((int)ImageArray[2] << 8) + (int)ImageArray[3];
    uint32_t isize;
    isize = (uiw * uih) << 1;
    SetGRAM(uix, uiy, uix + uiw - 1, uiy + uih - 1);
    WrGRAMs(ImageArray + 6, isize);
}

/****************************************************************************/
/*!
  @brief  Helper function to translate for different orientations and rotates
  @note
*/
/****************************************************************************/
void gfx4desp32P4::TranslateCoords(int x, int y, int w, int h, int deg, int fb){
	if ((deg == 90 || deg == 270) && rotation > 1) gfx_Swap(w, h);
	int fx = x; int fy = y; int fw = w; int fh = h;
	int tx, ty, tx2, ty2;
	int hres, vres;
	if (fb < CANVAS_BUFFER){
		hres = st_hres; vres = st_vres;
    } else if (fb < CANVAS_BUFFER_ARGB){
		hres = altst_hres; vres = altst_vres;
	} else {
		hres = altst_hresARGB; vres = altst_vresARGB;
	}
	switch (rotation){
		case 1:
			tx = fx; ty = fy;
			tx2 = fx + fw - 1;
			ty2 = fy + fh - 1;
			tx = (hres - 1) - tx2;
			ty = (vres - 1) - ty2;
			fx = tx;
			fy = ty;
			deg += 180;
			deg = deg % 360;
			break;
		case 2:
			ty = fy;
			tx2 = fx + fw - 1;
			fy = (vres - 1) - tx2;
			ty2 = (hres - 1) - fx;
			fx = ty;
			gfx_Swap(fw, fh);
			deg += 90;
			deg = deg % 360;
			break;
		case 3:
			tx = fx;
			tx2 = fx + fw - 1;
			ty2 = fy + fh - 1;
			fx = (hres - 1) - ty2;
			fy = tx;
			gfx_Swap(fw, fh);
			deg += 270;
			deg = deg % 360;
			break;
	}
	_translated[0] = fx;
	_translated[1] = fy;
	_translated[2] = fw;
	_translated[3] = fh;
	_translated[4] = deg;
}

/****************************************************************************/
/*!
  @brief  Draw image from selected frame buffer with Scale, Rotation and mirror defined by output window 
  @param  fb - frame buffer where image is stored
  @param  fx - top left x where image in frame buffer starts
  @param  fy - top right y where image in frame buffer starts
  @param  fw - width of image in frame buffer
  @param  fh - height of image in frame buffer
  @param  deg - angle of rotation, only 0, 90, 180, 270 degrees allowed
  @param  scW - Width of scaled image
  @param  scH - Height of scaled image
  @param  mirX - true if miiror on the X axis
  @param  mirY - true if miiror on the Y axis
  @param  altx - Alternative x position in pixels
  @param  alty - Alternative y position in pixels
  @note target buffer is currently selected frame buffer
*/
/****************************************************************************/
void gfx4desp32P4::DrawFrameBufferAreaScaleRotate(int fb, int fx, int fy, float fw, float fh, int deg, float scW, float scH, bool mirX, bool mirY, int altX, int altY, int fb2){
    uint16_t* tpto;
	int fbt;
	if (fb2 == -1){
	  tpto = (uint16_t*)SelectFB(frame_buffer);
	  fbt = frame_buffer;
    } else {	  
	  tpto = (uint16_t*)SelectFB(fb2);
	  fbt = fb2;
	}
	int x = altX;
	int y = altY;
	int nx, ny, nw, nh, ndeg;
	int ideg = deg;
	TranslateCoords(fx, fy, fw, fh, deg, fbt);
	nx = _translated[TRANS_X];
	ny = _translated[TRANS_Y];
	nw = _translated[TRANS_W];
	nh = _translated[TRANS_H];
	ndeg = _translated[TRANS_DEG];
	TranslateCoords(x, y, scW, scH, deg, frame_buffer);
	ndeg = ndeg % 360;
	uint16_t* pto = (uint16_t*)SelectFB(fb);
	int hres, vres;
	if (fb < CANVAS_BUFFER){
		hres = st_hres; vres = st_vres;
    } else if (fb < CANVAS_BUFFER_ARGB){
		hres = altst_hres; vres = altst_vres;
	} else {
		hres = altst_hresARGB; vres = altst_vresARGB;
	}
	if (ndeg == 90 || ndeg == 270) gfx_Swap(scW, scH);
	ppaAccelerator.scaleRotateImageFB(pto, hres, vres, nx, ny, nw, nh, tpto, scW, scH, false, deg, mirX, mirY, _translated[TRANS_X], _translated[TRANS_Y], 0, 0, colorFMT24 & (frame_buffer == 0), frame_buffer == CANVAS_BUFFER_ARGB);
} 

/****************************************************************************/
/*!
  @brief  Draw frame buffer area using screen co-ordinates from selected
              buffer to target buffer
  @param  fbnum - frame buffer number to draw from.
  @param  x1 - left co-ordinate
  @param  y1 - top co-ordinate
  @param  x2 - right co-ordinate
  @param  y2 - bottom co-ordinate
*/
/****************************************************************************/
void gfx4desp32P4::DrawFrameBufferAreaPPA(int fb, int x1, int y1, int x2, int y2){
  uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
  uint16_t* pto = (uint16_t*)SelectFB(fb);
  int fw = x2 - x1 + 1;
  int fh = y2 - y1 + 1;
  TranslateCoords(x1, y1, fw, fh, 0, frame_buffer);
  bool bswap = false;
  int hres, vres;
  if (fb < CANVAS_BUFFER){
	  hres = st_hres; vres = st_vres;
  } else if (fb < CANVAS_BUFFER_ARGB){
	  hres = altst_hres; vres = altst_vres;
  } else {
	  hres = altst_hresARGB; vres = altst_vresARGB;
  }
  ppaAccelerator.scaleRotateImageFB(pto, hres, vres, _translated[TRANS_X], _translated[TRANS_Y], _translated[TRANS_W], _translated[TRANS_H], tpto, _translated[TRANS_W], _translated[TRANS_H], bswap, 0, false, false,  _translated[TRANS_X], _translated[TRANS_Y], 0, 0, colorFMT24 & (frame_buffer == 0), frame_buffer == CANVAS_BUFFER_ARGB);
}

/****************************************************************************/
/*!
  @brief  Draw frame buffer area using screen co-ordinates from selected
              buffer to target buffer at position x, y
  @param  fbnum - frame buffer number to draw from.
  @param  x1 - left co-ordinate
  @param  y1 - top co-ordinate
  @param  x2 - right co-ordinate
  @param  y2 - bottom co-ordinate
  @param  x  - target x position
  @param  y  - target y position
*/
/****************************************************************************/
void gfx4desp32P4::DrawFrameBufferAreaXYPPA(int fb, int x1, int y1, int x2, int y2, int x, int y){
	DrawFrameBufferAreaScaleRotate(fb, x1, y1, x2 - x1 + 1, y2 - y1 + 1, 0, x2 - x1 + 1, y2 - y1 + 1, false, false, x, y, -1);
}

/****************************************************************************/
/*!
  @brief  Functions for drawing an LVGL buffer to display from RGB565 or RGB888 sources
  @param  srcBuff - buffer where image data is stored
  @param  scrW - width of source buffer
  @param  scrH - height of  of source buffer
  @param  x - top left x where image data is to be written
  @param  y - top left y where image data is to be written
  @param  bswap - true to swap bytes (RGB565) of source image data
  @param  fmt24 - true if source is RGB888
  @note output is always RGB 565
*/
/****************************************************************************/
void gfx4desp32P4::PPAflushArea(uint8_t* srcBuff, int srcW, int srcH, int x, int y, bool bswap, bool fmt24){
  uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
  uint16_t* pto = (uint16_t*)srcBuff;
  TranslateCoords(x, y, srcW, srcH, 0, frame_buffer);
  ppaAccelerator.scaleRotateImageFB(pto, srcW, srcH, 0, 0, srcW, srcH, tpto, srcW, srcH, bswap, _translated[TRANS_DEG], false, false, _translated[TRANS_X], _translated[TRANS_Y], 0, 0, fmt24, frame_buffer == CANVAS_BUFFER_ARGB);
}	

void gfx4desp32P4::PPAflushArea(uint8_t* srcBuff, int srcW, int srcH, int x, int y, bool bswap, int ofstW, int ofstH, bool fmt24){
  uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
  uint16_t* pto = (uint16_t*)srcBuff;
  TranslateCoords(x, y, srcW, srcH, 0, frame_buffer);
  ppaAccelerator.scaleRotateImageFB(pto, srcW, srcH, 0, 0, srcW, srcH, tpto, srcW, srcH, bswap, _translated[TRANS_DEG], false, false, _translated[TRANS_X], _translated[TRANS_Y], ofstW, ofstH, fmt24, frame_buffer == CANVAS_BUFFER_ARGB);
}

void gfx4desp32P4::PPAflushArea(uint16_t* srcBuff, int srcW, int srcH, int x, int y, bool bswap, bool fmt24){
  uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
  uint16_t* pto = srcBuff;
  TranslateCoords(x, y, srcW, srcH, 0, frame_buffer);
  ppaAccelerator.scaleRotateImageFB(pto, srcW, srcH, 0, 0, srcW, srcH, tpto, srcW, srcH, bswap, _translated[TRANS_DEG], false, false, _translated[TRANS_X], _translated[TRANS_Y], 0, 0, fmt24, frame_buffer == CANVAS_BUFFER_ARGB);
}

void gfx4desp32P4::PPAflushArea(uint16_t* srcBuff, int srcPw, int srcPh, int x, int y, int srcW, int srcH, bool bswap, int trgW, int trgH, int trgX, int trgY, int deg, bool fmt24){
  uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
  uint16_t* pto = srcBuff;
  TranslateCoords(x, y, srcW, srcH, 0, frame_buffer);
  ppaAccelerator.scaleRotateImageFB(pto, srcPw, srcPh, x, y, srcW, srcH, tpto, trgW, trgH, bswap, deg, false, false, trgX, trgY, 0, 0, fmt24, frame_buffer == CANVAS_BUFFER_ARGB);
}

/****************************************************************************/
/*!
  @brief  draws a rectangle to the current frame buffer using PPA accelerator.
  @param  x0 - The X position of the top left corner of the rectangle
  @param  y0 - The Y position of the top left corner of the rectangle
  @param  x1 - The X position of the bottom right corner of the rectangle
  @param  y1 - The Y position of the bottom right corner of the rectangle
  @param  color - color of the filled rectangle. Alpha Blend can be used.
  @note returns nothing.
*/
/****************************************************************************/
void gfx4desp32P4::RectangleFilledPPA(int x1, int y1, int x2, int y2, uint16_t color){
  if (_transparentColor == color && transparency) return;
  uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
  if (x1 > x2) gfx_Swap(x1, x2);
  if (y1 > y2) gfx_Swap(y1, y2);
  if (x1 >= clipx2 || x2 < clipx1 || y1 >= clipy2 || y2 < clipy1) return;
  if (x1 < clipx1) x1 = clipx1;
  if (y1 < clipy1) y1 = clipy1;
  if (x2 > clipx2) x2 = clipx2;
  if (y2 > clipy2) y2 = clipy2;
  int fw = x2 - x1 + 1;
  int fh = y2 - y1 + 1;
  TranslateCoords(x1, y1, fw, fh, 0, frame_buffer);
  int tAlpha = 255;
  if(alpha) tAlpha = __alpha;
  int hres, vres;
  if (frame_buffer < CANVAS_BUFFER){
	hres = st_hres; vres = st_vres;
  } else if (frame_buffer < CANVAS_BUFFER_ARGB){
	hres = altst_hres; vres = altst_vres;
  } else {
	hres = altst_hresARGB; vres = altst_vresARGB;
  }
  ppaAccelerator.blockFill(hres, vres, _translated[TRANS_X], _translated[TRANS_Y], _translated[TRANS_W], _translated[TRANS_H], tpto, color, tAlpha, colorFMT24, frame_buffer == CANVAS_BUFFER_ARGB);
}

/****************************************************************************/
/*!
  @brief  Draw single Userimage
  @param  ui - UserImage ID
  @note pre - drawWidget function
*/
/****************************************************************************/
void gfx4desp32P4::UserImage(uint16_t ui) { UserImage(ui, 0x7fff, 0x7fff); }


/****************************************************************************/
/*!
  @brief  Draw single Userimage in alternative location
  @param  ui - UserImage ID
  @param  altx - Alternative x position in pixels
  @param  alty - Alternative y position in pixels
  @note pre - drawWidget function
*/
/****************************************************************************/
void gfx4desp32P4::UserImage(uint16_t ui, int altx, int alty) {
    boolean setemp = sEnable;
    ScrollEnable(false);
    if (altx == 0x7fff && alty == 0x7fff) {
        DrawWidget(tuiIndex[ui], tuix[ui], tuiy[ui], tuiw[ui], tuih[ui], 0, 0,
            false, cdv[ui], ui);
    }
    else {
        DrawWidget(tuiIndex[ui], altx, alty, tuiw[ui], tuih[ui], 0, 0, false,
            cdv[ui], ui);
    }
    ScrollEnable(setemp);
}

/****************************************************************************/
/*!
  @brief  Draw frame from UserImages set
  @param  uisnb - UserImage ID
  @param  framenb - frame number
  @note pre - drawWidget function
*/
/****************************************************************************/
void gfx4desp32P4::UserImages(uint16_t uisnb, int16_t framenb) {
    tuiImageIndex[uisnb] = framenb;
    boolean setemp = sEnable;
    ScrollEnable(false);
    if (framenb > (gciobjframes[uisnb] - 1) || framenb < 0) {
        outofrange(tuix[uisnb], tuiy[uisnb], tuiw[uisnb], tuih[uisnb]);
    }
    else {
        DrawWidget(tuiIndex[uisnb], tuix[uisnb], tuiy[uisnb], tuiw[uisnb], tuih[uisnb], framenb, 0,
            true, cdv[uisnb], uisnb);
    }
    ScrollEnable(setemp);
}

/****************************************************************************/
/*!
  @brief  Draw frame from UserImages set
  @param  uisnb - UserImage ID
  @note pre - drawWidget function
*/
/****************************************************************************/
void gfx4desp32P4::imageShow(uint16_t uisnb) {
    boolean setemp = sEnable;
    ScrollEnable(false);
    if (gciobjframes[uisnb] > 0) {
        DrawWidget(tuiIndex[uisnb], tuix[uisnb], tuiy[uisnb], tuiw[uisnb], tuih[uisnb], tuiImageIndex[uisnb], 0,
            true, cdv[uisnb], uisnb);
    }
    else {
        DrawWidget(tuiIndex[uisnb], tuix[uisnb], tuiy[uisnb], tuiw[uisnb], tuih[uisnb], 0, 0,
            false, cdv[uisnb], uisnb);
    }
    ScrollEnable(setemp);
}


/****************************************************************************/
/*!
  @brief  Draw frame from UserImages set in alternative location
  @param  uis - UserImage ID
  @param  frame - frame number
  @param  offset - used for spectrum widgets
  @param  altx - alternative x position in pixels
  @param  alty - alternative y position in pixels
  @note pre - drawWidget function
*/
/****************************************************************************/
void gfx4desp32P4::UserImages(uint16_t uis, int16_t frame, int offset,
    int16_t altx, int16_t alty) {
    tuiImageIndex[uis] = frame;
    boolean setemp = sEnable;
    ScrollEnable(false);
    if (frame > (gciobjframes[uis] - 1) || frame < 0) {
        outofrange(altx, alty, tuiw[uis], tuih[uis]);
    }
    else {
        DrawWidget(tuiIndex[uis], altx, alty, tuiw[uis], tuih[uis], frame, offset,
            true, cdv[uis], uis);
    }
    ScrollEnable(setemp);
}

/****************************************************************************/
/*!
  @brief  Draw graphic box and red cross to signal widget out of range
  @param  euix - x position in pixels
  @param  euiy - y position in pixels
  @param  euiw - width in pixels
  @param  euih - height in pixels
  @note pre - drawWidget function
*/
/****************************************************************************/
void gfx4desp32P4::outofrange(int16_t euix, int16_t euiy, int16_t euiw,
    int16_t euih) {
    if (euix >= width || euiy >= height)
        return;
    if ((euix + euiw - 1) < 0 || (euiy + euih - 1) < 0)
        return;
    int cuix = euix;
    int cuiy = euiy;
    int cuiw = euiw;
    int cuih = euih;
    if (euix < 0) {
        cuix = 0;
        cuiw = euiw + euix;
    }
    if (euiy < 0) {
        cuiy = 0;
        cuih = euih + euiy;
    }
    if ((euix + euiw - 1) >= width)
        cuiw = euiw - ((euix + euiw - 1) - width) - 1;
    if ((euiy + euih - 1) >= height)
        cuih = euih - ((euiy + euih - 1) - height) - 1;
    RectangleFilled(cuix, cuiy, cuix + cuiw - 1, cuiy + cuih - 1, BLACK);
    Rectangle(cuix + 1, cuiy + 1, cuix + cuiw - 2, cuiy + cuih - 2, RED);
    //StartWrite();
    Line(cuix + 1, cuiy + 1, cuix + cuiw - 2, cuiy + cuih - 2, RED);
    Line(cuix + cuiw - 2, cuiy + 1, cuix + 1, cuiy + cuih - 2, RED);
    //EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw Led digits value
  @param  see manual
  @note accepts signed values upto 64 bits
*/
/****************************************************************************/
void gfx4desp32P4::LedDigitsDisplaySigned(int64_t newval, uint16_t index,
    int16_t Digits, int16_t MinDigits,
    int16_t WidthDigit,
    int16_t LeadingBlanks) {
    LedDigitsDisplaySigned(newval, index, Digits, MinDigits, WidthDigit,
        LeadingBlanks, 0x7fff, 0x7fff);
}

/****************************************************************************/
/*!
  @brief  Draw Led digits value
  @param  see manual
  @note accepts signed values upto 64 bits
*/
/****************************************************************************/
void gfx4desp32P4::LedDigitsDisplaySigned(int64_t newval, uint16_t index,
    int16_t Digits, int16_t MinDigits,
    int16_t WidthDigit,
    int16_t LeadingBlanks, int16_t altx,
    int16_t alty) {
    int16_t i, m, lstb, nv, digita[7];
    int leftpos = 0;
    nv = newval;
    lstb = 1;
    for (i = Digits; i > 0; i--) {
        m = nv % 10;
        if (LeadingBlanks && (i <= Digits - MinDigits)) {
            if (nv == 0) {
                m = 10;
                if (lstb == 1)
                    lstb = i;
            }
        }
        digita[i] = abs(m);
        nv /= 10;
    }
    if (newval < 0) {
        digita[lstb] = 11;
    }
    for (i = 1; i <= Digits; i++) {
        if (altx == 0x7fff && alty == 0x7fff) {
            UserImages(index, digita[i], leftpos);
        }
        else {
            UserImages(index, digita[i], leftpos, altx, alty);
        }
        leftpos += WidthDigit;
    }
}

/****************************************************************************/
/*!
  @brief  Draw Led digits value
  @param  see manual
  @note accepts un-signed values upto 64 bits
*/
/****************************************************************************/
void gfx4desp32P4::LedDigitsDisplay(int64_t newval, uint16_t index,
    int16_t Digits, int16_t MinDigits,
    int16_t WidthDigit, int16_t LeadingBlanks) {
    LedDigitsDisplay(newval, index, Digits, MinDigits, WidthDigit, LeadingBlanks,
        0x7fff, 0x7fff);
}

/****************************************************************************/
/*!
  @brief  Draw Led digits value
  @param  see manual
  @note accepts un-signed values upto 64 bits
*/
/****************************************************************************/
void gfx4desp32P4::LedDigitsDisplay(int64_t newval, uint16_t index,
    int16_t Digits, int16_t MinDigits,
    int16_t WidthDigit, int16_t LeadingBlanks,
    int16_t altx, int16_t alty) {
    int16_t i, k, lb;
    int64_t l;
    l = 1;
    for (i = 1; i < Digits; i++)
        l *= 10;
    lb = LeadingBlanks;
    for (i = 0; i < Digits; i++) {
        k = newval / l;
        newval -= k * l;
        if (lb && (i < Digits - MinDigits)) {
            if (k == 0)
                k = 10;
            else
                lb = 0;
        }
        l /= 10;
        if (altx == 0x7fff && alty == 0x7fff) {
            UserImages(index, k, i * WidthDigit);
        }
        else {
            UserImages(index, k, i * WidthDigit, altx, alty);
        }
    }
}

/****************************************************************************/
/*!
  @brief  Draw frame from UserImages set in new location
  @param  uisnb - UserImages widget number
  @param  framenb - selected frame
  @param  newx - new x position in pixels
  @param  newy - new y position in pixels
  @note maybe duplicate function
*/
/****************************************************************************/
void gfx4desp32P4::UserImages(uint16_t uisnb, int16_t framenb, int16_t newx,
    int16_t newy) {
    tuiImageIndex[uisnb] = framenb;
    uimage = false;
    boolean setemp = sEnable;
    ScrollEnable(false);
    if (framenb > (gciobjframes[uisnb] - 1) || framenb < 0) {
        outofrange(tuix[uisnb], tuiy[uisnb], tuiw[uisnb], tuih[uisnb]);
    }
    else {
        DrawWidget(tuiIndex[uisnb], newx, newy, tuiw[uisnb], tuih[uisnb], framenb,
            0, true, cdv[uisnb], uisnb);
    }
    ScrollEnable(setemp);
}


/****************************************************************************/
/*!
  @brief  Draw frame from UserImages set in default location
  @param  uis - Userimages widget number
  @param  frame - selected frame
  @param  offset - used for spectrum widgets
  @note pre drawWidget function
*/
/****************************************************************************/
void gfx4desp32P4::UserImages(uint16_t uis, int16_t frame, int offset) {
    tuiImageIndex[uis] = frame;
    boolean setemp = sEnable;
    ScrollEnable(false);
    if (frame > (gciobjframes[uis] - 1) || frame < 0) {
        outofrange(tuix[uis], tuiy[uis], tuiw[uis], tuih[uis]);
    }
    else {
        DrawWidget(tuiIndex[uis], tuix[uis], tuiy[uis], tuiw[uis], tuih[uis], frame,
            offset, true, cdv[uis], uis);
    }
    ScrollEnable(setemp);
}


/****************************************************************************/
/*!
  @brief  Draws rectangular segment of frame from UserImages set in default
  location
  @param  uino - Userimages widget number
  @param  frames - selected frame
  @param  uxpos - position of start of segment from top left corner of widget
  @param  uypos - position of start of segment from top left corner of widget
  @param  uwidth - width of segment
  @param  uheight - height of segment
*/
/****************************************************************************/
void gfx4desp32P4::UserImagesDR(uint16_t uino, int frames, int16_t uxpos,
    int16_t uypos, int16_t uwidth, int16_t uheight) {
    if (uxpos >= width || uypos >= height || uxpos < 0 || uypos < 0)
        return;
    if (uwidth < 1 || uheight < 1)
        return;
    tuiImageIndex[uino] = frames;
    uint32_t bgoff = 0;
    boolean setemp = sEnable;
    ScrollEnable(false);
    //if (uxpos + uwidth > tuiw[uino])
    //    uwidth = tuiw[uino] - uxpos;
    //if (uypos + uheight > tuih[uino])
    //    uheight = tuih[uino] - uypos;
    if ((GCItype == GCI_SYSTEM_USD && !userImag) || (GCItype == GCI_SYSTEM_JPEG_USD && !userImag) ||
        (GCItype == GCI_SYSTEM_PROGMEM && (!gcidatArray)))
        return;
    if (GCItype == GCI_SYSTEM_JPEG_USD || GCItype == GCI_SYSTEM_JPEG_FLASH){
		int twO; 
		int frame = 0;
		uint32_t jpegsize;
		if (_lastObject != uino || (_lastObject == uino && _lastFrame != frames)){
	        jpegsize = jpegSIZES[jpegOFFSETSpos[uino] + frames];
			if(GCItype != GCI_SYSTEM_JPEG_USD){
				twO = DecodeJPEGfromGCJinFlash(jpegOFFSETS[jpegOFFSETSpos[uino] + frames], jpegsize, tuiw[uino], tuih[uino]);
			} else {
				twO = DecodeJPEGfromGCJ(jpegOFFSETS[jpegOFFSETSpos[uino] + frames], jpegsize, tuiw[uino], tuih[uino]);
			}
			if (twO == 16) twO = 0;
			_lastFrame = frames; _lastObject = uino;
		} else {
			twO = lasttwO;
		}
		lasttwO = twO;
		if(transparency || alpha){
		    int th1 = uheight;
			uint32_t pos = ((tuiw[uino] + twO) * (uypos - tuiy[uino])) + (uxpos - tuix[uino]);
			//SetGRAM (uxpos + tuix[uino], uypos + tuiy[uino], uxpos + tuix[uino] + uwidth - 1, uypos + tuiy[uino] + uheight - 1);
			SetGRAM (uxpos, uypos, uxpos + uwidth - 1, uypos + uheight - 1);
			while(th1--){
				WrGRAMs((uint16_t*)rx_buf + pos, uwidth);
				pos += (tuiw[uino] + twO);
			}
		} else {
			uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
			uint16_t* pto = (uint16_t*)rx_buf;
			TranslateCoords(uxpos, uypos, uwidth, uheight, 0, frame_buffer);
			ppaAccelerator.scaleRotateImageFB(pto, tuiw[uino], tuih[uino], uxpos - tuix[uino], uypos - tuiy[uino], uwidth, uheight, tpto, uwidth, uheight, 0, _translated[TRANS_DEG], false, false, _translated[TRANS_X], _translated[TRANS_Y], twO, 0, colorFMT24, frame_buffer == CANVAS_BUFFER_ARGB);
		}
	} else {
	    uint32_t isize = tuiw[uino] * tuih[uino];
        uint32_t isize2 = uwidth * uheight;
        int16_t wuid = uwidth;
        int16_t huid = uheight;
        uint32_t pos;
        uint32_t uoff;
        int16_t x0 = 0;
        int16_t y0 = 0;
        int16_t x1 = 0;
        int16_t y1 = 0;
        pos = (isize * frames) << 1;
        uoff = ((uypos * tuiw[uino]) + uxpos) << 1;
        bgoff = tuiIndex[uino] + 8 + pos + uoff + 0;
        if (frames > (gciobjframes[uino] - 1) || frames < 0) {
            outofrange(tuix[uino] + uxpos, tuiy[uino] + uypos, uwidth, uheight);
            ScrollEnable(setemp);
            return;
        }
        x0 = tuix[uino] + uxpos;
        y0 = tuiy[uino] + uypos;
        x1 = tuix[uino] + uxpos + uwidth - 1;
        y1 = tuiy[uino] + uypos + huid - 1;
        usePushColors = (DisplayType == DISP_INTERFACE_RGB) && (x0 >= clipx1) &&
            (y0 >= clipy1) && (x1 <= clipx2) && (y1 <= clipy2) &&
            (!transalpha) && (!WriteFBonly) && (frame_buffer == visibleFB);
        SetGRAM(x0, y0, x1, y1);
        GCIseek(bgoff);
        uint32_t p = 0;
        while (huid--) {
            if (GCItype == GCI_SYSTEM_USD) {
                GCIreadToBuf(p, wuid << 1);
                p += (wuid << 1);
                bgoff += (tuiw[uino] << 1);
                GCIseek(bgoff);

            } else {
               if (usePushColors && frame_buffer == visibleFB) {
                   pushColors(GCIarray + bgoff, wuid);
               } else {
                   WrGRAMs(GCIarray + bgoff, wuid);
               }
               bgoff += (tuiw[uino] << 1);
            }
        }
        if (GCItype == GCI_SYSTEM_USD) {
            if (usePushColors && frame_buffer == visibleFB) {
                pushColors(psRAMbuffer1, isize2);
            } else {
                WrGRAMs(psRAMbuffer1, isize2);
            }
        }
	}
	ScrollEnable(setemp);
}

/****************************************************************************/
/*!
  @brief  Draws UserImages widget as a rounded rectangle.
  @param  ui - Index of GCI / GCJ USerimages widget
  @param  frame - the frame number of widget to be displayed
  @param  radius - Radius of the Rounded rectangle corners 
  @note   Widget is drawn at its current position and size. ImagSetWord can alter position only
*/
/****************************************************************************/
void gfx4desp32P4::UserImagesAsRoundRect(int ui, int frame, int radius){
	tuiImageIndex[ui] = frame;
	RoundRectFilledAA(tuix[ui], tuiy[ui], tuiw[ui], tuih[ui], radius, SelectDataSourceGCI(ui));
	tuiExtra2[ui] = tuiExtra2[ui] & 0x3fff;
	if (frame_buffer == CANVAS_BUFFER) tuiExtra2[ui] = (tuiExtra2[ui] & 0x3fff) | 0x8000;
	if (frame_buffer == CANVAS_BUFFER_ARGB) tuiExtra2[ui] = (tuiExtra2[ui] & 0x3fff) | 0x4000;
}

/****************************************************************************/
/*!
  @brief  Draws UserImage widget as a rounded rectangle.
  @param  ui - Index of GCI / GCJ USerimage widget
  @param  radius - Radius of the Rounded rectangle corners 
  @note   Widget is drawn at its current position and size. ImagSetWord can alter position only
*/
/****************************************************************************/
void gfx4desp32P4::UserImageAsRoundRect(int ui, int radius){
	tuiImageIndex[ui] = 0;
	RoundRectFilledAA(tuix[ui], tuiy[ui], tuiw[ui], tuih[ui], radius, SelectDataSourceGCI(ui));
	tuiExtra2[ui] = tuiExtra2[ui] & 0x3fff;
	if (frame_buffer == CANVAS_BUFFER) tuiExtra2[ui] = (tuiExtra2[ui] & 0x3fff) | 0x8000;
	if (frame_buffer == CANVAS_BUFFER_ARGB) tuiExtra2[ui] = (tuiExtra2[ui] & 0x3fff) | 0x4000;
}

/****************************************************************************/
/*!
  @brief  compatible Set GRAM window ready for wrGRAM/s functions.
  @param  x1 - left X position in pixels
  @param  y1 - top Y position in pixels
  @param  w - width in pixels
  @param  h - height in pixels
  @note Uses SetGRAM / setGRAM
*/
/****************************************************************************/
void gfx4desp32P4::setAddrWindow(int16_t x1, int16_t y1, int16_t w, int16_t h) {
    SetGRAM(x1, y1, x1 + w - 1, y1 + h - 1);
}

/****************************************************************************/
/*!
  @brief  Draws UserImage at current text cursor position
  @param  ui - UserImage widget ID number
  @note Draws image in the same manner as drawing text. If Scroll is enabled
    then image will also scroll while being drawn.
*/
/****************************************************************************/
void gfx4desp32P4::PrintImage(uint16_t ui) {
    if (cursor_x > textXmax/*(width - 1)*/)
        return;
    boolean tempnl = false;
    if (nl) {
        cursor_x = textXmin;
        tempnl = true;
        newLine(lastfsh, textsizeht, textXmin);
    }
    if (cursor_y > (height - 1))
        return;
    if ((GCItype == GCI_SYSTEM_USD && (!userImag)) || (GCItype == GCI_SYSTEM_JPEG_USD && (!userImag)) ||
		(GCItype == GCI_SYSTEM_PROGMEM && (!gcidatArray))/* || (GCItype == GCI_SYSTEM_JPEG_FLASH && (!GCIarray))*/)
        return;
    uint16_t iwidth = tuiw[ui];
    uint16_t iheight = tuih[ui];
    uint8_t mul = cdv[ui] / 8;
    uint32_t pos = tuiIndex[ui] + 6;
	int twO;
    bool jpegGC = ((GCItype == GCI_SYSTEM_JPEG_USD) || (GCItype == GCI_SYSTEM_JPEG_FLASH));
	if (jpegGC) {
		pos = 0;
		uint32_t jpegsize = jpegSIZES[jpegOFFSETSpos[ui]];
		if(_lastObject != ui || _lastFrame != 0){
			if(GCItype == GCI_SYSTEM_JPEG_USD){
				twO = DecodeJPEGfromGCJ(jpegOFFSETS[jpegOFFSETSpos[ui]], jpegsize, iwidth, iheight);
			} else {
				twO = DecodeJPEGfromGCJinFlash(jpegOFFSETS[jpegOFFSETSpos[ui]], jpegsize, iwidth, iheight);
			}
		} else {
			twO = lasttwO;
		}
		if (twO >= 16) twO = 0;
		_lastFrame = 0; _lastObject = ui;
		lasttwO = twO;
	} else {
		GCIseek(pos);
	}
    uint16_t ichunk = iwidth << (mul - 1);
	if (sEnable == false) {
		if (((cursor_y + iheight) - 1) > height - 1)
			iheight = iheight - ((cursor_y + iheight) - height);
	}
	boolean off = false;
	int cuiw = iwidth;
	if ((cursor_x + iwidth - 1) >= width) {
		cuiw = iwidth - ((cursor_x + iwidth - 1) - width) - 1;
		off = true;
	}
	for (int idraw = 0; idraw < iheight; idraw++) {
		nl = true;
		newLine(1, 1, cursor_x);
		if ((cursor_y - 1) < 0) {
			setGRAM(cursor_x, cursor_y + height - 1, cursor_x + cuiw - 1, cursor_y + height - 1);
		} else {
			setGRAM(cursor_x, cursor_y - 1, cursor_x + cuiw - 1, cursor_y - 1);
		}
		if (off) {
			if (jpegGC){
				WrGRAMs((uint16_t*)rx_buf + pos, cuiw << 0);
				pos = pos + ((iwidth + twO) << 1);
			} else {
				GCIread(buff, cuiw << (mul - 1));
				WrGRAMs(buff, cuiw);
				pos = pos + (iwidth << (mul - 1));
				GCIseek(pos);
			}
		} else {
			if (jpegGC){
				WrGRAMs((uint16_t*)rx_buf + pos, tuiw[ui]);
				pos += ((tuiw[ui] + twO) << 0);
			} else {
				GCIread(buff, ichunk);
				WrGRAMs(buff, ichunk >> 1);
			}
		}
	}
    if (tempnl) {
        nl = true;
        lastfsh = 1;
    }
}

/****************************************************************************/
/*!
  @brief  Draws rectangular segment of UserImage set in Alternate location
  @param  ui - Userimage widget number
  @param  uxpos - position of start of segment from top left corner of widget
  @param  uypos - position of start of segment from top left corner of widget
  @param  uwidth - width of segment
  @param  uheight - height of segment
  @param  uix - new x position
  @param  uiy - new y position
*/
/****************************************************************************/
void gfx4desp32P4::UserImageDR(uint16_t ui, int16_t uxpos, int16_t uypos,
    int16_t uwidth, int16_t uheight, int16_t uix,
    int16_t uiy) {
    if (uix > width || uiy > height || uix < 0 || uiy < 0)
        return;
    if ((uix + uwidth - 1) < 0 || (uiy + uheight - 1) < 0)
        return;
    if ((uix + uwidth) > width || (uiy + uheight) > height)
        return;
    ScrollEnable(false);
    uint32_t bgoff;
    if ((GCItype == GCI_SYSTEM_USD && (!userImag)) || (GCItype == GCI_SYSTEM_JPEG_USD && (!userImag)) ||
		(GCItype == GCI_SYSTEM_PROGMEM && (!gcidatArray)) || (GCItype == GCI_SYSTEM_JPEG_FLASH && (!gcidatArray)))
        return;
    if (uxpos + uwidth > tuiw[ui])
        uwidth = tuiw[ui] - uxpos;
    if (uypos + uheight > tuih[ui])
        uheight = tuih[ui] - uypos;
    if (GCItype == GCI_SYSTEM_JPEG_USD || GCItype == GCI_SYSTEM_JPEG_FLASH){
		int twO; 
		int frame = 0;
		uint32_t out_size = 0;
		uint32_t jpegsize;
		if (_lastObject != ui/* && _lastFrame != 0*/){
	        jpegsize = jpegSIZES[jpegOFFSETSpos[ui] + frame];
			if(GCItype == GCI_SYSTEM_JPEG_USD){
				userImag.seek(jpegOFFSETS[jpegOFFSETSpos[ui] + frame]);
				userImag.read(tx_buf, jpegsize);
			} else {
				memcpy(tx_buf, GCIarray + jpegOFFSETS[jpegOFFSETSpos[ui] + frame], jpegsize);
			}
			twO = 16 - (tuiw[ui] % 16);
			if (twO == 16) twO = 0;
			ESP_ERROR_CHECK(jpeg_decoder_process(jpgd_handle, &decode_cfg_rgb, tx_buf, jpegsize, rx_buf, rx_buffer_size, &out_size));
		} else {
			twO = lasttwO;
		}
		lasttwO = twO;
		_lastFrame = frame; _lastObject = ui;
		if(transparency || alpha){
		    int th1 = uheight;
			uint32_t pos = (((tuiw[ui] + twO) * uypos) + uxpos);
			SetGRAM (uix, uiy, uix + uwidth - 1, uiy + uheight - 1);
			while(th1--){
				WrGRAMs((uint16_t*)rx_buf + pos, uwidth);
				pos += (tuiw[ui] + twO);
			}
		} else {
			uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
			uint16_t* pto = (uint16_t*)rx_buf;
			TranslateCoords(uix, uiy, uwidth, uheight, 0, frame_buffer);
			ppaAccelerator.scaleRotateImageFB(pto, tuiw[ui], tuih[ui], uxpos, uypos, uwidth, uheight, tpto, uwidth, uheight, 0, _translated[TRANS_DEG], false, false, _translated[TRANS_X], _translated[TRANS_Y], twO, 0, colorFMT24, frame_buffer == CANVAS_BUFFER_ARGB);
		}
	} else {
	    uint32_t isize2 = uwidth * uheight;
        uint32_t p = 0;
        uint32_t uoff;
        uoff = ((uypos * tuiw[ui]) + uxpos) << 1;
        bgoff = tuiIndex[ui] + 6 + uoff + 0;
        GCIseek(bgoff);
        uint32_t ichunk = isize2 << 1;
        uint16_t steps;
        int bufsize = uwidth << 1;
        SetGRAM(uix, uiy, uix + uwidth - 1, uiy + uheight - 1);
        uint32_t fgap = tuiw[ui] << 1;
        steps = ichunk / bufsize;
        while (steps--) {
            if (GCItype == GCI_SYSTEM_USD) {
                GCIreadToBuf(p, bufsize);
                p += bufsize;
                bgoff = bgoff + fgap;
                GCIseek(bgoff);
            } else {
                WrGRAMs(GCIarray + bgoff, uwidth);
                bgoff += fgap;
            }
        }
        if (GCItype == GCI_SYSTEM_USD) WrGRAMs(psRAMbuffer1, p >> 1);
	}
}

/****************************************************************************/
/*!
  @brief  compatible function not recommended for P4 use, use Canvas buffer instead
*/
/****************************************************************************/
void gfx4desp32P4::UserImageDRcache(uint16_t ui, int16_t uxpos, int16_t uypos,
    int16_t uwidth, int16_t uheight, int16_t uix,
    int16_t uiy) {
    if (uix > width || uiy > height || uix < 0 || uiy < 0)
        return;
    if ((uix + uwidth - 1) < 0 || (uiy + uheight - 1) < 0)
        return;
    if ((uix + uwidth) > width || (uiy + uheight) > height)
        return;
    ScrollEnable(false);
    uint32_t bgoff;
    if ((GCItype == GCI_SYSTEM_USD && !userImag) ||
        (GCItype == GCI_SYSTEM_PROGMEM && (!gcidatArray)))
        return;
    if (!cache_Enabled) {
        AllocateDRcache(DRcache);
        //psRAMbuffer2 = (uint8_t*)ps_malloc(DRcache);
        //cache_Enabled = true;
    }
    int32_t bwidth = tuiw[ui] << 1;
    uint32_t slen;
    if (!cached || lastui != ui) {
        cache_Start = tuiIndex[ui] + (bwidth * uypos) + 6;
        slen = bwidth * uheight;
        GCIreadToBuff2(cache_Start, slen);
        cached = true;
    }
    if (uxpos + uwidth > tuiw[ui])
        uwidth = tuiw[ui] - uxpos;
    if (uypos + uheight > tuih[ui])
        uheight = tuih[ui] - uypos;

    uint32_t isize2 = uwidth * uheight;
    uint32_t p = 0;
    uint32_t uoff;
    uint32_t bchunk = bwidth * uheight;
    uoff = ((uypos * tuiw[ui]) + uxpos) << 1;
    bgoff = tuiIndex[ui] + 6 + uoff + 0;
    GCIseek(bgoff);
    uint32_t ichunk = isize2 << 1;
    uint16_t steps;
    int bufsize = uwidth << 1;
    int x1 = uix + uwidth - 1;
    int y1 = uiy + uheight - 1;
    usePushColors = (DisplayType == DISP_INTERFACE_RGB) && (uix >= clipx1) &&
        (uiy >= clipy1) && (x1 <= clipx2) && (y1 <= clipy2) &&
        (!transalpha) && (!WriteFBonly) && (frame_buffer == visibleFB);
    SetGRAM(uix, uiy, x1, y1);
    uint32_t fgap = tuiw[ui] << 1;
    steps = ichunk / bufsize;
    int32_t bsize;
    int32_t dist = (int32_t)bgoff - (int32_t)cache_Start;
    if (abs(dist) > bwidth) {
        int32_t lines = dist / bwidth;
        bsize = abs(lines) * bwidth;
        if (abs(lines) > uheight) {
            cache_Start = tuiIndex[ui] + (bwidth * uypos) + 6;
            slen = bwidth * uheight;
            GCIreadToBuff2(cache_Start, slen);
        }
        else {
            if (lines < 0) {
                memmove(psRAMbuffer2 + bsize, psRAMbuffer2, bchunk - bsize);
                cache_Start -= bsize;
                GCIreadToBuff2(cache_Start, bsize);
            }
            if (lines > 0) {
                memmove(psRAMbuffer2, psRAMbuffer2 + bsize, bchunk - bsize);
                cache_Start += bsize;
                GCIreadToBuff2(cache_Start + (bchunk - bsize), bchunk - bsize, bsize);
            }
        }
    }
    while (steps--) {
        memcpy(psRAMbuffer1 + p, psRAMbuffer2 + bgoff - cache_Start, bufsize);
        p += bufsize;
        bgoff = bgoff + fgap;
    }
    if (usePushColors && frame_buffer == visibleFB) {
        pushColors(psRAMbuffer1, p >> 1);
    }
    else {
        WrGRAMs(psRAMbuffer1, p >> 1);
    }
    lastui = ui;
}

/****************************************************************************/
/*!
  @brief  calculate x & y co-ordinate using given angle and length using current
    x - y origin (MoveTo)
  @param  angle - Degrees
  @param  length - Pixels from x - y origin
  @param  oxy - prefined 2 int array containing x at location 0 and y at
  location 1
*/
/****************************************************************************/
void gfx4desp32P4::Orbit(int angle, int lngth, int* oxy) {
    if(lngth < 0){
	  lngth = abs(lngth);
	  angle += 180;
	}
	float sx = cos((angle - 90) * 0.0174532925);
    float sy = sin((angle - 90) * 0.0174532925);
    oxy[0] = (int)(sx * lngth + cursor_x);
    oxy[1] = (int)(sy * lngth + cursor_y);
	lastOrbit[0] = oxy[0];
	lastOrbit[1] = oxy[1];
}

/****************************************************************************/
/*!
  @brief  calculate float x & y co-ordinate using given angle and length using current
    x - y origin (MoveTo)
  @param  angle - Degrees
  @param  length - Pixels from x - y origin
  @param  oxy - prefined 2 float array containing x at location 0 and y at
  location 1
*/
/****************************************************************************/
void gfx4desp32P4::Orbit(float angle, float lngth, float* oxy) {
    if(lngth < 0){
	  lngth = fabs(lngth);
	  angle += 180;
	}
	float sx = cos((angle - 90) * 0.0174532925);
    float sy = sin((angle - 90) * 0.0174532925);
    oxy[0] = (sx * lngth + cursor_x);
    oxy[1] = (sy * lngth + cursor_y);
	flastOrbit[0] = oxy[0];
	flastOrbit[1] = oxy[1];
}

/****************************************************************************/
/*!
  @brief  convert  R, G, B to RGB565
  @param  rc - RED
  @param  gc - GREEN
  @param  bc - BLUE
  @note returns 16bit RGB565 colour
*/
/****************************************************************************/
uint16_t gfx4desp32P4::RGBto565(uint8_t rc, uint8_t gc, uint8_t bc) {
    return (((rc & 0xF8) << 8) | ((gc & 0xFC) << 3) | (bc >> 3));
}

/****************************************************************************/
/*!
  @brief  various colour conversion functions
  @note see manual
*/
/****************************************************************************/
uint32_t gfx4desp32P4::bevelColor(uint16_t colorb) {
    return HighlightColors(colorb, 18);
}

uint32_t gfx4desp32P4::HighlightColors(uint16_t colorh, int step) {
    c565toRGBs(colorh);
    RGB2HLS();
    uint8_t oldred = GFX4dESP32_RED;
    uint8_t oldgreen = GFX4dESP32_GREEN;
    uint8_t oldblue = GFX4dESP32_BLUE;
    uint8_t tl = l;
    uint8_t th = h;
    uint8_t ts = s;
    HLS2RGB(th, tl - step, ts);
    if (GFX4dESP32_RED > oldred) {
        GFX4dESP32_RED = 0;
    }
    if (GFX4dESP32_GREEN > oldgreen) {
        GFX4dESP32_GREEN = 0;
    }
    if (GFX4dESP32_BLUE > oldblue) {
        GFX4dESP32_BLUE = 0;
    }
    uint16_t _dark = RGBs2COL(GFX4dESP32_RED, GFX4dESP32_GREEN, GFX4dESP32_BLUE);
    HLS2RGB(th, tl + step, ts);
    uint16_t _light = RGBs2COL(GFX4dESP32_RED, GFX4dESP32_GREEN, GFX4dESP32_BLUE);
    uint32_t bevcol = (_dark << 16) + _light;
    return bevcol;
}

uint16_t gfx4desp32P4::ColorFromTo(uint16_t a, uint16_t b, uint8_t step){
	int rf = a >> 11;
	int rt = b >> 11;
	int gf = (a >> 5) & 0x3f;
	int gt = (b >> 5) & 0x3f;
	int bf = a & 0x1f;
	int bt = b & 0x1f;
	float rr, gr, br, res;
	rr = rt - rf;
	gr = gt - gf;
	br = bt - bf;
	rf += (int)(rr / 255.0 * step);
	gf += (int)(gr / 255.0 * step);
	bf += (int)(br / 255.0 * step);
	return (rf << 11) + (gf << 5) + bf;
}

uint16_t gfx4desp32P4::RGBs2COL(uint8_t r, uint8_t g, uint8_t b) {
    return (b >> 2) | (g & 0x7E) << 4 | (r & 0x7c) << 9;
}

void gfx4desp32P4::RGB2HLS() {
    uint8_t cMax, cMin, Rdelta, Gdelta, Bdelta, cMpM, cMmM;
    if (GFX4dESP32_RED >= GFX4dESP32_GREEN) {
        cMax = GFX4dESP32_RED;
    }
    else {
        cMax = GFX4dESP32_GREEN;
    }
    if (GFX4dESP32_BLUE >= cMax) {
        cMax = GFX4dESP32_BLUE;
    }
    if (GFX4dESP32_RED <= GFX4dESP32_GREEN) {
        cMin = GFX4dESP32_RED;
    }
    else {
        cMin = GFX4dESP32_GREEN;
    }
    if (GFX4dESP32_BLUE <= cMin) {
        cMin = GFX4dESP32_BLUE;
    }
    cMpM = cMax + cMin;
    cMmM = cMax - cMin;
    l = ((cMpM * HLSMAX) + RGBMAX) / RGBMAXm2;
    if (cMax == cMin) {
        s = 0;
        h = UNDEFINED;
    }
    else {
        if (l <= (HLSMAX / 2)) {
            s = ((cMmM * HLSMAX) + (cMpM / 2)) / cMpM;
        }
        else {
            s = ((cMmM * HLSMAX) + ((RGBMAXm2 - cMpM) / 2)) / (RGBMAXm2 - cMpM);
        }
        Rdelta = (((cMax - GFX4dESP32_RED) * HLSMAXd6) + (cMmM / 2)) / cMmM;
        Gdelta = (((cMax - GFX4dESP32_GREEN) * HLSMAXd6) + (cMmM / 2)) / cMmM;
        Bdelta = (((cMax - GFX4dESP32_BLUE) * HLSMAXd6) + (cMmM / 2)) / cMmM;
        if (GFX4dESP32_RED == cMax) {
            h = Bdelta - Gdelta;
        }
        else if (GFX4dESP32_GREEN == cMax) {
            h = HLSMAXd3 + Rdelta - Bdelta;
        }
        else {
            h = HLSMAXm2d3 + Gdelta - Rdelta;
        }
        if (h < 0) {
            h += HLSMAX;
        }
        if (h > HLSMAX) {
            h -= HLSMAX;
        }
    }
}

void gfx4desp32P4::c565toRGBs(uint16_t i565) {
    GFX4dESP32_RED = (i565 & 0xF800) >> 9;
    GFX4dESP32_GREEN = (i565 & 0x07E0) >> 4;
    GFX4dESP32_BLUE = (i565 & 0x001F) << 2;
}

void gfx4desp32P4::HLS2RGB(int H, int L, int S) {
    uint8_t M1, M2;
    if (S == 0) {
        GFX4dESP32_RED = L;
        GFX4dESP32_GREEN = L;
        GFX4dESP32_BLUE = L;
    }
    else {
        if (L <= HLSMAXd2) {
            M2 = (L * (HLSMAX + S) + HLSMAXd2) / HLSMAX;
        }
        else {
            M2 = L + S - ((L * S + HLSMAXd2) / HLSMAX);
        }
        M1 = 2 * L - M2;
        if ((H > HLSMAX) || (H < 0))
            h = 0;
        GFX4dESP32_RED = hue_RGB(H + HLSMAXd3, M1, M2);
        GFX4dESP32_GREEN = hue_RGB(H, M1, M2);
        GFX4dESP32_BLUE = hue_RGB(H - HLSMAXd3, M1, M2);
    }
}

uint8_t gfx4desp32P4::hue_RGB(int Hin, int M1, int M2) {
    uint8_t Value;
    if (Hin < 0) {
        Hin += HLSMAX;
    }
    else if (Hin > HLSMAX) {
        Hin -= HLSMAX;
    }
    if (Hin < HLSMAXd6) {
        Value = M1 + ((M2 - M1) * Hin + HLSMAXd12) / HLSMAXd6;
    }
    else if (Hin < HLSMAXd2) {
        Value = M2;
    }
    else if (Hin < HLSMAXm2d3) {
        Value = M1 + ((M2 - M1) * (HLSMAXm2d3 - Hin) + HLSMAXd12) / HLSMAXd6;
    }
    else {
        Value = M1;
    }
    return Value;
}

/****************************************************************************/
/*!
  @brief  create a simple automated button using system font
  @param  hndl - the index of the button that will be returned if pressed
  @param  x - x position of button top left corner
  @param  y - y position of button top left corner
  @param  w - width of button
  @param  h - height of button
  @param  colorb - color of the button
  @param  btext - String containing text to be displayed on button
  @param  tfont - System font number used for text
  @param  tcolor - Color of text 
*/
/****************************************************************************/
void gfx4desp32P4::Buttonx(uint8_t hndl, int16_t x, int16_t y, int16_t w,
    int16_t h, uint16_t colorb, String btext, int8_t tfont,
    uint16_t tcolor) {
    if (ButtonxInitial) {
        ButtonxInitial = false;
    }
    nl = false;
    bactive[hndl] = true;
    bposx[hndl] = x;
    bposy[hndl] = y;
    bposw[hndl] = w;
    bposh[hndl] = h;
    bposc[hndl] = colorb;
    drawButton(0, x, y, w, h, colorb, btext, tfont, 1, 1, tcolor);
}

/****************************************************************************/
/*!
  @brief  create a simple automated button using flash font
  @param  hndl - the index of the button that will be returned if pressed
  @param  x - x position of button top left corner
  @param  y - y position of button top left corner
  @param  w - width of button
  @param  h - height of button
  @param  colorb - color of the button
  @param  btext - String containing text to be displayed on button
  @param  tfont - Inherent font id used for text
  @param  tcolor - Color of text 
  @param  compressed - true if compressed flash font
*/
/****************************************************************************/
void gfx4desp32P4::Buttonx(uint8_t hndl, int16_t x, int16_t y, int16_t w,
    int16_t h, uint16_t colorb, String btext,
    const uint8_t* tfont, uint16_t tcolor,
    bool compressed) {
    if (ButtonxInitial) {
        ButtonxInitial = false;
    }
    nl = false;
    bactive[hndl] = true;
    bposx[hndl] = x;
    bposy[hndl] = y;
    bposw[hndl] = w;
    bposh[hndl] = h;
    bposc[hndl] = colorb;
    drawButton(0, x, y, w, h, colorb, btext, tfont, 1, 1, tcolor, compressed);
}

/****************************************************************************/
/*!
  @brief  create a simple automated button using gci font
  @param  hndl - the index of the button that will be returned if pressed
  @param  x - x position of button top left corner
  @param  y - y position of button top left corner
  @param  w - width of button
  @param  h - height of button
  @param  colorb - color of the button
  @param  btext - String containing text to be displayed on button
  @param  tfont - Inherent font id used for text
  @param  tcolor - Color of text 
*/
/****************************************************************************/
void gfx4desp32P4::Buttonx(uint8_t hndl, int16_t x, int16_t y, int16_t w,
    int16_t h, uint16_t colorb, String btext,
    gfx4d_font tfont, uint16_t tcolor) {
    if (ButtonxInitial) {
        ButtonxInitial = false;
    }
    nl = false;
    bactive[hndl] = true;
    bposx[hndl] = x;
    bposy[hndl] = y;
    bposw[hndl] = w;
    bposh[hndl] = h;
    bposc[hndl] = colorb;
    drawButton(0, x, y, w, h, colorb, btext, tfont, 1, 1, tcolor);
}

/****************************************************************************/
/*!
  @brief  create a simple non automated button
  @param  state - the status of the button to be drawn
  @param  x - x position of button top left corner
  @param  y - y position of button top left corner
  @param  colorb - color of the button
  @param  tcolor - Color of text 
  @param  tfont - System font number used for text
  @param  tfontsizeh - height multiple of text 
  @param  tfontsize - width multiple of text
  @param  btext - String containing text to be displayed on button
*/
/****************************************************************************/
void gfx4desp32P4::Button(uint8_t state, int16_t x, int16_t y, uint16_t colorb,
    uint16_t tcolor, int8_t tfont, int8_t tfontsizeh,
    int8_t tfontsize, String btext) {
    uint8_t sl = btext.length();
    uint8_t fsww;
    uint8_t fshh;
    if (tfont == 1) {
        fsww = 6;
        fshh = 8;
    }
    else {
        fsww = 8;
        fshh = 16;
    }
    uint16_t sw = sl * fsww * tfontsize;
    uint16_t sh = fshh * tfontsizeh;
    drawButton(state, x, y, sw + (19 * tfontsize), sh + (9 * tfontsizeh), colorb,
        btext, tfont, tfontsize, tfontsizeh, tcolor);
}

/****************************************************************************/
/*!
  @brief  create a simple non automated button using flash font
  @param  state - the status of the button to be drawn
  @param  x - x position of button top left corner
  @param  y - y position of button top left corner
  @param  colorb - color of the button
  @param  tcolor - Color of text 
  @param  tfont - Inherent font id used for text
  @param  tfontsizeh - height multiple of text 
  @param  tfontsize - width multiple of text
  @param  btext - String containing text to be displayed on button
  @param  compressed - true if compressed flash font
*/
/****************************************************************************/
void gfx4desp32P4::Button(uint8_t state, int16_t x, int16_t y, uint16_t colorb,
    uint16_t tcolor, const uint8_t* tfont,
    int8_t tfontsizeh, int8_t tfontsize, String btext,
    bool compressed) {
    __tempFont(tfont, compressed);
    uint8_t fshh = fsh;
    uint16_t sw = strWidth(btext);
    uint16_t sh = fshh * tfontsizeh;
    __restoreFont();
    drawButton(state, x, y, sw + (19 * tfontsize), sh + (9 * tfontsizeh), colorb,
        btext, tfont, tfontsize, tfontsizeh, tcolor, compressed);
}

/****************************************************************************/
/*!
  @brief  create a simple non automated button using gci font
  @param  state - the status of the button to be drawn
  @param  x - x position of button top left corner
  @param  y - y position of button top left corner
  @param  colorb - color of the button
  @param  tcolor - Color of text 
  @param  tfont - Inherent font id used for text
  @param  tfontsizeh - height multiple of text 
  @param  tfontsize - width multiple of text
  @param  btext - String containing text to be displayed on button
*/
/****************************************************************************/
void gfx4desp32P4::Button(uint8_t state, int16_t x, int16_t y, uint16_t colorb,
    uint16_t tcolor, gfx4d_font tfont, int8_t tfontsizeh,
    int8_t tfontsize, String btext) {
    __tempFont(tfont);
    uint8_t fshh = fsh;
    uint16_t sw = strWidth(btext);
    uint16_t sh = fshh * tfontsizeh;
    __restoreFont();
    drawButton(state, x, y, sw + (19 * tfontsize), sh + (9 * tfontsizeh), colorb,
        btext, tfont, tfontsize, tfontsizeh, tcolor);
}

/****************************************************************************/
/*!
  @brief  create a simple non automated slider using system font
  @param  state - the state of the background to be drawn
  @param  x - x position of slider top left corner
  @param  y - y position of slider top left corner
  @param  r - width of slider
  @param  b - height of slider 
  @param  colorb - background color of slider
  @param  colort - knob color of slider
  @param  scale - range of slider 
  @param  value - value of slider 0 to range
*/
/****************************************************************************/
void gfx4desp32P4::Slider(uint8_t state, int16_t x, int16_t y, int16_t r,
    int16_t b, uint16_t colorb, uint16_t colort,
    int16_t scale, int16_t value) {
    int w = r - x;
    int h = b - y;
    drawButton(state, x, y, w, h, colorb, "", 1, 1, 1, colorb);
    uint16_t thw;
    uint16_t thh = h - 4;
    uint16_t ra = w - 4;
    thw = ra / 10;
    if (thw < 5)
        thw = 5;
    float rs = (((float)ra - (float)thw) / (float)scale) * (float)value;
    int rsc = (int)rs;
    RectangleFilled(x + 2 + rsc, y + 2, x + 2 + rsc + thw - 1, y + 2 + thh - 1,
        colort);
    Hline(x + 2 + rsc, y + 2, thw, tlight);
    Vline(x + 2 + rsc, y + 2, thh, tlight);
    Hline(x + 2 + rsc, y + 1 + thh, thw, tdark);
    Vline(x + 2 + rsc + thw, y + 2, thh, tdark);
}

/****************************************************************************/
/*!
  @brief  Helper function for simple buttons
*/
/****************************************************************************/
void gfx4desp32P4::drawButton(uint8_t updn, int16_t x, int16_t y, int16_t w,
    int16_t h, uint16_t colorb, String btext,
    const uint8_t* tfont, int8_t tfontsize,
    int8_t tfontsizeht, uint16_t tcolor,
    bool compressed) {
	StoreCursPos();
    boolean twrap = wrap;
    //boolean nlbckp = nl;
    //nl = false;
    wrap = false;
    int8_t tfh;
    uint8_t fsizebckup = textsize;
    uint8_t fsizehbckup = textsizeht;
    textsize = tfontsize;
    textsizeht = tfontsizeht;
    uint16_t tcolorbckup = textcolor;
    uint16_t tcolorbgbckup = textbgcolor;
    uint16_t curxbckup = cursor_x;
    uint16_t curybckup = cursor_y;
    uint32_t tcol = bevelColor(colorb);
    uint16_t _dark = tcol >> 16;
    uint16_t _light = tcol & 0xffff;
    tdark = _dark;
    tlight = _light;
    if (bxStyle == 0) {
        RectangleFilled(x + 2, y + 2, (x + 2) + w - 4, (y + 2) + h - 4, colorb);
        if (updn == 0) {
            Hline(x, y, w, _light);
            Hline(x + 1, y + 1, w - 2, _light);
            Vline(x, y, h, _light);
            Vline(x + 1, y + 1, h - 2, _light);
            Hline(x, y + h - 1, w, _dark);
            Hline(x + 1, y + h - 2, w - 2, _dark);
            Vline(x + w - 1, y, h, _dark);
            Vline(x + w - 2, y + 1, h - 2, _dark);
        }
        if (updn == 1) {
            Hline(x, y, w, _dark);
            Hline(x + 1, y + 1, w - 2, _dark);
            Vline(x, y, h, _dark);
            Vline(x + 1, y + 1, h - 2, _dark);
            Hline(x, y + h - 1, w, _light);
            Hline(x + 1, y + h - 2, w - 2, _light);
            Vline(x + w - 1, y, h, _light);
            Vline(x + w - 2, y + 1, h - 2, _light);
        }
    }
    if (bxStyle > 0) {
        int nh, g1, g2, nw;
        nw = h / 10;
        if (nw == 0)
            nw = 1;
        if (bxStyle == 1) {
            nh = h / 6;
            g1 = -1;
            g2 = 0;
        }
        if (bxStyle > 1) {
            nh = h >> 1;
            g1 = 28;
            g2 = 0;
        }
        if (bxStyle == 3) {
            g1 = 48;
            g2 = 28;
        }
        if (updn == 0)
            gradientShape(0, nw, x, y, w - 1, h - 1, nh, nh, nh, nh, 0, _dark,
                GRADIENT_RAISED, g1, colorb, GRADIENT_RAISED, g2, 0);
        if (updn == 1)
            gradientShape(0, nw, x, y, w - 1, h - 1, nh, nh, nh, nh, 0, _dark,
                GRADIENT_RAISED, 0, colorb, GRADIENT_RAISED, g2, 0);
    }
    TextColor(tcolor, tcolor);
    __tempFont(tfont, compressed);
    tfh = fsh;
    uint8_t blen = btext.length();
    size_t strw = strWidth(btext);
    if (blen > 0) {
        MoveTo(((x + (w / 2) - ((strw * textsize) / 2)) + updn),
            ((y + (h / 2) - ((tfh * textsizeht) / 2)) + 1 + updn));
        print(btext);
    }
    TextColor(tcolorbckup, tcolorbgbckup);
    __restoreFont();
    //textsize = fsizebckup;
    //textsizeht = fsizehbckup;
    //MoveTo(curxbckup, curybckup);
    //nl = nlbckp;
	RestoreCursPos();
    wrap = twrap;
}

/****************************************************************************/
/*!
  @brief  Helper function for simple buttons
*/
/****************************************************************************/
void gfx4desp32P4::drawButton(uint8_t updn, int16_t x, int16_t y, int16_t w,
    int16_t h, uint16_t colorb, String btext,
    gfx4d_font tfont, int8_t tfontsize,
    int8_t tfontsizeht, uint16_t tcolor) {
    boolean twrap = wrap;
    boolean nlbckp = nl;
    nl = false;
    wrap = false;
    int8_t tfh;
    uint8_t fsizebckup = textsize;
    uint8_t fsizehbckup = textsizeht;
    textsize = tfontsize;
    textsizeht = tfontsizeht;
    uint16_t tcolorbckup = textcolor;
    uint16_t tcolorbgbckup = textbgcolor;
    uint16_t curxbckup = cursor_x;
    uint16_t curybckup = cursor_y;
    uint32_t tcol = bevelColor(colorb);
    uint16_t _dark = tcol >> 16;
    uint16_t _light = tcol & 0xffff;
    tdark = _dark;
    tlight = _light;
    if (bxStyle == 0) {
        RectangleFilled(x + 2, y + 2, (x + 2) + w - 4, (y + 2) + h - 4, colorb);
        if (updn == 0) {
            Hline(x, y, w, _light);
            Hline(x + 1, y + 1, w - 2, _light);
            Vline(x, y, h, _light);
            Vline(x + 1, y + 1, h - 2, _light);
            Hline(x, y + h - 1, w, _dark);
            Hline(x + 1, y + h - 2, w - 2, _dark);
            Vline(x + w - 1, y, h, _dark);
            Vline(x + w - 2, y + 1, h - 2, _dark);
        }
        if (updn == 1) {
            Hline(x, y, w, _dark);
            Hline(x + 1, y + 1, w - 2, _dark);
            Vline(x, y, h, _dark);
            Vline(x + 1, y + 1, h - 2, _dark);
            Hline(x, y + h - 1, w, _light);
            Hline(x + 1, y + h - 2, w - 2, _light);
            Vline(x + w - 1, y, h, _light);
            Vline(x + w - 2, y + 1, h - 2, _light);
        }
    }
    if (bxStyle > 0) {
        int nh, g1, g2, nw;
        nw = h / 10;
        if (nw == 0)
            nw = 1;
        if (bxStyle == 1) {
            nh = h / 6;
            g1 = -1;
            g2 = 0;
        }
        if (bxStyle > 1) {
            nh = h >> 1;
            g1 = 28;
            g2 = 0;
        }
        if (bxStyle == 3) {
            g1 = 48;
            g2 = 28;
        }
        if (updn == 0)
            gradientShape(0, nw, x, y, w - 1, h - 1, nh, nh, nh, nh, 0, _dark,
                GRADIENT_RAISED, g1, colorb, GRADIENT_RAISED, g2, 0);
        if (updn == 1)
            gradientShape(0, nw, x, y, w - 1, h - 1, nh, nh, nh, nh, 0, _dark,
                GRADIENT_RAISED, 0, colorb, GRADIENT_RAISED, g2, 0);
    }
    TextColor(tcolor, tcolor);
    __tempFont(tfont);
    tfh = fsh;
    uint8_t blen = btext.length();
    size_t strw = strWidth(btext);
    if (blen > 0) {
        MoveTo(((x + (w / 2) - ((strw * textsize) / 2)) + updn),
            ((y + (h / 2) - ((tfh * textsizeht) / 2)) + 1 + updn));
        print(btext);
    }
    TextColor(tcolorbckup, tcolorbgbckup);
    __restoreFont();
    textsize = fsizebckup;
    textsizeht = fsizehbckup;
    MoveTo(curxbckup, curybckup);
    nl = nlbckp;
    wrap = twrap;
}

/****************************************************************************/
/*!
  @brief  Helper function for simple buttons
*/
/****************************************************************************/
void gfx4desp32P4::drawButton(uint8_t updn, int16_t x, int16_t y, int16_t w,
    int16_t h, uint16_t colorb, String btext,
    int8_t tfont, int8_t tfontsize, int8_t tfontsizeht,
    uint16_t tcolor) {
    boolean twrap = wrap;
    boolean nlbckp = nl;
    nl = false;
    wrap = false;
    int8_t tfw;
    int8_t tfh;
    if (tfont < 2) {
        tfw = 6;
        tfh = 8;
    }
    else {
        tfw = 9;
        tfh = 16;
    }
    uint8_t fsizebckup = textsize;
    uint8_t fsizehbckup = textsizeht;
    textsize = tfontsize;
    textsizeht = tfontsizeht;
    uint16_t tcolorbckup = textcolor;
    uint16_t tcolorbgbckup = textbgcolor;
    uint16_t curxbckup = cursor_x;
    uint16_t curybckup = cursor_y;
    uint32_t tcol = bevelColor(colorb);
    uint16_t _dark = tcol >> 16;
    uint16_t _light = tcol & 0xffff;
    tdark = _dark;
    tlight = _light;
    if (bxStyle == 0) {
        RectangleFilled(x + 2, y + 2, (x + 2) + w - 4, (y + 2) + h - 4, colorb);
        if (updn == 0) {
            Hline(x, y, w, _light);
            Hline(x + 1, y + 1, w - 2, _light);
            Vline(x, y, h, _light);
            Vline(x + 1, y + 1, h - 2, _light);
            Hline(x, y + h - 1, w, _dark);
            Hline(x + 1, y + h - 2, w - 2, _dark);
            Vline(x + w - 1, y, h, _dark);
            Vline(x + w - 2, y + 1, h - 2, _dark);
        }
        if (updn == 1) {
            Hline(x, y, w, _dark);
            Hline(x + 1, y + 1, w - 2, _dark);
            Vline(x, y, h, _dark);
            Vline(x + 1, y + 1, h - 2, _dark);
            Hline(x, y + h - 1, w, _light);
            Hline(x + 1, y + h - 2, w - 2, _light);
            Vline(x + w - 1, y, h, _light);
            Vline(x + w - 2, y + 1, h - 2, _light);
        }
    }
    if (bxStyle > 0) {
        int nh;
        int g1 = 0; int g2 = 0;
        int nw = h / 10;
        if (nw == 0)
            nw = 1;
        if (bxStyle == 1) {
            nh = h / 6;
            g1 = -1;
            g2 = 0;
        }
        if (bxStyle > 1) {
            nh = h >> 1;
            g1 = 28;
            g2 = 0;
        }
        if (bxStyle == 3) {
            g1 = 48;
            g2 = 28;
        }
        if (updn == 0)
            gradientShape(0, nw, x, y, w - 1, h - 1, nh, nh, nh, nh, 0, _dark,
                GRADIENT_RAISED, g1, colorb, GRADIENT_RAISED, g2, 0);
        if (updn == 1)
            gradientShape(0, nw, x, y, w - 1, h - 1, nh, nh, nh, nh, 0, _dark,
                GRADIENT_RAISED, 0, colorb, GRADIENT_RAISED, g2, 0);
    }
    TextColor(tcolor, tcolor);
    __tempFont(tfont);
    uint8_t blen = btext.length();
    if (blen > 0) {
        MoveTo(((x + (w / 2) - ((blen * tfw * textsize) / 2)) + updn),
            ((y + (h / 2) - ((tfh * textsizeht) / 2)) + 1 + updn));
        print(btext);
    }
    TextColor(tcolorbckup, tcolorbgbckup);
    __restoreFont();
    textsize = fsizebckup;
    textsizeht = fsizehbckup;
    MoveTo(curxbckup, curybckup);
    nl = nlbckp;
    wrap = twrap;
}

/****************************************************************************/
/*!
  @brief  Set a style for ButtonX buttons
  @param  bs - number of the style
*/
/****************************************************************************/
void gfx4desp32P4::ButtonXstyle(byte bs) { bxStyle = bs; }

/****************************************************************************/
/*!
  @brief  Enable of disable ButtonX button
  @param  butno - number of the button to set
  @param  act - true enabled, false disabled
*/
/****************************************************************************/
void gfx4desp32P4::ButtonActive(uint8_t butno, boolean act) {
    bactive[butno] = act;
}

/****************************************************************************/
/*!
  @brief  Delete buttonX and replace with button color
  @param  hndl - number of the button to set
*/
/****************************************************************************/
void gfx4desp32P4::DeleteButton(int hndl) {
    if (hndl > 0) {
        RectangleFilled(bposx[hndl], bposy[hndl], bposx[hndl] + bposw[hndl] - 1,
            bposy[hndl] + bposh[hndl] - 1, bposc[hndl]);
        bactive[hndl] = false;
    }
    else {
        for (int n = 0; n < 128; n++) {
            RectangleFilled(bposx[n], bposy[n], bposx[n] + bposw[n] - 1,
                bposy[n] + bposh[n] - 1, bposc[n]);
            bactive[n] = false;
        }
    }
}

/****************************************************************************/
/*!
  @brief  Delete ButtonX and replace with chosen color
  @param  hndl - number of the button to set
  @param  color - RGB565 color of filled rectangle to replace button
*/
/****************************************************************************/
void gfx4desp32P4::DeleteButton(int hndl, uint16_t color) {
    if (hndl > 0) {
        RectangleFilled(bposx[hndl], bposy[hndl], bposx[hndl] + bposw[hndl] - 1,
            bposy[hndl] + bposh[hndl] - 1, color);
        bactive[hndl] = false;
    }
    else {
        for (int n = 0; n < 128; n++) {
            RectangleFilled(bposx[n], bposy[n], bposx[n] + bposw[n] - 1,
                bposy[n] + bposh[n] - 1, color);
            bactive[n] = false;
        }
    }
}

/****************************************************************************/
/*!
  @brief  Delete ButtonX and replace with area of chosen image
  @param  hndl - number of the button to set
  @param  objBG - Userimage index to replace button
*/
/****************************************************************************/
void gfx4desp32P4::DeleteButtonBG(int hndl, int objBG) {
    if (hndl > 0) {
        UserImageDR(objBG, bposx[hndl], bposy[hndl], bposw[hndl], bposh[hndl],
            bposx[hndl], bposy[hndl]);
        bactive[hndl] = false;
    }
    else {
        for (int n = 0; n < 128; n++) {
            UserImageDR(objBG, bposx[n], bposy[n], bposw[n], bposh[n], bposx[n],
                bposy[n]);
            bactive[n] = false;
        }
    }
}

void gfx4desp32P4::gradientShapeAA(int vert, int ow, int xpos, int ypos, int w,
    int h, int r1, int r2, int r3, int r4,
    int darken, int32_t color, int sr1, int gl1,
    int32_t colorD, int sr3, int gl3, int gtb) {
    int32_t x[4];
    int32_t y[4];
	int radsGS[4];
	int mw, mh;
	int32_t pc1, pc2;
	int opc;
	bool gapL = false; 
	bool gapR = false;
	radsGS[0] = r1 + !(r1);
    radsGS[1] = r2 + !(r2);
    radsGS[2] = r3 + !(r3);
    radsGS[3] = r4 + !(r4);
    mh = max(radsGS[0] + radsGS[2], radsGS[1] + radsGS[3]);
    mw = max(radsGS[0] + radsGS[1], radsGS[2] + radsGS[3]);
    if (mh > h - 1)
        h = mh - 1;
    if (mw > w - 1)
        w = mw - 1;
	//if((radsGS[0] + radsGS[2]) < h + (h / 10)) gapL = true; 
   //if((radsGS[1] + radsGS[3]) < h + (h / 10)) gapR = true;	
    x[0] = xpos + r1; y[0] = ypos + r1;
    x[1] = xpos + w - r2 - 1; y[1] = ypos + r2;
    x[2] = xpos + r3; y[2] = ypos + h - 1 - r3;
    x[3] = xpos + w - r4 - 1; y[3] = ypos + h - 1 - r4;
    //(int GraisSunk, int Gstate, int Gglev, int Gh1, int Gpos, uint16_t colToAdj)
    grad1[0] = sr1; grad1[1] = darken; grad1[2] = gl1; grad1[3] = h; grad1[6] = ypos;
    grad1[7] = sr3; grad1[8] = darken; grad1[9] = gl3; grad1[10] = h - (ow << 1); grad1[13] = ypos + ow;
    bool outer, inner;
    outer = (sr1 != -1 && gl1 != -1);
    inner = (sr3 != -1 && gl3 != -1);
    //StartWrite();
    bool needsEndWrite = StartWrite();
	if (outer){
        gradON = 1;
        
		if(gapL){
		    //pc1 = radGS[0] * 100 / h; pc2 = radGS[2] * 100 / h;
			//opc = 100 - pc1 + pc2 / 2
			//pc1 += opc; pc2 += opc;
			grad1[0] = sr1; grad1[1] = darken; grad1[2] = gl1; grad1[3] = h / 2; grad1[6] = ypos;
			drawArc(x[0], y[0], r1, r1 - ow, 90, 180, color);//, 0, true);
		    drawArc(x[2], y[2], r3, r3 - ow, 0, 90, color);//, 0, true);
		} else {
		    grad1[0] = sr1; grad1[1] = darken; grad1[2] = gl1; grad1[3] = h; grad1[6] = ypos;
            grad1[7] = sr3; grad1[8] = darken; grad1[9] = gl3; grad1[10] = h - (ow << 1); grad1[13] = ypos + ow;
			drawArc(x[0], y[0], r1, r1 - ow, 90, 180, color);//, 0, true);
		    drawArc(x[2], y[2], r3, r3 - ow, 0, 90, color);//, 0, true);
		}
        if(gapR){
			grad1[0] = sr1; grad1[1] = darken; grad1[2] = gl1; grad1[3] = h / 2; grad1[6] = ypos;
			drawArc(x[1], y[1], r2, r2 - ow, 180, 270, color);//, 0, true);
			drawArc(x[3], y[3], r4, r4 - ow, 270, 360, color);//, 0, true);
		} else {
		    grad1[0] = sr1; grad1[1] = darken; grad1[2] = gl1; grad1[3] = h; grad1[6] = ypos;
            grad1[7] = sr3; grad1[8] = darken; grad1[9] = gl3; grad1[10] = h - (ow << 1); grad1[13] = ypos + ow;
			drawArc(x[1], y[1], r2, r2 - ow, 180, 270, color);//, 0, true);
            drawArc(x[3], y[3], r4, r4 - ow, 270, 360, color);//, 0, true);
		}
		
		//ArcAA(x[0], y[0], r1, r1 - ow, 90, 180, color, true);
		//drawArc(x[0], y[0], r1, r1 - ow, 90, 180, color, 0, true);
		//drawArc(x[1], y[1], r2, r2 - ow, 180, 270, color, 0, true);
		//drawArc(x[2], y[2], r3, r3 - ow, 0, 90, color, 0, true);
		//drawArc(x[3], y[3], r4, r4 - ow, 270, 360, color, 0, true);
    }
    if (inner){
        gradON = 2;
        drawArc(x[0], y[0], r1 - ow, 1, 90, 180, colorD);//, 0, true);
        PutPixelAlpha(x[0], y[0], colorD, 255);
        drawArc(x[1], y[1], r2 - ow, 1, 180, 270, colorD);//, 0, true);
        PutPixelAlpha(x[1], y[1], colorD, 255);
        drawArc(x[2], y[2], r3 - ow, 1, 0, 90, colorD);//, 0, true);
        PutPixelAlpha(x[2], y[2], colorD, 255);
        drawArc(x[3], y[3], r4 - ow, 1, 270, 360, colorD);//, 0, true);
        PutPixelAlpha(x[3], y[3], colorD, 255);
    }
    int l1, w1, l2, w2, l3, w3;

    for (int n = 0; n < h; n++){
       if (n < ow && outer){
         gradON = 1;
         HlineX(x[0] + 1, ypos + n, w - r1 - r2 - 1, color);
       }
       else if (n > h - ow - 1 && outer){
         gradON = 1;
         HlineX(x[2] + 1, ypos + n, w - r3 - r4 - 1, color);
       } else {
         w1 = 0; w2 = w - (ow << 1); w3 = 0; l2 = ow;
         if (n > r1 && n < h - r3 - 1) {
             w1 = ow;
         }
         if (n < r1){
            l2 = r1 - 1;
            w2 -= (r1 - ow - 1);
         }
         if (n < r2) w2 -= (r2 - ow - 1);

         if (n > r2 && n < h - r4 - 1) {
             w3 = ow;
         }
         if (n > h - r3){
            l2 = r3 - 1;
            w2 -= (r3 - ow - 1);
         }
         if (n > h - r4) w2 -= (r4 - ow - 1);
         if (outer){
             gradON = 1;
             if (w1 != 0) HlineX(xpos, ypos + n, ow, color);
             if (w3 != 0) HlineX(xpos + w - ow, ypos + n, ow, color);
         }
         if (inner) {
             gradON = 2;
             HlineX(xpos + l2, ypos + n, w2, colorD);
         }
       }
       //if (w2 == w) w2 -= (ow << 1);


    }
    if(needsEndWrite) EndWrite();
    gradON = 0;
}

void gfx4desp32P4::gradientShape(int vert, int ow, int xPos, int yPos, int w,
    int h, int r1, int r2, int r3, int r4,
    int darken, uint16_t color, int sr1, int gl1,
    uint16_t colorD, int sr3, int gl3, int gtb) {

    int arcn = 0; int xc1 = 0; int xc2 = 0; int xc3 = 0; int xc4 = 0; int yc1 = 0; int mi = 0;
    int cDraw = 0; int xgxl = 0; int lastxc1 = 0; int xgxr = 0; int tcount = 0; int tgss1 = 0;
    int tgss2 = 0; int drgs = 0; int yag = 0; int lastarc = 0; int lachk = 0;
    int c = 0; int xx = 0; int yy = 0; int x = 0; int y = 0; int c1 = 0; int xx1 = 0;
    int yy1 = 0; int xin = 0; int y1 = 0; int lasty1 = 0; int sCxPos = 0; int owMin = 0;
    int owMax = 0; int mirror = 0; int dbl1 = 0; int dbl2 = 0; int gradofs = 0; int hm = 0;
    int hmd = 0; int gof = 0; int owSet = 0; int rtSide = 0; int lasty = 0; int incy = 0;
    int incy1 = 0; int maxR = 0; int mh = 0; int mw = 0; int xa = 0; int xb = 0; int xc = 0;
    int ya = 0; int radsMinOw = 0; int txPos = 0; int tyPos = 0; int cErase = 0;
    int tow = 0; int tsr1 = 0; int tsr3 = 0; int tgl3 = 0; int tdarken = 0; int tgl1 = 0;
    int fill = 0; int yb = 0; int Lfill = 0; int Rfill = 0; int Ldw = 0; int Rdw = 0; int Lw = 0;
    int Rw = 0; int mfill = 0; int start = 0; int fin = 0; int fCmp = 0; int tFillL = 0;
    int tFillR = 0; int cropfill = 0;
    uint16_t tcolor = 0;
    uint16_t tcolorD = 0;
    uint16_t tgrad = 0;
    if (vert) {
        gfx_Swap(xPos, yPos);
        gfx_Swap(h, w);
        gfx_Swap(r2, r3);
    }
    int oddf = h % 2;
    radsGS[0] = r1 + !(r1);
    radsGS[1] = r2 + !(r2);
    radsGS[2] = r3 + !(r3);
    radsGS[3] = r4 + !(r4);
    mh = max(radsGS[0] + radsGS[2], radsGS[1] + radsGS[3]);
    mw = max(radsGS[0] + radsGS[1], radsGS[2] + radsGS[3]);
    if (mh > h - 1)
        h = mh - 1;
    if (mw > w - 1)
        w = mw - 1;
    maxR = max(max(radsGS[0], radsGS[1]), max(radsGS[2], radsGS[3]));
    if (maxR > MAX_ARCSIZE)
        return;
    mirror = ((radsGS[0] == radsGS[2]) && (radsGS[1] == radsGS[3]));
    dbl1 = (radsGS[0] == radsGS[1]);
    dbl2 = (radsGS[2] == radsGS[3]);
    xpGSaPos[0] = radsGS[0] - 1;
    ypGSaPos[0] = radsGS[0] - 1;
    xpGSaPos[1] = w - radsGS[1];
    ypGSaPos[1] = radsGS[1] - 1;
    xpGSaPos[2] = radsGS[2] - 1;
    ypGSaPos[2] = h - radsGS[2];
    xpGSaPos[3] = w - radsGS[3];
    ypGSaPos[3] = h - radsGS[3];
    owMin = yPos + ow;
    owMax = yPos + h - ow - 1;
    if (!(lastAsize))
        lastAsize = maxR;
    if (!(keepLastArc))
        memcpy(inx, lastArcOld, lastAsize << 1);
    arcn = 0;
    if (oddf && (GSCropArcLeft > -1 || GSCropArcRight > -1))
        cropfill = 1;
    while (arcn < 4) {
        arcn += (radsGS[arcn] < 1);
        if (arcn == 1 && dbl1 && mirror)
            break;
        arcn += (arcn == 1 && dbl1);
        if (arcn == 2 && dbl1 == 0 && mirror)
            break;
        if (arcn == 3 && dbl2)
            break;
        if (arcn && !(lachk))
            memset(inx, 0, radsGS[arcn] << 1);
        incy1 = radsGS[arcn] - ow;
        incy = radsGS[arcn];
        if (radsGS[arcn]) {
            radsMinOw = radsGS[arcn] - ow;
            c = 1 - radsGS[arcn];
            c1 = 1 - radsMinOw;
            xx = 1;
            xx1 = 1;
            yy = -2 * radsGS[arcn];
            yy1 = -2 * radsMinOw;
            x = 0;
            xin = 0;
            y = radsGS[arcn];
            y1 = radsMinOw;
            cErase = 0;
            while (cErase < 1 + GSErase) { // If Erase flag is set do twice, first to
                // erase last and immediately draw new
                if (GSErase) {               // ** function for rotaries **
                    if (!(cErase)) {
                        txPos = xPos;
                        tyPos = yPos;
                        xPos = GSEraseXpos;
                        yPos = GSEraseYpos;
                        tow = ow;
                        ow = 0;
                        tcolor = color;
                        color = GSEraseColour;
                        tcolorD = colorD;
                        colorD = GSEraseHeight;
                        tsr1 = sr1;
                        sr1 = GSERaisedSunk;
                        tgl1 = gl1;
                        gl1 = GSEraseGLevel;
                        tsr3 = sr3;
                        sr3 = -2;
                        tgl3 = gl3;
                        gl3 = GSErasePHeight;
                        tdarken = darken;
                        darken = 0;
                    }
                    else {
                        xPos = txPos;
                        yPos = tyPos;
                        ow = tow;
                        colorD = tcolorD;
                        sr3 = tsr3;
                        gl3 = tgl3;
                        sr1 = tsr1;
                        gl1 = tgl1;
                        color = tcolor;
                        darken = tdarken;
                    }
                }
                // ***************************************
                cDraw = 0;
                tcount = radsGS[arcn];
                if ((GSCropArcLeft > -1 || GSCropArcRight > -1) && GSSsxpos != -9999)
                    tcount = lastAsize; // For Slider
                lachk = (lastarc != radsGS[arcn]);
                while (cDraw < tcount) {
                    if (x < y && lachk) {
                        if (xin < y1) {
                            if (c1 >= 0) {
                                y1--;
                                yy1 += 2;
                                c1 += yy1;
                            }
                            inx[xin] = (inx[xin] & 0xFF00) | y1;
                            if (lasty1 != y1) {
                                inx[incy1] = (inx[incy1] & 0xFF00) | xin;
                                incy1--;
                            }
                            xin++;
                            xx1 += 2;
                            c1 += xx1;
                            lasty1 = y1;
                        }
                        if (c >= 0) {
                            y--;
                            yy += 2;
                            c += yy;
                        }
                        inx[x] = (inx[x] & 0x00FF) | ((y - 1) << 8);
                        if (lasty != y) {
                            inx[incy] = (inx[incy] & 0x00FF) | ((x - 1) << 8);
                            incy--;
                        }
                        x++;
                        xx += 2;
                        c += xx;
                        lasty = y;
                    }
                    xc3 = xPos + xpGSaPos[arcn];

                    xc4 = xc3;
                    xa = inx[cDraw] & 0x00FF;
                    xb = inx[cDraw] >> 8;
                    xc = lastArcOld[cDraw] >> 8; // Get outer and inner arc position
                    // ** Fixes error if old value isn't overwritten in the array **
                    if (cDraw && xa > lastxc1)
                        xa = lastxc1;
                    if (xb > radsGS[arcn])
                        xb = 0;
                    if (xa > radsGS[arcn])
                        xa = 0;
                    lastxc1 = xa;
                    xc2 = 0;

                    //  *************************************************************
                    if (arcn == 0) { // Calculate first arc
                        sCxPos = xPos + xpGSaPos[0];
                        yc1 = yPos + ypGSaPos[0] - cDraw;
                        xc1 = sCxPos - xb;
                        xc2 = sCxPos - xa;

                        if (GSCropArcLeft > -1) { // Modify if crop value set, only works
                            // for doubled and mirrored shapes
                            xc1 = xPos + GSCropArcLeft + xc;
                            if (cDraw >= lastAsize)
                                xc1 = xPos + GSCropArcLeft;
                            xc2 = xc1;
                        }
                    }
                    if (arcn ==
                        1 - dbl1) { // Calculate second arc or modify first if a double
                        sCxPos = xPos + xpGSaPos[1] - vert;
                        if (dbl1) {
                            xc3 = sCxPos + xa;
                            xc4 = sCxPos + xb;
                            if (GSCropArcLeft > 0 && xc2 > xc3)
                                xc3 = xc2;
                        }
                        else {
                            yc1 = yPos + ypGSaPos[1] - cDraw;
                            xc2 = sCxPos + xb;
                            xc1 = sCxPos + xa;
                        }
                    }
                    if (!(mirror)) {   // If mirrored, ignore calcs for next arcs
                        if (arcn == 2) { // Calculate third arc
                            sCxPos = xPos + xpGSaPos[2];
                            yc1 = yPos + ypGSaPos[2] + cDraw;
                            xc1 = sCxPos - xb;
                            xc2 = sCxPos - xa;
                        }
                        if (arcn ==
                            (3 - dbl2)) { // Calculate third arc or modify third if double
                            sCxPos = xPos + xpGSaPos[3] - vert;
                            if (dbl2) {
                                xc3 = sCxPos + xa;
                                xc4 = sCxPos + xb;
                            }
                            else {
                                yc1 = yPos + ypGSaPos[3] + cDraw;
                                xc2 = sCxPos + xb;
                                xc1 = sCxPos + xa;
                            }
                        }
                    }
                    if (GSCropArcRight > -1) { // Modify if crop value set, only works for
                        // doubled and mirrored shapes
                        xc3 = xPos + GSCropArcRight - xc;
                        if (cDraw >= lastAsize)
                            xc3 = xPos + GSCropArcRight + 1;
                        xc4 = xc3;
                        if (xc3 < xc2)
                            xc2 = xc3;
                    }
                    if (sr3 ==
                        -2) { // Check for erase flag and use new parameters for gradient
                        hm = colorD;
                        gradofs = gl3;
                    }
                    else {
                        hm = h;
                        gradofs = 0;
                    }
                    if (gtb) {
                        hmd = h << 1;
                        if (gtb == 2)
                            gof = h;
                        gradofs = h >> 1;
                        hm += gradofs;
                    }
                    else {
                        hmd = h;
                    }
                    mi = 0;
                    while (mi < 1 + mirror + cropfill) { // Do twice if mirrored

                        if (cropfill)
                            oddf = 0;
                        if (mi >= 1 && mi <= 2)
                            yc1 = yPos + (h - 1 - (yc1 - yPos)) + (mi == 2);
                        ya = yc1 - yPos;
                        owSet = (yc1 >= owMin && yc1 <= owMax);
                        rtSide = (arcn == 1 || arcn == 3);
                        tgss1 = GSSsxpos + GSSLastSliderVal + xc + 1;
                        tgss2 = GSSsxpos + GSSLastSliderVal - xc - 1;
                        yag = ya + gradofs;
                        if (cDraw < radsGS[arcn]) {
                            tgrad = Grad(sr1, darken, gl1, hm, yag, color);
                            if (ow) {
                                if (sr3 > -1 &&
                                    ((owSet && xc2 < xc3) || (rtSide && owSet && xc2 > xc3)))
                                    (vert == 1)
                                    ? VlineD(yc1, xc2, xc3,
                                        Grad(sr3, darken, gl3, hmd, ya + gof, colorD))
                                    : HlineD(yc1, xc2, xc3,
                                        Grad(sr3, darken, gl3, hmd, ya + gof, colorD));
                                if (xc1 < xc2)
                                    (vert == 1) ? VlineD(yc1, xc1, xc2, tgrad)
                                    : HlineD(yc1, xc1, xc2, tgrad);
                                if (!(owSet)) {
                                    if ((xc1 < xc4) || (rtSide && xc1 > xc4))
                                        (vert == 1) ? VlineD(yc1, xc1, xc4, tgrad)
                                        : HlineD(yc1, xc1, xc4, tgrad);
                                }
                                else {
                                    ///// ***** Added 9/12/2019 -- Condition for Slider alider
                                    ///animation to remove flicker *****
                                    if (!(GSSsxpos != -9999))
                                        (vert == 1) ? VlineD(yc1, xc1, xc2, tgrad)
                                        : HlineD(yc1, xc1, xc2, tgrad);
                                    if (xc3 < xc4)
                                        (vert == 1) ? VlineD(yc1, xc3, xc4, tgrad)
                                        : HlineD(yc1, xc3, xc4, tgrad);
                                }
                            }
                            else {
                                if ((xc1 < xc4) || (rtSide && xc4 < xc1))
                                    (vert == 1) ? VlineD(yc1, xc1, xc4, tgrad)
                                    : HlineD(yc1, xc1, xc4, tgrad);
                            }
                            if (GSSsxpos != -9999) {
                                drgs = 0;
                                if (GSCropArcRight > -1) {
                                    xgxl = tgss2;
                                    if (xc1 > xc4) {
                                        xgxr = xc4;
                                        drgs = 1;
                                    }
                                    else {
                                        if (xgxl < xc1) {
                                            xgxr = xc1 - 1;
                                            drgs = 1;
                                        }
                                    }
                                }
                                if (GSCropArcLeft > -1) {
                                    xgxl = tgss1;
                                    if (xc1 > xc4) {
                                        xgxr = xc1;
                                        drgs = 1;
                                    }
                                    else {
                                        if (xgxl > xc4) {
                                            xgxr = xc4 + 1;
                                            drgs = 1;
                                        }
                                    }
                                }

                                if (drgs)
                                    (vert == 1) ? VlineD(yc1, xgxl, xgxr, GSSBGColor)
                                    : HlineD(yc1, xgxl, xgxr, GSSBGColor);
                            }
                        }
                        else {
                            if (GSSsxpos != -9999) {
                                if (GSCropArcLeft > -1) {
                                    xgxl = GSSsxpos + xc + GSCropArcLeft - 1;
                                    xgxr = tgss1;
                                }
                                if (GSCropArcRight > -1) {
                                    xgxl = tgss2;
                                    xgxr = GSSsxpos + GSCropArcRight - xc;
                                }

                                (vert == 1) ? VlineD(yc1, xgxl, xgxr, GSSBGColor)
                                    : HlineD(yc1, xgxl, xgxr, GSSBGColor);
                            }
                        }
                        mi++;
                    }
                    cDraw++;
                }
                lastarc = radsGS[arcn];

                cErase++;
            }
        }
        arcn++;
    }
    // ** Fill space not occupied by arcs **
    if (!((radsGS[0] == radsGS[1] && radsGS[2] == radsGS[3]) &&
        (radsGS[0] == radsGS[2] && radsGS[0] + radsGS[2] >= h - 2)) ||
        (oddf && !(cropfill))) { // check to see if fill is needed
        fin = h;
        start = 0;
        if (radsGS[0] == radsGS[1])
            start = radsGS[0];
        if (radsGS[2] == radsGS[3])
            fin = h - radsGS[2];
        fill = start;
        while (fill < fin) {
            yb = yPos + fill;
            Lfill = xPos;
            Rfill = xPos + w - 1;
            Ldw = Lfill + ow;
            Rdw = Rfill - ow;
            if (fill <= ypGSaPos[0])
                Lfill = (xPos + xpGSaPos[0]) * (!dbl1);
            if (fill <= ypGSaPos[1])
                Rfill = (xPos + xpGSaPos[1]) * (!dbl1);
            if (fill >= ypGSaPos[2])
                Lfill = (xPos + xpGSaPos[2]) * (!dbl2);
            if (fill >= ypGSaPos[3])
                Rfill = (xPos + xpGSaPos[3]) * (!dbl2);
            Lw = ow - 1;
            Rw = ow - 1;
            if (Lfill > Ldw)
                Lw = 0;
            if (Rfill < Rdw)
                Rw = 0;
            mfill = fill + gradofs;
            fCmp = (Lfill < Rfill);
            tgrad = Grad(sr1, darken, gl1, hm, mfill, color);
            tFillL = Lfill + Lw; // ** added 28/11/2019 **
            tFillR = Rfill - Rw; // ** added 28/11/2019 **
            if (ow > 0) {
                if (yb >= owMin && yb <= owMax && fCmp) {
                    if (Lfill <= Ldw)
                        (vert == 1)
                        ? VlineD(yb, Lfill, tFillL, tgrad)
                        : HlineD(yb, Lfill, tFillL, tgrad); // ** modified 28/11/2019 **
                    if (sr3 > -1 && tFillL + 1 < tFillR)
                        (vert == 1) ? VlineD(yb, tFillL + 1, tFillR,
                            Grad(sr3, darken, gl3, hm, mfill, colorD))
                        : HlineD(yb, tFillL + 1, tFillR,
                            Grad(sr3, darken, gl3, hm, mfill,
                                colorD)); // ** modified 28/11/2019 **
                    if (Rfill >= Rdw)
                        (vert == 1)
                        ? VlineD(yb, tFillR, Rfill, tgrad)
                        : HlineD(yb, tFillR, Rfill, tgrad); // ** modified 28/11/2019 **
                }
                else {
                    if (fCmp)
                        (vert == 1) ? VlineD(yb, Lfill, Rfill, tgrad)
                        : HlineD(yb, Lfill, Rfill, tgrad);
                }
            }
            else {
                if (fCmp)
                    (vert == 1) ? VlineD(yb, Lfill, Rfill, tgrad)
                    : HlineD(yb, Lfill, Rfill, tgrad);
            }
            fill++;
        }
    }
    // ** reset used external intiables
    GSCropArcLeft = -1;
    GSCropArcRight = -1;
    keepLastArc = 0;
    GSErase = 0;
    GSSsxpos = -9999;
    GSSArconly = 0;
    if (!(protectLA))
        lastAsize = maxR; // Used for slider
    protectLA = 0;
    return;
}

/****************************************************************************/
/*!
  @brief  4DGL compatible Hline function
  @param  y - vertical start position
  @param  x1 - horizontal start position
  @param  x2 - horizontal end position
*/
/****************************************************************************/
void gfx4desp32P4::HlineD(int y, int x1, int x2, uint16_t color) {
    if (x1 > x2)
        gfx_Swap(x1, x2);
    Hline(x1, y, x2 - x1 + 1, color);
}

/****************************************************************************/
/*!
  @brief  4DGL compatible Vline function
  @param  x - horizontal  start position
  @param  y1 - vertical start position
  @param  y2 - vertical end position
*/
/****************************************************************************/
void gfx4desp32P4::VlineD(int x, int y1, int y2, uint16_t color) {
    if (y1 > y2)
        gfx_Swap(y1, y2);
    Vline(x, y1, y2 - y1 + 1, color);
}

void gfx4desp32P4::GradTriangleFilled(int x0, int y0, int x1, int y1, int x2,
    int y2, int color, int ncCol, int h,
    int ypos, int lev, int erase) {
    int a, b, y, last;
    if (y0 > y1) {
        gfx_Swap(y0, y1);
        gfx_Swap(x0, x1);
    }
    if (y1 > y2) {
        gfx_Swap(y2, y1);
        gfx_Swap(x2, x1);
    }
    if (y0 > y1) {
        gfx_Swap(y0, y1);
        gfx_Swap(x0, x1);
    }
    if (y0 == y2) {
        a = x0;
        b = x0;
        if (x1 < a) {
            a = x1;
        }
        else if (x1 > b) {
            b = x1;
        }
        if (x2 < a) {
            a = x2;
        }
        else if (x2 > b) {
            b = x2;
        }
        if (erase)
            color = Grad(0, 0, lev, h, y0 - ypos, ncCol);
        HlineD(y0, a, b, color);
        return;
    }
    int dx01, dy01, dx02, dy02, dx12, dy12, sa, sb, t1, t2, t3;
    dx01 = x1 - x0;
    dy01 = y1 - y0;
    dx02 = x2 - x0;
    dy02 = y2 - y0;
    dx12 = x2 - x1;
    dy12 = y2 - y1;
    sa = 0;
    sb = 0;
    if (y1 == y2) {
        last = y1; // Include y1 scanline
    }
    else {
        last = y1 - 1; // Skip it
    }
    if (last < 0)
        last = 0;
    for (y = y0; y <= last; y++) {
        t1 = x0;
        t2 = dy01;
        t3 = dx01;
        a = t1 + sa / t2;
        b = x0 + sb / dy02;
        sa += t3;
        sb += dx02;
        if (erase)
            color = Grad(0, 0, lev, h, y - ypos, ncCol);
        HlineD(y, a, b, color);
    }
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (y = y; y <= y2; y++) {
        t1 = x1;
        t2 = dy12;
        t3 = dx12;
        a = t1 + sa / t2;
        b = x0 + sb / dy02;
        sa += t3;
        sb += dx02;
        if (erase)
            color = Grad(0, 0, lev, h, y - ypos, ncCol);
        HlineD(y, a, b, color);
    }
}

uint16_t gfx4desp32P4::Grad(int GraisSunk, int Gstate, int Gglev, int Gh1,
    int Gpos, uint16_t colToAdj) {
    // **** Modified gradients for vertical again ****        *new*
    //uint32_t __alpha;
    //uint32_t __alphatemp;
    //uint16_t __colour;
    int tgcol = colToAdj;
    int CTAr, CTAg, CTAb;
    int CTALevel;
    int BflatMin = 0;
    int BflatMax = 0;
    int w = 0;
    int t = 0;

    if (Gglev > 63)
        Gglev = 63;
    if (GraisSunk > 99) { // **** new ****
        w = GraisSunk & 0x00ff;
        GraisSunk = GraisSunk >> 8;
    }
    if (GraisSunk >= 4) { // **** end of new ****
        Gpos <<= 1;
        if (Gpos > Gh1)
            Gpos = Gh1 - (Gpos - Gh1); // **** new ****
        if (Gpos > w << 1 && w > 0 && GraisSunk == 4) {
            t = Gpos - (w << 1);
            Gpos = (Gpos - t) + (t >> 1);
        }
        if (GraisSunk == 5 && w > 0) {
            Gh1 -= w;
            Gpos -= w;
        } // **** end of new ****

        if (GraisSunk == 4) {
            GraisSunk = 5;
        }
        else {
            GraisSunk = 4;
        }
    }
    Gpos -= (Gh1 >> 1);
    if (Gglev == -1 || GraisSunk >= 6) {
        Gglev = 31;
        BflatMax = Gh1 >> 2;
        BflatMin = 0 - BflatMax;
        if (Gpos > BflatMin && Gpos < BflatMax)
            Gpos = 0;
        if (Gpos <= BflatMin)
            Gpos += BflatMax;
        if (Gpos >= BflatMax)
            Gpos -= BflatMax;
    }

    CTALevel = (Gglev * Gpos) / Gh1;
    CTAr = (tgcol >> 11) - Gstate;
    CTAg = ((tgcol >> 6) & 0x001F) - Gstate;
    CTAb = (tgcol & 0x001F) - Gstate;
    if (!((GraisSunk & 0x01)))
        CTALevel = 0 - CTALevel;
    CTAr += CTALevel;
    CTAg += CTALevel;
    CTAb += CTALevel;
    CTAr = (CTAr > 0) * CTAr;
    CTAg = (CTAg > 0) * CTAg;
    CTAb = (CTAb > 0) * CTAb;
    if (CTAr > 31)
        CTAr = 31;
    if (CTAg > 31)
        CTAg = 31;
    if (CTAb > 31)
        CTAb = 31;
    if (CTAr < 0)
        CTAr = 0;
    if (CTAg < 0)
        CTAg = 0;
    if (CTAb < 0)
        CTAb = 0;

    tgcol = (CTAr << 11) | (CTAg << 6) | CTAb;

    return tgcol;
}

/****************************************************************************/
/*!
  @brief  Font 2 drawing routine for text window (helper function)
  @param  see manual
*/
/****************************************************************************/
void gfx4desp32P4::drawChar2tw(int16_t x, int16_t y, unsigned char c,
    uint16_t color, uint16_t bg, uint8_t size) {
    if ((x >= width) || (y >= height) || ((x + (8 + 1) * size - 1) < 0) ||
        ((y + 16 * size - 1) < 0))
        return;
    if (c < 32 && c > 128)
        return;
    temppix[257];
    uint16_t pc = 0;
    SetGRAM(x, y, x + 7, y + 15);
    uint8_t trow = 0x80;
    uint8_t chb;
    uint16_t c2pos = c * 16;
    for (uint8_t j = 0; j < 16; j++) {
        chb = font2[c2pos + j];
        for (uint8_t i = 0; i < 8; i++) {
            if (chb & trow) {
                temppix[pc] = color;
            }
            else {
                temppix[pc] = bg;
            }
            chb <<= 1;
            pc++;
        }
        trow = 0x80;
    }
    WrGRAMs(temppix, 128);
}

/****************************************************************************/
/*!
  @brief  Font 1 drawing routine for text window (helper function)
  @param  see manual
*/
/****************************************************************************/
void gfx4desp32P4::drawChar1tw(int16_t x, int16_t y, unsigned char c,
    uint16_t color, uint16_t bg, uint8_t size) {
    if ((x >= width) || (y >= height) || ((x + (fsw + 1) * size - 1) < 0) ||
        ((y + fsh * size - 1) < 0))
        return;
    if (c < 32 && c > 128)
        return;
    SetGRAM(x, y, x + 4, y + 7);
    uint8_t trow = 0x01;
    uint8_t chb;
    uint8_t chb1;
    temppix[40];
    uint8_t pc = 0;
    for (uint8_t j = 0; j < 8; j++) {
        for (uint8_t i = 0; i < 5; i++) {
            chb = font1[(c * 5) + i];
            chb1 = chb >> j;
            if (chb1 & trow) {
                temppix[pc] = color;
            }
            else {
                temppix[pc] = bg;
            }
            pc++;
        }
    }
    WrGRAMs(temppix, 40);
}

/****************************************************************************/
/*!
  @brief  value to String formatting functions for text window
  @param  see manual
*/
/****************************************************************************/
size_t gfx4desp32P4::TWprintf(const char* format, ...) {
    va_list args;

    va_start(args, format);
    int result = vsnprintf(__printf_buf, MAX_TXT_BUF_SIZE, format, args);
    va_end(args);

    if (result < 0)
    {
        // Error in formatting
        return 0;
    }

    return TWstring2write(__printf_buf);
}

void gfx4desp32P4::TWprintln(String istr) {
    TWstring2write(istr.c_str());
    TWwrite(13);
}

void gfx4desp32P4::TWprintln(char* istr) {
    TWstring2write(istr);
    TWwrite(13);
}

void gfx4desp32P4::TWprintln(int8_t istr) {
    TWprintf("%d\r", istr);
}

void gfx4desp32P4::TWprintln(uint8_t istr) {
    TWprintf("%u\r", istr);
}

void gfx4desp32P4::TWprintln(int16_t istr) {
    TWprintf("%d\r", istr);
}

void gfx4desp32P4::TWprintln(int istr) {
    TWprintf("%d\r", istr);
}

void gfx4desp32P4::TWprintln(uint16_t istr) {
    TWprintf("%u\r", istr);
}

void gfx4desp32P4::TWprintln(int32_t istr) {
    TWprintf("%ld\r", istr);
}

void gfx4desp32P4::TWprintln(uint32_t istr) {
    TWprintf("%lu\r", istr);
}

void gfx4desp32P4::TWprintln(int64_t istr) {
    TWprintf("%lld\r", istr);
}

void gfx4desp32P4::TWprintln(uint64_t istr) {
    TWprintf("%llu\r", istr);
}

void gfx4desp32P4::TWprintln(float istr) {
    TWprintf("%f\r", istr);
}

void gfx4desp32P4::TWprint(String istr) {
    TWprintf("%s", istr.c_str());
}

void gfx4desp32P4::TWprint(char* istr) {
    TWprintf("%s", istr);
}

void gfx4desp32P4::TWprint(int8_t istr) {
    TWprintf("%d", istr);
}

void gfx4desp32P4::TWprint(uint8_t istr) {
    TWprintf("%u", istr);
}

void gfx4desp32P4::TWprint(int16_t istr) {
    TWprintf("%d", istr);
}

void gfx4desp32P4::TWprint(uint16_t istr) {
    TWprintf("%u", istr);
}

void gfx4desp32P4::TWprint(uint istr) {
    TWprintf("%u", istr);
}

void gfx4desp32P4::TWprint(int32_t istr) {
    TWprintf("%ld", istr);
}

void gfx4desp32P4::TWprint(uint32_t istr) {
    TWprintf("%lu", istr);
}

void gfx4desp32P4::TWprint(int64_t istr) {
    TWprintf("%lld", istr);
}

void gfx4desp32P4::TWprint(uint64_t istr) {
    TWprintf("%llu", istr);
}

void gfx4desp32P4::TWprint(float istr) {
    TWprintf("%f", istr);
}

size_t gfx4desp32P4::TWstring2write(const char * istr) {
    char * c = (char *)istr;
    bool needsEndWrite = StartWrite();
	while (*c) {
        TWwrite(*c);
        c++;
    }
	if (needsEndWrite) EndWrite();
    return c - istr;
}


/****************************************************************************/
/*!
  @brief  returns a string of the last entered text before CR from text window
  @returns - String of text entered
*/
/****************************************************************************/
String gfx4desp32P4::GetCommand() {
    String tcmdtxt = cmdtxt;
    cmdtxt = "";
    return tcmdtxt;
}

/****************************************************************************/
/*!
  @brief  returns a string of the last entered text before CR from text window
  @param - twc RGB565 colour of text window text
*/
/****************************************************************************/
void gfx4desp32P4::TWtextcolor(uint16_t twc) { twcolnum = twc; }

/****************************************************************************/
/*!
  @brief  move the character position in text window
  @param - twcrx x character position in text window
  @param - twcry y character position in text window
*/
/****************************************************************************/
boolean gfx4desp32P4::TWMoveTo(uint8_t twcrx, uint8_t twcry) {
    if (twcrx <= chracc && twcry <= chrdwn && chracc > 0 && chrdwn > 0) {
        twcurx = txtx + (9 * twcrx);
        twcury = txty + (16 * twcry);
        twxpos = twcrx;
        twypos = twcry;
        return true;
    }
    else {
        return false;
    }
}

/****************************************************************************/
/*!
  @brief  print a string at character position x y
  @param - twcrx x character position in text window
  @param - twcry y character position in text window
  @param - istr string to be printed
*/
/****************************************************************************/
void gfx4desp32P4::TWprintAt(uint8_t pax, uint8_t pay, String istr) {
    if (TWMoveTo(pax, pay))
        TWprint(istr);
}

/****************************************************************************/
/*!
  @brief  write a character at current text window cursor position
  @param - txtinput character to be written
*/
/****************************************************************************/
void gfx4desp32P4::TWwrite(const char txtinput) {
    if (!txtwin) return;
	if (TWimgSet) {
        if (twcurson && twen)
            drawChar2TWimage(0, TWimage, 0, twcurx, twcury, txtf);
    }
    else {
        if (twcurson && twen)
            drawChar2tw(twcurx, twcury, 0, txtf, txtb, 1);
    }
    boolean skip2 = false;
    if (txtinput > 31) {
        twtext = twtext + char(txtinput);
        if (TWimgSet) {
            if (twen)
                drawChar2TWimage(txtinput - 32, TWimage, 0, twcurx, twcury, twcolnum);
        }
        else {
            if (twen)
                drawChar2tw(twcurx, twcury, txtinput - 32, twcolnum, txtb, 1);
        }
        txtbuf[(chracc * twypos) + twxpos] = txtinput;
        txfcol[(chracc * twypos) + twxpos] = twcolnum;
        twcurx = twcurx + 9;
        twxpos++;
        if ((twcurx + 8 + 1) > (txtw + txtx)) {
            twcury = twcury + 16;
            twcurx = txtx;
            twypos++;
            twxpos = 0;
        }
    }
    if (txtinput == 9) {
        uint tcnt = 0;
        uint ccpos = twcurx / 9;
        for (int n = 0; n < (chracc / 10); n++) {
            tcnt = tcnt + 14;
            if (tcnt > ccpos) {
                for (uint o = 0; o < (tcnt - ccpos); o++) {
                    twtext = twtext + char(32);
                    twcurx = twcurx + 9;
                    twxpos++;
                    txtbuf[(chracc * twypos) + twxpos] = 32; // 8 + 2
                    txfcol[(chracc * twypos) + twxpos] = twcolnum;
                }
                break;
            }
        }
    }
    if (txtinput == 13 || txtinput == 10) {
        twcury = twcury + 16;
        twypos++;
        uint8_t remtxt = chracc - twxpos + 1;
        twcurx = txtx;
        twxpos = 0;
        twcl = twcl + 1;
        if (txtinput == 13) {
            cmdtxt = twtext;
            twtext = "";
        }
        for (int n = 0; n < remtxt; n++) {
            txtbuf[(chracc * (twypos - 1)) + (chracc - n)] = char(0);
        }
    }
    if (txtinput == 8) {
        if (twypos < 1 && twxpos < 1) {
            return;
        }
        uint16_t lenct = twtext.length();
        if ((twcurx - txtx + 6) < (10) && lenct > 0 && twcury > fsh) {
            skip2 = true;
            twcury = twcury - 16;
            twypos--;
            twcurx = txtx + (((txtw / 9) * 9) - 9);
            twxpos = chracc - 1;
            if (TWimgSet) {
                if (twen)
                    drawChar2TWimage(0, TWimage, 0, twcurx, twcury, twcolnum);
            }
            else {
                if (twen)
                    drawChar2tw(twcurx, twcury, 0, twcolnum, txtb, 1);
            }
            txtbuf[(chracc * twypos) + twxpos] = txtinput;
            txfcol[(chracc * twypos) + twxpos] = twcolnum;
        }
        if (twcurx > txtx && lenct > 0 && skip2 == false) {
            twcurx = twcurx - 8 - 1;
            twxpos--;
            if (TWimgSet) {
                if (twen)
                    drawChar2TWimage(0, TWimage, 0, twcurx, twcury, twcolnum);
            }
            else {
                if (twen)
                    drawChar2tw(twcurx, twcury, 0, twcolnum, txtb, 1);
            }
            txtbuf[(chracc * twypos) + twxpos] = txtinput;
            txfcol[(chracc * twypos) + twxpos] = twcolnum;
        }
        tempcmd = "";
        for (int n = 0; n < (lenct - 1); n++) {
            tempcmd = tempcmd + twtext.charAt(n);
        }
        twtext = tempcmd;
    }
    if ((twcury - txty) + 16 > txth) {
        uint16_t tempc;
        uint16_t tempp;
        uint16_t tempcpos;
        uint16_t temptwcol;
        uint16_t temptwcolc;

        for (int n = 0; n < chrdwn - 1; n++) {
            //yield();
            for (int o = 0; o < chracc; o++) {
                tempcpos = ((n + 1) * chracc) + o;
                tempc = txtbuf[tempcpos];
                tempp = txtbuf[(n * chracc) + o];
                temptwcol = txfcol[tempcpos];
                temptwcolc = txfcol[(n * chracc) + o];
                txtbuf[(n * chracc) + o] = tempc;
                txfcol[(n * chracc) + o] = temptwcol;
                if (tempc < 32)
                    tempc = 32;
                if (tempp < 32)
                    tempp = 32;
                if (tempc != tempp || temptwcol != temptwcolc) {
                    if (TWimgSet) {
                        if (twen)
                            drawChar2TWimage(tempc - 32, TWimage, 0, txtx + (9 * o),
                                txty + (16 * n), temptwcol);
                    }
                    else {
                        //if (twen && rotation != PORTRAIT)
                        //    drawChar2tw(txtx + (9 * o), txty + (16 * n), tempc - 32,
                        //        temptwcol, txtb, 1);
                    }
                }
            }
        }
		if (!TWimgSet && twen){
			int yofst = ((txth / 16) * 16);
			if (rotation == PORTRAIT){
				DrawFrameBufferAreaXYPPA(frame_buffer, txtx, txty + 16, txtx + txtw - 1, txty + yofst - 1, txtx, txty);
				//RectangleFilledPPA(txtx, txty + yofst, txtx + txtw - 1, txty + yofst + 16, txtb);  
			} else {
				int fbBck = frame_buffer;
				DrawToframebuffer(WIDGET_BUFFER);
				DrawFrameBufferAreaXYPPA(fbBck, txtx, txty + 16, txtx + txtw - 1, txty + yofst - 1, txtx, txty);
				DrawToframebuffer(fbBck);
				DrawFrameBufferAreaXYPPA(WIDGET_BUFFER, txtx, txty, txtx + txtw - 1, txty + yofst - 1, txtx, txty);
			}
		}
		twcury = twcury - 16;
        twypos--;
        if (TWimgSet) {
            if (twen)
                UserImagesDR(TWimage, 0, twcurx - tuix[TWimage], twcury - tuiy[TWimage],
                    txtw - 1, 16);
        }
        else {
            if (twen)
                RectangleFilledPPA(twcurx, twcury, twcurx + (txtw - 1) - 1, twcury + 16,
                    txtb);
        }
    }
    if (twcurson) {
        if (TWimgSet) {
            if (twen)
                drawChar2TWimage(63, TWimage, 0, twcurx, twcury, txtf);
        }
        else {
            if (twen)
                drawChar2tw(twcurx, twcury, 63, txtf, txtb, 1);
        }
    }
}

/****************************************************************************/
/*!
  @brief  enable / disable text entry cursor (enabled by default)
  @param - twco true or false
*/
/****************************************************************************/
void gfx4desp32P4::TWcursorOn(bool twco) { twcurson = twco; }

/****************************************************************************/
/*!
  @brief  Clear text window and reset cursor position to 0, 0
  @param - none
*/
/****************************************************************************/
void gfx4desp32P4::TWcls() {
    if (TWimgSet) {
        UserImages(TWimage, 0);
    }
    else {
        RectangleFilled(txtx - 3, txty - 3, (txtx - 3) + (txtw + 2) - 1,
            (txty - 3) + (txth + 2) - 1, txtb);
		if (rotation != PORTRAIT){
			int fcBck = frame_buffer;
			DrawToframebuffer(WIDGET_BUFFER);
			RectangleFilled(txtx - 3, txty - 3, (txtx - 3) + (txtw + 2) - 1,
				(txty - 3) + (txth + 2) - 1, txtb);
			DrawToframebuffer(fcBck);
		}
    }
    twcurx = txtx;
    twcury = txty;
    twxpos = 0;
    twypos = 0;
    for (int n = 0; n < sizeof(txtbuf); n++) {
        txtbuf[n] = 0;
    }
}

/****************************************************************************/
/*!
  @brief  Set forground text color of font in text window
  @param  fcol - RGB565 foreground color
*/
/****************************************************************************/
void gfx4desp32P4::TWcolor(uint16_t fcol) {
    txtf = fcol;
    TWtextcolor(fcol);
}

/****************************************************************************/
/*!
  @brief  Set forground and background text color of font in text window
  @param  fcol - RGB565 foreground color
  @param  bcol - RGB565 background color
*/
/****************************************************************************/
void gfx4desp32P4::TWcolor(uint16_t fcol, uint16_t bcol) {
    txtf = fcol;
    txtb = bcol;
    TWtextcolor(fcol);
}

/****************************************************************************/
/*!
  @brief  create an automated text handling window with GCI image background
  @param  x - x position of top left corner of text window
  @param  y - y position of top left corner of text window
  @param  w - width of text window
  @param  h - height of text window
  @param  txtcolor - System font foreground color 
  @param  TWimg - Index of GCI userimage to be used as background
  @param  frcolor - Color of bevel frame around text window, set to BLACK for none
*/
/****************************************************************************/
void gfx4desp32P4::TextWindowImage(int16_t x, int16_t y, int16_t w, int16_t h,
    uint16_t txtcolor, uint16_t TWimg,
    uint16_t frcolor) {
    TWimage = TWimg;
    TWimgSet = 1;
    if (x < tuix[TWimg])
        x = tuix[TWimg];
    if (y < tuiy[TWimg])
        y = tuiy[TWimg];
    if (x + w > tuix[TWimg] + tuiw[TWimg])
        w = tuix[TWimg] + tuiw[TWimg] - x;
    if (y + h > tuiy[TWimg] + tuih[TWimg])
        h = tuiy[TWimg] + tuih[TWimg] - y;
    TextWindow(x, y, w, h, txtcolor, 0);
}

/****************************************************************************/
/*!
  @brief  create an automated text handling window
  @param  x - x position of top left corner of text window
  @param  y - y position of top left corner of text window
  @param  w - width of text window
  @param  h - height of text window
  @param  txtcolor - System font foreground color 
  @param  txbcolor - System font background color 
  @param  frcolor - Color of bevel frame around text window, set to BLACK for none
*/
/****************************************************************************/
void gfx4desp32P4::TextWindow(int16_t x, int16_t y, int16_t w, int16_t h,
    uint16_t txtcolor, uint16_t txbcolor,
    uint16_t frcolor) {
    int calcbuf = ((w / 8) + 1) * ((h / 16) + 2);
    //if (TWInitial) {
        txfcol = (uint16_t*)realloc(txfcol, calcbuf << 1);
        txtbuf = (uint8_t*)realloc(txfcol, calcbuf);
        TWInitial = false;
    //}
    twen = true;
    for (int n = 0; n < calcbuf; n++) {
        txtbuf[n] = 0;
    }
    twxpos = 0;
    twypos = 0;
    if (w < 24)
        w = 24;
    if (h < 31)
        h = 31;
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if ((w + x) > width)
        w = w - ((w + x) - width);
    if ((h + y) > height)
        h = h - ((h + y) - height);
    txtwin = true;
    TWtextcolor(txtcolor);
    txtf = txtcolor;
    txtb = txbcolor;
    txtx = x + 6;
    twcurx = txtx;
    txty = y + 6;
    twcury = txty;
    txtw = w - 8;
    txth = h - 8;
    twtext = "";
    twframe = true;
    twframecol = frcolor;
    chracc = txtw / 9;
    chrdwn = txth / 16;
    PanelRecessed(x, y, w, h, frcolor);
    RectangleFilled(x + 3, y + 3, (x + 3) + (w - 6) - 1, (y + 3) + (h - 6) - 1,
        txbcolor);
	if (rotation != PORTRAIT){
		int fcBck = frame_buffer;
		DrawToframebuffer(WIDGET_BUFFER);
		RectangleFilled(x + 3, y + 3, (x + 3) + (w - 6) - 1, (y + 3) + (h - 6) - 1, txbcolor);
		DrawToframebuffer(fcBck);
	}
    TWimgSet = 0;
}

/****************************************************************************/
/*!
  @brief  Enable or disable text window
  @param  twen - true enabled, false disabled
*/
/****************************************************************************/
void gfx4desp32P4::TWenable(boolean t) { twen = t; }

/****************************************************************************/
/*!
  @brief  Restore the text window with contents that may have been erased by Cls
*/
/****************************************************************************/
void gfx4desp32P4::TextWindowRestore() {
    twen = true;
    uint16_t chracc = txtw / (fsw + 1);
    uint8_t chrdwn = txth / fsh;
    txtwin = true;
    twtext = "";
    uint16_t tcoltw;
    if (TWimgSet) {
        UserImages(TWimage, 0);
    }
    else {
        if (twframe) {
            PanelRecessed(txtx - 6, txty - 6, txtw + 8, txth + 8, twframecol);
            RectangleFilled(txtx - 3, txty - 3, (txtx - 3) + (txtw + 2) - 1,
                (txty - 3) + (txth + 2) - 1, txtb);
			if (rotation != PORTRAIT){
				int fcBck = frame_buffer;
				DrawToframebuffer(WIDGET_BUFFER);
				RectangleFilled(txtx - 3, txty - 3, (txtx - 3) + (txtw + 2) - 1,
					(txty - 3) + (txth + 2) - 1, txtb);
				DrawToframebuffer(fcBck);
			}				
        }
        else {
            RectangleFilled(txtx - 3, txty - 3, (txtx - 3) + (txtw + 2) - 1,
                (txty - 3) + (txth + 2) - 1, txtb);
			if (rotation != PORTRAIT){
				int fcBck = frame_buffer;
				DrawToframebuffer(WIDGET_BUFFER);
				RectangleFilled(txtx - 3, txty - 3, (txtx - 3) + (txtw + 2) - 1,
					(txty - 3) + (txth + 2) - 1, txtb);
				DrawToframebuffer(fcBck);
			}				
        }
    }
    uint16_t tempc;
    for (int n = 0; n < (chrdwn - 1); n++) {
        yield();
        for (int o = 0; o < chracc; o++) {
            tempc = txtbuf[(n * chracc) + o];
            txtbuf[((n * chracc) + o)] = tempc;
            if (tempc < 32) {
                tempc = 32;
            }
            tcoltw = txfcol[(n * chracc) + o];
            if (TWimgSet) {
                drawChar2TWimage(tempc - 32, TWimage, 0, txtx + ((fsw + 1) * o),
                    txty + (fsh * n), tcoltw);
            }
            else {
                drawChar2tw(txtx + ((fsw + 1) * o), txty + (fsh * n), tempc - 32,
                    tcoltw, txtb, 1);
            }
        }
    }
}

/****************************************************************************/
/*!
  @brief  create an automated text handling window without frame
  @param  x - x position of top left corner of text window
  @param  y - y position of top left corner of text window
  @param  w - width of text window
  @param  h - height of text window
  @param  txtcolor - System font foreground color 
  @param  txbcolor - System font background color 
*/
/****************************************************************************/
void gfx4desp32P4::TextWindow(int16_t x, int16_t y, int16_t w, int16_t h,
    uint16_t txtcolor, uint16_t txbcolor) {
    twen = true;
    if (!(TWimgSet))
        TWimage = -1;
    int calcbuf = ((w / 8) + 1) * ((h / 16) + 2);
    //if (TWInitial) {
        txfcol = (uint16_t*)realloc(txfcol, calcbuf << 1);
        txtbuf = (uint8_t*)realloc(txtbuf, calcbuf);
        TWInitial = false;
    //}
    for (int n = 0; n < calcbuf; n++) {
        txtbuf[n] = 0;
    }
    if (w < 22)
        w = 22;
    if (h < 29)
        h = 29;
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if ((w + x) > width)
        w = w - ((w + x) - width);
    if ((h + y) > height)
        h = h - ((h + y) - height);
    twxpos = 0;
    twypos = 0;
    TWtextcolor(txtcolor);
    txtwin = true;
    txtf = txtcolor;
    txtb = txbcolor;
    txtx = x + 3;
    twcurx = txtx;
    txty = y + 3;
    twcury = txty;
    txtw = w - 6;
    txth = h - 6;
    twtext = "";
    twframe = false;
    chracc = txtw / 9;
    chrdwn = txth / 16;
    if (TWimage == -1)
        RectangleFilled(x, y, x + w - 1, y + h - 1, txbcolor);
	if (rotation != PORTRAIT){
		int fcBck = frame_buffer;
		DrawToframebuffer(WIDGET_BUFFER);
		RectangleFilled(x, y, x + w - 1, y + h - 1, txbcolor);
		DrawToframebuffer(fcBck);
	}		
}

void gfx4desp32P4::drawChar2TWimage(uint8_t ch, uint16_t uino, int frames,
    int16_t uxpos, int16_t uypos,
    uint16_t txtcol) {
    UIDRcharOn = 1;
    TWcharByte = 0;
    TWcharBit = 6;
    TWimageCount = ch << 4;
    TWimageTextCol = txtcol;
    UserImagesDR(uino, frames, uxpos - tuix[uino], uypos - tuiy[uino], 8, 16);
}

void gfx4desp32P4::Panel(int16_t x, int16_t y, int16_t w, int16_t h,
    uint16_t colorp) {
    RectangleFilled(x + 2, y + 2, (x + 2) + (w - 3) - 1, (y + 2) + (h - 3) - 1,
        colorp);
    uint32_t tcol = bevelColor(colorp);
    uint16_t _dark = tcol >> 16;
    uint16_t _light = tcol & 0xffff;
    Hline(x, y, w, _light);
    Hline(x + 1, y + 1, w - 2, _light);
    Vline(x, y, h, _light);
    Vline(x + 1, y + 1, h - 2, _light);
    Hline(x, y + h - 1, w, _dark);
    Hline(x + 1, y + h - 2, w - 2, _dark);
    Vline(x + w - 1, y, h, _dark);
    Vline(x + w - 2, y + 1, h - 2, _dark);
}

void gfx4desp32P4::PanelRecessed(int16_t x, int16_t y, int16_t w, int16_t h,
    uint16_t colorpr) {
    RectangleFilled(x, y, x + w - 1, y + h - 1, colorpr);
    uint32_t tcol = bevelColor(colorpr);
    uint16_t _dark = tcol >> 16;
    uint16_t _light = tcol & 0xffff;
    Hline(x + 2, y + 2, w - 4, _dark);
    Vline(x + 2, y + 2, h - 4, _dark);
    Hline(x + 3, y + h - 3, w - 5, _light);
    Vline(x + w - 3, y + 3, h - 5, _light);
}

int16_t gfx4desp32P4::XYposToDegree(int curX, int curY) {
    int delta, deg, adj;
    if (curY < 0) {
        if (curX < 0) {
            adj = 1;
            deg = 90;
        }
        else {
            adj = 2;
            deg = 180;
        }
    }
    else {
        if (curX < 0) {
            deg = 0;
            adj = 2;
        }
        else {
            deg = 270;
            adj = 1;
        }
    }
    curX = abs(curX);
    curY = abs(curY);
    if (curX < curY) {
        adj &= 1;
    }
    else {
        adj &= 2;
        gfx_Swap(curX, curY);
    }
    delta = at[(curX * 100) / curY];
    if (adj) {
        deg += 90 - delta;
    }
    else {
        deg += delta;
    }
    return deg;
}

uint16_t gfx4desp32P4::getNumberofObjects(void) { return gciobjnum; }

void gfx4desp32P4::ButtonDown(int hndl) {
    uint16_t x = bposx[hndl];
    uint16_t y = bposy[hndl];
    uint16_t w = bposw[hndl];
    uint16_t h = bposh[hndl];
    uint16_t colorbd = bposc[hndl];
    uint32_t tcol = bevelColor(colorbd);
    uint16_t _dark = tcol >> 16;
    if (bxStyle == 0) {
        Hline(x, y, w, colorbd);
        Hline(x + 1, y + 1, w - 2, colorbd);
        Vline(x, y, h, colorbd);
        Vline(x + 1, y + 1, h - 2, colorbd);
        Hline(x, y + h - 1, w, _dark);
        Hline(x + 1, y + h - 2, w - 2, colorbd);
        Vline(x + w - 1, y, h, _dark);
        Vline(x + w - 2, y + 1, h - 2, colorbd);
    }
    if (bxStyle > 0) {
        int nh, nw;
        nw = h / 10;
        if (nw == 0)
            nw = 1;
        if (bxStyle == 1) {
            nh = h / 6;
        }
        if (bxStyle > 1) {
            nh = h >> 1;
        }
        gradientShape(0, nw, x, y, w - 1, h - 1, nh, nh, nh, nh, 0, _dark,
            GRADIENT_RAISED, 0, -1, -1, -1, 0);
    }
}

void gfx4desp32P4::ButtonUp(int hndl) {
    if (bactive[hndl]) {
        uint16_t x = bposx[hndl];
        uint16_t y = bposy[hndl];
        uint16_t w = bposw[hndl];
        uint16_t h = bposh[hndl];
        uint16_t colorbu = bposc[hndl];
        uint32_t tcol = bevelColor(colorbu);
        uint16_t _dark = tcol >> 16;
        uint16_t _light = tcol & 0xffff;
        if (bxStyle == 0) {
            Hline(x, y, w, _light);
            Hline(x + 1, y + 1, w - 2, _light);
            Vline(x, y, h, _light);
            Vline(x + 1, y + 1, h - 2, _light);
            Hline(x, y + h - 1, w, _dark);
            Hline(x + 1, y + h - 2, w - 2, _dark);
            Vline(x + w - 1, y, h, _dark);
            Vline(x + w - 2, y + 1, h - 2, _dark);
        }
        g2 = 0;
        if (bxStyle > 0) {
            int nh, g1, nw;
            nw = h / 10;
            if (nw == 0)
                nw = 1;
            if (bxStyle == 1) {
                nh = h / 6;
                g1 = -1;
                g2 = 0;
            }
            if (bxStyle > 1) {
                nh = h >> 1;
                g1 = 28;
                g2 = 0;
            }
            if (bxStyle == 3) {
                g1 = 48;
                g2 = 28;
            }
            gradientShape(0, nw, x, y, w - 1, h - 1, nh, nh, nh, nh, 0, _dark,
                GRADIENT_RAISED, g1, -1, -1, -1, 0);
        }
    }
}

/****************************************************************************/
/*!
  @brief  Capture and save to uSD an area of the screen.
  @param  x - left X position in pixels
  @param  y - top Y position in pixels
  @param  w - width
  @param  h - height
  @param  fname - filname for saved image
  @note returns true if successful
*/
/****************************************************************************/
bool gfx4desp32P4::ScreenCapture(int16_t x, int16_t y, int16_t w, int16_t h,
    String fname) {
#ifndef USE_LITTLEFS_FILE_SYSTEM
    if (!sdok)
        return false;
    if (y < 0) {
        h -= 0 - y;
        y = 0;
    }
    if ((y + h) > height)
        h = height - y;

#ifdef USE_SDMMC_FILE_SYSTEM
    File tempfile;
    if (SD_MMC.exists(fname))
        return false;
    tempfile = SD_MMC.open(fname, FILE_WRITE);
#else
    FsFile tempfile;
    if (uSD.exists(fname))
        return false;
    tempfile = uSD.open(fname, FILE_WRITE);
#endif
    int n, o, wline;
    uint16_t sline[w];
    for (n = 0; n < h; n++) {
        wline = ReadLine(x, y + n, w, sline);
        if (n == 0) {
            tempfile.write(wline >> 8);
            tempfile.write(wline);
            tempfile.write(h >> 8);
            tempfile.write(h);
            tempfile.write(0x10);
            tempfile.write(n);
        }
        for (o = 0; o < wline; o++) {
            if (DisplayType == DISP_INTERFACE_RGB) {
                tempfile.write(sline[o] >> 8);
                tempfile.write(sline[o]);
            }
            else {
                tempfile.write(sline[o]);
                tempfile.write(sline[o] >> 8);
            }
        }
    }
    tempfile.close();
    return true;
#endif
    return false;
}

/****************************************************************************/
/*!
  @brief  Select font style for Font1 characters.
  @param  ctyp - 0 to 6 0 is default no style.
  @note adds effect to system Font1 with size of 3 or more.
*/
/****************************************************************************/
void gfx4desp32P4::FontStyle(uint8_t ctyp) { fstyle = ctyp % 6; }

/****************************************************************************/
/*!
  @brief  Get the number of the current frame buffer in use.
  @note returns current frame buffer number.
*/
/****************************************************************************/
uint8_t gfx4desp32P4::GetFrameBuffer() {
    return frame_buffer;
}

/****************************************************************************/
/*!
  @brief  Read the colour value of a pixel within a GCI object.
  @param  inum - GCI object index.
  @param  x - The X position that the pixel would be at if the image was drawn on-screen
  @param  y - The Y position that the pixel would be at if the image was drawn on-screen
  @note returns 16bit colour value.
*/
/****************************************************************************/
uint16_t gfx4desp32P4::ReadImagePixel(int inum, int x, int y) {
    if ((GCItype == GCI_SYSTEM_USD && (!userImag)) || (GCItype == GCI_SYSTEM_JPEG_USD && (!userImag)) ||
		(GCItype == GCI_SYSTEM_PROGMEM && (!gcidatArray))/* || (GCItype == GCI_SYSTEM_JPEG_FLASH && (!gcidatArray))*/)
        return 0;
    int yc = y - tuiy[inum];
    int xc = x - tuix[inum];
    if (y < tuiy[inum] || x < tuix[inum]) return 0;
    if (y > tuiy[inum] + tuih[inum] - 1 || x > tuix[inum] + tuiw[inum] - 1) return 0;
    int fr =  tuiImageIndex[inum];
	if (gciobjframes[inum] == 1) fr = 0;
	if (GCItype == GCI_SYSTEM_JPEG_USD || GCItype == GCI_SYSTEM_JPEG_FLASH){
		int twO = DecodeJpeg(inum, fr); 
		uint32_t pos = ((yc * (tuiw[inum] + twO)) << 1) + (xc << 1);
		return rx_buf[pos] + (rx_buf[pos + 1] << 8);
	} else {
		uint32_t ofst = 6;
		ofst += ((gciobjframes[inum] > 0) << 1);
		uint32_t pos = ((yc * tuiw[inum]) << 1) + (xc << 1);
		if (ofst == 8) ofst += (((tuiw[inum] * tuih[inum]) * tuiImageIndex[inum]) << 1);
		GCIreadToBuff(tuiIndex[inum] + ofst + pos, 2);
		return psRAMbuffer1[1] + (psRAMbuffer1[0] << 8);
	}
	return 0;
}

int gfx4desp32P4::DecodeJpeg(uint16_t ui, uint16_t frame){
	int twO; 
	uint32_t jpegsize;
	if (gciobjframes[ui] == 1) frame = 0;
	if (_lastObject != ui || _lastFrame != frame){
	    jpegsize = jpegSIZES[jpegOFFSETSpos[ui] + frame];
		if(GCItype == GCI_SYSTEM_JPEG_USD){
			twO = DecodeJPEGfromGCJ(jpegOFFSETS[jpegOFFSETSpos[ui] + frame], jpegsize, tuiw[ui], tuih[ui]);
		} else {
			twO = DecodeJPEGfromGCJinFlash(jpegOFFSETS[jpegOFFSETSpos[ui] + frame], jpegsize, tuiw[ui], tuih[ui]);
		}
		if (twO == 16) twO = 0;
	} else {
		twO = lasttwO;
	}
	_lastObject = ui; _lastFrame = frame;
	lasttwO = twO;
	return twO;
}

/****************************************************************************/
/*!
  @brief  Copies a horizontal line from GCI object to the current frame buffer.
  @param  inum - GCI object index.
  @param  x - The X position that the line would start if the image was drawn on-screen
  @param  y - The Y position that the line would start if the image was drawn on-screen
  @param  w - The length of the line to be copied
  @note returns nothing.
*/
/****************************************************************************/
void gfx4desp32P4::CopyImageLine(int inum, int x, int y, int w) {
    if (y > tuiy[inum] + tuih[inum] - 1 || y < 0)
        return;
    if (x > tuix[inum] + tuiw[inum] - 1 || (x + w - 1) < 0)
        return;
    if (w < 0) {
        x += w;
        //w *= -1;
        w = abs(w);
    }
    if (x < tuix[inum]) {
        w += x;
        x = 0;
    }
    //if ((x + w - 1) >= tuix[inum] + tuiw[inum] - 1)
    //    w = tuiw[inum] - x;
    int yc = y - tuiy[inum];
    int xc = x - tuix[inum];
    int fr =  tuiImageIndex[inum];
	if (gciobjframes[inum] == 1) fr = 0;
	 if ((GCItype == GCI_SYSTEM_USD && (!userImag)) || (GCItype == GCI_SYSTEM_JPEG_USD && (!userImag)) ||
		(GCItype == GCI_SYSTEM_PROGMEM && (!gcidatArray))/* || (GCItype == GCI_SYSTEM_JPEG_FLASH && (!gcidatArray))*/)
        return;
	if (GCItype == GCI_SYSTEM_JPEG_USD || GCItype == GCI_SYSTEM_JPEG_FLASH){
		int twO = DecodeJpeg(inum, fr);
		uint32_t pos = (yc * (tuiw[inum] + twO)) + xc;
		//if (frame_buffer == CANVAS_BUFFER_ARGB){
		PPAflushArea((uint16_t*)rx_buf + pos, w, 1, x, y, false, false);
		/*
		} else {
			usePushColors = (x >= clipx1) && (y >= clipy1) && (x + w <= clipx2) && (y <= clipy2) &&
			(!transalpha) && (!WriteFBonly) && (frame_buffer == visibleFB);
			SetGRAM(x, y, x + w - 1, y + 1);
			if (usePushColors) {
				pushColors((uint16_t*)rx_buf + pos, w);
			} else {
				WrGRAMs((uint16_t*)rx_buf + pos, w);
			}	
		}
		*/
	} else {
		uint32_t ofst = 6;
		ofst += ((gciobjframes[inum] > 0) << 1);
		uint32_t pos = ((yc * tuiw[inum]) << 1) + (xc << 1);
		if (ofst == 8) ofst += (((tuiw[inum] * tuih[inum]) * tuiImageIndex[inum]) << 1);
		GCIreadToBuff(tuiIndex[inum] + ofst + pos, w << 1);
		usePushColors = (x >= clipx1) && (y >= clipy1) && (x + w <= clipx2) && (y <= clipy2) &&
			(!transalpha) && (!WriteFBonly) && (frame_buffer == visibleFB);
		SetGRAM(x, y, x + w - 1, y + 1);
		if (usePushColors) {
			pushColors(psRAMbuffer1, w);
		} else {
			WrGRAMs(psRAMbuffer1, w);
		}
	}
}

/****************************************************************************/
/*!
  @brief  draws a horizontal line to the current frame buffer with color parameter switch.
  @param  x - The X position of the start of the line
  @param  y - The Y position of the start of the line
  @param  w - The length of the line to be drawn
  @param  color - can be a 16bit color or a GCI image or a FrameBuffer
                  HlineX(10, 10, 100, gfx.SelectDataSourceGCI(3)) draws a line at 10, 10 and a length of
                    100 pixels from GCI image index 3. x and y are relative to objects x and y
                    position.
                  HlineX(10, 10, 100, gfx.SelectDataSourceFB(3)) draws a line at 10, 10 and a length of
                    100 pixels from frame buffer 3 at the same x and y position.
  @note returns nothing.
*/
/****************************************************************************/
void gfx4desp32P4::HlineX(int x, int y, int w, int32_t color) {
    if (w < 1) return;
    uint8_t fB = 0;
    int16_t imageNum = -1;
    int gpos = 7 * (gradON == 2);
    if ((color & 0xffff0000) == 0xf00000) fB = color & 0x000f;
    if ((color & 0xffff0000) == 0xf0000) imageNum = color & 0xffff;
    uint16_t fgcol = color;
    if (fB) {
        CopyFrameBufferLine(x, y, w, fB);
    }
    else if (imageNum != -1) {
        CopyImageLine(imageNum, x, y, w);
    }
    else {
        if (gradON) {
            Hline(x, y, w, Grad(grad1[0 + gpos], grad1[1 + gpos], grad1[2 + gpos], grad1[3 + gpos], y - grad1[6 + gpos], fgcol));
        }
        else {
			Hline(x, y, w, fgcol);
        }
    }
}

/****************************************************************************/
/*!
  @brief  draws a vertical line to the current frame buffer with color parameter switch.
  @param  x - The X position of the start of the line
  @param  y - The Y position of the start of the line
  @param  w - The length of the line to be drawn
  @param  color - can be a 16bit color or a GCI image or a FrameBuffer
                  VlineX(10, 10, 100, gfx.SelectDataSourceFB(3)) draws a line at 10, 10 and a length of
                    100 pixels from GCI image index 3. x and y are relative to objects x and y
                    position.
                  VlineX(10, 10, 100, gfx.SelectDataSourceFB(3)) draws a line at 10, 10 and a length of
                    100 pixels from frame buffer 3 at the same x and y position.
  @note returns nothing.
*/
/****************************************************************************/
void gfx4desp32P4::VlineX(int x, int y, int w, int32_t color) {
	if (w < 1) return;
	bool needsEndWrite = StartWrite();
    uint8_t fB = 0;
    int16_t imageNum = -1;
    int gpos = 7 * (gradON == 2);
    if ((color & 0xffff0000) == 0xf00000) fB = color & 0x000f;
    if ((color & 0xffff0000) == 0xf0000) imageNum = color & 0xffff;
    uint16_t fgcol = color;
    if (fB || imageNum != -1 || (gradON && !gradientVert)) {
        for (int n = 0; n < w; n++) {
            if (fB) {
                fgcol = ReadPixelFromFrameBuffer(x, y + n, fB);
            }
            else {
                fgcol = ReadImagePixel(imageNum, x, y + n);
            }
            if (gradON) fgcol = Grad(grad1[0 + gpos], grad1[1 + gpos], grad1[2 + gpos], grad1[3 + gpos], (y + n) - grad1[6 + gpos], color);
            if (frame_buffer == CANVAS_BUFFER_ARGB){
				if (alpha){
					PutPixelAlpha(x, y + n, fgcol, __alpha);
				} else {
					PutPixelAlpha(x, y + n, fgcol, 255);
				}
			} else {
				PutPixel(x, y + n, fgcol);
			}
        }
    }
    else {
        if (gradientVert) fgcol = Grad(grad1[0 + gpos], grad1[1 + gpos], grad1[2 + gpos], grad1[3 + gpos], x - grad1[6 + gpos], color);
        Vline(x, y, w, fgcol);
    }
	if (needsEndWrite) EndWrite();
}

/****************************************************************************/
/*!
  @brief  draws a rectangle to the current frame buffer with color parameter switch.
  @param  x0 - The X position of the top left corner of the rectangle
  @param  y0 - The Y position of the top left corner of the rectangle
  @param  x1 - The X position of the bottom right corner of the rectangle
  @param  y1 - The Y position of the bottom right corner of the rectangle
  @param  color - can be a 16bit color or a GCI image or a FrameBuffer
                  RectangleFilledX(10, 10, 100, 100, gfx.SelectDataSourceGCI(3)) draws a rectangle at 10, 10
                    from GCI image index 3. x and y are relative to objects x and y.
                  RectangleFilledX(10, 10, 100, 100, gfx.SelectDataSourceFB(3)) draws a rectangle at 10, 10
                    from framebuffer 3 at the same x and y position.
  @note returns nothing.
*/
/****************************************************************************/
void gfx4desp32P4::RectangleFilledX(int x0, int y0, int x1, int y1, int32_t color) {
    uint8_t fB = 0;
    int16_t imageNum = -1;
    int w = x1 - x0 + 1;
    int h = y1 - y0 + 1;
    if ((color & 0xffff0000) == 0xf00000) fB = color & 0x000f;
    if ((color & 0xffff0000) == 0xf0000) imageNum = color & 0xffff;
    uint16_t fgcol = color;
    if (fB) {
        DrawFrameBufferArea(fB, x0, y0, x1, y1);
    }
    else if (imageNum != -1) {
        int fr =  tuiImageIndex[imageNum];
		if (gciobjframes[imageNum] == 1) fr = 0;
		UserImagesDR(imageNum, fr, x0, y0, x1 - x0 + 1, y1 - y0 + 1);
    }
    else {
        RectangleFilledPPA(x0, y0, x1, y1, color);
		//RectangleFilled(x0, y0, x1, y1, color);
    }
}

/****************************************************************************/
/*!
  @brief  draws a pixel to the current frame buffer with color parameter switch.
  @param  x - The X position of the pixel
  @param  y - The Y position of the pixel
  @param  color - can be a 16bit color or a GCI image or a FrameBuffer
                  PutPixelAlpha(10, 10, gfx.SelectDataSourceGCI(3), 255) draws a pixel at 10, 10
                    from GCI image index 3 and full alpha level. x and y are relative to objects x and y.
                  PutPixelAlpha(10, 10, gfx.SelectDataSourceFB(3), 127) draws a pixel at 10, 100
                    from GCI image index 3 with half alpha at the same x and y position.
                  PutPixelAlpha(10, 10, RED, 127) draws a RED pixel at 10, 10
                    with half alpha at the same x and y position.
  @param  alpha - the level of alpha that the pixel is drawn.
  @note returns nothing.
*/
/****************************************************************************/
void gfx4desp32P4::PutPixelAlpha(int x, int y, int32_t color, uint8_t alpha) {
    uint8_t FB = 0;
    int16_t imageNum = -1;
    uint16_t bg;
    int gpos = 7 * (gradON == 2);
    if ((color & 0xffff0000) == 0xf00000) FB = color & 0x0f;
    if ((color & 0xffff0000) == 0xf0000) imageNum = color & 0xffff;
    uint16_t fgcol;
    if (FB) {
        fgcol = ReadPixelFromFrameBuffer(x, y, FB);
    }
    else if (imageNum != -1) {
        fgcol = ReadImagePixel(imageNum, x, y);
    }
    else {
        fgcol = color & 0xffff;
    }
    if (gradON) {
        if (gradientVert) {
            fgcol = Grad(grad1[0 + gpos], grad1[1 + gpos], grad1[2 + gpos], grad1[3 + gpos], x - grad1[6 + gpos], fgcol);
        }
        else {
            fgcol = Grad(grad1[0 + gpos], grad1[1 + gpos], grad1[2 + gpos], grad1[3 + gpos], y - grad1[6 + gpos], fgcol);
        }
    }
    if (frame_buffer == CANVAS_BUFFER_ARGB){
		PutPixelARGB(x, y, fgcol, alpha);
	} else {
		bg = ReadPixel(x, y); //swin = true;
		calcAlpha(fgcol, bg, (uint8_t)alpha);
		PutPixel(x, y, __colour);
	}
}

/****************************************************************************/
/*!
  @brief  Sets the behaviour of how drawing with alpha blends with existing drawn area.
  @param  bl - ALPHA_BLEND or ALPHA_OVERWRITE
  @note once set, drawing to the CANVAS_BUFFER_ARGB will decide how color with alpha
        is handled. ALPHA_BLEND sets it so the highest alpha of the existing pixel or
		the new pixel will be the level set. ALPHA_OVERWRITE takes the new pixel alpha
		as the level set.
*/
/****************************************************************************/
void gfx4desp32P4::BlendAlphaBufferLevel(bool bl){
	aBlendType = bl;
}

/****************************************************************************/
/*!
  @brief  Set the frame buffer for drawing functions.
  @param  fbnum - frame buffer number 0 is main 1 to 3 are additional buffers
  @note once set, all drawing functions will be sent to specified frame
    buffer. If frame buffer 0 is set (default) all drawing functions will
    appear immediately on the display.
*/
/****************************************************************************/
void gfx4desp32P4::DrawToframebuffer(uint8_t fbnum) {
    if (rotation > 1){
		if (fbnum >= CANVAS_BUFFER && fbnum <= CANVAS_BUFFER3){
			width = altst_vres;
			height = altst_hres;
			__scrWidth = altst_hres << 1;
			__scrHeight = altst_vres;
			__fbSize = (width * height) << 1;
			ppaAccelerator.setPPAframebufferDimension(altst_hres, altst_vres);
		} else if (fbnum < CANVAS_BUFFER){
			width = st_vres;
			height = st_hres;
			__scrWidth = st_hres << 1;
			__scrHeight = st_vres;
			__fbSize = (width * height) << 1;
			ppaAccelerator.setPPAframebufferDimension(st_hres, st_vres);
		} else if (fbnum == CANVAS_BUFFER_ARGB){
			width = altst_vresARGB;
			height = altst_hresARGB;
			__scrWidth = altst_hresARGB << 2;
			__scrHeight = altst_vresARGB;
			__fbSize = (width * height) << 2;
			ppaAccelerator.setPPAframebufferDimension(altst_hresARGB, altst_vresARGB);
		} else {
			return;
		}
		__width = width;
		__height = height;
	} else {
		if (fbnum >= CANVAS_BUFFER && fbnum <= CANVAS_BUFFER3){
			width = altst_hres;
			height = altst_vres;
			__scrWidth = width << 1;
			__fbSize = (width * height) << 1;
			ppaAccelerator.setPPAframebufferDimension(altst_hres, altst_vres);
		} else if (fbnum < CANVAS_BUFFER){
			width = st_hres;
			height = st_vres;
			__scrWidth = width << 1;
			__fbSize = (width * height) << 1;
			ppaAccelerator.setPPAframebufferDimension(st_hres, st_vres);
		} else if (fbnum == CANVAS_BUFFER_ARGB){
			width = altst_hresARGB;
			height = altst_vresARGB;
			__scrWidth = width << 2;
			__fbSize = (width * height) << 2;
			ppaAccelerator.setPPAframebufferDimension(altst_hresARGB, altst_vresARGB);
		} else {
			return;
		}
		__scrHeight = height;
		__width = width;
		__height = height;
	}
	switch (fbnum) {
    case 0:
        frame_buffer = 0;
        break;
    case 1:
        frame_buffer = 1;
        if (!framebufferInit1)
            AllocateFB(1);
        break;
    case 2:
        frame_buffer = 2;
        if (!framebufferInit2)
            AllocateFB(2);
        break;
    case 3:
        frame_buffer = 3;
        if (!framebufferInit3)
            AllocateFB(3);
        break;
    case 4:
        frame_buffer = 4;
        if (!framebufferInit4)
            AllocateFB(4);
        break;
	case 5:
        frame_buffer = 5;
        if (!framebufferInit5)
            AllocateFB(5);
        break;
	case 6:
        frame_buffer = 6;
        if (!framebufferInit6)
            AllocateFB(6);
        break;
	case 7:
        frame_buffer = 7;
        if (!framebufferInit7)
            AllocateFB(7);
        break;
	case 8:
        frame_buffer = 8;
        if (!framebufferInit8)
            AllocateFB(8);
        break;
	case 9:
        frame_buffer = 9;
        if (!framebufferInit9)
            AllocateFB(9);
        break;
	case 10:
        frame_buffer = 10;
        if (!framebufferInit10)
            AllocateFB(10);
        break;
	case 11:
        frame_buffer = 11;
        if (!framebufferInit11)
            AllocateFB(11);
        break;
	case WIDGET_BUFFER:
        frame_buffer = WIDGET_BUFFER;
        if (!framebufferInit12)
            AllocateFB(WIDGET_BUFFER);
        break;
	case RGB888_BUFFER:
        frame_buffer = RGB888_BUFFER;
        if (!framebufferInit13)
            AllocateFB(RGB888_BUFFER);
        break;
	case CANVAS_BUFFER:
		frame_buffer = CANVAS_BUFFER;
		if (!framebufferInit14)
			AllocateFB(CANVAS_BUFFER);
		break;
	case CANVAS_BUFFER1:
		frame_buffer = CANVAS_BUFFER1;
		if (!framebufferInit15)
			AllocateFB(CANVAS_BUFFER1);
		break;
	case CANVAS_BUFFER2:
		frame_buffer = CANVAS_BUFFER2;
		if (!framebufferInit16)
			AllocateFB(CANVAS_BUFFER2);
		break;
	case CANVAS_BUFFER3:
		frame_buffer = CANVAS_BUFFER3;
		if (!framebufferInit17)
			AllocateFB(CANVAS_BUFFER3);
		break;
	case CANVAS_BUFFER_ARGB:
		frame_buffer = CANVAS_BUFFER_ARGB;
		if (!framebufferInit18)
			AllocateFB(CANVAS_BUFFER_ARGB);
		break;
	}
	if (!clippingON){
		clipx1 = 0; clipy1 = 0; clipx2 = __width -1; clipy2 = __height - 1;
	}
}

bool gfx4desp32P4::SpriteInit(uint16_t* sdata, size_t nums) {
    if (msprites < 1)
        return false;
    uint16_t scount = 0;
    int sdatpos = 0;
    int sprsize = 0;
    uint16_t cdpth = 1;
    uint16_t nextpos = 4;
    while (scount <= nums && cdpth > 0) {
        spriteData[sdatpos] = sdata[scount];
        spriteData[sdatpos + 1] = sdata[scount + 1];
        spriteData[sdatpos + 2] = sdata[scount + 2];
        cdpth = sdata[scount + 3];
        spriteData[sdatpos + 3] = nextpos;
        sprsize = (sdata[scount + 1] * sdata[scount + 2]) >> (cdpth - 1);
        if (cdpth == SPRITE_8BIT &&
            ((sdata[scount + 1] * sdata[scount + 2]) % 2) > 0)
            sprsize++;
        if (cdpth == SPRITE_4BIT &&
            ((sdata[scount + 1] * sdata[scount + 2]) % 4) > 0)
            sprsize++;
        nextpos = nextpos + sprsize + 4;
        sdatpos += 4;
        scount = nextpos - 4;
    }
    return true;
}

bool gfx4desp32P4::SpriteAdd(int pos, int snum, int x, int y, uint16_t* sdata) {
    if (snum > msprites)
        return false;
    int spos = spriteData[(snum << 2) + 3];
    byte coldepth = sdata[spos - 1];
    spriteList[(pos << 3)] = 0;
    spriteList[(pos << 3)] |= (coldepth << 1);
    spriteList[(pos << 3) + SPRITE_MEMPOS] = spos;
    spriteList[(pos << 3) + SPRITE_X] = x;
    spriteList[(pos << 3) + SPRITE_Y] = y;
    spriteList[(pos << 3) + SPRITE_WIDTH] = sdata[spos - 3];
    spriteList[(pos << 3) + SPRITE_HEIGHT] = sdata[spos - 2];
    spriteLast[pos << 1] = x;
    spriteLast[(pos << 1) + 1] = y;
    spriteNum[pos] = snum;
    if (pos + 1 > numSprites)
        numSprites = pos + 1;
    return true;
}

void gfx4desp32P4::SpriteAreaSet(uint16_t x, uint16_t y, uint16_t x1,
    uint16_t y1) {
    spriteArea[0] = x;
    spriteArea[1] = y;
    spriteArea[2] = x1;
    spriteArea[3] = y1;
    saSet = true;
    if (SpriteBKGfbNUM) DrawFrameBufferArea(SpriteBKGfbNUM, x, y, x1, y1);
}

void gfx4desp32P4::SpriteUseFrameBufferBackground(int fbnum) {
    SpriteBKGfbNUM = fbnum;
}

void gfx4desp32P4::SetSprite(int num, bool active, int x, int y, uint16_t bscolor,
    uint16_t* sdata) {
    bool delsprite = false;
    int lxy = num << 1;
    int dxy = num << 3;
    int dsx = spriteList[dxy + SPRITE_X];
    int dsy = spriteList[dxy + SPRITE_Y];
    if (spriteList[(num << 3) + 0] == 1 && active == 0) {
        delsprite = true;
    }
    if (spriteList[dxy] == 0) {
        spriteLast[lxy] = x;
        spriteLast[lxy + 1] = y;
    }
    spriteList[dxy] |= active;
    spriteList[dxy + SPRITE_X] = x;
    spriteList[dxy + SPRITE_Y] = y;
    int16_t tsx, tsy, tsx1, tsy1;
    if (!delsprite) {
        if (spriteLast[lxy] <= x) {
            tsx = spriteLast[lxy];
            tsx1 = x + spriteList[dxy + SPRITE_WIDTH];
        }
        else {
            tsx1 = spriteLast[lxy] + spriteList[dxy + SPRITE_WIDTH];
            tsx = x;
        }
        if (spriteLast[lxy + 1] <= y) {
            tsy = spriteLast[lxy + 1];
            tsy1 = y + spriteList[dxy + SPRITE_HEIGHT];
        }
        else {
            tsy1 = spriteLast[lxy + 1] + spriteList[dxy + SPRITE_HEIGHT];
            tsy = y;
        }
    }
    else {
        tsx = dsx;
        tsy = dsy;
        tsx1 = dsx + spriteList[dxy + SPRITE_WIDTH];
        tsy1 = dsy + spriteList[dxy + SPRITE_HEIGHT];
    }
    spriteLast[lxy] = x;
    spriteLast[lxy + 1] = y;
    if ((((tsx1 - tsx) >> 1) + 1 > spriteList[dxy + SPRITE_WIDTH] ||
        ((tsy1 - tsy) >> 1) + 1 > spriteList[dxy + SPRITE_HEIGHT]) &&
        spriteList[dxy + SPRITE_ACTIVE]) {
        spriteList[dxy + SPRITE_ACTIVE] |= 0;
        SpriteUpdate(dsx, dsy, dsx + spriteList[dxy + SPRITE_WIDTH],
            dsy + spriteList[dxy + SPRITE_HEIGHT], bscolor, sdata);
        spriteList[dxy + SPRITE_ACTIVE] |= 1;
        SpriteUpdate(x, y, x + spriteList[dxy + SPRITE_WIDTH],
            y + spriteList[dxy + SPRITE_HEIGHT], bscolor, sdata);
    }
    else {
        SpriteUpdate(tsx, tsy, tsx1, tsy1, bscolor, sdata);
    }
}

void gfx4desp32P4::SpriteUpdate(int16_t tsx, int16_t tsy, int16_t tsx1,
    int16_t tsy1, uint16_t bscolor, uint16_t* sdata) {
    saSet = true;
    if (tsx >= width || tsy >= height || tsx1 < 1 || tsy1 < 1)
        return;
    if (tsx < 0)
        tsx = 0;
    if (tsy < 0)
        tsy = 0;
    if (tsx1 >= width)
        tsx1 = width - 1;
    if (tsy1 >= height)
        tsy1 = height - 1;
    spriteArea[0] = tsx;
    spriteArea[1] = tsy;
    spriteArea[2] = tsx1;
    spriteArea[3] = tsy1;
    UpdateSprites(bscolor, sdata);
}

int gfx4desp32P4::GetSprite(int snum, int choice) {
    return spriteList[(snum << 3) + choice];
}

void gfx4desp32P4::UpdateSprites(uint16_t bscolor, uint16_t* sdata) {
    if (!saSet)
        return;
    spriteArea[3] = spriteArea[3] + 1;
    SetGRAM(spriteArea[0], spriteArea[1], spriteArea[2], spriteArea[3]);
    bool needsStartWrite = StartWrite();

    byte cdepth;
    uint16_t sspos = 0;
    int collide;
    int addr;
    int xo, yo;

    uint16_t wscolor = 0;
    uint16_t cscolor = 0;
    uint16_t tscolor = 0;
    uint16_t bufsp[spriteArea[2] + 20];
    uint16_t count = 0;
    int spriteAPos;
    for (int ns = 0; ns < numSprites; ns++) {
        spriteList[(ns << 3) + 6] = -1;
        spriteList[(ns << 3) + 7] = -1;
    }
    collide = -1;
    for (int y = 0; y < spriteArea[3] - spriteArea[1] + 1; y++) {
        count = 0;
        for (int x = 0; x < spriteArea[2] - spriteArea[0] + 1; x++) {
            if (SpriteBKGfbNUM) {
                wscolor = ReadPixelFromFrameBuffer(spriteArea[0] + x, spriteArea[1] + y, SpriteBKGfbNUM);
            }
            else {
                wscolor = bscolor;
            }
            collide = -1;
            for (int chk = 0; chk < numSprites; chk++) {
                xo = spriteArea[0] + x;
                yo = spriteArea[1] + y;
                addr = chk << 3;
                cdepth = spriteList[addr] >> 1;
                spriteAPos = spriteList[addr + SPRITE_MEMPOS];
                sdetaily = spriteList[addr + SPRITE_Y];
                sdetailh = spriteList[addr + SPRITE_HEIGHT];
                sdetailx = spriteList[addr + SPRITE_X];
                sdetailw = spriteList[addr + SPRITE_WIDTH];
                if ((spriteList[addr] & 0x01) && yo >= sdetaily &&
                    yo <= sdetaily + sdetailh - 1 && xo >= sdetailx &&
                    xo <= sdetailx + sdetailw - 1) {
                    tscolor = sdata[spriteAPos - 4];
                    sspos = ((yo - sdetaily) * sdetailw) + (xo - sdetailx);
                    if (cdepth == SPRITE_16BIT)
                        cscolor = sdata[spriteAPos + sspos];
                    if (cdepth == SPRITE_8BIT) {
                        uint16_t tcscol = sdata[spriteAPos + (sspos >> 1)];
                        cscolor = RGB3322565[(tcscol >> 8 * ((sspos % 2) == 0)) & 0xff];
                    }
                    if (cdepth == SPRITE_4BIT) {
                        uint16_t tcscol = sdata[spriteAPos + (sspos >> 2)];
                        cscolor = palette4bit[(tcscol >> ((3 - (sspos % 4)) * 4)) & 0x0f];
                    }
                    if (cscolor != tscolor) {
                        wscolor = cscolor;
                        if (collide == -1) {
                            collide = chk;
                        }
                        else if (collide != -1) {
                            if (spriteList[(collide << 3) + SPRITE_COLLIDE1] == -1) {
                                spriteList[(collide << 3) + SPRITE_COLLIDE1] = chk;
                                spriteList[addr + SPRITE_COLLIDE1] = collide;
                            }
                            else {
                                spriteList[(collide << 3) + SPRITE_COLLIDE2] = chk;
                                spriteList[addr + SPRITE_COLLIDE1] = collide;
                            }
                            collide = chk;
                        }
                    }
                }
            }
            bufsp[count] = wscolor;
            count++;
        }
        //uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
		//uint16_t* pto = (uint16_t*)bufsp;
		//spriteArea[0], spriteArea[1], spriteArea[2], spriteArea[3])
		//int w = spriteArea[2] - spriteArea[0] + 1;
		//int h = spriteArea[3] - spriteArea[1] + 1;
		//TranslateCoords(spriteArea[0], spriteArea[1], w, h, 0);
		//ppaAccelerator.scaleRotateImageFB(bufsp, w, h, 0, 0, w, h, tpto, w, h, false, _translated[TRANS_DEG], false, false, _translated[TRANS_X], _translated[TRANS_Y], 0, 0);
        WrGRAMs(bufsp, count);
    }
    if(needsStartWrite) EndWrite();
}

void gfx4desp32P4::SetNumberSprites(uint16_t numspr) { numSprites = numspr; }

int gfx4desp32P4::GetNumberSprites() { return numSprites; }

int16_t gfx4desp32P4::GetSpriteImageNum(int snum) { return spriteNum[snum]; }

uint16_t gfx4desp32P4::SpriteGetPixel(int snum, int xo, int yo, uint16_t tcolor,
    uint16_t* sdata) {
    uint16_t cscolor = 0x0000;
    uint16_t rcolor = tcolor;
    uint16_t sspos;
    byte cdepth;
    int chks, chke;
    if (snum < 0) {
        chks = 0;
        chke = numSprites;
    }
    else {
        chks = snum;
        chke = snum + 1;
    }
    for (int chk = chks; chk < chke; chk++) {
        uint16_t addr = chk << 3;
        cdepth = spriteList[addr] >> 1;
        if (spriteList[addr] && yo >= spriteList[addr + SPRITE_Y] &&
            yo <= spriteList[addr + SPRITE_Y] + spriteList[addr + SPRITE_HEIGHT] -
            1 &&
            xo >= spriteList[addr + SPRITE_X] &&
            xo <=
            spriteList[addr + SPRITE_X] + spriteList[addr + SPRITE_WIDTH] - 1) {
            sspos = ((yo - spriteList[addr + SPRITE_Y]) *
                spriteList[addr + SPRITE_WIDTH]) +
                (xo - spriteList[addr + SPRITE_X]);
            if (cdepth == SPRITE_16BIT)
                cscolor = sdata[spriteList[addr + SPRITE_MEMPOS] + sspos];
            if (cdepth == SPRITE_8BIT) {
                uint16_t tcscol =
                    sdata[spriteList[addr + SPRITE_MEMPOS] + (sspos >> 1)];
                cscolor = RGB3322565[(tcscol >> 8 * ((sspos % 2) == 0)) & 0xff];
            }
            if (cdepth == SPRITE_4BIT) {
                uint16_t tcscol =
                    sdata[spriteList[addr + SPRITE_MEMPOS] + (sspos >> 2)];
                cscolor = palette4bit[(tcscol >> ((3 - (sspos % 4)) * 4)) & 0x0f];
            }
            if (cscolor != tcolor)
                rcolor = cscolor;
        }
    }
    return rcolor;
}

int gfx4desp32P4::GetSpritesAt(int xo, int yo, uint16_t tcolor, int* slist,
    uint16_t* sdata) {
    uint16_t cscolor = 0x0000;
    uint16_t sspos;
    int snum = 0;
    int r = -1;
    byte cdepth;
    for (int chk = 0; chk < numSprites; chk++) {
        slist[chk] = -1;
        uint16_t addr = chk << 3;
        cdepth = spriteList[addr] >> 1;
        if (spriteList[addr] && yo >= spriteList[addr + SPRITE_Y] &&
            yo <= spriteList[addr + SPRITE_Y] + spriteList[addr + SPRITE_HEIGHT] -
            1 &&
            xo >= spriteList[addr + SPRITE_X] &&
            xo <=
            spriteList[addr + SPRITE_X] + spriteList[addr + SPRITE_WIDTH] - 1) {
            sspos = ((yo - spriteList[addr + SPRITE_Y]) *
                spriteList[addr + SPRITE_WIDTH]) +
                (xo - spriteList[addr + SPRITE_X]);
            if (cdepth == SPRITE_16BIT)
                cscolor = sdata[spriteList[addr + SPRITE_MEMPOS] + sspos];
            if (cdepth == SPRITE_8BIT) {
                uint16_t tcscol =
                    sdata[spriteList[addr + SPRITE_MEMPOS] + (sspos >> 1)];
                cscolor = RGB3322565[(tcscol >> 8 * ((sspos % 2) == 0)) & 0xff];
            }
            if (cdepth == SPRITE_4BIT) {
                uint16_t tcscol =
                    sdata[spriteList[addr + SPRITE_MEMPOS] + (sspos >> 2)];
                cscolor = palette4bit[(tcscol >> ((3 - (sspos % 4)) * 4)) & 0x0f];
            }
            if (cscolor != tcolor) {
                slist[snum] = chk;
                snum++;
                r = snum;
            }
        }
    }
    return r;
}

void gfx4desp32P4::SpriteEnable(int snum, bool sen) {
    spriteList[snum << 3] |= sen;
}

void gfx4desp32P4::SpriteSetPalette(int pnumber, uint16_t plcolor) {
    palette4bit[pnumber % 16] = plcolor;
}

uint16_t gfx4desp32P4::SpriteGetPalette(int pnumber) {
    return palette4bit[pnumber % 16];
}

void gfx4desp32P4::SetMaxNumberSprites(uint16_t snos) {
    snos = snos << 1;
    uint16_t bytes = (uint16_t)snos << 2;
    if ((spriteData = (int16_t*)malloc(bytes))) {
        memset(spriteData, 0, bytes);
    }
    bytes = (uint16_t)snos << 3;
    if ((spriteList = (int16_t*)malloc(bytes))) {
        memset(spriteList, 0, bytes);
    }
    bytes = (uint16_t)snos << 1;
    if ((spriteLast = (int16_t*)malloc(bytes))) {
        memset(spriteLast, 0, bytes);
    }
    bytes = (uint16_t)snos;
    if ((spriteNum = (int16_t*)malloc(bytes))) {
        memset(spriteNum, -1, bytes);
    }
    msprites = snos >> 1;
}

/*************************************************************************************************************/
/*!
  @brief  Force opacity over already set foreground background colors 
  @param  opacity - bool TRANSPARENT / OPAQUE
*/
/*************************************************************************************************************/
void gfx4desp32P4::Opacity(bool opacity) {
    if (opacity == false) {
        opacitystate = false;
    }
    else {
        opacitystate = true;
    }
}

/*************************************************************************************************************/
/*!
  @brief  Print a string at current cursor position 
  @param  strg - string to be written at XY position. Cursor pos is moved to end of string position
*/
/*************************************************************************************************************/
void gfx4desp32P4::putstr(String strg) { print(strg); }

/*************************************************************************************************************/
/*!
  @brief  Print a constant character string at current cursor position 
  @param  strg - character string to be written at XY position. Cursor pos is moved to end of string position
*/
/*************************************************************************************************************/
void gfx4desp32P4::putstr(const char* strg) { print(strg); }

/*************************************************************************************************************/
/*!
  @brief  Print a constant string at XY location 
  @param  xpos - X position of String
  @param  ypos - Y position of String
  @param  strg - string to be written at XY position. Cursor pos is moved to end of string position
*/
/*************************************************************************************************************/
void gfx4desp32P4::putstrXY(int xpos, int ypos, String strg) {
    MoveTo(xpos, ypos);
    print(strg);
}

/*************************************************************************************************************/
/*!
  @brief  Print a constant character string at XY location 
  @param  xpos - X position of String
  @param  ypos - Y position of String
  @param  strg - character string to be written at XY position. Cursor pos is moved to end of string position
*/
/*************************************************************************************************************/
void gfx4desp32P4::putstrXY(int xpos, int ypos, const char* strg) {
    MoveTo(xpos, ypos);
    print(strg);
}

/*************************************************************************************************************/
/*!
  @brief  Write a character at current cursor position 
  @param  chr - character to be written at XY position cursor position is moved to end of chracter position
*/
/*************************************************************************************************************/
void gfx4desp32P4::putch(char chr) { write(chr); }

/*************************************************************************************************************/
/*!
  @brief  Write a character at XY location 
  @param  xpos - X position of character
  @param  ypos - Y position of character
  @param  chr - character to be written at XY position cursor position is moved to end of chracter position
*/
/*************************************************************************************************************/
void gfx4desp32P4::putchXY(int xpos, int ypos, char chr) {
    MoveTo(xpos, ypos);
    write(chr);
}

/*************************************************************************************************************/
/*!
  @brief  Print a centered string at XY location 
  @param  xpos - X as axis for center of printed string
  @param  ypos - Y as axis for center of printed string
  @param  strg - string to be printed. Cursor is moved to end of centered text
*/
/*************************************************************************************************************/
void gfx4desp32P4::putstrCenteredXY(int xpos, int ypos, String strg) {
	MoveTo(xpos - (strWidth(strg) >> 1) + 1, ypos - (fsh >> 1));
    print(strg);
}

/*************************************************************************************************************/
/*!
  @brief  Print a centered string at a angular location 
  @param  xpos - X as axis for angular location
  @param  ypos - Y as axis for angular location
  @param  ang - angle 0 to 359
  @param  radius - radius from XY axis
  @param  strg - string to be printed. Cursor is moved to end of centered text
*/
/*************************************************************************************************************/
void gfx4desp32P4::putstrAngularXY(int xpos, int ypos, int ang, int radius, String strg) {
	int curX = cursor_x;
	int curY = cursor_y;
	int xy1[2];
	MoveTo(xpos, ypos);
	Orbit(ang, radius, xy1);
	MoveTo((int)xy1[0] - (strWidth(strg) >> 1), (int)xy1[1] - (fsh >> 1));
    print(strg);
	cursor_x = curX;
	cursor_y = curY;
}

/*************************************************************************************************************/
/*!
  @brief  Draw a single rolling digit 
  @param  xpos - X as left position of rolling digit
  @param  ypos - Y as top  position of rolling digit
  @param  width - width of rolling digit
  @param  height - height of rolling digit
  @param  val - position of the list
  @param  mul - the multiple of the val for smooth rolling
  @param  shadow - draws and alpha darkened gradient top and bottom 0 - 255
  @param  highlight - draws and alpha lightened gradient from centre to top and bottom 0 - 255
  @param  bkgcol - background colour of rolling digit  Text color and font set externally
*/
/*************************************************************************************************************/
void gfx4desp32P4::RollingDigit(int xpos, int ypos, int width, int height, int32_t val, int32_t mul, int shadow, int highlight, uint16_t bkgcol, int maxNum) {
	int sh = fsh * textsize;
	int gap;
	height -= 1;
	gap = height - sh;
	int tfb = frame_buffer;
	bool tw = wrap;
	bool senbl = sEnable;
	int len = 10;
	if (maxNum != 10) len = maxNum;
	float scrollPos;
	StoreCursPos();
	TextWrap(false);
	ScrollEnable(false);	
	DrawToframebuffer(WIDGET_BUFFER);
    int hh = sh + gap;
	int oh = (hh * len);
	scrollPos = ((float)oh / (float)len / mul) * val;
	if (val > (len * mul)) val = val % (len * mul);
	RectangleFilled(0, 0, width -1, (hh * 3) + (gap >> 1), bkgcol);
	int starty = (gap >> 1);
	int strtStr = (val / mul);
	int list[4];
	if (strtStr >= len - 1){
		list[0] = strtStr - 1;
		list[1] = strtStr + 0;
		list[2] = 0;
	} else {
		list[0] = strtStr - 1;
		list[1] = strtStr;
		list[2] = strtStr + 1;
	}
	for(int n = 0; n < 3; n++){
		if (list[n] >= 0) putstrCenteredXY((width >> 1) + 1, starty + (sh >> 1), String(list[n]));
		starty += hh;
	}
	int pos = scrollPos - (strtStr * (hh)) + (gap >> 1);
	int segh = hh;	
    DrawFrameBufferAreaXY(WIDGET_BUFFER, 0, pos + hh, width -1, pos + hh + segh, width, 0);
	if (shadow || highlight){
		int ab;
		if (shadow) ab = shadow / (hh >> 1);
		if (highlight) ab = highlight / (hh >> 1);
		AlphaBlend(ON);
		for (int n = 0; n < hh >> 1; n++){
		    AlphaBlendLevel(shadow - (n * ab));
			if (highlight){
			  Hline(width, (hh >> 1) - n - 1, width, WHITE);
              Hline(width, (hh >> 1) + n, width, WHITE);
            }
            if (shadow){
			  Hline(width, n - 1, width, BLACK);
              Hline(width, hh - n, width, BLACK);
            }			
		}
		AlphaBlend(OFF);
	}
    DrawToframebuffer(tfb);
    DrawFrameBufferAreaXY(WIDGET_BUFFER, width, 0, width + width - 1, height, xpos, ypos);	
	ScrollEnable(senbl);
	TextWrap(tw);
	RestoreCursPos();
}

/*********************************************************************************************/
/*!
  @brief  Draw multi digit rolling counter
  @param  xpos - X as left position of counter
  @param  ypos - Y as top  position of counter
  @param  width - the total width of counter. digits size if calculated by num and gap
  @param  height - height of counter
  @param  digits - number of digits in the counter
  @param  val - the value to be shown. Always a multiple of 10 eg 126 = 1260 input val
  @param  gapH - gap between digits. 0 if none
  @param  shadow - draws and alpha darkened gradient top and bottom 0 - 255
  @param  highlight - draws and alpha lightened gradient from centre to top and bottom 0 - 255
  @param  bkgcol - background colour of rolling window. Text color and font set externally
  @param  InvLast - bool - inverts the last digits fore an backfround colours
*/
/*********************************************************************************************/
void gfx4desp32P4::RollingCounter(int xpos, int ypos, int width, int height, int digits, int32_t val, int gapH, int shadow, int highlight, uint16_t bkgcol, bool InvLast, int maxNumLeading) {
	int sh = fsh * textsize;
	int gap;
	int32_t mul = 10;
	int32_t dmul = mul;
	int32_t lastdmul = mul;
	uint16_t colf = textcolor;
    uint16_t colb = textbgcolor;
	TextColor(textcolor, textcolor);
	int dw = (width + gapH)/ digits;
	gap = height - sh - 1;
	int32_t tempval;
	int32_t tempmul;
	int32_t digit0inc9;
	int32_t digitVal[digits];
	bool lastDigit9;
	int dpos = xpos + ((digits - 1) * dw);
	int equals9 = 0;
	for (int n = 0; n < digits; n ++){
		digitVal[n] = val % (dmul * 10);
		tempmul = dmul;
		digit0inc9 = digitVal[0] - ((9 * mul));
		lastDigit9 = ((digitVal[0] / mul) == 9);
		if (n > 0){
			if ((digitVal[n - 1] / lastdmul) == 9 && lastDigit9 && equals9 == (n - 1)){
			    equals9 ++;
				tempmul = lastdmul;
				tempval = (digitVal[n] / dmul * lastdmul) + (digit0inc9 * lastdmul / 10);
			} else {
			    tempmul = 1;
				tempval = digitVal[n] / dmul;
            }			
			if(n == digits - 1){
			    RollingDigit(dpos, ypos, dw - gapH, height, tempval, tempmul, shadow, highlight, bkgcol, maxNumLeading);
			} else {
				RollingDigit(dpos, ypos, dw - gapH, height, tempval, tempmul, shadow, highlight, bkgcol);
			}
		} else {			
			if (InvLast){
			    TextColor(bkgcol, bkgcol);
				RollingDigit(dpos, ypos, dw - gapH, height, digitVal[n], dmul, shadow, highlight, colf);
		        TextColor(colf, colf);
			} else {
				RollingDigit(dpos, ypos, dw - gapH, height, digitVal[n], dmul, shadow, highlight, bkgcol);
			}
		}
		dpos -= dw;
		lastdmul = dmul;
		dmul *= 10;
	}
	TextColor(colf, colb);
}

/*************************************************************************************************************/
/*!
  @brief  Draw list of strings in Rolling Window
  @param  xpos - X as left position of rolling window
  @param  ypos - Y as top  position of rolling window
  @param  width - the total width of rolling window
  @param  height - height of rolling window
  @param  val - position of the list
  @param  mul - the multiple of the val for smooth rolling
  @param  strgList - list of pre defined strings
  @param  len - number of strings in the list
  @param  shadow - draws and alpha darkened gradient top and bottom 0 - 255
  @param  highlight - draws and alpha lightened gradient from centre to top and bottom 0 - 255
  @param  bkgcol - background colour of rolling window  Text color and font set externally
*/
/*************************************************************************************************************/
void gfx4desp32P4::RollingStrings(int xpos, int ypos, int width, int height, int32_t val, int32_t mul, String* strgList, int len, int shadow, int highlight, uint16_t bkgcol) {
	int sh = fsh * textsize;
	int gap;
	gap = height - sh;
	int tfb = frame_buffer;
	bool tw = wrap;
	bool senbl = sEnable;
	int dir = 1;
	float scrollPos;
	StoreCursPos();
	TextWrap(false);
	ScrollEnable(false);
    uint16_t colf = textcolor;
    uint16_t colb = textbgcolor;
	TextColor(textcolor, textcolor);	
	DrawToframebuffer(WIDGET_BUFFER);
    int hh = sh + gap;
	int oh = (hh * len);
	scrollPos = ((float)oh / (float)len / mul) * val;
	if (val > (len * mul)) val = val % (len * mul);
	RectangleFilled(0, 0, width -1, (hh * 3) + (gap >> 1), bkgcol);
	int starty = gap >> 1;
	int strtStr = (val / mul);
	int list[4];
	if (strtStr >= len - 1){
		list[0] = strtStr - 1;
		list[1] = strtStr + 0;
		list[2] = 0;
	} else {
		list[0] = strtStr - 1;
		list[1] = strtStr;
		list[2] = strtStr + 1;
	}
	for(int n = 0; n < 3; n++){
		if (list[n] >= 0) putstrCenteredXY(width >> 1, starty + (sh >> 1), strgList[list[n]]);
		starty += hh;
	}
	int pos = scrollPos - (strtStr * (hh)) + (gap >> 1);
	int segh = hh;	
    DrawFrameBufferAreaXY(WIDGET_BUFFER, 0, pos + hh, width -1, pos + hh + segh, width, 0);
	if (shadow || highlight){
		int ab = 127 / (hh >> 1);
		AlphaBlend(ON);
		for (int n = 0; n < hh >> 1; n++){
		    AlphaBlendLevel(shadow - (n * ab));
			if (highlight){
			  Hline(width, (hh >> 1) - n - 1, width, WHITE);
              Hline(width, (hh >> 1) + n, width, WHITE);
            }
            if (shadow){
			  Hline(width, n - 1, width, BLACK);
              Hline(width, hh - n, width, BLACK);
            }			
		}
		AlphaBlend(OFF);
	}
    DrawToframebuffer(tfb);
    DrawFrameBufferAreaXY(WIDGET_BUFFER, width, 0, width + width - 1, height, xpos, ypos);	
	ScrollEnable(senbl);
	TextWrap(tw);
	RestoreCursPos();
	TextColor(colf, colb);
}

/*************************************************************************************************************/
/*!
  @brief  Draw a horizontally scrolled string inside defined window area
  @param  xpos - X as left position of scrolled string window
  @param  ypos - Y as top  position of scrolled string window
  @param  width - the total width of scrolled string window, height is automatic
  @param  scrollPos - position start point in the string to show
  @param  strg - string to be shown in window
  @param  bkgcol - background colour of scroll window  Text color and font set externally
  @param  bkgBuff - background colour will be replaced with image in the same location at specified framebuffer
*/
/*************************************************************************************************************/
void gfx4desp32P4::putstrScrolledXYW(int xpos, int ypos, int width, int scrollPos, String strg, uint16_t bkgcol, int bkgBuff) {
	int sw = strWidth(strg);
	int sh = fsh * textsize;
	int strSz = sw - scrollPos;
	int tfb = frame_buffer;
	bool tw = wrap;
	bool senbl = sEnable;
	int cx1 = clipx1;
	int cy1 = clipy1;
	int cx2 = clipx2;
	int cy2 = clipy2; 
	int cx1b = clipx1pos;
	int cy1b = clipy1pos;
	int cx2b = clipx2pos;
	int cy2b = clipy2pos; 
    bool tclip = clippingON;	
	TextWrap(false);
	ScrollEnable(false);
	StoreCursPos();
	Clipping(OFF);
	ClipWindow(xpos, ypos, xpos + width - 1, ypos + sh - 1);
    Clipping(ON);	
	DrawToframebuffer(WIDGET_BUFFER);	
	if (bkgBuff != -1){
		if (bkgBuff  > 9) bkgBuff = 9;
		DrawFrameBufferAreaPPA(bkgBuff, xpos, ypos, xpos + width - 1, ypos + sh - 1); 
	} else {
		RectangleFilledPPA(xpos, ypos, xpos + width - 1, ypos + sh - 1, bkgcol);
	}
	scrollPos = scrollPos % sw; 
	MoveTo(xpos - scrollPos, ypos);
	print(strg);
	if (strSz < width) print(strg);
    DrawToframebuffer(tfb);
    DrawFrameBufferAreaPPA(WIDGET_BUFFER, xpos, ypos, xpos + width - 1, ypos + sh - 1);
    Clipping(OFF);	
	ScrollEnable(senbl);
	TextWrap(tw);
	clipx1 = cx1;
	clipy1 = cy1;
	clipx2 = cx2;
	clipy2 = cy2;
	clipx1pos = cx1b;
	clipy1pos = cy1b;
	clipx2pos = cx2b;
	clipy2pos = cy2b; 
    if (tclip) Clipping(tclip);	
	RestoreCursPos();
}

/****************************************************************************/
/*!
  @brief  Set the frame buffer source of image data for some drawing operations
  @param  fb - frame buffer number to draw the image data from in the x y aligned area
  @returns value used by drawing function
*/
/****************************************************************************/
uint32_t gfx4desp32P4::SelectDataSourceFB(int fb){
	return FRAMEBUFFER_IMAGE + fb;
}

/****************************************************************************/
/*!
  @brief  Set the GCI image index source of image data for some drawing operations
  @param  gcImage - Image index to draw the image data from in the x y aligned area
  @returns value used by drawing function
*/
/****************************************************************************/
uint32_t gfx4desp32P4::SelectDataSourceGCI(int gcImage){
	return GCI_IMAGE + gcImage;
}

/****************************************************************************/
/*!
  @brief  Set the GCI image index and frame source of image data for some drawing operations
  @param  gcImage - Image index to draw the image data from in the x y aligned area
  @param  frame - frame number in image set
  @returns value used by drawing function
*/
/****************************************************************************/
uint32_t gfx4desp32P4::SelectDataSourceGCI(int gcImage, int frame){
	imageSetWord(gcImage, IMAGE_INDEX, frame);
	return GCI_IMAGE + gcImage;
}

int gfx4desp32P4::__gciCharWidth(uint16_t ch) {
    if (fno == 0) {
        gciFont.seek(ch * fsb + 8); // character offset 
        // (number of bytes per character * character value) +  8-byte header
        return textsize * (gciFont.read() << 8 | gciFont.read());
    }
    else if (fno == -1) {
        if (fntCmprs) {
            if (fontPtr[FONT_TYPE]) {
                return textsize * fntData[(ch - fso) * fsb];
            }
            else {
                return textsize * fsw;
            }
        }
        else {
            int i = ch * fsb + 8;
            return textsize * (fontPtr[i] << 8 | fontPtr[i + 1]);
        }
    }
    return textsize * fsw;
}

int gfx4desp32P4::charWidth(uint16_t ch) {
    if (fno != 0 && fno != -1)
        return (fsw + 1) * textsize;
    return __gciCharWidth(ch);
}

int gfx4desp32P4::charHeight(uint16_t ch) { return fsh * textsize; }

int gfx4desp32P4::strWidth(String ts) {
    // size_t len = ts.length();
    // if (fno != 0 && fno != -1)
    //     return len * ((fsw + 1) * textsize);
    // int width = 0;
    // for (size_t i = 0; i < len; i++) {
    //     width += __gciCharWidth(ts.charAt(i));
    // }
    // return width;
    return strWidth(ts.c_str());
}

int gfx4desp32P4::strWidth(const char* ts) {

    // Serial0.printf("Getting width of \"%s\"\n", ts);

    size_t len = strlen(ts);
    if (fno != 0 && fno != -1)
        return len * ((fsw + 1) * textsize);
    int width = 0;
    uint32_t utf8cp = 0;
    uint8_t utf8len = 0;


    for (size_t i = 0; i < len; i++) {

        char c = ts[i];

        uint16_t u16chr;
        // First we build the Utf8 character
        if (utf8len) {
            // If we already started building the utf-8, we continue
            // we can check this by checking expected length
            if ((c & 0xC0) != 0x80) {
                // Invalid UTF-8 sequence, handle error or ignore
                utf8len = 0;
                utf8cp = 0;
                // return 0; // Indicate failure
                continue;
            }
            utf8cp = (utf8cp << 6) | (c & 0x3F);
            utf8len--;
            if (utf8len != 0) continue; // not yet complete
            u16chr = static_cast<uint16_t>(utf8cp);
        }
        else {
            // Otherwise, let's figure out how many bytes to expect
            if ((c & 0x80) == 0) {
                // If the character is ASCII, directly write its Unicode value
                u16chr = static_cast<uint16_t>(c);
            }
            else if ((c & 0xE0) == 0xC0) {
                utf8cp = c & 0x1F;
                utf8len = 1;
                continue;
            }
            else if ((c & 0xF0) == 0xE0) {
                utf8cp = c & 0x0F;
                utf8len = 2;
                continue;
            }
            else if ((c & 0xF8) == 0xF0) {
                utf8cp = c & 0x07;
                utf8len = 3;
                continue;
            }
            else {
                // Invalid UTF-8 sequence, handle error or ignore
                // return 0; // Indicate failure
                continue;
            }
        }

        // draw character here
        width += __gciCharWidth(u16chr);
    }
    return width;
}
void gfx4desp32P4::imageSetWord(uint16_t img, byte controlIndex, int val1, int val2) {
    if(img > (gciobjnum - 1)) return;
	switch (controlIndex) {
    case IMAGE_XPOS:
        tuix[img] = val1;
        break;
    case IMAGE_YPOS:
        tuiy[img] = val1;
        break;
    case IMAGE_XYPOS:
        tuix[img] = val1;
        tuiy[img] = val2;
        break;
    case IMAGE_INDEX:
        tuiImageIndex[img] = val1;
        break;
	case IMAGE_ROTATE:
		tuiRM[img] = (tuiRM[img] & 0xf0) + ((val1 / 90) & 0x03); 
		break;
	case IMAGE_MIRROR_X:
		tuiRM[img] = (tuiRM[img] & 0xef) + ((val1 & 0x1) << 4); 
		break;
	case IMAGE_MIRROR_Y:
	    tuiRM[img] = (tuiRM[img] & 0xdf) + ((val1 & 0x1) << 5); 
		break;
	case IMAGE_SCALE_WIDTH:
	    if (val1 > __width) val1 = __width;
		tuScaleXY[img] = (tuScaleXY[img] & 0xffff00000) + val1;
		break;
	case IMAGE_SCALE_HEIGHT:
		if (val1 > __height) val1 = __height;
		tuScaleXY[img] = (tuScaleXY[img] & 0x0000ffff) + (val1 << 16);
		break;
	case IMAGE_SCALE_WIDTH_AND_HEIGHT:
		if (val1 > __width) val1 = __width;
		if (val2 > __height) val2 = __height;
		tuScaleXY[img] = val1 + (val2 << 16);
		break;
    }
}

int gfx4desp32P4::imageGetWord(uint16_t img, byte controlIndex) {
    int retval = -1;
	if (img > (gciobjnum - 1)) return -1;
    switch (controlIndex) {
    case IMAGE_XPOS:
        retval = tuix[img];
        break;
    case IMAGE_YPOS:
        retval = tuiy[img];
        break;
    case IMAGE_WIDTH:
        retval = tuiw[img];
        break;
    case IMAGE_HEIGHT:
        retval = tuih[img];
        break;
    case IMAGE_INDEX:
        retval = tuiImageIndex[img];
        break;
	case IMAGE_ROTATE:
		retval = (tuiRM[img] & 0x03) * 90; 
		break;
	case IMAGE_MIRROR_X:
		retval = (tuiRM[img] & 0x10) >> 4; 
		break;
	case IMAGE_MIRROR_Y:
		retval= (tuiRM[img] & 0x20) >> 5; 
		break;
	case IMAGE_SCALE_WIDTH:
	    retval = tuScaleXY[img] & 0x0000ffff;
		break;
	case IMAGE_SCALE_HEIGHT:
		retval = (tuScaleXY[img] & 0xffff0000) >> 16;
		break;
	case IMAGE_FRAMES:
		retval = gciobjframes[img];
		break;
    }
    return retval;
}

int16_t gfx4desp32P4::getImageValue(uint16_t ui) { return tuiImageIndex[ui]; }

#ifndef DISABLE_WIFI_FUNCTIONS
void gfx4desp32P4::DownloadFile(String WebAddr, String Fname) {
    Download(WebAddr, 0, "", Fname, false, empty);
}

void gfx4desp32P4::DownloadFile(String WebAddr, String Fname, const char* sha1) {
    Download(WebAddr, 0, "", Fname, true, sha1);
}

void gfx4desp32P4::DownloadFile(String Address, uint16_t port, String hfile,
    String Fname) {
    Download(Address, port, hfile, Fname, false, empty);
}

void gfx4desp32P4::Download(String Address, uint16_t port, String hfile,
    String Fname, bool certUsed, const char* sha1) {
    bool error = false;
    int16_t errornum = 0;
    int8_t retries = 3;
    dlok = false;
    int32_t lens;
    uint8_t buffDL[512] = { 0 };
    File Dwnload;
    HTTPClient http;
    WiFiClient client;
    WiFiClientSecure clientS;
    WiFiClient* stream;
    while (retries--) {
        if (port > 0) {
            http.begin(client, Address, port, hfile);
        }
        else {
            if (!certUsed) {
                http.begin(client, Address);
            }
            else {
                clientS.setCACert(sha1);
                http.begin(clientS, Address);
            }
        }
        int httpCode = http.GET();
        if (httpCode == 404 || httpCode == 403) {
            errornum = httpCode;
            error = true;
            goto skipDownload;
        }
        if (sdok == false) {
            errornum = -1;
            error = true;
            goto skipDownload;
        }
        lens = http.getSize();
        if (lens == 0) {
            errornum = -2;
            error = true;
            goto skipDownload;
        }
        if (SD_MMC.exists((char*)Fname.c_str())) {
            SD_MMC.remove((char*)Fname.c_str());
        }
        Dwnload = SD_MMC.open((char*)Fname.c_str(), FILE_WRITE);
        stream = http.getStreamPtr();
        while (http.connected() && (lens > 0 || lens == -1)) {
            size_t size = stream->available();
            if (size) {
                int cstream = stream->readBytes(
                    buffDL, ((size > sizeof(buffDL)) ? sizeof(buffDL) : size));
                if (!Dwnload.write(buffDL, cstream)) {
                    delay(500);
                    error = true;
                    break;
                }
                if (lens > 0) {
                    lens -= cstream;
                }
            }
        }
        http.end();
        if (Dwnload.size() == 0) {
            errornum = -3;
            error = true;
            goto skipDownload;
        }
    skipDownload:
        if (!error) {
            break;
        }
        http.end();
        clientS.stop();
        delay(500);
        lens = 0;
    }
    if (error) {
        if (errornum > 0) {
            print("HTTP error ");
            println(errornum);
        }
        else {
            if (errornum == -1) {
                println("SD Error");
            }
            if (errornum == -2) {
                println("Size Error");
            }
            if (errornum == -3) {
                println("0 Bytes Rec Error");
            }
            Dwnload.close();
            if (SD_MMC.exists((char*)Fname.c_str())) {
                SD_MMC.remove((char*)Fname.c_str());
            }
            delay(500);
            dlok = false;
        }
    }
    else {
        Dwnload.close();
        dlok = true;
    }
    http.end();
    return;
}
#endif

void gfx4desp32P4::PrintImageFile(String ifile) {
    if (cursor_x >= textXmax /*(width - 1)*/)
        return;
    boolean tempnl = false;
	bool jpegGC = false;
    if (nl) {
        tempnl = true;
        newLine(lastfsh, textsizeht, textXmin);
    }
    if (cursor_y > (height - 1) && (sEnable == false))
        return;
	String fnTemp = ifile;
    fnTemp.toUpperCase();
	ifile = "/" + ifile;
	dataFile = SD_MMC.open((char*)ifile.c_str());
	if (!dataFile)
		return;
	if (DATAsize() < 6)
		return;
	uint8_t mul;
	uint32_t pos;
	uint16_t ichunk;
	if (fnTemp.indexOf(".JPG") == (fnTemp.length() - 4) || fnTemp.indexOf(".JPEG") == (fnTemp.length() - 5)) {
		int dmy = DecodeJPEGfromFile(dataFile, 0, 0); // may be padded out
		delay(1);
		if (dmy == -1) return;
		_lastFrame = -1; _lastObject = -1;
		jpegGC = true;
		pos = 0;
		Piwidth = JPEGiSize[0];
		Piheight = JPEGiSize[1];
	} else {
		Piwidth = (DATAread() << 8);
		Piwidth = Piwidth + DATAread();
		Piheight = (DATAread() << 8);
		Piheight = Piheight + DATAread();
		mul = DATAread() / 8;
		DATAread(); // dummy read to move pointer
		pos = 6;
		ichunk = Piwidth << (mul - 1);
	}
    if (sEnable == false) {
        if (((cursor_y + Piheight) - 1) > height - 1) {
            Piheight = Piheight - ((cursor_y + Piheight) - height);
        }
    }
    boolean off = false;
    int cuiw = Piwidth;
    if ((cursor_x + Piwidth - 1) >= width) {
        cuiw = Piwidth - ((cursor_x + Piwidth - 1) - width) - 1;
        off = true;
    }
    for (int idraw = 0; idraw < Piheight; idraw++) {
        nl = true;
        newLine(1, 1, cursor_x);
        if ((cursor_y - 1) < 0) {
            setGRAM(cursor_x, cursor_y + height - 1, cursor_x + cuiw - 1,
                cursor_y + height - 1);
        }
        else {
            setGRAM(cursor_x, cursor_y - 1, cursor_x + cuiw - 1, cursor_y - 1);
        }
        if (off) {
            if (jpegGC){
				WrGRAMs((uint16_t*)rx_buf + pos, cuiw << 0);
				pos = pos + (Piwidth << 0);
			} else {
				DATAread(buff, cuiw << (mul - 1));
				WrGRAMs(buff, cuiw << (mul - 1));
				pos = pos + (Piwidth << (mul - 1));
				DATAseek(pos);
			}
        }
        else {
            if (jpegGC){
				WrGRAMs((uint16_t*)rx_buf + pos, Piwidth);
				pos += (Piwidth << 0);
			} else {
				DATAread(buff, ichunk);
				WrGRAMs(buff, ichunk);
			}
        }
    }
    if (tempnl) {
        nl = true;
        lastfsh = 1;
    }
	dataFile.close();
	_lastFrame = -1; _lastObject = -1;
}

void gfx4desp32P4::UserCharacterBG(uint32_t* data, uint8_t ucsize, int16_t ucx,
    int16_t ucy, uint16_t color, boolean draw,
    uint32_t bgindex) {
    UserCharacterBG(data, ucsize, ucx, ucy, color, draw, bgindex, true, 0);
}

void gfx4desp32P4::UserCharacterBG(int8_t ui, uint32_t* data, uint8_t ucsize,
    int16_t ucx, int16_t ucy, uint16_t color,
    boolean draw) {
    UserCharacterBG(data, ucsize, ucx, ucy, color, draw, tuiIndex[ui], false, ui);
}

void gfx4desp32P4::UserCharacterBG(uint32_t* data, uint8_t ucsize, int16_t ucx,
    int16_t ucy, uint16_t color, boolean draw,
    uint32_t bgindex, bool type, int8_t ui) {
    if ((!dataFile && type) || (!userImag && !type) ||
        GCItype != GCI_SYSTEM_USD) {
        return;
    }
    if (ucx < 0 || ucy < 0)
        return;
    uint16_t bwidth;
    if (type) {
        dataFile.seek(bgindex);
        bwidth = (dataFile.read() << 8) + dataFile.read();
    }
    else {
        bwidth = tuiw[ui];
    }
    uint32_t bgoff = bgindex + 6 + (((ucy * bwidth) + ucx) << 1);
    uint8_t left = 0;
    uint32_t tdw;
    uint32_t pix1;
    uint32_t pix2;
    tdw = *data++;
    uint8_t ucwidth = tdw;
    tdw = *data++;
    uint8_t ucheight = tdw;
    if ((ucx + ucwidth - 1) > (width - 1) || (ucy + ucheight - 1) > (height - 1))
        return;
    uint16_t ucloop = (ucwidth * ucheight) >> 1;
    setGRAM(ucx, ucy, ucx + ucwidth - 1, ucy + ucheight - 1);
    uint32_t test2 = 0;
    uint16_t bgbuf[ucloop << 1];
    tdw = *data++;
    for (int c = 0; c < ucloop; c++) {
        test2 = ((tdw >> (ucwidth - 1)) - left) & 0x1;
        if (test2 == 1 && draw) {
            pix1 = color;
        }
        else {
            if (type) {
                dataFile.seek(bgoff + (left << 1));
                pix1 = (dataFile.read() << 8) + dataFile.read();
            }
            else {
                userImag.seek(bgoff + (left << 1));
                pix1 = (userImag.read() << 8) + userImag.read();
            }
        }
        left++;
        test2 = ((tdw >> (ucwidth - 1)) - left) & 0x1;
        if (test2 == 1 && draw) {
            pix2 = color;
        }
        else {
            if (type) {
                dataFile.seek(bgoff + (left << 1));
                pix2 = (dataFile.read() << 8) + dataFile.read();
            }
            else {
                userImag.seek(bgoff + (left << 1));
                pix2 = (userImag.read() << 8) + userImag.read();
            }
        }
        left++;
        if (left > (ucwidth - 1)) {
            left = 0;
            bgoff = bgoff + (bwidth << 1);
            tdw = *data++;
        }
        bgbuf[c << 1] = pix1;
        bgbuf[(c << 1) + 1] = pix2;
    }
    WrGRAMs(bgbuf, ucloop << 1);
}

void gfx4desp32P4::UserCharacter(uint32_t* data, uint8_t ucsize, int16_t ucx,
    int16_t ucy, uint16_t color, uint16_t bgcolor) {
    uint8_t top = 0;
    uint8_t left = 0;
    uint32_t tdw;
    uint32_t pix1;
    uint32_t pix2;
    int mx;
    int my;
    tdw = *data++;
    uint8_t ucwidth = tdw;
    tdw = *data++;
    uint8_t ucheight = tdw;
    uint16_t ucloop = (ucwidth * ucheight) >> 1;
    uint32_t test2 = 0;
    tdw = *data++;
    if (ucx > -1 && ucy > -1 && (ucx + ucwidth - 1) < (width) &&
        (ucy + ucheight - 1) < (height)) {
        setGRAM(ucx, ucy, ucx + ucwidth - 1, ucy + ucheight - 1);
        for (int c = 0; c < ucloop; c++) {
            test2 = ((tdw >> (ucwidth - 1)) - left) & 0x1;
            pix1 = bgcolor;
            if (test2 == 1)
                pix1 = color;
            left++;
            test2 = ((tdw >> (ucwidth - 1)) - left) & 0x1;
            pix2 = bgcolor;
            if (test2 == 1)
                pix2 = color;
            left++;
            if (left > (ucwidth - 1)) {
                left = 0;
                tdw = *data++;
            }
            WrGRAM(pix1);
            WrGRAM(pix2);
        }
    }
    else {
        for (int c = 0; c < ucloop; c++) {
            mx = ucx + left;
            my = ucy + top;
            test2 = ((tdw >> (ucwidth - 1)) - left) & 0x1;
            if (mx > -1 && my > -1 && mx < width && my < height) {
                if (test2 == 1) {
                    PutPixel(mx, my, color);
                }
                else {
                    PutPixel(mx, my, bgcolor);
                }
            }
            left++;
            mx = ucx + left;
            test2 = ((tdw >> (ucwidth - 1)) - left) & 0x1;
            if (mx > -1 && my > -1 && mx < width && my < height) {
                if (test2 == 1) {
                    PutPixel(mx, my, color);
                }
                else {
                    PutPixel(mx, my, bgcolor);
                }
            }
            left++;
            if (left > (ucwidth - 1)) {
                left = 0;
                top++;
                tdw = *data++;
            }
        }
    }
}

bool gfx4desp32P4::CheckSD(void) { return sdok; }

bool gfx4desp32P4::CheckDL(void) { return dlok; }

void gfx4desp32P4::setCacheSize(uint32_t cs) { DRcache = cs; }

void gfx4desp32P4::setGCIsystem(uint8_t gs) {
    if (gs <= GCI_SYSTEM_SPECIAL) {
        Close4dGFX();
        GCItype = gs;
    }
}

uint8_t gfx4desp32P4::getGCIsystem() { return GCItype; }

void gfx4desp32P4::WriteGCIImageToGRAM(uint32_t Index, int x1, int y1, int x2, int y2, uint32_t len){
	setGRAM(x1, y1, x2, y2);
	//int32_t pos = 0;
	int bufSeg;
	int rem;
	int32_t pos = 0;
    bufSeg = len / SD_BUFF_SIZE;
	rem = len - (SD_BUFF_SIZE * bufSeg); 
	userImag.seek(Index);
	if (bufSeg > 0){
	  while (bufSeg --){
		userImag.read(buff, SD_BUFF_SIZE);
		WrGRAMs(buff, SD_BUFF_SIZE);
	  }
	  len = rem;
	}
	if (len > 0){
      userImag.read(psRAMbuffer1, len);
	  WrGRAMs(buff, len);
    }
}

void IRAM_ATTR gfx4desp32P4::GCIreadToBuff(uint32_t Index, uint32_t len) {
	switch (GCItype) {
    case GCI_SYSTEM_USD:
		userImag.seek(Index);
        userImag.read(psRAMbuffer1, len);
		break;
    case GCI_SYSTEM_PROGMEM:
        gciArrayPos = Index;
        memcpy(psRAMbuffer1, GCIarray + gciArrayPos, len);
        gciArrayPos += len;
        break;
    }
}

void gfx4desp32P4::GCIreadToBuff2(uint32_t Index, uint32_t len) {
    switch (GCItype) {
    case GCI_SYSTEM_USD:
        userImag.seek(Index);
        userImag.read(psRAMbuffer2, len);
        break;
    case GCI_SYSTEM_PROGMEM:
        gciArrayPos = Index;
        memcpy(psRAMbuffer2, GCIarray + gciArrayPos, len);
        gciArrayPos += len;
        break;
    }
}

void gfx4desp32P4::GCIreadToBuff2(uint32_t Index, uint32_t pos, uint32_t len) {
    switch (GCItype) {
    case GCI_SYSTEM_USD:
        userImag.seek(Index);
        userImag.read(psRAMbuffer2 + pos, len);
        break;
    case GCI_SYSTEM_PROGMEM:
        gciArrayPos = Index;
        memcpy(psRAMbuffer2 + pos, GCIarray + gciArrayPos, len);
        gciArrayPos += len;
        break;
    }
}

void gfx4desp32P4::GCIreadToBuf(uint32_t Index, uint32_t len) {
    switch (GCItype) {
    case GCI_SYSTEM_USD:
        userImag.read(psRAMbuffer1 + Index, len);
        break;
    case GCI_SYSTEM_PROGMEM:
        memcpy(psRAMbuffer1 + Index, GCIarray + gciArrayPos, len);
        gciArrayPos += len;
        break;
    }
}

void gfx4desp32P4::GCIread(uint8_t* dest, uint32_t len) {
    switch (GCItype) {
    case GCI_SYSTEM_USD:
        userImag.read(dest, len);
        break;
	case GCI_SYSTEM_JPEG_USD:
        userImag.read(dest, len);
        break;
    case GCI_SYSTEM_PROGMEM:
        memcpy(dest, GCIarray + gciArrayPos, len);
        gciArrayPos += len;
        break;
    case GCI_SYSTEM_JPEG_FLASH:
	    memcpy(dest, GCIarray + gciArrayPos, len);
        gciArrayPos += len;
        break;
	}
}

void gfx4desp32P4::DATAreadToBuff(uint32_t Index, uint32_t len) {
    switch (GCItype) {
    case GCI_SYSTEM_USD:
        dataFile.seek(Index);
        dataFile.read(psRAMbuffer1, len);
        break;
    case GCI_SYSTEM_PROGMEM:
        gciArrayPos = Index;
        memcpy(psRAMbuffer1, GCIarray + gciArrayPos, len);
        gciArrayPos += len;
        break;
    }
}

void gfx4desp32P4::DATAread(uint8_t* dest, uint32_t len) {
    dataFile.read(dest, len);
}

int16_t gfx4desp32P4::GCIread() {
    switch (GCItype) {
    case GCI_SYSTEM_USD:
        return (userImag.read());
        break;
    case GCI_SYSTEM_PROGMEM:
        return GCIarray[gciArrayPos++];
        break;
    }
    return -1;
}

int16_t gfx4desp32P4::DATread() { return (userDat.read()); }

int16_t gfx4desp32P4::DATAread() { return (dataFile.read()); }

void gfx4desp32P4::GCIseek(uint32_t Index) {
    switch (GCItype) {
    case GCI_SYSTEM_USD:
        userImag.seek(Index);
        break;
	case GCI_SYSTEM_JPEG_USD:
        userImag.seek(Index);
        break;
    case GCI_SYSTEM_PROGMEM:
        gciArrayPos = Index;
        break;
	case GCI_SYSTEM_JPEG_FLASH:
        gciArrayPos = Index;
        break;
    }
}

void gfx4desp32P4::DATAseek(uint32_t Index) { dataFile.seek(Index); }

uint32_t gfx4desp32P4::DATAsize() { return dataFile.size(); }

void gfx4desp32P4::Open4dGFX(const uint8_t* DATa, uint32_t DATlen,
    const uint8_t* GCIa, uint32_t GCIlen) {
    DATarray = DATa;
    GCIarray = GCIa;
    datArraySize = DATlen;
    gciArraySize = GCIlen;
    GCItype = GCI_SYSTEM_PROGMEM;
    Open4dGFX("gfx4dDummy");
}

void gfx4desp32P4::Open4dGFX(const uint8_t* JPEGa, uint32_t FLsize) {
    GCIarray = JPEGa;
    gciArraySize = FLsize;
    //GCItype = GCI_SYSTEM_JPEG_FLASH;
	_Open4dGFXjpeg("", GCI_SYSTEM_JPEG_FLASH); 
}

void gfx4desp32P4::UserImageHide(int hndl) { UserImageHide(hndl, BLACK); }

void gfx4desp32P4::UserImageHide(int hndl, uint16_t color) {
    if (hndl > 0) {
        RectangleFilled(tuix[hndl], tuiy[hndl], tuiw[hndl], tuih[hndl], color);
    }
    else {
        for (int n = 0; n < gciobjnum; n++) {
            RectangleFilled(tuix[n], tuiy[n], tuiw[n], tuih[n], color);
        }
    }
}

void gfx4desp32P4::UserImageHideBG(int hndl, int objBG) {
    if (hndl > 0) {
        UserImageDR(objBG, tuix[hndl], tuiy[hndl], tuiw[hndl], tuih[hndl],
            tuix[hndl], tuiy[hndl]);
    }
    else {
        for (int n = 0; n < gciobjnum; n++) {
            UserImageDR(objBG, tuix[n], tuiy[n], tuiw[n], tuih[n], tuix[n], tuiy[n]);
        }
    }
}

/****************************************************************************/
/*!
  @brief  Draw smooth line between 2 points with different radius at each end
  @param  xpos1 - start X as a floating point position
  @param  ypos1 - start Y as a floating point position
  @param  xpos2 - end X as a floating point position
  @param  ypos2 - end Y as a floating point position
  @param  xy1r - width of start of line as a floating point width
  @param  xy2r - width of end of line as a floating point width
  @param  fg_color - RGB565 colour
*/
/****************************************************************************/
void gfx4desp32P4::LineAA(float xpos1, float ypos1, float xpos2, float ypos2, float xy1r, float xy2r, int32_t fg_color) {
    if ((xy1r < 0.0) || (xy2r < 0.0))return;
    if ((fabsf(xpos1 - xpos2) < 0.01f) && (fabsf(ypos1 - ypos2) < 0.01f)) xpos2 += 0.01f;
    bool ex;
	bool op32 = (frame_buffer == CANVAS_BUFFER_ARGB);
    float h, dx, dy;
    int32_t x0 = (int32_t)floorf(fminf(xpos1 - xy1r, xpos2 - xy2r));
    int32_t x1 = (int32_t)ceilf(fmaxf(xpos1 + xy1r, xpos2 + xy2r));
    int32_t y0 = (int32_t)floorf(fminf(ypos1 - xy1r, ypos2 - xy2r));
    int32_t y1 = (int32_t)ceilf(fmaxf(ypos1 + xy1r, ypos2 + xy2r));
    int32_t ys = ypos1;
    if ((xpos1 - xy1r) > (xpos2 - xy2r)) ys = ypos2;
    float dr = xy1r - xy2r;
    float alphaL = 1.0f;
	float newAlph = 255;
	float alphAdj = 1.0;
	if (alpha){
		newAlph = /*0.39 * */__alpha; 
		alphAdj = __alpha / 255.0;
	}
    xy1r += 0.5;
    float xx, yy;
    float bax = xpos2 - xpos1;
    float bay = ypos2 - ypos1;
    uint8_t FB = 0;
    int16_t imageNum = -1;
    uint16_t fgcol;
    bool needsEndWrite = StartWrite();
    int32_t xs = x0;
    int srtCount, Count;
    int32_t cy, cx, cxofst;
    if ((fg_color & 0xffff0000) == 0xf00000) FB = fg_color & 0x0f;
    if ((fg_color & 0xffff0000) == 0xf0000) imageNum = fg_color & 0xffff;
    cy = ys;
    while (cy <= y1) {
        ex = false; yy = cy - ypos1;
        srtCount = 0xffff;
        Count = 0;
        cxofst = 0;
        cx = xs;
        while (cx <= x1) {
            if (FB) {
                fgcol = ReadPixelFromFrameBuffer(cx, cy, FB);
            }
            else if (imageNum != -1) {
                fgcol = ReadImagePixel(imageNum, cx, cy);
            }
            else {
                fgcol = fg_color & 0xffff;
            }
            if (ex && alphaL <= lineAAparam1) break;
            xx = cx - xpos1;
            h = fmaxf(fminf((xx * bax + yy * bay) / (bax * bax + bay * bay), 1.0f), 0.0f);
            dx = xx - bax * h, dy = yy - bay * h;
            alphaL = xy1r - (sqrt(dx * dx + dy * dy) + h * dr);
            if (!(alphaL <= lineAAparam1)) {
				if (!ex) {
                    ex = true; xs = cx;
                }
				if (op32){
					if (alphaL > lineAAparam2) {
						PutPixelARGB(cx, cy, fgcol, newAlph);
					} else {
						PutPixelARGB(cx, cy, fgcol, (uint8_t)(alphaL * lineAAparam0));
					}
				} else {
					if (srtCount == 0xffff) srtCount = cx;
					if (cx >= 0 && cx < width) {
						if (alphaL > lineAAparam2) {
							linebuff[Count++] = fgcol;
						}
						else {
							calcAlpha(fgcol, ReadPixel(cx, cy), (uint8_t)(alphaL * lineAAparam0));
							linebuff[Count++] = __colour;
						}
					}
					else {
						if (cx < 0) cxofst++;
					}
				}
            }
            cx++;
        }
        if (Count > 0 && !op32) {
            if (Count == 1 && cxofst == 0) {
                PutPixelAlpha(srtCount, cy, fgcol, /*255*/alphaL * lineAAparam0);
            }
            else {
                WriteLine(srtCount + cxofst, cy, Count, linebuff);
            }
        }
        cy++;
    }
    xs = x0;
    cy = ys - 1;
    while (cy >= y0) {
        ex = false; yy = cy - ypos1;
        srtCount = 0xffff;
        Count = 0;
        cxofst = 0;
        cx = xs;
        while (cx <= x1) {
            if (FB) {
                fgcol = ReadPixelFromFrameBuffer(cx, cy, FB);
            }
            else if (imageNum != -1) {
                fgcol = ReadImagePixel(imageNum, cx, cy);
            }
            else {
                fgcol = fg_color & 0xffff;
            }
            if (ex && alphaL <= lineAAparam1) break;
            xx = cx - xpos1;
            h = fmaxf(fminf((xx * bax + yy * bay) / (bax * bax + bay * bay), 1.0f), 0.0f);
            dx = xx - bax * h, dy = yy - bay * h;
            alphaL = xy1r - (sqrt(dx * dx + dy * dy) + h * dr);
            if (!(alphaL <= lineAAparam1)) {
                if (!ex) {
                    ex = true; xs = cx;
                }
                if (op32){
					if (alphaL > lineAAparam2) {
						PutPixelARGB(cx, cy, fgcol, newAlph);
					} else {
						PutPixelARGB(cx, cy, fgcol, (uint8_t)(alphaL * lineAAparam0));
					}
				} else {
					if (srtCount == 0xffff) srtCount = cx;
					if (cx >= 0 && cx < width) {
						if (alphaL > lineAAparam2) {
							linebuff[Count++] = fgcol;
						}
						else {
							calcAlpha(fgcol, ReadPixel(cx, cy), (uint8_t)(alphaL * lineAAparam0));
							linebuff[Count++] = __colour;
						}
					}
					else {
						if (cx < 0) cxofst++;
					}
				}
            }
            cx++;
        }
        if (Count > 0 && !op32) {
            if (Count == 1 && cxofst == 0) {
                PutPixelAlpha(srtCount, cy, fgcol, /*255*/alphaL * lineAAparam0);
            }
            else {
                WriteLine(srtCount + cxofst, cy, Count, linebuff);
            }
        }
        cy--;
    }
    if (needsEndWrite)
        EndWrite();
}

/****************************************************************************/
/*!
  @brief  Draw smooth filled circle using radii of LineAA function
  @param  x - left X as a floating point position
  @param  y - top Y as a floating point position
  @param  r - radius of circle
  @param  color - RGB565 colour
*/
/****************************************************************************/
void gfx4desp32P4::CircleFilledAA(float x, float y, float r, uint32_t color)
{
    LineAA(x, y, x, y, r, r, color);
}

/****************************************************************************/
/*!
  @brief  Draw smooth line between 2 points
  @param  x - start X as a floating point position
  @param  y - start Y as a floating point position
  @param  x1 - end X as a floating point position
  @param  y1 - end Y as a floating point position
  @param  w - width of line as a floating point width
  @param  color - RGB565 colour
*/
/****************************************************************************/
void gfx4desp32P4::LineAA(float x, float y, float x1, float y1, float w, uint32_t color)
{
    LineAA(x, y, x1, y1, w / 2.0, w / 2.0, color);
}

/****************************************************************************/
/*!
  @brief  Draw smooth triangle.
  @param  x0 - first X position in pixels
  @param  y0 - first Y position in pixels
  @param  x1 - second X position in pixels
  @param  y1 - second Y position in pixels
  @param  x2 - third X position in pixels
  @param  y2 - third Y position in pixels
  @param  w  - width of lines
  @param  color - RGB565 colour
  @note clipping is handled by Line function
*/
/****************************************************************************/
void gfx4desp32P4::TriangleAA(float x0, float y0, float x1, float y1,
    float x2, float y2, int w, uint16_t color) {
    bool needsEndWrite = StartWrite();
    LineAA(x0, y0, x1, y1, w, color);
    LineAA(x1, y1, x2, y2, w, color);
    LineAA(x2, y2, x0, y0, w, color);
    if (needsEndWrite)
        EndWrite();
}

void gfx4desp32P4::RoundRectAA(int16_t x, int16_t y, int16_t x1, int16_t y1,
        int16_t r, int16_t thk, uint16_t color) {
    bool needsEndWrite = StartWrite();
    int w = x1 - x + 1;
    int h = y1 - y + 1;
    int ri = r - thk; 
    RectangleFilled(x + r, y, x1 - r, y + thk, color);
    RectangleFilled(x + r, y + h - (thk+1), x1 - r, y + h - 1, color);
    RectangleFilled(x, y + r, x + thk, y + h - r - 1, color);
    RectangleFilled(x + w - (thk+1), y + r, x + w - 1, y + h - r - 1, color);
    drawArc(x + r, y + r, r, ri, 91, 179, color);
    drawArc(x + w - r - 1, y + r, r, ri, 181, 269, color);
    drawArc(x + w - r - 1, y + h - r - 1, r, ri, 271, 359, color);
    drawArc(x + r, y + h - r - 1, r, ri, 1, 89, color);
    if (needsEndWrite)
       EndWrite();
}

void gfx4desp32P4::RoundRectFilledAA(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, int32_t color)
{
  bool needsEndWrite = StartWrite();
  int32_t xs = 0;
  int32_t cnx = 0;
  if (r < 0)   r = 0;
  if (r > w >> 1) r = w >> 1;
  if (r > h >> 1) r = h >> 1;
  y += r;
  h -= 2 * r;
  float al;
  RectangleFilledX((int)x, (int)y, (int)(x + w - 1), (int)(y + h - 1), color);
  h--;
  x += r;
  w -= 2 * r + 1;
  int32_t r1 = r * r;
  int32_t r2 = r1 + ((r++) << 1) + 1;
  for (int32_t cny = r - 1; cny > 0; cny--)
  {
    int32_t dy2 = (r - cny) * (r - cny);
    for (cnx = xs; cnx < r; cnx++)
    {
      int32_t hyp2 = (r - cnx) * (r - cnx) + dy2;
      if (hyp2 <= r1) break;
      if (hyp2 >= r2) continue;
      al = sqrt(hyp2);
      uint8_t alpha = ~((int)((al - (int)al) * 255));
      if (alpha > 246) break;
      xs = cnx;
      if (alpha < 9) continue;
      PutPixelAlpha(x + cnx - r, y + cny - r, color, (uint8_t)alpha);
      PutPixelAlpha(x - cnx + r + w, y + cny - r, color, (uint8_t)alpha);
      PutPixelAlpha(x - cnx + r + w, y - cny + r + h, color, (uint8_t)alpha);
      PutPixelAlpha(x + cnx - r, y - cny + r + h, color, (uint8_t)alpha);
    }
    HlineX(x + cnx - r, y + cny - r, 2 * (r - cnx) + 1 + w, color);
    HlineX(x + cnx - r, y - cny + r + h, 2 * (r - cnx) + 1 + w, color);
  }
  if (needsEndWrite) EndWrite();
}

/****************************************************************************/
/*!
  @brief  Sets frame buffer for building layered widget 
  @param  bgColor - RGB565 color of background blanking rectangle
  @param  builtBuff - Buffer that widget layers will be built
  @param  x1 - Top left x position of anim area
  @param  y1 - Top left y position of anim area
  @param  x2 - Bottom right x position of anim area
  @param  y2 - Bottom right y position of anim area
  @returns Index of stored animation profile
  @note Allows simple widget layered build using plain background
*/
/****************************************************************************/
int gfx4desp32P4::StartAnim(uint32_t bgColor, int builtBuff, int x1, int y1, int x2, int y2){
  int tcount;
  tcount = StartAnim(-1, bgColor, builtBuff, x1, y1, x2, y2);
  return tcount;  
}

/****************************************************************************/
/*!
  @brief  Sets frame 3 buffers for building layered widget using background image
  @param  bkgBuff - First source buffer containing background image to copy
  @param  buildBuff - Second storage buffer for holding background image
  @param  builtBuff - Third buffer for building widget layers
  @param  x1 - Top left x position of anim area
  @param  y1 - Top left y position of anim area
  @param  x2 - Bottom right x position of anim area
  @param  y2 - Bottom right y position of anim area
  @returns Index of stored animation profile
  @note Allows complex widget layered build using image background
*/
/****************************************************************************/
int gfx4desp32P4::StartAnim(int16_t bkgBuff, uint32_t buildBuff, uint16_t builtBuff, int x1, int y1, int x2, int y2){
  int arraypos = 8 * animIndexCounter;
  if(bkgBuff != -1){
    DrawToframebuffer(buildBuff);
    DrawFrameBufferAreaPPA(bkgBuff, x1, y1, x2, y2);
    DrawToframebuffer(builtBuff);
    DrawFrameBufferAreaPPA(buildBuff, x1, y1, x2, y2);
  } else {
	RectangleFilled(x1, y1, x2, y2, buildBuff);  
  }
  anims[arraypos] = bkgBuff;
  anims[arraypos + 1] = buildBuff;
  anims[arraypos + 2] = builtBuff;
  anims[arraypos + 3] = x1; anims[arraypos + 4] = y1; anims[arraypos +5] = x2; anims[arraypos + 6] = y2;  
  return -32767 + (animIndexCounter++);
}

/****************************************************************************/
/*!
  @brief  Sets up anim when returning to rebuild layered widget
  @param  animIndex - index given for retreiving anim parameters
*/
/****************************************************************************/
void gfx4desp32P4::ContAnim(int animIndex){
  int arraypos = 8 * (animIndex + 32767);
  DrawToframebuffer(anims[arraypos + 2]);
  if(anims[arraypos] != -1){
    DrawFrameBufferAreaPPA(anims[arraypos + 1], anims[arraypos + 3], anims[arraypos + 4], anims[arraypos + 5], anims[arraypos + 6]);
  } else {
	RectangleFilled(anims[arraypos + 3], anims[arraypos + 4], anims[arraypos + 5], anims[arraypos + 6], anims[arraypos + 1]);  
  }
  
}

/****************************************************************************/
/*!
  @brief  Show built layered widget
  @param  animIndex - index given for retreiving anim parameters
*/
/****************************************************************************/
void gfx4desp32P4::ShowAnim(int animIndex, int fb){
  int arraypos = 8 * (animIndex + 32767);
  DrawToframebuffer(fb);
  DrawFrameBufferAreaPPA(anims[arraypos + 2], anims[arraypos + 3], anims[arraypos + 4], anims[arraypos + 5], anims[arraypos + 6]); 
  if(anims[arraypos] != -1){
    DrawToframebuffer(anims[arraypos + 2]);
    DrawFrameBufferAreaPPA(anims[arraypos + 1], anims[arraypos + 3], anims[arraypos + 4], anims[arraypos + 5], anims[arraypos + 6]);
  } else {
	DrawToframebuffer(anims[arraypos + 2]);
    RectangleFilled(anims[arraypos + 3], anims[arraypos + 4], anims[arraypos + 5], anims[arraypos + 6], anims[arraypos + 1]);	
  }
  DrawToframebuffer(fb);
}

/****************************************************************************/
/*!
  @brief  Show built layered widget
  @param  animIndex - index given for retreiving anim parameters
*/
/****************************************************************************/
void gfx4desp32P4::ShowAnim(int animIndex, int x, int y, int fb){
  int arraypos = 8 * (animIndex + 32767);
  DrawToframebuffer(fb);
  DrawFrameBufferAreaXYPPA(anims[arraypos + 2], anims[arraypos + 3], anims[arraypos + 4], anims[arraypos + 5], anims[arraypos + 6], x, y); 
  if(anims[arraypos] != -1){
    DrawToframebuffer(anims[arraypos + 2]);
    DrawFrameBufferAreaPPA(anims[arraypos + 1], anims[arraypos + 3], anims[arraypos + 4], anims[arraypos + 5], anims[arraypos + 6]);
  } else {
	DrawToframebuffer(anims[arraypos + 2]);
    RectangleFilled(anims[arraypos + 3], anims[arraypos + 4], anims[arraypos + 5], anims[arraypos + 6], anims[arraypos + 1]);	
  }
  DrawToframebuffer(fb);
}

/****************************************************************************/
/*!
  @brief  End layered widget anim
  @param  animIndex - index given for retreiving anim parameters
*/
/****************************************************************************/
void gfx4desp32P4::EndAnim(int animIndex){
  //int arraypos = 7 * animIndex;
  DrawToframebuffer(0);
}

/****************************************************************************/
/*!
  @brief  Draws a list of images in a rotary carousel
  @param  valdeg - angle in degrees of position of rotary carousel
  @param  xc - x center position of rotary carousel
  @param  yc - y center position of rotary carousel
  @param  sa - Start angle (value 0) of rotary carousel
  @param  imageArcRange - Range in degrees of carousel movement
  @param  numImages - Number of images in the image list
  @param  imageList - Array of image indexes
  @param  opac1 - opacity of images at outer range of carousel
  @param  opac2 - opacity of images at valdeg position
  @param  imageMask - Value setting the shape that the image will be drawn as
  @param  RRarc - if set as round rectangle, set the radius of the corners
  @param  selectedFrame - if set a frame will be drawn around selected image (only SHAPE_ROUNDED_RECTANGLE supported at present)
  @param  frameColour - Color of sectected frame  
  @param  frameWidth - Line thickness of selected frame 
  @param  frameGap - Gap between selected frame and image 
  @param  frameOpac - Opacity of selected frame
  @note   use with StartAnim, ContAnim, ShowAnim functions
*/
/****************************************************************************/
void gfx4desp32P4::AngularImageRotary(int valdeg, int xc, int yc, int sa, int imageArcRange, int imageCTRradius, int numImages, int *imageList, uint8_t opac1, uint8_t opac2, int imageMask, int RRarc, int selectedFrame, uint16_t frameColour, int frameWidth, int frameGap, int frameOpac){
  bool opacON = false;
  if (valdeg > imageArcRange) valdeg = imageArcRange;
  if (valdeg < 0) valdeg = 0;
  StoreCursPos();
  float opacityLstep;
  float opacityL;
  int degCount = valdeg;
  float newSa = sa - valdeg;
  int n, x, y, w, h;
  int xy[2];
  int alphabk = __alpha;
  bool alphaonbk = alpha;
  if(opac1 >= 1 || opac2 >= 1){
	if(opac1 > opac2) gfx_Swap(opac1, opac2);
	opacityLstep = (float)(opac2 - opac1) / imageArcRange;
	opacON = true;
	AlphaBlend(ON);
  }
  int rr;
  float gap = imageArcRange / (float)(numImages - 1); 
  for (n = 0; n < numImages; n++){
	MoveTo(xc, yc);
	w = imageGetWord(imageList[n], IMAGE_WIDTH);	
	h = imageGetWord(imageList[n], IMAGE_HEIGHT);
	if (RRarc <= 0){
	  rr = w;
	  if(w > h) rr = h;
	  rr = rr >> 1;
	} else {
	  rr = RRarc;
	}
	degCount = valdeg - (n * gap);
	AlphaBlendLevel(opac2 - (abs(degCount * opacityLstep)));
	Orbit((int)(newSa + (n * gap)), imageCTRradius, xy);
	x = xy[0]; y = xy[1];
	imageSetWord(imageList[n], IMAGE_XPOS, x - (w >> 1));
	imageSetWord(imageList[n], IMAGE_YPOS, y - (h >> 1)); 
	if(imageMask == SHAPE_CIRCLE){
	  CircleFilledAA(x, y, rr, GCI_IMAGE+imageList[n]);
	} else if (imageMask == SHAPE_RECTANGLE){
	  RectangleFilledX(x - (w >> 1), y - (h >> 1), x + (w >> 1) - 1, y + (h >> 1) - 1, GCI_IMAGE+imageList[n]);	
	} else if (imageMask == SHAPE_ROUNDED_RECTANGLE){
	  if(RRarc > (w >> 1)) RRarc = w >> 1;
	  if(RRarc > (h >> 1)) RRarc = h >> 1;
	  RoundRectFilledAA(x - (w >> 1), y - (h >> 1), w, h, RRarc, GCI_IMAGE+imageList[n]);
	} else {
	  UserImage(imageList[n]);
	}
  }
  if(selectedFrame == SHAPE_ROUNDED_RECTANGLE){
    Orbit(sa, imageCTRradius, xy);
	x = xy[0] - (w >> 1) - frameWidth - frameGap; y = xy[1] - (h >> 1) - frameWidth - frameGap;
	if(frameOpac != 255){
	  AlphaBlendLevel(frameOpac);
	  AlphaBlend(ON);
	}
	RoundRectAA(x, y, x + w + ((frameWidth + frameGap) << 1) - 1, y + h + ((frameWidth + frameGap) << 1) - 1, RRarc + frameWidth + frameGap, frameWidth, frameColour);
  }
  AlphaBlendLevel(alphabk);
  AlphaBlend(alphaonbk);
  //AngularScale(x, y, sa, ea, ticks, tickRadius, ticksMajor, ticksMajorRadius, tickLen, tickW, tickColor, tickMajorLen, tickMajorW, tickMajorColor, hasValues, valueRadius, minVal, maxVal, allowFloat, textColor, labels);
  RestoreCursPos();
}

/****************************************************************************/
/*!
  @brief  Draws an angular scale with optional values
  @param  x - x center position of rotary carousel
  @param  y - y center position of rotary carousel
  @param  sa - Start angle of acale
  @param  ea - End angle of scale
  @param  ticks - total number of ticks
  @param  tickRadius - radius at which the ticks are drawn
  @param  ticksMajor - Number of ticks that are major ticks
  @param  ticksMajorRadius - radius at which the major ticks are drawn
  @param  tickLen - length of minor ticks
  @param  tickW - line width of minor ticks
  @param  tickColor - RGB565 color of minor ticks
  @param  tickMajorLen - length of major ticks  
  @param  tickMajorW - line width of major ticks 
  @param  tickMajorColor - RGB565 color of major ticks 
  @param  hasValues - true or false
  @param  valueRadius - Radius of wher values will be printed
  @param  minVal - minimum value of scale
  @param  maxVal - maximum value of scale
  @param  allowFloat - true or false
  @param  fnt - font used for values, can be system, flash or GCI font
  @param  labels - Optional, use array of strings instead of values
  @note   use with StartAnim, ContAnim, ShowAnim functions
*/
/****************************************************************************/
void gfx4desp32P4::AngularScale(int x, int y, int sa, int ea, int ticks, int tickRadius, int ticksMajor, int ticksMajorRadius, int tickLen, float tickW, uint32_t tickColor, int tickMajorLen, float tickMajorW, uint32_t tickMajorColor, bool hasValues, int valueRadius, float minVal, float maxVal, bool allowFloat, int textColor, const uint8_t* fnt, String* labels){
  gfx4d_font gfnt;
  uint8_t sfnt;
  const uint8_t* cfnt = fontPtr;
  uint8_t tfno = fno;
  if(fno == 0){
	gfnt = gciFont;  
  } else if(fno == -1){
	cfnt = fontPtr;
  } else {
	sfnt = fno;
  }	
  Font(fnt, true);
  AngularScale(x, y, sa, ea, ticks, tickRadius, ticksMajor, ticksMajorRadius, tickLen, tickW, tickColor, tickMajorLen, tickMajorW, tickMajorColor, hasValues, valueRadius, minVal, maxVal, allowFloat, textColor, labels);
  if(tfno == 0){
	Font(gfnt);  
  } else if(tfno == -1){
	Font(cfnt, true);
  } else {
	Font(sfnt);
  }	
}

void gfx4desp32P4::AngularScale(int x, int y, int sa, int ea, int ticks, int tickRadius, int ticksMajor, int ticksMajorRadius, int tickLen, float tickW, uint32_t tickColor, int tickMajorLen, float tickMajorW, uint32_t tickMajorColor, bool hasValues, int valueRadius, float minVal, float maxVal, bool allowFloat, int textColor, uint8_t fnt, String* labels){
  gfx4d_font gfnt;
  uint8_t sfnt;
  const uint8_t* cfnt = fontPtr;
  uint8_t tfno = fno;
  if(fno == 0){
	gfnt = gciFont;  
  } else if(fno == -1){
	cfnt = fontPtr;
  } else {
	sfnt = fno;
  }	
  Font(fnt);
  AngularScale(x, y, sa, ea, ticks, tickRadius, ticksMajor, ticksMajorRadius, tickLen, tickW, tickColor, tickMajorLen, tickMajorW, tickMajorColor, hasValues, valueRadius, minVal, maxVal, allowFloat, textColor, labels);
  if(tfno == 0){
	Font(gfnt);  
  } else if(tfno == -1){
	Font(cfnt, true);
  } else {
	Font(sfnt);
  }	
}

void gfx4desp32P4::AngularScale(int x, int y, int sa, int ea, int ticks, int tickRadius, int ticksMajor, int ticksMajorRadius, int tickLen, float tickW, uint32_t tickColor, int tickMajorLen, float tickMajorW, uint32_t tickMajorColor, bool hasValues, int valueRadius, float minVal, float maxVal, bool allowFloat, int textColor, gfx4d_font fnt, String* labels){
  gfx4d_font gfnt;
  uint8_t sfnt;
  const uint8_t* cfnt = fontPtr;
  uint8_t tfno = fno;
  if(fno == 0){
	gfnt = gciFont;  
  } else if(fno == -1){
	cfnt = fontPtr;
  } else {
	sfnt = fno;
  }	
  Font(fnt);
  AngularScale(x, y, sa, ea, ticks, tickRadius, ticksMajor, ticksMajorRadius, tickLen, tickW, tickColor, tickMajorLen, tickMajorW, tickMajorColor, hasValues, valueRadius, minVal, maxVal, allowFloat, textColor, labels);
  if(tfno == 0){
	Font(gfnt);  
  } else if(tfno == -1){
	Font(cfnt, true);
  } else {
	Font(sfnt);
  }	
}

void gfx4desp32P4::AngularScale(int x, int y, int sa, int ea, int ticks, int tickRadius, int ticksMajor, int ticksMajorRadius, int tickLen, float tickW, uint32_t tickColor, int tickMajorLen, float tickMajorW, uint32_t tickMajorColor, bool hasValues, int valueRadius, float minVal, float maxVal, bool allowFloat, int textColor, String* labels){
  if(ea < sa) return;
  ea += 180;
  sa += 180;
  int tm = ticksMajor;
  if (tm == 0) tm = 1;
  uint16_t tcol = textcolor;
  uint16_t tbcol = textbgcolor;
  StoreCursPos();
  bool twr = wrap;
  wrap = false;
  int dir = 0;
  int labCount = 0; 
  bool sen = sEnable;
  //sEnable = false;  
  if (sen) ScrollEnable(false);
  bool needsEndWrite = StartWrite();
  int tickChange;
  tickChange = ticks / tm;
  float range;  
  if (minVal > maxVal) dir = 1;
  if (minVal == maxVal){
	dir = 2;
	minVal = 0;
  }
  range = fabs(maxVal - minVal);
  float angularRange = fabs(ea - sa);
  float scaleInc;
  scaleInc = range / tm;
  float scaleVal = minVal;
  float tickGap = (float)angularRange / ticks;
  float tpos = sa;
  float xy1[2];
  float xy2[2];
  for (int n = 0; n < ticks + (angularRange != 360); n++){
    MoveTo(x, y);
    Orbit(tpos, tickRadius, xy1);
    if (!(n % tickChange) && ticksMajor > 0){
      Orbit(tpos, ticksMajorRadius, xy1);
      Orbit(tpos, ticksMajorRadius + tickMajorLen, xy2);
      if(tickMajorW > 0) LineAA(xy1[0], xy1[1], xy2[0], xy2[1], 2, 2, tickMajorColor);
      if(hasValues){
	    Orbit(tpos, valueRadius, xy1);
        TextColor(textColor, textColor);
        if(dir == 2){
          putstrCenteredXY(xy1[0] + 1, xy1[1] + 1, labels[labCount]);
	    } else {		  
		  if(allowFloat){
		    putstrCenteredXY(xy1[0] + 1, xy1[1] + 1, String(scaleVal));
	      } else {
		    putstrCenteredXY(xy1[0] + 1, xy1[1] + 1, String((int32_t)scaleVal));
	      }
	    }
	  }
	  if(dir == 0){
	    scaleVal += scaleInc;
	  } else {
		scaleVal -= scaleInc;  
	  }
	  labCount ++;
    } else {
      Orbit(tpos, tickRadius, xy1);
      Orbit(tpos, tickRadius + tickLen, xy2);
      if(tickW > 0) LineAA(xy1[0], xy1[1], xy2[0], xy2[1], 1, 1, tickColor);
    }
    tpos += tickGap;
  }
  if (needsEndWrite) EndWrite();
  TextColor(tcol, tbcol);
  RestoreCursPos();
  wrap = twr;
  if (sen) ScrollEnable(true);
}

/****************************************************************************/
/*!
  @brief  Draws a flare around an arc
  @param  x - x center position of rotary carousel
  @param  y - y center position of rotary carousel
  @param  baser - External radius of arc to apply flare
  @param  ir - Internal radius of arc to apply flare
  @param  sa - Start angle of arc
  @param  ea - End angle of arc
  @param  colorfrom - RGB565 color to start flare
  @param  colorto - RGB565 color to end flare
  @param  roundEnds - true if rounded, false if straight
  @param  flareSize - size of flare in pixels
  @param  flareDir - direction of flare FLARE_EXTERNAL, FLARE_INTERNAL
*/
/****************************************************************************/
void gfx4desp32P4::ArcFlare(int x, int y, int baser, int ir, int sa, int ea, int32_t colorfrom, int32_t colorto, bool roundEnds, int flareSize, int flareDir){
  int n;
  uint16_t aa, ba;
  float c = 255;
  float flareInc = c / (float)flareSize;
  if(flareDir < 0){
	flareDir = -1;
  } else {
	flareDir = 1;  
  }
  if(flareDir == 1){
	ir -= flareSize;
	baser += flareSize;
  }
  for(n = 0; n < flareSize; n++) {
	baser -= flareDir;
	ir += flareDir;
	calcAlpha(colorfrom, colorto, (int)((float)n * flareInc));
    ArcAA(x, y, ir, baser, sa, ea, __colour, roundEnds);
  }
}

/****************************************************************************/
/*!
  @brief  Draws a flare around an angular gauge
  @param  x - x center position of angular gauge
  @param  y - y center position of angular gauge
  @param  sa - Start angle of gauge
  @param  ea - End angle of gauge
  @param  gaugeRadius - Radius of gauge to apply flare
  @param  gaugeThickness - Thickness of gauge to apply flare
  @param  colorfrom - RGB565 color to start flare
  @param  colorto - RGB565 color to end flare
  @param  roundEnds - true if rounded, false if straight
  @param  flareSize - size of flare in pixels
  @param  flareDir - direction of flare FLARE_EXTERNAL, FLARE_INTERNAL
*/
/****************************************************************************/
void gfx4desp32P4::AngularGaugeFlare(int x, int y, int sa, int ea, int gaugeRadius, int gaugeThickness, int32_t colorfrom, int32_t colorto, bool roundEnds, int flareSize, int flareDir){
  ArcFlare(x, y, gaugeRadius + gaugeThickness, gaugeRadius, sa, ea, colorfrom, colorto, roundEnds, flareSize, flareDir);
}

/****************************************************************************/
/*!
  @brief  Draws angular gauge
  @param val to be shown on gauge
  @param  x - x center position of gauge
  @param  y - y center position of gauge
  @param  sa - Start angle of gauge
  @param  ea - End angle of gauge
  @param  min - Minimum value of gauge
  @param  max - Maximum value of gauge
  @param  gaugeRadius - Radius of gauge
  @param  gaugeThickness - Thickness of gauge
  @param  gaugeLOcolor - RGB565 color of low to val
  @param  gaugeHIcolor - RGB565 color to high to val
  @param  roundEnds - true if rounded, false if straight
*/
/****************************************************************************/
void gfx4desp32P4::AngularGauge(float val, int x, int y, int sa, int ea, float min, float max, int gaugeRadius, int gaugeThickness, int32_t gaugeLOcolor, int32_t gaugeHIcolor, bool roundEnds){
  float angularRange = ea - sa;
  float range = fabs(max - min) + 1;
  float gaugeSeg = angularRange / range;
  float pos;
  int dir = (max < min);
  if(!dir){
    pos = (gaugeSeg * val) + sa;
    if(pos < ea) ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, pos, ea, gaugeLOcolor, roundEnds);
    if(pos > sa) ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, sa, pos, gaugeHIcolor, roundEnds);
  } else {
	pos = ea - (gaugeSeg * val);
	if(pos < ea) ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, sa, pos, gaugeLOcolor, false);
    if(pos > sa) ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, pos, ea, gaugeHIcolor, roundEnds);
  }
  lastAngle = pos;
}

/****************************************************************************/
/*!
  @brief  Draws angular tick gauge
  @param val to be shown on tick gauge
  @param  x - x center position of tick gauge
  @param  y - y center position of tick gauge
  @param  sa - Start angle of tick gauge
  @param  ea - End angle of tick gauge
  @param  min - Minimum value of tick gauge
  @param  max - Maximum value of tick gauge
  @param  gaugeRadius - Radius of tick gauge
  @param  gaugeThickness - Thickness of tick gauge
  @param  tickW - Line thickness of ticks
  @param  gaugeLOcolor - RGB565 color of low to val
  @param  gaugeHIcolor - RGB565 color to high to val
  @param  gaugeblendColor - Optional, if set colors will be blended from Lo to this color
*/
/****************************************************************************/
void gfx4desp32P4::AngularTickGauge(float val, int x, int y, int sa, int ea, float min, float max, int gaugeRadius, int gaugeThickness, float tickW, int32_t gaugeLOcolor, int32_t gaugeHIcolor, int32_t gaugeblendColor){
  ea += 180; sa += 180;
  float angularRange = ea - sa;
  float range = fabs(max - min) + 1;
  float gaugeSeg = angularRange / range;
  float pos;   
  int dir = (max < min);
  float xy1[2];
  float xy2[2];
  StoreCursPos();
  float gpos;
  int r, g, b; 
  float rb, gb, bb;
  float ri, gi, bi;
  bool blend = false;
  MoveTo(x, y);
  if (gaugeHIcolor != gaugeblendColor){
	r = gaugeHIcolor >> 11;
	g = (gaugeHIcolor >> 5) & 0x3f;
	b = gaugeHIcolor & 0x1f;
	rb = gaugeblendColor >> 11;
	gb = (gaugeblendColor >> 5) & 0x3f;
	bb = gaugeblendColor & 0x1f;
	ri = (float)(rb - r) / range;
	gi = (float)(gb - g) / range;
	bi = (float)(bb - b) / range;
	blend = true;
  }
  if(!dir){
    pos = (gaugeSeg * val) + sa;
    for(int n = 0; n < range; n++){
	  gpos = (n * gaugeSeg) + sa;
	  Orbit(gpos, gaugeRadius, xy1);
	  Orbit(gpos, gaugeRadius + gaugeThickness, xy2);
	  if (gpos <= pos){
		if(!blend){
		  LineAA(xy1[0], xy1[1], xy2[0], xy2[1],  tickW, gaugeHIcolor); 
		} else {
          rb = r + (n * ri); gb = g + (n * gi); bb = b + (n * bi);
		  LineAA(xy1[0], xy1[1], xy2[0], xy2[1],  tickW, ((uint16_t)rb << 11) + ((uint16_t)gb << 5) + (uint16_t)bb);
		}			
	  } else {
		LineAA(xy1[0], xy1[1], xy2[0], xy2[1],  tickW, gaugeLOcolor);  
	  }
	}
  } else {
	pos = ea - (gaugeSeg * val);
	for(int n = 0; n < range; n++){
	  gpos = ea - (n * gaugeSeg);
	  Orbit(gpos, gaugeRadius, xy1);
	  Orbit(gpos, gaugeRadius + gaugeThickness, xy2);
	  if (pos <= gpos){
		if(!blend){
		  LineAA(xy1[0], xy1[1], xy2[0], xy2[1],  tickW, gaugeHIcolor); 
		} else {
          rb = r + (n * ri); gb = g + (n * gi); bb = b + (n * bi);
		  LineAA(xy1[0], xy1[1], xy2[0], xy2[1],  tickW, ((uint16_t)rb << 11) + ((uint16_t)gb << 5) + (uint16_t)bb);
		}			
	  } else {
		LineAA(xy1[0], xy1[1], xy2[0], xy2[1],  tickW, gaugeLOcolor);  
	  }
	}
  }
  lastAngle = pos;
  RestoreCursPos();
}

/****************************************************************************/
/*!
  @brief  Draws angular gauge
  @param val to be shown on gauge
  @param  x - x center position of gauge
  @param  y - y center position of gauge
  @param  sa - Start angle of gauge
  @param  ea - End angle of gauge
  @param  min - Minimum value of gauge
  @param  max - Maximum value of gauge
  @param  gaugeRadius - Radius of gauge
  @param  gaugeThickness - Thickness of gauge
  @param  gaugeLOcolorLO - RGB565 low color at lower level
  @param  gaugeHIcolorLO - RGB565 high color at lower level
  @param  gaugeMIDcolorLO - RGB565 mid color at lower level
  @param  gaugeLOcolorHI - RGB565 low color at higher level
  @param  gaugeHIcolorHI - RGB565 high color at higher level
  @param  gaugeMIDcolorHI - RGB565 mid color at higher level
  @param  midVal - Middle value threshold
  @param  hiVal - High value threshold
*/
/****************************************************************************/
void gfx4desp32P4::AngularGauge(float val, int x, int y, int sa, int ea, float min, float max, int gaugeRadius, int gaugeThickness, uint32_t gaugeLOcolorLO, uint32_t gaugeLOcolorHI, uint32_t gaugeMIDcolorLO, uint32_t gaugeMIDcolorHI, uint32_t gaugeHIcolorLO, uint32_t gaugeHIcolorHI, int midVal, int hiVal){
  float angularRange = ea - sa;
  float range = fabs(max - min) + 1;
  float gaugeSeg = angularRange / range;
  if(midVal > range) midVal = range;
  if(hiVal > range) hiVal = range;
  int dir = (max < min);
  float pos, ea1, sa2, ea2, sa3;
  StoreCursPos();
  if(dir){
	gfx_Swap(gaugeLOcolorLO, gaugeHIcolorHI);
	gfx_Swap(gaugeLOcolorHI, gaugeHIcolorLO);
	gfx_Swap(gaugeMIDcolorHI, gaugeMIDcolorLO);
	gfx_Swap(midVal, hiVal);
	pos = ea - (gaugeSeg * val);
	ea1 = ea - (gaugeSeg * midVal);
    sa2 = ea1;
    ea2 = ea - (gaugeSeg * hiVal);
    sa3 = ea2; 
  } else {
    pos = (gaugeSeg * val) + sa;
    ea1 = (gaugeSeg * midVal) + sa;
    sa2 = ea1;
    ea2 = (gaugeSeg * hiVal) + sa;
    sa3 = ea2;  
  }
  if(pos < ea1){
    ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, pos, ea1, gaugeLOcolorLO, false);
    if(pos > sa) ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, sa, pos, gaugeLOcolorHI, false);
  } else {
    ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, sa, ea1, gaugeLOcolorHI, false);
  }
  if(pos >= sa2 && pos < ea2){
    ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, pos, ea2, gaugeMIDcolorLO, false);
    if(pos > sa2) ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, sa2, pos, gaugeMIDcolorHI, false);
  } else {
    if(pos < sa2){
      ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, sa2, ea2, gaugeMIDcolorLO, false);
    } else {
      ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, sa2, ea2, gaugeMIDcolorHI, false);
    }
  }
  if(pos >= sa3 && pos < ea){
    if(pos < ea) ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, pos, ea, gaugeHIcolorLO, false);
    if(pos > sa3) ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, sa3, pos, gaugeHIcolorHI, false);
  } else {
    if(pos < sa3){
      ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, sa3, ea, gaugeHIcolorLO, false);
    } else {
      ArcAA(x, y, gaugeRadius, gaugeRadius + gaugeThickness, sa3, ea, gaugeHIcolorHI, false);
    }
  }
  lastAngle = pos;
  RestoreCursPos();
}

/****************************************************************************/
/*!
  @brief  Draws angular needle
  @param val - position of needle
  @param  x - x center position of needle
  @param  y - y center position of needle
  @param  sa - Start angle of needle
  @param  ea - End angle of needle
  @param  radius - radius of tip of needle
  @param  centreRadius - radius of base of needle
  @param  minVal - Minimum value of needle movement
  @param  maxVal - Maximum value of needle movement
  @param  baseW - Thickness of needle at its base
  @param  tipW - Thickness of needle at its tip
  @param  color - RGB565 color of needle
  @param  baseRadius - Radius of circular base of needle
  @param  gbaseColor - RGB565 color of circular base of needle
  @param  basePivotRadius - Radius of circular pivot
  @param  basePivotColor - RGB565 color of circular pivot
  @param  flareSize - Optional, Size of needle flare
  @param  startAlpha - Optional, Opacity level of flare start
  @param  endAlpha - Optional, Opacity level of flare end
  @param  color2 - Optional, RGB565 flare to color
*/
/****************************************************************************/
void gfx4desp32P4::AngularNeedle(float val, int x, int y, int sa, int ea, int radius, int centreRadius, float minVal, float maxVal, float baseW, float tipW, int32_t color, int baseRadius, int32_t baseColor, int basePivotRadius, int32_t basePivotColor, int flareSize, int startAlpha, int endAlpha, int flareDir, int32_t color2){
  bool dir = (maxVal < minVal);
  ea += 180; sa += 180;
  float angularRange;
  float range;
  angularRange = ea - sa;
  if (dir){
	range = minVal - maxVal + 1;
  } else {
	range = maxVal - minVal + 1;
  }
  float angularSegment = angularRange / range;
  float pos;
  StoreCursPos(); 
  if (!dir){
    pos = (angularSegment * (val - minVal)) + sa;
  } else {
    pos = (angularSegment * (range - (val - maxVal))) + sa;
  }
  float xy1[2];
  float xy2[2];
  MoveTo(x, y);
  if(centreRadius != 0){
    Orbit(pos, centreRadius, xy2);
	Orbit(pos, radius, xy1);
	if(flareSize > 0 && flareDir == FLARE_EXTERNAL){
	  LineAAflare(xy2[0], xy2[1], xy1[0], xy1[1], baseW, tipW, flareSize, startAlpha, endAlpha, flareDir, color2);
	}
	lastAngle = pos;
    LineAA(xy2[0], xy2[1], xy1[0], xy1[1], baseW, tipW, color);
  } else {
    Orbit(pos, radius, xy1);
	if(flareSize > 0 && flareDir == FLARE_EXTERNAL){
	  LineAAflare(x, y, xy1[0], xy1[1], baseW, tipW, flareSize, startAlpha, endAlpha, flareDir, color2);
	}
	LineAA(x, y, xy1[0], xy1[1], baseW, tipW, color);
	xy2[0] = x; xy2[1] = y;
  }
  if(flareSize > 0 && flareDir == FLARE_INTERNAL){
	  LineAAflare(xy2[0], xy2[1], xy1[0], xy1[1], baseW, tipW, flareSize, startAlpha, endAlpha, flareDir, color2);
  }
  if(baseRadius > 0)
    CircleFilledAA(x, y, baseRadius, baseColor);
  if(basePivotRadius > 0)
    CircleFilledAA(x, y, basePivotRadius, basePivotColor);
  RestoreCursPos();
}

/****************************************************************************/
/*!
  @brief  Draws angular needle with 2 steps
  @param val - position of needle
  @param  x - x center position of needle
  @param  y - y center position of needle
  @param  sa - Start angle of needle
  @param  ea - End angle of needle
  @param  radius - radius of tip of needle
  @param  centreRadius - radius of base of needle
  @param  midRadius - radius of middle of needle
  @param  minVal - Minimum value of needle movement
  @param  maxVal - Maximum value of needle movement
  @param  baseW - Thickness of needle at its base
  @param  midW - Thickness of needle at its mid point
  @param  tipW - Thickness of needle at its tip
  @param  color - RGB565 color of needle
  @param  baseRadius - Radius of circular base of needle
  @param  gbaseColor - RGB565 color of circular base of needle
  @param  basePivotRadius - Radius of circular pivot
  @param  basePivotColor - RGB565 color of circular pivot
*/
/****************************************************************************/
void gfx4desp32P4::AngularNeedleDouble(float val, int x, int y, int sa, int ea, int radius, int centreRadius, int midRadius, float minVal, float maxVal, float baseW, float midW, float tipW, int32_t color, int baseRadius, int32_t baseColor, int basePivotRadius, int32_t basePivotColor){
  int dir = (maxVal < minVal);
  ea += 180; sa += 180;
  float angularRange = ea - sa;
  float range = abs(maxVal - minVal) + 1;
  float angularSegment = angularRange / range;
  float pos;
  StoreCursPos();
  if (!dir){
    pos = (angularSegment * val) + sa;
  } else {
    pos = ea - (angularSegment * val);
    }
  float xy1[2];
  float xy2[2];
  float xy3[2];
  MoveTo(x, y);
  Orbit(pos, radius, xy1);
  if(centreRadius > 0){
    Orbit(pos, centreRadius, xy2);
	lastAngle = pos;
    if(midW && midRadius == 0){
	  LineAA(xy2[0], xy2[1], xy1[0], xy1[1], baseW, tipW, color);
    } else {
	  Orbit(pos, midRadius, xy3);
	  LineAA(xy2[0], xy2[1], xy3[0], xy3[1], baseW, midW, color);
	  LineAA(xy3[0], xy3[1], xy1[0], xy1[1], midW, tipW, color);
	}
  } else {
    if(midW && midRadius == 0){
	  LineAA(x, y, xy1[0], xy1[1], baseW, tipW, color);
    } else {
	  Orbit(pos, midRadius, xy3);
	  LineAA(x, y, xy3[0], xy3[1], baseW, midW, color);
	  LineAA(xy3[0], xy3[1], xy1[0], xy1[1], midW, tipW, color);
	}
  }
  if(baseRadius > 0)
    CircleFilledAA(x, y, baseRadius, baseColor);
  if(basePivotRadius > 0)
    CircleFilledAA(x, y, basePivotRadius, basePivotColor);
  RestoreCursPos();
}

/****************************************************************************/
/*!
  @brief  Draws a knob at angular position
  @param val - position of angular knob
  @param  x - x center position of angular knob
  @param  y - y center position of angular knob
  @param  sa - Start angle of angular knob
  @param  ea - End angle of angular knob
  @param  radius - radius of angular knob
  @param  w - width of angular knob
  @param  sizeh - height of angular knob
  @param  minVal - Minimum value of angular knob movement
  @param  maxVal - Maximum value of angular knob movement
  @param  outlineSize - Thickness of knob outline
  @param  flareSize - Optional, Size of knob flare
  @param  startAlpha - Optional, Opacity level of flare start
  @param  endAlpha - Optional, Opacity level of flare end
  @param  flareDir - direction of flare FLARE_EXTERNAL, FLARE_INTERNAL
  @param  color2 - Optional, RGB565 flare to color
*/
/****************************************************************************/
void gfx4desp32P4::AngularKnob(float val, int x, int y, int sa, int ea, int radius, float w, float sizeh, float minVal, float maxVal, float outlineSize, int32_t colorOuter, int32_t colorInner, int flareSize, int startAlpha, int endAlpha, int flareDir, int32_t color2){
  int dir = (maxVal < minVal);
  ea += 180; sa += 180;
  float krad = sizeh;// / 2;
  float angularRange = ea - sa - w;
  float range = abs(maxVal - minVal) + 1;
  float angularSegment = angularRange / range;
  float pos;
  StoreCursPos();
  if (!dir){
    pos = (angularSegment * val) + sa;
  } else {
    pos = ea - (angularSegment * val) - w;
  }
  lastAngle = pos;
  float xy1[2];
  float xy2[2];
  MoveTo(x, y);
  Orbit(pos, radius, xy1);
  Orbit(pos + w, radius, xy2);
  if(flareSize > 0 && flareDir == FLARE_EXTERNAL) LineAAflare(xy1[0], xy1[1], xy2[0], xy2[1], krad, krad, flareSize, startAlpha, endAlpha, flareDir, color2);
  LineAA(xy1[0], xy1[1], xy2[0], xy2[1], krad, krad, colorOuter);
  if(outlineSize > 0) LineAA(xy1[0], xy1[1], xy2[0], xy2[1], krad - outlineSize, krad - outlineSize, colorInner);
  if(flareSize > 0 && flareDir == FLARE_INTERNAL) LineAAflare(xy1[0], xy1[1], xy2[0], xy2[1], krad, krad, flareSize, startAlpha, endAlpha, flareDir, color2);
  RestoreCursPos();
}

/****************************************************************************/
/*!
  @brief  Draws a flare around a circle
  @param  x - x center position of circle
  @param  y - y center position of circle
  @param  radius - radius of circle
  @param  flareSize - Size of knob flare
  @param  startAlpha -  Opacity level of flare start
  @param  endAlpha - Opacity level of flare end
  @param  flareDir - direction of flare FLARE_EXTERNAL, FLARE_INTERNAL
  @param  color -  RGB565 flare color
*/
/****************************************************************************/
void gfx4desp32P4::CircleFlare(float x, float y, int radius, int flareSize, int startAlpha, int endAlpha, int flareDir, uint32_t color){
  int n;
  StoreCursPos();
  if(startAlpha > 255) startAlpha = 255;
  if(endAlpha > 255) endAlpha = 255;
  if(startAlpha < 0) startAlpha = 0;
  if(endAlpha < 0) endAlpha = 0;
  float flareInc;
  int alphaRange = endAlpha - startAlpha;
  flareInc = (float)alphaRange / flareSize;
  AlphaBlend(ON);
  flareDir = 1;
  if(flareDir < 0) flareDir = -1;
  for(n = 0; n < flareSize; n++) {
	radius += flareDir;
    AlphaBlendLevel(startAlpha + (n * flareInc));
    CircleAA(x, y, radius, 2, color);
  }
  AlphaBlend(OFF);
  RestoreCursPos();
}

/****************************************************************************/
/*!
  @brief  Draws a flare around a rounded rectangle
  @param  x - x top left position
  @param  y - y top left position
  @param  x1 - x bottom right position
  @param  y1 - y bottom right position
  @param  r - radius of rounded rectangle corners
  @param  flareSize - Size of flare
  @param  startAlpha - Opacity level of flare start
  @param  endAlpha - Opacity level of flare end
  @param  flareDir - direction of flare FLARE_EXTERNAL, FLARE_INTERNAL
  @param  color - RGB565 flare color
*/
/****************************************************************************/
void gfx4desp32P4::RoundRectFlare(int16_t x, int16_t y, int16_t x1, int16_t y1,
    int16_t r, int flareSize, int startAlpha, int endAlpha, int flareDir, uint32_t color){
  int n;
  StoreCursPos();
  if(startAlpha > 255) startAlpha = 255;
  if(endAlpha > 255) endAlpha = 255;
  if(startAlpha < 0) startAlpha = 0;
  if(endAlpha < 0) endAlpha = 0;
  float flareInc;
  int alphaRange = endAlpha - startAlpha;
  flareInc = (float)alphaRange / flareSize;
  int d1 = 1, d2 = -1;
  if (flareDir < 0){
    d1 = -1; d2 = 1;
  }
  AlphaBlend(ON);
  for(n = 0; n < flareSize; n++) {
    r+= d1; x+= d2, y+= d2, x1+= d1; y1+= d1;
    if(r < 1) r = 1;
    AlphaBlendLevel(startAlpha + (n * flareInc));
    RoundRectAA(x, y, x1, y1, r, 2, color);
  }
  AlphaBlend(OFF);
  RestoreCursPos();
}

/****************************************************************************/
/*!
  @brief  Draws a flare around a rectangle
  @param  x - x top left position
  @param  y - y top left position
  @param  x1 - x bottom right position
  @param  y1 - y bottom right position
  @param  flareSize - Size of flare
  @param  startAlpha - Opacity level of flare start
  @param  endAlpha - Opacity level of flare end
  @param  flareDir - direction of flare FLARE_EXTERNAL, FLARE_INTERNAL
  @param  color - RGB565 flare color
*/
/****************************************************************************/
void gfx4desp32P4::RectangleFlare(int16_t x, int16_t y, int16_t x1, int16_t y1,
    int flareSize, int startAlpha, int endAlpha, int flareDir, uint32_t color){
    RoundRectFlare(x, y, x1, y1, 1, flareSize, startAlpha, endAlpha, flareDir, color);
}

/****************************************************************************/
/*!
  @brief  Draws a flare around a line
  @param  x - x line co-ords 1
  @param  y - y line co-ords 1
  @param  x1 - x line co-ords 2
  @param  y1 - y line co-ords 2
  @param  r - radius of line end 1
  @param  r1 - radius of line end 2
  @param  startAlpha - Opacity level of flare start
  @param  endAlpha - Opacity level of flare end
  @param  flareDir - direction of flare FLARE_EXTERNAL, FLARE_INTERNAL
  @param  color - RGB565 flare color
*/
/****************************************************************************/
void gfx4desp32P4::LineAAflare(float x, float y, float x1, float y1,
    float r, float r1, int flareSize, int startAlpha, int endAlpha, int flareDir, uint32_t color){
  int n;
  if(startAlpha > 255) startAlpha = 255;
  if(endAlpha > 255) endAlpha = 255;
  if(startAlpha < 0) startAlpha = 0;
  if(endAlpha < 0) endAlpha = 0;
  float flareInc;
  int alphaRange = endAlpha - startAlpha;
  flareInc = (float)alphaRange / flareSize;
  int d1 = 1, d2 = -1;
  if (flareDir < 0){
    d1 = -1; d2 = 1;
  }
  AlphaBlend(ON);
  for(n = 0; n < flareSize; n++) {
    r+= d1; r1+= d1;// x+= d2, y+= d2, x1+= d1; y1+= d1;
    if(r < 0.5) r = 0.5;
	if(r1 < 0.5) r1 = 0.5;
    AlphaBlendLevel(startAlpha + (n * flareInc));
    LineAA(x, y, x1, y1, r, r1, color);
  }
  AlphaBlend(OFF);
}

/****************************************************************************/
/*!
  @brief  Draws a dithered gradient filled rectangle
  @param  x - x top left position
  @param  y - y top left position
  @param  x1 - x bottom right position
  @param  y1 - y bottom right position
  @param  colfrom - RGB565 gradient from color
  @param  colto - RGB565 gradient to color
  @param  Orientation - Orientation of the gradient HORIZONTAL, VERTICAL
*/
/****************************************************************************/
void gfx4desp32P4::GradientRectangleFilled(int x1, int y1, int x2, int y2, int32_t colfrom, int32_t colto, bool Orientation){
  int cf = GetFrameBuffer();
  DrawDitheredGradientRectToFrameBuffer(cf, x1, y1, x2, y2, colfrom, colto, Orientation);  
}

/****************************************************************************/
/*!
  @brief  Prints an image from an area in a frame buffer to x y cursor coordinates
  @param  x1 - x top left position
  @param  y1 - y top left position
  @param  x1 - x bottom right position
  @param  y1 - y bottom right position
  @param  stepSize - Optional, size in pixels of height of chunk to be printed
*/
/****************************************************************************/
void gfx4desp32P4::PrintImageFromFrameBuffer(int fbuf, int x1, int y1, int x2, int y2, int stepSize){
	int startx, starty;
	startx = cursor_x;
	int cbuf = frame_buffer;
	int lc = 0;
	int h = y2 - y1 + 1;
	int w = x2 - x1 + 1;
	int w1 = w;
	int th;
	int nll = 0;
	int pipos = 0;
	int tfsh = fsh;
	int llastfsh;
	llastfsh = lastfsh;
	if (stepSize != -1){
	  fsh = stepSize;
	}
	int stps = h / fsh;
	int rem = h - (stps * fsh);
	int trem = rem;
	startx = cursor_x;
	if ((startx + w) > textXmax) w = textXmax - (startx + w);
	if (nl) println("");
	int tth;
	while (stps --){
	  if(stepSize != -1){
		th = stepSize;
	  } else {
	    th = fsh;
	  }
	  tth = th;
	  nll = 0;
      StartWrite();
	  while (th --){
	  	DrawToframebuffer(fbuf);
	    ReadLine(x1, y1 + pipos + nll, w, linebuff);
	    DrawToframebuffer(cbuf);
	    WriteLine(startx, cursor_y + nll, w, linebuff); 
		nll ++;
	  }
	  EndWrite();
      pipos += tth;
	  lc ++;
	  if (stps != 0) println("");
    }
	if (rem > 0){
	  fsh = rem;
	  lastfsh = rem;
	  println("");
	  nll = 0;
	  int ty;
	  while (rem --){
	  	DrawToframebuffer(fbuf);
	    ReadLine(x1, y1 + pipos + nll, w, linebuff);
	    DrawToframebuffer(cbuf);
	    ty = cursor_y + nll + tth - trem;
		if (ty <= getScrollareaY1()) WriteLine(startx, ty, w, linebuff); 
		nll ++;
	  }
	}
	fsh = tfsh;
	lastfsh = llastfsh;
}

bool gfx4desp32P4::CheckBoundaryJPEG(int tx2, int ty2, int tw2, int th2, int ttw, int tth){
    int tclipx1, tclipx2, tclipy1, tclipy2;
	if (frame_buffer < CANVAS_BUFFER){
		tclipx1 = clipx1; tclipx2 = clipx2; tclipy1 = clipy1; tclipy2 = clipy2;
	} else if (frame_buffer < CANVAS_BUFFER_ARGB){
		tclipx1 = 0; tclipx2 = __canvasWidth - 1; 0; tclipy2 = __canvasHeight - 1;
	} else {
		tclipx1 = 0; tclipx2 = __canvasWidthARGB - 1; tclipy1 = 0; tclipy2 = __canvasHeightARGB - 1;
	}	
	if (tx2 > tclipx2 || ty2 > tclipy2 || (tx2 + tw2 - 1) < tclipx1 || (ty2 + th2 - 1) < tclipy1) return false;
    int tx1 = 0;
    int ty1 = 0;
    int tw1 = tw2;
    int th1 = th2;
	if (ttw == -1 && tth == -1){
		tw1 = tw2;
		th1 = th2;
	} else {
		tw1 = tw2;
		th1 = th2;
		tw2 = ttw;
		th2 = tth;
	}
    int dist = 0; 
    if (tx2 < tclipx1){
		dist = tclipx1 - tx2;
		tx1 = dist;
		tw1 -= dist;
		tx2 += dist;
		tw2 -= dist;
    }
    if (ty2 < tclipy1){
		dist = tclipy1 - ty2;
		ty1 = dist;
		th1 -= dist;
		ty2 += dist;
		th2 -= dist;
    }
    if ((tx2 + tw2 - 1) > tclipx2){
		dist = (tx2 + tw2 - 1) - tclipx2;
		tw1 -= dist;
		tw2 -= dist;
    }	
    if ((ty2 + th2 - 1) > tclipy2){
        dist = (ty2 + th2 - 1) - tclipy2;
		th1 -= dist;
		th2 -= dist;
    }
    _bCoords[B_XPOS1] = tx1; _bCoords[B_YPOS1] = ty1; _bCoords[B_WIDTH1] = tw1; _bCoords[B_HEIGHT1] = th1; 
    _bCoords[B_XPOS2] = tx2; _bCoords[B_YPOS2] = ty2; _bCoords[B_WIDTH2] = tw2; _bCoords[B_HEIGHT2] = th2;
    return true;		
}

void gfx4desp32P4::SetCanvasDimensions(int w, int h){
	__canvasWidth = w;
	__canvasHeight = h;
	if (rotation > 1) {
		altst_hres = h;
		altst_vres = w;
	} else {
		altst_hres = w;
		altst_vres = h;
	}
}

void gfx4desp32P4::SetBlendCanvasDimensions(int w, int h){
	__canvasWidthARGB = w;
	__canvasHeightARGB = h;
	if (rotation > 1) {
		altst_hresARGB = h;
		altst_vresARGB = w;
	} else {
		altst_hresARGB = w;
		altst_vresARGB = h;
	}
}

void gfx4desp32P4::ShowCanvas(int x, int y, int w, int h, int tx, int ty){
	//if (rotation != 0) return;
	int ttx, tty, ttw, tth;
	int fbtemp = frame_buffer;
	if (x < 0) x = 0; 
	if (y < 0) y = 0;
	if ((x + w - 1) > __canvasWidth) x = __canvasWidth - w - 1;
	if ((y + h - 1) > __canvasHeight) y =  __canvasHeight - h - 1;
	uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
	uint16_t* pto = (uint16_t*)SelectFB(CANVAS_BUFFER);
	//DrawToframebuffer(CANVAS_BUFFER);
	TranslateCoords(x, y, w, h, 0, CANVAS_BUFFER);
	ttx = _translated[TRANS_X];
	tty = _translated[TRANS_Y];
	ttw = _translated[TRANS_W];
	tth = _translated[TRANS_H];
	TranslateCoords(tx, ty, w, h, 0, frame_buffer);
	
	DrawToframebuffer(fbtemp);
	ppaAccelerator.scaleRotateImageFB(pto, altst_hres, altst_vres, ttx, tty, ttw, tth, tpto, _translated[TRANS_W], _translated[TRANS_H], false, /*_translated[TRANS_DEG]*/0, false, false, _translated[TRANS_X], _translated[TRANS_Y], 0, 0, colorFMT24, frame_buffer == CANVAS_BUFFER_ARGB);
	__canvasXpos = x; __canvasYpos = y;
	__canvasSXpos = tx; __canvasSYpos = ty;
	__canvasSWidth = w; __canvasSHeight = h;
	__canvasWindow = true;
}

void gfx4desp32P4::ShowCanvas(int fb, int fb2, int px, int py, int x, int y, int w, int h, int tx, int ty){
	//if (rotation != 0) return;
	int ttx, tty, ttw, tth;
	int ppx, ppy, ppw, pph;
	int fbtemp = frame_buffer;
	bool setARGB = false;
	int sfb_hres = st_hres; int sfb_vres = st_vres;
	if (fb >= CANVAS_BUFFER && fb < CANVAS_BUFFER_ARGB){
		sfb_hres = altst_hres; sfb_vres = altst_vres;
	}
	int opFMT = 0;
	if (x < 0) x = 0; 
	if (y < 0) y = 0;
	if ((x + w - 1) > __canvasWidthARGB) x = __canvasWidthARGB - w - 1;
	if ((y + h - 1) > __canvasHeightARGB) y =  __canvasHeightARGB - h - 1;
	uint16_t* tpto = (uint16_t*)SelectFB(frame_buffer);
	uint16_t* pto2 = (uint16_t*)SelectFB(fb2);
	uint16_t* pto = (uint16_t*)SelectFB(fb);
	if (fb2 == CANVAS_BUFFER_ARGB) setARGB = true;
	//DrawToframebuffer(CANVAS_BUFFER);
	TranslateCoords(px, py, w, h, 0, fb);
	ppx = _translated[TRANS_X];
	ppy = _translated[TRANS_Y];
	ppw = _translated[TRANS_W];
	pph = _translated[TRANS_H];
	TranslateCoords(x, y, w, h, 0, fb2);
	ttx = _translated[TRANS_X];
	tty = _translated[TRANS_Y];
	ttw = _translated[TRANS_W];
	tth = _translated[TRANS_H];
	TranslateCoords(tx, ty, w, h, 0, frame_buffer);
	DrawToframebuffer(fbtemp);
	if (colorFMT24) opFMT = 1;
	//ppaAccelerator.scaleRotateImageFB(pto, altst_hres, altst_vres, ttx, tty, ttw, tth, tpto, _translated[TRANS_W], _translated[TRANS_H], false, /*_translated[TRANS_DEG]*/0, false, false, _translated[TRANS_X], _translated[TRANS_Y], 0, 0, colorFMT24, frame_buffer == CANVAS_BUFFER_ARGB);
	ppaAccelerator.blend(pto, sfb_hres, sfb_vres, ppx, ppy, ppw, pph, pto2, altst_hresARGB, altst_vresARGB, ttx, tty, ttw, tth, tpto, _translated[TRANS_W], _translated[TRANS_H], false, _translated[TRANS_X], _translated[TRANS_Y], colorFMT24, setARGB, opFMT);
	__canvasPXposARGB = px; __canvasPYposARGB = py;
	__canvasFBARGB = fb; __canvasFB2ARGB = fb2;
	__canvasXposARGB = x; __canvasYposARGB = y;
	__canvasSXposARGB = tx; __canvasSYposARGB = ty;
	__canvasSWidthARGB = w; __canvasSHeightARGB = h;
	__canvasWindowARGB = true;
}

void gfx4desp32P4::UpdateCanvas(int tx, int ty){
	if (!__canvasWindow) return;
	if (tx != 0xffff && ty != 0xffff){
		ShowCanvas(tx, ty, __canvasSWidth, __canvasSHeight, __canvasSXpos, __canvasSYpos);
	} else {
		ShowCanvas(__canvasXpos, __canvasYpos, __canvasSWidth, __canvasSHeight, __canvasSXpos, __canvasSYpos);
	}
}

void gfx4desp32P4::UpdateBlendCanvas(int tx, int ty){
	if (!__canvasWindowARGB) return;
	if (tx != 0xffff && ty != 0xffff){
		ShowCanvas(__canvasFBARGB, __canvasFB2ARGB, __canvasPXposARGB, __canvasPYposARGB, tx, ty, __canvasSWidthARGB, __canvasSHeightARGB, __canvasSXposARGB, __canvasSYposARGB);
	} else {
		ShowCanvas(__canvasFBARGB, __canvasFB2ARGB, __canvasPXposARGB, __canvasPYposARGB, __canvasXposARGB, __canvasYposARGB, __canvasSWidthARGB, __canvasSHeightARGB, __canvasSXposARGB, __canvasSYposARGB);
	}
}

void gfx4desp32P4::PPAblendTransAlpha(uint16_t btcol, bool btEN, uint8_t balphVal, bool baEN){
	ppaAccelerator.setTransAlpha(btcol, btEN, balphVal, baEN);
}

void gfx4desp32P4::setJPEGoutputBufferDimension(uint32_t w, uint32_t h){
	JPEGoutBufferSize = (w + 16) * (h + 16) * 2;
}

void gfx4desp32P4::setJPEGinputBufferSize(uint32_t s){
	JPEGinBufferSize = s;
}

/****************************************************************************/
/*!
  @brief  Opens a MJPEG video file and set playback parameters
  @param  fname - URL of MJPEG file stored in uSD
  @param  x - x top left position of video
  @param  y - y top left position
  @param  TargetFPS - Target FPS, may not meet this target dependant on size
  @param  altw - width to be scaled to
  @param  alth - height to be scaled to
  @param  bool - true if video restarts at the end
  @returns true if vido open successful
*/
/****************************************************************************/
bool gfx4desp32P4::OpenMJPEGvideoFile(String fname, int x, int y, float TargetFPS, int altw, int alth, bool loop){
	fname = "/" + fname;
	if (mjpeg) mjpeg.close();
	mjpeg = SD_MMC.open((char*)fname.c_str());
	if(mjpeg) _videoOK = true;
	_videoIsFile = true;
	_videoSource = MJPEG_VIDEO_FILE;
    _videoRestart = loop;
	_videoFileSize = mjpeg.size();
	OpenMJPEGvideo(TargetFPS, x, y, altw, alth);
	return _videoOK;
}

bool gfx4desp32P4::OpenMJPEGvideoArray(const uint8_t* array, uint32_t arraySize, int x, int y, float TargetFPS, int altw, int alth, bool loop){
	_videoArray = array;
	_videoOK = true;
	_videoIsFile = false;
	_videoSource = MJPEG_VIDEO_ARRAY;
    _videoRestart = loop;
	_videoFileSize = arraySize;
	OpenMJPEGvideo(TargetFPS, x, y, altw, alth);
	return _videoOK;
}

bool gfx4desp32P4::OpenMJPEGvideoArray(uint8_t* array, uint32_t arraySize, int x, int y, float TargetFPS, int altw, int alth, bool loop){
	_videoArray = array;
	_videoOK = true;
	_videoIsFile = false;
	_videoSource = MJPEG_VIDEO_ARRAY;
    _videoRestart = loop;
	_videoFileSize = arraySize;
	OpenMJPEGvideo(TargetFPS, x, y, altw, alth);
	return _videoOK;
}

void gfx4desp32P4::OpenMJPEGvideo(float Tfps, int x, int y, int altw, int alth){
	_videoDel = 1;
	_videoTnow = millis();
	if (!_videoBufInit){
		_videoBuf = (uint8_t*)heap_caps_aligned_alloc(64, MJPEG_BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
		_videoBufInit = true;
	}
	float tfps = 1000000;
	tfps = tfps / (Tfps * 1000) * 1000;
	_videoTFtimeInt = (int64_t)tfps;
	_videoTFtime = _videoTFtimeInt / 1000;
	_videoReadSize = MJPEG_BUFFER_SIZE >> 1;
	_videoChunk = _videoReadSize;
    _videoOfst = 0;
    _videoAddr = 0;
    _videoRestarting = false;
    _videoPos = 0;
    _videoFrameNum = 0;
	_videoX = x;
	_videoY = y;
	_videoW = 0;
	_videoH = 0;
	_videoAltw = altw;
	_videoAlth = alth;
	_videoGetDim = true;
	_videoArrayPos = 0;
	_videoSkippedFrames = 0;
	_videAspectDone = false;
	_videoTofset = 0;
}

/****************************************************************************/
/*!
  @brief  Close an open MJPEG video file
  @returns nothing
*/
/****************************************************************************/
void gfx4desp32P4::CloseMJPEGvideo(){
	if (_videoSource == MJPEG_VIDEO_FILE) mjpeg.close();
	_videoOK = false;
	_videoOfst = 0;
	_videoAddr = 0;
	_videoRestarting = false;
	_videoPos = 0;
	_videoFrameNum = 0;
	_videoRestart = false;
}

void gfx4desp32P4::MJPEGkeepAspectRatio(bool kasp){
	_videoKeepAspect = kasp;
}

void gfx4desp32P4::MJPEGTargetFPS(float fps){
	float tfps = 1000000;
	tfps = tfps / (fps * 1000) * 1000;
	_videoTFtimeInt = (int64_t)tfps;
	_videoTFtime = _videoTFtimeInt / 1000;
}

void gfx4desp32P4::MJPEGsetRepeat(bool r){
	_videoRestart = r;
}

void gfx4desp32P4::MJPEGvideoTimingReset(){
	_videoTnow = micros();
	_videoWaitTime = micros() - _videoTnow;
	_videoTofset = 0;
}

int gfx4desp32P4::MJPEGgetWidth(){
	return (_videoW);
}

int gfx4desp32P4::MJPEGgetHeight(){
	return (_videoH);
}

/****************************************************************************/
/*!
  @brief  Play the next frame of previously opened MJPEG vide file
  @returns The number of current video frame
*/
/****************************************************************************/
int64_t gfx4desp32P4::PlayMJPEGnextFrame(bool frcSW){
	uint32_t httpmjpegAvail; 
	_videoWaitTime = micros() - _videoTnow;
	if (_videoWaitTime < (_videoTFtimeInt - _videoTofset)) return _videoFrameNum;
	_videoTnow = micros();
	if (_videoFrameNum > 0) _videoTofset += (_videoWaitTime - _videoTFtimeInt) ;
	if(!_videoOK) return 0;
	if((_videoAddr < _videoFileSize || _videoChunk > 1) || _videoFileSize == -1){
		if (_videoChunk && _videoSource == MJPEG_VIDEO_FILE){
			if (_videoChunk > 0) mjpeg.read(_videoBuf + _videoOfst, _videoChunk);
		} else if (_videoSource == MJPEG_VIDEO_ARRAY){
			if (_videoChunk > 0) memcpy(_videoBuf + _videoOfst, _videoArray + _videoArrayPos, _videoChunk);
			_videoArrayPos += _videoChunk;
		}
		while (!(_videoBuf[_videoPos] == 0xff && _videoBuf[_videoPos + 1] == 0xd9)){
			_videoPos ++;
			if ((((_videoAddr + _videoChunk) >= _videoFileSize) || _videoArrayPos >= _videoFileSize) && _videoFileSize != -1){
				_videoPos = 0;
				if (_videoSource == MJPEG_VIDEO_FILE){
					mjpeg.seek(0);
				} else if (_videoSource == MJPEG_VIDEO_ARRAY){
					_videoArrayPos = 0;
				}
				_videoChunk = _videoReadSize;
				_videoOfst = 0;
				_videoAddr = 0;
				_videoPos = 0;
				_videoFrameNum = 0;
				_videoSkippedFrames = 0;
				if (_videoRestart == true){
					_videoRestarting = true;
				} else {
					_videoRestarting = false;
					return MJPEG_FILE_END;
				}
			}
		}
		int twO;
		if(!_videoRestarting){
			_videoPos += 1;
			_videoFrameNum ++;
			_videoAddr += _videoPos;
			if (((_videoW * _videoH) << 1) > JPEGoutputbufferSize) return MJPEG_SIZE_EXCEEDS_BUFFER;
			if (_videoPos > MJPEG_BUFFER_SIZE) return MJPEG_INPUT_SIZE_TOO_BIG;
			if (_videoFrameNum > 0){
				DecodeJPEGfromArray(_videoBuf/* + _videoFrameStartOffset*/, _videoPos - _videoFrameStartOffset + 2, _videoW, _videoH, frcSW);
				_videoW = _jpegWidth ;
				_videoH = _jpegHeight;
			}
			memcpy(_videoBuf, _videoBuf + _videoPos, _videoReadSize - _videoPos);
			if (_videoFrameNum > 0 && _videoW > 0){
				if (_videoAltw != -1){
					DrawJPEGbuffer(_videoX, _videoY, _jpegWidth, _jpegHeight, twO, _videoAltw, _videoAlth);
				} else {
					DrawJPEGbuffer(_videoX, _videoY, _jpegWidth, _jpegHeight, twO, _jpegHeight, _jpegWidth);
				}
			}
			_videoChunk = _videoPos;
			_videoOfst = _videoReadSize - _videoPos;
			_videoPos = 0;
		}
		_videoRestarting = false;
		if (_videoFrameNum == 1){
			if (_videoKeepAspect && _videAspectDone == false){
				if(_jpegWidth > _jpegHeight){
					float asp = (float)_jpegHeight / (_jpegWidth);
					float nh = (float)_videoAltw * asp;
					int oldvh = _videoAlth;
					_videoAlth = nh;
					int barsht = (oldvh - _videoAlth) / 2;
					RectangleFilledPPA(_videoX, _videoY, _videoX + _videoAltw - 1, _videoY + oldvh - 1, BLACK);
					_videoY += barsht;
				}				
				if(_jpegWidth < _jpegHeight){
					float asp = (float)_jpegWidth / _jpegHeight;
					float nw = (float)_videoAlth * asp;
					int oldvw = _videoAltw;
					_videoAltw = nw;
					int barsht = (oldvw - _videoAltw) / 2;
					RectangleFilledPPA(_videoX, _videoY, _videoX + oldvw - 1, _videoY + _videoAlth - 1, BLACK);
					_videoX += barsht;
				}
				_videAspectDone = true;
			}
			_oldJheight = _videoH; _oldJwidth = _videoW;
		}
	}
	return _videoFrameNum;
}

/****************************************************************************/
/*!
  @brief  Draws a JPEG image that may be embedded in a file 
  @param  tfile - the file that was openend previously - will return if not opened
  @param  top - the position in the file. Can be 0 if file contains 1 JPEG image
  @param  x - x position to draw the decoded image
  @param  y - y position to draw the decoded image
  @param  forceSoft - bool true if the software JPEG decoder is preferred
  @param  altw - the scaled width of the image (optional)
  @param  alth - the scaled height of the image (optional)
  @returns true if vido open successful
*/
/****************************************************************************/
void gfx4desp32P4::DrawJPEGinFile(File tfile, uint32_t tpos, uint32_t tfsize, int x, int y, bool forceSoft, int altw, int alth){
	if (!tfile) return;
	if (!JPEGinit) InitializeJPEG();
	if (forceSoft){
		if (jpegConfig == false){
			if (!JPEGinit) InitializeJPEG();
			jpeg_cfg.outbuf = (uint8_t *)rx_buf;
			jpeg_cfg.outbuf_size = rx_buffer_size;
			jpeg_cfg.indata = (uint8_t *)JPEGinp;
			jpeg_cfg.out_format = JPEG_IMAGE_FORMAT_RGB565;
			jpeg_cfg.out_scale = JPEG_IMAGE_SCALE_0;
			jpeg_cfg.flags.swap_color_bytes = 0;
			jpegConfig = true;
		}
		jpeg_cfg.indata_size = tfsize;
		jpeg_cfg.indata = (uint8_t *)JPEGinp;
		tfile.seek(tpos);
		tfile.read(JPEGinp, tfsize);
		esp_jpeg_decode(&jpeg_cfg, &outimg);
		JPEGiSize[0] = outimg.width;
		JPEGiSize[1] = outimg.height;
	} else {
		uint32_t out_size = 0;
	    tfile.seek(tpos);
	    tfile.read(tx_buf, tfsize);
	    jpeg_decoder_get_info(tx_buf, tfsize, &header_info);
	    JPEGiSize[0] = header_info.width;
	    JPEGiSize[1] = header_info.height;
		jpeg_decoder_process(jpgd_handle, &decode_cfg_rgb, tx_buf, tfsize, rx_buf, rx_buffer_size, &out_size);
	}
	int ttwO = 0;
	if (altw != -1){
		DrawJPEGbuffer(x, y, JPEGiSize[0], JPEGiSize[1], ttwO, altw, alth);
	} else {
		DrawJPEGbuffer(x, y, JPEGiSize[0], JPEGiSize[1], ttwO, JPEGiSize[0], JPEGiSize[1]);
	}
}

void gfx4desp32P4::InitializeJPEG(){
    decode_eng_cfg = {
        .timeout_ms = 40,
    };
	decode_cfg_rgb = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
    };
    decode_cfg_gray = {
        .output_format = JPEG_DECODE_OUT_FORMAT_GRAY,
    };
    rx_mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    tx_mem_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER,
    };
	ESP_ERROR_CHECK(jpeg_new_decoder_engine(&decode_eng_cfg, &jpgd_handle));
    rx_buffer_size = 0;
    rx_buf = (uint8_t*)jpeg_alloc_decoder_mem(JPEGoutputbufferSize, &rx_mem_cfg, &rx_buffer_size);
	JPEGinputbufferInit = true;
    tx_buffer_size = 0;
    JPEGhardInpBuffSize = JPEGoutputbufferSize >> 1;
	tx_buf = (uint8_t*)jpeg_alloc_decoder_mem(JPEGhardInpBuffSize >> 1, &tx_mem_cfg, &tx_buffer_size);
    JPEGoutputbufferInit = true;
    JPEGinit = true;
}

void gfx4desp32P4::MergeFrameBuffersPPA(uint8_t fbto, uint8_t fbfrom1, uint8_t fbfrom2, int alph) {
	ppaAccelerator.setTransAlpha(BLACK, true, alph, alph != 255);
	int sfb_hres = st_hres; int sfb_vres = st_vres;
	if (fbfrom1 >= CANVAS_BUFFER && fbfrom1 < CANVAS_BUFFER_ARGB){
		sfb_hres = altst_hres; sfb_vres = altst_vres;
	}
	int ttx, tty, ttw, tth;
	int ppx, ppy, ppw, pph;
	int w = width; int h = height;
	int opFMT = 0;
	bool setARGB = false;
	uint16_t* tpto = (uint16_t*)SelectFB(fbto);
	uint16_t* pto2 = (uint16_t*)SelectFB(fbfrom2);
	uint16_t* pto = (uint16_t*)SelectFB(fbfrom1);
	if (fbfrom2 == CANVAS_BUFFER_ARGB) setARGB = true;
	TranslateCoords(0, 0, w, h, 0, fbfrom1);
	ppx = _translated[TRANS_X];
	ppy = _translated[TRANS_Y];
	ppw = _translated[TRANS_W];
	pph = _translated[TRANS_H];
	TranslateCoords(0, 0, w, h, 0, fbfrom2);
	ttx = _translated[TRANS_X];
	tty = _translated[TRANS_Y];
	ttw = _translated[TRANS_W];
	tth = _translated[TRANS_H];
	TranslateCoords(0, 0, w, h, 0, fbto);
	if (colorFMT24) opFMT = 1;
	ppaAccelerator.blend(pto, sfb_hres, sfb_vres, ppx, ppy, ppw, pph, pto2, st_hres, st_vres, ttx, tty, ttw, tth, tpto, _translated[TRANS_W], _translated[TRANS_H], false, _translated[TRANS_X], _translated[TRANS_Y], colorFMT24, setARGB, opFMT);
}
	

