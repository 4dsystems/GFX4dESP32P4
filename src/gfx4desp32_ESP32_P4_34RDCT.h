#ifndef __GFX4D_ESP32_P4_34RDCT__
#define __GFX4D_ESP32_P4_34RDCT__

#define DISPLAY_TOUCH_TYPE DISP_TOUCH_CTP
#define ESP32_34RD_SWAP_TOUCHY 0
#define ESP32_34RD_SWAP_TOUCHXY 3

#include "gfx4desp32_mipi_common.h"
#include "gfx4desp32_mipi_panel_t.h"

#define TOUCH_CONTROLLER TOUCH_CONTROLLER_GT911

#include "esp_err.h"
#include "esp_log.h"
#include "hal/lcd_hal.h"
#include "hal/lcd_ll.h"

#include <Arduino.h>

#define EXAMPLE_LCD_NAME                JD9365
#define ESP32P4_34RD_WIDTH               (800)
#define ESP32P4_34RD_HEIGHT              (800)
//#define ESP32P4_784_COLOR_BITS          (ESP_PANEL_LCD_COLOR_BITS_RGB565)
                                                // or `ESP_PANEL_LCD_COLOR_BITS_RGB565`
#define ESP32P4_34RD_DSI_PHY_LDO_ID      (3)     // -1 if not used
#define ESP32P4_34RD_DSI_LANE_NUM        (2)     // ESP32-P4 supports 1 or 2 lanes
#define ESP32P4_34RD_DSI_LANE_RATE_MBPS  (1200)  /* Single lane bit rate, should consult the LCD supplier or check the
                                                 * LCD drive IC datasheet for the supported lane rate.
                                                 * ESP32-P4 supports max 1500Mbps
                                                 */
#define ESP32P4_34RD_DPI_CLK_MHZ         (60)
//#define ESP32P4_784_DPI_COLOR_BITS      (ESP32P4_784_COLOR_BITS)
#define ESP32P4_34RD_DPI_TIMINGS_HPW     (20)
#define ESP32P4_34RD_DPI_TIMINGS_HBP     (20)
#define ESP32P4_34RD_DPI_TIMINGS_HFP     (40)
#define ESP32P4_34RD_DPI_TIMINGS_VPW     (4)
#define ESP32P4_34RD_DPI_TIMINGS_VBP     (12)
#define ESP32P4_34RD_DPI_TIMINGS_VFP     (24)
#define ESP32P4_34RD_USE_EXTERNAL_CMD    (5)
#define ESP32P4_34RD_INPUT_COLOUR_FMT	(16)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your board spec ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define ESP32P4_34RD_RST_IO          (23)    // Set to -1 if not used
#define ESP32P4_34RD_BL_IO           (22)    // Set to -1 if not used
#define ESP32P4_34RD_BL_ON_LEVEL     (1)
#define ESP32P4_34RD_BL_OFF_LEVEL    (!ESP32P4_34RD_BL_ON_LEVEL)


class gfx4desp32_ESP32_P4_34RDCT : public gfx4desp32_mipi_panel_t
{
private:
    const char* TAG = "gfx4desp32_ESP32P4_101CT";
    int bk_pin = ESP32P4_34RD_BL_IO;
    int bk_on_lvl = ESP32P4_34RD_BL_ON_LEVEL;
    int bk_off_lvl = ESP32P4_34RD_BL_OFF_LEVEL;
	
	int hres = ESP32P4_34RD_WIDTH;
	int vres = ESP32P4_34RD_HEIGHT;
	int touch_type = 0x00;//ESP32_70_SWAP_TOUCHY + (DISPLAY_TOUCH_TYPE << 1) + (TOUCH_CONTROLLER << 4);
	
	int config[20] = {
		NAME_JD9365B,
		ESP32P4_34RD_WIDTH,
		ESP32P4_34RD_HEIGHT,		
		ESP32P4_34RD_DSI_PHY_LDO_ID,
		ESP32P4_34RD_DSI_LANE_NUM,
		ESP32P4_34RD_DSI_LANE_RATE_MBPS,
		ESP32P4_34RD_DPI_CLK_MHZ,
		ESP32P4_34RD_DPI_TIMINGS_HPW,
		ESP32P4_34RD_DPI_TIMINGS_HBP,
		ESP32P4_34RD_DPI_TIMINGS_HFP,
		ESP32P4_34RD_DPI_TIMINGS_VPW,
		ESP32P4_34RD_DPI_TIMINGS_VBP,
		ESP32P4_34RD_DPI_TIMINGS_VFP,
		ESP32P4_34RD_USE_EXTERNAL_CMD,
		ESP32P4_34RD_RST_IO,
		ESP32P4_34RD_INPUT_COLOUR_FMT,
	};

public:
    gfx4desp32_ESP32_P4_34RDCT();
    ~gfx4desp32_ESP32_P4_34RDCT();
};

#endif // __GFX4D_ESP32_P4_34RDCT__