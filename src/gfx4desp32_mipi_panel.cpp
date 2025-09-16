
#include "gfx4desp32_mipi_panel.h"

#define gfx_Swap(a, b)                                                             \
    {                                                                          \
        int32_t tab = a;                                                       \
        a = b;                                                                 \
        b = tab;                                                               \
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

#include "esp_err.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "hal/lcd_hal.h"
#include "hal/lcd_ll.h"
#include "esp_ldo_regulator.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_jd9365.h"
#include "esp_log.h"
#include "rom/cache.h"
#include "esp_rom_gpio.h"
#include "Mipi_Init_Codes.h"
#include "Wire.h"

//#include "esp_async_memcpy.h"
#include "esp_heap_caps.h"
#include "hal/cache_hal.h"

#define LCD_RGB_DATA_ENDIAN_MODE LCD_RGB_DATA_ENDIAN_LITTLE
#define LCD_DPI_COLOR_BITS ESP_PANEL_LCD_COLOR_BITS_RGB565

#include "esp_check.h"
#include "es8311.h"

#define EXAMPLE_SAMPLE_RATE 16000
#define EXAMPLE_VOICE_VOLUME 85                  // 0 - 100
#define EXAMPLE_MIC_GAIN (es8311_mic_gain_t)(3)  // 0 - 7
#define EXAMPLE_RECV_BUF_SIZE (10000)

static es8311_handle_t es_handle;
const char *TAG = "esp32p4_i2s_es8311";

esp_err_t es8311_codec_init(void) {
  /*es8311_handle_t */es_handle = es8311_create(0, 0x18/*ES8311_ADDRRES_0*/);
  
  ESP_RETURN_ON_FALSE(es_handle, ESP_FAIL, TAG, "es8311 create failed");
  const es8311_clock_config_t es_clk = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = EXAMPLE_SAMPLE_RATE * 256,
    .sample_frequency = EXAMPLE_SAMPLE_RATE
  };

  ESP_ERROR_CHECK(es8311_init(es_handle, &es_clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16));
  ESP_RETURN_ON_ERROR(es8311_sample_frequency_config(es_handle, es_clk.mclk_frequency, es_clk.sample_frequency), TAG, "set es8311 sample frequency failed");
  ESP_RETURN_ON_ERROR(es8311_microphone_config(es_handle, false), TAG, "set es8311 microphone failed");

  ESP_RETURN_ON_ERROR(es8311_voice_volume_set(es_handle, EXAMPLE_VOICE_VOLUME, NULL), TAG, "set es8311 volume failed");
  ESP_RETURN_ON_ERROR(es8311_microphone_gain_set(es_handle, EXAMPLE_MIC_GAIN), TAG, "set es8311 microphone gain failed");
  return ESP_OK;
}


gfx4desp32_mipi_panel::gfx4desp32_mipi_panel(
    int * config, int hres, int vres, int bk_pin, int bk_on_level,
    int bk_off_level, int ttype)
    : gfx4desp32P4() {

    //this->panel_config = panel_config;
    this->bk_config.pin = bk_pin;
    this->bk_config.on_level = bk_on_level;
    this->bk_config.off_level = bk_off_level;
    backlight = bk_pin;
    if ((ttype & 0x01) == 1){
		__TImode = true;
	} else {
		__TImode = false;
	}
	st_hres = hres;
	st_vres = vres;
	Pconfig = &config[0];
}

gfx4desp32_mipi_panel::~gfx4desp32_mipi_panel() {}

void gfx4desp32_mipi_panel::FrameBufferCopy(uint8_t* srcbuff, int srcX1, int srcY1, int srcX2, int srcY2, uint8_t* destbuff, int destX, int destY){
  /*
  esp_async_fbcpy_trans_desc_t fbcpy_trans_config = {
	.src_buffer = srcbuff,
    .dst_buffer = (void *)SelectFB(frame_buffer),
    .src_buffer_size_x = st_hres,
    .src_buffer_size_y = st_hres,
    .dst_buffer_size_x = st_hres,
    .dst_buffer_size_y = st_vres,
    .src_offset_x = srcX1,
    .src_offset_y = srcY1,
    .dst_offset_x = destX,
    .dst_offset_y = destY,
    .copy_size_x = srcX2 - srcX1,
    .copy_size_y = srcY2 - srcY1,
    .pixel_format_unique_id = {
      .color_type_id = mipi_dpi_panel->in_color_format,
    }
  };
  */
}

void gfx4desp32_mipi_panel::DisplayInit(int* mpconfig, int freq) {
  // Turn on the power for MIPI DSI PHY, so it can go from "No Power" state to "Shutdown" state
  esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
  esp_ldo_channel_config_t ldo_mipi_phy_config = {
    .chan_id = 3,       //EXAMPLE_MIPI_DSI_PHY_PWR_LDO_CHAN,
    .voltage_mv = 2500  //EXAMPLE_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
  };
  ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));
  ESP_LOGI(TAG, "MIPI DSI PHY Powered on");

  // create MIPI DSI bus first, it will initialize the DSI PHY as well
  esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
  esp_lcd_dsi_bus_config_t bus_config = {
    .bus_id = 0,
    .num_data_lanes = 2,
    .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
    .lane_bit_rate_mbps = mpconfig[MIPI_DSI_LANE_RATE_MBPS],
  };
  ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));

  ESP_LOGI(TAG, "Install MIPI DSI LCD control IO");
  esp_lcd_panel_io_handle_t mipi_dbi_io;
  // we use DBI interface to send LCD commands and parameters
  esp_lcd_dbi_io_config_t dbi_config = {
    .virtual_channel = 0,
    .lcd_cmd_bits = 8,    // according to the LCD spec
    .lcd_param_bits = 8,  // according to the LCD spec
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &mipi_dbi_io));

  ESP_LOGI(TAG, "Install MIPI DSI LCD data panel");
  //esp_lcd_panel_handle_t mipi_dpi_panel = NULL;
  esp_lcd_dpi_panel_config_t dpi_config;
  if (mpconfig[INPUT_COLOUR_FMT] == 24){
  dpi_config = {
    .virtual_channel = 0,
    .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
	.dpi_clock_freq_mhz = mpconfig[MIPI_DPI_CLK_MHZ],
    .in_color_format = LCD_COLOR_FMT_RGB888, // LCD_COLOR_FMT_RGB888,
    //.out_color_format = LCD_COLOR_FMT_RGB888, // Output as RGB888
	.video_timing = {
      .h_size = mpconfig[MIPI_WIDTH],
      .v_size = mpconfig[MIPI_HEIGHT],
      .hsync_pulse_width = mpconfig[MIPI_DPI_TIMINGS_HPW],
      .hsync_back_porch = mpconfig[MIPI_DPI_TIMINGS_HBP],
      .hsync_front_porch = mpconfig[MIPI_DPI_TIMINGS_HFP],
      .vsync_pulse_width = mpconfig[MIPI_DPI_TIMINGS_VPW],
      .vsync_back_porch = mpconfig[MIPI_DPI_TIMINGS_VBP],
      .vsync_front_porch = mpconfig[MIPI_DPI_TIMINGS_VFP],
    },
    .flags = {
      .use_dma2d = true,   // use DMA2D to copy draw buffer into frame buffer
      .disable_lp = true,  // disable low power mode
    }
  };
  } else {
  dpi_config = {
    .virtual_channel = 0,
    .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
    .dpi_clock_freq_mhz = mpconfig[MIPI_DPI_CLK_MHZ],
    .in_color_format = LCD_COLOR_FMT_RGB565, // LCD_COLOR_FMT_RGB888,
	.video_timing = {
      .h_size = mpconfig[MIPI_WIDTH],
      .v_size = mpconfig[MIPI_HEIGHT],
      .hsync_pulse_width = mpconfig[MIPI_DPI_TIMINGS_HPW],
      .hsync_back_porch = mpconfig[MIPI_DPI_TIMINGS_HBP],
      .hsync_front_porch = mpconfig[MIPI_DPI_TIMINGS_HFP],
      .vsync_pulse_width = mpconfig[MIPI_DPI_TIMINGS_VPW],
      .vsync_back_porch = mpconfig[MIPI_DPI_TIMINGS_VBP],
      .vsync_front_porch = mpconfig[MIPI_DPI_TIMINGS_VFP],
    },
    .flags = {
      .use_dma2d = true,   // use DMA2D to copy draw buffer into frame buffer
      .disable_lp = true,  // disable low power mode
    }
  };	  
  }
  if (freq != 0){
	  dpi_config.dpi_clock_freq_mhz = freq;
  }
  
  jd9365_vendor_config_t vendor_config = {
    .mipi_config = {
      .dsi_bus = mipi_dsi_bus,
      .dpi_config = &dpi_config,
    },
  };
  
  if (mpconfig[MIPI_USE_EXTERNAL_CMD] == 1){
	vendor_config.init_cmds = lcd_init_cmd1;
    vendor_config.init_cmds_size = sizeof(lcd_init_cmd1) / sizeof(lcd_init_cmd1[0]);  
  } else if (mpconfig[MIPI_USE_EXTERNAL_CMD] == 2){
    vendor_config.init_cmds = lcd_init_cmd2;
    vendor_config.init_cmds_size = sizeof(lcd_init_cmd2) / sizeof(lcd_init_cmd2[0]);
  } else if (mpconfig[MIPI_USE_EXTERNAL_CMD] == 3){
	vendor_config.init_cmds = lcd_init_cmd3;
    vendor_config.init_cmds_size = sizeof(lcd_init_cmd3) / sizeof(lcd_init_cmd3[0]);  
  }else if (mpconfig[MIPI_USE_EXTERNAL_CMD] == 4){
	vendor_config.init_cmds = lcd_init_cmd4;
    vendor_config.init_cmds_size = sizeof(lcd_init_cmd4) / sizeof(lcd_init_cmd4[0]);  
  }else if (mpconfig[MIPI_USE_EXTERNAL_CMD] == 5){
	vendor_config.init_cmds = lcd_init_cmd5;
    vendor_config.init_cmds_size = sizeof(lcd_init_cmd5) / sizeof(lcd_init_cmd5[0]);  
  }else if (mpconfig[MIPI_USE_EXTERNAL_CMD] == 6){
	vendor_config.init_cmds = lcd_init_cmd6;
    vendor_config.init_cmds_size = sizeof(lcd_init_cmd6) / sizeof(lcd_init_cmd6[0]);    
  }

  esp_lcd_panel_dev_config_t lcd_dev_config = {
    .reset_gpio_num = mpconfig[MIPI_DSI_RESET],
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    .bits_per_pixel = 16,  // 16 bits per pixel for RGB565
    .vendor_config = &vendor_config,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_jd9365(mipi_dbi_io, &lcd_dev_config, &mipi_dpi_panel));

  ESP_ERROR_CHECK(esp_lcd_panel_reset(mipi_dpi_panel));
  ESP_ERROR_CHECK(esp_lcd_panel_init(mipi_dpi_panel));

  void *fbptr = NULL;

  ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(mipi_dpi_panel, 1, &fbptr));
  ESP_LOGI(TAG, "MIPI LCD Frame Buffer allocated at %p size", fbptr);

  fb = (uint8_t *)fbptr;
}

void gfx4desp32_mipi_panel::DisplayControl(uint8_t cmd, uint32_t val) {
    
}

int gfx4desp32_mipi_panel::SpeakerVolume(int v){
	int retvol;
	es8311_voice_volume_set(es_handle, v, &retvol);
	return retvol;
}

int gfx4desp32_mipi_panel::MicrophoneGain(int g){
	switch(g){
		case 0:
			es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_MIN);
			break;
		case 1:
			es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_0DB);
			break;
		case 2:
			es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_6DB);
			break;
		case 3:
			es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_12DB);
			break;
		case 4:
			es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_18DB);
			break;
		case 5:
			es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_24DB);
			break;
		case 6:
			es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_30DB);
			break;
		case 7:
			es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_36DB);
			break;
		case 8:
			es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_42DB);
			break;
		case 9:
			es8311_microphone_gain_set(es_handle, ES8311_MIC_GAIN_MAX);
			break;
	}
	return g;
}

void gfx4desp32_mipi_panel::AudioInit(){
	pinMode(53, OUTPUT);
    digitalWrite(53, HIGH);
	es8311_codec_init();
}

void gfx4desp32_mipi_panel::DisplayControl(uint8_t cmd) {
    
}


void gfx4desp32_mipi_panel::__begin() {
    if (!psramFound()) {
		psramInit();
	}
	Wire.begin(7, 8, 400000);
	if (changePCLK) {
        DisplayInit(Pconfig, PCLKval);
        changePCLK = false;
    } else {
		DisplayInit(Pconfig);
	}
	st_hres = Pconfig[MIPI_WIDTH];
	st_vres = Pconfig[MIPI_HEIGHT];
    AllocateFB(0);
	SelectFB(0);
    /*** Set some Initial variables ***/
    rotation = 2;
	clipX1pos = 0;
    clipY1pos = 0;
    clipX2pos = (int)st_hres - 1;
    clipY2pos = (int)st_vres - 1;
    ClipWindow(0, 0, (int)st_hres - 1,
        (int)st_vres - 1);
    Clipping(true);
    Clipping(false);
	
	__scrWidth = (int)st_hres << 1;
    __scrHeight = (int)st_vres;
    __fbSize = (st_vres) * __scrWidth;
    __width = st_hres;
    __height = st_vres;
    /*** Scroll window set to maximum for GFX4dESP32 compatibilty ***/
    scroll_X1 = 0;
    scroll_Y1 = 0;
    scroll_X2 = __width - 1;
    scroll_Y2 = __height - 1;

    ledcAttach(backlight, 25000, 10);

    if (I2CInit == false) {
        Wire.begin(7, 8, 400000);
        I2CInit = true;
    }
    DisplayType = DISP_INTERFACE_MIPI;	
	//const uint32_t PSRAM_ALIGN = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_DATA);
    //const uint32_t INTRAM_ALIGN = PSRAM_ALIGN;
	config = ASYNC_MEMCPY_DEFAULT_CONFIG();
	config.backlog = 64;
    //config.dma_burst_size = 1600;
	//config.sram_trans_align = 16;
    //config.psram_trans_align = 16;
    config.flags = 0;
	esp_async_memcpy_install_gdma_axi(&config, &driver); // install driver with default DMA engine
	//esp_async_fbcpy_install(&configfbcpy, &driverfbcpy);
}

/****************************************************************************/
/*!
  @brief  Set display brightness
  @param  cntrst    0 - 15
  @note   ESP32 requires pin setup for Analog Write.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::Contrast(int ctrst) {

	if (ctrst > 15)
        ctrst = 15;
    if (ctrst < 0)
        ctrst = 0;
    switch (ctrst) {
    case 15:
        ledcWrite(backlight, 1023);
        break;
    case 14:
        ledcWrite(backlight, 820);
        break;
    case 13:
        ledcWrite(backlight, 608);
        break;
    case 12:
        ledcWrite(backlight, 460);
        break;
    case 11:
        ledcWrite(backlight, 352);
        break;
    case 10:
        ledcWrite(backlight, 276);
        break;
    case 9:
        ledcWrite(backlight, 224);
        break;
    case 8:
        ledcWrite(backlight, 188);
        break;
    case 7:
        ledcWrite(backlight, 140);
        break;
    case 6:
        ledcWrite(backlight, 105);
        break;
    case 5:
        ledcWrite(backlight, 78);
        break;
    case 4:
        ledcWrite(backlight, 62);
        break;
    case 3:
        ledcWrite(backlight, 47);
        break;
    case 2:
        ledcWrite(backlight, 35);
        break;
    case 1:
        ledcWrite(backlight, 28);
        break;
    case 0:
        ledcWrite(backlight, 0);
        break;
    }

}

/****************************************************************************/
/*!
  @brief  Switch on / off backlight
  @param  blight - true / false
  @note  compatible function for early IoD's. Sets backlight fully on.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::BacklightOn(bool blight) {

	if (blight) {
        ledcWrite(backlight, 1023);
    }
    else {
        ledcWrite(backlight, 0);
    }

}

/****************************************************************************/
/*!
  @brief  Set Screen Orientation - resets scroll and clip window to maximum
  @param  r Rotation 0 - 3 (0, 1, 2, 3)
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::panelOrientation(uint8_t r) {
    rotation = r % 4;
	if(r == 0) rotation = 2;
	if(r == 1) rotation = 3;
	if(r == 2) rotation = 0;
	if(r == 3) rotation = 1;
    if (rotation < 2) {
        ClipWindow(0, 0, (int)st_hres - 1,
            (int)st_vres - 1);
        Clipping(1);
        Clipping(0);
        __width = st_hres;
        __height = st_vres;
    }
    else {
        ClipWindow(0, 0, (int)st_vres - 1,
            (int)st_hres - 1);
        Clipping(1);
        Clipping(0);
        __width = st_vres;
        __height = st_hres;
    }
    clippingON = false;
    wrGRAM = 0;
    scroll_X1 = 0;
    scroll_Y1 = 0;
    scroll_X2 = __width - 1;
    scroll_Y2 = __height - 1;
	scroll_window_store_x1 = scroll_X1;
	scroll_window_store_y1 = scroll_Y1;
	scroll_window_store_x2 = scroll_X2;
	scroll_window_store_y2 = scroll_Y2;
    /*
	if (wrap){
		if (textXmax > (__width - 1)){
            textXmax = __width - 1;			
		}			
	} else {
		if (textXmaxBAK > (__width - 1)){
            textXmaxBAK = __width - 1;			
		}	
	}
	*/
}

/**********************************************************************/
/*!
  @brief    Get height in current Orientation
  @returns  __height height in pixels
*/
/**********************************************************************/
int16_t gfx4desp32_mipi_panel::getHeight(void) { return __height; }

/**********************************************************************/
/*!
  @brief    Get width in current Orientation
  @returns  __width width in pixels
*/
/**********************************************************************/
int16_t gfx4desp32_mipi_panel::getWidth(void) { return __width; }

/**********************************************************************/
/*!
  @brief    Get Maximum Y scroll window value in current Orientation
  @returns  scroll_Y2  position in pixels
  @note     Compatipble GFX4dESP32 function used by write and newline.
*/
/**********************************************************************/
int16_t gfx4desp32_mipi_panel::getScrollareaY1() { return (int16_t)scroll_Y2; }

/****************************************************************************/
/*!
  @brief    Get current Screen Orientation
  @returns  rotation 0 - 3 (0, 1, 2, 3)
*/
/****************************************************************************/
uint8_t gfx4desp32_mipi_panel::getPanelOrientation(void) { return rotation; }

/****************************************************************************/
/*!
  @brief    Set Invert mode
  @param    Inv    true / false
  @note     Needs coding. Not sure how to deal with this on RGB displays
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::Invert(bool Inv) {
    //panel_jd9365_invert_color(&mipi_dpi_panel, Inv);
	#if defined(CONFIG_IDF_TARGET_ESP32S3)
    int panel_id = rgb_panel->panel_id;
	for (int i = 0; i < 16; i++) {
        esp_rom_gpio_connect_out_signal(RGB_InvertFix[i], lcd_periph_rgb_signals.panels[panel_id].data_sigs[i],
            Inv, false);
    }
	#endif
}

/****************************************************************************/
/*!
  @brief  Set GRAM window ready for wrGRAM/s functions.
  @param  x1 left X position in pixels
  @param  y1 top Y position in pixels
  @param  x2 right X position in pixels
  @param  y2 bottom Y position in pixels
  @note   sets global variables GRAMx1, GRAMy1, GRAMx2, GRAMy2 to window pos
          sets GRAMxpos to x1 start and GRAMypos to y1 start
          resets PixelPos to 0 and sets pixel count to size of window
          sets flag wrGRAM to true indicating GRAM window set.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::SetGRAM(int16_t x1, int16_t y1, int16_t x2,
    int16_t y2) {
    GRAMx1 = x1;
    GRAMy1 = y1;
    GRAMx2 = x2;
    GRAMy2 = y2;
    wrGRAM = true;
    GRAMxpos = x1;
    GRAMypos = y1;
    pixelPos = 0;
    pixelCount = (GRAMx2 - GRAMx1) * (GRAMy2 - GRAMy1);
}

/****************************************************************************/
/*!
  @brief  Set start write condition
  @param  none
  @note   StartWrite disables flushing after frame buffer write.
          Flush area will be updated by subsequent writes so that EndWrite
          will flush all affected
*/
/****************************************************************************/
bool gfx4desp32_mipi_panel::StartWrite() {
	if(writeFBonly) return true; // return if already initiated
	writeFBonly = true;
	low_Y = __scrHeight - 1;
	high_Y = 0;
	high_ypos = low_Y;
	return true;
}

/****************************************************************************/
/*!
  @brief  Set end write condition
  @param  none
  @note   EndWrite will enable flush functions and flush the entire area
          that needs to be flushed
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::EndWrite() {
	writeFBonly = false;
	if(low_Y > high_Y) gfx_Swap(low_Y, high_Y);
	high_Y ++;
	if(high_Y > __scrHeight) high_Y = __scrHeight - 1;
	if(low_Y < 0) low_Y = 0;
	if(low_Y > high_Y) return;
	if(low_Y > __scrHeight) flush_pending = false;
	#ifdef BIT24_OUTPUT
	uint32_t bytes_to_flush = (high_Y - low_Y) * __scrWidth24;
	uint32_t addr = low_Y * __scrWidth24;
	#else
	uint32_t bytes_to_flush = (high_Y - low_Y) * __scrWidth;
	uint32_t addr = low_Y * __scrWidth;	
	#endif
	if(addr < 0){
		addr = 0;
		bytes_to_flush += addr; 
	}
	#ifdef BIT24_OUTPUT
	if((addr + bytes_to_flush) > __fbSize24) bytes_to_flush = __fbSize24 - addr;
    if(bytes_to_flush == 0) bytes_to_flush = __scrWidth24;
	#else
	if((addr + bytes_to_flush) > __fbSize) bytes_to_flush = __fbSize - addr;
    if(bytes_to_flush == 0) bytes_to_flush = __scrWidth;
	#endif
	Cache_WriteBack_Addr(CACHE_MAP_MASK, (uint32_t)(fb + addr), bytes_to_flush);
}

/****************************************************************************/
/*!
  @brief  Fill entire frame buffer with colour and flush
  @param  color 16 bit RGB565 colour
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::FillScreen(uint16_t color) {
    uint32_t total = __fbSize >> 1; // total size of framebuffer in pixels
    uint8_t* pto = SelectFB(frame_buffer);
    while (total--) {
        pto[0] = color;
        pto[1] = color >> 8;
        pto += 2;
    }
    if(frame_buffer == visibleFB); FlushArea(0, __scrHeight  - 1, -1);
}

/****************************************************************************/
/*!
  @brief  Draw selected frame buffer to the display
  @param  fbnum - frame buffer number to draw.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::DrawFrameBuffer(uint8_t fbnum) {
    uint8_t* tfrom = SelectFB(fbnum);
    uint8_t* to;
    to = (uint8_t*)fb;
    memcpy(to, tfrom, __fbSize);
    if(frame_buffer == visibleFB) FlushArea(0, __scrHeight, -1);
}

/****************************************************************************/
/*!
  @brief  Write array of colour data to selected frame buffer to the display
  @param  fbnum - frame buffer number to write to.
  @param  offset - position in destination frame buffer number to write to.
  @param  data - arrary containing data.
  @param  len - length of data array.
  @note   function for direct frame buffer writing without any x or y position
          Care to be taken to not exceed the current frame buffer size.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::WriteToFrameBuffer(uint32_t offset, uint8_t* data, uint32_t len) {
    uint8_t* to = SelectFB(frame_buffer);
    uint8_t* pto;
    uint32_t pc = 0;
    uint8_t colM, colL;
    pto = to + offset;
    if (!alpha) {
        memcpy(pto, data, len);
    }
    else {
        while (len -= 2) {
            colM = data[pc++];
            colL = data[pc++];
            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
            pto[0] = __colour;
            pto[1] = __colour >> 8;
            pto += 2;
        }
    }
	uint32_t ty = offset /__scrWidth;
	if(frame_buffer == visibleFB) FlushArea(ty, ty + 1, -1);
}

/****************************************************************************/
/*!
  @brief  Write array of colour data to selected frame buffer to the display
  @param  fbnum - frame buffer number to write to.
  @param  offset - position in destination frame buffer number to write to.
  @param  data - arrary containing 16 bit data.
  @param  len - length of data array.
  @note   function for direct frame buffer writing without any x or y position
          Care to be taken to not exceed the current frame buffer size.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::WriteToFrameBuffer(uint32_t offset, uint16_t* data, uint32_t len) {
    uint8_t* to = SelectFB(frame_buffer);
    uint8_t* pto;
    uint32_t pc = 0;
    uint16_t tcol;
    uint8_t colM, colL;
    pto = to + offset;
    if (!alpha) {
        memcpy(pto, data, len << 1);
    }
    else {
        while (len--) {
            tcol = data[pc++];
            colM = tcol >> 8;
            colL = tcol & 0xff;
            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
            pto[0] = __colour;
            pto[1] = __colour >> 8;
            pto += 2;
        }
    }
	uint32_t ty = offset /__scrWidth;
	if(frame_buffer == visibleFB) FlushArea(ty, ty + 1, -1);
}

/****************************************************************************/
/*!
  @brief  Draw frame buffer area relative to GCI widget from selected to buffer
  to target buffer
  @param  fbnum - frame buffer number to draw from.
              ui - GCI widget index
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::DrawFrameBufferArea(uint8_t fbnum, int16_t ui) {
    DrawFrameBufferArea(fbnum, tuix[ui], tuiy[ui], tuix[ui] + tuiw[ui] - 1,
        tuiy[ui] + tuih[ui] - 1);
}

/****************************************************************************/
/*!
  @brief  Draw frame buffer area using screen co-ordinates from selected
              buffer to target buffer
  @param  fbnum - frame buffer number to draw from.
              x1 - left co-ordinate
              y1 - top co-ordinate
              x2 - right co-ordinate
              y2 - bottom co-ordinate
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::DrawFrameBufferArea(uint8_t fbnum, int16_t x1,
    int16_t y1, int16_t x2,
    int16_t y2) {
    uint8_t* tfrom = SelectFB(fbnum);
    uint8_t* to = SelectFB(frame_buffer);
    int32_t x_start = 0;
    int32_t y_start = 0;
    int32_t x_end = 0;
    int32_t y_end = 0;
    uint8_t colM, colL;
    Clipping(false);
    // set area depending on orientation
    switch (rotation) {
    case 0:
        x_start = x1;
        y_start = y1;
        x_end = x2;
        y_end = y2;
        break;
    case 1:
        x_start = (st_hres - 1) - x2;
        x_end = (st_hres - 1) - x1;
        y_start = (st_vres - 1) - y2;
        y_end = (st_vres - 1) - y1;
        break;
    case 3:
        x_start = st_hres - 1 - y2;
        x_end = st_hres - 1 - y1;
        y_start = x1;
        y_end = x2;
        break;
    case 2:
        x_start = y1;
        x_end = y2;
        y_start = st_vres - 1 - x2;
        y_end = st_vres - 1 - x1;
    }
    if (x_start >= st_hres || x_end < 0 || y_start >= st_vres || y_end < 0)
        return;
    if (x_start < 0)
        x_start = 0;
    if (y_start < 0)
        y_start = 0;
    if (x_end >= st_hres)
        x_end = st_hres - 1;
    if (y_end >= st_vres - 1)
        y_end = st_vres - 1;
    uint32_t s_width = x_end - x_start + 1;
    uint32_t s_height = y_end - y_start + 1;
    uint32_t pc = (y_start * __scrWidth) + (x_start << 1);
    to += pc;
    tfrom += pc;
    int twidth;
    while (s_height--) {
        if (!transalpha) {
            memcpy(to, tfrom, s_width << 1);
			//esp_async_memcpy(driver, to, tfrom, s_width << 1, NULL, NULL);
            to += __scrWidth;
            tfrom += __scrWidth;
        }
        else {
            twidth = s_width;
            while (twidth--) {
                colM = tfrom[1];
                colL = tfrom[0];
                if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                    if (alpha) {
                        calcAlpha(colL + (colM << 8), to[0] | (to[1] << 8), __alpha);
                        to[0] = __colour;
                        to[1] = __colour >> 8;
                    }
                    else {
                        to[1] = colM;
                        to[0] = colL;
                    }
                }
                to += 2;
                tfrom += 2;
            }
            to += (__scrWidth - (s_width << 1));
            tfrom += (__scrWidth - (s_width << 1));
        }
    }
    if (frame_buffer == visibleFB)
    FlushArea(y_start, y_end, -1);
}

/****************************************************************************/
/*!
  @brief  Draw frame buffer area relative to GCI widget from selected to buffer
  to target buffer at position x, y
  @param  fbnum - frame buffer number to draw from.
          ui    - GCI widget index
		  x     - target x position
		  y     - target y position
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::DrawFrameBufferAreaXY(uint8_t fbnum, int16_t ui, int x, int y) {
    DrawFrameBufferAreaXY(fbnum, tuix[ui], tuiy[ui], tuix[ui] + tuiw[ui] - 1,
        tuiy[ui] + tuih[ui] - 1, x, y);
}


/****************************************************************************/
/*!
  @brief  Draw frame buffer area using screen co-ordinates from selected
              buffer to target buffer at position x, y
  @param  fbnum - frame buffer number to draw from.
              x1 - left co-ordinate
              y1 - top co-ordinate
              x2 - right co-ordinate
              y2 - bottom co-ordinate
			  x  - target x position
			  y  - target y position
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::DrawFrameBufferAreaXY(uint8_t fbnum, int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x, int16_t y) {
    uint8_t* tfrom = SelectFB(fbnum);
    uint8_t* to = SelectFB(frame_buffer);
    int32_t x_start = 0;
    int32_t y_start = 0;
	int32_t tx_start = 0;
    int32_t ty_start = 0;
	int32_t tx = 0;
	int32_t ty = 0;
	int xe;
	int ye;
    int32_t x_end = 0;
    int32_t y_end = 0;
    uint8_t colM, colL;
	int tw, th;
    Clipping(false);
    // set area depending on orientation
    switch (rotation) {
    case 0:
        x_start = x1;
        y_start = y1;
		tx_start = x;
        ty_start = y;
        x_end = x2;
        y_end = y2;
        break;
    case 1:
        tw = x2 - x1 + 1;
		th = y2 - y1 + 1;
		xe = x + tw - 1;
		ye = y + th - 1;
		x_start = (st_hres - 1) - x2;
		tx_start = (st_hres - 1) - xe;
        x_end = (st_hres - 1) - x1;
        y_start = (st_vres - 1) - y2;
		ty_start = (st_vres - 1)- ye;
        y_end = (st_vres - 1) - y1;
        break;
    case 3:
		tw = y2 - y1 + 1;
		ye = y + tw - 1;
		//ye = st_hres - y;
		x_start = (st_hres - 1) - y2;
		tx_start = (st_hres - 1) - ye;
        x_end = (st_hres - 1) - y1;
        y_start = x1;
		ty_start = x;
        y_end = x2;
        break;
    case 2:        
		tw = x2 - x1 + 1;
		xe = x + tw - 1;
		//xe = hres - y
		x_start = y1;
		tx_start = y;
        x_end = y2;
        y_start = (st_vres - 1) - x2;
		ty_start = (st_vres - 1) - xe;
        y_end = (st_vres - 1) - x1 - 1;
    }
    int sub = 0;
	if (x_start >= st_hres || x_end < 0 || y_start >= st_vres || y_end < 0 || tx_start >= st_hres || (tx_start + (x_end - x_start + 1)) < 0 || ty_start >= st_vres || (ty_start + (y_end - y_start + 1)) < 0)
        return;
    if (x_start < 0)
        x_start = 0; 
    if (y_start < 0)
        y_start = 0;
	tx = tx_start;
	if (tx_start < 0){
        sub = tx_start;
		tx_start = 0;
	}
    if (x_end >= st_hres)
        x_end = st_hres - 1;
    if (y_end >= st_vres - 1)
        y_end = st_vres - 1;
    uint32_t s_width = x_end - x_start + 1;
    uint32_t s_height = y_end - y_start + 1;
	int drawn = 0;
    int32_t offset = 0;
	if ((tx_start + s_width) > st_hres) offset = (tx_start + s_width - st_hres); 
	uint32_t pc = (y_start * __scrWidth) + (x_start << 1);	
	uint32_t pct = 0; 
	if (ty_start >= 0){
		pct = (ty_start * __scrWidth)/* + (tx_start << 1)*/;
	}
	if (sub >= 0) pct += (tx_start << 1);
    to += pct;
    tfrom += pc;
	ty = ty_start;
	int txx = tx;
	int32_t st = 0;
    int twidth;
	int count;
    while (s_height--) {
		count = 0;
		if (!transalpha) {
            if (ty >= 0 && ty < st_vres){
				//esp_async_memcpy(driver, to, tfrom + (abs(sub) << 1), (s_width - offset + sub) << 1, NULL, NULL);
				memcpy(to, tfrom + (abs(sub) << 1), (s_width - offset + sub) << 1);
				drawn ++;
			}
            if (ty >= 0) to += __scrWidth;
            tfrom += __scrWidth;
			ty ++;
        }
        else {
            twidth = s_width;
            if(ty >= 0 && ty < st_vres){
				drawn ++;
				while (twidth--) {
					if (txx >= 0 && txx < st_hres){
						colM = tfrom[1];
						colL = tfrom[0];
						if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
							if (alpha) {
								calcAlpha(colL + (colM << 8), to[0] | (to[1] << 8), __alpha);
								to[0] = __colour;
								to[1] = __colour >> 8;
							}
							else {
								to[1] = colM;
								to[0] = colL;
							}
						}
					    to += 2;
					    count ++;
					}
					tfrom += 2;
					txx ++;
				}
		    } else {
				if (ty >= 0) to += (s_width << 1);
				tfrom += (s_width << 1);
			}
			txx = tx;
			ty ++;
            if (ty > 0 && transalpha) to += (__scrWidth - (count << 1)/*(s_width << 1)*/);
            tfrom += (__scrWidth - (s_width << 1));
			count = 0;
        }
    }
    if (frame_buffer == visibleFB){
	  if (ty_start < 0) ty_start = 0; 	
	  if (drawn > 0) FlushArea(ty_start, ty_start + drawn, -1);
	}
}

/****************************************************************************/
/*!
  @brief  Write buffer of pixels to selected frame buffer
  @param  *color - 8 bit colour buffer
  @param  length of buffer in pixels
  @note   Write to previously set GRAM window from byte array.
          Clipping handles out of bounds also which does have a speed penalty.
          Used for writing from uSD images in byte chunks if needed.
          Flush occurs after GRAM window has been fully written to.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::WrGRAMs(uint8_t* color_data, uint32_t len) {
	if (!(wrGRAM))
        return; // return if GRAM window not set.
    uint32_t pc = 0;
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    uint8_t colM, colL;
    if (rotation == 0) {
        pto = tpto + (GRAMypos * __scrWidth) +
            (GRAMxpos << 1); // set frame buffer pointer to current x, y position
        while (len--) {
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) { // check if inside clip region
                if (frameBufferIData) {
                    colL = color_data[pc++];
                    colM = color_data[pc++];
                }
                else {
                    colM = color_data[pc++];
                    colL = color_data[pc++];
                }
                if (!transalpha) {
                    pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
                            pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            else {
                pc += 2;
            }
            GRAMxpos++;
            pto += 2;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + (GRAMypos * __scrWidth) +
                    (GRAMxpos << 1); // set new frame buffer pointer
            }
            pixelPos++;
        }
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
            (__scrWidth - ((GRAMxpos + 1) << 1));
        while (len--) {
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) {
                if (frameBufferIData) {
                    colL = color_data[pc++];
                    colM = color_data[pc++];
                }
                else {
                    colM = color_data[pc++];
                    colL = color_data[pc++];
                }
                if (!transalpha) {
                    pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
                            pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            else {
                pc += 2;
            }
            GRAMxpos++;
            pto -= 2;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
                    (__scrWidth - ((GRAMxpos + 1) << 1));
            }
            pixelPos++;
        }
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
        while (len--) {
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) {
                if (frameBufferIData) {
                    colL = color_data[pc++];
                    colM = color_data[pc++];
                }
                else {
                    colM = color_data[pc++];
                    colL = color_data[pc++];
                }
                if (!transalpha) {
                    pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
                            pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            else {
                pc += 2;
            }
            GRAMxpos++;
            pto += __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto =
                    tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
            }
            pixelPos++;
        }
    }
    if (rotation == 2) {
        pto = tpto + (GRAMypos << 1) + ((__scrHeight - GRAMxpos - 1) * __scrWidth);
        while (len--) {
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) {
                if (frameBufferIData) {
                    colL = color_data[pc++];
                    colM = color_data[pc++];
                }
                else {
                    colM = color_data[pc++];
                    colL = color_data[pc++];
                }
                if (!transalpha) {
                    pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
                            pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            else {
                pc += 2;
            }
            GRAMxpos++;
            pto -= __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + (GRAMypos << 1) +
                    ((__scrHeight - GRAMxpos - 1) * __scrWidth);
            }
            pixelPos++;
        }
    }
    if (GRAMypos >= GRAMy2  &&
        frame_buffer == visibleFB) { // if GRAM area is written to flush the area.
        wrGRAM = false;
        if(rotation == 0) FlushArea(GRAMy1, GRAMy2, -1); 
	    if(rotation == 1) FlushArea(__scrHeight - GRAMy2 - 1, __scrHeight - GRAMy1 - 1 , -1); 
	    if(rotation == 3) FlushArea(GRAMx1, GRAMx2, -1);
	    if(rotation == 2) FlushArea(__scrHeight - GRAMx2 - 1, __scrHeight - GRAMx1 - 1 , -1);
		// CPU writes data to PSRAM through DCache, data in PSRAM might not get
        // updated, so write back
    }
}

/****************************************************************************/
/*!
  @brief  Write flash array buffer of pixels to selected frame buffer
  @param  *color - 8 bit colour buffer
  @param  length of buffer in pixels
  @note   Write to previously set GRAM window from byte array.
          Clipping handles out of bounds also which does have a speed penalty.
          Used for writing from uSD images in byte chunks if needed.
          Flush occurs after GRAM window has been fully written to.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::WrGRAMs(const uint8_t* color_data, uint32_t len) {
    if (!(wrGRAM))
        return; // return if GRAM window not set.
    uint32_t pc = 0;
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    uint8_t colM, colL;

    if (rotation == 0) {
        pto = tpto + (GRAMypos * __scrWidth) +
            (GRAMxpos << 1); // set frame buffer pointer to current x, y position
        while (len--) {
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) { // check if inside clip region
                colM = color_data[pc++];
                colL = color_data[pc++];
                if (!transalpha) {
                    pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
                            pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            else {
                pc += 2;
            }
            GRAMxpos++;
            pto += 2;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + (GRAMypos * __scrWidth) +
                    (GRAMxpos << 1); // set new frame buffer pointer
            }
        }
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
            (__scrWidth - ((GRAMxpos + 1) << 1));
        while (len--) {
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) {
                colM = color_data[pc++];
                colL = color_data[pc++];
                if (!transalpha) {
                    pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
                            pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            else {
                pc += 2;
            }
            GRAMxpos++;
            pto -= 2;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
                    (__scrWidth - ((GRAMxpos + 1) << 1));
            }
        }
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
        while (len--) {
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) {
                colM = color_data[pc++];
                colL = color_data[pc++];
                if (!transalpha) {
                    pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
                            pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            else {
                pc += 2;
            }
            GRAMxpos++;
            pto += __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto =
                    tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
            }
        }
    }
    if (rotation == 2) {
        pto = tpto + (GRAMypos << 1) + ((__scrHeight - GRAMxpos - 1) * __scrWidth);
        while (len--) {
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) {
                colM = color_data[pc++];
                colL = color_data[pc++];
                if (!transalpha) {
                    pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
                            pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            else {
                pc += 2;
            }
            GRAMxpos++;
            pto -= __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + (GRAMypos << 1) +
                    ((__scrHeight - GRAMxpos - 1) * __scrWidth);
            }
        }
    }
	if (GRAMypos >= GRAMy2 &&
        frame_buffer == visibleFB) { // if GRAM area is written to flush the area.
        wrGRAM = false;
        if(rotation == 0) FlushArea(GRAMy1, GRAMy2, -1); 
	    if(rotation == 1) FlushArea(__scrHeight - GRAMy2 - 1, __scrHeight - GRAMy1 - 1 , -1); 
	    if(rotation == 3) FlushArea(GRAMx1, GRAMx2, -1);
	    if(rotation == 2) FlushArea(__scrHeight - GRAMx2 - 1, __scrHeight - GRAMx1 - 1 , -1);
		// CPU writes data to PSRAM through DCache, data in PSRAM might not get
        // updated, so write back
    }
}

/****************************************************************************/
/*!
  @brief  Flush frame buffer area recently written to
  @param  y1 start position in pixels
  @param  y2 end position in pixels
  @param  xpos x position in line
  @note   flush area will immediately flush defined area if StartWrite has not
          been initiated. If StartWrite has been initiated then the area will
          be updated only. If the area to be flushed is just a pixel then an
          area of just 2 bytes will be flushed
*/
/****************************************************************************/
void IRAM_ATTR gfx4desp32_mipi_panel::FlushArea(int y1, int y2, int xpos) {
	if(y1 < 0) y1 = 0;
	if(y2 > (__scrHeight)) y2 = __scrHeight;
	if (writeFBonly) { // just uptate flush area
		if (y1 < low_Y) low_Y = y1;
		if (y2 > high_Y) high_Y = y2;
		if (low_Y == high_Y) high_Y++;
		flush_pending = true;
		return;
	}
    if(y1 > y2) gfx_Swap(y1, y2);
	if(xpos != -1 && !flush_pending){ // if pixel and not in previous StartWrite condition
	  #ifdef BIT24_OUTPUT
	  Cache_WriteBack_Addr(CACHE_MAP_MASK, (uint32_t)(fb + (y1 * __width * 3) + (xpos * 3 )), 3);
	  #else
	  Cache_WriteBack_Addr(CACHE_MAP_MASK, (uint32_t)(fb + (y1 * __scrWidth) + (xpos << 1 )), 2); // flush pixel
	  #endif
	} else {
		#ifdef BIT24_OUTPUT
		uint32_t bytes_to_flush = (uint32_t)((y2 - y1 + 1) * __scrWidth24);
		int32_t addr = ((y1 * __scrWidth24) - 1);
		#else
		uint32_t bytes_to_flush = (uint32_t)(y2 - y1 + 1) * (st_hres << 1);
		int32_t addr = ((y1 * (st_hres << 1)) - 1);  
		#endif
		if(addr < 0){
			addr = 0;
			  //bytes_to_flush = abs(addr); 
		}
		#ifdef BIT24_OUTPUT
		if((addr + bytes_to_flush) > __fbSize24) bytes_to_flush = __fbSize24 - addr;
		if(bytes_to_flush == 0) bytes_to_flush = __scrWidth24;
		#else
		if((addr + bytes_to_flush) > __fbSize) bytes_to_flush = __fbSize - addr;
		if(bytes_to_flush < (st_hres << 1)) bytes_to_flush = st_hres << 1;
		#endif
		Cache_WriteBack_Addr(CACHE_MAP_MASK, (uint32_t)(fb + addr), bytes_to_flush); // flush area
	}
	flush_pending = false;
}

/****************************************************************************/
/*!
  @brief  Flush frame buffer area recently written to
      @param  x1 left start position in pixels
      @param  x2 right end position in pixels
  @param  y1 top start position in pixels
  @param  y2 bottom position in pixels
  @param  xpos x position in line
  @note   flush area will immediately flush defined area if StartWrite has not
          been initiated. If StartWrite has been initiated then the area will
          be updated only. If the area to be flushed is just a pixel then an
          area of just 2 bytes will be flushed
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::FlushArea(int x1, int x2, int y1, int y2, int xpos) {
    return; // left in for compatibility
}

/****************************************************************************/
/*!
  @brief  Write buffer of pixels to selected frame buffer
  @param  *color_data - 16 bit colour buffer
  @param  length of buffer in pixels
  @note   Write to previously set GRAM window from byte array.
          Clipping handles out of bounds also which does have a speed penalty.
          Used for writing from RAM locations in 16bit chunks if needed.
          Flush occurs after GRAM window has been fully written to.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::WrGRAMs(uint16_t* color_data, uint32_t len) {
	if (!(wrGRAM))
        return;
    uint32_t pc = 0;
    uint16_t tcol;
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    uint8_t colM, colL;
    if (rotation == 0) {
		pto = tpto + (GRAMypos * __scrWidth) + (GRAMxpos << 1);
        while (len--) {
            tcol = color_data[pc++];
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) {
                colM = tcol >> 8;
                colL = tcol & 0xff;
                if (!transalpha) {
					pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
							pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            GRAMxpos++;
			pto += 2;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
				pto = tpto + (GRAMypos * __scrWidth) + (GRAMxpos << 1);
            }
            pixelPos++;
        }
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
            (__scrWidth - ((GRAMxpos + 1) << 1));
        while (len--) {
            tcol = color_data[pc++];
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) {
                colM = tcol >> 8;
                colL = tcol & 0xff;
                if (!transalpha) {
                    pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
                            pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            GRAMxpos++;
            pto -= 2;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
                    (__scrWidth - ((GRAMxpos + 1) << 1));
            }
            pixelPos++;
        }
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
        while (len--) {
            tcol = color_data[pc++];
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) {
                colM = tcol >> 8;
                colL = tcol & 0xff;
                if (!transalpha) {
                    pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
                            pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            GRAMxpos++;
            pto += __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto =
                    tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
            }
            pixelPos++;
        }
    }
    if (rotation == 2) {
        pto = tpto + (GRAMypos << 1) + ((__scrHeight - GRAMxpos - 1) * __scrWidth);
        while (len--) {
            tcol = color_data[pc++];
            if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                GRAMypos <= clipY2) {
                colM = tcol >> 8;
                colL = tcol & 0xff;
                if (!transalpha) {
                    pto[1] = colM;
                    pto[0] = colL;
                }
                else {
                    if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                        if (alpha) {
                            calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                            pto[0] = __colour;
                            pto[1] = __colour >> 8;
                        }
                        else {
                            pto[1] = colM;
                            pto[0] = colL;
                        }
                    }
                }
            }
            GRAMxpos++;
            pto -= __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + (GRAMypos << 1) +
                    ((__scrHeight - GRAMxpos - 1) * __scrWidth);
            }
            pixelPos++;
        }
    }
    if (GRAMypos >= GRAMy2  &&
        frame_buffer == visibleFB) { // if GRAM area is written to flush the area.
        if(rotation == 0) FlushArea(GRAMy1, GRAMy2, -1); 
	    if(rotation == 1) FlushArea(__scrHeight - GRAMy2 - 1, __scrHeight - GRAMy1 - 1 , -1); 
	    if(rotation == 3) FlushArea(GRAMx1, GRAMx2, -1);
	    if(rotation == 2) FlushArea(__scrHeight - GRAMx2 - 1, __scrHeight - GRAMx1 - 1 , -1);
		wrGRAM = false;
    }
}

/****************************************************************************/
/*!
  @brief  Write buffer of pixels to selected frame buffer (compatible IoD
  Function)
  @param  *color_data - 32 bit colour buffer (2 x 16)
  @param  length of buffer in pixels
  @note   Write to previously set GRAM window from byte array.
          Clipping handles out of bounds also which does have a speed penalty.
          Used for writing from RAM locations in 16bit chunks if needed.
          Flush occurs after GRAM window has been fully written to.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::WrGRAMs(uint32_t* color_data, uint16_t len) {
    if (!(wrGRAM))
        return;
    uint32_t pc = 0;
    uint32_t tcol;
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    uint8_t div;
    uint8_t colM, colL;
    int innerloop;

    if (rotation == 0) {
        pto = tpto + (GRAMypos * __scrWidth) + (GRAMxpos << 1);
        while (len--) {
            innerloop = 2;
            div = 32;
            tcol = color_data[pc++];
            while (innerloop--) {
                if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                    GRAMypos <= clipY2) {
                    colM = tcol >> (div -= 8);
                    colL = tcol >> (div -= 8);
                    if (!transalpha) {
                        pto[1] = colM;
                        pto[0] = colL;
                    }
                    else {
                        if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                            if (alpha) {
                                calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                                pto[0] = __colour;
                                pto[1] = __colour >> 8;
                            }
                            else {
                                pto[1] = colM;
                                pto[0] = colL;
                            }
                        }
                    }
                }
                GRAMxpos++;
                pto += 2;
                if (GRAMxpos > GRAMx2) {
                    GRAMxpos = GRAMx1;
                    GRAMypos++;
                    pto = tpto + (GRAMypos * __scrWidth) + (GRAMxpos << 1);
                }
                pixelPos++;
            }
        }
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
            (__scrWidth - ((GRAMxpos + 1) << 1));
        while (len--) {
            innerloop = 2;
            div = 32;
            tcol = color_data[pc++];
            while (innerloop--) {
                if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                    GRAMypos <= clipY2) {
                    colM = tcol >> (div -= 8);
                    colL = tcol >> (div -= 8);
                    if (!transalpha) {
                        pto[1] = colM;
                        pto[0] = colL;
                    }
                    else {
                        if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                            if (alpha) {
                                calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                                pto[0] = __colour;
                                pto[1] = __colour >> 8;
                            }
                            else {
                                pto[1] = colM;
                                pto[0] = colL;
                            }
                        }
                    }
                }
                GRAMxpos++;
                pto -= 2;
                if (GRAMxpos > GRAMx2) {
                    GRAMxpos = GRAMx1;
                    GRAMypos++;
                    pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
                        (__scrWidth - ((GRAMxpos + 1) << 1));
                }
                pixelPos++;
            }
        }
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
        while (len--) {
            innerloop = 2;
            div = 32;
            tcol = color_data[pc++];
            while (innerloop--) {
                if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                    GRAMypos <= clipY2) {
                    colM = tcol >> (div -= 8);
                    colL = tcol >> (div -= 8);
                    if (!transalpha) {
                        pto[1] = colM;
                        pto[0] = colL;
                    }
                    else {
                        if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                            if (alpha) {
                                calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                                pto[0] = __colour;
                                pto[1] = __colour >> 8;
                            }
                            else {
                                pto[1] = colM;
                                pto[0] = colL;
                            }
                        }
                    }
                }
                GRAMxpos++;
                pto += __scrWidth;
                if (GRAMxpos > GRAMx2) {
                    GRAMxpos = GRAMx1;
                    GRAMypos++;
                    pto = tpto + (__scrWidth - (GRAMypos << 1) - 2) +
                        (GRAMxpos * __scrWidth);
                }
                pixelPos++;
            }
        }
    }
    if (rotation == 2) {
        pto = tpto + (GRAMypos << 1) + ((__scrHeight - GRAMxpos - 1) * __scrWidth);
        while (len--) {
            innerloop = 2;
            div = 32;
            tcol = color_data[pc++];
            while (innerloop--) {
                if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
                    GRAMypos <= clipY2) {
                    colM = tcol >> (div -= 8);
                    colL = tcol >> (div -= 8);
                    if (!transalpha) {
                        pto[1] = colM;
                        pto[0] = colL;
                    }
                    else {
                        if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                            if (alpha) {
                                calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                                pto[0] = __colour;
                                pto[1] = __colour >> 8;
                            }
                            else {
                                pto[1] = colM;
                                pto[0] = colL;
                            }
                        }
                    }
                }
                GRAMxpos++;
                pto -= __scrWidth;
                if (GRAMxpos > GRAMx2) {
                    GRAMxpos = GRAMx1;
                    GRAMypos++;
                    pto = tpto + (GRAMypos << 1) +
                        ((__scrHeight - GRAMxpos - 1) * __scrWidth);
                }
                pixelPos++;
            }
        }
    }
    if (GRAMypos >= GRAMy2  &&
        frame_buffer == visibleFB) { // if GRAM area is written to flush the area.
        if(rotation == 0) FlushArea(GRAMy1, GRAMy2, -1); 
	    if(rotation == 1) FlushArea(__scrHeight - GRAMy2 - 1, __scrHeight - GRAMy1 - 1 , -1); 
	    if(rotation == 3) FlushArea(GRAMx1, GRAMx2, -1);
	    if(rotation == 2) FlushArea(__scrHeight - GRAMx2 - 1, __scrHeight - GRAMx1 - 1 , -1);
		wrGRAM = false;
    }
}

/****************************************************************************/
/*!
  @brief  Write buffer of pixels to selected frame buffer
  @param  *color_data - 16 bit colour buffer
  @param  length of buffer in pixels
  @note   Write to previously set GRAM window from byte array.
          Fastest way to write GRAM. Not to be used for GCI or gfx4desp32
  functions. Used for LVGL sketches as no clipping or out of bounds supported.
          0 mode uses memcpy for frame buffer write.
          Flush occurs after GRAM window has been fully written to.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::pushColors(uint16_t* color_data, uint32_t len) {
    if (!(wrGRAM))
        return;
    uint32_t pc = 0;
    uint16_t tcol;
    uint16_t* tpto16 = (uint16_t*)SelectFB(frame_buffer);
    uint8_t* tpto = SelectFB(frame_buffer);
	uint8_t* pto;
	uint16_t* pto16;
    if (rotation == 0) {
        uint16_t* pfrom = color_data; // set pointer from 16bit array
        int16_t pcheight = GRAMy2 - GRAMy1 + 1;
        int16_t pcwidth = GRAMx2 - GRAMx1 + 1;
        pto = tpto + (GRAMypos * __scrWidth) +
            (GRAMxpos << 1); // set pointer to frame buffer
		pto16 = tpto16 + (GRAMypos * __width) + GRAMxpos;
        while (pcheight--) {
            esp_async_memcpy(driver, pto16, pfrom, pcwidth << 1, NULL, NULL);
			//memcpy(pto, pfrom, pcwidth << 1); // copy 'from' to 'to'
            GRAMypos++;
            pto = tpto + (GRAMypos * __scrWidth) +
    			(GRAMxpos << 1); // set new frame buffer pointer
			pto16 = tpto16 + (GRAMypos * __width) + GRAMxpos;
            pfrom += pcwidth;
            pixelPos += (pcwidth);
        }
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
            (__scrWidth - ((GRAMxpos + 1) << 1));
        while (len--) {
            tcol = color_data[pc++];
            pto[0] = tcol & 0xff;
            pto[1] = tcol >> 8;
            GRAMxpos++;
            pto -= 2;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
                    (__scrWidth - ((GRAMxpos + 1) << 1));
            }
            pixelPos++;
        }
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
        while (len--) {
            tcol = color_data[pc++];
            pto[0] = tcol & 0xff;
            pto[1] = tcol >> 8;
            GRAMxpos++;
            pto += __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto =
                    tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
            }
            pixelPos++;
        }
    }
    if (rotation == 2) {
        pto = tpto + (GRAMypos << 1) + ((__scrHeight - GRAMxpos - 1) * __scrWidth);
        while (len--) {
            tcol = color_data[pc++];
            pto[0] = tcol & 0xff;
            pto[1] = tcol >> 8;
            GRAMxpos++;
            pto -= __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + (GRAMypos << 1) +
                    ((__scrHeight - GRAMxpos - 1) * __scrWidth);
            }
            pixelPos++;
        }
    }
    if (GRAMypos >= GRAMy2 &&
        frame_buffer == visibleFB) { // if GRAM area is written to flush the area.
        wrGRAM = false;
        if(rotation == 0) FlushArea(GRAMy1, GRAMy2, -1); 
	    if(rotation == 1) FlushArea(__scrHeight - GRAMy2 - 1, __scrHeight - GRAMy1 - 1 , -1); 
	    if(rotation == 3) FlushArea(GRAMx1, GRAMx2, -1);
	    if(rotation == 2) FlushArea(__scrHeight - GRAMx2 - 1, __scrHeight - GRAMx1 - 1 , -1);
		// CPU writes data to PSRAM through DCache, data in PSRAM might not get
        // updated, so write back
    }
}

/****************************************************************************/
/*!
  @brief  Write buffer of pixels to selected frame buffer
  @param  *color_data - 8 bit colour buffer
  @param  length of buffer in pixels
  @note   Write to previously set GRAM window from byte array.
          Fastest way to write GRAM. Not to be used for GCI or gfx4desp32
  functions. Used for LVGL sketches as no clipping or out of bounds supported.
          0 mode uses memcpy for frame buffer write.
          Flush occurs after GRAM window has been fully written to.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::pushColors(uint8_t* color_data, uint32_t len) {
    if (!(wrGRAM))
        return;
    uint32_t pc = 0;
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    if (rotation == 0) {
        pto = tpto + (GRAMypos * __scrWidth) +
            (GRAMxpos << 1); // set pointer to frame buffer
        while (len--) {
            pto[1] = color_data[pc++];
            pto[0] = color_data[pc++];
            GRAMxpos++;
            pto += 2;
            if (GRAMxpos > GRAMx2) {

                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + (GRAMypos * __scrWidth) +
                    (GRAMxpos << 1); // set new frame buffer pointer
            }
            pixelPos++;
        }
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
            (__scrWidth - ((GRAMxpos + 1) << 1));
        while (len--) {
            pto[1] = color_data[pc++];
            pto[0] = color_data[pc++];
            GRAMxpos++;
            pto -= 2;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
                    (__scrWidth - ((GRAMxpos + 1) << 1));
            }
            pixelPos++;
        }
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
        while (len--) {
            pto[1] = color_data[pc++];
            pto[0] = color_data[pc++];
            GRAMxpos++;
            pto += __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto =
                    tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
            }
            pixelPos++;
        }
    }
    if (rotation == 2) {
        pto = tpto + (GRAMypos << 1) + ((__scrHeight - GRAMxpos - 1) * __scrWidth);
        while (len--) {
            pto[1] = color_data[pc++];
            pto[0] = color_data[pc++];
            GRAMxpos++;
            pto -= __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + (GRAMypos << 1) +
                    ((__scrHeight - GRAMxpos - 1) * __scrWidth);
            }
            pixelPos++;
        }
    }
    if (GRAMypos >= GRAMy2 &&
        frame_buffer == visibleFB) { // if GRAM area is written to flush the area.
        wrGRAM = false;
        if(rotation == 0) FlushArea(GRAMy1, GRAMy2, -1); 
	    if(rotation == 1) FlushArea(__scrHeight - GRAMy2 - 1, __scrHeight - GRAMy1 - 1 , -1); 
	    if(rotation == 3) FlushArea(GRAMx1, GRAMx2, -1);
	    if(rotation == 2) FlushArea(__scrHeight - GRAMx2 - 1, __scrHeight - GRAMx1 - 1 , -1);
		// CPU writes data to PSRAM through DCache, data in PSRAM might not get
        // updated, so write back
    }
}

void gfx4desp32_mipi_panel::pushColors(const uint8_t* color_data, uint32_t len) {
    if (!(wrGRAM))
        return;
    uint32_t pc = 0;
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    if (rotation == 0) {
        pto = tpto + (GRAMypos * __scrWidth) +
            (GRAMxpos << 1); // set pointer to frame buffer
        while (len--) {
            pto[1] = color_data[pc++];
            pto[0] = color_data[pc++];
            GRAMxpos++;
            pto += 2;
            if (GRAMxpos > GRAMx2) {

                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + (GRAMypos * __scrWidth) +
                    (GRAMxpos << 1); // set new frame buffer pointer
            }
        }
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
            (__scrWidth - ((GRAMxpos + 1) << 1));
        while (len--) {
            pto[1] = color_data[pc++];
            pto[0] = color_data[pc++];
            GRAMxpos++;
            pto -= 2;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
                    (__scrWidth - ((GRAMxpos + 1) << 1));
            }
        }
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
        while (len--) {
            pto[1] = color_data[pc++];
            pto[0] = color_data[pc++];
            GRAMxpos++;
            pto += __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto =
                    tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
            }
        }
    }
    if (rotation == 2) {
        pto = tpto + (GRAMypos << 1) + ((__scrHeight - GRAMxpos - 1) * __scrWidth);
        while (len--) {
            pto[1] = color_data[pc++];
            pto[0] = color_data[pc++];
            GRAMxpos++;
            pto -= __scrWidth;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
                pto = tpto + (GRAMypos << 1) +
                    ((__scrHeight - GRAMxpos - 1) * __scrWidth);
            }
        }
    }
    if (GRAMypos >= GRAMy2 &&
        frame_buffer == visibleFB) { // if GRAM area is written to flush the area.
        wrGRAM = false;
        if(rotation == 0) FlushArea(GRAMy1, GRAMy2, -1); 
	    if(rotation == 1) FlushArea(__scrHeight - GRAMy2 - 1, __scrHeight - GRAMy1 - 1 , -1); 
	    if(rotation == 3) FlushArea(GRAMx1, GRAMx2, -1);
	    if(rotation == 2) FlushArea(__scrHeight - GRAMx2 - 1, __scrHeight - GRAMx1 - 1 , -1);
		// CPU writes data to PSRAM through DCache, data in PSRAM might not get
        // updated, so write back
    }
}

/****************************************************************************/
/*!
  @brief  Write single pixel to selected frame buffer
  @param  color - 16 bit colour RGB565
  @note   Write to previously set GRAM window with RGB565 colour.
          Clipping handles out of bounds also which does have a speed penalty.
          Used for writing from RAM locations in single pixels.
          Flush occurs after GRAM window has been fully written to.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::WrGRAM(uint16_t color) {
    if (!(wrGRAM))
        return;
    if (transparency) {
        if (color == _transparentColor) {
            GRAMxpos++;
            if (GRAMxpos > GRAMx2) {
                GRAMxpos = GRAMx1;
                GRAMypos++;
            }
            pixelPos++;
            return;
        }
    }
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    if (rotation == 0) {
        pto = tpto + (GRAMypos * __scrWidth) + (GRAMxpos << 1);
        if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
            GRAMypos <= clipY2) {
            if(alpha) {
			  calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
			  pto[0] = __colour & 0xff;
              pto[1] = __colour >> 8;
			} else {
			  pto[0] = color & 0xff;
              pto[1] = color >> 8;
			}
        }
        GRAMxpos++;
        pto += 2;
        if (GRAMxpos > GRAMx2) {
            GRAMxpos = GRAMx1;
            GRAMypos++;
            pto = tpto + (GRAMypos * __scrWidth) + (GRAMxpos << 1);
        }
        pixelPos++;
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
            (__scrWidth - ((GRAMxpos + 1) << 1));
        if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
            GRAMypos <= clipY2) {
             if(alpha) {
			  calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
			  pto[0] = __colour & 0xff;
              pto[1] = __colour >> 8;
			} else {
			  pto[0] = color & 0xff;
              pto[1] = color >> 8;
			}
        }
        GRAMxpos++;
        pto -= 2;
        if (GRAMxpos > GRAMx2) {
            GRAMxpos = GRAMx1;
            GRAMypos++;
            pto = tpto + ((__scrHeight - GRAMypos - 1) * __scrWidth) +
                (__scrWidth - ((GRAMxpos + 1) << 1));
        }
        pixelPos++;
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
        if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
            GRAMypos <= clipY2) {
             if(alpha) {
			  calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
			  pto[0] = __colour & 0xff;
              pto[1] = __colour >> 8;
			} else {
			  pto[0] = color & 0xff;
              pto[1] = color >> 8;
			}
        }
        GRAMxpos++;
        pto += __scrWidth;
        if (GRAMxpos > GRAMx2) {
            GRAMxpos = GRAMx1;
            GRAMypos++;
            pto = tpto + (__scrWidth - (GRAMypos << 1) - 2) + (GRAMxpos * __scrWidth);
        }
        pixelPos++;
    }
    if (rotation == 2) {
        pto = tpto + (GRAMypos << 1) + ((__scrHeight - GRAMxpos - 1) * __scrWidth);
        if (GRAMxpos >= clipX1 && GRAMxpos <= clipX2 && GRAMypos >= clipY1 &&
            GRAMypos <= clipY2) {
             if(alpha) {
			  calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
			  pto[0] = __colour & 0xff;
              pto[1] = __colour >> 8;
			} else {
			  pto[0] = color & 0xff;
              pto[1] = color >> 8;
			}
        }
        GRAMxpos++;
        pto -= __scrWidth;
        if (GRAMxpos > GRAMx2) {
            GRAMxpos = GRAMx1;
            GRAMypos++;
            pto =
                tpto + (GRAMypos << 1) + ((__scrHeight - GRAMxpos - 1) * __scrWidth);
        }
        pixelPos++;
    }
    if (pixelPos >= pixelCount &&
        frame_buffer == visibleFB) { // if GRAM area is written to flush the area.
        wrGRAM = false;
    }
}

/****************************************************************************/
/*!
  @brief  Read a single pixel.
  @param  x left X position in pixels
  @param  y top Y position in pixels
  @note   returns RGB565 colour
*/
/****************************************************************************/
uint16_t gfx4desp32_mipi_panel::ReadPixel(uint16_t xrp, uint16_t yrp) {
    if (yrp > clipY2 || yrp < clipY1)
        return 0;
    if (xrp > clipX2 || yrp < clipX1)
        return 0;
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto = NULL;
    if (rotation == 0)
        pto = tpto + (yrp * __scrWidth) + (xrp << 1);
    if (rotation == 1)
        pto = tpto + ((__scrHeight - yrp - 1) * __scrWidth) +
        (__scrWidth - ((xrp + 1) << 1));
    if (rotation == 3) {
        int32_t temp = (__scrWidth - ((yrp + 1) << 1)) + (__scrWidth * xrp);
        if (temp < 0 || temp > __fbSize)
            return 0;
        pto = tpto + temp;
    }
    if (rotation == 2)
        pto = tpto + (yrp << 1) + ((__scrHeight - xrp - 1) * __scrWidth);
    return pto[0] + (pto[1] << 8);
}

uint16_t gfx4desp32_mipi_panel::ReadPixelFromFrameBuffer(uint16_t xrp, uint16_t yrp, uint8_t fB) {
    if (yrp > clipY2 || yrp < clipY1)
        return 0;
    if (xrp > clipX2 || yrp < clipX1)
        return 0;
    uint8_t* tpto = SelectFB(fB);
    uint8_t* pto;
    if (rotation == 0)
        pto = tpto + (yrp * __scrWidth) + (xrp << 1);
    if (rotation == 1)
        pto = tpto + ((__scrHeight - yrp - 1) * __scrWidth) +
        (__scrWidth - ((xrp + 1) << 1));
    if (rotation == 3) {
        int32_t temp = (__scrWidth - ((yrp + 1) << 1)) + (__scrWidth * xrp);
        if (temp < 0 || temp > __fbSize)
            return 0;
        pto = tpto + temp;
    }
    if (rotation == 2)
        pto = tpto + (yrp << 1) + ((__scrHeight - xrp - 1) * __scrWidth);
    return pto[0] + (pto[1] << 8);
}

/****************************************************************************/
/*!
  @brief  Read a line of pixels.
  @param  x left X position in pixels
  @param  y top Y position in pixels
  @param  w width of line
  @param  data - 16 bit user array
  @note   returns len
*/
/****************************************************************************/
uint16_t gfx4desp32_mipi_panel::ReadLine(int16_t x, int16_t y, int16_t w,
    uint16_t* data) {
    if (y > __height - 1 || y < 0)
        return 0;
    if (x > __width - 1 || (x + w - 1) < 0)
        return 0;
    if (x < 0) {
        w -= 0 - x;
        x = 0;
    }
    if ((x + w) > __width)
        w = __width - x;
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    int pc = 0;
    int readw = w;
    if (rotation == 0) {
        pto = tpto + (y * __scrWidth) + (x << 1);
        while (w--) {
            data[pc++] = pto[0] + (pto[1] << 8);
            pto += 2;
        }
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - y - 1) * __scrWidth) +
            (__scrWidth - ((x + 1) << 1));
        while (w--) {
            data[pc++] = pto[0] + (pto[1] << 8);
            pto -= 2;
        }
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - ((y + 1) << 1)) + (x * __scrWidth);
        while (w--) {
            data[pc++] = pto[0] + (pto[1] << 8);
            pto += __scrWidth;
        }
    }
    if (rotation == 2) {
        pto = tpto + (y << 1) + ((__scrHeight - x - 1) * __scrWidth);
        while (w--) {
            data[pc++] = pto[0] + (pto[1] << 8);
            pto -= __scrWidth;
        }
    }
    return readw;
}

/****************************************************************************/
/*!
  @brief  Copy a line of pixels from selected frame buffer to current frame buffer.
  @param  x left X position in pixels
  @param  y top Y position in pixels
  @param  w width of line
  @param  fb source frame buffer number
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::CopyFrameBufferLine(int16_t x, int16_t y, int16_t w,
    int Fb) {
    if (y > clipY2 || y < clipY1)
        return;
    if (x > clipX2 || (x + w - 1) < clipX1)
        return;
    if (w < 0) {
        x += w;
        w = abs(w);
    }
    if (x < clipX1) {
        w -= clipX1 - x;
        x = clipX1;
    }
    if ((x + w - 1) >= clipX2)
        w = clipX2 - x;
    uint8_t cb = GetFrameBuffer();
    DrawToframebuffer(Fb);
    ReadLine(x, y, w, linebuff);
    DrawToframebuffer(cb);
    WriteLine(x, y, w, linebuff);
}


/****************************************************************************/
/*!
  @brief  Write a line of pixels.
  @param  x left X position in pixels
  @param  y top Y position in pixels
  @param  w width of line
  @param  data - 16 bit user array
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::WriteLine(int16_t x, int16_t y, int16_t w,
    uint16_t* data) {
    if (y > clipY2 || y < clipY1)
        return;
    if (x > clipX2 || (x + w - 1) < clipX1)
        return;
    if (x < clipX1) {
        w -= clipX1 - x;
        x = clipX1;
    }
    if ((x + w - 1) > clipX2)
        w = clipX2 - x + 1;
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    uint8_t colM, colL;
    uint16_t tcol;
    int pc = 0;
    int flushw = w;
    if (rotation == 0) {
        pto = tpto + (y * __scrWidth) + (x << 1);
        while (w--) {
            tcol = data[pc++];
            colM = tcol >> 8;
            colL = tcol & 0xff;
            if (!transalpha) {
                pto[1] = colM;
                pto[0] = colL;
            }
            else {
                if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                    if (alpha) {
                        calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                        pto[0] = __colour;
                        pto[1] = __colour >> 8;
                    }
                    else {
                        pto[1] = colM;
                        pto[0] = colL;
                    }
                }
            }

            pto += 2;
        }
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - y - 1) * __scrWidth) +
            (__scrWidth - ((x + 1) << 1));
        while (w--) {
            tcol = data[pc++];
            colM = tcol >> 8;
            colL = tcol & 0xff;
            if (!transalpha) {
                pto[1] = colM;
                pto[0] = colL;
            }
            else {
                if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                    if (alpha) {
                        calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                        pto[0] = __colour;
                        pto[1] = __colour >> 8;
                    }
                    else {
                        pto[1] = colM;
                        pto[0] = colL;
                    }
                }
            }
            pto -= 2;
        }
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - ((y + 1) << 1)) + (x * __scrWidth);
        while (w--) {
            tcol = data[pc++];
            colM = tcol >> 8;
            colL = tcol & 0xff;
            if (!transalpha) {
                pto[1] = colM;
                pto[0] = colL;
            }
            else {
                if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                    if (alpha) {
                        calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                        pto[0] = __colour;
                        pto[1] = __colour >> 8;
                    }
                    else {
                        pto[1] = colM;
                        pto[0] = colL;
                    }
                }
            }
            pto += __scrWidth;
        }
    }
    if (rotation == 2) {
        pto = tpto + (y << 1) + ((__scrHeight - x - 1) * __scrWidth);
        while (w--) {
            tcol = data[pc++];
            colM = tcol >> 8;
            colL = tcol & 0xff;
            if (!transalpha) {
                pto[1] = colM;
                pto[0] = colL;
            }
            else {
                if (!(transparency && (colM == _transMSB && colL == _transLSB))) {
                    if (alpha) {
                        calcAlpha(colL + (colM << 8), pto[0] | (pto[1] << 8), __alpha);
                        pto[0] = __colour;
                        pto[1] = __colour >> 8;
                    }
                    else {
                        pto[1] = colM;
                        pto[0] = colL;
                    }
                }
            }
            pto -= __scrWidth;
        }
    }
	if (frame_buffer == visibleFB) { // if GRAM area is written to flush the area.
        if(rotation == 0) FlushArea(y, y, -1); 
	    if(rotation == 1) FlushArea(__scrHeight - y - 1, __scrHeight - y - 1 , -1); 
	    if(rotation == 3) FlushArea(x, x + flushw, -1);
	    if(rotation == 2) FlushArea(__scrHeight - x - 1 - flushw, __scrHeight - x - 1 , -1);
		// CPU writes data to PSRAM through DCache, data in PSRAM might not get
        // updated, so write back
    }
}

/****************************************************************************/
/*!
  @brief  Set clipping window ready for wrGRAM/s functions.
  @param  x1 left X position in pixels
  @param  y1 top Y position in pixels
  @param  x2 right X position in pixels
  @param  y2 bottom Y position in pixels
  @note   sets global variables clipX1pos, clipY1pos, clipX2pos, clipY2pos to
  window pos
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::ClipWindow(int x1, int y1, int x2, int y2) {
    clipX1pos = x1;
    clipY1pos = y1;
    clipX2pos = x2;
    clipY2pos = y2; // need to add check for out of bounds
    if (clipX1pos > __width - 1) clipX1pos = __width - 1;
    if (clipX1pos < 0) clipX1pos = 0;
    if (clipX2pos > __width - 1) clipX2pos = __width - 1;
    if (clipX2pos < 0) clipX2pos = 0;
    if (clipY1pos > __height - 1) clipY1pos = __height - 1;
    if (clipY1pos < 0) clipY1pos = 0;
    if (clipY2pos > __height - 1) clipY2pos = __height - 1;
    if (clipY2pos < 0) clipY2pos = 0;
}

/****************************************************************************/
/*!
  @brief  Enable / disable Clipping region
  @param  clipping - true / false
  @note   clipping is enabled by changing from defalult to user position and
          disabled by reverting back to default.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::Clipping(bool clipping) {
    if (clipping) {
        clipX1 = clipX1pos;
        clipY1 = clipY1pos;
        clipX2 = clipX2pos;
        clipY2 = clipY2pos;
    }
    else {
        if (rotation < 2) { // set to screen dimensions for disabled
            clipX1 = 0;
            clipY1 = 0;
            clipX2 = (int)st_hres - 1;
            clipY2 = (int)st_vres - 1;
        }
        else {
            clipX1 = 0;
            clipY1 = 0;
            clipX2 = (int)st_vres - 1;
            clipY2 = (int)st_hres - 1;
        }
    }
    clippingON = clipping;
    clipx1 = clipX1;
    clipy1 = clipY1;
    clipx2 = clipX2;
    clipy2 = clipY2;
}

/****************************************************************************/
/*!
  @brief  Set scroll window ready for scrolling.
  @param  x1 left X position in pixels
  @param  y1 top Y position in pixels
  @param  x2 right X position in pixels
  @param  y2 bottom Y position in pixels
  @note   sets global variables scroll_X1, scroll_Y1, scroll_X2, scroll_Y2 to
  window pos
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::setScrollArea(int x1, int y1, int x2, int y2) {
	if(x1 > x2 || y1 > y2 || x1 == x2 || y1 == y2) return;
	scroll_window_store_x1 = x1;
    scroll_window_store_y1 = y1;
    scroll_window_store_x2 = x2;
    scroll_window_store_y2 = y2;
    if (rotation < 2) {
        if (scroll_window_store_x1 < 0)
            scroll_window_store_x1 = 0;
        if (scroll_window_store_x2 > (int)st_hres - 1)
            scroll_window_store_x2 = (int)st_hres - 1;
        if (scroll_window_store_y1 < 0)
            scroll_window_store_y1 = 0;
        if (scroll_window_store_y2 > (int)st_vres - 1)
            scroll_window_store_y2 = (int)st_vres - 1;
    }
    else {
        if (scroll_window_store_x1 < 0)
            scroll_window_store_x1 = 0;
        if (scroll_window_store_x2 > (int)st_vres - 1)
            scroll_window_store_x2 = (int)st_vres - 1;
        if (scroll_window_store_y1 < 0)
            scroll_window_store_y1 = 0;
        if (scroll_window_store_y2 > (int)st_hres - 1)
            scroll_window_store_y2 = (int)st_hres - 1;
    }
	if (scroll_Enable) _ScrollEnable(true);
}

/****************************************************************************/
/*!
  @brief  Set scroll window ready for scrolling.
  @param  y1 top Y position in pixels
  @param  y2 bottom Y position in pixels
  @note   sets global variables scroll_X1, scroll_Y1, scroll_X2, scroll_Y2 to
  window pos compatible GFX4dESP32 scroll area for SPI displays and backward
  compatibilty with RGB displays
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::setScrollArea(int y1, int y2) {
    if(y1 > y2 || y1 == y2) return;
	scroll_window_store_x1 = 0;
    scroll_window_store_y1 = y1;
    scroll_window_store_x2 = __width - 1;
    scroll_window_store_y2 = y2;
    if (scroll_window_store_x1 < 0)
        scroll_window_store_x1 = 0;
    if (scroll_window_store_x2 > __width - 1)
        scroll_window_store_x2 = __width - 1;
    if (scroll_window_store_y1 < 0)
        scroll_window_store_y1 = 0;
    if (scroll_window_store_y2 > __height - 1)
        scroll_window_store_y2 = __height - 1;
	if (scroll_Enable) _ScrollEnable(true);
}

/****************************************************************************/
/*!
  @brief  Switch on / off automatic scroll
  @param  scrEn - true / false
  @note   enable automatic scrolling in conjunction with gfx4desp32 write and
  newline functions
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::_ScrollEnable(bool scrEn) { 
    //scroll_Enable = scrEn; 
	if (scrEn){
		scroll_X1 = scroll_window_store_x1;
		scroll_X2 = scroll_window_store_x2;
		scroll_Y1 = scroll_window_store_y1;
		scroll_Y2 = scroll_window_store_y2;
	} else {
		scroll_X1 = 0;
		scroll_X2 = __width -1;
		scroll_Y1 = 0;
		scroll_Y2 = __height -1;
	}
	scroll_Enable = scrEn;
}

/****************************************************************************/
/*!
  @brief  Set direction of scroll
  @param  scrEn - scroll_Direction 0 - 3
  @note   default direction is upwards
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::setScrollDirection(uint8_t scrDir) {
    scroll_Direction = scrDir % 4;
}

/****************************************************************************/
/*!
  @brief  Set blanking line colour after scroll has moved
  @param  scolor - RGB565 colour
  @note   this maybe could be the current text background colour
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::setScrollBlankingColor(int32_t scolor) {
    scroll_blanking = scolor;
}

/****************************************************************************/
/*!
  @brief  Set scroll behaviour
  @param  sspeed - enable and delay
  @note   0 - will scroll height defined by character height in one step.
          a value higher than 0 will scroll pixel line by pixel line delayed
          by the value in ms
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::SmoothScrollSpeed(uint8_t sspeed) {
    scroll_speed = sspeed;
}

/****************************************************************************/
/*!
  @brief  Perform scroll
  @param  steps - number of pixel lines to scroll
  @note   Scroll is carried out by moving the contents of the framebuffer to
          a new location defined by steps.
          if smoothScrollSpeed has been set then it will be scrolled pixel line
          by pixel line delayed by the ms value in smoothScrollSpeed.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::Scroll(int steps) {
    uint32_t x_start = 0;
    uint32_t y_start = 0;
    uint32_t x_end = 0;
    uint32_t y_end = 0;
    uint8_t scrdir = 0;
    Clipping(false);
    // set area depending on orientation
    if (rotation == 0) {
        x_start = scroll_X1;
        y_start = scroll_Y1;
        x_end = scroll_X2;
        y_end = scroll_Y2;
    }
    if (rotation == 1) {
        x_start = st_hres - 1 - scroll_X2;
        x_end = st_hres - 1 - scroll_X1;
        y_start = st_vres - 1 - scroll_Y2;
        y_end = st_vres - 1 - scroll_Y1;
    }
    if (rotation == 3) {
        x_start = st_hres - 1 - scroll_Y2;
        x_end = st_hres - 1 - scroll_Y1;
        y_start = scroll_X1;
        y_end = scroll_X2;
    }
    if (rotation == 2) {
        x_start = scroll_Y1;
        x_end = scroll_Y2;
        y_start = st_vres - 1 - scroll_X2;
        y_end = st_vres - 1 - scroll_X1;
    }
    uint32_t s_width = x_end - x_start + 1;
    uint32_t s_height = y_end - y_start + 1;
    scrdir = scroll_Directions[(rotation << 2) + scroll_Direction]; // calculate direction using
    // orientation and direction array
    uint32_t n, o;
    uint8_t* tfbbuf = SelectFB(frame_buffer);
    uint8_t* from; // create pointer for from area
    uint8_t* to;   // create pointer for to area
    int16_t s_steps = steps;
    uint32_t inc = 0;
    uint32_t s_inc = 0;
    int32_t pc;
    if (scroll_speed == 0 && steps > 1) {
        s_steps = 1;
        inc = steps;
    }
    else {
        s_steps = steps;
        inc = 1;
    }
    if (scrdir == 0) {
        while (s_steps--) {
            from = tfbbuf + ((y_start + inc) * __scrWidth) + (x_start << 1);
            to = tfbbuf + (y_start * __scrWidth) + (x_start << 1);
            n = s_height;
            while (n--) {
                if (n > inc - 1) {
					memcpy(to, from, s_width << 1);
                    from += __scrWidth;
                    to += __scrWidth;
                }
                else {
                    o = s_width;
                    while (o--) {
                        if (scroll_blanking != -1) {
                            to[0] = scroll_blanking;
                            to[1] = scroll_blanking >> 8;
                        }
                        to += 2;
                    }
                    to += (__scrWidth - (s_width << 1));
                }
            }
            if (frame_buffer == visibleFB)FlushArea(y_start, y_end, -1);
            delay(scroll_speed);
        }
    }
    if (scrdir == 1) {
        int32_t tempfrom;
        while (s_steps--) {
            tempfrom = ((y_start + s_height - 1 - inc) * __scrWidth) + (x_start << 1);
            from = tfbbuf + tempfrom;
            to = tfbbuf + ((y_start + s_height - 1) * __scrWidth) + (x_start << 1);
            n = s_height;
            while (n--) {
                if (n > inc - 1) {
                    if (tempfrom >= 0)
					    memmove(to, from, s_width << 1);
                    from -= __scrWidth;
                    tempfrom -= __scrWidth;
                    to -= __scrWidth;
                }
                else {
                    o = s_width;
                    while (o--) {
                        if (scroll_blanking != -1) {
                            to[0] = scroll_blanking;
                            to[1] = scroll_blanking >> 8;
                        }
                        to += 2;
                    }
                    to -= (__scrWidth + (s_width << 1));
                }
            }
            if (frame_buffer == visibleFB)FlushArea(y_start, y_end, -1);
            delay(scroll_speed);
        }
    }
    if (scrdir == 2) {
        while (s_steps--) {
            from = tfbbuf + (y_start * __scrWidth) + (x_start << 1);
            to = tfbbuf + (y_start * __scrWidth) + ((x_start + inc) << 1);
            n = s_height;
            while (n--) {
                memmove(to, from, (s_width - inc) << 1);
                pc = 0;
                s_inc = inc;
                while (s_inc--) {
                    if (scroll_blanking != -1) {
                        from[pc++] = scroll_blanking;
                        from[pc++] = scroll_blanking >> 8;
                    }
                }
                from += __scrWidth;
                to += __scrWidth;
            }
            if (frame_buffer == visibleFB)FlushArea(y_start, y_end, -1);
            delay(scroll_speed);
        }
    }
    if (scrdir == 3) {
        while (s_steps--) {
            from = tfbbuf + (y_start * __scrWidth) + ((x_start + inc) << 1);
            to = tfbbuf + (y_start * __scrWidth) + (x_start << 1);
            n = s_height;
            while (n--) {
                memmove(to, from, (s_width - inc) << 1);
                pc = ((s_width - 0 - inc) << 1);
                s_inc = inc;
                while (s_inc--) {
                    if (scroll_blanking != -1) {
                        to[pc++] = scroll_blanking;
                        to[pc++] = scroll_blanking >> 8;
                    }
                }
                from += __scrWidth;
                to += __scrWidth;
            }
            if (frame_buffer == visibleFB)FlushArea(y_start, y_end, -1);
            delay(scroll_speed);
        }
    }
    scrolled = true;
    Clipping(true);
}

/****************************************************************************/
/*!
  @brief  Draw simple Filled Rectangle.
  @param  x1 left X position in pixels
  @param  y1 top Y position in pixels
  @param  x2 right X position in pixels
  @param  y2 bottom Y position in pixels
  @param  color - RGB565 color
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::RectangleFilled(int x1, int y1, int x2, int y2,
    uint32_t color) {
    if ((color & 0xffff0000) == 0xf00000 || (color & 0xffff0000) == 0xf0000){
		RectangleFilledX(x1, y1, x2, y2, color);
		return;
	}
	if (transparency) {
        if (color == _transparentColor)
            return;
    }
    if (x1 > x2)
        gfx_Swap(x1, x2);
    if (y1 > y2)
        gfx_Swap(y1, y2);
    if (x1 >= __width || x2 < 0 || y1 >= __height || y2 < 0)
        return;
    if (x1 < 0)
        x1 = 0;
    if (y1 < 0)
        y1 = 0;
    //uint32_t pc = 0;
    int32_t xpos = x1;
    int32_t ypos = y1;
	pPos = (ypos * __scrWidth);
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    uint32_t pixels = ((x2 - x1) + 1) * ((y2 - y1) + 1);
    if (rotation == 0) {
		pto = tpto + pPos + (xpos << 1);
        while (pixels--) {
            if (xpos >= clipX1 && xpos <= clipX2 && ypos >= clipY1 &&
                ypos <= clipY2) {
                if (!alpha) {
					pto[0] = color;
                    pto[1] = color >> 8;	
                }
                else {
                    calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
                    pto[0] = __colour;
                    pto[1] = __colour >> 8;
                }
            }
            xpos++;
			pto += 2;
            if (xpos > x2) {
                xpos = x1;
                ypos++;
				pPos += __scrWidth;	
				//pPos += __scrWidth;
                pto = tpto + pPos + (xpos << 1);
            }
        }
        if (frame_buffer == visibleFB)
            FlushArea(y1, y2, -1);
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - ypos - 1) * __scrWidth) +
            (__scrWidth - ((xpos + 1) << 1));
        while (pixels--) {
            if (xpos >= clipX1 && xpos <= clipX2 && ypos >= clipY1 &&
                ypos <= clipY2) {
                if (!alpha) {
                    pto[0] = color;
                    pto[1] = color >> 8;
                }
                else {
                    calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
                    pto[0] = __colour;
                    pto[1] = __colour >> 8;
                }
            }
            xpos++;
            pto -= 2;
            if (xpos > x2) {
                xpos = x1;
                ypos++;
                pPos += __scrWidth;
                pto = tpto + ((__scrHeight - ypos - 1) * __scrWidth) +
                    (__scrWidth - ((xpos + 1) << 1));
            }
        }
        if (frame_buffer == visibleFB)
            FlushArea(__scrHeight - y2, __scrHeight - y1 - 1, -1);
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - ((ypos + 1) << 1)) +
            (xpos * __scrWidth);
        while (pixels--) {
            if (xpos >= clipX1 && xpos <= clipX2 && ypos >= clipY1 &&
                ypos <= clipY2) {
                if (!alpha) {
                    pto[0] = color;
                    pto[1] = color >> 8;
                }
                else {
                    calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
                    pto[0] = __colour;
                    pto[1] = __colour >> 8;
                }
            }
            xpos++;
            pto += __scrWidth;
            if (xpos > x2) {
                xpos = x1;
                ypos++;
                pto = tpto + (__scrWidth - (ypos << 1) - 2) +
                    (xpos * __scrWidth);
            }
        }
        if (frame_buffer == visibleFB)
		    FlushArea(x1, x2, -1);
    }
    if (rotation == 2) {
        pto = tpto + (ypos << 1) +
            ((__scrHeight - xpos - 1) * __scrWidth);
        while (pixels--) {
            if (xpos >= clipX1 && xpos <= clipX2 && ypos >= clipY1 &&
                ypos <= clipY2) {
                if (!alpha) {
                    pto[0] = color;
                    pto[1] = color >> 8;
                }
                else {
                    calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
                    pto[0] = __colour;
                    pto[1] = __colour >> 8;
                }
            }
            xpos++;
            pto -= __scrWidth;
            if (xpos > x2) {
                xpos = x1;
                ypos++;
                pto = tpto + (ypos << 1) +
                    ((__scrHeight - xpos - 1) * __scrWidth);
            }
        }
        if (frame_buffer == visibleFB)
           FlushArea(__scrHeight - x2 - 1, __scrHeight - x1 - 1, -1);
    }
}


/****************************************************************************/
/*!
  @brief  Select a frame buffer for all drawing actions.
  @param  sel 0 to 3
  @note   frame buffers are pre defined as empty and only sized to screen
  dimensions when the DrawToFramebuffer function is called.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::SetVisibleFrameBuffer(uint8_t sel) {
	// todo
	/*
	uint8_t* tfb;
	//tfb = SelectFB(sel);
	tfb = (uint8_t*)fb;
	tfb = SelectFB(sel);
	visibleFB = sel;
	FlushArea(0, st_vres - 1, -1);
	*/
}

/****************************************************************************/
/*!
  @brief  Select a frame buffer for all drawing actions.
  @param  sel 0 to 3
  @note   frame buffers are pre defined as empty and only sized to screen
  dimensions when the DrawToFramebuffer function is called.
*/
/****************************************************************************/
uint8_t* gfx4desp32_mipi_panel::SelectFB(uint8_t sel) {
    currFB = sel;
    switch (sel) {
    case 0:
        return (uint8_t*)fb;
        break;
    case 1:
        return psRAMbuffer3;
        break;
    case 2:
        return psRAMbuffer4;
        break;
    case 3:
        return psRAMbuffer5;
        break;
    case 4:
        return psRAMbuffer6;
        break;
	case 5:
        return psRAMbuffer7;
        break;
	case 6:
        return psRAMbuffer8;
        break;
	case 7:
        return psRAMbuffer9;
        break;
	case 8:
        return psRAMbuffer10;
        break;
	case 9:
        return psRAMbuffer11;
        break;
	case WIDGET_BUFFER:
        return workbuffer;
        break;
	case RGB888_BUFFER:
        return psRAMrgb888buffer;
        break;
    currFB = 0;
	}    
    return (uint8_t*)fb;
}

void gfx4desp32_mipi_panel::AllocateDRcache(uint32_t cacheSize) {
    psRAMbuffer2 = (uint8_t*)ps_malloc(cacheSize);
    cache_Enabled = true;
}

void gfx4desp32_mipi_panel::AllocateFB(uint8_t sel) {
    if (sel == 0) {
		psRAMbuffer1 = (uint8_t*)heap_caps_aligned_alloc(64, 2048000, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
    }
    if (sel == 1) {
		if(!framebufferInit1) psRAMbuffer3 = (uint8_t*)heap_caps_aligned_alloc(64, __fbSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
        framebufferInit1 = true;
    }
    if (sel == 2) {
		if(!framebufferInit2) psRAMbuffer4 = (uint8_t*)heap_caps_aligned_alloc(64, __fbSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
        framebufferInit2 = true;
    }
    if (sel == 3) {
		if(!framebufferInit3) psRAMbuffer5 = (uint8_t*)heap_caps_aligned_alloc(64, __fbSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
        framebufferInit3 = true;
    }
    if (sel == 4) {
        if(!framebufferInit4) psRAMbuffer6 = (uint8_t*)heap_caps_aligned_alloc(64, __fbSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
		framebufferInit4 = true;
    }
	if (sel == 5) {
        if(!framebufferInit5) psRAMbuffer7 = (uint8_t*)heap_caps_aligned_alloc(64, __fbSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
        framebufferInit5 = true;
    }
	if (sel == 6) {
        if(!framebufferInit6) psRAMbuffer8 = (uint8_t*)heap_caps_aligned_alloc(64, __fbSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
		framebufferInit6 = true;
    }
	if (sel == 7) {
        if(!framebufferInit7) psRAMbuffer9 = (uint8_t*)heap_caps_aligned_alloc(64, __fbSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
		framebufferInit7 = true;
    }
	if (sel == 8) {
        if(!framebufferInit8) psRAMbuffer10 = (uint8_t*)heap_caps_aligned_alloc(64, __fbSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
		framebufferInit8 = true;
    }
	if (sel == 9) {
        if(!framebufferInit9) psRAMbuffer11 = (uint8_t*)heap_caps_aligned_alloc(64, __fbSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
		framebufferInit9 = true;
    }
	if (sel == WIDGET_BUFFER) {
        workbuffer = (uint8_t*)heap_caps_aligned_alloc(64, __fbSize, MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM);
		framebufferInit12 = true;
    }
	if (sel == RGB888_BUFFER) {
        psRAMrgb888buffer = (uint8_t*)ps_malloc(__fbSize + (__fbSize >> 1));
        framebufferInit13 = true;
    }
}

void gfx4desp32_mipi_panel::CopyFrameBuffer(uint8_t fbto, uint8_t fbfrom1) {
    uint8_t* to = SelectFB(fbto);
    uint8_t* from1 = SelectFB(fbfrom1);
    memcpy(to, from1, __fbSize);
}

/****************************************************************************/
/*!
  @brief  Merge 2 frame buffers and send to specified frame buffer.
  @param  fbto - the sent to frame buffer usually 0
  @param  fbfrom1 - base frame buffer for the 2nd frame buffer to be merged to.
  @param  fbfrom2 - 2nd buffer to merge to the first;
  @param  transColor  - RGB565 colour equivelant to the transparent colour on
  the 2nd frame buffer
  @note   Using this function without first writing to a frame buffer will cause
          issue
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::MergeFrameBuffers(uint8_t fbto, uint8_t fbfrom1,
    uint8_t fbfrom2,
    uint16_t transColor) {
    uint8_t* to = SelectFB(fbto);
    uint8_t* from1 = SelectFB(fbfrom1);
    uint8_t* from2 = SelectFB(fbfrom2);
    uint16_t tcol1, tcol2;
    uint32_t len = __fbSize >> 1;
    while (len--) {
        tcol1 = from1[0] + (from1[1] << 8);
        tcol2 = from2[0] + (from2[1] << 8);
        if (tcol2 != transColor)
            tcol1 = tcol2;
        to[0] = tcol1 & 0xff;
        to[1] = tcol1 >> 8;
        to += 2;
        from1 += 2;
        from2 += 2;
    }
    if (frame_buffer == visibleFB) FlushArea(0, __scrHeight, -1);
}

/****************************************************************************/
/*!
  @brief  Merge 3 frame buffers and send to specified frame buffer.
  @param  fbto - the sent to frame buffer usually 0
  @param  fbfrom1 - base frame buffer for the 2nd frame buffer to be merged to.
  @param  fbfrom2 - 2nd buffer to merge to the first;
  @param  fbfrom3 - 3rd buffer to merge to the first;
  @param  transColor  - RGB565 colour equivelant to the transparent colour on
  the 2nd frame buffer
  @param  transColor  - RGB565 colour equivelant to the transparent colour on
  the 3rd frame buffer
  @note   Using this function without first writing to a frame buffer will cause
    issue
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::MergeFrameBuffers(uint8_t fbto, uint8_t fbfrom1,
    uint8_t fbfrom2, uint8_t fbfrom3,
    uint16_t transColor,
    uint16_t transColor1) {
    uint8_t* to = SelectFB(fbto);
    uint8_t* from1 = SelectFB(fbfrom1);
    uint8_t* from2 = SelectFB(fbfrom2);
    uint8_t* from3 = SelectFB(fbfrom3);
    uint16_t tcol1, tcol2, tcol3;
    uint32_t len = __fbSize >> 1;
    while (len--) {
        tcol1 = from1[0] + (from1[1] << 8);
        tcol2 = from2[0] + (from2[1] << 8);
        tcol3 = from2[0] + (from2[1] << 8);
        if (tcol2 != transColor)
            tcol1 = tcol2;
        if (tcol3 != transColor1)
            tcol1 = tcol3;
        to[0] = tcol1 & 0xff;
        to[1] = tcol1 >> 8;
        to += 2;
        from1 += 2;
        from2 += 2;
        from3 += 2;
    }
    if (frame_buffer == visibleFB) FlushArea(0, __scrHeight, -1);
}

/****************************************************************************/
/*!
  @brief  Draw a single pixel.
  @param  x left X position in pixels
  @param  y top Y position in pixels
  @param  color - RGB565 color
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::PutPixel(int16_t x, int16_t y, uint16_t color) {
    if (y > clipY2 || y < clipY1)
        return;
    if (x > clipX2 || x < clipX1)
        return;
    if (transparency) {
        if (color == _transparentColor)
            return;
    }
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    if (rotation == 0) {
        pto = tpto + (y * __scrWidth) + (x << 1);
        if (!alpha) {
            pto[0] = color;
            pto[1] = color >> 8;
        }
        else {
            calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
            pto[0] = __colour;
            pto[1] = __colour >> 8;
        }
        if (frame_buffer == visibleFB)
            FlushArea(y, y, x);
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - y - 1) * __scrWidth) +
            (__scrWidth - ((x + 1) << 1));
        if (!alpha) {
            pto[0] = color;
            pto[1] = color >> 8;
        }
        else {
            calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
            pto[0] = __colour;
            pto[1] = __colour >> 8;
        }
        if (frame_buffer == visibleFB)
            FlushArea(__scrHeight - y - 1, __scrHeight - y - 1, (__scrWidth - x + 1));
    }
    if (rotation == 3) {
        int32_t temp = (__scrWidth - ((y + 1) << 1)) + (__scrWidth * x);
        if (temp < 0 || temp > __fbSize)
            return;
        pto = tpto + temp;
        if (!alpha) {
            pto[0] = color;
            pto[1] = color >> 8;
        }
        else {
            calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
            pto[0] = __colour;
            pto[1] = __colour >> 8;
        }
        if (frame_buffer == visibleFB)
		    FlushArea(x, x, __scrWidth - y + 1);
    }
    if (rotation == 2) {
        pto = tpto + (y << 1) + ((__scrHeight - x - 1) * __scrWidth);
        if (!alpha) {
            pto[0] = color;
            pto[1] = color >> 8;
        }
        else {
            calcAlpha(color, pto[0] | (pto[1] << 8), __alpha);
            pto[0] = __colour;
            pto[1] = __colour >> 8;
        }
        if (frame_buffer == visibleFB)
			FlushArea(__scrHeight - x - 1, __scrHeight - x - 1, y);
    }
}

/****************************************************************************/
/*!
  @brief  Draw a fast horizontal line.
  @param  x left X position in pixels
  @param  y top Y position in pixels
  @param  w of line in pixels +ve or -ve
  @param  color - RGB565 color
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::Hline(int16_t x, int16_t y, int16_t w,
    uint32_t hcolor) {
	if ((hcolor & 0xffff0000) == 0xf00000 || (hcolor & 0xffff0000) == 0xf0000){
		HlineX(x, y, w, hcolor);
        return;
	}
    if (y > clipY2 || y < clipY1)
        return;
    if (x > clipX2 || (x + w - 1) < clipX1)
        return;
    if (transparency) {
        if (hcolor == _transparentColor)
            return;
    }
    if (w < 0) {
        x += w;
        //w *= -1;
        w = abs(w);
    }
    if (x < clipX1) {
        w -= clipX1 - x;
        x = clipX1;
    }
    if ((x + w) > clipX2 - 1)
        w = clipX2 - x + 1;
    uint8_t* tpto;
    uint8_t* pto;
    tpto = SelectFB(frame_buffer);
    int flushw = w;
    if (rotation == 0) {
        pto = tpto + (y * __scrWidth) + (x << 1);
        while (w--) {
            if (!alpha) {
                pto[0] = hcolor;
                pto[1] = hcolor >> 8;
            }
            else {
                calcAlpha(hcolor, pto[0] | (pto[1] << 8), __alpha);
                pto[0] = __colour;
                pto[1] = __colour >> 8;
            }
            pto += 2;
        }
        if (frame_buffer == visibleFB)
            FlushArea(y, y, -1);
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - y - 1) * __scrWidth) +
            (__scrWidth - ((x + 1) << 1));
        while (w--) {
            if (!alpha) {
                pto[0] = hcolor;
                pto[1] = hcolor >> 8;
            }
            else {
                calcAlpha(hcolor, pto[0] | (pto[1] << 8), __alpha);
                pto[0] = __colour;
                pto[1] = __colour >> 8;
            }
            pto -= 2;
        }
        if (frame_buffer == visibleFB)
            FlushArea(__scrHeight - y - 1, __scrHeight - y - 1, -1);
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - ((y + 1) << 1)) +
            (x * __scrWidth);
        while (w--) {
            if (!alpha) {
                pto[0] = hcolor;
                pto[1] = hcolor >> 8;
            }
            else {
                calcAlpha(hcolor, pto[0] | (pto[1] << 8), __alpha);
                pto[0] = __colour;
                pto[1] = __colour >> 8;
            }
            pto += __scrWidth;
        }
        if (frame_buffer == visibleFB)
		    FlushArea(x, x + flushw, -1);
    }
    if (rotation == 2) {
        pto = tpto + (y << 1) +
            ((__scrHeight - x - 1) * __scrWidth);
        while (w--) {
            if (!alpha) {
                pto[0] = hcolor;
                pto[1] = hcolor >> 8;
            }
            else {
                calcAlpha(hcolor, pto[0] | (pto[1] << 8), __alpha);
                pto[0] = __colour;
                pto[1] = __colour >> 8;
            }
            pto -= __scrWidth;
        }
        if (frame_buffer == visibleFB)
			FlushArea(__scrHeight - x - 1 - flushw, __scrHeight - x - 1, -1);    
    }
}

/****************************************************************************/
/*!
  @brief  Draw a fast vertical line.
  @param  x left X position in pixels
  @param  y top Y position in pixels
  @param  w of line in pixels +ve or -ve
  @param  color - RGB565 color
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::Vline(int16_t x, int16_t y, int16_t w,
    uint32_t vcolor) {
	if ((vcolor & 0xffff0000) == 0xf00000 || (vcolor & 0xffff0000) == 0xf0000){
		VlineX(x, y, w, vcolor);
        return;
	}
	if (x > clipX2 || x < clipX1)
        return;
    if (y > clipY2 || (y + w - 1) < clipY1)
        return;
    if (transparency) {
        if (vcolor == _transparentColor)
            return;
    }
    if (w < 0) {
        y += w;
        w *= -1;
    }
    if (y < clipY1) {
        w -= clipY1 - y;
        y = clipY1;
    }
    if ((y + w) > clipY2 - 1)
        w = clipY2 - y + 1;
    uint8_t* tpto = SelectFB(frame_buffer);
    uint8_t* pto;
    int flushw = w;
    if (rotation == 0) {
        pto = tpto + (y * __scrWidth) + (x << 1);
        while (w--) {
            if (!alpha) {
                pto[0] = vcolor;
                pto[1] = vcolor >> 8;
            }
            else {
                calcAlpha(vcolor, pto[0] | (pto[1] << 8), __alpha);
                pto[0] = __colour;
                pto[1] = __colour >> 8;
            }
            pto += __scrWidth;
        }
        if (frame_buffer == visibleFB)
            FlushArea(y, y + flushw, -1);
    }
    if (rotation == 1) {
        pto = tpto + ((__scrHeight - y - 1) * __scrWidth) +
            (__scrWidth - ((x + 1) << 1));
        while (w--) {
            if (!alpha) {
                pto[0] = vcolor;
                pto[1] = vcolor >> 8;
            }
            else {
                calcAlpha(vcolor, pto[0] | (pto[1] << 8), __alpha);
                pto[0] = __colour;
                pto[1] = __colour >> 8;
            }
            pto -= __scrWidth;
        }
        if (frame_buffer == visibleFB)
            FlushArea(__scrHeight - y - 1 - flushw, __scrHeight - y - 1, -1);
    }
    if (rotation == 3) {
        pto = tpto + (__scrWidth - ((y + 1) << 1)) +
            (x * __scrWidth);
        while (w--) {
            if (!alpha) {
                pto[0] = vcolor;
                pto[1] = vcolor >> 8;
            }
            else {
                calcAlpha(vcolor, pto[0] | (pto[1] << 8), __alpha);
                pto[0] = __colour;
                pto[1] = __colour >> 8;
            }
            pto -= 2;
        }
        if (frame_buffer == visibleFB)
            FlushArea(__scrHeight - x - 1, __scrHeight - x - 1, -1);
    }
    if (rotation == 2) {
        pto = tpto + (y << 1) +
            ((__scrHeight - x - 1) * __scrWidth);
        while (w--) {
            if (!alpha) {
                pto[0] = vcolor;
                pto[1] = vcolor >> 8;
            }
            else {
                calcAlpha(vcolor, pto[0] | (pto[1] << 8), __alpha);
                pto[0] = __colour;
                pto[1] = __colour >> 8;
            }
            pto += 2;
        }
        if (frame_buffer == visibleFB)
            FlushArea(x, x, -1);
    }
}

void gfx4desp32_mipi_panel::drawBitmap(int x1, int y1, int x2, int y2,
     uint16_t* c_data) {
     //esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2, y2, &c_data);
}

/****************************************************************************/
/*!
  @brief  ESP32-S3 function
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::__start_transmission() {
    
}

/****************************************************************************/
/*!
  @brief  Turn on Transparent mode
  @param  trans - transparency on / off
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::Transparency(bool trans) {
    transparency = trans;
    transalpha = alpha | transparency;

}

/****************************************************************************/
/*!
  @brief  Set transparent colour
  @param  color - RGB565 colour that won't be drawn in all functions when
          transparency enabled.
  @note   does not operate in pushColors function.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::TransparentColor(uint16_t color) {
    _transparentColor = color;
    _transMSB = color >> 8;
    _transLSB = color;
}

/****************************************************************************/
/*!
  @brief  Turn on Alpha Blend mode
  @param  alphablend - Alpha Blend on / off
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::AlphaBlend(bool alphablend) {
    alpha = alphablend;
    transalpha = alpha | transparency;
}

/****************************************************************************/
/*!
  @brief  Set Alpha Blend Level
  @param  alphaLevel - 0 to 255
  @note   does not operate in pushColors function.
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::AlphaBlendLevel(uint32_t alphaLevel) {
    __alpha = alphaLevel;
}

/****************************************************************************/
/*!
  @brief  IO Expander compatible functions without hardware
  @note   DigitalWrite - digitalWrite
		  DigitalRead  - digitalRead
		  PinMode	   - pinMode
*/
/****************************************************************************/
void gfx4desp32_mipi_panel::PinMode(byte pin, uint8_t mode) {
	pinMode(pin, mode);
}

void gfx4desp32_mipi_panel::DigitalWrite(byte pin, bool state) {
	digitalWrite(pin, state);
}

int gfx4desp32_mipi_panel::DigitalRead(byte pin) {
	return digitalRead(pin);
}

bool gfx4desp32_mipi_panel::__lcd_rgb_panel_fill_bounce_buffer(uint8_t* buffer)
{
    bool need_yield = false;
    return need_yield;
}

void gfx4desp32_mipi_panel::WriteReg(uint8_t addr, uint8_t reg, uint8_t data){
	Wire.beginTransmission(addr);
    Wire.write((uint8_t)reg);
	Wire.write((uint8_t)data);
    Wire.endTransmission();
}

void gfx4desp32_mipi_panel::DrawDitheredGradientRectToFrameBuffer(uint8_t fb, int x1, int y1, int x2, int y2, int32_t colfrom, int32_t colto, bool Orientation){
  int r, g, b; 
  int rb, gb, bb;
  float ri, gi, bi;
  int tx1, tx2, ty1, ty2;
  switch (rotation) {
    case 1:
	  tx1 = (st_hres - 1) - x2;
      tx2 = (st_hres - 1) - x1;
      ty1 = (st_vres - 1) - y2;
      ty2 = (st_vres - 1) - y1;
	  x1 = tx1; x2 = tx2; y1 = ty1, y2 = ty2;
	  gfx_Swap(colfrom, colto);
	  break;
	case 2:
	  ty1 = y1;
	  ty2 = y2;
	  y1 = (st_vres - 1) - x2;
	  y2 = (st_vres - 1) - x1;
	  x1 = ty1;
	  x2 = ty2;
	  if(Orientation == false){
		Orientation = true;
	  } else {
		Orientation = false;
	  }
	  gfx_Swap(colfrom, colto);
	  break;
	  case 3:
	  tx1 = x1;
	  tx2 = x2;
	  x1 = (st_hres - 1) - y2;
	  x2 = (st_vres - 1) - y1;
	  y1 = tx1;
	  y2 = tx2;
	  if(Orientation == false){
		Orientation = true;
	  } else {
		Orientation = false;
	  }
	  break;
  }
  if(x1 >= st_hres || y1 >= st_vres || x2 < 0 || y2 < 0) return;
  if(x1 < 0) x1 = 0;
  if(y1 < 0) y1 = 0;
  if(x2 >= st_hres) x2 = st_hres -1;
  if(y2 >= st_vres) y2 = st_vres -1;  
  if (!framebufferInit13)
    AllocateFB(RGB888_BUFFER);
  int cf = GetFrameBuffer();
  DrawToframebuffer(fb);
  DrawToframebuffer(cf);
  uint8_t* rgb888 = SelectFB(RGB888_BUFFER);
  uint16_t* rgb565 = (uint16_t*)SelectFB(fb);
  uint8_t* pto;
  int w = x2 - x1 + 1;
  int h = y2 - y1 + 1;
  int32_t x, y;
  uint32_t pos888;
  float range = w;
  if (Orientation == VERTICAL) range = h;
  r = (colfrom >> 11) << 3;
  g = ((colfrom >> 5) & 0x3f) << 2;
  b = (colfrom & 0x1f) << 3;
  rb = (colto >> 11) << 3;
  gb = ((colto >> 5) & 0x3f) << 2;
  bb = (colto & 0x1f) << 3;
  ri = (float)(rb - r) / range;
  gi = (float)(gb - g) / range;
  bi = (float)(bb - b) / range;
  int gCount;
  for(y = 0; y < h; y++){
    pto = rgb888 + ((((y1 + y) * st_hres) + x1) * 3);
	for(x = 0; x < w; x++){
	  gCount = x;
	  if(Orientation == VERTICAL) gCount = y;
	  pto[0] = r + ((float)gCount * ri);
      pto[1] = g + ((float)gCount * gi);
      pto[2] = b + ((float)gCount * bi);
      pto += 3;	
	}
  }		
  for (uint32_t y = 0; y < h; y++) {
    for (uint32_t x = 0; x < w; x++) {
	  uint32_t index = ((y1 + y) * st_hres + (x1 + x)) * 3;
      uint8_t r = rgb888[index];
      uint8_t g = rgb888[index + 1];
      uint8_t b = rgb888[index + 2];
	  uint16_t outColor = RGBto565(r, g, b);          
	  rgb565[(y1 + y) * st_hres + (x1 + x)] = outColor;	  
	  int err_b = b - ((outColor & 0x1F) * 255 / 31);
	  int err_g = g - (((outColor >> 5) & 0x3F) * 255 / 63);
	  int err_r = r - (((outColor >> 11) & 0x1F) * 255 / 31);
	  if (x + 1 < w) {
	    uint32_t nextIndex = (((y1 + y) * st_hres + (x1 + x) + 1) * 3);
        rgb888[nextIndex] = rgb888[nextIndex] + (err_r * 7 / 16);
        rgb888[nextIndex + 1] = rgb888[nextIndex + 1] + (err_g * 7 / 16);
        rgb888[nextIndex + 2] = rgb888[nextIndex + 2] + (err_b * 7 / 16);
      } 
	  if (y + 1 < h) {
        if (x > 0) {
		  uint32_t nextIndex = ((((y1 + y) + 1) * st_hres + ((x1 + x) - 1)) * 3);
          rgb888[nextIndex] = rgb888[nextIndex] + (err_r * 3 / 16);
          rgb888[nextIndex + 1] = rgb888[nextIndex + 1] + (err_g * 3 / 16);
          rgb888[nextIndex + 2] = rgb888[nextIndex + 2] + (err_b * 3 / 16);
        }
		uint32_t nextIndex = ((((y1 + y) + 1) * st_hres + (x1 + x)) * 3);
        rgb888[nextIndex] = rgb888[nextIndex] + (err_r * 5 / 16);
        rgb888[nextIndex + 1] = rgb888[nextIndex + 1] + (err_g * 5 / 16);
        rgb888[nextIndex + 2] = rgb888[nextIndex + 2] + (err_b * 5 / 16);

        if (x + 1 < w) {
		  uint32_t nextIndex = ((((y1 + y) + 1) * st_hres + (x1 + x) + 1) * 3);
          rgb888[nextIndex] = rgb888[nextIndex] + (err_r * 1 / 16);
          rgb888[nextIndex + 1] = rgb888[nextIndex + 1] + (err_g * 1 / 16);
          rgb888[nextIndex + 2] = rgb888[nextIndex + 2] + (err_b * 1 / 16);
        }
      }
    }
  }
  if (frame_buffer == 0){
	if(rotation == 0) FlushArea(y1, y2, -1); 
	if(rotation == 1) FlushArea(__scrHeight - y2 - 1, __scrHeight - y1 - 1 , -1); 
	if(rotation == 3) FlushArea(x1, x2, -1);
	if(rotation == 2) FlushArea(__scrHeight - x2 - 1, __scrHeight - x1 - 1 , -1);  
  }
}




