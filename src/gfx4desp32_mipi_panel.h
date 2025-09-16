#ifndef __GFX4D_MIPI_PANEL__
#define __GFX4D_MIPI_PANEL__
#define USE_SDFAT

#include "gfx4desp32P4.h"
#include "Wire.h"
#include "rom/cache.h"
#include "gfx4desp32_mipi_common.h"
#include "esp_async_memcpy.h"
//#include "esp_async_fbcpy.h"

#define DISPLAY_INTERFACE             DISP_INTERFACE_MIPI

#define GFX4d_TOUCH_RESET             0x04
#define GFX4d_TOUCH_INT               0x05

#define GEN4_RGB_DISPLAY 
#define GEN4_I2C_SDA              7
#define GEN4_I2C_SCL              8

#define MIPI_NAME					0
#define MIPI_WIDTH					1
#define MIPI_HEIGHT					2
#define MIPI_DSI_PHY_LDO_ID			3
#define MIPI_DSI_LANE_NUM			4
#define MIPI_DSI_LANE_RATE_MBPS		5
#define MIPI_DPI_CLK_MHZ			6
#define MIPI_DPI_TIMINGS_HPW		7
#define MIPI_DPI_TIMINGS_HBP		8
#define MIPI_DPI_TIMINGS_HFP		9
#define MIPI_DPI_TIMINGS_VPW		10
#define MIPI_DPI_TIMINGS_VBP		11
#define MIPI_DPI_TIMINGS_VFP		12
#define MIPI_USE_EXTERNAL_CMD		13
#define MIPI_DSI_RESET				14
#define INPUT_COLOUR_FMT			15

#define NAME_EK79007	0
#define NAME_HX8399		1 
#define NAME_ILI9881C	2
#define NAME_JD9165		3
#define NAME_JD9365		4
#define NAME_ST7701		5
#define NAME_ST7703		6
#define NAME_ST7796		7
#define NAME_ST77922	8
#define NAME_JD9365B	9
#define NAME_OTA7290	10
#define NAME_NV3051F	11

#define MIPI_DSI_DEFAULT_ESCAPE_CLOCK_FREQ_MHZ 30


struct esp_panel_lcd_init_cmd_t {
    int cmd;                /*!< The specific LCD command */
    const void *data;       /*!< Buffer that holds the command specific data */
    size_t data_bytes;      /*!< Size of `data` in memory, in bytes */
    unsigned int delay_ms;  /*!< Delay in milliseconds after this command */
};

class gfx4desp32_mipi_panel : virtual public gfx4desp32P4 {
private:
    void DisplayInit(int* mpconfig, int freq = 0);
    int RGB_InvertFix[16] = { 8, 3, 46, 9, 1, 5, 6, 7, 15, 16, 4, 45, 48, 47, 21, 14 };
    const char* TAG = "gfx4desp32P4_mipi_panel";
	esp_lcd_panel_handle_t mipi_dpi_panel = NULL;
    int currFB;
    int32_t GRAMx1;
    int32_t GRAMy1;
    int32_t GRAMx2;
    int32_t GRAMy2;
    bool wrGRAM;
    int16_t GRAMxpos;
    int16_t GRAMypos;
    uint32_t pixelPos;
    uint32_t pixelCount;
    uint32_t pPos;
    uint32_t nlSumGRAM;
    uint8_t Pixel[2];
    bool writeFBonly;
    uint16_t minFBflush;
    uint16_t maxFBflush;
    int clipX1;
    int clipY1;
    int clipX2;
    int clipY2;
    //int clipX1pos;
    //int clipY1pos;
    //int clipX2pos;
    //int clipY2pos;
	int* Pconfig  = NULL;
    //bool clippingON;
    uint8_t writeBuffInitial = 1;
    bool scroll_Enable;
    //uint8_t scroll_Direction;
    int32_t scroll_blanking;
    uint8_t scroll_speed;
    bool flush_pending;
    int backlight = 1;
    uint16_t _transparentColor;
    uint8_t _transMSB, _transLSB;
    uint8_t m_inp;
    uint8_t m_out;
    uint8_t m_pol;
    uint8_t m_ctrl;
    bool IOexpInit;
    int lasttxpos;
    bool frameBufferIData;
	int flushy1, flushy2;

    uint8_t pinNum2bitNum[8] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };

    uint8_t scroll_Directions[16] = { 0x00, 0x01, 0x03, 0x02,
                                     0x01, 0x00, 0x02, 0x03,
                                     0x03, 0x02, 0x01, 0x00,
                                     0x02, 0x03, 0x00, 0x01 };

    void __start_transmission();
    bool __lcd_rgb_panel_fill_bounce_buffer(uint8_t* buffer);

    void FlushArea(int y1, int y2, int xpos);
	async_memcpy_config_t config;//ASYNC_MEMCPY_DEFAULT_CONFIG();
	//esp_async_fbcpy_config_t configfbcpy;
// update the maximum data stream supported by underlying DMA engine
    //config.backlog = 8;
    async_memcpy_handle_t driver = NULL;
	//esp_async_fbcpy_handle_t driverfbcpy = NULL;

protected:
    int touchXraw;
    int touchYraw;


public:
	gfx4desp32_mipi_panel(int * config, int hres, int vres, int bk_pin, int bk_on_level, int bk_off_level, int ttype);
    ~gfx4desp32_mipi_panel();
    virtual void __begin() override;
    virtual void FlushArea(int x1, int x2, int y1, int y2, int xpos) override;
    virtual void WriteToFrameBuffer(uint32_t offset, uint8_t* data, uint32_t len) override;
    virtual void WriteToFrameBuffer(uint32_t offset, uint16_t* data, uint32_t len) override;
    virtual void DisplayControl(uint8_t cmd) override;
    virtual void DisplayControl(uint8_t cmd, uint32_t val) override;
    virtual void RectangleFilled(int x1, int y1, int x2, int y2, uint32_t color) override;
    virtual void Vline(int16_t x, int16_t y, int16_t w, uint32_t color) override;
    virtual void Hline(int16_t x, int16_t y, int16_t w, uint32_t hcolor) override;
    virtual void SetGRAM(int16_t x1, int16_t y1, int16_t x2, int16_t y2) override;
    virtual void WrGRAMs(uint8_t* color_data, uint32_t len) override;
    virtual void WrGRAMs(uint16_t* color_data, uint32_t len) override;
    virtual void WrGRAMs(uint32_t* color_data, uint16_t len) override;
    virtual void WrGRAM(uint16_t color) override;
    virtual void WrGRAMs(const uint8_t* color_data, uint32_t len) override;
    virtual void pushColors(uint16_t* color_data, uint32_t len) override;
    virtual void pushColors(uint8_t* color_data, uint32_t len) override;
    virtual void pushColors(const uint8_t* color_data, uint32_t len) override;
    virtual bool StartWrite() override;
    virtual void EndWrite() override;
    virtual void BacklightOn(bool blight) override;
    virtual void Scroll(int steps) override;
    virtual void _ScrollEnable(bool scrEn) override;
    virtual void setScrollArea(int y1, int y2) override;
    virtual void setScrollBlankingColor(int32_t scolor) override;
    virtual void SmoothScrollSpeed(uint8_t sspeed) override;
    virtual int16_t getScrollareaY1() override;
    virtual void PutPixel(int16_t x, int16_t y, uint16_t color) override;
    virtual void panelOrientation(uint8_t r) override;
    virtual void FillScreen(uint16_t color) override;
    virtual int16_t getHeight(void) override;
    virtual int16_t getWidth(void) override;
    virtual uint8_t getPanelOrientation(void) override;
    virtual void Contrast(int cntrst) override;
    virtual void Invert(bool Inv) override;
    virtual void Transparency(bool trans) override;
    virtual void TransparentColor(uint16_t color) override;
    virtual void AlphaBlend(bool alphablend) override;
    virtual void AlphaBlendLevel(uint32_t alphaLevel) override;
    virtual uint16_t ReadPixel(uint16_t xrp, uint16_t yrp) override;
    virtual uint16_t ReadPixelFromFrameBuffer(uint16_t xrp, uint16_t yrp, uint8_t fb) override;
    virtual uint16_t ReadLine(int16_t x, int16_t y, int16_t w, uint16_t* data) override;
    virtual void WriteLine(int16_t x, int16_t y, int16_t w, uint16_t* data) override;
    virtual void DrawFrameBuffer(uint8_t fbnum) override;
    virtual void DrawFrameBufferArea(uint8_t fbnum, int16_t ui) override;
    virtual void DrawFrameBufferArea(uint8_t fbnum, int16_t x1, int16_t y1, int16_t x2, int16_t y2) override;
	virtual void DrawFrameBufferAreaXY(uint8_t fbnum, int16_t ui, int x, int y) override;
	virtual void DrawFrameBufferAreaXY(uint8_t fbnum, int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x, int16_t y) override;
    virtual void MergeFrameBuffers(uint8_t fbto, uint8_t fbfrom1, uint8_t fbfrom2, uint16_t transColor) override;
    virtual void MergeFrameBuffers(uint8_t fbto, uint8_t fbfrom1, uint8_t fbfrom2, uint8_t fbfrom3, uint16_t transColor, uint16_t transColor1) override;
    virtual void drawBitmap(int x1, int y1, int x2, int y2, uint16_t* c_data) override;
    virtual void CopyFrameBuffer(uint8_t fbto, uint8_t fbfrom1) override;
    virtual void CopyFrameBufferLine(int16_t x, int16_t y, int16_t w, int fb) override;
	void FrameBufferCopy(uint8_t* srcbuff, int srcX1, int srcY1, int srcX2, int srcY2, uint8_t* destbuff, int destX, int destY);
    virtual void PinMode(byte pin, uint8_t mode) override;
    virtual void DigitalWrite(byte pin, bool state) override;
    virtual int DigitalRead(byte pin) override;
    virtual void AllocateFB(uint8_t sel) override;
    virtual void AllocateDRcache(uint32_t cacheSize) override;
	virtual int SpeakerVolume(int v) override;
	virtual int MicrophoneGain(int g) override;
	virtual void AudioInit() override;
	virtual void DrawDitheredGradientRectToFrameBuffer(uint8_t fb, int x1, int y1, int x2, int y2, int32_t colfrom, int32_t colto, bool Orientation) override;
	virtual void SetVisibleFrameBuffer(uint8_t sel) override;
    void* wb = NULL;
	//esp_panel_lcd_vendor_init_cmd_t* lcd_init_cmds = NULL;
    int32_t __scrWidth;
	int32_t __scrWidth24;
    int32_t __scrHeight;
    int32_t low_Y;
    int32_t high_Y;
    int32_t low_X;
    int32_t high_X;
    uint32_t high_ypos;
    uint16_t __width;
    uint16_t __height;
    uint32_t __fbSize;
	uint32_t __fbSize24;

    virtual void ClipWindow(int x1, int y1, int x2, int y2) override;
    virtual void Clipping(bool clipping) override;
    void setScrollArea(int x1, int y1, int x2, int y2);
    void setScrollDirection(uint8_t scrDir);
	void WriteReg(uint8_t addr, uint8_t reg, uint8_t data);
    virtual uint8_t* SelectFB(uint8_t sel) override;

    int calx1, calx2, caly1, caly2;
    int touchType;
};
#endif  // __GFX4D_RGB_PANEL__
