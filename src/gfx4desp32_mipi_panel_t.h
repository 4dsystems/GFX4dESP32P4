#ifndef __GFX4D_RGB_PANEL_T__
#define __GFX4D_RGB_PANEL_T__
#include "gfx4desp32P4_touch.h"
#include "gfx4desp32_mipi_panel.h"



#define TOUCH_CONTROLLER_FTXXX			 0
#define TOUCH_CONTROLLER_GT911			 1
#define TOUCH_CONTROLLER_GT928			 2
#define MAX_PENS 			 5	
#define CTP_DATA_LEN    	 1+6*MAX_PENS
#define GT911_I2C_ADDR_28  	 0x14
#define GT911_I2C_ADDR_BA  	 0x5D
#define GT911_MAX_CONTACTS 	 5
#define GT911_REG_CFG 		 0x8047
#define GT911_REG_CHECKSUM 	 0x80FF
#define GT911_REG_DATA 		 0x8140
#define GT911_REG_ID 		 0x8140
#define GT911_REG_COORD_ADDR 0x814E


class gfx4desp32_mipi_panel_t : public gfx4desp32_mipi_panel, public gfx4desp32P4_touch {
private:
  const char* TAG = "gfx4desp32_mipi_panel_t";
  //uint8_t _addr = GT911_I2C_ADDR_BA;
  int trotate;
  
public:
  gfx4desp32_mipi_panel_t(int * config, int hres, int vres, int bk_pin, int bk_on_level, int bk_off_level, int ttype);
  ~gfx4desp32_mipi_panel_t();

  virtual void touch_Set(uint8_t mode, bool aint = false) override;
  virtual bool touch_Update() override;
  virtual int touch_GetTouchPoints(int* tpx, int* tpy) override;
  int t_controller;
  int num_touch_points;
  int touchPoints[10];
  uint8_t gCTPData[CTP_DATA_LEN];
  int h_res;
  int v_res;
  int touch_res;
  int touch_int;
  bool alt_int;
};

#endif  // __GFX4D_RGB_PANEL_T__