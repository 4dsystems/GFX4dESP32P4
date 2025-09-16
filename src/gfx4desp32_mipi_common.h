#ifndef __GEN4_MIPI_COMMON__
#define __GEN4_MIPI_COMMON__

#define GEN4_MIPI_BK_LIGHT_ON_LEVEL      1
#define GEN4_MIPI_BK_LIGHT_OFF_LEVEL     !GEN4_MIPI_BK_LIGHT_ON_LEVEL

#define GEN4_MIPI_PIN_NUM_BK_LIGHT       26

#define DISP_INTERFACE_MIPI				 4

#define GEN4_MIPI_H_RES                  800
#define GEN4_MIPI_V_RES                  1280

#define TOUCH_CONTROLLER_FTXXX			 0
#define TOUCH_CONTROLLER_GT911			 1
#define TOUCH_CONTROLLER_GT928			 2
#define TOUCH_XY_SWAP_90				 1
#define TOUCH_XY_SWAP_180				 2
#define TOUCH_XY_SWAP_270				 3

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

#endif // __GEN4_MIPI_COMMON__