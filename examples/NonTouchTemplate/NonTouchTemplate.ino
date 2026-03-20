//
// NB! This is a file generated from the .4Dino file, changes will be lost
//     the next time the .4Dino file is built
//

// When using Arduino, you need to set the correct include file and class initialization to match your display.
// To save the hassle of doing this is manually, it is recommended to use Workshop4.

#include "gfx4desp32_ESP32_P4_70.h"

gfx4desp32_ESP32_P4_70 gfx = gfx4desp32_ESP32_P4_70();

#include "NonTouchTemplateConst.h"    // Note. This file will not be created if there are no generated graphics

// Uncomment if using GC* in program flash.
// #include "NonTouchTemplateGCx.h"     // Note. This file will be generated when graphics destination program flash

void setup()
{
  gfx.begin();
  gfx.Cls();
  gfx.ScrollEnable(false);
  gfx.BacklightOn(true);
  gfx.Orientation(PORTRAIT);
  gfx.SmoothScrollSpeed(5);
  gfx.TextColor(WHITE, BLACK); gfx.Font(2);  gfx.TextSize(1);
// Uncomment one of the three followin statements depending on the type of graphics file you are using GCI, GCJ, or GC* in program flash.
//gfx.Open4dGFX("NonTouchTemplate"); // Opens DAT and GCI files for read using filename without extension.
//gfx.Open4dGFX("NonTouchTemplate.gcj"); // Opens GCJ file for read using complete filename.
//gfx.Open4dGFX(NonTouchTemplate, NonTouchTemplate_size); // Opens GCJ from program space using array name and size
//gfx.Open4dGFX(NonTouchTemplate_dat, NonTouchTemplate_dat_size, NonTouchTemplate_gci, NonTouchTemplate_gci_size); // Opens DAT/GCI from program space using array name and size
} // end Setup **do not alter, remove or duplicate this line**

void loop()
{
  // put your main code here, to run repeatedly:
}

