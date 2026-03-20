//
// NB! This is a file generated from the .4Dino file, changes will be lost
//     the next time the .4Dino file is built
//
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "gfx4desp32_ESP32_P4_70CT.h"

gfx4desp32_ESP32_P4_70CT gfx = gfx4desp32_ESP32_P4_70CT();

#include "BasicOTAConst.h"    // Note. This file will not be created if there are no generated graphics

// Uncomment if using GC* in program flash.
#include "BasicOTAGCx.h"     // Note. This file will be generated when graphics destination program flash

// Change this match the Wi-Fi network
const char* ssid = "CruzLink";
const char* password = "YRRfAUFa0107!";

#define USE_UPDATED

void setup()
{
  gfx.begin();
  gfx.Cls();
  gfx.ScrollEnable(false);
  gfx.BacklightOn(true);
  gfx.Orientation(LANDSCAPE);
  gfx.SmoothScrollSpeed(5);
  gfx.TextColor(WHITE, BLACK); gfx.Font(2);  gfx.TextSize(1);
// Uncomment one of the three followin statements depending on the type of graphics file you are using GCI, GCJ, or GC* in program flash.
//gfx.Open4dGFX("BasicOTA"); // Opens DAT and GCI files for read using filename without extension.
//gfx.Open4dGFX("BasicOTA.gcj"); // Opens GCJ file for read using complete filename.
  gfx.Open4dGFX(BasicOTA, BasicOTA_size); // Opens GCJ from program space using array name and size
//gfx.Open4dGFX(BasicOTA_dat, BasicOTA_dat_size, BasicOTA_gci, BasicOTA_gci_size); // Opens DAT/GCI from program space using array name and size
//gfx.touch_Set(TOUCH_ENABLE);                // Global touch enabled

  WiFi.setPins(18, 19, 14, 15, 16, 17, 3);

#ifndef USE_UPDATED
  gfx.UserImage(iStatictext1) ;  // Statictext1
#else
  gfx.UserImage(iStatictext2) ;  // Statictext2
#endif

  gfx.MoveTo(0, 100); // Sets the cursor position to (x, y)
  gfx.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    gfx.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("4D-ESP32-P4");

  // No authentication by default so let's use 'admin'
  ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      gfx.println("Start updating " + type);
    })
    .onEnd([]() {
      gfx.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      gfx.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      gfx.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) gfx.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) gfx.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) gfx.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) gfx.println("Receive Failed");
      else if (error == OTA_END_ERROR) gfx.println("End Failed");
    });

  ArduinoOTA.begin();

  gfx.println("Ready");
  gfx.print("IP address: ");
  gfx.println(WiFi.localIP());
} // end Setup **do not alter, remove or duplicate this line**

void loop()
{
  ArduinoOTA.handle();
  // put your main code here, to run repeatedly:
  int itouched, val ;
  if(gfx.touch_Update())
  {
    itouched = gfx.imageTouched() ;
    switch (itouched)
    {                                                         // start touched selection **do not alter, remove or duplicate this line**
      // case statements for Knobs and Sliders go here
      default :                                               // end touched selection **do not alter, remove or duplicate this line**
        int button = gfx.ImageTouchedAuto();    // use default for keyboards and buttons
        val = gfx.getImageValue(button);
        switch (button)
        {                                                     // start button selection **do not alter, remove or duplicate this line**
          // case, one for each button or keyboard, default should end up as -1
        }                                                     // end button selection **do not alter, remove or duplicate this line**
    }
  }
}

