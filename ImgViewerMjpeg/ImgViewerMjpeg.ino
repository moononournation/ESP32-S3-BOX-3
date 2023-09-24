/*******************************************************************************
 * Motion JPEG Image Viewer
 * 
Arduino_GFX Motion JPEG Image Viewer example
MJPEG start
MJPEG end
Total frames: 104
Time used: 3071 ms
Average FPS: 33.9
Read MJPEG: 138 ms (4.5 %)
Decode video: 2221 ms (72.3 %)
Show video: 711 ms (23.2 %)
 * 
 * This is a simple Motion JPEG image viewer example
 * Image Source: https://www.pexels.com/video/earth-rotating-video-856356/
 * cropped: x: 598 y: 178 width: 720 height: 720 resized: 240x240
 * ffmpeg -i "Pexels Videos 3931.mp4" -ss 0 -t 20.4s -vf "reverse,setpts=0.5*PTS,fps=10,vflip,hflip,rotate=90,crop=720:720:178:598,scale=240:240:flags=lanczos" -pix_fmt yuv420p -q:v 9 earth.mjpeg
 *
 * Dependent libraries:
 * JPEGDEC: https://github.com/bitbank2/JPEGDEC.git
 *
 * Setup steps:
 * 1. Change your LCD parameters in Arduino_GFX setting
 * 2. Upload Motion JPEG file
 *   FFat (ESP32):
 *     upload FFat (FatFS) data with ESP32 Sketch Data Upload:
 *     ESP32: https://github.com/lorol/arduino-esp32fs-plugin
 ******************************************************************************/
#define MJPEG_FILENAME "/earth.mjpeg"
#define MJPEG_BUFFER_SIZE (240 * 240 * 2 / 10) // memory for a single JPEG frame

/*******************************************************************************
 * Start of Arduino_GFX setting
 ******************************************************************************/
#include <Arduino_GFX_Library.h>
#define GFX_DEV_DEVICE ARDUINO_ESP32_S3_BOX_3
#define GFX_BL 47
Arduino_DataBus *bus = new Arduino_ESP32SPI(4 /* DC */, 5 /* CS */, 7 /* SCK */, 6 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_ILI9342(bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */);
/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

#include <FFat.h>
#include <LittleFS.h>
#include <SPIFFS.h>
#include <SD.h>
#include <SD_MMC.h>

#include "MjpegClass.h"
static MjpegClass mjpeg;

/* variables */
static int total_frames = 0;
static unsigned long total_read_video = 0;
static unsigned long total_decode_video = 0;
static unsigned long total_show_video = 0;
static unsigned long start_ms, curr_ms;

// pixel drawing callback
static int jpegDrawCallback(JPEGDRAW *pDraw)
{
  // Serial.printf("Draw pos = %d,%d. size = %d x %d\n", pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
  unsigned long start = millis();
  gfx->draw16bitBeRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  total_show_video += millis() - start;
  return 1;
}

void setup()
{
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX Motion JPEG Image Viewer example");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Init Display
  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  if (!FFat.begin())
  {
    Serial.println(F("ERROR: File System Mount Failed!"));
    gfx->println(F("ERROR: File System Mount Failed!"));
  }
  else
  {
    File mjpegFile = FFat.open(MJPEG_FILENAME, "r");

    if (!mjpegFile || mjpegFile.isDirectory())
    {
      Serial.println(F("ERROR: Failed to open " MJPEG_FILENAME " file for reading"));
      gfx->println(F("ERROR: Failed to open " MJPEG_FILENAME " file for reading"));
    }
    else
    {
      uint8_t *mjpeg_buf = (uint8_t *)malloc(MJPEG_BUFFER_SIZE);
      if (!mjpeg_buf)
      {
        Serial.println(F("mjpeg_buf malloc failed!"));
      }
      else
      {
        Serial.println(F("MJPEG start"));

        start_ms = millis();
        curr_ms = millis();
        mjpeg.setup(
            &mjpegFile, mjpeg_buf, jpegDrawCallback, true /* useBigEndian */,
            0 /* x */, 0 /* y */, gfx->width() /* widthLimit */, gfx->height() /* heightLimit */);

        while (mjpegFile.available() && mjpeg.readMjpegBuf())
        {
          // Read video
          total_read_video += millis() - curr_ms;
          curr_ms = millis();

          // Play video
          mjpeg.drawJpg();
          total_decode_video += millis() - curr_ms;

          curr_ms = millis();
          total_frames++;
        }
        int time_used = millis() - start_ms;
        Serial.println(F("MJPEG end"));
        mjpegFile.close();
        float fps = 1000.0 * total_frames / time_used;
        total_decode_video -= total_show_video;
        Serial.printf("Total frames: %d\n", total_frames);
        Serial.printf("Time used: %d ms\n", time_used);
        Serial.printf("Average FPS: %0.1f\n", fps);
        Serial.printf("Read MJPEG: %lu ms (%0.1f %%)\n", total_read_video, 100.0 * total_read_video / time_used);
        Serial.printf("Decode video: %lu ms (%0.1f %%)\n", total_decode_video, 100.0 * total_decode_video / time_used);
        Serial.printf("Show video: %lu ms (%0.1f %%)\n", total_show_video, 100.0 * total_show_video / time_used);

        gfx->setTextSize(2);
        gfx->setTextColor(WHITE, BLACK);
        gfx->setCursor(0, 0);
        gfx->printf("JPEGDEC Decoder\n\n");
        gfx->printf("   Frames: %d\n", total_frames);
        gfx->printf("Time used: %d ms\n", time_used);
        gfx->printf("  Avg FPS: %0.1f\n", fps);
        gfx->printf("Read File: %lu ms (%0.1f%%)\n", total_read_video, 100.0 * total_read_video / time_used);
        gfx->printf("   Decode: %lu ms (%0.1f%%)\n", total_decode_video, 100.0 * total_decode_video / time_used);
        gfx->printf("  Display: %lu ms (%0.1f%%)\n", total_show_video, 100.0 * total_show_video / time_used);
      }
    }
  }
}

void loop()
{
}
