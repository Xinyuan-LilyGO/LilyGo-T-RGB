//Dave A.  www.plastibots.com  December 2022
//LilyGo T-RGB Crude screen gesture/swipe detection.  Allows for identification of left, right, up, down swipes that can be translated to actions.
//Relies on an interrupt on the touch pad to trigger when touched. Then begins collecting a start point, subsequent end points. Then
//  caclulates difference between start and end X and Y, determines which general direction the finger was moving and identifies this.
//It does not do gestures, multi-touch etc. This isn't an iphone after all!
//Credit to for code sample to build upon and convert: https://www.codeproject.com/Articles/5287929/Swipe-Gestures-Using-a-TFT-Display
//T-RGB Flashing Note:  Unplug USB, hold BOT button, plug in USB, release BOT button. Flash.  Make sure port is selected in IDE each time
//This code is free to use, but comes without warranty. Standard MIT license applies:  https://opensource.org/licenses/MIT

#include "XL9535_driver.h"
#include "pin_config.h"
#include <Arduino.h>
#include <Arduino_GFX_Library.h>        // https://github.com/moononournation/Arduino_GFX.git
#include "ft3267.h"                     // Touch module
#include "Orbitron_ExtraBold12pt7b.h" 



//T-RGB
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    -1, -1, -1, EXAMPLE_PIN_NUM_DE, EXAMPLE_PIN_NUM_VSYNC, EXAMPLE_PIN_NUM_HSYNC, EXAMPLE_PIN_NUM_PCLK,
    EXAMPLE_PIN_NUM_DATA1, EXAMPLE_PIN_NUM_DATA2, EXAMPLE_PIN_NUM_DATA3, EXAMPLE_PIN_NUM_DATA4, EXAMPLE_PIN_NUM_DATA5,
    EXAMPLE_PIN_NUM_DATA6, EXAMPLE_PIN_NUM_DATA7, EXAMPLE_PIN_NUM_DATA8, EXAMPLE_PIN_NUM_DATA9, EXAMPLE_PIN_NUM_DATA10, EXAMPLE_PIN_NUM_DATA11, 
    EXAMPLE_PIN_NUM_DATA13, EXAMPLE_PIN_NUM_DATA14, EXAMPLE_PIN_NUM_DATA15, EXAMPLE_PIN_NUM_DATA16, EXAMPLE_PIN_NUM_DATA17);
Arduino_GFX *gfx = new Arduino_ST7701_RGBPanel(bus, GFX_NOT_DEFINED, 0 /* rotation */, false /* IPS */, 480, 480,
                                               st7701_type2_init_operations, sizeof(st7701_type2_init_operations), true,
                                               50, 1, 30, 20, 1, 30);

typedef struct {
  uint8_t cmd;
  uint8_t data[16];
  uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;


//for touch
typedef struct {
  uint16_t x;
  uint16_t y;
} tsPoint_t;


//****************************************************************************************
#define debug false    //true to turn debugging on
//****************************************************************************************

#define TINTERVAL 100           //default 100 - if time is within this interval, assumes finger dragging across screen and not lifted
#define TOUCH_FIN_WAIT 200      //default 200 - time to wait after touch is done and assume finger lifted and then take action
#define TOUCH_THRESHOLD_X 100   //default 100
#define TOUCH_THRESHOLD_Y 100   //default 100
bool gotFirstPt = false;
uint32_t touchElapsedT = 0;
bool doOnce = true;
bool getPt(tsPoint_t  * point);
tsPoint_t _touchFirst;
tsPoint_t _touchLast;
char buffer[40];
int direction = 0;            //used to save directions 0=none, 1 = left, 2=right, 3=up, 4=down



DRAM_ATTR static const lcd_init_cmd_t st_init_cmds[] = {
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 0x05},
    {0xC0, {0x3b, 0x00}, 0x02},
    {0xC1, {0x0b, 0x02}, 0x02},
    {0xC2, {0x07, 0x02}, 0x02},
    {0xCC, {0x10}, 0x01},
    {0xCD, {0x08}, 0x01}, // 用565时屏蔽    666打开
    {0xb0, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x06, 0x05, 0x09, 0x08, 0x21, 0x06, 0x13, 0x10, 0x29, 0x31, 0x18}, 0x10},
    {0xb1, {0x00, 0x11, 0x16, 0x0e, 0x11, 0x07, 0x05, 0x09, 0x09, 0x21, 0x05, 0x13, 0x11, 0x2a, 0x31, 0x18}, 0x10},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 0x05},
    {0xb0, {0x6d}, 0x01},
    {0xb1, {0x37}, 0x01},
    {0xb2, {0x81}, 0x01},
    {0xb3, {0x80}, 0x01},
    {0xb5, {0x43}, 0x01},
    {0xb7, {0x85}, 0x01},
    {0xb8, {0x20}, 0x01},
    {0xc1, {0x78}, 0x01},
    {0xc2, {0x78}, 0x01},
    {0xc3, {0x8c}, 0x01},
    {0xd0, {0x88}, 0x01},
    {0xe0, {0x00, 0x00, 0x02}, 0x03},
    {0xe1, {0x03, 0xa0, 0x00, 0x00, 0x04, 0xa0, 0x00, 0x00, 0x00, 0x20, 0x20}, 0x0b},
    {0xe2, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x0d},
    {0xe3, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe4, {0x22, 0x00}, 0x02},
    {0xe5, {0x05, 0xec, 0xa0, 0xa0, 0x07, 0xee, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x10},
    {0xe6, {0x00, 0x00, 0x11, 0x00}, 0x04},
    {0xe7, {0x22, 0x00}, 0x02},
    {0xe8, {0x06, 0xed, 0xa0, 0xa0, 0x08, 0xef, 0xa0, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0x10},
    {0xeb, {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00}, 0x07},
    {0xed, {0xff, 0xff, 0xff, 0xba, 0x0a, 0xbf, 0x45, 0xff, 0xff, 0x54, 0xfb, 0xa0, 0xab, 0xff, 0xff, 0xff}, 0x10},
    {0xef, {0x10, 0x0d, 0x04, 0x08, 0x3f, 0x1f}, 0x06},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x13}, 0x05},
    {0xef, {0x08}, 0x01},
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x00}, 0x05},
    {0x36, {0x08}, 0x01},
    {0x3a, {0x66}, 0x01},
    {0x11, {0x00}, 0x80},
    // {0xFF, {0x77, 0x01, 0x00, 0x00, 0x12}, 0x05},
    // {0xd1, {0x81}, 0x01},
    // {0xd2, {0x06}, 0x01},
    {0x29, {0x00}, 0x80},
    {0, {0}, 0xff}};

XL9535 xl;

bool touch_pin_get_int = false;
void tft_init(void);
void lcd_cmd(const uint8_t cmd);
void lcd_data(const uint8_t *data, int len);



void setup() {
  // put your setup code here, to run once:
  pinMode(BAT_VOLT_PIN, ANALOG);

  Wire.begin(IIC_SDA_PIN, IIC_SCL_PIN, (uint32_t)400000);
  Serial.begin(115200);
  //while(!Serial);


  xl.begin();
  uint8_t pin = (1 << PWR_EN_PIN) | (1 << LCD_CS_PIN) | (1 << TP_RES_PIN) | (1 << LCD_SDA_PIN) | (1 << LCD_CLK_PIN) |
                (1 << LCD_RST_PIN) | (1 << SD_CS_PIN);

  xl.pinMode8(0, pin, OUTPUT);
  xl.digitalWrite(PWR_EN_PIN, 1);

  pinMode(EXAMPLE_PIN_NUM_BK_LIGHT, OUTPUT);
  digitalWrite(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
  
  //SD_init();


  // TouchPad
  xl.digitalWrite(TP_RES_PIN, 0);
  delay(200);
  xl.digitalWrite(TP_RES_PIN, 1);
  ft3267_init(Wire);
  delay(50);
  
  //attach interrup to touchpad.  Triggers when touched.
  pinMode(TP_INT_PIN, INPUT_PULLUP);
  attachInterrupt(TP_INT_PIN, [] { touch_pin_get_int = true; }, FALLING);
  

  gfx->begin();
  tft_init();
  
  gfx->setFont(&Orbitron_ExtraBold12pt7b);
  //void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
  //gfx->fillRoundRect(100, 150, 300, 300, 8, PURPLE);
  if (debug) {Serial.print("Lets do this....");Serial.println("");}

  gfx->fillRoundRect(120, 410, 230, 40, 8, BLUE);  //void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
}


void loop() 
{

  detectTouch();

}

//##########################################################################################
//##########################################################################################
void detectTouch()
{

  uint8_t touch_points_num;
  uint16_t x, y;
  
  tsPoint_t pt;

  if (touch_pin_get_int) 
  {
    touchElapsedT = millis();  //screen being touched
    doOnce = true;
  } else
  {
    touch_pin_get_int = false;
  }

  if ((millis() - touchElapsedT > TOUCH_FIN_WAIT) && gotFirstPt)  // capture end points and calc after finished touch, but don't go on forever (goFirstPt will be false)
  {
    gotFirstPt = false;      //reset it
    int32_t dx = _touchLast.x-_touchFirst.x;
    int32_t dy = _touchLast.y-_touchFirst.y;
    uint32_t adx=abs(dx);
    uint32_t ady=abs(dy);

    if (debug) {Serial.println("");Serial.print(millis());Serial.print(" DeltaX: ");Serial.print(adx);Serial.print(" DeltaY: ");Serial.print(ady);Serial.println("");}

        // swipe horizontal
        if(adx>ady && adx> TOUCH_THRESHOLD_X) 
        { 
          if(0>dx) { // swipe right to left
            sprintf(buffer, "     < < < < < <");
            direction = 1;
          } else { // swipe left to right
            sprintf(buffer, "     > > > > > >");
            direction = 2;
          }
        // swipe vertical
        } else if(ady>adx && ady>TOUCH_THRESHOLD_Y) { 
          if(0>dy) { // swipe bottom to top
            sprintf(buffer, "up up up up up");
            direction = 3;
          } else { // swipe top to bottom
            sprintf(buffer, "  down down");
            direction = 4;
          }
        }

        //To Do: add function to react to swipe direction here.
        // eg: doGui(direction);

        gfx->setTextColor(WHITE);
        gfx->fillRoundRect(120, 410, 230, 40, 8, BLUE);  //void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
        gfx->setCursor(145, 435);
        gfx->println(buffer);
        memset(buffer, 0, sizeof(buffer));
  }


  if (millis() - touchElapsedT < TINTERVAL)   //if time is within this interval, assumes finger dragging across screen and not lifted
  {
    if (!gotFirstPt)
    {
      if (getPt(&pt))
      {
        _touchFirst = pt;
        gotFirstPt = true;
        if (debug) {Serial.print("First: ");Serial.print(_touchFirst.x);Serial.print(" "); Serial.print(_touchFirst.y);Serial.println(" ");}   
      } 
    } 
    else   //get last pt
    {
      if (getPt(&pt))
      {
        if ((pt.x < 500 && pt.y < 500) && (pt.x > 20 && pt.y > 20)) //igore rogue returns
        {
          _touchLast = pt;
          if (debug) {Serial.print(" Last: ");Serial.print(_touchLast.x);Serial.print(" "); Serial.print(_touchLast.y);   Serial.println(" ");}
        }
      }
    }     

    touch_pin_get_int = false;
    
  }
      
}
//##########################################################################################

bool getPt(tsPoint_t  * point) 
{
  uint8_t touch_points_num;
  uint16_t x, y;
  if (touch_pin_get_int) 
  {
    ft3267_read_pos(&touch_points_num, &x, &y);
    point->x = x;
    point->y = y;
    return true;
  }
  return false;
}



//##########################################################################################
void lcd_send_data(uint8_t data) {
  uint8_t n;
  for (n = 0; n < 8; n++) {
    if (data & 0x80)
      xl.digitalWrite(LCD_SDA_PIN, 1);
    else
      xl.digitalWrite(LCD_SDA_PIN, 0);

    data <<= 1;
    xl.digitalWrite(LCD_CLK_PIN, 0);
    xl.digitalWrite(LCD_CLK_PIN, 1);
  }
}

//##########################################################################################
void lcd_cmd(const uint8_t cmd) {
  xl.digitalWrite(LCD_CS_PIN, 0);
  xl.digitalWrite(LCD_SDA_PIN, 0);
  xl.digitalWrite(LCD_CLK_PIN, 0);
  xl.digitalWrite(LCD_CLK_PIN, 1);
  lcd_send_data(cmd);
  xl.digitalWrite(LCD_CS_PIN, 1);
}

//##########################################################################################
void lcd_data(const uint8_t *data, int len) {
  uint32_t i = 0;
  if (len == 0)
    return; // no need to send anything
  do {
    xl.digitalWrite(LCD_CS_PIN, 0);
    xl.digitalWrite(LCD_SDA_PIN, 1);
    xl.digitalWrite(LCD_CLK_PIN, 0);
    xl.digitalWrite(LCD_CLK_PIN, 1);
    lcd_send_data(*(data + i));
    xl.digitalWrite(LCD_CS_PIN, 1);
    i++;
  } while (len--);
}

//##########################################################################################
void tft_init(void) {
  xl.digitalWrite(LCD_CS_PIN, 1);
  xl.digitalWrite(LCD_SDA_PIN, 1);
  xl.digitalWrite(LCD_CLK_PIN, 1);

  // Reset the display
  xl.digitalWrite(LCD_RST_PIN, 1);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl.digitalWrite(LCD_RST_PIN, 0);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  xl.digitalWrite(LCD_RST_PIN, 1);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  int cmd = 0;
  while (st_init_cmds[cmd].databytes != 0xff) {
    lcd_cmd(st_init_cmds[cmd].cmd);
    lcd_data(st_init_cmds[cmd].data, st_init_cmds[cmd].databytes & 0x1F);
    if (st_init_cmds[cmd].databytes & 0x80) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    cmd++;
  }
  Serial.println("Register setup complete");
}

//##########################################################################################
/*   The below functions are old and were not working well, so gave up
void doGesture()
{
  
  
  if (millis() - _touchTS > TOUCH_INTERVAL) 
  {
    _touchTS = millis();
    // process our touch events
    tsPoint_t pt;
    
    _touched = tryGetTouchEvent(&pt);
    if(_touched)
      _touchLast = pt;
      //Serial.print("Touched ");
    if (_touched != _touchedOld) {
      if (_touched) {
        _touchFirst = pt;
        //Serial.print(" FirstX: ");
        //Serial.print(_touchFirst.x);
        //Serial.print(" FirstY: ");
        //Serial.print(_touchFirst.y);
        
      } else {
        pt = _touchLast;
        //Serial.print(" LastX: ");
        //Serial.print(pt.x);
        //Serial.print(" LastY: ");
        //Serial.print(pt.y);

        // compute differences
        int32_t dx = pt.x-_touchFirst.x;
        int32_t dy = pt.y-_touchFirst.y;
        uint32_t adx=abs(dx);
        uint32_t ady=abs(dy);
        Serial.print(millis());
        Serial.print(" DeltaX: ");
        Serial.print(adx);
        Serial.print(" DeltaY: ");
        Serial.print(ady);
        Serial.println("");


        
        // swipe horizontal
        if(adx>ady && adx> TOUCH_THRESHOLD_X) 
        { 
          if(0>dx) { // swipe right to left
            //drawCentered("Right to left");
            //sprintf(buffer, "Right to left");
            sprintf(buffer, "< < < <");
            //_textTimeoutTS=millis();
          } else { // swipe left to right
            //drawCentered("Left to right");
            //sprintf(buffer, "Left to right");
            sprintf(buffer, "> > > >");
            //_textTimeoutTS=millis();
          }
        // swipe vertical
        } else if(ady>adx && ady>TOUCH_THRESHOLD_Y) { 
          if(0>dy) { // swipe bottom to top
            //drawCentered("Bottom to top");
            //sprintf(buffer, "Bottom to Top");
            sprintf(buffer, "up up up up");
            //_textTimeoutTS=millis();
          } else { // swipe top to bottom
            //drawCentered("Top to bottom");
            //sprintf(buffer, "Top to Bottom");
            sprintf(buffer, "v v v v");
            //_textTimeoutTS=millis();
          }
        }

        gfx->setTextColor(WHITE);
        gfx->fillRoundRect(120, 410, 230, 40, 8, BLUE);  //void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
        gfx->setCursor(145, 435);
        gfx->println(buffer);
        

      }
    }
    _touchedOld = _touched;
    memset(buffer, 0, sizeof(buffer));
  }
  // make the text we displayed disappear, if necessary
  //if (_textTimeoutTS && millis() - _textTimeoutTS > TEXT_TIMEOUT) {
  //  _textTimeoutTS = 0;
  //  gfx->fillScreen(BLACK);
  //}  
  
  //Serial.println(" ");
  touch_pin_get_int = false;
  
}
*/
// read the TFT touch
/*
bool tryGetTouchEvent(tsPoint_t  * point) 
{
  uint8_t touch_points_num;
  uint16_t x, y;
  ft3267_read_pos(&touch_points_num, &x, &y);
  //Serial.print("TP# ");Serial.print(touch_points_num);
  delay(1);
  if (touch_pin_get_int) {
    //char buff[40];
    //gfx->touchRead(&x, &y);
    //Serial.print(" touching ");
    ft3267_read_pos(&touch_points_num, &x, &y);
    point->x = x;
    point->y = y;
    //sprintf(buff, "X: %d Y: %d", x, y);
    //gfx->fillRoundRect(120, 210, 230, 40, 8, BLUE);
    //gfx->setCursor(145, 235);
    //gfx->println(buff);
    //memset(buff, 0, sizeof(buff));
    touch_pin_get_int = false;
    return true;
  }
  return false;
}
*/

//##########################################################################################
/*
static void getTouch()
{
  if (touch_pin_get_int) 
  {
    char buffer[40];
    uint8_t touch_points_num;
    //touch_point_t p = {0};
    //ft3267_read_pos(&touch_points_num, &p.x, &p.y);
    //sprintf(buffer, "X: %d Y: %d", p.x, p.y);

    //fx5x06_read_gesture();  //have to get this working at some point

    

    uint16_t x, y;
    ft3267_read_pos(&touch_points_num, &x, &y);
    sprintf(buffer, "X: %d Y: %d", x, y);

    gfx->setTextColor(WHITE);
    gfx->fillRoundRect(120, 410, 230, 40, 8, BLUE);  //void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
    
    //gfx->fillRoundRect(120, 250, 230, 100, 8, BLUE); 
    //gfx->setCursor(105, 350);
    //gfx->println("X: ");  
    //gfx->setCursor(145, 280);
    //gfx->println( p.x);  
    //gfx->setCursor(120, 320);
    //gfx->println((String)"Y: " + p.y);  
    //touch_pin_get_int = false;

    gfx->setCursor(145, 435);
    
    gfx->println(buffer);
    memset(buffer, 0, sizeof(buffer));
    touch_pin_get_int = false;
  }
}
*/
//##########################################################################################
/*
void testT()
{

  uint8_t touch_points_num;
  uint16_t x, y;
  tsPoint_t pt;
  char buff[40];
  //TINT

  if (touch_pin_get_int) 
  {
    ft3267_read_pos(&touch_points_num, &x, &y);
    //point->x = x;
    //point->y = y;
 
    
    
    sprintf(buff, "X: %d Y: %d", x, y);
    gfx->fillRoundRect(120, 210, 230, 340, 8, BLUE);
    gfx->setCursor(145, 235);
    gfx->println(buff);
    memset(buff, 0, sizeof(buff));
    
    if (doOnce) 
    {
        Serial.print("Start Touch ");
        gfx->setCursor(145, 265);
        gfx->println(touch_pin_get_int);
    }
    Serial.print(".");
    gfx->setCursor(145, 285);
    gfx->println(".");

    touch_pin_get_int = false;
    doOnce = false;
  }
  else
  {
    if (!doOnce)
    {
      Serial.println("Finished Touch");
       gfx->setCursor(145, 310);
       gfx->println(touch_pin_get_int);
      doOnce = true;
    }      
  }
}
*/
//##########################################################################################
