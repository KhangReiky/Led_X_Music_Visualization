/*
* Author: Truong Tan Khang
* Date  : 20/07/2020
* Use for music visualizer with esp32
*/
#include <Arduino.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "arduinoFFT.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "EEPROM.h"
#include <string.h>
#include <FastLED.h>
#include <math.h>
#include <stdio.h>
#include <BlynkSimpleEsp32_BLE.h>

#define BLYNK_USE_DIRECT_CONNECT
char auth_key[] = "fMHClDbTVSO9WVE2N9tk2UjYOhhFm9Mp";
volatile int Snake_Dir = 0;
BLYNK_WRITE(V0)
{
  Snake_Dir = 0;
}
BLYNK_WRITE(V1)
{
  Snake_Dir = 1;
}
BLYNK_WRITE(V2)
{
  Snake_Dir = 2;
}
BLYNK_WRITE(V3)
{
  Snake_Dir = 3;
}

arduinoFFT FFT = arduinoFFT();

#define SAMPLES 1024             // Must be a power of 2
#define SAMPLING_FREQUENCY 40000 // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.
#define amplitude 290            // Depending on your audio source level, you may need to increase this value
#define Left  true
#define Right false

#define LED_PIN     23
#define LED_COL     16
#define LED_ROW     15
#define LED_NUM     240

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

#define CHIPSET     WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS          48
#define FRAMES_PER_SECOND   100
CRGB KMV_Leds[LED_NUM];

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

uint8_t red[3]     = {255,  0, 0};
uint8_t blue[3]    = {0,  0, 255};
uint8_t green[3]   = {0,  255, 0};
uint8_t purple[3]  = {200,  0, 200};
uint8_t yellow[3]  = {200,  200, 0};
uint8_t skyblue[3] = {0,  200, 200};
uint8_t white[3]   = {150,  150, 150};

uint8_t gCurrentPatternNumber = 0;
uint8_t gHue = 0;
uint8_t gShowMakerNum = 0;

int dmax = LED_ROW - 1;
unsigned int sampling_period_us;
volatile byte Lpeak[] = {0,0,0,0,0,0,0,0};
volatile byte Rpeak[] = {0,0,0,0,0,0,0,0};
volatile uint8_t Lband[] = {0,0,0,0,0,0,0,0};
volatile uint8_t Rband[] = {0,0,0,0,0,0,0,0};
volatile uint8_t D_Lband[] = {0,0,0,0,0,0,0,0};
volatile uint8_t D_Rband[] = {0,0,0,0,0,0,0,0};

double LvReal[SAMPLES];
double LvImag[SAMPLES];

double RvReal[SAMPLES];
double RvImag[SAMPLES];

unsigned long newTime;

#define EEPROM_SIZE 2

// For generating UUIDs: https://www.uuidgenerator.net
// BLE Service
#define SERVICE_KMV_UUID              "4F00"
#define SERVICE_INFORMATION_UUID      "180A"

#define CHAR_UUID_KMV_COMM            "4F01"
#define CHAR_UUID_KMV_DATA            "4F02"
#define CHAR_UUID_DEVICE_INFOR        "2A00"
#define CHAR_UUID_MANUF_INFOR         "2A29"
#define CHAR_UUID_MODEL_NUM           "2A24"
#define CHAR_UUID_SERIAL_NUM          "2A25"

// BLE Server
BLEServer* pServer = NULL;
// BLE Characteristic
BLECharacteristic* KMV_Comm_Characteristic      = NULL;
BLECharacteristic* KMV_Data_Characteristic      = NULL;
BLECharacteristic* Device_infor_Characteristic  = NULL;
BLECharacteristic* Manuf_infor_Characteristic   = NULL;
BLECharacteristic* Model_num_Characteristic     = NULL;
BLECharacteristic* Serial_num_Characteristic    = NULL;

volatile bool Test_Color = false;
uint8_t Array_Test_Color[3];
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t Led_X_Mode[1] = {0};

bool Update_Flag = false;
bool Test_Color_Flag = false;

uint8_t Snake_Color[3] = {0,0,0};
uint8_t Food_Color[3] = {0,0,0};
int S_Head[2] = {0,0};
int F_Pos[2] = {0,0};
uint8_t S_Body[LED_NUM];
int S_Len = 1;
bool Need_New_Food = 1;
// AI Snake Mode
void AI_Hunting_Snake();
void Update_Color();
void Snake_Generate();
void Food_Generate();
bool Sniffing_Food();
void Eat_Food();
bool Find_Another_Way(int RL, int UD);
bool Check_Path(int *Check_Pos);
void Update_Body();
// Control Snake Mode
void Control_Snake();
bool Go_Ahead();
// EEPROM
void Setup_eeprom();
// Show Maker Mode
void Show_Maker_Mode();
void Next_Show_Maker();
void Rainbow_1();
void Rainbow_2();
void Fade_Led();
void Fade_Between(uint8_t *a, uint8_t *b, int del);
void Set_All_Led(uint8_t *a);
// Music Mode
void Music_Mode();
void nextPattern();
void Palette_Mode();
void FillLEDsFromPaletteColors( uint8_t colorIndex);
CRGB LightOut();
void LedDrawPeak();
void LedDownPeak();
void ChangePalettePeriodically();
void SetupTotallyRandomPalette();
void RedPalette();
void WhitePalette();
void PurplePalette();
void GreenPalette();
// BLE
void BLE_Config();
void Check_Connect_BLE();
void BLE_Notify();
void Parsing_Color_Data(String BLE_Data);
// FFT
void OptimizeBand();
void displayBand(bool channel, int band, int dsize);
void GetSampleMusic();

/** Server Callbacks **/
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

/** Security Callbacks **/
class MySecurity : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest(){
    ESP_LOGI(LOG_TAG, "PassKeyRequest");
    return 123456;
  }
  void onPassKeyNotify(uint32_t pass_key){
    ESP_LOGI(LOG_TAG, "The passkey Notify number:%d", pass_key);
  }
  bool onConfirmPIN(uint32_t pass_key){
    ESP_LOGI(LOG_TAG, "The passkey YES/NO number:%d", pass_key);
    return true;
  }
  bool onSecurityRequest(){
    ESP_LOGI(LOG_TAG, "SecurityRequest");
    return true;
  }

  void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl){
    ESP_LOGI(LOG_TAG, "Starting BLE work!");
    if(cmpl.success){
      esp_ble_gap_update_whitelist(true, cmpl.bd_addr);
      ESP_LOGD(LOG_TAG, "size: %d", length);
      Device_infor_Characteristic->setValue("KMV - Music Visualizer");
    }
  }
};

/** Bluetooth LE Characteristi Callbacks **/
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      std::string rxCommand = pCharacteristic -> getValue();
      String RxComm_Str = "";
      if (rxCommand.length() > 0) {
        for (int i = 0; i < rxCommand.length(); i++) {
          if (i < rxCommand.length()) {
            RxComm_Str += rxCommand[i];
          }
        }
        if (rxCommand.length() > 1) {
          std::string Setting_Mode = rxCommand.substr(0,2);
          if(Setting_Mode == "SM") 
          {
            Led_X_Mode[0] = rxCommand[2] - 48;
            EEPROM.writeUChar(0, Led_X_Mode[0]);
            EEPROM.commit();
            BLE_Notify();
            if((Led_X_Mode[0] == 2) || (Led_X_Mode[0] == 3))
            {
              Snake_Generate();
            }
            Serial.print("Mode: ");
            Serial.println(Led_X_Mode[0]);
          }
          else if(Setting_Mode == "TC")
          {
            if(rxCommand.length() > 2)
            {
              Test_Color = true;
              Parsing_Color_Data(RxComm_Str);
              Test_Color_Flag = 1;
            }
            else
            {
              Test_Color = false;
              Serial.println("Test Color End");
            }
          }
          else if(Setting_Mode == "SB")
          {
            uint8_t Led_Brightness = RxComm_Str.substring(2,5).toInt();
            FastLED.setBrightness(Led_Brightness);
            EEPROM.writeUChar(1, Led_Brightness);
            EEPROM.commit();
            Serial.print("Led Brightness: ");
            Serial.println(Led_Brightness);
          }
        }
      }
    }
};

typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { 
  Palette_Mode,Palette_Mode,Palette_Mode,Palette_Mode,Palette_Mode,Palette_Mode,
  Palette_Mode,Palette_Mode,Palette_Mode,Palette_Mode,Palette_Mode,Palette_Mode
};

typedef void (*Show_Maker_List[])();
Show_Maker_List gShow_Makers = {
  Rainbow_1,
  Rainbow_2,
  Fade_Led
};

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  delay(1000);
  Serial.begin(115200);

  Setup_eeprom();
  BLE_Config();
  // Blynk.begin(auth_key);

  Led_X_Mode[0] = EEPROM.readUChar(0);
  if((Led_X_Mode[0] == 2) || (Led_X_Mode[0] == 3))
  {
    Snake_Generate();
  }
  uint8_t Start_Brightness = EEPROM.readUChar(1);
  if(Start_Brightness == 0)
  {
    Start_Brightness = BRIGHTNESS;
  }
  Serial.print("Mode: ");
  Serial.println(Led_X_Mode[0]);
  Serial.print("Brightness: ");
  Serial.println(Start_Brightness);

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(KMV_Leds, LED_NUM).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( Start_Brightness );
  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
}

void loop()
{
  Check_Connect_BLE();
  if(Test_Color == false)
  {
    if(Led_X_Mode[0] == 0)
    {
      Music_Mode();
    }
    else if(Led_X_Mode[0] == 1)
    {
      Show_Maker_Mode();
    }
    else if(Led_X_Mode[0] == 2)
    {
      AI_Hunting_Snake();
    }
    else if(Led_X_Mode[0] == 3)
    {
      Control_Snake();
    }
  }
  else
  {
    if(Test_Color_Flag == 1)
    {
      for (int i = 0; i < LED_NUM; i++)
      {
        KMV_Leds[i] = CRGB(Array_Test_Color[0], Array_Test_Color[1], Array_Test_Color[2]);
      }
      FastLED.show();
      Serial.print("Test Color: ");
      Serial.print(Array_Test_Color[0]);
      Serial.print(" ");
      Serial.print(Array_Test_Color[1]);
      Serial.print(" ");
      Serial.println(Array_Test_Color[2]);
      Test_Color_Flag = 0;
    }
  }
}

void Setup_eeprom()
{
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("failed to initialise EEPROM");
    delay(100);
  }
}

void Control_Snake()
{
  bool Win_Or_Lose = 0;
  if(Need_New_Food == 1)
  {
    Food_Generate();
    Need_New_Food = 0;
    Update_Body();
    Update_Color();
    delay(100);
  }
  Win_Or_Lose = Go_Ahead();
  if(Win_Or_Lose == 1)
  {
    Update_Body();
    Update_Color();
    Serial.print(S_Head[0]);
    Serial.print(" ");
    Serial.println(S_Head[1]);
  }
  else
  {
    if(S_Len > (LED_NUM/2))
    {
      Serial.print("Win - Your Length: ");
      Serial.println(S_Len);
    }
    else
    {
      Serial.print("Lose - Your Length: ");
      Serial.println(S_Len);
    }
    Snake_Generate();
    Need_New_Food = 1;
    delay(1000);
  }
  delay(100);
}

bool Go_Ahead()
{
  bool GA_Flag = 0;
  if(Snake_Dir == 0)
  {
    S_Head[1] += 1;
  }
  else if(Snake_Dir == 1)
  {
    S_Head[1] -= 1;
  }
  else if(Snake_Dir)
  {
    S_Head[0] -= 1;
  }
  else if(Snake_Dir)
  {
    S_Head[0] += 1;
  }
  GA_Flag = Check_Path(S_Head);
  return GA_Flag;
}

void AI_Hunting_Snake()
{
  bool Continue_Or_Not = 0;
  if(Need_New_Food == 1)
  {
    Food_Generate();
    Need_New_Food = 0;
    Update_Body();
    Update_Color();
    delay(100);
  }
  Continue_Or_Not = Sniffing_Food();
  if(Continue_Or_Not == 1)
  {
    Update_Body();
    Update_Color();
    Serial.print(S_Head[0]);
    Serial.print(" ");
    Serial.println(S_Head[1]);
  }
  else
  {
    if(S_Len > (LED_NUM/2))
    {
      Serial.print("Win - Your Length: ");
      Serial.println(S_Len);
    }
    else
    {
      Serial.print("Lose - Your Length: ");
      Serial.println(S_Len);
    }
    Snake_Generate();
    Need_New_Food = 1;
    delay(1000);
  }
  delay(100);
}

void Update_Color()
{
  FastLED.clear();
  uint8_t Food_Pos = F_Pos[0]*LED_ROW;
  if(F_Pos[0]%2 == 0)
  {
    Food_Pos += F_Pos[1];
  }
  else
  {
    Food_Pos = Food_Pos + 14 - F_Pos[1];
  }
  KMV_Leds[Food_Pos] = CRGB::Red;

  for (int i = 0; i < S_Len; i++)
  {
    uint8_t Snake_Pos = S_Body[i];
    KMV_Leds[Snake_Pos] = CRGB::Green;
  }
  FastLED.show();
}

void Snake_Generate()
{
  S_Head[0] = random8(0, LED_COL-1);
  S_Head[1] = random8(0, LED_ROW-1);
  S_Len = 1;
}

void Food_Generate()
{
  volatile bool FG_Flag = 0;
  do
  {
    F_Pos[0] = random8(0, LED_COL-1);
    F_Pos[1] = random8(0, LED_ROW-1);
    FG_Flag = Check_Path(F_Pos);
  } while (FG_Flag == 0);
}

bool Sniffing_Food()
{
  volatile bool Sniffing_Flag = 0;
  int Right_Left = 0; // Right = -1, Ledt = 1
  int Up_Down = 0;    // Up = -1, Down = 1
  if(((abs(F_Pos[0] - S_Head[0]) == 1) && (abs(F_Pos[1] - S_Head[1]) == 0))
    || ((abs(F_Pos[0] - S_Head[0]) == 0) && (abs(F_Pos[1] - S_Head[1]) == 1)))
  {
    Eat_Food();
    return 1;
  }
  else
  {
    if(abs(F_Pos[0] - S_Head[0]) > 0)
    {
      if(F_Pos[0] > S_Head[0])
      {
        S_Head[0] += 1;
        Sniffing_Flag = Check_Path(S_Head);
        if(Sniffing_Flag == 0)
        {
          S_Head[0] -= 1;
          Right_Left = -1;
        }
      }
      else if(F_Pos[0] < S_Head[0])
      {
        S_Head[0] -= 1;
        Sniffing_Flag = Check_Path(S_Head);
        if(Sniffing_Flag == 0)
        {
          S_Head[0] += 1;
          Right_Left = 1;
        }
      }
    }
    if((abs(F_Pos[1] - S_Head[1]) > 0) && (Sniffing_Flag == 0))
    {
      if(F_Pos[1] > S_Head[1])
      {
        S_Head[1] += 1;
        Sniffing_Flag = Check_Path(S_Head);
        if(Sniffing_Flag == 0)
        {
          S_Head[1] -= 1;
          Up_Down = -1;
        }
      }
      else if(F_Pos[1] < S_Head[1])
      {
        S_Head[1] -= 1;
        Sniffing_Flag = Check_Path(S_Head);
        if(Sniffing_Flag == 0)
        {
          S_Head[1] += 1;
          Up_Down = 1;
        }
      }
    }
  }
  if(Sniffing_Flag == 0)
  {
    Sniffing_Flag = Find_Another_Way(Right_Left, Up_Down);
  }
  return Sniffing_Flag;
}

void Eat_Food()
{
  S_Head[0] = F_Pos[0];
  S_Head[1] = F_Pos[1];
  S_Len += 1;
  Need_New_Food = 1;
}

bool Find_Another_Way(int RL, int UD)
{
  volatile bool FAW_Flag = 0;
  if((RL != 0) && (UD != 0))
  {
    uint8_t Ran_1 = random8(1);
    if(Ran_1 == 0)
    {
      S_Head[0] += RL;
      FAW_Flag = Check_Path(S_Head);
      if(FAW_Flag == 0)
      {
        S_Head[0] -= RL;
        S_Head[1] += UD;
        FAW_Flag = Check_Path(S_Head);
        if(FAW_Flag == 0)
        {
          S_Head[1] -= UD;
        }
      }
    }
    else if(Ran_1 == 1)
    {
      S_Head[1] += UD;
      FAW_Flag = Check_Path(S_Head);
      if(FAW_Flag == 0)
      {
        S_Head[1] -= UD;
        S_Head[0] += RL;
        FAW_Flag = Check_Path(S_Head);
        if(FAW_Flag == 0)
        {
          S_Head[0] -= RL;
        }
      }
    }
  }
  else if((RL != 0) && (UD == 0))
  {
    uint8_t Ran_2 = random8(1);
    if(Ran_2 == 0)
    {
      S_Head[1] += 1;
      FAW_Flag = Check_Path(S_Head);
      if(FAW_Flag == 0)
      {
        S_Head[1] -= 2;
        FAW_Flag = Check_Path(S_Head);
        if(FAW_Flag == 0)
        {
          S_Head[1] += 1;
        }
      }
    }
    else if(Ran_2 == 1)
    {
      S_Head[1] -= 1;
      FAW_Flag = Check_Path(S_Head);
      if(FAW_Flag == 0)
      {
        S_Head[1] += 2;
        FAW_Flag = Check_Path(S_Head);
        if(FAW_Flag == 0)
        {
          S_Head[1] -= 1;
        }
      }
    }
    if(FAW_Flag == 0)
    {
      S_Head[0] += RL;
      FAW_Flag = Check_Path(S_Head);
      if(FAW_Flag == 0)
      {
        S_Head[0] -= RL;
      }
    }
  }
  else if((RL == 0) && (UD != 0))
  {
    uint8_t Ran_3 = random8(1);
    if(Ran_3 == 0)
    {
      S_Head[0] += 1;
      FAW_Flag = Check_Path(S_Head);
      if(FAW_Flag == 0)
      {
        S_Head[0] -= 2;
        FAW_Flag = Check_Path(S_Head);
        if(FAW_Flag == 0)
        {
          S_Head[0] += 1;
        }
      }
    }
    else if(Ran_3 == 1)
    {
      S_Head[0] -= 1;
      FAW_Flag = Check_Path(S_Head);
      if(FAW_Flag == 0)
      {
        S_Head[0] += 2;
        FAW_Flag = Check_Path(S_Head);
        if(FAW_Flag == 0)
        {
          S_Head[0] -= 1;
        }
      }
    }
    if(FAW_Flag == 0)
    {
      S_Head[1] += UD;
      FAW_Flag = Check_Path(S_Head);
      if(FAW_Flag == 0)
      {
        S_Head[1] -= UD;
      }
    }
  }
  return FAW_Flag;
}

bool Check_Path(int *Check_Pos)
{
  volatile bool Check_Flag = 0;
  uint8_t Check_Object = Check_Pos[0]*LED_ROW;
  if(Check_Pos[0]%2 == 0)
  {
    Check_Object += Check_Pos[1];
  }
  else
  {
    Check_Object = Check_Object + 14 - Check_Pos[1];
  }
  for (int i = 0; i < S_Len; i++)
  {
    if(S_Body[i] == Check_Object)
    {
      return 0;
    }
    if(S_Body[S_Len - 1] != Check_Object)
    {
      Check_Flag = 1;
    }
  }
  return Check_Flag;
}

void Update_Body()
{
  for (int i = S_Len; i > 1; i--)
  {
    S_Body[i-1] = S_Body[i-2];
  }
  if(S_Head[0] < 0)
  {
    S_Head[0] += LED_COL;
  }
  else if(S_Head[0] > (LED_COL - 1))
  {
    S_Head[0] -= LED_COL;
  }
  if(S_Head[1] < 0)
  {
    S_Head[1] += LED_ROW;
  }
  else if(S_Head[1] > (LED_ROW - 1))
  {
    S_Head[1] -= LED_ROW;
  }
  uint8_t Head_Pos = S_Head[0]*LED_ROW;
  if(S_Head[0]%2 == 0)
  {
    Head_Pos += S_Head[1];
  }
  else
  {
    Head_Pos = Head_Pos + 14 - S_Head[1];
  }
  S_Body[0] = Head_Pos;
}

void Show_Maker_Mode()
{
  gShow_Makers[gShowMakerNum]();
  FastLED.show();  
  FastLED.delay(1000/FRAMES_PER_SECOND);
  gHue++;
  if(gHue == 255)
  {
    Next_Show_Maker();
  }
}

void Next_Show_Maker()
{
  gShowMakerNum = (gShowMakerNum + 1) % ARRAY_SIZE(gShow_Makers);
}

void Rainbow_1() 
{
  fill_rainbow( KMV_Leds, LED_NUM, gHue, 7);
}

void Rainbow_2() 
{
  fill_rainbow( KMV_Leds, LED_NUM, gHue, 20);
}

void Fade_Led() 
{
  Fade_Between(white, blue, FRAMES_PER_SECOND);
  delay(FRAMES_PER_SECOND);
  Fade_Between(blue, skyblue, FRAMES_PER_SECOND);
  delay(FRAMES_PER_SECOND);
  Fade_Between(skyblue, green, FRAMES_PER_SECOND);
  delay(FRAMES_PER_SECOND);
  Fade_Between(green, yellow, FRAMES_PER_SECOND);
  delay(FRAMES_PER_SECOND);
  Fade_Between(yellow, red, FRAMES_PER_SECOND);
  delay(FRAMES_PER_SECOND);
  Fade_Between(red, purple, FRAMES_PER_SECOND);
  delay(FRAMES_PER_SECOND);
  Fade_Between(purple, white, FRAMES_PER_SECOND);
  delay(FRAMES_PER_SECOND);
  Fade_Between(white, white, FRAMES_PER_SECOND);
  delay(FRAMES_PER_SECOND);
}

void Fade_Between(uint8_t *a, uint8_t *b, int del) 
{
  int steps = 100;
  double dsteps = 100.0;
  double s1, s2, s3, tmp1, tmp2, tmp3;
  s1 = double((b[0] - a[0])) / dsteps; 
  s2 = double((b[1] - a[1])) / dsteps; 
  s3 = double((b[2] - a[2])) / dsteps; 
  tmp1 = a[0], tmp2 = a[1], tmp3 = a[2];
  for (int i = 0; i < steps; i++) 
  { 
    tmp1 += s1;
    tmp2 += s2; 
    tmp3 += s3;      
    for (int j = 0; j < LED_NUM; j++)
    KMV_Leds[j] = CRGB((int)round(tmp1), (int)round(tmp2), (int)round(tmp3)); 
    FastLED.show(); 
    delay(del);
  }
  Set_All_Led(b);
}

void Set_All_Led(uint8_t *a) 
{
  for (int i = 0; i < LED_NUM; i++) 
  KMV_Leds[i] = CRGB(a[0], a[1], a[2]);  
  FastLED.show();
}

void Music_Mode()
{
  GetSampleMusic();
  OptimizeBand();
  gPatterns[gCurrentPatternNumber]();
  LedDrawPeak();
  FastLED.show();
  if(millis()%2 == 0)
  {
    LedDownPeak();
    FastLED.show();
  }
  gHue++;
  if(gHue == 255)
  {
    nextPattern();
  }
}

void nextPattern()
{
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE(gPatterns);
}

void Palette_Mode()
{
  ChangePalettePeriodically();
  static uint8_t startIndex = 0;
  startIndex = startIndex + 1; // Motion speed
  FillLEDsFromPaletteColors(startIndex);
}

void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
  uint8_t brightness = BRIGHTNESS;
  for( int i = 0; i < LED_NUM; i++) 
  {
    uint8_t Col = i/LED_ROW;
    uint8_t Row = i%LED_ROW;
    if(Col < 8)
    {
      if(Col%2 == 0)
      {
        if(Row < D_Lband[Col])
        {
          KMV_Leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        }
        else
        {
          LightOut();
        }
      }
      else
      {
        if(Row < (LED_ROW - D_Lband[Col]))
        {
          LightOut();
        }
        else
        {
          KMV_Leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        }
      }
    }
    else
    {
      if(Col%2 == 0)
      {
        if(Row < D_Rband[15-Col])
        {
          KMV_Leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        }
        else
        {
          LightOut();
        }
      }
      else
      {
        if(Row < (LED_ROW - D_Rband[15-Col]))
        {
          LightOut();
        }
        else
        {
          KMV_Leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        }
      }
    }
    colorIndex += 3;
  }
}

CRGB LightOut()
{
  return CRGB(0,0,0);
}

void LedDrawPeak()
{
  static uint8_t colorIndex = 0;
  uint8_t brightness = BRIGHTNESS;
  for (int i = 0; i < LED_COL; i++)
  {
    uint8_t Led_Seq;
    if(i < 8)
    {
      if(i%2 == 0)
      {
        Led_Seq = i*LED_ROW + Lpeak[i];
        for (int j = 0; j < (LED_ROW - Lpeak[i]); j++)
        {
          KMV_Leds[i*LED_ROW + Lpeak[i] + 1 + j] = LightOut();
        }
      }
      else
      {
        Led_Seq = i*LED_ROW + 14 - Lpeak[i];
        for (int j = 0; j < (LED_ROW - Lpeak[i]); j++)
        {
          KMV_Leds[i*LED_ROW + 14 - Lpeak[i] - 1 - j] = LightOut();
        }
      }
    }
    else
    {
      if(i%2 == 0)
      {
        Led_Seq = i*LED_ROW + Rpeak[15-i];
        for (int j = 0; j < (LED_ROW - Rpeak[15-i]); j++)
        {
          KMV_Leds[i*LED_ROW + Rpeak[15-i] + 1 + j] = LightOut();
        }
      }
      else
      {
        Led_Seq = i*LED_ROW + 14 - Rpeak[15-i];
        for (int j = 0; j < (LED_ROW - Rpeak[15-i]); j++)
        {
          KMV_Leds[i*LED_ROW + 14 - Rpeak[15-i] - 1 - j] = LightOut();
        }
      }
    }
    KMV_Leds[Led_Seq] = ColorFromPalette( PartyColors_p, colorIndex, brightness, currentBlending);
    colorIndex += 3;
  }
}

void LedDownPeak()
{
  for (int i = 0; i < LED_COL; i++)
  {
    if((i < 8) && (Lpeak[i] > 0))
    {
      Lpeak[i] -= 1;
      if(D_Lband[i] > Lpeak[i]) {Lband[i] -= 1;}
    }
    else if((i >= 8) && (Rpeak[15-i] > 0))
    {
      Rpeak[15-i] -= 1;
      if(D_Rband[15-i] > Rpeak[15-i]) {Rband[15-i] -=1;}
    }
  }
  LedDrawPeak();
}

void ChangePalettePeriodically()
{
  uint8_t secondHand = (millis() / 6000) % 60;
  static uint8_t lastSecond = 99;   
  if( lastSecond != secondHand) 
  {
    lastSecond = secondHand;
    if( secondHand == 0)        { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
    else if( secondHand == 5)   { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND; }
    else if( secondHand == 10)  { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
    else if( secondHand == 15)  { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND; }
    else if( secondHand == 20)  { RedPalette();                             currentBlending = LINEARBLEND; }
    else if( secondHand == 25)  { WhitePalette();                           currentBlending = LINEARBLEND; }      
    else if( secondHand == 30)  { PurplePalette();                          currentBlending = LINEARBLEND; }
    else if( secondHand == 35)  { GreenPalette();                           currentBlending = LINEARBLEND; }
    else if( secondHand == 30)  { currentPalette = CloudColors_p;           currentBlending = LINEARBLEND; }
    else if( secondHand == 40)  { currentPalette = OceanColors_p;           currentBlending = LINEARBLEND; }
    else if( secondHand == 35)  { currentPalette = ForestColors_p;          currentBlending = LINEARBLEND; }
    else if( secondHand == 40)  { currentPalette = LavaColors_p;            currentBlending = LINEARBLEND; }
    else if( secondHand == 45)  { currentPalette = PartyColors_p;           currentBlending = LINEARBLEND; }
    else if( secondHand == 50)  { SetupTotallyRandomPalette();              currentBlending = LINEARBLEND; }
    else if( secondHand == 55)  { SetupTotallyRandomPalette();              currentBlending = LINEARBLEND; }
  }
}

void SetupTotallyRandomPalette()
{
  for( int i = 0; i < 16; i++) {
    currentPalette[i] = CHSV( random8(), 255, random8());
  }
}

void RedPalette()
{
  fill_solid( currentPalette, 16, CRGB::Red);    
  currentPalette[0] = CRGB::Red;
  currentPalette[2] = CRGB::White;
  currentPalette[4] = CRGB::Red;
  currentPalette[6] = CRGB::Blue;
  currentPalette[8] = CRGB::Red;
  currentPalette[12] = CRGB::Yellow;      
}

void WhitePalette()
{
  fill_solid( currentPalette, 16, CRGB::White);    
  currentPalette[0] = CRGB::White;
  currentPalette[4] = CRGB::White;
  currentPalette[8] = CRGB::White;
  currentPalette[12] = CRGB::Red;  
}

void PurplePalette()
{
  fill_solid( currentPalette, 16, CRGB::Purple);    
  currentPalette[0] = CRGB::Purple;
  currentPalette[2] = CRGB::Blue;
  currentPalette[4] = CRGB::Purple;
  currentPalette[6] = CRGB::Red;
  currentPalette[8] = CRGB::Purple;
  currentPalette[12] = CRGB::Green;    
}

void GreenPalette()
{
  fill_solid( currentPalette, 16, CRGB::Green);    
  currentPalette[0] = CRGB::Green;
  currentPalette[2] = CRGB::White;
  currentPalette[4] = CRGB::Green;
  currentPalette[6] = CRGB::Red;
  currentPalette[8] = CRGB::Green;
  currentPalette[12] = CRGB::Purple;
}

void BLE_Config()
{
  // Create the BLE Device
  BLEDevice::init("Music Visualizer 1");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new MySecurity());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService_KMV          = pServer->createService(SERVICE_KMV_UUID);
  BLEService *pService_device_infor = pServer->createService(SERVICE_INFORMATION_UUID);

  // KMV Service
  KMV_Comm_Characteristic = pService_KMV->createCharacteristic(
                    CHAR_UUID_KMV_COMM,
                    BLECharacteristic::PROPERTY_READ|
                    BLECharacteristic::PROPERTY_WRITE
                  );
  KMV_Data_Characteristic = pService_KMV->createCharacteristic(
                    CHAR_UUID_KMV_DATA,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
  KMV_Data_Characteristic->addDescriptor(new BLE2902);

  // Device information service
  Device_infor_Characteristic = pService_device_infor->createCharacteristic(
                    CHAR_UUID_DEVICE_INFOR,
                    BLECharacteristic::PROPERTY_READ
                  );
  Manuf_infor_Characteristic = pService_device_infor->createCharacteristic(
                    CHAR_UUID_MANUF_INFOR,
                    BLECharacteristic::PROPERTY_READ
                  );
  Model_num_Characteristic = pService_device_infor->createCharacteristic(
                    CHAR_UUID_MODEL_NUM,
                    BLECharacteristic::PROPERTY_READ
                  );
  Serial_num_Characteristic = pService_device_infor->createCharacteristic(
                    CHAR_UUID_SERIAL_NUM,
                    BLECharacteristic::PROPERTY_READ
                  );

  KMV_Comm_Characteristic->setCallbacks(new MyCallbacks);

  // Start the service
  pService_KMV->start();
  pService_device_infor->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  // BLESecurity *pSecurity = new BLESecurity();
  // pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY);
  // pSecurity->setCapability(ESP_IO_CAP_OUT);
  // pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  // pSecurity->setPassKey(123456);

  Device_infor_Characteristic->setValue("KMV - Music Visualizer");
  Manuf_infor_Characteristic->setValue("KMV JSC");
  Model_num_Characteristic->setValue("Version 1.0");
  Serial_num_Characteristic->setValue("00001");

  Serial.println("Waiting a client connection to notify ...");
}

void Check_Connect_BLE()
{
  // Device is connected
  if (deviceConnected) {
    // do nothing
  }
  // Device disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    // restart advertising
    pServer->startAdvertising();
    Serial.println("Start advertising"); 
    oldDeviceConnected = deviceConnected;
  }
  // Device connecting
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("Client connected");
    BLE_Notify();
  }
}

void BLE_Notify()
{
  KMV_Data_Characteristic->setValue(Led_X_Mode, 1);
  KMV_Data_Characteristic->notify();
}

void Parsing_Color_Data(String BLE_Data)
{
  Array_Test_Color[0] = BLE_Data.substring(2,5).toInt();
  Array_Test_Color[1] = BLE_Data.substring(5,8).toInt();
  Array_Test_Color[2] = BLE_Data.substring(8,11).toInt();
}

void OptimizeBand()
{
  for (int i = 0; i < 8; i++)
  {
    if(Lband[i] > D_Lband[i])
    {
      D_Lband[i] += 1;
    }
    else if(Lband[i] < D_Lband[i])
    {
      D_Lband[i] -= 1;
    }

    if(Rband[i] > D_Rband[i])
    {
      D_Rband[i] += 1;
    }
    else if(Rband[i] < D_Rband[i])
    {
      D_Rband[i] -= 1;
    }

    if(D_Lband[i] > Lpeak[i]) {Lpeak[i] = D_Lband[i];}
    if(D_Rband[i] > Rpeak[i]) {Rpeak[i] = D_Rband[i];}
  }
}

void displayBand(bool channel, int band, int dsize)
{
  dsize /= amplitude;
  if (dsize > dmax) 
  {
    dsize = dmax;
  }

  if(channel == Left)
  {
    Lband[band] = dsize;
  }
  else
  {
    Rband[band] = dsize;
  }
}

void GetSampleMusic()
{
    for (int i = 0; i < SAMPLES; i++) {
    newTime = micros();
    //VP input = Left, VN = Right
    LvReal[i] = analogRead(36); // Using Arduino ADC nomenclature. A conversion takes about 1uS on an ESP32
    LvImag[i] = 0;
    RvReal[i] = analogRead(39); // Using Arduino ADC nomenclature. A conversion takes about 1uS on an ESP32
    RvImag[i] = 0;
    while ((micros() - newTime) < sampling_period_us) { /* do nothing to wait */ }
    }
    FFT.Windowing(LvReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(LvReal, LvImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(LvReal, LvImag, SAMPLES);

    FFT.Windowing(RvReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(RvReal, RvImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(RvReal, RvImag, SAMPLES);
    
    for (int i = 1; i < (SAMPLES/2); i++)
    {   // Don't use sample 0 and only the first SAMPLES/2 are usable.
        // Each array element represents a frequency and its value, is the amplitude. Note the frequencies are not discrete.
        if (LvReal[i] > 2*amplitude || RvReal[i] > 2*amplitude) // Add a crude noise filter, 10 x amplitude or more
        {
            if (i<=4) {
                displayBand(Left,0,(int)LvReal[i]);  // 125Hz
                displayBand(Right,0,(int)RvReal[i]); // 125Hz
            }
            else if (i>4 && i<=7) {
                displayBand(Left,1,(int)LvReal[i]);  // 250Hz
                displayBand(Right,1,(int)RvReal[i]); // 250Hz
            }
            else if (i>7 && i<=13) {
                displayBand(Left,2,(int)LvReal[i]);  // 500Hz
                displayBand(Right,2,(int)RvReal[i]); // 500Hz
            }
            else if (i>13 && i<=26) {
                displayBand(Left,3,(int)LvReal[i]);  // 1000Hz
                displayBand(Right,3,(int)RvReal[i]); // 1000Hz
            }
            else if (i>26 && i<=52) {
                displayBand(Left,4,(int)LvReal[i]);  // 2000Hz
                displayBand(Right,4,(int)RvReal[i]); // 2000Hz
            }
            else if (i>52 && i<=103) {
                displayBand(Left,5,(int)LvReal[i]);  // 4000Hz
                displayBand(Right,5,(int)RvReal[i]); // 4000Hz
            }
            else if (i>103 && i<=206) {
                displayBand(Left,6,(int)LvReal[i]);  // 8000Hz
                displayBand(Right,6,(int)RvReal[i]); // 8000Hz
            }
            else if (i>206 && i<=411) {
                displayBand(Left,7,(int)LvReal[i]);  // 16000Hz
                displayBand(Right,7,(int)RvReal[i]); // 16000Hz
            }
        }
    }
}