#include <esp_now.h>  //引用函式庫
#include <WiFi.h>
#include <Wire.h>
#include <EEPROM.h>

#include "RC.h"

#define WIFI_CHANNEL 4   /定義變數
#define PWMOUT  // normal esc, uncomment for serial esc
#define LED 2
#define CALSTEPS 256 // gyro and acc calibration steps
//#define externRC // use of external RC receiver in ppmsum mode
//#define webServer // use of webserver to change PID

extern int16_t accZero[3]; //告知程式到別的地方找尋變數的定義
extern float yawRate;
extern float rollPitchRate;
extern float P_PID;
extern float I_PID;
extern float D_PID;
extern float P_Level_PID;
extern float I_Level_PID;
extern float D_Level_PID;

volatile boolean recv;  //定義變數具有最佳化、多執行緒相關的特殊屬性，阻止編譯器因誤認某段程式碼無法被程式碼本身所改變
//volatile int peernum = 0;
//esp_now_peer_info_t slave;

void recv_cb(const uint8_t *macaddr, const uint8_t *data, int len) //定義recv_cb函式
{
  recv = true;
  //Serial.print("recv_cb ");  //印出recv_cb
  //Serial.println(len); //印出len值並換行
  if (len == RCdataSize) //如果RC數據大小等於len的值
  {
    for (int i=0;i<RCdataSize;i++) //在i的值小於RC數據大小時會執行此迴圈
    RCdata.data[i] = data[i];  //把RC數據的陣列指定給data陣列
  }
  /*
  if (!esp_now_is_peer_exist(macaddr))  //如果不存在於mac address
  {
    Serial.println("adding peer "); //印出adding peer且換行
    esp_now_add_peer(macaddr, ESP_NOW_ROLE_COMBO, WIFI_CHANNEL, NULL, 0); //使用此函式以配對esp設備
    peernum++; //peernum+1
  }
  */
};

#define ACCRESO 4096  //定義變數
#define CYCLETIME 3
#define MINTHROTTLE 1090
#define MIDRUD 1495
#define THRCORR 19

enum ang { ROLL,PITCH,YAW }; //列舉不同角度(滾動、俯仰、偏離)

static int16_t gyroADC[3];  //宣告陣列值在函式呼叫之間將其值保留在記憶體中的變數
static int16_t accADC[3];
static int16_t gyroData[3];
static float angle[2]    = {0,0};  
extern int calibratingA;  //告知程式到別的地方找尋變數的定義

#ifdef flysky     //檢查定義是否存在
  #define ROL 0
  #define PIT 1
  #define THR 2
  #define RUD 3
#else //orangerx
  #define ROL 1
  #define PIT 2
  #define THR 0
  #define RUD 3
#endif

#define AU1 4
#define AU2 5
static int16_t rcCommand[] = {0,0,0}; //宣告陣列在函式呼叫之間將其值保留在記憶體中的變數

#define GYRO     0
#define STABI    1
static int8_t flightmode;  //宣告flightmode在函式呼叫之間將其值保留在記憶體中的變數
static int8_t oldflightmode;  //宣告oldflightmode在函式呼叫之間將其值保留在記憶體中的變數

boolean armed = false; //armed的布林值為0
uint8_t armct = 0; 
int debugvalue = 0;

void setup() //定義setup()函式
{
  Serial.begin(115200); Serial.println();  //設定band在115200執行，並輸出

  delay(3000); // give it some time to stop shaking after battery plugin
  MPU6050_init();  //輸入MPU6050 ID number
  MPU6050_readId(); // must be 0x68, 104dec //讀取MPU6050 ID number 
  
  EEPROM.begin(64); //將ID分配給64個RAM，且size介於0-4096
  if (EEPROM.read(63) != 0x55) Serial.println("Need to do ACC calib"); //如果讀到的63個RAM不是0*55的格式，就會印出Need to do ACC calib
  else ACC_Read(); // eeprom is initialized   //ACC讀取成功
  if (EEPROM.read(62) != 0xAA) Serial.println("Need to check and write PID"); //如果讀到的62個RAM不是0*AA的格式，就會印出Need to check and write PID
  else PID_Read(); // eeprom is initialized   //PID讀取成功

  
  WiFi.mode(WIFI_STA); // Station mode for esp-now //連接WIFI
  #if defined webServer
    setupwebserver(); //啟動setupwebserver
    delay(500); 
  #endif


  #if defined externRC //如果externRC被定義
    init_RC(); //寫入RC函式
  #else 
    Serial.printf("This mac: %s," , WiFi.macAddress().c_str()); //輸出(MACAddress,wifi 位置)
    Serial.printf(", channel: %i\n", WIFI_CHANNEL);  //輸出WIFIchannel
    if (esp_now_init() != 0) Serial.println("*** ESP_Now init failed");  //初始化ESP
    esp_now_register_recv_cb(recv_cb);  //設定收到數據時CALLBACK的函式名稱。 通過ESP-NOW接收到數據後，將被呼叫以便進一步處理資料。
  #endif

  delay(500); 
  pinMode(LED,OUTPUT);  //這裡設定所要輸出的PIN腳模式
  digitalWrite(LED,LOW);  // 設定PIN LED腳位為低電位 = 0V
  initServo(); //初始化
}

uint32_t rxt; // receive time, used for falisave

void loop()   //定義loop()
{
  uint32_t now,mnow,diff; //unsign int now,mnow,diff
  now = millis(); // actual time  
  if (debugvalue == 5) mnow = micros(); //如果debugvalue == 5, 系統秒數指定給mnow

  #if defined webServer //如果webServer被定義，就啟動網站伺服器
    loopwebserver();
  #endif

  if (recv)  //如果有接收到
  {
    recv = false;  //recv指定成false
    #if !defined externRC  //如果externRC沒有被定義，執行buf_to_rc()
      buf_to_rc() 
    #endif

    if (debugvalue == 4) Serial.printf("%4d %4d %4d %4d \n", rcValue[0], rcValue[1], rcValue[2], rcValue[3]);  //如果debugvalue等於4，就輸出
  
    if      (rcValue[AU1] < 1300) flightmode = GYRO; //如果rcValue[AU1] < 1300，飛行模式設定成重力模式
    else                          flightmode = STABI;   //否則設定成穩定模式
    if (oldflightmode != flightmode) //若原本的飛行模式不等於新的，執行zeroGyroAccI()
    {
      zeroGyroAccI(); //歸零重力加速度
      oldflightmode = flightmode; //再把新的flightmode指定給舊的flightmode
    }

    if (armed)  //如果有配備的話
    {
      rcValue[THR]    -= THRCORR; 
      rcCommand[ROLL]  = rcValue[ROL] - MIDRUD; //rcValue[ROL] - MIDRUD指定給接收滾動命令
      rcCommand[PITCH] = rcValue[PIT] - MIDRUD;//rcValue[PIT] - MIDRUD指定給接受俯仰的命令
      rcCommand[YAW]   = rcValue[RUD] - MIDRUD; //rcValue[RUD] - MIDRUD指定給接收偏差的命令
      }  
    else
    {  
      if (rcValue[THR] < MINTHROTTLE) armct++; 
      if (armct >= 25) 
      { 
        digitalWrite(LED,HIGH); // 設定PIN LED腳位為高電位 = 5V
        armed = true;
      }
    }

    if (debugvalue == 5) Serial.printf("RC input ms: %d\n",now - rxt); //如果debugvalue等於5，則輸出
    rxt = millis();
  }

  Gyro_getADC(); //執行紀錄角速度傳感器
  
  ACC_getADC(); //執行加速度傳感器

  getEstimatedAttitude(); //執行估計姿態函數

  pid(); //執行pid控制器

  mix(); //執行mix函式

  writeServo(); //執行紀錄函式
  
  // Failsave part //故障保護
  if (now > rxt+90)
  {
    rcValue[THR] = MINTHROTTLE;
    if (debugvalue == 5) Serial.printf("RC Failsafe after %d \n",now-rxt); //如果debugvalue等於5，則執行
    rxt = now; 
  }

  // parser part //數據解析
  if (Serial.available()) //如果Serial.available()是true則執行
  {
    char ch = Serial.read(); //把Serial.read()讀出來的字元指定給ch
    // Perform ACC calibration //加速度校準
    if (ch == 10) Serial.println(); //如果ch的ASCII碼等於10
    else if (ch == 'A') //如果ch的值等於'A'
    { 
      Serial.println("Doing ACC calib"); //輸出Doing ACC calib
      calibratingA = CALSTEPS;
      while (calibratingA != 0) //當calibratingA不等於0
      {
        delay(CYCLETIME); //延遲一循環時間
        ACC_getADC(); //執行加速度傳感器
      }
      ACC_Store(); //儲存加速度值
      Serial.println("ACC calib Done"); //輸出ACC calib Done
    }
    else if (ch == 'R') //如果ch的值等於'R'
    {
      Serial.print("Act Rate :  ");                //輸出結果
      Serial.print(yawRate); Serial.print("  ");
      Serial.print(rollPitchRate); Serial.println();
      Serial.println("Act PID :");
      Serial.print(P_PID); Serial.print("  ");
      Serial.print(I_PID); Serial.print("  ");
      Serial.print(D_PID); Serial.println();
      Serial.print(P_Level_PID); Serial.print("  ");
      Serial.print(I_Level_PID); Serial.print("  ");
      Serial.print(D_Level_PID); Serial.println();
    }
    else if (ch == 'D') //如果ch的值等於'D'
    {
      Serial.println("Loading default PID"); //輸出Loading default PID
      yawRate = 6.0;
      rollPitchRate = 5.0;
      P_PID = 0.15;    // P8
      I_PID = 0.00;    // I8
      D_PID = 0.08; 
      P_Level_PID = 0.35;   // P8
      I_Level_PID = 0.00;   // I8
      D_Level_PID = 0.10;
      PID_Store(); //儲存PID值
    }
    else if (ch == 'W') //如果ch的值等於'W'
    {
      char ch = Serial.read(); //把Serial.read()讀出來的字元指定給ch
      int n = Serial.available();  //把Serial.available()的值指定給n
      if (n == 3) //如果n=3
      {
        n = readsernum();     //把readsernum()指定給n
        if      (ch == 'p') { P_PID       = float(n) * 0.01 + 0.004; Serial.print("pid P ");       Serial.print(P_PID); }   //輸出結果
        else if (ch == 'i') { I_PID       = float(n) * 0.01 + 0.004; Serial.print("pid I ");       Serial.print(I_PID); }
        else if (ch == 'd') { D_PID       = float(n) * 0.01 + 0.004; Serial.print("pid D ");       Serial.print(D_PID); }
        else if (ch == 'P') { P_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level P "); Serial.print(P_Level_PID); }
        else if (ch == 'I') { I_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level I "); Serial.print(I_Level_PID); }
        else if (ch == 'D') { D_Level_PID = float(n) * 0.01 + 0.004; Serial.print("pid Level D "); Serial.print(D_Level_PID); }
        else Serial.println("unknown command");
      }
      else if (ch == 'S') { PID_Store(); Serial.print("stored in EEPROM"); } //如果ch的值等於'S'，PID的值被儲存，輸出stored in EEPROM
      else 
      {
        Serial.println("Input format wrong");                                //輸出結果
        Serial.println("Wpxx, Wixx, Wdxx - write gyro PID, example: Wd13");
        Serial.println("WPxx, WIxx, WDxx - write level PID, example: WD21");
      }
    }
    else if (ch >= '0' && ch <='9') debugvalue = ch -'0'; //如果ch在0到9之間，則debugvalue被指定為ch -'0'
    else
    {
      Serial.println("A - acc calib");   //輸出結果形式
      Serial.println("D - write default PID"); 
      Serial.println("R - read actual PID");
      Serial.println("Wpxx, Wixx, Wdxx - write gyro PID");
      Serial.println("WPxx, WIxx, WDxx - write level PID");
      Serial.println("WS - Store PID in EEPROM");
      Serial.println("Display data:");
      Serial.println("0 - off");
      Serial.println("1 - Gyro values");
      Serial.println("2 - Acc values");
      Serial.println("3 - Angle values");
      Serial.println("4 - RC values");
      Serial.println("5 - Cycletime");
    }
  }

  if      (debugvalue == 1) Serial.printf("%4d %4d %4d \n", gyroADC[0], gyroADC[1], gyroADC[2]);  //如果debugvalue == 1輸出角速度的值
  else if (debugvalue == 2) Serial.printf("%5d %5d %5d \n", accADC[0], accADC[1], accADC[2]); //如果debugvalue == 2輸出加速度值
  else if (debugvalue == 3) Serial.printf("%3f %3f \n", angle[0], angle[1]);  //如果debugvalue == 3輸出角度值
  
  delay(CYCLETIME-1);  

  if (debugvalue == 5) //如果debugvalue == 5
  {
    diff = micros() - mnow; //設diff為micros() - mnow
    Serial.println(diff); //輸出diff的值
  }
}

int readsernum() //設定 readsernum()輸出值為int型態
{
  int num; 
  char numStr[3]; //定義 numStr字串
  numStr[0] = Serial.read(); //把 Serial.read()讀取值放到字串第一個
  numStr[1] = Serial.read();//再把Serial.read()的另一個讀取值放到字串第二個
  return atol(numStr); //傳回numStr傳遞給函數調用的C-type字符串轉換為長整數
}
