#include <Arduino.h>
#include <Wire.h>
#include "setupInit.h"
#include "adcRead.h"
#include "dmp.h"
#include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// ESP-NOW
uint8_t broadcastAddress[] = {0xE8, 0xDB, 0x84, 0x16, 0xE2, 0x68}; //接收端廣播MAC地址，數值為十六進制
//uint8_t实际上是一个char。所以输出uint8_t类型的变量实际上输出其对应的字符，而不是数值。
//the MAC address E8:DB:84:16:E2:68 is assigned to the broadcastAddress array. This means that any message sent using this address as the destination will be received by all devices within the ESP-NOW network.

// PD Monitor Data，decalre the variable
typedef struct espNowSendData {
  float current[3];
  float signalVolt[32];
  float ypr[3];
  bool mpuError;
} espNowSendData;       //espNowSendData 包含 current、signalVolt、ypr、mpuError，指定 current 的數值以 espNowSendData.current 的方式指定，以此類推

// PD Reset Notification，decalre the variable
//bool 布林值
typedef struct espNowResetMS {
  bool resetMPU;
  bool resetESP;
} espNowResetMS;

espNowSendData pdMonitorData;
espNowResetMS resetMS;      //通过将 pdMonitorData 声明为 espNowSendData 结构的实例，可以创建一个可以保存该类型数据的变量。它允许您儲存和操作结构中成员变量的值，例如访问或修改pdMonitorData的current、signalVolt、ypr和mpuError字段。

// PD Voltage，decalre the variable
//const 宣告常數
const float ri[3] = {98.5E3, 101.4E3, 100.5E3};        // Input resistor: ~100K Ohms
const float rf = 200;          // Feedback resistor: 200 Ohms
const float rg = 100;          // Ground resistor: 100 Ohms
float sample = 50;           // ADC measure sample，取樣數越高執行時間越久，累積2000筆資料約需1秒鐘
adcRead adc;                   // ADC setting and current convert
float current[3] = {1e-4, 1e-4, 1e-4};                 // Store current data of 3 PDs
float signalVolt[32];                 // 

// MPU6050 ypr angle of PD
TwoWire I2CMPU = TwoWire(1);      //the TwoWire object I2CMPU is being initialized to use the I2C bus number 1
bool dmpReady = false;         // Set true if DMP init was successful
float yprDeg[3];               // MPU6050 ypr angle of PD in degrees
float *yprPD;                  // MPU6050 ypr angle of PD in radian
bool resetMPU = false;         // Set true if user send msg to request PD's MPU6050 reset

// FreeRTOS program
TaskHandle_t handelTaskMPU;

// ESP-NOW sent callback function
void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  // Serial.print("Last Packet Send Status: ");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}       //在所提供的代码片段中，函数体被注释掉了，这意味着函数的实际功能并未实现。这种注释块通常用作未来代码的占位符或调试目的。

// ESP-NOW receive callback funciton
void onDataRecv(const uint8_t *macAddr, const uint8_t *data, int dataLen){
  // Serial.println("Receive ESP-NOW Data");
  memcpy(&resetMS, data, sizeof(resetMS));
  if(resetMS.resetMPU) resetMPU = true;
  // 若resetMS.resetMPU為真，則設定resetMPU為真
  if(resetMS.resetESP) ESP.restart();
}

// MPU6050 task function
void taskMPU(void *pvParam){
  pdMonitorData.mpuError = false;
  dmpReady = initMPU(&Wire1);
  //initialize the MPU6050 sensor and check if it is ready for use
  
  while(1){
    vTaskDelay(5 / portTICK_PERIOD_MS);   // Release the CPU0
    //Task 的 blocked 狀態通常是 task 進入了一個需要等待某事件發生的狀態，這個事件通常是執行時間到了(例如 systick interrupt)或是同步處理的回應，如果像一開始的 ATaskFunciton() 中使用 while(1){} 這樣的無限迴圈來作等待事件，會占用 CPU 運算資源，也就是 task 實際上是在 running，但又沒做任何事情，占用著資源只為了等待 event，所以比較好的作法是改用 vTaskDelay()，當 task 呼叫了 vTaskDelay()，task 會進入 blocked 狀態，就可以讓出 CPU 資源了
    //vTaskDelay()：這個函式的參數如果直接給數值，是 ticks，例如 vTaskDelay(250) 是暫停 250 個 ticks 的意思，由於每個 CPU 的一個 tick 時間長度不同，FreeRTOS 提供了 portTICK_RATE_MS 這個巨集常數，可以幫我們轉換 ticks 數為毫秒 (ms)，也就是說 vTaskDelay( 250/portTICK_RATE_MS ) 這個寫法，就是讓 task 暫停 250 毫秒(ms)的意思 (v8.2.1 改名為 portTICK_PERIOD_MS)
 
    // User send msg to request PD's MPU6050 reset
    if(resetMPU) {
      if(!initMPU(&Wire1)){
        dmpReady = false;
      }
      resetMPU = false;
    }

    // Check if DMP status is ready, or send error msg to reset the ESP32
    if(!dmpReady) {
      pdMonitorData.mpuError = true;
      continue;
    }
    //continue 強制結束這一迴圈，不繼續後續程式執行，進入下一次迭代

    yprPD = getYPR();       // Get ypr data

    if(!yprPD) continue;    // If get ypr data error, don't convert data to ESP-NOW sent struct
    memcpy(pdMonitorData.ypr, yprPD, 3* sizeof(yprPD));
    // Serial.printf("Task Gyro is at Core %u\n", xPortGetCoreID());
  }
}

// Setup function
//In summary, the setup() function initializes various settings and peripherals required by the program, such as serial communication, WiFi, ESP-NOW, I2C, and task creation for MPU6050. It is executed once at the beginning of the program before the loop() function is called repeatedly.
void setup() {
  //The setup() function is a predefined function in Arduino that is called once at the beginning of the program execution. It is typically used to initialize various settings, libraries, and peripherals before the main execution of the program begins.
  Serial.begin(115200);     //communicate with the Arduino board via the serial port and send/receive data

  initWiFi();     //initialize the WiFi settings
  initEspNow(onDataSent, onDataRecv, broadcastAddress);     //initialize the ESP-NOW protocol
  
  // I2C Setting
  Wire.begin();     // Initializes the primary I2C bus (Wire)
  Wire.setClock(400000);  // 400 kHz, Sets the clock frequency of the primary I2C bus to 400 kHz
  Wire1.begin(19, 18);      //Initializes an additional I2C bus (Wire1) with SDA pin 19 and SCL pin 18
  Wire1.setClock(400000);  // 400 kHz, Sets the clock frequency of the additional I2C bus (Wire1) to 400 kHz

  adc = adcRead(ri, rf, rg);
  // pinMode(5, INPUT);

  // Arrange MPU6050 task to core 2
  xTaskCreatePinnedToCore(taskMPU, "Task MPU", 3000, NULL, 2, &handelTaskMPU, 0);     //The task runs the taskMPU function. It specifies a stack size of 3000 bytes, no parameter (NULL), a priority level of 2, and assigns the task handle to the handelTaskMPU variable. The task is pinned to core 0 (0 argument).
}

// Loop function
void loop() {
  // if(digitalRead(5) == HIGH) return;

  adc.getCurrent(current);

  // for(int i = 0; i<=3; i++){
  //   Serial.printf("%.5f, ",current[i]);
  // }
  // Serial.println("\n");
  memcpy(pdMonitorData.current, current, sizeof(current));
  
  adc.getSignal(signalVolt);
  // Serial.println("---");
  // for(int i = 0; i < 32; i++){
  //   Serial.printf("%.5f, ",signalVolt[i]);
  // }
  // Serial.println("\n");
  memcpy(pdMonitorData.signalVolt, signalVolt, sizeof(signalVolt));

  // Send current data to BS via ESP-NOW
  esp_err_t espNowSendResult = esp_now_send(broadcastAddress, (uint8_t *) &pdMonitorData, sizeof(pdMonitorData));     //Sends the data stored in the pdMonitorData structure to the base station (BS) using the ESP-NOW protocol.
  // Serial.println(esp_err_to_name(espNowSendResult));
  // delay(1000);

  // Serial.printf("Free heap size %u bytes.\n", ESP.getFreeHeap());
  // Serial.printf("Task Loop is at Core %u\n", xPortGetCoreID());
  // Serial.printf("The number of free bytes in the heap is %u\n", xPortGetFreeHeapSize());
}