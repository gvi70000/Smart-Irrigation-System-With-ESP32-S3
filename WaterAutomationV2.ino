#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SparkFun_BMP581_Arduino_Library.h"

// Use it for printing messages
//#define DEBUG

// If we use the water pressure sensor to monitor pressure and leaks
//#define HAS_PRESSURE

// Control valve if a leak is detected
//#define HAS_LEAK_CONTROL

#if defined DEBUG
   #define debug_begin(x) Serial.begin(x)
   #define debug(x)       Serial.print(x)
   #define debugln(x)     Serial.println(x)
#else
   #define debug_begin(x)
   #define debug(x)
   #define debugln(x)
#endif

// We have 3 or 4 sensors?
#ifdef HAS_PRESSURE
  constexpr uint8_t SENSORS_NO      = 4;
#else
  constexpr uint8_t SENSORS_NO      = 3;
#endif

// 388 bytes for calibration, 1 byte for C(alibration), 1 byte checksum, 2 bytes start/end message marker
// 388 bytes for calibration, 1 byte for C(alibration), 1 byte checksum, 2 bytes start/end message marker
constexpr uint16_t CALIB_DATA_LEN   = 388;
// Buffer length Tx and Rx
constexpr uint16_t BUFFER_LEN	= CALIB_DATA_LEN + 4;
// Max. no. of devices
constexpr uint8_t DEV_MAX           = 10;
// Where in EEPROM we start to store calibration data
constexpr uint8_t CAL_START         = DEV_MAX + 1; // Pos 11
// Where in EEPROM we stop to store calibration data
constexpr uint16_t CAL_END          = CAL_START + CALIB_DATA_LEN; // Pos 38

// Used in serial communication with VL sensor
constexpr uint8_t START_MARKER		  = 60; // < Start marker for serial
constexpr uint8_t END_MARKER			  = 62; // > End marker for serial
constexpr uint8_t CMD_CALIBRATE		  =	67; // Send C to start calibration
constexpr uint8_t CMD_STORE_CAL		  =	83; // Send S to store calibration data we send

constexpr uint8_t RESPONSE_LENGTH	  = 76; // Get L to inform about length data
constexpr uint8_t RESPONSE_CAL_DONE	= 67; // Get C to inform about calibration data
constexpr uint8_t RESPONSE_FAIL		  = 70; // Get F to indicate something went wrong
constexpr uint8_t RESPONSE_OK			  = 83; // CMD_STORE_CAL // Get S to indicate something went OK

#ifdef HAS_PRESSURE
// 4.5v 4,7k 1.47k 1.072V
  //constexpr float ADC_FACTOR      0.000805664062f
  constexpr float VOLT_TO_BAR_F1    = 0.008308813477f; // This is 10.313f * ADC_FACTOR
  constexpr float VOLT_TO_BAR_F2    = 1.25f;
  constexpr uint8_t PRESS_PIN       = 2; // Analog read Pressure
  // R1 = 9.95, R2=3.264 => 1.112V from 4.5V
  // P = 0.0024 * ADC
#endif
// Pins used
constexpr uint8_t BTN_PIN           = 1; // Analog read buttons
constexpr uint8_t LED_PIN           = 4;
constexpr uint8_t LEVEL_PIN         = 7;
constexpr uint8_t CTRL_PIN2         = 15;

// WiFi Connect period in ms
constexpr uint16_t UART_SPEED        = 9600;
// WiFi Connect period in ms
constexpr uint16_t WiFi_Period       = 1000;
// Send data to MQTT brocker every 30 seconds
constexpr uint16_t mqttPeriod       = 10; 
// Min. water level in the well in m
constexpr float minWellLevel        = 0.5;
// Delay for MQTT functions
constexpr uint8_t delayMQTT         = 50;
// WiFi Connect period
constexpr uint16_t VL53L4CX_WAIT    = 6000;
// Debounce time in milliseconds
constexpr uint8_t DEBOUNCE_DELAY    = 100;

// ADC Values for buttons 1 to 10
constexpr uint16_t BTN_ADC_MIN[] = {430, 870, 1280, 1650, 1990, 2315, 2615, 2895, 3155, 3400};
constexpr uint16_t BTN_ADC_MAX[] = {470, 830, 1320, 1690, 2030, 2355, 2655, 2935, 3195, 3440};

enum Button { None, Btn1, Btn2, Btn3, Btn4, Btn5, Btn6, Btn7, Btn8, Btn9, Btn10 };
enum DeviceStatus { Off, On };
enum SensorNumber { Level, Temp, Press, WaterPress};

typedef struct ButtonObject {
  const char* topicState;
  const char* topicTime;
  uint8_t pin;
  uint8_t pinState;
  uint8_t timeTarget;
  volatile uint16_t timeSeconds;
};

typedef struct SensorObject {
  const char* name;
  float val;
};

#ifdef HAS_LEAK_CONTROL
  ButtonObject myBackUp[2];
#endif
ButtonObject myDevices[DEV_MAX];
SensorObject mySensors[SENSORS_NO];

// MQTT Server IP
const char* mqtt_server		= "XXX.XXX.XXX.XXX";
// MQTT Server username
const char* mqtt_username	= "MQTT_USER";
// MQTT Server password
const char* mqtt_password = "MQTT_PASSWORD";
// Client name
const char* clientID      = "Valves";
// Number of devices to be used
const char* maxDevNo      = "water/MaxDevNo";
// Calibration topic
constexpr char *topic_CalDist = "Level/Calibrate";
#ifdef HAS_LEAK_CONTROL
  // Leak alarm topic
  constexpr char *topic_LeakAlarm = "water/LeakAlarm";
  // Pressure drop value for alarm in Bar
  constexpr char *topic_LeakValue = "water/LeakValue";
#endif
// MQTT Server port
constexpr uint16_t MQTT_Port   = 1883;

volatile uint8_t interruptBMP = 0;

// Array stored in RTC memory to hold:
// 0-9 Time for each device in Min, 10 Maximum number of devices used
uint8_t DEV_IN_USE = 1;

// Stores the time when the sensors must be read
volatile uint16_t mqttTime;

// Current button
uint8_t crtButton = None;
// Timer
hw_timer_t * timer = NULL;
// Calibration Distance
uint8_t calibrateDistance = 0;
// Used for timeout
uint32_t timeCheck;
#ifdef HAS_LEAK_CONTROL
  // Store the alarm status
  uint8_t alarmLeak = 0;
  // Store the last pressure reading
  float prevPressureVal = 0.0f;
  // If pressure drops with 1 Bar the alarm is raised
  float pressureThreshold = 1.0f; 
#endif

BMP581 pressureSensor;
WiFiClient wifi_Client;
PubSubClient mqtt_Client(wifi_Client);

// Interrupt callback function for timer
void IRAM_ATTR onTimer() {
  mqttTime++;
  for (int cnt = 0; cnt < DEV_IN_USE; cnt++) {
    // If the device is On, increment the timer
    if(myDevices[cnt].pinState) {
      myDevices[cnt].timeSeconds++;
    }
  }
}

// Interrupt callback function for BMP581
void bmp581InterruptHandler() {
  interruptBMP = 1;
}

void sendToSTM(const uint8_t msgType) {
  uint16_t txLen = 4;
  // Determine the message length and fill the buffer based on message type
  if(msgType == CMD_STORE_CAL) {
    txLen = BUFFER_LEN;
  }
  uint8_t txBuffer[txLen];
  // Initialize the common header for all messages
  txBuffer[0] = START_MARKER;
  txBuffer[1] = msgType;
  uint8_t checkSum = msgType;
  if(msgType == CMD_STORE_CAL) {
    for (uint16_t cnt = CAL_START; cnt < CAL_END; cnt++) {
      uint16_t tmp = cnt - 9;
      txBuffer[tmp] = EEPROM.read(cnt);
      checkSum += txBuffer[tmp];
    }
  }
  // Add checksum and end marker
  txBuffer[txLen - 2] = checkSum;
  txBuffer[txLen - 1] = END_MARKER;
  // Send the message
  Serial1.write(txBuffer, txLen);
}

uint8_t ProcessReceivedMessage() {
  digitalWrite(LED_PIN, LOW);
  uint8_t retunValue = 0;
  // If we have 4 bytes in the buffer and the first one is <
  if((Serial1.available() > 3) && (Serial1.read() == START_MARKER)) {
    // Get the message type
    uint8_t msgType = Serial1.read();
    // Keep in mind that we already read 2 bytes
    // Depending on message type we need to wait for some more bytes
    uint16_t expectedMsgLen = 0; 
    // expectedMsgLen in below switch is ignoring the last 2 bytes we receive
    switch(msgType) {
      case RESPONSE_LENGTH: // We get 6 bytes in total
        expectedMsgLen = 2;
      break;
      case RESPONSE_CAL_DONE: // We get 392 bytes in total
        expectedMsgLen = CALIB_DATA_LEN;
      break;
      default: // RESPONSE_OK, RESPONSE_FAIL // We get 4 bytes in total
        expectedMsgLen = 0;
      break;
    }

    uint8_t receivedCheckSum = 0;
    // If we get a 4 byte message < M M >
    // The third byte we read is the checkSum
    if(!expectedMsgLen) {
      receivedCheckSum = Serial1.read();
      if(receivedCheckSum == msgType) {
        retunValue = msgType;
      }
    } else {
      // Wait to receive all bytes or return timeout
      timeCheck = millis();
      while(Serial1.available() < expectedMsgLen) {
        // If we have a timeout, exit the function
        if((millis() - timeCheck) > VL53L4CX_WAIT) {
          // Reuse timeCheck 
          timeCheck = 0;
          break;
        }
      }
      // If we do not have a timeout
      if(timeCheck) {
        // Buffer to hold received values, START_MARKER not included
        uint8_t rxBuffer[expectedMsgLen];
        // Add message type as it is needed in check sum calculation
        uint8_t checksum = msgType;
        // Add all bytes in the buffer and calculate check sum in the same loop
        for (uint16_t i = 0; i < expectedMsgLen; i++) {
          rxBuffer[i] = Serial1.read();
          // Calculate the check sum
          checksum += rxBuffer[i];
          // Serial.print(i); Serial.print(">>>"); Serial.println(rxBuffer[i]);
        }
        // By now we have received the last 2 bytes
        receivedCheckSum = Serial1.read();
        // Do not check for > end marker as we test the check sum 
        // Check message validity
        if (checksum == receivedCheckSum) {
          // What type of message we have?
          switch (msgType) {
            case RESPONSE_OK:
              retunValue = RESPONSE_OK;
            break;
            case RESPONSE_FAIL:
              // Handle failure response
              retunValue = RESPONSE_FAIL;
            break;
            case RESPONSE_LENGTH:
              // Process length data
              {
              mySensors[Level].val = (float)*(uint16_t*)&rxBuffer[0];
              retunValue = RESPONSE_LENGTH;
              }
            break;
            case RESPONSE_CAL_DONE:
            {
              // Process calibration data. From 0 to DEV_MAX we have the device data
              for(uint16_t cnt = CAL_START; cnt < CAL_END; cnt++) {
                EEPROM.write(cnt, rxBuffer[cnt - CAL_START]);
                // Serial.print(cnt);Serial.print("---");Serial.print(cnt - CAL_START);
                // Serial.print("---");Serial.println(rxBuffer[cnt - CAL_START]); 
              }
              // No need to wait after commit
              EEPROM.commit();
              retunValue = RESPONSE_CAL_DONE;
            }
            break;
          } // End switch
        } // End CheckSum check
      } // End timeout check
    } // End longer message check
  }
  digitalWrite(LED_PIN, HIGH);
  Serial1.flush();
  return retunValue;
}

// Signal errors error
void ledSignal(uint8_t err) {
  // Delay for MQTT functions
  constexpr uint8_t stateDelay = 250;
  uint8_t blinkCnt = 0;
  switch(err) {
    case 1:
      blinkCnt = 3;
      break;
    case 2:
      blinkCnt = 4;
      break;
    default:
      blinkCnt = 2;
  }
  for(uint8_t c = 0; c < blinkCnt; c++){
    digitalWrite(LED_PIN, HIGH);
    delay(stateDelay);
    digitalWrite(LED_PIN, LOW);
    delay(stateDelay);
  }
}

void writeDefaultEEPROM() {
  for(uint8_t cnt = 0; cnt < DEV_MAX; cnt++) {
    EEPROM.write(cnt, 0);
  }
  EEPROM.write(DEV_MAX, 1);
  EEPROM.commit();
  delay(delayMQTT);
}

void setup() {
  constexpr uint8_t TX1_PIN = 17;
  constexpr uint8_t RX1_PIN = 18;
  analogSetAttenuation(ADC_2_5db);
  analogReadResolution(12);
  Serial1.begin(UART_SPEED, SERIAL_8N1, RX1_PIN, TX1_PIN);
  #ifdef DEBUG
    debug_begin(UART_SPEED);
  #endif
  // Serial.begin(9600);
  EEPROM.begin(CAL_END);
  // writeDefaultEEPROM();
  pinMode(LED_PIN, OUTPUT);
  pinMode(LEVEL_PIN, OUTPUT);
  pinMode(CTRL_PIN2, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // Level pin not used. The VL sensor is powerd all the time
  digitalWrite(LEVEL_PIN, LOW);
  digitalWrite(CTRL_PIN2, LOW);
  // Initialize valves and sensors
  initObjects();
  // Load number of devices from EEPROM
  DEV_IN_USE = EEPROM.read(DEV_MAX);
  Serial1.flush();
  // Wait for STM board to wakeUp. Getting here takes 115ms
  while((millis() - timeCheck) < VL53L4CX_WAIT) {
    if(ProcessReceivedMessage()) break;
    delay(5);
  }
  // Now send the calibration data
  sendToSTM(CMD_STORE_CAL);

  // // Setup BMP581 temperature and pressure sensor
  bmpInit();
  // Setup and connect to WiFi
  startWiFi();
  // Set MQTT server IP and port
  mqtt_Client.setServer(mqtt_server, MQTT_Port);
  // Attach the MQTT callback function
  mqtt_Client.setCallback(mqttCallback);
  // Connect to MQTT broker
  ConnectToMQTT();
  if (!mqtt_Client.connected()) {
    mqtt_reconnect();
  }
  // Timer initialization
  // Set timer frequency to 1Mhz (default timer frequency is 80MHz)
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
  // Set alarm to call onTimer function every second (value in microseconds).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
  ledSignal(0);
  crtButton = None;
}

void loop() {
  // // Used to check ADC values
  // if (mqttTime >= mqttPeriod) {
  //   uint16_t adc_val = analogRead(BTN_PIN);
  //   debugln(adc_val);
  //   mqttTime = 0;
  // }
  yield();
  ProcessReceivedMessage();
  mqtt_Client.loop();
  // Is any button pressed?
  getButtons();
  if (crtButton) {
    uint8_t tmp = crtButton - 1;
    crtButton = None;
    // Accept button press events only for devices that are used
    if(tmp < DEV_IN_USE) {
      // Is the device already turned ON?
      if(myDevices[tmp].pinState) {
        // If a button is pressed and device is ON then turn off the device, set the time and publish MQTT
        deviceOnOff(tmp, Off, 1);
      } else {
        // If a button is pressed turn on the device, set the time and publish MQTT
        deviceOnOff(tmp, On, 1);
      }
    }
  }

  // Check if any device needs to be turned Off
  for (uint8_t cnt = 0; cnt < DEV_IN_USE; cnt++) {
    // Target time is in Minutes, START_MARKER has a value of 60, so...
    // If timeSeconds is not 0 and timeSeconds is equal or larger than target time in seconds
    if ((myDevices[cnt].timeSeconds) && (myDevices[cnt].timeSeconds >= (myDevices[cnt].timeTarget * START_MARKER))) {
      deviceOnOff(cnt, Off, 1);
    }
  }
  // Read temperature, pressure and water level
  if (mqttTime >= mqttPeriod) {
    #ifdef HAS_PRESSURE
      // Get data from Analog pressure sensor
      getWaterPressure();
    #endif
    // Get data from onboard BMP581
    getTempPress();
    // Check if we have new data from VL53L4CX
    ProcessReceivedMessage();
    // Publish the topics
    publishSensors();
    // If water level is too low turn Off all devices, set the time to 0 and publish MQTT
    if (mySensors[Level].val <= minWellLevel) {
      for (int cnt = 0; cnt < DEV_IN_USE; cnt++) {
        deviceOnOff(cnt, Off, 1);
      }
    }
    mqttTime = 0;
  }
  // Check if calibration was requested
  if(calibrateDistance) {
    // Send request to STM board
    sendToSTM(CMD_CALIBRATE);
    // Wait for reply
    delay(VL53L4CX_WAIT);
    // If calibration is OK
    if(ProcessReceivedMessage() == RESPONSE_CAL_DONE){
      publishToMqtt(topic_CalDist, "1");
    } else {
      publishToMqtt(topic_CalDist, "0");
    }
    calibrateDistance = 0;
  }
}

#ifdef HAS_PRESSURE
  // Voltage divider 10k/3.2k
  // V_In max = 4.5V, V_Div = 1.09V
  // P = 0 Bar V = 0.121V
  // P = 5 Bar V = 0.606V
  // P = 10 Bar V = 1.09
  // P = 10.313*V - 1.25
  void getWaterPressure() {
    // Transform ADC reading in volts
    //float voltage = (float)adc1_get_raw(ADC1_CHANNEL_1) * ADC_FACTOR;
    // Transform voltage in Bar
    mySensors[WaterPress].val = (float)analogRead(PRESS_PIN) * VOLT_TO_BAR_F1 + VOLT_TO_BAR_F2;
    #ifdef HAS_LEAK_CONTROL
      if((mySensors[WaterPress].val - prevPressureVal) > pressureThreshold) {
        digitalWrite(LEVEL_PIN, HIGH);
        alarmLeak = 1;
        myBackUp[0] = myDevices[0];
        myBackUp[1] = myDevices[1];
        // Cut off water supply. Valve 0 NC and 1 NO    
        myDevices[0].pinState = 0;
        publishToMqtt(myDevices[0].topicState, String(myDevices[0].pinState));
        digitalWrite(myDevices[0].pin, myDevices[0].pinState);
        myDevices[1].pinState = 1;
        publishToMqtt(myDevices[1].topicState, String(myDevices[1].pinState));
        digitalWrite(myDevices[1].pin, myDevices[1].pinState);
        publishToMqtt(topic_LeakAlarm, "1");
      } else {
        // If the alarm is no longer valid
        if(alarmLeak) {
          digitalWrite(LEVEL_PIN, LOW);
          alarmLeak = 0;
          myDevices[0] = myBackUp[0];
          publishToMqtt(myDevices[0].topicState, String(myDevices[0].pinState));
          digitalWrite(myDevices[0].pin, myDevices[0].pinState);
          myDevices[1] = myBackUp[1];
          publishToMqtt(myDevices[1].topicState, String(myDevices[1].pinState));
          digitalWrite(myDevices[1].pin, myDevices[1].pinState);
          publishToMqtt(topic_LeakAlarm, "0");
        }
      }
      prevPressureVal = mySensors[WaterPress].val;
    #endif
  }
#endif

// Function to debounce the button
void getButtons() {
  uint16_t adc_val = analogRead(BTN_PIN);
  crtButton = None;
  // Reset current button if adc_val is greater than a threshold
  if (adc_val > BTN_ADC_MAX[9]) {
    return;
  } else {
    delay(DEBOUNCE_DELAY);
    adc_val = analogRead(BTN_PIN);
    for (uint8_t cnt = 0; cnt < DEV_MAX; cnt++) {
      if((adc_val > BTN_ADC_MIN[cnt]) && (adc_val < BTN_ADC_MAX[cnt])) {
        crtButton = (cnt + 1);
        return;
      }
    }
  }
}

//Turns On or Off a device and updates MQTT topic
void deviceOnOff(uint8_t idx, uint8_t State, uint8_t sendMessage) {
  myDevices[idx].pinState = State;
  // If the state is On
  if (State) {
    digitalWrite(myDevices[idx].pin, HIGH);
  } else {
    digitalWrite(myDevices[idx].pin, LOW);
  }
  myDevices[idx].timeSeconds = 0; // resetTimer
  if (sendMessage) {
    publishToMqtt(myDevices[idx].topicState, String(myDevices[idx].pinState));
  }
}

//Publish value to MQTT topic
void publishToMqtt(const char* mqtt_topic, String mqtt_value) {
  if (!mqtt_Client.publish(mqtt_topic, mqtt_value.c_str())) {
    mqtt_Client.connect(clientID, mqtt_username, mqtt_password);
    delay(delayMQTT);
    mqtt_Client.publish(mqtt_topic, mqtt_value.c_str());
  }
  delay(delayMQTT);
}

void publishTopics() {
  // Publis and subscribe for each valve
  for (uint8_t cnt = 0; cnt < DEV_IN_USE; cnt++) {
    publishToMqtt(myDevices[cnt].topicState, String(myDevices[cnt].pinState));
    delay(delayMQTT);
    mqtt_Client.subscribe(myDevices[cnt].topicState);
    delay(delayMQTT);
    publishToMqtt(myDevices[cnt].topicTime, String(myDevices[cnt].timeTarget));
    delay(delayMQTT);
    mqtt_Client.subscribe(myDevices[cnt].topicTime);
    delay(delayMQTT);
  }

  #ifdef HAS_LEAK_CONTROL
    publishToMqtt(topic_LeakAlarm, "0");
    delay(delayMQTT);
    mqtt_Client.subscribe(topic_LeakAlarm);
    delay(delayMQTT);
    publishToMqtt(topic_LeakValue, String(pressureThreshold));
    delay(delayMQTT);
    mqtt_Client.subscribe(topic_LeakValue);
    delay(delayMQTT);
  #endif

  // Publish/subscribe the calibration Distance, should be 0
  publishToMqtt(topic_CalDist, String(calibrateDistance));
  delay(delayMQTT);
  mqtt_Client.subscribe(topic_CalDist);
  delay(delayMQTT);
}

void publishSensors() {
  for (uint8_t cnt = 0; cnt < SENSORS_NO; cnt++) {
    publishToMqtt(mySensors[cnt].name, String(mySensors[cnt].val));
  }
}

// Connect to MQTT brocker
void ConnectToMQTT() {
  if (mqtt_Client.connect(clientID, mqtt_username, mqtt_password)) {
    publishSensors();
    publishTopics();
    publishToMqtt(maxDevNo, String(DEV_IN_USE));
    delay(delayMQTT);
    mqtt_Client.subscribe(maxDevNo);
  } else {
    ledSignal(2);
  }
}

//Reconnect to MQTT brocker
void mqtt_reconnect() {
  uint8_t retry = 10;
  while (!mqtt_Client.connected() && retry) {
    ConnectToMQTT();
    retry--;
  }
}

void setNewDevNo() {
  EEPROM.write(DEV_MAX, DEV_IN_USE);
  EEPROM.commit();
  for(uint8_t cnt = DEV_IN_USE; cnt < DEV_MAX; cnt++) {
    deviceOnOff(cnt, Off, 0);
    mqtt_Client.unsubscribe(myDevices[cnt].topicState);
    mqtt_Client.unsubscribe(myDevices[cnt].topicTime);
    delay(delayMQTT);
  }
  // To update the devices
  publishTopics();
}
// MQTT callback function
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  debug("Message arrived [");
  debug(topic);
  debugln("]");
  
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }
  // If we get calibration value of 1
  if (strcmp(topic, topic_CalDist) == 0) {
    calibrateDistance = messageTemp.toInt();
  }
  #ifdef HAS_LEAK_CONTROL
  if (strcmp(topic, topic_LeakValue) == 0) {
    pressureThreshold = messageTemp.toFloat();
  }
  #endif
  // If we change the number of devices
  if (strcmp(topic, maxDevNo) == 0) {
    DEV_IN_USE = messageTemp.toInt();
    if(DEV_IN_USE > DEV_MAX) DEV_IN_USE = DEV_MAX;
    if(DEV_IN_USE < 1) DEV_IN_USE = 1;
    setNewDevNo();
  } else {
    for (uint8_t cnt = 0; cnt < DEV_IN_USE; cnt++) {
      // If the device state is changed
      if (strcmp(topic, myDevices[cnt].topicState) == 0) {
        myDevices[cnt].pinState = messageTemp.toInt();
        deviceOnOff(cnt, myDevices[cnt].pinState, 0);
      }
      // If the device time is changed
      if (strcmp(topic, myDevices[cnt].topicTime) == 0) {
        myDevices[cnt].timeTarget = messageTemp.toInt();
        EEPROM.write(cnt, myDevices[cnt].timeTarget);
        EEPROM.commit();
        delay(delayMQTT);
      }
    }
  }
}

// Wi-Fi connection SetUp - Wasted a day with connection issues because of buggy library
void startWiFi() {
  // WiFi network name
  constexpr char *ssid = "WIFI_NETWORK_NAME";
  // WiFi network password
  constexpr char *password = ""WIFI_NETWORK_PASSWORD";
  WiFi.mode(WIFI_STA);
  WiFi.hostname(clientID);
  // debugln(WiFi.macAddress());
  WiFi.useStaticBuffers(true);
  // WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK);   //WIFI_AUTH_WEP
  WiFi.disconnect();
  delay(WiFi_Period);
  WiFi.begin(ssid, password);
  // Wait to connect to WiFi 
  // WiFi.waitForConnectResult(1000);
  while (WiFi.waitForConnectResult(WiFi_Period) != WL_CONNECTED) {
    ledSignal(1);
  }
}

// Initialize devices, sensors
void initObjects() {
  constexpr char* deviceTopics[] = {
    "water/State1", "water/State2", "water/State3", "water/State4", "water/State5",
    "water/State6", "water/State7", "water/State8", "water/State9", "water/State10",
    "water/Time1", "water/Time2", "water/Time3", "water/Time4", "water/Time5",
    "water/Time6", "water/Time7", "water/Time8", "water/Time9", "water/Time10"
  };

  constexpr uint8_t pins[] = {10, 11, 12, 13, 14, 21, 47, 48, 35, 36};

  for (uint8_t cnt = 0; cnt < DEV_MAX; cnt++) {
    myDevices[cnt].topicState = deviceTopics[cnt];
    myDevices[cnt].topicTime = deviceTopics[cnt + 10];
    myDevices[cnt].timeTarget = EEPROM.read(cnt);
    myDevices[cnt].timeSeconds = 0;
    myDevices[cnt].pin = pins[cnt];
    pinMode(myDevices[cnt].pin, OUTPUT);
    digitalWrite(myDevices[cnt].pin, LOW);
  }

  constexpr char* sensorNames[] = {
    "water/Level", "water/AirTemperature", "water/AirPressure", "water/WaterPressure"
  };

  for (uint8_t cnt = 0; cnt < SENSORS_NO; cnt++) {
    mySensors[cnt].name = sensorNames[cnt];
    mySensors[cnt].val = 0.0f;
  }
}

// Initialize BMP581 sensor
void bmpInit() {
  constexpr uint32_t oorCenter  = 101325;
  constexpr uint8_t oorWindow   = 255;
  constexpr uint8_t SDA_Pin     = 8;
  constexpr uint8_t SCL_Pin     = 9;
  constexpr uint8_t INT_PIN     = 16;
  Wire.begin(SDA_Pin, SCL_Pin);
  while (pressureSensor.beginI2C(BMP581_I2C_ADDRESS_SECONDARY) != BMP5_OK) {
    debugln("Error: BMP581 not connected, check wiring and I2C address!");
    delay(WiFi_Period);
  }

  int8_t err = BMP5_OK;
  err = pressureSensor.setODRFrequency(BMP5_ODR_01_HZ);
  if (err != BMP5_OK) {
    debugln("ODR setting failed! Error code: ");
    debugln(err);
  }

  bmp5_oor_press_configuration oorConfig {
    .oor_thr_p     = oorCenter,
    .oor_range_p   = oorWindow,
    .cnt_lim       = BMP5_OOR_COUNT_LIMIT_3,
    .oor_sel_iir_p = BMP5_DISABLE
  };
  pressureSensor.setOORConfig(&oorConfig);

  BMP581_InterruptConfig interruptConfig = {
    .enable   = BMP5_INTR_ENABLE,
    .drive    = BMP5_INTR_PUSH_PULL,
    .polarity = BMP5_ACTIVE_HIGH,
    .mode     = BMP5_PULSED,
    .sources  = {
      .drdy_en = BMP5_ENABLE,
      .fifo_full_en = BMP5_DISABLE,
      .fifo_thres_en = BMP5_DISABLE,
      .oor_press_en = BMP5_DISABLE
    }
  };
  err = pressureSensor.setInterruptConfig(&interruptConfig);

  if (err != BMP5_OK) {
    debugln("Interrupt settings failed! Error code: ");
    debugln(err);
  }

  attachInterrupt(INT_PIN, bmp581InterruptHandler, RISING);
}

// Get temperature and pressure  from BMP581
void getTempPress() {
  if (interruptBMP) {
    interruptBMP = 0;
    int8_t err = BMP5_OK;
    uint8_t interruptStatus = 0;
    err = pressureSensor.getInterruptStatus(&interruptStatus);
    
    if (err != BMP5_OK) {
      debugln("Get interrupt status failed! Error code: ");
      debugln(err);
      return;
    }

    if (interruptStatus & BMP5_INT_ASSERTED_DRDY) {
      bmp5_sensor_data data = {0, 0};
      err = pressureSensor.getSensorData(&data);

      if (err == BMP5_OK) {
        mySensors[Temp].val = data.temperature;
        mySensors[Press].val = data.pressure;
      } else {
        debug("Error getting data from sensor! Error code: ");
        debugln(err);
      }
    }

    if (interruptStatus & BMP5_INT_ASSERTED_PRESSURE_OOR) {
      debugln("Out of range condition triggered!");
    }

    if (!(interruptStatus & (BMP5_INT_ASSERTED_PRESSURE_OOR | BMP5_INT_ASSERTED_DRDY))) {
      debugln("Wrong interrupt condition!");
    }
  }
}
