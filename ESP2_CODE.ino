/************************************************************
  Authors:
  Manuel Córdoba Ramos (University of Malaga)
  Alejandro Daniel Rodríguez Pijuan (University of Malaga)
  Date: 18/01/25

  This code, developed for ESP32, controls the operation of a car created for the subject Industrial Informatics 2024/25,
  of the B.S. Industrial Electronics Engineering at the University of Malaga.

  This ESP32 works in conjunction with:
  1. An Android application that governs its operation via Bluetooth.
  2. Another ESP32, which is in charge of other independent tasks, one of which is to control a pair of DC motors.
  3. An ESP32Cam, in charge of providing images for the Android application.

  This code is responsible for:
  1. Receiving messages from the Android application via bluetooth.
  2. Control a set of RGB LEDs, by means of 3 digital outputs driving three transistor gates.
  3. Controls the operation of a mobile radar consisting of a servomotor
     (which is responsible for the rotation) and two oppositely positioned
     HC-SR04 ultrasonic distance sensors.
  4. Send the radar information via Bluetooth to the application.
  5. Publish radar information in an MQTT topic

  All the code related to serial port communication has been commented,
  it is recommended to uncomment it in case you want to debug the code.

  CORE 0 is in charge of Bluetooth communication,
  while CORE 1 is in charge of performing the other tasks.

  If there are problems with space when uploading the code to the board,
  it is recommended to use partition scheme: No OTA (2MB APP/2MB SPIFFS).

  For more information, visit the github repository: https://github.com/Manuel-C-R/ESP32-CAR
*************************************************************/

#include "BluetoothSerial.h"
#include <ESP32Servo.h>
#include <Ticker.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "QuickMedianLib.h"


BluetoothSerial serialBT;        // Used to manage the bluetooth connection
volatile int receivedBTData[4];  // Stores the message received by bluetooth
volatile int dataIndex;          // Index used to traverse the array receivedBTData
char receivedBTChar;             // Stores the byte received by bluetooth


volatile bool msgRadarON;   // Activated when a message is received via Bluetooth indicating that the radar should be activated
volatile bool msgRadarOFF;  // Activated when a message is received via Bluetooth indicating that the radar should be deactivated
volatile bool msgLedsON;    // Activated when a message is received via Bluetooth indicating that the LEDs should be turned on
volatile bool msgLedsOFF;   // Activated when a message is received via Bluetooth indicating that the LEDs should be turned off


volatile int pwmValueRed;    // Color code value received by bluetooth for red
volatile int pwmValueGreen;  // Color code value received by bluetooth for green
volatile int pwmValueBlue;   // Color code value received by bluetooth for blue


Servo servo;                                     // Object used for servo control
volatile bool rotateRadar = false;               // It is activated when the servo must rotate.
float radarFilteredDistances[8];                 // Stores the measurements (filtered by median) made by the radar at each position in centimeters [cm].
const int servoPosition[] = { 0, 45, 90, 135 };  // Positions in sexagesimal degrees at which the servo has to move
bool poseIsIncreasing = true;                    // It is activated when the servo is advancing towards higher positions in degrees.
int servoIndex = 0;                              // Index used to iterate through the servo positions.
float distancesToFilter[5];                      // Stores all measurements made at the same position for filtering with the median.
Ticker radarTimer;                               // Timer used to count the time between radar movements.

// Wi-Fi and MQTT communication variables
const char *MQTT_BROKER_ADRESS = "mqtt.eclipseprojects.io";
const uint16_t MQTT_PORT = 1883;
const char *MQTT_CLIENT_NAME = "MCR_ESP32";

WiFiClient espClient;
PubSubClient mqttClient(espClient);


#define pinLedR 25
#define pinLedG 26
#define pinLedB 27
#define pinEcho 16
#define pinTrig 17
#define pinEcho2 13
#define pinTrig2 14
#define pinServo 19

#define radarPeriod 1  // seconds

#define frequencyLed 5000

#define channelPWM_R 1
#define channelPWM_G 2
#define channelPWM_B 3

#define wifiSSID "iPhone de Manu"
#define wifiPassword "1234ABCD"

#define topicNameMQTT "MCR/radar/data"


/*************************************** Functions *************************************/

/*
 * TimerCountCompleted() is executed when the count is over and the radar must rotate.
 */

void TimerCountCompleted(void) {
  rotateRadar = true;
}

/*
 * GenerateCommaSeparatedString() generates a text String from an array (of defined length)
 * of float data. Each piece of data is displayed to two decimal places,
 * and the data is separated by commas.
 */

String GenerateCommaSeparatedString(const float dataArray[], const int dataSize) {

  String result = "";

  for (int i = 0; i < dataSize; i++) {

    result += String(dataArray[i], 2);  // Convert each float to String with 2 decimal places

    if (i < dataSize - 1) {
      result += ",";  // Add a comma after each value, except the last one.
    }
  }
  return result;
}


/*
 * MeasureDistanceCM() measures the distance with the ultrasonic sensor,
 * receiving as parameter the trig and echo pins.
 * It returns the measured distance in centimeters, with a maximum value
 * saturated at 400cm (as indicated by the manufacturer).
 */

float MeasureDistanceCM(const int trig, const int echo) {

  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long duration = pulseIn(echo, HIGH, 30000);           // Maximum wait 30ms
  float distance = (((float)duration * 0.0343) / 2.0);  // Distance is calculated from the time of flight
                                                        // at the speed of sound

  if (distance > 400.0) {  // Saturated at 400cm
    distance = 400.0;
  }

  return distance;
}



/*
 * FilteredMedianDistance() performs 5 distance measurements at the same position
 * and performs a median filtering, returning the result of this process.
 * The QuickMedian library is used to perform the median.
 */

float FilteredMedianDistance(const int trig, const int echo) {

  for (int j = 0; j < 5; j++) {
    distancesToFilter[j] = MeasureDistanceCM(trig, echo);
  }

  return QuickMedian<float>::GetMedian(distancesToFilter, 5);
}


/*
 * ConnectWiFi() performs Wi-Fi connection to the specified network.
 */

void ConnectWiFi() {
  WiFi.begin(wifiSSID, wifiPassword);
  //Serial.print("Connecting to WiFi ");
  //Serial.print(wifiSSID);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    //Serial.print(".");
  }

  //Serial.println(" Connected!");
  //Serial.print("IP address: ");
  //Serial.println(WiFi.localIP());
}

/*
 * InitMqtt() initiates the connection to the MQTT server.
 */

void InitMqtt() {
  mqttClient.setServer(MQTT_BROKER_ADRESS, MQTT_PORT);
}

/*
 * ConnectMqtt() performs the MQTT connection.
 */

void ConnectMqtt() {

  while (!mqttClient.connected()) {

    //Serial.print("Starting MQTT connection...");

    if (mqttClient.connect(MQTT_CLIENT_NAME)) {

      //Serial.print("Client connected");

    } else {

      //Serial.print("Failed MQTT connection, rc=");
      //Serial.print(mqttClient.state());
      //Serial.println(" try again in 5 seconds");

      delay(5000);
    }
  }
}


/*
 * HandleMqtt() keeps the MQTT connection active
 */

void HandleMqtt() {

  if (!mqttClient.connected()) {
    ConnectMqtt();
  }
  mqttClient.loop();
}


/*
 * PublishData() sends the radar information via Bluetooth to the application and 
 * publishes radar information in an MQTT topic
 */

void PublishData(const float dataArray[], const int dataSize) {

  HandleMqtt();  // Ensures connection to the MQTT server

  String payload = GenerateCommaSeparatedString(dataArray, dataSize);  // Generates the text string to send

  mqttClient.publish(topicNameMQTT, (char *)payload.c_str());  // Publication in the MQTT topic

  serialBT.println(payload);  // Sending data via Bluetooth
}


/*************************************** Bluetooth function ************************************/
/*
 * This function is executed every time an event related to Bluetooth communication occurs.
 * The events considered by the function are the following:
 * 1. The Bluetooth SPP profile is initialised.
 * 2. A client is connected
 * 3. A client is disconnected
 * 4. Data arrives via Bluetooth
 */

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

void callback_function(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {

  if (event == ESP_SPP_START_EVT) {

    // The Bluetooth SPP profile is initialised.
    //Serial.println("SPP initialised");

  } else if (event == ESP_SPP_SRV_OPEN_EVT) {

    // A client is connected
    //Serial.println("Client connected");

  } else if (event == ESP_SPP_CLOSE_EVT) {

    // A client is disconnected
    //Serial.println("Client disconnected");

  } else if (event == ESP_SPP_DATA_IND_EVT) {

    // Data arrives via Bluetooth
    //Serial.println("Data received:");

    // Initialisation of variables that are going to store the received data
    dataIndex = 0;
    receivedBTData[0] = 0;
    receivedBTData[1] = 0;
    receivedBTData[2] = 0;
    receivedBTData[3] = 0;

    // All data received are read.
    // The longest possible message is encoded with 4 bytes,
    // therefore, the first 4 bytes received are stored in 4 data of type int.
    while (serialBT.available()) {

      receivedBTChar = serialBT.read();

      if (dataIndex <= 3) {
        receivedBTData[dataIndex] = int(receivedBTChar);
        dataIndex++;
      }
    }

    // Received data is displayed via serial port
    //Serial.println(receivedBTData[0]);
    //Serial.println(receivedBTData[1]);
    //Serial.println(receivedBTData[2]);
    //Serial.println(receivedBTData[3]);

    // The first byte received indicates the type of message received,
    // each message being identified by a number.


    switch (receivedBTData[0]) {
      case 1:
        // Indicates that the LEDs are to be switched on with the
        // RGB code received in the following bytes
        msgLedsON = true;
        pwmValueRed = receivedBTData[1];
        pwmValueGreen = receivedBTData[2];
        pwmValueBlue = receivedBTData[3];
        break;
      case 2:
        // Indicates that the LEDs are to be switched off
        msgLedsOFF = true;
        break;
      case 12:
        // Indicates that the radar is to be switched on
        msgRadarON = true;
        break;
      case 13:
        // Indicates that the radar is to be switched off
        msgRadarOFF = true;
        break;
      default:
        //Serial.println("Unidentified message");
        break;
    }

    // The core that has performed the Bluetooth function is displayed via the serial port.
    //Serial.print("Bluetoothe function performed by the CORE: ");
    //Serial.println(xPortGetCoreID());
  }
}


/*************************************** Setup ************************************/
void setup() {

  //Serial.begin(115200);  // Serial port initialisation

  // Initialisation of Bluetooth
  serialBT.begin("ESP32-2");
  serialBT.register_callback(callback_function);
  //Serial.println("The ESP32 is ready, you can pair it via Bluetooth.");
  //Serial.println("Name: ESP32-2");


  ConnectWiFi();  // Init Wi-Fi connection
  InitMqtt();     // Init MQTT connection


  // Ultrasonic distance sensors
  pinMode(pinTrig, OUTPUT);
  pinMode(pinEcho, INPUT_PULLUP);
  pinMode(pinTrig2, OUTPUT);
  pinMode(pinEcho2, INPUT_PULLUP);


  // LEDs configuration
  pinMode(pinLedR, OUTPUT);
  pinMode(pinLedG, OUTPUT);
  pinMode(pinLedB, OUTPUT);

  ledcSetup(channelPWM_R, frequencyLed, 8);
  ledcAttachPin(pinLedR, channelPWM_R);
  ledcWrite(channelPWM_R, 0);

  ledcSetup(channelPWM_G, frequencyLed, 8);
  ledcAttachPin(pinLedG, channelPWM_G);
  ledcWrite(channelPWM_G, 0);

  ledcSetup(channelPWM_B, frequencyLed, 8);
  ledcAttachPin(pinLedB, channelPWM_B);
  ledcWrite(channelPWM_B, 0);


  // Servo configuration
  servo.attach(pinServo, 500, 2500);
  servo.write(servoPosition[servoIndex]);


  //Serial.println("Setup performed by the CORE:");
  //Serial.println(xPortGetCoreID());
}

/*************************************** Loop ************************************/
void loop() {

  // Functions to be performed depending on the message received

  if (msgRadarON) {
    msgRadarON = false;

    radarTimer.attach(radarPeriod, TimerCountCompleted);
    poseIsIncreasing = true;
    servoIndex = 0;
    servo.write(servoPosition[servoIndex]);

    // Reset of saved values
    for (int j = 0; j < 8; j++) {
      radarFilteredDistances[j] = 0;
    }
  }


  if (msgRadarOFF) {
    msgRadarOFF = false;
    rotateRadar = false;
    servo.write(0);  // When the radar is deactivated, it returns to the initial position.
    radarTimer.detach();
  }


  if (rotateRadar) {
    rotateRadar = false;

    // Stores the filtered measurements performed
    radarFilteredDistances[servoIndex] = FilteredMedianDistance(pinTrig, pinEcho);
    radarFilteredDistances[(servoIndex + 4)] = FilteredMedianDistance(pinTrig2, pinEcho2);

    // Manages the next position to rotate
    // If the radar is at the start or end position, it publishes the measured data

    if (poseIsIncreasing) {
      servoIndex++;
    } else {
      servoIndex--;
    }

    if (servoIndex == 4) {
      servoIndex = 2;
      poseIsIncreasing = false;
      PublishData(radarFilteredDistances, 8);
    }

    if (servoIndex == -1) {
      servoIndex = 1;
      poseIsIncreasing = true;
      PublishData(radarFilteredDistances, 8);
    }

    // Moves radar to the next position
    servo.write(servoPosition[servoIndex]);
  }



  if (msgLedsON) {
    msgLedsON = false;

    ledcWrite(channelPWM_R, pwmValueRed);
    ledcWrite(channelPWM_G, pwmValueGreen);
    ledcWrite(channelPWM_B, pwmValueBlue);
  }


  if (msgLedsOFF) {
    msgLedsOFF = false;
    ledcWrite(channelPWM_R, 0);
    ledcWrite(channelPWM_G, 0);
    ledcWrite(channelPWM_B, 0);
  }
}