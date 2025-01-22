/************************************************************
  Authors:
  Manuel Córdoba Ramos (University of Malaga)
  Alejandro Daniel Rodríguez Pijuan (University of Malaga)
  Date: 18/01/25

  This code, developed for ESP32, controls the operation of a car created for the subject Industrial Informatics 2024/25,
  of the B.S. Industrial Electronics Engineering at the University of Malaga.

  This ESP32 works in conjunction with:
  1. An Android application that governs its operation via Bluetooth.
  2. Another ESP32, which is in charge of other independent tasks, one of which is to communicate via MQTT.
  3. An ESP32Cam, in charge of providing images for the Android application.

  This code is responsible for:
  1. Receiving messages from the Android application via bluetooth.
  2. Control two pairs of DC motors.
  3. Control a buzzer.
  4. To control LED lights used as driving indicators.
  5. Operate an LCD screen, communicated with the ESP via I2C communication protocol,
  used to display real-time information, or text strings received from the Android application.

  While the car is in motion, it should receive a message from the app via bluetooth,
  as a WatchDog, in order to avoid accidents due to sudden falls of the application.
  If this message is not received in time, the car will stop.

  All the code related to serial port communication has been commented,
  it is recommended to uncomment it in case you want to debug the code.

  CORE 0 is in charge of Bluetooth communication,
  while CORE 1 is in charge of performing the other tasks.

  If there are problems with space when uploading the code to the board,
  it is recommended to use partition scheme: No OTA (2MB APP/2MB SPIFFS).

  For more information, visit the github repository: https://github.com/Manuel-C-R/ESP32-CAR
*************************************************************/

#include "BluetoothSerial.h"
#include <LiquidCrystal_I2C.h>
#include <Ticker.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD screen object (screen address, x size, y size)
BluetoothSerial serialBT;            // Used to manage the bluetooth connection
volatile int receivedBTData[4];      // Stores the message received by bluetooth
volatile int dataIndex;              // Index used to traverse the array receivedBTData
char receivedBTChar;                 // Stores the byte received by bluetooth
String receivedBTText;               // Stores the text received by bluetooth


Ticker timerWatchDog;  // Timer used as a watchdog to prevent the car from losing control

volatile bool msgGoForward;           // Activated when a message is received via Bluetooth indicating that the car should move forward
volatile bool msgGoBack;              // Activated when a message is received via Bluetooth indicating that the car should move back
volatile bool msgGoRight;             // Activated when a message is received via Bluetooth indicating that the car should move right
volatile bool msgGoLeft;              // Activated when a message is received via Bluetooth indicating that the car should move left
volatile bool msgStopCar;             // Activated when a message is received via Bluetooth indicating that the car must stop
volatile bool msgIndicatorsON;        // Activated when a message is received via Bluetooth indicating that the indicators should be activated
volatile bool msgIndicatorsOFF;       // Activated when a message is received via Bluetooth indicating that the indicators should be deactivated
volatile bool msgBuzzerON;            // Activated when a message is received via Bluetooth indicating that the buzzer should be activated
volatile bool msgBuzzerOFF;           // Activated when a message is received via Bluetooth indicating that the buzzer should be deactivated
volatile bool msgGetResponseFromApp;  // Activated when the watchdog reset message is received via bluetooth
volatile bool msgCopyOfESP2;          // Activated when a copy of the ESP2 bluetooth message is received for display on the LCD screen
volatile bool msgNewTextLcd;          // Activated when a Bluetooth message indicates a new text for LCD has arrived
volatile bool msgLcdON;               // Activated when a message is received via Bluetooth indicating that the LCD should be activated
volatile bool msgLcdOFF;              // Activated when a message is received via Bluetooth indicating that the LCD should be deactivated

volatile int pwmValue;               // PWM value received by bluetooth
volatile int msgESP2ID;              // Stores the identifier of the copy of the message, identical to the one from the ESP32, received via Bluetooth
volatile int msgESP2ColorRed;        // Stores the value of Red assigned to LEDs
volatile int msgESP2ColorGreen;      // Stores the value of Green assigned to LEDs
volatile int msgESP2ColorBlue;       // Stores the value of Blue assigned to LEDs
volatile bool lcdModeStatus = true;  // Status mode for LCD is active
volatile bool lcdModeCustomText;     // Custom Text mode for LCD is active


volatile bool indicatorsON;  // Stores the status of indicators
bool carIsMoving;            // Stores the status of car
volatile bool lcdIsON;       // Stores the status of LCD


#define pinMotorA_PWM 18
#define pinMotorA_G1 17
#define pinMotorA_G2 16
#define pinMotorB_PWM 25
#define pinMotorB_G1 26
#define pinMotorB_G2 27
#define pinIndicatorFrontRight 32
#define pinIndicatorFrontLeft 33
#define pinIndicatorBackRight 14
#define pinIndicatorBackLeft 13
#define pinBuzzer 19

#define channelPWM_MotorA 0
#define channelPWM_MotorB 1
#define channelPWM_Buzzer 2

#define frequencyPWMMotors 5000
#define frequencyPWMBuzzer 2000
#define resolutionPWMMotors 8

#define watchDogPeriod 0.3 // seconds


/*************************************** Functions *************************************/

/*
 * PrintRow() writes to the specified row of the LCD display the text passed as a parameter.
 * The text will appear centred in the row.
 */

void PrintRow(const int row, const String text) {

  String textRow = text;

  //Remove the blanks at the end
  while (textRow.endsWith(" ")) {
    textRow.remove(textRow.length() - 1);
  }

  // Calculates the starting position
  const int startingPosition = (16 - textRow.length()) / 2;

  // Show text centred on the row
  lcd.setCursor(startingPosition, row);
  lcd.print(textRow);
}


/*
 * SetNewText() writes to the LCD display the text received by Bluetooth.
 */

void SetNewText(void) {

  lcdIsON = true;
  // The first character of the text received from the app is a ‘*’,
  // followed by the desired text, so the first element of the String is removed.
  receivedBTText = receivedBTText.substring(1);

  // It is separated into the two rows
  String firstRow = receivedBTText.substring(0, 16);
  String secondRow = receivedBTText.substring(16, 32);

  //Serial.println(firstRow);
  //Serial.println(secondRow);

  lcd.clear();
  PrintRow(0, firstRow);
  PrintRow(1, secondRow);
}


/*
 * SetLcdStatusText() writes to the LCD display the text the
 * text passed as a parameter for each row.
 * Only display text if status mode is active.
 */

void SetLcdStatusText(const String textFirstRow, const String textSecondRow) {

  if (lcdIsON && lcdModeStatus) {
    lcd.clear();
    PrintRow(0, textFirstRow);
    PrintRow(1, textSecondRow);
  }
}


/*
 * AllIndicatorsOFF() turn OFF all the indicators
 */

void AllIndicatorsOFF(void) {
  digitalWrite(pinIndicatorFrontRight, LOW);
  digitalWrite(pinIndicatorFrontLeft, LOW);
  digitalWrite(pinIndicatorBackRight, LOW);
  digitalWrite(pinIndicatorBackLeft, LOW);
}


/*
 * SetGoForwardIndicators() sets the necessary indicators to go forward
 */

void SetGoForwardIndicators(void) {
  digitalWrite(pinIndicatorFrontRight, HIGH);
  digitalWrite(pinIndicatorFrontLeft, HIGH);
  digitalWrite(pinIndicatorBackRight, LOW);
  digitalWrite(pinIndicatorBackLeft, LOW);
}


/*
 * SetGoBackIndicators() sets the necessary indicators to go back
 */

void SetGoBackIndicators(void) {
  digitalWrite(pinIndicatorFrontRight, LOW);
  digitalWrite(pinIndicatorFrontLeft, LOW);
  digitalWrite(pinIndicatorBackRight, HIGH);
  digitalWrite(pinIndicatorBackLeft, HIGH);
}

/*
 * SetGoLeftIndicators() sets the necessary indicators to go left
 */

void SetGoLeftIndicators(void) {
  digitalWrite(pinIndicatorFrontRight, LOW);
  digitalWrite(pinIndicatorFrontLeft, HIGH);
  digitalWrite(pinIndicatorBackRight, LOW);
  digitalWrite(pinIndicatorBackLeft, HIGH);
}

/*
 * SetGoRightIndicators() sets the necessary indicators to go right
 */

void SetGoRightIndicators(void) {
  digitalWrite(pinIndicatorFrontRight, HIGH);
  digitalWrite(pinIndicatorFrontLeft, LOW);
  digitalWrite(pinIndicatorBackRight, HIGH);
  digitalWrite(pinIndicatorBackLeft, LOW);
}


/*
 * GoForward() makes the car to move forward, with all associated actions.
 * 1. Motors activation
 * 2. Displays information on the LCD display, and via the serial port (in case you want to debug the code)
 * 3. Activate the watchdog timer
 * 4. Activate the indicators if necessary
 */

void GoForward(void) {

  carIsMoving = true;

  // Motors activation
  digitalWrite(pinMotorA_G1, HIGH);
  digitalWrite(pinMotorA_G2, LOW);
  ledcWrite(channelPWM_MotorA, pwmValue);

  digitalWrite(pinMotorB_G1, HIGH);
  digitalWrite(pinMotorB_G2, LOW);
  ledcWrite(channelPWM_MotorB, pwmValue);

  // Displays information on the LCD display,
  // and via the serial port (in case you want to debug the code)
  //Serial.println("Going Forward");
  SetLcdStatusText("GOING", "FORWARD");

  // Activate the watchdog timer
  timerWatchDog.attach(watchDogPeriod, NoResponseFromApp);

  // Activate the indicators if necessary
  if (indicatorsON) {
    SetGoForwardIndicators();

  } else {
    AllIndicatorsOFF();
  }
}


/*
 * GoBack() makes the car to move back, with all associated actions.
 * 1. Motors activation
 * 2. Displays information on the LCD display, and via the serial port (in case you want to debug the code)
 * 3. Activate the watchdog timer
 * 4. Activate the indicators if necessary
 */

void GoBack(void) {

  carIsMoving = true;

  // Motors activation
  digitalWrite(pinMotorA_G1, LOW);
  digitalWrite(pinMotorA_G2, HIGH);
  ledcWrite(channelPWM_MotorA, pwmValue);

  digitalWrite(pinMotorB_G1, LOW);
  digitalWrite(pinMotorB_G2, HIGH);
  ledcWrite(channelPWM_MotorB, pwmValue);

  // Displays information on the LCD display,
  // and via the serial port (in case you want to debug the code)
  //Serial.println("Going Back");
  SetLcdStatusText("GOING", "BACK");

  // Activate the watchdog timer
  timerWatchDog.attach(watchDogPeriod, NoResponseFromApp);

  // Activate the indicators if necessary
  if (indicatorsON) {
    SetGoBackIndicators();

  } else {
    AllIndicatorsOFF();
  }
}

/*
 * GoLeft() makes the car to move left, with all associated actions.
 * 1. Motors activation
 * 2. Displays information on the LCD display, and via the serial port (in case you want to debug the code)
 * 3. Activate the watchdog timer
 * 4. Activate the indicators if necessary
 */

void GoLeft(void) {

  carIsMoving = true;

  // Motors activation
  digitalWrite(pinMotorA_G1, HIGH);
  digitalWrite(pinMotorA_G2, LOW);
  ledcWrite(channelPWM_MotorA, pwmValue);

  digitalWrite(pinMotorB_G1, LOW);
  digitalWrite(pinMotorB_G2, HIGH);
  ledcWrite(channelPWM_MotorB, pwmValue);

  // Displays information on the LCD display,
  // and via the serial port (in case you want to debug the code)
  //Serial.println("Going Left");
  SetLcdStatusText("GOING", "LEFT");

  // Activate the watchdog timer
  timerWatchDog.attach(watchDogPeriod, NoResponseFromApp);

  // Activate the indicators if necessary
  if (indicatorsON) {
    SetGoLeftIndicators();

  } else {
    AllIndicatorsOFF();
  }
}


/*
 * GoRight() makes the car to move right, with all associated actions.
 * 1. Motors activation
 * 2. Displays information on the LCD display, and via the serial port (in case you want to debug the code)
 * 3. Activate the watchdog timer
 * 4. Activate the indicators if necessary
 */

void GoRight(void) {

  carIsMoving = true;

  // Motors activation
  digitalWrite(pinMotorA_G1, LOW);
  digitalWrite(pinMotorA_G2, HIGH);
  ledcWrite(channelPWM_MotorA, pwmValue);

  digitalWrite(pinMotorB_G1, HIGH);
  digitalWrite(pinMotorB_G2, LOW);
  ledcWrite(channelPWM_MotorB, pwmValue);

  // Displays information on the LCD display,
  // and via the serial port (in case you want to debug the code)
  //Serial.println("Going Right");
  SetLcdStatusText("GOING", "RIGHT");

  // Activate the watchdog timer
  timerWatchDog.attach(watchDogPeriod, NoResponseFromApp);

  // Activate the indicators if necessary
  if (indicatorsON) {
    SetGoRightIndicators();

  } else {
    AllIndicatorsOFF();
  }
}


/*
 * StopCar() makes the car stop, with all associated actions.
 * 1. Motors deactivation
 * 2. Displays information on the LCD display, and via the serial port (in case you want to debug the code)
 * 3. Deactivate the indicators
 */

void StopCar(void) {

  carIsMoving = false;

  // Motors deactivation
  digitalWrite(pinMotorA_G1, LOW);
  digitalWrite(pinMotorA_G2, LOW);
  ledcWrite(channelPWM_MotorA, 0);

  digitalWrite(pinMotorB_G1, LOW);
  digitalWrite(pinMotorB_G2, LOW);
  ledcWrite(channelPWM_MotorB, 0);

  // Displays information on the LCD display,
  // and via the serial port (in case you want to debug the code)
  //Serial.println("Car Stopped");
  SetLcdStatusText("CAR", "STOPPED");

  // Deactivate the watchdog timer
  timerWatchDog.detach();

  // Deactivate the indicators
  AllIndicatorsOFF();
}

/*
 * NoResponseFromApp() is the function executed when no response is received via Bluetooth
 * from the app that governs the control of the car. The purpose of this function 
 * is that in the event of a sudden crash of the application, the car does not remain moving 
 * indefinitely, thus avoiding an accident.
 */

void NoResponseFromApp(void) {

  carIsMoving = false;

  // Motors deactivation
  digitalWrite(pinMotorA_G1, LOW);
  digitalWrite(pinMotorA_G2, LOW);
  ledcWrite(channelPWM_MotorA, 0);

  digitalWrite(pinMotorB_G1, LOW);
  digitalWrite(pinMotorB_G2, LOW);
  ledcWrite(channelPWM_MotorB, 0);

  // Displays information on the LCD display,
  // and via the serial port (in case you want to debug the code)
  //Serial.println("Car stopped due to non-response");
  SetLcdStatusText("CAR STOPPED", "NO RESPONSE");

  // Deactivate the watchdog timer
  timerWatchDog.detach();

  // Deactivate the indicators
  AllIndicatorsOFF();
}


/*
 * ManageCopyOfESP2msg() is used to identify the message received by the app,
 * which is a copy of the one sent to ESP2, to display on the LCD screen the action taken.
 */

void ManageCopyOfESP2msg(const int numMsg) {

  // A switch is made to classify the message
  switch (numMsg) {
    case 1:
      SetLcdStatusText("LEDs ON", "R" + String(msgESP2ColorRed) + " G" + String(msgESP2ColorGreen) + " B" + String(msgESP2ColorBlue));
      break;
    case 2:
      SetLcdStatusText("LEDs", "OFF");
      break;
    case 12:
      SetLcdStatusText("RADAR", "ON");
      break;
    case 13:
      SetLcdStatusText("RADAR", "OFF");
      break;
    default:
      break;
  }
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
    receivedBTText = "";

    // All data received are read.
    // The longest possible message is encoded with 4 bytes (with the exception of a text message),
    // therefore, the first 4 bytes received are stored in 4 data of type int.
    while (serialBT.available()) {

      receivedBTChar = serialBT.read();

      if (dataIndex <= 3) {
        receivedBTData[dataIndex] = int(receivedBTChar);
        dataIndex++;
      }

      // Simultaneously, all the received bytes are stored in a String,
      // in case the received data form a text string.
      receivedBTText += receivedBTChar;
    }

    // Received data is displayed via serial port
    //Serial.println(receivedBTData[0]);
    //Serial.println(receivedBTData[1]);
    //Serial.println(receivedBTData[2]);
    //Serial.println(receivedBTData[3]);

    // The first byte received indicates the type of message received,
    // each message being identified by a number.
    // The most special is 42 (* in ASCII code), because it indicates that
    // the bytes received after it form a text string to be displayed on the LCD,
    //and therefore, the data stored in the receivedBTText variable will be used.

    switch (receivedBTData[0]) {
      case 1:
        // This message is a copy of the message received in ESP2,
        // and indicates that the LEDs are to be switched on with the
        // RGB code received in the following bytes
        msgCopyOfESP2 = true;
        msgESP2ID = 1;
        msgESP2ColorRed = receivedBTData[1];
        msgESP2ColorGreen = receivedBTData[2];
        msgESP2ColorBlue = receivedBTData[3];
        break;
      case 2:
        // This message is a copy of the message received in ESP2,
        // and indicates that the LEDs are to be switched off
        msgCopyOfESP2 = true;
        msgESP2ID = 2;
        break;
      case 3:
        // The indicators must be activated
        msgIndicatorsON = true;
        break;
      case 4:
        // The indicators must be deactivated
        msgIndicatorsOFF = true;
        break;
      case 5:
        // The car must move forward, with the PWM value received
        // in the second byte, in the range (0-255)
        msgGoForward = true;
        pwmValue = receivedBTData[1];
        break;
      case 6:
        // The car must move back, with the PWM value received
        // in the second byte, in the range (0-255)
        msgGoBack = true;
        pwmValue = receivedBTData[1];
        break;
      case 7:
        // The car must move right, with the PWM value received
        // in the second byte, in the range (0-255)
        msgGoRight = true;
        pwmValue = receivedBTData[1];
        break;
      case 8:
        // The car must move left, with the PWM value received
        // in the second byte, in the range (0-255)
        msgGoLeft = true;
        pwmValue = receivedBTData[1];
        break;
      case 9:
        // The car must be stopped
        msgStopCar = true;
        break;
      case 10:
        // The buzzer must be activated
        msgBuzzerON = true;
        break;
      case 11:
        // The buzzer must be deactivated
        msgBuzzerOFF = true;
        break;
      case 12:
        // This message is a copy of the message received in ESP2,
        // and indicates that the radar is to be switched on
        msgCopyOfESP2 = true;
        msgESP2ID = 12;
        break;
      case 13:
        // This message is a copy of the message received in ESP2,
        // and indicates that the radar is to be switched off
        msgCopyOfESP2 = true;
        msgESP2ID = 13;
        break;
      case 14:
        // A response is obtained from the application while the car is in motion,
        // to reset the watchdog timer, indicating that it is functioning correctly.
        msgGetResponseFromApp = true;
        break;
      case 38:
        // The LCD must be activated
        msgLcdON = true;
        break;
      case 39:
        // The LCD must be deactivated
        msgLcdOFF = true;
        break;
      case 40:
        // The LCD Status mode must be activated
        lcdModeStatus = true;
        lcdModeCustomText = false;
        break;
      case 41:
        // The LCD Custom Text mode must be activated
        lcdModeStatus = false;
        lcdModeCustomText = true;
        break;
      case 42:
        // A text string has been received to be displayed on the LCD screen
        msgNewTextLcd = true;
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
  serialBT.begin("ESP32-1");
  serialBT.register_callback(callback_function);
  //Serial.println("The ESP32 is ready, you can pair it via Bluetooth.");
  //Serial.println("Name: ESP32-1");

  // Motors
  // Motor A
  pinMode(pinMotorA_PWM, OUTPUT);
  pinMode(pinMotorA_G1, OUTPUT);
  pinMode(pinMotorA_G2, OUTPUT);
  ledcSetup(channelPWM_MotorA, frequencyPWMMotors, resolutionPWMMotors);
  ledcAttachPin(pinMotorA_PWM, channelPWM_MotorA);
  digitalWrite(pinMotorA_G1, LOW);
  digitalWrite(pinMotorA_G2, LOW);
  ledcWrite(channelPWM_MotorA, 0);

  // Motor B
  pinMode(pinMotorB_PWM, OUTPUT);
  pinMode(pinMotorB_G1, OUTPUT);
  pinMode(pinMotorB_G2, OUTPUT);
  ledcSetup(channelPWM_MotorB, frequencyPWMMotors, resolutionPWMMotors);
  ledcAttachPin(pinMotorB_PWM, channelPWM_MotorB);
  digitalWrite(pinMotorB_G1, LOW);
  digitalWrite(pinMotorB_G2, LOW);
  ledcWrite(channelPWM_MotorB, 0);


  // Buzzer
  ledcAttachPin(pinBuzzer, channelPWM_Buzzer);


  // Indicators
  pinMode(pinIndicatorFrontRight, OUTPUT);
  pinMode(pinIndicatorFrontLeft, OUTPUT);
  pinMode(pinIndicatorBackRight, OUTPUT);
  pinMode(pinIndicatorBackLeft, OUTPUT);


  // LCD Screen
  lcd.init();
  lcd.clear();

  //Serial.println("Setup performed by the CORE:");
  //Serial.println(xPortGetCoreID());
}


/*************************************** Loop ************************************/
void loop() {

  // Functions to be performed depending on the message received

  if (msgGoForward) {
    msgGoForward = false;
    GoForward();
  }

  if (msgGoBack) {
    msgGoBack = false;
    GoBack();
  }

  if (msgGoLeft) {
    msgGoLeft = false;
    GoLeft();
  }

  if (msgGoRight) {
    msgGoRight = false;
    GoRight();
  }

  if (msgStopCar) {
    msgStopCar = false;
    StopCar();
  }

  if (msgBuzzerON) {
    msgBuzzerON = false;
    SetLcdStatusText("BUZZER", "RINGING");
    ledcWriteTone(channelPWM_Buzzer, frequencyPWMBuzzer);
    //Serial.println("Buzzer Ringing");
  }

  if (msgBuzzerOFF) {
    msgBuzzerOFF = false;
    SetLcdStatusText("BUZZER", "OFF");
    ledcWrite(channelPWM_Buzzer, 0);
    //Serial.println("Buzzer Off");
  }

  if (msgNewTextLcd) {
    msgNewTextLcd = false;
    if (lcdModeCustomText) {
      SetNewText();
    }
  }

  if (msgLcdON) {
    msgLcdON = false;
    lcd.backlight();
    lcdIsON = true;
    SetLcdStatusText("LCD", "ON");
  }

  if (msgLcdOFF) {
    msgLcdOFF = false;
    lcd.noBacklight();
    lcd.clear();
    lcdIsON = false;
  }

  if (msgGetResponseFromApp) {
    msgGetResponseFromApp = false;

    if (carIsMoving) {
      timerWatchDog.detach();
      timerWatchDog.attach(watchDogPeriod, NoResponseFromApp);
    } else {
      timerWatchDog.detach();
    }
  }

  if (msgCopyOfESP2) {
    msgCopyOfESP2 = false;
    ManageCopyOfESP2msg(msgESP2ID);
  }

  if (msgIndicatorsON) {
    msgIndicatorsON = false;
    indicatorsON = true;
    SetLcdStatusText("INDICATORS", "ON");
  }

  if (msgIndicatorsOFF) {
    msgIndicatorsOFF = false;
    indicatorsON = false;
    SetLcdStatusText("INDICATORS", "OFF");
  }
}