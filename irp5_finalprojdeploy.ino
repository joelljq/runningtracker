/* Includes ---------------------------------------------------------------- */
#include <irp5-finalproj_inferencing.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <String.h>
#include <npsoe_iot_kit.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include "demoOled.h"
#include "logo.h"
Adafruit_MPU6050 mpu;
/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 9.80665f
#define MAX_ACCEPTED_RANGE 2.0f  // starting 03/2022, models are generated setting range to +-2, but this example use Arudino library which set range to +-4g. If you are using an older model, ignore this value and use 4.0f instead

/* Constant defines -------------------------------------------------------- */
// IRP5: The minimum value for the classification output to be considered certain
#define CONFIDENCE_THRESHOLD 0.5f
#define LED 12
#define LED_R 23  // LED pin
#define LED_G 18
#define PB1 26
#define PB2 27
#define buzzerPin 14
#define BOTtoken "API TOKEN"
#define CHAT_ID "CHATID"



/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;  // Set this to true to see e.g. features generated from the raw signal
unsigned long timeStamp = 0;
bool currPBVal, oldPBVal, PBVal;
unsigned long previousMillis = 0;
unsigned long interval = 2000;
static char recv_buf[512];
static bool is_exist = false;
static bool is_join = false;
const char *ssid = "antecon";
const char *password = "mindylam";
WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);
/**
* @brief      Arduino setup function
*/


static int at_send_check_response(const char *p_ack, int timeout_ms, const char *p_cmd, ...) {
  int ch;

  int index = 0;
  int startMillis = 0;
  va_list args;
  memset(recv_buf, 0, sizeof(recv_buf));
  va_start(args, p_cmd);
  Serial2.printf(p_cmd, args);  // send command to LoRa E5
  Serial.printf(p_cmd, args);
  va_end(args);
  delay(200);
  startMillis = millis();

  if (p_ack == NULL) {
    return 0;
  }

  do {
    while (Serial2.available() > 0) {  // as long as LoRa module receives data
      ch = Serial2.read();             // read the data
      recv_buf[index++] = ch;          // store in recv_buf
      Serial.print((char)ch);          // display in Serial Monitor
      delay(2);
    }

    if (strstr(recv_buf, p_ack) != NULL) {  // if p_ack is found in recv_buf, return 1
      return 1;
    }

  } while (millis() - startMillis < timeout_ms);
  return 0;
}

static void recv_parse(char *p_msg) {
  if (p_msg == NULL) {
    return;
  }
  char *p_start = NULL;

  int rssi = 0;
  int snr = 0;

  char data;

  p_start = strstr(p_msg, "RX");
  if (p_start && (1 == sscanf(p_start, "RX: \"%2x\"", &data))) {
    Serial.printf("rx: %d\n", data);
    if (data == 0x01) {
      digitalWrite(LED_G, HIGH);
      digitalWrite(LED_R, LOW);
      for (long i = 0; i < 2048 * 3; i++) {
        bot.sendMessage(CHAT_ID, "You did not run any KM!", "");
        digitalWrite(buzzerPin, HIGH);
        delayMicroseconds(244);
        digitalWrite(buzzerPin, LOW);
        delayMicroseconds(244);
      }
    } else {
      digitalWrite(LED_R, HIGH);
      digitalWrite(LED_G, LOW);
    }
  }

  p_start = strstr(p_msg, "RSSI");  // find "RSSI" in p_msg
  // if p_start is not NULL, capture the RSSI value as integer
  if (p_start && (1 == sscanf(p_start, "RSSI %d,", &rssi))) {
    Serial.print("rssi: ");
    Serial.println(rssi);
  }

  p_start = strstr(p_msg, "SNR");  // find "SNR" in p_msg
  // if p_start is not NULL, capture the RSSI value as integer
  if (p_start && (1 == sscanf(p_start, "SNR %d", &snr))) {
    Serial.print("snr: ");
    Serial.println(snr);
  }
}

void setup() {
  
  //Modified here
  //IRP5: accelerometer setup in this section
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  delay(5000);
  Serial.begin(115200);
  Serial2.begin(9600);  // Setup communication with Lora-E5 module that is connected to UART2
  Serial.print("E5 LORAWAN TEST\r\n");

  if (at_send_check_response("+AT: OK", 100, "AT\r\n")) {
    is_exist = true;
    at_send_check_response("+ID: AppEui", 1000, "AT+ID\r\n");
    at_send_check_response("+MODE: LWOTAA", 1000, "AT+MODE=LWOTAA\r\n");

    // use the following 2 lines for AS923 band, ensure similar setting in TTN
    at_send_check_response("+DR: AS923", 1000, "AT+DR=AS923\r\n");
    at_send_check_response("+CH: NUM", 1000, "AT+CH=NUM,0-1\r\n");

    // use the following 2 lines for EU868 band, ensure similar setting in TTN
    // at_send_check_response("+DR: EU868", 1000, "AT+DR=EU868\r\n");
    // at_send_check_response("+CH: NUM", 1000, "AT+CH=NUM,0-2\r\n");

    at_send_check_response("+CH: NUM", 1000, "AT+CH=NUM,0-1\r\n");
    at_send_check_response("+KEY: APPKEY", 1000, "AT+KEY=APPKEY,\"684CCFC00208B86578301697F8D820CD\"\r\n");
    at_send_check_response("+CLASS: C", 1000, "AT+CLASS=A\r\n");
    at_send_check_response("+PORT: 8", 1000, "AT+PORT=8\r\n");

    delay(200);
    Serial.println("LoRaWAN");
    is_join = true;
  } else {
    is_exist = false;
    Serial.print("No E5 module found.\r\n");
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);



  if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 6) {
    ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 6 (the 6 sensor axes)\n");
    return;
  }
  pinMode(LED, OUTPUT);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  oledSetTextCursor(0, 0);
  oledInit();
  oled.clearDisplay();
  oled.drawBitmap(0, 0, image_data_LOGOarray, 128, 64, 1);
  oled.display();
  pinMode(PB1, INPUT_PULLUP);
  pinMode(PB2, INPUT_PULLUP);
  oldPBVal = digitalRead(PB1);
  timeStamp = millis();
  pinMode(buzzerPin, OUTPUT);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  Serial.print("Connecting to ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  WiFi.waitForConnectResult();
  Serial.print(", WiFi connected, IP address: ");
  Serial.println(WiFi.localIP());
  bot.sendMessage(CHAT_ID, "Bot started up", "");
}
float totaldistance = 0.0;
float pointsdistance = 0.0;
int points = 0;
bool stop = false;
unsigned long stopStartTime;
String results;
bool running = false;

void oledprint(String r) {
  oledSetTextCursor(1, 3);
  oled.printf("You are %s", r);
  oledSetTextCursor(1, 4);
  oled.printf("Total KM: %.2f", totaldistance);
  oledSetTextCursor(1, 5);
  oled.printf("Total Points: %d", points);
}


void getresults() {
  ei_printf("\nStarting inferencing in 2 seconds...\n");
  delay(1000);
  ei_printf("\nStarting inferencing in 1 seconds...\n");
  delay(1000);
  ei_printf("Sampling...\n");
  // Allocate a buffer here for the values we'll read from the IMU
  float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };
  for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 6) {
    // Determine the next tick (and then sleep later)
    uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);
    //modified here
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    // IRP5: Read the accelerometer data in this section, inside the for loop
    // The 3 axis reading is stored into an array called buffer
    buffer[ix + 0] = a.acceleration.x;
    buffer[ix + 1] = a.acceleration.y;
    buffer[ix + 2] = a.acceleration.z;
    buffer[ix + 3] = g.gyro.x;
    buffer[ix + 4] = g.gyro.y;
    buffer[ix + 5] = g.gyro.z;
    delayMicroseconds(next_tick - micros());
  }
  // Turn the raw buffer in a signal which we can the classify
  signal_t signal;
  int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
  if (err != 0) {
    ei_printf("Failed to create signal from buffer (%d)\n", err);
    return;
  }



  // Run the classifier
  ei_impulse_result_t result = { 0 };



  err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }



  // print the predictions
  float maxp = 0.0, temp;
  int imax = 0;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    temp = result.classification[ix].value;
    ei_printf("    %s: %.5f\n", result.classification[ix].label, temp);
    if (result.classification[ix].value > maxp) {
      maxp = temp;
      imax = ix;
    }
  }
  if (maxp > CONFIDENCE_THRESHOLD) {
    ei_printf("RESULT = %s\n", result.classification[imax].label);
    if (result.classification[imax].label == "running") {
      results = "running";
    } else if (result.classification[imax].label == "walking") {
      results = "walking";
    } else if (result.classification[imax].label == "stop") {
      results = "stop";
    }
  } else {
    ei_printf("RESULT is uncertain.\n");
    results = "uncertain";
  }


#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif
}

void blinkRedLED(void){
  timeStamp = millis();
  if(millis() - timeStamp >= 1000) {
    digitalWrite(LED, !digitalRead(LED));
    timeStamp = millis();
  } 
}

void pointssystem(String r) {
  if (r == "running") {
    totaldistance += 0.01;
    pointsdistance += 0.01;
    oledprint(r);
    oled.display();
  } else if (r == "walking") {
    totaldistance += 0.005;
    pointsdistance += 0.005;
    oledprint(r);
    oled.display();
  } else if (r == "stop") {
    if (!stop) {
      stopStartTime = millis();
      stop = true;
      oledprint(r);
      oledSetTextCursor(1, 7);
      oled.printf("");
      oled.display();
    } else {
      if (millis() - stopStartTime >= 10000) {
        oledprint(r);
        oledSetTextCursor(1, 7);
        oled.printf("PLEASE START WALKING");
        oled.display();
        stop = false;
        blinkRedLED();
        for (long i = 0; i < 2048 * 3; i++) {
          digitalWrite(buzzerPin, HIGH);
          delayMicroseconds(244);
          digitalWrite(buzzerPin, LOW);
          delayMicroseconds(244);
        }
      }
    }
  }

  if (pointsdistance >= 0.1) {  // if distance reached 1 km
    points += 2;
    pointsdistance -= 0.1;
    oledSetTextCursor(1, 6);
    oled.printf("You have ran a KM!");  // subtract 1 km from distance
  } else {
    oledSetTextCursor(1, 6);
    oled.printf("");
  }
}
bool pb1PressedChk() {
  currPBVal = digitalRead(PB1);
  if (currPBVal != oldPBVal) {
    // there is a change from the old value, can be H-L or L-H
    delay(50);               // debounce
    oldPBVal = !oldPBVal;    // new value read
    if (currPBVal == LOW) {  // new value is low
      return true;
    }
  }
  return false;
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop() {
  oled.clearDisplay();
  if (pb1PressedChk()) {
    running = !running;
    Serial.print(running);
  }
  if (running) {
    getresults();
    pointssystem(results);
    PBVal = digitalRead(PB2);
    Serial.print(PBVal);
    if (PBVal == LOW) {
      running = !running;
      Serial.print(is_exist);
      if (is_exist) {
        oled.clearDisplay();
        oled.print("Sending Data To Lora");
        oled.display();
        int ret = 0;
        if (is_join) {
          ret = at_send_check_response("+JOIN: Network joined", 12000, "AT+JOIN\r\n");
          if (ret) {
            is_join = false;
            char cmd[128];
            sprintf(cmd, "AT+CMSGHEX=\"%04X%04X\"\r\n", (int)(totaldistance * 100), (int)(points));
            ret = at_send_check_response("Done", 10000, cmd);

            if (ret) {
              recv_parse(recv_buf);
            } else {
              Serial.print("Send failed!\r\n\r\n");
            }
          } else {
            at_send_check_response("+ID: AppEui", 1000, "AT+ID\r\n");
            Serial.print("JOIN failed!\r\n\r\n");
          }
        } else {
          char cmd[128];
          sprintf(cmd, "AT+CMSGHEX=\"%04X%04X\"\r\n", (int)(totaldistance * 100), (int)(points));
          ret = at_send_check_response("Done", 10000, cmd);
          if (ret) {
            recv_parse(recv_buf);
          } else {
            Serial.print("Send failed!\r\n\r\n");
          }
          delay(5000);
        }
      }
    }
  } else {
    if (totaldistance != 0.0) {
      String message = "Your Run Summary\nTotal Distance Covered: " + String(totaldistance) + " KM\nTotal Points Currently: " + String(points);
      bot.sendMessage(CHAT_ID, message, "");
      oled.clearDisplay();
      oledSetTextCursor(1, 1);
      oled.print("Your Run Summary");
      oledSetTextCursor(1, 2);
      oled.printf("Total Distance: %.2f", totaldistance);
      oledSetTextCursor(1, 3);
      oled.printf("Total Points: %d", points);
      oled.display();
      delay(2000);
    }
    totaldistance = 0.0;
    pointsdistance = 0.0;
    oled.clearDisplay();
    oledSetTextCursor(4, 1);
    oled.print("Press Button");
    oledSetTextCursor(4, 2);
    oled.print("To Start");
    oled.display();
  }
}
