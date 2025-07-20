//------------------------------- LIBRARIES ------------------------------//
#include <ESP32Servo.h>
#include <WiFiManager.h>
#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <time.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

#include <ElegantOTA.h>

//------------------------------ MACROS ------------------------------//

//-_-_-_-_-_-_ FIREBASE -_-_-_-_-_-//
#define FIREBASE_HOST "*************************"
#define FIREBASE_AUTH "*******************************"
#define API_KEY "*************************************"
#define DATABASE_URL "*****************************"
#define USER_EMAIL "hamdiesp@gmail.com"
#define USER_PASSWORD "esp2025"

//-_-_-_-_-_-_ servo pins -_-_-_-_-_-//
#define SERVO1_PIN 19
#define SERVO2_PIN 22

//-_-_-_-_-_-_ potentiometer pins -_-_-_-_-_-//
#define POT_PIN    36
#define LOWER_POT_LIMIT 5
#define UPPER_POT_LIMIT 4090


//-_-_-_-_-_-_ stepper config -_-_-_-_-_-//
#define DIR_PIN    12
#define STEP_PIN   14
#define ENA_PIN    27
#define microsteps 16
#define stepsperrevolution 200
#define STEPS_PER_UNIT_VOLUME 3200 // unit volume is 1ml

//------------------------------- INSTANCES -------------------------------//
Servo servo1;
Servo servo2;

FirebaseData    fbdo_stream;
FirebaseData    fbdo_set;
FirebaseJson    json;
FirebaseAuth    auth;
FirebaseConfig  config;

WiFiManager wifiManager;
AsyncWebServer server(80);
DNSServer dns;

//------------------------------- GLOBAL VARIABLES -------------------------------//
bool  firebaseConnected = false;
bool  newActionflag     = false;

String uid          = "";
String newTimeStamp = "";
String oldTimeStamp = "";
String action       = "";

// Firebase paths
const String listenerPath = "/actions/Hamdi/action";  // Option 1 (URL encoding)
const String tokenPath = "/actions/Hamdi/token";
const String dosePath = "/actions/Hamdi/dose";
const String feedbackPath = "/new_child/";

// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";

int potValue  = 0;
int volume    = 5000;
int counter   = 0;

unsigned long ota_progress_millis = 0;

//------------------------------- FUNCTION PROTOTYPES -------------------------------//
void  command_CPR(String action);
int motors(int stepper_command,int servo1_angle,int servo2_angle);
int move_stepper(int volume);
void stop_motors();
//*****************************************************************************//
void check_wifi();
void check_wifi_setup();
//*****************************************************************************//
void update_status(String status);
void update_IP(String IP);
void streamTimeoutCallback(bool timeout);
void streamCallback(FirebaseStream data);
//**************************************************************************** */
void collect();
void doser();
void purge();
void diluer();
void cycle_automatique();
float get_dose();
String get_token();
unsigned long getTime();
//*********************************************************************************** */
void onOTAEnd(bool success);
void onOTAProgress(size_t current, size_t final);
void onOTAStart();

//------------------------------- SETUP -------------------------------//
void setup() {
  Serial.begin(115200);

// Initialize Firebase configuration
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

// Connect to Wi-Fi
  // Setup WiFiManager
  WiFiManager wifiManager;
  wifiManager.autoConnect("ESP32-Setup"); // "ESP32-Setup" is the AP name if no saved credentials are found

  // Print IP address after successful connection
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Firebase setup after connecting to Wi-Fi
  Firebase.begin(&config, &auth);
  Serial.println("Getting User UID"); //Getting the user UID might take a few seconds
  while ((auth.token.uid) == "") 
  {
    Serial.print('.');
    delay(500);
  }

  String IP = WiFi.localIP().toString();
  update_IP(IP);
  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.print(uid);

  // Streaming (whenever data changes on a path)
  // Begin stream on a database path --> board1/outputs/digital
  if (!Firebase.RTDB.beginStream(&fbdo_stream, listenerPath.c_str())){
    Serial.printf("stream begin error, %s\n\n", fbdo_stream.errorReason().c_str());
  }else{
    Serial.println("stream began");
  }
    

  // Assign a calback function to run when it detects changes on the database
  Firebase.RTDB.setStreamCallback(&fbdo_stream, streamCallback, streamTimeoutCallback);

  // Attach servo pins
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  
  // Intialise stepper pins
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW);

  configTime(0, 0, ntpServer);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is ElegantOTA AsyncDemo.");
  });

  ElegantOTA.begin(&server);

  //ota callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.println("HTTP server started");

  Serial.println("Starting:");
}

//-------------------------- MAIN LOOP -------------------------------//
void loop() {
  // check wifi connexion
  check_wifi();

  // check for any OTA request
  ElegantOTA.loop();

  // run machine in case of a new command from firebase
  // flag change triggred by stream callback function
  
  if(newActionflag)
  {
    command_CPR(action);
    newActionflag = false;
  }
}

void command_CPR(String action)
{
  Serial.println("entered command_cpr");
  Serial.println(action[0]);

  switch (action[0]) {
    case 'c':
      if (action[1] = 'o')
      {
        collect();
      }else{
        if (action[1] = 'y')
        cycle_automatique();
      }
      break;

    case 'd':
      if (action[1] = 'o')
      {
        doser();
      }else{
        if (action[1] = 'i')
        diluer();
      }
      break;

    case 'p':
      purge();
      break;

    case 's':
      stop_motors();
      break;

    default:
      Serial.println("Invalid command");
      break;
  }
}


void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

int motors(int volume, int angleServo1, int angleServo2)
{
  servo1.write(angleServo1);
  servo2.write(angleServo2);
  delay(1000);
  int error = move_stepper(volume);
  return error;
}

int move_stepper(int volume)
{
  // enable driver
  digitalWrite(ENA_PIN, LOW);

  // change sense of direction
  if(volume>0)
  {
    digitalWrite(DIR_PIN, HIGH);
  }else{
    digitalWrite(DIR_PIN, LOW);
  }

  int steps = abs(volume) * STEPS_PER_UNIT_VOLUME;
  int step_counter = 0;

  while((step_counter < steps) && newActionflag)
  { 
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(50);
    step_counter++;
  }

  stop_motors();

  if (step_counter++ == steps)
  {
    // feedback success
    return 1;
  }else{
    //feedback failure
    return 0;
  }

}    

void update_status(String status)
{
  if (Firebase.RTDB.setString(&fbdo_set, "/actions/mohamed%20amine/status",status)) {
    Serial.println("status value set successfully!");
  } else {
    Serial.print("Failed to set string value, reason: ");
    Serial.println(fbdo_set.errorReason());
  }
}

void update_feedback_state(String status, String token)
{
  String statusPath="/new_child/";
  statusPath.concat(token);
  statusPath.concat("/feedback/state");
  Serial.println(statusPath);
  if (Firebase.RTDB.setString(&fbdo_set, statusPath, status))
  {
    Serial.println("feedback state set successfully!");
  }
  else
  {
    Serial.print("Failed to set feedback state value, reason: ");
    Serial.println(fbdo_set.errorReason());
  }
}

void update_feedback_date(String token)
{
  String datePath="/new_child/";
  datePath.concat(token);
  datePath.concat("/feedback/date");
  unsigned long date = getTime();
  if (Firebase.RTDB.setInt(&fbdo_set, datePath, date))
  {
    Serial.println("date set successfully!");
  }
  else
  {
    Serial.print("Failed to set date value, reason: ");
    Serial.println(fbdo_set.errorReason());
  }
}

void update_IP(String IP)
{
  if (Firebase.RTDB.setString(&fbdo_set, "/actions/mohamed%20amine/machineIP",IP)) {
    Serial.println("IP set successfully!");
  } else {
    Serial.print("Failed to set IP, reason: ");
    Serial.println(fbdo_set.errorReason());
  }
}

void check_wifi()
{
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("...Connecting to Wi-Fi...");
  }
    //Serial.println("Connected to Wi-Fi!");
}

void check_wifi_setup()
{
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("...Connecting to Wi-Fi...");
  }
    Serial.println("Connected to Wi-Fi!");
}

void streamCallback(FirebaseStream data){
  //Serial.println("entered callback function");
  
  if (counter == 1) //if it's not the firt trigger
  {
    // Get the path that triggered the function
    String streamPath = String(data.dataPath());
    // if the data returned is an integer, there was a change on the GPIO state on the following path /{gpio_number}
    if (data.dataTypeEnum() == fb_esp_rtdb_data_type_string)
    {
      action = data.stringData();
      Serial.println(action);
      newActionflag = true;
      if(action[0] == 's') {
        newActionflag = false;
      }
    }else {
      Serial.println("error with action variable type");
    } 
    Serial.printf("Received stream payload size: %d (Max. %d)\n\n", data.payloadLength(), data.maxPayloadLength());
  }else{
    if(counter == 0){
      Serial.println("first run of the centery");
      counter = 1;
    }
  }
}

void streamTimeoutCallback(bool timeout){
  if (timeout)
    Serial.println("stream timeout, resuming...\n");
  if (!fbdo_stream.httpConnected())
    Serial.printf("error code: %d, reason: %s\n\n", fbdo_stream.httpCode(), fbdo_stream.errorReason().c_str());
}

float get_dose()
{
  float dose;
  if (Firebase.RTDB.getString(&fbdo_set, "/actions/mohamed%20amine/dose")) 
    {
      String dose_string = fbdo_set.stringData();
      dose_string.trim();
      dose = dose_string.toFloat();
      Serial.println("dose received: "); Serial.println(dose);
    }else{
      Serial.println("Failed to get dose from Firebase:");
      Serial.println(fbdo_set.errorReason());
    }
    return dose;
}

String get_token()
{
  String token;
  if(Firebase.RTDB.getString(&fbdo_set, "/actions/mohamed%20amine/token")) 
    {
      token = fbdo_set.stringData();
      token.trim();
      Serial.println("token received: "); Serial.println(token);
    }else{
      Serial.println("Failed to get token from Firebase:");
      Serial.println(fbdo_set.errorReason());
    }
    return token;
}

void collect()
{
  update_status("collect en cours");
  float dose = get_dose();
  if(motors(dose, 180, 180))
  {
    update_status("collect done");
  }
  else{
    update_status("error in collect : incomplete actuation");
  } 
}

void cycle_automatique()
{
  update_status("cycle automatique en cours");
  float dose = get_dose();
  purge();
  doser();
  diluer();
  update_status("cycle automatique done");
  //update_feedback_state("done");
}

void doser()
{
  update_status("dose en cours");
  float dose = get_dose();
  motors(dose, 0, 0);
  update_status("dose done");
}

void diluer()
{
  update_status("diluer en cours");
  float dose = get_dose();
  motors(dose, 0, 0);
  update_status("diluer done");
  String token = get_token();
  update_feedback_state("done", token);
  update_feedback_date(token);
}

void purge()
{
  update_status("purge en cours");
  motors(1, 0, 0);
  update_status("purge done");
}

void stop_motors()
{
  digitalWrite(ENA_PIN, HIGH);
  newActionflag = false;
  update_status("processus stopped");
  Serial.println("processus stopped");
}

unsigned long getTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}
