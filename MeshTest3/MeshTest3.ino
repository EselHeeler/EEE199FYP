#include <Wire.h> //allows communication with I2C devices
#include <MQ135.h>
#include <Arduino.h>
#include "PMS.h"
#include "SoftwareSerial.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "painlessMesh.h"
#include <Arduino_JSON.h>


#define PMS7003_TX 32
#define PMS7003_RX 33
#define PIN_MQ135 34

#define PMS7003_BAUD 9600

Adafruit_BME680 bme; // I2C
MQ135 mq135_sensor(PIN_MQ135);

SoftwareSerial pms_ss(PMS7003_TX, PMS7003_RX);
PMS pms(pms_ss);
PMS::DATA pms_data;

// MESH Details
#define   MESH_PREFIX     "RNTMESH" //name for your MESH
#define   MESH_PASSWORD   "MESHpassword" //password for your MESH
#define   MESH_PORT       5555 //default port

//Number for this node
int nodeNumber = 3;

//String to send to other nodes with sensor readings
String readings;

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

int t1,t2,t3;

// User stub
void sendMessage() ; // Prototype so PlatformIO doesn't complain
String getReadings(); // Prototype for sending sensor readings

//Create tasks: to send messages and get readings;
Task taskSendMessage(TASK_SECOND * 60 , TASK_FOREVER, &sendMessage);
  
String getReadings () {
  JSONVar jsonReadings;
  t1=millis();
  jsonReadings["node"] = nodeNumber;
  jsonReadings["temperature"] = bme.temperature;
  jsonReadings["pressure"] = bme.pressure/100.0;
  jsonReadings["humidity"] = bme.humidity;
  jsonReadings["voc"] = bme.gas_resistance / 1000.0;

  float rzero = mq135_sensor.getRZero();
  float correctedRZero = mq135_sensor.getCorrectedRZero(bme.temperature, bme.humidity);
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(bme.temperature, bme.humidity);
  jsonReadings["co2"] = correctedPPM;

  float co_value = analogRead(35);
  jsonReadings["co"] = co_value;
  
  
  if (pms_ss.available()) { 
    pms_ss.read(); 
  }
  pms.requestRead();
  if (pms.readUntil(pms_data)) {
    jsonReadings["pm1"] = pms_data.PM_AE_UG_2_5;
    jsonReadings["pm2"] = pms_data.PM_AE_UG_10_0;
  }

  t2=millis();
  t3=t2-t1;
  jsonReadings["time"] = t3;
  readings = JSON.stringify(jsonReadings);
  return readings;
}

void sendMessage () {
  
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
  }
  
  String msg = getReadings();
  mesh.sendBroadcast(msg);
  Serial.print(msg);
}

void initSensors(){
  
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);  
  }
  
  pms_ss.begin(PMS7003_BAUD);
  
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("Received from %u msg=%s\n", from, msg.c_str());
  JSONVar myObject = JSON.parse(msg.c_str());
  int node = myObject["node"];
  double temperature = myObject["temperature"];
  double pressure = myObject["pressure"];
  double humidity = myObject["humidity"];
  double voc = myObject["voc"];
  double co2 = myObject["co2"];
  double co = myObject["co"];
  double pm1 = myObject["pm1"];
  double pm2 = myObject["pm2"];
  double t3 = myObject["time"];
  
  Serial.print("Node: ");
  Serial.println(node);
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hpa");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("VOC: ");
  Serial.print(voc);
  Serial.println(" KOhms");
  Serial.print("CO2: ");
  Serial.print(co2);
  Serial.println(" ppm");
  Serial.print("CO: ");
  Serial.print(co);
  Serial.println(" ppm");
  Serial.print("PM 2.5: ");
  Serial.print(pm1);
  Serial.println(" ug/m3");
  Serial.print("PM 10: ");
  Serial.print(pm2);
  Serial.println(" ug/m3");
  Serial.print("Time taken: ");
  Serial.print(t3);
  Serial.println(" ms");
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

void setup() {
  Serial.begin(115200);
  
  initSensors();
  
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
