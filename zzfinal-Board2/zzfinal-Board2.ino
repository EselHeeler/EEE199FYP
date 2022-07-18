#include <Wire.h> //allows communication with I2C devices
#include <WiFi.h>
#include <MQ135.h>
#include <Arduino.h>
#include "SoftwareSerial.h"
#include "PMS.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h> 

#define PMS7003_TX 32
#define PMS7003_RX 33
#define PIN_MQ135 34

#define PMS7003_BAUD 9600

Adafruit_BME680 bme; // I2C
MQ135 mq135_sensor(PIN_MQ135);

SoftwareSerial pms_ss(PMS7003_TX, PMS7003_RX);
PMS pms(pms_ss);
PMS::DATA pms_data;

uint8_t broadcastAddress[] = {0x9C, 0x9C, 0x1F, 0x10, 0x2C, 0x74};

constexpr char WIFI_SSID[] = "DILNET-WIFIMobile";

typedef struct struct_message {
    int id; // must be unique for each sender board
    float temp;
    float humidity;
    float pressure;
    float voc;
    float co_2;
    float co;
    float pm_1;
    float pm_2;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Create peer interface
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

int32_t getWiFiChannel(const char *ssid)
{
  if (int32_t n = WiFi.scanNetworks()){
    for (uint8_t i=0; i<n; i++){
      if (!strcmp(ssid, WiFi.SSID(i).c_str())){
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

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

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {

  myData.id=2;
  
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");
  myData.temp = bme.temperature; 

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");
  myData.pressure = (bme.pressure/100.0);

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");
  myData.humidity = bme.humidity;

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");
  myData.voc = (bme.gas_resistance / 1000.0);

  float rzero = mq135_sensor.getRZero();
  float correctedRZero = mq135_sensor.getCorrectedRZero(bme.temperature, bme.humidity);
  float resistance = mq135_sensor.getResistance();
  float ppm = mq135_sensor.getPPM();
  float correctedPPM = mq135_sensor.getCorrectedPPM(bme.temperature, bme.humidity);
  Serial.print("MQ135 RZero: ");
  Serial.print(rzero);
  Serial.print("\t Corrected RZero: ");
  Serial.print(correctedRZero);
  Serial.print("\t Resistance: ");
  Serial.print(resistance);
  Serial.print("\t PPM: ");
  Serial.print(ppm);
  Serial.print("\t Corrected PPM: ");
  Serial.print(correctedPPM);
  Serial.println("ppm");
  myData.co_2 = correctedPPM;

  float co_value = analogRead(35);
  Serial.print("Carbon Monoxide: ");
  myData.co = co_value;
  Serial.print(co_value);
  Serial.println("ppm");
  
  
  if (pms_ss.available()) { 
    pms_ss.read(); 
  }
  pms.requestRead();
  if (pms.readUntil(pms_data)) {
    Serial.print("PM 2.5 (ug/m3): "); 
    myData.pm_1 = pms_data.PM_AE_UG_2_5;
    Serial.println(pms_data.PM_AE_UG_2_5);
    Serial.print("PM 10.0 (ug/m3): "); 
    myData.pm_2 = pms_data.PM_AE_UG_10_0;
    Serial.println(pms_data.PM_AE_UG_10_0);
    
  }

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
    
  delay(60000);
}
