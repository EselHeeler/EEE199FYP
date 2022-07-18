#include <esp_now.h>
#include <WiFi.h>
#include <ThingsBoard.h>

#define TOKEN "EJkh7m8eDOnTMglqiTqM"
#define THINGSBOARD_SERVER "thingsboard.cloud"

static const char* ssid = "DILNET-WIFIMobile";
static const char* password = "12345678";

// Initialize ThingsBoard client
WiFiClient espClient;
// Initialize ThingsBoard instance
ThingsBoard tb(espClient);
// the Wifi radio's status
int status = WL_IDLE_STATUS;

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

// Create a structure to hold the readings from each board
struct_message board1;
struct_message board2;

// Create an array with all the structures
struct_message boardsStruct[2] = {board1, board2};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  
  char macStr[18];
  Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);

  boardsStruct[myData.id-1].temp=myData.temp;
  boardsStruct[myData.id-1].humidity=myData.humidity;
  boardsStruct[myData.id-1].pressure=myData.pressure;
  boardsStruct[myData.id-1].voc=myData.voc;
  boardsStruct[myData.id-1].co_2=myData.co_2;
  boardsStruct[myData.id-1].co=myData.co;
  boardsStruct[myData.id-1].pm_1=myData.pm_1;
  boardsStruct[myData.id-1].pm_2=myData.pm_2;
 

  if (myData.id==1){

  Serial.print("Temp: ");
  Serial.println(boardsStruct[myData.id-1].temp);
  
  Serial.print("Humidity: ");
  Serial.println(boardsStruct[myData.id-1].humidity); 

  Serial.print("Pressure: ");
  Serial.println(boardsStruct[myData.id-1].pressure); 
  
  Serial.print("VOC: ");
  Serial.println(boardsStruct[myData.id-1].voc); 

  Serial.print("CO2: ");
  Serial.println(boardsStruct[myData.id-1].co_2); 

  Serial.print("CO: ");
  Serial.println(boardsStruct[myData.id-1].co); 

  Serial.print("PM2.5: ");
  Serial.println(boardsStruct[myData.id-1].pm_1); 

  Serial.print("PM10: ");
  Serial.println(boardsStruct[myData.id-1].pm_2); 

  Serial.println();
  
  tb.sendTelemetryFloat("NODE 1 TEMP", boardsStruct[myData.id-1].temp);
  tb.sendTelemetryFloat("NODE 1 HUMIDITY", boardsStruct[myData.id-1].humidity); 
  tb.sendTelemetryFloat("NODE 1 PRESSURE", boardsStruct[myData.id-1].pressure); 
  tb.sendTelemetryFloat("NODE 1 VOC", boardsStruct[myData.id-1].voc); 
  tb.sendTelemetryFloat("NODE 1 CO2", boardsStruct[myData.id-1].co_2); 
  tb.sendTelemetryFloat("NODE 1 CO", boardsStruct[myData.id-1].co); 
  tb.sendTelemetryFloat("NODE 1 PM2.5", boardsStruct[myData.id-1].pm_1); 
  tb.sendTelemetryFloat("NODE 1 PM10", boardsStruct[myData.id-1].pm_2);
  
  }

  if (myData.id==2){
  
  Serial.print("Temp: ");
  Serial.println(boardsStruct[myData.id-1].temp);
  
  Serial.print("Humidity: ");
  Serial.println(boardsStruct[myData.id-1].humidity); 

  Serial.print("Pressure: ");
  Serial.println(boardsStruct[myData.id-1].pressure); 
  
  Serial.print("VOC: ");
  Serial.println(boardsStruct[myData.id-1].voc); 

  Serial.print("CO2: ");
  Serial.println(boardsStruct[myData.id-1].co_2); 

  Serial.print("CO: ");
  Serial.println(boardsStruct[myData.id-1].co); 

  Serial.print("PM2.5: ");
  Serial.println(boardsStruct[myData.id-1].pm_1); 

  Serial.print("PM10: ");
  Serial.println(boardsStruct[myData.id-1].pm_2); 

  Serial.println();
  tb.sendTelemetryFloat("NODE 2 TEMP", boardsStruct[myData.id-1].temp);
  tb.sendTelemetryFloat("NODE 2 HUMIDITY", boardsStruct[myData.id-1].humidity); 
  tb.sendTelemetryFloat("NODE 2 PRESSURE", boardsStruct[myData.id-1].pressure); 
  tb.sendTelemetryFloat("NODE 2 VOC", boardsStruct[myData.id-1].voc); 
  tb.sendTelemetryFloat("NODE 2 CO2", boardsStruct[myData.id-1].co_2); 
  tb.sendTelemetryFloat("NODE 2 CO", boardsStruct[myData.id-1].co); 
  tb.sendTelemetryFloat("NODE 2 PM2.5", boardsStruct[myData.id-1].pm_1); 
  tb.sendTelemetryFloat("NODE 2 PM10", boardsStruct[myData.id-1].pm_2); 
  }
}
 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_AP_STA);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.print(".");
  }

 

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (!tb.connected()) {
    // Connect to the ThingsBoard
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect");
      return;
    }
    if (tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Connected");
      return;
    }
  }
}
