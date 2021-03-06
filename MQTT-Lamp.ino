/*
 * CAVALO VENDADO'S MQTT IOT LAMP CONTROLLER
 * PART OF OUR AUTOMATION PROJECT.
 * 
 * THIS SOFTWARE IS BASED OFF OF ESP8266 MQTT EXAMPLE
 * DEVELOPED BY THE CAVALO VENDADO ROBOTICS TEAM
 * COLÉGIO MARISTA MEDIANEIRA ERECHIM-RS FIRST NUMBER: 16786
 */


#include <WiFi.h>
#include <PubSubClient.h> 
const int LED = 2;
const int Bot1 = 26;
const int Bot2 = 25;
const int Bot3 = 33;
const int Bot4 = 32;

const int Rele1 = 13;
const int Rele2 = 18;
const int Rele3 = 14;
const int Rele4 = 27;

const char* ssid = "roboticawifi2.4g";
const char* password = "roboticamedia123";
const char* mqtt_server = "192.168.1.12";

bool lampstates[4] = {0,0,0,0};
byte outmsg[1]={0};
static unsigned long lastInterrupt = 0;

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
bool LastAnyBtn = false;
void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.println("] ");
  if(strcmp(topic, "/lamps/0") == 0){
    lampstates[0] = payload[0];
    digitalWrite(Rele1, !lampstates[0]);
  }
  if(strcmp(topic, "/lamps/1") == 0){
    lampstates[1] = payload[0];
    digitalWrite(Rele2, !lampstates[1]);
  }
  if(strcmp(topic, "/lamps/2") == 0){
    lampstates[2] = payload[0];
    digitalWrite(Rele3, !lampstates[2]);
  }
  if(strcmp(topic, "/lamps/3") == 0){
    lampstates[3] = payload[0];
    digitalWrite(Rele4, !lampstates[3]);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP-door-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      //always subscribe to topics after connecting to mqtt server.
      Serial.println("connected");
      client.subscribe("/lamps/0");
      client.subscribe("/lamps/1");
      client.subscribe("/lamps/2");
      client.subscribe("/lamps/3");
    } else {
      //print error code
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void setup() {
  pinMode(LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  //set all I/O modes
  pinMode(Bot1, INPUT_PULLUP);
  pinMode(Bot2, INPUT_PULLUP);
  pinMode(Bot3, INPUT_PULLUP);
  pinMode(Bot4, INPUT_PULLUP);
  pinMode(Rele1, OUTPUT);
  pinMode(Rele2, OUTPUT);
  pinMode(Rele3, OUTPUT);
  pinMode(Rele4, OUTPUT);
  //turn off all lights.
  digitalWrite(Rele1, HIGH);
  digitalWrite(Rele2, HIGH);
  digitalWrite(Rele3, HIGH);
  digitalWrite(Rele4, HIGH);
}

void loop() {
  bool conneceted = client.connected();
  digitalWrite(LED, conneceted); //use builtin LED as an indicator. Turns on when connected.
  if (!conneceted) {
    reconnect();
  }
  client.loop();
  bool btn1 = !digitalRead(Bot1);
  bool btn2 = !digitalRead(Bot2);
  bool btn3 = !digitalRead(Bot3);
  bool btn4 = !digitalRead(Bot4);
  
  bool anyBtn = (btn1 or btn2 or btn3 or btn4);
  //Serial.println(anyBtn);
  if(anyBtn != LastAnyBtn and anyBtn){ //if any button is pressed
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterrupt > 200){ //if not pressed within 100ms
      //turn respective lamp on/off
      if(btn1){
        outmsg[0]={!lampstates[0]};
        client.publish("/lamps/0", outmsg,1,true);
      }
      if(btn2){
        outmsg[0]={!lampstates[1]};
        client.publish("/lamps/1", outmsg,1,true);
      }
      if(btn3){
        outmsg[0]={!lampstates[2]};
        client.publish("/lamps/2", outmsg,1,true);
      }
      if(btn4){
        outmsg[0]={!lampstates[3]};
        client.publish("/lamps/3", outmsg,1,true);
      }
    }
    lastInterrupt = interruptTime;
  }
  LastAnyBtn = anyBtn;
}
