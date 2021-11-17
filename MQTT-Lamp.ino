/*
 * CAVALO VENDADO'S MQTT IOT LAMP CONTROLLER
 * PART OF OUR AUTOMATION PROJECT.
 * 
 * THIS SOFTWARE IS BASED OFF OF ESP8266 MQTT EXAMPLE
 * DEVELOPED BY THE CAVALO VENDADO ROBOTICS TEAM
 * COLÉGIO MARISTA MEDIANEIRA ERECHIM-RS FIRST NUMBER: 16786
 */


#include <ESP8266WiFi.h>
#include <PubSubClient.h> 
const int Bot1 = 12;
const int Bot2 = 5;
const int Bot3 = 4;
const int Bot4 = 13;

const int Rele1 = 14;
const int Rele2 = 16;
const int Rele3 = 0;
const int Rele4 = 15;

const char* ssid = "roboticawifi2.4g";
const char* password = "roboticamedia123";
const char* mqtt_server = "192.168.1.12";

bool lampstates[4] = {0,0,0,0};

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
    lampstates[0] = payload[0] - 48;
    digitalWrite(Rele1, !lampstates[0]);
  }
  if(strcmp(topic, "/lamps/1") == 0){
    lampstates[1] = payload[0] - 48;
    digitalWrite(Rele2, !lampstates[1]);
  }
  if(strcmp(topic, "/lamps/2") == 0){
    lampstates[2] = payload[0] - 48;
    digitalWrite(Rele3, !lampstates[2]);
  }
  if(strcmp(topic, "/lamps/3") == 0){
    lampstates[3] = payload[0] - 48;
    digitalWrite(Rele4, !lampstates[3]);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-door";
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
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  //set all I/O modes
  pinMode(Bot1, INPUT);
  pinMode(Bot2, INPUT);
  pinMode(Bot3, INPUT);
  pinMode(Bot4, INPUT);
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
  if (!conneceted) {
    reconnect();
  }
  pinMode(BUILTIN_LED, !conneceted); //use builtin LED as an indicator. Turns on when connected.
  client.loop();

  
  bool anyBtn = (digitalRead(Bot1) or digitalRead(Bot2) or digitalRead(Bot3) or digitalRead(Bot4));
  if(anyBtn != LastAnyBtn and anyBtn){ //if any button is pressed
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterrupt > 200){ //if not pressed within 200ms
      //turn respective lamp on/off
      if(digitalRead(Bot1)){
        char outmsg[]={!lampstates[0] + 48};
        client.publish("/lamps/0", outmsg);
      }
      if(digitalRead(Bot2)){
        char outmsg[]={!lampstates[1] + 48};
        client.publish("/lamps/1", outmsg);
      }
      if(digitalRead(Bot3)){
        char outmsg[]={!lampstates[2] + 48};
        client.publish("/lamps/2", outmsg);
      }
      if(digitalRead(Bot4)){
        char outmsg[]={!lampstates[3] + 48};
        client.publish("/lamps/3", outmsg);
      }
    }
    lastInterrupt = interruptTime;
  }
  LastAnyBtn = anyBtn;
}
