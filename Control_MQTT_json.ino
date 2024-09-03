#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>

#ifdef __cplusplus
  extern "C" {
#endif
  uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

char ChValue[5];
char relay_pin_index[5];

// WiFi Config
#define WIFI_STA_NAME1 "Mild_Saras"
#define WIFI_STA_PASS1 "Mild0835417135"

#define WIFI_STA_NAME2 "Smartfarm_99_5G"
#define WIFI_STA_PASS2 "99TKC999"

// MQTT Config
#define MQTT_SERVER   "110.164.181.55"
#define MQTT_PORT     1883
#define MQTT_USERNAME ""
#define MQTT_PASSWORD ""
#define MQTT_NAME     "Control_55"

#define MQTT_TOPIC_SUB "showroom/control/Control_55"
#define MQTT_TOPIC_PUB "showroom/monitor/Control_55"


WiFiClient client;
PubSubClient mqtt(client);

bool send = false;
bool ping = false;

// Relay
//int relaypin[] = {33, 25, 26, 27};
int relaypin[] = {25, 26, 27, 33};
int arr_size = 4;
int btnpin[] = {-1, -1, -1, -1}; // button pin
int btnstatus[] = {0, 0, 0, 0}; // HIGH/LOW status button

// Relay state structure
struct RelayState {
    int pinIndex;
    int status;
    unsigned long endTime;
    bool active;
};

RelayState relayStates[4]; // Adjust the size if needed
char payload[256] ="A";

unsigned long lastTempPublish = 0;
const unsigned long tempPublishInterval = 10000; // 10 seconds

void connectToWiFi(const char* ssid, const char* password) {
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);
    Serial.print("Connecting to Wi-Fi");

    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void setup() {
    Serial.begin(115200);

    Serial.println();

    connectToWiFi(WIFI_STA_NAME1, WIFI_STA_PASS1);
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Failed to connect to WiFi network 1, trying network 2...");
        connectToWiFi(WIFI_STA_NAME2, WIFI_STA_PASS2);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Failed to connect to both WiFi networks, restarting...");
        ESP.restart();
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setCallback(callback);
    mqtt.setKeepAlive(3600);

    for (int i = 0; i < arr_size; i++) {
        pinMode(relaypin[i], OUTPUT);
        digitalWrite(relaypin[i], LOW);

        // Initialize relay states
        relayStates[i].pinIndex = i;
        relayStates[i].status = LOW;
        relayStates[i].endTime = 0;
        relayStates[i].active = false;
    }

    while (!client.connected()) {
        Serial.println("Connecting to MQTT...");
        
        if (mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD)) {
            Serial.println("MQTT Connected");
        } else {
            Serial.print("failed with state ");
            Serial.print(mqtt.state());
            delay(5000);
        }
    }
    
    mqtt.subscribe(MQTT_TOPIC_SUB);
    
    for (int i = 0; i < arr_size; i++) {
        pinMode(relaypin[i], OUTPUT);
        pinMode(btnstatus[i], INPUT);
        digitalWrite(relaypin[i], LOW);
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, length);
    Serial.println("callback -----------------");

    if (doc["status"] == "check") {
        sendRelayStatus(); // ส่งสถานะรีเลย์ทั้งหมดกลับไปยัง MQTT

    } else if (doc["status"] == "control1") {
        Serial.println("Loop control 1 -----------------");
        int time = doc["time"];
        long timedelay = time * 1000 * 60;

        Serial.println("Turn ON");
        
        for (int i = 0; i < arr_size; i++) {
            if (doc["relay"][i] == 1) {
                digitalWrite(relaypin[i], HIGH);  
            }
        }
        sendRelayStatus();
        delay(timedelay);
        for (int i = 0; i < arr_size; i++) {
            if (doc["relay"][i] == 1) {
                digitalWrite(relaypin[i], LOW);
            }
        }
        Serial.println("Turn OFF");

        sendRelayStatus(); // ส่งสถานะรีเลย์ทั้งหมดหลังจากควบคุมเสร็จแล้ว

    } else if (doc["status"] == "control2") {
        Serial.println("Loop control 2 -----------------");
        int relay_pin = doc["relay"][0];
        int relay_status = doc["relay"][1];
        digitalWrite(relaypin[relay_pin], relay_status);

        sendRelayStatus(); // ส่งสถานะรีเลย์ทั้งหมด

    } else if (doc["status"] == "control3") {
        int relay_pin_index = doc["relay"][0];
        int relay_status = doc["relay"][1];
        int time = doc["relay"][2];
        unsigned long timedelay;

        Serial.println("Loop control 3 -----------------");
        if (relay_status == 1 && time == 0) {
            timedelay = ULONG_MAX - millis();
        } else {
            timedelay = time * 1000 * 60;
        }

        if (relay_pin_index >= 0 && relay_pin_index < arr_size) {
            int relay_pin = relaypin[relay_pin_index];
            digitalWrite(relay_pin, relay_status);

            // Update relay state
            relayStates[relay_pin_index].pinIndex = relay_pin_index;
            relayStates[relay_pin_index].status = relay_status;
            relayStates[relay_pin_index].endTime = millis() + timedelay;
            relayStates[relay_pin_index].active = true;
      
            //sendRelayStatus(); // ส่งสถานะรีเลย์ทั้งหมด //เข้าต้องการส่ง state เฉพาะกรณีตอบกลับเพียงอย่างเดียว

            Serial.print("Relay ");
            Serial.print(relay_pin_index);
            Serial.print(" (Pin ");
            Serial.print(relay_pin);
            Serial.print(") status set to ");
            Serial.print(relay_status);
            Serial.print(" for ");
            Serial.print(time);
            Serial.println(" minutes.");
        
      }
    } else if (doc["status"] == "reset") {
        Serial.println("!! Restart !!");
        ESP.restart();
    }
}

void sendRelayStatus() {
    StaticJsonDocument<256> doc;
    JsonArray relayArray = doc.createNestedArray("relay");

    Serial.println("sendRelay Status");
    for (int i = 0; i < arr_size; i++) {
        int status = digitalRead(relaypin[i]);
        relayArray.add(status);

        // พิมพ์ข้อมูลที่ถูกส่งไปยัง MQTT
        Serial.print("Relay ");
        Serial.print(i);
        Serial.print(" Status: ");
        Serial.println(status);
    }

    char buffer[256];
    size_t n = serializeJson(doc, buffer);

    mqtt.publish(MQTT_TOPIC_PUB, buffer, false);
}

void loop() {    
    if (mqtt.connected()) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastTempPublish >= tempPublishInterval) {
            lastTempPublish = currentMillis;

            float temp;
            temp = (temprature_sens_read() - 32) / 1.8;
            dtostrf(temp, 4, 2, ChValue);
            }      
        }
     else {
        Serial.println("Reconnecting to MQTT...");
        mqtt.connect(MQTT_NAME, MQTT_USERNAME, MQTT_PASSWORD);
            mqtt.subscribe(MQTT_TOPIC_SUB);
        }
    
    mqtt.loop();

    // Check relay states and turn off relays if time is up
    unsigned long currentMillis = millis();
    for (int i = 0; i < arr_size; i++) {
        if (relayStates[i].active && relayStates[i].status == HIGH) {      
            if (relayStates[i].endTime != ULONG_MAX && currentMillis >= relayStates[i].endTime) {
                digitalWrite(relaypin[relayStates[i].pinIndex], LOW);
                relayStates[i].active = false;
                Serial.print("Relay ");
                Serial.print(relayStates[i].pinIndex);
                Serial.print(" (Pin ");
                Serial.print(relaypin[relayStates[i].pinIndex]);
                Serial.println(") turned off.");
            sendRelayStatus();     
            }
        }
    }
    
}

