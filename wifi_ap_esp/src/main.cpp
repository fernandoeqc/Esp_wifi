/**
   ESPNOW - Basic communication - Slave
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and a Slave ESP32
   Description: This sketch consists of the code for the Slave module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Slave >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.
*/
#include <Arduino.h>

#include <esp_now.h>
#include <WiFi.h>

#define CHANNEL 1

#define LED1 2
#define ERRO_TEMPO 25 //E1
#define ERRO_DADOS 26 //E2
#define BUT 27

unsigned long currentMillis;
unsigned long previousMillis;
static unsigned long counter = 0;

// Init ESP Now with fallback
void InitESPNow()
{
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK)
    {
        Serial.println("ESPNow Init Success");
    }
    else
    {
        Serial.println("ESPNow Init Failed");
        // Retry InitESPNow, add a counte and then restart?
        // InitESPNow();
        // or Simply Restart
        ESP.restart();
    }
}

// config AP SSID
void configDeviceAP()
{
    const char *SSID = "Slave_1";
    bool result = WiFi.softAP(SSID, "Slave_1_Password", CHANNEL, 0);
    if (!result)
    {
        Serial.println("AP Config failed.");
    }
    else
    {
        Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
    }
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    char macStr[18];
    unsigned char actual_data = 0;
    static unsigned char last_data = 0;
    static int error = 0;

    actual_data = *data;

    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    /* 
    Serial.print("Last Packet Recv from: ");
    Serial.println(macStr);
    Serial.print("Last Packet Recv Data: ");
    Serial.println(*data);
    Serial.println("");
 */
    if (actual_data != 0)
    {
        //FALHA NO RECEBIMENTO DO PACOTE
        if (actual_data - last_data != 1)
        {
            error++;

            digitalWrite(ERRO_DADOS, LOW);
            printf("erro: %d ####### ", error);
            // adicionar millis ao sd  ==== millis();
        }
        //Tudo certo
        else
        {
            digitalWrite(ERRO_DADOS, HIGH);
        }
    }

    last_data = actual_data;
    printf("actual: %d, last: %d. millis: %lu\n", actual_data, last_data, millis());

    counter = 0;
}

void setup()
{
    pinMode(LED1, OUTPUT);
    pinMode(ERRO_TEMPO, OUTPUT);
    pinMode(ERRO_DADOS, OUTPUT);
    pinMode(BUT, INPUT_PULLUP);

    digitalWrite(ERRO_TEMPO, LOW);
    digitalWrite(ERRO_DADOS, LOW);

    Serial.begin(115200);
    Serial.println("ESPNow/Basic/Slave Example");
    //Set device in AP mode to begin with
    WiFi.mode(WIFI_AP);
    // configure device AP mode
    configDeviceAP();
    // This is the mac address of the Slave in AP Mode
    Serial.print("AP MAC: ");
    Serial.println(WiFi.softAPmacAddress());
    // Init ESPNow with a fallback logic
    InitESPNow();
    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info.
    esp_now_register_recv_cb(OnDataRecv);
}

bool test_but = 1;

void loop()
{
    

    currentMillis = millis();

    if (currentMillis - previousMillis >= 4000)
    {
        previousMillis = currentMillis;

        counter++;

        if (counter > 4)
        {
            Serial.printf("Perda de comunicacao, millis: %lu\n", millis());
            digitalWrite(LED1, HIGH);
            digitalWrite(ERRO_TEMPO, LOW);
        }
        else
        {
            digitalWrite(LED1, LOW);
            digitalWrite(ERRO_TEMPO, HIGH);
        }

        if(!digitalRead(BUT))
        {
            if(test_but)
            {
                test_but = 0;
                digitalWrite(LED1, HIGH);
                digitalWrite(ERRO_TEMPO, LOW);
                digitalWrite(ERRO_DADOS, LOW);
            }
        }
        else test_but = 1;
    }
}
