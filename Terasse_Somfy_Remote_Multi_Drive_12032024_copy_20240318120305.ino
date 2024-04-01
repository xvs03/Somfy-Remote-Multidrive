// Projekt: Somfy Antriebssteuerung mit CC1101 Modul und ESP8266 D1 mini

/* LOG:
 * 24.06.22 else if defenition für MQTT Befehle für Antrieb 1 (testweise)
 * 22.06.22 MQTT Befehle definiert
 * 14.06.22 ESP in das WLAN einbinden ; MQTT Topics erstellen
 * 14.06.22 Kommandos "Up","Down","My" und "Prog" für Antriebe 1-4 eingepflegt 
 * 02.06.22 Erweiterung der Kommandos (prüfen auf Swtch Case)| Kommando "Down" für Windshot SÜD/RECHTS, Drive Number 1 eingepflegt.
 * 02.04.22 Alle Antriebe (1-4) mit der Antriebsvorwahl und dem Befehl "Up" bedienbar. Rolling Code wird richtig auf EEPROM gespeichert
 * 
 * Nächster Arbeitsschritt: Die restlichen Befehle einbinden.
 * 
 * 
 * 
 * 
 */

/* ####################################################
 *  PIN SET
 *  ESP8266:        CC1101:
 *  
 *  3,3 V  ------   VCC
 *  GND    ------   GND
 *  D7     ------   MOSI
 *  D6     ------   MISO
 *  D5     ------   SCK
 *  D2     ------   GD02
 *  D1     ------   GD00
 *  D8     ------   CSN
 *  
 *  
 *  >>> Available commands over IDE Input<<<
 *  
 * Name     Description                                       HEX code
 * 
 * My       The My button pressed                             1
 * Up       The Up button pressed                             2
 * MyUp     The My and Up button pressed at the same time     3
 * Down     The Down button pressed                           4
 * MyDown   The My and Down button pressed at the same time   5
 * UpDown   The Up and Down button pressed at the same time   6
 * Prog     The Prog button pressed                           8
 * SunFlag  Enable sun and wind detector                      9
 * Flag     Disable sun detector                              A
 * 
 * 
 * >>> Available commands over MQTT<<<
 * 
 * Drive 1:         MQTT Command in "String" Format recive
 * 
 * Kein Kommando    00
 * 
 * My (Stopp)       1m
 * Up               1u
 * Down             1d
 * Prog             1p
 * 
 * 
 * Drive 2:
 * 
 * My (Stopp)       2m
 * Up               2u
 * Down             2d
 * Prog             2p
 * 
 * 
 * Drive 3:
 * 
 * My (Stopp)       3m
 * Up               3u
 * Down             3d
 * Prog             3p
 * 
 * 
 * Drive 4/(5):
 * 
 * My (Stopp)       4m
 * Up               4u
 * Down             4d
 * Prog             4p
 * 
 * Nach jedem Befehlsempfang wird die angewählte Drive Number auf 0=neutral gesetzt.
 * 
 * #############################
 * 
 * Sicht aus Wohnzimmerfenster !!!
 * 
 * Drive 0 = kein Antrieb vorgewählt
 * Drive 1 = Windshot  SÜD/RECHTS
 * Drive 2 = Windshot  SÜD/LINKS
 * Drive 3 = Windshot  WEST
 * Drive 4/(5) = Sonnenschutz OBEN/LINKS+RECHTS
 * 
 * 
 * Rolling Code Speicher:
 * 
 * EEPROM_ADRESS 0 = Windshot SÜD/RECHTS
 * EEPROM_ADRESS 2 = Windshot SÜD/LINKS
 * EEPROM_ADRESS 4 = Windshot WEST
 * EEPROM_ADRESS 6 = Sonnenschutz OBEN/RECHTS
 * EEPROM_ADRESS 8 = Sonnenschutz OBEN/LINKS
 */


#include "credentials.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
//#include <ArduinoOTA.h>

#include <EEPROM.h>
#include <EEPROMRollingCodeStorage.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <SomfyRemote.h>

#define DEBUG

#define EMITTER_GPIO 2  // ESP8266 vs. ESP32 different!!!
//#define EEPROM_ADDRESS 0 // Speicher Adresse des fortlaufenden Rollingcodes

//#define REMOTE 0x5184c8 // Code der Fernbedienung  Antriebsnummer 1   = Windshot SÜD/RECHTS
#define REMOTE1 0x5184c8 // Code der Fernbedienung  Antriebsnummer 1   = Windshot SÜD/RECHTS
#define REMOTE2 0x65dc00 // Code der Fernbedienung  Antriebsnummer 2   = Windshot SÜD/LINKS
#define REMOTE3 0x25b5d5 // Code der Fernbedienung  Antriebsnummer 3   = Windshot WEST
#define REMOTE4 0xc6c78f // Code der Fernbedienung  Antriebsnummer 4/(5) = Sonnenschutz OBEN|LINKS/RECHTS Zwillingsbetrieb

#define CC1101_FREQUENCY 433.42

int DriveNumber = 0;   // Prefix Drive Code 1-4 | 0 = no Drive select
bool frame1x = false; // Frame Antriebsauswahl nur 1x anzeigen



// EEPROM Rolling Code Storage für alle Remotes

EEPROMRollingCodeStorage rollingCodeStorage1(0); // Antriebsnummer 1
EEPROMRollingCodeStorage rollingCodeStorage2(2); // Antriebsnummer 2
EEPROMRollingCodeStorage rollingCodeStorage3(4); // Antriebsnummer 3
EEPROMRollingCodeStorage rollingCodeStorage4(6); // Antriebsnummer 4/5


// Zuordnung REMOTE's > Rolling Code Storage

SomfyRemote somfyRemote1(EMITTER_GPIO, REMOTE1, &rollingCodeStorage1);
SomfyRemote somfyRemote2(EMITTER_GPIO, REMOTE2, &rollingCodeStorage2);
SomfyRemote somfyRemote3(EMITTER_GPIO, REMOTE3, &rollingCodeStorage3);
SomfyRemote somfyRemote4(EMITTER_GPIO, REMOTE4, &rollingCodeStorage4);


//EEPROMRollingCodeStorage rollingCodeStorage(EEPROM_ADDRESS);
//SomfyRemote somfyRemote(EMITTER_GPIO, REMOTE, &rollingCodeStorage);

// WLAN und MQTT
// Daten FritzBOX aus "credentials.h"

const char* ssid = networkSSID;
const char* password = networkPASSWORD;


// Daten openHAB Server aus "credentials.h"

const char* mqttServer = mqttSERVER;
const char* mqttUsername = mqttUSERNAME;
const char* mqttPassword = thingKEY;
const char* mqttDeviceId = thingID;


// Topic's Puplish Variablen     + bei Topic in OH3 Channel eintragen +
                                                                                              
char varWlanSignalTopic[] =  "Somfy_espRC_Terrasse/WLAN_Signal";                     

// Topic's Subscribe

// Kommando Empfang für jeden Antrieb
char varAntriebsKommandoFrameTopic[] = "Somfy_espRC_Terrasse/AntriebsKommandoFrame"; // Antriebskommando Empfang für alle Antriebe über Kommando ID
char varAntriebsKommandoFrameTopic_1[] = "Somfy_espRC_Terrasse/AntriebsKommandoFrame/Antrieb_1"; // Antriebskommando Empfang
char varAntriebsKommandoFrameTopic_2[] = "Somfy_espRC_Terrasse/AntriebsKommandoFrame/Antrieb_2"; // Antriebskommando Empfang
char varAntriebsKommandoFrameTopic_3[] = "Somfy_espRC_Terrasse/AntriebsKommandoFrame/Antrieb_3"; // Antriebskommando Empfang
char varAntriebsKommandoFrameTopic_4_5[] = "Somfy_espRC_Terrasse/AntriebsKommandoFrame/Antrieb_4_5"; // Antriebskommando Empfang


// Kommando Rückmeldung für jeden Antrieb
char varAntriebsKommandoResponse[]= "Somfy_espRC_Terrasse/AntriebsKommandoResponse"; // Antriebskommando Response (Feedback)
char varAntriebsKommandoResponse_1[]= "Somfy_espRC_Terrasse/AntriebsKommandoResponse/Antrieb_1"; // Antriebskommando Response (Feedback)
char varAntriebsKommandoResponse_2[]= "Somfy_espRC_Terrasse/AntriebsKommandoResponse/Antrieb_2"; // Antriebskommando Response (Feedback)
char varAntriebsKommandoResponse_3[]= "Somfy_espRC_Terrasse/AntriebsKommandoResponse/Antrieb_3"; // Antriebskommando Response (Feedback)
char varAntriebsKommandoResponse_4_5[]= "Somfy_espRC_Terrasse/AntriebsKommandoResponse/Antrieb_4_5"; // Antriebskommando Response (Feedback)
//char RollingKommandoTopic[] = "Somfy_espRC_Terrasse/RollingKommando"; // Rollkommando Down,Up,My (Stopp),Prog



// LAST WILL TESTAMENT
const char willTopic[] =  "Somfy_espRC_Terrasse/LastWill";
int willQoS = 0;
bool willRetain = true;
const char willMessage[] = "OFFLINE";
const char willMessageOn[] = "ONLINE";


WiFiClient espClient;  
PubSubClient client(espClient);

// Variablen für ESP Infos

int wlanSig = 0; // WLAN Siegnalstärke init
bool connectTry = false; // Status WLAN reconnect

// Variable für MQTT Kommando

String mqttCommand = "00"; // MQTT Kommando


// Variablen für Zeitmaschine

long now = millis(); // Variable füer Zeitmaschine Sendetimer MQTT Server
// long now2 = millis(); // Reserve

const int interval = 5000; // Zeit in ms für Sendeintervall MQTT Daten
long lastMsg = 0; // Differenmerker in ms, wann letzte MQTT Daten gesendet wurden


//  ### FUNKTIONEN ###

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
} 


// Aufruf bei Änderung des abonierten Topic´s "Antriebsnummer" Vorwahl

void callback(char* varAntriebsKommandoFrameTopic, byte* message, unsigned int length) {
  Serial.println(" Message arrived on topic: ");
  Serial.println(varAntriebsKommandoFrameTopic);  // Antriebskommando MQTT
  Serial.print("Message: ");
  String value = ""; // Leersetzen des Variableninhalts (sonst Pufferüberlauf durch die += Funktion)
  
// Scheife zum zusammensetzen der Message
  for (int i = 0; i < length; i++) {
   Serial.print((char)message[i]); // Ausgabe der Message
   value  += ((char)message[i]); // Konvertierung von Char zu String (vorher String leerschreiben)
   }

   mqttCommand = value; // String MQTT Kommando
  }




  void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    // Create a random client ID
    //clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(mqttDeviceId, mqttUsername, mqttPassword, willTopic, willQoS, willRetain, willMessage)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish(outTopic, ESPHostname);
      // ... and resubscribe
      client.subscribe(varAntriebsKommandoFrameTopic);
      Serial.println("KommandoFrameTopic wird aboniert");
    } else {
        Serial.println("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
   }
}

// Funktion Klartext Antriebsbeschreibung nach Auswahl

void driveClearText(){
  if (DriveNumber == 1){
  Serial.print(" Windshot SÜD/RECHTS");
  }
  if (DriveNumber == 2){
  Serial.print(" Windshot SÜD/LINKS");
  }
  if (DriveNumber == 3){
  Serial.print(" Windshot WEST");
  }
  if (DriveNumber == 4){
  Serial.print(" Sonnenschutz OBEN|LINKS/RECHTS");
  }
}

// Funktion Kommando an vorgewählten Antrieb senden und Rolling Code (Fortlaufende Nummer | Abweichung darf nicht größer 3 sein, sonst Antrieb neu an Remote anlernen) speichern
void sendCC1101Command(Command command) {
  // Für Antrieb/Remote Nr.: 1
  if (DriveNumber == 1){
  ELECHOUSE_cc1101.SetTx();
  somfyRemote1.sendCommand(command);
  ELECHOUSE_cc1101.setSidle();
  delay(100);
  Serial.println("Kommando von Remote 1 an Antrieb 1 gesendet !");
  }
  // Für Antrieb/Remote Nr.: 2
  if (DriveNumber == 2){
  ELECHOUSE_cc1101.SetTx();
  somfyRemote2.sendCommand(command);
  ELECHOUSE_cc1101.setSidle();
  delay(100);
  Serial.println("Kommando von Remote 2 an Antrieb 2 gesendet !");
  }
  // Für Antrieb/Remote Nr.: 3
  if (DriveNumber == 3){
  ELECHOUSE_cc1101.SetTx();
  somfyRemote3.sendCommand(command);
  ELECHOUSE_cc1101.setSidle();
  delay(100);
  Serial.println("Kommando von Remote 3 an Antrieb 3 gesendet !");
  }
  // Für Antrieb/Remote Nr.: 4/5
  if (DriveNumber == 4){
  ELECHOUSE_cc1101.SetTx();
  somfyRemote4.sendCommand(command);
  ELECHOUSE_cc1101.setSidle();
  delay(100);
  Serial.println("Kommando von Remote 4 an Antrieb 4 gesendet !");
  }
}


// ### SETUP ###

void setup() {
  Serial.begin(115200);
  // Kommunikation over WiFI and MQTT
  setup_wifi(); 
  delay(500);
  client.setServer(mqttSERVER, 1883); 
  client.setCallback(callback); 

  
  //somfyRemote.setup();
  pinMode(EMITTER_GPIO, OUTPUT);
  digitalWrite(EMITTER_GPIO, LOW);
  delay(500);
  ELECHOUSE_cc1101.Init();
  Serial.println("Initialisiere CC1101 Transmitter");
  delay(500);
  ELECHOUSE_cc1101.setMHZ(CC1101_FREQUENCY);
  Serial.print("Setze die Sendefrequenz auf ");
  Serial.print(CC1101_FREQUENCY);
  Serial.println(" MHz");



#if defined(ESP32)
  if (!EEPROM.begin(8)) {
    Serial.println("failed to EEPROM ESP32");
    delay(1000);
  }
#elif defined(ESP8266)
    EEPROM.begin(8); // Anzahl der benötigten Speicherbytes 8
    Serial.println("Erkenne ESP8266, reserviere EEPROM Speicher");
#endif
Serial.print("\n");
Serial.println("Wähle die Antriebsnummer (1-4)");
} // SETUP END


                  // LOOP
void loop() {

  // MQTT
  if (!client.connected()) {  
    reconnect();  
  }
  client.loop();

  wlanSig = WiFi.RSSI(); // WLAN Signal ermitteln

  // Sendeintervall für Daten MQTT (Zeitmaschine)

  now = millis(); // Zeitstempel 

  if ((now - lastMsg > interval)) {
       lastMsg = now;

  // Wieder ONLINE an MQTT Server

  
  client.publish(willTopic, willMessageOn, willRetain);   // Topic für Last Will Message: "Client wieder ONLINE" 


// WLAN Signalstärke senden an MQTT Server (OH)
                     
  client.publish(varWlanSignalTopic, String(wlanSig).c_str());    
  client.publish(varAntriebsKommandoResponse, String(mqttCommand).c_str()); // Sende Status MQTT Kommando alle 5s       

  } // Sendeintervall ENDE

  



  
// ### Antriebsvorwahl über Serial Monitor ### 
   
  // Antriebsnummer im Serial Monitor eingeben (1-4) !
  if ((Serial.available() > 0) && (DriveNumber == 0)) {
    DriveNumber = Serial.parseInt();
    const String string = Serial.readStringUntil('\n');
  }
    

  // Wenn Antriebsnummer 1-4 gewählt...
  if ((DriveNumber > 0) && (frame1x == false)) {
    frame1x = true;
    Serial.print("Du hast Antriebsnummer ");
    Serial.print(DriveNumber);
    driveClearText();
    Serial.println(" angewählt");
    Serial.println("Erwarte Fahrbefehl: Up, Down, My =(Stopp) oder Prog ! ");    
  }
          
// Rolling Befehlserkennung Antrieb 1 (Windshot SÜD/RECHTS) "Up" , "Down" , "My" (Stopp) oder "Prog" über Serial Monitor oder MQTT
          
          // Rolling Befehl für Antrieb 1 über Serial Monitor erwartet..., aber nur wenn kein MQTT Befehl anliegt.
       if ((Serial.available() > 0) && (DriveNumber == 1) && (mqttCommand == "00")){
          const String string = Serial.readStringUntil('\n');
          
          // Bei erkennen des rolling Kommandos "Up" für Antrieb 1
          if (string == "Up") {
            Serial.println("|Up| Befehl erkannt, Windshot SÜD/RECHTS sollte HOCH fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }

          // Bei erkennen des rolling Kommandos "Down" für Antrieb 1
          if (string == "Down") {
            Serial.println("|Down| Befehl erkannt, Windshot SÜD/RECHTS sollte RUNTER fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }

          // Bei erkennen des rolling Kommandos "My" (Stopp) für Antrieb 1
          if (string == "My") {
            Serial.println("|My (Stopp)| Befehl erkannt, Windshot SÜD/RECHTS sollte STOPPEN");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }

          // Bei erkennen des Kommandos "Prog" für Antrieb 1
          if (string == "Prog") {
            Serial.println("|Prog| Befehl erkannt, Windshot SÜD/RECHTS wird an RC angelernt");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }

// Sendeprodzedur über Serial Monitor beenden Antriebsvorwahl von 1 auf 0 setzen
            DriveNumber = 0;
            //mqttCommand = "00"; // MQTT Kommando default
            Serial.println("Antrieb Windshot SÜD/RECHTS wieder abgewählt !");
            Serial.print("\n");
            //Serial.println("MQTT Komandostring wieder auf 00 (default) gesetzt");
            //Serial.print("\n");
            frame1x = false;
            #ifdef DEBUG
            Serial.println("Senden an Antrieb beendet !");
            Serial.print("\n");
            Serial.println("Du kannst jetzt wieder einen Antrieb wählen (1-4) oder MQTT Befehl geben");
            #endif
            
 } // Befehlsabarbeitung für Antrieb 1 Serial Monitor  


                                                  
                                                          // ### MQTT Befehlsauswertung ###



// ### Antrieb 1 ###
                                                          
// MQTT Befehlserkennung "1u" = "Up" für Antrieb 1                                                      
                                                                                                         
        if (mqttCommand != "00"){                                                                
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "1u" für Antrieb 1
          if (mqttCommand == "1u"){
            DriveNumber = 1; // Setzen der Antriebsnummer über MQTT String
            const String string = "Up";
            Serial.print("\n");
            Serial.println("|Up|MQTT Befehl erkannt, Windshot SÜD/RECHTS sollte HOCH fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_1, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          }

// MQTT Befehlserkennung "1d" = "Down" für Antrieb 1                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "1u" für Antrieb 1
          if (mqttCommand == "1d"){
            DriveNumber = 1; // Setzen der Antriebsnummer über MQTT String
            const String string = "Down";
            Serial.print("\n");
            Serial.println("|Down|MQTT Befehl erkannt, Windshot SÜD/RECHTS sollte RUNTER fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_1, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          }         

// MQTT Befehlserkennung "1m" = "My (Stopp)" für Antrieb 1                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "1m" für Antrieb 1
          if (mqttCommand == "1m"){
            DriveNumber = 1; // Setzen der Antriebsnummer über MQTT String
            const String string = "My";
            Serial.print("\n");
            Serial.println("|Stopp|MQTT Befehl erkannt, Windshot SÜD/RECHTS sollte STOPPEN ");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_1, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          }   

// MQTT Befehlserkennung "1p" = "Prog" für Antrieb 1                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "1m" für Antrieb 1
          if (mqttCommand == "1p"){
            DriveNumber = 1; // Setzen der Antriebsnummer über MQTT String
            const String string = "Prog";
            Serial.print("\n");
            Serial.println("|Prog|MQTT Befehl erkannt, Windshot SÜD/RECHTS sollte zum programmieren bereit sein ");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_1, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          }   

// ### Antrieb 2 ###

// MQTT Befehlserkennung "2u" = "Up" für Antrieb 2                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "2u" für Antrieb 2
          if (mqttCommand == "2u"){
            DriveNumber = 2; // Setzen der Antriebsnummer über MQTT String
            const String string = "Up";
            Serial.print("\n");
            Serial.println("|Prog|MQTT Befehl erkannt, Windshot SÜD/LINKS sollte hochfahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_2, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          } 

 // MQTT Befehlserkennung "2d" = "Down" für Antrieb 2                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "2d" für Antrieb 2
          if (mqttCommand == "2d"){
            DriveNumber = 2; // Setzen der Antriebsnummer über MQTT String
            const String string = "Down";
            Serial.print("\n");
            Serial.println("|Down|MQTT Befehl erkannt, Windshot SÜD/LINKS sollte runter fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_2, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          }     

  // MQTT Befehlserkennung "2m" = "My (Stopp)" für Antrieb 2                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "2m" für Antrieb 2
          if (mqttCommand == "2m"){
            DriveNumber = 2; // Setzen der Antriebsnummer über MQTT String
            const String string = "My";
            Serial.print("\n");
            Serial.println("|My|MQTT Befehl erkannt, Windshot SÜD/LINKS sollte stoppen");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_2, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          }  

  // MQTT Befehlserkennung "2p" = "Prog" für Antrieb 2                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "2p" für Antrieb 2
          if (mqttCommand == "2p"){
            DriveNumber = 2; // Setzen der Antriebsnummer über MQTT String
            const String string = "Prog";
            Serial.print("\n");
            Serial.println("|Prog|MQTT Befehl erkannt, Windshot SÜD/LINKS ");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_2, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          } 


// ### Antrieb 3 ###

// MQTT Befehlserkennung "3u" = "Up" für Antrieb 3                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "3u" für Antrieb 3
          if (mqttCommand == "3u"){
            DriveNumber = 3; // Setzen der Antriebsnummer über MQTT String
            const String string = "Up";
            Serial.print("\n");
            Serial.println("|Prog|MQTT Befehl erkannt, Windshot WEST sollte hochfahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_3, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          } 

 // MQTT Befehlserkennung "3d" = "Down" für Antrieb 3                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "3d" für Antrieb 3
          if (mqttCommand == "3d"){
            DriveNumber = 3; // Setzen der Antriebsnummer über MQTT String
            const String string = "Down";
            Serial.print("\n");
            Serial.println("|Down|MQTT Befehl erkannt, Windshot WEST sollte runter fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_3, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          }     

  // MQTT Befehlserkennung "3m" = "My (Stopp)" für Antrieb 3                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "3m" für Antrieb 3
          if (mqttCommand == "3m"){
            DriveNumber = 3; // Setzen der Antriebsnummer über MQTT String
            const String string = "My";
            Serial.print("\n");
            Serial.println("|My|MQTT Befehl erkannt, Windshot WEST sollte stoppen");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_3, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          }  

  // MQTT Befehlserkennung "3p" = "Prog" für Antrieb 3                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "3p" für Antrieb 3
          if (mqttCommand == "3p"){
            DriveNumber = 3; // Setzen der Antriebsnummer über MQTT String
            const String string = "Prog";
            Serial.print("\n");
            Serial.println("|Prog|MQTT Befehl erkannt, Windshot");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_3, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          } 



          // ### Antrieb 4/(5) ###

// MQTT Befehlserkennung "4u" = "Up" für Antrieb 4/(5)                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "4u" für Antrieb 4/(5)
          if (mqttCommand == "4u"){
            DriveNumber = 4; // Setzen der Antriebsnummer über MQTT String
            const String string = "Up";
            Serial.print("\n");
            Serial.println("|Prog|MQTT Befehl erkannt, Sonnenschutz sollte auffahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_4_5, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          } 

 // MQTT Befehlserkennung "4d" = "Down" für Antrieb 4/(5)                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "4d" für Antrieb 4/(5)
          if (mqttCommand == "4d"){
            DriveNumber = 4; // Setzen der Antriebsnummer über MQTT String
            const String string = "Down";
            Serial.print("\n");
            Serial.println("|Down|MQTT Befehl erkannt, Sonnenschutz sollte zu fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_4_5, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          }     

  // MQTT Befehlserkennung "4m" = "My (Stopp)" für Antrieb 4/(5)                                                      
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "4m" für Antrieb 4/(5)
          if (mqttCommand == "4m"){
            DriveNumber = 4; // Setzen der Antriebsnummer über MQTT String
            const String string = "My";
            Serial.print("\n");
            Serial.println("|My|MQTT Befehl erkannt, Sonnenschutz sollte stoppen");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_4_5, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          }  

  // MQTT Befehlserkennung "4p" = "Prog" für Antrieb 4/(5)                                                     
                                                                                                         
                                                                            
                                                                                                          
          // Bei Erkennen des MQTT Kommandos "4p" für Antrieb 4/(5)
          if (mqttCommand == "4p"){
            DriveNumber = 4; // Setzen der Antriebsnummer über MQTT String
            const String string = "Prog";
            Serial.print("\n");
            Serial.println("|Prog|MQTT Befehl erkannt für Sonnenschutz ");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);      // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            sendCC1101Command(command);                           // Erstmal Ausblenden wegen Rollingcode Indifferenzen
            //Serial.println("Hätte funktioniert, wenn das Senden des Kommandoframes nicht ausgeblendet wäre :-)");
            Serial.print("\n");
            client.publish(varAntriebsKommandoResponse_4_5, String(mqttCommand).c_str());  // Sende Status MQTT Kommando
            delay(100); // kleines Päuschen
          } 

// Sendeprodzedur MQTT beenden Antriebsvorwahl von X auf 0 setzen
            DriveNumber = 0;
            mqttCommand = "00"; // MQTT Kommando default
            client.publish(varAntriebsKommandoResponse, String(mqttCommand).c_str()); // Sende Status MQTT Kommando
            Serial.print("\n");
            Serial.println("Message MQTT: ");
            Serial.println(mqttCommand);
            Serial.print("\n");
            Serial.println("Antrieb wieder abgewählt !");
            Serial.print("\n");
            Serial.println("MQTT Komandostring wieder auf 00 (default) gesetzt");
            Serial.print("\n");
            frame1x = false;
            #ifdef DEBUG
            Serial.println("Senden an Antrieb beendet !");
            Serial.print("\n");
            Serial.println("Erwarte neuen Rollingbefehl !");
            #endif
     } // ### MQTT Befehlsabarbeitung ENDE ###



     

// Rolling Befehlserkennung Antrieb 2 (Windshot SÜD/LINKS) "Up" , "Down" , "My" (Stopp).

          // Rolling Befehl für Antrieb 2 erwartet...
       if ((Serial.available() > 0) && (DriveNumber == 2)){
          const String string = Serial.readStringUntil('\n');
          
          // Bei erkennen des rolling Kommandos "Up" für Antrieb 2
          if (string == "Up") {
            Serial.println("|Up| Befehl erkannt, Windshot SÜD/LINKS sollte HOCH fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }

          // Bei erkennen des rolling Kommandos "Down" für Antrieb 2
          if (string == "Down") {
            Serial.println("|Down| Befehl erkannt, Windshot SÜD/LINKS sollte RUNTER fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }

          // Bei erkennen des rolling Kommandos "My" für Antrieb 2
          if (string == "My") {
            Serial.println("|My (Stopp)| Befehl erkannt, Windshot SÜD/LINKS sollte STOPPEN");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }


          // Bei erkennen des Kommandos "Prog" für Antrieb 2
          if (string == "Prog") {
            Serial.println("|Prog| Befehl erkannt, Windshot SÜD/LINKS wird an RC angelernt");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }
          
          // Sendeprodzedur beenden Antriebsvorwahl von 2 auf 0 setzen
            DriveNumber = 0;
            Serial.println("Antrieb Windshot SÜD/LINKS wieder abgewählt !");
            frame1x = false;
            #ifdef DEBUG
            Serial.println("Senden an Antrieb beendet !");
            Serial.print("\n");
            Serial.println("Du kannst jetzt wieder einen Antrieb wählen (1-4)");
            #endif
     }

   
// Rolling Befehlserkennung Antrieb 3 (Windshot WEST) "Up" , "Down" , "My" (Stopp). 
          
          // Rolling Befehl für Antrieb 3 erwartet...
       if ((Serial.available() > 0) && (DriveNumber == 3)){
          const String string = Serial.readStringUntil('\n');
          
          // Bei erkennen des rolling Kommandos "Up" für Anrieb 3
          if (string == "Up") {
            Serial.println("|Up| Befehl erkannt, Windshot WEST sollte HOCH fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }

          // Bei erkennen des rolling Kommandos "Down" für Anrieb 3
          if (string == "Down") {
            Serial.println("|Down| Befehl erkannt, Windshot WEST sollte RUNTER fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }

          // Bei erkennen des rolling Kommandos "My" (Stopp) für Anrieb 3
          if (string == "My") {
            Serial.println("|My (Stopp)| Befehl erkannt, Windshot WEST sollte STOPPEN");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }


          // Bei erkennen des Kommandos "Prog" für Antrieb 3
          if (string == "Prog") {
            Serial.println("|Prog| Befehl erkannt, Windshot WEST wird an RC angelernt");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }
          
            // Sendeprodzedur beenden Antriebsvorwahl von 3 auf 0 setzen  
            DriveNumber = 0;
            Serial.println("Antrieb Windshot WEST wieder abgewählt !");
            frame1x = false;
            #ifdef DEBUG
            Serial.println("Senden an Antrieb beendet !");
            Serial.print("\n");
            Serial.println("Du kannst jetzt wieder einen Antrieb wählen (1-4)");
            #endif
     }

// Rolling Befehlserkennung Antrieb 4/(5) (Sonnenschutz) "Up" , "Down" , "My" (Stopp).
   
          // Rolling Befehl für Antrieb 4/(5) erwartet...
       if ((Serial.available() > 0) && (DriveNumber == 4)){
          const String string = Serial.readStringUntil('\n');
          
          // Bei erkennen des rolling Kommandos "Up" für Antrie 4/(5)
          if (string == "Up") {
            Serial.println("|Up| Befehl erkannt, Sonnenschutz OBEN|LINKS/RECHTS sollte AUF fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }

          // Bei erkennen des rolling Kommandos "Down" für Antrieb 4/(5) 
          if (string == "Down") {
            Serial.println("|Down| Befehl erkannt, Sonnenschutz OBEN|LINKS/RECHTS sollte ZU fahren");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }

          
          // Bei erkennen des rolling Kommandos "My" (Stopp) für Antrieb 4/(5) 
          if (string == "My") {
            Serial.println("|My (Stopp)| Befehl erkannt, Sonnenschutz OBEN|LINKS/RECHTS sollte STOPPEN");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }

          // Bei erkennen des Kommandos "Prog" für Antriebe 4/(5)
          if (string == "Prog") {
            Serial.println("|Prog| Befehl erkannt, Windshot Sonnenschutz OBEN|LINKS/RECHTS wird an RC angelernt");
            Serial.print("\n");
            Serial.println("Sende folgenden Rollingcode und Datenframe an den Antrieb");
            Serial.print("\n");
            const Command command = getSomfyCommand(string);
            sendCC1101Command(command);
          }
          
            // Sendeprodzedur beenden Antriebsvorwahl von 4/(5) auf 0 setzen
            DriveNumber = 0;
            Serial.println("Antriebe Sonnenschutz wieder abgewählt !");
            frame1x = false;
            #ifdef DEBUG
            Serial.println("Senden an Antrieb beendet !");
            Serial.print("\n");
            Serial.println("Du kannst jetzt wieder einen Antrieb wählen (1-4)");
            #endif
     }
   
} // LOOP END
