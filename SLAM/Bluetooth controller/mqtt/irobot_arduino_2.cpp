//#include <StandardCplusplus.h>

#include <ESP8266WiFi.h>

#include <stdio.h>

#include <PubSubClient.h>

#include <string.h>

//#include <set>



const char* ssid = "iRobot";

const char* password =  "medycyna";

const char* mqttServer = "192.168.250.55";

const int mqttPort = 1883;

const char* mqttUser = "user2";

const char* mqttPassword = "user2";





WiFiClient espClient;



PubSubClient client(espClient);



//set<int> control_cmds = {128,131,137,138,144,145,146};

//set<int> io_cmds = {142,148,149};




void setup() {

  pinMode(15, OUTPUT);

  pinMode(13, OUTPUT);

  digitalWrite(15, HIGH); //niebieska

  digitalWrite(13, LOW);  //zielona

  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {

    digitalWrite(15, HIGH); //niebieska

    delay(500);

    //Serial.println("Łączę do sieci WiFi..");

    digitalWrite(15, LOW);

  }

  //Serial.println("Podłączony do sieci WiFi");

  //digitalWrite(13, LOW);

  client.setServer(mqttServer, mqttPort);

  client.setCallback(callback);

  while (!client.connected()) {

    //Serial.println("Podłączenie do MQTT...");

    if (client.connect("ESP8266Client4", mqttUser, mqttPassword )) {

      //Serial.println("Podłączony");

      digitalWrite(13, HIGH);//zapal zieloną

    } else {

      //Serial.print("błąd: ");

      //Serial.print(client.state());

      digitalWrite(15, HIGH); //niebieska

      delay(1500);

      digitalWrite(15, LOW); //niebieska

    }

  }

  client.publish("irobot/log", "Pozdrowienia z iROBOTA nr 2");

  client.subscribe("irobot/2");

  digitalWrite(15, LOW);//zgaś niebieską

  //Serial.println("Start");

}





void callback(char* topic, byte* payload, unsigned int length) {

  //memcpy(dest, source, sizeof dest); to trzeba dodać

   digitalWrite(15, HIGH);

  int len_topic = strlen(topic);

  char int_str[20];



  char* topic_ = new char[len_topic];

  char* payload_ = new char[length];

  memcpy(topic_, topic, len_topic);

  memcpy(payload_, payload, length);

  client.publish("irobot/log", (String("topic_")+":"+String(topic_) + "|").c_str() );



  if (length==0) return;

  sprintf(int_str, "%d",  payload_[0]);

  client.publish("irobot/log", (String("cmd:")+int_str).c_str());



  if (String(topic_).substring(0,8)==String("irobot/2")) {

    byte cmd = payload_[0];

    client.publish("irobot/log", "topic irobot2 ---");



    if (cmd==128 || cmd==131 || cmd==137 || cmd==145) {

      String msg;

      char znak;

      client.publish("irobot/log", "control cmds");



      for (int i = 0; i < length; i++) {

        sprintf(int_str, "%d",  payload_[i]);

        client.publish("irobot/log", int_str);

        //Serial.write(payload_[i]);

        //Serial.print(payload_[i]);

        msg+=(char) payload_[i];

      }

      //Serial.write(payload_, length);

      Serial.write(msg.toInt());

      client.publish("irobot/log", msg.c_str());



    } else if (cmd==142 || cmd==148 || cmd==149) { //control_cmds.find(cmd) != control_cmds.end()

      client.publish("irobot/log", "control io");



    } else if (cmd==255) {

      Serial.write(128);

      Serial.write(131);

      delay(1000);

      Serial.write(173);

      client.publish("irobot/log", "restart ---");



    } else if (cmd==254) {

      Serial.write(137);

      Serial.write(255);

      Serial.write(56);

      Serial.write(1);

      Serial.write(244);

      client.publish("irobot/log", "run ---");





    }

  } else if (String(topic)=="irobot/log") {



  } else if (String(topic)=="irobot/2") {



  }









  //Serial.print("Przybyła wiadomość o topiku: ");



  //Serial.println(topic);





  //Serial.print("Wiadomość:");



  //client.publish("irobot/log", (String("topic")+String(topic)).c_str() );







  //client.publish("irobot/log", "odczytano:" );





  //client.publish("irobot/log", (char*) payload );


  //int a=kolor.toInt();

  //Serial.print(kolor);



  //Serial.println("stop");



//  if(kolor == "niebieska_on"){digitalWrite(15, HIGH);}



//  if(kolor == "niebieska_off"){digitalWrite(15, LOW);}



//  if(kolor == "czerwona_on"){digitalWrite(13, HIGH);}



//  if(kolor == "czerwona_off"){digitalWrite(13, LOW);}



//  if(kolor == "zolta_on"){digitalWrite(12, HIGH);}



//  if(kolor == "zolta_off"){digitalWrite(12, LOW);}







  //Serial.println("-----------------------");

  delay(10);

  digitalWrite(15, LOW);

  //digitalWrite(13, LOW);

  //digitalWrite(12, LOW);

}





void loop() {



  client.loop();



}