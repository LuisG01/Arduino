
#include <SPI.h>
#include <SD.h>
#include <GCS_MAVLink.h>
#include <SoftwareSerial.h>
const int chipSelect = 4;

SoftwareSerial Serial2(9,10); //RX, TX
#define _MavLinkSerial        Serial2
#define START                   1
#define MSG_RATE            10              // Hertz



// ******************************************

// Message #73  GLOBAL_POSITION_INT 
uint32_t   boot_time_ms = 0;
uint32_t   unix_time = 0;
int32_t    g_lat = 0;            
int32_t    g_lng = 0;            
int32_t    g_alt = 0; 
int32_t    g_rel_alt = 0;


// ******************************************
uint8_t     MavLink_Connected;
uint8_t     buf[MAVLINK_MAX_PACKET_LEN];

uint16_t  hb_count;

unsigned long MavLink_Connected_timer;
unsigned long hb_timer;

int aaaaa = 0;
mavlink_message_t msg;

// ******************************************

const int LED = 7;
const int SENSOR = 8;           // pin de conexion a sensor AVOID
int valor;                      // valor = 1 del sensor corresponde a 0 objetos detectados | valor = 0 corresponde a un objeto detectado
int valor_anterior = 1;         // iniciar el sistema SIN deteccion de objetos
int counter2 = 0;     // doble del contador real
int counter = 0;      //contador real
int flag = 0;

// ****************************************



void setup()  {

  _MavLinkSerial.begin(57600);
  //Serial.begin(9600);
  MavLink_Connected = 0;
  MavLink_Connected_timer=millis();
  hb_timer = millis();
  hb_count = 0;  
  pinMode(LED, OUTPUT);
  pinMode(SENSOR, INPUT);
  //analogReference(DEFAULT);
  if (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  //Serial.println("card initialized.");

}


// ******************************************
void loop()  {
    
  uint16_t len;
    if(millis()-hb_timer > 1500) {
        hb_timer=millis();
        if(!MavLink_Connected) {    // Start requesting data streams from MavLink
             mavlink_msg_request_data_stream_pack(0xFF,0xBE,&msg,1,1,MAV_DATA_STREAM_POSITION, MSG_RATE, START);
             len = mavlink_msg_to_send_buffer(buf, &msg);
             _MavLinkSerial.write(buf,len);
            delay(10);
            }
        }
  if((millis() - MavLink_Connected_timer) > 1500)  {   // if no HEARTBEAT from APM  in 1.5s then we are not connected
      MavLink_Connected=0;
      hb_count = 0;
      } 
  valor = digitalRead(SENSOR);  //entrega HIGH cuando NO detecta obstáculo
  digitalWrite(LED, !valor);
  if (valor_anterior != valor) {
    counter2 ++;
    if (flag == 0) {
      flag = 1;
       _MavLink_receive();                   // Check MavLink communication  
    } else {
      flag = 0;
      counter = counter2 / 2;
      /*Serial.print("Counter: ");
      Serial.println(counter);
      Serial.print("LAT: ");
      Serial.println(g_lat);
      Serial.print("LNG: ");
      Serial.println(g_lng);
      Serial.print("ALT: ");
      Serial.println(g_alt);
      Serial.print("ALT_REL: ");
      Serial.println(g_rel_alt);
      Serial.println(" ");
      Serial.println(" ");
      
      delay(100);*/
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      
      if (dataFile) {
        dataFile.print(counter);
        dataFile.print(",   ");;
        dataFile.print(g_lat);
        dataFile.print(",   ");
        dataFile.print(g_lng);
        dataFile.print(",   ");
        dataFile.print(g_alt);
        dataFile.print(",   ");
        dataFile.println(g_rel_alt);
        delay(10);
        dataFile.close();
        delay(100);
      }
      // if the file isn't open, pop up an error:
      else {
        //Serial.println("error opening datalog.txt");
      } 
    }
  }
  valor_anterior = valor;
      
}


void _MavLink_receive() { 
  
  mavlink_message_t msg;
  mavlink_status_t status;
	while(_MavLinkSerial.available())   
                { 
		            uint8_t c = _MavLinkSerial.read();
                if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) 
                      {
		      switch(msg.msgid) 
			    {
                              case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:   //  73
                                  g_lat = mavlink_msg_global_position_int_get_lat(&msg);
                                  g_lng = mavlink_msg_global_position_int_get_lon(&msg);
                                  g_alt = mavlink_msg_global_position_int_get_alt(&msg);
                                  g_rel_alt = mavlink_msg_global_position_int_get_relative_alt(&msg);
                                  break; 
                              default:
				  break;
			    }

		      }
                }
}


