/*
 * Project MH-Z16-CO2
 * Description: Seeed Studio MH-Z16 NDIR CO2 Sensor 0-5000ppm, 8-bit (0-255), UV Sensor, and BME280 Temp, Press, Humidity
 * Author:  PJB
 * Date:  05-16-20 begin date
 */

#include "Particle.h"
#include "math.h"
#include "JsonParserGeneratorRK.h"

SYSTEM_THREAD(ENABLED);

#define UPDATE_INTERVAL 10000  //1 sec = 1000 millis

int min_time, min_last;
int CO2, CO2TC;
float UVAP, UVind;
double UVAP_d, UVind_d;
unsigned long UpdateInterval;

const unsigned char cmd_get_sensor[] = {0xff,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};

const unsigned char cmd_calibratezero[] = {0xff,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78};

const unsigned char cmd_calibratespan[] = {0xff,0x01,0x88,0x07,0xD0,0x00,0x00,0x00,0xA0};

SerialLogHandler logHandler(LOG_LEVEL_INFO);

void setup()
{ 
  //Particle.variable("CO2ppm", CO2);
  //Particle.variable("CO2TempC", CO2TC);
  //Particle.variable("UVAPower", UVAP_d);
  //Particle.variable("UVIndex", UVind_d);
  delay(100);
  
  pinMode(A0, INPUT);

  Serial1.begin(9600,SERIAL_8N1);
  Serial1.setTimeout(500);             // Wait up to 500ms for the data to return in full
  delay(100);

  Serial.begin(9600);
  delay(100);

  //Serial1.write(cmd_calibratezero, sizeof(cmd_calibratezero));
  //Serial1.write(cmd_calibratespan, sizeof(cmd_calibratespan));

  Log.info("System version: %s", (const char*)System.version());
  Log.info("Setup Complete");
  UpdateInterval = millis();
  min_last=Time.minute()-1;
}

void loop()
{
  Time.zone(-7);

  if(millis() - UpdateInterval > UPDATE_INTERVAL)
  {
    if(GetCO2(CO2, CO2TC)) Log.info("CO2(ppm): %d, CO2-T(C): %d", CO2, CO2TC);

    getUVAPower(UVAP);
    Log.info("UVAPower(mW/cm2): %f", UVAP);
    UVind = ((113.0f*UVAP)-83.0f)/21.0f;
    Log.info("UVIndex: %f", UVind);
    UVAP_d=double(UVAP);
    UVind_d=double(UVind); 
    
    min_time=Time.minute();
    if((min_time!=min_last)&&(min_time==0||min_time==10||min_time==20||min_time==30||min_time==40||min_time==50))
    {
      createEventPayload(CO2, CO2TC, UVAP, UVind);
      min_last = min_time;
      Log.info("Last Update: %d", min_last);
    }
    UpdateInterval = millis();
  }
}

bool GetCO2(int& CO2, int& CO2TC)
{
  byte data[9];
  char chksum = 0x00;
  int len = -1, ntry=0, nmtry=10, i =0;
  while((len < 9 && ntry < nmtry))
    {
    Serial1.flush();                                                             // Flush TX buffer
    while(Serial1.read() >= 0);                                                  // Flush RX buffer
    Serial1.write(cmd_get_sensor, 9);
    for(uint32_t msTimeout=millis(); (len=Serial1.available())<9 && (millis()-msTimeout<1000); Particle.process());
    Log.trace("[MHZ] Available: %d", len);
    Log.trace("[MHZ] Num. try-: %d", ntry);
    ntry++; 
    }
  if (len != 9) return false;
  for(i = 0; i < len; i++) data[i] = Serial1.read();
  for(i = 1; i < 9; i++) chksum += data[i];
  if(chksum==0) CO2 = (256*(int)data[2]) + (int)data[3];
  if(chksum==0) CO2TC = (int)data[4] - 40;
  return true;
}

float getUVAPower(float &UVAP)
{
  int sensorvalue;
  int sum=0;
  for(int i=0; i<=1024; i++)
  {
    sensorvalue=analogRead(A0);     //0-3.3V mapped to 0-4095
    delayMicroseconds(10);
    sum = sum + sensorvalue;
  }
  //per GUVA-S12SD datasheet: Photocurrent(nA)=113xUV Power(mW/cm2)
  //per GUVA-S12SD datasheet: Photocurrent(nA)=21xUVIndex+83
  //per Gove Data schematic Photocurrent(nA)= VsensorxR2/((R1+R2)*R3))/10^9, where R1=3.3kOhm, R2=1kOhm, R3=10MOhm for v1.1, 1Mohm for version v1.1b - confirmed v1.1  
  UVAP = (((float(sum))/1024.0f)*(3.3f/4096.0f)*1000000000.0f*3300.0f/((1000.0f+3300.0f)*1000000.0f))/113.0f;  //mW/cm2
  return UVAP;
}

void createEventPayload(int CO2, int CO2TC, float UVAP, float UVind)
{
  JsonWriterStatic<256> jw;
  {
    JsonWriterAutoObject obj(&jw);
    jw.insertKeyValue("CO2(ppm)", CO2);
    jw.insertKeyValue("CO2_T(C)", CO2TC);
    jw.insertKeyValue("UVAPower(mW/cm2)", UVAP);
    jw.insertKeyValue("UVIndex", UVind);
  }
  Particle.publish("CooCooCachoo", jw.getBuffer(), PRIVATE);
}