#define HC595_CLOCK 27   // SRCLK/SHCP  11
#define HC595_LATCH 14   // RCLK / STCP 12
#define HC595_DATA 13   // SER / DS    14
#define HC595_OUTPUT_ENABLE         4
#define RS485_RTS 22
#define POWER_LED 15

#define SERIAL_DEBUG_DISABLED

#include <ModbusMaster.h>
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"

using namespace sensesp;

reactesp::ReactESP app;
uint8_t relay = 1;
uint8_t segment = 0xf7;
uint8_t segment_counter = 0;
uint8_t voltage_counter = 0;
uint8_t display[4];
uint16_t n4dva16_voltage[16];
uint32_t button_age[4];
const uint8_t button_pin[] = {18,19,21,23};
uint8_t channel = 0;
uint8_t channel_old = 0;
uint8_t channel_age = 0;

ModbusMaster node;
uint8_t modbus_ok= 0;

hw_timer_t *output_timer = NULL;

uint8_t SEG8Code[] = {0,0,0,0,0,0,0,0, //0-7 ASCII
                      0,0,0,0,0,0,0,0, //8-15
                      0,0,0,0,0,0,0,0, //16-23
                      0,0,0,0,0,0,0,0, //24-31
                      0,0,0,0,0,0,0,0, //32-39
                      0,0,0,0,0,0,0,0, //40-47
                      0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07, //48-55, numerals 0-7
                      0x7f,0x6f,0,0,0,0,0,0, //56-63, numerals 8 and 9
                      0,0x77,0x7c,0x39,0x5e,0x79,0x71,0, //A-F
                      0x76};//H

float n4dva16_ch1() {return n4dva16_voltage[0]*0.01f;} 
float n4dva16_ch2() {return n4dva16_voltage[1]*0.01f;}
float n4dva16_ch3() {return n4dva16_voltage[2]*0.01f;}
float n4dva16_ch4() {return n4dva16_voltage[3]*0.01f;}
float n4dva16_ch5() {return n4dva16_voltage[4]*0.01f;}
float n4dva16_ch6() {return n4dva16_voltage[5]*0.01f;}
float n4dva16_ch7() {return n4dva16_voltage[6]*0.01f;}
float n4dva16_ch8() {return n4dva16_voltage[7]*0.01f;}
float n4dva16_ch9() {return n4dva16_voltage[8]*0.01f;}
float n4dva16_ch10() {return n4dva16_voltage[9]*0.01f;}
float n4dva16_ch11() {return n4dva16_voltage[10]*0.01f;}
float n4dva16_ch12() {return n4dva16_voltage[11]*0.01f;}
float n4dva16_ch13() {return n4dva16_voltage[12]*0.01f;}
float n4dva16_ch14() {return n4dva16_voltage[13]*0.01f;}
float n4dva16_ch15() {return n4dva16_voltage[14]*0.01f;}
float n4dva16_ch16() {return n4dva16_voltage[15]*0.01f;}

void preTransmission()
{
  digitalWrite(RS485_RTS, 1);
}

void postTransmission()
{
  digitalWrite(RS485_RTS, 0);
}

void Digital_Output_Byte(uint8_t dat)   //Send 1 byte
{
    uint8_t i;
    for(i=8;i>=1;i--)
    {
        if(dat & 0x80){
      digitalWrite(HC595_DATA, 1);
    } else {
      digitalWrite(HC595_DATA, 0);
    }       //Sends data bit by bit from high to low.
        dat <<= 1;
    digitalWrite(HC595_CLOCK, 0);
      digitalWrite(HC595_CLOCK, 1);
    }
}

void Output_ES32A08(uint8_t Num, uint8_t Seg, uint8_t out)
{
    Digital_Output_Byte(out);
    Digital_Output_Byte(Seg);
    Digital_Output_Byte(Num);
  digitalWrite(HC595_LATCH, 0);
  digitalWrite(HC595_LATCH, 1);
}

void IRAM_ATTR Update_Outputs()
{
  Output_ES32A08(display[3-segment_counter],segment >> segment_counter,0);
  segment_counter++;
  if(segment_counter>3)
    segment_counter=0;
  for(int i=0;i<4;i++)
  {
    if (digitalRead(button_pin[i]))
      button_age[i]=0;
    else
      button_age[i]++;
  }

  if ((button_age[0] == 5 ) && (channel>0))
    channel--;

  if ((button_age[0] == 500 ) && (channel>0))
  {
    channel--;
    button_age[0] = 250;
  }

  if ((button_age[1] == 5 ) && (channel<15))
    channel++;

  if ((button_age[1] == 500 ) && (channel<15))
  {
    channel++;
    button_age[1] = 250;
  }

  /*segment >>= 1;
  if (segment == 0xf)
    segment=0xf7;*/
}

void Display_Update()
{
  char buffer[10];

  if (channel_age)
  {
    sprintf(buffer, "CH%2u",channel+1);
    channel_age--;
  } 
  else
    sprintf(buffer, "%4u",n4dva16_voltage[channel]);

  if (channel != channel_old)
  {
    channel_age=10;
    channel_old = channel;
  }

  //digitalWrite(POWER_LED, !digitalRead(POWER_LED));
  /*if (voltage_counter & 1)
  {
    sprintf(buffer, "%4u",n4dva16_voltage[voltage_counter>>1]);
  }
  else
  {
    sprintf(buffer, "CH%2u",(voltage_counter>>1)+1);
  }*/

  display[0]=SEG8Code[buffer[0]];
  display[1]=SEG8Code[buffer[1]];
  display[2]=SEG8Code[buffer[2]];
  display[3]=SEG8Code[buffer[3]];

  if (modbus_ok)
    display[1] |= 0x80;
  voltage_counter++;
  if (voltage_counter>31)
    voltage_counter=0;
}

void Modbus_Read()
{
  uint8_t result;
  digitalWrite(POWER_LED, 1);
  modbus_ok = 0;
  // Read 16 registers starting at 0x3100)
  result = node.readInputRegisters(0x0000, 16);
  //result = node.readHoldingRegisters(0x00f7, 1);
  if (result == node.ku8MBSuccess)
  {
    modbus_ok=1;
    for(int i=0;i<16;i++)
    {
      n4dva16_voltage[i]=node.getResponseBuffer(i);
      //n4dva16_voltage[i]=node.u8ModbusADU[i];
      /*if (n4dva16_voltage[i] > 9999)
        n4dva16_voltage[i]=9999;*/
    }
    //n4dva16_voltage[15]=node.u8ModbusADUSize;
  }
  digitalWrite(POWER_LED, 0);
}


// The setup function performs one-time application initialization.
void setup() {
  //SetupLogging();
  output_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(output_timer, &Update_Outputs, true);
  timerAlarmWrite(output_timer, 1000, true);
  timerAlarmEnable(output_timer);
  
  pinMode(HC595_CLOCK, OUTPUT);
  pinMode(HC595_LATCH, OUTPUT);
  pinMode(HC595_DATA, OUTPUT);
  pinMode(HC595_OUTPUT_ENABLE, OUTPUT);
  pinMode(POWER_LED, OUTPUT);
  for (int i=0;i<4;i++)
    pinMode(button_pin[i],INPUT_PULLUP);
  
  digitalWrite(HC595_OUTPUT_ENABLE,1);
  Output_ES32A08(0,0,0);
  digitalWrite(HC595_OUTPUT_ENABLE,0);

  pinMode(RS485_RTS, OUTPUT);
  digitalWrite(RS485_RTS, 0);
  Serial.begin(9600);

  // Modbus slave ID 1
  node.begin(32, Serial);
  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Linda-ES32A08")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();
  WiFi.mode(WIFI_STA);
  WiFi.begin("animalhouse", "acatisadog");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    digitalWrite(POWER_LED, !digitalRead(POWER_LED));
  }

  /*app.onRepeat(1, [] ()
  {
    Cyclic_Callback_1ms();
  });*/

  app.onRepeat(100, [] ()
  {
    Display_Update();
  });

  app.onRepeat(1000, [] ()
  {
    Modbus_Read();
  });

  auto* voltage_input1 = new RepeatSensor<float>(10000, n4dva16_ch1);
  auto* voltage_input2 = new RepeatSensor<float>(10000, n4dva16_ch2);
  auto* voltage_input3 = new RepeatSensor<float>(10000, n4dva16_ch3);
  auto* voltage_input4 = new RepeatSensor<float>(10000, n4dva16_ch4);
  auto* voltage_input5 = new RepeatSensor<float>(10000, n4dva16_ch5);
  auto* voltage_input6 = new RepeatSensor<float>(10000, n4dva16_ch6);
  auto* voltage_input7 = new RepeatSensor<float>(10000, n4dva16_ch7);
  auto* voltage_input8 = new RepeatSensor<float>(10000, n4dva16_ch8);
  auto* voltage_input9 = new RepeatSensor<float>(10000, n4dva16_ch9);
  auto* voltage_input10 = new RepeatSensor<float>(10000, n4dva16_ch10);
  auto* voltage_input11 = new RepeatSensor<float>(10000, n4dva16_ch11);
  auto* voltage_input12 = new RepeatSensor<float>(10000, n4dva16_ch12);
  auto* voltage_input13 = new RepeatSensor<float>(10000, n4dva16_ch13);
  auto* voltage_input14 = new RepeatSensor<float>(10000, n4dva16_ch14);
  auto* voltage_input15 = new RepeatSensor<float>(10000, n4dva16_ch15);
  auto* voltage_input16 = new RepeatSensor<float>(10000, n4dva16_ch16);

  voltage_input1->connect_to(new SKOutputFloat("sensors.n4dva16_ch1.voltage","/Sensors/N4DVA16 Channel 1/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input2->connect_to(new SKOutputFloat("sensors.n4dva16_ch2.voltage","/Sensors/N4DVA16 Channel 2/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input3->connect_to(new SKOutputFloat("sensors.n4dva16_ch3.voltage","/Sensors/N4DVA16 Channel 3/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input4->connect_to(new SKOutputFloat("sensors.n4dva16_ch4.voltage","/Sensors/N4DVA16 Channel 4/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input5->connect_to(new SKOutputFloat("sensors.n4dva16_ch5.voltage","/Sensors/N4DVA16 Channel 5/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input6->connect_to(new SKOutputFloat("sensors.n4dva16_ch6.voltage","/Sensors/N4DVA16 Channel 6/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input7->connect_to(new SKOutputFloat("sensors.n4dva16_ch7.voltage","/Sensors/N4DVA16 Channel 7/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input8->connect_to(new SKOutputFloat("sensors.n4dva16_ch8.voltage","/Sensors/N4DVA16 Channel 8/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input9->connect_to(new SKOutputFloat("sensors.n4dva16_ch9.voltage","/Sensors/N4DVA16 Channel 9/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input10->connect_to(new SKOutputFloat("sensors.n4dva16_ch10.voltage","/Sensors/N4DVA16 Channel 10/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input11->connect_to(new SKOutputFloat("sensors.n4dva16_ch11.voltage","/Sensors/N4DVA16 Channel 11/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input12->connect_to(new SKOutputFloat("sensors.n4dva16_ch12.voltage","/Sensors/N4DVA16 Channel 12/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input13->connect_to(new SKOutputFloat("sensors.n4dva16_ch13.voltage","/Sensors/N4DVA16 Channel 13/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input14->connect_to(new SKOutputFloat("sensors.n4dva16_ch14.voltage","/Sensors/N4DVA16 Channel 14/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input15->connect_to(new SKOutputFloat("sensors.n4dva16_ch15.voltage","/Sensors/N4DVA16 Channel 15/Voltage",new SKMetadata("V","Analog input voltage")));
  voltage_input16->connect_to(new SKOutputFloat("sensors.n4dva16_ch16.voltage","/Sensors/N4DVA16 Channel 16/Voltage",new SKMetadata("V","Analog input voltage")));
}

void loop()
{
  app.tick();
}
