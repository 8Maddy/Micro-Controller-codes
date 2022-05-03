#include <SoftwareSerial.h>
#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10);
int DC_Voltage_LSB = 0b00000000;
int DC_Voltage_MSB = 0b00000000;
int DC_Current_LSB = 0b00000000;
int DC_Current_MSB = 0b00000000;
int Throttle_Percent_LSB=0;
int Throttle_Percent_MSB=0;
int Motor_RPM_LSB = 0;
int Motor_RPM_MSB = 0;
int Motor_Temperature_LSB = 0;
int Controller_Temperature_MSB = 0;
int Subtotal_LSB =0;
int Subtotal_MSB =0;
int X=0;
int Y=0;
float KM=0;
float Kmh=0;

SoftwareSerial BluetoothSerial(0, 1);

void setup()
{
  Serial.begin(19200);
//  BluetoothSerial.begin(19200);

  while (BluetoothSerial.available())
  {
  Serial.write(BluetoothSerial.read());
  }
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();
}

void loop()
{
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
  {      
    if (canMsg.can_id == 0x90F8109A)
    {
      Throttle_Percent_LSB = canMsg.data[1];
      Throttle_Percent_MSB = canMsg.data[2];
      
      Motor_RPM_LSB = canMsg.data[4];
      Motor_RPM_MSB = canMsg.data[5];

      Motor_Temperature_LSB = canMsg.data[6];
      
      Controller_Temperature_MSB = canMsg.data[7];
    }
    if (canMsg.can_id == 0x90F8108D)
    {
      DC_Voltage_LSB = canMsg.data[0];
      DC_Voltage_MSB = canMsg.data[1];

      DC_Current_LSB = canMsg.data[2];
      DC_Current_MSB = canMsg.data[3]; 

      Subtotal_LSB = canMsg.data[4];    
      Subtotal_MSB = canMsg.data[5];    
    }
     X=((( Subtotal_MSB << 8)+  Subtotal_LSB)/1)/64;
     KM = 0.00152*X;
     Y= (Motor_RPM_MSB << 8) + Motor_RPM_LSB;
     Kmh = 2*3.142*0.06*Y*0.4826;
    
//     Serial.print("RPM = ");
//     Serial.print(((Motor_RPM_MSB << 8) + Motor_RPM_LSB));
     Serial.print(" "); 
     Serial.print("km/h = ");
     Serial.print(Kmh,0);
     Serial.print(" "); 
     Serial.print("M.T = ");
     Serial.print((Motor_Temperature_LSB-40)); 
     Serial.print(" "); 
     Serial.print("C.T = ");
     Serial.print((Controller_Temperature_MSB-40)); 
     Serial.print(" ");
     Serial.print("Voltage = ");
     Serial.print((float(DC_Voltage_MSB << 8) + DC_Voltage_LSB) / 10);
     Serial.print(" ");
     Serial.print("Current = ");
     Serial.print((float(DC_Current_MSB << 8) + DC_Current_LSB) / 10);
     Serial.print(" ");
//     Serial.print("Throttle = ");
//     Serial.print(((Throttle_Percent_MSB << 8)+ Throttle_Percent_LSB));
     Serial.print(" ");  
     Serial.print("Range = ");
     Serial.print(KM,5);
     Serial.println(" "); 
}
}
