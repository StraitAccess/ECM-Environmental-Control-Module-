/*IN THIS VERSION
 * 1. All heating and cooling code has been removed.
 * 2. The usage of the new sensor has been implemented with relay inputs. PI control is now removed, and overshooting
 * will be controlled by the balacing the time delay (normally around 30s) with that of the hystersis setting on the
 * sensor.
 * 3. Interlock with spinning machine has been enabled again
 */

#include<Controllino.h>   //General controllino library
#include <SPI.h>          
#include <Sensirion.h>    //Library needed for the temperature/humidity sensor
#include "ModbusRtu.h"    /* Usage of ModBusRtu library allows you to implement the Modbus RTU protocol in your sketch. */

//--------------------------------------------------------------------------------------------//
//--------------------------------ModbusRTU----------------------------------------
//ModbusRTU
//Initiate modbus variables
#define SlaveModbusAdd  5     //This is for communication with the screen
#define RS485Serial     3     //Still for communication with the screen
Modbus ControllinoModbusSlave(SlaveModbusAdd, RS485Serial, 0);
uint16_t ModbusSlaveRegisters[16];

//--------------------------------------------------------------------------------------------//
//--------------------------------Pin Declarations----------------------------------------
//declare all the pins on the Controllino as FAN1, FAN2 etc
int fan1 = CONTROLLINO_D1;         //first fan in the line
int fan2 = CONTROLLINO_R5;         //Fan connected to drying cycle
int fan3 = CONTROLLINO_D3;         //Fan connected to wetting cycle
int fan4 = CONTROLLINO_D4;         //Final fan before the HEPA
int heater = CONTROLLINO_R0;
int cooler = CONTROLLINO_R2;
int watervalve = CONTROLLINO_R3;
int humidifier = CONTROLLINO_R4;
int floatswitch = CONTROLLINO_A1;
int spinready = CONTROLLINO_R14;
int R0 = CONTROLLINO_R0;
int dryinginput = CONTROLLINO_A3;  //The input from the sensor connected to relay #1, indicating that the system is too wet and drying needs to occur
int wettinginput = CONTROLLINO_A4;  //The input from the sensor connected to relay #2, indicating that the system is too dry and wetting needs to occur

//--------------------------------------------------------------------------------------------//
//--------------------------------Variable Declarations----------------------------------------
//These variables are to carry the condition over to the control of fans, eg. heating will switch on only three fans. Values are defined in
int wetting = 0;
int drying = 0;

int manwater = 0;
int purge = 0;

//Set the reference fan speeds
int MFS = 230;  //Max Fan Speed
int SFS = 100; //Standby Fan Speedd
int LFS = 0; //Least Fan Speed

int j = 0; //counter
//--------------------------------------------------------------------------------------------//
//-------------------------------Setup-----------------------------------------

void setup()
{
  delay(2000);
  Serial.begin(9600);
  Serial.println("Started");                    //Start serial comms, only for when the USb is plugged in
 
  pinMode(fan1, OUTPUT);              //fan 1
  pinMode(fan2, OUTPUT);              //fan 2
  pinMode(fan3, OUTPUT);              //fan 3
  pinMode(fan4, OUTPUT);              //fan 4

  //pinMode(heater, OUTPUT);            //heating
  //pinMode(cooler, OUTPUT);            //cooling
  pinMode(watervalve, OUTPUT);        //water valve
  pinMode(floatswitch, INPUT);        //float switch
  pinMode(humidifier, OUTPUT);        //humidifier
  pinMode(dryinginput, INPUT);
  pinMode(wettinginput, INPUT);

  ControllinoModbusSlave.begin( 19200 );        // Start the communication over the ModbusRTU protocol. Baund rate is set at 19200.

  Serial.begin(9600);

  digitalWrite(CONTROLLINO_D10, HIGH);
  digitalWrite(spinready, HIGH);
}

//--------------------------------------------------------------------------------------------//
//-------------------------------Loop-----------------------------------------
  
void loop()
{
  //Poll all the register values
   ControllinoModbusSlave.poll(ModbusSlaveRegisters, 16);
    
  //Update and read Modbus register values
    manwater = ModbusSlaveRegisters[6];
    purge = ModbusSlaveRegisters[8];

    ActuatorSwitching();  //Switch actuators on and off

    FanControl();            //Control fan speeds

    FloatSwitch();           //Fill the water tank if needed

    SpinReady();             //Pull the spinready value high or low
    delay(1000);
}

//--------------------------------------------------------------------------------------------//
//-------------------------------Functions-----------------------------------------

void ActuatorSwitching()         //Switch actuators on and off (based on if the current time is more than the allowable window size for each actuator), and allow manual switching on and off of actuators.
{
   //---------------Humidity-------------------
   if (digitalRead(wettinginput) == HIGH)    /////Wetting/////
      {
      digitalWrite(humidifier, HIGH);
      digitalWrite(fan2,LOW);
      wetting = 1;
      drying = 0;
      }
   else if (digitalRead(dryinginput) == HIGH)    /////Drying/////
      {
      digitalWrite(humidifier, LOW);
      digitalWrite(fan2,HIGH);
      wetting = 0;
      drying = 1;
      }
   else
      {
      digitalWrite(humidifier, LOW);
      digitalWrite(fan2,LOW);
      wetting = 0;
      drying = 0;
      }
   }
      
void FanControl() //This function controls the speeds of the fans
{
  if (purge == 1)
  {
    analogWrite(fan1, 250);
    digitalWrite(fan2, HIGH);
    analogWrite(fan3, 250);  
    analogWrite(fan4, 250);
  }
  /*  //Test the purge function with the code below, comment out the code after it. 
  else
  {
    analogWrite(fan1, 0);
    digitalWrite(fan2, LOW);
    analogWrite(fan3, 0);  
    analogWrite(fan4, 0);
  }
  */  
  
  else if (wetting == 1 || drying == 1)  //this must include all drying, cooling and wetting
  {
    analogWrite(fan1, 200);
    analogWrite(fan4, 200);
      if (drying == 1)
      {
          //digitalWrite(fan2,HIGH);
          analogWrite(fan3,LFS);
      }
      else
      {
          //digitalWrite(fan2,LOW);
          analogWrite(fan3,230);
      }
  }
  else 
  {
    analogWrite(fan1, SFS);
    digitalWrite(fan2, LOW);
    analogWrite(fan3, SFS);  
    analogWrite(fan4, SFS);
  }
  
}

void FloatSwitch()  //fill water tank if needed
{
  if (digitalRead(floatswitch) == HIGH || manwater == 1)
  {
    digitalWrite(watervalve, HIGH);
  }
  else
  {
    digitalWrite(watervalve, LOW);
  }
}

void SpinReady()    //enable the spin ready tag when inside the required temperature and humidity values. This value will alwyas be high in this version
{
  //if(digitalRead(wettinginput) == HIGH)
  if(digitalRead(wettinginput) == LOW && digitalRead(dryinginput) == LOW)
  {
    digitalWrite(spinready, HIGH);
    Serial.println("Spinning is ready");
    digitalWrite(CONTROLLINO_D10, HIGH);
    j = 0;
    Serial.println(j);
  }
  else
  {
    j++;
    Serial.println("Spinning is NOT ready");
    Serial.println(j);
    if (j >= 230) //five (5) minutes??
    {
    digitalWrite(spinready, LOW);
    digitalWrite(CONTROLLINO_D10, LOW);
    j = 0;
    }
  }
}



