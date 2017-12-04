#include<Controllino.h>
#include <SPI.h>
#include <Sensirion.h>
#include "ModbusRtu.h"    /* Usage of ModBusRtu library allows you to implement the Modbus RTU protocol in your sketch. */

//--------------------------------ModbusRTU----------------------------------------
//ModbusRTU
#define SlaveModbusAdd  5
#define RS485Serial     3
Modbus ControllinoModbusSlave(SlaveModbusAdd, RS485Serial, 0);
uint16_t ModbusSlaveRegisters[16];

//Temp and Humid sensor pins
const uint8_t dataPin =  20;                  // SHT serial data
const uint8_t sclkPin =  6;                   // SHT serial clock, D4
Sensirion sht = Sensirion(dataPin, sclkPin);  //Start the temp/humidity sensor

//Global variables of temp and humid
uint16_t rawData;                             
float temperature;
float humidity;
float dewpoint;

//--------------------------------Pin Declarations----------------------------------------
//declare all the pins as FAN1, FAN2 etc
int fan1 = CONTROLLINO_D1;
int fan2 = CONTROLLINO_R5;
int fan3 = CONTROLLINO_D3;
int fan4 = CONTROLLINO_D5;
int heater = CONTROLLINO_R0;
int cooler = CONTROLLINO_R2;
int watervalve = CONTROLLINO_R3;
int humidifier = CONTROLLINO_R4;
int floatswitch = CONTROLLINO_A1;
int spinready = CONTROLLINO_D10;
int R0 = CONTROLLINO_R0;
//--------------------------------Variable Declarations----------------------------------------
//These variables are to carry the condition over to the control of fans, eg. heating will switch on only three fans
int heating = 0;
int cooling = 0;
int wetting = 0;
int drying = 0;

//Manual toggling of fans will be controlled by the following variables
int manheating = 0;
int mancooling = 0;
int manwetting = 0;
int mandrying = 0;
int manwater = 0;
int purge = 0;

//Set the fan speeds
int MFS = 230;  //Max Fan Speed
int SFS = 100; //Standby Fan Speed
int LFS = 0; //Least Fan Speed

//--------------------------------PI control----------------------------------------
/////Temperature////
//float t_error;           //temp error
float t_setpoint;          //temp setpoint
float t_controlsignal;     //The control signal, this value determines the "power" of the element
float t_Kp = 0.8;            //K_proportional
float t_Ki = 0.3;          //K_integral
float t_error;             //Temp error, difference between current temp and set temp
float t_cumerror;          //Used for the cumulative error of the temperature, which is used together with K_i
float t_dutycycle;         //how long the temperature element must be on, based on t_controlsignal
int t_maxerror = 5;       //maximum expected temperature error/range, this value allows the softsign function to near 0 in the correct manner
float t_maxduty = 0.8;     //maximum duty cycle allowed, for safety purposes (0.5 = 50%)

//////Humidity//////
float h_setpoint;         //humidity setpoint
float h_controlsignal;
float h_error;
float h_cumerror;
float h_dutycycle;        //how long the humidity actuator must be on
int w_maxerror = 20;      //maximum expected humidity error/range

//Wetting
float w_Kp = 3;
float w_Ki = 0.1;
float w_maxduty = 1;      //maximum duty cycle allowed, for safety purposes

//Drying
float d_Kp = 2;
float d_Ki = 0.3;
float d_maxduty = 1;      //maximum duty cycle allowed, for safety purposes


//--------------------------------General timing items----------------------------------------
//Timing
int i = 1;                //counter for cumulative errors
int j = 0;                // readtempdata timer, to ensure that the data is not read every cycle
int counter = 0;          //initialise the counter
int windowsize = 5;  //10 seconds window size
unsigned long windowStartTime;   
unsigned long now;   //Current time












//-------------------------------Setup-----------------------------------------

void setup()
{
  //Set the controllino Real Time Clock 
  Controllino_RTC_init(0);                      //Initialise the clock
  Controllino_SetTimeDate(7,3,11,17,0,0,0);     // set initial values to the RTC chip, (Day of the month, Day of the week, Month, Year, Hour, Minute, Second


  delay(2000);
  Serial.begin(9600);
  Serial.println("Started");                    //Start serial comms, only for when the USb is plugged in

  
 
  pinMode(fan1, OUTPUT);              //fan 1
  pinMode(fan2, OUTPUT);              //fan 2
  pinMode(fan3, OUTPUT);              //fan 3
  pinMode(fan4, OUTPUT);              //fan 4

  pinMode(heater, OUTPUT);            //heating
  pinMode(cooler, OUTPUT);            //cooling
  pinMode(watervalve, OUTPUT);        //water valve
  pinMode(floatswitch, INPUT);        //float switch
  pinMode(humidifier, OUTPUT);        //humidifier

  ControllinoModbusSlave.begin( 19200 );        // Start the communication over the ModbusRTU protocol. Baund rate is set at 19200.

  Serial.begin(9600);


  i = 1;
  windowStartTime = Controllino_GetSecond();;    //start the timer, this will be compared to the window at a later stage to control when actuators must switch on and off

/*
 //Set fan start speeds
 //ideal speeds
 //wetting
  analogWrite(fan1, 200);             
  digitalWrite(fan2,LOW);
  analogWrite(fan3, 230);
  analogWrite(fan4, 200);
//drying
  analogWrite(fan1, 200);             
  digitalWrite(fan2,HIGH);
  analogWrite(fan3, 10);
  analogWrite(fan4, 200);
 */
 
  delay(45000);                     //Wait for the screen to start up

  int t_setpoint = 25;                //Set the default temperature
  int h_setpoint = 38;                //Set the default humidity

  //Write the default values to the screen
  ModbusSlaveRegisters[9] = t_setpoint;
  ModbusSlaveRegisters[10] = h_setpoint;
  //ControllinoModbusSlave.poll(ModbusSlaveRegisters, 16);
}









//-------------------------------Loop-----------------------------------------
  
void loop()
{

  j++;                //increment the readtempdata line
  if (j >= 1000)      //only read every x cycles
  {
    readTempData();
    j = 0;           //Reset the counter  
  }

  //Poll all the register values
   ControllinoModbusSlave.poll(ModbusSlaveRegisters, 16);
   
  //Read temp and humidity from sensor.
  
  //Update and read Modbus register values
    mancooling = ModbusSlaveRegisters[0];
    manheating = ModbusSlaveRegisters[1];
    manwetting = ModbusSlaveRegisters[2];
    ModbusSlaveRegisters[3] = 10*temperature; //send current temperature to the screen
    mandrying = ModbusSlaveRegisters[4];
    ModbusSlaveRegisters[5] = 10*humidity;    //send current humidity to the screen
    manwater = ModbusSlaveRegisters[6];
    purge = ModbusSlaveRegisters[8];
    t_setpoint = ModbusSlaveRegisters[9];
    h_setpoint = ModbusSlaveRegisters[10];

    //Serial.println(purge);
    
/*//test the purging
    digitalWrite(CONTROLLINO_D7, ModbusSlaveRegisters[8]);// purge
    
    purge = ModbusSlaveRegisters[4];
    Serial.println(purge);
 
    digitalWrite(CONTROLLINO_D8, ModbusSlaveRegisters[8]);// purge
*/

  //Do PI calculation for temperature
    t_error = t_setpoint - temperature;                              //determine error here
    t_cumerror = (t_cumerror + t_error)/i;                           //determine cumulative error here
    t_controlsignal = PIControl(t_error, t_cumerror, t_Kp, t_Ki);    //calculate the control signal for this variable

  //Set signal values of control elements
    t_dutycycle = ActuatorDutyCycle(t_controlsignal, t_maxduty);     //calculate how long the actuator needs to stay on for, i.e. duty cycle


  //Do PI calculation for humidity
    h_error = h_setpoint - humidity;
    h_cumerror = (h_cumerror + h_error)/i;                           //determine cumulative error here

    if (h_error > 0)     //h_error = h_setpoint - humidity; 
    {//then wet
      h_controlsignal = PIControl(h_error, h_cumerror, w_Kp, w_Ki);    //calculate the control signal for this variable
      h_dutycycle = ActuatorDutyCycle(h_controlsignal, w_maxduty);     //calculate how long the actuator needs to stay on for, i.e. duty cycle
    }
    else
    {//then dry
      h_controlsignal = PIControl(h_error, h_cumerror, d_Kp, d_Ki);    //calculate the control signal for this variable
      h_dutycycle = ActuatorDutyCycle(h_controlsignal, d_maxduty);     //calculate how long the actuator needs to stay on for, i.e. duty cycle
    }


    //Serial.println(t_dutycycle);


  //Round duty cycles down for safety, don't have fine enough control on timer

  //Get current time
  
    now = Controllino_GetSecond();
    //Serial.println(now);
    
    if (now > windowsize)  //reset window, cumerror and counter
    { 
      //windowStartTime = windowsize;
      Controllino_SetTimeDate(7,3,11,17,0,0,0); // set initial values to the RTC chip, (Day of the month, Day of the week, Month, Year, Hour, Minute, Second
      t_cumerror = 0;
      h_cumerror = 0;                   
      i = 1;
      counter++;
      //Serial.println("checked");
    }

  //Switch actuators on and off
    ActuatorSwitching(now);  //Switch actuators on and off

    FanControl();            //Control fan speeds

    FloatSwitch();           //Fill the water tank if needed

    SpinReady();             //Pull the spinready value high or low
}










//-------------------------------Functions-----------------------------------------

void readTempData()    //ReadTempData
{
  sht.measTemp(&rawData);              // sht.meas(TEMP, &rawData, BLOCK)
  temperature = sht.calcTemp(rawData);
  sht.measHumi(&rawData);              // sht.meas(HUMI, &rawData, BLOCK)
  humidity = sht.calcHumi(rawData, temperature);
  //dewpoint = sht.calcDewpoint(humidity, temperature);
  logData();
}


void logData()         //logData
{
    Serial.print("Temperature = ");
    Serial.print(temperature);

    Serial.print(" C, Humidity = ");
    Serial.println(humidity);

    //Serial.print(" %, Dewpoint = ");
    //Serial.print(dewpoint);
    //Serial.println(" C");
}


float PIControl(float error, float cumerror, float Kp, float Ki)  //PIControl
{
float controlsignal = Kp*error + Ki*cumerror;
return controlsignal;
}


float ActuatorDutyCycle(float controlsignal, float maxduty)   //ActuatorDutycycle
{
  float dutycycle;
  float softsign = (controlsignal)/(1 + abs(controlsignal));
  dutycycle = softsign*maxduty*windowsize;
  return dutycycle;
}


void ActuatorSwitching(int now)         //ActuatorSwitching
{
  //---------------Temperature-------------------
  if (manheating == 1)
  {
      //Serial.println("1");
      digitalWrite(heater, HIGH);
      heating = 1;
      digitalWrite(cooler, LOW);
      cooling = 0;
  }
  else if (mancooling == 1)
  {
      //Serial.println("2");
      digitalWrite(cooler, HIGH);
      cooling = 1;
      digitalWrite(heater, LOW);
      heating = 0;
  }
  /////Heating/////
  else if (t_error > 0 ) //t_setpoint - temperature, so if t_error > 1 then system is too cool and needs to heat
  {
      //Serial.print("t_error");
      //Serial.println(t_error);
      //Serial.println("heating");
      if (t_dutycycle > now - windowStartTime) //determine if time has lapsed and if the window is completed.
      {
      //Serial.println(t_error);

      digitalWrite(heater, HIGH);
      heating = 1;
      digitalWrite(cooler, LOW);
      cooling = 0;
      }
      else
      {
      digitalWrite(heater, LOW);
      heating = 0;
      digitalWrite(cooler, LOW);
      cooling = 0;
      }
  }
  else
  /////Cooling/////
  { 
    t_dutycycle = -1*t_dutycycle;
      //Serial.print("t_error");
      //Serial.println(t_error);
      //Serial.println("cooling");
        //    Serial.println(t_dutycycle);
      if (t_dutycycle > now - windowStartTime) //determine if time has lapsed and if the window is completed.
      {
      digitalWrite(cooler, HIGH);
      cooling = 1;
      digitalWrite(heater, LOW);
      heating = 0;
      }
      else
      {
      digitalWrite(heater, LOW);
      heating = 0;
      digitalWrite(cooler, LOW);
      cooling = 0;
      }
  }

   //---------------Humidity-------------------

   if (manwetting == 1)
   {
      digitalWrite(humidifier, HIGH);
      digitalWrite(fan2,LOW);
      wetting = 1;
      drying = 0;
   }
   else if (mandrying == 1)
   {
      digitalWrite(humidifier, LOW);
      digitalWrite(fan2,HIGH);
      wetting = 0;
      drying = 1;
   }
   else if (h_error > 0)
   {/////Wetting/////
      if (h_dutycycle > now - windowStartTime) //determine if time has lapsed and if the window is completed.
      {
      //Serial.println("wetting");
      digitalWrite(humidifier, HIGH);
      digitalWrite(fan2,LOW);
      wetting = 1;
      drying = 0;
      }
      else
      {
      digitalWrite(humidifier, LOW);
      digitalWrite(fan2,LOW);
      wetting = 0;
      drying = 1;
      }
   }
   else
   {/////Drying/////
      h_dutycycle = -1*h_dutycycle;
      if (h_dutycycle > now - windowStartTime) //determine if time has lapsed and if the window is completed.
      {
      //Serial.println("drying");
      digitalWrite(humidifier, LOW);
      digitalWrite(fan2,HIGH);
      wetting = 0;
      drying = 1;
      }
      else
      {
      digitalWrite(humidifier, LOW);
      digitalWrite(fan2,LOW);
      wetting = 1;
      drying = 0;
      }
   }
}


void FanControl()
{
  if (purge == 1)
  {
    analogWrite(fan1, 250);
    digitalWrite(fan2, HIGH);
    analogWrite(fan3, 250);  
    analogWrite(fan4, 250);
  }
  else if (heating == 1 || cooling == 1 || wetting == 1 || drying == 1)  //this must include all drying, cooling and wetting
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



/*
 * Calibrated values for the above
    analogWrite(fan1, 100);
    digitalWrite(fan2, LOW);
    analogWrite(fan3, 100);  
    analogWrite(fan4, 200);
*/



void FloatSwitch()
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


void SpinReady()
{
  int tr = abs(t_error);
  int hr = abs(h_error);
  if (tr < 1 && hr < 2)
  {
    digitalWrite(spinready, HIGH);
  }
  else
  {
    digitalWrite(spinready, LOW);
  }
}



