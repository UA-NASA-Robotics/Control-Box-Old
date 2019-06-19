#define SPACEE_INTERFACE
//control.ino
//
//Version 19
//Created by Zac Kilburn with guidence from Igor Vinograd :)
//3/7/14


//Software Packages Include
#include <FastTransfer.h>
#include <Timers.h>
#include "SoftwareSerial.h"
//#include <digitalWriteFast.h>
//#include "Adafruit_Thermal.h"
#include <avr/pgmspace.h>

#define GETTING_COMMS   0
#define AUTONOMOUS      1
#define MACRO           2
#define NORMAL          3
int GlobalState;
//Internal File Includes
//#include "LED.h"
#include "Methods.h"
#include "Definitions.h"
#include "Variables.h"
//#include "Printer.h"
#include "Comms.h"
#include "Screen_Outgoing.h"
#include "Screen.h"
#include "MacroControl.h"
#include "PinMap.h"
#include "Debug.h"

Timers batteryPresentWarningPage(2500);
Timers batteryWarningTimer(500);
bool batteryPowered = false;
bool flash=true;
bool bootup = true;
//#define GETTING_COMMS   0
//#define AUTONOMOUS      1
//#define MACRO           2
//#define NORMAL          3


void setup()
{
//  printerPrint();
//  while(1) ;
  //setupLED();
  initializeCommunicaton();
  initializePins();
  delay(3000);
  checkForBattery();
 
  
 digitalWrite(13,HIGH);
 delay(500);
 digitalWrite(13,LOW);
 delay(500);
 digitalWrite(13,HIGH);
 
}

void loop() {
  controlsUpdate();
  macroUpdate();
  updateCommunication();
  //UpdateLEDs(NORMAL);
  //Battery Monitor display
  if(bootup && !batteryPresentWarningPage.timerDone() && batteryPowered)  //We are bootin up and there is a battery
  {
      //Wait until timer has expired to show the message
  }
  else if(bootup && batteryPowered) //if we are batteryPowered and bootup, but timer has expired
  {
    CTS = true;
    writeObject(FORMMESSAGE, 0, 1);
    bootup=false;
  }
  else 
  {
    updateScreen();
  }
  updateDebug();
  
  if(batteryPowered)
      monitorBattery();
  //ledUpdate();

}


void checkForBattery(void)
{
  for(int i=0;i<100;i++)
  {
    readBatteryLevel(); //Initialize the filter and grab values to get an average that makes sence
  }

  //Check if battery is there
  if(readBatteryLevel()<BATTERY_DETECT_LEVEL)
  {
    
  }
  else
  {
    batteryPowered=true;
    writeObject(FORMMESSAGE,5,1);
    delay(2500);
  }
}

void monitorBattery(void)
{
  static bool batteryLowWarning=false;
  if(readBatteryLevel() < BATTERY_LOW_LEVEL)
  {
    batteryLowWarning=true;
    if(batteryWarningTimer.timerDone())
    {
      
        if(flash)
        {
          writeObject(FORMMESSAGE, 10, 1);
          flash=false;
        }
        else
        {
          writeObject(FORMMESSAGE, 19, 1);
          flash=true;
        }      
    }    
  }
  else if(batteryLowWarning)
  {    
    writeObject(FORMMESSAGE, 0, 1);
    batteryLowWarning = false;
  }
}

float avgBatt;
float readBatteryLevel(void)
{
  int battADC= analogRead(JOYSTICK_1_Y);
  float voltage = battADC * 50.0/1024.0;  //Implied 10ths
  voltage *= VOLTAGE_DIVIDER_RATIO;       //0-84.0 (0-8.4volts)
  avgBatt -= avgBatt/16.0;
  avgBatt += voltage;
  
  return avgBatt/16.0;
}




