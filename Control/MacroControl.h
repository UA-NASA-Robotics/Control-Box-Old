//MAY NEED TUNED TO ALLOW THE MACRO TO START SUCCESSFULLY
Timers macroSetDelay(250);

inline void macroUpdate()
{
  //Robot says its Done with macro and control box needs to finish the macro
  if ((commMacro == 0) && (internalMacroKeeper != 0) && macroSetDelay.timerDone())
  {
    
    //internal systems reset
    internalMacroKeeper = 0;
    internalSubKeeper = 0;
    //tell the robot we are done as well   
    sendDataNavigation(0, 0);
    
  

    //wipe any pending commands from screen
    macro_command_screen=0;
    
    
    commTimer.resetTimer();
    GlobalState = NORMAL;
  }
  //---------------------------------------------------------------------
  //FROM THE SCREEN -- MACRO PRESENT
  if (macro_command_screen != 0 && internalMacroKeeper == 0)
  {

    
    //Change control board state to macro state
    //let it know which macro
    internalMacroKeeper = macro_command_screen;
    internalSubKeeper = macro_sub_command;
     
     
    //Tell the comm board about the screens command 
    #ifdef SPACEE_INTERFACE      
       sendDataMotor(leftMotorSpeed, rightMotorSpeed, actuatorSpeed, conveyorButton, internalMacroKeeper, internalSubKeeper);
    #else
       sendDataNavigation(macro_command_screen, macro_sub_command);
    #endif
    //trick the system into believing the robot has responded that it got the command (AKA dont exit macro accidently, seeing that the commMacro=0)
    commMacro= macro_command_screen;

    //erase the data from the screen
    macro_command_screen = 0;
    macro_sub_command = 0;
    
    //Timer to attempt to let the command propagate into the system, so by the time we check the macrostate of the robot, it will have gotten the command
    macroSetDelay.resetTimer();
    noMacroPresent=false;
    macroDelay.resetTimer();
    //GlobalState = MACRO;
  }
  if (digitalRead(MACRO_BUTTON))
  {
    //digitalWrite(13,!digitalRead(13));
    //delay(1000);
    killMacro();
  }
}















