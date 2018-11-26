/*
Make an initialize routine that sets everything to factory default
Do a pass at reducing file size for AVR (get rid of simulation stuff )
Report:
  total time
  time in each step
  Max temp
  time above min reflow temp

Concepts were taken from:
  https://github.com/rocketscream/Reflow-Oven-Controller/blob/master/reflowOvenController.ino
  https://github.com/adafruit/Adafruit_MAX31856
  
Implementation Notes:
1. String conversion "toDouble" needs to be added
  a. To WString.h add the following

    double toDouble(void) const;

  b. to WString.cpp add the following:

    double String::toDouble(void) const {
      if (buffer)
        return atof(buffer);
      return 0;
    }

2.


TODO:
Clean up pushbutton Analog input read  It shouldn't be getting an AI every scan.  
  PB.tick only processes the input every ms.  
  Perhaps modify PB library to process the input, as digital read would necessearily slow down the scan rate if read every second as well
Modify buzzer library to steal the arbitrator
Loading configuration in setup is not correct for a first start

*/


//Decide if you want to enable the Loop Statistics Library
  #define EnableLoopStatistics //enable loop() scan time statistics
  #define ENABLEWIFI
  #ifdef ENABLEWIFI
    #define EnableWebServer
  #endif

#ifdef __AVR__
#error AVR Not Supported Yet
#endif

#if !(defined(ESP8266) || defined(ESP32))
#error Only ESP32 or ESP8266 Microcontrollers supported
#endif

#ifdef ENABLEWIFI
  #ifdef ESP8266
    #include <ESP8266WiFi.h>
    #include <ESP8266mDNS.h>
    #include <WebSocketsServer.h>
    #include <ArduinoJson.h>
    #ifdef EnableWebServer    
      #include <ESP8266WebServer.h>
      //#include <SPIFFS.h> //SPIFFS File system)
      #include <FS.h>
    #endif
  #endif
  #ifdef ESP32
    #include <WiFi.h>
    #include <ESPmDNS.h>
    #ifdef EnableWebServer      
      #include <WebServer.h>
      #include <SPIFFS.h> //SPIFFS File system)
    #endif
  #endif    
  #include <WebSocketsServer.h>
  #include <ArduinoJson.h>
  #include <ArduinoOTA.h>
  //#include <WiFiManager.h>
#else
  //Define this in order to put it to sleep
  #ifdef ESP8266
    #include <ESP8266WiFi.h>
  #endif
  #ifdef ESP32
    #include <WiFi.h>   
  #endif

#endif
#include <Buzzer.h>
#include <Button.h>
#include <PID.h>
#include <SPI.h>
#include <EEPROM.h>
#ifdef EnableLoopStatistics
  #include <LoopStatistics.h>
#endif
#include <LPF.h> //https://github.com/terryjmyers/LowPassFilter
#include <string>
#include <TerryMyersHelperFunctions.h>
#include <MAX31856.h> //https://github.com/terryjmyers/MAX31856
//define GPIO pins
#ifdef ESP8266
  #define SCKPIN 14
  #define MOSIPIN 13
  #define MISOPIN 12
  #define CS_PIN 15
  #define OUTPUT_PIN 4
  #define LED_PIN 2
  #define DRDY_PIN 5
  #define BUZZER_PIN 16
#endif


#ifdef ESP32
  #define SCKPIN 14
  #define MOSIPIN 13
  #define MISOPIN 12
  #define CS_PIN 15
  #define OUTPUT_PIN 2 //Shared with boot pin.  Default is weak pulldown
  #define LED_PIN 0 //Shared with boot pin.  Default is weak pullup
  #define DRDY_PIN 27
  #define BUZZER_PIN 4
#endif

//Configuration data stored to "EEPROM" (its really flash...).  Stored this data in SPIFFS in a json was considered, but using
//  the EEPROM library made the code smaller and simpiler.  If AVR is used then it will still work as well and be stored
//  in an actual EEPROM
  #define NUMPROFILES 6
  struct ReflowProfileStruct {
    double MaxStartTemp;

    double PreHeatTemp;
    double PreHeatRate;

    double SoakRate;
    double SoakTemp;

    double ReflowRate;
    double ReflowPeakTemp; //Maximum reflow temperature
    double ReflowPeakTempPreact; //Temperature before peak to turn off heating.

    double ReflowPeakTime; //Time to be within 5C of peak
    double ReflowPeakTimePreact; //Time before the PeakTime to transition to COOL

    double CoolDownRate;
  };
  struct PIDConfigStruct {
    double Kc;
    double Ti;
    double Td;
    double Min;
    double Max;
    bool Direction;
    double DeadTime; //User entered deadtime to allow the reflow Profile to be a bit more responsive.
  };


  //Main configuration struct
  struct config_t
  {
    uint8_t DebugLevel;
    char SoftwareFirmwareVersion[8] = "0.1";
    char CalibrationDate[16];
    char ModelNumber[16] = "ESPReflow";
    uint16_t SerialNumber = 1;

    PIDConfigStruct pid;
    uint16_t PWMPeriodTime;
    #ifdef ENABLEWIFI
    uint8_t DefaultWiFiNetwork; //index into array of WiFi of the default network to connect to
    #endif  
    ReflowProfileStruct Profile[NUMPROFILES];
    ReflowProfileStruct DefaultProfile[2];

    MAX31856_REG_Struct MAX31856;
    uint32_t UARTSpeed;
		uint8_t TabLength;
  } Config;
  
//General
  uint32_t __UPTIME; //System up time in seconds, displayed on System webpage
  uint32_t UpTimeTimer;
  String GlobalErrorMessage;
  String GlobalMessage;
  uint8_t DebugLevel; //debugging bit able to be turned on and off from serialconsole to expose various things
  bool RestartCMD;
  bool SaveConfigCMD;
  uint32_t SaveConfigTimer;
  uint32_t SaveConfigTime = 60000;
  uint8_t Dashboard; //int to display various configurations
  uint32_t DashboardTimer;
  uint32_t DashboardTime = 99;
  bool *ActiveResponseCMD;
  uint32_t DRDYWatchDogTimer;
  double *ActiveResponse;
  //WiFi
    #ifdef ENABLEWIFI


      bool WiFiFunctionsEnabled; //Enable/Disable all functions relating to WiFi: Telnet, Alexa, Webserver, etc.  Set programmatically
      bool WiFiFailSafeEnable = true; //Enable Disable automatically starting the AP if STA or or BT or AP is not on
      bool WiFiConnecting;//Configuration of the currently connecting WiFi 
      
      //telnet server (socket connections),provided for configuration through Telnet
        #define TELNET_MAX_CLIENTS 2 //probably never need more than 1, but its here nonetheless
        #define TELNETPORT 23 //default is 23
        #define TELNET_TIMEOUT_TIME 0 //Set to 0 to disable
        WiFiServer TelnetServer(TELNETPORT);
        struct TelnetServerClients { //custom struct to handle each client data and login information independantly
          WiFiClient clients;
          uint32_t TimeoutTimer;
          String clientBuffer = ""; //Create a global string that is added to character by character to create a final serial read
          String clientLastLine = ""; //Create a global string that is the Last full Serial line read in from sUARTBuffer
          String login = "";
          String password = "";
          uint8_t loginStep;
          uint8_t Dashboard;
          bool isAuthenticated;
          bool NewLine;
        };
        struct TelnetServerClients Telnet[TELNET_MAX_CLIENTS];

      #ifdef EnableWebServer
        //Web servers
        #define HTTPPORT 80
        #define WEBSOCKETSPORT 81
        #ifdef ESP32
          WebServer HTTPserver(HTTPPORT);
          File fsUploadFile; //For SPIFFS Editor Upload         
        #endif
        #ifdef ESP8266
          ESP8266WebServer HTTPserver(HTTPPORT);
          File fsUploadFile; //For SPIFFS Editor Upload         
        #endif
          WebSocketsServer webSocket = WebSocketsServer(WEBSOCKETSPORT);
          uint8_t WebSocketClientConnected;
      #endif


    #endif

  #ifdef EnableLoopStatistics
  //Setup a instance of LoopStatistics
    LoopStatistics LT;
    uint32_t LoopStatisticsTimer;
  #endif
  //Serial Read tags
  //UART Serial
    #define UART_DEFAULT_SPEED 1500000
    uint8_t TerminalClearScreenCMD[7] = { 0x1B , 0x5B, 0x32, 0x4A, 0x1B , 0x5B, 0x48 };
    #ifdef __AVR__
	 #define UART_BUFFER_SIZE 62 //max size of serial buffer
    #endif  
    #if defined(ESP8266) ||  defined(ESP32)
      #define UART_BUFFER_SIZE 256 //max size of serial buffer
    #endif
    String sUARTBuffer = ""; //Software serial buffer
    String sLastUARTLine = ""; //sUARTBuffer is copied into this tag when a \r, \n, or \0 character is receieved.  The text is processed then cleared out as a handshake
    uint8_t UARTDashboard; //int to display various configurations
  //House keeping timer.  Only check non-critical things every so often
    uint32_t HousekeepingTimer;
    uint32_t HousekeepingTime = 3; //less important periodic housekeeping tasks
  
//Reflow
  //State/Step engine
    typedef enum ReflowStep
    {
      IDLE,
      PREHEAT,
      SOAK,
      REFLOW,
      COOL,
      COMPLETE,
      PAUSED
    } reflowStep_t; 
  //Timers associated with every state/step
    struct ReflowStepTimes {
      uint32_t StartTime; //Start Time in seconds recorded from __UPTIME
      uint16_t Time; //Total Step Time in seconds
    };

  struct ReflowSteps {
    uint32_t UpdateTimer; //Timer to check whether to update the Step.  Used to increase MCU performance by not checking for step transitions every scan
    String MSG;       //Message to user

    reflowStep_t Step;    //Reflow Step
    reflowStep_t StepREM;   
    //Timers for each state that needs a timer
    ReflowStepTimes Idle; 
    ReflowStepTimes Preheat;
    ReflowStepTimes Soak;
    ReflowStepTimes Reflow;
    ReflowStepTimes Cool;

    double RateSetpoint;  //The current Step's Temperature Rate setpoint.  This is used to display in dashboard to tell the user what the current rate setpoint is.  NOT the pid setpoint
    double TempSetpoint;  //The current Step's Temperature transition setpoint.  This is used to display in dashboard to tell the user what setpoint its currently going to to transition to the next step.  NOT the pid setpoint

  };
  struct TimerStruct {
    uint32_t Timer;
    uint16_t Time;

    bool ONS;
  };
  struct ReflowStruct {
    ReflowSteps Step;
    bool Status;
    uint16_t Time; //total reflow time in seconds
    double MaxTemp; //maximum temperature recorded during reflow
    TimerStruct AboveSoakTempTimer;
    TimerStruct ReflowPeakTimer;
    double SP;    //pid setpoint.  Passed into pid library by address
    double *PV;   //pid Process Variable(Temp or Temp Rate).  Passed into pid library by address
    double CV;    //pid Control Variable (0-100%).  Passed into pid library by address

  };
  ReflowStruct Reflow;

  struct TempStruct {
    double Actual;        //TC temperature
    double Rate;        //TC Rate
    double CJ;
    double ActualPrev;     //TC temperature from last measurement, used for Rate calculation
    uint8_t Fault;        //Fault word directly from MAX31856
    uint8_t FaultREM;        //Fault word directly from MAX31856
    String FaultMSG;      //A human readable string of the fault
    uint32_t FaultMSGTimer;   //A human readable string of the fault
    uint32_t DRDYFlag;      //MAX31856 DRDY pin interupt flag.  Set in ISR.
    bool NewFlag;       //Set for one loop of the code when DRDYFlag is set to true
    uint32_t ConversionTimer; //Timer to calculate the time between DRDYFlag interupts
    int16_t ConversionTime; //Stores the conversion time between DRDYFlags.  Used during simulation

    bool Simulate;        //Disconnect Actual from TC measurement, and simulate the temperature based on pid output using a deadtime and first order lag-lead simulator
    uint32_t SimulateDRDYFlagTimer; //used when simulation is on to simulate the MAX31856 DRDY pin.
  };
  
  TempStruct Temp { //Initialize the Struct.
    9999,   //Actual
    0.0 , //Rate
    0.0 , //CJ
    0.0,  //ActualPrev
    0,    //Fault;
    0,    //FaultREM;
    "",   //FaultMSG 

    false,  //DRDYFlag;
    false,  //NewFlag;
    0,    //ConversionTimer;
    332 , //ConversionTime;

    false,  //Simulate
    0   //SimulateDRDYFlagTimer;
  };
  LPF TempActualLPF(1);
  LPF TempRateLPF(1);
  LPF LPFCJTemp(5.0);

  struct SoftwarePWMStruct {
    double DutyCycle; //0-100% desired duty cycle
    uint32_t millsREM;
    uint32_t PeriodTimer;
    uint16_t PWMTime;
    uint16_t PWMPeriodTimeElapsed;
    bool Energize; //logical output mapped directly to pin
  };
  SoftwarePWMStruct Output  {
    0.0,  //DutyCycle
    0,    //millsREM
    0,    //PeriodTimer
    0,    //PWMTime;
    0,    //PWMPeriodTimeElapsed;
    false //Energize
  };
  PID pid(&Reflow.PV, &Reflow.CV, &Reflow.SP, Config.pid.Kc, Config.pid.Ti, Config.pid.Td, Config.pid.Direction);
//Simulation for testing/debugging
  #define DeadtimeSize 10
  LPF HeatingElementsTempLPF(1); //LPF to simulate how hot teh heating elements get
  double HeatingElementsTemp;
  double HeatingElementsTempSpeedIncrease = 0.1;
  double HeatingElementsTempSpeedDecrease = 0.025;
  double HeatingElementsActual;
  LPF OutputLPF(2); //LPF to simulate output process lag
  double OutputDeadtime[DeadtimeSize]; //Array to simulate deadtime

    //Explicitly call define functions here because Arduino compiler is being an asshole
    void EndProgram(String ErrorMessage);
    void storeStruct(void *data_source, size_t size);
    void loadStruct(void *data_dest, size_t size);
    void MAX31856SetConfig(void);
    Buzzer BuzzerPowerOn(BUZZER_PIN,  1000, 500, 0);
    Buzzer BuzzerButtonPause(BUZZER_PIN, 400, 50, 0);
    Buzzer BuzzerButtonStartRestart(BUZZER_PIN, 500, 50,50,3);
    Buzzer BuzzerFeedbackNegative(BUZZER_PIN, 200, 250);
    Buzzer BuzzerFeedbackPositive(BUZZER_PIN, 3000, 50);
    Buzzer BuzzerPreHeatStart( BUZZER_PIN,  2000, 125);
    Buzzer BuzzerSoakStart(BUZZER_PIN,    2200, 125, 125, 2);
    Buzzer BuzzerReflowStart(BUZZER_PIN,  2400, 125, 125, 3);
    Buzzer BuzzerCooldownStart(BUZZER_PIN,  2600, 250, 250, 8);
    Buzzer BuzzerReflowComplete(BUZZER_PIN, 1000,2000,3000,1,2);

    bool PBInput;
    Button PB(&PBInput,10,1000);
    uint32_t AIReadmillisLast;

void ICACHE_RAM_ATTR ISR_DRDY_PIN(void) { //Interuppt service routine
                      //ISR when MAX31856 DRDY pin goes low.  Sets DRDYFlag to true.  MAX31856 resets the pin to HIGH after a read.
  if (Temp.Simulate) return;
  Temp.DRDYFlag = millis();
  if (DebugLevel == 17) Serial.println(F("DRDY"));
} //===============================================================================================
    //SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
    //SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
    //SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
    //SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
void setup() {
  //Start Serial at a low normal speed to show the load configuration
  Serial.begin(9600);
  Serial.println(F("\r\r\n\nSerial Output Started\r\n"));
  
  //Load Configuration from EEPROM
    Serial.print(F("Loading Configuration..."));
    loadStruct(&Config, sizeof(Config));
    if (Config.SerialNumber == 65535) {//first time running software, flash memory needs to be configured
      Serial.println(F("Program First Time Running, Setting Factory Default Configuration"));
      FactoryDefault();
      Restart();
    }
    else {
      Serial.println(F("OK"));
    }
    //Reserve some string space
    sUARTBuffer.reserve(UART_BUFFER_SIZE);
    sLastUARTLine.reserve(UART_BUFFER_SIZE);

    //give the user a brief chance set factory default settings at the very beggining
    Serial.print(F("\r\nSend '1' to reset BAUD rate to ")); Serial.println(UART_DEFAULT_SPEED);
    Serial.println(F("Send '2' to reset to Factory Default\r\n"));
    Serial.flush();
    uint32_t start = millis();
    while (millis() - start < 1000) {
			if (UARTRead()) {
				if (sLastUARTLine == F("1")) {
					Config.UARTSpeed = UART_DEFAULT_SPEED;
					storeStruct(&Config, sizeof(Config));
					Restart();
				}
				else if (sLastUARTLine == F("2")) {
					FactoryDefault();
					Restart();
				}
				else {
					Serial.println(F("Send 1 or 2"));
				}
				sLastUARTLine = "";
			};      
      delay(5);
    }

	String BaudMessage;
	BaudMessage = (F("\r\r\n\nSet baud rate to ")); BaudMessage += Config.UARTSpeed; BaudMessage += F("\r\r\n\n");
	Serial.print(BaudMessage);  Serial.flush();

	#ifdef ESP32
		Serial.SetbaudRate(Config.UARTSpeed);
	#endif  
	#ifdef ESP8266
		Serial.end();
		Serial.begin(Config.UARTSpeed);
	#endif    
	Serial.println(F("\r\r\n\nSerial Output Started"));

	Serial.println(F("Configuring MCU pins"));
	pinMode(CS_PIN, OUTPUT);
	digitalWrite(CS_PIN, HIGH);

	pinMode(OUTPUT_PIN, OUTPUT);
	digitalWrite(OUTPUT_PIN, LOW);

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);

	pinMode(A0, INPUT);

    pinMode(DRDY_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(DRDY_PIN), ISR_DRDY_PIN, FALLING);

  Serial.println(F("Starting SPI Bus"));
  #ifdef ESP8266
    SPI.begin();
    SPI.setHwCs(true);
  #endif
  #ifdef ESP32
    SPI.begin(SCKPIN, MISOPIN, MOSIPIN, CS_PIN);
    SPI.setHwCs(true);
  #endif

//Configure MAX31856
	Serial.print("Configuring MAX31856...");
	ReadTemperature();
	if (Temp.Fault > 0) {
		Serial.println("Failure");
		Serial.println(Temp.FaultMSG);
	}
	else {
		Serial.println("OK");		
	}


  pid.Kc = Config.pid.Kc;
  pid.Ti = Config.pid.Ti;
  pid.Td = Config.pid.Td;
  pid.SetOutputLimits(Config.pid.Min, Config.pid.Max);
  pid.Direction = Config.pid.Direction;
  pid.Mode = MANUAL;

  Reflow.PV = &Temp.Actual;
  Reflow.Step.Step = IDLE;


  //Initialize some timers
	uint32_t time = millis();
	Output.PeriodTimer = time;
	Reflow.Step.UpdateTimer = time;
	Temp.ConversionTimer = time;
	Temp.SimulateDRDYFlagTimer = time;
	HousekeepingTimer = time;
	SaveConfigTimer = time;
	DRDYWatchDogTimer = time;


#ifndef ENABLEWIFI
  #ifdef ESP8266
    WiFi.setSleepMode(WIFI_MODEM_SLEEP);
    WiFi.mode(WIFI_OFF);
  #endif
  #ifdef ESP32
    WiFi.mode(WIFI_MODE_NULL);
  #endif
#endif // !ENABLEWIFI

  BuzzerPowerOn.Trigger = true;
  Serial.println(WelcomeMessage());
}
//SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
//SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
//SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP
//SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUP

//LOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOP
//LOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOP
//LOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOP
//LOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOP
void loop() {
  HouseKeeping();
  
	if (Temp.DRDYFlag > 0 || millis() - DRDYWatchDogTimer > 2000) { //Read temperature after a WD timer in case communication is lost
		ReadTemperature();
		Temp.DRDYFlag = 0;
		DRDYWatchDogTimer = millis();
	}

  HandleStateEngine();
  pid.Compute(Temp.NewFlag); Temp.NewFlag = false;
  Output.DutyCycle = Reflow.CV; // Map the output of the PID loop to the DutyCycle when the PID is running

  HandleOutputs();

  if (Temp.Simulate && millis() - Temp.SimulateDRDYFlagTimer > Temp.ConversionTime) {
    Temp.SimulateDRDYFlagTimer = millis();
    Temp.DRDYFlag = true;
  }
}
//LOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOP
//LOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOP
//LOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOP
//LOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOP
//===============================================================================================
void HouseKeeping() {
  #ifdef EnableLoopStatistics
    if (DebugLevel == 10) {
      if (millis() - LoopStatisticsTimer >= 1000) {
        LoopStatisticsTimer = millis();
        LT.printStatistics();
      }
      LT.tick();
    }
  #endif

  //Process pushbutton inputs
    if (millis() != AIReadmillisLast) { //same logic is in PB libary, so only need to update this at each millis change
      AIReadmillisLast = millis();
      uint16_t tempAI = analogRead(A0);
      if (tempAI < 100) {
        PBInput = false;
      }
      if (tempAI > 900) {
        PBInput = true;
      }
    }
    PB.tick();    
    if (PB.UpRisingEdge && !PB.LongUpRisingEdge) {
      if (Reflow.Step.Step == IDLE) {//if IDLE, start
        Reflow.Status = true;
        Serial.println(F("Pushbutton START"));      
      }
      else if (Reflow.Step.Step == PAUSED) {//if PAUSED, restart
        Reflow.Step.Step = Reflow.Step.StepREM; //go back to the step you were in when you paused
        pid.Mode = AUTO;
        BuzzerButtonStartRestart.Trigger = 1;
        Serial.println(F("Pushbutton RESTART"));
      }
      else if (Reflow.Step.Step != IDLE && Reflow.Step.Step != PAUSED) { 
        Reflow.Step.StepREM = Reflow.Step.Step; //rmemeber what step you came from
        Reflow.Step.Step = PAUSED;
        pid.Mode = MANUAL;
        BuzzerButtonPause.Trigger = 1;
        Serial.println(F("Pushbutton PAUSED"));
      }
      else {
        BuzzerFeedbackNegative.Trigger = 1;
      }
    }
    if (PB.LongDownRisingEdge) {
      if (Reflow.Step.Step != IDLE) {
        Reflow.Status = false;
        Serial.println(F("Pushbutton Stop"));
      }
      else {
        BuzzerFeedbackNegative.Trigger = 1;
      }
    }
    if (BuzzerFeedbackNegative.Trigger) Serial.println(F("Pushbutton no action"));

  if (millis() - HousekeepingTimer > HousekeepingTime) {
    HousekeepingTimer = millis();
    ProcessSerialCommands();

    if (millis() - UpTimeTimer >= 1000) {
      UpTimeTimer = UpTimeTimer + 1000;
      __UPTIME++;
    }

    //if (millis() - Temp.DRDYFlag > 2000) MAX31856Read();

	if (SaveConfigCMD && (millis() - SaveConfigTimer > SaveConfigTime)) { //Save configuration if the flag is set
		storeStruct(&Config, sizeof(Config));
		Serial.println("\tConfiguration Saved");
		SaveConfigCMD = false;
		SaveConfigTimer = millis();
	}


    if (RestartCMD) Restart();
    BuzzerPowerOn.tick();
    BuzzerButtonPause.tick();
    BuzzerButtonStartRestart.tick();
    BuzzerFeedbackNegative.tick();
    BuzzerFeedbackPositive.tick();
    BuzzerPreHeatStart.tick();
    BuzzerSoakStart.tick();
    BuzzerReflowStart.tick();
    BuzzerCooldownStart.tick();
    BuzzerReflowComplete.tick();

    if (Temp.Fault > 0 && millis() - Temp.FaultMSGTimer > 10000) {
      Temp.FaultMSGTimer = millis();
      Serial.println(Temp.FaultMSG);
    }
    
  }

  HandleDashboard();
}//===============================================================================================
void HandleOutputs(void) {
  uint32_t now = millis();
  //SSR Output  
  
    if (now != Output.millsREM) { // check the Output software PWM each millisecond.  This is done for performance reasons to make the scan time faster
      Output.millsREM = now;

      //Enforce a Valid Duty Cycle
        if (Output.DutyCycle > 100.0) Output.DutyCycle = 100.0;
        if (Output.DutyCycle < 0.0) Output.DutyCycle = 0.0;
      
      Output.PWMTime = (uint16_t)(Output.DutyCycle / 100.0 * (double)Config.PWMPeriodTime);
      
      //Reset period timer if applicable
        Output.PWMPeriodTimeElapsed = now - Output.PeriodTimer;
        if (Output.PWMPeriodTimeElapsed >= Config.PWMPeriodTime) Output.PeriodTimer += Config.PWMPeriodTime;
        
      if (Output.PWMTime < Output.PWMPeriodTimeElapsed) {
        Output.Energize = false;
        digitalWrite(OUTPUT_PIN, false);
      }
      else {
        Output.Energize = true;
        digitalWrite(OUTPUT_PIN, true);
      }
    }
  //LED Output
    digitalWrite(LED_PIN, Output.Energize);
    
}//===============================================================================================
void HandleStateEngine(void) {
  uint32_t now = millis();
  if (now - Reflow.Step.UpdateTimer >= 47) {
    Reflow.Step.UpdateTimer = now;

    if (Reflow.Step.Step != IDLE) {
      Reflow.Time = __UPTIME - Reflow.Step.Preheat.StartTime; //accumulate Total Time
      if (Temp.Actual >= Reflow.MaxTemp) Reflow.MaxTemp = Temp.Actual; //Record Max Temperature
      if (Reflow.Status==false) Reflow.Step.Step = COMPLETE;
    }

    //Time how long the temperature is over the MinReflowTemp and adjust the timer accordingly
      if (Reflow.Step.Step == REFLOW || Reflow.Step.Step == COOL) {

        if (Temp.Actual >= Config.Profile[0].ReflowPeakTemp - 5) {//When you are above near the peak start recording
          if (Reflow.ReflowPeakTimer.ONS == false) {//Start a Timer
            Reflow.ReflowPeakTimer.ONS = true;
            Reflow.ReflowPeakTimer.Timer = __UPTIME;
          }
          Reflow.ReflowPeakTimer.Time = __UPTIME - Reflow.ReflowPeakTimer.Timer; //Record the time spent above the MinReflowTemp
        }
        else {
          Reflow.ReflowPeakTimer.ONS = false;
        }


        if (Temp.Actual >= Config.Profile[0].SoakTemp){//When you are above the minimum reflow temp start a timer
          if (Reflow.AboveSoakTempTimer.ONS == false) {//Start a Timer
            Reflow.AboveSoakTempTimer.ONS = true;
            Reflow.AboveSoakTempTimer.Timer = __UPTIME;
          }
          Reflow.AboveSoakTempTimer.Time = __UPTIME - Reflow.AboveSoakTempTimer.Timer; //Record the time spent above the MinReflowTemp
        }
        else {
          Reflow.AboveSoakTempTimer.ONS = false;
        }
      }
      else {
        Reflow.AboveSoakTempTimer.ONS = false;
        Reflow.ReflowPeakTimer.ONS = false;
      }


    switch (Reflow.Step.Step) {
      case IDLE:
        if (Reflow.Status == true) {
          pid.Mode = AUTO;

          //Reset all timers and data from the previous reflow
            Reflow.Step.Preheat.Time = 0;
            Reflow.Step.Soak.Time = 0;
            Reflow.Step.Reflow.Time = 0;
            Reflow.Step.Cool.Time = 0;
            Reflow.Step.Preheat.Time = 0;
            Reflow.Time = 0;
            Reflow.ReflowPeakTimer.Time = 0;
            Reflow.AboveSoakTempTimer.Time = 0;

            Reflow.MaxTemp = Temp.Actual;
          Reflow.Step.Preheat.StartTime = __UPTIME;
          Reflow.Step.Step = PREHEAT;
          BuzzerPreHeatStart.Trigger = true;
        }
        break;    
      case PREHEAT:       
        Reflow.Step.Preheat.Time = __UPTIME - Reflow.Step.Preheat.StartTime;//Time this step                
        Reflow.SP = Config.Profile[0].PreHeatRate;
        Reflow.PV = &Temp.Rate; 
        Reflow.Step.TempSetpoint = Config.Profile[0].PreHeatTemp;
        Reflow.Step.RateSetpoint = Config.Profile[0].PreHeatRate;

        //If your Ramp rate is reasonable, then change your setpoint a bit early to slow it down
          if (Temp.Rate > Reflow.Step.RateSetpoint * 0.5 && Temp.Actual >= Reflow.Step.TempSetpoint - Temp.Rate * Config.pid.DeadTime) {
            Reflow.SP = Config.Profile[0].SoakRate;
            Reflow.Step.RateSetpoint = Config.Profile[0].SoakRate;
          }
        //Transition
          if (Temp.Actual >= Config.Profile[0].PreHeatTemp) {
            Reflow.Step.Soak.StartTime = __UPTIME;
            Reflow.Step.Step = SOAK;
            BuzzerSoakStart.Trigger = true;
          }
        break;
      case SOAK:        
        Reflow.Step.Soak.Time = __UPTIME - Reflow.Step.Soak.StartTime;//Time this step
        Reflow.SP = Config.Profile[0].SoakRate;
        Reflow.PV = &Temp.Rate;
        Reflow.Step.TempSetpoint = Config.Profile[0].SoakTemp;
        Reflow.Step.RateSetpoint = Config.Profile[0].SoakRate;

        //If your Ramp rate is reasonable, then change your setpoint a bit early to speed it up
          if (Temp.Rate > Reflow.Step.RateSetpoint * 0.5 && Temp.Actual >= Reflow.Step.TempSetpoint - Temp.Rate * Config.pid.DeadTime) {
            Reflow.SP = Config.Profile[0].ReflowRate;
            Reflow.Step.RateSetpoint = Config.Profile[0].ReflowRate;
          }
          //Transition
          if (Temp.Actual >= Config.Profile[0].SoakTemp) {
            Reflow.Step.Reflow.StartTime = __UPTIME;
            Reflow.Step.Step = REFLOW;
            BuzzerReflowStart.Trigger = true;
          }
          break;
      case REFLOW:
        Reflow.Step.Reflow.Time = __UPTIME - Reflow.Step.Reflow.StartTime;//Time this step
        Reflow.SP = Config.Profile[0].ReflowRate;
        Reflow.PV = &Temp.Rate;
        Reflow.Step.TempSetpoint = Config.Profile[0].ReflowPeakTemp;
        Reflow.Step.RateSetpoint = Config.Profile[0].ReflowRate;


        if (Temp.Actual >= Config.Profile[0].ReflowPeakTemp - Config.Profile[0].ReflowPeakTempPreact) {
          Reflow.SP = Config.Profile[0].CoolDownRate;//constrain setpoint when it goes over
          Reflow.Step.RateSetpoint = Config.Profile[0].CoolDownRate;
        }
        //Transition
        if (Reflow.ReflowPeakTimer.Time >= Config.Profile[0].ReflowPeakTime - Config.Profile[0].ReflowPeakTimePreact) { //When the temperature peaks and starts to go down
          Reflow.Step.Cool.StartTime = __UPTIME;
          Reflow.Step.Step = COOL;
          BuzzerCooldownStart.Trigger = true;
        }

        break;
      case COOL:
        Reflow.Step.Cool.Time = __UPTIME - Reflow.Step.Cool.StartTime;//Time this step
        Reflow.SP = Config.Profile[0].CoolDownRate;
        Reflow.PV = &Temp.Actual;
        Reflow.Step.TempSetpoint = Config.Profile[0].MaxStartTemp;
        Reflow.Step.RateSetpoint = Config.Profile[0].CoolDownRate;

        //Transition
        if (Temp.Actual <= Config.Profile[0].MaxStartTemp) {
          Reflow.Step.Step = COMPLETE;
          BuzzerCooldownStart.Trigger = true;

          //Correct for going over or under the peak temp
          double error = Reflow.MaxTemp - Config.Profile[0].ReflowPeakTemp;
          Config.Profile[0].ReflowPeakTempPreact += error * 0.5;
          Config.Profile[0].ReflowPeakTempPreact = constrain(Config.Profile[0].ReflowPeakTempPreact, 0, 30); //Constrain to a reasonable value in case the error is wild

                                        //Correct for going over or under the peak time
          error = Reflow.ReflowPeakTimer.Time - Config.Profile[0].ReflowPeakTime;
          Config.Profile[0].ReflowPeakTimePreact += error * 0.5;
          Config.Profile[0].ReflowPeakTimePreact = constrain(Config.Profile[0].ReflowPeakTimePreact, 0, Config.Profile[0].ReflowPeakTime);//Constrain to a reasonable value in case the error is wild

          SaveConfigCMD = true;

        }
        break;
      case COMPLETE:

        Reflow.Step.Step = IDLE;
        BuzzerReflowComplete.Trigger = true;
        Reflow.Status = false;
        pid.Mode = MANUAL; //Turn off PID control
        Reflow.CV = pid.CVMin; //Turn PWM to 0%
        break;
    }

  }


}//===============================================================================================


String getStateString(void) {
  String s;
  switch (Reflow.Step.Step) {
  case IDLE:
    s = F("IDLE, Ready to Start");
    break;
  case PREHEAT:
    s = F("PREHEAT to "); s += String(Config.Profile[0].PreHeatTemp, 1); s += F("C at ");  s += String(Config.Profile[0].PreHeatRate, 1); s += F("C/s");
    break;
  case SOAK:
    s = F("SOAK up to "); s += String(Config.Profile[0].SoakTemp, 1); s += F("C at ");  s += String(Config.Profile[0].SoakRate, 1); s += F("C/s");
    break;
  case REFLOW:
    s = F("REFLOW to "); s += String(Config.Profile[0].ReflowPeakTemp, 1); s += F("C at ");  s += String(Config.Profile[0].ReflowRate, 1); s += F("C/s");
    break;
  case COOL:
    s = F("COOL back down to "); s += String(Config.Profile[0].MaxStartTemp, 1); s += F("C at ");  s += String(Config.Profile[0].CoolDownRate, 1); s += F("C/s");
    break;
  case COMPLETE:
    s = F("COMPLETE");
    break;
  case PAUSED:
    s = F("PAUSED.");
    break;
  }
  return s;
}//===============================================================================================
void MAX31856GetFaultString(String &s, uint8_t &Fault = Temp.Fault) {

	s = "";
	if (Fault > 0) {	
		s += F("Thermocouple ERROR 0x"); s += String(Temp.Fault, HEX); s += F(":\r\n");
		if (Fault & 0x20) {
			s += F("\tERROR: Communication failure to MAX31856.\r\n"); //bit 05
		}
		else {
			if (Fault & 0x80) s += F("\tERROR: The Cold-Junction temperature is outside of the normal operating range.\r\n"); //bit 07, directly from MAX31856 SR register
			if (Fault & 0x40) s += F("\tERROR: The Thermocouple Hot Junction temperature is outside of the normal operating range.\r\n"); //bit 06, directly from MAX31856 SR register
			//if (Fault && 0x16) s += F("\tERROR: UNUSED ERROR.\r\n"); //bit 04
			//if (Fault && 0x08) s += F("\tERROR: UNUSED ERROR.\r\n"); //bit 03
			//if (Fault && 0x04) s += F("\tERROR: UNUSED ERROR\r\n"); //bit 02
			if (Fault & 0x02) s += F("\tERROR: The input voltage is negative or greater than VDD\r\n"); //Bit01, directly from MAX31856 SR register
			if (Fault & 0x01) s += F("\tERROR: An open circuit such as broken thermocouple wires has been detected.\r\n");//Bit00, directly from MAX31856 SR register
		}
	}
	else {
		s += F("Thermocouple Status OK");
	}

}//===============================================================================================
void ReadTemperature() {  

	Temp.Fault = 0;


	if (Temp.Simulate) { //simulate a conversion time
		Temp.ConversionTime = -1;
		Temp.FaultMSG = F("Thermocouple Status SIMULATED");

		//Simulate temperature
		if (Output.Energize) HeatingElementsActual += -(HeatingElementsActual - 650)*HeatingElementsTempSpeedIncrease;
		HeatingElementsActual += - (HeatingElementsActual - Temp.Actual)*HeatingElementsTempSpeedDecrease; //decrease temperature proportional to the temperature delta 
		HeatingElementsTemp = HeatingElementsTempLPF.step(HeatingElementsActual);
		OutputDeadtime[DeadtimeSize - 1] = Output.DutyCycle; //Move the current desired output into the last element of the array
		for (int i = 1; i <DeadtimeSize ; i++) { //simulate process dead time
			OutputDeadtime[i - 1] = OutputDeadtime[i];
			//Serial.print("i="); Serial.print(i); Serial.print("="); Serial.println(OutputDeadtime[i]);
		}
		double DelayedOutput = OutputLPF.step(OutputDeadtime[0]); //move the first element of the deadtime array into a LPF to simulate process lag
		Temp.Actual = Temp.Actual + DelayedOutput /100*1.2; //Increase temperature proportional to output
		Temp.Actual = Temp.Actual - (Temp.Actual - 20)*0.00075; //decrease temperature proportional to the temperature delta 
		//if (Reflow.Step.Step == COOL) Temp.Actual = Temp.Actual - (Temp.Actual - 20)*0.01; //decrease temperature proportional to the temperature delta 
	}
	else { 
		//calculate the time interval from the previous call to this function for debugging purposes
		if (Temp.DRDYFlag > 0) { 
			uint16_t Tconv = Temp.DRDYFlag - Temp.ConversionTimer;
			if (Tconv > 0.5 * MAX31856.Tconv && Tconv < 2 * MAX31856.TconvMax) {
				Temp.ConversionTime = Tconv; //gross error checking or starting the conversion process.  At no time should there ever be a conversion aboove like 650ms
			}
			else {
				Temp.ConversionTime = 0;
			}
			Temp.ConversionTimer = Temp.DRDYFlag;  //remember for next time
		}
		else { //This would only happen if this routine is triggered from the DRDYWatchDogTimer
			Temp.ConversionTime = 0;
		}

		MAX31856Read();	//Update all of registers.
						
		if (MAX31856.REG.CR0.WORD != Config.MAX31856.REG.CR0.WORD) {
			MAX31856Write(&Config.MAX31856);	//Try to send current configuration
			MAX31856Read();				//Update all of registers.
			if (MAX31856.REG.CR0.WORD != Config.MAX31856.REG.CR0.WORD) Temp.Fault = bitSet(Temp.Fault, 5); //if you still don't match after sending configuration, you have no connection
		}

		//Check for any TC faults and calculate
		if ((Temp.Fault & 0x20)==0) Temp.Fault = MAX31856.REG.SR.WORD & 0xC3; //As long as you are communicating, Filter out bits 2, 3, 4, 5 as we really don't care about the threshold settings
		MAX31856GetFaultString(Temp.FaultMSG);
		if (Temp.Fault == 0) {
			//Calculate Temperature
			Temp.Actual = TempActualLPF.step(MAX31856.LTCT);
			Temp.NewFlag = true;
			//Calculate Rate
			if (Temp.ConversionTime > 0.5 * MAX31856.Tconv && Temp.ConversionTime < 2 * MAX31856.TconvMax && Temp.ConversionTime != 0) { //Only if the conversion time is reasonable to prevent gigantic numbers
				Temp.Rate = TempRateLPF.step(Temp.Actual - Temp.ActualPrev) / (double(Temp.ConversionTime) / 1000.0); //Pass Rate through a LPF
				if (Temp.Rate < 0.005 && Temp.Rate > -0.005) Temp.Rate = 0;//Floor the Rate if its under the noise floor          
			}
			else {
				Temp.Rate = 0.0; 
			}
			Temp.ActualPrev = Temp.Actual; //remember for next time  
		}
		else {
			Temp.Actual = 9999.0; //Set a value when there is a fault condition
		}

		//Check for the only fault that affects the CJ and calculate
		if (MAX31856.REG.SR.CJ_Range == false) {
			Temp.CJ = LPFCJTemp.step(MAX31856.CJT);
		}
		else {
			Temp.CJ = 9999.0;//Set a value when there is a fault condition
		}
	}

	if (Temp.FaultREM != Temp.Fault) {//Fault Status Changed
		if (Temp.Fault == 0)  {
			Serial.println("\tThermocouple Status OK");
		}
		else {
			Serial.println(Temp.FaultMSG);
		}
	}
	Temp.FaultREM = Temp.Fault;
} //===============================================================================================
void HandleDashboard(void) {
  if (Dashboard & 1) {
    if (millis() - DashboardTimer > DashboardTime) {
      DashboardTimer = millis();

      String temp = getDashboard();
      Serial.println(F("\r\r\n\r\n\nNOTE:If you can see this, you are not using the correct Terminal Emulator.  Try Putty Instead\r\r\n\r\n\n"));
      SerialClearScreen();
      Serial.println(temp);
    }
  }
}//===============================================================================================
void GetConfigString(String &s) {
  MAX31856Read();
  s = Line();
  s += F("MAX31856 Register Mapping:\r\n");
  s += Line();
  MAX31856GetREGMapString(s);
  MAX31856GetREGMapStringCR0(s);
  MAX31856GetREGMapStringCR1(s);
  s += F("\t"); MAX31856GetEstimatedConversionTimeString(s); s += F("\r\n");
  s += F("	Actual Last Conversion Time: "); s += Temp.ConversionTime; s += F("ms"); s += F("\r\n");
  MAX31856GetREGMapStringTemp(s);
  MAX31856GetREGMapStringFAULT(s);
	s += Temp.FaultMSG; s += F("\r\n");
  s += Line();
} //===============================================================================================
void ProfileClear(struct ReflowProfileStruct &p) {
  p.MaxStartTemp = 0;
  p.PreHeatRate = 0;
  p.PreHeatTemp = 0;
  p.SoakRate = 0;
  p.SoakTemp = 0;
  p.ReflowRate = 0;
  p.ReflowPeakTemp = 0;
  p.ReflowPeakTempPreact = 0;
  p.ReflowPeakTime = 0;
  p.ReflowPeakTimePreact = 0;
  p.CoolDownRate = 0;
}//===============================================================================================
void FactoryDefault(void) {
  String temp;
  Config.DebugLevel = 0;
  temp = F("0.1"); temp.toCharArray(Config.SoftwareFirmwareVersion, sizeof(Config.SoftwareFirmwareVersion));
  temp = __DATE__; temp.toCharArray(Config.CalibrationDate, sizeof(Config.CalibrationDate));
  temp = F("ESPReflow"); temp.toCharArray(Config.ModelNumber, sizeof(Config.ModelNumber));
  Config.SerialNumber = 1;
  Config.pid.Kc = 35.0;
  Config.pid.Ti = 217;
  Config.pid.Td = 5.0;
  Config.pid.Min = 0;
  Config.pid.Max = 100;
  Config.pid.Direction = DIRECT;
  Config.pid.DeadTime = 5.0;

  //load 0's for all profiles
  for (int i = 0; i < NUMPROFILES; i++) {
    ProfileClear(Config.Profile[i]);
  }
  //Leaded Profile
  Config.DefaultProfile[0].MaxStartTemp = 50;
  Config.DefaultProfile[0].PreHeatRate = 3;
  Config.DefaultProfile[0].PreHeatTemp = 125;
  Config.DefaultProfile[0].SoakRate = 0.5;
  Config.DefaultProfile[0].SoakTemp = 183;
  Config.DefaultProfile[0].ReflowRate = 3; 
  Config.DefaultProfile[0].ReflowPeakTemp = 225;
  Config.DefaultProfile[0].ReflowPeakTempPreact = 7;
  Config.DefaultProfile[0].ReflowPeakTime = 20;
  Config.DefaultProfile[0].ReflowPeakTimePreact = 5;
  Config.DefaultProfile[0].CoolDownRate = -5;

  //Lead-free
  Config.DefaultProfile[1].MaxStartTemp = 50;
  Config.DefaultProfile[1].PreHeatRate = 3;
  Config.DefaultProfile[1].PreHeatTemp = 150;
  Config.DefaultProfile[1].SoakRate = 0.5;
  Config.DefaultProfile[1].SoakTemp = 217;
  Config.DefaultProfile[1].ReflowRate = 1.5;
  Config.DefaultProfile[1].ReflowPeakTemp = 250;
  Config.DefaultProfile[1].ReflowPeakTempPreact = 7;
  Config.DefaultProfile[1].ReflowPeakTime = 30;
  Config.DefaultProfile[1].ReflowPeakTimePreact = 5;
  Config.DefaultProfile[1].CoolDownRate = -5;

  Config.Profile[0] = Config.DefaultProfile[0];//Set Active Profile to Loaded profile

  Config.PWMPeriodTime = 5000;
  Config.UARTSpeed = UART_DEFAULT_SPEED;
	Config.TabLength = 8;
  MAX31856SetDefaultConfig();
  storeStruct(&Config, sizeof(Config));
  SaveConfigCMD = true;
  Serial.println F("  Factory Default Settings Restored to Configuration");
} //===============================================================================================
void MAX31856SetDefaultConfig(void) {
	MAX31856SetFactoryDefault(&Config.MAX31856); //Set the Factory Default Values to the Config Struct (also sends it to IC but that's ok)
	//Modify a few of the values for project specific things
	  Config.MAX31856.REG.CR0.CMODE = true;		//Auto conversion,
	  Config.MAX31856.REG.CR0.OCFAULT0 = false;	//Open Circuit detection
	  Config.MAX31856.REG.CR0.OCFAULT1 = true;
	  Config.MAX31856.REG.CR1.AVGSEL = 4;		//16 samples averaged
	MAX31856WriteRegisters(&Config.MAX31856); //Write the new registers to the IC
	MAX31856ReadRegisters(); //Read back the registers into the running memory
	SaveConfigCMD = true;
} //===============================================================================================


void Restart() {  
  Serial.print("REBOOTING"); //tell the serial monitor
  Serial.flush(); //flush all outgoing serial data to client
  delay(100);//needed to ensure everything went out
  ESP.restart();
}//===============================================================================================
bool UARTRead(void) {
  /*Read hardware serial port and build up a string.  When a newline, carriage return, or null value is read consider this the end of the string
  RETURN 0 when no new full line has been received yet
  RETURN 1 when a new full line as been received.  The new line is put into sLastUARTLine
  */
  //Returns 0 when no new full line, 1 when there is a new full line
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r' || inChar == '\0') {//if the incoming character is a LF, CR, or Null finish the string
                                 //flush the rest of the hardware buffer in case more than one "end of line character" is received.
      while (Serial.available()) {
        char throwaway = Serial.read();
      }
      sLastUARTLine = sUARTBuffer; //Transfer the entire string into the last serial line
      sUARTBuffer = ""; // clear the string buffer to prepare it for more data:
      return true;
    }
    sUARTBuffer += inChar;// add it to the inputString:
  }
  return false;
}//===============================================================================================
void StringSplit(String &text, String *StringSplit, const char * delimiter) {
  /*
  Perform a CSV string split.
  INPUTS:
  text - CSV string to separate
  StringSplit - Pointer to an array of Strings
  DESCRIPTION:
  text can have CSV or not
  Each CSV is placed into a different index in the Array of Strings "StringSplit", starting with element 1.
  Element 0 (StringSplit[0]) will contain the number of CSV found.  Rember to use String.toInt() to convert element[0]
  */

  char *p;
  char buf[256]; //create a char buff array
           //text += "\0"; //add null string terminator
  text.toCharArray(buf, 64); //convert string to char array

                 //Split string
  byte PTR = 1; //Indexer
  p = strtok(buf, delimiter);
  do
  {
    StringSplit[PTR] = p;//place value into array
    PTR++; //increment pointer for next loop
    p = strtok(NULL, delimiter); //do next split
  } while (p != NULL);

  //Place the number of CSV elements found into element 0
  StringSplit[0] = String(PTR - 1);

}//===============================================================================================
void SerialClearScreen(void) {
  Serial.write(27); // ESC
  Serial.print(F("[2J")); // clear screen
  Serial.write(27); // ESC
  Serial.print(F("[H")); // cursor to home
}//===============================================================================================

String getDashboard(void) {
  String s;

  s += Line();
  s += Config.ModelNumber; s += F("\tSystem Uptime: "); s += getUpTimeString(__UPTIME); s += F("\r\n");
  if (Temp.Simulate) {
    s += F("TEMPERATURE SIMULATION IS ON"); s += F("\r\n");
  }
  s += Line();
  s += F("State =   "); s += getStateString(); s += F("\r\n");
  s += F("\r\n");
  s += StringIndent(F("Description"), 3); s += F("sp\t");  s += F("act\t");   s += F("EngUnits\r\n");
  s += F("--------------------------------------------------\r\n");
  s += StringIndent(F("Temperature"), 3);	s += String(Reflow.Step.TempSetpoint, 1);			s += F("\t"); s += String(Temp.Actual, 2);    s += F("\tC\r\n");
  s += StringIndent(F("Ramp Rate"), 3);		s += String(Reflow.Step.RateSetpoint, 1);			s += F("\t"); s += String(Temp.Rate, 4);      s += F("\tC/s\r\n");
  s += StringIndent(F("Output"), 3);		s += F("\t"); s += String(Output.DutyCycle, 1);		s += F("\t%     ("); (Output.Energize) ? s += F("ON") : s += F("OFF"); s += F(")\r\n");
  s += StringIndent(F("Output.PWMTime"), 3);	s += F("\t"); s += Output.PWMTime;					s += F("\tms\r\n");
  s += StringIndent(F("Output.PWMPeriodTimeElapsed"), 3);	s += Output.PWMPeriodTimeElapsed;		s += F("\tms\r\n");
  s += StringIndent(F("HeatingElementsTemp"), 3);	s += HeatingElementsTemp;					s += F("\tC\r\n");
  s += F("\r\n");
  s += StringIndent(F("PreHeat Time"), 3);                           s += F("\t"); s += Reflow.Step.Preheat.Time;    s += F("\ts\r\n");
  s += StringIndent(F("Soak Time"), 3);                           s += F("\t"); s += Reflow.Step.Soak.Time;     s += F("\ts\r\n");
  s += F("Reflow Time (>"); s += String(Config.Profile[0].SoakTemp,0); s += F("C)\t "); s += Reflow.AboveSoakTempTimer.Time;    s += F("\ts\r\n");
  s += F("At Peak Temp(>"); s += String(Config.Profile[0].ReflowPeakTemp - 5,0); s += F("C)\t");  s += Reflow.ReflowPeakTimer.Time;   s += F("\ts\r\n");
  s += StringIndent(F("Max Temp"), 3);                           s += F("\t"); s += String(Reflow.MaxTemp, 2);   s += F("\tC\r\n");
  s += StringIndent(F("Cool Down Step Time"), 3);                           s += F("\t"); s += Reflow.Step.Cool.Time;     s += F("\ts\r\n");
  s += StringIndent(F("Total Time"), 3);                           s += F("\t"); s += Reflow.Time;         s += F("\ts\r\n");
  s += F("\r\n");
  if (Dashboard & 2) { //if bit 1 is set, then display the pid stuff
    s += F("PID Parameters:\r\n");
    s += F("PID.SP =    ");   s += String(Reflow.SP, 3);  s += F("\t%\r\n");
    s += F("PID.PV =    ");   s += String(*Reflow.PV, 4); s += F("\t%\r\n");
    s += F("PID.CV =    ");   s += String(Reflow.CV, 3);  s += F("\t%\r\n");
    s += F("pid.Kc =    ");   s += String(pid.Kc, 6); s += F("\r\n");
    s += F("pid.Ti =    ");   s += String(pid.Ti, 6); s += F("\r\n");
    s += F("pid.Td =    ");   s += String(pid.Td, 6); s += F("\r\n");
    s += F("pid.KcInternal =  ");   s += String(pid.KcInternal, 6); s += F("\r\n");
    s += F("pid.TiInternal =  ");   s += String(pid.TiInternal, 6); s += F("\r\n");
    s += F("pid.TdInternal =  ");   s += String(pid.TdInternal, 6); s += F("\r\n");
    s += F("pid.error =   ");   s += String(pid.error, 6); s += F("\r\n");
    s += F("pid.dInput =    ");   s += String(pid.dPV, 6); s += F("\r\n");
    s += F("pid.UpdateTime =  ");   s += pid.UpdateTime; s += F("\r\n");
    s += F("pid.RateTimer =   ");   s += pid.RateTimer; s += F("\r\n");
    s += F("pid.RateTime =    ");   s += pid.RateTime; s += F("\r\n");
    s += F("pid.ISum =    ");   s += String(pid.ISum, 6); s += F("\r\n");
    s += F("pid.outMin =    ");   s += pid.CVMin; s += F("\r\n");
    s += F("pid.outMax =    ");   s += pid.CVMax; s += F("\r\n");
    s += F("Config.pid.DeadTime = ");   s += Config.pid.DeadTime; s += F("\r\n");
    s += F("Output = "); s += String(pid.KcInternal *pid.error, 6); s += F(" ("); s += String(pid.Kc, 2); s += F(" * "); s += String(pid.error, 6); s += F(") + ");
    s += pid.ISum; s += F(" ("); s += String(pid.KcInternal, 2); s += F(" / "); s += String(pid.TiInternal, 6); s += F(" * ");  s += String(pid.error, 6); s += F(") - ");
    s += String(pid.KcInternal * pid.TdInternal * pid.dPV, 6);  s += F(" ( ");  s += String(pid.KcInternal, 2); s += F(" * ");  s += String(pid.TdInternal, 6); s += F(" * "); s += String(pid.dPV, 6); s += F(")\r\n");
    s += F("\r\n");
  }

  s += String(Temp.CJ,2); s += F("C (CJ)\r\n");
  s += F("\r\n");
  s += Temp.FaultMSG;
  s += F("\r\n");
  s += GlobalErrorMessage;  s += F("\r\n");
  s += F("(Note: send 'dashboard' or '?' to exit dashboard)"); s += F("\r\n");
  s += Line();
  return s;
}//===============================================================================================
void GetProfileString(String &s, uint8_t x = 0) {
  //Serial.println("here0"); Serial.flush(); delay(10);
  
  if (x < NUMPROFILES) {
		s += F("\t");		s += StringIndent(F("MaxStartTemp"),2);  s += String(Config.Profile[x].MaxStartTemp, 0);     s += F(" C\r\n\r\n");
   // Serial.println(Config.Profile[x].MaxStartTemp); Serial.flush(); delay(10);
  //  Serial.println(String(Config.Profile[x].MaxStartTemp, 0)); Serial.flush(); delay(10);
   
   // Serial.println("here"); Serial.flush(); delay(10);

    
		s += F("\t");		s += StringIndent(F("PreHeatRate"),2);   s += String(Config.Profile[x].PreHeatRate, 1);    s += F(" C/s\r\n");
		s += F("\t");		s += StringIndent(F("PreHeatTemp"), 2);   s += String(Config.Profile[x].PreHeatTemp, 0);    s += F(" C\r\n\r\n");
		s += F("\t");		s += StringIndent(F("SoakRate"), 2);    s += String(Config.Profile[x].SoakRate, 1);     s += F(" C/s\r\n");
		s += F("\t");		s += StringIndent(F("SoakTemp"), 2);    s += String(Config.Profile[x].SoakTemp, 0);     s += F(" C\r\n\r\n");
   // Serial.println("here2"); Serial.flush(); delay(10);
		s += F("\t");		s += StringIndent(F("ReflowRate"), 2);    s += String(Config.Profile[x].ReflowRate, 1);   s += F(" C/s\r\n");
		s += F("\t");		s += StringIndent(F("ReflowPeakTemp"), 2);  s += String(Config.Profile[x].ReflowPeakTemp, 0); s += F(" C\r\n");
		s += F("\t");		s += StringIndent(F("ReflowPeakTempPreact"), 3); s += String(Config.Profile[x].ReflowPeakTempPreact, 1); s += F(" C\r\n");
		s += F("\t");		s += StringIndent(F("ReflowPeakTime"), 2);  s += String(Config.Profile[x].ReflowPeakTime, 1); s += F(" s\n\n");
   // Serial.println("here4"); Serial.flush(); delay(10);
		s += F("\t");		s += StringIndent(F("ReflowPeakTimePreact"), 2); s += String(Config.Profile[x].ReflowPeakTimePreact, 1); s += F(" s\r\n\r\n");
		s += F("\t");		s += StringIndent(F("CoolDownRate"), 2);  s += String(Config.Profile[x].CoolDownRate, 1);   s += F(" C/s");
  }
  else
  {
    s += F("ERROR: Profile Number out of range");
  }

}//===============================================================================================
void HelpMenu(String& s) {
  s += F("\r\n");
  s += Line();//====================================================================
  s += F("HELP MENU\r\n");
  s += Config.ModelNumber;  s += F("\r\n");
  s += F("\r\n");
  s += F("COMMANDS:\r\n");
  s += F("\tAll commands are always accessible.  The different menus below are for help menu organization only:\r\n");
  s += F("\tCommand are not case sensitive. e.g. 'Restart' or 'restart' will still restart the system\r\n\r\n");
	s += F("\t");		s += StringIndent(F("general"), 2);		s += F("List general system commands : \r\n");
	s += F("\t");   s += StringIndent(F("temp"), 2);			s+= F("List temperature related commands\r\n");
	s += F("\t");   s += StringIndent(F("pid"), 2);				s+= F("List PID related commands\r\n");
	s += F("\t");		s += StringIndent(F("profile"), 2);		s+= F("Edit/Save/Load Reflow Profiles\r\n");
	s += F("\t");		s += StringIndent(F("(dash)board"), 2); s+= F("Start/Stop system dashboard (optional parameter: refresh time in ms.  default is 500.  e.g. 'dashboard 100')\r\n");
  s += F("\r\n");
  s += F("\r\n");
  s += Line();//====================================================================
}//===============================================================================================
void TemperatureMenu(String& s) {
  s += F("\r\n");
  s += Line();//====================================================================
  s += F("HELP MENU>Temperature \r\n");
  s += Config.ModelNumber;  s += F("\r\n");
  s += F("\r\n");
  if (Temp.Simulate) {
    s += F("TEMPERATURE SIMULATION IS ON"); s += F("\r\n");   
  } 
  s += F("Temp =    "); s += String(Temp.Actual, 1); s += F("C  "); s += Temp.Rate; s += F("C/s \r\n");
  s += F("COMMANDS:\r\n");
  s += F("\t'temp ?' - this help menu\r\n");
  s += F("\t'temp status' - Display MAX31856 data registers\r\n");
  s += F("\t'temp sim' - turn on/off temperature simulation\r\n");
  s += F("\t"); if (!Temp.Simulate) s += F("(DISABLED)"); s += F("'temp <value>' - Force the temperature to a value.  Only works if simulation is on\r\n");
  s += F("\t'temp avg <0/1/2/4/8/16>' - Write to the MAX31856 to average 2, 4, 8, or 16 values to reduce noise.  0 and 1 disable this feature\r\n");
  s += F("\t'temp Default' - Set the MAX31856 data registers to the default values from this program\r\n");
  s += F("\t'temp FactoryDefault' - Set the MAX31856 data registers to the factory default values from the data sheet. (really only useful for testing this codes implementation on a new chip)\r\n");
  s += F("\r\n");
  s += F("\r\n");
  s += Line();//====================================================================
}//===============================================================================================
void HelpMenuGeneral(String& s) {
  s += F("\r\n");
  s += Line();//====================================================================
  s += F("HELP MENU>General\r\n");
  s += Config.ModelNumber;  s += F("\r\n");
  s += F("\r\n");
  s += F("COMMANDS:\r\n");
  s += F("\t'output <value>' - Manually set output from 0-100%\r\n");
  s += F("\t'start' - Starts / Restarts reflow sequence\r\n");
  s += F("\t'pause' - Temporarilty pause reflow sequence.  Turn off all outputs\r\n");
  s += F("\t'stop' - Stop reflow sequence\r\n");
  s += F("\t'sp <value>' - Set the temperature setpoint manually\r\n");
  s += F("\t'system' - display system information\r\n");
  s += F("\t'save' - save current settings now\r\n");
  s += F("\t'TabLength <value>' - Configure your terminals tab length in number of characters for better formatted tables. eg 'TabLength 8'.  Current Value = "); s += Config.TabLength; s += F(" \r\n");
  s += F("\t'restart' - reboot the unit\r\n");
  s += F("\t'FactoryDefault' - Reset configuration to Factory Default (pid parameters, Profiles, etc)\r\n");
  s += F("\tDebugging tools : \r\n");
  s += F("\t\t'debug X' - to turn debugging code on/off, optional level parameter.  eg 'debug 4' \r\n");
  s += F("\r\n");
  s += Line();//====================================================================
}//===============================================================================================
void PIDMenu(String& s) {
  s += F("\r\n");
  s += Line();//====================================================================
  s += F("HELP MENU>pid\r\n");
  s += Config.ModelNumber;  s += F("\r\n");
  s += F("\r\n");
  s += F("\tMode: "); (pid.Mode) ? s += F("AUTOMATIC") : s += F("MANUAL"); s += F("\r\n");
  s += F("\tKc = "); s += String(pid.Kc, 5); s += F("\r\n");
	s += F("\tTi = "); s += String(pid.Ti, 5); s += F("\r\n");
	s += F("\tTd = "); s += String(pid.Td, 5); s += F("\r\n");
	s += F("\tSP = "); s += Reflow.SP; s += F("\r\n");
	s += F("\tPV = "); s += *Reflow.PV; s += F("\r\n");
	s += F("\tCV = "); s += Reflow.CV; s += F(" %\r\n");
	s += F("\tMin = "); s += String(pid.CVMin,1); s += F("\r\n");
	s += F("\tMax = "); s += String(pid.CVMax, 1); s += F("\r\n");
	s += F("\tDeadTime = "); s += String(Config.pid.DeadTime, 1); s += F(" s\r\n");
	s += F("\tSSRPWMTime = "); s += String(Config.PWMPeriodTime); s += F(" ms\r\n");
	s += F("\\rDirection = "); s += Config.PWMPeriodTime; s += F("\r\n");
	
	s += F("COMMANDS:\r\n");
	s += F("\t'pid ?' - this help menu\r\n");
	s += F("\t'sp <value>' - Set the temperature setpoint manually\r\n");
	s += F("\t'pid auto' - Sets PID mode to AUTOMATIC\r\n");
	s += F("\t'pid man' - Sets PID mode to MANAUL / Off \r\n");
	s += F("\t'pid Kc <value>' - Sets Kc to the <value>\r\n");
	s += F("\t'pid Ti <value>' - Sets Ti to the <value>\r\n");
	s += F("\t'pid Td <value>' - Sets Td to the <value>\r\n");
	s += F("\t'pid init' - Initializes the PID by clearing all algorithm variables\r\n");
	s += F("\t'pid limits <min> <max>' - Sets the limits of the output between <min> and <max>\r\n");
	s += F("\t'pid DeadTime <seconds>' - Sets the systems expected DeadTime.\tUsed for estimating when to transition from PreHeat to Soak and from Soak to Reflow - Default 10s\r\n");
	s += F("\t'pid PWMTime <ms>' - Sets the SSR PWM Period Time - Default 5000\r\n");
	s += F("\t'pid Direction' - Switch PID direction\r\n");
	
  s += F("\r\n");
  s += F("\r\n");
  s += Line();//====================================================================
}//===============================================================================================
String WelcomeMessage(void) {
  //Printout a welcome message
  String s; s.reserve(300);
  s += Line(); //====================================================================
  s += Config.ModelNumber;  s += (F(" v")); s += Config.SoftwareFirmwareVersion;  s += F("\r\n");
  s += F("Send '?' for help (all commands must be followed by at least one of the follow: LF, CR, or null character)\r\n");
  s += Line(); //====================================================================
  s += F("\r\n>");
  return s;
}//===============================================================================================
void DashboardMenu(String& s) {
  s += F("\r\n");
  s += Line();//====================================================================
  s += F("HELP MENU>Dashboard\r\n");
  s += F("\r\n");
  s += F("COMMANDS:\r\n");
  s += F("\t'Dashboard' - toggle dashboard on/offn\r");
  s += F("\t'Dashboard pid' - toggles on/off extra information for pid tuning\r\n");
  s += F("\t'Dashboard <value>' - Update Dashboard refresh rate in ms\r\n");
  s += F("\r\n");
  s += Line();//====================================================================
}//===============================================================================================
void ProfileMenu(String& s) {
s += F("\r\n");
s += Line();//====================================================================
s += F("HELP MENU>Profiles\r\n");
s += F("\r\n");
s += F("\tThere are 8 total profiles saved to memory: The current active profile, 5 saved profiles, and two default profiles: Leaded and Lead-Free.\r\n");
s += F("\tThe Active Profile is the only profile that is editable.  If you want to edit profile 2, you must load profile 2, edit it, then copy back to profile 2\r\n");
s += F("\r\n");
s += F("COMMANDS:\r\n");
s += F("\t'profile' - display active profile\r\n");
s += F("\t'profile display <1-"); s += NUMPROFILES - 1; s += F("> - display saved profile.  e.g. 'profile display 3'\r\n");
s += F("\t'profile copy <1-"); s += NUMPROFILES - 1; s += F("> <1-"); s += NUMPROFILES - 1; s += F("> - copy profiles.  e.g. 'profile copy 1 2', copies profile 1 into profile 2\r\n");
s += F("\t'profile load <1-"); s += NUMPROFILES - 1; s += F("> - Loads Selected profile into active profile e.g. 'profile load 3'\r\n");
s += F("\t'profile load <leaded/leadfree> - Loads a Default Leaded or Lead Free profile into the active profile. e.g. 'profile load leaded'\r\n");
s += F("\r\n");
s += F("EDIT ACTIVE PROFILE:\r\n");
s += F("\t'MaxStartTemp <C>' - e.g. 'maxstarttemp 50'\r\n");
s += F("\t'PreHeatRate <C/s>' - e.g. 'PreHeatRate 1.5'\r\n");
s += F("\t'PreHeatTemp <C>' - e.g. 'PreHeatTemp 125'\r\n");
s += F("\t'SoakRate <C/s>' - e.g. 'SoakRate 0.5'\r\n");
s += F("\t'SoakTemp <C>' - e.g. 'SoakTemp 183'\r\n");
s += F("\t'ReflowRate <C/s>' - e.g. 'ReflowRate 1.5'\r\n");
s += F("\t'ReflowPeakTemp <C>' - e.g. 'ReflowPeakTemp 235'\r\n");
s += F("\t'ReflowPeakTempPreact <C>' - e.g. 'ReflowPeakTempPreact 7'\r\n");
s += F("\t'ReflowPeakTime <s>' - e.g. 'maxstarttemp 20'\r\n");
s += F("\t'ReflowPeakTimePreact <s>' - e.g. 'maxstarttemp 5'\r\n");
s += F("\t'CoolDownRate <C/s>' - e.g. 'maxstarttemp -5'\r\n");
s += F("\r\n");
s += Line();//====================================================================
}//===============================================================================================

String ProcessTextCommand(String &s) { //
  //Process a text based command.  Returns some text to display
  //Pass in a CSV command from the HelpMenu()
  String sReturn; sReturn.reserve(2500);//Text to return
  String a[8]; a[0].reserve(40); a[1].reserve(40); a[2].reserve(40); a[3].reserve(40); a[4].reserve(40); a[5].reserve(40); a[6].reserve(40); a[7].reserve(40); //Create a small array to store the parsed strings 0-7


		//--Split the incoming serial data to an array of strings, where the [0]th element is the number of CSVs, and elements [1]-[X] is each CSV
			//If no Commas are detected the string will still placed into the [1]st array
  StringSplit(s, &a[0], " ");
  a[1].toLowerCase();

  /*
  Commands can be any text that you can trigger off of.

  For Example have the user type in '9 255' for a command to manually set pin 9 PWM to 255.
  */
  //Process single string serial commands without a CSV
  //Do something with the values by adding custom if..then statements
  //---------------------------------------------------------------------------
  //Main Menu Commands
  if (a[1] == F("?") || a[1] == F("help")) {
    Dashboard = 0;
    HelpMenu(sReturn); goto Bottom;
  }
 


  
  if (a[1] == F("lpf")) {
		TempActualLPF.RC = a[2].toDouble();
  }
  if (a[1] == F("temp8")) {
	  MAX31856.REG.CR1.TCTYPE = 12;
	  MAX31856WriteRegisters();
	  MAX31856ReadRegisters();
  }



  if (a[1] == F("general")) HelpMenuGeneral(sReturn);

  if (a[1] == F("save")) { 
    sReturn = F("\tSave Request Recieved");
    SaveConfigCMD = true; 
  }
  if (a[1].startsWith(F("output"))) {
    if (pid.Mode  == MANUAL) {
      Reflow.CV = a[2].toFloat();
      Output.PeriodTimer = millis();
      sReturn = F(" Output.DutyCycle set"); 
      goto Bottom;
    }
    else {
      sReturn = F(" PID loop is in AUTO mode, is a reflow running?"); 
      goto Bottom;
    }
  }

  if (a[1].startsWith(F("config"))) { //hidden from menu system
	if (a[2].startsWith(F("serial"))) {
		if (a[0].toInt() == 2) {
			sReturn = F("	Serial Number: "); sReturn += Config.SerialNumber;
		} else if (a[0].toInt() == 3 && a[3].toInt() > 0 && a[3].toInt() <= 65535) {
			Config.SerialNumber = a[3].toInt();
			SaveConfigCMD = true;
			sReturn = F("	Serial Number Saved: "); sReturn += Config.SerialNumber; 
			goto Bottom;
		}  
	}
	if (a[2].startsWith(F("calib"))) {
		if (a[0].toInt() == 2) {
			sReturn = F(" Calibration Date: "); sReturn += Config.CalibrationDate;
			goto Bottom;
		}
		else if (a[0].toInt() == 3 ) {
			if (a[3].length() > 5 && a[3].length() <= 16) {
				a[3].toCharArray(Config.CalibrationDate, 16);
				SaveConfigCMD = true;
				sReturn = F(" Calibration Date Saved: "); sReturn += Config.CalibrationDate;
				goto Bottom;
			}
		}

	}
	
	if (a[2].startsWith(F("model"))) {
		if (a[0].toInt() == 2) {
			sReturn = F(" Model Number: "); sReturn += Config.ModelNumber;
			goto Bottom;
		}
		else if (a[0].toInt() == 3) {
			if (a[3].length() > 5 && a[3].length() <= sizeof(Config.ModelNumber)) {
				a[3].toCharArray(Config.ModelNumber, sizeof(Config.ModelNumber));
				SaveConfigCMD = true;
				sReturn = F(" Model Number Set: "); sReturn += Config.ModelNumber;
				goto Bottom;
			}
		}
	}
	if (a[2].startsWith(F("firm"))) {
		if (a[0].toInt() == 2) {
			sReturn = F(" Software Firmware Version: "); sReturn += Config.SoftwareFirmwareVersion;
			goto Bottom;
		}
		else if (a[0].toInt() == 3) {
			if (a[3].length() >= 3 && a[3].length() <= sizeof(Config.SoftwareFirmwareVersion)) {
				a[3].toCharArray(Config.SoftwareFirmwareVersion, sizeof(Config.SoftwareFirmwareVersion));
				SaveConfigCMD = true;
				sReturn = F(" Software Firmware Version Saved: "); sReturn += Config.SoftwareFirmwareVersion;
				goto Bottom;
			}
		}
	}
	
  }

  if (a[1].startsWith(F("pro"))) {//profile

    if (a[2] == F("?")) { //profile ?
      ProfileMenu(sReturn);
      goto Bottom;
    }
    if (a[0].toInt() == 1) { //default to profile 0
      sReturn += F("Active Profile:\r\n");
      GetProfileString(sReturn); 
      goto Bottom;
    }

    if (a[2].startsWith(F("dis")) && a[0].toInt() == 3) {//profile display X
      if (a[3].toInt() < NUMPROFILES) {//i.e. "profile 3", list this profile
        uint8_t x = a[2].toInt();
        sReturn += F("Profile "); sReturn += x;  sReturn += F(":\r\n");
        GetProfileString(sReturn, x);
      } else{
        sReturn = F(" ERROR: Only profiles 1 - "); sReturn += NUMPROFILES - 1; sReturn += F(".  Profile 0 is active profile\r\n");
      }
      goto Bottom;
    }

    if (a[2] == F("copy") && a[0].toInt() == 4 && a[3].toInt() < NUMPROFILES && a[4].toInt() < NUMPROFILES) {     
      Config.Profile[a[4].toInt()] = Config.Profile[a[3].toInt()];
      sReturn = F("\tCopied Profile "); sReturn += a[3].toInt(); sReturn += F(" to "); sReturn += a[4].toInt(); sReturn += F("\r\n");
      uint8_t x = a[4].toInt();
      sReturn += F("Profile "); sReturn += x;  sReturn += F(":\r\n");
      GetProfileString(sReturn, x);
      SaveConfigCMD = true;
      goto Bottom;
    }

    if (a[2] == F("load") && a[0].toInt() == 3) {
	if (a[3] == F("leaded")) {
        Config.Profile[0] = Config.DefaultProfile[0];
        sReturn += F("\tLeaded Profile Loaded\r\n");
        SaveConfigCMD = true;
      }
      else if (a[3] == F("leadfree")) {
        Config.Profile[0] = Config.DefaultProfile[1];
        sReturn += F("\Lead-Free Profile Loaded\r\n");
        SaveConfigCMD = true;
      }
	 else if (a[3].toInt() >= 1 && a[3].toInt() <= 5) {
		 Config.Profile[0] = Config.Profile[a[3].toInt()];
		 sReturn += F("\tProfile Loaded\r\n");
		 SaveConfigCMD = true;
	 }
      else {
        sReturn = F("Syntax Error");
      }
      goto Bottom;
    }
  }

  if ((a[1] == F("maxstarttemp") || a[1] == F("maxstartemp")) && a[0].toInt() == 2) {
    double temp = a[2].toDouble();
    if (temp >= 0 && temp <= 400) {
      Config.Profile[0].MaxStartTemp = a[2].toDouble();
      sReturn += F("\tMaxStartTemp updated in active profile:\r\r\n\n");
      GetProfileString(sReturn);
    }
    else {
      sReturn = F(" ERROR: Temperature must be between 0-400C");
    }
    goto Bottom;
  }

  if (a[1] == F("preheatrate") && a[0].toInt() == 2) {
    double temp = a[2].toDouble();
    if (temp > 0 && temp <= 10) {
      Config.Profile[0].PreHeatRate = a[2].toDouble();
      sReturn += F("\tPreHeatRate updated in active profile:\r\r\n\n");
      GetProfileString(sReturn);
    }
    else {
      sReturn = F(" ERROR: Rate must be between 0-10C/s");
    }
    goto Bottom;
  }


  if ((a[1] == F("preheattemp") || a[1] == F("preheatemp")) && a[0].toInt() == 2) {
    double temp = a[2].toDouble();
    if (temp >= 0 && temp <= 400) {
      Config.Profile[0].PreHeatTemp = a[2].toDouble();
      sReturn = F("\tPreHeatTemp updated in active profile:\r\r\n\n");
      GetProfileString(sReturn);
    }
    else {
      sReturn = F(" ERROR: Temperature must be between 0-400C");
    }
    goto Bottom;
  }


  if (a[1] == F("soakrate") && a[0].toInt() == 2) {
    double temp = a[2].toDouble();
    if (temp > 0 && temp <= 10) {
      Config.Profile[0].SoakRate = a[2].toDouble();
      sReturn += F("\tSoakRate updated in active profile:\r\r\n\n");
      GetProfileString(sReturn);
    }
    else {
      sReturn = F(" ERROR: Rate must be between 0-10C/s");
    }
    goto Bottom;
  }


  if (a[1] == F("soaktemp") && a[0].toInt() == 2) {
    double temp = a[2].toDouble();
    if (temp >= 0 && temp <= 400) {
      Config.Profile[0].SoakTemp = a[2].toDouble();
      sReturn += F("\tSoakTemp updated in active profile:\r\r\n\n");
      GetProfileString(sReturn);
    }
    else {
      sReturn = F(" ERROR: Temperature must be between 0-400C");
    }
    goto Bottom;
  }


  if (a[1] == F("reflowrate") && a[0].toInt() == 2) {
    double temp = a[2].toDouble();
    if (temp > 0 && temp <= 10) {
      Config.Profile[0].ReflowRate = a[2].toDouble();
      sReturn += F("\tReflowRate updated in active profile:\r\r\n\n");
      GetProfileString(sReturn);
    }
    else {
      sReturn = F(" ERROR: Rate must be between 0-10C/s");
    }
    goto Bottom;
  }


  if (a[1] == F("reflowpeaktemp") && a[0].toInt() == 2) {
    double temp = a[2].toDouble();
    if (temp >= 0 && temp <= 400) {
      Config.Profile[0].ReflowPeakTemp = a[2].toDouble();
      sReturn += F("\tReflowPeakTemp updated in active profile:\r\r\n\n");
      GetProfileString(sReturn);
      goto Bottom;
    }
    else {
      sReturn = F(" ERROR: Temperature must be between 0-400C");
    }
    goto Bottom;
  }


  if ((a[1] == F("reflowpeaktemppreact") || a[1] == F("reflowpeaktempreact")) && a[0].toInt() == 2) {
    double temp = a[2].toDouble();
    if (temp >= 0 && temp <= 400) {
      Config.Profile[0].ReflowPeakTempPreact = a[2].toDouble();
      sReturn += F("\tReflowPeakTempPreact updated in active profile:\r\r\n\n");
      GetProfileString(sReturn);
      goto Bottom;
    }
    else {
      sReturn = F(" ERROR: Temperature must be between 0-400C");
    }
    goto Bottom;
  }


  if (a[1] == F("reflowpeaktime") && a[0].toInt() == 2) {
    double temp = a[2].toDouble();
    if (temp >= 0 && temp <= 400) {
      Config.Profile[0].ReflowPeakTime = a[2].toDouble();
      sReturn += F("\tReflowPeakTime updated in active profile:\r\r\n\n");
      GetProfileString(sReturn);
      goto Bottom;
    }
    else {
      sReturn = F(" ERROR: Time must be between 0-400s");
    }
    goto Bottom;
  }


  if (a[1] == F("reflowpeaktimepreact") && a[0].toInt() == 2) {
    double temp = a[2].toDouble();
    if (temp >= 0 && temp <= 400) {
      Config.Profile[0].ReflowPeakTimePreact = a[2].toDouble();
      sReturn += F("\tReflowPeakTimePreact updated in active profile:\r\r\n\n");
      GetProfileString(sReturn);
      goto Bottom;
    }
    else {
      sReturn = F(" ERROR: Time must be between 0-400s");
    }
    goto Bottom;
  }


  if (a[1] == F("cooldownrate") && a[0].toInt() == 2) {
    double temp = a[2].toDouble();
    if (temp < 0 && temp >= -10) {
      Config.Profile[0].CoolDownRate = a[2].toDouble();
      sReturn += F("\tCoolDownRate updated in active profile:\r\r\n\n");
      GetProfileString(sReturn);
    }
    else {
      sReturn = F(" ERROR: Rate must be between -10-0C/s (negative nuymber)");
    }
    goto Bottom;
  }

  if (a[1] == F("heatup") && a[0].toInt() == 2) {
    HeatingElementsTempSpeedIncrease = a[2].toDouble();
    sReturn += F("\tHeatingElementsTempSpeedIncrease updated\r\n");
    goto Bottom;
  }

  if (a[1] == F("heatdown") && a[0].toInt() == 2) {
    HeatingElementsTempSpeedDecrease = a[2].toDouble();
    sReturn += F("\tHeatingElementsTempSpeedDecreases updated\r\n");
    goto Bottom;
  }


  if (a[1].startsWith(F("dash"))) {

    if (a[2] == F("?")) {
      DashboardMenu(sReturn);
      goto Bottom;
    }
    //If the last parameter entered is greater than 0, set the Dashboard time to it
    if (a[a[0].toInt()].toInt() > 0) {
      DashboardTime = a[a[0].toInt()].toInt();
      sReturn = F(" Dashboard Update Time Set to "); sReturn += DashboardTime;  sReturn += F("ms");
      goto Bottom;
    }

    if (a[0].toInt() == 1) { //If the dashboard is on, and there is only one parameter, they just entered "dash" turn it off
      Dashboard ^= 1 << 0; //toggle bit 0
      if (Dashboard & 1 == 0) {//dashboard was just turned off
        SerialClearScreen();
        Serial.print(WelcomeMessage());
        sReturn = F("Dashboard Stopped\r\r\n\n>");
        goto Bottom;
      }
      else {
        sReturn = F(" Dashboard Started");
        goto Bottom;
      }
    }

    if (a[2] == "pid") {
      Dashboard ^= 1 << 1; //toggle bit 1
      sReturn = F(" Dashboard pid parameters toggled");
      goto Bottom;
    }
  }

  if (a[1].startsWith(F("start"))) {
    if (Reflow.Step.Step == IDLE) {
      Reflow.Status = true;
      sReturn = F(" STATE Engine Started");
      goto Bottom;
    } else if (Reflow.Step.Step == PAUSED) {
      Reflow.Step.Step = Reflow.Step.StepREM; //go back to the step you were in when you paused
      pid.Mode = AUTO;
      sReturn = F(" STATE Engine Restart");
      goto Bottom;
    }
    else {
      sReturn = F(" STATE Engine already running");
      goto Bottom;
    }
  }
  if (a[1].startsWith(F("stop")) || a[1].startsWith(F("abort"))) {
    if (Reflow.Step.Step != IDLE) {
      Reflow.Status = false;
      sReturn = F(" STATE Engine Stopped");
      goto Bottom;
    }   
    else {
      sReturn = F(" STATE Engine not running");
      goto Bottom;
    }
  }

  if (a[1].startsWith(F("pause")) || a[1].startsWith(F("hold"))) {
    if (Reflow.Step.Step == IDLE) {
      sReturn = F(" STATE Engine is IDLE");
      goto Bottom;
    }
    else if (Reflow.Step.Step == PAUSED) {
      sReturn = F(" STATE Engine is already PAUSED");
      goto Bottom;
    } else  { //equivilant: (Reflow.Step.Step != IDLE && Reflow.Step.Step != PAUSED)
      Reflow.Step.StepREM = Reflow.Step.Step; //rmemeber what step you came from
      Reflow.Step.Step = PAUSED;
      pid.Mode = MANUAL;
      sReturn = F(" STATE Engine paused");
      goto Bottom;
    }
  }

  if (a[1].startsWith(F("pid"))) {
    if (a[2].startsWith(F("?"))) {
      PIDMenu(sReturn);
      goto Bottom;
    }

    if (a[2].startsWith(F("sp"))) {
      Reflow.SP = a[2].toDouble();
      sReturn = F(" sp set");
      goto Bottom;
    }

    if (a[2].startsWith(F("pwm"))) {
      if (a[3].toInt() > 100) {
        Config.PWMPeriodTime = a[3].toInt();
        sReturn = F("\tPWM Period Time Set");
        goto Bottom;
      }
      else {
        sReturn = F("\tPWM Time too small");
        goto Bottom;
      }
    }

    if (a[2].startsWith(F("init"))) {
      pid.Initialize(a[2].toDouble());
      sReturn = F(" PID Initialized");
      goto Bottom;
    }
    if (a[2].startsWith(F("kc")) || a[2].startsWith(F("kp"))) {
      Config.pid.Kc = a[3].toFloat();
      pid.Kc = Config.pid.Kc;
      SaveConfigCMD = true;
      sReturn = F(" Kc set");
      goto Bottom;
    }
    if (a[2].startsWith(F("ti")) || a[2].startsWith(F("ki"))) {
      Config.pid.Ti = a[3].toFloat();
      pid.Ti = Config.pid.Ti;
      SaveConfigCMD = true;
      sReturn = F(" Ti set");
      goto Bottom;
    }
    if (a[2].startsWith(F("td")) || a[2].startsWith(F("kd"))) {
      Config.pid.Td = a[3].toFloat();
      pid.Td = Config.pid.Td;
      SaveConfigCMD = true;
      sReturn = F(" Td set");
      goto Bottom;
    }

    if (a[2].startsWith(F("dir"))) {
      Config.pid.Direction = !Config.pid.Direction;
      pid.Direction = Config.pid.Direction;
      SaveConfigCMD = true;
      sReturn = F(" pid direction Set");
      goto Bottom;
    }

    if (a[2].startsWith(F("man"))) {
      pid.Mode = MANUAL;
      sReturn = F(" pid set to MANUAL mode");
      goto Bottom;
    }
    if (a[2].startsWith(F("auto"))) {
      pid.Mode = AUTO;
      sReturn = F(" pid set to AUTOMATIC mode");
      goto Bottom;
    }

    if (a[2].startsWith(F("limits"))) {
      if (a[3].toFloat() < a[4].toFloat()) {
        Config.pid.Min = a[3].toFloat();
        Config.pid.Max = a[4].toFloat();
        pid.SetOutputLimits(a[3].toFloat(), a[4].toFloat());
        SaveConfigCMD = true;
        sReturn = F(" pid min/max set");
        goto Bottom;
      }
      else {
        sReturn = F(" Invalid Limits");
        goto Bottom;
      }
    }

    if (sReturn == "") {
      PIDMenu(sReturn);
      goto Bottom;
    }
  }
  

  if (a[1].startsWith(F("temp"))) {
		if (a[2].toInt() != 0.0) { //temperature simulation requested
			if (Temp.Simulate) {
			Temp.Actual = a[2].toInt();
			sReturn = F(" Temperature Forced");
			goto Bottom;
			}
			else {
			sReturn = F(" Temperature Simulation must be turned on to force the temperature");
			goto Bottom;
			}
		}
		if (a[2] == F("?")) {
			TemperatureMenu(sReturn);
			goto Bottom;
		}
		if (a[2] == F("status")) {
			GetConfigString(sReturn);
			goto Bottom;
		}
		if (a[2] == F("sim") && a[0].toInt() == 2) {
			Temp.Simulate = !Temp.Simulate;
			(Temp.Simulate) ? sReturn = F(" Temperature Simulation ON") : sReturn = F(" Temperature Simulation OFF");
			goto Bottom;
		}    
		if (a[2] == F("avg")) { //temp avg XX
			uint8_t avgsel = a[3].toInt();
			if (avgsel != 0 && avgsel != 1 && avgsel != 2 && avgsel != 4 && avgsel != 8 && avgsel != 16) {
				sReturn = F("Invalid Sample Averaging Selection.  The options are 1, 2, 4, 8, or 16");
				goto Bottom;
			}
			else {
				//First turn off continuous conversions because the datasheet says to not change the averaging in the middle of a conversion
				MAX31856ReadRegisters(); //refresh CR0 and CR1
				MAX31856.REG.CR0.CMODE = 0;
				MAX31856WriteRegisters(); //Write the new config down to stop conversions
				delay(500); //Wait for any conversiosn to finish.yeah I know this is bad and I should yield...but its ok...

				uint8_t bitmask=0;
				if (avgsel == 2)  bitmask = 1;
				if (avgsel == 4)  bitmask = 2;
				if (avgsel == 8)  bitmask = 3;
				if (avgsel == 16) bitmask = 4;
				MAX31856.REG.CR1.AVGSEL = bitmask; //add the new configuration
				MAX31856WriteRegisters();
				MAX31856.REG.CR0.CMODE = 1; //Set it back to Auto Conversion
				MAX31856WriteRegisters();

				sReturn = F(" Temperature averaging updated");
				goto Bottom;
			}
		}
		if (a[2] == F("factorydefault")) {     //temp factorydefault
			MAX31856SetFactoryDefault();
			Config.MAX31856 = MAX31856;
			sReturn = F(" TC registers set to Factory Default");   
			SaveConfigCMD = true;
			goto Bottom;
		}
		if (a[2] == F("default")) {		//temp default
			MAX31856SetDefaultConfig();
			sReturn = F(" TC registers set to Default");     
			goto Bottom;
		}
		if (sReturn == "") {
				TemperatureMenu(sReturn);
				goto Bottom;
		}
  }



  //---------------------------------------------------------------------------
  //General Menu
  if (a[1] == F("system")) {
    sReturn += Line();//====================================================================
    sReturn += F("SYSTEM INFORMATION\r\n");
    sReturn += Config.ModelNumber;  sReturn += F("\r\n");
		sReturn += F("\tSoftware Firmware Version: "); sReturn += Config.SoftwareFirmwareVersion; sReturn += F("\r\n");
		sReturn += F("\t");  sReturn += StringIndent(F("Model Number:"),3);		sReturn += Config.ModelNumber; sReturn += F("\r\n");
		sReturn += F("\t"); sReturn += StringIndent(F("Serial Number:"), 3);		sReturn += Config.SerialNumber; sReturn += F("\r\n");
		sReturn += F("\t"); sReturn += StringIndent(F("Calibration Date:"),3); sReturn += Config.CalibrationDate; sReturn += F("\r\n");
    sReturn += F("\r\n");
    sReturn += F("Microcontroller statistics:\r\n");
		sReturn += F("\t");	sReturn += StringIndent(F("Up Time:"),3); sReturn += getUpTimeString(__UPTIME); sReturn += F("\r\n");
		sReturn += F("\t");	sReturn += StringIndent(F("FreeHeap:"), 3); sReturn += ESP.getFreeHeap(); sReturn += F(" B\r\n");
		sReturn += F("\t");  sReturn += StringIndent(F("FlashChipSize:"), 3); sReturn += ESP.getFlashChipSize(); sReturn += F(" B\r\n");
		sReturn += F("\t");	sReturn += StringIndent(F("SdkVersion:"), 3); sReturn += ESP.getSdkVersion(); sReturn += F("\r\n");
		sReturn += F("\t");  sReturn += StringIndent(F("CpuFreqMHz:"), 3); sReturn += ESP.getCpuFreqMHz(); sReturn += F(" Mhz\r\n");
		sReturn += F("\t");  sReturn += StringIndent(F("FlashChipSpeed:"), 3); sReturn += ESP.getFlashChipSpeed() / 1000000; sReturn += F(" Mhz\r\n");
		sReturn += F("\t");  sReturn += StringIndent(F("FlashChipMode:"), 3); sReturn += ESP.getFlashChipMode(); sReturn += F("\r\n");
		sReturn += F("\t");  sReturn += StringIndent(F("CycleCount:"), 3); sReturn += ESP.getCycleCount(); sReturn += F("\r\n");
    sReturn += F("\r\n");
    sReturn += Line();//====================================================================
    goto Bottom;
  }

  if (a[1] == F("restart")) { 
    RestartCMD = true; 
    sReturn = F("\r\n");
    goto Bottom;
  }

	if (a[0].toInt() == 2 && a[1] == F("tablength") && a[2].toInt() >0 && a[2].toInt() <= 16) {
		Config.TabLength = a[2].toInt();
		SaveConfigCMD = true;
		sReturn = F("\tTab Length Set to \r\n"); 	sReturn += Config.TabLength;	sReturn = F("\r\n");
		goto Bottom;
	}


  //debug level
    if (a[1] == F("debug")) {
      DebugLevel = a[2].toInt();
      if (a[2] == "") {//toggle it
        if (DebugLevel > 0) {
          DebugLevel = 0;
          SaveConfigCMD = true;
          sReturn = F(" debug off");
          goto Bottom;
        }
        else if (DebugLevel == 0) {
          DebugLevel = 1;
          SaveConfigCMD = true;
          sReturn = F(" debug on");
          goto Bottom;
        }
        SaveConfigCMD = true;
      }
      else {
        sReturn = F(" Debug Level set to ");
        sReturn += DebugLevel;
        goto Bottom;
      }
    }
  if (a[1] == F("factorydefault")) {
	  sReturn = F("OK");
    FactoryDefault();
    goto Bottom;
  }

  if (a[1] == F("serial")) {
    uint32_t SerialSpeed = a[2].toInt();
    Serial.print(F("Setting Serial BAUD rate to ")); Serial.print(SerialSpeed); Serial.println(F(".  Please reconnect at this speed"));
    Serial.flush();   // wait for send buffer to empty
    Serial.end();
    delay(1);
    Serial.begin(SerialSpeed);
    Serial.println(F("\r\r\n\n"));
    sReturn = F(" Serial Settings Updated");
    goto Bottom;
  }
  /*
  //Buzzer Testing
  if (a[1] == F("buz")) {
    if (a[2] == F("pow")) {
      BuzzerPowerOn.Trigger = true;
      Serial.println(F("Power Buzzer Triggered"));
    }
    if (a[2] == F("but")) {
      BuzzerButtonFeedback.Trigger = true;
      sReturn = F("Button Buzzer Triggered");
    }
    if (a[2] == F("pre")) {
      BuzzerPowerOn.Trigger = true;
      sReturn = F("PreHeat Buzzer Triggered");
    }
    if (a[2] == F("soak")) {
      BuzzerSoakStart.Trigger = true;
      sReturn = F("Reflow Buzzer Triggered");
    }
    if (a[2] == F("reflowstart")) {
      BuzzerReflowStart.Trigger = true;
      sReturn = F("ReflowStart Buzzer Triggered");
    }
    if (a[2] == F("cool")) {
      BuzzerCooldownStart.Trigger = true;
      sReturn = F("Cooldown Buzzer Triggered");
    }
    if (a[2] == F("reflowcom")) {
      BuzzerReflowComplete.Trigger = true;
      sReturn = F("ReflowComplete Buzzer Triggered");
    }
  }

  */

Bottom:
  if (sReturn == "") { 
    sReturn = F(" ERROR: Invalid Command");
    BuzzerFeedbackNegative.Trigger = true;
  } //if it made it through all of the above and sReturn is still blank return that its an invalid command
  else {
    BuzzerFeedbackPositive.Trigger = true;
  }
  return sReturn;
}//===============================================================================================
void ProcessSerialCommands(void) {
  if (UARTRead() == false) return ; //Read Data in from the Serial buffer, Immediately return if there is no new data

                     //Process the last serial line and print it
  Serial.print(F("\r\n>"));  Serial.println(sLastUARTLine);
  Serial.println(ProcessTextCommand(sLastUARTLine));
  sLastUARTLine = ""; //Clear out buffer, This should ALWAYS be the last line in this if..then
}//===============================================================================================
void EndProgram(String ErrorMessage) {
  //An unrecoverable error occured
  //Loop forever and display the error message
  String s;
  s += F("Major Error: ");
  s += ErrorMessage;
  s += F(".  Cycle power to restart (probably won't help)");
  GlobalErrorMessage = s;
  uint32_t timer;
  while (1) {
    if ((millis() - timer) > 5000) {
      timer = millis();
      Serial.println(s);
    }
    yield();
  }
}//===============================================================================================
