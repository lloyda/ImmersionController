/*
EmonTx CT123 example

An example sketch for the emontx module for
CT only electricity monitoring.

Part of the openenergymonitor.org project
Licence: GNU GPL V3

Author: Lloyd Adams
Based on code by Glyn Hudson, Trystan Lea
Builds upon JeeLabs RF12 library and Arduino

*/
//#define DEBUG

const int CT1 = 1; 
const int CT2 = 0;                                                      // Set to 0 to disable CT channel 2
const int CT3 = 0;                                                      // Set to 0 to disable CT channel 3

#define freq RF12_868MHZ   // Frequency of RF12B module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. You should use the one matching the module you have.
const int emonTx_nodeID = 10;
const int emonBase_nodeID = 15; 
const int emonGLCD_nodeID = 20;

const int nodeID = 5;                                                  // This modules RFM12B node ID
const int networkGroup = 210;                                           // Our wireless network group - needs to be same as emonBase and emonGLCD

const int UNO = 1;                                                      // Set to 0 if your not using the UNO bootloader (i.e using Duemilanove) - All Atmega's shipped from OpenEnergyMonitor come with Arduino Uno bootloader
#include <avr/wdt.h>                                                     

#include <JeeLib.h>                                                     // Download JeeLib: http://github.com/jcw/jeelib
ISR(WDT_vect) { Sleepy::watchdogEvent(); }                              // Attached JeeLib sleep function to Atmega328 watchdog -enables MCU to be put into sleep mode inbetween readings to reduce power consumption 

#include "EmonLib.h"
EnergyMonitor ct1,ct2,ct3;                                              // Create  instances for each CT channel

typedef struct { int power1, power2, power3, battery, Vrms; } PayloadTX;      // create structure - a neat way of packaging data for RF comms
PayloadTX emontx;

typedef struct { int power1, temperature, Vrms, power2; } PayloadImm;      // create structure - a neat way of packaging data for RF comms
PayloadImm ImmerCtl; 

const int LEDpin = 9;   // On-board emonTx LED 
const int SSRpin = 6;
int flipflop = 0;

#include <PID_v1.h>

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,2,5,1, DIRECT);

void setup() 
{
	Serial.begin(9600);
	Serial.println("Immersion Controller"); 
	Serial.println("OpenEnergyMonitor.org");
	
	ct1.voltageTX(230, 1.7);    // US Robotics=235, BT=224
	ct1.currentTX(1, 111.1);                                            // Setup emonTX CT channel (channel, calibration)

	rf12_initialize(nodeID, freq, networkGroup);                          // initialize RFM12B
	rf12_sleep(RF12_SLEEP);

	pinMode(LEDpin, OUTPUT);    // Setup indicator LED
	pinMode(SSRpin, OUTPUT);    // Setup SSR control
	digitalWrite(LEDpin, HIGH);

	//initialize the PID
	Input = 255;
	Setpoint = 255;   // 255 - off)

	//turn the PID on
	myPID.SetMode(AUTOMATIC);

	if (UNO) wdt_enable(WDTO_8S);    // Enable anti crash (restart) watchdog if UNO bootloader is selected. Watchdog does not work with duemilanove bootloader                                                             // Restarts emonTx if sketch hangs for more than 8s

}

void loop() 
{ 



	if (rf12_recvDone()){
		if (rf12_crc == 0 && (rf12_hdr & RF12_HDR_CTL) == 0)  // and no rf errors
		{
			int node_id = (rf12_hdr & 0x1F);	

			if (node_id == emonTx_nodeID)// === EMONTX node ID ====
			{

				//                                ImmerCtl.power1=ct1.calcIrms(1480) * 230.0;				
				ct1.calcVI(20,2000);                                                  // Calculate all. No.of wavelengths, time-out 
				ImmerCtl.power1 = ct1.realPower;        // Calculate CT 1 power
				ImmerCtl.Vrms = ct1.Vrms*100;	
#ifdef DEBUG
				Serial.print("Power1:");
				Serial.print(ImmerCtl.power1);          // Output to serial  
				Serial.print ("Vrms:");
				Serial.print(ImmerCtl.Vrms);Serial.print(' ');
				Serial.println (emontx.Vrms);
#endif
				
				
				// last_emontx = millis();                 // set time of last update to now
				emontx = *(PayloadTX*) rf12_data;       // get emontx payload data

				if (emontx.power2 < 50) output_control (0);    // Kill the immersion
#ifdef DEBUG 
				Serial.println();
				Serial.print("1 emontx: ");
				Serial.print(emontx.power1);
				Serial.print(' ');
				Serial.print(emontx.power2);
				Serial.print(' ');
				Serial.println(emontx.Vrms);              // print data to serial
				Serial.println(emontx.Vrms - ImmerCtl.Vrms);
				delay(100);
#endif  
				
				// delay to make sure printing finished
				ImmerCtl.power2 = emontx.power2;
				ImmerCtl.temperature = ImmerCtl.power1 - emontx.power2;
				// set emonglcd payload
				int i = 0; while (!rf12_canSend() && i<10) {rf12_recvDone(); i++;}  // if ready to send + exit loop if it gets stuck as it seems too
				rf12_sendStart(0, &ImmerCtl, sizeof ImmerCtl);                      // send emonglcd data
				rf12_sendWait(0);
#ifdef DEBUG 
				Serial.println("Immersion Data sent");                                // print status
#endif                               
				digitalWrite(LEDpin, HIGH); delay(2); digitalWrite(LEDpin, LOW);      // flash LED
				Setpoint = (emontx.power1 - emontx.power2-100 - ImmerCtl.power1);
//				Setpoint = (3600 - emontx.power2-100 - ImmerCtl.power1);

				if (Setpoint < 0) Setpoint =0;
				Input = ImmerCtl.power1;
				myPID.Compute();
				Serial.print ("Setpoint:");Serial.print (Setpoint);Serial.print (" Input:");Serial.print (Input);

				Serial.print (" Output:");
				Serial.println (255-Output);
				analogWrite(SSRpin,255-Output);
flipflop = flipflop + 10;
if (flipflop > 255) flipflop =0;

				//emontx_sleep(10);                                                      // sleep or delay in seconds - see emontx_lib

			}
		}
		delay(100);
		if (UNO) wdt_reset();
	}
}

