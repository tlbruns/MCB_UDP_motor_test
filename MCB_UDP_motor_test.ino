#include <i2c_t3.h>
#include <SPI.h>
#include <ArduinoSTL.h>
#include "LS7366R.h"
#include "AD5761R.h"
#include "si5351.h"
#include "MCB.h"
#include "MCBmodule.h"
#include "MCBpins.h"
#include "PID_f32.h"
#include <arm_math.h>
#include <IntervalTimer.h>
#include <Ethernet.h>
#include <EthernetUdp.h> 

// GLOBAL VARIABLES

// Motor Control Board
const int8_t MCBmodules_num = 6; // number of modules (i.e. motors) plugged into this board
MCB MotorBoard(MCBmodules_num);	// construct motor control board
int8_t currentMotorSelected = 0; // for manual control using Up/Down buttons

// toggle LEDs for timing w/oscilloscope
bool pidLedState = false;
bool udpLedState = false;
bool buttonLedState = false;

// PID timer interrupt
IntervalTimer PIDTimer;
void PIDTimerISR(void);
uint32_t timeStepPID = 1000; // [us]
float kp = 0.0004, ki = 0.000002, kd = 0.01; // work well for 1 kHz (1000 us)
//uint32_t timeStepPID = 500; // [us]
//float kp = 0.0002, ki = 0.000001, kd = 0.01; // work ok for 2 kHz (500 us)

// UDP timer interrupt
IntervalTimer UDPTimer;
void UDPTimerISR(void);
uint32_t timeStepUDP = 5000; // [us]

// Button read timer interrupt
IntervalTimer buttonTimer;
void buttonTimerISR(void);
uint32_t timeStepButton = 10000; // [us]
uint32_t buttonCountChange = 500; // [counts]

// sine wave trajectory
float phase = 0.0;
const int32_t sinAmplitude = 10000; // [counts]
const float twopi = 2 * PI;
int32_t countDesired = 0;

// Ethernet
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 177);
unsigned int localPort = 8888;
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; // buffer to hold incoming UDP packet
EthernetUDP Udp;

//----------------------------------------------------------------//
void setup()
{
	MotorBoard.waitForButtonHold(); // wait until Menu/Up/Down have been held for 2 seconds

	MotorBoard.init();
	
	for (int ii = 0; ii < MCBmodules_num; ii++)
	{
		MotorBoard.modules.at(ii).setGains(kp, ki, kd);
		MotorBoard.modules.at(ii).setCountDesired(countDesired);
	}

	MotorBoard.enableAllAmps(); // enable amp

	Ethernet.begin(mac, ip); // start Ethernet and UDP
	Udp.begin(localPort);

	PIDTimer.begin(PIDTimerISR, timeStepPID); // attach function and call every timeStepPID [us]
	UDPTimer.begin(UDPTimerISR, timeStepUDP);
	buttonTimer.begin(buttonTimerISR, timeStepButton);
}

//----------------------------------------------------------------//

void loop()
{
	// everything is interrupt driven
}

//----------------------------------------------------------------//

void PIDTimerISR(void)
{
	cli(); // disable all interrupts to ensure this process completes sequentially

	//pidLedState = !pidLedState; // blink LED for timing with oscilloscope
	//MotorBoard.setLEDG(0, pidLedState);

	MotorBoard.stepPID();
	
	//pidLedState = !pidLedState;
	//MotorBoard.setLEDG(0, pidLedState);

	sei();
}

//----------------------------------------------------------------//

void UDPTimerISR(void)
{
	cli(); // disable all interrupts to ensure this process completes sequentially

	//udpLedState = !udpLedState;
	//MotorBoard.setLEDG(1, udpLedState);

	// if there's data available, read a packet
	int packetSize = Udp.parsePacket();
	if (packetSize) {
		Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
		
		for (int ii = 0; ii < MCBmodules_num; ii++)
		{
			MotorBoard.modules.at(ii).setCountDesired(packetBuffer[ii] * 50);

		}
	}

	//udpLedState = !udpLedState;
	//MotorBoard.setLEDG(1, udpLedState);

	sei();
}

//----------------------------------------------------------------//

void buttonTimerISR(void)
{
	//buttonLedState = !buttonLedState; // blink LED for timing with oscilloscope
	//MotorBoard.setLEDG(2, buttonLedState);

	// check Local <-> Remote switch on motherboard
	if (!digitalReadFast(MotorBoard.pins.CTRL)) { // 'Remote' option
		// generate sin wave trajectory
		for (int ii = 0; ii < MCBmodules_num; ii++)
		{
			countDesired = (int32_t)(arm_sin_f32(phase + 0.25*ii) * sinAmplitude);
			MotorBoard.modules.at(ii).setCountDesired(countDesired);
		}
		
		phase = phase + 0.02;
		if (phase >= twopi) phase = 0.0;
	}
	else { // 'Local' option
		MotorBoard.readButtons();

		if (MotorBoard.isMenuPressed()) {
			MotorBoard.disableAllAmps(); // stop motors momentarily
			for (int ii = 0; ii < MCBmodules_num; ii++) {
				MotorBoard.setLEDG(ii, false);
			}
			while (MotorBoard.isMenuPressed()) {
				//MotorBoard.modules.at(currentMotorSelected).setCountDesired(countDesired);
				if (MotorBoard.isUpPressed()) {
					MotorBoard.setLEDG(currentMotorSelected, false);
					currentMotorSelected++;
					if (currentMotorSelected > (MCBmodules_num-1)) { currentMotorSelected = 0; }
					MotorBoard.setLEDG(currentMotorSelected, true);
				}
				else if (MotorBoard.isDownPressed()) {
					MotorBoard.setLEDG(currentMotorSelected, false);
					currentMotorSelected--;
					if (currentMotorSelected < 0) { currentMotorSelected = (MCBmodules_num-1); }
					MotorBoard.setLEDG(currentMotorSelected, true);
				}
				delayMicroseconds(500000); // wait for human's slow reaction time
				MotorBoard.readButtons();
			}
			MotorBoard.enableAllAmps();
		}
		else if (MotorBoard.isUpPressed()) {
			countDesired += buttonCountChange;
			MotorBoard.modules.at(currentMotorSelected).setCountDesired(countDesired);
		}
		else if (MotorBoard.isDownPressed()) {
			countDesired -= buttonCountChange;
			MotorBoard.modules.at(currentMotorSelected).setCountDesired(countDesired);
		}
		
	}

	//buttonLedState = !buttonLedState;
	//MotorBoard.setLEDG(2, buttonLedState);
}