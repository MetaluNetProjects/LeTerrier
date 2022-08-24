/*********************************************************************
 *
 *                Store on 8X2A
 *
 *********************************************************************
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Antoine Rousseau  aug 20 2022
 ********************************************************************/

/*
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
# MA  02110-1301, USA.
*/
#define BOARD 8X2A

#include <fruit.h>


#include <pid.h>
#include <ramp.h>
#include <dcmotor.h>
//#include <analog.h>
#include <switch.h>
#include <dmx_slave.h>

//-------------  Timer1 macros :  ---------------------------------------- 
//prescaler=PS fTMR1=FOSC/(4*PS) nbCycles=0xffff-TMR1init T=nbCycles/fTMR1=(0xffff-TMR1init)*4PS/FOSC
//TMR1init=0xffff-(T*FOSC/4PS) ; max=65536*4PS/FOSC : 
//ex: PS=8 : T=0.01s : TMR1init=0xffff-15000
//Maximum 1s !!
#define	TMR1init(T) (0xffff-((T*FOSC)/32000)) //ms ; maximum: 8MHz:262ms 48MHz:43ms 64MHz:32ms
#define	TMR1initUS(T) (0xffff-((T*FOSC)/32000000)) //us ; 
#define InitTimer(T) do{ TMR1H=TMR1init(T)/256 ; TMR1L=TMR1init(T)%256; PIR1bits.TMR1IF=0; }while(0)
#define InitTimerUS(T) do{ TMR1H=TMR1initUS(T)/256 ; TMR1L=TMR1initUS(T)%256; PIR1bits.TMR1IF=0; }while(0)
#define TimerOut() (PIR1bits.TMR1IF)

DCMOTOR_DECLARE(C);
DCMOTOR_DECLARE(D);

long int TestVar,TestVar2;
int dmxchan; // DMX first channel.

unsigned char PERIOD=10;
unsigned char mainCount=0;
t_delay mainDelay;

void highInterrupts()
{
	if(PIR1bits.TMR1IF) {
		DCMOTOR_CAPTURE_SERVICE(C);
		DCMOTOR_CAPTURE_SERVICE(D);
		InitTimerUS(10);
	}
}

void lowInterrupts()
{
	DMXSlaveISR();
}


// --------- Fraise Watchdog : ----------------//
unsigned int wdC = 0; //watchdog count

void wdReset(void)
{
	wdC = 0;
}

#define wdOK() (wdC < (200*2)) // 2 seconds
#define wdService() do {if(wdOK()) wdC++;} while(0)
//----------------------------------------------//

void setup(void)
{
	fruitInit();
	switchInit();
	
// ------- init PWM1 to pulse MOTC_IN1
	//PSTR1CON=0;
	//PSTR1CONbits.STR1D=1;

// ------- init PWM2 to pulse MOTD_EN

	//PSTR2CON=0;
	//PSTR2CONbits.STR2B=1;

#if 1
// ---------- capture timer : TMR1 ------------
	T1CON=0b00110011;//src=fosc/4,ps=8,16bit r/w,on.
	PIE1bits.TMR1IE=1;  //1;
	IPR1bits.TMR1IP=1;
	
	dcmotorInit(C);
	dcmotorInit(D);

	DCMOTOR(C).Setting.onlyPositive = 1;
	DCMOTOR(C).Setting.PosErrorGain = 4;
	DCMOTOR(D).Setting.onlyPositive = 1;
	DCMOTOR(D).Setting.PosErrorGain = 4;

	pinModeDigitalOut(MOTC_ENDLED);
	pinModeDigitalOut(MOTD_ENDLED);
	
	pinModeDigitalIn(DMX_ENABLE);

#if 0
	DCMOTOR(C).Setting.PosWindow = 1;
	DCMOTOR(C).Setting.PwmMin = 50;
	DCMOTOR(C).Setting.PosErrorGain = 6;
	DCMOTOR(C).Setting.onlyPositive = 0;
	
	DCMOTOR(C).PosRamp.maxSpeed = 40;
	DCMOTOR(C).PosRamp.maxAccel = 50;
	DCMOTOR(C).PosRamp.maxDecel = 50;
	rampSetPos(&DCMOTOR(C).PosRamp, 0);

	DCMOTOR(C).PosPID.GainP = 120; //90
	DCMOTOR(C).PosPID.GainI = 10;
	DCMOTOR(C).PosPID.GainD = 0;
	DCMOTOR(C).PosPID.MaxOut = 1023;

	DCMOTOR(C).VolVars.homed = 0;

	DCMOTOR(D).Setting.PosWindow = 100;
	DCMOTOR(D).Setting.PwmMin = 50;
	DCMOTOR(D).Setting.PosErrorGain = 1;
	
	DCMOTOR(D).PosRamp.maxSpeed = 2000;
	DCMOTOR(D).PosRamp.maxAccel = 5000;
	DCMOTOR(D).PosRamp.maxDecel = 5000;
	rampSetPos(&DCMOTOR(D).PosRamp, 16384);

	DCMOTOR(D).PosPID.GainP = 200; //90
	DCMOTOR(D).PosPID.GainI = 1;
	DCMOTOR(D).PosPID.GainD = 0;
	DCMOTOR(D).PosPID.MaxOut = 1023;

	DCMOTOR(D).VolVars.homed = 0;
#endif
#endif
//	analogInit();
	
	switchInit();
	//switchSelect(0,K7);

	DMXSlaveInit();					// init DMX512 receiver

	EEreadMain();
	delayStart(mainDelay, 5000); 	// init the mainDelay to 5 ms
}

// ---------- Main loop ------------

#define REWIND_SPEED -350
#define END_POS_TRANS 10900
#define MAX_SPEED_TRANS 350
#define END_POS_ROT 9977
#define MAX_SPEED_ROT 395

#define CONSIGN_POS(p, max) (((long int)p * max) / 256)
#define CONSIGN_SPEED(s, max) (((long int)s * max) / 256)
void dmxService()
{
	static int pos1, pos2;
	static byte speed1, speed2;
	static byte oldDMxEnable = 0;

	if(digitalRead(DMX_ENABLE) != oldDMxEnable) {
		oldDMxEnable = digitalRead(DMX_ENABLE);
		if(oldDMxEnable == 0) {
			DCMOTOR(C).Setting.Mode = 0; // mode PWM
			DCMOTOR(C).Vars.PWMConsign = 0;
			rampSetPos(&DCMOTOR(C).PosRamp, 0);
			DCMOTOR(D).Setting.Mode = 0; // mode PWM
			DCMOTOR(D).Vars.PWMConsign = 0;
			rampSetPos(&DCMOTOR(D).PosRamp, 0);
		}
	}
	if(oldDMxEnable == 0) return;
	//if(digitalRead(DMX_ENABLE) == 0) return;

	pos1 = DMXSlaveGet(dmxchan);
	if(pos1 == 0) pos1 = -2;
	speed1 = DMXSlaveGet(dmxchan + 1);
	pos2 = DMXSlaveGet(dmxchan + 2);
	if(pos2 == 0) pos2 = -10;
	speed2 = DMXSlaveGet(dmxchan + 3);
	
	if(DCMOTOR(C).VolVars.homed == 0) {
		DCMOTOR(C).Setting.Mode = 0; // mode PWM
		DCMOTOR(C).Vars.PWMConsign = REWIND_SPEED;
		rampSetPos(&DCMOTOR(C).PosRamp, 0);
	} else {
		rampGoto(&DCMOTOR(C).PosRamp, CONSIGN_POS(pos1, END_POS_TRANS));
		DCMOTOR(C).PosRamp.maxSpeed = CONSIGN_SPEED(speed1, MAX_SPEED_TRANS);
		DCMOTOR(C).Setting.Mode = 2; // mode POSITION
	}

	if(DCMOTOR(D).VolVars.homed == 0) {
		DCMOTOR(D).Setting.Mode = 0; // mode PWM
		DCMOTOR(D).Vars.PWMConsign = REWIND_SPEED;
		rampSetPos(&DCMOTOR(D).PosRamp, 0);
	} else {
		rampGoto(&DCMOTOR(D).PosRamp, CONSIGN_POS(pos2, END_POS_ROT));
		DCMOTOR(D).PosRamp.maxSpeed = CONSIGN_SPEED(speed2, MAX_SPEED_ROT);
		DCMOTOR(D).Setting.Mode = 2; // mode POSITION
	}
}

void loop() {
	fraiseService();
	//analogService();
	switchService();
	dmxService();

	if(delayFinished(mainDelay)) // when mainDelay triggers 
	{
		delayStart(mainDelay, 5000); 	// re-init mainDelay

		//DCMOT_GETPOS(X) = Analog_Get(0);
		//if(!switchSend()) analogSend();
		switchSend();

		wdService();
		dmxService();

		fraiseService();
		/*if(!wdOK()) {
			DCMOTOR(C).Vars.PWMConsign = 0;
			DCMOTOR(C).Setting.Mode = 0;
			DCMOTOR(D).Vars.PWMConsign = 0;
			DCMOTOR(D).Setting.Mode = 0;
		}*/

		digitalWrite(MOTC_ENDLED, digitalRead(MOTC_END) == MOTC_ENDLEVEL);
		digitalWrite(MOTD_ENDLED, digitalRead(MOTD_END) == MOTD_ENDLEVEL);

		DCMOTOR_COMPUTE(C, ASYM);
		DCMOTOR_COMPUTE(D, ASYM);

		fraiseService();
		
		if(++mainCount > PERIOD){
			static byte printstate = 0;
			mainCount = 0;
			#if 0
			if(t2%2) printf("C MX %ld %ld %d %d\n",DCMOT_GETPOS(X),(long)(DCMOT(X).PosRamp.ConsignPos>>RAMP_UINCPOW), DCMOT(X).Vars.PWMConsign,DCMOT(X).Setting.Mode/*.VolVars.homed*/);
			/*else printf("C MD %ld %ld %d %d\n",DCMOT_GETPOS(D),(long)(DCMOT(D).PosRamp.ConsignPos>>RAMP_UINCPOW), DCMOT(D).Vars.PWMConsign,DCMOT(D).VolVars.homed);*/
			#endif
			if(printstate == 0) {
				printf("CM %d %ld %d %ld\n",
					DCMOTOR(C).Vars.PWMConsign, rampGetPos(&DCMOTOR(C).PosRamp),
					DCMOTOR(D).Vars.PWMConsign, rampGetPos(&DCMOTOR(D).PosRamp));
			} else if(printstate == 1) {
				printf("Cm %ld %d %ld %d\n", 
					DCMOTOR(C).VolVars.Position, DCMOTOR(C).VolVars.end, 
					DCMOTOR(D).VolVars.Position, DCMOTOR(D).VolVars.end);
			}
			if(printstate++ > 2) printstate = 0;
		}
	}
}


void fraiseReceiveChar()
{
	unsigned char c;
	unsigned char l = fraiseGetLen();	
	c=fraiseGetChar();
	if(c=='L'){	
		c=fraiseGetChar();
		/*if(c=='0') 
			{LED=0;}
		else LED=1;*/
	}
	else if(c=='E') {
		printf("C");
		for(c=1;c<l;c++) printf("%c",fraiseGetChar());
		putchar('\n');
	}
	else if(c=='W') { //watchdog
		wdReset();
	}
	else if(c=='S') { //EEPROM save
		if((fraiseGetChar() == 'A')
		&& (fraiseGetChar() == 'V')
		&& (fraiseGetChar() == 'E'))
			EEwriteMain();
	}
}


void fraiseReceive()
{
	unsigned char c;//,c2;
	
	c=fraiseGetChar();

	switch(c) {
		//PARAM_CHAR(1,t2); break;
		PARAM_CHAR(2,PERIOD); break;
		//case 20 : Servo_Input(); break;
		case 120 : DCMOTOR_INPUT(C); break;
		case 121 : DCMOTOR_INPUT(D); break;
		case 122 : dmxchan = fraiseGetInt(); break;
		case 124 : DMXSlaveSet(fraiseGetInt(), fraiseGetChar()); break;
	}
}

void EEdeclareMain()
{
	DCMOTOR_DECLARE_EE(C);
	DCMOTOR_DECLARE_EE(D);
	EEdeclareInt(&dmxchan);
}
