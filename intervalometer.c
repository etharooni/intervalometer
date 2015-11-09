#define F_CPU 16000000UL

#include <avr/io.h> 
#include <util/delay.h>
#include <avr/interrupt.h>
//#include <avr/power.h>
#include <avr/sleep.h>
//#include <math.h>
//#include <string.h>

#define led_a 7		//pd7
#define led_b 6		//pd6
#define led_c 5		//pd5
#define led_d 4		//pd4
#define led_e 1		//pb1
#define led_f 0		//pb0
#define led_g 1		//pd1
#define led_dp 0	//pd0
#define led_col 2	//pb2

#define led_d1 5	//pc5
#define led_d2 4	//pc4
#define led_d3 3	//pc3
#define led_d4 2	//pc2

#define enc_pina 1 //PC1/PCINT8
#define enc_pinb 0 //PC0/PCINT9
#define enc_mode 2 //PD2/INT0

#define irPin 4
#define cableRelease 3 //PB3

#define cablereleasetime 2000

#define timerfreq 16.384 //ms
#define fasttimer_freq 2 //mhz
#define encmaxchange 3
#define padding 64 //ms
#define indicator 250 //ms

#define but_start 3 //PD3/INT1

#define led_us 1000

typedef enum {F, T} bit;

bit centerColon = F;
bit dPoints[4] = {F};
int mode = 0;
bit running = T;
bit execute = F;
long long encDis = 0;
long long encDisBig = 0;
long long encDis_prev = 0;
int A_minutes = 0;
int A_seconds = 30; //eeprom
int B_minutes = 0;
int B_seconds = 0; //eeprom
int num_amount = 0;
int enc_prevstate = 0;
unsigned long long tick = 0;
int encoderbutton = 1;
bit push_rotate = F;
int prev_encoderbutton = 1;
bit sleepEnable = T;

void ledSeg(int seg, bit dp){

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * \
	*     +---A---+      *                     *                   *
	*     |       |      *      A' - PD7       *                   *
	*     F       B      *      B' - PD6       *                   *
	*     |       |      *      C' - PD5       *     Dig1 - PC5    *
	*     +---G---+      *      D' - PD4       *     Dig2 - PC4    *
	*     |       |      *      E' - PB1       *     Dig3 - PC3    *
	*     E       C      *      F' - PB0       *     Dig4 - PC2    *
	*     |       |      *      G' - PD1       *                   *
	*     +---D---+      *     DP' - PD0       *                   *
	*                DP  *                     *                   *
	\ * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	
	//Segments E, F on Port B - others on Port D
	//the display has a common anode, so the segments are inverted

	switch (seg){
		default:
			PORTD |=((1<<led_a)|(1<<led_b)|(1<<led_c)|(1<<led_d)|(1<<led_g)|(1<<led_dp)); //clear a,b,c,d,g, and dp on port D
			PORTB |=((1<<led_e)|(1<<led_f)); //clear e,f on port B
			break;
		case 0:
			//A,B,C,D,E,F
			PORTD &=~((1<<led_a)|(1<<led_b)|(1<<led_c)|(1<<led_d));
			PORTB &=~((1<<led_e)|(1<<led_f));
			break;
		case 1:
			//B,C
			PORTD &=~((1<<led_b)|(1<<led_c));
			break;
		case 2:
			//A,B,D,E,G
			PORTD &=~((1<<led_a)|(1<<led_b)|(1<<led_d)|(1<<led_g));
			PORTB &=~(1<<led_e);
			break;
		case 3:
			//A,B,C,D,G
			PORTD &=~((1<<led_a)|(1<<led_b)|(1<<led_c)|(1<<led_d)|(1<<led_g));
			break;
		case 4:
			//B,C,F,G
			PORTD &=~((1<<led_b)|(1<<led_c)|(1<<led_g));
			PORTB &=~(1<<led_f);
			break;
		case 5:
			//A,C,D,F,G
			PORTD &=~((1<<led_a)|(1<<led_c)|(1<<led_d)|(1<<led_g));
			PORTB &=~(1<<led_f);
			break;
		case 6:
			//A,C,D,E,F,G
			PORTD &=~((1<<led_a)|(1<<led_c)|(1<<led_d)|(1<<led_d)|(1<<led_g));
			PORTB &=~((1<<led_e)|(1<<led_f));
			break;
		case 7:
			//A,B,C
			PORTD &=~((1<<led_a)|(1<<led_b)|(1<<led_c));
			break;
		case 8:
			//A,B,C,D,E,F,G
			PORTD &=~((1<<led_a)|(1<<led_b)|(1<<led_c)|(1<<led_d)|(1<<led_g));
			PORTB &=~((1<<led_e)|(1<<led_f));
			break;
		case 9:
			//A,B,C,F,G
			PORTD &=~((1<<led_a)|(1<<led_b)|(1<<led_c)|(1<<led_g));
			PORTB &=~(1<<led_f);
			break;
	}

	if (centerColon == T){
		PORTB |=(1<<led_col);
	}else{
		PORTB &=~(1<<led_col);
	}

	switch (dp){
		case T:
			PORTD &=~(1<<led_dp);
			break;
		case F:
			PORTD |=(1<<led_dp);
			break;
	}
}

void ledCharSeg(char seg, bit dp){
	switch (seg){
		default:
			PORTD |= ((1<<led_a)|(1<<led_b)|(1<<led_c)|(1<<led_d)|(1<<led_g)|(1<<led_dp)); //clear a,b,c,d,g, and dp on port D
			PORTB |= ((1<<led_e)|(1<<led_f)); //clear e,f on port B
			//PORTD &= ~((1<<led_a)|(1<<led_b)|(1<<led_c)|(1<<led_d)|(1<<led_g));
			//PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'a':
		case 'A':
			//EFABCG
			PORTD &= ~((1<<led_a)|(1<<led_b)|(1<<led_c)|(1<<led_g));
			PORTB &=~((1<<led_e)|(1<<led_f));
			break;
		case 'b':
		case 'B':
			//DCGEF
			PORTD &= ~((1<<led_c)|(1<<led_d)|(1<<led_g));
			PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'c':
		case 'C':
			//GED
			PORTD &= ~((1<<led_d)|(1<<led_g));
			PORTB &= ~((1<<led_e));
			break;
		case 'd':
		case 'D':
			//GECDB
			PORTD &= ~((1<<led_b)|(1<<led_c)|(1<<led_d)|(1<<led_g));
			PORTB &= ~((1<<led_e));break;
		case 'e':
		case 'E':
			//AFGED
			PORTD &= ~((1<<led_a)|(1<<led_d)|(1<<led_g));
			PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'f':
		case 'F':
			//AFGE
			PORTD &= ~((1<<led_a)|(1<<led_g));
			PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'g':
		case 'G':
			//AFEDC
			PORTD &= ~((1<<led_a)|(1<<led_c)|(1<<led_d));
			PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'h':
		case 'H':
			//FEGC
			PORTD &= ~((1<<led_c)|(1<<led_g));
			PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'i':
		case 'I':
			//BC
			PORTD &= ~((1<<led_b)|(1<<led_c));
			break;
		case 'j':
		case 'J':
			//EDCB
			PORTD &= ~((1<<led_b)|(1<<led_c)|(1<<led_d));
			PORTB &= ~((1<<led_e));
			break;
		case 'k':
		case 'K':
			//FEG
			PORTD &= ~((1<<led_g));
			PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'l':
		case 'L':
			//FED
			PORTD &= ~((1<<led_d));
			PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'm':
		case 'M':
			//ECA
			PORTD &= ~((1<<led_a)|(1<<led_c));
			PORTB &= ~((1<<led_e));
			break;
		case 'n':
		case 'N':
			//EGC
			PORTD &= ~((1<<led_c)|(1<<led_g));
			PORTB &= ~((1<<led_e));
			break;
		case 'o':
		case 'O':
			//EGCD
			PORTD &= ~((1<<led_c)|(1<<led_d)|(1<<led_g));
			PORTB &= ~((1<<led_e));
			break;
		case 'p':
		case 'P':
			//EFABG
			PORTD &= ~((1<<led_a)|(1<<led_b)|(1<<led_g));
			PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'q':
		case 'Q':
			//ABGFC
			PORTD &= ~((1<<led_a)|(1<<led_b)|(1<<led_c)|(1<<led_g));
			PORTB &= ~((1<<led_f));
			break;
		case 'r':
		case 'R':
			//EG
			PORTD &= ~((1<<led_g));
			PORTB &= ~((1<<led_e));
			break;
		case 's':
		case 'S':
			//AFGCD
			PORTD &= ~((1<<led_a)|(1<<led_c)|(1<<led_d)|(1<<led_g));
			PORTB &= ~((1<<led_f));
			break;
		case 't':
		case 'T':
			//FGED
			PORTD &= ~((1<<led_d)|(1<<led_g));
			PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'u':
		case 'U':
			//FEDCB
			PORTD &= ~((1<<led_b)|(1<<led_c)|(1<<led_d));
			PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'v':
		case 'V':
			//EDC
			PORTD &= ~((1<<led_c)|(1<<led_d));
			PORTB &= ~((1<<led_e));
			break;
		case 'w':
		case 'W':
			//FGBD
			PORTD &= ~((1<<led_b)|(1<<led_d)|(1<<led_g));
			PORTB &= ~((1<<led_f));
			break;
		case 'x':
		case 'X':
			//EFGBC
			PORTD &= ~((1<<led_b)|(1<<led_c)|(1<<led_g));
			PORTB &= ~((1<<led_e)|(1<<led_f));
			break;
		case 'y':
		case 'Y':
			//FGBCD
			PORTD &= ~((1<<led_b)|(1<<led_c)|(1<<led_d)|(1<<led_g));
			PORTB &= ~((1<<led_f));
			break;
		case 'z':
		case 'Z':
			//ABGED
			PORTD &= ~((1<<led_a)|(1<<led_b)|(1<<led_d)|(1<<led_g));
			PORTB &= ~((1<<led_e));
			break;
	}
	if (centerColon == T){
		PORTB |=(1<<led_col);
	}else{
		PORTB &=~(1<<led_col);
	}

	switch (dp){
		case T:
			PORTD &=~(1<<led_dp);
			break;
		case F:
			PORTD |=(1<<led_dp);
			break;
	}
}

void ledAddr(int addr){
	switch (addr){
		default:
			PORTC &=~((1<<led_d1)|(1<<led_d2)|(1<<led_d3)|(1<<led_d4)); //reset all digits
			break;
		case 1:
			PORTC |=(1<<led_d1);
			break;
		case 2:
			PORTC |=(1<<led_d2);
			break;
		case 3:
			PORTC |=(1<<led_d3);
			break;
		case 4:
			PORTC |=(1<<led_d4);
			break;
	}

}
void ledWriteSegs(int num, bit dpA = dPoints[0], bit dpB = dPoints[1], bit dpC = dPoints[2], bit dpD = dPoints[3]){ //update segments
	int digit=4;
	bit dpstate = F;
	int x = 10;
	bit showDigits = T;
	if (num > 9999) showDigits = F;
	for (digit=4;digit>=1;digit--){
		//int length=intlen(num);
		//if (length > 4) num = num / pow(10,length-4); //keep only the first four digits
		if (showDigits == T){
			x = num % 10;
		}else{
			x = 10; //10 - not in case
		}
		switch (digit){
			case 1:
				dpstate = dpA;
				dPoints[0] = dpstate;
				break;
			case 2:
				dpstate = dpB;
				dPoints[1] = dpstate;
				break;
			case 3:
				dpstate = dpC;
				dPoints[2] = dpstate;
				break;
			case 4:
				dpstate = dpD;
				dPoints[3] = dpstate;
				break;
		}
		PORTC &=~((1<<led_d1)|(1<<led_d2)|(1<<led_d3)|(1<<led_d4)); //clear all digits
		PORTD |=((1<<led_a)|(1<<led_b)|(1<<led_c)|(1<<led_d)|(1<<led_g)|(1<<led_dp)); //clear a,b,c,d,g, and dp on port D
		PORTB |=((1<<led_e)|(1<<led_f)); //clear e,f on port B
		ledSeg(x,dpstate);
		ledAddr(digit);
		num = num / 10;
		_delay_us(led_us);
	}
}

void ledWriteChar(char string[4], bit dpA = dPoints[0], bit dpB = dPoints[1], bit dpC = dPoints[2], bit dpD = dPoints[3]){ //update segments
	int digit=4;
	bit dpstate = F;
	char c;
	for (digit=4;digit>=1;digit--){
		switch (digit){
			case 1:
				dpstate = dpA;
				dPoints[0] = dpstate;
				break;
			case 2:
				dpstate = dpB;
				dPoints[1] = dpstate;
				break;
			case 3:
				dpstate = dpC;
				dPoints[2] = dpstate;
				break;
			case 4:
				dpstate = dpD;
				dPoints[3] = dpstate;
				break;
		}
		c = string[digit-1];
		PORTC &=~((1<<led_d1)|(1<<led_d2)|(1<<led_d3)|(1<<led_d4)); //clear all digits
		PORTD |=((1<<led_a)|(1<<led_b)|(1<<led_c)|(1<<led_d)|(1<<led_g)|(1<<led_dp)); //clear a,b,c,d,g, and dp on port D
		PORTB |=((1<<led_e)|(1<<led_f)); //clear e,f on port B
		ledCharSeg(c, dpstate);
		ledAddr(digit);
		_delay_us(led_us);
	}
}

int intlen(int lenstart) {
	int end = 0;
	while(lenstart > 0) {
    		lenstart = lenstart/10;
    		end++;
	}
	return end;
}  

int splitnums(int numA, int numB){
	return (numA*100)+numB;
}

void indicate(){ //todo: flash
	unsigned long long tickprev = tick;
	while((tick - tickprev)*timerfreq < indicator){
		ledWriteSegs(10000,T,F,F,F);
	}
	tickprev = tick;
	while((tick - tickprev)*timerfreq < indicator){
		ledWriteSegs(10000,F,T,F,F);
	}
	tickprev = tick;
	while((tick - tickprev)*timerfreq < indicator){
		ledWriteSegs(10000,F,F,T,F);
	}
	tickprev = tick;
	while((tick - tickprev)*timerfreq < indicator){
		ledWriteSegs(10000,F,F,F,T);
	}
}

ISR(PCINT1_vect){ //encoder interrupt   todo:if button pushed AND... change A_minutes
	int enc_A = (PINC >> enc_pina) & 1;
	int enc_B = (PINC >> enc_pinb) & 1;
	if (((PIND >> 2) & 1) == 0) push_rotate = T;
	//note - inverted
	if ((enc_A == 1)&&(enc_B == 1)){//		0,0 - state 1
		if (enc_prevstate == 4){
			encDisBig++;
		}else if (enc_prevstate == 2){
			encDisBig--;
		}
		enc_prevstate = 1;
	}else if ((enc_A == 0)&&(enc_B == 1)){//	1,0 - state 2
		if (enc_prevstate == 1){
			encDisBig++;
		}else if (enc_prevstate == 3){
			encDisBig--;
		}
		enc_prevstate = 2;
	}else if ((enc_A == 0)&&(enc_B == 0)){//	1,1 - state 3
		if (enc_prevstate == 2){
			encDisBig++;
		}else if (enc_prevstate == 4){
			encDisBig--;
		}
		enc_prevstate = 3;
	}else if ((enc_A == 1)&&(enc_B == 0)){//	0,1 - state 4
		if (enc_prevstate == 3){
			encDisBig++;
		}else if (enc_prevstate == 1){
			encDisBig--;
		}
		enc_prevstate = 4;
	}
	encDis = encDisBig/4;
	if ((encDis - encDis_prev) > encmaxchange) encDis_prev = encDis;
}

void setup(){
	DDRD = 0b11110011;
	DDRC = 0b111100;
	DDRB = 0b1111;
	TCCR2B |= (1<<CS22)|(1<<CS21)|(1<<CS20); //set the timer0 prescaler to 1024 <- overflow triggered every 16ms at 16mhz <- p.164
	// 1 / ( (16000000/1024) / (2^8) ) * 1000 = 16.384ms
	//         ^hertz  ^prescale ^bits = 61.03515625 hertz
	
	TIMSK2 |= (1<<TOIE2); //enable overflow interrupts for timer0
	TCNT2 = 0; //reset and init counter
	tick = 0;
	
	PCMSK1 |= (1 << PCINT8)|(1 << PCINT9); //set interrupts on PCINT8 (PC0) and PCINT9 (PC1) for encoder on channel 1	
	PCICR |= (1 << PCIE1);//Enable channel 1 (PCINTs 8 and 9)
	EICRA |= (1<<ISC00)|(1<<ISC01)|(1<<ISC10)|(1<<ISC11); //set interrupts on rising edges of INT0(PD2) and INT1(PD3) <- p.73 of doc8271
	EIMSK  |= (1<<INT1); //enable interrupts on INT0, INT1
	EIMSK  &= ~(1<<INT0);
	sei();
}

void sleep_wakeUp(){
	sleep_disable();
	sleepEnable = F;
	setup();
}

void sleepNow(){
	sleepEnable = T;
	EIMSK |= (1<<INT0);
	PCICR &= ~(1 << PCIE1);
	TIMSK2 &= ~(1<<TOIE2);
	sei();
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	sleep_mode();
}

void sleepTimer(){
	sleepEnable = T;
	EIMSK |= (1<<INT0);
	PCICR &= ~(1 << PCIE1);
	sei();
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;
	set_sleep_mode(SLEEP_MODE_PWR_SAVE);
	sleep_enable();
	sleep_mode();
}

ISR(INT1_vect){ //start button interrupt
	if(sleepEnable == T){
		sleep_wakeUp();
	}else{
		if (execute == F){
			mode = 4;
		}else{
			mode = 1;
		}
		running = F;
	}
}
ISR(INT0_vect){ //encoder button interrupt
	if(sleepEnable == T){
		sleep_wakeUp();
	}
}

ISR(TIMER2_OVF_vect){ // timer0 overflow interrupt triggered every ~16ms
	tick++;
}

void checkEncoderButton(){
	encoderbutton = (PIND >> 2) & 1;
	if ((encoderbutton == 1) && (prev_encoderbutton == 0)){
			if (push_rotate == T){
				push_rotate = F;
			}else{
				mode++;
				if (mode>3)	mode = 0;
				running = F;
			}
	}//else{A_minutes = encoderbutton;}
	prev_encoderbutton = encoderbutton;
}

void setupIRtimer(){
	TCCR2B &= ~(1<<CS22)|(1<<CS20);//timer prescaler set to 8
	//(1/(16000000/8)*1000*1000 = 0.5 us precision
	sei();
}

void irPause(unsigned int time){
	unsigned long long start = (tick << 8)+TCNT2;
	while((((tick << 8)+TCNT2)-start)/fasttimer_freq <= time){
	}
}

void irOn(unsigned int time, int freq){
	int pause = (1000/freq/2)-4;
	unsigned long long start = (tick << 8)+TCNT2;
	while((((tick << 8)+TCNT2)-start)/fasttimer_freq <= time){
		PORTB |= (1<<irPin);
		irPause(pause);
		PORTB &= ~(1<<irPin);
		irPause(pause);
	}
}

void NikonSnap(){
	int irFreq = 40;
	irOn(2000,irFreq);
	irPause(27830);
	irOn(390,irFreq);
	irPause(1580);
	irOn(410,irFreq);
	irPause(3580);
	irOn(400,irFreq);
}

void PentaxSnap(){
	int irFreq = 38;
	irOn(13000,irFreq);
	irPause(3000);
	for (int i=0;i<7;i++){
		irOn(1000,irFreq);
		irPause(1000);
	};
}

void OlympusSnap(){
	int irFreq = 40;
	bool _seq[33] = {0,1,1,0,0,0,0,1,1,1,0,1,1,1,0,0,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1};
	irOn(8972,irFreq);
	irPause(4384);
	irOn(624,irFreq);
	for (int i=0;i<33;i++){
		if (_seq[i]==0){
			irPause(488);
			irOn(600,irFreq);
		}
		else{
			irPause(1600);
			irOn(600,irFreq);
		}
	};
}

void MinoltaSnap(){
	int irFreq = 38;
	bool _seq[34] = {0,0,1,0,1,1,0,0,0,1,0,1,0,0,1,1,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1};
	irOn(3750,irFreq);
	irPause(1890);
	for (int i=0;i<34;i++){
		if (_seq[i]==0){
			irOn(456,irFreq);
			irPause(487);
		}
		else{
			irOn(456,irFreq);
			irPause(1430);
		}
	};
}

void SonySnap(){
	int irFreq = 40;
	bool _seq[21] = {1,0,1,1,0,1,0,0,1,0,1,1,1,0,0,0,1,1,1,1};
	int j;
	for (j=0;j<3;j++) {
		irOn(2320,irFreq);
		irPause(650);
		int i;
		for (i=0;i<20;i++){
			if (_seq[i]==0){
				irOn(575,irFreq);
				irPause(650);
			}
			else{
				irOn(1175,irFreq);
				irPause(650);
			}
		}
		irPause(10000);
	}
}

void CanonSnap(){
	int irFreq = 33;
	for(int i=0; i<16; i++) { 
		PORTB |= (1<<irPin);
		irPause(11);
		PORTB &= ~(1<<irPin);
		irPause(11);
	} 
	irPause(7330); 
	for(int i=0; i<16; i++) { 
		PORTB |= (1<<irPin);
		irPause(11);
		PORTB &= ~(1<<irPin);
		irPause(11);
	}
}

void SnapAll(){
	setupIRtimer();
	NikonSnap();
	CanonSnap();
	SonySnap();
	MinoltaSnap();
	OlympusSnap();
	PentaxSnap();
	setup();
	//cable release:
	PORTB |= (1<<cableRelease);
	unsigned long long start = tick;
	while ((tick-start)*timerfreq <= cablereleasetime){
	}
	PORTB &= ~(1<<cableRelease);
}

int main(){
	setup();
	while(1){
		while (1){ //Time A, Time B, number, half press/full/both+camera (canon,nikon...,all), 
			switch(mode){
				case 0: //A time
					centerColon = T;
					running = T;
					while(running == T){
						if (push_rotate == T){
							A_minutes = A_minutes + (encDis-encDis_prev);
							encDis_prev=encDis;
						}else{
							A_seconds = A_seconds + (encDis-encDis_prev);
							encDis_prev=encDis;
							if (A_seconds < 0){
								if (A_minutes != 0) A_seconds = 59; else A_seconds = 0;
								A_minutes--;
							}
							if (A_seconds > 59){
								if (A_minutes != 60) A_seconds = 0;else A_seconds = 59;
								A_minutes++;
							}
						}
						if (A_minutes < 0) A_minutes = 0;
						if (A_minutes > 60)A_minutes = 60;
						ledWriteSegs(splitnums(A_minutes,A_seconds),T,F,F,F);
						checkEncoderButton();
					}
					break;
				case 1: //B time
					centerColon = T;
					running = T;
					while(running == T){
						if (push_rotate == T){
							B_minutes = B_minutes + (encDis-encDis_prev);
							encDis_prev=encDis;
						}else{
							B_seconds = B_seconds + (encDis-encDis_prev);
							encDis_prev=encDis;
							if (B_seconds < 0){
								if (B_minutes != 0) B_seconds = 59; else B_seconds = 0;
								B_minutes--;
							}
							if (B_seconds > 59){
								if (B_minutes != 60) B_seconds = 0;else B_seconds = 59;
								B_minutes++;
							}
						}
						if (B_minutes < 0) B_minutes = 0;
						if (B_minutes > 60)B_minutes = 60;
						ledWriteSegs(splitnums(B_minutes,B_seconds),F,T,F,F);
						checkEncoderButton();
					}
					break;				
				case 2: //number
					centerColon = F;
					running = T;
					while(running == T){
						if (push_rotate == T){
							num_amount = num_amount + (encDis-encDis_prev);
						}else{
							num_amount = num_amount + (encDis-encDis_prev)*10;
						}
						encDis_prev=encDis;
						if (num_amount < 0) num_amount = 0;
						if (num_amount > 9999) num_amount = 9999;
						ledWriteSegs(num_amount,F,F,T,F);
						checkEncoderButton();
					}
					break;
				case 3:	//Options
					centerColon = F;
					running = T;
					while(running == T){
						int foobar=(((tick << 8)+TCNT2)/1562)%9999;
						ledWriteSegs(foobar,F,F,F,T);
						checkEncoderButton();
					}
					break;
				case 4: //execution
					execute = T;
					centerColon = F;
					running = T;
					//eeprom
					int i = 0;
					long Atime = ((((A_minutes*60)+A_seconds)*1000)-padding)-indicator*4;
					long Btime = ((((B_minutes*60)+B_seconds)*1000)-padding)-indicator*4;
					unsigned long long foobar = tick;
					while((tick-foobar)*timerfreq < 1000){
						ledWriteChar("snap",F,F,F,F);
					}
					//SnapAll();

					foobar = tick;
					while((tick-foobar)*timerfreq < 1000){
							ledWriteChar("boom",F,F,F,F);
					}
					foobar = tick;
					while ((tick-foobar)*timerfreq < 5000){
						sleepTimer();
					}
					foobar = tick;
					while((tick-foobar)*timerfreq < 10000){
						ledWriteChar("Done",F,F,F,F);
					}
					//}
					/*else{
						while((debounce_tick > 62)&&(running = T)){ //1s
							ledWriteSegs(10000,T,T,T,T); //error
						}
						running = F;
					}*/
					sleepNow();
					mode = 0;
					sleep_wakeUp();
					running = F;
					execute = F;
				break;	
			}
	
		}
	}
	return 0;
}
