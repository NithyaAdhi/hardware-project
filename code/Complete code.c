
#include <avr/io.h>             // This header file includes the appropriate Input/output definitions for the device 
#include <util/delay.h>         // to use delay function we need to include this library
#include <stdlib.h>             // we'll be using itoa() function to convert integer to character array that resides in this library
#include <avr/interrupt.h>

#define F_CPU 8000000L


#define US_TRIG_1_POS	PC0         // Trigger pin of Ultrasonic 1 is connected to the Port C pin 0
#define US_ECHO_1_POS	PC1         // Echo pin of Ultrasonic 1 is connected to the Port C pin 1

#define US_TRIG_2_POS   PC2         // Trigger pin of Ultrasonic 2 is connected to the Port C pin 2
#define US_ECHO_2_POS   PC3         // Echo pin of Ultrasonic 2 is connected to the Port C pin 3


#define US_1_ERROR		-1      // Defining four more variables two know if the ultrasonic sensor is working or not 
#define	US_1_NO_OBSTACLE -2
#define US_2_ERROR  -1
#define US_2_NO_OBSTACLE -2

int distance_1, distance_2;      // variable for store distance readings of two ultrsonic sensors

#define DRIVE_MOTOR PC4     // MAIN DRIVE MOTOR OF THE CART connected to the Port C pin 4
#define BUZZER PC5          // BUZZER is connected to the port C pin 5
#define BLADE_MOTOR PC6     // All 4 blade motors drive by single output and it is connected with the port C pin 6

#define SWA PA0          // Bottom side Proximity sensors are connected to port A pin 0 to 3
#define SWB PA1
#define SWC PA2
#define SWD PA3


#define SWP PA4         //  Top side Proximity sensors are connected to port A pin 4 to 7
#define SWQ PA5
#define SWR PA6
#define SWS PA7




#define ACTUATOR_A_DIR1 PB0   // Actuator A direction control pins are connected with port B pin 0 and 1
#define ACTUATOR_A_DIR2 PB1

#define ACTUATOR_B_DIR1 PB2   // Actuator B direction control pins are connected with port B pin 2 and 3
#define ACTUATOR_B_DIR2 PB3

#define ACTUATOR_C_DIR1 PB4   // Actuator C direction control pins are connected with port B pin 4 and 5
#define ACTUATOR_C_DIR2 PB5

#define ACTUATOR_D_DIR1 PB6   // Actuator D direction control pins are connected with port B pin 6 and 7
#define ACTUATOR_D_DIR2 PB7


int enc = 0;         // Encoder conting variable

int PRX_A_ENABLE, PRX_B_ENABLE, PRX_C_ENABLE, PRX_D_ENABLE=0;    // variables for identify the activation of proximity sensors
int ACTUATOR_A_UP,ACTUATOR_B_UP,ACTUATOR_C_UP,ACTUATOR_D_UP =0;  // variables for identify the position of the actuator


int PRX_A,PRX_B,PRX_C,PRX_D,PRX_P,PRX_Q,PRX_R,PRX_S;   // variable for proximity identification process

void HCSR04Init();        // Call to Initialize the ultrasonic sensors
void HCSR04_1_Trigger();  // Call to send a trigger signal to ultrasonic sensor 1
void HCSR04_2_Trigger();  // Call to send a trigger signal to ultrasonic sensor 2

void HCSR04Init()
{
	
	DDRC|=(1<<US_TRIG_1_POS);  // Ultrasonic sensor 1 Trigger pin set as output
	DDRC|=(1<<US_TRIG_2_POS);  // Ultrasonic sensor 2 Trigger pin set as output
	
}



void HCSR04_1_Trigger()
{   // this function will generate ultrasonic sound wave for 15 microseconds 
		
	PORTC|=(1<<US_TRIG_1_POS);	//high
	
	_delay_us(15);				//wait 15uS
	
	PORTC &=~(1<<US_TRIG_1_POS);	//low
}

void HCSR04_2_Trigger()
{   // this function will generate ultrasonic sound wave for 15 microseconds 
	
	
	PORTC|=(1<<US_TRIG_2_POS);	//high
	
	_delay_us(15);				//wait 15uS
	
	PORTC &=~(1<<US_TRIG_2_POS);	//low
}



uint16_t GetPulseWidth_1()
{
	// this function will be used to measure the pulse duration. When the ultra sound echo back after hitting an object
	// the microcontroller will read the pulse using the echo pin of the ultrasonic sensor connected to it. 
	
	uint32_t i,result;

	// Section - 1: the following lines of code before the section - 2 is checking if the ultrasonic is working or not
	// it check the echo pin for a certain amount of time. If there is no signal it means the sensor is not working or not connect properly
	for(i=0;i<600000;i++)
	{
		if(!(PINC & (1<<US_ECHO_1_POS)))   // if echo pulse not receive continue and watch until receive a pulse, if receive break the loop and goto measure the pulse duration
		continue;	//Line is still low, so wait
		else
		break;		//High edge detected, so break.
	}

	if(i==600000)
	return US_1_ERROR;	//Indicates time out
	
	//High Edge Found
	
	// Section -2 : This section is all about preparing the timer for counting the time of the pulse. Timers in microcontrllers is used for timimg operation
	//Setup Timer1
	TCCR1A=0X00;
	TCCR1B=(1<<CS11);	// This line sets the resolution of the timer. Maximum of how much value it should count.
	TCNT1=0x00;			// This line start the counter to start counting time

	// Section -3 : This section checks weather the there is any object or not
	for(i=0;i<600000;i++)                // the 600000 value is used randomly to denote a very small amount of time, almost 40 miliseconds 
	{
		if(PINC & (1<<US_ECHO_1_POS))
		{
			if(TCNT1 > 60000) break; else continue;   // if the TCNT1 value gets higher than 60000 it means there is not object in the range of the sensor
		}
		else
		break;
	}

	if(i==600000)
	return US_1_NO_OBSTACLE;	//Indicates time out

	//Falling edge found

	result=TCNT1;          // microcontroller stores the the value of the counted pulse time in the TCNT1 register. So, we're returning this value to the 
	                       // main function for utilizing it later 

	//Stop Timer
	TCCR1B=0x00;

	if(result > 60000)
	return US_1_NO_OBSTACLE;	//No obstacle
	else
	return (result>>1);
}


uint16_t GetPulseWidth_2()
{
	// this function will be used to measure the pulse duration. When the ultra sound echo back after hitting an object
	// the microcontroller will read the pulse using the echo pin of the ultrasonic sensor connected to it. 
	
	uint32_t i,result;

	// Section - 1: the following lines of code before the section - 2 is checking if the ultrasonic is working or not
	// it check the echo pin for a certain amount of time. If there is no signal it means the sensor is not working or not connect properly
	for(i=0;i<600000;i++)
	{
		if(!(PINC & (1<<US_ECHO_2_POS)))    // if echo pulse not receive continue and watch until receive a pulse, if receive break the loop and goto measure the pulse duration
		continue;	//Line is still low, so wait
		else
		break;		//High edge detected, so break.
	}

	if(i==600000)
	return US_2_ERROR;	//Indicates time out
	
	//High Edge Found
	
	// Section -2 : This section is all about preparing the timer for counting the time of the pulse. Timers in microcontrllers is used for timimg operation
	//Setup Timer1
	TCCR1A=0X00;
	TCCR1B=(1<<CS11);	// This line sets the resolution of the timer. Maximum of how much value it should count.
	TCNT1=0x00;			// This line start the counter to start counting time

	// Section -3 : This section checks weather the there is any object or not
	for(i=0;i<600000;i++)                // the 600000 value is used randomly to denote a very small amount of time, almost 40 miliseconds 
	{
		if(PINC & (1<<US_ECHO_2_POS))
		{
			if(TCNT1 > 60000) break; else continue;   // if the TCNT1 value gets higher than 60000 it means there is not object in the range of the sensor
		}
		else
		break;
	}

	if(i==600000)
	return US_2_NO_OBSTACLE;	//Indicates time out

	//Falling edge found

	result=TCNT1;          // microcontroller stores the the value of the counted pulse time in the TCNT1 register. So, we're returning this value to the 
	                       // main function for utilizing it later 

	//Stop Timer
	TCCR1B=0x00;

	if(result > 60000)
	return US_2_NO_OBSTACLE;	//No obstacle
	else
	return (result>>1);
}




void read_sensor(){            // Check the status of proximity sensors
	if((PINA & (1<< SWA))){
		PRX_A = 1;             // if proximity sensor A is activated, then "PRX_A" variable set to 1
	}

	if((PINA & (1<< SWB))){
		PRX_B = 1;
	}

	if((PINA & (1<< SWC))){
		PRX_C = 1;
	}
	
	if((PINA & (1<< SWD))){
		PRX_C = 1;
	}
	
	//********************************
	
	if((PINA & (1<< SWP))){
		PRX_P = 1;
	}
	if((PINA & (1<< SWQ))){
		PRX_Q = 1;
	}
	if((PINA & (1<< SWR))){
		PRX_R = 1;
	}
	if((PINA & (1<< SWS))){
		PRX_S = 1;
	}	
	
}	



void initInterrupts(void); //The Interrupt Service Routine for external INT1

int start_count =0;


ISR(INT1_vect)
{

/* When an interrupt occurs, we only have to check the level of 
pin PD5 to determine the direction */

	if(PIND & _BV(PD5))
		enc++; //Increase enc

	else
		enc--; //Decrease enc
}



void initInterrupts(void)
{
	DDRD &= ~(1<<PD5);  // interrupt pin D5 set as input
	DDRD &= ~(1<<PD3);  // interrupt pin D3 set as input

	PORTD |= (1<<PD5);  //Enable the pull-up resistors
    PORTD |= (1<<PD3);
	MCUCR |= (1<<ISC11); //Falling edge in INT1 (PD3) to cause interrupt
	
	GICR |= (1<<INT1);   //Enable and INT1

}


//****************** main loop *************************************************************************************
	
int main()
{

initInterrupts(); //Set all the pins on registers

sei(); //Turn on interrupts

PORTC |= (1<< DRIVE_MOTOR); // CART DRIVES FROWARD
	
	while(1) {
		
		uint16_t r1;   // 16 bit two variables for save ultrasonic pulse duration
		uint16_t r2;
		
		_delay_ms(100);	

		
	
		HCSR04Init();  // Initialize ultrasonic sensors


	
		while(1)
		{
			
			read_sensor();      // Read the status of proximity sensors
			
			//Send a trigger pulse
			HCSR04_1_Trigger();               // calling the ultrasonic sound wave generator function

			//Measure the width of pulse
			r1=GetPulseWidth_1();             // getting the duration of the ultrasound took to echo back after hitting the object

			//Handle Errors
			
			
			HCSR04_2_Trigger();
			
			r2=GetPulseWidth_2();
	
			distance_1=(r1*0.034/2.0);	// This will give the distance in centimeters of 1st sensor
				
			distance_2=(r2*0.034/2.0);   // This will give the distance in centimeters of 2nd sensor

			
	
	
	
				
		if((distance_1 < 50)||(distance_2 <50)){  // if detects any object by any ultrasonic sensor
			
			PORTC &= ~(1<< DRIVE_MOTOR);   // turn off the drive motor
			PORTC |= (1<< BUZZER);         // Turn on the buzzer
			_delay_ms(60000);              // Wait 1 minute
			
			PORTC |= (1<< DRIVE_MOTOR);    // Start to go forward again
			PORTC &= ~(1<< BUZZER);        // Turn off the buzzer
		}	




//*************** Function of Upper side proximity sensors **********************************************************************		
		 
		if((PRX_P==1)||(PRX_Q==1)||(PRX_R==1)||(PRX_S==1)){  //if any fixed object hits the wheel, turn off the system
subloop1:			
			PORTC &= ~(1<< DRIVE_MOTOR);
			
			PORTC &= ~(1<< BLADE_MOTOR);

			
			goto subloop1;           // stuck the whole system in subloop1
				
		}	


//************ Function of Down side proxmity sensors ************************************************************************************		

///////////// ******************* Sensor A ******************

		if(PRX_A==1){    // if 1st lower side proximity turn on, then reset the encoder counter and set the "PRX_A_ENAGLE" variable to one
			enc=0;
		    PRX_A_ENABLE = 1;
			
		}
		
	if((PRX_A_ENABLE == 1)&&(enc >= 100)){  // move up the actuator A
		
		PORTB |= (1<< ACTUATOR_A_DIR1);   
		PORTB &= ~(1<< ACTUATOR_A_DIR2);
		
		ACTUATOR_A_UP = 1;
		
	    _delay_ms(2000);
	  
	}
	
	if((ACTUATOR_A_UP ==1 )&&(PRX_A==0))	{  // if the detected object released, then move down the actuator A again
		
 		PORTB &= ~(1<< ACTUATOR_A_DIR1);
		PORTB |= (1<< ACTUATOR_A_DIR2);  
		_delay_ms(1000);
	}		
		
		
///////////// ******************* Sensor B ******************


		if(PRX_B==1){  // do the same process that described in sensor A
			enc=0;
		    PRX_B_ENABLE = 1;
			
		}
		
	if((PRX_B_ENABLE == 1)&&(enc >= 100)){  // move up the actuator B
		
		PORTB |= (1<< ACTUATOR_B_DIR1);   
		PORTB &= ~(1<< ACTUATOR_B_DIR2);
		
		ACTUATOR_B_UP = 1;
		
	    _delay_ms(2000);
	  
	}
	
	if((ACTUATOR_B_UP ==1 )&&(PRX_B==0))	{  // move down the actuator A
		
 		PORTB &= ~(1<< ACTUATOR_B_DIR1);
		PORTB |= (1<< ACTUATOR_B_DIR2);  
		_delay_ms(1000);
	}		
			
		
///////////// ******************* Sensor C ******************	

		if(PRX_C==1){         // do the same process that described in sensor A
			enc=0;
		    PRX_C_ENABLE = 1;
			
		}
		
	if((PRX_C_ENABLE == 1)&&(enc >= 100)){  // move up the actuator C
		
		PORTB |= (1<< ACTUATOR_C_DIR1);   
		PORTB &= ~(1<< ACTUATOR_C_DIR2);
		
		ACTUATOR_C_UP = 1;
		
	    _delay_ms(2000);
	  
	}
	
	if((ACTUATOR_C_UP ==1 )&&(PRX_C==0))	{  // move down the actuator C
		
 		PORTB &= ~(1<< ACTUATOR_C_DIR1);
		PORTB |= (1<< ACTUATOR_C_DIR2);  
		_delay_ms(1000);
	}	
	
									

///////////// ******************* Sensor D ******************
	
		if(PRX_D==1){         // do the same process that described in sensor A
			enc=0;
		    PRX_D_ENABLE = 1;
			
		}
		
	if((PRX_D_ENABLE == 1)&&(enc >= 100)){  // move up the actuator D
		
		PORTB |= (1<< ACTUATOR_D_DIR1);   
		PORTB &= ~(1<< ACTUATOR_D_DIR2);
		
		ACTUATOR_D_UP = 1;
		
	    _delay_ms(2000);
	  
	}
	
	if((ACTUATOR_D_UP ==1 )&&(PRX_D==0))	{  // move down the actuator D
		
 		PORTB &= ~(1<< ACTUATOR_D_DIR1);
		PORTB |= (1<< ACTUATOR_D_DIR2);  
		_delay_ms(1000);
	}	
	
	
				_delay_ms(30);
				
				
				
			}
		}
		
	}
