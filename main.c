//Authors: Quoc Huynh, Callen Wilson
//Lab 13: 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"

#define clear_display 0x01
#define return_cursor 0x02
#define move_cursor_right 0x06
#define move_cursor_left 0x08
#define display_right 0x1C
#define display_left 0x18
#define cursor_blink 0x0F
#define cursor_off 0x0C
#define cursor_on 0x0E
#define set_4bit 0x28
#define set_8bit 0x38
#define entry_mode 0x06
#define func_8bit 0x0x32
#define set5x7font 0x20
#define firstRow 0x80
#define secondRow 0xC0

void PortF_Init(void);
void PWM_Init(void);
void Timer0A_usd(void);
void Timer1A_1sd(void);
void Timer2A_usd(void);
void Timer4A_msd(void);
void timer3A_captureTimer(void);
void Timer5A_init(void);
void frontDrive(void);
void reverseDrive(void);
void HC05_Init(void);
void left90(void);
void left45(void);
void right90(void);
void right45(void);
void stopDrive(void);
void LCD_init(void);
void LCD_Cmd(unsigned char cmd);
void LCD_write_char(unsigned char data);
void LCD_write_nibble(unsigned char data, unsigned char cmd);
void LCD_string(char *str);
char blueTooth_Read(void);
void blueTooth_Write(unsigned char data);
void write_string(char *str);
void Bluetooth_init(void);
void servo_Init(void);
void servoMotor(int pwm);
char servoBT_input(char c);
void findDist(void);
void distanceInit(void);
void check_obstacle(void);
void portA_init(void);
void portB_init(void);
void delay_ms(int n);
void delay_ms4A(int n);
void delay_us(int n);
void servo_delay_us(int n);
void servoMotor(int pwm);
void timer1A_delay(int ttime);

void ADC_Init(void);
void get_temperature(void);
void userInput(void);
void TIMER5A_Handler(void);
void processBluetoothCommand(void);
void DisableInterrupts(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);
void TIMER2A_Handler(void);
int isBluetoothDataAvailable(void);


int duty_cycle1 = 4999; //7812
int duty_cycle2 = 4999;
int cycle_count = 5000;
int temperature = 0;
int temperatureF = 0;
char snum[50];
char snum2 [24];
unsigned long dist = 0;
#define TRIGGER_DIST 30 //30CM DETECTION
char op = '0';  // Global variable for condition
//--------------LCD------------------
void LCD_init(void){
    	SYSCTL_RCGC2_R |= 0x9;
	//	GPIO_PORTB_DIR_R |= 0xFF;
	//	GPIO_PORTB_DEN_R |= 0xFF;
	// PA4-PA7 for Data and PD0-PD3 for command
    	GPIO_PORTA_DIR_R |= 0xF0;
    	GPIO_PORTA_DEN_R |= 0xF0;
    	GPIO_PORTD_DIR_R |= 0x0F;
    	GPIO_PORTD_DEN_R |= 0x0F;
   	 
    	LCD_Cmd(0x20);  // set 5x7 font
    	LCD_Cmd(0x28);	// set 4-bit
    	LCD_Cmd(0x06);	// move cursor right
    	LCD_Cmd(0x01);	// clear display
    	LCD_Cmd(0x0F);	// cursor blink
}
void LCD_Cmd(unsigned char cmd){
    	LCD_write_nibble(cmd &0xF0, 0);  // 0=command, RS =0
    	LCD_write_nibble(cmd << 4, 0);
   	 
    	// if lowest 4-bit data takes more time
    	if(cmd < 4)
            	delay_ms4A(2);
    	else
            	delay_us(40);
}
void LCD_string(char *str){
    	int i;
    	for(i=0; str[i]!=0; i++){
            	LCD_write_char(str[i]);
    	}
    
}
void LCD_write_nibble(unsigned char data, unsigned char cmd){
    	data = data & 0xF0;	// D4-7 data/command
    	cmd = cmd & 0x0F;	// D0-3 control , RS =0 for control, RS =1 for data
    	//GPIO_PORTB_DATA_R = data | cmd | 0x04;  // send data and command to LCD
   	 
    	GPIO_PORTA_DATA_R = data;
    	GPIO_PORTD_DATA_R = cmd;
    	GPIO_PORTD_DATA_R |= 0x04;
    	GPIO_PORTA_DATA_R |= data; // Ensuring data
    	GPIO_PORTD_DATA_R = 0;
	//GPIO_PORTD_DATA_R |= 0x04;
    	GPIO_PORTA_DATA_R = 0; // Done writing to LCD
}

void LCD_write_char(unsigned char data){
    	LCD_write_nibble(data &0xF0, 0x01);
    	LCD_write_nibble(data << 4, 0x01);
    	delay_us(40);
}


//-------------BT------------------
void HC05_Init(void) {
	SYSCTL_RCGCUART_R |= 0x02;   // Enable UART1 clock
	delay_ms(1);
    
	UART1_CTL_R = 0;         	// uart1 disabled
	UART1_IBRD_R = 325;      	// int(50000000/(16*9600))=325.52
	UART1_FBRD_R = 33;       	// round(0.52*64)=33.28
	UART1_LCRH_R = 0x60;     	// 8-bit word-len, with no parity bit set
	UART1_CTL_R = 0x301;     	// enables the uart1, rxe, and txe
}
void Bluetooth_init(void) {
	SYSCTL_RCGC2_R |= 0x02;  	// enable Port B clock
	delay_ms(1);
	GPIO_PORTB_AFSEL_R |= 0x03;  // enables alternate function on PB0, PB1
	GPIO_PORTB_DEN_R |= 0x03;	// enables digital I/O PB0, PB1
	GPIO_PORTB_PCTL_R &= ~0x000000FF; // clear PCTL bit  PB0, PB1
	GPIO_PORTB_PCTL_R |= 0x00000011; // configure uart for PB0, PB1
	GPIO_PORTB_AMSEL_R &= ~0x03; // disable the analog function on PB0, PB1
}
char blueTooth_Read(void) {
	char data;
	while((UART1_FR_R & (1<<4)) != 0); // waits for Rx buffer to be not full
	data = UART1_DR_R;         	 
	return data;
}
void blueTooth_Write(unsigned char data) {
	while((UART1_FR_R & (1<<5)) != 0); //waits for Tx buffer to be not full
	UART1_DR_R = data;             	// writes the data
}
void write_string(char *str) {
	int i;
	for(i = 0; i < strlen(str); i++) {
    	blueTooth_Write(str[i]);
	}
}

//-----------DRIVE Functions-----------
void stopDrive(void) {
	//GPIO_PORTA_DATA_R &= ~0xC; // clear PA2, PA3 (stop motor)
	GPIO_PORTF_DATA_R &= ~0x1E; // clear PF2, PF3 (stop motor)
	//GPIO_PORTF_DATA_R = 0x04;   
	timer1A_delay(100000);
    	PWM0_2_CMPA_R = duty_cycle1*0;
    	PWM0_2_CMPB_R = duty_cycle2*0;
   	timer1A_delay(100000);
    	PWM0_2_CMPA_R = duty_cycle1;
    	PWM0_2_CMPB_R = duty_cycle2;
    	timer1A_delay(100000);
}
void frontDrive(void){
    	GPIO_PORTF_DATA_R |= 0x10;
    	GPIO_PORTF_DATA_R &= ~0x2;
    	GPIO_PORTF_DATA_R |= 0x8;
    	GPIO_PORTF_DATA_R &= ~0x4;
    	timer1A_delay(20000);
}

void reverseDrive(void){

    	GPIO_PORTF_DATA_R |= 0x2;
    	GPIO_PORTF_DATA_R &= ~0x10;
    	GPIO_PORTF_DATA_R |= 0x4;
    	GPIO_PORTF_DATA_R &= ~0x8;
    
  	timer1A_delay(830000);
}
void right90(void) {
	GPIO_PORTF_DATA_R &= ~0x10;  
	GPIO_PORTF_DATA_R |= 0x2;    

	GPIO_PORTF_DATA_R |= 0x8;    
	GPIO_PORTF_DATA_R &= ~0x4;   

	PWM0_2_CMPA_R = duty_cycle1;
	PWM0_2_CMPB_R = duty_cycle2;
    
    	delay_ms4A(2000);
    	stopDrive();
}

void right45(void) {
	GPIO_PORTF_DATA_R &= ~0x10;  
	GPIO_PORTF_DATA_R |= 0x2;    

	GPIO_PORTF_DATA_R |= 0x8;    
	GPIO_PORTF_DATA_R &= ~0x4;   

	PWM0_2_CMPA_R = duty_cycle1;
	PWM0_2_CMPB_R = duty_cycle2;

    	delay_ms4A(1000);
    	stopDrive();
}

void left90(void) {
	GPIO_PORTF_DATA_R |= 0x10;    
	GPIO_PORTF_DATA_R &= ~0x2;    

	GPIO_PORTF_DATA_R &= ~0x8;    
	GPIO_PORTF_DATA_R |= 0x4;    

	PWM0_2_CMPA_R = duty_cycle1;
	PWM0_2_CMPB_R = duty_cycle2;
    
    	delay_ms4A(2000);
    	stopDrive();
}

void left45(void) {
	GPIO_PORTF_DATA_R |= 0x10;    
	GPIO_PORTF_DATA_R &= ~0x2;    

	GPIO_PORTF_DATA_R &= ~0x8;    
	GPIO_PORTF_DATA_R |= 0x4;    

	PWM0_2_CMPA_R = duty_cycle1;
	PWM0_2_CMPB_R = duty_cycle2;
    
    	delay_ms4A(1000);
    	stopDrive();
}


//---------------- Temperature sensor--------------------
// PD3
void ADC_Init(void){
	SYSCTL_RCGC2_R |= 0x08;  	// Enable clock for Port D
	SYSCTL_RCGCADC_R |= 0x01;	// Enable clock for ADC0

	/* Configure PD3 for ADC input */
	GPIO_PORTD_AFSEL_R |= 0x08;  // Enable alternate function for PD3
	GPIO_PORTD_DEN_R &= ~0x08;   // Disable digital function for PD3
	GPIO_PORTD_AMSEL_R |= 0x08;  // Enable analog function for PD3

	/* Initialize ADC0 */
	ADC0_ACTSS_R &= ~0x08;   	// Disable SS3 during configuration
	ADC0_EMUX_R &= ~0xF000;  	// Software trigger conversion
	ADC0_SSMUX3_R = 4;       	// Select AIN4 (PD3)
	ADC0_SSCTL3_R |= 0x06;   	// Take one sample at a time, set flag at 1st sample
	ADC0_ACTSS_R |= 0x08;    	// Enable ADC0 sequencer 3
	ADC0_PC_R = 0x01;        	// Configure ADC speed (default)
}

void get_temperature(void){
	ADC0_PSSI_R |= 0x08;     	// Start ADC conversion
	while((ADC0_RIS_R & 8) == 0x0);  // Wait for conversion to complete

	int adc_value = ADC0_SSFIFO3_R & 0xFFF; // Read ADC value
	// Convert ADC value to temperature (assuming 3.3V reference)
	temperature = ((adc_value - 620) * 100) / 1241;
    	//int temperature = ((ADC0_SSFIFO3_R &0xFFF)*190)/4095;
	delay_us(100);
	temperatureF = (temperature * 9 / 5) + 32; // Convert to Fahrenheit

	ADC0_ISC_R |= 0x8;  // Clear ADC interrupt flag
}


//-----------Port/PWM Init-------------
/*void portA_init(void){
            	volatile unsigned long delay;
    	SYSCTL_RCGC2_R |= 0x1;
    	delay = SYSCTL_RCGC2_R;    
    	//GPIO_PORTA_LOCK_R = 0x4C4F434B;
    	GPIO_PORTA_DIR_R |= 0xC; // PA0, PA1 for output to motor IN1, IN2
	//	GPIO_PORTA_CR_R = 0xC;
    	GPIO_PORTA_DEN_R = 0xC;
    
}*/

void PWM_Init(void){
	// PWM for PE4: M0PWM4 : Module 0 Generator 2 A
	// PWM for PE5: M0PWM5 : Module 0 Generator 2 B
    	SYSCTL_RCGCPWM_R |= 0x01; // M0PWM4/M0PWM5.. For module 0..
    	SYSCTL_RCGC2_R |= 0x30;  // PORTE.. M0PWM4.. PE4/PE5.. Mod0Gen2(A)..
    	while((SYSCTL_PRPWM_R & 0x01)!= 0x01); // V.important to wait
    	SYSCTL_RCC_R &= 0xFFF1FFFF; //
    	SYSCTL_RCC_R |= 0x001E0000; // PWM module frequency 1-MHz.. Bit20=1(selects the frequency divisor option)
    
    	GPIO_PORTE_CR_R |= 0x30;
    	GPIO_PORTE_AFSEL_R |= 0x30; // PE4, PE5
    	GPIO_PORTE_PCTL_R |= 0x00440000;
	//	GPIO_PORTF_DIR_R = 0x30;
    	GPIO_PORTE_DEN_R |= 0x30;
   	 
	//	GPIO_PORTF_DATA_R = 0x04;
    	// PWMm_g_CTL_R
    	PWM0_2_CTL_R = 0x00; // Disable M0 and down counter ...
	//	GPIO_PORTF_DATA_R = 0x04;
	//	PWM0_1_CTL_R &= ~0x2;  // Down counter set 0 in bit 1
	//	GPIO_PORTF_DATA_R = 0x04;
    	PWM0_2_GENA_R |= 0xC8;
    	PWM0_2_GENB_R |= 0xC08;
    	PWM0_2_LOAD_R = cycle_count; // Seting a 50-Hz signal. 1MHz/50 = 20000
    	PWM0_2_CMPA_R = duty_cycle1;  
    	PWM0_2_CMPB_R = duty_cycle2;
    	PWM0_2_CTL_R = 0x01; // Enable M0 and down mode
    	PWM0_ENABLE_R = 0x30; // Enable M0PWM4 & M0PWM5.. Channel 6 (PWM1)
}

void PortF_Init(void){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x20;  	// 1) Enable clock for Port F
	delay = SYSCTL_RCGC2_R;  	// Allow time for clock to stabilize
	GPIO_PORTF_LOCK_R = 0x4C4F434B; // 2) Unlock Port F (only necessary for PF0)
	GPIO_PORTF_CR_R |= 0x1E; 	// 3) Allow changes to PF3, PF2
	GPIO_PORTF_AMSEL_R &= ~0x1E; // 4) Disable analog function on PF3, PF2
	GPIO_PORTF_PCTL_R &= ~0x000FFFF0; // 5) Clear PCTL bits for PF3, PF2 (GPIO function)
	GPIO_PORTF_DIR_R |= 0x1E;	// 6) Set PF3, PF2 as output
	GPIO_PORTF_AFSEL_R &= ~0x1E; // 7) Disable alternate functions on PF3, PF2
	GPIO_PORTF_DEN_R |= 0x1E;	// 8) Enable digital function for PF3, PF2
}
void distanceInit(void){ // PF0 = Eco, PF1 = Trigger
    	SYSCTL_RCGC2_R |= 0x02;
    	//GPIO_PORTB_DIR_R &= ~0x4; // Eco/input/capture pin -- PB6
    	GPIO_PORTB_DIR_R |= 0x8; // trigger pin -- PB7
    	GPIO_PORTB_DEN_R |= 0xC; // PB6, PB7
    	GPIO_PORTB_AFSEL_R |= 0x4; // PB6.. Eco pin
    	//GPIO_PORTB_PCTL_R &= ~0x000F00;
    	GPIO_PORTB_PCTL_R |= 0x000700;
    	delay_us(10);
}
//----------------------Servo----------------------
void servo_Init(void) { //initialized F0 for servo
	SYSCTL_RCGCGPIO_R |= 0x20;   // enable clock for Port F
   // while ((SYSCTL_PRGPIO_R & 0x20) == 0); // had to do this to make it work, it needs to wait for portF to be ready *UPDATE FIXED PROBLEM no need to wait for PortF to be ready
    
	GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO PF0
	GPIO_PORTF_CR_R |= 0x01;    	// allows changes to gpio PF0
    
	GPIO_PORTF_AFSEL_R &= ~0x01;  // disables the alternate function on PF0
	GPIO_PORTF_PCTL_R &= ~0x0000000F; // clears PCTL bits for PF0 (disables the T0CCP0)
    
	GPIO_PORTF_DIR_R |= 0x01;	// sets PF0 as output
	GPIO_PORTF_DEN_R |= 0x01;	// enables digital function on PF0
	GPIO_PORTF_AMSEL_R &= ~0x01; // disables analog on PF0
}
void servoMotor(int pwm) {
	for (int i = 0; i < 15; i++) {
    	GPIO_PORTF_DATA_R |= 0x01; // Set PF0 HIGH
    	servo_delay_us(pwm); 	 
    	GPIO_PORTF_DATA_R &= ~0x01; // Set PF0 LOW
    	servo_delay_us(20000 - pwm);
	}
}
void servo_delay_us(int n) {
	Timer0A_usd();
	for (int i = 0; i < n; i++) {
    	while ((TIMER0_RIS_R & 0x01) == 0); // Wait for TimerA timeout flag
    	TIMER0_ICR_R = 0x01; // Clear the TimerA timeout flag
	}
}
//------------Timers/Delays init-----------------
void Timer0A_usd(void) {
	SYSCTL_RCGCTIMER_R |= 0x01; // Enable clock to Timer0
	TIMER0_CTL_R = 0x00; // Disable Timer before initialization
	TIMER0_CFG_R = 0x00; // 32-bit option
	TIMER0_TAMR_R = 0x02; // Periodic mode and down-counter
	TIMER0_TAILR_R = 50 - 1; // Timer A interval load value register
	TIMER0_ICR_R = 0x1; // Clear the TimerA timeout flag
	TIMER0_CTL_R |= 0x01; // Enable Timer A after initialization
}
void Timer1A_1sd(void) {
	SYSCTL_RCGCTIMER_R |= 0x02;  // Enable clock to Timer1
	TIMER1_CTL_R = 0x00;     	// Disable Timer before initialization
	TIMER1_CFG_R = 0x04;     	// 16-bit option
	TIMER1_TAMR_R = 0x02;    	// Periodic mode and down-counter
	TIMER1_TAILR_R = 50 - 1; 	// Timer A interval load value register
	TIMER1_ICR_R = 0x1;      	// Clear the TimerA timeout flag
	TIMER1_CTL_R |= 0x01;    	// Enable Timer A after initialization
	TIMER1_TAPR_R = 3-1;     	// Prescalar value
}
void Timer2A_usd(void){
 	SYSCTL_RCGCTIMER_R |= 0x04; 	/* enable clock to Timer0, Timer1, Timer2 */
    
    	TIMER2_CTL_R = 0x00;        	/* disable Timer before initialization */
	TIMER2_CFG_R = 0x00;     	/* 32-bit option */
	TIMER2_TAMR_R = 0x02;    	/* periodic mode and down-counter */
	TIMER2_TAILR_R = 50 - 1;  /* Timer A interval load value register */
	TIMER2_ICR_R = 0x1;      	/* clear the TimerA timeout flag*/
	TIMER2_CTL_R |= 0x01;    	/* enable Timer A after initialization */
    	TIMER2_TAPR_R = 1-1;                	// Prescalar value.. Can extend the cycle time max 256 times
}
void timer3A_captureTimer(void){
    	SYSCTL_RCGCTIMER_R |= 0x08;
    	TIMER3_CTL_R &= ~0x01; // disabale first
    	TIMER3_CFG_R = 0x04;  // 32- bits
    	TIMER3_TAMR_R = 0x17; // Up counter, cacpture mode, edge time
	//	TIMER0_ICR_R = 0x01;
    	TIMER3_CTL_R = 0x0C; // Enable TimerA, capture botthe edges
    	TIMER3_CTL_R |= 0x01;
    	//TIMER0_IMR_R = 0x06;
    	//TIMER0_ICR_R = 0x0;
    	delay_us(10);
}
void Timer4A_msd(void) {
	SYSCTL_RCGCTIMER_R |= 0x10;  //Enable clock to Timer4
	TIMER4_CTL_R = 0x00;     	// Disable Timer4 before initialization
	TIMER4_CFG_R = 0x00;     	// 32-bit option
	TIMER4_TAMR_R = 0x02;    	// Periodic mode and down-counter
	TIMER4_TAILR_R = 50000 - 1;  // Set the Timer A interval load value (e.g., 10sec)
	TIMER4_ICR_R = 0x1;      	// Clear the TimerA timeout flag
	TIMER4_IMR_R |= 0x1;     	// Enable the interrupt for TimerA timeout
	TIMER4_TAPR_R = 1 - 1;   	// Prescalar value (no prescaling)
	// Can extend the cycle time max 256 times using the prescaler
	TIMER4_CTL_R |= 0x021;   	// Enable Timer A after initialization
    
	// NVIC_EN0_R |= 0x1000000;	// Enable Timer4A interrupt (uncomment for interrupt setup)
	// NVIC_PRI4_R = (NVIC_PRI4_R & 0x00FFFFFF) | 0x80000000; // Set interrupt priority
}
void Timer5A_init(void) {
    	// 250,000,000 - 1 for 5 seconds
    	// 2,500,000 - 1 for 50 ms
	SYSCTL_RCGCTIMER_R |= 0x20;  // Enable clock for Timer5
	TIMER5_CTL_R = 0x00;     	// Disable Timer5 before initialization
	TIMER5_CFG_R = 0x00;     	// 32-bit mode
	TIMER5_TAMR_R = 0x02;    	// Periodic mode
	TIMER5_TAILR_R = 2500000 - 1 ;   // Set timeout at every 50 ms
	TIMER5_ICR_R = 0x01;     	// Clear timeout flag
	TIMER5_IMR_R |= 0x01;    	// Enable Timer5A timeout interrupt
	NVIC_EN2_R |= (1 << (92 - 64)); // Enable interrupt for Timer5A (IRQ 92)
	TIMER5_CTL_R |= 0x01;    	// Enable Timer5A
}

int isBluetoothDataAvailable(void) {
	return !(UART1_FR_R & (1<<4)); // If bit is 0, data is available
}
void TIMER5A_Handler(void) {
	if (TIMER5_MIS_R & 0x01) {  
    	TIMER5_ICR_R = 0x01;  // Clear the interrupt flag
   	 
    	if (isBluetoothDataAvailable()) {  // Check if data is available
        	op = blueTooth_Read();  // Read Bluetooth input
       	 
        	switch (op) {
            	case 'S':    
                	write_string("Temporary Stop");
                                    	stopDrive();
                	delay_ms4A(2000);  
                	break;

            	case 'D':
                	write_string("Setting Sensor Straight \n");
                	servoMotor(1600); // Align sensor forward
                	delay_ms4A(100);
                	write_string("Checking Distance \n");
                	findDist();
                	sprintf(snum, "Distance is %d", dist);
                	write_string(snum);
                	break;

            	case 'T':
                	write_string("Checking Temp \n");
                	get_temperature();
                	sprintf(snum, "Temp is %d", temperatureF);
                	write_string(snum);
                	break;

            	default:
                	write_string("No valid operation.\n");
                	break;
        	}
    	}
	}
}

//---------------------Delays-------------------------------
void timer1A_delay(int ttime){  // x times 1-sec delay
	//Timer1A_1sd();
    	int i;
    
	for(i = 0; i < ttime; i++) {
        	while ((TIMER1_RIS_R & 0x01) == 0);  	/* wait for TimerA timeout flag */
    	TIMER1_ICR_R = 0x01;  	/* clear the TimerA timeout flag */
	}
}
void delay_ms(int n) {
	Timer1A_1sd();
	int i;
    
	for(i = 0; i < n; i++) {
    	while ((TIMER1_RIS_R & 0x01) == 0);  /* wait for TimerA timeout flag */
    	TIMER1_ICR_R = 0x01;            	/* clear the TimerA timeout flag */
	}
}
void delay_us(int n){
    	Timer2A_usd();
    	int i;
    
	for(i = 0; i < n; i++) {
        	while ((TIMER2_RIS_R & 0x01) == 0);  	/* wait for TimerA timeout flag */
    	TIMER2_ICR_R = 0x01;  	/* clear the TimerA timeout flag */
	}
}
void delay_ms4A(int n) {
	Timer4A_msd();  // Initialize Timer 4A
	int i;
	for (i = 0; i < n; i++) {
    	// Wait for Timer 4A timeout flag
    	while ((TIMER4_RIS_R & 0x1) == 0);  // Check if Timer4A has timed out
    	TIMER4_ICR_R = 0x1;  // Clear the Timer4A timeout flag
	}
}
unsigned long int SW1;
unsigned long int SW2;
//-----------------------Functions------------------
void findDist(void){
    	unsigned long lastEdge =0, curEdge =0;
    	GPIO_PORTB_DATA_R &= ~0x8; // Sending 0 to trigger pin PF1
    	delay_us(10);
    	GPIO_PORTB_DATA_R |= 0x8; // Triggering US signal PF1
    	delay_us(10);
    	GPIO_PORTB_DATA_R &= ~0x8; // Again send 0 to PB4
    	//delay_us(5);

	//	while(1){
            	TIMER3_ICR_R = 0x04; // Timer A Capture Mode Event Raw Interrupt
    	//delay_us(1);
            	while((TIMER3_RIS_R & 4) == 0) ; // wait till captured

            	if((GPIO_PORTB_DATA_R & 0x4)== 0x4){ // Echo high
                    	lastEdge = TIMER3_TAR_R & 0xFFFFFF;
                    	delay_us(10);
                    	while((GPIO_PORTB_DATA_R & 0x4)== 0x00);
                    	TIMER3_ICR_R = 0x04;    
                	//GPIO_PORTF_DATA_R = 0x04;

                    	while((TIMER3_RIS_R & 4)==0); //
                    	curEdge = TIMER3_TAR_R & 0xFFFFFF;
                	delay_us(1000);
                    	if(curEdge < lastEdge){
                    	//    	curEdge = curEdge + 0xFFFFFF; // 1-overflow
                    	}
                    	dist = ((curEdge - lastEdge)*34)/100000;
                    	//GPIO_PORTF_DATA_R = 0x02;    

            	}
            	delay_ms(500);
	//	}
}

//-------------------object avoidance--------------------

//-------------------MAIN------------------------------------------
int main(){
	int duty_up = duty_cycle1, duty_up2 = duty_cycle1; //PWM0_0_LOAD_R-1;

	//motor,timer and port initializations
	PortF_Init();
	//portA_init();
	Timer0A_usd();
	Timer1A_1sd();
	Timer2A_usd();
	Timer4A_msd();
	timer3A_captureTimer();
	distanceInit();
	PWM_Init();
	servo_Init();
	ADC_Init();
	//BT Initializations
	Bluetooth_init();
  HC05_Init();
	LCD_init();
    
	Timer5A_init();
    
	write_string("Hello! 123\n");
	delay_ms4A(200);
	write_string("Enter S to stop, T for temp, and D for distance \n");
	LCD_Cmd(0x01);  // clear display
	LCD_Cmd(0x80);	// first row
	LCD_Cmd(0x0C);
	delay_ms4A(1000);
	frontDrive();
	// don't forget to add delays to allow components time to work
	// 600 = right90
	// 1000 = right45
	// 1600 = straight
	// 2100 = left45
	// 2500 = left90
	// display direction, turn, distance
    
	while (1) {
        	servoMotor(1600);
        	delay_ms4A(200);
        	findDist();    
        	if (dist > 30) {
                	frontDrive();
                	delay_ms4A(200);
                	// display front and distance
                	LCD_Cmd(0x01);  // clear display
                	LCD_Cmd(0x80);	// first row
                	LCD_string("Driving Straight");
                	LCD_Cmd(0xC0);
                	sprintf(snum, "Distance: %d cm", dist);
                	LCD_string(snum);
        	} else {
                	stopDrive();
                	delay_ms4A(200);

                	// turn right45
                	LCD_Cmd(0x01);  // clear display
                	LCD_Cmd(0x80);	// first row
                	LCD_string("Check Right45");
           	 
                	servoMotor(1000);
                	findDist();

                	LCD_Cmd(0xC0);
                	sprintf(snum, "Distance: %d cm", dist);
                	LCD_string(snum);   	 
                	delay_ms4A(500);
                	if (dist > 30) {
                        	LCD_Cmd(0x01);  // clear display
                        	LCD_Cmd(0x80);	// first row
                        	LCD_string("Right45 Clear");
                        	right45();
                        	delay_ms4A(200);
                        	continue;
                	}
               	 
                	// turn left45
                	LCD_Cmd(0x01);  // clear display
                	LCD_Cmd(0x80);	// first row
                	LCD_string("Check Left45");
                	servoMotor(2100);
                	findDist();
                	if (dist > 30) {
                        	LCD_Cmd(0x01);  // clear display
                        	LCD_Cmd(0x80);	// first row
                        	LCD_string("Left45 Clear");
                        	left45();
                        	delay_ms4A(200);
                        	continue;
                	}
               	 
                	// turn right90
                	LCD_Cmd(0x01);  // clear display
                	LCD_Cmd(0x80);	// first row
                	LCD_string("Check Right90");
                	servoMotor(600);
                	findDist();
                	if (dist > 30) {
                        	LCD_Cmd(0x01);  // clear display
                        	LCD_Cmd(0x80);	// first row
                        	LCD_string("Right90 Clear");
                        	right90();
                        	delay_ms4A(200);
                        	continue;
                	}            	 
               	 
                	// turn left90
                	LCD_Cmd(0x01);  // clear display
                	LCD_Cmd(0x80);	// first row
                	LCD_string("Check left90");               	 
                	servoMotor(2500);
                	findDist();
                	if (dist > 30) {
                        	LCD_Cmd(0x01);  // clear display
                        	LCD_Cmd(0x80);	// first row
                        	LCD_string("Left90 Clear");
                        	left90();
                        	delay_ms4A(200);
                        	continue;
                	}
                	LCD_Cmd(0x01);  // clear display
                	LCD_Cmd(0x80);	// first row
                	LCD_string("Blocked");
                	delay_ms4A(200);
                	stopDrive();
        	}
	}

}

void DisableInterrupts(void)
{
	__asm ("	CPSID  I\n");
}

/*********** EnableInterrupts ***************
*
* emable interrupts
*
* inputs:  none
* outputs: none
*/

void EnableInterrupts(void)
{
	__asm  ("	CPSIE  I\n");
}

/*********** WaitForInterrupt ************************
*
* go to low power mode while waiting for the next interrupt
*
* inputs:  none
* outputs: none
*/

void WaitForInterrupt(void)
{
	__asm  ("	WFI\n");
//GPIOPortF_Handler();
}







