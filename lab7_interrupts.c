#include "msp.h"
#include <stdint.h>
#include "CortexM.h"

/****************************************
* CLOCK Init
***************************************/

uint32_t ClockFrequency = 48000000;      // cycles/second
int32_t Prewait = 0;                    // loops between BSP_Clock_InitFastest() called and PCM idle (expect 0)
uint32_t CPMwait = 0;                   // loops between Power Active Mode Request and Current Power Mode matching requested mode (expect small)
uint32_t Postwait = 0;                  // loops between Current Power Mode matching requested mode and PCM module idle (expect about 0)
uint32_t IFlags = 0;                    // non-zero if transition is invalid
uint32_t Crystalstable = 0;             // loops before the crystal stabilizes (expect small)
uint32_t PWM = 3000;

void Clock_Init48MHz(void){
    // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
    while(PCM->CTL1&0x00000100){
        //  while(PCMCTL1&0x00000100){
        Prewait = Prewait + 1;
        if(Prewait >= 100000){
            return;                           // time out error
        }
    }

    // request power active mode LDO VCORE1 to support the 48 MHz frequency
    PCM->CTL0 = (PCM->CTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
    0x695A0000 |                // write the proper PCM key to unlock write access
    0x00000001;                 // request power active mode LDO VCORE1

    // check if the transition is invalid (see Figure 7-3 on p344 of datasheet)
    if(PCM->IFG&0x00000004){
        IFlags = PCM->IFG;                    // bit 2 set on active mode transition invalid; bits 1-0 are for LPM-related errors; bit 6 is for DC-DC-related error
        PCM->CLRIFG = 0x00000004;             // clear the transition invalid flag
        // to do: look at CPM bit field in PCMCTL0, figure out what mode you're in, and step through the chart to transition to the mode you want
        // or be lazy and do nothing; this should work out of reset at least, but it WILL NOT work if Clock_Int32kHz() or Clock_InitLowPower() has been called
        return;
    }

    // wait for the CPM (Current Power Mode) bit field to reflect a change to active mode LDO VCORE1
    while((PCM->CTL0&0x00003F00) != 0x00000100){
        CPMwait = CPMwait + 1;
        if(CPMwait >= 500000){
            return;                           // time out error
        }
    }

    // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
    while(PCM->CTL1&0x00000100){
        Postwait = Postwait + 1;
        if(Postwait >= 100000){
            return;                           // time out error
        }
    }

    // initialize PJ.3 and PJ.2 and make them HFXT (PJ.3 built-in 48 MHz crystal out; PJ.2 built-in 48 MHz crystal in)
    PJ->SEL0 |= 0x0C;
    PJ->SEL1 &= ~0x0C;                    // configure built-in 48 MHz crystal for HFXT operation
    CS->KEY = 0x695A;                     // unlock CS module for register access
    CS->CTL2 = (CS->CTL2&~0x00700000) |   // clear HFXTFREQ bit field
    0x00600000 |                 // configure for 48 MHz external crystal
    0x00010000 |                 // HFXT oscillator drive selection for crystals >4 MHz
    0x01000000;                  // enable HFXT
    CS->CTL2 &= ~0x02000000;              // disable high-frequency crystal bypass
    // wait for the HFXT clock to stabilize
    while(CS->IFG&0x00000002){
        CS->CLRIFG = 0x00000002;              // clear the HFXT oscillator interrupt flag
        Crystalstable = Crystalstable + 1;
        if(Crystalstable > 100000){
            return;                           // time out error
        }
    }

    // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 0
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL&~0x0000F000)|FLCTL_BANK0_RDCTL_WAIT_2;
    // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 1
    FLCTL->BANK1_RDCTL = (FLCTL->BANK1_RDCTL&~0x0000F000)|FLCTL_BANK1_RDCTL_WAIT_2;
    CS->CTL1 = 0x20000000 |               // configure for SMCLK divider /4
    0x00100000 |                 // configure for HSMCLK divider /2
    0x00000200 |                 // configure for ACLK sourced from REFOCLK
    0x00000050 |                 // configure for SMCLK and HSMCLK sourced from HFXTCLK
    0x00000005;                  // configure for MCLK sourced from HFXTCLK
    CS->KEY = 0;                          // lock CS module from unintended access
    ClockFrequency = 48000000;
    //  SubsystemFrequency = 12000000;
}

// delay function which delays about 6*ulCount cycles
// ulCount=8000 => 1ms = (8000 loops)*(6 cycles/loop)*(20.83 ns/cycle)
void delay(unsigned long ulCount){
    __asm (  "pdloop:  subs    r0, #1\n"
        "    bne    pdloop\n");
}


// ------------Clock_Delay1ms------------
// Simple delay function which delays about n milliseconds.
// Inputs: n, number of msec to wait
// Outputs: none
void Clock_Delay1ms(uint32_t n){
    while(n){
        delay(ClockFrequency/9162);   // 1 msec, tuned at 48 MHz
        n--;
    }
}


/****************************************
*  SysTick Timer Init
***************************************/
void SysTick_Init(){
    SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
    SysTick->CTRL = 0x00000007;           // enable SysTick with no interrupts
}

void SysTick_Wait(uint32_t delay){
    SysTick->LOAD = (delay - 1);// count down to zero
    SysTick->VAL = 0;          // any write to CVR clears it and COUNTFLAG in CSR
    while(( SysTick->CTRL&0x00010000) == 0){};
}

// Time delay using busy wait.
// assumes 48 MHz bus clock
void SysTick_Wait10ms(uint32_t delay){
    uint32_t i;
    for(i=0; i<delay; i++){
        SysTick_Wait(480000);  // wait 10ms (assumes 48 MHz clock)
    }
}

// ------------Clock_Delay1us------------
// Simple delay function which delays about n microseconds.
// Inputs: n, number of us to wait
// Outputs: none
void Clock_Delay1us(uint32_t n){
  n = (382*n)/100;; // 1 us, tuned at 48 MHz
  while(n){
    n--;
  }
}

#define LED (*((volatile uint8_t *)(0x42098040)))
void SysTick_Handler(void)
{
    //    P1->OUT = P1->OUT ^ 0x01;// Toggle the LED value
    P1->OUT = LED ^= 0x01; // toggle P1.0
}

/*************************************
*  Launchpad init
************************************/
void LaunchPad_Init(void){
    P1->SEL0 &= ~0x13;
    P1->SEL1 &= ~0x13;    // 1) configure P1.4 and P1.1 as GPIO
    P1->DIR &= ~0x12;     // 2) make P1.4 and P1.1 in
    P1->DIR |= 0x01;      //    make P1.0 out
    P1->REN |= 0x12;      // 3) enable pull resistors on P1.4 and P1.1
    P1->OUT |= 0x12;      //    P1.4 and P1.1 are pull-up
    P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;    // 1) configure P2.2-P2.0 as GPIO
    P2->DIR |= 0x07;      // 2) make P2.2-P2.0 out
    P2->DS |= 0x07;       // 3) activate increased drive strength
    P2->OUT &= ~0x07;     //    all LEDs off
}

void Motor_InitSimple(void){
    // Initializes the 6 GPIO lines and puts driver to sleep
    // Returns right away
    // initialize P1.6 and P1.7 and make them outputs
    P1->SEL0 &= ~0xC0;
    P1->SEL1 &= ~0xC0;
    P1->DIR |= 0xC0;
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;

    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;

    P1->OUT &= ~0xC0;
    P2->OUT &= ~0xC0;   // off
    P3->OUT &= ~0xC0;   // low current sleep mode
}

void Motor_StopSimple(void){
    // Stops both motors, puts driver to sleep
    // Returns right away
    P1->OUT &= ~0xC0;
    P2->OUT &= ~0xC0;   // off
    P3->OUT &= ~0xC0;   // low current sleep mode
}

void Motor_RotateLeft(uint32_t duty, uint32_t time){
    // Drives both motors forward at duty (100 to 9900)
    // Runs for time duration (units=10ms), and then stops
    // Stop the motors and return if any bumper switch is active
    // Returns after time*10ms or if a bumper switch is hit

    uint32_t i = 0;

    // write this as part of Lab 5
    //P1->OUT &= ~0xC0; // set direction forward using P1.6 and P1.7
    P1->OUT &= ~0x40;  // set direction forward using P1.6 and P1.7
    P1->OUT &= ~0x80;
    P3->OUT |= 0xC0; // enable both motors P3.6 and P3.7
    uint16_t duty1=10000-duty;

    for(i; i < time; i=i+1){
        P2->OUT |= 0xC0; // Turn on motors for P2.6 and P2.7
        Clock_Delay1us(duty);
        P2->OUT &= ~0xC0; // Turn motors off for P2.6 and P2.7
        Clock_Delay1us(duty1);
    }
}

void Motor_RotateRight(uint32_t duty, uint32_t time){
    // Drives both motors forward at duty (100 to 9900)
    // Runs for time duration (units=10ms), and then stops
    // Stop the motors and return if any bumper switch is active
    // Returns after time*10ms or if a bumper switch is hit

    uint32_t i = 0;

    // write this as part of Lab 5
    //P1->OUT &= ~0xC0; // set direction forward using P1.6 and P1.7
    P1->OUT |= 0x40;  // set direction forward using P1.6 and P1.7
    P1->OUT |= 0x80;
    P3->OUT |= 0xC0; // enable both motors P3.6 and P3.7
    uint16_t duty1=10000-duty;

    for(i; i < time; i=i+1){
        P2->OUT |= 0xC0; // Turn on motors for P2.6 and P2.7
        Clock_Delay1us(duty);
        P2->OUT &= ~0xC0; // Turn motors off for P2.6 and P2.7
        Clock_Delay1us(duty1);
    }
}

void Motor_ForwardSimple(uint32_t duty, uint32_t time){
    // Drives both motors forward at duty (100 to 9900)
    // Runs for time duration (units=10ms), and then stops
    // Stop the motors and return if any bumper switch is active
    // Returns after time*10ms or if a bumper switch is hit

    uint32_t i = 0;
    // write this as part of Lab 5
    //P1->OUT &= ~0xC0; // set direction forward using P1.6 and P1.7
    P1->OUT &= ~0x40;  // set direction forward using P1.6 and P1.7
    P1->OUT |= 0x80;
    P3->OUT |= 0xC0; // enable both motors P3.6 and P3.7
    uint16_t duty1=10000-duty;

    for(i; i < time; i=i+1){
        P2->OUT |= 0xC0; // Turn on motors for P2.6 and P2.7
        Clock_Delay1us(duty);
        P2->OUT &= ~0xC0; // Turn motors off for P2.6 and P2.7
        Clock_Delay1us(duty1);
    }
}
void Motor_BackwardSimple(uint32_t duty, uint32_t time){
    // Drives both motors backward at duty (100 to 9900)
    // Runs for time duration (units=10ms), and then stops
    // Runs even if any bumper switch is active
    // Returns after time*10ms

    uint32_t i = 0;

    // write this as part of Lab 5
    P1->OUT |= 0x40;  // set direction forward using P1.6 and P1.7
    P1->OUT &= ~0x80; // enable both motors P3.6 and P3.7
    P3->OUT |= 0xC0;


    uint16_t duty1=10000-duty;

    for(i; i < time; i=i+1){

        P2->OUT |= 0xC0; // Turn on motors for P2.6 and P2.7
        Clock_Delay1us(duty);
        P2->OUT &= ~0xC0; // Turn motors off for P2.6 and P2.7
        Clock_Delay1us(duty1);
    }
}
void Motor_LeftSimple(uint32_t duty, uint32_t time){
    // Drives just the left motor forward at duty (100 to 9900)
    // Right motor is stopped (sleeping)
    // Runs for time duration (units=10ms), and then stops
    // Stop the motor and return if any bumper switch is active
    // Returns after time*10ms or if a bumper switch is hit

    uint32_t i = 0;

    // write this as part of Lab 5
    P1->OUT &= ~0x40; // set direction forward using P1.7
    P3->OUT |= 0x40; // enable both motors P3.7
    uint16_t duty1=10000-duty;

    for(i; i < time; i=i+1){
        Clock_Delay1us(duty);
        P2->OUT |= 0x40; // Turn on motors for P2.7
        Clock_Delay1us(duty1);
        P2->OUT &= ~0x40; // Turn motors off for P2.7
    }
}
void Motor_RightSimple(uint32_t duty, uint32_t time){
    // Drives just the right motor forward at duty (100 to 9900)
    // Left motor is stopped (sleeping)
    // Runs for time duration (units=10ms), and then stops
    // Stop the motor and return if any bumper switch is active
    // Returns after time*10ms or if a bumper switch is hit

    uint32_t i = 0;
    // write this as part of Lab 5
    P1->OUT |= 0x80; // set direction forward using P1.6
    P3->OUT |= 0x80; // enable both motors P3.6
    uint16_t duty1=10000-duty;

    for(i; i < time; i=i+1){
        Clock_Delay1us(duty);
        P2->OUT |= 0x80; // Turn on motors for P2.6
        Clock_Delay1us(duty1);
        P2->OUT &= ~0x80; // Turn motors off for P2.6
    }
}

// P6.1, 6.4, and 6.5 as turn signal and brake LEDs
// Brake light - 6.1
// Left Turn - 6.4
// Right Turn - 6.5

void LED_Init(){
    P6->SEL0 &= ~0x32;
    P6->SEL1 &= ~0x32;
    P6->DIR |= 0x32;
    P4->REN |= 0x32; // enable pull resistor

    P6->OUT &= ~0x32; // Turn off LEDS
}

// Enable left turn signal
void turnLeftOn(){
    P6->OUT |= 0x10;
}

// Disable left turn signal
void turnLeftOff(){
    P6->OUT &= ~0x10;
}


// Enable right turn signal
void turnRightOn(){
    P6->OUT |= 0x20;
}

// Disable right turn signal
void turnRightOff(){
    P6->OUT &= ~0x20;
}

// Turn brake light on
void brakeLightOn(){
    P6->OUT |= 0x02;
}

// Turn brake light off
void brakeLightOff(){
    P6->OUT &= ~0x02;
}

// Enable all LEDs
void lightAll(){
    P6->OUT |= 0x32;
    SysTick_Wait10ms(100);
    P6->OUT &= ~0x32;
    SysTick_Wait10ms(100);
}

// Turn robot around 180
void turn180(){
    brakeLightOn();


    int16_t position = 99; // Defined value for position
    int16_t reading = 99; // Reading from sensor to sensor integration
    Motor_BackwardSimple(2000, 15);
    Motor_RotateRight(PWM, 5);

    while(position!= 0){
        Motor_RotateRight(PWM, 3);
        reading = Reflectance_Read(250);
        position = Reflectance_Position(reading);
//        if(position == 42){
//            return 42;
//        }
    }


    //Motor_BackwardSimple(PWM, 10);
//    Motor_StopSimple();
//    Motor_RotateRight(PWM, 150);
   Motor_StopSimple();
   brakeLightOff();
   //return 0;

}

// Pivot robot 90 degrees to the right
void turn90Right(){
    Motor_StopSimple();
    Motor_ForwardSimple(PWM, 3);
    turnRightOn();
    Motor_RotateRight(PWM, 20);
    Motor_StopSimple();
    turnRightOff();
}

// Pivot robot 90 degrees to the left
void turn90Left(){
    Motor_StopSimple();
    Motor_ForwardSimple(PWM, 3);
    turnLeftOn();
    Motor_RotateLeft(PWM, 20);
    Motor_StopSimple();
    turnLeftOff();
}


//------------LaunchPad_Input------------
// Input from Switches
// Input: none
// Output: 0x00 none
//         0x01 Button1
//         0x02 Button2
//         0x03 both Button1 and Button2
uint8_t LaunchPad_Input(void){
    return ((((~(P1->IN))&0x10)>>3)|(((~(P1->IN))&0x02)>>1));   // read P1.4,P1.1 inputs
}

//------------LaunchPad_Output------------
// Output to LaunchPad LEDs
// Input: 0 off, bit0=red,bit1=green,bit2=blue
// Output: none
void LaunchPad_Output(uint8_t data){  // write three outputs bits of P2
    P2->OUT = (P2->OUT&0xF8)|data;
}

// Driver test
void Pause(void){
    while(LaunchPad_Input()==0);  // wait for touch
    while(LaunchPad_Input());     // wait for release
}

volatile uint32_t FallingEdges1;

// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
// Interrupt on falling edge (on touch)
void BumpInt_Init(void(*task)(uint8_t)){
    // write this as part of Lab 7
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED; // Configure P4.0, 4.2, 4.3, 4.5, 4.6, 4.7 GPIO
    P4->DIR &= ~0xED; // make input
    P4->REN |= 0xED; // enable pull resistor
    P4->OUT |= 0xED; // pull-up
    P4->IES |= 0xED;
    P4->IFG &= ~0xED;
    P4->IE |= 0xED;
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF) | 0x00400000;
    NVIC->ISER[1] = 0x00000040;

    /*P2->SEL0 &= ~0x07;
    P2->SEL1 &= ~0x07;    // 1) configure P2.2-P2.0 as GPIO
    P2->DIR |= 0x07;      // 2) make P2.2-P2.0 out
    P2->DS |= 0x07;       // 3) activate increased drive strength
    P2->OUT &= ~0x07;     //    all LEDs off*/
}
// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 Bump5
// bit 4 Bump4
// bit 3 Bump3
// bit 2 Bump2
// bit 1 Bump1
// bit 0 Bump0
uint8_t Bump_Read(void){
    // write this as part of Lab 7
    uint8_t read = P4->IN&0xED;
    uint8_t top3 = read&0xE0;
    uint8_t bits2 = read&0x0C;
    uint8_t out = (top3>>2)|(bits2>>1)|(read&0x01);
    return out; // replace this line// replace this line
}
// we do not care about critical section/race conditions
// triggered on touch, falling edge
// write this as part of Lab 7
void PORT4_IRQHandler(void){
    printf("IRQHANDLER \n");
    P4->IFG &= ~0x08;
    uint32_t pin = P4->IV;
    HandleCollision(pin);
}

/***************************
* main.c
***************************/
uint8_t CollisionData, CollisionFlag;  // mailbox
uint8_t CollisionData, CollisionFlag;  // mailbox
void HandleCollision(uint32_t bumpSensor){
    CollisionData = bumpSensor;
    CollisionFlag = 1;
    turn90Right();
    turn90Right();
}

int main(void){  // test of interrupt-driven bump interface
    int16_t position = 0; // Defined value for position
    int16_t reading = 0; // Reading from sensor to sensor integration
    uint8_t rightleft = 0; // left = 0, right = 1
    SysTick_Init();
    Clock_Init48MHz();   // 48 MHz clock; 12 MHz Timer A clockß
    CollisionFlag = 0;
    LaunchPad_Init();
    LED_Init(); // Enable LEDs
    Reflectance_Init();
    Motor_InitSimple();        // activate Lab 5 - PWM
    BumpInt_Init(&HandleCollision); // Enable interupt bumpers
    DisableInterrupts();
    EnableInterrupts(); // Set interrupts to enabled
    P5->OUT |= 0x08;
    uint16_t turnCount = 0;

    while(1){

        // Read reflectance from IR sensor
        reading = Reflectance_Read(500);
        position = Reflectance_Position(reading);
        printf("%d \n", position);

        // Midde position scenario, move forward
        if(position == 0){
            //printf("Go forward \n");
            Motor_ForwardSimple(PWM, 5);
        }


        // ID 1 indicates complete white space. Turn around completely. If we've turned around 4 times, go forward a bit.
        // This is infinite loop prevention
        if(position == 1){
            turn180();
            if(turnCount >= 4){
                Motor_ForwardSimple(PWM, 10);
            }
            continue;
        } else{
            turnCount = 0;
        }

        // ID 42 means treasure found, we stop and end loop
        if(position == 42){
            Motor_StopSimple();
            break;
        }

        // Position ID 2 indicates a crossroad. If we've turned right before, turn left. Vice versa.
        if(position == 2){
            if(rightleft == 0){
                turn90Right();
            }else{
                turn90Left();
            }
            continue;
        }

        // Position ID 3 indicates right turn intersection. Turn right
        if(position == 3){
            turn90Right();
            rightleft = 0;
            continue;
        }

        // Position ID 4 indicates left turn intersection. Turn left.
        if(position == 4){
            turn90Left();
            rightleft = 1;
            continue;
        }

        // Account for offset from center and make minor adjustments.
        if(position < 0 && position >= -332){
            Motor_LeftSimple(4000, 1);
        }

        if(position > 0 && position <= 332 ){
            Motor_RightSimple(4000, 1);
        }
    }

    // Flash treasure lights. Success!
    uint8_t i = 0;

    for(i; i < 3; i = i+1){
        lightAll();
    }
}
