# Example-Code-embedded

I made this Github to post examples of my embedded c code for a motor controls project.

Notes
/*
* File:   MC Master V1.0.c
* Author: Ramon Meza
*
* Created on August 16, 2017, 5:05 PM
* 
* The purpose of this main code is to take commands from from BOKAM GD5 board 
* to control 3 DC motors. The communication is via I2C and the data sent and 
* received is commands and Motor Feedback.
* Two MC Slaves are involved for a total of 3 MC boards. 
* This code is only for the Master MC board.
* The communication between MC Master board and two slaves is SPI.
*  
* This code will be used on L3 Motor control board. Attached on the motor
* control board is the POLOLU(MC33926 Motor Driver Carrier).
*  
* Version #1 Code/Testing
* Compiler CCS PCH For 18F Processor
* 
* UPDATES:
* 11/29/2017 - Motivo requested changes/I2C delays added
* 
* 
* 
*/
//////// These are the fuses for 18F46K22.h found in CCS files inside devices.
//////// Text file found in PICC folder has Fuse definitions.
//////// Fuses: LP,XT,HSH,HSM,ECH,ECH_IO,RC,RC_IO,INTRC_IO,INTRC,ECM,ECM_IO
//////// Fuses: ECL,ECL_IO,NOPLLEN,PLLEN,PRIMARY_SW,PRIMARY_ON,NOFCMEN,FCMEN
//////// Fuses: NOIESO,IESO,PUT,NOPUT,NOBROWNOUT,BROWNOUT_SW,BROWNOUT_NOSL
//////// Fuses: BROWNOUT,BORV29,BORV25,BORV22,BORV19,NOWDT,WDT_NOSLEEP
//////// Fuses: WDT_SW,WDT,WDT1,WDT2,WDT4,WDT8,WDT16,WDT32,WDT64,WDT128
//////// Fuses: WDT256,WDT512,WDT1024,WDT2048,WDT4096,WDT8192,WDT16384
//////// Fuses: WDT32768,CCP2B3,CCP2C1,NOPBADEN,PBADEN,CCP3E0,CCP3B5
//////// Fuses: NOHFOFST,HFOFST,TIMER3B5,TIMER3C0,CCP2C0,CCP2D2,NOMCLR,MCLR
//////// Fuses: NOSTVREN,STVREN,NOLVP,LVP,NOXINST,XINST,DEBUG,NODEBUG
//////// Fuses: PROTECT,NOPROTECT,CPB,NOCPB,CPD,NOCPD,WRT,NOWRT,WRTC,NOWRTC
//////// Fuses: WRTB,NOWRTB,WRTD,NOWRTD,EBTR,NOEBTR,EBTRB,NOEBTRB
////////////////////////////////////////////////////////////////////////////////
//######################### Preprocessor Directives ############################
#include <ZooxMainHeader.h>                               // Include header goes here
#include <18f46k22.h>                                     // Make sure its the correct device
#fuses INTRC_IO,NOWDT,NOPROTECT,NOLVP                     // Fuses choses from .h File of device
#DEVICE ADC = 10s                                         // ADC Preprocessor needed
#use delay(clock= 32 MHZ)                                // Fastest speed Device can run
#use rs232(baud=10417, xmit=PIN_D6, rcv=PIN_D7)           // Using CSS Pre-Processor Directive RS232
#define PWM_PIN PIN_C2                                    // Unmodified board is D1 //C2 ONLY FOR Master PWM
#use pwm(output = PWM_PIN, frequency = 20kHz)             // Using CSS Pre-Processor Directive PWM Function
#use fast_io(a)                                           // Using CSS Pre-Processor Directive fast_io
#use fast_io(c)                                           // Using CSS Pre-Processor Directive fast_io
#use fast_io(d)                                           // Using CSS Pre-Processor Directive fast_io
#use SPI(MASTER,FORCE_HW,SPI1,MODE=0,BITS=16,CLOCK_HIGH=1)// Using CSS Pre-Processor Directive SPI
#define Device_SDA PIN_D1
#define Device_SCL PIN_D0
#use I2C(MASTER,SDA=Device_SDA,SCL=Device_SCL,FAST=100000,FORCE_HW)// Using CSS Pre-Processor Directive I2C
 
#include <ios.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
 
//#################### Initialization of Functions #############################
void PositionControl(void);
void ForceFeedback(void);
signed int16 ConvertTorque (void); // Function Passes two signed bytes
void Setmotors(int16,int16);
void I2C (void);
void send2_i2c(unsigned int8); 
void rcv4_i2c(unsigned int8);  
 
 
//void TorqueCommand (signed int16); // Function Receives a signed two bytes
//void new_main(); //************************************************************* Testing Main
//####################### Global Variables #####################################
////////////////////// Direction Controls //////////////////////////////////////
//int16 data = 0; // Direction to send to slaves
//int16 selection = 0; // Speed for slaves
int16 direction = 0; // Direction upper 4bits of 16bit selection
/////////////////////// Feedback Variables /////////////////////////////////////
int16 FB1 = 0; // Feedback from motor #1
int16 FB2 = 0; // Feedback from motor #2
int16 FB3 = 0; // Feedback from motor #3
signed int16 motor_current = 0; // torque and position of motors 
/////////////////////// CAN Commands ///////////////////////////////////////////
int16 Position_Control = 0xDAA1; // Set angle / Position
int16 Passive_Mode     = 0xEBB1; // Passive Mode
int16 Normal_Report    = 0xCAC1; // ALL 
int16 Set_Torque       = 0XDCC1; // Force Feedback
////////////////////// CAN Variables ///////////////////////////////////////////
int16 can_byte1 = 0; // CAN command  Unsigned for storage
int16 can_byte2 = 0; // CAN position Unsigned for storage
////////////////////// I2C Variables ///////////////////////////////////////////
unsigned int8 send_buffer[4]; 
unsigned int8 rcv_buffer[4];
////////////////////// Miscellaneous Variables /////////////////////////////////
char position_flag = 0;
char torque_flag = 0;
char timer_up = 0;
int16 torque_limit = 1000; //680 -  2 amps roughly // 900 - 4.6 amps roughly
char return_flag = 0;
int16 position = 2048;
 
 
#int_timer1
void timer1_10ms()
{
    timer_up = 1; // 10 ms flag set up for I2C read
}
//########################## M A I N ###########################################
void main(){ 
    //new_main();
    ///////////////////////// I/O  S E T  U P //////////////////////////////////
    // Set Inputs or Outputs. Needed when using "#use fast_io(x)"///////////////
    // Port A         
    //                     7: IN    RA7 OSC1 CLKI                   // NOT USING
    //                    /6: IN    RA6 OSC2 CLK0                   // NOT USING
    //                   / 5: OUT   RA5 SS1 // SPI_CS1
    //                  / /4: IN    RA4 C1OUT SRQ T0CKI             // NOT USING
    //                 / / 3: IN    RA3 AN3 C1IN+ VREF+             // NOT USING
    //                / / /2: IN    RA2 AN2 C2IN+ VREF- DACOUT      // NOT USING
    //               / / / 1: OUT   RA1 Inverse on MC33926 Motor Driver Board 
    //              / / / /0: OUT   RA0 Enable on MC33926 Motor Driver Board
    //             / / / /
    //            76543210
     set_tris_a(0b11011100);
    //output_a   (0b00000000);
    // Port C         
    //                     7: OUT   RC7 LED Negative logic
    //                    /6: IN    RC6 AN18 TX1 CK1                // NOT USING
    //                   / 5: OUT   RC5 SPI_SDO1
    //                  / /4: IN    RC4 SPI_SDI1    
    //                 / / 3: OUT   RC3 SPI_SCK1
    //                / / /2: OUT   RC2 CCP1 P1A T5CKI              // NOT USING     
    //               / / / 1: OUT   RC1 SPI_CS2 
    //              / / / /0: IN    RC0 P2B SOSCO T1CKI T3CKI T3G   // NOT USING
    //             / / / /
    //            76543210
     set_tris_c(0b01010001);
    //output_c   (0b00000000);
    // Port D        
    //                     7: IN    RD7 RX
    //                    /6: OUT   RD6 TX 
    //                   / 5: OUT   RD5 IN1 on MC33926 Motor Driver Board 
    //                  / /4: IN    RD4 AN24 P2D SDO2 // SPI_SDO2   // NOT USING
    //                 / / 3: IN    RD3 AN23 P2C SS2  // SPI_CS2    // NOT USING
    //                / / /2: IN    RD2 AN22 P2B                    // NOT USING    
    //               / / / 1: IN    RD1 I2C_SDA2 // SPI_SDI2 SLAVE  // NOW USING I2C
    //              / / / /0: IN    RD0 I2C_SCL2 // SPI_SCK2 SLAVE  // NOW USING I2C
    //             / / / /
    //            76543210
    set_tris_d (0b10011111);
    output_d   (0b11111111);
    ////////////////////// P I N  S E T T I N G ////////////////////////////////
    output_high(PIN_C7); //This is LED pin Negative Logic
    output_high(PIN_A0); //This is EN pin
    output_low (PIN_A1); //This is INV pin
    output_high(PIN_D5); //This is IN1 pin
    output_high(PIN_A5); //Active high for slave 1
    output_high(PIN_C1); //Active high for slave 2
    //////////////////// I N I T I A L I Z E  ADC //////////////////////////////
    setup_adc(ADC_CLOCK_INTERNAL); //enables the a/d module
    //and sets the clock to internal adc clock
    setup_adc_ports(sAN10); //sets all the adc pins to analog
    set_adc_channel(10); //the next read_adc call will read channel 10
    delay_us(10); //a small delay is required after setting the channel
    /////////////////// I N I T I A B L I Z E  SPI2 ////////////////////////////
    //setup_spi2(SPI_SLAVE | SPI_L_TO_H);
    ////////////////// I N I T I A L I Z E   T I M E R 1 ///////////////////////
    setup_timer_1 (T1_INTERNAL | T1_DIV_BY_8);
    enable_interrupts (INT_TIMER1);
    enable_interrupts (GLOBAL);
    set_timer1(55535); // Starts Timer 1 
    ////////////////////////////////////////////////////////////////////////////
    SetMotors(0,0x1000); // Set all motors off and read all feedbacks
    ////////////////////// MAIN LOOP ///////////////////////////////////////////
    while(TRUE){
        
        if (position_flag == 0 && torque_flag == 0)
            I2C();
        
        if (can_byte1 == Position_Control){ // Set Angle name for Henry
            position_flag = 0;
            //PositionControl(); 1ST unit not to have position control
        }else if(can_byte1 == Set_Torque){ // Name from Henry
            torque_flag = 0;
            ForceFeedback();
        }else if (can_byte1 == Normal_Report || can_byte1 == Passive_Mode){
            position_flag = 0;
            torque_flag = 0;
            SetMotors(0,0x1000); // Set all motors off
        }
        
    }// End of While   
}// E N D   O F   M A I N
//##############################################################################
void ForceFeedback(void){
    
    signed int16 torque_command = 0; //Command value is signed 10bit value sent in two bytes
    int16 direction = 0; // Set Direction
//int16 pwm = 0; // Set PWM
    int16 pwm = 0;
//int16 scaler = 2; // added to make range from 0 - 1000 on the pwm scale
    float scaler = 1.96;
//int16 scaler = 489; // 169 used to scale MAX possible input to 680 roughly 2 amps per motor 
                        // 389 used to scale MAX possible input to 900 roughly 4.6 amps per motor 
    while(can_byte1 == Normal_Report || can_byte1 == Set_Torque ){
       
        // If Torque Command is received then store byte 2 as a signed value
        if ( can_byte1 == Set_Torque){
            torque_command = (signed int16)can_byte2;
        }
        // The following will determine the direction and current to provide motors
        if (torque_command > 0){
            direction = 0x3000; // means right for motor command
//pwm = torque_command + scaler;
            pwm = (float)torque_command * scaler; 
        }else if (torque_command < 0){ 
            direction = 0x2000;// means left for motor command
//pwm = abs(torque_command) + scaler;
            pwm = abs((float)torque_command) * scaler;
        }else{ 
            direction = 0x1000; // means off for motor command
            pwm = 0;
        }
        
        // The following will evaluate if the motors are on in extremes 
        //if (can_byte1 == Set_Torque){
             if (torque_command > 0 && position == 0){
                 return_flag = 1;
                 output_low(PIN_C7); //This is LED pin Negative Logic
             }else if(torque_command < 0 && position == 4095){
                 return_flag = 1;
             }else return_flag = 0;
 
         //}
            
        
        SetMotors(pwm,direction);
        I2C();
   
    }// End of While 
    
    if (can_byte1 == Position_Control)
        position_flag = 1;
}//E N D   O F   F O R C E  F E E D B A C K
//##############################################################################
//###############################################################################################################################################
//SPECIAL FUNCTIONS
//################################################################################################################################################
void SetMotors (int16 torque, int16 dir_control){
    int16 command = 0;
   
    // The following will turn off the motors at extremes sensor positions
    if (return_flag == 0){
        if (position == 0 || position == 4095){
//torque = 0; Taken off per Will from Motivo request
        }
    }
    
    if(position != 0 && position != 4095){
        return_flag = 0;
        output_high(PIN_C7); //This is LED pin Negative Logic
    }
    
//printf("Flag        = %Lx\n\r", return_flag);       // Prints value to screen
    
    // The following limits the current to the motors
    if (torque > torque_limit){
        torque = torque_limit;
    }
    
    //The following will set motor controls for motor #1
    if (dir_control == 0x1000){ // off mode
        torque = 0;
    }else if (dir_control == 0x2000){ // turn right
        output_low(PIN_A1); //Set the pin A0 to High   // This is INV pin
    }else if (dir_control == 0x3000){
        output_high(PIN_A1); //Set the pin A0 to High   // This is INV pin
    }
    pwm_set_duty_percent(torque); //Sets the duty cycle scaled from 0 to 1000 for motor#1
    
    //The following will prepare the data packet for command for slave motors 
    command = torque & 0x0FFF; // Make speed data only 12bit readable
    command = command | dir_control; // Combine upper 4 bits to 12 for 2 bytes of data packet for slaves
    
    //The following will send data packets to slave devices for motor control and receive FB value of motors
    output_low(PIN_A5); // Chip select slave#1 
    FB2 = spi_xfer(command); //write 'command' to SPI1 device the same time you are reading a value
    output_high(PIN_A5); // Chip select slave#1
    output_low(PIN_C1); // Chip select slave#2
    FB3 = spi_xfer(command); //write 'command' to SPI2 device the same time you are reading a value
    output_high(PIN_C1); // Chip select slave#2
    
    // The following will read the FB for motor#1
    FB1 = read_adc(); //Starts the conversion, reads result and stores it in value 10bit
}// E N D   O F   S E T _ M O T OR S 
//################################################################################################################################################
//################################################################################################################################################
void send2_i2c (unsigned int8 address){
    i2c_start ();
    i2c_write (address); // Sends the Device address
    delay_us(30);
    i2c_write (send_buffer[0]); // Sends LSB of torque data motor #1
    delay_us(20);
    i2c_write (send_buffer[1]); // Sends MSN of torque data motor #2 
    delay_us(20);
    i2c_stop (); // Sends stop
}
//################################################################################################################################################
//################################################################################################################################################
void rcv4_i2c (unsigned int8 address){
   i2c_start ();
   i2c_write (address + 1);
   delay_us (30);// before 30  
   rcv_buffer[0] = i2c_read ();
   delay_us (20);// 20 before
   rcv_buffer[1] = i2c_read ();
   delay_us (20);// 20 before
   rcv_buffer[2] = i2c_read ();
   delay_us (20);// 20 before
   rcv_buffer[3] = i2c_read (0);
   delay_us (20);// 20 before
   i2c_stop ();
}
//################################################################################################################################################
//################################################################################################################################################
signed int16 ConvertTorque (void){  // Function Converts FB to -511 to 511 scale 
    // To scale FB up to 4.6 amps... roughly 890-900 PWM ... MAX FB 70
   float f1 = 0;
   float f2 = 0;
   float f3 = 0;
   float average_torque = 0;
   float scaler = 5.3; // max 676 scale// 1.323 x 4
   
   signed int16 send = 0;
   
   //if(FB1 > 70)  Taken out at Motivo per Will request
   //    FB1 = 70;
   //if(FB2 > 70)
   //    FB2 = 70;
   //if(FB3 > 70)
   //    FB3 = 70;
   
   //f1 = ((float)FB1 * (511.0/70.0));
   //f2 = ((float)FB2 * (511.0/70.0));
   //f3 = ((float)FB3 * (511.0/70.0));
   
    f1 = ((float)FB1 * scaler);
    f2 = ((float)FB2 * scaler);
    f3 = ((float)FB3 * scaler);
   
   average_torque  = (f1 + f2 + f3)/3.0;
   
// added so there isn't a roll over // added at Motivo
   if (average_torque > 511)
       average_torque = 511;
   
   if (input(PIN_A1)  == TRUE){  // Turns motor right means positive torque 
       send = (signed int16)average_torque;
   }else if(input(PIN_A1) == FALSE){ // If low means motor left then negative
       send = -((signed int16)average_torque);
   }
   
   return (send);
}
//################################################################################################################################################
//################################################################################################################################################
void I2C(void){
    
    //signed int16 signed_can_byte2 = 0; // added for testing
    
    /////////////////// Store Local Variables //////////////////////////////////
    int16 CAN_P  = 0; // Primary Command
    int16 CAN_PV = 0; // Primary values
    int16 CAN_R  = 0; // Redundant Command
    int16 CAN_RV = 0; // Redundant values
    //signed signed_can_byte2 = 0; // Used for testing Data between GD5 board
    /////////////////// Global Variables ///////////////////////////////////////
    motor_current = 0;
    can_byte1 = 0;
    can_byte2 = 0;
    ////////////////////////////////////////////////////////////////////////////
    // Prepares constant motor current sending
    motor_current = ConvertTorque();
    send_buffer[0] = make8(motor_current,0); // Torque data LSB
    send_buffer[1] = make8(motor_current,1); // Torque data MSB 
    
    // The following will check if time is up and restart timer
    while(timer_up == 0); // Checks if time is up to start I2C 
    set_timer1(55535); // Restarts Timer1
    timer_up = 0;      // Timer Flag is reset 
    
    // Sends torque to Primary and receives commands
    send2_i2c (0x20);
    delay_us(500); // original 20us
    rcv4_i2c (0x20);
    
    // Stores received commands in variable
    CAN_P = make16(rcv_buffer[0],rcv_buffer[1]);
    CAN_PV = make16(rcv_buffer[3],rcv_buffer[2]);
    
    // Sends torque to Redundant and receives commands 
    send2_i2c (0x22);
    delay_us(500); // original 20us
    rcv4_i2c (0x22);
    
    // Stores received commands in variable
    CAN_R = make16(rcv_buffer[0],rcv_buffer[1]);
    CAN_RV = make16(rcv_buffer[3],rcv_buffer[2]);
    
    
    if(CAN_P == Position_Control || CAN_P == Passive_Mode || CAN_P == Set_Torque){
        can_byte1 = CAN_P;
        can_byte2 = CAN_PV;
    }else if(CAN_R == Position_Control || CAN_R == Passive_Mode || CAN_R == Set_Torque){
        can_byte1 = CAN_R;
        can_byte2 = CAN_RV;
    }else{
        can_byte1 = CAN_P;
        can_byte2 = CAN_PV;
    }
    
    // The following will track position
    if (can_byte1 == Normal_Report)
        position = can_byte2;
 
//if(can_byte1 == 0xDCC1){
//    signed_can_byte2 = (signed int16)can_byte2;
//    if (signed_can_byte2 != 100 && signed_can_byte2 != -100){
//        printf("can_byte1: %LX, can_byte2: %Ld\n\r", can_byte1, can_byte2);
//        printf("********************************************************************************\n\r");
//        output_low(PIN_C7); //This is LED pin Negative Logic
//   }
//}
    
 
} // END OF I2C
//################################################################################################################################################
//################################################################################################################################################
//################################################################################################################################################
//################################################################################################################################################
 
 
 
/*
void TorqueCommand (signed int16 torque){
    int16 direction = 0;
    int16 pwm = 0;
    
    if (torque > 0){
        direction = 0x3000; // means right for motor command
        pwm = torque + 169;
    }else if (torque < 0){ 
        direction = 0x2000;// means left for motor command
        pwm = abs(torque) + 169;
    }else{ 
        direction = 0x1000; // means off for motor command
        pwm = 0;
    }
    
    /*
    printf("Direction  = %LX\n\r", direction);    // Prints value to screen
    printf("PWM        = %Ld\n\r", pwm);          // Prints value to screen 
    printf("Torque Variable  = %Ld\n\r", torque);          // Prints value to screen
    printf("PV  = %Ld\n\r", can_byte2);          // Prints value to screen
    
    
    SetMotors(pwm,direction);
}
*/
/*
    
 
        Counter++;
        if (Counter >= 100){ // 100
            //printf("-------------------> MASTER IN ROBOT MODE \n\r"); // Prints to screen
            printf("Current Position = %Lu\n\r", current_position ); 
            //printf("Position Req = %Lu\n\r", target_position ); 
            //printf("Feed Back 1  = %LX\n\r", FB1);       // Prints value to screen
            //printf("Feed Back 2  = %LX\n\r", FB2);  // Prints value to screen
            //printf("Feed Back 3  = %LX\n\r", FB3);  // Prints value to screen
            //printf("Integral     = %Ld\n\r", ki* integral);       // Prints value to screen
            //printf("Derivatie    = %Ld\n\r", kd *derivative);       // Prints value to screen
            //printf("PWM          = %Ld\n\r", pwm);       // Prints value to screen
            //printf("SPEED        = %Lu\n\r", speed);       // Prints value to screen
            //printf("Error        = %Ld\n\r", error);       // Prints value to screen
            //printf("Selection    = %LX\n\r", selection);       // Prints value to screen
            //printf( "%s%x%s%x%s%x%s%x%s", " ", rcv_buffer[0], " ", rcv_buffer[1], " ", rcv_buffer[2], " ", rcv_buffer[3], ";\r  ");
            Counter = 0;
        }
        
        */
/*
void new_main()
{
    set_tris_a(0b11011100);
    set_tris_c(0b01010001);
    set_tris_d(0b10011111);
    output_d  (0b11111111);
    ////////////////////// P I N  S E T T I N G ////////////////////////////////
    output_high(PIN_C7); //This is LED pin Negative Logic
    output_high(PIN_A0); //This is EN pin
    output_low (PIN_A1); //This is INV pin
    output_high(PIN_D5); //This is IN1 pin
    output_high(PIN_A5); //Active high for slave 1
    output_high(PIN_C1); //Active high for slave 2
    //////////////////// I N I T I A L I Z E  ADC //////////////////////////////
    setup_adc(ADC_CLOCK_INTERNAL); //enables the a/d module
    //and sets the clock to internal adc clock
    setup_adc_ports(sAN10); //sets all the adc pins to analog
    set_adc_channel(10); //the next read_adc call will read channel 10
    delay_us(10); //a small delay is required after setting the channel
 
    //setup_timer_1(T1_INTERNAL|T1_DIV_BY_8);
    //int16 prev_time = 0xffff;
    int16 my_command_from_CAN = 0XDCC1;
    signed int16 my_value_from_CAN = 0;
    int16 my_torque = 0;
    int16 my_dir = 0x1000;
    
 
 
    while (TRUE)
    {
        // received motor command from CAN board
        if (my_command_from_CAN == 0xDCC1)
        {
            signed int16 signed_my_value_from_CAN = (signed int16)my_value_from_CAN;
            if (signed_my_value_from_CAN > 0)
            {
                my_torque = signed_my_value_from_CAN + 169;
                my_dir = 0x3000;
            }
            if (signed_my_value_from_CAN < 0)
            {
                my_torque = abs(signed_my_value_from_CAN) + 169;
                my_dir = 0x2000;
            }
            if (my_torque > torque_limit)
                my_torque = torque_limit;
        }
 
        // send command to master motor
        if (my_dir == 0x2000)
            output_low(PIN_A1); // turn left
        else if (my_dir == 0x3000)
            output_high(PIN_A1); // turn right
        //pwm_set_duty_percent(my_torque);
 
        // send command to slave motors
        int16 my_slave_motor_command = (my_torque & 0x0FFF) | my_dir;
        output_low(PIN_A5);
        int16 my_FB2 = spi_xfer(my_slave_motor_command);
        output_high(PIN_A5);
        output_low(PIN_C1);
        int16 my_FB3 = spi_xfer(my_slave_motor_command);
        output_high(PIN_C1);
        int16 my_FB1 = read_adc();
 
        // clamp feedback from motors and compute average torque from motors
        if (my_FB1 > 30) my_FB1 = 30;
        if (my_FB2 > 30) my_FB2 = 30;
        if (my_FB3 > 30) my_FB3 = 30;
        float f1 = ((float)my_FB1 * (511.0/30.0));
        float f2 = ((float)my_FB2 * (511.0/30.0));
        float f3 = ((float)my_FB3 * (511.0/30.0));
        float my_average_torque = (f1 + f2 + f3)/3.0;
 
        // prepare CAN data
        signed int16 send_to_CAN = (signed int16)my_average_torque;
        if (input(PIN_A1) == FALSE)
            send_to_CAN = -send_to_CAN;
 
        // send to and read from CAN board
        send_buffer[0] = make8(send_to_CAN,0); // Torque data LSB
        send_buffer[1] = make8(send_to_CAN,1); // Torque data MSB
 
//        int16 curr_time = get_timer1();
//        if (curr_time - prev_time)
//            delay_us(curr_time - prev_time);
        delay_ms(8);
        send2_i2c (0x20);
        delay_us(20);
        rcv4_i2c (0x20);
        //set_timer1(0);
        my_command_from_CAN = make16(rcv_buffer[0],rcv_buffer[1]);
        my_value_from_CAN = make16(rcv_buffer[3],rcv_buffer[2]);
 
        if (my_command_from_CAN == 0xDCC1){
            
            if (my_value_from_CAN != 431)
                if (my_value_from_CAN != -431){
                    printf("can_byte1: %LX, can_byte2: %Ld\n\r", my_command_from_CAN, my_value_from_CAN);
                    printf("********************************************************************************\n\r");
                    //output_low(PIN_C7); //This is LED pin Negative Logic
                }
            
            /*
            if (my_value_from_CAN != 2048){
                printf("can_byte1: %LX, can_byte2: %Ld\n\r", my_command_from_CAN, my_value_from_CAN);
                printf("********************************************************************************\n\r");
            }
                
            
        }
    } //End of While
}
* 
* 
* 
* void PositionControl(void){
    
    signed int16 target_position = 0;
    signed int16 current_position = 2048; // Set to middle position just in case
    
    signed int32 error = 0;
    signed int32 last_error = 0;
    signed int32 integral = 0;
    signed int32 derivative = 0;
    
    float kp = 20; // works well with 200 // 250 to high begins to oscillate
    float ki = 0; // works good with 0.9
    float kd = 0; // working before 300 bigger better // 500
    
    //float dt = 0; // estimated time for PID loop
    
    float pwm = 0;
    float torque_limit = 610; 
    
    char C;
    float sample;
    
    selection = 0;
    data = 0;
 
    while(can_byte1 == Normal_Report || can_byte1 == Position_Control){
        ///////////////////////////////////////////////////
        //                Trouble shooting               ///////////////////////
        ///////////////////////////////////////////////////
        // The following will run only if keyboard is hit
        if (kbhit()) {
            SetMotors(0,0x1000);
            C = toupper(getc());
            if(C == 'P'){
                printf("Enter Kp = \n\r"); 
                cin >> sample;
                kp = sample; 
            }else if(C == 'D'){
                printf("Enter Kd = \n\r");
                cin >> sample;
                kd = sample;
            }else if(C == 'I'){
                printf("Enter Ki = \n\r");
                cin >> sample;
                ki = sample; 
            }else if(C == 'E'){
                printf("Enter Max SPEED = \n\r");
                cin >> torque_limit;
            }
        } // end of kbhit
        ///////////////////////////////////////////////////
        //                Trouble shooting               ///////////////////////
        ///////////////////////////////////////////////////
        
        
        // The following compares Command and sets the appropriate vales to
        // Current Position and or Target position.
        if      (can_byte1 == Position_Control ){
            target_position = (signed int16)can_byte2;
        }else{
            current_position = (signed int16)can_byte2;
        }
        
        ///////////////////////////////////////////////////
        //                TESTING                        ///////////////////////
        ///////////////////////////////////////////////////
        // Proportion calculation part for PID Loop
        error = target_position - current_position;
        // Integral portion of the the PID Loop
        if(abs(error)<= 100){
            integral = integral + error;
        }else{ 
            integral = 0;
        }
        // Resets Integral when error reaches zero
        if (error == 0) // not sure if this works properly
            integral = 0; // not sure if this works properly
        // Derivative portion of the PID Loop
        derivative = (error - last_error);
        last_error = error;
        // PID Loop equation
        pwm = (kp * error) + (ki * integral) + (kd * derivative);
        // Sets limit of motor torques 
        if (pwm > torque_limit)
            pwm = torque_limit;
        else if (pwm < -torque_limit)
            pwm = -(torque_limit);
        // The following if statements decide motor direction
        if (pwm > 0) {
            data = 0x3000;// data = 3 means turn motor right 
            selection = (int16)pwm;
        }else if (pwm < 0) {
            data = 0x2000;// data = 2 means turn motor left
            selection = -((int16)pwm);
        }else if (pwm == 0) {
            data = 0x1000;// data = 1 means turn motor off
        }
        ///////////////////////////////////////////////////
        //                TESTING                        ///////////////////////
        ///////////////////////////////////////////////////
            
        SetMotors(selection,data);
        
        I2C(); // Checks commands
        /*
        Counter++;
        if (Counter >= 150){ // 100
            //printf("-------------------> MASTER IN ROBOT MODE \n\r"); // Prints to screen
            //printf("Current Position = %Ld\n\r", current_position ); 
            printf("Set Angle = %Ld\n\r", target_position ); 
            //printf("Feed Back 1  = %LX\n\r", FB1);       // Prints value to screen
            //printf("Feed Back 2  = %LX\n\r", FB2);  // Prints value to screen
            //printf("Feed Back 3  = %LX\n\r", FB3);  // Prints value to screen
            //printf("Integral     = %Ld\n\r", integral);       // Prints value to screen
            //printf("Derivatie    = %Ld\n\r", derivative);       // Prints value to screen
            //printf("PWM          = %Ld\n\r", pwm);       // Prints value to screen
            //printf("SPEED        = %Lu\n\r", speed);       // Prints value to screen
            printf("Error        = %Ld\n\r", error);       // Prints value to screen
            //printf("Selection    = %LX\n\r", selection);       // Prints value to screen
            //printf( "%s%x%s%x%s%x%s%x%s", " ", rcv_buffer[0], " ", rcv_buffer[1], " ", rcv_buffer[2], " ", rcv_buffer[3], ";\r  ");
            Counter = 0;
        } 
        
    }// END OF WHILE
    if (can_byte1 == Set_Torque)
        torque_flag = 1;
    
}//E N D  O F  P O S I T I O N  C O N T R O L 
*/
