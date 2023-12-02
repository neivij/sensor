/*
 * Code for EDI SPI dasy chain sensor node with BNO055.
 *
 * Configurable parameters:
 *      NUMBER_OF_SENSORS   - max. number of sensors in the chain. Must comply with the masterboard code.
 *
 * PCB connections:
 *      P1.1 - CON2 Pin 4 (MISO)
 *      P1.2 - CON1 Pin 1 (MOSI)
 *      P1.4 - CON2 PIN3 (CLK1)
 *      P1.0 - CON2 PIN3 (CLK1)
 *      P1.5 - GND
 *      P2.0 - CON2 PIN3 (CLK1, TA1.0)
 *      P2.1 - CON2 PIN3 (CLK1, TA1.1)
 *      P1.6 - BNO_SCL_RX
 *      P1.7 - BNO_SDA_TX
 *      P2.6 - BNO reset pin
 *      P2.7 - BNO INT pin
 *      P3.7 - CON1 PIN2 (CLK2)
 */

#include<msp430.h>
#include<stdlib.h>
#include<stdio.h>
#include"string.h"
#include"bno055_basic.h"
#include"uart_support.h"

#define LINACCEL
/* Defines */
#define GROUP_ID                0xBA
#define DEVICES_IN_GROUP        26
#define DEVICE_DATA_LEN         12
#define DEVICE_GROUP_NR         20
#define RAW_G_DATA_LEN BNO055_GRAVITY_DATA_Z_MSB_ADDR - BNO055_GRAVITY_DATA_X_LSB_ADDR

#define BYTES_PASS              (COMM_HEADER_LEN + DEVICE_GROUP_NR*DEVICE_DATA_LEN)
#define BYTES_IN_PACKET         (COMM_HEADER_LEN + DEVICE_DATA_LEN * DEVICES_IN_GROUP)
/// Start counting from 0

#define TIMER_A1_UP()           TA1CTL  |= 16             // Start timer Up mode
#define TIMER_A1_STOP()         TA1CTL  &= ~48             // Stop timer
#define TIMER_A1_ZERO()         TA1R     = 1;
/* Typedefs */
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;

/* Local functions */
void init_all();            // initializes modules
static void bno_i2c_read(uint8_t reg_addr, uint8_t* data, uint8_t size);
static void bno_i2c_write(uint8_t reg_addr, uint8_t* data, uint8_t size);
static void reset_init_config_bno055(void);
uint16_t crc16_calc(uint8_t *byt, uint16_t size);


/* Global variables */
uint8_t data_buffer[DEVICE_DATA_LEN];
uint8_t bno055_active_page = 0;                         // This is the default.
uint8_t bno055_opmode = BNO055_OPERATION_MODE_CONFIG;   // This is the default.
uint16_t rx_counter = 0;
uint8_t timer_counter = DEVICE_GROUP_NR;
uint8_t syn_flag = 1;
uint8_t timeout_flag = 0;

uint8_t  accel_buffer[6];
uint16_t accelx;
uint16_t accely;
uint16_t accelz;

uint8_t byte;

void formatAccelData(char* output, const uint8_t* accel_buffer) {
    int i;
    for (i = 0; i < 6; ++i) {
        *output++ = "0123456789ABCDEF"[accel_buffer[i] >> 4];
        *output++ = "0123456789ABCDEF"[accel_buffer[i] & 0xF];
    }
    *output = '\n'; // Null-terminate the string
}

/* Main */
int main(void) {


    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    init_all();
    reset_init_config_bno055();
    __bis_SR_register(GIE);
    P1SEL|= BIT2;
    P1SEL2|= BIT2;
    while(1)
    {
//        uint16_t msg_crc16;
//        uint8_t k;


        TA1CCR0   = 19999;               // In UP mode interrupt frequency = 16/8/(19999+1) [MHz] = 0.1kHz (10ms)
        //RECEIVE DATA HERE:
#ifndef LINACCEL
        // Assuming accel_buffer contains raw acceleration data
        bno_i2c_read(BNO055_ACCEL_DATA_X_LSB_ADDR, accel_buffer, 6);
#else
        bno_i2c_read(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, accel_buffer, 6);
#endif
        // Convert raw acceleration values to ASCII strings
        int accelx = (accel_buffer[1] << 8) | accel_buffer[0];
        int accely = (accel_buffer[3] << 8) | accel_buffer[2];
        int accelz = (accel_buffer[5] << 8) | accel_buffer[4];
/*
        char accelx_str[10];  // Adjust the size based on your needs
        char accely_str[10];
        char accelz_str[10];

        // Convert integers to ASCII strings
        sprintf(accelx_str, "%d\0", accelx);
        sprintf(accely_str, "%d\0", accely);
        sprintf(accelz_str, "%d\0", accelz);

        // Send the ASCII strings over UART
        for (i = 0; accelx_str[i] != '\0'; i++) {
            while (!(IFG2 & UCA0TXIFG));
            UCA0TXBUF = accelx_str[i];
        }
*/
        int i;
        char output[13];
        formatAccelData(output, accel_buffer);
        //send acceleration values as binary data over UART
        for (i = 0; i < 13; i++) {
            while (!(IFG2 & UCA0TXIFG));
            UCA0TXBUF = output[i];
        }
/*
        while (!(IFG2 & UCA0TXIFG));  // Wait for the UART buffer to be ready
        UCA0TXBUF = ' ';  // Add a space between x, y, and z values

        for (i = 0; accely_str[i] != '\0'; i++) {
            while (!(IFG2 & UCA0TXIFG));
            UCA0TXBUF = accely_str[i];
        }

        while (!(IFG2 & UCA0TXIFG));  // Wait for the UART buffer to be ready
        UCA0TXBUF = ' ';  // Add a space between x, y, and z values

        for (i = 0; accelz_str[i] != '\0'; i++) {
            while (!(IFG2 & UCA0TXIFG));
            UCA0TXBUF = accelz_str[i];
        }
        // Add newline characters or any other formatting as needed
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = '\r';

        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = '\n';
*/
        _delay_cycles(10000);//1ms delay
    }
}


void init_all(){
    /// Clock
    BCSCTL1=CALBC1_16MHZ;   //16MHz clock
    DCOCTL=CALDCO_16MHZ;
    IFG1 &= ~(OFIFG);                  // Clear OSCFault flag

    /// GPIOs
    P2OUT &= ~BIT6;         // BNO in reset state
    P2DIR |= BIT6;          // BNO in active state

    /// UART@115200 UCA0. Tx port is disabled!
    UCA0CTL1=UCSSEL_2+UCSWRST;  //keep reset on while changing settings

    UCA0BR0 = 8;                  // Clock Prescaler - lower byte
    UCA0BR1 = 0;                  // Clock Prescaler - upper byte
    UCA0MCTL = UCOS16 | UCBRF_11; // Enable oversampling and set UCBRF to 11

    P1SEL|= BIT1;
    P1SEL2|= BIT1;
    UCA0CTL1&=~UCSWRST; // clear reset to enable UART
//    IE2 |= UCA0RXIE;    // Enable rx interrupts.

    /// I2C USCIB0
    UCB0CTL1|=UCSWRST;                  // put module in reset state
    UCB0CTL1|=UCSSEL_3;                 // select SMCLK
    UCB0CTL0|=UCMODE_3+UCMST+UCSYNC;    // set I2C MODE MASTER MODE
    UCB0I2CSA = BNO055_I2C_ADDR1;       // set sensor address
    UCB0BR0= 40;                        // Prescaler for maximum data rate (400kHz)
    UCB0BR1=0;
    P1SEL|=BIT6+BIT7;
    P1SEL2|=BIT6+BIT7;                  // P1.6=SCL (I2C) P1.7=SDA (I2C)
    UCB0CTL1 &= ~UCSWRST;               // start I2C

    // Timer A
    TA1CTL   |= TASSEL_2|ID_3|TAIE; // clock: SMCLK, divider:/8 , allow interrupts, mode: Stop.
                                    // Default compare settings, CCIF not necessary
    TA1CCR0   = 19999;               // In UP mode interrupt frequency = 16/8/(19999+1) [MHz] = 0.1kHz (10ms)
}


/*
 * Function for reading data from BNO055 sensor
 *  > uint8_t reg_addr  address of the first register
 *  > uint8_t* data     pointer to data array where data will be saved
 *  > uint8_t size      number of consecutive regsters to read (address auto increment)
 */

static void bno_i2c_read(uint8_t reg_addr, uint8_t* data, uint8_t size){
    UCB0CTL1 &= ~UCSWRST;
    IFG2 &= ~(UCB0TXIFG|UCB0RXIFG);
    // Transmit - send read addres
    UCB0CTL1 |= UCTR + UCTXSTT;     // Start. Transmiter mode. Send slave address
    while(!(IFG2&UCB0TXIFG));       // Wait for tx buffer to be ready
    UCB0TXBUF = reg_addr;           // Write data to be transmited (register address)
    while(UCB0CTL1 & UCTXSTT);      // Poll til Start condition is finished. And the first data to be transmitted can be written
    if(UCB0STAT & UCNACKIFG){
        UCB0CTL1 |= UCTXSTP;        // Stop. In case slave address is not acknowledged. UCBxTXBUF will be discarded
        while(UCB0CTL1 & UCTXSTP);      // Poll til previous data transfer is completed
        UCB0CTL1 |= UCSWRST;
        return;
    }
    // Receive
    // Restart I2C communication
    // Send BNO sensor address with read bit
    IFG2 &= ~UCB0TXIFG;
    UCB0CTL1 &= ~UCTR;              // Receiver mode
    UCB0CTL1 |= UCTXSTT;            // Restart. Send slave address.
    while (UCB0CTL1 & UCTXSTT);     // Poll til Start condition is finished
    if(UCB0STAT & UCNACKIFG){
        UCB0CTL1 |= UCTXSTP;        // Stop. In case slave address is not acknowledged.
        while(UCB0CTL1 & UCTXSTP);      // Poll til previous data transfer is completed
        UCB0CTL1 |= UCSWRST;
        return;
    }
    // Read data into the provided buffer
    uint8_t k;
    for(k=0;k<(size-1);k++){
        uint16_t timeout_cnt=0;         // Timeout counter to monitor i2c hanging.
        while (!(IFG2 & UCB0RXIFG)){
            if( UCB0STAT & UCNACKIFG){
                UCB0CTL1 |= UCTXSTP;        // Stop. In case slave address is not acknowledged. UCBxTXBUF will be discarded
                while(UCB0CTL1 & UCTXSTP);      // Poll til previous data transfer is completed
                return;
            }
            if( timeout_cnt>10000){
                UCB0CTL1 |= UCSWRST;
                reset_init_config_bno055();
                return;
            }
            timeout_cnt++;
        }
        data[k] = UCB0RXBUF;
        IFG2 &= ~UCB0RXIFG;

    }
    UCB0CTL1 |= UCTXSTP;                // Stop.
    while (!(IFG2 & UCB0RXIFG));        // Wait for data
    data[k] = UCB0RXBUF;
    while(UCB0CTL1 & UCTXSTP);      // Poll til previous data transfer is completed
    UCB0CTL1 |= UCSWRST;

}


/*
 * Function for writing values in BNO055 rgisters
 *  > uint8_t reg_addr  address of the first register
 *  > uint8_t* data     pointer to data array where data will be saved
 *  > uint8_t size      number of consecutive regsters to read (address auto increment)
 */

static void bno_i2c_write(uint8_t reg_addr, uint8_t* data, uint8_t size){
    UCB0CTL1 &= ~UCSWRST;
    while(UCB0CTL1 & UCTXSTP);      // Poll til previous data transfer is completed
    // Transmit
    UCB0CTL1 |= UCTR + UCTXSTT;     // Start. Transmiter mode. Send slave address
    UCB0TXBUF = reg_addr;           // Write data to be transmited (register address)
    while(UCB0CTL1 & UCTXSTT);      // Poll til Start condition is finished. And the first data to be transmitted can be written
    if(UCB0STAT & UCNACKIFG){
        UCB0CTL1 |= UCTXSTP;        // Stop. In case slave address is not acknowledged. UCBxTXBUF will be discarded
        return;
    }
    while (!(IFG2 & UCB0TXIFG));
    UCB0TXBUF = data[0];
    uint8_t k;
    for(k=0;k<(size-1);k++){
        while (!(IFG2 & UCB0TXIFG));
        UCB0TXBUF = data[k+1];
    }
    while (!(IFG2 & UCB0TXIFG));
    UCB0CTL1 |= UCTXSTP;            // Stop.
    IFG2 &= ~UCB0TXIFG;
    while(UCB0CTL1 & UCTXSTP);      // Poll til previous data transfer is completed
    UCB0CTL1 |= UCSWRST;
}


// sets P1.5 function to GPIO input
static void config_clk_input(void){
    UCA0CTL1 |=UCSWRST; // SPI disabled
    P1SEL &=~BIT4;      // set P1.4 pin as I/O port
    P1SEL2&=~BIT4;      // set P1.4 pin as I/O port
    P1DIR &= ~BIT4;     // set P1.4 as input
    //P1IFG &= ~BIT4;       // Clear gpio interrupts
    //P1IE  |= BIT4;        // Enable gpio interrupts
}

// sets P1.5 pin function to SPI CLK
static void config_clk_spi(void){
    //P1IE  &= ~BIT4;           // Disable gpio interrupts
    //P1IFG &= ~BIT4;           // Clear gpio interrupts
    P1SEL |= BIT4;
    P1SEL2 |= BIT4;
    UCA0CTL1 &= ~UCSWRST;       // Enable SPI

}

static void reset_init_config_bno055(void){
        UCB0CTL1 |= UCSWRST;
        P2OUT &= ~BIT6;             // BNO_RST pin
        _delay_cycles(200);         // Valid delay for 8 MHz clock to initiate BNO055 reset (>20 ns)
        P2OUT |= BIT6;              // BNO_RST pin
        __delay_cycles(10600000);    // Valid delay for 8 MHz clock for sensor to setup form RESET (>650 ms)
        UCB0CTL1 &= ~UCSWRST;
        // Reset default page.
        bno055_active_page = 0;
        // Write operation mode in to sensor. Default after POR is configuration mode
#ifndef LINACCEL
        bno055_opmode = BNO055_OPERATION_MODE_ACCONLY;
        bno_i2c_write(BNO055_OPR_MODE_ADDR, &bno055_opmode , 1);    // Enter data fusion mode
        __delay_cycles(114000);                                      // Valid delay for 8 MHz clock to allow sensor to switch from CONFIG to NDOF mode (>7 ms)
#else
        bno055_opmode = BNO055_OPERATION_MODE_NDOF;
        bno_i2c_write(BNO055_OPR_MODE_ADDR, &bno055_opmode , 1);    // Enter data fusion mode
        __delay_cycles(114000);                                      // Valid delay for 8 MHz clock to allow sensor to switch from CONFIG to NDOF mode (>7 ms)
#endif
        uint8_t bno055_accel_Settings = BNO055_ACCEL_UNIT_MG;
        bno_i2c_write(BNO055_UNIT_SEL_ADDR, &bno055_accel_Settings, 1);    // change to  milliG
        __delay_cycles(114000);                                      // Valid delay for 8 MHz clock to allow sensor to switch from CONFIG to NDOF mode (>7 ms)

        bno055_accel_Settings = BNO055_ACCEL_BW_125HZ;
        bno_i2c_write(BNO055_ACCEL_CONFIG_ADDR, &bno055_accel_Settings, 1);    // change to  125Hz sampling rate
        __delay_cycles(114000);                                      // Valid delay for 8 MHz clock to allow sensor to switch from CONFIG to NDOF mode (>7 ms)

        UCB0CTL1 |= UCSWRST;
}

uint16_t crc16_calc(uint8_t *byt, uint16_t size)
{
    /* Calculate CRC-16 value; uses The CCITT-16 Polynomial,
    expressed as X^16 + X^12 + X^5 + 1 */
    uint16_t crc = 0xffff;
    uint16_t ind;
    uint8_t b;

    for( ind=0; ind<size; ++ind )
    {
        crc ^= (((uint16_t) byt[ind]) << 8);
        for( b=0; b<8; ++b )
        {
            if( crc & 0x8000 )
            {
                crc = (crc << 1)^0x1021;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/* Interrupts */
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI_RX_ISR (void)
{
    // Count received bytes
    if(UCA0STAT & UCOE)
    {
        rx_counter +=2;
    }
    else
    {
        rx_counter++;
    }

    // Read the RX buffer
    byte = UCA0RXBUF;
    TIMER_A1_ZERO();

    // Check if received data are in sync with packets
    if(syn_flag)
    {
        // If not in sync, check if byte sequence matches received bytes so far
        if((rx_counter <= sizeof(sync_seq)) && (sync_seq[rx_counter-1]!=byte))
        {
            rx_counter = 0;
        }
        // If all bytes had a match, reset everything and start data transmission phase
        if(rx_counter >= sizeof(sync_seq))
        {
            syn_flag = 0;
            timeout_flag = 0;
            TIMER_A1_UP();
        }
    }

}

#pragma vector = TIMER1_A1_VECTOR
__interrupt void myTimer(void)
{
    switch(TA1IV)
    {
    case TA1IV_TAIFG:
        TIMER_A1_STOP();
        TIMER_A1_ZERO();
        syn_flag = 1;
        //rx_counter = 0;
        timeout_flag = 1;
        //P1OUT ^= BIT6;
        break;
    default:
        break;
    }
}

