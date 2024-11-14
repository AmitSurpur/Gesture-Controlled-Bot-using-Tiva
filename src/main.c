#include <stdint.h>

#define SYSCTL_RCGCGPIO      (*((volatile uint32_t *)0x400FE608))
#define SYSCTL_RCGCUART      (*((volatile uint32_t *)0x400FE618))

#define GPIO_PORTF_DIR       (*((volatile uint32_t *)0x40025400))
#define GPIO_PORTF_DEN       (*((volatile uint32_t *)0x4002551C))
#define GPIO_PORTF_DATA      (*((volatile uint32_t *)0x400253FC))

#define GPIO_PORTE_AFSEL     (*((volatile uint32_t *)0x40024420))
#define GPIO_PORTE_DEN       (*((volatile uint32_t *)0x4002451C))
#define GPIO_PORTE_PCTL      (*((volatile uint32_t *)0x4002452C))

#define UART5_DR             (*((volatile uint32_t *)0x40011000))
#define UART5_FR             (*((volatile uint32_t *)0x40011018))
#define UART5_IBRD           (*((volatile uint32_t *)0x40011024))
#define UART5_FBRD           (*((volatile uint32_t *)0x40011028))
#define UART5_LCRH           (*((volatile uint32_t *)0x4001102C))
#define UART5_CTL            (*((volatile uint32_t *)0x40011030))

#define GPIO_PORTB_DIR_R     (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_DEN_R     (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_DATA_R    (*((volatile unsigned long *)0x400053FC))

// Bit fields
#define UART_FR_RXFE         (1 << 4)   // Receive FIFO empty
#define UART_FR_TXFF         (1 << 5)   // Transmit FIFO full

#define GPIO1                (1 << 1)   // RED LED on PF1
#define GPIO2                (1 << 2)   // BLUE LED on PF2
#define GPIO3                (1 << 3)   // GREEN LED on PF3

void delay(uint32_t counter);
void hc05_init(void);
char bluetooth_read(void);
void bluetooth_write(char data);
void bluetooth_write_string(const char *str);
void delayMs(int n);
void move_forward(void);
void move_backward(void);
void turn_left(void);
void turn_right(void);
void PortB_Init(void);
void stop(void);

void PortB_Init(void) {
    SYSCTL_RCGCGPIO |= 0x02;           // Enable clock for PORTB
    while ((SYSCTL_RCGCGPIO & 0x02) == 0); // Wait until PORTB is ready
    GPIO_PORTB_DIR_R |= 0x3C;            // Set PB2, PB3, PB4, PB5 as output
    GPIO_PORTB_DEN_R |= 0x3C;            // Enable digital functionality on PB2, PB3, PB4, PB5
}

int main(void) {
    hc05_init();  // Initialize UART5 for Bluetooth
    PortB_Init(); // Initialize PORTB for motor control

    // Enable clock to GPIOF and configure PF1, PF2, and PF3 as output
    SYSCTL_RCGCGPIO |= (1 << 5);  // Enable clock to GPIOF
    GPIO_PORTF_DIR |= (GPIO1 | GPIO2 | GPIO3);  // Set PF1, PF2, and PF3 as outputs
    GPIO_PORTF_DEN |= (GPIO1 | GPIO2 | GPIO3);  // Enable PF1, PF2, and PF3 as digital pins

    while (1) {
        char c = bluetooth_read();  // Read character from UART5
        //this portion is to test the recieving of characters from bluetooth module
        // // Control LEDs based on received character
        // if (c == 'A') {
        //     GPIO_PORTF_DATA |= GPIO1;  // Turn on RED LED
        //     bluetooth_write_string("RED LED ON\n");
        // } else if (c == 'B') {
        //     GPIO_PORTF_DATA &= ~GPIO1;  // Turn off RED LED
        //     bluetooth_write_string("RED LED OFF\n");
        // } else if (c == 'C') {
        //     GPIO_PORTF_DATA |= GPIO2;  // Turn on BLUE LED
        //     bluetooth_write_string("BLUE LED ON\n");
        // } else if (c == 'D') {
        //     GPIO_PORTF_DATA &= ~GPIO2;  // Turn off BLUE LED
        //     bluetooth_write_string("BLUE LED OFF\n");
        // } else if (c == 'E') {
        //     GPIO_PORTF_DATA |= GPIO3;  // Turn on GREEN LED
        //     bluetooth_write_string("GREEN LED ON\n");
        // } else if (c == 'F') {
        //     GPIO_PORTF_DATA &= ~GPIO3;  // Turn off GREEN LED
        //     bluetooth_write_string("GREEN LED OFF\n");
        // }

        //control motor
        if (c == 'F'){
            move_forward();
            bluetooth_write_string("Moving Forward Sir!\n");
            delayMs(1000);
            //stop();
        }
        else if(c == 'B'){
            move_backward();
            bluetooth_write_string("Moving Backward, whatchout!\n");
            delayMs(1000);
            //stop();
        }
        else if(c == 'L'){
            turn_left();
            bluetooth_write_string("Remember movie 'cars'!-->'turn left to go right', so here we turn right\n");
            delayMs(1000);
            //stop();
        }
        else if(c == 'R'){
            turn_right();
            bluetooth_write_string("Lets move right!\n");
            delayMs(1000);
            //stop();
        }
        else if(c == 'S'){
            stop();
            bluetooth_write_string("Ruk ja bhai!!!\n");
            // delayMs(1000);
            // stop();
        }
        
        // else{
        //     bluetooth_write_string("Boi! you provided wrong command!\n");
        // }
    }
}

void move_forward() {
    GPIO_PORTB_DATA_R = 0x14;            // IN1 high, IN2 low, IN3 high, IN4 low
}

void move_backward() {
    GPIO_PORTB_DATA_R = 0x14>>1;            // IN1 low, IN2 high, IN3 low, IN4 high
}

void turn_left() {
    GPIO_PORTB_DATA_R = 0x10;            // IN1 high, IN2 low, IN3 low, IN4 low
}

void turn_right() {
    GPIO_PORTB_DATA_R = 0x04;            // IN1 low, IN2 low, IN3 high, IN4 low
}

void stop() {
    GPIO_PORTB_DATA_R = 0x00;            // All pins low
}

void delayMs(int n) {
    int i, j;
    for (i = 0; i < n; i++)
        for (j = 0; j < 3180; j++) {}    // 1 ms delay
}

/* Initialize UART5 for Bluetooth communication */
void hc05_init(void) {
    // Enable clocks for UART5 and GPIOE
    SYSCTL_RCGCUART |= (1 << 5);   // Enable clock to UART5
    SYSCTL_RCGCGPIO |= (1 << 4);   // Enable clock to GPIOE

    // Configure GPIO pins for UART5 (PE4 and PE5)
    GPIO_PORTE_AFSEL |= (1 << 4) | (1 << 5);     // Enable alternate functions on PE4, PE5
    GPIO_PORTE_DEN |= (1 << 4) | (1 << 5);       // Enable digital on PE4, PE5
    GPIO_PORTE_PCTL = (GPIO_PORTE_PCTL & 0xFF00FFFF) | 0x00110000;  // Configure PE4 and PE5 for UART

    // Configure UART5
    UART5_CTL &= ~0x01;               // Disable UART5 during configuration
    UART5_IBRD = 104;                 // Set integer part of baud rate (9600)
    UART5_FBRD = 11;                  // Set fractional part of baud rate (9600)
    UART5_LCRH = (0x3 << 5);          // Set 8-bit data, no parity, 1 stop bit
    UART5_CTL |= (0x01 | (1 << 8) | (1 << 9));  // Enable UART5, Tx, and Rx
}

/* Read data from Bluetooth (UART5) */
char bluetooth_read(void) {
    while (UART5_FR & UART_FR_RXFE);  // Wait until data is received
    return (char)(UART5_DR & 0xFF);   // Read received data
}

/* Write a character to Bluetooth (UART5) */
void bluetooth_write(char data) {
    while (UART5_FR & UART_FR_TXFF);  // Wait until transmit buffer is empty
    UART5_DR = data;                  // Send character
}

/* Write a string to Bluetooth */
void bluetooth_write_string(const char *str) {
    while (*str) {
        bluetooth_write(*(str++));  // Send each character
    }
}

/* Simple delay function */
void delay(uint32_t counter) {
    for (uint32_t i = 0; i < counter; i++) {
        __asm__("nop");
    }
}
