// Baremetal Remote Controlled Car
// Ported from Arduino to Baremetal targeting the ATMega328p microcontroller

#include <avr/io.h> // Defines pin/register names
#include <avr/interrupt.h> // Allows use of interrupts
#include <util/delay.h> // For _delay_ms() and _delay_us()
#include <stdint.h> // For data types like uint8_t, uint32_t
#include <stdbool.h> // For bool type - true/false


// Motor A (Left Motor)
#define MOTOR_A_IN1_DDR DDRB
#define MOTOR_A_IN1_PORT PORTB
#define MOTOR_A_IN1 PB1 // Arduino Digital Pin 9

#define MOTOR_A_IN2_DDR DDRD
#define MOTOR_A_IN2_PORT PORTD
#define MOTOR_A_IN2 PD6 // Arduino Digital Pin 6

// Motor B (Right Motor)
#define MOTOR_B_IN3_DDR DDRD
#define MOTOR_B_IN3_PORT PORTD
#define MOTOR_B_IN3 PD5 // Arduino Digital Pin 5

#define MOTOR_B_IN4_DDR DDRD
#define MOTOR_B_IN4_PORT PORTD
#define MOTOR_B_IN4 PD3 // Arduino Digital Pin 3

// Ultrasonic Sensor (HC-SR04)
#define TRIG_DDR DDRD
#define TRIG_PORT PORTD
#define TRIG_BIT PD2 // Arduino Digital Pin 2

#define ECHO_DDR DDRB
#define ECHO_PINR PINB
#define ECHO_BIT PB5 // Arduino Digital Pin 13

// Infrared (IR) Receiver
#define IR_PINR PINB
#define IR_DDR DDRB
#define IR_BIT PB3 // Arduino Digital Pin 11

// UART SETUP - For debug/ printing
#define BAUD_RATE 9600UL
#define UBRR_VAL ((F_CPU / (16UL * BAUD_RATE)) - 1) // Formula for baud rate register (from ATmega328P datasheet)


// TIMER 1 - used for IR commands and time the sonar echo

static volatile uint16_t timer1_overflows = 0;

ISR(TIMER1_OVF_vect)
{
    timer1_overflows++;
}

void timer1_init(void)
{
    TCCR1A = 0;
    TCCR1B = (1 << CS11); // Prescaler = 8, so 0.5 µs per tick
    TIMSK1 = (1 << TOIE1); // Enable overflow interrupt so we can track long durations
    TCNT1 = 0;
    timer1_overflows = 0;
}

uint32_t timer1_now(void)
{
    uint8_t sreg = SREG; cli();
    uint16_t t = TCNT1;
    uint16_t ov = timer1_overflows;
    if ((TIFR1 & (1 << TOV1)) && t < 0x8000) ov++; // Overflow happened but ISR hasn't run yet
    SREG = sreg;
    return ((uint32_t)ov << 16) | t;
}

uint32_t ticks_to_us(uint32_t ticks)
{
    return ticks >> 1; // Each tick is 0.5 µs, so divide by 2 to get µs
}


// IR RECEIVER

// Stores the latest decoded IR frame
typedef struct {
    uint8_t cmd;
    bool repeat;
    bool ready;
} IrFrame;

static volatile IrFrame ir_result;

// State machine states
typedef enum {
    IR_IDLE,
    IR_LEADER_LOW,
    IR_LEADER_SPACE,
    IR_DATA_PULSE,
    IR_DATA_SPACE,
} IrState;

static volatile IrState ir_state = IR_IDLE;
static volatile uint32_t ir_edge_ts = 0;
static volatile uint32_t ir_data = 0;
static volatile uint8_t  ir_bit = 0;

// Returns true if value is within 30% of target
bool in_range(uint32_t value, uint32_t target)
{
    uint32_t margin = target * 30 / 100;
    return (value >= target - margin) && (value <= target + margin);
}

// Measures time between edges to decode the NEC IR protocol
ISR(PCINT0_vect)
{
    uint32_t now = timer1_now();
    uint32_t elapsed = ticks_to_us(now - ir_edge_ts);
    ir_edge_ts = now;

    bool line_high = (IR_PINR & (1 << IR_BIT));

    switch (ir_state)
    {
        case IR_IDLE:
            if (!line_high) ir_state = IR_LEADER_LOW;
            break;

        case IR_LEADER_LOW:
            if (line_high && in_range(elapsed, 9000))
                ir_state = IR_LEADER_SPACE;
            else
                ir_state = IR_IDLE;
            break;

        case IR_LEADER_SPACE:
            if (!line_high)
            {
                if (in_range(elapsed, 4500))
                {
                    ir_data  = 0;
                    ir_bit   = 0;
                    ir_state = IR_DATA_PULSE;
                }
                else if (in_range(elapsed, 2250))
                {
                    if (!ir_result.ready)
                    {
                        ir_result.repeat = true;
                        ir_result.ready  = true;
                    }
                    ir_state = IR_IDLE;
                }
                else
                {
                    ir_state = IR_IDLE;
                }
            }
            break;

        case IR_DATA_PULSE:
            if (line_high && in_range(elapsed, 562))
                ir_state = IR_DATA_SPACE;
            else
                ir_state = IR_IDLE;
            break;

        case IR_DATA_SPACE:
            if (!line_high)
            {
                if (in_range(elapsed, 562))  {}
                else if (in_range(elapsed, 1687)) { ir_data |= (1UL << ir_bit); }
                else { ir_state = IR_IDLE; break; }

                ir_bit++;

                if (ir_bit == 32)
                {
                    uint8_t addr    = (ir_data >>  0) & 0xFF;
                    uint8_t addr_inv= (ir_data >>  8) & 0xFF;
                    uint8_t cmd     = (ir_data >> 16) & 0xFF;
                    uint8_t cmd_inv = (ir_data >> 24) & 0xFF;

                    bool checksum_ok = ((addr + addr_inv) == 0xFF) && ((cmd + cmd_inv) == 0xFF);

                    if (checksum_ok && !ir_result.ready)
                    {
                        ir_result.cmd = cmd;
                        ir_result.repeat = false;
                        ir_result.ready = true;
                    }
                    ir_state = IR_IDLE;
                }
                else
                {
                    ir_state = IR_DATA_PULSE;
                }
            }
            break;
    }
}

void ir_init(void)
{
    IR_DDR  &= ~(1 << IR_BIT);
    PORTB   &= ~(1 << IR_BIT);
    PCICR   |=  (1 << PCIE0);
    PCMSK0  |=  (1 << PCINT3);
}

// Check if a new IR frame has arrived - returns true and fills *cmd if true
bool ir_get_frame(uint8_t *cmd, bool *repeat)
{
    if (!ir_result.ready) return false;
    uint8_t sreg = SREG; cli();
    *cmd    = ir_result.cmd;
    *repeat = ir_result.repeat;
    ir_result.ready = false;
    SREG = sreg;
    return true;
}


// UART for debugging (from ATmega328P datasheet)
void USART_Init(unsigned int ubrr)
{
    UBRR0H = (unsigned char)(ubrr >> 8);
    UBRR0L = (unsigned char)(ubrr);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8 data bits, 1 stop bit
}

void USART_Transmit(unsigned char data)
{
    while (!(UCSR0A & (1 << UDRE0)));   // Wait for transmit buffer to be empty
    UDR0 = data;
}

void uart_putc(char c)   { USART_Transmit((unsigned char)c); }

void uart_puts(const char *s)
{
    int i = 0;
    while (s[i] != '\0') { uart_putc(s[i]); i++; }
}

void uart_println(const char *s) { uart_puts(s); uart_putc('\r'); uart_putc('\n'); }

void uart_print_u32(uint32_t v)
{
    if (v >= 10) uart_print_u32(v / 10); // Print all digits before the last
    uart_putc((v % 10) + '0'); // Convert digit to ASCII character
}


// ULTRASONIC SONAR (HC-SR04)
float sonar_cm(void)
{
    TRIG_PORT &= ~(1 << TRIG_BIT); // Start LOW
    _delay_us(2);
    TRIG_PORT |=  (1 << TRIG_BIT); // Go HIGH
    _delay_us(10);
    TRIG_PORT &= ~(1 << TRIG_BIT); // Back LOW

    // Waiting for ECHO to go HIGH
    uint32_t start = timer1_now();
    while (!(ECHO_PINR & (1 << ECHO_BIT)))
    {
        if (ticks_to_us(timer1_now() - start) > 30000) return 0.0f;
    }

    // Counting how long ECHO stays HIGH
    start = timer1_now();
    while (ECHO_PINR & (1 << ECHO_BIT))
    {
        if (ticks_to_us(timer1_now() - start) > 30000) return 0.0f;
    }
    uint32_t duration_us = ticks_to_us(timer1_now() - start);

    // Convert to centimetres
    return (duration_us * 0.0343f) / 2.0f;
}


// MOTOR CONTROL
void motor_set(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4)
{
    if (in1) MOTOR_A_IN1_PORT |=  (1 << MOTOR_A_IN1);
    else     MOTOR_A_IN1_PORT &= ~(1 << MOTOR_A_IN1);

    if (in2) MOTOR_A_IN2_PORT |=  (1 << MOTOR_A_IN2);
    else     MOTOR_A_IN2_PORT &= ~(1 << MOTOR_A_IN2);

    if (in3) MOTOR_B_IN3_PORT |=  (1 << MOTOR_B_IN3);
    else     MOTOR_B_IN3_PORT &= ~(1 << MOTOR_B_IN3);

    if (in4) MOTOR_B_IN4_PORT |=  (1 << MOTOR_B_IN4);
    else     MOTOR_B_IN4_PORT &= ~(1 << MOTOR_B_IN4);
}

void move_forward(void)  { uart_println("Moving Forward...");  motor_set(1, 0, 1, 0); }
void move_backward(void) { uart_println("Moving Backward..."); motor_set(0, 1, 0, 1); }
void move_right(void)    { uart_println("Moving Right...");    motor_set(1, 0, 0, 1); }
void move_left(void)     { uart_println("Moving Left...");     motor_set(0, 1, 1, 0); }
void motor_stop(void)    { uart_println("Stopping...");        motor_set(0, 0, 0, 0); }

// Map IR button codes to motor actions
void handle_manual(uint8_t cmd)
{
    switch (cmd)
    {
        case 0x09: move_forward();  break;
        case 0x15: move_backward(); break;
        case 0x40: move_left();     break;
        case 0x43: move_right();    break;
        case 0x45: motor_stop();    break;
        default:   break;
    }
}


// SELF-NAVIGATION (Obstacle Avoidance)
void self_navigation(void)
{
    float distance_cm = sonar_cm();

    if (distance_cm == 0.0f)
    {
        uart_println("No echo from sonar.");
        return;
    }

    uart_print_u32((uint32_t)distance_cm);
    uart_println(" cm");

    if (distance_cm <= 10.0f)
    {
        motor_stop();
        _delay_ms(500);
        move_right();
        _delay_ms(500);
    }
    else
    {
        move_forward();
    }
}


int main(void)
{
    MOTOR_A_IN1_DDR |= (1 << MOTOR_A_IN1);
    MOTOR_A_IN2_DDR |= (1 << MOTOR_A_IN2);
    MOTOR_B_IN3_DDR |= (1 << MOTOR_B_IN3);
    MOTOR_B_IN4_DDR |= (1 << MOTOR_B_IN4);

    TRIG_DDR |=  (1 << TRIG_BIT);
    ECHO_DDR &= ~(1 << ECHO_BIT);

    USART_Init(UBRR_VAL);
    timer1_init();
    ir_init();

    sei();

    uart_println("Baremetal Car Ready.");

    bool self_driving = false;

    while (1)
    {
        uint8_t cmd;
        bool repeat;

        if (ir_get_frame(&cmd, &repeat))
        {
            if (!repeat)
            {
                if (cmd == 0x46)
                {
                    motor_stop();
                    self_driving = !self_driving;
                    uart_println(self_driving ? "Self-drive ON" : "Self-drive OFF");
                }
                else if (!self_driving)
                {
                    handle_manual(cmd);
                }
            }
        }

        if (self_driving)
        {
            self_navigation();
        }
    }
}
