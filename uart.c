#include <avr/io.h>
#define F_CPU 7372800UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#define BAUD 115200
#define UBRNUM (F_CPU / BAUD / 16 - 1)

#define LED1 5

#define RET_OK 1
#define RET_ERROR 0

#define SEND_DATA_MASK 0x3f
#define DATA_MASK 0x3f

volatile uint8_t uart_buff[256];
uint8_t uart_begin = 0;
volatile uint8_t uart_end = 0;

uint8_t line[128];
uint8_t line_len = 0;

uint8_t data[64];
uint8_t data_begin = 0;
uint8_t data_end = 0;

uint8_t msg[64];
uint8_t msg_len = 0;

volatile uint8_t send_data[64];
uint8_t send_data_begin = 0;
volatile uint8_t send_data_end = 0;

const uint8_t CRLF[] PROGMEM = "\r\n";
const uint8_t OK[] PROGMEM = "OK\r\n";
const uint8_t ERROR[] PROGMEM = "ERROR\r\n";
const uint8_t SEND_OK[] PROGMEM = "SEND OK\r\n";
const uint8_t AT[] PROGMEM = "OK\r\n";
const uint8_t AT_CIPCLOSE[] PROGMEM = "AT+CIPCLOSE\r\n";
const uint8_t AT_CIPSTART[] PROGMEM = "AT+CIPSTART=\"TCP\",\"192.168.1.174\",5555\r\n";
const uint8_t AT_CIPSEND[] PROGMEM = "AT+CIPSEND=";

uint8_t strcmp_flash(
    const uint8_t *s1, 
    const uint8_t *s2) 
{
    uint8_t i;
    for (i = 0; s1[i] && s1[i] == pgm_read_byte(&s2[i]); i++) {}
    if (s1[i] == pgm_read_byte(&s2[i]))
        return 1;
    return 0;
}

inline void init_uart() {
    UBRRH = (uint8_t) (UBRNUM >> 8);
    UBRRL = (uint8_t) UBRNUM;
    UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

ISR(USART_RXC_vect) {
    uint8_t data = UDR;
    uart_buff[uart_end++] = data;
}

inline void uart_send(uint8_t data) {
    while (!(UCSRA & (1 << UDRE)));
    UDR = data;
}

inline uint8_t uart_pick() {
    while (uart_begin == uart_end) {}
    return uart_buff[uart_begin++];
}

void uart_send_string_flash(const uint8_t *s) {
    uint8_t data, i;
    for (i = 0; (data = pgm_read_byte(&s[i])) != 0; i++) {
        uart_send(data);
    }
}

void uart_send_string(const uint8_t *s) {
    uint8_t i;
    for (i = 0; s[i] != 0; i++) {
        uart_send(s[i]);
    }
}

void send_byte(uint8_t b) {
    send_data[send_data_end++] = b;
}

void send_array(const uint8_t *a, uint8_t len) {
    uint8_t i;
    for (i = 0; i < len; i++)
        send_byte(a[i]);
}

inline void init() {
    init_uart();

    DDRC = (1 << LED1);

    wdt_enable(WDTO_2S);
    wdt_reset();

    sei();
}

void readline() {
    line_len = 0;
    while (1) {
        uint8_t ch = uart_pick();
        line[line_len++] = ch;
        if (ch == '\n') {
            line[line_len] = '\0';
            return;
        }
    }
}

uint8_t read_util_ok_error() {
    while (1) {
        readline();
        if (strcmp_flash(line, OK))
            return RET_OK;
        else if (strcmp_flash(line, ERROR))
            return RET_ERROR;
    }
}

inline void cmd_AT() {
    uart_send_string_flash(AT);
    read_util_ok_error();
}

inline void cmd_AT_CIPCLOSE() {
    uart_send_string_flash(AT_CIPCLOSE);
    read_util_ok_error();
}

inline void cmd_AT_CIPSTART() {
    uart_send_string_flash(AT_CIPSTART);
    read_util_ok_error();
}

uint8_t number[4];
uint8_t number_len;

void reset() {
    while (1) {}
}

uint8_t data_pick() {
    uint8_t res = data[data_begin];
    data_begin = (data_begin + 1) & DATA_MASK;
    return res;
}

void data_put(uint8_t ch) {
    data[data_end] = ch;
    data_end = (data_end + 1) & DATA_MASK;
}

uint8_t data_len() {
    return (data_end - data_begin) & DATA_MASK;
}

uint8_t send_data_pick() {
    uint8_t res = send_data[send_data_begin];
    send_data_begin = (send_data_begin + 1) & SEND_DATA_MASK;
    return res;
}

inline uint8_t send_data_len(uint8_t end) {
    return (end - send_data_begin) & SEND_DATA_MASK;
}

void handle_tcp_recv();

inline void cmd_AT_CIPSEND(uint8_t end) {
    uint8_t len = send_data_len(end);
    uart_send_string_flash(AT_CIPSEND);
    number_len = 0;
    while (len != 0) {
        number[number_len++] = len % 10;
        len /= 10;
    }
    while (number_len != 0) {
        uart_send(number[--number_len] + '0');
    }
    uart_send_string_flash(CRLF);

    while (uart_begin == uart_end) {}
    handle_tcp_recv();

    uint8_t ret = read_util_ok_error();
    if (ret == RET_ERROR) 
        reset();

    uart_pick();
    uart_pick();

    while (send_data_begin != end)
        uart_send(send_data_pick());

    while (1) {
        readline();
        if (strcmp_flash(line, SEND_OK))
            break;
    }
    readline();
}

void handle_data() {
    uint8_t len = data_len();
    for (; len != 0; len--) {
        data_pick();
        PORTC ^= (1 << LED1);
    }
}

void handle_tcp_recv() {
    uint8_t ch = uart_buff[uart_begin];
    if (ch == '+') {
        while (1) {
            ch = uart_pick();
            if (ch == ',')
                break;
        }

        uint8_t number = 0;
        while (1) {
            ch = uart_pick();
            if (ch == ':')
                break;
            number *= 10;
            number += ch - '0';
        }
        for (; number != 0; number--) {
            data_put(uart_pick());
        }
        handle_data();
    }
}

inline void twinkling_led() {
    PORTC ^= (1 << LED1);
    _delay_ms(50);
    PORTC ^= (1 << LED1);
    _delay_ms(50);
    PORTC ^= (1 << LED1);
    _delay_ms(50);
    PORTC ^= (1 << LED1);
    _delay_ms(50);
}

int main() {
    init();
    
    twinkling_led();

    wdt_reset();

    cmd_AT();
    cmd_AT_CIPCLOSE();
    cmd_AT_CIPSTART();

    wdt_reset();

    while (1) {
        if (uart_begin != uart_end)
            handle_tcp_recv();

        if (send_data_begin != send_data_end) {
            cmd_AT_CIPSEND(send_data_end);
        }

        wdt_reset();
    }
    return 0;
}
