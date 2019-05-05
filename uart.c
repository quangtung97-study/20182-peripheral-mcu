#include <avr/io.h>
#define F_CPU 7372800UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <avr/boot.h>
#include <avr/eeprom.h>

#define BAUD 115200
#define UBRNUM (F_CPU / BAUD / 16 - 1)

#define LED1 3
#define LED2 4
#define LED3 5

#define RET_OK 1
#define RET_ERROR 0
#define RET_NONE 0

#define FALSE 0
#define TRUE 1

#define SEND_DATA_MASK 0x3f
#define DATA_MASK 0x7f

#define MSG_PING 2
#define MSG_REQUEST_PAGE 3
#define MSG_RECV_PAGE 4
#define MSG_START_BOOT 5

volatile uint8_t uart_buff[256];
uint8_t uart_begin = 0;
volatile uint8_t uart_end = 0;

uint8_t line[128];
uint8_t line_len = 0;

uint8_t data[128];
uint8_t data_begin = 0;
uint8_t data_end = 0;

uint8_t msg[128];
uint8_t msg_len = 0;
uint8_t msg_expected_len = 0;
uint8_t msg_is_reading_header = TRUE;

volatile uint8_t send_data[64];
uint8_t send_data_begin = 0;
volatile uint8_t send_data_end = 0;

const uint8_t CRLF[] PROGMEM = "\r\n";
const uint8_t OK[] PROGMEM = "OK\r\n";
const uint8_t ERROR[] PROGMEM = "ERROR\r\n";
const uint8_t SEND_OK[] PROGMEM = "SEND OK\r\n";
const uint8_t AT[] PROGMEM = "AT\r\n";
const uint8_t AT_CIPCLOSE[] PROGMEM = "AT+CIPCLOSE\r\n";
const uint8_t AT_CIPSTART[] PROGMEM = "AT+CIPSTART=\"TCP\",\"192.168.43.181\",5555\r\n";
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

void reset() {
    while (1) {}
}

inline void init_uart() {
    UBRRH = (uint8_t) (UBRNUM >> 8);
    UBRRL = (uint8_t) UBRNUM;
    UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
    UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0);
}

inline void revert_init_uart() {
    UBRRH = 0;
    UBRRL = 0;
    UCSRB = 0;
    UCSRC = (1 << URSEL);
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
    send_data[send_data_end] = b;
    send_data_end = (send_data_end + 1) & SEND_DATA_MASK;
}

void msg_start_boot() {
    send_byte(1);
    send_byte(MSG_START_BOOT);
}

void msg_request_page() {
    send_byte(2);
    send_byte(MSG_REQUEST_PAGE);
    send_byte(SPM_PAGESIZE);
}

inline void move_ivt_bootloader() {
    GICR = (1 << IVCE);
	GICR = (1 << IVSEL);
}

inline void move_ivt_application() {
    GICR = (1 << IVCE);
	GICR = 0;
}

inline void ping() {
    send_byte(1);
    send_byte(MSG_PING);
}

uint8_t timer0_count = 0;
ISR(TIMER0_OVF_vect) {
    timer0_count++;
    if (timer0_count == 30) {
        timer0_count = 0;
        ping();
    }
}

inline void init() {
    init_uart();

    DDRC = (1 << LED1) | (1 << LED2) | (1 << LED3);

    wdt_enable(WDTO_2S);
    wdt_reset();

    TCCR0 = 0b00000101;
    TIMSK = (1 << TOIE0);

    move_ivt_bootloader();

    sei();
}

inline void revert_init() {
    cli();

    move_ivt_application();

    TIMSK = 0;
    TCCR0 = 0;

    DDRC = 0;

    revert_init_uart();
}

void readline() {
    line_len = 0;
    line[0] = '\0';
    while (1) {
        uint8_t ch = uart_pick();
        line[line_len++] = ch;
        if (ch == '\n') {
            line[line_len] = '\0';
            return;
        }
    }
}

uint8_t read_until_ok_error() {
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
    read_until_ok_error();
}

inline void cmd_AT_CIPCLOSE() {
    uart_send_string_flash(AT_CIPCLOSE);
    read_until_ok_error();
}

inline void cmd_AT_CIPSTART() {
    uart_send_string_flash(AT_CIPSTART);
    uint8_t res = read_until_ok_error();
    if (res == RET_ERROR)
        reset();
}

uint8_t number[4];
uint8_t number_len;

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

uint8_t handle_tcp_recv();
void loop_handle_tcp_recv();

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

    loop_handle_tcp_recv();

    uint8_t ret = read_until_ok_error();
    if (ret == RET_ERROR) 
        reset();

    uart_pick(); // Character '>'
    uart_pick(); // Character ' '

    while (send_data_begin != end)
        uart_send(send_data_pick());

    while (1) {
        readline();
        if (strcmp_flash(line, SEND_OK))
            break;
    }
}

#define APP_CODE_SIZE (6 * 1024)
uint16_t page_addr = 0;

void start_app() {
	asm("eor R30, R30");
	asm("eor R31, R31");
	asm("ijmp");
}

inline void revert_init();

inline void write_spm_page() {
    eeprom_busy_wait();

    boot_page_erase(page_addr);
    boot_spm_busy_wait();

    uint8_t i;
    for (i = 0; i < SPM_PAGESIZE; i += 2) {
        uint16_t w = msg[i + 2];
        w <<= 8;
        w |= msg[i + 1];

        boot_page_fill(page_addr + i, w);
    }

    boot_page_write(page_addr);
    boot_spm_busy_wait();

    page_addr += SPM_PAGESIZE;

    if (page_addr == APP_CODE_SIZE) {
        boot_rww_enable();
        revert_init();
        start_app();
    }
    else {
        msg_request_page();
    }
}

void handle_msg() {
    uint8_t type = msg[0];
    switch (type) {
        case MSG_RECV_PAGE:
            write_spm_page();
            break;

        default:
            break;
    }
}

void handle_data() {
    uint8_t len = data_len();
    for (; len != 0; len--) {
        uint8_t e = data_pick();
        if (msg_is_reading_header) {
            msg_expected_len = e;
            msg_len = 0;
            msg_is_reading_header = FALSE;
        }
        else {
            msg[msg_len++] = e;
            if (msg_len == msg_expected_len) {
                handle_msg();
                msg_is_reading_header = TRUE;
            }
        }
    }
}

uint8_t handle_tcp_recv() {
    uint8_t ch = uart_buff[uart_begin];
    if (ch == '\r') {
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

        return RET_OK;
    }
    else {
        return RET_NONE;
    }
}

void loop_handle_tcp_recv() {
    uint8_t ret;
    do {
        while (uart_begin == uart_end) {}
        ret = handle_tcp_recv();
    } while (ret == RET_OK);
}

inline void twinkling_led() {
    PORTC ^= (1 << LED1);
    _delay_ms(100);
    PORTC ^= (1 << LED1);
    _delay_ms(100);
    PORTC ^= (1 << LED1);
    _delay_ms(100);
    PORTC ^= (1 << LED1);
    _delay_ms(100);
}

int main() {
    init();

    twinkling_led();

    wdt_reset();

    cmd_AT();
    cmd_AT_CIPCLOSE();
    cmd_AT_CIPSTART();

    PORTC |= (1 << LED2);

    wdt_reset();

    msg_start_boot();
    msg_request_page();

    wdt_reset();

    while (1) {
        if (uart_begin != uart_end) {
            handle_tcp_recv();
        }

        if (send_data_begin != send_data_end) {
            cmd_AT_CIPSEND(send_data_end);
        }

        wdt_reset();
    }
    return 0;
}
