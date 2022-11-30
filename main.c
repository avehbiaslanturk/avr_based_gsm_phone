#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <ctype.h>
#include <util/delay.h>
#include <string.h>
#include <avr/eeprom.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include "Font.h"
#include "softuart.h"


#define BAUD 9600
#include <util/setbaud.h>

#define MAX 64

// MACROS FOR DEBUG LED
#define DEBUG_LED_PIN PD7
#define DEBUG_LED_PORT PORTD
#define DEBUG_LED_DDR DDRD
#define DEBUG_LED_OUTPUT (DEBUG_LED_DDR |= (1 << DEBUG_LED_PIN))
#define DEBUG_LEDON (DEBUG_LED_PORT |= (1 << DEBUG_LED_PIN))
#define DEBUG_LEDOFF (DEBUG_LED_PORT &= ~(1 << DEBUG_LED_PIN))
#define DEBUG_LED_TOGGLE (DEBUG_LED_PORT ^= (1 << DEBUG_LED_PIN))
// ---------------------

// MACROS FOR LCD
#define DC PB0
#define CE PB2
#define RST PB1
#define LCD_BACKGROUND_LED_OUTPUT (DDRD |= (1 << DD6))
#define LCD_BACKGROUND_LED_ON (PORTD |= (1 << PD6))
#define LCD_BACKGROUND_LED_OFF (PORTD &= ~(1 << PD6))
//---------------

// MACROS FOR ROTARY ENCODER
#define ROT_OUTPUT_A PC3  // CLK
#define ROT_OUTPUT_B PC2  // DT
#define ROT_PORT PORTC
#define ROT_DDR DDRC
#define ROT_PIN PINC
#define ROT_BUTTON PC1
// -----------------

// MACROS FOR BUTTON
#define BUTTON PC0
#define BUTTON_PORT PORTC
#define BUTTON_DDR	DDRC
#define BUTTON_PIN	PINC
// ----------------




volatile int16_t rot_counter = 0;
volatile uint8_t a_state;
volatile uint8_t a_last_state;
volatile uint8_t rot_button_state = 1;
volatile uint8_t rot_button_pressed_now = 0;
volatile uint8_t rot_button_count_value = 0;
volatile uint8_t rot_button_last_status;

volatile uint8_t button_state = 1;
volatile uint8_t button_pressed_now = 0;
volatile uint8_t button_last_status;
volatile uint8_t button_count_value = 0;


volatile uint16_t timer_overflow_count = 0;
volatile uint16_t dummy_stopwatch = 0;
uint16_t timer_checker_for_signal_quality = 0;




volatile unsigned char incoming_number[14];
char number_to_call[14];
uint8_t number_to_call_index = 0;

volatile char signal_quality_string[5];
uint8_t signal_quality;

volatile char battery_percantage_string[5];

volatile unsigned char buffer[MAX];
volatile uint8_t indexofbuffer = 0;

uint8_t buffer_flash_to_sram[20];


volatile unsigned char content_of_sms[64];

volatile uint8_t flag_ok_status;

volatile uint8_t flag_new_sms = 0;
volatile uint8_t flag_unread_sms = 0;
volatile uint8_t flag_read_sms_complate = 0;

volatile uint8_t flag_incoming_call = 0;
volatile uint8_t flag_unanswered_call = 0;
volatile uint8_t flag_call_not_answered = 0;
volatile uint8_t flag_answered_call = 0;

volatile uint8_t flag_update_lcd = 1;
uint8_t main_menu_active = 1;
uint8_t nav_menu_active = 0;
uint8_t call_menu_active = 0;
uint8_t call_active = 0;




// EEPROM variable initialization
uint8_t EEMEM NonVolatile_Numbers[10][14];
uint8_t EEMEM eeprom_number_count;
// -----------------------------


void uart_puts(char*);
void uart_putc(unsigned char);
void lcd_write(char *);
void lcd_write_p(const char *data);
void lcd_set_colomn_row(char, char);
void lcd_clear();




void remove_spaces (char* str_trimmed, char* str_untrimmed)
{
	while (*str_untrimmed != '\0')
	{
		if(!isspace(*str_untrimmed))
		{
			*str_trimmed = *str_untrimmed;
			str_trimmed++;
		}
		str_untrimmed++;
	}
	*str_trimmed = '\0';
}


void extract_number(char *data) // Who sent an SMS ?
{
	uint8_t token_index = 0;
	char *token;
	token = strtok(data, "\"");
	token_index++;
	while(token != NULL)
	{
		token = strtok(NULL, "\"");
		token_index ++;
		if(token_index == 2)
		{
			strcpy((char *)incoming_number, token);
		}
	}
}



uint8_t get_signal_quality()
{
	indexofbuffer = 0;
	memset((char *)buffer, '\0', sizeof(buffer));
	uint8_t signal_quality;
	uart_puts("AT+CSQ\n");
	_delay_ms(100);
	signal_quality = atoi((char *)signal_quality_string);
	return signal_quality;

	// return(atoi(signal_quality_string));
}

char *get_signal_quality_string()
{
	get_signal_quality();
	return (char *)signal_quality_string;
}

void update_battery_persentage()
{
	indexofbuffer = 0;
	memset((char *)buffer, '\0', sizeof(buffer));
	uart_puts("AT+CBC\n");
	_delay_ms(100);
	// battery_percantage_string variable will be updated in ISR
}





void clean_eeprom()
{
	for(int i = 0; i < 256; i++)
		eeprom_write_byte((uint8_t*)i, '\0');
}

void clear_buffer_and_set_flags_forSMS()
{
	memset((char *)content_of_sms, '\0', sizeof(content_of_sms));
	memset((char *)incoming_number, '\0', sizeof(incoming_number));
	flag_unread_sms = 0;
}

void sms_check_and_activity()
{

	if(strstr((char *)content_of_sms,"acik"))
	{
		DEBUG_LEDON;
		lcd_clear();
		lcd_set_colomn_row(0,0);
		lcd_write_p(PSTR("LED Acildi"));
		clear_buffer_and_set_flags_forSMS();
	}

	if(strstr((char *)content_of_sms,"kapali"))
	{
		DEBUG_LEDOFF;
		lcd_clear();
		lcd_set_colomn_row(0,0);
		lcd_write_p(PSTR("LED Kapandi"));
		clear_buffer_and_set_flags_forSMS();
	}



	//icerik gelen numaradan ise ama yine gecersiz ise buffer i temizle




	clear_buffer_and_set_flags_forSMS();

}


uint8_t gsm_end_of_data(unsigned char data) // detect "end of line" from uart
{
	static uint8_t cr_flag = 0;
	static uint8_t data_step = 0;

	data_step++;

	if(data == '\r')
	{
		cr_flag = 1;
		data_step = 1;
	}

	if( (data_step == 2) && (cr_flag = 1) && (data == '\n')) //CRLF came and there is nothing between them
	{
		data_step = 0;
		cr_flag = 0;
		return 1;
	}


	else
	{
		return 0;
	}
}





void sim800_init()
{
	_delay_ms(3000);
	uart_puts("AT+CMGF=1\n"); // some settings about sms
	_delay_ms(100);
	uart_puts("AT+CNMI=1,2,0,0,0\n"); // some settings about sms
	_delay_ms(100);
	uart_puts("AT+CLIP=1\n"); // show calling line identity when receiving a mobile call
	_delay_ms(100);

	memset((char *)buffer, '\0', sizeof(buffer));
	indexofbuffer = 0;
}


ISR(USART_RX_vect)   // UART Rx interrupt ISR
{

	if(indexofbuffer == MAX-1)
	{
		indexofbuffer = 0;
	}

	unsigned char received = UDR0;

	if(gsm_end_of_data(received))
	{	//istenilen komutu elde etikten sonra da buffer'i temizlemem lazim

		if(flag_new_sms == 1) // Message has arrived. Copy it to the buffer.
		{
			strcpy((char *)content_of_sms, (char *)buffer);
			flag_new_sms = 0;
			flag_unread_sms = 1;
			indexofbuffer = 0;
			memset((char *)buffer, '\0', sizeof(buffer));
		}

		if((flag_incoming_call == 1) && (strstr((char *)buffer, "CLIP"))) // Call received. Extract the number
		{
			extract_number((char *)buffer);
			flag_incoming_call = 0;
			flag_unanswered_call = 1;
			flag_update_lcd = 1;
			indexofbuffer = 0;

			memset((char *)buffer, '\0', sizeof(buffer));
		}


		if(strstr((char *)buffer, "CMT")) // There is a new message
		{
			extract_number((char *)buffer);
			flag_new_sms = 1;
			indexofbuffer = 0;
			memset((char *)buffer, '\0', sizeof(buffer));
		}

		if(strstr((char *)buffer, "RING")) // There is a new incomming call
		{
			extract_number((char *)buffer); // Probably it is not needed
			flag_incoming_call = 1;
			indexofbuffer = 0;
			memset((char *)buffer, '\0', sizeof(buffer));
		}

		if(strstr((char *)buffer, "NO CARRIER")) // Call has not been answered
		{
			flag_call_not_answered = 1;
			indexofbuffer = 0;
			memset((char *)buffer, '\0', sizeof(buffer));
		}



		if(strstr((char *)buffer, "CSQ")) // get and parse signal quality value
		{
			char *token;
			token = strtok((char *)buffer, ":");
			token = strtok(NULL, ":");
			token = strtok(token, ",");
			remove_spaces((char *)signal_quality_string, token);
			indexofbuffer = 0;
			memset((char *)buffer, '\0', sizeof(buffer));

		}

		if(strstr((char *)buffer, "CBC")) // get and parse battery status value
		{
			char *token = NULL;
			token = strtok((char *)buffer, ",");
			token = strtok(NULL, ",");
			strcpy((char *)battery_percantage_string, token);
			indexofbuffer = 0;
			memset((char *)buffer, '\0', sizeof(buffer));
		}



		memset((char *)buffer, '\0', sizeof(buffer));
		indexofbuffer = 0;



	}

	else // save uart byte to the buffer
	{
		buffer[indexofbuffer] = received;
		indexofbuffer += 1;
	}

}
void check_rotary();
void check_buttons();
void timer2_setup();


ISR(TIMER2_OVF_vect)
{
	check_rotary();
	check_buttons();
	timer_overflow_count ++;
	if(timer_overflow_count == 122) // It is about 1 second with this timer settings
	{
		dummy_stopwatch ++;
		timer_overflow_count = 0;
	}

}



// -------------- UART Functions START --------------------

void uart_putc(unsigned char data)
{
	// wait for transmit buffer to be empty
	while(!(UCSR0A & (1 << UDRE0)));

	// load data into transmit register
	UDR0 = data;
}

void uart_puts(char* s)
{
	// transmit character until NULL is reached
	while(*s > 0) uart_putc(*s++);
}

void uart_init()
{
	UBRR0H = UBRRH_VALUE; //calculates baud rate prescaler for the high bit
	UBRR0L = UBRRL_VALUE;// same as above, just for the low bit

#if USE_2x
	UCSR0A |= _BV(U2X0);
#else
	UCSR0A &= ~(_BV(U2X0));
#endif

	UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00);
	UCSR0B |= _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
}

// ------------ UART Funtions END ---------------

// ##############################################

// ------------ SPI Functions START -------------

void spi_master_init(void)
{
	//set MOSI, SCK and SS as output
	DDRB |= (1<<PB3)|(1<<PB5)|(1<<PB2);
	//DDRB |= (1<<PB3)|(1<<PB5);
	//set SS to high
	PORTB |= (1<<PB2);
	//enable master SPI at clock rate Fck/16
	SPCR = (1<<SPE)|(1<<MSTR) | (1 << SPR1);
}

void spi_master_send(uint8_t data)
{
	//select slave
	//PORTB &= ~(1<<PB2);

	//send data
	SPDR=data;
	//wait for transmition complete
	while (!(SPSR &(1<<SPIF)));
	//SS to high

	//PORTB |= (1<<PB2);
}

uint8_t spi_master_send_and_receive(uint8_t data)
{
	// transmit data
	SPDR = data;

	// Wait for reception complete
	while(!(SPSR & (1 << SPIF)));

	// return Data Register
	return SPDR;
}

void SPI_send_string(char *string)
{
	while(*string != '\0')
	{
		spi_master_send(*string);
		string++;
	}

}

// ------------ SPI Functions END -------------

// ############################################

// ------------ LCD Functions START -----------

char lcd_buffer[15];

void lcd_command(char data)
{
	PORTB &= ~(1 << DC);
	PORTB &= ~(1 << CE);
	spi_master_send(data);
	PORTB |= (1 << DC);
	PORTB |= (1 << CE);
}

void lcd_clear()
{
	PORTB &= ~(1 << CE);
	PORTB |= (1 << DC);
	int i = 0;
	for (i = 0; i < 504; i++)
	{
		spi_master_send(0b00000000);
	}


	PORTB &= ~(1 << DC);
	PORTB |= (1 << CE);
}

void lcd_reset()
{
	PORTB &= ~(1 << RST);
	_delay_ms(100);
	PORTB |= (1 << RST);
}

void lcd_write(char *data)
{
	PORTB |= (1 << DC);
	PORTB &= ~(1 << CE);
	uint8_t lenan = strlen(data);
	for(uint8_t g = 0; g < lenan; g++)
	{
		for(uint8_t index = 0; index < 5; index++)
		{
			spi_master_send(pgm_read_byte(&ASCII[data[g]-0x20][index]));

		}
		spi_master_send(0x00);
	}
	PORTB |= (1 << CE);
	PORTB &= ~(1 << DC);

}

void lcd_write_p(const char *data) // lcd write from flash memory, not sram
{
	strcpy_P((char *)buffer_flash_to_sram, data);
	lcd_write((char *)buffer_flash_to_sram);
}


void lcd_write_int(int16_t number)
{
	memset((char *)lcd_buffer, '\0', sizeof(lcd_buffer));
	sprintf(lcd_buffer, "%d", number);
	lcd_write(lcd_buffer);
}

void lcd_write_float(float number)
{
	memset((char *)lcd_buffer, '\0', sizeof(lcd_buffer));
	sprintf(lcd_buffer, "%f", number);
	lcd_write(lcd_buffer);

}



void lcd_set_colomn_row(char x, char y)  /* set the column and row */
{

	lcd_command((x |= (1 << 7)));
	lcd_command((y |= (1 << 6)));
}


void lcd_init()
{
	DDRB |= (1 << DC); // LCD DC output
	DDRB |= (1 << CE); // LCD CE OUTPUT
	LCD_BACKGROUND_LED_OUTPUT;
	LCD_BACKGROUND_LED_ON;

	PORTB |= (1 << CE); //
	_delay_ms(100);

	lcd_reset();  /* reset the display */
	lcd_command(0x21);  /* command set in addition mode */
	//100001
	lcd_command(0xC0);  /* set the voltage by sending C0 means VOP = 5V */
	//11000000
	lcd_command(0x07);  /* set the temp. coefficient to 3 */
	//111
	lcd_command(0x13);  /* set value of Voltage Bias System */
	//10011    bias n4
	lcd_command(0x20);  /* command set in basic mode */
	//100000
	lcd_command(0x0C);  /* display result in normal mode */
	//1100
}

// ------------ LCD Functions END -----------



// ------------ GPS Functions START ---------



// ------------ Rotary Encoder & Button Functions ---------

void rotary_setup()
{
	ROT_DDR &= ~((1 << ROT_OUTPUT_A) | (1 << ROT_OUTPUT_B) | (1 << ROT_BUTTON)); // rot pins are input
	a_last_state = ((ROT_PIN & (1 << ROT_OUTPUT_A)) >> ROT_OUTPUT_A); // read first state
	ROT_PORT |= (1 << ROT_BUTTON) | (1 << ROT_OUTPUT_A) | (1 << ROT_OUTPUT_B); // enable the pull-up resistor for rot button

}

void button_setup()
{
	BUTTON_DDR &= ~(1 << BUTTON); // button pin in input
	BUTTON_PORT |= (1 << BUTTON); // enable the pull-up resistor for the button
}

void check_rotary()
{
	a_state = ((ROT_PIN & (1 << ROT_OUTPUT_A)) >> ROT_OUTPUT_A); // read a state
	if(a_state != a_last_state)
	{
		if(((ROT_PIN & (1 << ROT_OUTPUT_B)) >> ROT_OUTPUT_B) != a_state) // read b state and compare
		{																 // clockwise
			rot_counter++;
			flag_update_lcd = 1;

		}
		else
		{
			rot_counter--;
			flag_update_lcd = 1;
		}
	}
	a_last_state = a_state;

}

void check_buttons()
{
	// POLLING FOR ROT BUTTON

	rot_button_state = (ROT_PIN & (1 << ROT_BUTTON));
	if((!rot_button_state) && (rot_button_pressed_now == 0))
	{
		rot_button_count_value ++;
		rot_button_pressed_now = 1;
		flag_update_lcd = 1;

	}

	else if(rot_button_state)
	{
		rot_button_pressed_now = 0;
	}


	// POLLING FOR BUTTON
	button_state = (BUTTON_PIN & (1 << BUTTON));
	if((!button_state) && (button_pressed_now == 0))
	{
		button_count_value ++;
		button_pressed_now = 1;
		flag_update_lcd = 1;

	}

	else if(button_state)
	{
		button_pressed_now = 0;
	}



}

uint8_t button_is_pressed()
{
	if((BUTTON_PIN & (1 << BUTTON)) != 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

uint8_t rot_button_is_pressed()
{
	if((ROT_PIN & (1 << ROT_BUTTON)) != 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}



// ------------ Rotary Encoder & Button Functions END ---------


// -------- Timer 2 Setup START ----------




void timer2_setup()
{
	// Normal mode
	TCCR2B |= (1 << CS21) | (1 << CS22); // Prescaler 256
	TIMSK2 |= (1 << TOIE2); // Timer 2 overflow interrupt enable
}



// -------- Timer 2 Setup END ------------



// ----- GPS Functions START -------


typedef struct
{
	char time[10];
	char latitude[20];
	char longitude[20];
	char altitude[10];

}gps_unparsed_packet_t;


typedef struct
{
	char hh[3];  // hour
	char mm[3];  //minute
	char time[10]; // Exact string time like 15:36
	float latitude;
	float longitude;
	int altitude;
}gps_parsed_packet_t;

gps_parsed_packet_t get_gps_data(char *buffer)
{
	// enable serial uart
	// wait until reach the GPGGA data

	gps_unparsed_packet_t unparsed_gps;
	gps_parsed_packet_t parsed_gps;

	uint8_t index = 0;


	char *data = strtok(buffer, ",");
	index ++;
	//printf("index:%d ->", index);
	//printf("%s\n", data);
	while(data != NULL)
	{
		data = strtok(NULL, ",");
		index ++;
		if(index == 2)
		{
			strcpy(unparsed_gps.time, data);
		}

		if(index == 3)
		{
			strcpy(unparsed_gps.latitude, data);
		}

		if(index == 5)
		{
			strcpy(unparsed_gps.longitude, data);
		}

		if(index == 10)
		{
			strcpy(unparsed_gps.altitude, data);
		}

		//printf("index:%d ->", index);
		//printf("%s\n", data);

	}



	sscanf(unparsed_gps.time, "%2s%2s", parsed_gps.hh, parsed_gps.mm);

	// get time data like HH:MM style
	strcat(parsed_gps.time, parsed_gps.hh);
	strcat(parsed_gps.time, ":");
	strcat(parsed_gps.time, parsed_gps.mm);

	//puts(parsed_gps.time);

	float dd_buffer = 0;
	float mm_mm_buffer = 0;
	float ddd_buffer = 0;


	// Convert lat. and long. and alt to google maps style
	sscanf(unparsed_gps.latitude, "%2f%5f", &dd_buffer, &mm_mm_buffer);
	parsed_gps.latitude = (dd_buffer + (mm_mm_buffer / 60.0));

	sscanf(unparsed_gps.longitude, "%3f%5f", &ddd_buffer, &mm_mm_buffer);
	parsed_gps.longitude = (ddd_buffer + (mm_mm_buffer / 60));


	sscanf(unparsed_gps.altitude, "%d", &parsed_gps.altitude);


	return parsed_gps;
}



// -------- Menu Functions START ----------

void main_menu1()
{
	lcd_clear();
	lcd_set_colomn_row(0, 0);
	lcd_write_p(PSTR("Pil:%"));
	lcd_write((char *)battery_percantage_string);
	lcd_set_colomn_row(0, 1);
	lcd_write_p(PSTR("Sinyal:"));
	lcd_write_int(signal_quality);
	lcd_set_colomn_row(0, 4);

	lcd_set_colomn_row(0, 5);
	lcd_write_p(PSTR("Cagri | Menu"));
}


void sub_menu_call()
{
	lcd_clear();
	lcd_set_colomn_row(0, 0);
	lcd_write_p(PSTR("2. menuyum"));

}

void nav_menu_get_location()
{
	lcd_clear();
	lcd_set_colomn_row(0, 0);
	lcd_write_p(PSTR("GPS Acil Konum"));
	lcd_set_colomn_row(0, 1);
	lcd_write_p(PSTR("Bildirimi"));

	lcd_set_colomn_row(0, 5);
	lcd_write_p(PSTR("Geri     Tamam"));

}

void nav_menu_get_weather()
{
	lcd_clear();
	lcd_set_colomn_row(0, 0);
	lcd_write_p(PSTR("Hava Durumu"));
	lcd_set_colomn_row(0, 5);
	lcd_write_p(PSTR("Geri    Tamam"));
}

void number_check_and_shift()
{
	if(rot_counter <= 9 && rot_counter >= 0)
	{
		number_to_call[number_to_call_index] = rot_counter + '0';
		lcd_set_colomn_row(number_to_call_index * 5, 1);
		lcd_write_int(rot_counter);

		if(rot_button_count_value)
		{
			number_to_call_index ++;
			rot_button_count_value = 0;
			rot_counter = 0;

		}

		if(button_count_value)
		{
			button_count_value = 0;
			flag_update_lcd = 1;

			if(number_to_call_index <= 0)
			{
				main_menu_active = 1;
				call_menu_active = 0;
				rot_button_count_value = 0;
				rot_counter = 0;
			}

			else
			{
				number_to_call_index --;
				rot_button_count_value = 0;
				button_count_value = 0;
			}

		}

	}

	else
	{
		rot_counter = 0;
		flag_update_lcd = 1;
	}
}



//char mynumber[] = "+905365147299";



// PD2 RX
// PD3 TX
//char gps_data[] = "$GPGGA,134058.00,3735.18684,N,03652.69910,E,1,05,1.83,719.9,M,27.2,M,,*5D";


int main(void)
{
	//gps_parsed_packet_t gps_parsed;
	//gps_parsed = get_gps_data(gps_data);
	uint8_t gps_buffer_index = 0;
	char gps_buffer[128];

	char c;
	softuart_init();
	softuart_turn_rx_off();
	sei();
	uart_init();
	spi_master_init();
	sim800_init();

	DEBUG_LED_OUTPUT; // enable led to debug

	//clean_eeprom();
	//save_number_toEEPROM("+905365147299");

	rotary_setup();
	button_setup();
	timer2_setup();


	lcd_init();
	lcd_clear();
	lcd_set_colomn_row(0, 0);
	lcd_write_p(PSTR("Merhaba Ahmet"));

	// Wait until the connection has established
	/*
	uint8_t signal_quality = 0;
	do {

		lcd_clear();
		lcd_set_colomn_row(0, 0);
		lcd_write_p(PSTR("Baglanti Kuruluyor..."));
		lcd_set_colomn_row(0, 3);
		lcd_write_p(PSTR("Sinyal Degeri:"));
		lcd_set_colomn_row(0, 4);
		lcd_write(get_signal_quality_string());
		lcd_set_colomn_row(15, 4);
		lcd_write_p(PSTR("rssi(0-31)"));


		_delay_ms(2000);
		signal_quality = get_signal_quality();

	} while (signal_quality < 15);
	 */

	lcd_clear();
	softuart_turn_rx_on();


	while(1)
	{

		if((uint16_t)(dummy_stopwatch - timer_checker_for_signal_quality) >= 10)
		{
			//DEBUG_LED_TOGGLE;
			timer_checker_for_signal_quality = dummy_stopwatch;
			signal_quality = get_signal_quality();
			update_battery_persentage();
			flag_update_lcd = 1;
		}



		if(flag_unread_sms)
		{
			sms_check_and_activity();
		}


		if(flag_call_not_answered)
		{
			flag_call_not_answered = 0;

			lcd_clear();
			lcd_set_colomn_row(0, 0);
			lcd_write_p(PSTR("Cagri"));
			lcd_set_colomn_row(0, 1);
			lcd_write_p(PSTR("Sonlandi"));
			_delay_ms(5000);
			main_menu_active = 1;
			rot_button_count_value = 0;
			button_count_value = 0;
			lcd_clear();

		}

		/*
		if (softuart_kbhit())
		{
			c = softuart_getchar();
			if(gps_buffer_index >= 126)
			{
				gps_buffer_index = 0;
			}

			if(c == '\n')
			{
				if(strstr((char *)gps_buffer, "$GPGGA"))
				{
					//DEBUG_LEDON;
					memset(gps_buffer, '\0', sizeof(gps_buffer));
					gps_buffer_index = 0;
				}

				if(strstr((char *)gps_buffer, "GPTXT"))
				{
					//DEBUG_LEDON;
					memset(gps_buffer, '\0', sizeof(gps_buffer));
					gps_buffer_index = 0;
				}
			}

			else
			{
				gps_buffer[gps_buffer_index] = c;
				gps_buffer_index ++;

			}

		}

		 */

		if(flag_update_lcd)
		{

			if(flag_unanswered_call)
			{
				main_menu_active = 0;
				lcd_clear();
				lcd_set_colomn_row(0, 0);
				lcd_write_p(PSTR("Cagri Aliniyor"));
				lcd_set_colomn_row(0, 2);
				lcd_write(incoming_number);
				lcd_set_colomn_row(0, 5);
				lcd_write_p(PSTR("Cevapla|Reddet"));
				if(button_count_value)
				{
					uart_puts("ATA\n");
					do
					{
						lcd_clear();
						lcd_set_colomn_row(0, 2);
						lcd_write_p(PSTR("Cagri Aktif"));
						lcd_set_colomn_row(50, 5);
						lcd_write("Kapat");

					} while (!rot_button_count_value);
					uart_puts("ATH\n");
					lcd_clear();
					lcd_set_colomn_row(0, 2);
					lcd_write_p(PSTR("Cagri Sonlandirildi"));
					rot_button_count_value = 0;
					button_count_value = 0;
					main_menu_active = 1;
					flag_update_lcd = 1;
				}

				if(rot_button_count_value)
				{
					uart_puts("ATH\n");
					rot_button_count_value = 0;
					button_count_value = 0;
					main_menu_active = 1;
					flag_update_lcd = 1;
				}

				memset((char *)incoming_number, '\0', sizeof(incoming_number));
				flag_unanswered_call = 0;

			}



			if(main_menu_active)
			{
				main_menu1();
				if(rot_button_count_value)  // Switch to nav menu
				{
					nav_menu_active = 1;
					rot_counter = 1;
					rot_button_count_value = 0;
					button_count_value = 0;
				}

				if(button_count_value)	// Switch to call menu
				{
					memset(number_to_call, '\0', sizeof(number_to_call));
					number_to_call_index = 0;
					call_menu_active = 1;
					main_menu_active = 0;
					rot_counter = 0;
					button_count_value = 0;
				}

				flag_update_lcd = 0;
			}



			if(nav_menu_active)
			{
				main_menu_active = 0;

				if(button_count_value) // Go back the main menu again
				{
					nav_menu_active = 0;
					main_menu_active = 1;
					rot_counter = 0;
					rot_button_count_value = 0;
					button_count_value = 0;
				}

				switch (rot_counter)
				{
				case 1:
					nav_menu_get_location();
					flag_update_lcd = 0;
					break;
				case 2:
					nav_menu_get_weather();
					flag_update_lcd = 0;
					break;

				default:
					rot_counter = 1;
					break;
				}
			}


			if(call_menu_active)
			{

				flag_update_lcd = 0;
				lcd_clear();
				lcd_set_colomn_row(0, 0);
				lcd_write_p(PSTR("Numara Cevir"));
				lcd_set_colomn_row(0, 5);
				lcd_write_p(PSTR("Geri  "));
				switch (number_to_call_index)
				{
				case 0:
					number_check_and_shift();
					break;
				case 1:
					number_check_and_shift();
					break;
				case 2:
					number_check_and_shift();
					break;
				case 3:
					number_check_and_shift();
					break;
				case 4:
					number_check_and_shift();
					break;
				case 5:
					number_check_and_shift();
					break;
				case 6:
					number_check_and_shift();
					break;
				case 7:
					number_check_and_shift();
					break;
				case 8:
					number_check_and_shift();
					break;
				case 9:
					number_check_and_shift();
					break;
				case 10:
					number_check_and_shift();
					lcd_set_colomn_row(50, 5);
					lcd_write_p(PSTR("Ara"));
					break;
					// number is end
				case 11:

					lcd_set_colomn_row(50, 5);
					lcd_write_p(PSTR("Ara"));
					if(rot_button_count_value)
					{
						call_active = 1;
						call_menu_active = 0;
						rot_button_count_value = 0;
						rot_counter = 0;
						button_count_value = 0;
					}
					break;



				default:

					break;
				}

				lcd_set_colomn_row(0, 1);
				lcd_write(number_to_call);


			}


			if(call_active)
			{
				lcd_clear();
				flag_update_lcd = 1;
				lcd_set_colomn_row(0, 0);
				lcd_write_p(PSTR("Numara"));
				lcd_set_colomn_row(0, 1);
				lcd_write_p(PSTR("cevriliyor"));
				lcd_set_colomn_row(0, 3);
				lcd_write(number_to_call);
				uart_puts("ATD");
				uart_puts(number_to_call);
				uart_puts(";\n");
				lcd_set_colomn_row(0, 5);
				// Numarayi cevirecek AT kodu
				lcd_write_p(PSTR("Cikis"));

				if(button_count_value)
				{
					// Cagridan cikacak AT kodu
					lcd_clear();
					lcd_set_colomn_row(0, 1);
					lcd_write("Cagri Sonlandirildi");
					uart_puts("ATH\n");
					_delay_ms(3000);
					call_active = 0;
					rot_counter = 0;
					rot_button_count_value = 0;
					button_count_value = 0;
					main_menu_active = 1;
					flag_update_lcd = 1;
				}

			}




		}




	}


}
