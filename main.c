#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include "lcd/hd44780.h"

#define LED_P			1
#define LED_DDR			DDRC
#define LED_PORT		PORTC

#define LED_AS_OUTPUT	( LED_DDR |= (1 << LED_P) )
#define LED_ON			( LED_PORT |= (1 << LED_P) )
#define LED_OFF			( LED_PORT &= ~(1 << LED_P) )

#define DHT11_P		5
#define DHT11_DDR	DDRC
#define DHT11_PORT	PORTC
#define DHT11_PIN	PINC

#define DHT11_AS_OUTPUT		( DHT11_DDR |= (1 << DHT11_P) )
#define DHT11_AS_INPUT		( DHT11_DDR &= ~(1 << DHT11_P) )
#define DHT11_HIGH_LEVEL	( DHT11_PORT |= (1 << DHT11_P) )
#define DHT11_LOW_LEVEL		( DHT11_PORT &= ~(1 << DHT11_P) )
#define DHT11_IS_HIGH		( DHT11_PIN & (1 << DHT11_P) )
#define DHT11_IS_LOW		( !DHT11_IS_HIGH )

#define TIMER0_PRESCALER_1MHZ	(1 << CS01) // 8 prescaler for 8MHz MCU clock gives 1MHz timer

typedef struct dht11_response
{
	uint8_t humidity;
	uint8_t temperature;
} DHT11_Measurements;

void timer0_reset(void)
{
	TCCR0B = 0;
	TCCR0B |= TIMER0_PRESCALER_1MHZ;
	TCNT0 = 0;
}

void dht11_idle(void)
{
	DHT11_AS_OUTPUT;
	DHT11_HIGH_LEVEL;
}

void dht11_start(void)
{
	DHT11_LOW_LEVEL;
	_delay_ms(20);
	DHT11_HIGH_LEVEL;
}

uint8_t dht11_get_response(uint8_t* response)
{
	for (uint8_t i = 0; i < 5; i++)
	{
		for (int8_t j = 7; j >= 0; j--)
		{
			// wait on sensor low level for ~50 us
			timer0_reset();
			while (DHT11_IS_LOW)
			{
				if (TCNT0 > 60) return 0;
			}
			
			// read bit value by high level length
			timer0_reset();
			while (DHT11_IS_HIGH)
			{
				if (TCNT0 > 100) return 0;
			}
			
			// read 0 value - high level was between ~26-28 us
			if (TCNT0 < 35)
			{
				response[i] &= ~(1 << j);
			}
			// read 1 value
			else if (TCNT0 >= 35 && TCNT0 < 100)
			{
				response[i] |= (1 << j);
			}
			else
			{
				return 0;
			}
		}
	}
	
	return 1;
}

uint8_t dht11_is_valid_response(uint8_t* response)
{
	uint8_t check = (response[0] + response[1] + response[2] + response[3]) & 0xFF;
	
	if (response[4] != check)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

uint8_t dht11_receive(DHT11_Measurements *measurements)
{
	DHT11_AS_INPUT;
	
	// wait 20-40 us for sensor to respond
	timer0_reset();
	while (DHT11_IS_HIGH)
	{
		if (TCNT0 > 50) return 0;
	}
	
	// wait on sensor low level for ~80 us
	timer0_reset();
	while (DHT11_IS_LOW)
	{
		if (TCNT0 > 90) return 0;
	}
	
	// wait on sensor high level for ~80 us
	timer0_reset();
	while (DHT11_IS_HIGH)
	{
		if (TCNT0 > 90) return 0;
	}
	
	uint8_t response[5];
	
	if (!dht11_get_response(response)) return 0;
	if (!dht11_is_valid_response(response)) return 0;
	
	measurements->humidity = response[0];
	measurements->temperature = response[2];
	
	return 1;
}

void display_temperature(DHT11_Measurements measurements) 
{
	char buffer[16];
	lcd_puts("t=");
	itoa(measurements.temperature, buffer, 10);
	lcd_puts(buffer);
	lcd_putc(0xDF);
	lcd_puts("C");
}

void display_humidity(DHT11_Measurements measurements)
{
	char buffer[16];
	lcd_puts(" rh=");
	itoa(measurements.humidity, buffer, 10);
	lcd_puts(buffer);
	lcd_puts("%");
}

int main(void)
{
	lcd_init();
	lcd_clrscr();
	lcd_puts("Wait...");
	
	_delay_ms(2000);
	
	dht11_idle();
	
	LED_AS_OUTPUT;
	
    while (1) 
    {
		DHT11_Measurements measurements;
		
		dht11_start();
		
		if (dht11_receive(&measurements))
		{
			lcd_clrscr();
			display_temperature(measurements);
			display_humidity(measurements);
		}
		
		_delay_ms(500);
		
		dht11_idle();
    }
}

