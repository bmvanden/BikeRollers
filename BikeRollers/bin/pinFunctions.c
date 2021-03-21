void pinMode(char port, uint8_t pin, int type) {
	switch (port) {
		case 'B':
			if (type == INPUT) {
				DDRB &= ~(1 << pin);
			} else if (type == OUTPUT) {
				DDRB |= (1 << pin);
			}
		case 'C':
			if (type == INPUT) {
				DDRC &= ~(1 << pin);
			} else if (type == OUTPUT) {
				DDRC |= (1 << pin);
			}		
		case 'D':
			if (type == INPUT) {
				DDRD &= ~(1 << pin);
				} else if (type == OUTPUT) {
				DDRD |= (1 << pin);
			}		
		default:
			return;
	}
}

int digitalRead(char port, uint8_t pin) {
	switch (port) {
		case 'B':
			return PINB & (1 << pin);
		case 'C':
			return PINC & (1 << pin);
		case 'D':
			return PIND & (1 << pin);
		default:
			return 0;
	}
}

void digitalWrite(char port, uint8_t pin, int value) {
	switch (port) {
		case 'B':
			if (value == HIGH) {
				PORTB |= (1 << pin);
				} else if (value == TOGGLE) {
				PORTB ^= (1 << pin);
				} else {
				PORTB &= ~(1 << pin);
			}
		case 'C':
			if (value == HIGH) {
				PORTC |= (1 << pin);
				} else if (value == TOGGLE) {
				PORTC ^= (1 << pin);
				} else {
				PORTC &= ~(1 << pin);
			}
		case 'D':
			if (value == HIGH) {
				PORTD |= (1 << pin);
				} else if (value == TOGGLE) {
				PORTD ^= (1 << pin);
				} else {
				PORTD &= ~(1 << pin);
			}
		default:
			return;
	}
}