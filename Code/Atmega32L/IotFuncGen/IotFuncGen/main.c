/*
 * IoT Dual Channel Function Generator V1.4
 * Designed by: Michael Khomyakov
 * July 2019
 * Department of Electric and Electronic Engineering
 * Ariel University
 */ 
//#include "main.h"
#include "Headers\main.h"
#include "Headers\SPIMaster.h"
#include "Headers\USART.h"
#include "Headers\LCD.h"
#include "Headers\Version.h"

void beep() { // d - f - a
		for (uint8_t ptrm = 0; ptrm < 30; ptrm++) {
			MISC_PORT |= BUZZER;
			_delay_us(426);
			MISC_PORT &= ~BUZZER;
			_delay_us(426);
		}
}

void playMelody(bool power_on) {
	if (power_on) {
		for (uint8_t ptrm = 0; ptrm < 236; ptrm++) {
			MISC_PORT |= BUZZER;
			_delay_us(426);
			MISC_PORT &= ~BUZZER;
			_delay_us(426);
		}
		for (uint16_t ptrm = 0; ptrm < 296; ptrm++) {
			MISC_PORT |= BUZZER;
			_delay_us(338);
			MISC_PORT &= ~BUZZER;
			_delay_us(338);
		}
		for (uint16_t ptrm = 0; ptrm < 704; ptrm++) {
			MISC_PORT |= BUZZER;
			_delay_us(284);
			MISC_PORT &= ~BUZZER;
			_delay_us(284);
		}
	}
	else {
		for (uint16_t ptrm = 0; ptrm < 296; ptrm++) {
			MISC_PORT |= BUZZER;
			_delay_us(338);
			MISC_PORT &= ~BUZZER;
			_delay_us(338);
		}
		for (uint8_t ptrm = 0; ptrm < 236; ptrm++) {
			MISC_PORT |= BUZZER;
			_delay_us(426);
			MISC_PORT &= ~BUZZER;
			_delay_us(426);
		}
		for (uint16_t ptrm = 0; ptrm < 352; ptrm++) {
			MISC_PORT |= BUZZER;
			_delay_us(568);
			MISC_PORT &= ~BUZZER;
			_delay_us(568);
		}
	}
}

void shutdownSequence(bool is_erase_requested) {
	beep();
	clear_LCD();
	print_LCD_line("<<Shutdown request>>", LCD_LINE_1);
	print_LCD_line("Device shutting down", LCD_LINE_2);
	print_LCD_line("in X sec            ", LCD_LINE_3);
	for (uint8_t cntx = 5; cntx > 0; cntx--) {
		print_LCD_char(cntx + '0',LCD_LINE_3, 3);
		_delay_ms(DELAY_COMMAND_MS);
	}
	//send_command_UART("SHDN\r\n");
	if (is_erase_requested) erase_EEPROM_1K();
	playMelody(false);
	DISABLE_DEVICE();
}

void Init_Timer() {
	TIMSK |= (1 << OCIE1A);
	TCNT1 = 0;
	TCCR1B |= (1 << WGM12);
	OCR1A = 1000;
	//ENABLE_TIMER();
}

void Init_Ports() {
	ENCODER_A_DDR &= ~ENCODER_A;
	ENCODER_B_DDR &= ~ENCODER_B;
	ENCODER_A_PORT |= ENCODER_A;
	ENCODER_B_PORT |= ENCODER_B;
	SPI_CE_DDR |= DACA_NCE | DACB_NCE | DACA_BIAS_NCE | DACB_BIAS_NCE | FG0_NCE | FG1_NCE;
	LCD_POT_DDR |= POT_LCD_NCE;
	LCD_CONTROL_DDR |= LCD_RS | LCD_E;
	LCD_DATA_DDR |= LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7;
	ADC_DDR &= ~VBAT_ADC & ~PWR_IND;
	SPI_DDR |= SPI_MOSI | SPI_SCK;
	SPI_DDR &= ~SPI_MISO;
	UART_DDR |= UART_TX;
	FG_SEL_DDR |= FG0_SEL | FG1_SEL;
	PB_DDR &= ~S_INT;
	MISC_DDR |= PS_HOLD | BUZZER;
}

void Init_Device() {
	Init_Ports();
	ENABLE_DEVICE();
	Init_SPI_All();
	Init_LCD();
	_delay_ms(DELAY_COMMAND_MS);
	Init_LCD_4bit();
	Init_UART();
	Init_ADC();
}

void Init_ADC() {
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN);
	ADMUX |= (1 << REFS0);
}

void Init_UI() {
	brightnessAnimation();
	setContrastLCD(100);
	LCD_logo_display();
	playMelody(true);
	_delay_ms(2 * DELAY_COMMAND_MS);
}

void selectOutputType(enum Device device, enum WaveformType waveformType) {
	switch(waveformType) {
		case SQUARE:
			if (device == FG0) FG_SEL_PORT &= ~FG0_SEL;
			else if (device == FG1) FG_SEL_PORT &= ~FG1_SEL;
			break;
		default:
			if (device == FG0) FG_SEL_PORT |= FG0_SEL;
			else if (device == FG1) FG_SEL_PORT |= FG1_SEL;
			break;			
	}
}

void setFunction(enum Device device, uint32_t waveFrequency, enum WaveformType mode) {
	Init_SPI_AD9834();
	waveFrequency *= AD9834_FREQ_FACTOR;
	uint16_t freq_reg_lsb = waveFrequency & 0x0003FFF;
	uint16_t freq_reg_msb = ((waveFrequency >> 14) & 0x0003FFF);
	uint8_t freq_reg_lsb_a = ((freq_reg_lsb >> 8) & 0x3F);
	uint8_t freq_reg_lsb_b = (freq_reg_lsb & 0xFF);
	uint8_t freq_reg_msb_a = ((freq_reg_msb >> 8) & 0x3F);
	uint8_t freq_reg_msb_b = (freq_reg_msb & 0xFF);
		
	SPI_write_16bit(AD9834_CONSECUTIVE_WRITE,0x00,device);
	SPI_write_16bit(AD9834_FREQUENCY_REGISTER_ADDR |freq_reg_lsb_a, freq_reg_lsb_b, device);
	SPI_write_16bit(AD9834_FREQUENCY_REGISTER_ADDR |freq_reg_msb_a, freq_reg_msb_b, device);
	SPI_write_16bit(AD9834_PHASE_REGISTER_ADDR ,0x00, device);
	switch(mode) {
		case SINE: SPI_write_16bit(AD9834_EXIT_RESET ,OPBITEN, device); break;
		case TRIANGLE: SPI_write_16bit(AD9834_EXIT_RESET, MODE, device); break;
		case SQUARE: SPI_write_16bit(AD9834_EXIT_RESET, OPBITEN | DIV2, device); break;
		default: SPI_write_16bit(AD9834_EXIT_RESET, SLEEP1, device); break;
		}
		selectOutputType(device, mode);
	Init_SPI_All();
}

int setAmplitude(uint16_t valueIn, enum Device device) {
	uint16_t value = 4095 - valueIn;
	uint8_t dac_msb = ((value >> 8) & 0x0F);
	uint8_t dac_lsb = value & 0xFF;
	if (device == FG0)		SPI_write_16bit(AMPLITUDE_A_ADDR | dac_msb, dac_lsb, DACA);
	else if (device == FG1) SPI_write_16bit(AMPLITUDE_B_ADDR | dac_msb, dac_lsb, DACB);
	else return -1; 
	return 0;
}

bool pollSwitch() {
	if (!(PB_PIN & S_INT)) {
		while(!(PB_PIN & S_INT));
		return true; 
	}
	else return false; 	
}

EncoderState pollEncoder() {
	EncoderState state = NONE;
	volatile bool currentB = PIN_ENCODER & (1 << ENCODER_B);
	volatile bool currentA = PIN_ENCODER & (1 << ENCODER_A);
	/* State change */
	if ((Encoder.previousA && Encoder.previousB) && (!currentA && currentB)) Encoder.encoderSeqCntCW++;
	else if ((!Encoder.previousA && Encoder.previousB) && (!currentA && !currentB)) Encoder.encoderSeqCntCW++;
	else if ((!Encoder.previousA && !Encoder.previousB) && (currentA && !currentB)) Encoder.encoderSeqCntCW++;
	else if ((Encoder.previousA && !Encoder.previousB) && (currentA && currentB)) Encoder.encoderSeqCntCW++;
	
	else if ((Encoder.previousA && Encoder.previousB) && (currentA && !currentB)) Encoder.encoderSeqCntCCW++;
	else if ((Encoder.previousA && !Encoder.previousB) && (!currentA && !currentB)) Encoder.encoderSeqCntCCW++;
	else if ((!Encoder.previousA && !Encoder.previousB) && (!currentA && currentB)) Encoder.encoderSeqCntCCW++;
	else if ((!Encoder.previousA && Encoder.previousB) && (currentA && currentB)) Encoder.encoderSeqCntCCW++;
	
	if (Encoder.encoderSeqCntCW == 4) {
		Encoder.encoderSeqCntCW = 0;
		state = CW;
	}
	else if (Encoder.encoderSeqCntCCW == 4) {
		Encoder.encoderSeqCntCCW = 0;
		state = CCW;
	}
	else state = NONE;
	
	Encoder.previousA = currentA;
	Encoder.previousB = currentB;
	_delay_us(100);
	return state;
}

void brightnessAnimation() {
	for (uint8_t ix = 0; ix < 101; ix = ix + 1) {
		setBrightnessLCD(ix);
		_delay_ms(10);
	}
}

void setContrastLCD(uint8_t value) {
	uint8_t final_value = (value * MAX_8BIT) / 100;
	SPI_write_16bit(CONTRAST_ADDR,255 - final_value,LCD_POT);
}

int setBrightnessLCD(uint8_t value) {
	if (value > 100) return -1;
	value = 75 + ((value * 25) / 100);
	uint16_t transformed_val = (value * DAC_DC_RATIO);
	uint8_t byte_a = (transformed_val >> 8) & 0x0F;
	uint8_t byte_b = transformed_val & 0xFF; 
	SPI_write_16bit(VOLUME_ADDR | byte_a ,byte_b ,DACB);
	return 0;
}

int setBiasDC(enum Device device, uint16_t value, bool sign) {
	uint8_t reg_msb = (value >> 8) & 0x0F;
	uint8_t reg_lsb = value & 0xFF;
		switch(device) {
			case FG0:
				if (sign) SPI_write_16bit(BIAS_A_NEG_ADDR | reg_msb, reg_lsb, DACA_BIAS);
				else	  SPI_write_16bit(BIAS_A_POS_ADDR | reg_msb, reg_lsb, DACA_BIAS);
				break;
			case FG1:
				if (sign) SPI_write_16bit(BIAS_B_NEG_ADDR | reg_msb, reg_lsb, DACB_BIAS);
				else	  SPI_write_16bit(BIAS_B_POS_ADDR | reg_msb, reg_lsb, DACB_BIAS);
				break;
			default: return -1;	// Wrong device
				break;
		}
	return 0;
	}

void erase_EEPROM_1K() {
	for (uint16_t ptr = 0; ptr < EEPROM_ADRESS_SPAN + 1; ptr++) eeprom_write_byte((uint8_t *)ptr, 0);
}

uint8_t getParametersLCD(uint8_t parameter) {
	uint8_t val = 0;
	if (parameter == CONTRAST) LCD.contrast = val;
	else if (parameter == BRIGHTNESS) LCD.brightness = val;
	return val;
}

void updateBatteryStatus() {
	ADMUX = 0;
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	PowerStatus.battery_voltage = ADC * BATTERY_ADC_FACTOR_A * BATTERY_ADC_FACTOR_B / 10;
}

void updateAcStatus() {
	volatile uint16_t adcx = 0;
	ADMUX |= 1;
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	adcx = ADC;
	ADCSRA |= (1 << ADSC);
	while(ADCSRA & (1 << ADSC));
	adcx = (adcx + ADC) / 2; // Double sum average
	if (adcx < POWER_ADC_THRESHOLD) PowerStatus.ac_power_PowerStatus = false;
	else PowerStatus.ac_power_PowerStatus = true;
}

void clearWaveformValues() {
	FunctionGenerator.frequency_A = 0; 
	FunctionGenerator.frequency_B = 0;
	FunctionGenerator.amplitude_A = 0; 
	FunctionGenerator.amplitude_B = 0;
	FunctionGenerator.output_type_A = OFF; 
	FunctionGenerator.output_type_B = OFF;
	FunctionGenerator.bias_A = 0; 
	FunctionGenerator.bias_B = 0;
	FunctionGenerator.bias_A_sign = POSITIVE;
	FunctionGenerator.bias_B_sign = POSITIVE;
}

void handleLCD(MainScreen screen, DisplayPointer displayPointer, bool pointerActive, bool paramActive) {
	
	if (pointerActive) {
		switch(displayPointer) {
			case PTR_NULL:
			print_LCD_char(' ', LCD_LINE_1, 0);
			print_LCD_char(' ', LCD_LINE_2, 0);
			print_LCD_char(' ', LCD_LINE_3, 0);
			print_LCD_char(' ', LCD_LINE_4, 0);
			break;
			
			case PTR_TYPE_A: case PTR_TYPE_B:
			print_LCD_char('>', LCD_LINE_1, 0);
			print_LCD_char(' ', LCD_LINE_2, 0);
			print_LCD_char(' ', LCD_LINE_3, 0);
			print_LCD_char(' ', LCD_LINE_4, 0);
			break;
			
			case PTR_AMP_A: case PTR_AMP_B:
			print_LCD_char(' ', LCD_LINE_1, 0);
			print_LCD_char('>', LCD_LINE_2, 0);
			print_LCD_char(' ', LCD_LINE_3, 0);
			print_LCD_char(' ', LCD_LINE_4, 0);
			break;
			
			case PTR_FREQ_A: case PTR_FREQ_B: case PTR_SETT:
			print_LCD_char(' ', LCD_LINE_1, 0);
			print_LCD_char(' ', LCD_LINE_2, 0);
			print_LCD_char('>', LCD_LINE_3, 0);
			print_LCD_char(' ', LCD_LINE_4, 0);
			break;
			
			case PTR_BIAS_A: case PTR_BIAS_B: case PTR_SHUTDOWN:
			print_LCD_char(' ', LCD_LINE_1, 0);
			print_LCD_char(' ', LCD_LINE_2, 0);
			print_LCD_char(' ', LCD_LINE_3, 0);
			print_LCD_char('>', LCD_LINE_4, 0);
			break;
		}
	}
	
	else if (paramActive) {
		switch(displayPointer) {
			case PTR_TYPE_A:
			switch(FunctionGenerator.output_type_A) {
				case SINE:
				UI.type_A[0] = 'S';
				UI.type_A[1] = 'I';
				UI.type_A[2] = 'N';
				break;
				
				case TRIANGLE:
				UI.type_A[0] = 'T';
				UI.type_A[1] = 'R';
				UI.type_A[2] = 'N';
				break;
				
				case SQUARE:
				UI.type_A[0] = 'S';
				UI.type_A[1] = 'Q';
				UI.type_A[2] = 'R';
				break;
				
				case DC:
				UI.type_A[0] = ' ';
				UI.type_A[1] = 'D';
				UI.type_A[2] = 'C';
				break;
				
				case OFF:
				UI.type_A[0] = 'O';
				UI.type_A[1] = 'F';
				UI.type_A[2] = 'F';
				break;
		}
		
			case PTR_TYPE_B:
			switch(FunctionGenerator.output_type_B) {
				case SINE:
				UI.type_B[0] = 'S';
				UI.type_B[1] = 'I';
				UI.type_B[2] = 'N';
				break;
				
				case TRIANGLE:
				UI.type_B[0] = 'T';
				UI.type_B[1] = 'R';
				UI.type_B[2] = 'N';
				break;
				
				case SQUARE:
				UI.type_B[0] = 'S';
				UI.type_B[1] = 'Q';
				UI.type_B[2] = 'R';
				break;
				
				case DC:
				UI.type_B[0] = ' ';
				UI.type_B[1] = 'D';
				UI.type_B[2] = 'C';
				break;
				
				case OFF:
				UI.type_B[0] = 'O';
				UI.type_B[1] = 'F';
				UI.type_B[2] = 'F';
				break;
			}
			
			print_LCD_char(UI.type_B[0], LCD_LINE_1, 17);
			print_LCD_char(UI.type_B[1], LCD_LINE_1, 18);
			print_LCD_char(UI.type_B[2], LCD_LINE_1, 19);
			break;
			
			case PTR_AMP_A:
			uintToString(FunctionGenerator.amplitude_A, &UI.amplitude_A[0], 2);
			print_LCD_char(UI.amplitude_A[0], LCD_LINE_2, 14);
			print_LCD_char(UI.amplitude_A[1], LCD_LINE_2, 15);
			break;
				
			case PTR_AMP_B:
			uintToString(FunctionGenerator.amplitude_B, &UI.amplitude_B[0], 2);
			print_LCD_char(UI.amplitude_B[0], LCD_LINE_2, 14);
			print_LCD_char(UI.amplitude_B[1], LCD_LINE_2, 15);
			break;
			
			case PTR_FREQ_A:
			uintToString(FunctionGenerator.frequency_A, &UI.frequency_A[0], 7);
			print_LCD_char(UI.frequency_A[0], LCD_LINE_3, 7);
			print_LCD_char(UI.frequency_A[1], LCD_LINE_3, 9);
			print_LCD_char(UI.frequency_A[2], LCD_LINE_3, 10);
			print_LCD_char(UI.frequency_A[3], LCD_LINE_3, 11);
			print_LCD_char(UI.frequency_A[4], LCD_LINE_3, 13);
			print_LCD_char(UI.frequency_A[5], LCD_LINE_3, 14);
			print_LCD_char(UI.frequency_A[6], LCD_LINE_3, 15);
			break;
			
			case PTR_FREQ_B:
			uintToString(FunctionGenerator.frequency_B, &UI.frequency_B[0], 7);
			print_LCD_char(UI.frequency_B[0], LCD_LINE_3, 7);
			print_LCD_char(UI.frequency_B[1], LCD_LINE_3, 9);
			print_LCD_char(UI.frequency_B[2], LCD_LINE_3, 10);
			print_LCD_char(UI.frequency_B[3], LCD_LINE_3, 11);
			print_LCD_char(UI.frequency_B[4], LCD_LINE_3, 13);
			print_LCD_char(UI.frequency_B[5], LCD_LINE_3, 14);
			print_LCD_char(UI.frequency_B[6], LCD_LINE_3, 15);
			break;
			
			case PTR_BIAS_A: 
			if (FunctionGenerator.bias_A_sign == POSITIVE) UI.bias_A_sign = '+';
			else UI.bias_A_sign = '-';
			uintToString(FunctionGenerator.bias_A, &UI.bias_A[0], 3);
			print_LCD_char(UI.bias_A_sign, LCD_LINE_4, 7);
			print_LCD_char(UI.bias_A[0], LCD_LINE_4, 8);
			print_LCD_char(UI.bias_A[1], LCD_LINE_4, 10);
			print_LCD_char(UI.bias_A[2], LCD_LINE_4, 11);
			break;
			
			case PTR_BIAS_B:
			if (FunctionGenerator.bias_B_sign == POSITIVE) UI.bias_B_sign = '+';
			else UI.bias_B_sign = '-';
			uintToString(FunctionGenerator.bias_A, &UI.bias_B[0], 3);
			print_LCD_char(UI.bias_B_sign, LCD_LINE_4, 7);
			print_LCD_char(UI.bias_B[0], LCD_LINE_4, 8);
			print_LCD_char(UI.bias_B[1], LCD_LINE_4, 10);
			print_LCD_char(UI.bias_B[2], LCD_LINE_4, 11);
			break;
			default: break;			
			
		}
	}
	
	else {
		switch(screen) {
			case MAIN_SCREEN_A:
			print_LCD_line(LCD_MAIN_STRING_1, LCD_LINE_1);
			print_LCD_line(LCD_MAIN_STRING_2, LCD_LINE_2);
			print_LCD_line(LCD_MAIN_STRING_3, LCD_LINE_3);
			print_LCD_line(LCD_MAIN_STRING_4, LCD_LINE_4);
			
			print_LCD_char(UI.type_A[0], LCD_LINE_1, 17);
			print_LCD_char(UI.type_A[1], LCD_LINE_1, 18);
			print_LCD_char(UI.type_A[2], LCD_LINE_1, 19);
			
			print_LCD_char(UI.amplitude_A[0], LCD_LINE_2, 14);
			print_LCD_char(UI.amplitude_A[1], LCD_LINE_2, 15);
			
			print_LCD_char(UI.frequency_A[0], LCD_LINE_3, 7);
			print_LCD_char(UI.frequency_A[1], LCD_LINE_3, 9);
			print_LCD_char(UI.frequency_A[2], LCD_LINE_3, 10);
			print_LCD_char(UI.frequency_A[3], LCD_LINE_3, 11);
			print_LCD_char(UI.frequency_A[4], LCD_LINE_3, 13);
			print_LCD_char(UI.frequency_A[5], LCD_LINE_3, 14);
			print_LCD_char(UI.frequency_A[6], LCD_LINE_3, 15);
			
			print_LCD_char(UI.bias_A_sign, LCD_LINE_4, 7);
			print_LCD_char(UI.bias_A[0], LCD_LINE_4, 8);
			print_LCD_char(UI.bias_A[1], LCD_LINE_4, 10);
			print_LCD_char(UI.bias_A[2], LCD_LINE_4, 11);
			break;
			
			case MAIN_SCREEN_B:
			print_LCD_line(LCD_MAIN_STRING_1, LCD_LINE_1);
			print_LCD_line(LCD_MAIN_STRING_2, LCD_LINE_2);
			print_LCD_line(LCD_MAIN_STRING_3, LCD_LINE_3);
			print_LCD_line(LCD_MAIN_STRING_4, LCD_LINE_4);
			
			print_LCD_char(UI.type_B[0], LCD_LINE_1, 17);
			print_LCD_char(UI.type_B[1], LCD_LINE_1, 18);
			print_LCD_char(UI.type_B[2], LCD_LINE_1, 19);
			
			print_LCD_char(UI.amplitude_B[0], LCD_LINE_2, 14);
			print_LCD_char(UI.amplitude_B[1], LCD_LINE_2, 15);
			
			print_LCD_char(UI.frequency_B[0], LCD_LINE_3, 7);
			print_LCD_char(UI.frequency_B[1], LCD_LINE_3, 9);
			print_LCD_char(UI.frequency_B[2], LCD_LINE_3, 10);
			print_LCD_char(UI.frequency_B[3], LCD_LINE_3, 11);
			print_LCD_char(UI.frequency_B[4], LCD_LINE_3, 13);
			print_LCD_char(UI.frequency_B[5], LCD_LINE_3, 14);
			print_LCD_char(UI.frequency_B[6], LCD_LINE_3, 15);
			
			print_LCD_char(UI.bias_B_sign, LCD_LINE_4, 7);
			print_LCD_char(UI.bias_B[0], LCD_LINE_4, 8);
			print_LCD_char(UI.bias_B[1], LCD_LINE_4, 10);
			print_LCD_char(UI.bias_B[2], LCD_LINE_4, 11);
			break;
			
			case PARAMS_SCREEN:
			print_LCD_line(LCD_MAIN_SETTINGS_STRING_1, LCD_LINE_1);
			print_LCD_line(LCD_MAIN_SETTINGS_STRING_2, LCD_LINE_2);
			print_LCD_line(LCD_MAIN_SETTINGS_STRING_3, LCD_LINE_3);
			print_LCD_line(LCD_MAIN_SETTINGS_STRING_4, LCD_LINE_4);
			
			uintToString(PowerStatus.battery_voltage, UI.batteryPowerStatus, 3);
			print_LCD_char(UI.batteryPowerStatus[0], LCD_LINE_3, 13);
			print_LCD_char(UI.batteryPowerStatus[1], LCD_LINE_3, 15);
			print_LCD_char(UI.batteryPowerStatus[2], LCD_LINE_3, 16);
			
			if (PowerStatus.ac_power_PowerStatus) {
				print_LCD_char('O', LCD_LINE_3, 16);
				print_LCD_char('N', LCD_LINE_3, 17);
				print_LCD_char(' ', LCD_LINE_3, 18);
			}
			
			else {
				print_LCD_char('O', LCD_LINE_3, 16);
				print_LCD_char('F', LCD_LINE_3, 17);
				print_LCD_char('F', LCD_LINE_3, 18);
			}
			break;
			
			default: break;
		}					
	}
}

void handleFunctionGenerator(DisplayPointer displayPointer) {
	switch(displayPointer) {
		case PTR_TYPE_A: case PTR_FREQ_A: setFunction(FG0, FunctionGenerator.frequency_A, FunctionGenerator.output_type_A); break;
		case PTR_TYPE_B: case PTR_FREQ_B: setFunction(FG1, FunctionGenerator.frequency_B, FunctionGenerator.output_type_B); break;
		case PTR_AMP_A: setAmplitude(FunctionGenerator.amplitude_A * MAX_12BIT / 70, FG0); break;
		case PTR_AMP_B: setAmplitude(FunctionGenerator.amplitude_B * MAX_12BIT / 70, FG1); break;
		case PTR_BIAS_A: 
			if (FunctionGenerator.bias_A_sign == POSITIVE) setBiasDC(FG0, 0, NEGATIVE);
			else setBiasDC(FG0, 0, POSITIVE);
			setBiasDC(FG0, FunctionGenerator.bias_A * MAX_12BIT / 330, FunctionGenerator.bias_A_sign);
			break;
		case PTR_BIAS_B:
			if (FunctionGenerator.bias_B_sign == POSITIVE) setBiasDC(FG1, 0, NEGATIVE);
			else setBiasDC(FG1, 0, POSITIVE);
			setBiasDC(FG0, FunctionGenerator.bias_B * MAX_12BIT / 330, FunctionGenerator.bias_B_sign);
			break;
			
			default: break;
	}
}

void uintToString(uint32_t number, char *string, uint8_t length) {
	uint8_t iPtr = 0;
	while(iPtr < length) {
		string[length - iPtr - 1] = (number % 10) + '0';
		number /= 10;
		iPtr++;
	}
}

void LCD_logo_display() {
	char lbuff[20];
	print_LCD_line("Mobile Function Generator", LCD_LINE_1);
	snprintf(lbuff, 20, " Firmware V%c.%c.%c ", FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_BUILD);
	print_LCD_line(lbuff, LCD_LINE_2);
	print_LCD_line("  KhomLabs Design   ", LCD_LINE_3);
	print_LCD_line("  <Initializing...> ", LCD_LINE_4);
}

int main() {
	/* Testing definitions */
	#ifdef PRE_PROG
		ENABLE_DEVICE();
		while(1);
	#endif
	
	Screen display;
	EncoderState encoderState = NONE;
	static DisplayPointer displayPointer;
	bool switchState = false;
	
	static bool parameterSelectionActivated = false;
	static bool displayPointerActivated = false;
	static bool functionalityChanged = false;
	static uint8_t buttonPressCounter = 0;
	static uint16_t prevBatVoltage = 0;
	static bool prevAcPowerStatus = false;

	Init_Device();
	Init_UI();
	
	clear_LCD();
	print_LCD_line("	 Loading...     ", LCD_LINE_2);
	
	/* Initialization sequence */
	display.mainScreen = MAIN_SCREEN_A;
	display.stateChanged = false;
	functionalityChanged = false;
	parameterSelectionActivated = false;
	displayPointerActivated = false;
	buttonPressCounter = 0;
	clearWaveformValues();
	handleLCD(display.mainScreen, PTR_NULL, false, false);
	_delay_ms(1500);
	
	while(1) {
		
		encoderState = pollEncoder();
		switchState = pollSwitch();
		
		if (display.stateChanged) {
			display.stateChanged = false;
			handleLCD(display.mainScreen, displayPointer, displayPointerActivated, parameterSelectionActivated);
		}
		if (functionalityChanged) {
			functionalityChanged = false;
			handleFunctionGenerator(displayPointer);
		}
		
		if (!displayPointerActivated && !parameterSelectionActivated) {
			switch(display.mainScreen) {
				case MAIN_SCREEN_A:
					if (encoderState == CW) {
						display.mainScreen = MAIN_SCREEN_B;
						display.stateChanged = true;
					}
					
					else if (encoderState == CCW) {
						display.mainScreen = PARAMS_SCREEN;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = true;
						displayPointer = PTR_TYPE_A;
					}
					break;
					
				case MAIN_SCREEN_B:
					if (encoderState == CW) {
						display.mainScreen = PARAMS_SCREEN;
						display.stateChanged = true;
					}
					
					else if (encoderState == CCW) {
						display.mainScreen = MAIN_SCREEN_A;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = true;
						displayPointer = PTR_TYPE_B;
					}
					break;
					
				case PARAMS_SCREEN:
					if (encoderState == CW) {
						display.mainScreen = MAIN_SCREEN_A;
						display.stateChanged = true;
					}
					
					else if (encoderState == CCW) {
						display.mainScreen = MAIN_SCREEN_B;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = true;
						displayPointer = PTR_SETT;
						display.stateChanged = true;
					}
					updateAcStatus();
					updateBatteryStatus();
					if ((PowerStatus.battery_voltage != prevBatVoltage) || (PowerStatus.ac_power_PowerStatus != prevAcPowerStatus)) display.stateChanged = true;
					prevBatVoltage = PowerStatus.battery_voltage;
					prevAcPowerStatus = PowerStatus.ac_power_PowerStatus;
			
					break;	
					
				default: break;
					 				
			}
		}
		else if (displayPointerActivated && !parameterSelectionActivated) { 
			switch(displayPointer) {
				case PTR_NULL: break;
				/* Channel A Block */
				case PTR_TYPE_A: 
					if (encoderState == CW) {
						displayPointer = PTR_AMP_A;
						display.stateChanged = true;
					}
					else if (encoderState == CCW) {
						displayPointer = PTR_BIAS_A;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = false;
						parameterSelectionActivated = true;
						display.stateChanged = true;
					}
					break;
				case PTR_AMP_A:
					if (encoderState == CW) {
						displayPointer = PTR_FREQ_A;
						display.stateChanged = true;
					}
					else if (encoderState == CCW) {
						displayPointer = PTR_TYPE_A;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = false;
						parameterSelectionActivated = true;
						display.stateChanged = true;
					}
					break;
				case PTR_FREQ_A:
					if (encoderState == CW) {
						displayPointer = PTR_BIAS_A;
						display.stateChanged = true;
					}
					else if (encoderState == CCW) {
						displayPointer = PTR_AMP_A;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = false;
						parameterSelectionActivated = true;
						display.stateChanged = true;
					}
					break;
				case PTR_BIAS_A:
					if (encoderState == CW) {
						displayPointer = PTR_TYPE_A;
						display.stateChanged = true;
					}
					else if (encoderState == CCW) {
						displayPointer = PTR_FREQ_A;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = false;
						parameterSelectionActivated = true;
						display.stateChanged = true;
					}
					break;
				/* Channel B Block */
				case PTR_TYPE_B:
					if (encoderState == CW) {
						displayPointer = PTR_AMP_B;
						display.stateChanged = true;
					}
					else if (encoderState == CCW) {
						displayPointer = PTR_BIAS_B;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = false;
						parameterSelectionActivated = true;
						display.stateChanged = true;
					}
					break;
				case PTR_AMP_B:
					if (encoderState == CW) {
						displayPointer = PTR_FREQ_B;
						display.stateChanged = true;
					}
					else if (encoderState == CCW) {
						displayPointer = PTR_TYPE_B;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = false;
						parameterSelectionActivated = true;
						display.stateChanged = true;
					}
					break;
				case PTR_FREQ_B:
					if (encoderState == CW) {
						displayPointer = PTR_BIAS_B;
						display.stateChanged = true;
					}
					else if (encoderState == CCW) {
						displayPointer = PTR_AMP_B;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = false;
						parameterSelectionActivated = true;
						display.stateChanged = true;
					}
					break;
				case PTR_BIAS_B:
					if (encoderState == CW) {
						displayPointer = PTR_TYPE_B;
						display.stateChanged = true;
					}
					else if (encoderState == CCW) {
						displayPointer = PTR_FREQ_B;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = false;
						parameterSelectionActivated = true;
						display.stateChanged = true;
					}
					break;
				case PTR_SETT:
					if (encoderState == CW) {
						displayPointer = PTR_SHUTDOWN;
						display.stateChanged = true;
					}
					else if (encoderState == CCW) {
						displayPointer = PTR_SHUTDOWN;
						display.stateChanged = true;
					}
					else if (switchState) {
						displayPointerActivated = false;
						parameterSelectionActivated = true;
						display.stateChanged = true;
					}
					break;
				case PTR_SHUTDOWN:
					if (encoderState == CW) {
						displayPointer = PTR_SETT;
						display.stateChanged = true;
					}
					else if (encoderState == CCW) {
						displayPointer = PTR_SETT;
						display.stateChanged = true;
					}
					else if (switchState) shutdownSequence(false);
					break;
					
					default: break;
			}
		}
		else if (!displayPointerActivated && parameterSelectionActivated) {
			switch(displayPointer) {
				
				case PTR_NULL: break;
				
				case PTR_TYPE_A:
				if (encoderState == CW) {
					display.stateChanged = true;
					functionalityChanged = true;
					switch(FunctionGenerator.output_type_A) {
						case OFF: FunctionGenerator.output_type_A = SINE; break;
						case SINE: FunctionGenerator.output_type_A = TRIANGLE; break;
						case TRIANGLE: FunctionGenerator.output_type_A = SQUARE; break;
						case SQUARE: FunctionGenerator.output_type_A = DC; break;
						case DC: default: FunctionGenerator.output_type_A = OFF; break;
					}
				}
				else if (encoderState == CCW) {
					display.stateChanged = true;
					functionalityChanged = true;
					switch(FunctionGenerator.output_type_A) {
						case OFF: FunctionGenerator.output_type_A = DC; break;
						case SINE: FunctionGenerator.output_type_A = OFF; break;
						case TRIANGLE: FunctionGenerator.output_type_A = SINE; break;
						case SQUARE: FunctionGenerator.output_type_A = TRIANGLE; break;
						case DC: default: FunctionGenerator.output_type_A = SQUARE; break;
					}
				}
				
				else if (switchState) {
					parameterSelectionActivated = false;
					functionalityChanged = false;
					displayPointerActivated = false;
					display.stateChanged = true;
					displayPointer = PTR_NULL;
				}
				
				break;
				
				case PTR_AMP_A:
				if (encoderState == CW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.amplitude_A >= 70) FunctionGenerator.amplitude_A = 70;
					else {
						switch(buttonPressCounter) {
							case 0: FunctionGenerator.amplitude_A++; break;
							case 1: FunctionGenerator.amplitude_A += 10; break;
						}
					}
				}
				else if (encoderState == CCW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.amplitude_A <= 0) FunctionGenerator.amplitude_A = 0;
					else {
						switch(buttonPressCounter) {
							case 0: FunctionGenerator.amplitude_A--; break;
							case 1: FunctionGenerator.amplitude_A -= 10; break;
						}
					}
				}
				
				else if (switchState) {
					if (buttonPressCounter < 1) buttonPressCounter++;
					else {
						buttonPressCounter = 0;
						parameterSelectionActivated = false;
						functionalityChanged = false;
						displayPointerActivated = false;
						displayPointer = PTR_NULL;
					}
					display.stateChanged = true;
				}
				break;

				case PTR_FREQ_A:
				if (encoderState == CW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.frequency_A >= 1000000) FunctionGenerator.frequency_A = 1000000;
					else {
						switch(buttonPressCounter) {
							case 0: FunctionGenerator.frequency_A++; break;
							case 1: FunctionGenerator.frequency_A += 10; break;
							case 2: FunctionGenerator.frequency_A += 100; break;
							case 3: FunctionGenerator.frequency_A += 1000; break;
							case 4: FunctionGenerator.frequency_A += 10000; break;
							case 5: FunctionGenerator.frequency_A += 100000; break;
						}
					}
				}
				else if (encoderState == CCW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.frequency_A <= 0) FunctionGenerator.frequency_A = 0;
					else {
						switch(buttonPressCounter) {
							case 0: FunctionGenerator.frequency_A--; break;
							case 1: FunctionGenerator.frequency_A -= 10; break;
							case 2: FunctionGenerator.frequency_A -= 100; break;
							case 3: FunctionGenerator.frequency_A -= 1000; break;
							case 4: FunctionGenerator.frequency_A -= 10000; break;
							case 5: FunctionGenerator.frequency_A -= 100000; break;
						}
					}
				}
				
				else if (switchState) {
					if (buttonPressCounter < 5) buttonPressCounter++;
					else {
						buttonPressCounter = 0;
						parameterSelectionActivated = false;
						functionalityChanged = false;
						displayPointerActivated = false;
						displayPointer = PTR_NULL;
					}
					display.stateChanged = true;
				}
				break;
				
				case PTR_BIAS_A:
				if (encoderState == CW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.bias_A_sign == POSITIVE) {
						if (FunctionGenerator.bias_A >= 330) FunctionGenerator.bias_A = 330;
						else {
							switch(buttonPressCounter) {
								case 0: FunctionGenerator.bias_A++; break;
								case 1: FunctionGenerator.bias_A += 10; break;
								case 2: FunctionGenerator.bias_A += 100; break;
							}
						}
					}		
					
					else if (FunctionGenerator.bias_A_sign == NEGATIVE) {
						if (FunctionGenerator.bias_A >= 0) FunctionGenerator.bias_A_sign = POSITIVE;
						else {
							switch(buttonPressCounter) {
								case 0: FunctionGenerator.bias_A--; break;
								case 1: FunctionGenerator.bias_A -= 10; break;
								case 2: FunctionGenerator.bias_A -= 100; break;
							}
						}
					}
				}
				
				else if (encoderState == CCW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.bias_A_sign == NEGATIVE) {
						if (FunctionGenerator.bias_A >= 330) FunctionGenerator.bias_A = 330;
						else {
							switch(buttonPressCounter) {
								case 0: FunctionGenerator.bias_A++; break;
								case 1: FunctionGenerator.bias_A += 10; break;
								case 2: FunctionGenerator.bias_A += 100; break;
							}
						}
					}
					else if (FunctionGenerator.bias_A_sign == POSITIVE) {
						if (FunctionGenerator.bias_A <= 0) FunctionGenerator.bias_A_sign = POSITIVE;
						else  {
							switch(buttonPressCounter) {
								case 0: FunctionGenerator.bias_A--; break;
								case 1: FunctionGenerator.bias_A -= 10; break;
								case 2: FunctionGenerator.bias_A -= 100; break;
							}
						}
					}
				}
				
				else if (switchState) {
					if (buttonPressCounter < 2) buttonPressCounter++;
					else {
						buttonPressCounter = 0;
						parameterSelectionActivated = false;
						functionalityChanged = false;
						displayPointerActivated = false;
						displayPointer = PTR_NULL;
					}
					display.stateChanged = true;
				}
				break;
				
				case PTR_TYPE_B:
				if (encoderState == CW) {
					display.stateChanged = true;
					functionalityChanged = true;
					switch(FunctionGenerator.output_type_B) {
						case OFF: FunctionGenerator.output_type_B = SINE; break;
						case SINE: FunctionGenerator.output_type_B = TRIANGLE; break;
						case TRIANGLE: FunctionGenerator.output_type_B = SQUARE; break;
						case SQUARE: FunctionGenerator.output_type_B = DC; break;
						case DC: default: FunctionGenerator.output_type_B = OFF; break;
					}
				}
				else if (encoderState == CCW) {
					display.stateChanged = true;
					functionalityChanged = true;
					switch(FunctionGenerator.output_type_B) {
						case OFF: FunctionGenerator.output_type_B = DC; break;
						case SINE: FunctionGenerator.output_type_B = OFF; break;
						case TRIANGLE: FunctionGenerator.output_type_B = SINE; break;
						case SQUARE: FunctionGenerator.output_type_B = TRIANGLE; break;
						case DC: default: FunctionGenerator.output_type_B = SQUARE; break;
					}
				}
				
				else if (switchState) {
					parameterSelectionActivated = false;
					functionalityChanged = false;
					displayPointerActivated = false;
					display.stateChanged = true;
					displayPointer = PTR_NULL;
				}
				
				break;
				
				case PTR_AMP_B:
				if (encoderState == CW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.amplitude_B >= 70) FunctionGenerator.amplitude_B = 70;
					else {
						switch(buttonPressCounter) {
							case 0: FunctionGenerator.amplitude_B++; break;
							case 1: FunctionGenerator.amplitude_B += 10; break;
						}
					}
				}
				else if (encoderState == CCW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.amplitude_B <= 0) FunctionGenerator.amplitude_B = 0;
					else {
						switch(buttonPressCounter) {
							case 0: FunctionGenerator.amplitude_B--; break;
							case 1: FunctionGenerator.amplitude_B -= 10; break;
						}
					}
				}
				
				else if (switchState) {
					if (buttonPressCounter < 1) buttonPressCounter++;
					else {
						buttonPressCounter = 0;
						parameterSelectionActivated = false;
						functionalityChanged = false;
						displayPointerActivated = false;
						displayPointer = PTR_NULL;
					}
					display.stateChanged = true;
				}
				
				break;

				case PTR_FREQ_B:
				if (encoderState == CW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.frequency_B >= 1000000) FunctionGenerator.frequency_B = 1000000;
					else {
						switch(buttonPressCounter) {
							case 0: FunctionGenerator.frequency_B++; break;
							case 1: FunctionGenerator.frequency_B += 10; break;
							case 2: FunctionGenerator.frequency_B += 100; break;
							case 3: FunctionGenerator.frequency_B += 1000; break;
							case 4: FunctionGenerator.frequency_B += 10000; break;
							case 5: FunctionGenerator.frequency_B += 100000; break;
						}
					}
				}
				else if (encoderState == CCW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.frequency_B <= 0) FunctionGenerator.frequency_B = 0;
					else {
						switch(buttonPressCounter) {
							case 0: FunctionGenerator.frequency_B--; break;
							case 1: FunctionGenerator.frequency_B -= 10; break;
							case 2: FunctionGenerator.frequency_B -= 100; break;
							case 3: FunctionGenerator.frequency_B -= 1000; break;
							case 4: FunctionGenerator.frequency_B -= 10000; break;
							case 5: FunctionGenerator.frequency_B -= 100000; break;
						}
					}
				}
					
				else if (switchState) {
					if (buttonPressCounter < 5) buttonPressCounter++;
					else {
						buttonPressCounter = 0;
						parameterSelectionActivated = false;
						functionalityChanged = false;
						displayPointerActivated = false;
						displayPointer = PTR_NULL;
					}
					display.stateChanged = true;
				}
				break;
					
				case PTR_BIAS_B:
				if (encoderState == CW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.bias_B_sign == POSITIVE) {
						if (FunctionGenerator.bias_B >= 330) FunctionGenerator.bias_B = 330;
						else {
							switch(buttonPressCounter) {
								case 0: FunctionGenerator.bias_B++; break;
								case 1: FunctionGenerator.bias_B += 10; break;
								case 2: FunctionGenerator.bias_B += 100; break;
							}
						}
					}
					else if (FunctionGenerator.bias_B_sign == NEGATIVE) {
						if (FunctionGenerator.bias_B >= 0) FunctionGenerator.bias_B_sign = POSITIVE;
						else {
							switch(buttonPressCounter) {
								case 0: FunctionGenerator.bias_B--; break;
								case 1: FunctionGenerator.bias_B -= 10; break;
								case 2: FunctionGenerator.bias_B -= 100; break;
							}
						}
					}
				}
							
				else if (encoderState == CCW) {
					display.stateChanged = true;
					functionalityChanged = true;
					if (FunctionGenerator.bias_B_sign == NEGATIVE) {
						if (FunctionGenerator.bias_B >= 330) FunctionGenerator.bias_B = 330;
						else {
							switch(buttonPressCounter) {
								case 0: FunctionGenerator.bias_B++; break;
								case 1: FunctionGenerator.bias_B += 10; break;
								case 2: FunctionGenerator.bias_B += 100; break;
							}
						}
					}
					else if (FunctionGenerator.bias_B_sign == POSITIVE) {
						if (FunctionGenerator.bias_B <= 0) FunctionGenerator.bias_B_sign = POSITIVE;
						else  {
							switch(buttonPressCounter) {
								case 0: FunctionGenerator.bias_B--; break;
								case 1: FunctionGenerator.bias_B -= 10; break;
								case 2: FunctionGenerator.bias_B -= 100; break;
							}
						}
					}
				}
						
				else if (switchState) {
					if (buttonPressCounter < 2) buttonPressCounter++;
					else {
						buttonPressCounter = 0;
						parameterSelectionActivated = false;
						functionalityChanged = false;
						displayPointerActivated = false;
						displayPointer = PTR_NULL;
					}
					display.stateChanged = true;
				}
				break;
				
				default: break;
			}
		}
	}
}