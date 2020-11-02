/*
 Name:		stm32_ads1220.ino
 Created:	31.10.2020 22:02:04
 Author:	Павел
*/

#include "my_api.h"

/* Global definitions */

ADS1220 adc_module;
USBSerial usb_serial;
RTClock rtc_clock;
HardwareTimer acquisition_timer(ACQUISITION_TIMER_NUMBER);

volatile uint8_t status = 0;

int16_t adc_module_channels[] = { MUX_AIN0_AIN1, MUX_AIN2_AIN3 }; //Internal indexes
float calibration_coefficients[] = { 1, 1 }; // V/V
float calibration_offset[] = { 0.000010, 0.000010 }; //V
int16_t adc_module_gain[] = { PGA_GAIN_1, PGA_GAIN_1 }; //Internal indexes
time_t acquisition_limit = 1; //Seconds
int16_t acquisition_speed = DR_20SPS;
uint16_t acquisition_period = 250; //mS = 4 Hz

/* ISRs */

void rtc_alarm_isr()
{
	status |= STATUS_RTC_ALARM;
}

void acq_timer_isr()
{
	status |= STATUS_TIMER_OVF;
}

/* Functions */

void reset_rtc(time_t limit)
{
	rtc_clock.setTime(0);
	rtc_clock.setAlarmTime(limit);
	status &= ~STATUS_RTC_ALARM;
}

void acquisition()
{
	//Prepare
	if (status & STATUS_ACQUISITION)
	{
		usb_serial_println("RECURSION!");
		return;
	}
	float buffer[arraySize(adc_module_channels)];
	for (uint8_t i = 0; i < arraySize(buffer); i++)
	{
		buffer[i] = ADS1220_NO_DATA;
	}
	status |= STATUS_ACQUISITION;
	reset_rtc(acquisition_limit);
	acquisition_timer.refresh();
	//Go
	while (!(status & (STATUS_STOP_REQ | STATUS_RTC_ALARM)))
	{
		status &= ~STATUS_TIMER_OVF;
		digitalWrite(PIN_SYNC_OUTPUT, HIGH);
		for (size_t i = 0; i < arraySize(adc_module_channels); i++)
		{
			adc_module.select_mux_channels(adc_module_channels[i]);
			adc_module.set_pga_gain(adc_module_gain[i]);
			adc_module.start_conversion(); //Discard the first conversion after MUX switching
			if (buffer[i] != ADS1220_NO_DATA)
			{
				buffer[i] = (buffer[i] * REFERENCE_VOLTAGE) / ADC_FULL_SCALE; //While waiting, compute last result for this channel
				buffer[i] = buffer[i] * calibration_coefficients[i] + calibration_offset[i];
			}
			usb_serial_print(adc_module_channels[i]); usb_serial_print(": "); //Print current channel constant
			adc_module.read_result_blocking();
			adc_module.start_conversion();
			if (buffer[i] != ADS1220_NO_DATA)
			{
				usb_serial_println(buffer[i], 6); //And print it (microvolts precision)
			}
			else
			{
				usb_serial_println("NOT_READY!");
			}
			buffer[i] = adc_module.read_result_blocking(); //Use second result
		}
		digitalWrite(PIN_SYNC_OUTPUT, LOW);
		//Wait for next point time
		if (usb_serial.available()) process_command(); //While waiting again, check for any commands (A = STOP in particular)
		while (!(status & STATUS_TIMER_OVF));
	}
	usb_serial_println("FINISHED.");
	status &= ~(STATUS_ACQUISITION | STATUS_STOP_REQ);
}

void process_command()
{
	static char buf[PC_BUFFER_LEN];
	static size_t len;
	len += usb_serial.readBytes(buf + len, arraySize(buf) - len);
	if (len == PC_BUFFER_LEN)
	{
		usb_serial_println("OVERFLOW!");
		usb_serial.flush();
		return;
	}
	if (len < 2) return;
	if (buf[len - 1] != '\n') return;
	len -= (buf[len - 2] == '\r') ? 2 : 1;
	bool has_args = len > 1;
	switch (*buf)
	{
	case 'A':
		if (status & STATUS_ACQUISITION)
		{
			status |= STATUS_STOP_REQ;
		}
		else
		{
			if (has_args) 
			{ 
				acquisition_limit = atoi(buf + 1);
				if (acquisition_limit > 0)
				{
					update_setting(&acquisition_limit, EEP_RANGE_ACQ_LIM);
				}
				else
				{
					load_setting(&acquisition_limit, EEP_RANGE_ACQ_LIM);
					usb_serial_println("INVALID!");
				}
			}
			status |= STATUS_START_REQ;
		}
		break;
	case 'C':
		parse_arguments_F(buf + 1, calibration_coefficients, has_args, EEP_RANGE_COEFFICIENTS);
		break;
	case 'O':
		parse_arguments_F(buf + 1, calibration_offset, has_args, EEP_RANGE_OFFSETS);
		break;
	case 'G':
		parse_arguments_I(buf + 1, adc_module_gain, has_args, EEP_RANGE_GAIN);
		break;
	case 'H':
		parse_arguments_I(buf + 1, adc_module_channels, has_args, EEP_RANGE_CHANNELS);
		break;
	case 'I':
		print_settings();
		break;
	case 'R':
		nvic_sys_reset();
		break;
	case 'F':
		EEPROM.format();
		break;
	default:
		usb_serial_println("UNKNOWN!");
		break;
	}
	usb_serial.flush();
	len = 0;
	usb_serial_println("PARSED.");
}

void print_settings()
{
	print_setting_array(adc_module_channels, "Channels (code): ");
	print_setting_array(calibration_coefficients, "Cal. coefficients (V/V): ", 6);
	print_setting_array(calibration_offset, "Cal. offsets (V): ", 6);
	print_setting_array(adc_module_gain, "Channel gain (code): ");
	print_setting(acquisition_limit, "Acq. limit (s): ");
	print_setting(acquisition_speed, "Speed (code): ");
	print_setting(acquisition_period, "Period (mS): ");
}

void load_settings()
{
	load_setting_array(adc_module_channels, EEP_RANGE_CHANNELS);
	load_setting_array(calibration_coefficients, EEP_RANGE_COEFFICIENTS);
	load_setting_array(calibration_offset, EEP_RANGE_OFFSETS);
	load_setting_array(adc_module_gain, EEP_RANGE_GAIN);
	load_setting(&acquisition_limit, EEP_RANGE_ACQ_LIM);
	load_setting(&acquisition_speed, EEP_RANGE_SPEED);
	load_setting(&acquisition_period, EEP_RANGE_ACQ_PERIOD);
}

/* Main */

void setup() {
	static_assert(MAX_CHANNELS >= CURRENT_CHANNELS, "Too many channels defined!");
	//Settings
	EEPROM.init(0x801F000, 0x801F800, 0x400); //STM32F103C8 are medium-density devices
	load_settings();
	//Hardware
	pinMode(LED_BUILTIN, OUTPUT_OPEN_DRAIN);
	digitalWrite(LED_BUILTIN, LOW);
	pinMode(PIN_SYNC_OUTPUT, OUTPUT_OPEN_DRAIN);
	adc_module.begin(PIN_ADC_CS, PIN_ADC_DRDY);
	adc_module.set_data_rate(acquisition_speed);
	rtc_clock.attachAlarmInterrupt(rtc_alarm_isr);
	acquisition_timer.attachInterrupt(0, acq_timer_isr);
	acquisition_timer.setPeriod(static_cast<uint32_t>(acquisition_period) * 1000u);
	//Wait for PC
	uint32_t i;
	for (i = 0; i < WAIT_FOR_PC; i++)
	{
		if (usb_serial) break;
		delay(1);
	}
	delay(1000);
	digitalWrite(LED_BUILTIN, HIGH);
	if (i == WAIT_FOR_PC)
	{
		nvic_sys_reset(); //If no connection has been established, reset
	}
	//Print EEPROM contents
	#ifdef EEPROM_TRACE
	load_settings(); //Once more for debug
	#endif
	usb_serial_println("Goodnight moon.");
	print_settings();
	usb_serial_println("READY...");
}

void loop() {
	process_command();
	if (status & STATUS_START_REQ)
	{
		status &= ~STATUS_START_REQ;
		acquisition();
	}
	delay(1);
}
