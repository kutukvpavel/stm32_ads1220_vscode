#pragma once

#include <Arduino.h>
#include <ADS1220.h>
#include <RTClock.h>
#include <EEPROM.h>
#include <inttypes.h>

/* Constant definitions */

#define PIN_ADC_CS PA4
#define PIN_ADC_DRDY PB0
#define WAIT_FOR_PC 15000 //mS
#define PC_BUFFER_LEN 32 //bytes
#define REFERENCE_VOLTAGE 2.048 //V
#define ADC_FULL_SCALE 0x7FFFFF //3-byte-wide integer
#define ACQUISITION_TIMER_NUMBER 2
#define PIN_SYNC_OUTPUT PA3
#define MAX_CHANNELS 0xF //To preserve EEPROM layout even when the number of channels changes
#define CURRENT_CHANNELS 2

/* Globals */

extern ADS1220 adc_module;
extern USBSerial usb_serial;
extern RTClock rtc_clock;
extern HardwareTimer acquisition_timer;

#define STATUS_ACQUISITION _BV(0)
#define STATUS_RTC_ALARM _BV(1)
#define STATUS_STOP_REQ _BV(2)
#define STATUS_TIMER_OVF _BV(3)
#define STATUS_START_REQ _BV(4)
extern volatile uint8_t status;

/* EEPROM storage */

#define EEP_SIZE_RATIO(array) ( sizeof(array[0]) / sizeof(uint16_t) )
#define EEP_RANGE_CHANNELS ( MAX_CHANNELS * EEP_SIZE_RATIO(adc_module_channels) )
#define EEP_RANGE_COEFFICIENTS (EEP_RANGE_CHANNELS + MAX_CHANNELS * EEP_SIZE_RATIO(calibration_coefficients) )
#define EEP_RANGE_OFFSETS (EEP_RANGE_COEFFICIENTS + MAX_CHANNELS * EEP_SIZE_RATIO(calibration_offset) )
#define EEP_RANGE_SPEED (EEP_RANGE_OFFSETS + 1)
#define EEP_RANGE_GAIN (EEP_RANGE_SPEED + MAX_CHANNELS * EEP_SIZE_RATIO(adc_module_gain) )
#define EEP_RANGE_ACQ_LIM ( EEP_RANGE_GAIN + sizeof(acquisition_limit) / sizeof(uint16_t) )
#define EEP_RANGE_ACQ_PERIOD ( EEP_RANGE_ACQ_LIM + 1 )

extern int16_t adc_module_channels[CURRENT_CHANNELS];
extern float calibration_coefficients[CURRENT_CHANNELS];
extern float calibration_offset[CURRENT_CHANNELS];
extern int16_t adc_module_gain[CURRENT_CHANNELS];
extern time_t acquisition_limit;
extern int16_t acquisition_speed;
extern uint16_t acquisition_period;

/* Inlined */

template<typename T, size_t s> inline constexpr size_t arraySize(const T(&)[s]) { return s; }

/* Prototypes */

//USB Serial
template<typename T> void usb_serial_println(T arg, int i);
template<typename T> void usb_serial_println(T arg);
template<typename T> void usb_serial_print(T arg);
template<typename T> void usb_serial_print(T arg, int i);
//EEPROM
template<typename T> void update_setting_array(const T* array, uint16_t addr);
template<typename T> void load_setting_array(T* array, uint16_t addr);
template<typename T> void load_setting(T* value, uint16_t addr);
template<typename T> void update_setting(const T* value, uint16_t addr);
template<typename T> void print_setting_array(const T* array, const char* name, int precision = -1);
template<typename T> void print_setting(T value, const char* name);
//Commands
bool parse_helper(char* buf, bool has_args, char** value, size_t* i);
void parse_arguments_F(char* buf, float* array, bool has_args, uint16_t addr);
template<typename T> void parse_arguments_I(char* buf, T* array, bool has_args, uint16_t addr);

/* Template definitions */

/* USB Serial */

template<typename T> void usb_serial_println(T arg, int i)
{
	if (usb_serial) usb_serial.println(arg, i);
}
template<typename T> void usb_serial_println(T arg)
{
	if (usb_serial) usb_serial.println(arg);
}
template<typename T> void usb_serial_print(T arg)
{
	if (usb_serial) usb_serial.print(arg);
}
template<typename T> void usb_serial_print(T arg, int i)
{
	if (usb_serial) usb_serial.print(arg, i);
}

/* EEPROM */

template<typename T> void update_setting_array(const T* array, uint16_t addr)
{
	for (size_t i = 0; i < arraySize(adc_module_channels); i++)
	{
		update_setting(&(array[i]), addr - i * EEP_SIZE_RATIO(array));
	}
}

template<typename T> void load_setting_array(T* array, uint16_t addr)
{
	for (size_t i = 0; i < arraySize(adc_module_channels); i++)
	{
		load_setting(&(array[i]), addr - i * EEP_SIZE_RATIO(array));
	}
}

template<typename T> void load_setting(T* value, uint16_t addr)
{
	#ifdef EEPROM_TRACE
	usb_serial_println("Loading var:");
	#endif
	if (sizeof(T) > sizeof(uint16_t))
	{
		uint16_t buffer[EEP_SIZE_RATIO(value)];
		uint8_t j;
		for (j = 0; j < EEP_SIZE_RATIO(value); j++)
		{
			#ifdef EEPROM_TRACE
			usb_serial_println(addr - j);
			#endif
			auto res = EEPROM.read(addr - j, buffer + j);
			if (res != EEPROM_OK && res != EEPROM_BAD_ADDRESS)
			{
				usb_serial_println("EEPROM_READ!");
				break;
			}
		}
		if (j == EEP_SIZE_RATIO(value))
			*value = *reinterpret_cast<T *>(buffer);
	}
	else
	{
		uint16_t* buffer;
		auto res = EEPROM.read(addr, buffer);
		if (res == EEPROM_OK) *value = *reinterpret_cast<T*>(buffer);
		else if (res != EEPROM_BAD_ADDRESS) usb_serial_println("EEPROM_READ!");
		#ifdef EEPROM_TRACE
		usb_serial_println(addr);
		#endif
	}
}

template<typename T> void update_setting(const T* value, uint16_t addr)
{
	if (sizeof(T) > sizeof(uint16_t))
	{
		const uint16_t *buffer = reinterpret_cast<const uint16_t *>(value);
		for (uint8_t j = 0; j < EEP_SIZE_RATIO(value); j++)
		{
			auto res = EEPROM.write(addr - j, buffer[j]);
			if (res != EEPROM_OK) usb_serial_println("EEPROM_WRITE!");
		}
	}
	else
	{
		if (EEPROM.write(addr, *reinterpret_cast<const uint16_t *>(value)) != EEPROM_OK) usb_serial_println("EEPROM_WRITE!");
	}
}

template<typename T> void print_setting_array(const T* array, const char* name, int precision = -1)
{
	usb_serial_println(name);
	for (size_t i = 0; i < arraySize(adc_module_channels) - 1; i++)
	{
		precision > -1 ? usb_serial_print(array[i], precision) : usb_serial_print(array[i]);
		usb_serial_print(", ");
	}
	precision > -1 ? 
		usb_serial_println(array[arraySize(adc_module_channels) - 1], precision) : 
		usb_serial_println(array[arraySize(adc_module_channels) - 1]);
}

template<typename T> void print_setting(T value, const char* name)
{
	usb_serial_print(name);
	usb_serial_println(value);
}

/* Commands */

template<typename T> void parse_arguments_I(char* buf, T* array, bool has_args, uint16_t addr)
{
	char* v;
	size_t i;
	if (parse_helper(buf, has_args, &v, &i))
	{
		T c = static_cast<T>(atoi(v));
		if (c >= 0)
		{
			array[i] = c;
			update_setting_array(array, addr);
		}
		else
		{
			usb_serial_println("INVALID!");
		}
	}
}