#include "my_api.h"

uint8_t hex2byte(char hex);

uint8_t hex2byte(char hex) {
    if(hex > 0x39) hex -= 7; // adjust for hex letters upper or lower case
    return(hex & 0xf);
}

bool parse_helper(char* buf, bool has_args, char** value, size_t* i)
{
	if (has_args)
	{
		*i = hex2byte(buf[1]);
		if (buf[0] != ' ') *i += hex2byte(buf[0]) * 0x10;
		if (*i >= 0 && *i < arraySize(adc_module_channels))
		{
			*value = buf + 2;
			return true;
		}
		else
		{
			usb_serial_println("OUT_OF_RANGE!");
		}
	}
	else
	{
		usb_serial_println("EMPTY!");
	}
	return false;
}

void parse_arguments_F(char* buf, float* array, bool has_args, uint16_t addr)
{
	char* v;
	size_t i;
	if (parse_helper(buf, has_args, &v, &i))
	{
		float c = atof(v);
		if (c == c && c != 0)
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