#include "my_api.h"

bool parse_helper(char* buf, bool has_args, char** value, size_t* i)
{
	if (has_args)
	{
		char index[3];
		strncpy(index, buf, arraySize(index) - 1);
		index[arraySize(index) - 1] = '\0';
		*i = atoi(index);
		if (*i >= 0 && *i < arraySize(adc_module_channels))
		{
			*value = buf + arraySize(index) - 1;
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