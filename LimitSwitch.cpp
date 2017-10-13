#include "LimitSwitch.h"

EventGroupHandle_t LimitSwitch_Base::eventGroup = xEventGroupCreate();

LimitSwitch_Base::LimitSwitch_Base(int port, int pin, int channel)
: pinControl(port, pin, true, true, channel, true, channel){
	_channel = channel;
}
