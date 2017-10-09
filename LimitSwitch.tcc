template <int channel>
LimitSwitch<channel>::LimitSwitch(int port, int pin)
: LimitSwitch_Base(port, pin, channel) {
	pinControl.setInterruptHandler(channel, IRQHandler);
}

template <int channel>
void LimitSwitch<channel>::IRQHandler(portBASE_TYPE* pxHigherPriorityTaskWoken){
	if(((LPC_GPIO_PIN_INT->FALL) >> channel) & 1){ // Buttons go low when pressed.
		xEventGroupSetBitsFromISR(getLimitSwitch()->eventGroup, (1 << channel), pxHigherPriorityTaskWoken);
	} else {
		xEventGroupClearBitsFromISR(getLimitSwitch()->eventGroup, (1 << channel));
	}
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(channel));
}