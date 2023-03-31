import uasyncio as asyncio
from hid_services import Keyboard
from machine import Pin, ADC
import time

class Device:
    def __init__(self, name="Mbits Keyboard L"):
        # Define state
        self.keys = []
        self.updated = False
        self.active = True

        # Create our device
        self.keyboard = Keyboard(name)
        # Set a callback function to catch changes of device state
        self.keyboard.set_state_change_callback(self.keyboard_state_callback)

        # Create ADC object for potentiometer pin
        self.pot = ADC(Pin(25))
        self.pot.atten(ADC.ATTN_11DB)

        # Initialize previous value and ticks
        self.previous_value = 0
        self.previous_ticks = time.ticks_ms()

    # Function that catches device status events
    def keyboard_state_callback(self):
        if self.keyboard.get_state() is Keyboard.DEVICE_IDLE:
            return
        elif self.keyboard.get_state() is Keyboard.DEVICE_ADVERTISING:
            return
        elif self.keyboard.get_state() is Keyboard.DEVICE_CONNECTED:
            return
        else:
            return

    def keyboard_event_callback(self, bytes):
        print("Keyboard state callback with bytes: ", bytes)

    def advertise(self):
        self.keyboard.start_advertising()

    def stop_advertise(self):
        self.keyboard.stop_advertising()

    async def advertise_for(self, seconds=100):
        self.advertise()

        while seconds > 0 and self.keyboard.get_state() is Keyboard.DEVICE_ADVERTISING:
            await asyncio.sleep(1)
            seconds -= 1

        if self.keyboard.get_state() is Keyboard.DEVICE_ADVERTISING:
            self.stop_advertise()

    # Input loop
    async def gather_input(self):
        while self.active:
            # Read the ADC value of the potentiometer pin
            adc_value = self.pot.read()

            # Debounce the reading with a minimum change of 500 and a minimum time of 200ms
            if abs(adc_value - self.previous_value) > 500:
                current_ticks = time.ticks_ms()
                if current_ticks - self.previous_ticks > 200:
                    self.previous_ticks = current_ticks
                    self.previous_value = adc_value
                    # Map the ADC value to a key value
                    if adc_value < 1000:
                        self.key = 0x16
                    else:
                        self.key = 0x00

                    self.updated = True

            await asyncio.sleep_ms(10)

    # Bluetooth device loop
    async def notify(self):
        while self.active:
            # If the variables changed do something depending on the device state
            if self.updated:
                # If connected, set keys and notify
                # If idle, start advertising for 30s or until connected
                if self.keyboard.get_state() is Keyboard.DEVICE_CONNECTED:
                    self.keyboard.set_keys(self.key)
                    self.keyboard.notify_hid_report()
                elif self.keyboard.get_state() is Keyboard.DEVICE_IDLE:
                    await self.advertise_for(100)
                self.updated = False
                self.keyboard.set_keys()
                self.keyboard.notify_hid_report()

            if self.keyboard.get_state() is Keyboard.DEVICE_CONNECTED:
                await asyncio.sleep_ms(50)
            else:
                await asyncio.sleep(2)

    async def co_start(self):
        # Start our device
        if self.keyboard.get_state() is Keyboard.DEVICE_STOPPED:
            self.keyboard.start()
            self.active = True
            await asyncio.gather(self.advertise_for(100), self.gather_input(), self.notify())

    async def co_stop(self):
        self.active = False
        self.keyboard.stop()

    def start(self):
        asyncio.run(self.co_start())

    def stop(self):
        asyncio.run(self.co_stop())

    
if __name__ == "__main__":
    d = Device()
    d.start()
   
