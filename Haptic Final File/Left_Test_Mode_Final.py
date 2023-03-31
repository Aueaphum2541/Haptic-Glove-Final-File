import uasyncio as asyncio
from machine import SoftSPI, Pin, SoftI2C, ADC
from hid_services import Keyboard
import time
import neopixel
from MPU6050 import accel


MBits_I2C_ADDRESS = 0x08
REG_ADD_SERVO_1 = 1
REG_ADD_SERVO_2 = 2
REG_ADD_SERVO_3 = 3
REG_ADD_SERVO_4 = 4

i2c = SoftI2C(scl=Pin(21), sda=Pin(22))
mpu = accel(i2c, addr=0x69)


def i2c_write(address, data):
    i2c.writeto(MBits_I2C_ADDRESS, bytes([address, data]))


def disable_servo(servo):
    if servo == 1000:
        i2c_write(REG_ADD_SERVO_1, 0)
        i2c_write(REG_ADD_SERVO_2, 0)
        i2c_write(REG_ADD_SERVO_3, 0)
        i2c_write(REG_ADD_SERVO_4, 0)
    else:
        i2c_write(servo, 0)


def set_servo_positions(positions):
    positions = [max(0, min(p, 180)) for p in positions]
    i2c_write(REG_ADD_SERVO_1, positions[0])
    i2c_write(REG_ADD_SERVO_2, positions[1])
    i2c_write(REG_ADD_SERVO_3, positions[2])
    i2c_write(REG_ADD_SERVO_4, positions[3])


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
        self.mpu = mpu

        
        # Initialize servo control variables
        self.servo_positions = [0, 1, 2, 3]
        self.servo_button_pin = Pin(4, Pin.IN, Pin.PULL_UP)
        self.previous_value = 0
        
         # Create ADC object for potentiometer pin
        self.pot = ADC(Pin(25))
        self.pot.atten(ADC.ATTN_11DB)
        
        # Create input Mode
        self.input_mode = "poten"
        
        # Initialize previous value and ticks
        self.previous_value = 0
#         self.previous_ticks = time.ticks_ms()
    

       
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

    async def gather_input(self):
        input_mode = "poten" # Default input mode
        self.previous_ticks = time.ticks_ms()
        while self.active:
            if self.input_mode == "poten":
            # Check potentiometer
                adc_value = self.pot.read()
                if abs(adc_value - self.previous_value) > 500:
                    current_ticks = time.ticks_ms()
                    if current_ticks - self.previous_ticks > 200:
                        self.previous_ticks = current_ticks
                        self.updated = True
                        if adc_value < 1000:
                            self.key = 0x16
                        else:
                            self.key = 0x00
                    self.previous_value = adc_value
                    
            elif self.input_mode == "imu":
            # Read z-axis acceleration value
                values = mpu.get_values()
                z_acc = int(mpu.get_values()["AcZ"])
                if z_acc > -5000:
                    current_ticks = time.ticks_ms()
                    if current_ticks - self.previous_ticks > 200:
                        self.previous_ticks = current_ticks
                        self.updated = True
                        self.key = 0x16
                    else:
                        self.key = 0x00


         # Check push button to change input mode
            if not Pin(36, Pin.IN, Pin.PULL_UP).value():
                if self.input_mode == "poten":
                    print('poten mode')
                    self.input_mode = "imu"
                    print('imu mode')
                    
                else:
                    self.input_mode = "poten"
                    
           
            # Check servo button
            if not self.servo_button_pin.value():
            # Increment servo position
                self.servo_positions[0] += 10
                self.servo_positions[1] += 10
                self.servo_positions[2] += 10
                self.servo_positions[3] += 10
            # Ensure position is within bounds
                self.servo_positions[0] = max(0, min(self.servo_positions[0], 180))
                self.servo_positions[1] = max(0, min(self.servo_positions[0], 180))
                self.servo_positions[2] = max(0, min(self.servo_positions[0], 180))
                self.servo_positions[3] = max(0, min(self.servo_positions[0], 180))
            # Set servo position
                set_servo_positions(self.servo_positions)
            # Wait a bit to debounce button
                await asyncio.sleep_ms(50) 
            
            await asyncio.sleep_ms(100)

    
    async def led_control(self):
        leds = neopixel.NeoPixel(Pin(13), 25)
        button = Pin(36, Pin.IN, Pin.PULL_UP)
        color = (0, 0, 255) # Default color is blue
        while True:
            if not button.value(): # Button is pressed
                if color == (0, 0, 255): # Blue color, switch to red
                    color = (255, 0, 0)
                else: # Red color, switch to blue
                   color = (0, 0, 255)
                leds[0] = color
                leds.write()
                time.sleep(0.5) # Delay for half a second before checking button again
            else:
                await asyncio.sleep(0.1)


    # Bluetooth device loop
    async def notify(self):
        previous_ticks = time.ticks_ms()
        while self.active:
        # If the variables changed do something depending on the device state
            if self.updated:
                current_ticks = time.ticks_ms()
                if current_ticks - previous_ticks > 200:
                    previous_ticks = current_ticks
                # If connected, set keys and notify
                # If idle, start advertising for 30s or until connected
                    if self.keyboard.get_state() is Keyboard.DEVICE_CONNECTED:
                       self.keyboard.set_keys(self.key)
                       self.keyboard.notify_hid_report()
                    elif self.keyboard.get_state() is Keyboard.DEVICE_IDLE:
                        await self.advertise_for(30)
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
            await asyncio.gather(self.advertise_for(30), self.led_control(), self.gather_input(), self.notify())


#     async def servo_control(self):
#         servo_pin = Pin(4, Pin.IN, Pin.PULL_UP)  # Button to activate servo
#         positions = [0, 0, 0, 0]  # Initial positions of servos
#         increment = 18  # Increment for each press of the button
# 
#         while True:
#             if not servo_pin.value() :
#                 positions[0] += increment  # Increase position of first servo
#                 if positions[0] > 180:
#                     positions[0] = 180
#                     increment = -18
#                 elif positions[0] < 0:
#                     positions[0] = 0
#                     increment = 18
#                 set_servo_positions(positions)
#                 await asyncio.sleep_ms(200)  # Debounce button press
#             else:
#                 await asyncio.sleep(1)  # Poll button every 10ms


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









