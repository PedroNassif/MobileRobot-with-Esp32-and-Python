
from machine import Pin, Timer, PWM
from time import sleep,sleep_ms
import ubluetooth

#class classbluetooth.BLE
#BLE.active([active,]/)
#BLE.config('param',/)
#BLE.irq(handler, /)
#Frequencia definida
frequency = 1900

#Definição dos pinos para os motores
pin1 = Pin(12, Pin.OUT)
pin2 = Pin(14, Pin.OUT)
enableA = PWM(Pin(13), frequency)
pin3 = Pin(25,Pin.OUT)
pin4 = Pin(26,Pin.OUT)
enableB = PWM(Pin(27), frequency)

# Pinos do Sensor
sensor1 = Pin(4, Pin.IN)
sensor2 = Pin(32, Pin.IN)

# Variaveis GLobais
message = "Ready to start!"
mode = 0

class ESP32_BLE():
    def __init__(self, name):
        # Create internal objects for the onboard LED
        # blinking when no BLE device is connected
        # stable ON when connected
        # Pin 12 --> Bluethoot LED
        
        self.timer = Timer(0)
        self.name = name
        self.ble = ubluetooth.BLE()
        self.ble.active(True)
        self.disconnected()
        self.ble.irq(self.ble_irq)
        self.register()
        self.advertiser()
        self.connect = 0

    def connected(self):
        self.timer.deinit()
        self.connect = 1
    def disconnected(self):
        self.connect = 0
    def ble_irq(self, event, data):
        global message
        global mode
        if event == 1: #_IRQ_CENTRAL_CONNECT:
            # A central has connected to this peripheral
            self.connected()
        elif event == 2: #_IRQ_CENTRAL_DISCONNECT:
            # A central has disconnected from this peripheral.
            self.advertiser()
            self.disconnected()
        elif event == 3: #_IRQ_GATTS_WRITE:
            # A client has written to this characteristic or descriptor.
            buffer = self.ble.gatts_read(self.rx)
            message = buffer.decode('UTF-8').strip()
            if message == "!B10;":
                mode = 1
            elif message == "!B20:":
                mode = 2
            elif message == "!B309":
                mode = 3
            #print(message)
            #print(mode)
            
    def register(self):
        # Nordic UART Service (NUS)
        NUS_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E'
        RX_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E'
        TX_UUID = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E'
        BLE_NUS = ubluetooth.UUID(NUS_UUID)
        BLE_RX = (ubluetooth.UUID(RX_UUID), ubluetooth.FLAG_WRITE)
        BLE_TX = (ubluetooth.UUID(TX_UUID), ubluetooth.FLAG_NOTIFY)
        BLE_UART = (BLE_NUS, (BLE_TX, BLE_RX,))
        SERVICES = (BLE_UART, )
        ((self.tx, self.rx,), ) = self.ble.gatts_register_services(SERVICES)
    def send(self, data):
        if(self.connect):
            try:
                self.ble.gatts_notify(0, self.tx, data + '\n')
            except Exception as e:
                print('Bluetooth error:', e)
        else:
            print('Bluetooth is disconnected!')
    def advertiser(self):
        name = bytes(self.name, 'utf-8')
        adv_data = bytearray('\x02\x01\x02', 'utf-8') + bytearray((len(name) + 1, 0x09), 'utf-8') + name
        self.ble.gap_advertise(100, adv_data)
        print(adv_data)
        print("\r\n")
        
class DCMotor():
  #the min_duty and max_duty are defined for 15000Hz frequency
  #you can pass as arguments
  def __init__(self, pin1, pin2, enable_pin, min_duty=750, max_duty=1023):
    self.pin1 = pin1
    self.pin2= pin2
    self.enable_pin = enable_pin
    self.min_duty = min_duty
    self.max_duty = max_duty
  
  #speed value can be between 0 and 100
  def forward(self, speed):
    self.speed = speed
    self.enable_pin.duty(self.duty_cycle(self.speed))
    self.pin1.value(1)
    self.pin2.value(0)

  def backwards(self, speed):
    self.speed = speed
    self.enable_pin.duty(self.duty_cycle(self.speed))
    self.pin1.value(0)
    self.pin2.value(1)

  def stop(self):
    self.enable_pin.duty(0)
    self.pin1.value(0)
    self.pin2.value(0)
        
  def duty_cycle(self, speed):
    if self.speed <= 0 or self.speed > 100:
      duty_cycle = 0
    else:
      duty_cycle = int (self.min_duty + (self.max_duty - self.min_duty)*((self.speed - 1)/(100-1)))
    return duty_cycle

class car():
    def __init__(self):
        self.motor_right = DCMotor(pin1, pin2, enableA)
        self.motor_left = DCMotor(pin3, pin4, enableB)
        
    def break_car(self):
        self.motor_right.stop()
        self.motor_left.stop()
     
    #normal versions 
    def foward(self):
        self.motor_right.forward(1)
        self.motor_left.forward(1)
    
    def backward(self):
        self.motor_right.backwards(1)
        self.motor_left.backwards(1)
           
    def turn_right(self):
        self.motor_right.forward(1)
        self.motor_left.backwards(1)
        
    def turn_left(self):
        self.motor_right.backwards(1)
        self.motor_left.forward(1)
        
    #soft versions
    def foward_soft(self):
        self.motor_right.forward(50)
        self.motor_left.forward(50)
    
    def backward_soft(self):
        self.motor_right.backwards(50)
        self.motor_left.backwards(50)        
    def turn_right_soft(self):
        self.motor_right.forward(1)
        self.motor_left.forward(100)
        
    def turn_left_soft(self):
        self.motor_right.forward(100)
        self.motor_left.forward(1)
        
ble = ESP32_BLE("ESP32_caixa")
carro = car()

while True:
    while mode == 1: #mode control
        if message == "!B516":
            print("Car is going Foward")
            carro.foward()
            
        elif message == "!B615":
            print("Car is going Backwards")
            carro.backward()
            
        elif message == "!B813":
            print("Car is going to the right")
            carro.turn_right()
            
        elif message == "!B714":
            print("Car is going to the left")
            carro.turn_left()
            
        else:
            carro.break_car()
            message = " "
        
        sleep(0.15)
        
    while mode == 2: #mode autocontrol
        read1 = sensor1.value()
        read2 = sensor2.value()
        leitura = read1 + read2
        
        if leitura == 2:
            read1 = sensor1.value()
            carro.foward()
            
        elif leitura == 1:
            carro.break_car()
            read1 = sensor1.value()
            read2 = sensor2.value()
            
            if read1 == 0:
                carro.turn_left()
                sleep(1)
                carro.break_car()
                
            else:
                carro.turn_right()
                sleep(1)
                carro.break_car()
        
        else:
            if read1 == 0:
                carro.turn_left()
                sleep(1)
                carro.break_car()
                    
            elif read2 == 0:
                carro.turn_right()
                sleep(1)
                carro.break_car()
                
        sleep(0.5)
        
    while mode == 3: #mode control soft curves
        if message == "!B516":
            print("Car is going Foward")
            carro.foward_soft()
            
        elif message == "!B615":
            print("Car is going Backwards")
            carro.backward_soft()
            
        elif message == "!B813":
            print("Car is going to the right")
            carro.turn_right_soft()
            
        elif message == "!B714":
            print("Car is going to the left")
            carro.turn_left_soft()
            
        else:
            carro.break_car()
            message = " "
        
        sleep(0.15)
    sleep(0.5)

