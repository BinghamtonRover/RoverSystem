import Gamepad
import client as mqtt
import struct

address = 'tcp://localhost:1833'
client = mqtt.Client('TransmitterClient')
client.connect(address)
client.loop_start()

if Gamepad.available():
	gamepad = Gamepad.Xbox360()


while True:
	eventType, control, value = gamepad.getNextEvent()
	print(eventType, control, value)
	if eventType == 'AXIS':
		if control == 'LEFT-Y':
			client.publish('ControllerData', struct.pack('cf', 'L',  value))
		elif control == 'RIGHT-Y':
			client.publish('ControllerData', struct.pack('cf', 'R', value))
