#!/usr/bin/env python

import time
import json
import serial
import paho.mqtt.client as mqtt

subscription = "/bensRoom/hdmiSelector1"

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
	print("Connected with result code "+str(rc))

	# Subscribing in on_connect() means that if we lose the connection and
	# reconnect then subscriptions will be renewed.
	print("Subscribing to " + subscription)
	client.subscribe(subscription)
	print("Ok... Fully connected")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
	#print(msg.topic+" "+str(msg.payload))
	try:
		jsonDict = json.loads(msg.payload)
	except:
		print("Failed to parse json from payload")
		print("Message payload looked like this:\n " + msg.payload)
		return

	#print "As dict: "
	#print jsonDict
	command = jsonDict["command"]
	#print("Command is " + command)
	process_command(command)

def process_command(command):
	serial_command = ""
	if command == "next":
		serial_command = "sw +"
	elif command == "prev":
		serial_command = "sw -"
	elif command == "podOff":
		serial_command = "pod off"
	elif command == "podOn":
		serial_command = "pod on"
	else:
		# Splitting this should give us ['',<hdmi-number>]
		split_items = command.split("hdmi")
		if len(split_items) == 2:
			try:
				# Make sure this is an int
				int(split_items[1])
			except:
				print("Bad formed command, couldn't parse hdmi input number")
				return

			serial_command="sw i%02d" % (int(split_items[1]),)
	print "Writing serial command: " + serial_command
	do_serial_command(serial_command)


def do_serial_command(serial_command):
	global serial
	serial_command+='\r'
	# Make sure we don't have any leftover garbage from the last command
	serial.flushInput();
	serial.write(serial_command)
	# Read number of bytes if they exist or wait for them to arrive until timeout
	response_bytes = serial.read(len(serial_command)+11) # we are expecting " Command OK" which is 11 bytes
	response_str = response_bytes.decode("utf-8")
	if response_str.find( "Command OK", len(serial_command) ) == -1:
		print("Command failed. Got: " + response_str)

def main():
	# configure the serial connections (the parameters differs on the device you are connecting to)
	global serial
	serial = serial.Serial(
	    port='/dev/ttyUSB0',
	    baudrate=19200,
	    parity=serial.PARITY_NONE,
	    stopbits=serial.STOPBITS_ONE,
	    bytesize=serial.EIGHTBITS,
	    timeout=0.5 # Timeout for reading data off serial port
	)

	serial.isOpen()

	client = mqtt.Client()
	client.on_connect = on_connect
	client.on_message = on_message

	client.connect("192.168.1.199", 1883, 60)

	# Blocking call that processes network traffic, dispatches callbacks and
	# handles reconnecting.
	# Other loop*() functions are available that give a threaded interface and a
	# manual interface.
	client.loop_forever()



if __name__ == '__main__':
	main()
