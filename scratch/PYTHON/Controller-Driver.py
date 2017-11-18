from inputs import get_gamepad
from socket import error as socket_error
import paramiko
import os
import sys

#  Here we scan for input and write it to a log file
def ButtonScan():	
	#  Dictionary for slightly better efficiency in button lookup	
	buttons = { "BTN_NORTH" : 'Y', "BTN_EAST" : 'B', "BTN_SOUTH" : 'A', "BTN_WEST" : 'X' }	
	log = open("button_log.txt", "w+")
	#  Number of times we write to the file
	#  We arbitrarily cap that at 30 here
	writes = 0
	while writes < 30:
		#  Get events from controller
		events = get_gamepad()
		for event in events:
			#  Here we print the events before anything happens
			#  Sometimes the controller has latent remaining from previous use
			#  This can mess up our log.
			#  The print statement helps flush that data out
			#  TODO: Flush latent input in a non-hackish manner
			print(event.code, event.state)
			#  We arbitrarily only care if the input is one of the four alphabetical buttons
			if event.code in buttons and event.state == 1:
				button = buttons[event.code]
				log.write("Event #%d:\t%c\n" % (writes+1,button))
				writes += 1

#  Here we are going to send the 'button_log.txt' file to our student drives hosted on our binghamton accounts
def SendFile():
	host = "bingsuns.cc.binghamton.edu"
	#  Default port for a lot of SSH uses
	port = 22
	#  PODS user ID
	#  For example, I put 'pnakoni1'
	username = "pnakoni1"
	#  PODS Password
	password = "PASSWORD"
	#  The name of the file created on your Binghamton drive
	#  Writes to home  directory
	filepath = 'rover-data.txt'
	#  Name of the button log file in the same directory as this program
	localpath = 'button_log.txt'

	#  Try to locate the host (a.k.a binghamton university)
	try:
		transport = paramiko.Transport((host, port))
	except (paramiko.SSHException, socket.error) as err:
		print("Error: Could not connect to host or port")
		print(err)
		sys.exit()
	#  Connect to your account
	try:
		transport.connect(username = username, password = password)
	except (paramiko.ssh_exception.AuthenticationException, paramiko.SSHException, socket_error) as err: 
		print(err)
		sys.exit()
	sftp = paramiko.SFTPClient.from_transport(transport)
	#  Save the file to your account
	sftp.put(localpath, filepath)

	sftp.close()
	transport.close()


if __name__ == "__main__":
	ButtonScan()
	SendFile()
