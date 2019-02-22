import subprocess as s
import sys
import os
from itertools import chain

def main():
    if len(sys.argv) != 6:
        print("Invalid parameter length\nExpecting 5 parameters: [delay] [loss] [duplicate] [corrupt] [reorder]")
        return
    # Order of parameters: delay,loss,duplicate,corrput,reorder
    parameters = ['delay ' + sys.argv[1],'loss ' + sys.argv[2],'duplicate ' + sys.argv[3],'corrupt ' + sys.argv[4],'reorder ' + sys.argv[5]]

    #Split them up to (name, value) in order toevaluate them
    temp = [parameter.split(" ") for parameter in parameters]

    #Took out any parameter that was 0 or less because netem would yell otherwise
    temp = list(filter(lambda parameter: int(parameter[1]) > 0,temp))

    #This code is for deleting the reorder parameter if the delay is 0 because reorder requires delay to be > 0 or else it yells
    if 'delay' not in list(chain.from_iterable(temp)):
        print("IN")
        for i in range(0,len(temp)):
            if temp[i][0] == 'reorder':
                temp.pop(i)

    #Replacing either ms to the delay parameter and a percentage to the rest
    for i in range(0,len(temp)):
        if temp[i][0] == 'delay':
            temp[i][1] += 'ms'
        else:
            temp[i][1] += '%'

    #Unpacking the temp to useable parameters to call
    parameters = list(chain.from_iterable(temp))
    
    '''
    The argument breakdown for the next line is as follows:
        sudo: Because netem requires sudo privileges
        tc: A user-space utility program used to manipulate packet control settings in the linux kernel (a.k.a Traffic Control)
        qdisc: Short for 'queueing discipline'. Basically a queue for packets that the linux kernel uses and how linux handles incoming/sending packets.
        add: The command to add a qdisc to something
        dev: Short for device. Specifying that we want to add a qdisc to a device
        lo: A specific kind of device called a "loopback device". Another name for it is localhost and it's address is "127.0.0.1". We can use it to send packets locally
        root: Saying that we want to attach this qdisc to the root of the device (The device being "lo" in this scenario)
        netem: An enhancement of the Linux traffic control facilities that allow to add delay, packet loss, duplication,reorder, and more to packets outgoing from a specific network interface (In our case, the network interface is "lo")
        
    '''

    arguments = ['sudo','tc','qdisc','add','dev','lo','root','netem'] + parameters
    #Calling the netem
    s.call(arguments)
    
main()
