import subprocess as s
import sys
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
    
    arguments = ['sudo','tc','qdisc','add','dev','lo','root','netem'] + parameters
    #Calling the netem
    s.call(arguments)
    #Calling the program to listen in to the modified network at localhost
    s.call(["./connectionTest"]);
    
main()
