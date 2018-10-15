import subprocess as s
import sys
from itertools import chain

def main():
    if len(sys.argv) != 6:
        print("Invalid parameter length\nExpecting 5 parameters: [delay] [loss] [duplicate] [corrupt] [reorder]")
        return
    # Order of parameters: delay,loss,duplicate,corrput,reorder
    parameters = ['delay ' + sys.argv[1],'loss ' + sys.argv[2],'duplicate ' + sys.argv[3],'corrupt ' + sys.argv[4],'reorder ' + sys.argv[5]]
    temp = [parameter.split(" ") for parameter in parameters]
    temp = list(filter(lambda parameter: int(parameter[1]) > 0,temp))
    for i in range(0,len(temp)):
        if temp[i][0] == 'delay':
            temp[i][1] += 'ms'
        else:
            temp[i][1] += '%'

    parameters = list(chain.from_iterable(temp))

    arguments = ['sudo','tc','qdisc','add','dev','lo','root','netem'] + parameters
    s.call(arguments)
    s.call(["./connectionTest"]);
    
main()
