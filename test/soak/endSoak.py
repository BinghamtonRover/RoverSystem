import subprocess as s
s.call(["sudo","tc","qdisc","del","dev","lo","root","netem"])
