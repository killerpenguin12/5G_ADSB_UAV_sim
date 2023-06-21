import subprocess 
import sys 
import numpy as np

x = np.linspace(1,100) 
 
for scriptInstance in [1,2,3]: 
    sys.stdout=open('result%s.txt' % scriptInstance,'w') 
    subprocess.check_call(['python3','JonathanBarrettUAT.py'], \ 
		stdout=sys.stdout, stderr=subprocess.STDOUT) 