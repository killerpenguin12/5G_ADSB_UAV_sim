import matplotlib.pyplot as plt
import mso.propagation_param as p

class Grapher:
    def __init__(self) -> None:
        pass
    
    def guess_seq_len(self, seq):
        guess = 1
        max_len = len(seq) // 2
        for x in range(2, max_len):
            if seq[0:x] == seq[x:2*x] :
                return x
        return guess

    def graphMsos(self, *args):
        fig = plt.figure()
        for i in range(len(args)):
            plt.plot(args[i])
        if len(args) == 1:
            print("Sequence Length: ", self.guess_seq_len(args[0]))
        plt.show(block=False)

