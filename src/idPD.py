#Inexact differential PD control



class PD(object):

    def __init__(self,P,D,scal):
        self.P = P
        self.D = D
        self.scal = scal
        self.lasterror = 0
    
    def ctrl(self,error):

        result = self.P*error+self.D*(error-self.lasterror)
        result *= self.scal
        self.lasterror = error
        return result


class idPD(object):

    def __init__(self,P,D,scal,alpha):
        self.P = P
        self.D = D
        self.scal = scal
        self.alpha = alpha
        self.lasterror = 0
        self.lastdev = 0

    def ctrl(self,error):
        dev = self.D*(1-self.alpha)*(error-self.lasterror)+self.alpha*self.lastdev

        result = self.P*error+dev
        result *= self.scal

        self.lasterror = error
        self.lastdev = dev

        return result



if __name__ == "__main__":

    a = idPD(P=10,D=4,scal=1,alpha=0.15)
    while True:
        error = int(input())
        print(a.ctrl(error))