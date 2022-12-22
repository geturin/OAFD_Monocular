#Inexact differential PD control


class PD(object):

    def __init__(self,P,D,scal):
        self.P = P
        self.D = D
        self.scal = scal
        self.lasterror = 0
    
    def ctrl(self,error):
        self.de = self.D*(error-self.lasterror)
        result = self.P*error + self.de
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
        self.dev = self.D*(1-self.alpha)*(error-self.lasterror)+self.alpha*self.lastdev

        result = self.P*error+self.dev
        result *= self.scal

        self.lasterror = error
        self.lastdev = self.dev

        return result



if __name__ == "__main__":

    a = idPD(P=10,D=4,scal=1,alpha=0.15)
    b = PD(P=10, D=4, scal=1)
    while True:
        error = float(input())
        print(a.ctrl(error),b.ctrl(error))