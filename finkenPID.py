class PIDController:
    'Common base class for all PID'

    def __init__(self, newP, newI, newD):
        self.p = newP
        self.i = newI
        self.d = newD
        self.e = 0
        self.e_prev = 0
        self.e_sum = 0
        
    def sign(self, x):
        return (x<0 and -1) or 1
        
    def step(self, newE, deltat):
        self.e_prev = self.e
        self.e = newE
        if self.sign(newE) == self.sign(self.e_prev):
            self.e_sum = self.e_sum + newE * deltat
        else:
            self.e_sum = 0
        #print('P:{} I:{} D:{} E:{} E_PREV:{} E_SUM:{}'.format(self.p,self.i,self.d,self.e,self.e_prev,self.e_sum))
        return (self.p*self.e + self.d*((self.e-self.e_prev)/deltat) + self.i*self.e_sum)
        
    def reset(self):
        self.e = 0
        self.e_prev = 0
        self.e_sum = 0
    
    def printValues(self):
        simxAddStatusbarMessage("error: " + e)
        simxAddStatusbarMessage("previous error: " + e_prev)
        simxAddStatusbarMessage("accumulated error: " + e_sum)
        simxAddStatusbarMessage("p: " + p)
        simxAddStatusbarMessage("i: " + i)
        simxAddStatusbarMessage("d: " + d)
