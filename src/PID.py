

import time


class pid:
    def __init__(self,Kp,Ki,Kd):
        self.P = 0
        self.I = 0
        self.D = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_time = 0
        self.time = 0
        self.last_Error = 0
        self.Error = 0

    def update_pid(self,point,data):
        self.Error = point - data
        self.time = time.time()

        elapsed_time = int((self.time - self.last_time)*1000)
        # print(elapsed_time)
        # print("error -> %f"%self.Error)
        self.P = self.Error * self.Kp
        self.D = ((self.Error - self.last_Error)/(elapsed_time) )* self.Kd
        self.I += (self.Error ) * self.Ki

        if self.I > 10000: self.I = 10000
        if self.I < -10000: self.I = -10000

        self.last_time = self.time
        self.last_Error = self.Error
        pid = (self.P + self.I + self.D )
        # print('pid -> %d'%pid)
        if pid > 255: pid = 255
        if pid < -255: pid = -255
        return pid

    def get_term_i(self):
        return self.I

    def get_term_p(self):
        return self.P

    def get_term_d(self):
        return self.D
    def resetI(self):
         self.I = 0
