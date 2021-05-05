import math
import random

class Distribution(object):

    def levy(self, scale=1):
        self.x = random.randrange(1,10) 
        pd = (scale/math.sqrt(2*math.pi*math.pow(self.x,3)))*math.exp(-scale/(2*self.x))*math.pow(self.x,-3/2)
        return pd

    def uniform(self, start=0, stop=9):
        return random.randint(start, stop)
