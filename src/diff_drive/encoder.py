from __future__ import division

from  Encoder import Encoder as _encoder

import time 

class Encoder:
    """Monitors a single wheel encoder and accumulates delta ticks
    since the last time they were requested.
    """
    def __init__(self, pin_a, pin_b):
        self.setRange(-32768, 32767)
        self.initCount(0)
        self.isReversed = False
        self.gpio_encoder = _encoder(pin_a, pin_b)
        self.ppr = 40000  # Pulses Per Revolution of the encoder
        self.tstop = 20  # Loop execution duration (s)
        self.tsample = 0.002  # Sampling period for code execution (s)
        self.tdisp = 0.5  # Sampling period for values display (s)

    def setRange(self, low, high):
        self.range = high - low + 1
        self.lowThresh = low + self.range*30//100
        self.highThresh = low + self.range*70//100

    def initCount(self, startCount):
        self.delta = 0
        self.last = startCount

    def update(self, newCount):
        if self.last > self.highThresh and newCount < self.lowThresh:
            # Wrapped around the upper limit
            increment = newCount + self.range - self.last
        elif self.last < self.lowThresh and newCount > self.highThresh:
            # Wrapped around the lower limit
            increment = newCount - self.range - self.last
        else:
            increment = newCount - self.last

        self.delta += increment
        self.last = newCount
        
    def setReversed(self, isReversed):
        self.isReversed = isReversed

    def getDelta(self):
        delta = self.delta
        self.delta = 0
        if self.isReversed:
            return -delta
        else:
            return delta

    def getLimits(self):
        return {
            'range': self.range,
            'lowThresh': self.lowThresh,
            'highThresh': self.highThresh
        }
    
    def get_ticks(self):
        while tcurr <= tstop:
            time.sleep(tsample)
            tcurr = time.perf_counter() - tstart
            left_ticks = 360 / ppr * encoder1.read()
            right_ticks = 360 / ppr * encoder2.read()
            tprev = tcurr

