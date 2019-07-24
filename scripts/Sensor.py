#!/usr/bin/env python3
import numpy as np
class Sensor:
  def sensor(self,pos):
    # return pos[0]**5+pos[1]**5
    term1 = 0.75 * np.exp(-np.power(9*pos[0]-2,2)/4 - np.power(9*pos[1]-2,2)/4);
    term2 = 0.75 * np.exp(-np.power(9*pos[0]+1,2)/49 - (9*pos[1]+1)/10);
    term3 = 0.5 * np.exp(-np.power(9*pos[0]-7,2)/4 - np.power(9*pos[1]-3,2)/4);
    term4 = -0.2 * np.exp(-np.power(9*pos[0]-4,2) - np.power(9*pos[1]-7,2));
    return term1+term2+term3+term4
