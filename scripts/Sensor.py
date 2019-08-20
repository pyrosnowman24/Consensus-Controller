#!/usr/bin/env python3
import numpy as np
class Sensor:
  def sensor(self,pos):
    # return pos[0]**5+pos[1]**5
# http://delivery.acm.org/10.1145/310000/305745/p78-renka.pdf?ip=129.115.236.164&id=305745&acc=ACTIVE%20SERVICE&key=F82E6B88364EF649%2E01FBE2B8DA4426C6%2E4D4702B0C3E38B35%2E4D4702B0C3E38B35&__acm__=1565281100_395709d94b8ccff767cd9a0b75e4f645
# F1
    # term1 = 0.75 * np.exp(-np.power(9*pos[0]-2,2)/4 - np.power(9*pos[1]-2,2)/4);
    # term2 = 0.75 * np.exp(-np.power(9*pos[0]+1,2)/49 - (9*pos[1]+1)/10);
    # term3 = 0.5 * np.exp(-np.power(9*pos[0]-7,2)/4 - np.power(9*pos[1]-3,2)/4);
    # term4 = -0.2 * np.exp(-np.power(9*pos[0]-4,2) - np.power(9*pos[1]-7,2));
    # return term1+term2+term3+term4
# F8
    term1 = np.exp(-np.power(5-10*pos[0],2)/2)
    term2 = .75*np.exp(-np.power(5-10*pos[1],2)/2)
    term3 = .75*np.exp(-np.power(5-10*pos[0],2)/2)*np.exp(-np.power(5-10*pos[1],2)/2)
    return term1+term2+term3
# F10
    # term1 = np.exp(-.04*np.sqrt(np.power(80*pos[0]-40,2)+np.power(90*pos[1]-45,2)))
    # term2 = np.cos(.15*np.sqrt(np.power(80*pos[0]-40,2)+np.power(90*pos[1]-45,2)))
    # return term1*term2
# F7
    # term1 = 2*np.cos(10*pos[0])*np.sin(10*pos[1])
    # term2 = np.sin(10*pos[0]*pos[1])
    # return term1+term2
# F3
    # term1 = 1.25+np.cos(5.4*pos[1])
    # term2 = 6+6*np.power(3*pos[0]-1,2)
    # return term1/term2
