import numpy as np
from math import pi
import os, time, sys

old_ctrls = []
count_low = 0
count_max = 0
count_low_r = 0
count_max_r = 0
ct_ll, ct_lm, ct_rl, ct_rm = 0,0,0,0
k_max = 15#*pi/180 

U = 7
ctrl_cmd = [-k_max, -k_max*4/(U),-k_max*2/(U),0,k_max*2/(U),k_max*4/(U),k_max]
print('old',ctrl_cmd)
old_ctrls = [16,16,-16]
#4.28571
for i in range(len(old_ctrls)):
    if [0-(1e-3)] <=  np.abs(old_ctrls[i])-(1e-3) <= k_max*2/(U):
        count_low += 1 
        print(count_low)
        if count_low == 3:
            k_max = k_max - (5)
            count_low = 0
    elif np.abs(old_ctrls[i]) >= k_max:
        count_max += 1
        if count_max == 3:
            k_max = k_max + (5)
            count_max = 0

            
'''if len(old_ctrls) == 3:
    for i in range(len(old_ctrls)):
        #if (old_ctrls[i] >= 0 and old_ctrls[i+1] <= 0) or (old_ctrls[i] <= 0 and old_ctrls[i+1] >= 0):
        #    tmp = 0
        #else: 
        tmp = old_ctrls[i]
        if tmp < 0:
            print('tmp',np.round(np.abs(tmp),3)-1e-3 )
            if np.round(tmp,3)+1e-3 >= -np.round(k_max*2/U,3):
                ct_ll += 1
                if ct_ll == 3:
                    k_max = k_max - (5)
                    ct_ll = 0
            elif tmp <= -k_max:
                ct_lm += 1 

                if ct_lm == 3:
                    k_max = k_max + (5)
                    ct_lm = 0
        else:
            if np.round(tmp,3)+1e-3 <= np.round(k_max*2/U,3):
                ct_rl += 1

                if ct_rl == 3:
                    k_max = k_max - (5)
                    ct_rl = 0
            elif np.abs(tmp)+1e-3 >= k_max:
                ct_rm += 1 

                if ct_rm == 3:
                    k_max = k_max + (5)
                    ct_rm = 0'''
    #old_ctrls = []
ctrl_cmd = [-k_max, -k_max*4/(U),-k_max*2/(U),0,k_max*2/(U),k_max*4/(U),k_max]
#print(count_low)
print('new',ctrl_cmd)