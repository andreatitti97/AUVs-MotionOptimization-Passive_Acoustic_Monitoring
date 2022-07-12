
import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, filtfilt


class Trajectory:

    def __init__(self,  ts, ys, yds=None, ydds=None, misc=None):
        
        n_time_steps = ts.size
        assert(n_time_steps==ys.shape[0])
        dt_mean = np.mean(np.diff(ts))
        
        if yds is None:
            yds = diffnc(ys,dt_mean)
        else:
            assert(ys.shape==yds.shape)
        
        if ydds is None:
            ydds = diffnc(yds,dt_mean)
        else:
            assert(ys.shape==ydds.shape)
        
        if misc is not None:
            assert(n_time_steps==misc.shape[0])
            
        self.dim_ = 1
        if ys.ndim==2:
            self.dim_ = ys.shape[1]
            
        self.ts_ = ts
        self.dt_mean = dt_mean
        self.ys_ = ys
        self.yds_ = yds
        self.ydds_ = ydds
        self.misc_ = misc

    def setMisc(self,misc):
        assert(misc.shape[0]==self.ts_.shape[0])
        self.misc_ = misc

    def length(self):
        return self.ts_.shape[0]

    def duration(self):
        return self.ts_[-1]-self.ts_[0]

    def dim(self):
        return self.dim_
            
    def dim_misc(self):
        if self.misc_ is None:
            return 0
        else:
            return self.misc_.shape[1]
            
    def initial_y(self):
        return self.ys_[0]
        
    def final_y(self):
        return self.ys_[-1]
        
    def startTimeAtZero(self):
        self.ts_ = self.ts_ - self.ts_[0]
        
    def getRangePerDim(self):
        return self.ys_.max(axis=0)-self.ys_.min(axis=0)
        
    def crop(self,fro,to,as_times=False):        
        # Crop trajectory from 'fro' to 'to'
        # if as_times is False, 'fro' to 'to' are interpreted as indices 
        # if as_times is True, 'fro' to 'to' are interpreted as times
        
        # No need to crop empty trajectory
        if self.ts_.size==0:
            return
        
        assert(fro<to)
        
        if as_times:
            if (fro>self.ts_[-1]):
                print("WARNING: Argument 'fro' out of range, because "+str(fro)+" > "+str(self.ts_[-1])+". Not cropping")
                return 
            if (to<self.ts_[0]):
                print("WARNING: Argument 'to' out of range, because "+str(to)+" < "+str(self.ts_[0])+". Not cropping")
                return 
                
            # Convert time 'fro' to index 'fro' 
            if fro<=self.ts_[0]:
                # Time 'fro' lies before first time in trajectory
                fro = 0 
            else:
                # Get first index when time is larger than 'fro'                
                fro = np.argmax(self.ts_>=fro)
                
            if to>=self.ts_[-1]:
                # Time 'to' is larger than the last time in the trajectory
                to = len(self.ts_)-1
            else:
                # Get first index when time is smaller than 'to'                
                to = np.argmax(self.ts_>=to)
            
        assert(to<self.length())
        self.ts_ = self.ts_[fro:to]
        self.ys_ = self.ys_[fro:to,:]
        self.yds_ = self.yds_[fro:to,:]
        self.ydds_ = self.ydds_[fro:to,:]
        if self.misc_ is not None:
            self.misc_ = self.misc_[fro:to,:]
            
        
            
    def generatePolynomialTrajectory(ts, y_from, yd_from, ydd_from, y_to, yd_to, ydd_to):
        
        a0 = y_from
        a1 = yd_from
        a2 = ydd_from / 2

        a3 = -10 * y_from - 6 * yd_from - 2.5 * ydd_from + 10 * y_to - 4 * yd_to + 0.5 * ydd_to
        a4 = 15 * y_from + 8 * yd_from + 2 * ydd_from - 15  * y_to  + 7 * yd_to - ydd_to
        a5 = -6 * y_from - 3 * yd_from - 0.5 * ydd_from  + 6 * y_to  - 3 * yd_to + 0.5 * ydd_to

        n_time_steps = ts.size
        n_dims = y_from.size
  
        ys = np.zeros([n_time_steps,n_dims])
        yds = np.zeros([n_time_steps,n_dims])
        ydds = np.zeros([n_time_steps,n_dims])

        for i in range(n_time_steps):
            t = (ts[i] - ts[0]) / (ts[n_time_steps - 1] - ts[0])
            ys[i,:] = a0 + a1 * t + a2 * pow(t, 2) + a3 * pow(t, 3) + a4 * pow(t, 4) + a5 * pow(t, 5)
            yds[i,:] = a1 + 2 * a2 * t + 3 * a3 * pow(t, 2) + 4 * a4 * pow(t, 3) + 5 * a5 * pow(t, 4)
            ydds[i,:] = 2 * a2 + 6 * a3 * t + 12 * a4 * pow(t, 2) + 20 * a5 * pow(t, 3)

        yds /= (ts[n_time_steps - 1] - ts[0])
        ydds /= pow(ts[n_time_steps - 1] - ts[0], 2)

        return Trajectory(ts, ys, yds, ydds)

    def generatePolynomialTrajectoryThroughViapoint(ts, y_from, y_yd_ydd_viapoint, viapoint_time, y_to):
        
        n_time_steps = ts.size
        n_dims = y_from.size
  
        assert(n_dims==y_to.size)
        assert(3*n_dims==y_yd_ydd_viapoint.size)# Contains y, yd and ydd, so *3
  
        viapoint_time_step = 0
        while viapoint_time_step<n_time_steps and ts[viapoint_time_step]<viapoint_time:
            viapoint_time_step+=1
  
        #if (viapoint_time_step>=n_time_steps)
        #{
        #cerr << __FILE__ << ":" << __LINE__ << ":"
        #cerr << "ERROR: the time vector does not contain any time smaller than " << viapoint_time << ". Returning min-jerk trajectory WITHOUT viapoint." <<  endl
        #return Trajectory()
        #}

        yd_from        = np.zeros(n_dims)
        ydd_from       = np.zeros(n_dims)
        
        y_viapoint     = y_yd_ydd_viapoint[0*n_dims:1*n_dims]
        yd_viapoint    = y_yd_ydd_viapoint[1*n_dims:2*n_dims]
        ydd_viapoint   = y_yd_ydd_viapoint[2*n_dims:3*n_dims]
        
        yd_to          = np.zeros(n_dims)
        ydd_to         = np.zeros(n_dims)

        traj1 = Trajectory.generatePolynomialTrajectory(ts[:viapoint_time_step], y_from, yd_from, ydd_from, y_viapoint, yd_viapoint, ydd_viapoint)

        traj2 = Trajectory.generatePolynomialTrajectory(ts[viapoint_time_step:], y_viapoint, yd_viapoint, ydd_viapoint, y_to, yd_to, ydd_to)
        
        traj1.append(traj2)
        
        return traj1

def diffnc(X,dt):
    # [X] = diffc(X,dt) does non causal differentiation with time interval
    # dt between data points. The returned vector (matrix) is of the same length
    # as the original one
    #
    # Stefan Schaal December 29, 1995. Converted to Python by Freek Stulp
    
    (n_samples, n_dims)  = X.shape
    fil = np.array([1.0,0.0,-1.0])/2/dt
    XX = np.empty([n_samples+2, n_dims])
    for i_dim in range(n_dims):
        XX[:,i_dim] = np.convolve(X[:,i_dim],fil)
    
    X = XX[1:-1,:]
    X[0,:] = X[1,:]
    X[-1,:] = X[-2,:]
    return X

def butter_lowpass(cutoff, fs, order=3):
    # http://scipy.github.io/old-wiki/pages/Cookbook/ButterworthBandpass
    nyq = 0.5 * fs
    cut = cutoff / nyq
    b, a = butter(order, cut, btype='low',analog=False)
    return b, a


def butter_lowpass_filter(data, cutoff, fs, order=3):
    # http://scipy.github.io/old-wiki/pages/Cookbook/ButterworthBandpass
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = filtfilt(b, a, data, axis=0)
    return y
