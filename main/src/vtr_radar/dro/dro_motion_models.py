import numpy as np
import torch


class MotionModel:
    def __init__(self, state_size, device='cpu'):
        self.state_size = torch.tensor(state_size).to(device)
        self.device = device
        self.time = None
        self.num_steps = None
        self.t0 = None

    def setTime(self, time, t0):
        self.original_time = time.clone()
        self.original_t0 = t0.clone()
        self.time = (time - t0).float() * torch.tensor(1.0e-6).to(self.device)
        self.t0 = t0
        self.num_steps = torch.tensor(len(time)).to(self.device)

    def getLocalTime(self, time):
        return (time - self.t0).float() * 1.0e-6

    def getInitialState(self):
        return torch.zeros(self.state_size, device=self.device)

class ConstVelConstW(MotionModel):
    def __init__(self, device='cpu'):
        super().__init__(3, device)
    
    # Make sure that state and azimuths are torch tensors
    def getVelPosRot(self, state, with_jac = False):
        with torch.no_grad():
            # The state is an array of size 2. Duplicate it for each azimuth
            rot = state[2]*self.time.unsqueeze(1)
            pos = torch.stack((state[0]*self.time, state[1]*self.time), dim=1).unsqueeze(2)
            vel = state[:2].unsqueeze(0).unsqueeze(2).clone()

            # Transform the velocities to body centric
            c_rot = torch.cos(rot)
            s_rot = torch.sin(rot)
            vx_c_rot = vel[:, 0]*c_rot
            vy_s_rot = vel[:, 1]*s_rot
            vx_s_rot = vel[:, 0]*s_rot
            vy_c_rot = vel[:, 1]*c_rot
            vel_body = torch.cat((vx_c_rot + vy_s_rot, vy_c_rot - vx_s_rot), dim=1)

            if not with_jac:
                return vel_body, pos, rot

            d_rot_d_state = torch.zeros((self.num_steps, 1, 1)).to(self.device)
            d_rot_d_state[:, :, 0] = self.time.unsqueeze(1)

            d_pos_d_state = torch.zeros((self.num_steps, 2, 3)).to(self.device)
            d_pos_d_state[:, 0, 0] = self.time
            d_pos_d_state[:, 1, 1] = self.time

            d_vel_body_d_state = torch.empty((self.num_steps, 2, 3), device=self.device)
            d_vel_body_d_state[:, 0, 0] = c_rot.squeeze()
            d_vel_body_d_state[:, 0, 1] = s_rot.squeeze()
            d_vel_body_d_state[:, 1, 0] = -s_rot.squeeze()
            d_vel_body_d_state[:, 1, 1] = c_rot.squeeze()
            d_vel_body_d_state[:, 0, 2] = vy_c_rot.squeeze()*self.time -vx_s_rot.squeeze()*self.time
            d_vel_body_d_state[:, 1, 2] = -vx_c_rot.squeeze()*self.time -vy_s_rot.squeeze()*self.time

            return vel_body, d_vel_body_d_state, pos, d_pos_d_state, rot, d_rot_d_state

    def getPosRotSingle(self, state, time):
        with torch.no_grad():
            local_time = self.getLocalTime(time)
            rot = state[2]*local_time
            pos = torch.stack((state[0]*local_time, state[1]*local_time))
            return pos, rot
        
class ConstBodyVelGyro(MotionModel):
    def __init__(self, device='cpu'):
        super().__init__(2, device)

        self.initialised = False

    def setGyroData(self, gyro_time, gyro_yaw):
        self.first_gyro_time = gyro_time[0]
        self.gyro_time = torch.tensor(gyro_time - self.first_gyro_time).double().to(self.device)
        self.first_gyro_time *= 1.0e-6
        self.gyro_time *= 1.0e-6
        self.gyro_yaw = torch.tensor(gyro_yaw).double().to(self.device)
        self.gyro_yaw_original = self.gyro_yaw.clone()

        self.bin_integral = (self.gyro_yaw[1:] + self.gyro_yaw[:-1])*(self.gyro_time[1:] - self.gyro_time[:-1])/2.0
        self.coeff = (self.gyro_yaw[1:] - self.gyro_yaw[:-1])/(self.gyro_time[1:] - self.gyro_time[:-1])
        self.coeff = torch.cat((self.coeff, self.coeff[-1].unsqueeze(0)), dim=0)
        self.offset = self.gyro_yaw[:-1] - self.coeff[:-1]*self.gyro_time[:-1]
        self.offset = torch.cat((self.offset, self.offset[-1].unsqueeze(0)), dim=0)

        self.initialised = True


    def setTime(self, time, t0):
        if not self.initialised:
            raise ValueError("Gyro data not set")
        with torch.no_grad():
            super().setTime(time, t0)
            # Get the integral be
            time_local = time.double()*1e-6 - self.first_gyro_time
            time_start = t0.double()*1e-6 - self.first_gyro_time
            start_idx = torch.searchsorted(self.gyro_time, time_start)
            end_idx = torch.searchsorted(self.gyro_time, time_local)
            if end_idx[-1] == 0:
                raise ValueError("Time is before the first gyro data: Currently not supported")
            mask = start_idx == end_idx
            self.r = torch.zeros_like(time).float()
            self.r[mask] = ((time_local[mask]*self.coeff[start_idx-1] + self.offset[start_idx-1] + time_start*self.coeff[start_idx-1] + self.offset[start_idx-1])*0.5*(time_local[mask] - time_start)).float()

            first_bucket = (self.coeff[start_idx-1]*time_start + self.offset[start_idx-1] + self.gyro_yaw[start_idx])*0.5*(self.gyro_time[start_idx]-time_start)

            last_bucket = (self.coeff[end_idx-1]*time_local + self.offset[end_idx-1] + self.gyro_yaw[end_idx-1])*0.5*(time_local - self.gyro_time[end_idx-1])

            mask_one = start_idx + 1 == end_idx
            self.r[mask_one] = (first_bucket + last_bucket[mask_one]).float()
            mask_rest = start_idx + 1 < end_idx
            cumulative_integral = torch.cumsum(self.bin_integral[start_idx:torch.max(end_idx)], dim=0)
            self.r[mask_rest] = (first_bucket + cumulative_integral[end_idx[mask_rest]-2-start_idx] + last_bucket[mask_rest]).float()
            


            self.cos_r = torch.cos(self.r)
            self.sin_r = torch.sin(self.r)

            delta_time = time_local[1:] - time_local[:-1]

            cumulative_cos_r = torch.cumsum((self.cos_r[:-1]+self.cos_r[1:])*0.5*delta_time, dim=0)
            cumulative_sin_r = torch.cumsum((self.sin_r[:-1]+self.sin_r[1:])*0.5*delta_time, dim=0)

            self.R_integral = torch.empty((len(time), 2, 2), device=self.device)
            self.R_integral[0, :, :] = 0.0
            self.R_integral[1:, 0, 0] = cumulative_cos_r
            self.R_integral[1:, 0, 1] = -cumulative_sin_r
            self.R_integral[1:, 1, 0] = cumulative_sin_r
            self.R_integral[1:, 1, 1] = cumulative_cos_r
            


    def getVelPosRot(self, state, with_jac = False):
        with torch.no_grad():
            # The state is an array of size 2. Duplicate it for each azimuth
            rot = self.r.unsqueeze(1).clone()
            body_vel = state.unsqueeze(0).clone()
            pos = self.R_integral @ body_vel.unsqueeze(2)

            if not with_jac:
                return body_vel, pos, rot
            
            d_rot_d_state = None

            d_vel_body_d_state = torch.zeros((1, 2, 2), device=self.device)
            d_vel_body_d_state[:, 0, 0] = 1.0
            d_vel_body_d_state[:, 1, 1] = 1.0

            d_pos_d_state = self.R_integral.clone()

            return body_vel, d_vel_body_d_state, pos, d_pos_d_state, rot, d_rot_d_state

    def getPosRotSingle(self, state, time):
        if isinstance(time, np.int64) or isinstance(time, int):
            time = torch.tensor(time, dtype=torch.int64).to(self.device)
        with torch.no_grad():
            times = torch.arange(self.t0, time, 625, dtype=torch.int64).to(self.device)
            if times[-1] != time:
                times = torch.cat((times, time.unsqueeze(0)))
            og_times = self.original_time.clone()
            og_t0 = self.original_t0.clone()

            self.setTime(times, self.t0)

            pos = (self.R_integral[-1, :, :] @ state.unsqueeze(1)).squeeze()
            rot = self.r[-1]

            self.setTime(og_times, og_t0)
            return pos, rot
