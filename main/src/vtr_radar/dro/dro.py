import time
import os
# Enable caching of compilation
os.environ.setdefault("TORCHINDUCTOR_FX_GRAPH_CACHE", "1")

import torch
import torchvision
import numpy as np
from dro_motion_models import ConstBodyVelGyro, ConstVelConstW


kDefaultDroOpts = {
    'estimation': {
        'use_gyro': True,
        'estimate_gyro_bias': False,
        'estimate_vy_bias': False,
        'vy_bias_prior': 0.0,
        'max_acceleration': 10.0,
        'min_time_bias_init': 1.0,
        'T_axle_radar': np.eye(4),
        'gyro_bias_alpha': 0.01,
    },
    'gp': {
        'lengthscale_az': 2.0,
        'lengthscale_range': 4.0,
        'sz': 0.6,
    },
    'radar': {
        'del_f': 893.0e6,
        'ft': 76.04e9,
        'meas_freq': 1600.0,
        'beta_corr_fact': 0.944,
        'range_offset': -0.31,
        'nb_azimuths': None,
        'resolution': None,
        'doppler_enabled': None,
    },
    'direct': {
        'min_range': 4.0,
        'max_range': 70.0,
        'local_map_res': 0.1,
        'max_local_map_range': 120.0,
        'local_map_update_alpha': 0.1,
    },
    'doppler': {
        'min_range': 4.0,
        'max_range': 200.0,
    },
    'solver': {
        'nb_iter': 250,
        'cost_tol': 1e-6,
        'step_tol': 1e-5,
    },
}


class Dro():
    def __init__(self, opts):
        torch.set_float32_matmul_precision('high')
        with torch.no_grad():
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            self.initialized = False
            self.opts = opts
            self.timestamps = None

            # Some hardcoded parameters
            self.kImgPadding = 200
            self.kOptFirstStep = 0.1

            self.max_diff_vel = opts['estimation']['max_acceleration'] * 0.25

            # Load the motion model
            self.use_gyro = opts['estimation']['use_gyro']
            if self.use_gyro:
                self.motion_model = ConstBodyVelGyro(device=self.device)
            else:
                self.motion_model = ConstVelConstW(device=self.device)
            self.state_init = self.motion_model.getInitialState()

            self.offset = opts['radar']['range_offset']


            # Read the estimation options about the biases
            self.estimate_gyro_bias = opts['estimation']['estimate_gyro_bias']
            self.estimate_vy_bias = opts['estimation']['estimate_vy_bias']
            self.vy_bias = opts['estimation']['vy_bias_prior']


            # Initialise the GP parameters
            self.l_az = float(opts['gp']['lengthscale_az'])
            self.l_range = float(opts['gp']['lengthscale_range'])
            self.size_az = int(self.l_az)
            self.size_range = int(self.l_range)

            # Radar parameters (for the doppler and convolution)
            df_dt = float(opts['radar']['del_f']) * float(opts['radar']['meas_freq'])
            self.radar_beta = float(opts['radar']['beta_corr_fact']) * (float(opts['radar']['ft']) + float(opts['radar']['del_f'])/2.0) / df_dt


            # Prepare the local_map
            local_map_res = float(opts['direct']['local_map_res'])
            max_local_map_range = float(opts['direct']['max_local_map_range'])
            local_map_size = int(max_local_map_range/local_map_res)*2 + 1
            self.local_map = torch.zeros((local_map_size, local_map_size)).to(self.device)
            temp_x = (torch.arange( -local_map_size//2, local_map_size//2, 1).to(self.device) + 1) * local_map_res
            X = -temp_x.unsqueeze(0).T.repeat(1,local_map_size)
            Y = temp_x.unsqueeze(0).repeat(local_map_size,1)
            self.local_map_xy = torch.stack((X, Y), dim=2).unsqueeze(-1).to(self.device)
            self.local_map_res = torch.tensor(local_map_res).to(self.device)
            self.local_map_zero_idx = torch.tensor(int(max_local_map_range/local_map_res)).to(self.device)
            self.local_map_polar = torch.zeros((self.local_map_xy.shape[0], self.local_map_xy.shape[1], 2)).to(self.device)
            self.local_map_polar[:, :, 0] = torch.atan2(self.local_map_xy[:, :, 1, 0], self.local_map_xy[:, :, 0, 0])
            self.local_map_polar[:, :, 1] = torch.sqrt(self.local_map_xy[:, :, 0, 0]**2 + self.local_map_xy[:, :, 1, 0]**2)

            local_map_update_alpha = float(opts['direct']['local_map_update_alpha'])
            self.one_minus_alpha = torch.tensor(1 - local_map_update_alpha).to(self.device)
            self.alpha = torch.tensor(local_map_update_alpha).to(self.device)

            self.max_range_local_map = np.sqrt(2)*max_local_map_range

            self.current_rot = torch.tensor(0.0).to(self.device).double()
            self.current_pos = torch.zeros(2).to(self.device).double()

            self.max_acc = torch.tensor(float(opts['estimation']['max_acceleration'])).to(self.device)

            self.previous_vel = torch.tensor(0.0).to(self.device)

            self.step_counter = 0

            self.kImgPadding = torch.tensor(self.kImgPadding).to(self.device)

            if opts['radar']['nb_azimuths'] is not None and opts['radar']['resolution'] is not None and opts['radar']['doppler_enabled'] is not None:
                temp_radar_info = {
                    'resolution': opts['radar']['resolution'],
                    'chirps': [0, 1] if opts['radar']['doppler_enabled'] else [0, 0],
                }

                self.initialize(temp_radar_info)

                self.warmupCompiledCallables(opts)



    # Ugly code to force the compilation of the callables during initialization, so that the first call to odometryStep is not much slower than the others.
    # Can be refactored later for readability.
    def warmupCompiledCallables(self, opts):
        if os.environ.get("DRO_COMPILE_DEBUG", "0") == "1":
            try:
                import torch._logging as torch_logging
                torch_logging.set_logs(recompiles=True, guards=True, dynamic=True)
            except Exception:
                pass

        self.getUpDownPolarImages = torch.compile(self.getUpDownPolarImages, dynamic=True)
        self.prepareLocalMapPolarCoords = torch.compile(self.prepareLocalMapPolarCoords, dynamic=True)
        self.bilinearInterpolation = torch.compile(self.bilinearInterpolation, dynamic=True)
        self.bilinearInterpolationSparse = torch.compile(self.bilinearInterpolationSparse, dynamic=True)
        self.perLineInterpolation = torch.compile(self.perLineInterpolation, dynamic=True)
        self.polarToCartCoordCorrectionSparse = torch.compile(self.polarToCartCoordCorrectionSparse, dynamic=True)
        self.polarCoordCorrection = torch.compile(self.polarCoordCorrection)
        self.imgDopplerInterpAndJacobian = torch.compile(self.imgDopplerInterpAndJacobian, dynamic=True)
        self.cartToLocalMapIDSparse = torch.compile(self.cartToLocalMapIDSparse, dynamic=True)
        self.moveLocalMap = torch.compile(self.moveLocalMap, dynamic=True)

        # Warm up compiled callables so compile happens during initialization.
        nb_azimuths = opts['radar']['nb_azimuths']
        self.nb_azimuths = torch.tensor(nb_azimuths).to(self.device)
        self.azimuths = torch.linspace(0.0, 2*torch.pi, nb_azimuths).to(self.device)
        res = opts['radar']['resolution']
        if opts['radar']['doppler_enabled']:
            nb_ranges = int(max(float(opts['doppler']['max_range']) / res, float(opts['direct']['max_range']) / res)) + 1
        else:
            nb_ranges = int(float(opts['direct']['max_range']) / res) + 1
        warmup_img = torch.zeros((nb_azimuths, nb_ranges), device=self.device)
        # Populate a fake scan with enough non-zero values to build realistic sparse masks.
        nb_non_zero = min(120000, warmup_img.numel())
        indices = torch.randperm(warmup_img.numel(), device=self.device)[:nb_non_zero]
        warmup_img.view(-1)[indices] = 1.0
        warmup_img_np = warmup_img.detach().cpu().numpy().astype(np.float32)

        self.timestamps = torch.arange(nb_azimuths, device=self.device).float()
        if self.use_gyro:
            warmup_gyro_time = self.timestamps.detach().cpu().numpy().astype(np.float64)
            warmup_gyro_yaw = np.zeros_like(warmup_gyro_time)
            self.motion_model.setGyroData(warmup_gyro_time, warmup_gyro_yaw)
        self.motion_model.setTime(self.timestamps, self.timestamps[0])
        dirs = torch.empty((self.nb_azimuths, 2), device=self.device)
        dirs[:, 0] = torch.cos(self.azimuths)
        dirs[:, 1] = torch.sin(self.azimuths)
        self.vel_to_bin_vec = self.vel_to_bin * dirs
        self.chirp_up = True
        self.prev_chirp_up = True

        # Build Doppler sparse tensors using the same odometryStep preparation logic.
        if self.use_doppler:
            odd_img, even_img = self.getUpDownPolarImages(warmup_img_np[:, self.min_range_idx:(self.max_range_idx+1)])
            self.temp_even_img = torch.cat((
                torch.zeros((self.nb_azimuths, self.kImgPadding), dtype=torch.float32).to(self.device),
                even_img,
                torch.zeros((self.nb_azimuths, self.kImgPadding), dtype=torch.float32).to(self.device)
            ), dim=1)
            temp_odd_img = torch.cat((
                torch.zeros((self.nb_azimuths, self.kImgPadding), dtype=torch.float32).to(self.device),
                odd_img,
                torch.zeros((self.nb_azimuths, self.kImgPadding), dtype=torch.float32).to(self.device)
            ), dim=1)

            self.odd_coeff = torch.empty_like(self.temp_even_img, device=self.device)
            self.odd_coeff[:, :-1] = temp_odd_img[:, 1:] - temp_odd_img[:, :-1]
            self.odd_coeff[:, -1] = 0
            self.odd_bias = temp_odd_img.clone()

            mask_doppler = self.temp_even_img != 0
            self.temp_even_img_sparse = self.temp_even_img[mask_doppler]
            self.doppler_az_ids_sparse = torch.arange(self.nb_azimuths, device=self.device).unsqueeze(-1).repeat(1, self.temp_even_img.shape[1])[mask_doppler]
            self.doppler_bin_vec_sparse = torch.arange(self.nb_bins, device=self.device).unsqueeze(0).repeat(self.nb_azimuths, 1)[mask_doppler]

        # Build direct sparse tensors using the same odometryStep preparation logic.
        max_range_idx_direct = int(self.max_range_idx_direct.item())
        temp_intensity = warmup_img[:, :max_range_idx_direct]
        mask_direct = temp_intensity != 0
        min_range_idx_direct = int(self.min_range_idx_direct.item())
        mask_direct[:, :min_range_idx_direct] = False
        self.polar_intensity_sparse = temp_intensity[mask_direct]

        self.direct_r_sparse = self.range_vec.unsqueeze(0).repeat(self.nb_azimuths, 1)[mask_direct]
        self.direct_az_ids_sparse = torch.arange(self.nb_azimuths, device=self.device).unsqueeze(-1).repeat(1, max_range_idx_direct)[mask_direct]
        self.direct_r_ids_sparse = torch.arange(max_range_idx_direct, device=self.device).unsqueeze(0).repeat(self.nb_azimuths, 1)[mask_direct]
        if self.use_doppler:
            self.mask_direct_even = torch.empty_like(mask_direct, device=self.device)
            self.mask_direct_even[1::2] = False
            self.mask_direct_even[::2] = True
            self.mask_direct_even = self.mask_direct_even[mask_direct]
            self.mask_direct_odd = torch.empty_like(mask_direct, device=self.device)
            self.mask_direct_odd[::2] = False
            self.mask_direct_odd[1::2] = True
            self.mask_direct_odd = self.mask_direct_odd[mask_direct]
        else:
            self.mask_direct_even = torch.ones_like(self.polar_intensity_sparse, device=self.device, dtype=torch.bool)
            self.mask_direct_odd = torch.zeros_like(self.mask_direct_even, device=self.device, dtype=torch.bool)

        self.direct_nb_non_zero = torch.tensor(self.polar_intensity_sparse.shape[0], device=self.device)
        self.direct_r_ids_even = self.direct_r_ids_sparse[self.mask_direct_even]
        self.direct_r_ids_odd = self.direct_r_ids_sparse[self.mask_direct_odd]
        self.direct_r_even = self.direct_r_sparse[self.mask_direct_even]
        self.direct_r_odd = self.direct_r_sparse[self.mask_direct_odd]
        self.direct_az_ids_even = self.direct_az_ids_sparse[self.mask_direct_even]
        self.direct_az_ids_odd = self.direct_az_ids_sparse[self.mask_direct_odd]

        # Prepare polar coordinate tensor needed by polarCoordCorrection.
        max_range_idx = self.max_range_idx if hasattr(self, 'max_range_idx') else max_range_idx_direct
        range_vec = torch.arange(max_range_idx, device=self.device).float() * res + (res * 0.5)
        self.polar_coord_raw_gp_infered = torch.zeros((self.nb_azimuths, max_range_idx, 2), device=self.device)
        self.polar_coord_raw_gp_infered[:, :, 0] = self.azimuths.unsqueeze(1).repeat(1, max_range_idx)
        self.polar_coord_raw_gp_infered[:, :, 1] = range_vec.unsqueeze(0).repeat(self.nb_azimuths, 1)

        self.polar_intensity = torch.tensor(
            warmup_img_np[:, :max(self.max_id, self.max_range_idx_direct)],
            dtype=torch.float32,
            device=self.device,
        )
        polar_std = torch.std(self.polar_intensity, dim=1)
        polar_mean = torch.mean(self.polar_intensity, dim=1)
        self.polar_intensity -= (polar_mean.unsqueeze(1) + 2 * polar_std.unsqueeze(1))
        self.polar_intensity[self.polar_intensity < 0] = 0
        self.polar_intensity = torchvision.transforms.functional.gaussian_blur(self.polar_intensity.unsqueeze(0), (9,1), 3).squeeze()
        self.polar_intensity /= torch.max(self.polar_intensity, dim=1, keepdim=True)[0]
        self.polar_intensity[torch.isnan(self.polar_intensity)] = 0

        warmup_shift = torch.zeros((nb_azimuths,), device=self.device)
        warmup_pos = torch.zeros((nb_azimuths, 2, 1), device=self.device)
        warmup_rot = torch.zeros((nb_azimuths, 1), device=self.device)
        warmup_sparse_count = max(1, int(self.direct_nb_non_zero.item()))
        warmup_sparse_xy = torch.zeros((warmup_sparse_count, 2, 1), device=self.device)
        warmup_sparse_azr = torch.zeros((warmup_sparse_count, 2), device=self.device)

        self.getUpDownPolarImages(warmup_img_np[:, self.min_range_idx:(self.max_range_idx+1)])
        self.prepareLocalMapPolarCoords(self.local_map_polar, float(self.local_map_res))
        self.bilinearInterpolation(self.local_map, self.local_map_polar.clone())
        self.polarToCartCoordCorrectionSparse(warmup_pos, warmup_rot, warmup_shift)
        polar_coord_corrected = self.polarCoordCorrection(warmup_pos, warmup_rot)
        if self.use_doppler:
            self.imgDopplerInterpAndJacobian(warmup_shift)
        cart_corrected_sparse, _, _ = self.polarToCartCoordCorrectionSparse(warmup_pos, warmup_rot, warmup_shift)
        cart_idx_sparse = self.cartToLocalMapIDSparse(cart_corrected_sparse).squeeze()
        self.bilinearInterpolationSparse(self.local_map, cart_idx_sparse)

        prev_shifted = self.perLineInterpolation(self.polar_intensity[:, :self.max_id], warmup_shift)
        polar_coord_corrected[:,:,0] -= (self.azimuths[0])
        polar_coord_corrected[polar_coord_corrected[:,:,0] < 0] = polar_coord_corrected[polar_coord_corrected[:,:,0] < 0] + torch.tensor((2 * torch.pi, 0)).to(self.device)
        polar_coord_corrected[:,:,0] *= (self.nb_azimuths / (2 * torch.pi))
        polar_coord_corrected[:,:,1] -= (res / 2.0)
        polar_coord_corrected[:,:,1] /= res
        prev_shifted = torch.concatenate((prev_shifted, prev_shifted[0,:].unsqueeze(0)), dim=0)
        polar_target = self.bilinearInterpolation(prev_shifted, polar_coord_corrected)
        temp_polar_to_interp = self.prepareLocalMapPolarCoords(self.local_map_polar, float(res))
        polar_target = torch.concatenate((polar_target, polar_target[0,:].unsqueeze(0)), dim=0)
        local_map_update = self.bilinearInterpolation(polar_target, temp_polar_to_interp)
        self.local_map_blurred = torchvision.transforms.functional.gaussian_blur(local_map_update.unsqueeze(0).unsqueeze(0), 3).squeeze()
        self.bilinearInterpolationSparse(self.local_map_blurred, cart_idx_sparse)
        self.moveLocalMap(torch.zeros((2,), device=self.device), torch.tensor(0.0, device=self.device))

        # Warm up the first-step optimisation path as executed in odometryStep.
        saved_step_counter = self.step_counter
        saved_state_init = self.state_init.clone()
        saved_previous_vel = self.previous_vel.clone()
        saved_max_diff_vel = self.max_diff_vel.clone() if isinstance(self.max_diff_vel, torch.Tensor) else self.max_diff_vel
        self.step_counter = 0
        _ = self.solve(self.state_init.clone(), nb_iter=self.opts['solver']['nb_iter'], cost_tol=self.opts['solver']['cost_tol'], step_tol=self.opts['solver']['step_tol'])
        self.step_counter = saved_step_counter
        self.state_init = saved_state_init
        self.previous_vel = saved_previous_vel
        self.max_diff_vel = saved_max_diff_vel


    def odometryStep(self, radar_data, imu_data):
        with torch.no_grad():
            if self.initialized == False:
                self.initialize(radar_data)
            # Prepare the radar data from the input
            timestamps = radar_data['timestamps']
            azimuths = radar_data['azimuths']
            polar_image = radar_data['polar']
            res = self.opts['radar']['resolution']

            # Dirty way to account for the offset
            offset = self.offset / res
            if offset > 0:
                polar_image = np.concatenate(np.zeros((polar_image.shape[0], int(np.round(offset)))), polar_image, axis=1)
            elif offset < 0:
                polar_image = polar_image[:, int(np.round(-offset)):]

            # Re-initialize if the scan we receive is smaller than the pre-set max range size
            if polar_image.shape[1] < int(self.max_range_idx_direct):
                self.initialize({**radar_data, 'polar': polar_image})

            # Prepare the chirp direction
            if self.use_doppler:
                self.chirp_up = (radar_data['chirps'][0] == 0)
            else:
                self.chirp_up = radar_data['chirps'][0] == 0
                    
            # Prepare the timestamps
            if self.timestamps is None:
                self.max_diff_vel = self.max_acc * (timestamps[-1] - timestamps[0]) * 10e-6
            self.timestamps = torch.tensor(timestamps).to(self.device).squeeze()
            delta_time = 0.25#(self.timestamps[0] - last_scan_time)*10e-6
            

            # Update the pose and the local map
            if self.step_counter > 0:
                # Get the velocities and positions of the previous scan's azimuths
                vel_body, prev_scan_pos, prev_scan_rot = self.motion_model.getVelPosRot(self.state_init, with_jac=False)

                # Get delta pose from the beginning of the previous scan to the beginning of the current scan
                frame_pos, frame_rot = self.motion_model.getPosRotSingle(self.state_init, self.timestamps[0])

                # Update the current position and rotation
                rot_mat = torch.tensor([[torch.cos(self.current_rot), -torch.sin(self.current_rot)], [torch.sin(self.current_rot), torch.cos(self.current_rot)]]).to(self.device)
                self.current_pos = self.current_pos + rot_mat @ frame_pos.double()
                self.current_rot = self.current_rot + frame_rot.double()
                
                # Prepare the local map (undistort the previous scan, project it and the local map 
                # to the beginning of the current scan, and update the local map)
                # Get the shift for each line 
                shift = (vel_body.reshape((-1,1,2)) @ self.vel_to_bin_vec.reshape((-1,2,1))).squeeze()
                per_line_shift = shift/2.0
                if not self.prev_chirp_up:
                    per_line_shift = -per_line_shift
                if self.use_doppler:
                    per_line_shift[1::2] *= -1
                
                torch.cuda.synchronize()
                temp_t1 = time.time()
                # Correct for the Doppler shift
                prev_shifted = self.perLineInterpolation(self.polar_intensity[:,:self.max_id], per_line_shift)



                rot_mats_transposed = torch.concatenate((torch.cos(prev_scan_rot), torch.sin(prev_scan_rot), -torch.sin(prev_scan_rot), torch.cos(prev_scan_rot)), dim=1).reshape((-1,2,2))
                prev_scan_pos = prev_scan_pos.reshape((-1,2,1))
                pos = rot_mats_transposed@(-prev_scan_pos + frame_pos.reshape((-1,2,1))) 
                rot = -prev_scan_rot + frame_rot


                polar_coord_corrected = self.polarCoordCorrection(pos, rot)
                polar_coord_corrected[:,:,0] -= (self.azimuths[0])
                polar_coord_corrected[polar_coord_corrected[:,:,0]<0] = polar_coord_corrected[polar_coord_corrected[:,:,0]<0] + torch.tensor((2*torch.pi, 0)).to(self.device)
                polar_coord_corrected[:,:,0] *= ((self.nb_azimuths) / (2*torch.pi))
                polar_coord_corrected[:,:,1] -= (res/2.0)
                polar_coord_corrected[:,:,1] /= res
                prev_shifted = torch.concatenate((prev_shifted, prev_shifted[0,:].unsqueeze(0)), dim=0)
                polar_target = self.bilinearInterpolation(prev_shifted, polar_coord_corrected)


                # Get the coordinates of the local map in the undistorted polar image
                temp_polar_to_interp = self.prepareLocalMapPolarCoords(self.local_map_polar, res)
                polar_target = torch.concatenate((polar_target, polar_target[0,:].unsqueeze(0)), dim=0)
                local_map_update = self.bilinearInterpolation(polar_target, temp_polar_to_interp)


                # Update the local map
                if self.step_counter == 1:
                    self.local_map = local_map_update
                else:
                    self.moveLocalMap(frame_pos, frame_rot)
                    self.local_map = self.one_minus_alpha * self.local_map + self.alpha * local_map_update


            torch.cuda.synchronize()
            t1 = time.time()

            # Update the IMU data in the motion model
            if self.use_gyro:
                self.motion_model.setGyroData(
                    gyro_time = np.array([imu['timestamp'] for imu in imu_data]),
                    gyro_yaw = np.array([imu['angular_velocity'][2] for imu in imu_data]) - self.gyr_bias
                )

            # Query the GP interpolation of the up and down chirp images
            if self.use_doppler:
                odd_img, even_img = self.getUpDownPolarImages(polar_image[:,self.min_range_idx:(self.max_range_idx+1)])
            


            # Prepare the data in torch
            self.azimuths = torch.tensor(azimuths).to(self.device).float()
            self.nb_azimuths = torch.tensor(len(azimuths)).to(self.device)
            self.motion_model.setTime(self.timestamps, self.timestamps[0])

            # Initialise the direction vectors
            dirs = torch.empty((self.nb_azimuths, 2), device=self.device)
            dirs[:, 0] = torch.cos(self.azimuths)
            dirs[:, 1] = torch.sin(self.azimuths)
            self.vel_to_bin_vec = self.vel_to_bin*dirs



            ### Preparation for the doppler
            if self.use_doppler:
                # Padding the images for the doppler cost
                self.temp_even_img = torch.cat((torch.zeros((self.nb_azimuths, self.kImgPadding), dtype=torch.float32).to(self.device), even_img, torch.zeros((self.nb_azimuths, self.kImgPadding), dtype=torch.float32).to(self.device)), dim=1)
                temp_odd_img = torch.cat((torch.zeros((self.nb_azimuths, self.kImgPadding), dtype=torch.float32).to(self.device), odd_img, torch.zeros((self.nb_azimuths, self.kImgPadding), dtype=torch.float32).to(self.device)), dim=1)

                # Coefficients for the interpolation
                self.odd_coeff = torch.empty_like(self.temp_even_img, device=self.device)
                self.odd_coeff[:, :-1] = temp_odd_img[:, 1:] - temp_odd_img[:, :-1]
                self.odd_coeff[:, -1] = 0
                self.odd_bias = temp_odd_img.clone()

                mask_doppler = self.temp_even_img != 0
                self.temp_even_img_sparse = self.temp_even_img[mask_doppler]
                # Get the idx of the non zero values
                self.doppler_az_ids_sparse = torch.arange(self.nb_azimuths, device=self.device).unsqueeze(-1).repeat(1,self.temp_even_img.shape[1])[mask_doppler]
                self.doppler_bin_vec_sparse = torch.arange(self.nb_bins, device=self.device).unsqueeze(0).repeat(self.nb_azimuths,1)[mask_doppler]




            ### Preparation for the direct cost
            # Create the polar coordinates for the image
            self.polar_intensity = torch.tensor(polar_image[:,:max(self.max_id, self.max_range_idx_direct)]).to(self.device)
            polar_std = torch.std(self.polar_intensity, dim=1)
            polar_mean = torch.mean(self.polar_intensity, dim=1)
            self.polar_intensity -= (polar_mean.unsqueeze(1) + 2*polar_std.unsqueeze(1))
            self.polar_intensity[self.polar_intensity < 0] = 0
            self.polar_intensity = torchvision.transforms.functional.gaussian_blur(self.polar_intensity.unsqueeze(0), (9,1), 3).squeeze()
            self.polar_intensity /= torch.max(self.polar_intensity, dim=1, keepdim=True)[0]
            self.polar_intensity[torch.isnan(self.polar_intensity)] = 0

            # Preparation for the future localMap update (at the loop)
            range_vec = torch.arange(self.max_range_idx).to(self.device).float() * res + (res*0.5)
            self.polar_coord_raw_gp_infered = torch.zeros((self.nb_azimuths, self.max_range_idx, 2)).to(self.device)
            self.polar_coord_raw_gp_infered[:, :, 0] = self.azimuths.unsqueeze(1).repeat(1, self.max_range_idx)
            self.polar_coord_raw_gp_infered[:, :, 1] = range_vec.unsqueeze(0).repeat(self.nb_azimuths, 1)

            # Get sparse intensity information
            temp_intensity = self.polar_intensity[:, :self.max_range_idx_direct]
            mask_direct = temp_intensity != 0
            mask_direct[:, :self.min_range_idx_direct] = False
            self.polar_intensity_sparse = temp_intensity[mask_direct]


            self.direct_r_sparse = self.range_vec.unsqueeze(0).repeat(self.nb_azimuths, 1)[mask_direct]
            self.direct_az_ids_sparse = torch.arange(self.nb_azimuths, device=self.device).unsqueeze(-1).repeat(1,self.max_range_idx_direct)[mask_direct]
            self.direct_r_ids_sparse = torch.arange(self.max_range_idx_direct, device=self.device).unsqueeze(0).repeat(self.nb_azimuths, 1)[mask_direct]
            if self.use_doppler:
                self.mask_direct_even = torch.empty_like(mask_direct, device=self.device)
                self.mask_direct_even[1::2] = False
                self.mask_direct_even[::2] = True
                self.mask_direct_even = self.mask_direct_even[mask_direct]
                self.mask_direct_odd = torch.empty_like(mask_direct, device=self.device)
                self.mask_direct_odd[::2] = False
                self.mask_direct_odd[1::2] = True
                self.mask_direct_odd = self.mask_direct_odd[mask_direct]
            else:
                self.mask_direct_even = torch.ones_like(self.polar_intensity_sparse, device=self.device, dtype=torch.bool)
                self.mask_direct_odd = torch.zeros_like(self.mask_direct_even, device=self.device, dtype=torch.bool)

            self.direct_nb_non_zero = torch.tensor(self.polar_intensity_sparse.shape[0], device=self.device)
            self.direct_r_ids_even = self.direct_r_ids_sparse[self.mask_direct_even]
            self.direct_r_ids_odd = self.direct_r_ids_sparse[self.mask_direct_odd]
            self.direct_r_even = self.direct_r_sparse[self.mask_direct_even]
            self.direct_r_odd = self.direct_r_sparse[self.mask_direct_odd]
            self.direct_az_ids_even = self.direct_az_ids_sparse[self.mask_direct_even]
            self.direct_az_ids_odd = self.direct_az_ids_sparse[self.mask_direct_odd]


            ### Perform the optimisation
            if self.motion_model.state_size == 3 and self.use_gyro:
                self.state_init[:2] = self.state_init[:2]*(1+self.state_init[2]*delta_time)
            if torch.norm(self.state_init[:2]) < 0.75:
                self.state_init[:] = 0.0
            result = self.solve(self.state_init, self.opts['solver']['nb_iter'], self.opts['solver']['cost_tol'], self.opts['solver']['step_tol'])


            # Check if the the angular velocity is not too high
            # If it is, we set it to the previous value (preventing potential catastrophic failure)
            if isinstance(self.motion_model, ConstVelConstW):
                if self.step_counter > 0:
                    if torch.abs(result[2]) > maxAngVel(result[:2]):
                        result[2] = self.prev_state[2]
                self.prev_state = result.clone()

            # Update the vy bias if needed
            if self.use_doppler and self.estimate_vy_bias and np.linalg.norm(result[:2].cpu().numpy()) > 3.0:
                save_vy_bias = self.vy_bias
                self.vy_bias = 0.0
                vel_doppler_only = self.solve(self.state_init, self.opts['solver']['nb_iter'], self.opts['solver']['cost_tol'], self.opts['solver']['step_tol'], doppler_only=True)[:2]
                self.vy_bias = save_vy_bias

                vel_doppler_only = np.concatenate((vel_doppler_only.cpu().numpy(), [0]))
                T_axle_radar = self.opts['estimation']['T_axle_radar']
                if self.use_gyro:
                    # Get the average angular velocity between the first and last azimuth
                    gyro_data = np.mean([imu['angular_velocity'][2] for imu in imu_data])
                    gyro_data = T_axle_radar[:3, :3] @ np.array([0, 0, gyro_data])
                    axle_vel = T_axle_radar[:3, :3] @ vel_doppler_only + np.cross(gyro_data, T_axle_radar[3, :3])
                else:
                    axle_vel = T_axle_radar[:3, :3] @ vel_doppler_only
                vy = (T_axle_radar[:3,:3].T@(np.array([0, axle_vel[1], 0])))[1]
                self.vy_bias = 0.01 * vy + (0.99) * self.vy_bias

            # Update the gyro bias if needed
            if self.estimate_gyro_bias:
                velocity_norm = np.linalg.norm(result[:2].cpu().numpy())
                self.velocities_for_gyro_bias.pop(0)
                self.velocities_for_gyro_bias.append(velocity_norm)
                self.mean_gyr.pop(0)
                self.mean_gyr.append(np.mean([imu['angular_velocity'][2] for imu in imu_data]))

                # Check if all the velocities are under 0.05
                if all(vel < 0.05 for vel in self.velocities_for_gyro_bias):
                    if not self.gyr_bias_init:
                        self.gyr_bias = self.mean_gyr[len(self.mean_gyr)//2]
                        self.gyr_bias_init = True
                    else:
                        self.gyr_bias = self.gyr_bias_alpha * self.mean_gyr[len(self.mean_gyr)//2] + (1 - self.gyr_bias_alpha) * self.gyr_bias
                


            self.state_init = result.clone()

            self.prev_chirp_up = self.chirp_up
            self.step_counter += 1
            return result.detach().cpu().numpy(), self.local_map.detach().cpu().numpy()
        
    
    def isDopplerEnabled(self, radar_data):
        return radar_data['chirps'][0] != radar_data['chirps'][1]


    def initialize(self, radar_data):
        with torch.no_grad():
            self.initialized = True
            res = self.opts['radar']['resolution']
            self.vel_to_bin = 2*self.radar_beta / res

            self.max_range_idx_direct = torch.tensor(int(np.floor(self.opts['direct']['max_range'] / res))).to(self.device)
            self.min_range_idx_direct = torch.tensor(int(np.ceil(self.opts['direct']['min_range'] / res))).to(self.device)
            self.max_id = int(self.max_range_local_map / res)

            # Make sure max_range is not larger than the number of range bins in the polar image
            polar = radar_data.get('polar')
            if polar is not None:
                available_range_bins = polar.shape[1]
                if available_range_bins < int(self.max_range_idx_direct):
                    self.max_range_idx_direct = torch.tensor(available_range_bins).to(self.device)
                if available_range_bins < self.max_id:
                    self.max_id = available_range_bins

            # Doppler shift to range
            self.shift_to_range = torch.tensor(res / 2.0).to(self.device)
            self.range_vec = torch.arange(self.max_range_idx_direct).to(self.device).float() * res + (res/2.0)

            self.use_doppler = self.isDopplerEnabled(radar_data)

            range_start = int(np.ceil(float(self.opts['doppler']['min_range']) / res))
            range_end = int(np.floor(float(self.opts['doppler']['max_range']) / res))
            self.nb_bins = range_end - range_start + 1 + 2*self.kImgPadding

            # Prepare the GP convolutions for the image interlacing
            x = np.arange(-self.size_az, self.size_az+1)
            mask_smooth = x%2 == 0
            mask_interp = x%2 != 0
            x_smooth = x[mask_smooth].astype(np.float32)
            x_interp = x[mask_interp].astype(np.float32)
            y = np.arange(-self.size_range, self.size_range+1)
            XX_smooth, YY_smooth = np.meshgrid(x_smooth, y)
            XX_interp, YY_interp = np.meshgrid(x_interp, y)
            self.X_smooth = np.vstack((XX_smooth.T.flatten(), YY_smooth.T.flatten())).T
            self.X_interp = np.vstack((XX_interp.T.flatten(), YY_interp.T.flatten())).T

            sz = float(self.opts['gp']['sz'])
            n_smooth = self.X_smooth.shape[0]
            K_smooth = self.seKernel(self.X_smooth, self.X_smooth, self.l_az, self.l_range) + sz**2*np.eye(n_smooth)
            Kinv_smooth = np.linalg.inv(K_smooth)
            ks_smooth = self.seKernel(np.array([[0, 0]]), self.X_smooth, self.l_az, self.l_range)
            self.beta_smooth = (ks_smooth@Kinv_smooth).flatten()

            n_interp = self.X_interp.shape[0]
            K_interp = self.seKernel(self.X_interp, self.X_interp, self.l_az, self.l_range) + sz**2*np.eye(n_interp)
            Kinv_interp = np.linalg.inv(K_interp)
            ks_interp = self.seKernel(np.array([[0, 0]]), self.X_interp, self.l_az, self.l_range)
            self.beta_interp = (ks_interp@Kinv_interp).flatten()

            self.beta_smooth_torch_conv = torch.nn.Conv2d(1, 1, (len(x_smooth), len(y)), bias=False, padding=(len(x_smooth)//2, len(y)//2))
            beta_smooth_tensor = torch.tensor(self.beta_smooth.reshape((1, 1, len(x_smooth), len(y))).astype(np.float32)).to(self.device)
            self.beta_smooth_torch_conv.weight = torch.nn.Parameter(beta_smooth_tensor)
            self.beta_interp_torch_conv = torch.nn.Conv2d(1, 1, (len(x_interp), len(y)), bias=False, padding=(len(x_interp)//2, len(y)//2))
            beta_interp_tensor = torch.tensor(self.beta_interp.reshape((1, 1, len(x_interp), len(y))).astype(np.float32)).to(self.device)
            self.beta_interp_torch_conv.weight = torch.nn.Parameter(beta_interp_tensor)


            # Doppler range bounds
            self.max_range_idx = int(np.floor(float(self.opts['doppler']['max_range']) / res))
            self.min_range_idx = int(np.ceil(float(self.opts['doppler']['min_range']) / res))
            if polar is not None and available_range_bins < self.max_range_idx:
                self.max_range_idx = available_range_bins

            self.gyr_bias = 0.0

            if self.estimate_gyro_bias:
                self.gyr_bias_init = False
                num_velocities_for_gyro_bias = 7
                self.velocities_for_gyro_bias = [100.0]*num_velocities_for_gyro_bias
                self.mean_gyr = [0.0]*num_velocities_for_gyro_bias
                self.gyr_bias_alpha = self.opts['estimation']['gyro_bias_alpha']

    def seKernel(self, X1, X2, l_az, l_range):
        with torch.no_grad():
            temp_X1 = X1.copy()
            temp_X2 = X2.copy()
            temp_X1[:, 0] = temp_X1[:, 0] / l_az
            temp_X2[:, 0] = temp_X2[:, 0] / l_az
            temp_X1[:, 1] = temp_X1[:, 1] / l_range
            temp_X2[:, 1] = temp_X2[:, 1] / l_range
            # Replace dist with np only operations to avoid potential issues with torch and the GPU
            dist = np.sum((temp_X1[:, np.newaxis, :] - temp_X2[np.newaxis, :, :])**2, axis=2)
            return np.exp(-dist/2)



    # Get the polar images from the input image using the GP interpolation
    def getUpDownPolarImages(self, img):
        # Prepare the input for the torch convolution
        with torch.no_grad():
            mean_even = np.mean(img[::2, :])
            mean_odd = np.mean(img[1::2, :])
            in_even = img[::2, :] - mean_even
            in_odd = img[1::2, :] - mean_odd

            in_even_device = torch.tensor(in_even).unsqueeze(0).unsqueeze(0).to(self.device)
            in_odd_device = torch.tensor(in_odd).unsqueeze(0).unsqueeze(0).to(self.device)

            even_smooth_torch = self.beta_smooth_torch_conv(in_even_device)
            even_interp_torch = self.beta_interp_torch_conv(in_even_device)
            odd_smooth_torch = self.beta_smooth_torch_conv(in_odd_device)
            odd_interp_torch = self.beta_interp_torch_conv(in_odd_device)
            # Remove extra rows if the output of the convolution is larger than the input
            if even_smooth_torch.shape[2] > in_even.shape[0]:
                even_smooth_torch = even_smooth_torch[:, :, :-1, :]
            if even_interp_torch.shape[2] > in_even.shape[0]:
                even_interp_torch = even_interp_torch[:, :, :-1, :]
            if odd_smooth_torch.shape[2] > in_odd.shape[0]:
                odd_smooth_torch = odd_smooth_torch[:, :, :-1, :]
            if odd_interp_torch.shape[2] > in_odd.shape[0]:
                odd_interp_torch = odd_interp_torch[:, :, :-1, :]

            out_even = torch.zeros((1, 1, img.shape[0], img.shape[1]), dtype=torch.float32).to(self.device)
            out_odd = torch.zeros((1, 1, img.shape[0], img.shape[1]), dtype=torch.float32).to(self.device)
            out_even[:, :, ::2, :] = even_smooth_torch
            out_even[:, :, 1:-1:2, :] = even_interp_torch[:, :, 1:, :]
            out_odd[:, :, ::2, :] = odd_interp_torch
            out_odd[:, :, 1::2, :] = odd_smooth_torch
            out_odd[:, :, -1, :] = 0


            # Get standard deviation of each image (under the median)
            even_std = torch.std(out_even, dim=3, keepdim=True)
            odd_std = torch.std(out_odd)
            odd_std = torch.std(out_odd, dim=3, keepdim=True)
            out_even -= 2.0*even_std
            out_odd -= 2.0*odd_std
            out_even[out_even < 0] = 0
            out_odd[out_odd < 0] = 0

            # Add gaussian blur to the images
            out_even = torchvision.transforms.functional.gaussian_blur(out_even, (9,1), 3)
            out_odd = torchvision.transforms.functional.gaussian_blur(out_odd, (9,1), 3)

            # Normalise each row by the maximum value
            out_even = out_even / torch.max(out_even, dim=3, keepdim=True)[0]
            out_odd = out_odd / torch.max(out_odd, dim=3, keepdim=True)[0]

            # Replace NaN values by 0
            out_even[torch.isnan(out_even)] = 0
            out_odd[torch.isnan(out_odd)] = 0

            out_even = out_even.squeeze()
            out_odd = out_odd.squeeze()

            out_even[:self.size_az, :] = 0
            out_even[-self.size_az:, :] = 0
            out_odd[:self.size_az, :] = 0
            out_odd[-self.size_az:, :] = 0
            out_even[:, :self.size_range] = 0
            out_even[:, -self.size_range:] = 0
            out_odd[:, :self.size_range] = 0
            out_odd[:, -self.size_range:] = 0

            return out_odd, out_even


            
    def prepareLocalMapPolarCoords(self, local_map_polar, res):
        with torch.no_grad():
            temp_polar_to_interp = local_map_polar.clone()
            temp_polar_to_interp[:,:,0] -= (self.azimuths[0])
            temp_polar_to_interp[temp_polar_to_interp[:,:,0]<0] = temp_polar_to_interp[temp_polar_to_interp[:,:,0]<0] + torch.tensor((2*torch.pi, 0)).to(self.device)
            temp_polar_to_interp[:,:,0] *= ((self.nb_azimuths) / (2*torch.pi))
            temp_polar_to_interp[:,:,1] -= (res/2.0)
            temp_polar_to_interp[:,:,1] /= res
            return temp_polar_to_interp


    # Perform the bilinear interpolation of the image im at the coordinates az_r (az the vertical axis, r the horizontal axis)
    def bilinearInterpolation(self, im, az_r):
        with torch.no_grad():
            az0 = torch.floor(az_r[:, :, 0]).int()
            az1 = az0 + 1
            
            r0 = torch.floor(az_r[:, :, 1]).int()
            r1 = r0 + 1

            az0 = torch.clamp(az0, 0, im.shape[0]-1)
            az1 = torch.clamp(az1, 0, im.shape[0]-1)
            r0 = torch.clamp(r0, 0, im.shape[1]-1)
            r1 = torch.clamp(r1, 0, im.shape[1]-1)
            az_r[:,:,0] = torch.clamp(az_r[:,:,0], 0, im.shape[0]-1)
            az_r[:,:,1] = torch.clamp(az_r[:,:,1], 0, im.shape[1]-1)
            
            Ia = im[ az0, r0 ]
            Ib = im[ az1, r0 ]
            Ic = im[ az0, r1 ]
            Id = im[ az1, r1 ]
            
            local_1_minus_r = (r1.float()-az_r[:, :, 1])
            local_r = (az_r[:, :, 1]-r0.float())
            local_1_minus_az = (az1.float()-az_r[:, :, 0])
            local_az = (az_r[:, :, 0]-az0.float())
            wa = local_1_minus_az * local_1_minus_r
            wb = local_az * local_1_minus_r
            wc = local_1_minus_az * local_r
            wd = local_az * local_r

            img_interp = wa*Ia + wb*Ib + wc*Ic + wd*Id

            return img_interp


    # Same a bilinearInterpolation_ but for the sparse case
    def bilinearInterpolationSparse(self, im, az_r):
        with torch.no_grad():
            az0 = torch.floor(az_r[:, 0]).int()
            az1 = az0 + 1
            
            r0 = torch.floor(az_r[:, 1]).int()
            r1 = r0 + 1

            az0 = torch.clamp(az0, 0, im.shape[0]-1)
            az1 = torch.clamp(az1, 0, im.shape[0]-1)
            r0 = torch.clamp(r0, 0, im.shape[1]-1)
            r1 = torch.clamp(r1, 0, im.shape[1]-1)
            az_r[:,0] = torch.clamp(az_r[:,0], 0, im.shape[0]-1)
            az_r[:,1] = torch.clamp(az_r[:,1], 0, im.shape[1]-1)
            
            Ia = im[ az0, r0 ]
            Ib = im[ az1, r0 ]
            Ic = im[ az0, r1 ]
            Id = im[ az1, r1 ]
            
            local_1_minus_r = (r1.float()-az_r[:, 1])
            local_r = (az_r[:, 1]-r0.float())
            local_1_minus_az = (az1.float()-az_r[:, 0])
            local_az = (az_r[:, 0]-az0.float())
            wa = local_1_minus_az * local_1_minus_r
            wb = local_az * local_1_minus_r
            wc = local_1_minus_az * local_r
            wd = local_az * local_r

            img_interp = wa*Ia + wb*Ib + wc*Ic + wd*Id

            d_I_d_az_r = torch.empty((az_r.shape[0], 1, 2), device=self.device)
            d_I_d_az_r[:, 0, 0] = (Ib - Ia)*local_1_minus_r + (Id - Ic)*local_r
            d_I_d_az_r[:, 0, 1] = (Ic - Ia)*local_1_minus_az + (Id - Ib)*local_az
            return img_interp, d_I_d_az_r

        

    # Cost function and Jacobian for the Doppler and direct cost functions
    def costFunctionAndJacobian(self, state, doppler, direct, degraded=False):
        with torch.no_grad():
            state_size = len(state)
            velocities, d_vel_d_state, pos, d_pos_d_state, rot, d_rot_d_state = self.motion_model.getVelPosRot(state, with_jac=True)
            velocities = velocities.reshape((-1,1,2))
            mask = velocities[:,0,0] > 3.0
            velocities[mask,0,1] = velocities[mask,0,1] + self.vy_bias
            velocities[~mask,0,1] = velocities[~mask,0,1] + velocities[~mask,0,0]*self.vy_bias/3.0
            d_vel_d_state[~mask,1,:] = d_vel_d_state[~mask,1,:] + self.vy_bias/3.0 * d_vel_d_state[~mask,0,:]
            shifts = (velocities @ self.vel_to_bin_vec.reshape((-1,2,1))).squeeze()
            d_shift_d_state = self.vel_to_bin_vec.reshape((-1,1,2)) @ d_vel_d_state
            if not self.chirp_up:
                shifts = -shifts
                d_shift_d_state = -d_shift_d_state

            # Doppler cost
            if doppler:
                interp_sparse, aligned_odd_coeff_sparse = self.imgDopplerInterpAndJacobian(shifts)
                residual = interp_sparse * self.temp_even_img_sparse
                jacobian = aligned_odd_coeff_sparse.reshape((-1, 1, 1)) @ d_shift_d_state[self.doppler_az_ids_sparse,:,:] * (self.temp_even_img_sparse.unsqueeze(-1).unsqueeze(-1))
                residual = residual.flatten()
                jacobian = jacobian.reshape((-1,state_size))
                if degraded:
                    weights = ((torch.clip(torch.abs(interp_sparse - self.temp_even_img_sparse), 0, 1) - 1)**6 ).flatten().unsqueeze(-1)
                    jacobian = jacobian * weights
            # Direct cost
            if direct:
                cart_corrected_sparse, d_cart_d_rot_sparse, d_cart_d_shift_sparse = self.polarToCartCoordCorrectionSparse(pos, rot, shifts)

                # Get the corresponding localMap coordinates
                cart_idx_sparse = self.cartToLocalMapIDSparse(cart_corrected_sparse).squeeze()

                interp_direct_sparse, d_interp_direct_d_xy_sparse = self.bilinearInterpolationSparse(self.local_map_blurred, cart_idx_sparse)
                residual_direct_sparse = interp_direct_sparse * (self.polar_intensity_sparse)

                d_cart_sparse_d_state = (d_cart_d_shift_sparse @ d_shift_d_state.reshape((-1,1,state_size)))[self.direct_az_ids_sparse,:,:]
                if d_rot_d_state is not None:
                    d_cart_sparse_d_state[:,:,-1] += (d_cart_d_rot_sparse@(d_rot_d_state[self.direct_az_ids_sparse].reshape((-1,1,1))) ).squeeze()
                d_cart_sparse_d_state += d_pos_d_state[self.direct_az_ids_sparse].reshape((-1,2,state_size))
                d_cart_sparse_d_state[:,0,:] = d_cart_sparse_d_state[:,0,:] / (-self.local_map_res)
                d_cart_sparse_d_state[:,1,:] = d_cart_sparse_d_state[:,1,:] / self.local_map_res


                jacobian_direct_sparse = ((d_interp_direct_d_xy_sparse @ d_cart_sparse_d_state) * (self.polar_intensity_sparse.unsqueeze(-1).unsqueeze(-1))).squeeze()


                residual_direct = residual_direct_sparse.flatten()
                jacobian_direct = jacobian_direct_sparse.reshape((-1,state_size))
                if degraded:
                    weights_direct = ((torch.clip(torch.abs(interp_direct_sparse - self.polar_intensity_sparse), 0, 1) - 1)**6 ).flatten().unsqueeze(-1)
                    jacobian_direct = jacobian_direct * weights_direct


            if doppler and direct:
                residual = torch.cat((residual, residual_direct), 0)
                jacobian = torch.cat((jacobian, jacobian_direct), 0)
                return residual, jacobian
            elif doppler:
                return residual, jacobian
            elif direct:
                return residual_direct, jacobian_direct





    # Perform linear interpolation of the image im using the shift (in pixels)
    # (used for correcting the doppler shift when undistorting the scan)
    def perLineInterpolation(self, img, shift):
        with torch.no_grad():
            shift_int = torch.floor(shift).int()
            shift_frac = shift - shift_int.float()
            az = torch.tile(torch.arange(img.shape[0]).unsqueeze(1), (1, img.shape[1])).to(self.device)
            r_0 = torch.tile(torch.arange(img.shape[1]).unsqueeze(0), (img.shape[0], 1)).to(self.device)
            r_0 = r_0 + shift_int.reshape(-1, 1)
            r_1 = r_0 + 1
            r_0 = torch.clamp(r_0, 0, img.shape[1]-1)
            r_1 = torch.clamp(r_1, 0, img.shape[1]-1)
            Ia = img[az, r_0]
            Ib = img[az, r_1]
            interp = (1-shift_frac).reshape(-1,1)*Ia + shift_frac.reshape(-1,1)*Ib
            return interp

    # Correcting the scan polar coordinates to cartesian coordinates based on the per azimuth poses for the direct cost function
    def polarToCartCoordCorrectionSparse(self, pos, rot, doppler_shift):
        with torch.no_grad():
            # Get the polar coordinates of the image
            c_az_min = torch.cos(self.azimuths)
            s_az_min = torch.sin(self.azimuths)
            c_az = c_az_min[self.direct_az_ids_sparse]
            s_az = s_az_min[self.direct_az_ids_sparse]
            even_range = self.range_vec[self.direct_r_ids_even] - doppler_shift[self.direct_az_ids_even] * self.shift_to_range
            odd_range = self.range_vec[self.direct_r_ids_odd] + doppler_shift[self.direct_az_ids_odd] * self.shift_to_range
            x = torch.empty(self.direct_nb_non_zero, device=self.device)
            x[self.mask_direct_even] = c_az[self.mask_direct_even] * even_range
            x[self.mask_direct_odd] = c_az[self.mask_direct_odd] * odd_range
            y = torch.empty(self.direct_nb_non_zero, device=self.device)
            y[self.mask_direct_even] = s_az[self.mask_direct_even] * even_range
            y[self.mask_direct_odd] = s_az[self.mask_direct_odd] * odd_range


            # Rotate the coordinates
            c_rot_min = torch.cos(rot.squeeze())
            s_rot_min = torch.sin(rot.squeeze())
            c_rot = c_rot_min[self.direct_az_ids_sparse]
            s_rot = s_rot_min[self.direct_az_ids_sparse]
            x_c_rot = x * c_rot
            y_s_rot = y * s_rot
            x_s_rot = x * s_rot
            y_c_rot = y * c_rot
            x_rot = x_c_rot - y_s_rot
            y_rot = x_s_rot + y_c_rot

            # Translate the coordinates
            x_trans = x_rot + pos.squeeze()[self.direct_az_ids_sparse, 0]
            y_trans = y_rot + pos.squeeze()[self.direct_az_ids_sparse, 1]

            # Stack the coordinates
            cart = torch.stack((x_trans.unsqueeze(-1), y_trans.unsqueeze(-1)), dim=1)

            # Compute the jacobians
            d_cart_d_rot = torch.zeros((self.direct_nb_non_zero, 2, 1), device=self.device)
            d_cart_d_rot[:, 0, 0] = -y_rot
            d_cart_d_rot[:, 1, 0] = x_rot


            d_cart_d_shift = torch.empty((self.nb_azimuths, 2, 1), device=self.device)
            d_cart_d_shift[::2, 0, 0] = c_az_min[::2]*-self.shift_to_range
            d_cart_d_shift[1::2, 0, 0] = c_az_min[1::2]*self.shift_to_range
            d_cart_d_shift[::2, 1, 0] = s_az_min[::2]*-self.shift_to_range
            d_cart_d_shift[1::2, 1, 0] = s_az_min[1::2]*self.shift_to_range

            d_trans_d_cart = torch.empty((self.nb_azimuths, 2, 2), device=self.device)
            d_trans_d_cart[:,0,0] = c_rot_min
            d_trans_d_cart[:,0,1] = -s_rot_min
            d_trans_d_cart[:,1,0] = s_rot_min
            d_trans_d_cart[:,1,1] = c_rot_min


            d_cart_d_shift = d_trans_d_cart @ d_cart_d_shift

            return cart, d_cart_d_rot, d_cart_d_shift


    # Correcting the scan polar coordinates to cartesian coordinates based on the per azimuth poses
    # (used for scan undistortion before updating the local map)
    def polarCoordCorrection(self, pos, rot):
        with torch.no_grad():
            # Get the polar coordinates of the image
            polar_coord = self.polar_coord_raw_gp_infered
            pos = pos.reshape((-1, 2))
            rot = rot.reshape((-1, 1))

            c_az = torch.cos(polar_coord[:, :, 0])
            s_az = torch.sin(polar_coord[:, :, 0])
            x = c_az * polar_coord[:, :, 1]
            y = s_az * polar_coord[:, :, 1]

            # Rotate the coordinates
            c_rot = torch.cos(rot)
            s_rot = torch.sin(rot)
            x_rot = x * c_rot - y * s_rot
            y_rot = x * s_rot + y * c_rot

            # Translate the coordinates
            x_trans = x_rot + pos[:, 0].unsqueeze(1)
            y_trans = y_rot + pos[:, 1].unsqueeze(1)

            # Get the new polar coordinates
            polar = torch.zeros((self.nb_azimuths, polar_coord.shape[1], 2)).to(self.device)
            polar[:, :, 0] = torch.atan2(y_trans, x_trans)
            sq_norm = x_trans**2 + y_trans**2
            polar[:, :, 1] = torch.sqrt(sq_norm)

            return polar



    # Perform the linear interpolation of the image per row based on the estimated Doppler shift
    # (used in Doppler-based velocity constraint)
    def imgDopplerInterpAndJacobian(self, shift):
        with torch.no_grad():
            shift_int = torch.floor(-shift).int()
            bin_mat_shifted_int = self.doppler_bin_vec_sparse + shift_int[self.doppler_az_ids_sparse]
            shift_frac = (-shift - shift_int)[self.doppler_az_ids_sparse]
            aligned_odd_coeff = self.odd_coeff[self.doppler_az_ids_sparse, bin_mat_shifted_int]
            odd_interp = shift_frac*aligned_odd_coeff + self.odd_bias[self.doppler_az_ids_sparse, bin_mat_shifted_int]

            return odd_interp, -aligned_odd_coeff


    # Gradient ascent solver for the state estimation
    def solve(self, state_init, nb_iter=20, cost_tol=1e-6, step_tol=1e-6, degraded=False, doppler_only=False):
        with torch.no_grad():
            # As there is no local map yet at the first scan, we remove the angular velocity
            # from the state (if any)
            if (not self.use_gyro) and (self.step_counter == 0 or doppler_only):
                remove_angular = torch.tensor(True).to(self.device)
                doppler_only = True
            else:
                remove_angular = torch.tensor(False).to(self.device)
            # If there is no local map yet and no Doppler cost, we return the initial state
            # (no registration possible yet)
            if self.step_counter == 0 and not self.use_doppler:
                return state_init

            # The gradient ascent keep track of the last increasing state and gradient
            # Thus, if the cost function decreases, we go back to the last increasing
            # state and reduce the step size
            state = state_init.clone()
            first_cost = torch.tensor(np.inf).to(self.device)
            prev_cost = first_cost
            first_quantum = self.kOptFirstStep
            step_quantum = first_quantum
            last_increasing_state = state.clone()
            last_increasing_grad = torch.zeros_like(state)
            for i in torch.arange(nb_iter, device=self.device):
                
                res, jac = self.costFunctionAndJacobian(state, self.use_doppler, (not doppler_only) and (self.step_counter > 0), degraded)

                if remove_angular and not self.use_gyro:
                    jac = jac[:, :-1]


                grad = 3*torch.sum(res.flatten().unsqueeze(-1)**2 * jac.reshape((-1,jac.shape[-1])), 0)
                cost = torch.sum((res**3).flatten())

                if i == 0:
                    last_increasing_grad = grad.clone()
                else:
                    if cost < prev_cost:
                        state = last_increasing_state.clone()
                        grad = last_increasing_grad.clone()
                        step_quantum = step_quantum / 2
                    else:
                        last_increasing_state = state.clone()
                        last_increasing_grad = grad.clone()

                grad_norm = torch.linalg.norm(grad)

                if step_quantum < 1e-5:
                    break


                if grad_norm < 1e-9:
                    break
                step = (grad / grad_norm) * step_quantum

                
                if remove_angular and not self.use_gyro:
                    step = torch.cat((step, torch.zeros(1).to(self.device)), dim=0)
                
                state += step

                step_norm = torch.linalg.norm(step)
                cost_change = cost - prev_cost

                if i == 0:
                    first_cost = cost

                if step_norm < step_tol:
                    break

                if torch.abs(cost_change/cost) < cost_tol:
                    break
                prev_cost = cost


            vel, _, _ = self.motion_model.getVelPosRot(state, with_jac=False)
            try_degraded = (isinstance(self.motion_model, ConstVelConstW)) and (torch.abs(state[2]) > maxAngVel(state[:2]))
            try_degraded = try_degraded or (torch.abs(torch.norm(vel[-1,:]) - self.previous_vel) > self.max_diff_vel)
            if try_degraded:
                if not degraded:
                    state = self.solve(state_init, nb_iter=nb_iter, cost_tol=cost_tol, step_tol=step_tol, degraded=True)

            if not degraded:
                vel, _, _ = self.motion_model.getVelPosRot(state, with_jac=False)
                self.previous_vel = torch.norm(vel[-1,:])
                self.max_diff_vel = self.motion_model.time[-1] * self.max_acc
            
            
            return state

    # Helper function to get the local map indices from the cartesian coordinates
    def cartToLocalMapID(self, xy):
        with torch.no_grad():
            out = torch.empty_like(xy, device=self.device)
            out[:,:,0,0] = (xy[:,:,0,0] / (-self.local_map_res)) + self.local_map_zero_idx
            out[:,:,1,0] = (xy[:,:,1,0] / (self.local_map_res)) + self.local_map_zero_idx
            return out

    # Same as cartToLocalMapID_ but for the sparse case
    def cartToLocalMapIDSparse(self, xy):
        with torch.no_grad():
            out = torch.empty_like(xy, device=self.device)
            out[:,0,0] = (xy[:,0,0] / (-self.local_map_res)) + self.local_map_zero_idx
            out[:,1,0] = (xy[:,1,0] / (self.local_map_res)) + self.local_map_zero_idx
            return out



    # Move localMap to the new position and rotation (used for updating the local map)
    def moveLocalMap(self, pos, rot):
        with torch.no_grad():
            # Set to zero the first and last row and column of the localMap
            self.local_map[0, :] = 0
            self.local_map[-1, :] = 0
            self.local_map[:, 0] = 0
            self.local_map[:, -1] = 0

            # Get the coordinate of the new localMap in the former localMap
            temp_rot_mat = torch.tensor([[torch.cos(rot), -torch.sin(rot)], [torch.sin(rot), torch.cos(rot)]]).to(self.device)
            temp_pos = pos.reshape((-1,1))

            # Get the new coordinates
            new_xy = temp_rot_mat @ self.local_map_xy + temp_pos
            new_idx = self.cartToLocalMapID(new_xy)

            # Get the new localMap via bilinear interpolation
            self.local_map = self.bilinearInterpolation(self.local_map, new_idx).squeeze().float()



    # Get the Doppler velocity separately from the odometry step for the tuning of lateral velocity bias
    def getDopplerVelocity(self):
        with torch.no_grad():
            if not self.use_doppler:
                raise ValueError("Doppler not used")
            result = self.solve(self.state_init, self.opts['solver']['nb_iter'], self.opts['solver']['cost_tol'], self.opts['solver']['step_tol'], doppler_only=True)
            return result[:2].detach().cpu().numpy()

    # Pull the state estimate
    def getAzPosRot(self):
        with torch.no_grad():
            rot_mat = torch.tensor([[torch.cos(self.current_rot), -torch.sin(self.current_rot)], [torch.sin(self.current_rot), torch.cos(self.current_rot)]]).to(self.device)

            _, scan_pos, scan_rot = self.motion_model.getVelPosRot(self.state_init, with_jac=False)
            pos = rot_mat @ scan_pos.double() + self.current_pos.unsqueeze(1)
            rot = scan_rot.double() + self.current_rot

            return pos.detach().cpu().numpy(), rot.detach().cpu().numpy()

    def getPose(self, time):
        with torch.no_grad():
            frame_pos, frame_rot = self.motion_model.getPosRotSingle(self.state_init, time)
            frame_pos = frame_pos.detach().cpu().numpy().astype(np.float64)
            frame_rot = frame_rot.detach().cpu().numpy().astype(np.float64)

            c_rot = np.cos(self.current_rot.detach().cpu().numpy().astype(np.float64))
            s_rot = np.sin(self.current_rot.detach().cpu().numpy().astype(np.float64))
            rot_mat = np.array([[c_rot, -s_rot], [s_rot, c_rot]])
            pos = (rot_mat @ frame_pos.T).T + self.current_pos.detach().cpu().numpy().astype(np.float64)
            rot = frame_rot + self.current_rot.detach().cpu().numpy().astype(np.float64)
            pose = np.zeros((4,4), dtype=np.float64)
            pose[0:2,0:2] = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])
            pose[0:2,3] = pos
            pose[2,2] = 1.0
            pose[3,3] = 1.0

            return pose



# Euristic on the maximum angular velocity acceptable for a given velocity
# (used to detect degraded mode when not using a gyro)
def maxAngVel(vel):
    min_ang_vel = 0.15
    max_ang_vel = 1.0
    max_vel = 20
    min_vel = 10
    vel_norm = torch.norm(vel)
    if vel_norm < min_vel:
        return max_ang_vel
    elif vel_norm > max_vel:
        return min_ang_vel
    else:
        a = (min_ang_vel - max_ang_vel) / (max_vel - min_vel)
        b = max_ang_vel - a*min_vel 
        return a*vel_norm + b

