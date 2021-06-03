string distortion_model     # distortion model associated with the camera
float64[] d_mat     # distortion parameters associated with the distortion_model for standard plumb bob, of the form: K1, K2, P1, P2, K3 etc.
float64[] k_mat     # intrinsic camera matrix. (f1 gamma px, 0 fy py, 0 0 1)