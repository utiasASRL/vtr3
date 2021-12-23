import numpy as np


class Line:

  def __init__(self, cloud, line_pts, m, b):
    self.m = m
    self.b = b

    idx = line_pts[0]
    start_idx = idx
    self.start = np.sqrt(pow(cloud[start_idx, 0], 2) + pow(cloud[start_idx, 1], 2))
    idx = line_pts[-1]
    end_idx = idx
    self.end = np.sqrt(pow(cloud[end_idx, 0], 2) + pow(cloud[end_idx, 1], 2))


class Himmelsbach:

  def __init__(self, points):
    # points (np.ndarray): (N, 6) [x, y, z, intensity, laser_number, time]
    self.points = points
    self.size = points.shape[0]

    self.z_offset = 2.13

    self.alpha = 2.0 * np.pi / 180.0
    self.tolerance = 0.25
    self.Tm = 0.4
    self.Tm_small = 0.2
    self.Tb = 0.8
    self.Trmse = 0.1
    self.Tdprev = 1.0

    self.rmin = 3.0
    self.num_bins_small = 30
    self.bin_size_small = 3.0
    self.num_bins_large = 30
    self.bin_size_large = 3.0

    self.n_bins = self.num_bins_small + self.num_bins_large

    self.segments = []

  def set_alpha(self, alpha):
    self.alpha = alpha

  def set_tolerance(self, tolerance):
    self.tolerance = tolerance

  def set_thresholds(self, Tm, Tm_small, Tb, Trmse, Tdprev):
    self.Tm = Tm
    self.Tm_small = Tm_small
    self.Tb = Tb
    self.Trmse = Trmse
    self.Tdprev = Tdprev

  def sort_points_segments(self):
    num_segments = int(np.ceil((2 * np.pi) / self.alpha))
    self.segments = [[] for i in range(num_segments)]
    for i in range(self.size):
      point = self.points[i, :]
      angle = np.arctan2(point[1], point[0])
      if angle < 0:
        angle += 2 * np.pi
      segment = int(np.floor(angle / self.alpha))
      self.segments[segment].append(i)

  def sort_points_bins(self, segment):
    bins = [[] for i in range(self.n_bins)]
    rsmall = self.rmin + self.bin_size_small * self.num_bins_small
    rlarge = rsmall + self.bin_size_large * self.num_bins_large
    for idx in segment:
      point = self.points[idx, :]
      r = np.sqrt(point[0]**2 + point[1]**2)
      bin = -1
      if self.rmin <= r and r < rsmall:
        bin = (r - self.rmin) / self.bin_size_small
      elif rsmall <= r and r < rlarge:
        bin = self.num_bins_small + (r - rsmall) / self.bin_size_large
      if bin >= 0:
        bins[int(bin)].append(idx)
    # The point with the lowest z-coordinate in each bin becomes the representative point
    bins_out = [-1 for i in range(self.n_bins)]
    i = 0
    for bin_pts in bins:
      zmin = 1e9
      lowest = -1
      for idx in bin_pts:
        point = self.points[idx, :]
        if point[2] < zmin:
          zmin = point[2]
          lowest = idx
      bins_out[i] = lowest
      i += 1

    return bins_out

  def fitline(self, line_set):
    A = np.zeros((len(line_set), 2))
    B = np.zeros((len(line_set), 1))
    for i in range(len(line_set)):
      idx = line_set[i]
      point = self.points[idx, :]
      A[i, 0] = np.sqrt(pow(point[0], 2) + pow(point[1], 2))
      A[i, 1] = 1
      B[i, 0] = point[2]
    m, b = np.linalg.lstsq(A, B, rcond=None)[0]
    #m,b = np.linalg.solve(np.dot(A.T, A), np.dot(A.T, B))

    x = np.array([m[0], b[0]])
    error = (A @ x - B.transpose())[0]
    return np.sqrt(error @ error / float(len(line_set))), m[0], b[0]

  def distpointline(self, line, idx):
    point = self.points[idx, :]
    r = np.sqrt(pow(point[0], 2) + pow(point[1], 2))
    return abs(point[2] - line.m * r - line.b) / np.sqrt(1 + pow(line.m, 2))

  def compute_model_and_get_inliers(self):
    ground_idx = []
    if (self.size == 0):
      return

    # Sort points into segments
    self.sort_points_segments()
    for segment in self.segments:
      # Sort points into bins
      if (len(segment) == 0):
        continue
      bins = self.sort_points_bins(segment)
      lines = []
      line_set = []
      c = 0
      i = 0
      while i < len(bins) - 1:
        idx = bins[i]
        if (idx < 0):
          i += 1
          continue
        elif (len(line_set) >= 2):
          rmse, m, b = self.fitline(line_set + [idx])
          if (abs(m) <= self.Tm and (abs(m) > self.Tm_small or abs(b + self.z_offset) <= self.Tb) and
              rmse <= self.Trmse):
            line_set.append(idx)
            i += 1
          else:
            rmse, m, b = self.fitline(line_set)
            line = Line(self.points, line_set, m, b)
            lines.append(line)
            print(f'm:{m} | b:{b}')
            line_set = []
            c += 1
        else:
          dprev = self.distpointline(lines[c - 1], idx) if len(lines) > 0 else -1
          if (dprev <= self.Tdprev or c == 0 or len(line_set) != 0):
            line_set.append(idx)
          i += 1
      if len(line_set) >= 2:
        rmse, m, b = self.fitline(line_set)
        line = Line(self.points, line_set, m, b)
        lines.append(line)
        line_set = []

      # Assign points as inliers if they are within a threshold of the ground model
      for idx in segment:
        # get line that's closest to the candidate point based on distance to endpoints
        closest = -1
        dmin = 10000
        point = self.points[idx, :]
        r = np.sqrt(pow(point[0], 2) + pow(point[1], 2))
        for i in range(len(lines)):
          d1 = abs(lines[i].start - r)
          d2 = abs(lines[i].end - r)
          if ((d1 < dmin or d2 < dmin) and (abs(lines[i].m) < self.Tm)):
            dmin = min(d1, d2)
            closest = i
        if (closest >= 0):
          e = self.distpointline(lines[closest], idx)
          if (e < self.tolerance):
            ground_idx.append(idx)
    return ground_idx
