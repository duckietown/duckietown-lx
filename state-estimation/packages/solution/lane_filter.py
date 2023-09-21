from collections import OrderedDict
from scipy.stats import multivariate_normal
import numpy as np
from scipy.ndimage.filters import gaussian_filter
from math import floor, sqrt

from solution.histogram_filter import histogram_update, histogram_predict, histogram_prior


class LaneFilterHistogram:
    """Generates an estimate of the lane pose.


    Creates and maintain a histogram grid filter to estimate the lane pose.
    Lane pose is defined as the tuple (`d`, `phi`) : lateral deviation and angulare deviation from the
    center of the lane.

    Predict step : Uses the estimated linear and angular velocities to predict the change in the lane pose.
    Update Step : The filter receives a segment list. For each segment, it extracts the corresponding lane
    pose "votes",
    and adds it to the corresponding part of the histogram.

    Best estimate correspond to the slot of the histogram with the highest voted value.

    Args:
        configuration (:obj:`List`): A list of the parameters for the filter

    """

    mean_d_0: float
    mean_phi_0: float
    sigma_d_0: float
    sigma_phi_0: float
    delta_d: float
    delta_phi: float
    d_max: float
    d_min: float
    phi_max: float
    phi_min: float
    cov_v: float
    linewidth_white: float
    linewidth_yellow: float
    lanewidth: float
    min_max: float
    sigma_d_mask: float
    sigma_phi_mask: float
    range_min: float
    range_est: float
    range_max: float

    def __init__(self, **kwargs):
        param_names = [
            "mean_d_0",
            "mean_phi_0",
            "sigma_d_0",
            "sigma_phi_0",
            "delta_d",
            "delta_phi",
            "d_max",
            "d_min",
            "phi_max",
            "phi_min",
            "linewidth_white",
            "linewidth_yellow",
            "lanewidth",
            "sigma_d_mask",
            "sigma_phi_mask",
            "range_min",
            "range_est",
            "range_max",
            "encoder_resolution",
            "wheel_radius",
            "wheel_baseline",
        ]

        for p_name in param_names:
            assert p_name in kwargs, (p_name, param_names, kwargs)
            setattr(self, p_name, kwargs[p_name])

        self.d, self.phi = np.mgrid[
            self.d_min : self.d_max : self.delta_d, self.phi_min : self.phi_max : self.delta_phi
        ]
        self.grid_spec = {
            "d": self.d,
            "phi": self.phi,
            "delta_d": self.delta_d,
            "delta_phi": self.delta_phi,
            "d_min": self.d_min,
            "d_max": self.d_max,
            "phi_min": self.phi_min,
            "phi_max": self.phi_max,
            "range_min": self.range_min,
            "range_est": self.range_est,
            "range_max": self.range_max,
        }
        self.road_spec = {
            "linewidth_white": self.linewidth_white,
            "linewidth_yellow": self.linewidth_yellow,
            "lanewidth": self.lanewidth,
        }
        self.robot_spec = {
            "wheel_radius": self.wheel_radius,
            "wheel_baseline": self.wheel_baseline,
            "encoder_resolution": self.encoder_resolution,
        }
        self.belief = np.empty(self.d.shape)
        self.mean_0 = [self.mean_d_0, self.mean_phi_0]
        self.cov_0 = [[self.sigma_d_0, 0], [0, self.sigma_phi_0]]
        self.cov_mask = [self.sigma_d_mask, self.sigma_phi_mask]

        # Additional variables
        self.initialized = False
        self.initialize()

    def initialize(self):
        self.belief = histogram_prior(self.belief, self.grid_spec, self.mean_0, self.cov_0)
        self.initialized = True

    def predict(self, left_encoder_delta_ticks, right_encoder_delta_ticks):
        if not self.initialized:
            return
        self.belief = histogram_predict(
            self.belief,
            left_encoder_delta_ticks,
            right_encoder_delta_ticks,
            self.grid_spec,
            self.robot_spec,
            self.cov_mask,
        )

    def update(self, segments):
        if not self.initialized:
            return
        (measurement_likelihood, self.belief) = histogram_update(
            self.belief, segments, self.road_spec, self.grid_spec
        )

    def getEstimate(self):
        maxids = np.unravel_index(self.belief.argmax(), self.belief.shape)
        d_max = self.d_min + (maxids[0] + 0.5) * self.delta_d
        phi_max = self.phi_min + (maxids[1] + 0.5) * self.delta_phi

        return [d_max, phi_max]
