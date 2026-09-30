"""Loader for the CSVs written by extract_bags.py (numpy only)."""
import os

import numpy as np


def load_csv(path):
    a = np.genfromtxt(path, delimiter=",", names=True)
    return a


class Trial:
    def __init__(self, data_dir, name):
        self.name = name
        d = os.path.join(data_dir, name)
        self.lily = load_csv(os.path.join(d, "lily_odom.csv"))
        self.naut = load_csv(os.path.join(d, "naut_odom.csv"))
        self.desired = load_csv(os.path.join(d, "desired_vel.csv"))
        self.cmd = load_csv(os.path.join(d, "cmd_vel.csv"))
        self.elements = load_csv(os.path.join(d, "elements.csv"))
        self.thrust_l = load_csv(os.path.join(d, "thrust_left.csv"))
        self.thrust_r = load_csv(os.path.join(d, "thrust_right.csv"))
        self.t0 = self.lily["t"][0]
