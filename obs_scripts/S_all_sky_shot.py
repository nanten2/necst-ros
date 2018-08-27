#! /usr/bin/env python3

import os
import time
import datetime
import argparse
import S_opt_point
import sys

# Info
# ----

name = 'all_sky_shot'
description = 'Get all sky shot data'

# Default parameters
# ------------------

# Argument handler
# ================

p = argparse.ArgumentParser(description=description)

# Main
# ====

name = "all_sky_shot"

opt = S_opt_point.opt_point_controller()
opt.start_observation()





