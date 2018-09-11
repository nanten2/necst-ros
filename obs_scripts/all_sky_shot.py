#! /usr/bin/env python3

import os
import time
import datetime
import argparse
import opt_point
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

opt = opt_point.opt_point_controller()
_sort = input('Do you want to start observation in El sort mode ([y]/n)?')
if _sort == 'y':
    _sort = 'el'
    print('***start observation in EL sort mode***')
else:
    _sort = 'az'
    print('***start observation in Az sort mode (Defalut)***')
opt.start_observation(sort=_sort)





