#! /usr/bin/env python3

import os
import time
import datetime
import argparse
import opt_point
import sys
arg = sys.argv

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
arg.append("")
if arg[1] == "r_az":
    _sort = "r_az"
    print('***start observation in reverse Az sort mode***')    
elif arg[1] != '':
    _sort = 'el'
    print('***start observation in EL sort mode***')
else:
    _sort = 'az'
    print('***start observation in Az sort mode (Defalut)***')
opt.start_observation(sort=_sort)





