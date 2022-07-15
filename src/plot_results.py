#!/usr/bin/python


import matplotlib.pyplot as plot
import numpy
import sys
from optparse import OptionParser

parser = OptionParser()
parser.add_option("--imu_data", dest="imu_data",
                  default="", help="The filename that contains the original imudata.")
(options, args) = parser.parse_args()

imu_original = None
if options.imu_data != '':
  imu_original = numpy.genfromtxt(options.imu_data, usecols = (0,1,2,3,4,5,6))



times  =  imu_original[:, 0]
times_offset =  times- times[0]

#corr = np.correlate(omega_predicted_norm, omega_measured_norm, "full")
#discrete_shift = corr.argmax() - (np.size(omega_measured_norm) - 1)

# if imu_original is not None:
plot.plot(times_offset,  numpy.clip(imu_original[:, 3],-1,1)  , '-', label="predict",
            alpha=0.5, color="blue")
plot.plot(times_offset, numpy.clip(imu_original[:, 6],-1,1), '-', label="original",
            alpha=0.5, color="red")
# plot.axis('equal')
plot.legend()
# Show the plot and wait for the user to close.
plot.show()
