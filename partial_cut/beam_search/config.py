# Relative paths from src folder
INPUT_FILE_PATH = '../../models/tue_logo.stl'
OUTPUT_FOLDER = '../../output'


### Parameters ###
# Number of bsp's considered at one time
BEAM_WIDTH = 3

# Spacing of the uniform planes when cutting (in mm)
PLANE_SPACER = 5
# Number of rays shot per vertex to calculate 
SEAM_OCCLUSION_RAY_COUNT = 20
# RMS distance threshold for "sufficiently different" w.r.t. diagonal of printing volume
SUF_DIFF_DISTANCE = 0.1
# difference in angle, multiplied by pi
SUF_DIFF_ANGLE = 0.1

# Print Volume (w, h, d) (in mm)
# set the same as the Anycubic Photon Mono 4k
PRINT_VOLUME = (132, 165, 80)


### Objective Function Weights and constants ###
PART_WEIGHT = 1
UTIL_WEIGHT = 0.1
SEAM_WEIGHT = 0

