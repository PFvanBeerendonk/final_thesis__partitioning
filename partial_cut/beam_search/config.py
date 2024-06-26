# Relative paths from src folder
INPUT_FILE_PATH = '../../models/tue_logo.stl'
OUTPUT_FOLDER = '../../output'





### Parameters ###
# Number of bsp's considered at one time
BEAM_WIDTH = 3

# Spacing of the uniform planes when cutting (in mm)
PLANE_SPACER = 10

# Print Volume (w, h, d) (in mm)
# set the same as the Anycubic Photon Mono 4k
PRINT_VOLUME = (132, 165, 80)
# Radius of the connector (in mm)
CONNECTOR_RADIUS = 10

### Objective Function Weights and constants ###
PART_WEIGHT = 1
UTIL_WEIGHT = 0.05
SEAM_WEIGHT = 0.1

