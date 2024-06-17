# Relative paths from src folder
INPUT_FILE_PATH = '../../examples/tue_logo__by_pieter.stl'
OUTPUT_FOLDER = '../../output'





### Parameters ###
# Number of bsp's considered at one time
BEAM_WIDTH = 4

# Spacing of the uniform planes when cutting (in mm)
PLANE_SPACER = 25

# Print Volume (w, h, d) (in mm)
PRINT_VOLUME = (100, 100, 100)
# Radius of the connector (in mm)
CONNECTOR_RADIUS = 10

### Objective Function Weights and constants ###
PART_WEIGHT = 1
UTIL_WEIGHT = 0.05
CONNECTOR_WEIGHT = 1
CONNECTOR_CONSTANT = 10
# FRAGILITY_WEIGHT = 1
# SEAM_WEIGHT = 0.1
# SYMMETRY_WEIGHT = 0.25

