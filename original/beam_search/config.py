# Relative paths from src folder
INPUT_FILE_PATH = '../../examples/Bunny-LowPoly.stl'
OUTPUT_FOLDER = '../../output'





### Parameters ###
# Number of bsp's considered at one time
BEAM_WIDTH = 4

# Spacing of the uniform planes when cutting
PLANE_SPACER = 25

# Print Volume (w, h, d) (in mm)
PRINT_VOLUME = (100, 100, 100)

### Objective Function Weights ###
PART_WEIGHT = 1
UTIL_WEIGHT = 0.05
CONNECTOR_WEIGHT = 1
FRAGILITY_WEIGHT = 1
SEAM_WEIGHT = 0.1
SYMMETRY_WEIGHT = 0.25

