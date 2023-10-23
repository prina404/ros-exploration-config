
define hokuyolaser ranger
(
	sensor
	(
		# laser-specific properties
		range [ 0.0  15.0] # maximum range, actual detection up to 30m is set inside launch file via maxURange
		fov 360.0
		samples 800
	)

	# generic model properties
	color "blue"
	size [ 0.01 0.01 0.01 ] # dimensions from LMS200 data sheet
)


define floorplan model
(
	color "gray30"

	boundary 0

	gui_nose 0
	gui_grid 0
	gui_move 0
	gui_outline 0

	gripper_return 0
	fiducial_return 0
	laser_return 1
)

define pioneer3at position
(
	color "red"          # Default color.
	drive "diff"         # Differential steering model.
	gui_nose 1           # Draw a nose on the robot so we can see which way it points
	obstacle_return 1    # Can hit things.
	laser_return 1	     # reflects sonar beams


	# alternative odometric localization with simple error model
	localization "odom"                 # Change to "gps" to have impossibly perfect, global odometry
	odom_error [ 0 0 0 0]      # Odometry error or slip in X, Y and Theta (Uniform random distribution)
	#localization "gps"


	# Actual size
	size [0.02 0.02 0.04]

	# The pioneer's center of rotation is offset from its center of area
	origin [0 0 0 0]

	# Estimated mass in KG
	mass 2.0

	hokuyolaser()
)

interval_sim 100  # simulation timestep in milliseconds
speedup 5

