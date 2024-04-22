import pyrealsense2 as rs

# Create a pipeline
pipeline = rs.pipeline()

# Start the pipeline
pipeline.start()

# Get the active device
device = pipeline.get_active_profile().get_device()

# Get the depth sensor
depth_sensor = device.first_depth_sensor()

# Get the intrinsic matrix of the depth sensor
intrinsics = depth_sensor.get_stream_intrinsics(rs.stream.depth)

# Print the intrinsic matrix
print("Intrinsic matrix:")
print("Width:", intrinsics.width)
print("Height:", intrinsics.height)
print("Fx:", intrinsics.fx)
print("Fy:", intrinsics.fy)
print("Cx:", intrinsics.ppx)
print("Cy:", intrinsics.ppy)
print("Coeffs:", intrinsics.coeffs)

# Stop the pipeline
pipeline.stop()
