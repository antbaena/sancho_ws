std_msgs/Header header
uint32 id

geometry_msgs/Point[] keypoints         # [x, y, 0.0] (2D, z=0)
geometry_msgs/Point[] keypoints3d       # [x, y, z]

float32[17] scores
float32 avg_depth
