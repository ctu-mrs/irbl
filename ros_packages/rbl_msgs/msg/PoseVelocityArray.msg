# States (pose + velocity) of the other UAVs in the group, expressed in header.frame_id.
# Field-for-field identical to filter_reflective_uavs/msg/PoseVelocityArray, so RBL can be fed
# group states without depending on that package.
std_msgs/Header header
int32[] ids
geometry_msgs/Pose[] poses
geometry_msgs/Vector3[] velocities
