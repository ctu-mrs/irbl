# A control reference with a Position+Heading.
# Local replacement for mrs_msgs/Reference, with an identical wire format, so the irbl stack no
# longer needs the mrs_msgs package as a dependency.

geometry_msgs/Point position

# Heading is atan2() of XY-world projection of the UAV's body X-axis.
float64 heading
