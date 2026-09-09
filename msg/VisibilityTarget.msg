# One target to be observed by the visibility device, as a ball.
#
# The single-target path (VisibilityTask/PlanVisibilityPath) instead sends a
# cloud of points that computeTargetMES() collapses into ONE enclosing sphere.
# VisTSP needs the targets kept apart -- each ball is its own cluster in the
# GTSP instance -- so it names each one explicitly.
geometry_msgs/Point center
float64 radius
