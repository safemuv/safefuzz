#!/usr/bin/env python3
# NB. I tested this in a REPL in the docker container...

import pymesh
# Used blender to convert the DAE model to a .obj file
mesh = pymesh.load_mesh("boeing2.obj")

# The point to be tested - needs to be in the correct reference frame for the
# aircraft. Note when I loaded it into Blender, the aircraft seemed to be
# on its side as if it was banking 90 degrees, so the orientation must also
# be transformed
testpoint = np.array([2.6, 4.2, -1.0])

# Run the distance query and get its results as an array
# https://pymesh.readthedocs.io/en/latest/api_geometry_processing.html#distance-to-mesh-query
dist_res = pymesh.distance_to_mesh(mesh, [testpoint])

# Get the squared distance
sqr_dist = dist_res[0]

print("Test point = " + str(testpoint) + "Squared dist = " + str(sqr_dist))

# Now publish sqr_dist on a topic

