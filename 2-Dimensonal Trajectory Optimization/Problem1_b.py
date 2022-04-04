import numpy as np
import gtsam
import matplotlib.pyplot as plt
from Problem1_a import sensor_xy #Import functions from Problem1a to load sensor x and y coordinates from .G2O file

#poses,edges,info = edges_vertices_and_info() #Load poses, edges and info
is3D = False

graph, initial = gtsam.readG2o("input_INTEL_g2o.g2o", is3D)
priorModel = gtsam.noiseModel.Diagonal.Variances(gtsam.Point3(1e-6, 1e-6, 1e-8))
graph.add(gtsam.PriorFactorPose2(0, gtsam.Pose2(), priorModel))
params = gtsam.GaussNewtonParams()
optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
result = optimizer.optimize()

def generate_xy_updated(result):
    poses = [result.atPose2(i) for i in range(result.size())]
    return np.array( [ [pose.x(), pose.y(), pose.theta()] for pose in poses])


updated_x_y = generate_xy_updated(result) #From NonLinear Graph Updates
initial_x_y = sensor_xy() #From Problem 1a

#Generate Plot for Updated (Optimised) and Sensor (Unoptimised) Trajectory
plt.figure()
plt.title("Problem1_b")
plt.plot(initial_x_y[:,0],initial_x_y[:,1],linestyle="-",label = "Unoptimised Trajectory", color = '#1f77b4')
plt.plot(updated_x_y[:,0],updated_x_y[:,1],linestyle="-",label="Optimised Trajectory", color='#ff7f0e')


plt.legend()
plt.axis("equal")
plt.show()
