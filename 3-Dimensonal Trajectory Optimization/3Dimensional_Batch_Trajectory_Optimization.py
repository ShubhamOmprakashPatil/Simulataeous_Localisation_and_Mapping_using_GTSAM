import numpy as np
import gtsam
import matplotlib.pyplot as plt
from Load_Data_3D import edges_vertices_and_info, sensor_xyz #Import functions from Load_Data to load poses, edges, info and sensor x and y coordinates from .G2O file

is3D = True
isam = gtsam.ISAM2() #Initialise gtsam.isam2()
graph, initial = gtsam.readG2o("parking-garage.g2o", is3D)
priorModel = gtsam.noiseModel.Diagonal.Variances(np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4]))
firstKey = initial.keys()[0]
graph.add(gtsam.PriorFactorPose3(firstKey, gtsam.Pose3(), priorModel))
params = gtsam.GaussNewtonParams()
optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
result = optimizer.optimize()

def generate_xy_updated(result):
    poses = [result.atPose3(i) for i in range(result.size())]
    return np.array( [ [pose.x(), pose.y(), pose.z()] for pose in poses])


updated_x_y = generate_xy_updated(result) #From NonLinear Graph Updates
initial_x_y = sensor_xyz() #From Load_Data

#Generate Plot for Updated (Optimised) and Sensor (Unoptimised) Trajectory
plt.figure()

ax = plt.axes(projection='3d')
ax.plot3D(initial_x_y[:,0],initial_x_y[:,1],initial_x_y[:,2],linestyle="-",label = "Unoptimised Trajectory",color = '#1f77b4')
ax.plot3D(updated_x_y[:,0],updated_x_y[:,1], updated_x_y[:,2],linestyle="-",label="Optimised Trajectory",color='#ff7f0e')
ax.set_xlim3d(-200, 50)
ax.set_ylim3d(0, 250)
ax.set_zlim3d(-10, 5)
plt.title("3-Dimensional Batch Trajectory Optimization")
plt.legend()
plt.grid(False)
plt.show()
