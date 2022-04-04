import numpy as np
import gtsam
import matplotlib.pyplot as plt
from Load_Data_2D import edges_vertices_and_info, sensor_xy #Import functions from Load_Data to load poses, edges, info and sensor x and y coordinates from .G2O file

poses,edges,info = edges_vertices_and_info() #Load poses, edges and info

isam = gtsam.ISAM2() #Initialise gtsam.isam2()
for pose in poses:
    graph = gtsam.NonlinearFactorGraph() #Initialise the graph
    initialEstimate = gtsam.Values()
    (idp,x,y,theta) = pose #Load idp, x, y and theta for every time step reading from the .g2o file
    if idp ==0: #Initial state
        priorNoise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.3, 0.3, 0.1])) #Generate noise matrix
        graph.add(gtsam.PriorFactorPose2(0,gtsam.Pose2(x,y,theta),priorNoise)) #Initialise the first point on the graph using first reading of the .g2o file and generated noise
        initialEstimate.insert(int(idp),gtsam.Pose2(x,y,theta)) #Insert pose in the generated Graph

    else:                                           #For consecutive sensor readings (poses/vertex)
        prevPose = result.atPose2(int(idp-1))  #Calculate prev pose from pre-generated (/updated) result
        initialEstimate.insert(int(idp),prevPose) #Insert it into the graph
        for edge in edges:
            ide1,ide2,dx,dy,dtheta,info1,info2,info3,info4,info5,info6 = edge #Load data for each edge
            info = [ [info1,info2,info3],[info2,info4,info5],[info3,info5,info6]] #Load the information as a matrix (see formula in PDF below the algorithm)
            if ide2 == idp: #If intersection occurs (that is the robot comes back to the same position as before)
                cov = np.linalg.inv(info) #Generate covariance by taking inverse of the information matrix
                Model = gtsam.noiseModel.Gaussian.Covariance(cov) #Develop Gaussian smooth model with defined covariance
                graph.add(gtsam.BetweenFactorPose2(int(ide1),int(ide2),gtsam.Pose2(dx,dy,dtheta),Model)) #Add the constrainsts with the information noise model

    isam.update(graph,initialEstimate) #Update the graph for each pose
    result = isam.calculateEstimate() #Calcualate result for each pose

def generate_xy_updated(result):
    poses = [result.atPose2(i) for i in range(result.size())]
    return np.array( [ [pose.x(), pose.y(), pose.theta()] for pose in poses])


updated_x_y = generate_xy_updated(result) #From NonLinear Graph Updates
initial_x_y = sensor_xy() #From Load_Data

#Generate Plot for Updated (Optimised) and Sensor (Unoptimised) Trajectory
plt.figure()
plt.title("2D Incremental Trajectory Optimization")
plt.plot(initial_x_y[:,0],initial_x_y[:,1],linestyle="-",label = "Unoptimised Trajectory", color = '#1f77b4')
plt.plot(updated_x_y[:,0],updated_x_y[:,1],linestyle="-",label="Optimised Trajectory", color='#ff7f0e')

plt.legend()
plt.axis("equal")
plt.show()
