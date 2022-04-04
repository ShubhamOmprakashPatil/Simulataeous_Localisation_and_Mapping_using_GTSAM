import numpy as np
import gtsam
import matplotlib.pyplot as plt
from Problem2_a import edges_vertices_and_info, sensor_xyz #Import functions from Problem2a to load poses, edges, info and sensor x and y coordinates from .G2O file

poses,edges,info = edges_vertices_and_info() #Load poses, edges and info

def quaternion_rotation_matrix(Q):
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix

isam = gtsam.ISAM2() #Initialise gtsam.isam2()
for pose in poses:
    graph = gtsam.NonlinearFactorGraph() #Initialise the graph
    initialEstimate = gtsam.Values()
    (idp,x,y,z,qx,qy,qz,qw) = pose #Load idp, x, y and theta for every time step reading from the .g2o file
    if idp ==0: #Initial state
        priorNoise = gtsam.noiseModel.Diagonal.Variances(np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4])) #Generate noise matrix
        QQ = np.array([qw,qx,qy,qz])
        rot_matrix = quaternion_rotation_matrix(QQ) #Convert Quartanion to rotation matrix #VERY IMPORTANT STEP
        point3 = np.array([x,y,z]) #Translation
        point3 = np.reshape(point3,(3,1))
        graph.add(gtsam.PriorFactorPose3(0,gtsam.Pose3(gtsam.Rot3(rot_matrix),point3),priorNoise)) #Initialise the first point on the graph using first reading of the .g2o file and generated noise
        initialEstimate.insert(int(idp),gtsam.Pose3(gtsam.Rot3(rot_matrix),point3)) #Insert pose in the generated Graph

    else:                                           #For consecutive sensor readings (poses/vertex)
        prevPose = result.atPose3(int(idp-1))  #Calculate prev pose from pre-generated (/updated) result
        initialEstimate.insert(int(idp),prevPose) #Insert it into the graph
        for edge in edges:
            ide1,ide2,dx,dy,dz,dqx,dqy,dqz,dqw,info1,info2,info3,info4,info5,info6,info7,info8,info9,info10,info11,info12,info13,info14,info15,info16,info17,info18,info19,info20,info21 = edge #Load data for each edge
            info = [  [info1, info2, info3, info4, info5, info6],  #Generate the Information Matrix
                      [info2, info7, info8, info9, info10, info11],
                      [info3, info8, info12, info13, info14, info15],
                      [info4, info9,info13,info16,info17,info18],
                      [info5,info10,info14,info17,info19,info20],
                      [info6,info11,info15,info18,info20,info21]   ]


            if ide2 == idp: #If intersection occurs (that is the robot comes back to the same position as before)
                cov = np.linalg.inv(info) #Generate covariance by taking inverse of the information matrix
                Model = gtsam.noiseModel.Gaussian.Covariance(cov) #Develop Gaussian smooth model with defined covariance
                QQQ = np.array([dqw,dqx,dqy,dqz])
                rot_matrix_2 = quaternion_rotation_matrix(QQQ)
                point3_2 = np.array([dx,dy,dz])
                point3_2 = np.reshape(point3_2,(3,1))
                graph.add(gtsam.BetweenFactorPose3(int(ide1),int(ide2),gtsam.Pose3(gtsam.Rot3(rot_matrix_2),point3_2),Model)) #Add the constrainsts

    isam.update(graph,initialEstimate) #Update the graph for each pose
    result = isam.calculateEstimate() #Calcualate result for each pose


def generate_xy_updated(result):
    poses = [result.atPose3(i) for i in range(result.size())]
    return np.array( [ [pose.x(), pose.y(), pose.z()] for pose in poses])


updated_x_y = generate_xy_updated(result) #From NonLinear Graph Updates
initial_x_y = sensor_xyz() #From Problem 1a

#Generate Plot for Updated (Optimised) and Sensor (Unoptimised) Trajectory
plt.figure()

ax = plt.axes(projection='3d')
ax.plot3D(initial_x_y[:,0],initial_x_y[:,1],initial_x_y[:,2],linestyle="-",label = "Unoptimised Trajectory",color = '#1f77b4')
ax.plot3D(updated_x_y[:,0],updated_x_y[:,1], updated_x_y[:,2],linestyle="-",label="Optimised Trajectory",color='#ff7f0e')
ax.set_xlim3d(-200, 50)
ax.set_ylim3d(0, 250)
ax.set_zlim3d(-10, 5)
plt.title("Problem2_c")
plt.legend()

plt.show()
