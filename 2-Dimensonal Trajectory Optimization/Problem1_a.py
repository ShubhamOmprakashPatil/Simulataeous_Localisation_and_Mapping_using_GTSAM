import numpy as np
import gtsam
import matplotlib.pyplot as plt

#WRITE FUNCTION TO READ AND LOAD VERTICES AND EDGES FROM THE G2O FILE

def read_g20_vertices(fileName):
    f = open(fileName,'r') #Open .g2o file
    A = f.readlines()
    f.close()

    IND = []
    X = []
    Y = []
    THETA = []

    for line in A:
        if "VERTEX_SE2" in line:
            (ver,ind,x,y,theta) = line.split(' ') #Split numbers against the spaces between them
            IND.append(float(ind))
            X.append(float(x))
            Y.append(float(y))
            THETA.append(float(theta.rstrip('\n')))

    return (IND,X,Y,THETA)

def read_g20_edges(fileName):
    f = open(fileName,'r')
    A = f.readlines()
    f.close()

    IDE1 = []
    IDE2 = []
    DX = []
    DY = []
    DTHETA = []
    INFO1 = []
    INFO2 = []
    INFO3 = []
    INFO4 = []
    INFO5 = []
    INFO6 = []
    for line in A:
        if "EDGE_SE2" in line:
            (edge,ide1,ide2,dx,dy,dtheta,info1,info2,info3,info4,info5,info6) = line.split(' ')
            IDE1.append(float(ide1))
            IDE2.append(float(ide2))
            DX.append(float(dx))
            DY.append(float(dy))
            DTHETA.append(float(dtheta.rstrip('\n')))
            INFO1.append(float(info1))
            INFO2.append(float(info2))
            INFO3.append(float(info3))
            INFO4.append(float(info4))
            INFO5.append(float(info5))
            INFO6.append(float(info6))
    return (IDE1,IDE2,DX,DY,DTHETA,INFO1,INFO2,INFO3,INFO4,INFO5,INFO6)

def sensor_xy():
    #LOAD THE VERTICES AND EDGES
    (idp_, x_, y_, theta_) = read_g20_vertices("input_INTEL_g2o.g2o")
    (ide1_,ide2_,dx_,dy_,dtheta_,info1_,info2_,info3_,info4_,info5_,info6_) = read_g20_edges("input_INTEL_g2o.g2o")
    #CONVERT VERTICES TO NUMPY ARRAY
    idp_ = np.array(idp_)
    x_ = np.array(x_)
    y_ = np.array(y_)
    theta_ = np.reshape(theta_,(1228,1))

    #RESHAPE
    idp_ = np.reshape(idp_,(1228,1))
    x_ = np.reshape(x_,(1228,1))
    y_ = np.reshape(y_,(1228,1))
    initial_x_y = np.concatenate((x_,y_),axis=1)
    return initial_x_y


def edges_vertices_and_info():
    #LOAD THE VERTICES AND EDGES
    (idp_, x_, y_, theta_) = read_g20_vertices("input_INTEL_g2o.g2o")
    (ide1_,ide2_,dx_,dy_,dtheta_,info1_,info2_,info3_,info4_,info5_,info6_) = read_g20_edges("input_INTEL_g2o.g2o")
    #CONVERT VERTICES TO NUMPY ARRAY
    idp_ = np.array(idp_)
    x_ = np.array(x_)
    y_ = np.array(y_)
    theta_ = np.reshape(theta_,(1228,1))

    #RESHAPE
    idp_ = np.reshape(idp_,(1228,1))
    x_ = np.reshape(x_,(1228,1))
    y_ = np.reshape(y_,(1228,1))

    #CONVERT EDGES TO NUMPY ARRAY
    ide1_ = np.array(ide1_)
    ide2_ = np.array(ide2_)
    dx_ = np.array(dx_)
    dy_ = np.array(dy_)
    dtheta_ = np.array(dtheta_)
    info1_ = np.array(info1_)
    info2_ = np.array(info2_)
    info3_ = np.array(info3_)
    info4_ = np.array(info4_)
    info5_ = np.array(info5_)
    info6_ = np.array(info6_)

    #RESHAPE
    ide1_ = np.reshape(ide1_,(1483,1))
    ide2_ = np.reshape(ide2_,(1483,1))
    dx_ = np.reshape(dx_,(1483,1))
    dy_ = np.reshape(dy_,(1483,1))
    dtheta_ = np.reshape(dtheta_,(1483,1))
    info1_ = np.reshape(info1_,(1483,1))
    info2_ = np.reshape(info2_,(1483,1))
    info3_ = np.reshape(info3_,(1483,1))
    info4_ = np.reshape(info4_,(1483,1))
    info5_ = np.reshape(info5_,(1483,1))
    info6_ = np.reshape(info6_,(1483,1))

    #CONCATENATE VERTICES, EDGES AND INFO
    poses = np.concatenate((idp_,x_,y_,theta_),axis=1)
    edges = np.concatenate((ide1_,ide2_,dx_,dy_,dtheta_,info1_,info2_,info3_,info4_,info5_,info6_),axis=1)
    info = np.concatenate((info1_,info2_,info3_,info4_,info5_,info6_),axis=1)

    return poses, edges, info
