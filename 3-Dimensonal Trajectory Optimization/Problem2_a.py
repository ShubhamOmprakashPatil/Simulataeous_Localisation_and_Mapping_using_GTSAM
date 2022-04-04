import numpy as np
import gtsam
import matplotlib.pyplot as plt

#WRITE FUNCTION TO READ AND LOAD VERTICES AND EDGES FROM THE G2O FILE
def read_g20_vertices(fileName):
    f = open(fileName,'r') #Open the file
    A = f.readlines()
    f.close()

    IND = [] #Generate list for each value (Index, x-y-z coordinates, qx,qy,qz,qw quartenion coordinates)
    X = []
    Y = []
    Z = []
    QX = []
    QY = []
    QZ = []
    QW = []

    for line in A:
        if "VERTEX_SE3:QUAT" in line:
            ver,ind,x,y,z,qx,qy,qz,qw,spaceV = line.split(' ') #Split numbers against spaces between them
            IND.append(float(ind))
            X.append(float(x))
            Y.append(float(y))
            Z.append(float(z))
            QX.append(float(qx))
            QY.append(float(qy))
            QZ.append(float(qz))
            QW.append(float(qw.rstrip('\n')))
    return (IND,X,Y,Z,QX,QY,QZ,QW)

def read_g20_edges(fileName):
    f = open(fileName,'r')
    A = f.readlines()
    f.close()

    IDE1 = []
    IDE2 = []
    X = []
    Y = []
    Z = []
    QX = []
    QY = []
    QZ = []
    QW = []
    INFO1 = []
    INFO2 = []
    INFO3 = []
    INFO4 = []
    INFO5 = []
    INFO6 = []
    INFO7 = []
    INFO8 = []
    INFO9 = []
    INFO10 = []
    INFO11 = []
    INFO12 = []
    INFO13 = []
    INFO14 = []
    INFO15 = []
    INFO16 = []
    INFO17 = []
    INFO18 = []
    INFO19 = []
    INFO20 = []
    INFO21 = []
    for line in A:
        if "EDGE_SE3:QUAT" in line:
            (edge,ide1,ide2,x,y,z,qx,qy,qz,qw,info1,info2,info3,info4,info5,info6, info7,info8,info9,info10,info11,info12,info13,info14,info15,info16,info17,info18,info19,info20,info21,spaceE) = line.split(' ')
            IDE1.append(float(ide1))
            IDE2.append(float(ide2))
            X.append(float(x))
            Y.append(float(y))
            Z.append(float(z.rstrip('\n')))
            QX.append(float(qx))
            QY.append(float(qy))
            QZ.append(float(qz))
            QW.append(float(qw))
            INFO1.append(float(info1))
            INFO2.append(float(info2))
            INFO3.append(float(info3))
            INFO4.append(float(info4))
            INFO5.append(float(info5))
            INFO6.append(float(info6))
            INFO7.append(float(info7))
            INFO8.append(float(info8))
            INFO9.append(float(info9))
            INFO10.append(float(info10))
            INFO11.append(float(info11))
            INFO12.append(float(info12))
            INFO13.append(float(info13))
            INFO14.append(float(info14))
            INFO15.append(float(info15))
            INFO16.append(float(info16))
            INFO17.append(float(info17))
            INFO18.append(float(info18))
            INFO19.append(float(info19))
            INFO20.append(float(info20))
            INFO21.append(float(info21))

    return (IDE1,IDE2,X,Y,Z,QX,QY,QZ,QW,INFO1,INFO2,INFO3,INFO4,INFO5,INFO6,INFO7,INFO8,INFO9,INFO10,INFO11,INFO12,INFO13,INFO14,INFO15,INFO16,INFO17,INFO18,INFO19,INFO20,INFO21)

def sensor_xyz():

    #LOAD THE VERTICES AND EDGES
    idp_,x_,y_,z_,qx_,qy_,qz_,qw_ = read_g20_vertices("parking-garage.g2o")

    #CONVERT VERTICES TO NUMPY ARRAY
    idp_ = np.array(idp_)
    x_ = np.array(x_)
    y_ = np.array(y_)
    z_ = np.array(z_)
    qx_ = np.array(qx_)
    qy_ = np.array(qy_)
    qz_ = np.array(qz_)
    qw_ = np.array(qw_)

    #RESHAPE
    idp_ = np.reshape(idp_,(1661,1))
    x_ = np.reshape(x_,(1661,1))
    y_ = np.reshape(y_,(1661,1))
    z_ = np.reshape(z_,(1661,1))
    qx_ = np.reshape(qx_,(1661,1))
    qy_ = np.reshape(qy_,(1661,1))
    qz_ = np.reshape(qz_,(1661,1))
    qw_ = np.reshape(qw_,(1661,1))

    initial_x_y_z = np.concatenate((x_,y_,z_),axis=1)

    return initial_x_y_z

def edges_vertices_and_info():
    #LOAD THE VERTICES AND EDGES
    idp_,x_,y_,z_,qx_,qy_,qz_,qw_ = read_g20_vertices("parking-garage.g2o")
    ide1_,ide2_,dx_,dy_,dz_,dqx_,dqy_,dqz_,dqw_,info1_,info2_,info3_,info4_,info5_,info6_,info7_,info8_,info9_,info10_,info11_,info12_,info13_,info14_,info15_,info16_,info17_,info18_,info19_,info20_,info21_ = read_g20_edges("parking-garage.g2o")

    #CONVERT VERTICES TO NUMPY ARRAY
    idp_ = np.array(idp_)
    x_ = np.array(x_)
    y_ = np.array(y_)
    z_ = np.array(z_)
    qx_ = np.array(qx_)
    qy_ = np.array(qy_)
    qz_ = np.array(qz_)
    qw_ = np.array(qw_)

    #RESHAPE
    idp_ = np.reshape(idp_,(1661,1))
    x_ = np.reshape(x_,(1661,1))
    y_ = np.reshape(y_,(1661,1))
    z_ = np.reshape(z_,(1661,1))
    qx_ = np.reshape(qx_,(1661,1))
    qy_ = np.reshape(qy_,(1661,1))
    qz_ = np.reshape(qz_,(1661,1))
    qw_ = np.reshape(qw_,(1661,1))

    #CONVERT EDGES TO NUMPY ARRAY
    ide1_ = np.array(ide1_)
    ide2_ = np.array(ide2_)
    dx_ = np.array(dx_)
    dy_ = np.array(dy_)
    dz_ = np.array(dz_)
    dqx_ = np.array(dqx_)
    dqy_ = np.array(dqy_)
    dqz_ = np.array(dqz_)
    dqw_ = np.array(dqw_)
    info1_ = np.array(info1_)
    info2_ = np.array(info2_)
    info3_ = np.array(info3_)
    info4_ = np.array(info4_)
    info5_ = np.array(info5_)
    info6_ = np.array(info6_)
    info7_ = np.array(info7_)
    info8_ = np.array(info8_)
    info9_ = np.array(info9_)
    info10_ = np.array(info10_)
    info11_ = np.array(info11_)
    info12_ = np.array(info12_)
    info13_ = np.array(info13_)
    info14_ = np.array(info14_)
    info15_ = np.array(info15_)
    info16_ = np.array(info16_)
    info17_ = np.array(info17_)
    info18_ = np.array(info18_)
    info19_ = np.array(info19_)
    info20_ = np.array(info20_)
    info21_ = np.array(info21_)

    #RESHAPE
    ide1_ = np.reshape(ide1_,(6275,1))
    ide2_ = np.reshape(ide2_,(6275,1))

    dx_ = np.reshape(dx_,(6275,1))
    dy_ = np.reshape(dy_,(6275,1))
    dz_ = np.reshape(dz_,(6275,1))
    dqx_ = np.reshape(dqx_,(6275,1))
    dqy_ = np.reshape(dqy_,(6275,1))
    dqz_ = np.reshape(dqz_,(6275,1))
    dqw_ = np.reshape(dqw_,(6275,1))

    info1_ = np.reshape(info1_,(6275,1))
    info2_ = np.reshape(info2_,(6275,1))
    info3_ = np.reshape(info3_,(6275,1))
    info4_ = np.reshape(info4_,(6275,1))
    info5_ = np.reshape(info5_,(6275,1))
    info6_ = np.reshape(info6_,(6275,1))
    info7_ = np.reshape(info7_,(6275,1))
    info8_ = np.reshape(info8_,(6275,1))
    info9_ = np.reshape(info9_,(6275,1))
    info10_ = np.reshape(info10_,(6275,1))
    info11_ = np.reshape(info11_,(6275,1))
    info12_ = np.reshape(info12_,(6275,1))
    info13_ = np.reshape(info13_,(6275,1))
    info14_ = np.reshape(info14_,(6275,1))
    info15_ = np.reshape(info15_,(6275,1))
    info16_ = np.reshape(info16_,(6275,1))
    info17_ = np.reshape(info17_,(6275,1))
    info18_ = np.reshape(info18_,(6275,1))
    info19_ = np.reshape(info19_,(6275,1))
    info20_ = np.reshape(info20_,(6275,1))
    info21_ = np.reshape(info21_,(6275,1))

    #CONCATENATE VERTICES, EDGES AND INFO
    poses = np.concatenate((idp_,x_,y_,z_,qx_,qy_,qz_,qw_),axis=1)
    edges = np.concatenate((ide1_,ide2_,dx_,dy_,dz_,dqx_,dqy_,dqz_,dqw_,info1_,info2_,info3_,info4_,info5_,info6_,info7_,info8_,info9_,info10_,info11_,info12_,info13_,info14_,info15_,info16_,info17_,info18_,info19_,info20_,info21_),axis=1)
    info = np.concatenate((info1_,info2_,info3_,info4_,info5_,info6_,info7_,info8_,info9_,info10_,info11_,info12_,info13_,info14_,info15_,info16_,info17_,info18_,info19_,info20_,info21_),axis=1)

    return poses, edges, info
