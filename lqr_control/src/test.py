import matplotlib.pyplot as plt
import matplotlib.patches as patches
from PIL import Image
import numpy as np
from math import cos, sin
from numpy.linalg import inv

T = 1.0
dt = 0.2
traj_length = int(T/dt)

Q = np.eye(3)
#Q[2,2] = 0
Q_f = np.eye(3)
#Q_f[2,2] = 0
R = np.eye(2)*0.01
prev_u_ = [np.array([0, 0])] * traj_length
prev_x_ = [np.array([0, 0])] * traj_length
S_ = [None] * traj_length

trajectory_ = []

for i in range(traj_length):
    trajectory_.append(np.array([i*0.1, i*0.2, -np.pi/2]))
    prev_x_[i] = trajectory_[i]

def computeAMatrix(x, u):
    A = np.eye(3)
    A[1,2] = -u[0]*sin(x[2])*dt
    A[0,2] = u[0]*cos(x[2])*dt
    return A

def computeBMatrix(x):
    B = np.zeros((3,2))
    #B[0,0] = -dt
    #B[2,1] = -dt
    B[0,0] = cos(x[2]) * dt
    B[1,0] = sin(x[2]) * dt
    B[2,1] = dt
    return B

def computeNextState(x, u):
    return computeAMatrix(x, u) @ x + computeBMatrix(x) @ u

def computeRicatti(init_x):
    S_[traj_length-1] = Q_f
    for t in range(traj_length-2, -1, -1):
        #print(f"\n\n\nat index {t}")
        last_S = S_[t+1]
        #print(f"last_S: {last_S}")
        A = computeAMatrix(prev_x_[t], prev_u_[t])
        B = computeBMatrix(prev_x_[t])
        #print(f"A: {A}")
        #print(f"B: {B}")

        #print(f"R + B.T @ last_S @ B:{R+B.T@last_S@B}")
        #print(f"B.T @ last_S @ A:{B.T @ last_S @ A}")

        K = np.linalg.inv(R + B.T @ last_S @ B) @ B.T @ last_S @ A
        #print(f"K: {K}")
        S_[t] = A.T @ last_S @ A - (A.T @ last_S @ B) @ K + Q
        #print(f"S_{t}: {S_[t]}")

    #print(f"S: {S_}")

    cur_x = init_x
    for t in range(traj_length):
        #print(f"\n\nat time: {t}")
        A = computeAMatrix(prev_x_[t], prev_u_[t])
        B = computeBMatrix(prev_x_[t])
        cur_S = S_[t]

        #print(f"inv(R + B.T @ cur_S @ B): {inv(R + B.T @ cur_S @ B)}")
        #print(f"cur_S: {cur_S}")

        K = inv(R + B.T @ cur_S @ B) @ B.T @ cur_S @ A
        #print(f"at index {t} K = {K}")
        #print(f"at index {t} got error {cur_x - trajectory_[t]}")
        #error_mat = np.array([[cos(cur_x[2]), sin(cur_x[2]), 0],
        #                      [-sin(cur_x[2]), cos(cur_x[2]), 0],
        #                      [0, 0, 1]
        #                      ])
        error_mat = np.eye(3)
        u_star = -K @ (error_mat @ (cur_x - trajectory_[t]))


        prev_u_[t] = u_star
        prev_x_[t] = cur_x
        #print(f"u_star at {t}: {u_star}")

        print(f"state at {t}: {cur_x} control {u_star} error {cur_x - trajectory_[t]}")
        cur_x = computeNextState(cur_x, u_star)

for i in range(2):
    print(f"prev_x_: {prev_x_}")
    computeRicatti(np.array([0,0,0]))
#print(f"u_traj: {prev_u_}")
