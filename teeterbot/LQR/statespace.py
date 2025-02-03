'''

Nandagopan.K

'''
import numpy as np
import control
from control.matlab import c2d



# L = 0.031
# M = 1.5
# m = 0.5
# R = 0.08
# D = 0.25
# Jw = 0.007932625
# Jd = 0.0006646250000000001
# Jp = 0.00016 

L = 0.16
M = 10.0
m = 1
R = 1.0
D = 0.3
Jw = 0.0200
Jd = 0.09634
Jp = 0.0297


a23 = (-M**2 * L**2 * 9.81) / (M * Jp + 2 * (Jp + M * L**2) * (m + Jw / R**2))
a43 = ((M**2 * 9.81 * L) + (2 * M * 9.81 * L) * (m + Jw / R**2)) / ((M * Jp) + 2 * (Jp + M * L**2) * (m + Jw / R**2))
b21 = ((Jp + M * L**2) / R + M * L) / ((M * Jp) + 2 * (Jp + M * L**2) * (m + Jw / R**2))
b41 = ((-(R + L) * M) / R - (2 * (m + Jw / R**2))) / ((M * Jp) + 2 * (Jp + M * L**2) * (m + Jw / R**2))
b61 = (D / (2 * R)) / (Jd + (D ** 2 / (2 * R)) * (m * R + (Jw / R)))
b62 = -b61
b22 = b21
b42 = b41
# print(b61)

A = np.array([[0, 1, 0, 0],
              [0, 0, a23, 0],
              [0, 0, 0, 1],
              [0, 0, a43, 0]])

# print(A)

# K = np.array([[0, 0, 50, 0]])

B = np.array([[0],
              [b21],
              [0],
              [b41]])
# print(B)
# A1 = np.add(A, np.dot(B, K))
# print(A1)
# abs_a1 = abs(A1)
# a1 = (A + (B @ K))
# print(abs(A1))
# eigenvalues, eigenvectors = np.linalg.eig(abs_a1)
# print(eigenvalues)
# print(abs(a1))


# print(A1)
C = np.array([[1, 0, 0, 0],
              [0, 0, 1, 0]])  # Transpose C

# print(C)
D = 0

sysc = control.StateSpace(A, B, C, D)
Ts = 0.01
ssd = control.matlab.c2d(sysc, Ts)
# print(ssd)
Ad, Bd, Cd = ssd.A, ssd.B, ssd.C
# print(Bd)
# abs_ad = abs(Ad)
# eigenvalues, eigenvectors = np.linalg.eig(abs_ad)
# print(eigenvalues)


A2 = np.array([[0, 1],
               [0, 0]])

B2 = np.array([[0],
               [b61]])
# print(B2)

C2 = np.array([[1, 0]])

D2 = 0

sysc2 = control.StateSpace(A2, B2, C2, D2)
Ts2 = 0.01
ssd2 = control.matlab.c2d(sysc2, Ts2)
Ad2, Bd2, Cd2 = ssd2.A, ssd2.B, ssd2.C
# print(ssd2)
# print(Bd2)


# A3 = np.array([[0, 1, 0, 0, 0, 0],
#               [0, 0, a23, 0, 0, 0],
#               [0, 0, 0, 1, 0, 0],
#               [0, 0, a43, 0, 0, 0],
#               [0, 0, 0, 0, 0, 1],
#               [0, 0, 0, 0, 0 ,0]])

# B3 = np.array([[0, 0],
#                [b21, b22],
#                [0, 0],
#                [b41, b42],
#                [0, 0],
#                [b61, b62]])

# # print(B3)

# sysc2 = control.StateSpace(A2, B2, C2, D)
# Ts2 = 0.01
# ssd2 = control.matlab.c2d(sysc2, Ts2)
# Ad2, Bd2, Cd2 = ssd2.A, ssd2.B, ssd2.C
