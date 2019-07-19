import numpy as np
from numpy.linalg import inv


# state vector and co-variance matrix: y_hat = [x, y, z, qw, qx, qy, qz]^T, 7x1
y_hat= np.matrix([[0, 0, 0, qw_0, qx_0, qy_0, qz_0]], np.float).transpose()  # initial state vector
Q_yhat_yhat = np.matrix([[0.1^2, 0, 0, 0, 0, 0, 0],
                          [0, 0.1^2, 0, 0, 0, 0, 0],
                          [0, 0, 0.1^2, 0, 0, 0, 0],
                          [0, 0, 0, 0.1^2, 0, 0, 0],
                          [0, 0, 0, 0, 0.1^2, 0, 0],
                          [0, 0, 0, 0, 0, 0.1^2, 0],
                          [0, 0, 0, 0, 0, 0, 0.1^2]], np.float)  # 7x7

# transiation matrix
T = np.identity(7, np.float)  # 7x7


# prediction
for i in range(0, 100):
    # predicted state at epoch k+1
    y_bar = T * y_hat  # 7x1
    Q_ybar_ybar = T * Q_yhat_yhat * T.transpose()  # 7x7

    # observation at epoch k+1
    I = np.matrix([])  # 9x1
    Q_II = np.matrix([])  # 9x9

    # observation equation: I_bar = A*y_bar_k+1
    A = np.matrix([])  # design matrix, 9x7
    I_bar = A * y_bar  # predicted observation, 9x1
    Q_Ibar_Ibar = A * Q_ybar_ybar * A.transpose()  # 9x9

    # innovation vector
    d = I - I_bar  # 9x1
    Q_dd = Q_II + Q_Ibar_Ibar  # 9x9

    # gain matrix
    K = Q_ybar_ybar * A.transpose() * inv(Q_dd)  # 9x9

    # updated state
    y_hat = y_bar + K * d  # 9x1
    Q_yhat_yhat = Q_ybar_ybar - K * Q_dd * K.transpose()



