# Takes Allan Deviation CSV files and computes IMU noise parameters
#imports:
import os
import numpy as np
import pandas as pd


# Global Constants:
CWD = os.getcwd()

TAU_FIELD = 'tau'
ADEV_FIELD = 'adev'

X_FILENAME = 'imu_data/Allan Deviation of Gyro X axis_allan_dev.csv'
Y_FILENAME = 'imu_data/Allan Deviation of Gyro Y axis_allan_dev.csv'
Z_FILENAME = 'imu_data/Allan Deviation of Gyro Z axis_allan_dev.csv'

X_ALLAN_FILE = os.path.join(CWD,X_FILENAME)
Y_ALLAN_FILE = os.path.join(CWD,Y_FILENAME)
Z_ALLAN_FILE = os.path.join(CWD,Z_FILENAME)


#Returns a tuple of (N,B,K) values
def get_allan_values(allan_file):
# pick indices for each region manually (based on your plot)
    i_arw = 3   # short-term
    i_bias = np.argmin(allan_file[ADEV_FIELD])  # minimum point
    i_rrw = -3  # long-term tail

    tau_arw  = allan_file[TAU_FIELD].iloc[i_arw]
    sigma_arw = allan_file[ADEV_FIELD].iloc[i_arw]

    tau_rrw  = allan_file[TAU_FIELD].iloc[i_rrw]
    sigma_rrw = allan_file[ADEV_FIELD].iloc[i_rrw]

    sigma_bias = allan_file[ADEV_FIELD].iloc[i_bias]

    # calculate coefficients
    N = sigma_arw * np.sqrt(tau_arw)
    B = sigma_bias / 0.664
    K = sigma_rrw * np.sqrt(3.0 / tau_rrw)

    print(f"Angle Random Walk (N): {N:.4e}")
    print(f"Bias Instability (B): {B:.4e}")
    print(f"Rate Random Walk (K): {K:.4e}")
    return (N, B, K)

def main():
    allan_df = pd.read_csv(X_ALLAN_FILE)
    x_values = get_allan_values(allan_df)

    allan_df = pd.read_csv(Y_ALLAN_FILE)
    y_values = get_allan_values(allan_df)

    allan_df = pd.read_csv(Z_ALLAN_FILE)
    z_values = get_allan_values(allan_df)  
    

if __name__ == "__main__":
    main()
