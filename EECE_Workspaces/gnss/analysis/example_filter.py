#Modified from DelftStack https://www.delftstack.com/howto/python/low-pass-filter-python/
#Original author: Vaibhhav Khetarpal

import numpy as np
from scipy.signal import butter, sosfilt, freqz
import matplotlib.pyplot as plt

def butter_filter(raw_data, cutoff_freq, sampl_freq, filt_type, filt_order):
    nyq_freq = sampl_freq / 2 #set the Nyquist frequency (important to avoid aliasing)
    sos = butter(N = filt_order, Wn = cutoff_freq / nyq_freq, btype=filt_type, analog=False, output='sos')
    filtered_data = sosfilt(sos, raw_data)
    return sos, filtered_data

# Setting filter requirements
order = 1 #you can increase this to make the filter "sharper"
sampl_freq = 200 #change to sampling frequency of your data collection
cutoff_freq = 3.667 #modify this value to get an appropriate cutoff frequency. This can't be any higher than sampling freq 

# Creating some made up, noisy data tp show filter properties
T = 5.0  # value taken in seconds
n = int(T * fs)  # indicates total samples
t = np.linspace(0, T, n, endpoint=False)

data = (
    np.sin(1.2 * 2 * np.pi * t)
    + 1.5 * np.cos(9 * 2 * np.pi * t)
    + 0.5 * np.sin(12.0 * 2 * np.pi * t)
)

# Filtering and plotting
sos, y = butter_filter(data, cutoff_freq, sampl_freq, "lowpass", 5)

plt.subplot(2, 1, 2)
plt.plot(t, data, "b-", label="data")
plt.plot(t, y, "g-", linewidth=2, label="filtered data")
plt.xlabel("Time [sec]")
plt.title("Low pass filter data")
plt.grid()
plt.legend()

plt.subplots_adjust(hspace=0.35)
plt.show()