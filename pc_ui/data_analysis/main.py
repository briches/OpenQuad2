import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import rfft, rfftfreq
import numpy as np

if __name__ == '__main__':
    # Import data
    output_df = pd.read_csv("PID tuning - Test 6 Fixed motor fw.csv")
    # print(output_df)
    
    test_filt = pd.read_csv("test_filt_1.csv")
    
    test_pitch = pd.read_csv("20210119_203330_.csv")
    test_pitch.plot(x='time', y=['pitch', 'filtered'])
    
    N = len(output_df['Time']) - 1
    print(N)
    
    step_function = np.zeros(1000)
    step_function[101:300] = 100
    
    # Design cheby2 filter
    # order = 4  # Order of the filter
    attenuation = 40  # dB attenuation in the stop band
    f_sample = 250  # Sampling Frequency ( 2 half cycles per sample )
    f_pass = 15  # Pass band edge

    h = signal.firwin(11, f_pass, pass_zero='lowpass', fs=f_sample)
    
    h_min_phase = signal.minimum_phase(h)
    
    print(h_min_phase)
    
    # # Apply the filter
    # output_df['FilteredAngle'] = signal.lfilter(h, [1.0], output_df['Angle'])
    output_df['MinPhaseFiltered'] = signal.lfilter(h_min_phase, [1.0], output_df['Angle'])
    
    # output_df.plot(x='Time', figsize=(6, 5), subplots=True)
    # output_df.plot(x='Time', y=['Angle', 'FilteredAngle', 'MinPhaseFiltered'], legend=True)
    # plt.margins(0, 0.1)
    
    # step_filtered = signal.lfilter(h, [1.0], step_function)
    step_filtered_min_phase = signal.lfilter(h_min_phase, [1.0], step_function)

    plt.figure()
    plt.plot(step_function, color='black', label='step function', linewidth=0.0, markersize=2, marker='o')
    plt.plot(test_filt['output'], color='red', label='CM7 test', linewidth=0.5, markersize=2, marker='o')
    plt.plot(step_filtered_min_phase, color='green', label='min phase ref', linewidth=0.5, markersize=2, marker='o')
    plt.legend()
    plt.grid(which='both', axis='both')
    plt.margins(0, 0.1)
    
    # Calculate and plot FFTs
    yf_unfiltered = rfft(test_pitch['pitch'].to_numpy())
    yf_filtered = rfft(test_pitch['filtered'].to_numpy())
    xf = rfftfreq(len(test_pitch['time']), 1/f_sample)
    
    plt.figure()
    plt.plot(xf, np.log10(np.abs(yf_unfiltered)), color='red', label='raw')
    plt.plot(xf, np.log10(np.abs(yf_filtered)), color='green', label='filtered')
    plt.legend()
    plt.grid(which='both', axis='both')
    plt.margins(0, 0.1)

    plt.show()
