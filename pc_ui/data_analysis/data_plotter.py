import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import rfft, rfftfreq
import numpy as np

path = 'C:/Users/Brandon/Desktop/OpenQuad2/fw/oq2/logs/'
# file_relative_path = '20210211_143414_.log'
# file_relative_path = '20210211_144811_.log'
# file_relative_path = '20210211_145005_.log'  #
# file_relative_path = '20210211_145518_.log'  # dynamic learning, ODR_DIV9, 476HZ


# file_relative_path = '20210211_145836_.log'  # static learning, odr_div9, 476hz
# title_string = file_relative_path + ' static ODR_DIV9, 476HZ'

# file_relative_path = '20210211_150132_.log'  # static learning, odr_div9, 476hz
# title_string = file_relative_path + ' static ODR_DIV100, 476HZ'


# file_relative_path = '20210211_150949_.log'  # static learning, lp_off, 476hz
# title_string = file_relative_path + ' static ODR_DIV100, 476HZ'

# file_relative_path = '20210211_152235_.log'  # static learning, lp_off, 476hz
# title_string = file_relative_path + ' static ODR_DIV9, 476HZ, LSM9DS1_HP_ULTRA_LIGHT'

# file_relative_path = '20210211_152515_.log'  # static learning, lp_off, 476hz
# title_string = file_relative_path + ' static ODR_DIV9, 476HZ, LSM9DS1_HP_LIGHT'

# file_relative_path = '20210211_164306_.log'  # static learning, lp_off, 476hz
# title_string = file_relative_path + ' static ODR_DIV9, 476HZ, LSM9DS1_HP_ULTRA_LOW'

# file_relative_path = '20210211_164535_.log'  # static learning, lp_off, 476hz
# title_string = file_relative_path + ' static ODR_DIV9, 476HZ, LPFHPFLPF LSM9DS1_HP_ULTRA_LOW'

# file_relative_path = '20210211_164824_.log'  # static learning, 476hz
# title_string = file_relative_path + ' static ODR_DIV9, 476HZ, LSM9DS1_HP_ULTRA_LOW'

# file_relative_path = '20210212_110837_.log'  # static learning, , 476hz
# title_string = file_relative_path + ' static ODR_DIV50, 476HZ, LPFHPFLPF LSM9DS1_HP_MEDIUM'

# file_relative_path = '20210212_111410_.log'  # static learning, 476hz
# title_string = file_relative_path + ' static ODR_DIV50, 476HZ, LPFHPF LSM9DS1_HP_MEDIUM'

# file_relative_path = '20210212_111811_.log'  # static learning, 476hz
# title_string = file_relative_path + ' static ODR_DIV50, 476HZ, LPFHPF LSM9DS1_HP_MEDIUM, LSM9DS1_LP_LIGHT'

# file_relative_path = '20210212_112220_.log'  # static learning, 476hz
# title_string = file_relative_path + ' static ODR_DIV50, 476HZ, LPFHPF LSM9DS1_HP_MEDIUM, LSM9DS1_LP_MEDIUM'

# file_relative_path = '20210212_112526_.log'  # static learning, 476hz
# title_string = file_relative_path + ' static ODR_DIV50, 476HZ, LPFHPFLPF LSM9DS1_HP_MEDIUM, LSM9DS1_LP_MEDIUM'

# file_relative_path = '20210212_112746_.log'  # static learning, 476hz
# title_string = file_relative_path + ' static ODR_DIV50, 476HZ, LPFHPFLPF LSM9DS1_HP_LIGHT, LSM9DS1_LP_MEDIUM'

# file_relative_path = '20210212_113035_.log'  # static learning, 476hz
# title_string = file_relative_path + ' static ODR_DIV100, 476HZ, LPFHPFLPF LSM9DS1_HP_LIGHT, LSM9DS1_LP_MEDIUM'

# file_relative_path = '20210212_113450_.log'  # static learning, 476hz
# title_string = file_relative_path + ' static ODR_DIV400, 476HZ, LPFHPFLPF LSM9DS1_HP_LIGHT, LSM9DS1_LP_MEDIUM'

file_relative_path = '20210212_120335_.log'  # static learning, 476hz
title_string = file_relative_path + ' static ODR_DIV400, 476HZ, LPFHPFLPF LSM9DS1_HP_LIGHT, LSM9DS1_LP_MEDIUM'


if __name__ == '__main__':
    
    column_names = ['timestamps', 'label', 'ax', 'ay', 'az', 'gx', 'gy', 'gz', 'pitch']
    data = pd.read_csv(path + file_relative_path, header=None, names=column_names, skiprows=0)
    
    data.plot(x='timestamps', y=['pitch'])
    plt.title(title_string)

    data.plot(x='timestamps', y=['ax', 'ay', 'az'])
    plt.title(title_string)

    data.plot(x='timestamps', y=['gx', 'gy', 'gz'])
    plt.title(title_string)
    
    f_sample = 250

    # AS numpy arrays
    pitch_array = data['pitch'].to_numpy()
    # roll_array = data['roll'].to_numpy()
    
    print(len(pitch_array))
    # print(len(roll_array))
    
    min_pitch = np.min(pitch_array)
    max_pitch = np.max(pitch_array)

    # min_roll = np.min(roll_array)
    # max_roll = np.max(roll_array)
    
    print(title_string)
    print("Pitch variation: ", max_pitch - min_pitch)
    # print("Roll variation: ", max_roll-min_roll)

    # Calculate DFT
    pitch_fft = rfft(pitch_array)
    # roll_fft = rfft(roll_array)
    x_fft = rfftfreq(len(data['timestamps']), 1/f_sample)

    plt.figure()
    plt.plot(x_fft, np.log10(np.abs(pitch_fft)), color='red', label='pitch_fft')
    # plt.plot(x_fft, np.log10(np.abs(roll_fft)), color='green', label='roll_fft')
    plt.legend()
    plt.grid(which='both', axis='both')
    plt.title(title_string)
    
    plt.margins(0, 0.1)
    
    plt.show()
    
    ''' EOF '''
