"""
contains functions for the main application 
provided in "parse_fc_sd.py"
"""

from enum import Enum
import os
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

LOCAL_PRESSURE = 1028 # in hPa
alt_ground = 0
silent_mode = False
save_gps = False
fname_input = ''
fname_output = ''
debug = False

plot_using_time = False

class DATA_INDEX(Enum):
    ACCx = 0
    ACCy = 1
    ACCz = 2
    
    GYROx = 3
    GYROy = 4
    GYROz = 5

    PRESSURE = 6

    LATITUDE = 7
    LONGITUDE = 8

    MINUTE = 9
    SECOND = 10
    SUBSEC = 11

    FLIGHT_STATE = 12
    CONTINUITY = 13

column_names_av = ['S',]
for idx in DATA_INDEX:
    column_names_av.append(idx.name)
column_names_av.append('E')

column_names_pr = ['S','PRESSURE_TANK','TEMP_TANK','VALVE_STATUS', 'MINUTE', 'SECOND', 'SUBSEC', 'E']

def get_av_stats(df):
    
    df_alt = get_altitudes_agl(df)
    df_time_sec = get_time_seconds(df)
    list_state_changes = get_state_change_indices(df)

    max_alt = np.max(df_alt)                    # true apogee
    for i in range(df_alt.shape[0]):
        if df_alt[i] == max_alt:
            max_alt_idx = i                     # true apogee index
    
    max_alt_time = df_time_sec[max_alt_idx]     # true apogee time

    if not os.path.exists(fname_output):
        os.mkdir(fname_output)
    os.chdir(fname_output)

    if len(list_state_changes) > 2:
        # now stats recorded by FC
        fc_apogee = [df_alt[list_state_changes[1]], df_time_sec[list_state_changes[1]]]
        fc_main = [df_alt[list_state_changes[2]], df_time_sec[list_state_changes[2]]]
        fc_landing = [df_alt[list_state_changes[3]], df_time_sec[list_state_changes[3]]]

        times = [max_alt_time, fc_apogee[1], fc_main[1], fc_landing[1]]
        alts = [max_alt, fc_apogee[0], fc_main[0], fc_landing[0]]

        df_stats = pd.DataFrame(list(zip(times, alts)), columns=['Time (s)', 'Altitude (ft AGL)'],
                                index=['True apogee', 'FC apogee', 'FC main', 'FC landing'])
        
        delta_t = fc_apogee[1] - max_alt_time
        if silent_mode == False:
            print('\nFlight statistics:')
            print(df_stats)
            print('')

            # calculate stuff and print
            print('Time difference from true apogee:   %.3f\t(s)' % (delta_t))
            print('Calculated altitude difference:     %.3f\t(ft)' % (0.5 * delta_t**2 * 32.17404855))
            print('Calculated altitude difference:     %.3f\t(m)' % (0.5 * delta_t**2 * 9.80665))
            print('Calculated drogue deployment speed: %.3f\t(m/s)' % (delta_t * 9.80665))
        
        f_out = open(fname_output + '.txt', 'a')
        f_out.write('\nFlight statistics:\n')
        dfAsString = df_stats.to_string(header=True, index=True)
        f_out.write(dfAsString)
        f_out.write('\n\n')
        f_out.write('Time difference from true apogee:   %.3f\t(s)\n' % (delta_t))
        f_out.write('Calculated altitude difference:     %.3f\t(ft)\n' % (0.5 * delta_t**2 * 32.17404855))
        f_out.write('Calculated altitude difference:     %.3f\t(m)\n' % (0.5 * delta_t**2 * 9.80665))
        f_out.write('Calculated drogue deployment speed: %.3f\t(m/s)\n' % (delta_t * 9.80665))
        f_out.close()

    if save_gps:
        save_gps_csv(fname_output + '_gps.csv', df)

    os.chdir('../')

def parse(filename):
    """
    parses the input FC telemetry text file. 
    returns df_av and df_pr containing the parsed data, converted to float64.
    """
    global fname_input
    fname_input = filename

    global fname_output
    fname_output = fname_input[:-4] + '_stats'

    separate_file_av_pr_strings(fname_input, 'av.txt', 'pr.txt')
    df_av = csv2df('av.txt', 'av')
    df_pr = csv2df('pr.txt', 'pr')

    return df_av, df_pr


def get_altitudes_agl(df):
    global alt_ground
    pressure = df.loc[:,'PRESSURE']
    altitude = pressure2altitude(np.array(pressure))
    # alt_ground = np.average(altitude[:10])

    return altitude - alt_ground


def get_state_change_indices(df):
    """
    assuming a valid avionics telemetry dataframe was passed.
    returns a list containing the indices where the flight state changes.
    """
    current_state = 0
    state_changes = []
    counter = 0

    for state_value in df.loc[:,DATA_INDEX.FLIGHT_STATE.name]:
        if state_value != current_state:
            current_state = state_value
            state_changes.append(counter)
        counter += 1
    
    return state_changes


def get_time_seconds(df):
    """
    assuming a valid avionics telemetry dataframe was passed.
    converts the minutes, seconds, subseconds, into just seconds.
    returns list containing result.
    """

    PRESCALER = 255
    minutes = df.loc[:, DATA_INDEX.MINUTE.name]
    seconds = df.loc[:, DATA_INDEX.SECOND.name]
    subsec = df.loc[:, DATA_INDEX.SUBSEC.name]
    start_sec = 60*minutes[0] + seconds[0] + (PRESCALER - subsec[0]) / (PRESCALER + 1)
    return (60 * minutes + seconds + (PRESCALER - subsec) / (PRESCALER + 1) - start_sec)


def plot_av(df):
    # *** DOES NOT ACLL PLT.SHOW() *** to avoid blocking/hanging in this function

    alt_agl = get_altitudes_agl(df)
    state_changes = get_state_change_indices(df)
    if True:
        x = get_time_seconds(df)
    else:
        x = np.arange(df.shape[0])

    # now plot
    plt.rc('lines', linewidth=2)
    plt.rc('axes', grid=True)
    plt.rc('grid', linestyle='--')

    # pressure and altitude
    fig_pressure, ax_pressure = plt.subplots(2, 1)
    ax_pressure[0].plot(x, np.array(df.loc[:, DATA_INDEX.PRESSURE.name]))
    ax_pressure[0].set_ylabel('Pressure (hPa)')
    ax_pressure[1].plot(x, np.array(alt_agl))
    ax_pressure[1].set_xlabel('Time (s)')
    ax_pressure[1].set_ylabel('Altitude AGL (ft)')

    if len(state_changes) > 0:
        for index in state_changes:
            ax_pressure[1].axvline(x[index], color='red')
    
    # accelerometer axes
    fig_acc, ax_acc = plt.subplots(3, 1)
    ax_acc[0].plot(x, np.array(df.loc[:, DATA_INDEX.ACCx.name])/1000)
    ax_acc[0].set_ylabel('AccX (g)')
    ax_acc[1].plot(x, np.array(df.loc[:, DATA_INDEX.ACCy.name])/1000)
    ax_acc[1].set_ylabel('AccY (g)')
    ax_acc[2].plot(x, np.array(df.loc[:, DATA_INDEX.ACCz.name])/1000)
    ax_acc[2].set_ylabel('AccZ (g)')
    ax_acc[2].set_xlabel('Time (s)')

    # verify time is straight
    if debug:
        fig_time, ax_time = plt.subplots(2, 1)
        increments = np.arange(0, len(x))
        ax_time[0].plot(increments, np.array(x))
        ax_time[0].set_ylabel('Converted time (s)')
        ax_time[0].set_xlabel('Sample number')

    # save plots
    os.chdir(fname_output)
    fig_pressure.savefig(fname_output + '_pressure_alt.png')
    fig_acc.savefig(fname_output + '_acc.png')
    os.chdir('../')

    # plt.close('all')

def save_gps_csv(filename, df):    
    lat = df.loc[:, DATA_INDEX.LATITUDE.name]
    long = df.loc[:, DATA_INDEX.LONGITUDE.name]
    clat, clong = lat[0], long[0]

    extracted_gps_df = df.iloc[[0], [DATA_INDEX.LATITUDE.value, DATA_INDEX.LONGITUDE.value]]

    # find all first instances of distinct, nonzero data
    for idx in range(df.shape[0]):
        if lat[idx] == 0 or long[idx] == 0:
            continue
        elif lat[idx] != clat or long[idx] != clong:
            clat = lat[idx]
            clong = long[idx]
            current_row = [clat, clong]
            extracted_gps_df.loc[extracted_gps_df.shape[0]] = current_row
    
    extracted_gps_df.iloc[1:,:].to_csv(filename, sep=',', header=True, index=False) # remove first 0, 0 entry


def csv2df(filename, subteam):
    """
    converts the CSV data in the input file to a pandas dataframe as float64 values.
    returns the dataframe.

    subteam must be either 'av' or 'pr', as a string.
    """
    if (os.path.exists(fname_output + '/' + filename)):

        if subteam == 'av':
            column_names = column_names_av
        elif subteam == 'pr':
            column_names = column_names_pr
        df = pd.read_csv(fname_output + '/' + filename, sep=',', names=column_names, index_col=False, 
                            usecols=column_names[1:-1], dtype=np.float64, skip_blank_lines=True)

    else:
        print("Bad filename, does not exist. Cannot convert to pandas dataframe.")
    
    return df


def separate_file_av_pr_strings(f_in, f_av_out, f_pr_out):
    """
    Parameters
    ----------
    f_in        input file

    f_av_out    output filename for avionics strings
    
    f_pr_out    output filename for propulsion strings
    """
    global alt_ground

    if not os.path.exists(fname_output):
        os.mkdir(fname_output)
    
    file_in = open(f_in, 'r')
    file_av_out = open(fname_output + '/' + f_av_out, 'w')
    file_pr_out = open(fname_output + '/' + f_pr_out, 'w')
    file_copy = open(fname_output + '/' + f_in, 'w')

    while True:
        next_line = file_in.readline()
        file_copy.write(next_line)
        if 'alt_ground='in next_line:
            alt_ground = float(next_line[next_line.index('=') + 1:-1])

        if len(next_line) > 4:
            if 'S' in next_line and 'GPS' not in next_line:
                if 'S,ACCx' in next_line: # can't convert this, skip it
                    continue
                file_av_out.write(next_line)
            elif 'P' in next_line:
                file_pr_out.write(next_line)
            
        if not next_line:
            break

    file_in.close()
    file_av_out.close()
    file_pr_out.close()
    file_copy.close()


def pressure2altitude(pressure):
    return 145442.1609 * (1.0 - pow(pressure/LOCAL_PRESSURE, 0.190266436))


if __name__ == "__main__":
    
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)


    # df = csv2df('AA000002.txt', 'av')
    # plot_av
    # df_agl = get_altitudes_agl(df)

    # save_gps('gps.csv', df)
    # plot_av(df)
    # plt.show()
