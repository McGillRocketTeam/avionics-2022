"""
running the ejection code from 2020-21 for Flight Computer
in Python with altitude and acceleration data from
tmflights (https://git.gag.com/?p=fw/tmflights;a=summary).

also generate strings to test parser/gui for ground station.

string formats:
avionics - S,ACCx,ACCy,ACCz,GYROx,GYROy,GYROz,PRESSURE,LAT,LONG,HOUR,MIN,SEC,STATE,CONT,E
propulsion - P,PRESSURE,TEMPERATURE,VALVE_STATUS,E

author: jasper yun
"""

from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
import os


data_filename = "flight_data/2018-04-08-serial-1257-flight-0014.csv"
data_filename = "flight_data/2018-04-22-serial-4225-flight-0015.csv"
data_filename = "flight_data/2018-08-11-serial-4263-flight-0001.csv"
data_filename = "flight_data/2017-06-03-serial-3418-flight-0001.csv"
data_filename = "flight_data/2019-03-31-serial-3418-flight-0003.csv"
data_filename = "flight_data/2019-05-04-serial-1257-flight-0017.csv"
data_filename = "flight_data/2019-05-26-serial-1257-flight-0018.csv"
data_filename = "flight_data/2019-09-01-serial-3418-flight-0004.csv"
data_filename = "flight_data/2019-11-09-serial-3462-flight-0005.csv"
data_filename = "flight_data/2019-11-10-serial-4357-flight-0003.csv"
data_filename = "flight_data/2020-07-18-serial-5018-flight-0011.csv"
data_filename = "flight_data/2020-08-01-serial-5129-flight-0003.csv"
data_filename = "flight_data/2020-08-08-serial-4411-flight-0002.csv"
data_filename = "flight_data/2020-09-05-serial-4967-flight-0002.csv"
data_filename = "flight_data/2020-10-10-serial-2378-flight-0021.csv" # FAILS!! EARLY EJECTION

gui_strings_filename = "string_data_subsec/" + data_filename[12:-4] + "_combined.csv"
gui_strings_filename_av = "string_data_subsec/" + data_filename[12:-4] + "_av_only.csv"
gui_strings_filename_prop = "string_data_subsec/" + data_filename[12:-4] + "_prop_only.csv"


# for this basic ejection sim we don't need every telemetry topic;
# in the future, can add more if we do EKF or similar
# req_cols_mega = ['time', 'pressure', 'altitude', 'accel_x', 'accel_y', 'accel_z']
# req_cols_metrum = ['time', 'pressure', 'altitude', 'acceleration']


# lists (arrays) for ejection
NUM_MEAS_REG = 50
NUM_DESCENDING_SAMPLES = 30
MAIN_DEPLOYMENT = 600
LANDING_SAMPLES = 50
LANDING_THRESHOLD = 5
NUM_STRINGS_PRELAUNCH = 100
PRELAUNCH_SAMPLE_RATE = 0.5 # time between samples

alt_previous = NUM_MEAS_REG * [0]
time_previous = NUM_MEAS_REG * [0]
timalt_previous = NUM_MEAS_REG * [0]
timsqr_previous = NUM_MEAS_REG * [0]

currElem = 0
cutoff = 0.001
prevTick = 0
currTick = 0
alt_ground = 0
state = 0 # pre-launch, pre-apogee, post-apogee, landed

alt_filtered_hist = []
SmoothData = 0

# navigate to folder where script is stored
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)

# read in the data
df = pd.read_csv(data_filename)

# extract data
time = df["time"] + NUM_STRINGS_PRELAUNCH * PRELAUNCH_SAMPLE_RATE
pressure = df["pressure"]
altitude = df["altitude"] * 3.28084 # convert to ft
lat = df["latitude"]
long = df["longitude"]

# flag whether or not to generate strings for gs gui
is_generate_gui_strings = True

if 'accel_x' in df.columns:
    acc_x = df["accel_x"]
    acc_y = df["accel_y"]
    acc_z = df["accel_z"]
    gyro_x = df["gyro_roll"]
    gyro_y = df["gyro_pitch"]
    gyro_z = df["gyro_yaw"]
    
    def get_acc_norm(data_counter):
        return (np.sqrt(acc_x[data_counter]**2 + acc_y[data_counter]**2 + acc_z[data_counter]**2))

else:
    acceleration = df["acceleration"]

    # doesn't have acc/gyro info in 3 axes, can't generate gs gui strings
    is_generate_gui_strings = False

    def get_acc_norm(data_counter):
        return abs(acceleration[data_counter])

def main():
    global state

    # manually set ground altitude since data doesn't have enough 
    # datapoints to average it out
    alt_ground = altitude[0]
    alt_filtered_hist.append(0)
    SmoothData = alt_ground # initialize
    # print("alt ground: " + str(alt_ground))

    # variables to track events
    alt_launch = 0
    alt_main = 0
    alt_drogue = 0
    alt_landing = 0

    time_launch = 0
    time_main = 0
    time_drogue = 0
    time_landing = 0

    # initialize alt_filtered
    prevTick = time[0]
    currTick = time[1]
    currAlt = altitude[1] - alt_ground
    alt_filtered = runAltitudeMeasurements(currTick, currAlt)

    # counter to go through the data
    data_counter = 1

    # save pre-launch strings (generated using flight data's 0th elements)
    if is_generate_gui_strings:
        generate_prelaunch_strings(100)

        file_strings = open(gui_strings_filename, 'a')
        file_strings_av = open(gui_strings_filename_av, 'a')
        file_strings_prop = open(gui_strings_filename_prop, 'a')

    # waiting for launch
    while (alt_filtered < 20):
    # while (get_acc_norm(data_counter) < 14): # pull 14 - 10 = 4 G => launch
        data_counter += 1
        currTick = time[data_counter]
        currAlt = altitude[data_counter] - alt_ground
        alt_filtered = runAltitudeMeasurements(currTick, currAlt)
        if is_generate_gui_strings:
            generate_string(file_strings, file_strings_av, file_strings_prop, data_counter)

    alt_launch = alt_filtered
    time_launch = currTick
    state = 1

    # launched, waiting for apogee
    numNVals = 0
    fittedSlope = 0

    is_use_fc_ejection_2020_21 = False
    alt_filtered_max = 0

    while True:
        # get new data
        data_counter += 1
        currAlt = altitude[data_counter] - alt_ground
        currTick = time[data_counter]
        alt_filtered = runAltitudeMeasurements(currTick, currAlt)
        if is_generate_gui_strings:
            generate_string(file_strings, file_strings_av, file_strings_prop, data_counter)

        if is_use_fc_ejection_2020_21: # use ejection logic that Jennie wrote
            fittedSlope = LSLinRegression()

            if fittedSlope < 0:
                numNVals += 1
                if numNVals > 30:
                    break
            else:
                numNVals = 0
        
        else: # experimental apogee detection
            if alt_filtered > alt_filtered_max:
                alt_filtered_max = alt_filtered
                continue # no point doing rest of math if still going upward
            else:
                fittedSlope = LSLinRegression()
                # norm_acceleration = np.sqrt(acc_x[data_counter]**2 + acc_y[data_counter]**2 + acc_z[data_counter]**2)
                norm_acceleration = get_acc_norm(data_counter)
                if fittedSlope < 0 and norm_acceleration < 10:
                    numNVals += 1
                    if numNVals > 10:
                        break

                else:
                    numNVals = 0

    # reached apogee
    alt_drogue = altitude[data_counter] - alt_ground
    time_drogue = currTick
    state = 2
    print("detected apogee:     %.3f" % (alt_drogue))
    print("acceleration:        %.3f" % (get_acc_norm(data_counter)))
    print("actual apogee:       %.3f" % (max(altitude)))
    print("difference:          %.3f" % (max(altitude) - alt_drogue))
    print("alt ground:          %.3f" % (alt_ground))
    print("actual delta apogee: %.3f" % (max(altitude) - alt_drogue - alt_ground))
    print("")

    # wait for main deployment altitude
    # needs to be more sophisticated to determine nominal cases, off-nominal, etc.
    if (alt_drogue > 1500):
        MAIN_DEPLOYMENT = 1500
    else:
        MAIN_DEPLOYMENT = alt_drogue # deploy immediately

    while alt_filtered > MAIN_DEPLOYMENT:
        data_counter += 1
        currTick = time[data_counter]
        currAlt = altitude[data_counter] - alt_ground
        
        alt_filtered = runAltitudeMeasurements(currTick, currAlt)
        if is_generate_gui_strings:
            generate_string(file_strings, file_strings_av, file_strings_prop, data_counter)
    
    alt_main = alt_filtered
    time_main = currTick
    state = 3

    # landing detection
    count = 0
    while count < LANDING_SAMPLES:
        data_counter += 1
        if data_counter >= len(time):
            print("data_counter out of range, landing not detected")
            break

        currTick = time[data_counter]
        currAlt = altitude[data_counter] - alt_ground

        alt_filtered = runAltitudeMeasurements(currTick, currAlt)
        if is_generate_gui_strings:
            generate_string(file_strings, file_strings_av, file_strings_prop, data_counter)

        # difference = filterAltitude(currAlt, currTick) - alt_previous[NUM_MEAS_REG - 1]
        difference = alt_filtered - alt_previous[NUM_MEAS_REG - 1]
        if difference < 0:
            difference *= -1

        if  difference < LANDING_THRESHOLD:
            count += 1
        else:
            count = 0
    
    alt_landing = alt_filtered
    time_landing = currTick
    state = 4

    if is_generate_gui_strings:
        file_strings.close()
        file_strings_av.close()
        file_strings_prop.close()

    # print results
    print("Launch altitude:  %.3f\t time = %.3f" % (alt_launch, time_launch))
    print("Apogee altitude:  %.3f\t time = %.3f" % (alt_drogue, time_drogue))
    print("Main altitude:    %.3f\t time = %.3f" % (alt_main, time_main))
    print("Landing altitude: %.3f\t time = %.3f" % (alt_landing, time_landing))

    # plot results
    plt.rc('lines', linewidth=2)
    plt.rc('axes', grid=True)
    plt.rc('grid', linestyle='--')

    fig, (ax1) = plt.subplots(1, 1)
    if is_use_fc_ejection_2020_21:
        # fig.suptitle("Simulated Ejection Using tmflights Data and Detected Events - FC 2020-21 Logic")
        fig.suptitle(data_filename)
    else:
        # fig.suptitle("Simulated Ejection Using tmflights Data and Detected Events")
        fig.suptitle(data_filename)
    ax1.plot(time, altitude)
    ax1.plot(time[0:len(alt_filtered_hist)], alt_filtered_hist)
    ax1.axvline(time_launch, color='red')
    ax1.axvline(time_drogue, color='orange')
    ax1.axvline(time_main, color='green')
    ax1.axvline(time_landing, color='cyan')
    ax1.set_ylabel('Filtered Altitude (ft)')
    ax1.set_xlabel('Time (s)')
    ax1.legend(['Altitude', 'Altitude AGL'])
    # fig2, ax2 = plt.subplots(1, 1)
    # ax2.plot(time[0:len(alt_filtered_hist)], altitude[0:len(alt_filtered_hist)] - alt_filtered_hist)

    # plt.show()
    fig.savefig(data_filename[12:-4] + '_plot.png')

# -------------------------------------- #
# functions for ejection
# -------------------------------------- #

def storeAltitude(new_altitude, deltaT, cTime):
    global currElem
    alt_previous[currElem] = new_altitude
    time_previous[currElem] = cTime
    timalt_previous[currElem] = cTime * new_altitude
    timsqr_previous[currElem] = cTime * cTime

    if currElem == NUM_MEAS_REG - 1:
        currElem = 0
    else:
        currElem += 1


def filterAltitude(altitude, deltaT):
    # global cutoff
    # alpha = deltaT / (cutoff + deltaT)

    # return (altitude * alpha + (1 - alpha) * alt_previous[NUM_MEAS_REG - 1])
    
    # source:
    # https://kiritchatterjee.wordpress.com/2014/11/10/a-simple-digital-low-pass-filter-in-c/
    
    # global SmoothData
    # LPF_Beta = 1.2
    # SmoothData = SmoothData - (LPF_Beta * (SmoothData - altitude))
    # return SmoothData

    return altitude

    # cutoff freq: 20 Hz
    # sampling rate: 0.01 s --> 100 Hz
    # then RC = 1 / (2 * pi * cutoffFreqHz)
    # coeff1 = T / (T + RC)
    # coeff2 = RC / (T + RC)

    # cutoffFreq_Hz = 2000
    # T = 0.01
    # RC = 1 / (2*np.pi * 20)
    # c1 = T / (T + RC)
    # c2 = RC / (T + RC)

    # return c1 * altitude + c2 * alt_previous[currElem-1]

    


def runAltitudeMeasurements(currTick, currAlt):
    global prevTick
    T = (float) (currTick - prevTick)
    prevTick = currTick

    alt_filtered = filterAltitude(currAlt, T)
    storeAltitude(alt_filtered, T, currTick)
    alt_filtered_hist.append(alt_filtered)
    return alt_filtered


def LSLinRegression():
    xsum = 0
    ysum = 0
    xy = 0
    xsqr = 0

    for i in range(NUM_MEAS_REG):
        xsum += time_previous[i]
        ysum += alt_previous[i]
        xy += timalt_previous[i]
        xsqr += timsqr_previous[i]
    
    return ((float) (NUM_MEAS_REG*xy - (xsum*ysum)) / (NUM_MEAS_REG*xsqr - (xsum*xsum)))


# -------------------------------------- #
# functions for GUI string generation
# -------------------------------------- #

def generate_prelaunch_strings(num_strings):

    cont = 3
    state_local = 0

    pressure_tank = 750
    temperature_tank = 25
    valve_status = 0

    file_combined = open(gui_strings_filename, 'w')
    file_av = open(gui_strings_filename_av, 'w')
    file_prop = open(gui_strings_filename_prop, 'w')

    for i in range(num_strings):
        curr_time = time[0] - (num_strings - i - 1) * 0.5
        # hour = curr_time // 3600
        min = (int) (np.floor(curr_time)) // 60
        sec = np.floor(curr_time) - min*60
        secondfraction = (curr_time - np.floor(curr_time))
        
        # emulating RTC subsecond register behavior for FC
        # https://docs.google.com/document/d/1Gee41Y2KkPEGCr-uRRefLvJdC1EJoabWFdKovKoiT4Q/edit#heading=h.ker8g3yifzs9
        PS = 255
        subsec = PS - secondfraction*(PS + 1)

        # generate some little bit of noise
        x = np.random.rand()

        data_string_avionics = "S,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%04.2f,%03.7f,%03.7f,%02d,%02d,%02d,%d,%d,E\n"  % (
                            acc_x[0] + x/3.53, acc_y[0] - x/5.2, acc_z[0] + x/10.0,
                            gyro_x[0] + x/10.0, gyro_y[0] + x/10.0, gyro_z[0] - x/10.0,
                            pressure[0] + x/100.0, lat[0] + x/10000.0, long[0] - x/10000.0,
                            min, sec, subsec, state_local, cont
                        )

        data_string_propulsion = "P,%03.2f,%02.2f,%d,%02d,%02d,%02d,E\n" % (pressure_tank + x, temperature_tank + x, valve_status, min, sec, subsec)

        # save to file
        file_combined.write(data_string_avionics)
        file_combined.write(data_string_propulsion)

        file_av.write(data_string_avionics)
        file_prop.write(data_string_propulsion)

    file_combined.close()
    file_av.close()
    file_prop.close()



def generate_string(file_combined, file_av, file_prop, data_counter):
    global state
    
    # cont = 0
    # if state < 2:
    #     cont = 15 # all e-matches good
    # elif state == 2:
    #     cont = 3  # 2 e-matches good
    # else:
    #     cont = 0  # no e-matches good
    conts = [3, 3, 1, 0, 0]
    cont = conts[state]

    # generate hour, min, sec from time
    curr_time = time[data_counter]
    #hour = curr_time // 3600
    min = (int) (np.floor(curr_time)) // 60
    sec = np.floor(curr_time) - min*60
    secondfraction = (curr_time - np.floor(curr_time))
        
    # emulating RTC subsecond register behavior for FC
    # https://docs.google.com/document/d/1Gee41Y2KkPEGCr-uRRefLvJdC1EJoabWFdKovKoiT4Q/edit#heading=h.ker8g3yifzs9
    PS = 255
    subsec = PS - secondfraction*(PS + 1)

    # S,ACCx,ACCy,ACCz,GYROx,GYROy,GYROz,PRESSURE,LAT,LONG,HOUR,MIN,SEC,STATE,CONT,E
    # data_string = "S," + str(acc_x[data_counter]) + "," + str(acc_y[data_counter]) + "," + str(acc_z[data_counter]) + "," \
    #                 + str(gyro_x[data_counter]) + "," + str(gyro_y[data_counter]) + "," + str(gyro_z[data_counter]) + "," \
    #                     + str(pressure[data_counter]) + "," + str(lat[data_counter]) + "," + str(long[data_counter]) + "," \
    #                         + str(hour) + "," + str(min) + "," + str(sec) + "," + str(state), + "," + str(cont) + ",E"
    data_string_avionics = "S,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%03.2f,%04.2f,%03.7f,%03.7f,%02d,%02d,%02d,%d,%d,E\n"  % (
                            acc_x[data_counter], acc_y[data_counter], acc_z[data_counter],
                            gyro_x[data_counter], gyro_y[data_counter], gyro_z[data_counter],
                            pressure[data_counter], lat[data_counter], long[data_counter],
                            min, sec, subsec, state, cont
                        )
    
    # P,PRESSURE,TEMPERATURE,VALVE_STATUS,E
    pressure_tank = 0
    temperature_tank = 0
    valve_status = 0

    if state == 0: # 
        pressure_tank = 750 + np.random.rand() # psi
        temperature_tank = 25 + np.random.rand() * 2 # deg C
        valve_status = 1 # just to change it up
    
    elif state == 1:
        pressure_tank = 750 - (time[data_counter] - NUM_STRINGS_PRELAUNCH * PRELAUNCH_SAMPLE_RATE)**2 * 56 # motor burn time of 3.65 s, empty tank at 3.66 s with this function
        if pressure_tank < 0:
            pressure_tank = 0
        
        temperature_tank = 25 - time[data_counter]*2 # i really dont know
        if temperature_tank < -100:
            temperature_tank = -100
        
        valve_status = 0 # closed
    else:
        pressure_tank = 0
        temperature_tank = 2
        valve_status = 0

    # generate string
    data_string_propulsion = "P,%03.2f,%02.2f,%d,%02d,%02d,%02d,E\n" % (pressure_tank, temperature_tank, valve_status, min, sec, subsec)

    # return data_string_avionics, data_string_propulsion
    file_combined.write(data_string_avionics)
    file_av.write(data_string_avionics)

    if state < 20:
        file_combined.write(data_string_propulsion)
        file_prop.write(data_string_propulsion)


if __name__ == "__main__":
    main()
