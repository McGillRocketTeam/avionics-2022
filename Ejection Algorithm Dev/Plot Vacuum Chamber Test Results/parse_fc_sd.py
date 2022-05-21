"""
parse FC strings logged to SD card.
format of strings follows:
    S,ACCx,ACCy,ACCz,MAGx,MAGy,MAGz,PRESSURE,LAT,LONG,HOUR,MIN,SEC,SUBSEC,STATE,CONT,E\r\n

this script parses the string and then plots the pressure, latitude, and longitude.

author: jasper yun
date:   2021-12-16
"""

from matplotlib import pyplot as plt
import os
import parse_fc_sd_functions
import argparse

# application_path = os.path.dirname(sys.executable) # if exe

def main():
    filename = menu()
    do_parse_plot(filename)
    

def menu():
    print("Parse and Plot FC Telemetry.")
    while True:
        filename = input("Please enter filename of file to be parsed (.txt): ")
        if check_file_exists(filename):
            return filename
        else:
            print("Invalid file. Please try again.")


def do_parse_plot(filename):
    df_av, df_pr = parse_fc_sd_functions.parse(filename)

    parse_fc_sd_functions.get_av_stats(df_av)
    parse_fc_sd_functions.plot_av(df_av)
    plt.show()
    
    if not parse_fc_sd_functions.silent_mode:
        plt.show()


def check_file_exists(filename):
    if os.path.exists(filename):
        return True
    else:
        return False

if __name__ == '__main__':
    # navigate to folder where this script is stored
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)

    # setup CLI
    parser = argparse.ArgumentParser(description='parse and plot FC telemetry')
    parser.add_argument('-f', '--file', nargs='*', metavar='filename', type=str, help='input filename')
    parser.add_argument('-s', '--silent', action='store_true', help='hides plots and text output')
    parser.add_argument('--gps', action='store_true', help='save extracted gps coordinates')
    parser.add_argument('-b', '--batch', nargs=1, metavar='batch filename', type=str, help='text file containing filenames to be parsed')

    args = parser.parse_args()

    parse_fc_sd_functions.silent_mode = args.silent
    parse_fc_sd_functions.save_gps = args.gps

    if args.file is None and args.batch is None:
        main()
    else:
        if args.file is not None and args.batch is not None:
            filenames = args.file
            fbatch = open(args.batch[0], 'r')
            lines = fbatch.readlines()
            for line in lines:
                filenames.append(line.strip())
        
        elif args.file is not None:
            filenames = args.file
        
        else:
            fbatch = open(args.batch[0], 'r')
            lines = fbatch.readlines()
            filenames = []
            for line in lines:
                filenames.append(line.strip())

        for fname in filenames:
            if check_file_exists(fname):
                do_parse_plot(fname)
            else:
                print('Skipping bad file: %s\n' % fname)
    
