![McGill Rocket Team Logo](https://raw.githubusercontent.com/McGillRocketTeam/ground-station-2019/master/media/MRT-logo.png)
# Radios Project for 2021-2022
Contains all projects that use both the SX1262 (SRADio) and XTEND.
Each radio module needs to interact with two different microcontrollers: Teensy 4.0 and ST Chip on the FC

## Library Conventions
Current libraries should always be **working** and updated in the main radios folder as both a zipped file and raw files.

## SRADio Teensy Library
Download the zipped library and add it to the arduino as a custom library. You can do so by adding the unzipped library folder into *~/Documents/Arduino/libraries/* or a similar folder depending on where the path has been set.

## SRADio FC Library
Donwload the zipped library and unzip it. Simply add the files *sx126x.h* and *sx126x_reg.h* to *My_Project/Core/Inc*. Also, add *sx126x.c* to *My_Project/Core/Src*. In *sx126x*, at around lines 241, there will be a list of static params that are the parameters for the setup to the radios. Make sure that the power amplifier's (PA) values match with the TX parameters values in the datasheet.