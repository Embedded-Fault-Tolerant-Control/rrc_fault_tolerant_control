# Fault Tolerant Control of Unmanned Aerial Vehicles (UAVs)
Table of Contents:
- [Fault Tolerant Control of Unmanned Aerial Vehicles (UAVs)](#fault-tolerant-control-of-unmanned-aerial-vehicles-uavs)
  - [What this repository contains?](#what-this-repository-contains)
  - [Getting started with code](#getting-started-with-code)
    - [Pre-requisites](#pre-requisites)
    - [Setting up](#setting-up)
    - [Run a sample code](#run-a-sample-code)
      - [Drone simulation without any faults (trajectory-healthy.py)](#drone-simulation-without-any-faults-trajectory-healthypy)
      - [Drone simulation with faulty motor(s) (trajectory-faulty.py)](#drone-simulation-with-faulty-motors-trajectory-faultypy)
      - [Read a Dataset in MATLAB (read_dataset.m)](#read-a-dataset-in-matlab-read_datasetm)
      - [Read GPS data during flight (print_save_gps.py)](#read-gps-data-during-flight-print_save_gpspy)

## What this repository contains?
- Python code
  - [trajectory-healthy.py](src/trajectory-healthy.py) - For a normal trajectory
  - [trajectory-faulty.py](src/trajectory-faulty.py) - For a motor fault induced during normal trajectory
  - [print_save_gps.py](src/print_save_gps.py) - For reading GPS sensor data during a flight
- MATLAB code
  - [read_dataset.m](src_matlab/read_dataset.m) - For reading a dataset file and making sense of it
- Generated datasets
  - [prototype-set-1](dist/prototype-set-1) - (Prototyped) Propellor fault data is logged for 2 different use-cases (hover and go in a straight line) with multiple props (5mm, 10mm, 20mm, 25mm, 40mm). This dataset also contains the compressed flight testing videos of each scenario.
  - [simulation-set-1](dist/simulation-set-1) - (Simulated) Motor fault data is logged for 4 different drone frames (Quad+, QuadX, Hexa+, HexaX)

## Getting started with code

### Pre-requisites

- Python 3.8
- SITL (preferred) OR Actual drone

  (This needs to have the SERVOn_FUNCTION parameters; which are missing in the PX4 firmware, but available on the ArduPilot firmware)
  - Pixhawk hardware running [ArduPilot](https://github.com/ArduPilot/ardupilot) (and not [PX4](https://github.com/PX4/PX4-Autopilot)) firmware. Should also contain SERVOn_FUNCTION params.
  - OR the official [ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html). You can read more about [setting up SITL here](https://ardupilot.org/dev/docs/SITL-setup-landingpage.html#sitl-setup-landingpage).
- Ground Station software
  - APM Planner - [Install guide](https://ardupilot.org/planner2/docs/installing-apm-planner-2.html)
  - OR QGroundControl - [Install guide](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html)
- MAVExplorer.py - [Install guide](https://ardupilot.org/dev/docs/using-mavexplorer-for-log-analysis.html) (MAVExplorer can be used for generating graphs from log files)

### Setting up
- Clone this repository
- Open the repository project in your IDE, or cd your command prompt to the directory
- Run `pip install -r requirements.txt` (Python 2), <br>
or `pip3 install -r requirements.txt` (Python 3) <br>
in the command prompt
- (Optional for Windows) You need the `<Your Drive>/ArduPilot/Tools/autotest/` folder in your $PATH Environment Variable. This folder contains the `sim_vehicle.py` file. Alternatively, to run the simulation, you can cd to this directory and run the `sim_vehicle.py` file whenever needed. 

### Run a sample code

#### Drone simulation without any faults ([trajectory-healthy.py](src/trajectory-healthy.py))
- Run the SITL simulator <br>
`sim_vehicle.py -v ArduCopter`
- Run the ground station software (APM/QGC) (if it already isn't running)
- Wait for a few seconds for SITL to connect to the ground station <br>
In case the ground station does not directly connect to the SITL, you can manually try connecting to `127.0.0.1` on ports 14450, 14451, 5760 over UDP.
- Open `trajectory-healthy.py` in case you want to edit; or directly run it <br>
`python trajectory-healthy.py`
  - (Optional) In case your code does not connect to the SITL, you might want to change the connection_string value. This can be done by [passing an argument](src/trajectory-healthy.py#L56), or by [modifying the value in code](src/trajectory-healthy.py#L63).
- Code will automatically exit on completion
- You can download the log file from SITL [as mentioned here](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#accessing-log-files)
- (Optional) You can generate graphs from this log file by running <br>
`MAVExplorer.py <path-to-log-file>.log`

#### Drone simulation with faulty motor(s) ([trajectory-faulty.py](src/trajectory-faulty.py))
- Run the SITL simulator <br>
`sim_vehicle.py -v ArduCopter`
- Run the ground station software (APM/QGC) (if it already isn't running)
- Wait for a few seconds for SITL to connect to the ground station <br>
In case the ground station does not directly connect to the SITL, you can manually try connecting to `127.0.0.1` on ports 14450, 14451, 5760 over UDP.
- Open `trajectory-faulty.py` in case you want modify which motors should stop working:
  - [Line 99](src/trajectory-faulty.py#L99) - In case of QuadCopter simualtion, keep the next two lines commented. In case of HexaCopter simualtion, uncomment the next two lines.
  - [Line 146](src/trajectory-faulty.py#L146) - Use `set_motor_mode(MOTOR_NUMBER, FUNCTION)` function to set a value of 1 for all the motors you wish to override (in flight, by overtaking control from the AutoPilot code)
  - [Line 149](src/trajectory-faulty.py#L149) - Use `set_servo(MOTOR_NUMBER, PWM_VALUE)` function to send a PWM signal to one of the overridden (in previous step) motors. This will not work if the motor is still in control with the AutoPilot. (i.e. if previous step is skipped)
- Run `python trajectory-faulty.py`
  - (Optional) In case your code does not connect to the SITL, you might want to change the connection_string value. This can be done by [passing an argument](src/trajectory-faulty.py#L56), or by [modifying the value in code](src/trajectory-faulty.py#L63).
- Code will automatically exit on completion
- You can download the log file from SITL [as mentioned here](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#accessing-log-files)
- (Optional) You can generate graphs from this log file by running <br>
`MAVExplorer.py <path-to-log-file>.log`

#### Read a Dataset in MATLAB ([read_dataset.m](src_matlab/read_dataset.m))

Logic for the MATLAB file is as follows:
1. Opens relevant dataset such as `m1.mat`
2. Sets which variables (log outputs) are to be read from the file. e.g. `{AHR2,PARM}`. Note that log outputs are priority based and each module (gyro, baro, ekf, ...) has its own update frequency for generating a log entry.
3. Searches the last occourence of our override code, in the `PARM` array inside the dataset.
4. Stores the Timestamp (TimeUS) in microseconds, for receipt of the last override packet found in the dataset.
5. Now, the code selects the dataset array to be studied. Barometer altitude, in this case.
6. Finds the Timestamp in Barometer array, which is nearest to the Timestamp of the override.
7. Stores Timestamp key and value for Barometer array
8. Plots graph of the Barometer Altitude values, starting from the Timestamp chosen in last step.

Steps to run:
- Open the `read_dataset.m` file in MATLAB
- Make sure the `read_dataset.m` file and the `dist` folder are added to the MATLAB current path
  - You can do this by simply right-clicking on the file/folder in the file manager (on left side panel) and selecting Add to Path > Selected Folders and Subfolders
- You can modify the required parameters in file
  - filename - Path to the dataset file, [dist/set-1/quad-x/m1.mat](dist/set-1/quad-x/m1.mat) in the example
  - varsToRead - Array of modules to read from dataset. You can directly load the .mat file in your workspace in order to see all possible combinations of dataset arrays you can choose here/ import here.
  - searchParam - Parameter to search. In our case, the fault is introduced using SERVO1_FUNCTION. This will be SERVO2_FUNCTION, SERVO3_... and so on for other motors. searchParam variable is used and the last use occurence is found, for the param.
  - selectedArray - Your choice of array (from varsToRead), on which you want to work/ play around.
- Run the `read_dataset.m` file

#### Read GPS data during flight ([print_save_gps.py](src/print_save_gps.py))

- Start by going to ardupilot/ArduCopter and running sim_vehicle.py --map
- Then in other termininal going to myexamples and running print_gps.py
- Python print_gps.py –connect 127.0.0.1:14551
- Wait for sitl drone to stop/ get in RTL mode
- Data will be printed in terminal as well as in the gps_data.csv
