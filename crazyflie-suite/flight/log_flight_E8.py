"""
Execute a Crazyflie flight and log it.
"""

import argparse
import time
from datetime import datetime
from pathlib import Path

import yaml
import numpy as np
import pandas as pd
import os
import sys
import enum
import scipy.signal
from threading import Thread

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie import Console
from cfclient.utils.input import JoystickReader
from cfclient.utils.config import Config

import flight.utils as util
from flight.FileLogger import FileLogger
from flight.NatNetClient import NatNetClient

# TODO: merge these? (prepared trajectories and trajectories)
from flight.trajectories import takeoff, landing
from flight.prepared_trajectories import *

class Mode(enum.Enum):
    MANUAL = 1
    AUTO = 2
    MODE_SWITCH = 3
    DONT_FLY = 4

class LogFlight():
    def __init__(self, args):
        self.args = args
        self.optitrack_enabled = False
        self.console_dump_enabled = False

        cflib.crtp.init_drivers(enable_debug_driver=False)
        self._cf = Crazyflie(rw_cache="./cache")
        self._jr = JoystickReader(do_device_discovery=False)

        # Set flight mode
        if self.args["trajectory"] is None or \
                self.args["trajectory"][0] == "none":
            self.mode = Mode.DONT_FLY
            print("Mode set to [DONT_FLY]")
        elif self.args["trajectory"][0] == "manual": 
            self.mode = Mode.MANUAL
            print("Mode set to [MANUAL]")
        elif self.args["safetypilot"]:
            self.mode = Mode.MODE_SWITCH
            print("Mode set to [MODE_SWITCH]")
        else:
            self.mode = Mode.AUTO
            print("Mode set to [AUTO]")
        
        # Setup for specified mode
        if self.mode == Mode.AUTO:
            self.is_in_manual_control = False
            # Make sure drone is setup to perform autonomous flight
            if args["uwb"] == "none":
                assert args["optitrack"] == "state", "OptiTrack state needed in absence of UWB"
                assert args["estimator"] == "kalman", "OptiTrack state needs Kalman estimator"
        elif self.mode == Mode.DONT_FLY:
            self.is_in_manual_control = False
        else:
            # Check if controller is connected
            assert self.controller_connected(), "No controller detected."
            self.setup_controller(map="flappy")
            self.is_in_manual_control = True

        # Setup the logging framework
        self.setup_logger()

        # Setup optitrack if required
        if not args["optitrack"] == "none":
            self.setup_optitrack()

    def get_filename(self):
        # create default fileroot if not provided
        if self.args["fileroot"] is None:
            date = datetime.today().strftime(r"%Y_%m_%d")
            self.args["fileroot"] = "data/" + date
        
        fileroot = self.args["fileroot"] 
        
        # create default filename if not provided
        if self.args["filename"] is None:
            estimator = self.args["estimator"]
            uwb = self.args["uwb"]
            optitrack = self.args["optitrack"]
            trajectory = self.args["trajectory"]
            date = datetime.today().strftime(r"%Y-%m-%d+%H:%M:%S")

            traj = '_'.join(trajectory)
            if optitrack == "logging":
                options = "{}+{}+optitracklog+{}".format(estimator, uwb, traj)
            elif optitrack == "state":
                options = "{}+{}+optitrackstate+{}".format(estimator, uwb, traj)
            else:
                options = "{}+{}+{}".format(estimator, uwb, traj)
            
            name = "{}+{}.csv".format(date, options)
            fname = os.path.normpath(os.path.join(os.getcwd(), fileroot, name))

        else:
            # make sure provided filename is unique
            if self.args["filename"].endswith(".csv"):
                name = self.args["filename"][:-4]
            else:
                name = self.args["filename"]
            new_name = name + ".csv"
            fname = os.path.normpath(os.path.join(
                os.getcwd(), fileroot, new_name))

            i = 0
            while os.path.isfile(fname):
                i = i + 1
                new_name = name + "_" + str(i) + ".csv"
                fname = os.path.normpath(os.path.join(
                    os.getcwd(), fileroot, new_name))

        return fname


    def setup_logger(self):
        # Create filename from options and date
        self.log_file = self.get_filename()
        # Create directory if not there
        Path(self.args["fileroot"]).mkdir(exist_ok=True)

        print("Log location: {}".format(self.log_file))

        # Logger setup
        logconfig = self.args["logconfig"]
        self.flogger = FileLogger(self._cf, logconfig, self.log_file)
        self.flogger.enableAllConfigs()

    def setup_optitrack(self):
        self.ot_id = self.args["optitrack_id"]
        self.ot_position = np.zeros(3)
        self.ot_attitude = np.zeros(3)
        self.ot_quaternion = np.zeros(4)
        self.filtered_pos = np.zeros(3)
        self.ot_filter_sos = scipy.signal.butter(N=4, Wn=0.1, btype='low',
                                            analog=False, output='sos')
        self.pos_filter_zi = [scipy.signal.sosfilt_zi(self.ot_filter_sos),
                              scipy.signal.sosfilt_zi(self.ot_filter_sos),
                              scipy.signal.sosfilt_zi(self.ot_filter_sos)]
        # Streaming client in separate thread
        streaming_client = NatNetClient()
        streaming_client.newFrameListener = self.ot_receive_new_frame
        streaming_client.rigidBodyListener = self.ot_receive_rigidbody_frame
        streaming_client.run()
        self.optitrack_enabled = True
        print("OptiTrack streaming client started")

        # TODO: do we need to return StreamingClient?


    def paramcallback(self,value, name):
        print("changed " + value + " to value of " + name)

    def ranging_algo(self, algo):
        self._cf.param.add_update_callback(group="ranging_mode", name="range_modus", cb=self.paramcallback)
        if algo == "disabled":
            self._cf.param.set_value("ranging_mode.range_modus", "0")
            #time.sleep(1)
        if algo == "vanilla":
            self._cf.param.set_value("ranging_mode.range_modus", "1")
            #time.sleep(1)
        if algo == "cu":
            self._cf.param.set_value("ranging_mode.range_modus", "2")
            #time.sleep(1)
        if algo == "ci-cu":
            self._cf.param.set_value("ranging_mode.range_modus", "3")
            #time.sleep(1)

    def tdoa_status(self, algo, value):
   
        self._cf.param.add_update_callback(group="tdoa_mode", name="tdoa_modus", cb=self.paramcallback)
        if algo == "full":
            self._cf.param.set_value("tdoa_mode.tdoa_modus", "1")
            #time.sleep(1)
        if algo == "lim":
            if value == 1:
                self._cf.param.set_value("tdoa_mode.tdoa_modus", "1")
                #time.sleep(1)
            if value == 0:
                self._cf.param.set_value("tdoa_mode.tdoa_modus", "0")
                #time.sleep(1)

    def reset_estimator(self):
        self._cf.param.add_update_callback(group="kalman", name="resetEstimation", cb=self.paramcallback)
        # Kalman
        if self.args["estimator"] == "kalman":
            self._cf.param.set_value("kalman.resetEstimation", "1")
            time.sleep(1)
            self._cf.param.set_value("kalman.resetEstimation", "0")
        # Complementary (needs changes to firmware)
        if self.args["estimator"] == "complementary":
            try:
                self._cf.param.set_value("complementaryFilter.reset", "1")
                time.sleep(1)
                self._cf.param.set_value("complementaryFilter.reset", "0")
                #time.sleep(1)
            except:
                pass

    def ot_receive_new_frame(self, *args, **kwargs):
        pass

    def ot_receive_rigidbody_frame(self, id, position, rotation):
        # Check ID
        if id in self.ot_id:
            # get optitrack data in crazyflie global frame
            pos_in_cf_frame = util.ot2control(position)
            att_in_cf_frame = util.quat2euler(rotation)
            quat_in_cf_frame = util.ot2control_quat(rotation)
            
            idx = self.ot_id.index(id)

            if idx==0:
                # main drone
                ot_dict = {
                    "otX0": pos_in_cf_frame[0],
                    "otY0": pos_in_cf_frame[1],
                    "otZ0": pos_in_cf_frame[2],
                    "otRoll0": att_in_cf_frame[0],
                    "otPitch0": att_in_cf_frame[1],
                    "otYaw0": att_in_cf_frame[2]
                }
                self.ot_position = pos_in_cf_frame
                self.ot_attitude = att_in_cf_frame
                self.ot_quaternion = quat_in_cf_frame
                self.flogger.registerData("ot0", ot_dict)
                (self.filtered_pos[0], self.pos_filter_zi[0]) = scipy.signal.sosfilt(
                    self.ot_filter_sos, [self.ot_position[0]], zi=self.pos_filter_zi[0]
                )
                (self.filtered_pos[1], self.pos_filter_zi[1]) = scipy.signal.sosfilt(
                    self.ot_filter_sos, [self.ot_position[1]], zi=self.pos_filter_zi[1]
                )
                (self.filtered_pos[2], self.pos_filter_zi[2]) = scipy.signal.sosfilt(
                    self.ot_filter_sos, [self.ot_position[2]], zi=self.pos_filter_zi[2]
                )
            elif idx==1:
                ot_dict = {
                    "otX1": pos_in_cf_frame[0],
                    "otY1": pos_in_cf_frame[1],
                    "otZ1": pos_in_cf_frame[2],
                    "otRoll1": att_in_cf_frame[0],
                    "otPitch1": att_in_cf_frame[1],
                    "otYaw1": att_in_cf_frame[2]
                }
                self.flogger.registerData("ot1", ot_dict)



    def do_taskdump(self):
        self._cf.param.set_value("system.taskDump", "1")

    def process_taskdump(self, console_log):
        file = self.get_filename()
        # Dataframe placeholders
        label_data, load_data, stack_data = [], [], []

        # Get headers
        headers = []
        for i, line in enumerate(console_log):
            if "Task dump" in line:
                headers.append(i)
        # None indicates the end of the list
        headers.append(None)

        # Get one task dump
        for i in range(len(headers) - 1):
            dump = console_log[headers[i] + 2 : headers[i + 1]]

            # Process strings: strip \n, \t, spaces, SYSLOAD:
            loads, stacks, labels = [], [], []
            for line in dump:
                entries = line.strip("SYSLOAD: ").split("\t")
                loads.append(entries[0].strip())  # no sep means strip all space, \n, \t
                stacks.append(entries[1].strip())
                labels.append(entries[2].strip())

            # Store labels
            if not label_data:
                label_data = labels

            # Append to placeholders
            load_data.append(loads)
            stack_data.append(stacks)

        # Check if we have data at all
        if headers[0] is not None and label_data:
            # Put in dataframe
            load_data = pd.DataFrame(load_data, columns=label_data)
            stack_data = pd.DataFrame(stack_data, columns=label_data)

            # Save dataframes
            load_data.to_csv(file.strip(".csv") + "+load.csv", sep=",", index=False)
            stack_data.to_csv(file.strip(".csv") + "+stackleft.csv", sep=",", index=False)
        else:
            print("No task dump data found")


    def controller_connected(self):
        """ Return True if a controller is connected """
        return len(self._jr.available_devices()) > 0

    def setup_controller(self, map="PS3_Mode_1"):
        devs = []
        for d in self._jr.available_devices():
            devs.append(d.name)
        
        if len(devs)==1:
            input_device = 0
        else:
            print("Multiple controllers detected:")
            for i, dev in enumerate(devs):
                print(" - Controller #{}: {}".format(i, dev))
            
            input_device = int(input("Select controller: "))

        if not input_device in range(len(devs)):
            raise ValueError
        
        self._jr.start_input(devs[input_device])
        self._jr.set_input_map(devs[input_device], map)

    def connect_crazyflie(self, uri):   
        """Connect to a Crazyflie on the given link uri"""
        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        if self.mode == Mode.AUTO or self.mode == Mode.DONT_FLY:
            self._cf.open_link(uri)
        else:
            # Add callbacks for manual control
            self._cf.param.add_update_callback(
                group="imu_sensors", name="AK8963", cb=(
                    lambda name, found: self._jr.set_alt_hold_available(
                        eval(found))))
            # self._jr.assisted_control_updated.add_callback(
            #     lambda enabled: self._cf.param.set_value("flightmode.althold",
            #                                          enabled))
            self._cf.open_link(uri)
            self._jr.input_updated.add_callback(self.controller_input_cb)
            
            if self.mode == Mode.MODE_SWITCH:
                self._jr.alt1_updated.add_callback(self.mode_switch_cb)


    def _connected(self, link):
        """This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print("Connected to %s" % link)
        # set estimator
        if args["estimator"]=="kalman":
            self._cf.param.set_value("stabilizer.estimator", "2")
        self.flogger.start()
        print("logging started")

    def _connection_failed(self, link_uri, msg):
        print("Connection to %s failed: %s" % (link_uri, msg))
        self.flogger.is_connected = False

    def _connection_lost(self, link_uri, msg):
        print("Connection to %s lost: %s" % (link_uri, msg))
        self.flogger.is_connected = False

    def _disconnected(self, link_uri):
        print("Disconnected from %s" % link_uri)
        self.flogger.is_connected = False

    def ready_to_fly(self):
        # Wait for connection
        timeout = 10
        while not self._cf.is_connected():
            print("Waiting for Crazyflie connection...")
            time.sleep(2)
            timeout -= 1
            if timeout<=0:
                return False
        
        # Wait for optitrack
        if self.optitrack_enabled:
            while (self.ot_position == 0).any():
                print("Waiting for OptiTrack fix...")
                time.sleep(2)
                timeout -= 1
                if timeout <= 0:
                    return False

            print("OptiTrack fix acquired")

        self.ranging_algo("disabled")
        print("Reset Estimator...")
        self._cf.param.set_value("kalman.maxPos", "1000")
        self._cf.param.set_value("kalman.maxVel", "10")
        self.reset_estimator()
        time.sleep(3)   # wait for kalman to stabilize

        return True
        
    def start_flight(self):
        if self.ready_to_fly():
            if self.mode == Mode.MANUAL:
                print("Manual Flight - Ready to fly")
                self.manual_flight()
            elif self.mode == Mode.DONT_FLY:
                print("Ready to not fly")
                try:
                    while True:
                        pass
                except KeyboardInterrupt:
                    print("Flight stopped")
            else:
                # Build trajectory
                setpoints = self.build_trajectory(self.args["trajectory"], self.args["space"])
                # Do flight
                if self.mode == Mode.AUTO:
                    print("Autonomous Flight - Starting flight")
                    self.follow_setpoints(self._cf, setpoints, self.args["optitrack"])
                    print("Flight complete.")
                else:
                    print("Ready to fly")
                    self.manual_flight()
                    print("Starting Trajectory")
                    self.follow_setpoints(self._cf, setpoints, self.args["optitrack"])

        else:
            print("Timeout while waiting for flight ready.")

    def controller_input_cb(self, *data):
        # only forward control in manual mode
        if self.is_in_manual_control:
            self._cf.commander.send_setpoint(*data)

    def mode_switch_cb(self, auto_mode):
        if auto_mode:
            print("Switching autonomous flight")
            self.is_in_manual_control = False
        else:
            print("Switching to manual control")
            self.is_in_manual_control = True

    def manual_flight(self):
        self.is_in_manual_control = True
        while(self.is_in_manual_control):
            if self.args["optitrack"]=="state":
                # self._cf.extpos.send_extpos(
                #     self.filtered_pos[0], self.filtered_pos[1], self.filtered_pos[2]
                #     )
                self._cf.extpos.send_extpos(
                    self.ot_position[0], self.ot_position[1], self.ot_position[2]
                    )
                # self._cf.extpos.send_extpose(
                #     self.ot_position[0], self.ot_position[1], self.ot_position[2],
                #     self.ot_quaternion[0], self.ot_quaternion[1], self.ot_quaternion[2], self.ot_quaternion[3]
                #     )
            time.sleep(0.01)

    def build_trajectory(self, trajectories, space):
        # Load yaml file with space specification
        with open(space, "r") as f:
            space = yaml.full_load(f)
            home = space["home"]
            ranges = space["range"]

        # Account for height offset
        altitude = home["z"] + ranges["z"]
        side_length = min([ranges["x"], ranges["y"]]) * 2
        radius = min([ranges["x"], ranges["y"]])
        x_bound = [home["x"] - ranges["x"], home["x"] + ranges["x"]]
        y_bound = [home["y"] - ranges["y"], home["y"] + ranges["y"]]

        # Build trajectory
        # Takeoff
        setpoints = takeoff(home["x"]+0.5, home["y"]-0.5, altitude, 0.0)
        for trajectory in trajectories:
            # If nothing, only nothing
            if trajectory == "nothing":
                setpoints = None
                return setpoints
            elif trajectory == "hover":
                setpoints += hover(home["x"]+0.5, home["y"]-0.5, altitude)
            elif trajectory == "hover_fw":
                setpoints += hover_fw(home["x"], home["y"], altitude)
            elif trajectory == "square":
                setpoints += square3(home["x"], home["y"], side_length, altitude)
            elif trajectory == "square_fw":
                setpoints += square_fw(home["x"], home["y"], side_length, altitude)
            elif trajectory == "octagon":
                setpoints += octagon(home["x"], home["y"], radius, altitude)
            elif trajectory == "triangle":
                setpoints += triangle(home["x"], home["y"], radius, altitude)
            elif trajectory == "hourglass":
                setpoints += hourglass(home["x"], home["y"], side_length, altitude)
            elif trajectory == "random":
                setpoints += randoms(home["x"], home["y"], x_bound, y_bound, altitude)
            elif trajectory == "scan":
                setpoints += scan(home["x"], home["y"], x_bound, y_bound, altitude)
            else:
                raise ValueError("{} is an unknown trajectory".format(trajectory))

        # Add landing
        setpoints += landing(home["x"]+0.5, home["y"]-0.5, altitude, 0.0)

        return setpoints

    def update_every_second(self, stop, point):
        while True:
            self._cf.commander.send_position_setpoint(*point[:4])
            time.sleep(0.05)
            if stop():
                break

    def follow_setpoints(self, cf, setpoints, optitrack):
        # Counter for task dump logging
        time_since_dump = 0.0
        current_tdoa_state = 1
        # Start
        try:
            print("Flight started")
            # Do nothing, just sit on the ground
            if setpoints is None:
                while True:
                    time.sleep(0.1)
                    time_since_dump += 0.05

                    # Task dump
                    if time_since_dump > 2:
                        print("Do task dump")
                        self.do_taskdump()
                        time_since_dump = 0.0

            # Do actual flight
            else:
                for i, point in enumerate(setpoints):
                    print("Next setpoint: {}".format(point))
                    first_setpoint_done = i
                    # Compute time based on distance
                    # Take-off
                    if i == 0:
                        distance = point[2]
                    # No take-off
                    else:
                        distance = np.sqrt(
                            ((np.array(point[:3]) - np.array(setpoints[i - 1][:3])) ** 2).sum()
                        )

                    # If zero distance, at least some wait time
                    if distance == 0.0:
                        wait = 2
                    else:
                        wait = distance * 2

                    # Send position and wait
                    time_passed = 0.0
                    
                    while time_passed < wait:
                        if self.is_in_manual_control:
                            self.manual_flight()
                        # If we use OptiTrack for control, send position to Crazyflie
                        if optitrack == "state":
                            cf.extpos.send_extpos(
                                self.filtered_pos[0], self.filtered_pos[1], self.filtered_pos[2]
                            )

                        cf.commander.send_position_setpoint(*point[:4])
                        
                        

                        # if we fly out of the cyberzoo, reconnect tdoa and send optitrack position 
                    #    if abs(self.filtered_pos[0])>3 or abs(self.filtered_pos[1])>3:
                         #   if(current_tdoa_state == 0):
                             #   self.tdoa_status(args["coverage"],1) 
                                #tell the quad where we are
                             #   time.sleep(0.05)
                               # cf.extpos.send_extpos(self.filtered_pos[0], self.filtered_pos[1], self.filtered_pos[2])
                             #   current_tdoa_state = 1

                        time.sleep(0.05)
                        time_passed += 0.05
                        time_since_dump += 0.05
                    thread = 1
                    while first_setpoint_done == 0:
                        stop_threads = False
                        start = 0
                        if thread == 1:
                            t = Thread(target=self.update_every_second, args = (lambda : stop_threads, point))
                            t.start()  
                            thread = 0
                        start = input("Input 1 if you dare to start and suffer the consequences of crash: ")
                        if start == "1":
                            print("setpoint done")
                          #  state = 1
                          #  stop_threads = True
                          #  t.join()
                            
                           # first_setpoint_done = 2
                            self.ranging_algo(args["ranging_algo"])
                            start = 0


                    if(current_tdoa_state != point[4]):
                        print("want to change tdoa")
                        self.tdoa_status(args["coverage"],point[4])
                       # time.sleep(0.05)
                        current_tdoa_state = point[4]
                   #     current_tdoa_state = point[4]
                    # Task dump
                    # if time_since_dump > 2:
                    #     print("Do task dump")
                    #     self.do_taskdump()
                    #     time_since_dump = 0.0

                # Finished
                cf.commander.send_stop_setpoint()

        # Prematurely break off flight / quit doing nothing
        except KeyboardInterrupt:
            if setpoints is None:
                print("Quit doing nothing!")
            else:
                print("Emergency landing!")
                wait = setpoints[i][2] * 2
                cf.commander.send_position_setpoint(setpoints[i][0], setpoints[i][1], 0.0, 0.0)
                time.sleep(wait)
                cf.commander.send_stop_setpoint()


    def setup_console_dump(self):
        # Console dump file
        self.console_log = []
        console = Console(self._cf)
        console.receivedChar.add_callback(self._console_cb)
        self.console_dump_enabled = True

    def _console_cb(self, text):
        # print(text)
        self.console_log.append(text)

    def end(self):
        self._cf.close_link()
        # Process task dumps
        # TODO: add timestamps / ticks (like logging) to this
        if self.console_dump_enabled:
            self.process_taskdump(self.console_log)

if __name__ == "__main__":

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--fileroot", type=str, default=None)
    parser.add_argument("--keywords", nargs="+", type=str.lower, default=None)
    parser.add_argument("--logconfig", type=str, required=True)
    parser.add_argument("--space", type=str, required=True)
    parser.add_argument(
        "--estimator",
        choices=["complementary", "kalman"],
        type=str.lower,
        required=True,
    )
    parser.add_argument(
        "--uwb", choices=["none", "twr", "tdoa"], type=str.lower, required=True
    )
    parser.add_argument("--flow", action="store_true")
    parser.add_argument("--trajectory", nargs="+", type=str.lower, default=None)
    parser.add_argument("--safetypilot", action="store_true")
    parser.add_argument(
        "--optitrack",
        choices=["none", "logging", "state"],
        type=str.lower,
        default="none",
    )
    parser.add_argument("--optitrack_id", nargs="+", type=int, default=None)
    parser.add_argument("--filename", type=str, default=None)
    parser.add_argument("--uri", type=str, default="radio://0/80/2M/E7E7E7E7E7")
    parser.add_argument(
        "--ranging_algo", choices=["disabled","vanilla", "cu", "ci-cu"], type=str.lower, required=True
    )
    parser.add_argument(
        "--coverage", choices=["full", "lim"], type=str.lower, required=True
    )
    
    args = vars(parser.parse_args())

    # Set up log flight
    lf = LogFlight(args)
    lf.connect_crazyflie(args["uri"])
    # Set up print connection to console
    # TODO: synchronize this with FileLogger: is this possible?
    lf.setup_console_dump()

    try:
        lf.start_flight()
    except KeyboardInterrupt:
        print("Keyboard Interrupt")

    # End flight
    print("Done - Please Wait for CF to disconnect")
    time.sleep(1)
    lf.end()
