
import yaml
import threading
import argparse

from sr.robot import *

parser = argparse.ArgumentParser()
parser.add_argument('-c', '--config',
                    type=argparse.FileType('r'),
                    default='games/two_colours_assignment.yaml')
parser.add_argument('robot_scripts',
                    type=argparse.FileType('r'),
                    nargs='*')
parser.add_argument("-t", "--delta", type=float, default=[0.05], nargs=1)
parser.add_argument("-f", "--file", 
                    type=argparse.FileType("a"),
                    nargs=1
                    )
args = parser.parse_args()
finished = [False]

def read_file(fn):
    with open(fn, 'r') as f:
        return f.read()

robot_scripts = args.robot_scripts
prompt = "Enter the names of the Python files to run, separated by commas: "
while not robot_scripts:
    robot_script_names = raw_input(prompt).split(',')
    if robot_script_names == ['']: continue
    robot_scripts = [read_file(s.strip()) for s in robot_script_names]

with args.config as f:
    config = yaml.load(f)

sim = Simulator(config, background=False)

class RobotThread(threading.Thread):
    def __init__(self, zone, script, *args, **kwargs):
        super(RobotThread, self).__init__(*args, **kwargs)
        self.zone = zone
        self.script = script
        self.daemon = True

    def run(self):
        def robot():
            with sim.arena.physics_lock:
                robot_object = SimRobot(sim)
                robot_object.zone = self.zone
                robot_object.location = sim.arena.start_locations[self.zone]
                robot_object.heading = sim.arena.start_headings[self.zone]
                return robot_object
            
        def get_delta():
            if args.delta:
                return args.delta[0]
            return 0.05
        
        def get_file():
            if args.file:
                return args.file[0]
            return None

        exec(self.script, {'Robot': robot, "get_dt": get_delta, "get_file": get_file, "finished": finished})

threads = []
for zone, robot in enumerate(robot_scripts):
    thread = RobotThread(zone, robot)
    thread.start()
    threads.append(thread)

sim.run(finished)

# Warn PyScripter users that despite the exit of the main thread, the daemon
# threads won't actually have gone away. See commit 8cad7add for more details.
threads = [t for t in threads if t.is_alive()]
if threads:
    print("WARNING: {0} robot code threads still active.".format(len(threads)))
    #####                                                               #####
    # If you see the above warning in PyScripter and you want to kill your  #
    # robot code you can press Ctrl+F2 to re-initialize the interpreter and #
    # stop the code running.                                                #
    #####                                                               #####
