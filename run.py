from env.empty_env import *
from args.arg_lists import *
from args.util import save_gif, generate_results_dir
import argparse

parser = argparse.ArgumentParser(description='This script handles running motion planning algorithms')

parser.add_argument('--alg', dest='alg', metavar='alg_name', default='rrt',
        help='Name of the algorithm we want to try. Valid algorithms: ' + str(algorithms.keys()))

parser.add_argument('--vehicle', dest='veh', metavar='veh_name', default='particle-2d',
        help='Name of the vehicle dynamics we want. Valid vehicle dynamics: ' + str(vehicles.keys()))

parser.add_argument('--map', dest='map', metavar='map_name', default='empty-2D',
        help='Name of the map we want to evaluate on. Valid maps: ' + str(maps.keys()))

parser.add_argument('--iters', dest='iter', metavar='iters', default='5000',
        help='Maximum number of samples')

parser.add_argument('--delta', dest='delta', metavar='delta', default='20',
        help='Maximum change in position per tree expansion.')

parser.add_argument('--map-file', dest='map_file', metavar='map_file', default='test_maze2.xml',
        help='File name for loading the configuration. The file should reside in env/map_data/ .')

parser.add_argument('--render', dest='rend', metavar='rend', default='',
        help='Whether to render the environment or not. Leave empty for no, insert anything for yes.')

parser.add_argument('--break-on-path', dest='break', metavar='break', default='',
        help='Whether to finish planning if a path is founc. Leave empty for no, insert anything for yes.')

parser.add_argument('--save-run', dest='save', metavar='save', default='',
        help='Whether to save the run as an animation. Any string is considered as true.')

args = parser.parse_args()

if args.alg not in algorithms.keys():
    print(str(args.alg) + " is not a valid algorithm. Please choose from the list: " + str(algorithms.keys()))
    exit(-1)

if args.map not in maps.keys():
    print(str(args.alg) + " is not a valid map. Please choose from the list: " + str(maps.keys()))
    exit(-1)

if args.veh not in vehicles.keys():
    print(str(args.alg) + " is not a valid vehicle. Please choose from the list: " + str(vehicles.keys()))
    exit(-1)

fileName = "env/map_data/" + args.map_file

env = maps[args.map](init_path=fileName)
vehicle = vehicles[args.veh]
planner = algorithms[args.alg](env, float(args.delta), vehicleDynamics=vehicle)
render = False
if args.rend:
    render = True

graph, img_data = planner.plan(int(args.iter), True)

if args.save:
    dir = generate_results_dir(args.map, args.alg)
    save_gif(img_data, dir+'run.gif')

#time.sleep(100)
