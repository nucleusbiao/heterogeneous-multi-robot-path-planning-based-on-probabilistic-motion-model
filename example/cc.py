import yaml
import matplotlib
# matplotlib.use("Agg")
from matplotlib.patches import Circle, Rectangle, Arrow,RegularPolygon
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from matplotlib import animation
import matplotlib.animation as manimation
import argparse
import math
import time,os
Colors = ['orange']#, 'blue', 'green']



if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("map", help="input file containing map")
  parser.add_argument("schedule", help="schedule for agents")
  parser.add_argument('--video', dest='video', default=None, help="output video file (or leave empty to show on screen)")
  parser.add_argument("--speed", type=int, default=1, help="speedup-factor")
  args = parser.parse_args()


  with open(args.map) as map_file:
    map = yaml.load(map_file)

  with open(args.schedule) as states_file:
    schedule = yaml.load(states_file)

  aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

  fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
  ax = fig.add_subplot(111, aspect='equal')
  fig.subplots_adjust(left=0,right=1,bottom=0,top=1, wspace=None, hspace=None)
  # self.ax.set_frame_on(False)
  xmin = -0.5
  ymin = -0.5
  xmax = map["map"]["dimensions"][0] - 0.5
  ymax = map["map"]["dimensions"][1] - 0.5
  ax.set_xticks(np.arange(xmin,xmax,1))
  ax.set_yticks(np.arange(ymin,ymax,1))
  plt.grid()
 
    # self.ax.relim()
  plt.xlim(xmin, xmax)
  plt.ylim(ymin, ymax)
    # self.ax.set_xticks([])
    # self.ax.set_yticks([])
    # plt.axis('off')
    # self.ax.axis('tight')
    # self.ax.axis('off')

  temp=patches.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='red')
  ax.add_patch(temp)

  for o in map["map"]["obstacles"]:
     x, y = o[0], o[1]
     temp=patches.Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='red', edgecolor='red')
     ax.add_patch(temp)
    
    # create agents
    # draw goals first
  for d, i in zip(map["agents"], range(0, len(map["agents"]))):
    temp=patches.Rectangle((d["goal"][0] - 0.25, d["goal"][1] - 0.25), 0.5, 0.5, facecolor=Colors[i%len(Colors)], edgecolor='black', alpha=0.5)
    ax.add_patch(temp)
  for d, i in zip(map["agents"], range(0, len(map["agents"]))):
    name = d["name"]
    temp=patches.Circle((d["start"][0], d["start"][1]), 0.2, facecolor=Colors[i%len(Colors)], edgecolor='black')
    ax.add_patch(temp)
 
  plt.savefig('1.pdf')
 

