# -*- coding: utf-8 -*-
"""
Created on Mon Nov  1 18:38:23 2021

@author: omar
Language: Python
Required libraries: numpy, math, random
Optional libraries: pygame (for visulization only)
"""

"""
------------------------------------------------------------------------------------------------------------------------------------------
                                                *** RRT_planner ***
                                                
    This script contains the main algorithm for the RRT path planning.  
------------------------------------------------------------------------------------------------------------------------------------------
"""

import pygame as pg
import pandas as pd
from Path_Planning_Library import Path_Planning_scene
from Path_Planning_Library import RRT
from Path_Planning_Library import RRT_star

from sys import exit
import networkx as nx

def main():
    # initilize simulation parameters 
    dimensions_window = (800,800)
    dimensions = ([0,10],[0,10])
    start = (1,1)
    goal = (7,9)
    iteration = 0
    Max_Iteration = 700
    Neighberhood = 5
    method = 'RRT_star'
    #method = 'RRT'
    Solution_path = []
    Solution_cost = float('inf')
    pg.init()
    obstacles = pd.read_csv('obstacles.csv', sep=',', comment='#',header=None).values
    print('obstacles = ',obstacles)


    Env = Path_Planning_scene(start,goal,obstacles,dimensions,dimensions_window)
    Env.draw_scene()
    
    
    Graph = RRT(start,goal,obstacles,dimensions)
    Graph_star = RRT_star(start,goal,obstacles,dimensions,Neighberhood)
    
    if method == 'RRT_star':
        Graph = Graph_star
    if method[:3]=='RRT':
        while True:  
            while iteration<Max_Iteration and not Graph.goal_reached:
                if iteration % 100 == 0:
                    Graph.Tree = Graph.bias()
                    Env = Path_Planning_scene(start,goal,obstacles,dimensions,dimensions_window)
                    Env.draw_scene()
                    for node in Graph.Tree:
                        pg.draw.circle(Env.Scene, Env.Blue, Env.transform_node(Graph.Tree.nodes[node]["position"]), Env.node_size,Env.node_thickness)
                        pg.draw.circle(Env.Scene, Env.Grey, Env.transform_node(Graph.Tree.nodes[node]["position"]), Env.node_size+5,Env.node_thickness+5)
                    for edge in Graph.Tree.edges:
                       pg.draw.line(Env.Scene, Env.Black, Env.transform_node(Graph.Tree.nodes[edge[0]]["position"]), Env.transform_node(Graph.Tree.nodes[edge[1]]["position"]),Env.edge_thickness) 
 
                                    
                else:
                    Graph.Tree = Graph.expand()
                    Env = Path_Planning_scene(start,goal,obstacles,dimensions,dimensions_window)
                    Env.draw_scene()
                    for node in Graph.Tree:
                        pg.draw.circle(Env.Scene, Env.Blue, Env.transform_node(Graph.Tree.nodes[node]["position"]), Env.node_size,Env.node_thickness)
                        pg.draw.circle(Env.Scene, Env.Grey, Env.transform_node(Graph.Tree.nodes[node]["position"]), Env.node_size+5,Env.node_thickness+5)
                    for edge in Graph.Tree.edges:
                       pg.draw.line(Env.Scene, Env.Black, Env.transform_node(Graph.Tree.nodes[edge[0]]["position"]), Env.transform_node(Graph.Tree.nodes[edge[1]]["position"]),Env.edge_thickness)

                   
                if Graph.goal_reached:
                    print("Goal is reached at iteration ", iteration)
                    Graph.generate_path()
                    Env.draw_path(Graph.Tree,Graph.path)
                    if Solution_cost>Graph.calculate_cost():
                        Solution_path = Graph.path
                        Solution_cost = Graph.calculate_cost()
                    print("the path found is ",Graph.path)
                    print("Total cost of path is: ",Graph.calculate_cost())
                if method == 'RRT_star':
                    Graph.goal_reached=False             
                iteration += 1

                
                pg.display.update()
                pg.event.clear()
                pg.event.wait(1)
    
                for event in pg.event.get():
                    if event.type == pg.QUIT:
                        pg.display.quit()
                        pg.quit()
                        exit()
            
            if Solution_path == []:
                print("Goal is not reached at max iteration ", Max_Iteration)
            else:
                print('The solution path after Max_iteration: ',Solution_path)
                print("Total cost of solution path is: ",Solution_cost)
                Env.draw_path(Graph.Tree,Solution_path)
           
            pg.display.update()
            pg.event.clear()
            pg.event.wait()    
            for event in pg.event.get():
                if event.type == pg.QUIT:
                    pg.display.quit()
                    pg.quit()
                    exit()
                    
                

if __name__ == '__main__':
    main()
    