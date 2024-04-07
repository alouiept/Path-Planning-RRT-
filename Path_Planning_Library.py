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
                                                *** Path_Planning_Library ***
                                                
    This script contains classes and methods to use for RRT path planning.  
------------------------------------------------------------------------------------------------------------------------------------------
"""

import random
import networkx as nx
import pygame as pg

class Path_Planning_scene:
    def __init__(self,start_node,goal_node,obstacles,Scene_dim,Window_dimensions):
        self.start = start_node
        self.goal = goal_node
        self.boundaries = Window_dimensions
        self.Height,self.Width = self.boundaries 
        self.Scene_dimensions = Scene_dim
        # simulation window settings
        self.Env_window_name = 'RRT path planning'
        pg.display.set_caption(self.Env_window_name)
        self.Scene = pg.display.set_mode((self.Height,self.Width))
        self.Scene.fill((255,255,255))
        self.node_size = 5
        self.node_thickness = 0
        self.edge_thickness = 2
        
        # Obstacles settings
        self.obstacles = obstacles
        self.size = obstacles[:,2].tolist()
        self.obs_positions = obstacles[:,0:2].tolist()

        
        #┼ Colors
        self.Grey = (70,70,70)
        self.Blue = (0,0,255)
        self.Red = (255,0,0)
        self.Green = (0,255,0)
        self.Yellow = (255,255,0)
        self.Black = (0,0,0)
        self.White = (255,255,255)
        

    def transform_scence(self):
        Width_scence = self.Scene_dimensions[0][1]-self.Scene_dimensions[0][0]
        Height_scence = self.Scene_dimensions[1][1]-self.Scene_dimensions[1][0]
        y_scale = self.Height/Height_scence
        x_scale = self.Width/Width_scence
        translate_x = -self.Scene_dimensions[0][0]*x_scale
        translate_y = -self.Scene_dimensions[1][0]*y_scale
        self.obs_positions = [[obstacle[0]*x_scale+translate_x,obstacle[1]*y_scale+translate_y] for obstacle in self.obstacles]
        self.size = (self.obstacles[:,2] * x_scale).tolist()
        self.start = (self.start[0]*x_scale+translate_x,self.start[1]*y_scale+translate_y)
        self.goal = (self.goal[0]*x_scale+translate_x,self.goal[1]*y_scale+translate_y)
        return self.obs_positions,self.size,self.start,self.goal
    

        
    def transform_node(self,node):
        Width_scence = self.Scene_dimensions[0][1]-self.Scene_dimensions[0][0]
        Height_scence = self.Scene_dimensions[1][1]-self.Scene_dimensions[1][0]
        y_scale = self.Height/Height_scence
        x_scale = self.Width/Width_scence
        translate_x = -self.Scene_dimensions[0][0]*x_scale
        translate_y = -self.Scene_dimensions[1][0]*y_scale
        node = (node[0]*x_scale+translate_x,node[1]*y_scale+translate_y)
        
        return node
        
    def draw_scene(self):
        self.transform_scence()
        pg.draw.circle(self.Scene, self.Red, self.start, self.node_size+20,0)
        pg.draw.circle(self.Scene, self.Green, self.goal, self.node_size+20,0)
        self.draw_obstacles()
        
    
    def draw_path(self,Tree,path):
        for i in range(0,len(path)-1):
            pg.draw.line(self.Scene, self.Green, self.transform_node(Tree.nodes[path[i]]["position"]), self.transform_node(Tree.nodes[path[i+1]]["position"]),self.edge_thickness+10)
        for node in path:
            pg.draw.circle(self.Scene, self.Green, self.transform_node(Tree.nodes[node]["position"]), self.node_size,self.node_thickness)
            pg.draw.circle(self.Scene, self.Grey, self.transform_node(Tree.nodes[node]["position"]), self.node_size+5,self.node_thickness+5)
    
    def draw_obstacles(self):
        pos_list =self.obs_positions.copy()
        size_list = self.size.copy()
        while not pos_list == []:
            obstacle = pos_list.pop(0)
            size = size_list.pop(0)
            pg.draw.circle(self.Scene,self.Grey, obstacle, size/2,0)
        


class RRT:
    def __init__(self,start_node,goal_node,obstacles,Scene_dim):
        self.start = start_node
        self.goal = goal_node
        self.is_goal = False
        self.Tree = nx.Graph()
        self.Scene_dimensions = Scene_dim
        
        # Initilize the tree 
        self.Tree.add_node(0)
        self.Tree.nodes[0]["position"]= self.start
        self.Tree.nodes[0]["parent"]=0
        
       # Obstacles settings
        self.obstacles = obstacles
        self.safety_margin = 0.2
        size = obstacles[:,2].tolist()
        self.size=[d+self.safety_margin for d in size]
        self.obs_positions = obstacles[:,0:2].tolist()
        
        # Initialize path
        self.goal_reached = None
        self.path = []
       
    
    def distance(self,p1,p2):
        xp1,yp1 = p1
        xp2,yp2 = p2
        d = ((xp1-xp2)**2+(yp1-yp2)**2)**0.5 
        return d
    
    def sampling_scene(self):
        node_obstacle_collision = True
        while node_obstacle_collision:
            obstacles = self.obs_positions.copy()
            radii = [diameter/2 for diameter in self.size]
            New_node_pos = (float('%0.2f' % random.uniform(self.Scene_dimensions[0][0]+1,self.Scene_dimensions[0][1]-1)),
                            float('%0.2f' % random.uniform(self.Scene_dimensions[1][0]+1,self.Scene_dimensions[1][1]-1)))
            while not obstacles == []:
                obstacle = obstacles.pop(0)
                radius = radii.pop(0)
                if self.distance(obstacle,New_node_pos)<radius:
                    break
            node_obstacle_collision = False
        return New_node_pos

    def nearest_node(self,New_node_pos):
        dmin=self.distance(self.Tree.nodes[0]["position"],New_node_pos)
        Nearest_node = 0
        for node in self.Tree:
            d_current=self.distance(self.Tree.nodes[node]["position"],New_node_pos)
            if d_current<dmin:
                dmin = d_current
                Nearest_node = node
        return Nearest_node
     
    def cross_obstacle(self,node1_pos,node2_pos):
        node1_pos_x,node1_pos_y=node1_pos
        node2_pos_x,node2_pos_y=node2_pos
        obstacles = self.obs_positions.copy()
        radii = [diameter/2 for diameter in self.size]
        while not obstacles == []:
            obstacle = obstacles.pop(0)
            radius = radii.pop(0)
            for i in range(0,101):
                u=i/100
                node_pos_x=node1_pos_x+u*(node2_pos_x-node1_pos_x)
                node_pos_y=node1_pos_y+u*(node2_pos_y-node1_pos_y)
                node_pos = (node_pos_x,node_pos_y)
                if self.distance(obstacle,node_pos)<radius:
                    return True
        return False
    
  
    def step(self,Nearest_node_ID,New_node_pos,goal_radius=1,step_size=0.4):      
        x_new = New_node_pos[0]
        y_new = New_node_pos[1]
        x_nearest = self.Tree.nodes[Nearest_node_ID]["position"][0]
        y_nearest = self.Tree.nodes[Nearest_node_ID]["position"][1]
        d = self.distance(self.Tree.nodes[Nearest_node_ID]["position"], New_node_pos)
        if d ==0:
            d=1
        Node_step_pos = (x_nearest+step_size*(x_new-x_nearest)/d,y_nearest+step_size*(y_new-y_nearest)/d)
        ID_new = len(self.Tree)
        if not self.cross_obstacle(self.Tree.nodes[Nearest_node_ID]["position"], Node_step_pos):
            if self.distance(self.goal,Node_step_pos)<goal_radius:
                self.Tree.add_node(ID_new)
                self.Tree.nodes[ID_new]["position"] = self.goal
                self.Tree.nodes[ID_new]["parent"] = Nearest_node_ID
                cost_edge = self.distance(self.goal,Node_step_pos) 
                self.Tree.add_edge(Nearest_node_ID, ID_new)
                self.Tree[Nearest_node_ID][ID_new]["cost_edge"] = cost_edge
                self.goal_reached = True
            else :
                self.Tree.add_node(ID_new)
                self.Tree.nodes[ID_new]["position"] = Node_step_pos
                self.Tree.nodes[ID_new]["parent"] = Nearest_node_ID
                cost_edge = self.distance(self.Tree.nodes[Nearest_node_ID]["position"],Node_step_pos) 
                self.Tree.add_edge(Nearest_node_ID, ID_new)
                self.Tree[Nearest_node_ID][ID_new]["cost_edge"] = cost_edge

                             
    def generate_path(self):
        self.path = nx.shortest_path(self.Tree, 0,len(self.Tree)-1,method = "bellman-ford",weight = "cost_edge")
        
        return self.path
    
    
    def bias(self):
        ID_nearest = self.nearest_node(self.goal)
        self.step(ID_nearest, self.goal)
        return self.Tree
    
    def expand(self):
        New_node_pos = self.sampling_scene()
        ID_nearest = self.nearest_node(New_node_pos)
        self.step(ID_nearest, New_node_pos)
        return self.Tree 
        
    
    def calculate_cost(self):
        cost = 0
        for i in range(0,len(self.path)-1):
            cost +=  self.Tree[self.path[i]][self.path[i+1]]["cost_edge"]
        return float('%0.3f' %cost)

class RRT_star(RRT):
    def __init__(self,start_node, goal_node, obstacles, Scene_dim,Neighborhood):
        RRT.__init__(self, start_node, goal_node, obstacles, Scene_dim)
        self.Neighborhood = Neighborhood
        
    def Neighbors_in_neighborhood(self,New_node_pos):
        List_neighbors = []
        for node in self.Tree:
            if self.distance(self.Tree.nodes[node]['position'],New_node_pos)<self.Neighborhood:
                List_neighbors.append(node)
                
        return List_neighbors
    
    def generate_path_to_node(self,NeighborID):
        path_to_node = nx.shortest_path(self.Tree, 0,NeighborID,method = "bellman-ford",weight = "cost_edge")
        
        return path_to_node
    
    def calculate_cost_of_path(self,path):
        cost = 0
        for i in range(0,len(path)-1):
            cost +=  self.Tree[path[i]][path[i+1]]["cost_edge"]
        return float('%0.3f' %cost)
    
    def nearest_node_in_subgraph(self,List_nodes,New_node_pos):
        SubGraph = self.Tree.subgraph(List_nodes)
        dmin=self.distance(SubGraph.nodes[0]["position"],New_node_pos)
        Nearest_node = 0
        for node in self.Tree:
            d_current=self.distance(self.Tree.nodes[node]["position"],New_node_pos)
            if d_current<dmin:
                dmin = d_current
                Nearest_node = node
        return Nearest_node
    
    def rewire_Tree(self,New_ID):
        New_node_pos = self.Tree.nodes[New_ID]['position']
        List_neighbors = self.Neighbors_in_neighborhood(New_node_pos)
        path_to_New = self.generate_path_to_node(New_ID)
        path_to_New_cost = self.calculate_cost_of_path(path_to_New)
        for neighbor in List_neighbors:
            path_to_neighbor = self.generate_path_to_node(neighbor)
            path_to_neighbor_cost = self.calculate_cost_of_path(path_to_neighbor)
            New_cost = path_to_New_cost+self.distance(New_node_pos, self.Tree.nodes[neighbor]['position'])
            
            if path_to_neighbor_cost>New_cost and not self.cross_obstacle(New_node_pos, self.Tree.nodes[neighbor]['position']):
                self.Tree.remove_edge(neighbor, self.Tree.nodes[neighbor]['parent'])
                self.Tree.nodes[neighbor]['parent']=New_ID
                self.Tree.add_edge(New_ID, neighbor)
                self.Tree[New_ID][neighbor]['cost_edge'] = self.distance(New_node_pos, self.Tree.nodes[neighbor]['position'])

    def bias(self):
        ID_nearest = self.nearest_node(self.goal)
        self.step(ID_nearest, self.goal)
        self.rewire_Tree(len(self.Tree)-1)
        return self.Tree
    
    def expand(self):
        New_node_pos = self.sampling_scene()
        ID_nearest = self.nearest_node(New_node_pos)
        self.step(ID_nearest, New_node_pos)
        self.rewire_Tree(len(self.Tree)-1)
        return self.Tree 
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        