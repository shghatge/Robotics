from __future__ import division
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.lines as mlines
import numpy as np
import random, math


class RRT(object):

        def __init__(self, vertices, start, goal, step, color,rrt2):
                self.vertices = vertices
                self.goal = goal
                self.start = start
                self.step = step
                self.edges = []
                self.nodes = []
                self.nodes_parent = []
                self.nodes.append(start)
                self.nodes_parent.append(-1)
                self.done = False
                self.node_color = color
                self.rrt2 = rrt2
                self.load_edges(vertices)

        def load_edges(self, verts):
                num_verts = len(self.vertices)
                first = False
                last = False
                first_pt = []

                for i in range( num_verts - 1 ):

                        if first == False:
                                if verts[i][0] == 0 and verts[i][1] == 0:
                                        first = True
                                        first_pt = [ verts[i+1][0], verts[i+1][1] ]
                                        continue

                        if first == True:
                                if verts[i+1][0] == 0 and verts[i+1][1] == 0:
                                        self.edges.append( [ verts[i][0], verts[i][1], first_pt[0], first_pt[1] ] )
                                        first = False
                                        continue

                        if first == True:
                                self.edges.append( [ verts[i][0], verts[i][1], verts[i+1][0], verts[i+1][1] ] )



        def get_dist(self, pt1, pt2):
                return math.sqrt( (pt1[0] - pt2[0]) * (pt1[0] - pt2[0]) + (pt1[1] - pt2[1]) * (pt1[1] - pt2[1])  )


        def find_closest_node(self, randQ):             
                min_dist = 9999999999
                min_index = -1

                for i in range( len (self.nodes) ):
                        dist = self.get_dist(randQ, self.nodes[i])
                        if dist < min_dist:
                                min_dist = dist
                                min_index = i
                # print("min "+str(min_index)+" dist "+str(min_dist))
                return min_index, min_dist

        def collision_segment(self, p1,p2,p3,p4):
                s1 = [p2[0]-p1[0], p2[1]-p1[1]]
                s2 = [p4[0]-p3[0], p4[1]-p3[1]]
                div = (-s2[0]*s1[1] + s1[0]*s2[1])
                if(div == 0):
                        return False
                s = (-1*s1[1]*(p1[0]-p3[0]) + s1[0] * (p1[1]-p3[1])) / div
                t = (s2[0]*(p1[1]-p3[1]) - s2[1] * (p1[0] - p3[0])) / div
                if(s >= 0 and s<=1 and t>=0 and t<=1):
                        x = p1[0] + (t * s1[0])
                        y = p1[1] + (t * s1[1])
                        if(self.pointIsClose([x,y],p3) or self.pointIsClose([x,y],p4) ):
                                return False
                        return True
                return False

        def collision_detect(self, p1, p2):

                for i in range( len (self.edges) ):

                        if self.collision_segment(p1, p2, (self.edges[i][0], self.edges[i][1]), (self.edges[i][2], self.edges[i][3])) == True:
                                # print("Collision Detected between " + str([p1,p2])+" and "+str(self.edges[i]))
                                return True             
                return False

        def collision_detect_num(self, p1, p2):

                num = 0
                for i in range( len (self.edges) ):

                        if self.collision_segment(p1, p2, (self.edges[i][0], self.edges[i][1]), (self.edges[i][2], self.edges[i][3])) == True:
                                # print("Collision Detected between " + str([p1,p2])+" and "+str(self.edges[i]))
                                num += 1                
                return num

        def is_goal_reached(self, pt):

                if ( self.get_dist(pt, self.goal) < self.step ) and self.collision_detect(pt, self.goal) == False:

                        plt.plot([self.goal[0], pt[0]], [self.goal[1], pt[1]], marker = 'o', color = self.node_color)
                        plt.pause(.001)
                        self.nodes.append(self.goal)
                        self.nodes_parent.append(len(self.nodes) - 2)
                        self.done = True

                        return True
                else:
                        return False

        def isclose(self,a, b, rel_tol=1e-09, abs_tol=0.0):
                return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

        def pointIsClose(self, p1, p2):
                return self.isclose(p1[0],p2[0]) and self.isclose(p1[1],p2[1])

        def plot_and_checkGoal(self, pt1, pt2):

                plt.plot([pt1[0], pt2[0]], [pt1[1], pt2[1]], marker = 'o', color = self.node_color)
                plt.pause(.001)
                if self.is_goal_reached(pt2):
                        return True
                else:
                        return False


        def grow_to_randq(self, randQ):

                m_size = 15
                
                closest_node_index, dist = self.find_closest_node(randQ)
                closest_node = self.nodes[closest_node_index]

                if dist < self.step:
                        return

                if(self.get_dist(randQ, closest_node) < self.step):

                        if self.collision_detect(closest_node, randQ) == True:
                                return

                        self.nodes.append(list(randQ).copy())
                        self.nodes_parent.append( closest_node_index )

                        if self.plot_and_checkGoal(closest_node, randQ) == True:
                                return


                nQ = [ randQ[0] - closest_node[0], randQ[1] - closest_node[1] ]
                nQ[0] = nQ[0] / dist
                nQ[1] = nQ[1] / dist

                new_node = [ 0, 0]
                new_node[0] = closest_node[0] + nQ[0] * self.step
                new_node[1] = closest_node[1] + nQ[1] * self.step

                if self.collision_detect(closest_node, new_node) == True:
                        return

                self.nodes.append(new_node.copy())
                self.nodes_parent.append( closest_node_index )

                if self.plot_and_checkGoal(closest_node, new_node) == True:
                        return

                prev_node = new_node.copy()

                new_node[0] = new_node[0] + nQ[0] * self.step
                new_node[1] = new_node[1] + nQ[1] * self.step
                        
                while(True):

                        if(self.get_dist(randQ, prev_node) < self.step):

                                if self.collision_detect(prev_node, new_node) == True:
                                        return

                                self.nodes.append(list(new_node).copy())
                                self.nodes_parent.append( len(self.nodes) - 2 )

                                if self.plot_and_checkGoal(prev_node, new_node) == True:
                                        return

                                break

                        else:

                                if self.collision_detect(new_node, prev_node) == True:
                                        return

                                self.nodes.append(new_node.copy())
                                self.nodes_parent.append( len(self.nodes) - 2 )

                                if self.plot_and_checkGoal(prev_node, new_node) == True:
                                        return

                        prev_node = new_node.copy()     

                        new_node[0] = new_node[0] + nQ[0] * self.step
                        new_node[1] = new_node[1] + nQ[1] * self.step
                
        def draw_path(self):

                if self.done == False:
                        return

                point = self.nodes[-1]
                parent_ind = self.nodes_parent[ len(self.nodes) - 1]
                # parent = self.nodes[parent_ind]

                while(parent_ind != -1):

                        # print("len "+str(len(self.nodes))+" "+str(parent_ind))
                        parent = self.nodes[parent_ind]
                        plt.plot([point[0], parent[0]], [point[1], parent[1]], marker = 'o', color = 'xkcd:green')
                        plt.pause(0.001)
                        parent_ind = self.nodes_parent[parent_ind]
                        point = list(parent).copy()
                point = self.rrt2.nodes[-1]
                parent_ind = self.rrt2.nodes_parent[ len(self.rrt2.nodes) - 1]
                # parent = self.nodes[parent_ind]

                while(parent_ind != -1):

                        # print("len "+str(len(self.nodes))+" "+str(parent_ind))
                        parent = self.rrt2.nodes[parent_ind]
                        plt.plot([point[0], parent[0]], [point[1], parent[1]], marker = 'o', color = 'xkcd:green')
                        plt.pause(0.001)
                        parent_ind = self.rrt2.nodes_parent[parent_ind]
                        point = list(parent).copy()
                plt.show(block=True)
                
        def grow_tree(self, randQ):
                plt.ion()
                plt.show()
                self.grow_to_randq(randQ)
                if self.done == True:
                        self.draw_path()





                
