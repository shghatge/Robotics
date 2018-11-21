# Lab4 #
#### Usage ####

1. Need to have Matplotlib installed.
2. Go to the folder containing scripts Draw.py RRT.py etc
3.  Run following command 
```python Draw.py obstacles.txt goal.txt 20```
4. The third parameter is the step size
5. For bidirectional RRT run following command
```python Draw.py obstacles.txt goal.txt 20```
6. Close the plot window to exit program

#### Method ####
##### Single RRT #####
###### Draw.py ######
 - Main method calls growTree method on the RRT
###### RRT.py: ######
- GrowTree : This method has a loop for maximum 100000 iterations. Each iteration grows the tree in a random direction till non collision. If goal is reached calls DrawPth
- DrawPath : Draws path from goal to start node by calling parents
- grow_to_randq: Expands the tree till collision is detected or goal is reached in given direction
- plot_and_check_goal : plots new node and checks if it is goal
- collision_detect : detects the collision between two line segments.

##### Bi-Directional RRT #####
###### DrawBi.py: ######
a) Main method of DrawBi has a loop for 100000 interations. Each iteration grows one tree in one direction and sets the new node as the goal for other tree
###### RRTBi.py: ######
- GrowTree : Here this method only expands one tree till no collision in oner direction by calling grow_to_randQ. If goal is reached, calls draw_path function
- DrawPath : Here DrawPath draws the path from latest nodes of both trees to their respective goal by ploting the nodes in green color
- grow_to_randq: Expands the tree till collision is detected or goal is reached in given direction
- plot_and_check_goal : plots new node and checks if it is goal
- collision_detect : detects the collision between two line segments.


#### Video Link ####
Please find the below link to the video which demonstrates the functioning of the script.
https://www.youtube.com/watch?v=ElkyuS-5Azc
#### Others ####
We have biased the bidirectional RRT towards goal a bit i.e. for every 7 iterations it will try to move towards the goal
For Bidirectional RRT we are switching the trees and one of the tree grows in a random direction. The new node is the goal for the second tree.
 And the second tree grows towards a random node with a bias of once every 7 iteration towards the goal.