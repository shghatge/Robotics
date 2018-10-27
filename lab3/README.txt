
The zipped folder contains the launch.launch file.
It also contains the pathPlan.py script file which has to be run to navigate the robot along the shortest path in Vgraph.

Please find the link to the youtuve video below.
https://www.youtube.com/watch?v=fhZ7lMqVHVQ

NOTE : We are publishing the markers to 'visualization_marker' topic.

These are the commands to execute the script.

1) Run 'roscore' in a terminal.
          roscore
2) Run Rviz in another terminal. Below is the command.
          roslaunch vgraph launch.launch
3) execute the python script with the below command.
          python pathPlan.py

Please find the details about the function inside the python file

load_obstacles() : reads the obstacles from the text file.
load_goal() : reads the goal co-ordinates from the text file.
pub_path() : publishes the shortest path between the starting and end point using markers.
publish_lines() : publishes all the edges in the VGraph using markers.
collision_detect() : detects the collision between two line segments.
djks() : Dijkstra's algorithm for finding the shortest distance to the nodes along with the path.
get_shortest_path() : returns the shortest path between the star and goal, with respect to the indices of the vertices.
get_path_in_points() : returns the path in terms of points and the order in whcih they have to traversed.
getDist(): returns the distance between two points which has to be travelled to move to that point.
getAngle() : the angle the robot has to turn to move towards a point from another.
translate_robot() : translates the robot by a given distance.
rotate_robot() : rotates the robot by a given angle.
move_robot() : moves the robot by rotating and translating to the points in the shortest path.

