graph = { 0:{1:6, 4:1},
          1:{2:5, 3:3, 4:2},
          2:{2:5, 3:5},
          3:{2:5, 4:1},
          4:{0:1, 1:2, 3:1},
        }



def djks(graph_cost, start, end):
	visited = []
	unvisited = graph.keys()
	distance = []
	path = [None]*len(graph.keys())
	
	for i in unvisited:
		distance.append(float('inf'))

	distance[start] = 0
	
	while len(unvisited):

		curr_node = None
		min_ = float('inf')
		
		for i in unvisited:
			if( distance[i] < min_):
				curr_node = i
		
		visited.append(curr_node)
		unvisited.remove(curr_node)
		adj = graph[curr_node].keys()
		
		index = 0
		for i in adj:
			if( distance[i] > ( distance[visited[-1]] + graph[curr_node][i]) ):
				distance[i] = distance[visited[-1]] + graph[curr_node][i]
				path[i] = visited[-1]

		index += 1
	return path


print("Starting djks")

path = djks(graph, 0, 2 )
print(path)
