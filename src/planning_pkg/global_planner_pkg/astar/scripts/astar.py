import heapq


class Node():
	def __init__(self, pid, position, on_road, edges, parent=None):
		self.pid = pid
		self.position = position
		self.on_road = on_road
		self.edges = edges
		self.parent = parent

	def __eq__(self, other):
		return self.pid == other.pid

	def __hash__(self):
		return self.pid
	

def node_from_pid(node_map, pid):
	for n in node_map:
		if n.pid == pid:
			return n
	return None


def node_distsq(n1, n2):
	return (n1.position[0] - n2.position[0])**2 + (n1.position[1] - n2.position[1])**2


def retrace(n: Node):
	path = [n]
	while n.parent is not None:
		n = n.parent
		path.append(n)
	path.reverse()
	return path


def find_closest(as_map, position): # Use other one instead
    closest = None
    smallest = None
    for n in as_map:
        d = (n.position[0] - position[0])**2 + (n.position[1] - position[1])**2
        if smallest is None or d < smallest:
            smallest = d
            closest = n
    return closest


def astar(node_map, current, end):
	open_set = set()
	open_heap = [] # heap of tuple: (node's distance to end, node)
	visited_set = set()

	open_set.add(current)
	open_heap.append((0, current))
	while open_set:
		current = heapq.heappop(open_heap)[1] # get node of lowest distance in heap
		if current == end: # found the end, retrace steaps
			return retrace(current)
		open_set.remove(current)
		visited_set.add(current) # time to calculate the current node
		for neighbor_pid in current.edges:
			neighbor = node_from_pid(node_map, neighbor_pid)
			if neighbor not in visited_set: # if not already calculated
				neighbor.H = node_distsq(neighbor, end)
				if neighbor not in open_set: # if not already in open set
					open_set.add(neighbor)
					heapq.heappush(open_heap, (neighbor.H, neighbor))
				neighbor.parent = current
	return []
