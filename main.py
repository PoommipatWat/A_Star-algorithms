
class TreeNode:
    def __init__(self) -> None:
        self.tree = {}
    
    def insert(self, node : str, heuristic : float, link : list):
        self.tree[node] = {'heuristic': heuristic, 'link': {link[i][0]: link[i][1] for i in range(len(link))}}

    def get(self):
        return self.tree
    
class AStar:
    def __init__(self, tree : dict) -> None:
        self.tree = tree
        self.queue = []
        self.min_cost = None

    def sort_queue(self):
        self.queue = sorted(self.queue, key = lambda x: list(x.values())[0])

    def clear_queue(self):
        if self.min_cost != None:
            self.queue = [q for q in self.queue if list(q.values())[0] <= self.min_cost]

    def search(self, start : str, goal : str, show_queue = False):
        for node in self.tree[start]['link']:
            calculated = self.tree[node]['heuristic'] + self.tree[start]['link'][node]
            self.queue.append({f'{start}-{node}': calculated})
            if goal == node:
                if self.min_cost == None or calculated < self.min_cost:
                    self.min_cost = calculated
        self.clear_queue()
        self.sort_queue()
        print(self.queue) if show_queue else None
        
        while len(self.queue) > 1:
            current_node = list(self.queue[0].keys())[0]
            last_current_node = list(self.queue[0].keys())[0].split('-')[-1]
            last_cost_node = list(self.queue[0].values())[0]
            self.queue.pop(0)
            for node in self.tree[last_current_node]['link']:
                if node not in current_node:
                    calculated = self.tree[node]['heuristic'] + self.tree[last_current_node]['link'][node] + last_cost_node - self.tree[last_current_node]['heuristic']
                    self.queue.append({f'{current_node}-{node}': calculated})
                    if goal == node:
                        if self.min_cost == None or calculated < self.min_cost:
                            self.min_cost = calculated
            self.clear_queue()
            self.sort_queue()
            print(self.queue) if show_queue else None
        return list(self.queue[0].keys())[0], list(self.queue[0].values())[0]

trees = TreeNode()
trees.insert('S', 12, [['A', 10], ['B', 8]])
trees.insert('A', 5, [['S', 10], ['C', 2], ['G', 10]])
trees.insert('B', 5, [['S', 8], ['G', 16], ['D', 8]])
trees.insert('C', 5, [['A', 2], ['G', 9], ['E', 3]])
trees.insert('D', 2, [['B', 8], ['G', 3], ['H', 1]])
trees.insert('E', 2, [['C', 3], ['G', 2]])
trees.insert('G', 0, [['A', 10], ['B', 16], ['C', 9], ['D', 3], ['E', 2]])
trees.insert('H', 1, [['D', 1], ['F', 1]])
trees.insert('F', 1, [['H', 1]])

astar = AStar(trees.get())
path_planning, min_cost = astar.search('S', 'G')

print(f'Path Planning: {path_planning}, Min Cost: {min_cost}')

trees = TreeNode()
trees.insert('A', 16, [['B', 5], ['C', 5]])
trees.insert('B', 17, [['A', 5], ['C', 4], ['D', 3]])
trees.insert('C', 13, [['A', 5], ['B', 4], ['D', 7], ['H', 8], ['E', 7]])
trees.insert('D', 16, [['B', 3], ['C', 7], ['H', 11], ['K', 16], ['L', 13], ['M', 14]])
trees.insert('E', 16, [['C', 7], ['H', 5], ['F', 4]])
trees.insert('F', 20, [['E', 4], ['G', 9]])
trees.insert('G', 17, [['F', 9], ['N', 12]])
trees.insert('H', 11, [['C', 8], ['D', 11], ['E', 5], ['I', 3]])
trees.insert('I', 10, [['H', 3], ['J', 4]])
trees.insert('J', 8, [['I', 4], ['P', 8], ['N', 3]])
trees.insert('K', 4, [['D', 16], ['L', 5], ['P', 4], ['N', 7]])
trees.insert('L', 7, [['D', 13], ['K', 5], ['M', 9], ['O', 4]])
trees.insert('M', 10, [['D', 14], ['L', 9], ['O', 5]])
trees.insert('N', 7, [['G', 12], ['J', 3], ['K', 7], ['P', 7]])
trees.insert('O', 5, [['L', 4], ['M', 5]])
trees.insert('P', 0, [['J', 8], ['K', 4], ['N', 7]])

astar = AStar(trees.get())
path_planning, min_cost = astar.search('A', 'P')
print(f'Path Planning: {path_planning}, Min Cost: {min_cost}')
