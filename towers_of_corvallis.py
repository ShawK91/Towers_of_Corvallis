import sys, os, math, numpy as np
#
# class Nodes: #Class wrapper to make lists hashable
#     def __init__(self, node):
#         self.node = node


num_discs = 7
nmax = 100000
is_admissible = 1


class Tower_Of_Corvallis:
    def __init__(self, initial_config):
        self.initial_config = initial_config

    def to_tuple(self, l): return (tuple(elm) for elm in l)

    def to_list(self, t): return [list(elm) for elm in t]

    def get_top_elm(self, lst):
        if len(lst) == 0: return []
        else: return [lst[0]]

    def get_nbrs(self, node):
        nbrs = []

        #First
        temp_node = self.to_list(node)
        nbrs.append((tuple(temp_node[0][1:]), tuple(self.get_top_elm(temp_node[0]) + temp_node[1]), tuple(temp_node[2])))

        #Second
        temp_node = self.to_list(node)
        nbrs.append((tuple(self.get_top_elm(temp_node[1]) + temp_node[0]), tuple(temp_node[1][1:]), tuple(temp_node[2])))

        #third
        temp_node = self.to_list(node)
        nbrs.append((tuple(temp_node[0]), tuple(temp_node[1][1:]), tuple(self.get_top_elm(temp_node[1]) + temp_node[2])))

        #Fourth
        temp_node = self.to_list(node)
        nbrs.append((tuple(temp_node[0]), tuple(self.get_top_elm(temp_node[2]) + temp_node[1]), tuple(temp_node[2][1:])))

        return nbrs



class Astar:
    def __init__(self):
        pass

    def compute_heuristic(self, node, goal):
        if is_admissible: #Admissible heuristic
            cost = 0.0
            first_peg = node[0]
            for i, disc in enumerate(first_peg):
                for num in first_peg[i+1:]:
                    if disc > num: cost += 1

            #len
            cost += len(node[1]) + len(node[2])

            return cost



        else: #Non-admissible heuristic
            return 0


    def start_search(self, nmax, start, goal, task):
        #Containers
        open_set = list()
        closed_set = set()
        back_node = dict()
        g_score = dict()
        h_score = dict()

        #Init
        g_score[start] = 0
        h_score[start] = self.compute_heuristic(start, goal)
        open_set.append((g_score[start] + h_score[start], start))

        for step in range(nmax):
            open_set.sort()
            current_f, node = open_set.pop(0)

            if goal == node:
                #print 'GOAL FOUND'
                # Backpropagate path
                soln = [node]
                while node != start:
                    node = back_node[node]
                    soln.append(node)
                soln.reverse()
                return soln, step+1


            closed_set.add(node)

            nbrs = task.get_nbrs(node)
            for nbr in nbrs: #For al nbrs
                if nbr in closed_set: continue #Ignore if in closed set

                if nbr in open_set: #If node already in open set
                    if (g_score[node] + 1) < g_score[nbr]: #If current path cost is better
                        g_score[nbr] = g_score[node] + 1
                        back_node[nbr] = node
                        open_set.remove(nbr); open_set.append((g_score[nbr] + h_score[nbr], nbr))

                else: #Found new node unexplored before
                    back_node[nbr] = node
                    g_score[nbr] = g_score[node] + 1
                    h_score[nbr] = self.compute_heuristic(nbr, goal)
                    open_set.append((g_score[nbr] + h_score[nbr], nbr))

            #print 'Step', step, 'Node', node
        return None, step


def data_io(filename):
    starts = []
    for i, line in enumerate(open(filename)):
        if i == 0: continue
        line = line.strip('\n')

        num = []
        for char in line: num.append(int(char))
        starts.append(((tuple(num)), (), ()))
    starts.pop(-1)
    return starts

def get_goal(num_discs):
    state = [i for i in range(num_discs)]
    state.reverse()
    return ((tuple(state), (), ()))



all_starts = data_io('perms-' + str(num_discs) + '.txt')
for start in all_starts:
    start = start; goal = get_goal(num_discs)
    task = Tower_Of_Corvallis(start)
    agent = Astar()
    soln, steps = agent.start_search(nmax, start, goal, task)
    if soln: #Goal found
        print 'Goal found in', steps, 'steps. Soln = ', soln
    else:
        print 'Failed'







def a_star(problem, start, goal):
    '''
    Finds a path from point start to point goal using the A* algorithm.
    '''

    # set up a clean slate for this pass of the algorithm.
    # The open set contains points at the perimeter: we know how to reach
    # these points from the start, but have not yet explored all their neighbors.
    open_set = set()

    # the open_queue contains the same points as the open_set, but associates them
    # with their f-score, and indeed is kept ordered by f-score. This lets
    # us quickly choose the most promising points in the open set to explore next.
    # It technically obviates the open_set but as an implementation detail it's
    # easier to store them separately and use the set to check for membership
    # and the queue to keep them sorted by f-score.
    open_queue = list()

    # The closed set contains points that we are finished with and won't visit
    # again. We know how to reach them from start but have already explored
    # all their neighbors.
    closed_set = set()

    # The came_from dict is a map from each point to one of it's neighbors.
    # You can think of it as a vector field flowing back to the start. If you
    # iteratively follow the
    came_from = dict()

    # the g-score is the currently best known cost to reach each point. It
    # is syncronized with the came_from vector field: if you followed it all
    # the way back to the start, the cost would be exactly value found in g-score
    # for that point. It isn't necessarily the best possible way to get to that
    # point, just the best way we've discovered so far.
    g_score = dict()

    # the h-score is the estimate for how far away from the goal this point
    # is, as estimated by the problem's heuristic function.
    h_score = dict()

    # f can be computed rather than stored.
    def f_score(point):
        return g_score[point] + h_score[point]

    # we can kick off the algorithm by placing only the start point in the open set.
    g_score[start] = 0
    h = problem.heuristic(start, goal)
    h_score[start] = h
    open_set.add(start)
    open_queue.append((f_score(start), start))
    problem.on_open(start, h, 0, h)

    # keep searching until we find the goal, or until all possible pathes have been exhausted.
    while open_set:
        open_queue.sort()
        next_f, point = open_queue.pop(0)
        open_set.remove(point)

        if problem.is_goal(point, goal):
            # reached goal, unwind path
            path = [point]
            while point != start:
                point = came_from[point]
                path.append(point)
            path.reverse()
            return path

        closed_set.add(point)
        problem.on_close(point)

        for neighbor in problem.neighbor_nodes(point):
            if not neighbor in closed_set:
                tentative_g_score = g_score[point] + problem.distance_between_neighbors(neighbor, point)

                if neighbor not in open_set:
                    # new territory to explore
                    came_from[neighbor] = point
                    g = tentative_g_score
                    h = problem.heuristic(neighbor, goal)
                    g_score[neighbor] = g
                    h_score[neighbor] = h
                    open_set.add(neighbor)
                    f = g + h
                    open_queue.append((f, neighbor))
                    problem.on_open(neighbor, f, g, h)

                else:
                    # reconnected to previously explored area
                    if tentative_g_score < g_score[neighbor]:
                        # but we found a better route than before!
                        came_from[neighbor] = point
                        g = tentative_g_score
                        g_score[neighbor] = g
                        h = problem.heuristic(neighbor, goal)
                        h_score[neighbor] = h
                        f = g + h

                        problem.on_update(neighbor, f, g, h)

    raise PathNotFound("no path from %s to %s." % (str(start), str(goal)))












