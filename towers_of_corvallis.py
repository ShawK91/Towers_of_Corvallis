import time, numpy as np
#
# class Nodes: #Class wrapper to make lists hashable
#     def __init__(self, node):
#         self.node = node


num_discs = 7
nmax = 100000
is_admissible = 1
beamwidth = float("inf")

#Save filename
if is_admissible: save_filename = 'Logs/Admissible/' + str(beamwidth) + '_beam_' + str(num_discs) + '_discs.txt'
else: save_filename = 'Logs/Inadmissible/' + str(beamwidth) + '_beam_' + str(num_discs) + '_discs.txt'




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



class Beam_Search:
    def __init__(self, beamwidth):
        self.beamwidth = beamwidth

    def compute_heuristic(self, node, goal):
        time_start = time.time()
        if is_admissible: #Admissible heuristic
            cost = 0.0
            first_peg = node[0]
            for i, disc in enumerate(first_peg):
                for num in first_peg[i+1:]:
                    if num > disc: cost += 2

            #len
            cost += len(node[1]) + 2*len(node[2])

            return cost, time.time() - time_start



        else: #Non-admissible heuristic
            cost = 0.0
            first_peg = node[0]
            for i, disc in enumerate(first_peg):
                for num in first_peg[i+1:]:
                    if num > disc: cost += 2

            second_peg = node[1]
            for i, disc in enumerate(second_peg):
                for num in second_peg[i + 1:]:
                    if num > disc: cost += 1

            third_peg = node[2]
            for i, disc in enumerate(third_peg):
                for num in third_peg[i + 1:]:
                    if num > disc: cost += 1

            #len
            cost += len(node[1]) + 2*len(node[2])

            return cost, time.time() - time_start




            return 0, time.time() - time_start


    def start_search(self, nmax, start, goal, task):
        heuristic_time = 0.0

        #Containers
        open_set = list()
        closed_set = set()
        back_node = dict()
        g_score = dict()
        h_score = dict()

        #Init
        g_score[start] = 0
        h, time_cost = self.compute_heuristic(start, goal)
        h_score[start] = h; heuristic_time += time_cost
        open_set.append((g_score[start] + h_score[start], start))

        for step in range(nmax):
            open_set.sort()
            if len(open_set) > self.beamwidth: open_set.pop(-1) #Throw away the last one
            if len(open_set) == 0: break
            current_f, node = open_set.pop(0)

            if goal == node:
                #print 'GOAL FOUND'
                # Backpropagate path
                soln = [node]
                while node != start:
                    node = back_node[node]
                    soln.append(node)
                soln.reverse()
                return soln, step+1, heuristic_time


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

                    h, time_cost = self.compute_heuristic(nbr, goal)
                    h_score[nbr] = h; heuristic_time += time_cost
                    open_set.append((g_score[nbr] + h_score[nbr], nbr))

            #print 'Step', step, 'Node', node
        return None, step, heuristic_time

def data_io(filename):
    starts = []; raw_nums = []
    for i, line in enumerate(open(filename)):
        line = line.strip('\n')
        if i == 0 or len(line) == 0: continue

        num = []
        for char in line: num.append(int(char))
        starts.append(((tuple(num)), (), ()))
        raw_nums.append(float(line))
    return starts, raw_nums

def get_goal(num_discs):
    state = [i for i in range(num_discs)]
    state.reverse()
    return ((tuple(state), (), ()))



all_starts, raw_nums = data_io('perms-' + str(num_discs) + '.txt')
tracker = [] #[start node, Is_Success, path_length, nodes_expanded, cpu_time, cpu_time_heuristic]
for start, start_raw in zip(all_starts, raw_nums):
    start = start; goal = get_goal(num_discs)
    task = Tower_Of_Corvallis(start)
    agent = Beam_Search(beamwidth)
    start_time = time.time()
    soln, steps, heuristic_time = agent.start_search(nmax, start, goal, task)
    time_elapsed = time.time()-start_time
    if soln: #Goal found
        print 'Goal found in', steps, 'steps. Soln = ', soln
        tracker.append([start_raw, 1, len(soln), steps, time_elapsed*1000000, heuristic_time*1000000])
    else:
        print 'Failed'
        tracker.append([start_raw, 0, 0,steps, time_elapsed*1000000, heuristic_time*1000000])

print tracker
np.savetxt(save_filename, np.array(tracker), fmt='%.0f',  delimiter=',')

# #Save Coarse Stats
# tracker = np.array(tracker)
# average = np.average(tracker)
# stds = np.std(tracker)
# final =


# handle = open('try.txt', 'w+')
# for row in tracker:
#     for item in handle:
#         handle.write("%s\n" % item)






















