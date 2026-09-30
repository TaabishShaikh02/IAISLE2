import time
import heapq
from collections import deque

GOAL = (1,2,3,4,5,6,7,8,0)

def neighbors(state):
    idx = state.index(0)
    r, c = divmod(idx, 3)
    moves = []
    for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
        nr, nc = r+dr, c+dc
        if 0 <= nr < 3 and 0 <= nc < 3:
            nidx = nr*3+nc
            new_state = list(state)
            new_state[idx], new_state[nidx] = new_state[nidx], new_state[idx]
            moves.append(tuple(new_state))
    return moves

def bfs(start):
    frontier = deque([start])
    visited = {start}
    nodes_expanded = 0
    parent = {start: None}
    while frontier:
        state = frontier.popleft()
        nodes_expanded += 1
        if state == GOAL:
            return nodes_expanded, parent, state
        for n in neighbors(state):
            if n not in visited:
                visited.add(n)
                parent[n] = state
                frontier.append(n)
    return nodes_expanded, parent, None

def manhattan(state):
    dist = 0
    for i, v in enumerate(state):
        if v == 0:
            continue
        gr, gc = divmod(v-1, 3) if v != 0 else (2,2)
        # goal position of value v is index v-1 for v in 1..8, 0 is at index 8
        goal_idx = v - 1 if v != 0 else 8
        gr, gc = divmod(goal_idx, 3)
        cr, cc = divmod(i, 3)
        dist += abs(gr-cr) + abs(gc-cc)
    return dist

def astar(start):
    counter = 0
    frontier = [(manhattan(start), counter, start, 0)]
    visited = {start: 0}
    nodes_expanded = 0
    parent = {start: None}
    while frontier:
        f, _, state, g = heapq.heappop(frontier)
        if state == GOAL:
            return nodes_expanded, parent, state
        nodes_expanded += 1
        for n in neighbors(state):
            ng = g + 1
            if n not in visited or ng < visited[n]:
                visited[n] = ng
                parent[n] = state
                counter += 1
                heapq.heappush(frontier, (ng + manhattan(n), counter, n, ng))
    return nodes_expanded, parent, None

# Scrambled start state, 20 random moves away from GOAL (solvable, guaranteed reachable)
START = (4,1,0,2,6,3,7,5,8)

def run_trials(fn, start, trials=5):
    times = []
    nodes = None
    for _ in range(trials):
        t0 = time.perf_counter()
        nodes, _, goal_found = fn(start)
        t1 = time.perf_counter()
        times.append((t1-t0)*1000)  # ms
    avg_time = sum(times)/len(times)
    return avg_time, nodes, times

if __name__ == "__main__":
    print("Start state:", START)
    print()
    bfs_time, bfs_nodes, bfs_times = run_trials(bfs, START, trials=5)
    astar_time, astar_nodes, astar_times = run_trials(astar, START, trials=5)

    print("BFS individual run times (ms):", [round(t,4) for t in bfs_times])
    print("BFS avg time (ms):", round(bfs_time,4))
    print("BFS min/max time (ms):", round(min(bfs_times),4), round(max(bfs_times),4))
    print("BFS nodes expanded:", bfs_nodes)
    print()
    print("A* individual run times (ms):", [round(t,4) for t in astar_times])
    print("A* avg time (ms):", round(astar_time,4))
    print("A* min/max time (ms):", round(min(astar_times),4), round(max(astar_times),4))
    print("A* nodes expanded:", astar_nodes)
