from pathfinding_modules.HybridAstar import *

if __name__ == '__main__':
#    print(f"Between {PLANNING_WAYPOINTS[1]} and {PLANNING_WAYPOINTS[0]}")
#    main_hybrid_a(PLANNING_WAYPOINTS[1], PLANNING_WAYPOINTS[0], True)
    print("Executing hybrid A* algorithm")
    for i in range(len(PLANNING_WAYPOINTS)):
       for j in range(len(PLANNING_WAYPOINTS)):
            if i == j:
                continue
            start_pos = PLANNING_WAYPOINTS[i]
            end_pos = PLANNING_WAYPOINTS[j]
            print(f"{i} to {j}")
            try:
                main_hybrid_a(start_pos, end_pos,True)
            except KeyboardInterrupt:
                continue
            except ValueError:
                continue
