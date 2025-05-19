import datetime

def print_info(real_map: str) -> None:
    if real_map == "true":
        environment = "Real World"
    else:
        environment = "Simulation"
    print()
    print("************ TTK4192 - Assignment 4 **************************")
    print("AI planners: STP Planner")
    print("Path-finding: Enhanced RRT*")
    print(f"GNC Controller: Differential Flatness + PID")
    print("Manipulator: OpenMANIPULATOR-X")
    print(f"Environment: {environment}")
    print("Robot: Turtlebot3 waffle-pi")
    print("date: " + str(datetime.date.today()))
    print("**************************************************************")
    print("Press Enter to start ...")