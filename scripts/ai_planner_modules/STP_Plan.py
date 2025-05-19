import subprocess
from pathlib import Path
import rospy
import os

class STP_Plan(): # TODO move this to AI planner modules
    def __init__(self):
        self.ws_name = os.environ.get("ROS_WS_NAME", "group3_ws")
        self.home = str(Path.home())
        self.root = self.home + "/" + self.ws_name + "/src"
        self.domain = "PDDL_domain_v3.pddl"
        self.problem = "PDDL_problem_v3.pddl"


    def run_planner(self, plan=False):
        """
        Run the STP planner with the given domain and problem files.
        :param plan: If True, run the planner and save the plan to a file.
        :return: Plan
        """
        # copy domain and problem
        subprocess.run(["cp", f"{self.root}/assignment4_ttk4192/PDDL/{self.problem}", f"{self.root}/temporal-planning-main/temporal-planning/domains/ttk4192/problem"])
        subprocess.run(["cp", f"{self.root}/assignment4_ttk4192/PDDL/{self.domain}", f"{self.root}/temporal-planning-main/temporal-planning/domains/ttk4192/domain"])
        if plan:
            rospy.loginfo("Replanning, running the STP planner.")
            subprocess.run(f"""source {self.root}/temporal-planning-main/bin/activate && cd {self.root}/temporal-planning-main/temporal-planning && python2.7 bin/plan.py stp-2 ./domains/ttk4192/domain/{self.domain} ./domains/ttk4192/problem/{self.problem} && mv {self.root}/temporal-planning-main/temporal-planning/tmp_sas_plan.1 {self.root}/assignment4_ttk4192/scripts""", shell=True, executable="/bin/bash")
        else:
            rospy.loginfo("Not replanning, using the previous plan.")
    def parse_plan(self):
        plan = []
        filepath = self.home + "/" + self.ws_name +"/src/assignment4_ttk4192/scripts/tmp_sas_plan.1"

        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith(";"):
                    continue
                parts = line.split(":", 1)
                start_time = float(parts[0])
                action_part = parts[1].strip()
                action_str, duration_str = action_part.rsplit(")", 1)
                action = action_str.strip("() ").split(" ")
                duration = float(duration_str.strip("[] "))
                plan.append({"start_time": start_time, "action": action, "duration": duration })
        return plan