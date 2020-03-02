import os, sys, time
import logging
import numpy as np
import json, time
import learning_scripts.learning_openai as baselines
import learning_scripts.reps_garage_learning as garage
# from utils.drawer import Drawer
from utils.env_creator import Creator
# import learning_scripts.reps_garage_learning as garage

class Commander: 
    def __init__(self):
        pass

    def parse_plan(self, kind):
        with open(kind+".json") as f:
            data = json.load(f)
        return data

    def run(self):
        # create env_creator instance
        env_creator = Creator()
        # parse plan
        plan = self.parse_plan("plan")
        for k, v in plan.items():
            n = int(v["repeat"])
            for i in range(n):
                v["iteration"] = str(i)
                v["env_creator"] = env_creator
                if "baselines" in k:
                    # create learning_library instance (learner) passing env_creator inside
                    learner = baselines.LearningOpenAI(**v)
                    # call learner.train_model
                    learner.train_model(10000)
                    del learner
                elif "garage" in k:
                    learner = garage.LearningGarage(**v)
                    learner.train_model()
                    del learner


if __name__ =='__main__':
    Commander().run()
    # Commander().demonstrations()
