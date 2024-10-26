# coding: utf-8
#
# Copyright 2021 The Technical University of Denmark
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#    http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from __future__ import annotations
import sys
import itertools
import numpy as np
from utils import pos_add, pos_sub, APPROX_INFINITY
from collections import deque, defaultdict

import domains.hospital.state as h_state
import domains.hospital.goal_description as h_goal_description
import domains.hospital.level as h_level


class HospitalZeroHeuristic: 
    def __init__(self):
        pass

    def preprocess(self, level: h_level.HospitalLevel):
        pass

    def __call__(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        return 0
    
    
class HospitalGoalCountHeuristics:

    # Write your own implementation of the goal count heuristics here. Remember that the goal count heuristics is simply the number of goals that are not satisfied in the current state. 

    def __init__(self):
        pass

    def preprocess(self, level: h_level.HospitalLevel):
        # This function will be called a single time prior to the search allowing us to preprocess the level such as
        # pre-computing lookup tables or other acceleration structures

        # Your code here (if necessary!)
        pass

    def __call__(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        # This functions makes the heuristic class callable. When called, the heuristic class should return the 
        # heuristic function value computed on the given state
        count = 0
        for (goal_position, goal_char, is_positive_literal) in h_goal_description.goals:
            char = state.object_at(goal_position)
            if is_positive_literal and goal_char != char:
                count = count + 1
            elif not is_positive_literal and goal_char == char:
                count = count + 1
        return count


class HospitalAdvancedHeuristics:

    # Write your own implementation of the advanced heuristics here.

    def __init__(self):
        self.distances = None
        self.goal_chars = None
        self.agent_chars = None
        self.goals = None
    def __call__(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        # This functions makes the heuristic class callable. When called, the heuristic class should return the 
        # heuristic function value computed on the given state
        count = 0
        for (goal_position, goal_char, is_positive_literal) in h_goal_description.agent_goals:
            for(agent_position, agent_char) in h_state.agent_positions:
                if(agent_char == goal_char):
                    xdiff = abs(agent_position[0] - goal_position[0])
                    ydiff = abs(agent_position[1] - goal_position[1])
                    count += xdiff + ydiff
                    break
        for (goal_position, goal_char, is_positive_literal) in h_goal_description.box_goals:
            for(box_position, box_char) in h_state.agent_positions:
                if(box_char == goal_char):
                    xdiff = abs(box_position[0] - goal_position[0])
                    ydiff = abs(box_position[1] - goal_position[1])
                    count += xdiff + ydiff
                    break
        return count
        

    
