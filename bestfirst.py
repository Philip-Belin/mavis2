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
import heapq
import itertools

import domains.hospital.goal_description as h_goal_description
import domains.hospital.state as h_state
import domains.hospital.heuristics as h_heuristics


# Here we define a priority queue which allows the priority of elements to be updated in constant time.
# This priority queue is therefore suitable for usage as the frontier in a best-first search.

class PriorityQueue:

    def __init__(self):
        self.heap = []
        self.entry_finder = {}
        self.counter = itertools.count()

    def add(self, element: h_state.HospitalState, priority: int):
        # The elements are stored in a queue as a triplet (priority, count, element)
        # Python sorts tuples by comparing the first position and if these tie progressing to the next position until
        # it either finds a position in the tuples where they differ or all positions has been compared (in which case
        # the two tuples are identical. By storing the priority in the first position, we ensures that the element
        # with the lowest priority will be placed in the front of the queue.
        # The second 'count' component is used to break ties when two elements have the same priority.
        # By using a monotonically increasing counter we ensure that:
        # 1) In case of identical priorities elements are ordered by their insertion order (i.e. LIFO behaviour).
        # 2) Since all elements has a unique count, we can never have a tie and therefore never need to compare
        #    the element itself.
        # The entry finder stores a reference to all elements which ensures that we later can access the entry.
        count = next(self.counter)
        entry = [priority, -count, element]
        heapq.heappush(self.heap, entry)
        self.entry_finder[element] = entry

    def change_priority(self, element: h_state.HospitalState, new_priority: int):
        # We cannot change the priority of an element already in the heap as that would break the heap invariant.
        # Instead we invalidate the current entry by replacing the element with None and then inserting the element
        # again with the new priority.
        entry = self.entry_finder.pop(element)
        entry[2] = None
        # Add new entry with new priority
        self.add(element, new_priority)

    def pop(self) -> h_state.HospitalState:
        # Since some of the elements in the queue might have been invalidated by the 'change_priority' method, we need
        # to keep taking elements from the queue until we find a valid entry.
        while True:
            entry = heapq.heappop(self.heap)
            if entry[2] is not None:
                break
        state = entry[2]
        self.entry_finder.pop(state)
        return state

    def clear(self):
        self.heap.clear()
        self.entry_finder.clear()
        self.counter = itertools.count()

    def size(self) -> int:
        return len(self.entry_finder)

    def get_priority(self, element) -> int:
        entry = self.entry_finder.get(element)
        if entry is None:
            return None
        return entry[0]


class FrontierBestFirst:

    def __init__(self):
        self.goal_description = None
        self.priority_queue = PriorityQueue()
        
    def prepare(self, goal_description: h_goal_description.HospitalGoalDescription):
        self.goal_description = goal_description
        # Prepare is called at the beginning of a search and since we will sometimes reuse frontiers for multiple
        # searches, prepares must ensure that state is cleared.
        self.priority_queue.clear()

    def g(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        return state.path_cost
    
    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        return 0
    
    def f(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        return FrontierBestFirst.h(self,state, goal_description)+FrontierBestFirst.g(self,state, goal_description)
    
    def add(self, state: h_state.HospitalState):
        if not FrontierBestFirst.contains(self,state):
           self.priority_queue.add(state,FrontierBestFirst.f(self,state,self.goal_description))
        elif self.priority_queue.get_priority(self,state)>FrontierBestFirst.f(self,state,self.goal_description):
            self.priority_queue.change_priority(self,state,FrontierBestFirst.f(self,state,self.goal_description))
        return 

    def pop(self) -> h_state.HospitalState:
        return self.priority_queue.pop()

    def is_empty(self) -> bool:
        return self.priority_queue.size() == 0

    def size(self) -> int:
        return self.priority_queue.size()
    
    def contains(self, state: h_state.HospitalState) -> bool:
        return not self.priority_queue.get_priority(state) is None


# The FrontierAStar and FrontierGreedy classes extend the FrontierBestFirst class, that is, they are
# exact copies of the above class but where the methods for computing the cost 'g' and the heuristic
# function 'h' are modified.

class FrontierAStar(FrontierBestFirst):

    def __init__(self, heuristic):
        super().__init__()
        self.heuristic = heuristic
        
    def g(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        return len(state.extract_plan(state))
    
    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        return h_heuristics.HospitalZeroHeuristic(state)
        

class FrontierGreedy(FrontierBestFirst):

    def __init__(self, heuristic):
        super().__init__()
        self.heuristic = heuristic

    def g(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:  
        return 0
    
    def h(self, state: h_state.HospitalState, goal_description: h_goal_description.HospitalGoalDescription) -> int:
        return h_heuristics.HospitalAdvancedHeuristics(state)
