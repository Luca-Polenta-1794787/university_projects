import gym
from gym import spaces
import numpy as np
import dataclasses
from typing import Dict, List, Optional, Tuple

class PacManGameEnv(gym.Env):
    _rows: int
    _cols: int
    _states: np.array
    _rewards: np.array
    _remaining_balls: int
    _total_reward: float
    _max_reward: float
    _action_semantics: List[str]
    _actions: Dict[str, np.array]
    _init_state: Tuple[int, int]
    _current_state: Tuple[int, int]
    _ghosts: List[Tuple[int, int]]
    _walls: List[Tuple[int, int]]
    _saving_step_cost: float
    
    def __init__(self, 
                 step_cost: float,
                 ghosts: List[Tuple[int, int]],
                 obstacles: Optional[List[Tuple[int, int]]] = None) -> None:
        
        # I remove duplicates to avoid possible future errors
        ghosts = list(dict.fromkeys(ghosts))
        if obstacles is not None:
            obstacles = list(dict.fromkeys(obstacles))
        
        # Set general parameters
        self._saving_step_cost=step_cost
        self._rows = 8
        self._cols = 8
        self._total_reward = 0.0
        self._max_reward = 0.0
        self._remaining_balls = 0
        self._ghosts = ghosts
        self._walls = [] if obstacles is None else obstacles
        self._states = np.zeros((self._rows, self._cols))
        self._init_state = (1, 1)
        self._current_state = self._init_state
        self._action_semantics = ['down', 'left', 'right', 'up']
        self.action_space = spaces.Discrete(len(self._action_semantics))
        self._actions = np.array([[1, 0], [0, -1], [0, 1], [-1, 0]])
        
        # I set all the ghosts
        for r, c in self._ghosts:
            self._states[r, c] = -1
        
        # I add the obstacles of the horizontal wall above and below:
        for i in range(self._cols):
            self._walls.append((i, 0))
            self._walls.append((i, self._cols-1))
        # I add the left and right vertical wall obstacles:
        for i in range(self._rows):
            self._walls.append((0, i))
            self._walls.append((self._rows-1, i))
        # If the user has inserted some perimeter wall manually, I remove the duplicate:
        self._walls = list(dict.fromkeys(self._walls))
        # I set all the obstacles
        for r, c in self._walls:
            self._states[r, c] = 1   
        
        # I generate the reward table as empty and then fill it up if there are not walls and it is not the initial position
        self._rewards = np.zeros((self._rows, self._cols))
        self._rewards[self._init_state] = 0.0
        for r in range(self._rows):
            for c in range(self._rows):
                if (r, c) in self._ghosts:
                    self._rewards[r, c] = -10.0 * self._saving_step_cost
                elif (r, c) not in self._walls and (r,c) != self._init_state: 
                    self._rewards[r, c] = self._saving_step_cost
                    # When a reward is set, I add it to the max rewards that pacman can get and I increase the num of balls
                    self._max_reward += self._rewards[r, c]
                    self._remaining_balls += 1
                
    def reset(self):
        self._current_state = self._init_state        
        self._total_reward = 0.0
        self._max_reward = 0.0
        self._remaining_balls = 0
        self._rewards = np.ones((self._rows, self._cols))
        self._rewards[self._init_state] = 0.0
        for r in range(self._rows):
            for c in range(self._rows):
                if (r, c) in self._ghosts:
                    self._rewards[r, c] = -10.0 * self._saving_step_cost
                elif (r, c) not in self._walls and (r,c) != self._init_state:
                    self._rewards[r, c] = self._saving_step_cost
                    self._max_reward += self._rewards[r, c]
                    self._remaining_balls += 1
    
    def _transition(self, state: Tuple[int, int], a: np.array) -> Tuple[int, int]:
        n_actions = len(self._actions)
        a = self._actions[a + n_actions if a < 0 else a % n_actions]
        new_r = max(0, min(self._states.shape[0] - 1, state[0] + a[0]))
        new_c = max(0, min(self._states.shape[1] - 1, state[1] + a[1]))
        # If there is a wall I don't move, otherwise I go there
        return (new_r, new_c) if self._states[new_r, new_c] != 1. else state
        
    def step(self, action):
        actionName = self._action_semantics[action]
        
        prev_state = self._current_state
        self._current_state = self._transition(self._current_state, action)
        newReward = self.reward
        newTermination = self.termination
        checkKilledByGhost = self.checkTouchGhost
        
        # I write some info to print in the main that call this env
        info="\nNext selected action: "+actionName+"\nReward obtained from this action: "+str(newReward)+"\n"
        if (checkKilledByGhost==True):
            # If Pacman hits a ghost, it returns to the starting position and I returns a message of that event
            self._current_state=self._init_state
            info+="You have been eaten! Start over without resecting the map\n"
        
        return self._states, newReward, newTermination, info
    
    def render(self, mode='human') -> None:
        grid = np.array(self._states, dtype=str)
        r, c = self._current_state
        grid[r, c] = 'Pac'
        
        for i in range(len(grid)):
            for j in range(len(grid[i])):
                if(grid[i, j] == "1.0"):
                    grid[i, j] = "Wal"
        
        for (r, c) in self._ghosts:
            # If there is no wall, I put the ghost
            if 'Wal' not in grid[r, c]:
                grid[r, c] = 'Gho'
        
        print(grid)

    def ritValReward(self) -> (float, float):
        return self._total_reward, self._max_reward
    
    def descrEnv(self) -> str:
        return "The environment grid is "+str(self._rows)+"x"+str(self._rows)+" in size and it is composed as follows:\n"
    
    @property
    def actions(self) -> List[str]:
        return self._action_semantics
    
    @property
    def current_state(self) -> Tuple[int, int]:
        return self._current_state
    
    @property
    def reward(self) -> float:
        r, c = self._current_state
        # If I am in a state with a positive reward, I reset it because if Pacman returns there he will no longer find the ball to eat
        if(self._rewards[r, c]>0.0):
            temp = self._rewards[r, c]
            self._total_reward += temp
            self._rewards[r, c] = 0.0
            self._remaining_balls -= 1
            return temp
        else:
            self._total_reward+=self._rewards[r, c]
            return self._rewards[r, c]
    
    @property
    def checkTouchGhost(self) -> bool:
        return self._current_state in self._ghosts
    
    @property
    def termination(self) -> bool:
        return self._remaining_balls == 0