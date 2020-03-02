#!/usr/bin/env python
# -*- coding: utf-8 -*-

from abc import ABC, abstractmethod
import numpy as np
from gym_tracked.envs import rotations as rt
'''
There will be added subscription to the stair parameter 
extraction node and the <self.stair will be provided and memorized>
'''

class Criterion(ABC):
    @abstractmethod
    def calculate(self, model, stair):
        pass

class NESM(Criterion):

    def __init__(self, robot, stair):
        self.stair = stair
        self.robot = robot
        alpha = stair['angle']
        phi = np.arctan(robot['height']/robot['length'])
        # hardcoded, it is the distance from the contact point to the robot center
        # when the robot inclined its flippers to 45 degrees and the center has the maximum height over the surface
        self.max_height = 0.296

    def update(self, **stair):
        self.stair = stair

    def distance(self, model):
        if model.position.x > 0.6:
            base_com = 0.05
            Mx, My, Mz = model.position.x, model.position.y, model.position.z
            dist = abs(self.stair['A']*Mx + self.stair['B']*My + self.stair['C']*Mz + self.stair['D'])
            dist /= (self.stair['A']**2 + self.stair['B']**2+ self.stair['C']**2)**0.5
            print('\n', 'DISTANCE TO THE STAIR', dist, '\n')
            return dist
        else:
            return -1        

    def calculate(self, robot_pose, robot_conf, stair):
        """
            Calculates assesment of stability considering ESM
            Args:
                model: position and orientation of the robot 
                in the climbing stair frame.
                
                stair: parameters of stair and estimation of 
                its surface eq. parameters in the same frame.
            Return:
                float: Stability assesment from 0 - stable to 1 - unstable.
        """
        d = self.distance(robot_pose)
        if d != -1:
            H = d - self.robot['height']*0.5
            H /= (self.max_height - self.robot['height']*0.5)
            return H if H > 0 else 0
        else:
            return 0


class CoMProjection(Criterion):
    def calculate_stability(self, model, stair):
        return 0.

class SupportPolygon(Criterion):
    def __init__(self, robot):
        # Length of flipper
        self.PI = 3.14
        self.fl = robot['flipper']
        # Length of robot's base
        self.bl= robot['length']
        self.max_projection = 2 * self.fl + self.bl

    def update(self, **stair):
        pass

    '''
    def caseI(self, bl, fl, a, b):
        return bl + fl*np.cos(a) + fl*np.cos(b)

    def caseII(self, bl, fl, a, b):
        d = (bl**2 + fl**2 -  2 * bl * fl * np.cos(self.PI - b))**.5
        phi = np.arccos((bl**2 + d**2 - fl**2) / (2 * bl * d))
        return (d**2+fl**2-2*d*fl*np.cos(self.PI - a - phi))**.5

    def caseIII(self, bl, fl, a, b):
        d = (bl**2 + fl**2 -  2 * bl * fl * np.cos(self.PI-b))**.5
        phi = np.arccos((bl**2 + d**2 - fl**2) / (2 * bl * d))
        return d + np.cos(a - phi) * fl
    '''

    def calculate(self, model, rconf, stair):
        # Unsresolved issue, when the robot is parallel on the stair
        # model.orientation.y is twice less than stair['slope']
        model_angle = abs(model.orientation.y)*2
        model_projection = np.cos(model_angle - stair['slope'])
        current_length = (np.cos(rconf['front'])+np.cos(rconf['rear']))*self.fl+self.bl
        current_proj = current_length * model_projection
        total_length = 2 * self.fl + self.bl
        if model.position.x > stair['first_edge'] and model.position.x < stair['last_edge']:
            return current_proj / total_length
        else:
            return 1.
        '''
            if rconf['front'] == 0:
                asign = 1
            else:
                asign = rconf['front']/abs(rconf['front'])
            a = abs(rconf['front'])

            if rconf['rear'] == 0:
                bsign = 1
            else:
                bsign = rconf['rear']/abs(rconf['rear'])
            b = abs(rconf['rear'])

            if asign >= 0 and bsign >= 0:
                return self.caseI(self.bl, self.fl, a, b)  / self.max_projection
            elif asign <= 0 and bsign <= 0:
                return self.caseII(self.bl, self.fl, a, b) / self.max_projection
            elif asign >= 0 and bsign <= 0:
                return self.caseIII(self.bl, self.fl, a, b) / self.max_projection
            elif asign <= 0 and bsign >= 0:
                return self.caseIII(self.bl, self.fl, b, a) / self.max_projection
            else:
                raise RuntimeError('Not in any of cases, termination')
        else:
            return 1.
        '''




class Criteria:

    def __init__(self, **kwargs):
        self.stair = self.build_stair(kwargs)
        self.criteria = {}
        for arg in kwargs['criteria']:
            if arg == 'nesm':
                self.criteria['nesm'] = NESM(kwargs['robot'], self.stair)
            if arg == 'com':
                self.criteria.append(CoMProjection())       
            if arg == 'projection':
                self.criteria['projection'] = SupportPolygon(kwargs['robot'])  

    def update_criteria(self, **kwargs):
        self.stair = self.build_stair(kwargs)
        for k, v in self.criteria.items():
            v.update(**self.stair)

    def build_stair(self, stair):
        N = stair['n_steps']
        H = stair['h_step']
        L = stair['tread']
        S = stair['x_initial']
        A = np.array([[S+(N-1)*L, 0., N*H], [S, 0.2, H], [S, -0.2, H]])
        B = np.array([-1, -1, -1])
        X = np.linalg.inv(A).dot(B)
        return {
            'first_edge': S,
            'last_edge': S + (N-1)*L,
            'n_steps': N,
            'h_steps': H,
            'giron': L,
            'slope': np.arctan(H / L),
            'angle': np.arctan(N*H/((N-1)*L)),
            'A': X[0],
            'B': X[1],
            'C': X[2],
            'D': 1
        }

    def calculate(self, model, rconf):
        '''
        Calculate assesments for added criteria.
        '''
        result = []
        for k in self.criteria.keys():
            result.append(self.criteria[k].calculate(model, rconf, self.stair))
        return result



    
