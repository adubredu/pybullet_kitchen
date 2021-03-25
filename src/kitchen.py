import os, sys 
import time
import pybullet as p
import pybullet_data  
import numpy as np 



class Kitchen:
    def __init__(self): 
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        p.setAdditionalSearchPath(model_path+'/models') 
        kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
        p.setGravity(0, 0, -9.81)  
        self.floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
        self.kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
        self.table = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)  
        self.drawer_to_joint_id = {1: 18, 
                                   2: 22, 
                                   3: 27, 
                                   4: 31,
                                   5: 37, 
                                   6: 40, 
                                   7: 48, 
                                   8: 53, 
                                   9: 56, 
                                   10: 58, 
                                   11: 14}
        self.drawer_to_joint_limits = {1: (0, 1.57), 
                                       2: (-1.57, 0), 
                                       3: (-1.57, 0), 
                                       4: (0, 1.57),
                                       5: (0.0, 0.4), 
                                       6: (0.0, 0.4), 
                                       7: (0, 1.57), 
                                       8: (-1.57, 0), 
                                       9: (0.0, 0.4), 
                                       10: (0.0, 0.4), 
                                       11: (0, 1.57)}
        
            
    def open_drawer(self, drawer_id):
        joint_id = self.drawer_to_joint_id[drawer_id]
        open_angle = self.drawer_to_joint_limits[drawer_id][1]
        p.setJointMotorControl2(bodyIndex=self.kitchen, jointIndex=joint_id, controlMode=p.POSITION_CONTROL, targetPosition=open_angle, maxVelocity=0.5) 
        p.stepSimulation()


    def close_drawer(self, drawer_id):
        joint_id = self.drawer_to_joint_id[drawer_id]
        close_angle = self.drawer_to_joint_limits[drawer_id][0]
        p.setJointMotorControl2(bodyIndex=self.kitchen, jointIndex=joint_id, controlMode=p.POSITION_CONTROL, targetPosition=close_angle, maxVelocity=0.5) 
        p.stepSimulation()


### Example usage
if __name__ == '__main__':
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) 

    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(enableConeFriction=0)

    #initialize kitchen object
    kitchen = Kitchen()
    time.sleep(30)
    #open all drawers
    for i in range(10):
        drawer_id = i+1
        kitchen.open_drawer(drawer_id)
        time.sleep(1)

    time.sleep(10)

    #close all drawers
    for i in range(10):
        drawer_id = i+1
        kitchen.close_drawer(drawer_id)
        time.sleep(1)

    time.sleep(10)




