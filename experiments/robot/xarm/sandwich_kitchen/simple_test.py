import cv2
import numpy as np

import wrs.basis.robot_math as rm
from wrs.robot_con.xarm_lite6.xarm_lite6_x import XArmLite6X
import wrs.visualization.panda.world as wd
import wrs.modeling.geometric_model as mgm

base = wd.World(cam_pos=[2, 0, 1], lookat_pos=[0, 0, 0])
mgm.gen_frame().attach_to(base)

if __name__ == "__main__":
    # joint configuration: joint values for 6 joints
    # end-effector 2: position [x,y,z] and orientation [rotation matrix 3x3]
    '''simulated robot'''
    # rbt = XArmLite6WG2(pos=np.array([0, 0, .005]), enable_cc=True)  # enable collision check
    # # robot go to a given joint configuration, rbt.rand_conf() can generate a random configuration
    # random_conf = rbt.rand_conf()
    # tgt_pos, tgt_rotmat = rbt.fk(random_conf)
    # calculated_jnt = rbt.ik(tgt_pos, tgt_rotmat)
    # rbt.goto_given_conf(random_conf)
    # rbt.gen_meshmodel(rgb=[0,0,1], alpha=0.5).attach_to(base)

    # # show each joints
    # joint1_rotate = random_conf + [0, np.pi/3, 0, 0, 0, 0]
    # rbt.goto_given_conf(joint1_rotate)
    # rbt.gen_meshmodel(rgb=[1,0,0], alpha=0.5).attach_to(base)
    # base.run() # also terminate the program
    
    '''real robot'''
    rbtx = XArmLite6X(ip='192.168.1.152', has_gripper=True)
    # print the current joint angles
    print(repr(rbtx.get_jnt_values()))
    # rbtx._gripper_x.close()
    pos, rotmat = rbtx.get_pose()
    rotq = rm.rotmat_to_euler(rotmat)
    eepose = np.hstack((pos, rotq))
    print(repr(pos))
    print(repr(rotmat))
    print('current joint values:',repr(rbtx.get_jnt_values()))
    # # move to a given joint configuration (home joint configuration)
    # rbtx.homeconf()
    # rbtx._gripper_x.close()