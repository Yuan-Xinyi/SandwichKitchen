import cv2
import numpy as np
import json
import time

from wrs.robot_con.xarm_lite6.xarm_lite6_x import XArmLite6X
import wrs.visualization.panda.world as wd
import wrs.modeling.geometric_model as mgm

base = wd.World(cam_pos=[2, 0, 1], lookat_pos=[0, 0, 0])
mgm.gen_frame().attach_to(base)

sandwich_recipe = {
    "Basic Sandwich": ["Bread1", "Lettuce1", "Ham1", "Bread2"],
    "Double Sandwich": ["Bread1", "Lettuce1", "Ham1", "Lettuce2",  "Ham2", "DoubleBread2"],
    "Veggie Sandwich": ["Bread1", "Lettuce1", "Lettuce2", "Bread2"],
    "Meat Sandwich": ["Bread1", "Ham1", "Ham2", "Bread2"],
    "Gluten-free Sandwich": ["GF,Lettuce1andHam1", "GF,Lettuce2"]
}


food_items_cfg = json.load(open("experiments/robot/xarm/sandwich_kitchen/food_items.json"))
print('[INFO] Loaded sandwich placement config.')

if __name__ == "__main__":    
    '''real robot'''
    rbtx = XArmLite6X(ip='192.168.1.152', has_gripper=True)
    init_jnt = [-2.171331,  0.448032,  1.690477, -3.188763, -1.261539,  1.098714]
    rbtx._gripper_x.open()
    rbtx.move_j(init_jnt)
    print('[INFO] Robot moved to initial joint configuration.')
    _, init_rotmat = rbtx.get_pose()
    print('current joint values:',repr(rbtx.get_jnt_values()))

    sandwich_order = "Gluten-free Sandwich"
    action_seq = sandwich_recipe[sandwich_order]
    print(f"[INFO] Preparing to make a {sandwich_order} with actions: {action_seq}")
    for item in action_seq:
        item_cfg = food_items_cfg[item]
        # move upward of the food item
        rbtx.move_p(pos=np.array(item_cfg["preparation_pos"]), rot=init_rotmat)
        rbtx._gripper_x.close()
        down_pos = np.array([item_cfg["preparation_pos"][0], item_cfg["preparation_pos"][1], 0.019])
        # move downward to pick the food item
        rbtx._gripper_x.set(item_cfg["gripper_width"])
        rbtx.move_p(pos=down_pos, rot=init_rotmat)
        rbtx._gripper_x.close()
        up_pos = np.array([item_cfg["preparation_pos"][0], item_cfg["preparation_pos"][1], 0.12])
        rbtx.move_p(pos=up_pos, rot=init_rotmat)

        # grasp the food item to the cooking position
        before_move2table = np.array([item_cfg["move2table"][0], item_cfg["move2table"][1], 0.12])
        rbtx.move_p(pos=before_move2table, rot=init_rotmat)
        rbtx.move_p(pos=np.array(item_cfg["move2table"]), rot=init_rotmat)
        rbtx.move_p(pos=np.array(item_cfg["release_pos"]), rot=init_rotmat)
        rbtx._gripper_x.open()
        time.sleep(1)

    rbtx.move_j(init_jnt)
    print('[INFO] Sandwich making completed. Robot returned to initial configuration.')
