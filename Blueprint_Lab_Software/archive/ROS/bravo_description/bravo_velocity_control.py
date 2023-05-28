
import time

import pybullet as p

ARM_TYPE = 'Bravo5S'

class BravoBulletVelocityControl:

    def __init__(self):
        p.connect(p.GUI)
        if ARM_TYPE == 'Bravo7C':
            self.arm = p.loadURDF("urdf/bravo_7C.urdf",useFixedBase=True)
        elif ARM_TYPE == 'Bravo5C':
            self.arm = p.loadURDF("urdf/bravo_5C.urdf", useFixedBase=True)
        elif ARM_TYPE == 'Bravo7S':
            self.arm = p.loadURDF("urdf/bravo_7S.urdf", useFixedBase=True)
        elif ARM_TYPE == 'Bravo5S':
            self.arm = p.loadURDF("urdf/bravo_5S.urdf", useFixedBase=True)

        print("num joints:",p.getNumJoints(self.arm))
        #self.arm = Bravo7Bullet(position=(0, 0, 0), orientation=(0, 0, 0))
        mag = 4
        self.b_slider = p.addUserDebugParameter("b_Slider", -mag, mag, 0)
        self.c_slider = p.addUserDebugParameter("c_Slider", -mag, mag, 0)
        self.d_slider = p.addUserDebugParameter("d_Slider", -mag, mag, 0)
        self.e_slider = p.addUserDebugParameter("e_Slider", -mag, mag, 0)
        self.f_slider = p.addUserDebugParameter("f_Slider", -mag, mag, 0)
        self.g_slider = p.addUserDebugParameter("g_Slider", -mag, mag, 0)

    def run(self):

        velocity_b = p.readUserDebugParameter(self.b_slider)
        velocity_c = p.readUserDebugParameter(self.c_slider)
        velocity_d = p.readUserDebugParameter(self.d_slider)
        velocity_e = p.readUserDebugParameter(self.e_slider)
        velocity_f = p.readUserDebugParameter(self.f_slider)
        velocity_g = p.readUserDebugParameter(self.g_slider)
        if ARM_TYPE == 'Bravo7C':
            velocities = [velocity_g,velocity_f,velocity_e,velocity_d,velocity_c,velocity_b]
            p.setJointMotorControlArray(self.arm, [2, 3, 4, 5, 6, 7], p.VELOCITY_CONTROL, targetVelocities=velocities)
        elif ARM_TYPE == 'Bravo5C':
            velocities = [velocity_e, velocity_d, velocity_c, velocity_b]
            p.setJointMotorControlArray(self.arm, [2, 3, 4, 5], p.VELOCITY_CONTROL, targetVelocities=velocities)
        elif ARM_TYPE == 'Bravo7S':
            velocities = [velocity_g,velocity_f,velocity_e,velocity_d,velocity_c,velocity_b]
            p.setJointMotorControlArray(self.arm, [2, 3, 6, 7, 8, 9], p.VELOCITY_CONTROL, targetVelocities=velocities)
        elif ARM_TYPE == 'Bravo5S':
            velocities = [velocity_e, velocity_d, velocity_c, velocity_b]
            p.setJointMotorControlArray(self.arm, [2, 3, 6, 7], p.VELOCITY_CONTROL, targetVelocities=velocities)



        p.stepSimulation()
        time.sleep(1. / 240.)


if __name__ == '__main__':
    sim = BravoBulletVelocityControl()
    for i in range(10000000):
        sim.run()
