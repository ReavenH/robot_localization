import numpy as np
from zh_Utilities import landmarks, brickMap, hmRPYG, hmRPYP, poseTags, drawGroundOG, drawRigidBodyOG, keyboardCtrl, drawFloor, drawLineOG, robot, R
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import time
from zh_DrawSceneOpenGL import brickPoses, myBrickMap, myTags
import scipy.io


'''
TODO:
1. Joint angle rearA-LinkageC is wrong. Done.
2. Coordinate propagation is wrong. (not symmetric). Done.
3. The translation of the linkageA-C joint is wrong (should not be all zeros). Done.
4. Bind the OBJ file to the linkage model.
5. Calculate the body pose based on the standing pose from the ground (if it is needed for verification).
6. Design the brick placing system, first finish the linkage model design.
'''

if __name__ == "__main__":
    
    try:
        myTags = landmarks(hmRPYP, poseTags, None)  # tags use RPYP pose. Not specifying the axes.
        myBrickMap = brickMap(hmRPYG, None)
        myRobot = robot(hmRPYG, None, poseTags)
        myRobot.bodyPose[-1] = myBrickMap.brickThickness + myRobot.initFeetPos[0][1] - myRobot.linkageFrameOffsets[0][-1]+0.02 # Here is 0.015+0.085-(0.5 * 0.0376- 0.01145)
        # Could use the brickPoses instead of brickThickness, add it to for loop
        myRobot.feetPosControl(myRobot.initFeetPos)
        myRobot.propagateAllLegJointPoses()

        pygame.init()  # init the pygame lib.

        window = (1600, 1200)  # in pixels.
        
        pygame.display.set_mode(window, DOUBLEBUF|OPENGL)
        pygame.display.set_caption('DEMO Scene Simulation')

        # enable depth test. This avoids the incorrect transparency between objects.
        # this should be AFTER the pygame / glfw window is initialized.
        glEnable(GL_DEPTH_TEST)

        glClearColor(0.5, 0.5, 0.5, 1.0)

        gluPerspective(60, (window[0] / window[1]), 0.001, 10.0)

        
        glTranslatef(-0.5, -0.5, -1.7)
        glRotatef(-45, 1, 0, 0)  
        glRotatef(45, 0, 0, 1) 
        glTranslatef(0.5, 0, 0)
        
        # get the current view angle.
        hmScene = hmRPYP(-45, 0, 0, np.array([-0.5, -0.5, -1.7])).dot(hmRPYP(0, 0, 45, np.array([0.5, 0, 0])))
        angle_x, angle_y, angle_z = R.from_matrix(hmScene[:3, :3]).as_euler('zyx', degrees=True)[::-1]
        transScene = hmScene[:3, -1].flatten()
        distance = transScene[-1]
        print("angles XYZ: {} {} {}".format(angle_x, angle_y, angle_z))
        print("Translation XYZ: ", transScene)
        print("Linkage angles: ", myRobot.linkageAngles)

        # 7.15 by Haocheng Peng, some parameters used in single gait control
        cycle_input = 0.0
        increment = 0.01
        duration = 0.01
        # used to store data
        store_array=np.zeros((10000, 4, 3))
        cnt=0
        while True:
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)  # clear the screen.

            # interactive view angle.
            keyboardCtrl()

            drawFloor()

            for _, pose in enumerate(myTags.poses):
                drawGroundOG(myTags.hm(*pose[1:4], pose[4:]))
                drawRigidBodyOG(myTags.hm(*pose[1:4], pose[4:]).dot(myTags.vertices))

            for _, pose in enumerate(brickPoses):
                 _ = myBrickMap.place(pose)




            # by haocheng 7.15 use the signlegait and inverse kinematics to achieve movement sequence
            swing_height,x_distance,z_distance=myRobot.signlewalkgait(1,cycle_input)
            '''
            if(swing_height<0.033):
                swing_height=0.033
            if(z_distance<0.027):
                z_distance=0.027
            '''
            #myRobot._inverseKinematics(1, myRobot.TransPos[3][1][0]+x_distance, abs(-myRobot.TransPos[3][2][0]-swing_height)+0.085, (myRobot.TransPos[3][0][0]+z_distance)+0.025)
            myRobot.feetPosControl1(1, x_distance,-abs(swing_height) + 0.085,z_distance + 0.025)

            '''
            store_array[cnt, 0, 0] = swing_height
            store_array[cnt, 0, 1] = x_distance
            store_array[cnt, 0, 2] = z_distance
            '''


            swing_height, x_distance,z_distance = myRobot.signlewalkgait(4, cycle_input)
            '''
            if (swing_height < 0.033):
                swing_height = 0.033
            if (z_distance < 0.027):
                z_distance = 0.027
            '''
            #myRobot._inverseKinematics(4, myRobot.TransPos[3][1][0]+x_distance, abs(-myRobot.TransPos[3][2][0]-swing_height)+0.085, (-myRobot.TransPos[3][0][0]+z_distance)+0.025)
            myRobot.feetPosControl1(4, x_distance, -abs(swing_height) + 0.085, z_distance + 0.025)

            '''
            store_array[cnt, 1, 0] = swing_height
            store_array[cnt, 1, 1] = x_distance
            store_array[cnt, 1, 2] = z_distance
            
            print("swing height: ", swing_height)
            print("x distance: ",x_distance)
            print("z distance: ",z_distance)
            



            print("leg{}\nx: {}\ny: {}\nz: {}\n".format(4, myRobot.TransPos[3][1][
                0] + x_distance, -myRobot.TransPos[3][2][0] - swing_height, (-myRobot.TransPos[2][0][0]+z_distance)))
            print("leg{}\nx: {}\ny: {}\nz: {}\n".format(1, myRobot.TransPos[0][1][0]+x_distance, -myRobot.TransPos[0][2][0]-swing_height, (myRobot.TransPos[0][0][0]+z_distance)+0.025))
            
            '''

            cycle_input2 = cycle_input + 0.5
            if cycle_input2>1:
                cycle_input2 -= 1


            swing_height, x_distance,z_distance = myRobot.signlewalkgait(2,cycle_input2)
            '''
            if (swing_height < 0.033):
                swing_height = 0.033
            if (z_distance < 0.027):
                z_distance = 0.027
            '''
            #myRobot._inverseKinematics(2, myRobot.TransPos[1][1][0]+x_distance, abs(-myRobot.TransPos[1][2][0]-swing_height)+0.085, (myRobot.TransPos[1][0][0]+z_distance)+0.025)
            myRobot.feetPosControl1(2, x_distance, -abs(swing_height) + 0.085, z_distance + 0.025)
            '''
            store_array[cnt, 2, 0] = swing_height
            store_array[cnt, 2, 1] = x_distance
            store_array[cnt, 2, 2] = z_distance
            '''
            swing_height, x_distance,z_distance0 = myRobot.signlewalkgait(3,cycle_input2)
            '''
            if (swing_height < 0.033):
                swing_height = 0.033
            if (z_distance < 0.027):
                z_distance = 0.027
            '''

            #myRobot._inverseKinematics(3, myRobot.TransPos[2][1][0]+x_distance, abs(-myRobot.TransPos[2][2][0]-swing_height)+0.085, (-myRobot.TransPos[2][0][0]+z_distance)+0.025)
            myRobot.feetPosControl1(3, x_distance, -abs(swing_height) + 0.085, z_distance + 0.025)
            '''
            store_array[cnt, 3, 0] = swing_height
            store_array[cnt, 3, 1] = x_distance
            store_array[cnt, 3, 2] = z_distance
            '''

            '''
            cnt+=1
            if(cnt==99):
                scipy.io.savemat('example.mat', {'array': store_array})

            
            print("swing height: ", swing_height)
            print("x distance: ", x_distance)
            print("z distance: ", z_distance)
            
            print('transpose',myRobot.TransPos[2][1][0])
            print('x_distance: ',x_distance)
            print("leg{}\nx: {}\ny: {}\nz: {}\n".format(3, myRobot.TransPos[2][1][
                0] + x_distance, -myRobot.TransPos[2][2][0] - swing_height, -(-myRobot.TransPos[2][0][0]+z_distance)))
            print("leg{}\nx: {}\ny: {}\nz: {}\n".format(2, myRobot.TransPos[1][1][0] + x_distance, -myRobot.TransPos[1][2][0] - swing_height, myRobot.TransPos[1][0][0] + z_distance))
            '''

            # wait 0.01s
            time.sleep(duration)

            # increase cycle_input larger incremant means higher speed
            cycle_input += increment*4

            # check if it is equal to 1
            if cycle_input > 1:
                cycle_input = cycle_input - 1



            myRobot.propagateAllLegJointPoses()
            myRobot.drawRobotBody()
            myRobot.drawAllLegLinkagesOG()

            pygame.display.flip()
            pygame.time.wait(10)

            '''
            # 7.10 update the action sequence, let each legs move in each time
            stepIndex = (stepIndex + 1) % 8
            # 2 legs' movement in one step, and one step divide into 2 parts
            if stepIndex == 2 or stepIndex == 4 or stepIndex == 6 or stepIndex == 0:
                time.sleep(1)
            '''

    except KeyboardInterrupt:
        pygame.quit()
        quit()