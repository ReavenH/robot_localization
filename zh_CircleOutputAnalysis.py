import numpy as np
from scipy.io import loadmat

matfile = loadmat('zh_CircleOutput.mat')
yaw = np.array(matfile['yaw'])
print("The imported mutual yaw array is: \n{}".format(yaw))

# error range 5~10 degs is acceptable.
# TODO: 
# 1. check whether there is an intersection. Use max and min difference;
# 2. if there is an intersection, find the orthognal two lines;
# 3. calculate the mean yaw(s) and the centeroid offset.

def computePoseFromMutualGradients(yaw, minCircles = 3, tolerance = 8, verbose = False):
    '''
    compute the 2D pose of the robot body based on the mutual yaw matrix returned by the 
    circles.

    Input: mutual yaw mtx, in degrees.
    Output: 1) an array containing 1 yaw (if only one line clustered), or 2 if two lines are clustered.
            2) the centeroid offset.
    '''

	# step 1: get the centeroid.
    centroid = 0
    # step 2: in each row, cluster the similar yaw values.
    lineYaw = np.array([])
    entryVisited = np.zeros((yaw.shape)).astype('bool')
    for row in range(yaw.shape[0]):
        for column in range(yaw.shape[1]):
            if entryVisited[row][column] == False and entryVisited[column][row] == False:
                idx = np.where(np.abs(yaw[row, :] - yaw[row, column]) <= tolerance)[0]
                if verbose: print("idx:\n{}\n".format(idx))
                if len(idx) >= 3:
                        print("Line(s) found.\n")
                        meanYaw = np.mean(yaw[row, idx])
                        groupLine = np.where(np.abs(lineYaw - meanYaw) <= tolerance)[0]
                        if groupLine.size == 0:
                                lineYaw = np.append(lineYaw, meanYaw)
                        else:
                                lineYaw[groupLine] = (lineYaw[groupLine] + meanYaw) / 2  # update the mean yaw.
                        print("mean:\n{}\n".format(meanYaw))
                        print("groupLine:\n{}\n".format(groupLine))
                        entryVisited[row, idx] = True
                        entryVisited[idx, row] = True
                        if verbose:
                                print("lineYaw:\n{}\n".format(lineYaw))
                                print("entryVisited:\n{}\n".format(entryVisited))
                else:
                        print("No lines found.\n")
                        entryVisited[row, idx] = True
                        entryVisited[idx, row] = True
                        '''
                        if verbose: 
                           print(lineYaw)  
                           print(entryVisited)
                        '''
                if verbose: print("----------------------------------------------------------")
                     
if __name__ == "__main__": 
     yaw = loadmat('zh_CircleOutput.mat')['yaw']
     computePoseFromMutualGradients(yaw, verbose=True)