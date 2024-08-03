from zh_Utilities import flipLinkage
import numpy as np
import time

myFlip = flipLinkage(0.034, 0.16, np.sqrt((0.06 - 0.034 + np.sqrt(0.16 ** 2 - 0.09 ** 2)) ** 2 + 0.09 ** 2), 0.06)
myFlip.kinematics(myFlip.angleBeta)

time.sleep(100)