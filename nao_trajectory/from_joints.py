import numpy as np
data  = np.load('joints.npy')
lst = data

for i in range(int(len(lst)/50)):



    pose = lst[i*50]

    rhand = pose[16]

    lhand = pose[13]
    
    x = -rhand[2]
    z = 54*rhand[1]/160
    y = rhand[0]

    print(x,y,z)


print(i)


    
    