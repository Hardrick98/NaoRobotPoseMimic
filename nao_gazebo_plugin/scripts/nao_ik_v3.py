from hashlib import shake_128
import numpy as np
import nao_fk as fk
import trans_utils as tu


def IK_LL(T=fk.FK_LL()):
    ThighLength = 100
    TibiaLength = 102.9
    FootHeight = 45.19
    HipOffsetZ = 85
    HipOffsetY = 50
    Ab = fk.A([0, HipOffsetY, -HipOffsetZ]) 
    Ae = fk.A([0, 0, -FootHeight]) 

    the1min= -1.145303
    the1max= 0.740810
    the2min= -0.379472
    the2max= 0.790477
    the3min= -1.773912
    the3max= 0.484090
    the4min= -0.092346
    the4max= 2.112528
    the5min= -1.189516
    the5max= 0.922747
    the6min= -0.397880
    the6max=  0.769001

    soln = []


    T_hat = np.linalg.pinv(Ab)@T@np.linalg.pinv(Ae)
    T_tilde = fk.R_x(np.pi/4)@T_hat
    T_prime = np.linalg.pinv(T_tilde)
        
    d = T_prime[0,3]**2+T_prime[1,3]**2+T_prime[2,3]**2
    #rounding here prevents numerical error from pushing us to slighly out of arccos domain
    val = np.round((ThighLength**2 + TibiaLength**2 - d)/(2*TibiaLength*ThighLength),4)
    if val > 1:
        the4prime = 1
    elif val < -1:
        the4prime = -1
    else: 
        the4prime = np.arccos(val)


    
    the4 = np.pi-the4prime

    the6 = np.arctan2(T_prime[1,3],T_prime[2,3]) 
    #print("T6: ",the6)
    for n in range(2): # looping for the6
        if n != 0:
            the6 = 0
        if the6 < the6min or the6 > the6max:
            continue

        for i in range(2): #looping for the4
            if i != 0:
                the4 = -the4
            if the4 < the4min or the4 > the4max:
                continue

            T56 = T_n_s(0,-np.pi/2,0,the6)
            T_prime_tilde = T_tilde@np.linalg.pinv(T56@fk.R_z(np.pi)@fk.R_y(-np.pi/2))
            T_dprime = np.linalg.pinv(T_prime_tilde)

            top = T_dprime[1,3]*TibiaLength+ThighLength*np.cos(the4) + ThighLength*T_dprime[0,3]*np.sin(the4)
            bot = ThighLength**2*np.sin(the4)**2 + (TibiaLength+ThighLength*np.cos(the4))**2

            the5 = np.arcsin(-top/bot)
            #print(the6)

            for j in range(2): #looping for the5
                if j != 0:
                    the5 = np.pi - the5
                #print("T5: ", the5)
                if the5 < the5min or the5 > the5max:
                    continue

                T34 = T_n_s(-ThighLength,0,0,the4)
                #print(T34)
                T45 = T_n_s(-TibiaLength,0,0,the5)
                #print(T45)
                T_tprime = T_prime_tilde@np.linalg.pinv(T34@T45)
                #print(T_tprime)

                the2 = np.arccos(T_tprime[1,2])-np.pi/4

                for k in range(2): # looping for the2
                    if k != 0:
                        the2 = -np.arccos(T_tprime[1,2])-np.pi/4
                    #print("T2: ", the2)
                    if the2 < the2min or the2 > the2max:
                        continue
            
                    the3 = np.arcsin(T_tprime[1,1]/np.sin(the2+np.pi/4))

                    the1 = np.pi/2 + np.arccos(T_tprime[0,2]/np.sin(the2+np.pi/4))

                    for l in range(2): # looping for the3
                        if l != 0: 
                            the3 = np.pi - the3
                        #print("T3: ", the3)
                        if the3 < the3min or the3 > the3max:
                            continue

                        for m in range(2): # looping for the1
                        
                            if m != 0:
                                the1 = np.pi/2 - np.arccos(T_tprime[0,2]/np.sin(the2+np.pi/4))
                            #print("T1: ",the1)
                            if the1 < the1min or the1 > the1max:
                                continue

                            #print("Trying:\n")
                            #print(the1,the2,the3,the4,the5,the6)
                            if checkGoodLL(the1,the2,the3,the4,the5,the6,T):
                                if soln == []:
                                    soln.append([the1,the2,the3,the4,the5,the6])
                                else:
                                    soln = checkBetterLL(soln,[the1,the2,the3,the4,the5,the6],T)
                            
                                #soln[0].append(the1)
                                #soln[1].append(the2)
                                #soln[2].append(the3)
                                #soln[3].append(the4)
                                #soln[4].append(the5)
                                #soln[5].append(the6)
    print (soln)
    if any(isinstance(i, list) for i in soln):
        return soln[0]
    else:
        return soln


def IK_RL(T=fk.FK_RL()):
    ThighLength = 100
    TibiaLength = 102.9
    FootHeight = 45.19
    HipOffsetZ = 85
    HipOffsetY = 50
    Ab = fk.A([0, -HipOffsetY, -HipOffsetZ]) 
    Ae = fk.A([0, 0, -FootHeight]) 

    the1min= -1.145303
    the1max= 0.740810
    the2min= -0.738321
    the2max= 0.414754
    the3min= -1.772308
    the3max= 0.485624
    the4min= -0.103083
    the4max= 2.120198
    the5min= -1.186448
    the5max= 0.932056
    the6min= -0.785875
    the6max= 0.388676

    soln = []


    T_hat = np.linalg.pinv(Ab)@T@np.linalg.pinv(Ae)
    T_tilde = fk.R_x(-np.pi/4)@T_hat
    T_prime = np.linalg.pinv(T_tilde)
    #d = np.sqrt(T_prime[0,3]**2+T_prime[1,3]**2+T_prime[2,3]**2)
    #the4prime = np.arccos((ThighLength**2 + TibiaLength**2 - d**2)/(2*TibiaLength*ThighLength))
    
    d = T_prime[0,3]**2+T_prime[1,3]**2+T_prime[2,3]**2
    #rounding here prevents numerical error from pushing us to slighly out of arccos domain
    val = np.round((ThighLength**2 + TibiaLength**2 - d)/(2*TibiaLength*ThighLength),4)
    if val > 1:
        the4prime = 1
        print("WARNING: using arccos(1) due to excessively large input")
    elif val < -1:
        the4prime = -1
        print("WARNING: using arccos(-1) due to excessively small input")
    else: 
        the4prime = np.arccos(val)
    the4 = np.pi-the4prime

    the6 = np.arctan2(T_prime[1,3],T_prime[2,3]) 
    #print("T6: ",the6)
    for n in range(2): # looping for the6
        if n != 0:
            the6 = 0
        if the6 < the6min or the6 > the6max:
            print("skipping t6= ",the6)
            continue

        for i in range(2):
            if i != 0:
                the4 = -the4
            #print("T4: ", the4)
            if the4 < the4min or the4 > the4max:
                print("skipping t4= ",the4)
                continue

            T56 = T_n_s(0,-np.pi/2,0,the6)

            T_prime_tilde = T_tilde@np.linalg.pinv(T56@fk.R_z(np.pi)@fk.R_y(-np.pi/2))
            T_dprime = np.linalg.pinv(T_prime_tilde)

            top = T_dprime[1,3]*TibiaLength+ThighLength*np.cos(the4) + ThighLength*T_dprime[0,3]*np.sin(the4)
            bot = ThighLength**2*np.sin(the4)**2 + (TibiaLength+ThighLength*np.cos(the4))**2

            the5 = np.arcsin(-top/bot)

            for j in range(2):
            
                if j != 0:
                    the5 = np.pi - the5
                #print("T5: ", the5)
                if the5 < the5min or the5 > the5max:
                    print("skipping t5= ",the5)
                    continue

                T34 = T_n_s(-ThighLength,0,0,the4)
                T45 = T_n_s(-TibiaLength,0,0,the5)
                T_tprime = T_prime_tilde@np.linalg.pinv(T34@T45)

                the2 = np.arccos(T_tprime[1,2])-np.pi/4

                for k in range(2):
                    if k != 0:
                        the2 = -np.arccos(T_tprime[1,2])-np.pi/4
                    #print("T2: ", the2)
                    if the2 < the2min or the2 > the2max:
                        print("skipping t2= ",the2)
                        continue
            
                    the3 = np.arcsin(T_tprime[1,1]/np.sin(the2-np.pi/4))

                    the1 = np.pi/2 + np.arccos(T_tprime[0,2]/np.sin(the2-np.pi/4))

                    for l in range(2):
                        if l != 0: 
                            the3 = np.pi - the3
                        #print("T3: ", the3)
                        if the3 < the3min or the3 > the3max:
                            print("skipping t3= ",the3)
                            continue

                        for m in range(2):
                            if m != 0:
                                the1 = np.pi/2 - np.arccos(T_tprime[0,2]/np.sin(the2-np.pi/4))
                            #print("T1: ",the1)
                            if the1 < the1min or the1 > the1max:
                                print("skipping t1= ",the1)
                                continue

                            #print("Trying:\n")
                            #print(the1,the2,the3,the4,the5,the6)
                            if checkGoodRL(the1,the2,the3,the4,the5,the6,T):
                                print("Solution found!")
                                if soln == []:
                                    soln.append([the1,the2,the3,the4,the5,the6])
                                else:
                                    soln = checkBetterRL(soln,[the1,the2,the3,the4,the5,the6],T)
                            
                                #soln[0].append(the1)
                                #soln[1].append(the2)
                                #soln[2].append(the3)
                                #soln[3].append(the4)
                                #soln[4].append(the5)
                                #soln[5].append(the6)

    print (soln)
    if any(isinstance(i, list) for i in soln):
        return soln[0]
    else:
        return soln

def T_n_s (a, alp, d, the):
    # CANNOT SUBSTITUTE fk.T_n --> creates 3D arrays and relies on list structure
    Tn = np.array([[np.cos(the), -np.sin(the), 0, a],
                        [np.sin(the)*np.cos(alp), np.cos(the)*np.cos(alp), -np.sin(alp), -d*np.sin(alp)],
                        [np.sin(the)*np.sin(alp), np.cos(the)*np.sin(alp), np.cos(alp), d*np.cos(alp)],
                        [0, 0, 0, 1]])

    return Tn

def checkGoodLL(t1,t2,t3,t4,t5,t6,T_goal):
    T_goal = np.array(T_goal)
    P_goal = tu.transmat2sixvec(T_goal)

    T_cur = fk.FK_LL(t1,t2,t3,t4,t5,t6)
    P_cur = tu.transmat2sixvec(T_cur)

    #allow 1 mm position error and 0.1 rad rotation error
    x_err = P_goal[0] - P_cur[0]
    y_err = P_goal[1] - P_cur[1]
    z_err = P_goal[2] - P_cur[2]


    pos_err = np.sqrt(x_err**2 + y_err**2 + z_err**2)

    #print(x_err, y_err, z_err)
    #print(pos_err)

    Rx_err = P_goal[3] - P_cur[3]
    Ry_err = P_goal[4] - P_cur[4]
    Rz_err = P_goal[5] - P_cur[5]

    #print(Rx_err,Ry_err,Rz_err)

    rot_err = np.sqrt(Rx_err**2 + Ry_err**2 + Rz_err**2)

    #print(rot_err)
    
    if pos_err < 25.4 and rot_err < 0.5:
        return True
    else: return False
def checkGoodRL(t1,t2,t3,t4,t5,t6,T_goal):
    T_goal = np.array(T_goal)
    P_goal = tu.transmat2sixvec(T_goal)

    T_cur = fk.FK_RL(t1,t2,t3,t4,t5,t6)
    P_cur = tu.transmat2sixvec(T_cur)

    #allow 1 mm position error and 0.1 rad rotation error
    x_err = T_goal[3,0] - T_cur[3,0]
    y_err = T_goal[3,1] - T_cur[3,1]
    z_err = T_goal[3,2] - T_cur[3,2]

    pos_err = np.sqrt(x_err**2 + y_err**2 + z_err**2)

    print("trying:",t1,t2,t3,t4,t5,t6)
    print(pos_err)

    Rx_err = P_goal[3] - P_cur[3]
    Ry_err = P_goal[4] - P_cur[4]
    Rz_err = P_goal[5] - P_cur[5]

    rot_err = np.sqrt(Rx_err**2 + Ry_err**2 + Rz_err**2)
    print(rot_err)
    
    if pos_err < 25.4 and rot_err < 0.50:
        return True
    else: return False


def checkBetterRL(s1,s2,goal):
    P1 = fk.FK_RL(s1[0][0],s1[0][1],s1[0][2],s1[0][3],s1[0][4],s1[0][5])
    P2 = fk.FK_RL(s2[0],s2[1],s2[2],s2[3],s2[4],s2[5])
    g1 = P1[0,3]**2+P1[1,3]**2+P1[2,3]**2
    g2 = P2[0,3]**2+P2[1,3]**2+P2[2,3]**2
    g_goal = goal[0,3]**2 + goal[1,3]**2 + goal[2,3]**2
    q1 = np.abs(g1-g_goal)
    q2 = np.abs(g2-g_goal)

    if q2 < q1:
        return [s2]
    else: return s1

def checkBetterLL(s1,s2,goal):
    P1 = fk.FK_LL(s1[0][0],s1[0][1],s1[0][2],s1[0][3],s1[0][4],s1[0][5])
    P2 = fk.FK_LL(s2[0],s2[1],s2[2],s2[3],s2[4],s2[5])
    g1 = P1[0,3]**2+P1[1,3]**2+P1[2,3]**2
    g2 = P2[0,3]**2+P2[1,3]**2+P2[2,3]**2
    g_goal = goal[0,3]**2 + goal[1,3]**2 + goal[2,3]**2
    q1 = np.abs(g1-g_goal)
    q2 = np.abs(g2-g_goal)

    if q2 < q1:
        print("better")
        return [s2]
    else: 
        print("worse")
        return s1

if __name__ == "__main__":
    print("Left Leg: ")
    IKLL = IK_LL()
    print(IKLL)
    print("Right Leg: ")
    IKRL = IK_RL()
    
    #LLT=np.array([[9.91492293e-01,  2.87930223e-04, -1.30165091e-01,  4.71580516e+00],
    #    [-2.75089191e-04,  9.99999955e-01,  1.16631872e-04,  4.99708743e+01],
    #    [ 1.30165119e-01, -7.98325924e-05,  9.91492327e-01, -3.17259889e+02],
    #    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
    print("\nCustom test \n")
    IKRL = IK_RL(fk.FK_RL(0, 0, -np.pi/8, np.pi/4, -np.pi/6, 0))
    #print(IKRL)
