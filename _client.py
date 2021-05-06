import socket
import numpy as np
import math
from tf.transformations import compose_matrix, quaternion_from_euler, quaternion_from_matrix, inverse_matrix, euler_from_quaternion
import evaluate_phase_1_without_2

# helper fcns for transformations
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-4


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))
    
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

# Calculates Rotation Matrix given euler angles.
def eulerAnglesToRotationMatrix(theta) :
    
    R_x = np.array([[1,         0,                  0                   ],
                    [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                    [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                    ])
        
        
                    
    R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                    [0,                     1,      0                   ],
                    [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                    ])
                
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
                    
                    
    R = np.dot(R_z, np.dot( R_y, R_x ))

    return R


# THE CLIENT:
def cartesian_move_real_panda(_T_=None,res=0,should_grasp=0,obj_width=0.0,insert_= 0, max_time=5.0,force_limit = 15.0, the_ee = 0, verbose=0):
    host = socket.gethostbyname('132.72.103.111')
    port = 12346
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host,port))

    if _T_ is None: # hold reset position
        # default if no _R_ specified 
        print("Using default Orientation...")
        R_ = eulerAnglesToRotationMatrix([3.14159, 0.0, 0.0]) # from euler
        r11 = R_[0,0]
        r21 = R_[1,0]
        r31 = R_[2,0]

        r12 = R_[0,1]
        r22 = R_[1,1] 
        r32 = R_[2,1] 

        r13 = R_[0,2]
        r23 = R_[1,2]
        r33 = R_[2,2]

        x = 0.3068  # input("goal x ")
        y = 0.0  # input("goal y ")
        z = 0.4787  # input("goal z ")
        euler = rotationMatrixToEulerAngles(R_[0:3, 0:3])
    else: 
        print("Using given _T_ matrix (4x4)!")
        r11 = _T_[0,0]
        r21 = _T_[1,0]
        r31 = _T_[2,0]

        r12 = _T_[0,1]
        r22 = _T_[1,1] 
        r32 = _T_[2,1] 

        r13 = _T_[0,2]
        r23 = _T_[1,2]
        r33 = _T_[2,2]

        x = _T_[0,3]
        y = _T_[1,3]
        z = _T_[2,3]
        euler = rotationMatrixToEulerAngles(_T_[0:3, 0:3])

    # grasp mode: 0 - dont change, 1 - open @ width, 2 - close on object, 3 - release
    sg = should_grasp
    # reset with theta (res=1)
    # reset with joint goal before cartesian motion?
    theta = 0.0
    # find exact position and report delta!
    if insert_ == 1:
        insert = 1  # if inserting object:
        sg = 0  # do not release
        res = 0  # do not reset joint pose
    else:
        insert = 0

    if res >= 4: the_ee = 1


    # if (0.5 * math.pi) <= euler[2] <= (1.5 * math.pi):
    #     theta = -math.pi
    # elif (-0.5 * math.pi) >= euler[2] >= (-1.5 * math.pi):
    #     theta = -math.pi
    print("EULER Z ANGLE: ", euler[2], " and divided by pi: ", euler[2]/math.pi)
    if (-0.5 * math.pi) < euler[2] < (0.5 * math.pi):
        theta = 0.0; print(1)
        if (0.49 * math.pi) < euler[2] < (0.51 * math.pi): theta = -euler[2]; print(1.5)
    elif (0.5 * math.pi) < euler[2] <= (0.7 * math.pi):
        if y > 0.0:
            theta = -math.pi; print(2)
        else:
            theta = 0.51 * math.pi; print(2.5)
    elif (0.7 * math.pi) < euler[2] < (1.5 * math.pi): theta = -math.pi; print(3)
    elif (-0.5 * math.pi) >= euler[2] > (-0.9 * math.pi):
        if y > 0.0:
            theta = -math.pi; print(4)
        else:
            theta = 0.51 * math.pi; print(4.5)
    elif (-0.9 * math.pi) >= euler[2] >= (-1.5 * math.pi): theta = -math.pi; print(5)
    #elif (-1.5 * math.pi) >= euler[2] >= -2.0


    # send pose & ori to server
    s.sendall(str(x)+" "+str(y)+" "+str(z)+" "+str(r11)+" "+str(r21)+" "+str(r31)+" "+str(r12)+" "+str(r22)+" "+str(r32)+" "+str(r13)+" "+str(r23)+" "+str(r33)
              +" "+str(res)+" "+str(sg)+" "+str(obj_width)+" "+str(theta)+" "+str(insert)+" "+str(max_time)+" "+str(force_limit)+" "+str(the_ee))

    srv_msg = s.recv(1024)  # get EoM statement
    report = s.recv(1024) # get EoM report
    s.close()
    # reformat report to array type
    grasped_ = 0
    try:
        report_ = map(float, report.split(' '))
        pos_ =  report_[0:3]
        error_ = report_[3:7]
        ori_ = np.array([report_[7:10],report_[10:13],report_[13:16]])
        stopped_ = report_[16]
        grasped_ = report_[17]
    except:
        pass
    if stopped_ == 1.0:
        print('Collision, I have stopped.')
    if verbose==1:
        print(srv_msg)
        print("[x,y,z]", pos_)
        print("[errors: x,y,z,ori]", error_)
        print("Reached Orientation Matrix")
        print(ori_)
        euler_ori = rotationMatrixToEulerAngles(ori_)
        print("Euler: ", euler_ori)

    return srv_msg, pos_, error_, ori_, stopped_, grasped_

def find_exact_pose(_T_=None,obj_last_descent=0.0, object_class=None, the_ee=0, object_width=0.05):
    print(" Find Exact Pose Script GO! ")
    path_to_Dataset = "/home/dan/franka/Data2Tb/Dataset/"
    points_array = np.load(path_to_Dataset + 'real_and_sim/0_bias_and_pt_array/points_array_2p5mm_15mm.npy')
    reset = 0
    should_grasp = 0
    ins = 1
    gear_mode_xtra = 0.0
    # set contact force limits and other parameters specific to the object
    if object_class == 'gear1':
        force_limit = 8.0
        gear_mode_xtra = 0.01
    elif object_class == 'gear2':
        force_limit = 8.0
        gear_mode_xtra = 0.01
    elif object_class == 'compound_gear':
        force_limit = 17.0
    elif object_class in ['shaft1', 'shaft2']:
        force_limit = 16.0
        the_ee = 1
    else:
        force_limit = 15.0

    x_start = _T_[0, 3]
    y_start = _T_[1, 3]
    z_start = _T_[2, 3]  # goal z @ fully inserted.

    _T_[0, 3] += 0.0  # x
    _T_[1, 3] += 0.0  # y
    _T_[2, 3] += obj_last_descent - 0.002  # set z to current position - 2mm
    srv_msg, pos_, error_, ori_, stopped_, grasped_ = cartesian_move_real_panda(_T_, res=reset, should_grasp=2, obj_width=object_width, insert_=ins, max_time=1.5, force_limit=force_limit, the_ee=the_ee)

    # loop A
    while stopped_ == 0.0:  # maybe the initial position was good enough. (works!)
        print("DOWN LOOP A")

        _T_[2, 3] -= 0.002
        srv_msg, pos_, error_, ori_, stopped_, grasped_ = cartesian_move_real_panda(_T_, res=reset, should_grasp=should_grasp, insert_=ins, max_time=1.5, force_limit=force_limit, the_ee=the_ee)

        if (_T_[2, 3] - z_start) < (obj_last_descent - 0.0075):  # INSERT SUCCESS descent fully (unless gear, then descent to 1cm above and then down in 4 steps)
            _T_[2, 3] = z_start + gear_mode_xtra
            srv_msg, pos_, error_, ori_, stopped_, grasped_ = cartesian_move_real_panda(_T_, res=reset, should_grasp=should_grasp, insert_=ins, max_time=5.0, force_limit=force_limit, the_ee=the_ee)

            while stopped_ == 0.0 and (_T_[2, 3] - z_start) > 0.0025 and object_class in ['gear1', 'gear2']:  # do the last 1 cm in four steps for gears
                _T_[2, 3] -= gear_mode_xtra / 4
                srv_msg, pos_, error_, ori_, stopped_, grasped_ = cartesian_move_real_panda(_T_, res=reset, should_grasp=should_grasp, insert_=ins, max_time=2.0, force_limit=force_limit, the_ee=the_ee)

            if stopped_ == 0.0:
                if pos_[2] == z_start:
                    print("Finished successfully at pose ", pos_, " unstopped.")
                    return 1
                else:
                    print("Finished loop A ... going into loop B")
                    stopped_ = 1.0
            else:
                print("Stopped at ", pos_, " in loop A")
                if object_class not in ["shaft1", "shaft2"]:
                    print("Finished - I hope I did good.")
                    return 1
    # loop B
    _T_[2, 3] = pos_[2]; print(" GOING INTO LOOP B !!!")
    cartesian_move_real_panda(_T_, res=reset, should_grasp=should_grasp, insert_=ins, max_time=2.0, force_limit=force_limit, the_ee=the_ee)
    stopped_ = 1.0
    _ctr_ = 0
    _ctr2_ = 0

    while stopped_ == 1.0:
        _T_[0, 3] = x_start + points_array[_ctr_, 0]  # x
        _T_[1, 3] = y_start + points_array[_ctr_, 1]  # y
        print('New x and y: ', _T_[0, 3], ' ', _T_[1, 3])


        _T_[2, 3] += 0.008  # go up - out of collision into new x,y
        print("UP LOOP B +0.008")
        cartesian_move_real_panda(_T_, res=reset, should_grasp=should_grasp, insert_=ins, max_time=2.0, force_limit=force_limit, the_ee=the_ee)

        _T_[2, 3] -= 0.006  # go down
        print("DOWN LOOP B.1.1 -0.006")
        srv_msg, pos_, error_, ori_, stopped_, grasped_ = cartesian_move_real_panda(_T_, res=reset, should_grasp=should_grasp, insert_=ins, max_time=2.0, force_limit=force_limit, the_ee=the_ee)

        while stopped_ == 0.0: # go down until collision
            _T_[2, 3] -= 0.002
            print("DOWN LOOP B.1.2 -0.002")
            srv_msg, pos_, error_, ori_, stopped_, grasped_ = cartesian_move_real_panda(_T_, res=reset, should_grasp=should_grasp, insert_=ins, max_time=2.0, force_limit=force_limit, the_ee=the_ee)

            if stopped_ == 0.0: _ctr2_ += 1
            elif stopped_ == 1.0: _ctr2_ = 0
            if _ctr2_ >= 5:
                _T_[2,3] = z_start
                cartesian_move_real_panda(_T_, res=reset, should_grasp=should_grasp, insert_=ins, max_time=2.0, force_limit=force_limit, the_ee=the_ee)
                print("Finished insert!")
                return 1

        slack_ = 0.015
        if object_class in ['shaft1', 'shaft2']: slack_ = 0.0025
        if pos_[2] < z_start + slack_:
            if stopped_ == 0: print('Finished and releasing part (unstopped)')
            else: print('Finished and releasing part (stopped, there might be a problem?!)')
            return 1
        _ctr_ += 1





def picturetime(scene_number, image_index, current_reference_point_wrt_world, paths, model_phase_1=None, model_phase_2=None, take_new_images=True, pipeline = None, without_2 = False):
    model_phase_1, model_phase_2, pipeline = evaluate_phase_1_without_2.evaluate_image(scene_number, image_index,
                                                                    current_reference_point_wrt_world, paths,
                                                                    model_phase_1, model_phase_2, take_new_images,
                                                                    evaluate_sim=False, pipeline=pipeline, without_2=without_2)

    evaluate_phase_1_without_2.remove_wrong_standing_shafts_and_shafts_from_the_sides(paths.real_unsorted_csv_path, paths.real_csv_path, scene_number)
    objects_number = evaluate_phase_1_without_2.joint_duplicates_and_choose_theta(paths.real_csv_path, scene_number)




if __name__=="__main__":
    table_z = 0.03
    EE_tip_to_EE_CS = 0.009
    table_corner_z = 0.034
    obj_last_descent = 0.033
    foto_height = 0.53
    foto_height_close = 0.31

    x = 0.5
    y = 0.0
    z = table_z + 0.2
    a = 180
    b = 0
    c = 0

    R_ = eulerAnglesToRotationMatrix([math.radians(a), math.radians(b), math.radians(c)]) # from euler
    print("Goal Orientation Matrix: ")
    print(R_)

    _T_ = compose_matrix(angles=[math.radians(a), math.radians(b), math.radians(c)], translate=[x, y, z])
    print(_T_)


    # from sim_parameters import TCP_wrt_gear1_matrix2
    # _T_2 = np.dot(_T_, TCP_wrt_gear1_matrix2)

    the_ee = 0  # 0 - center of fingers, 1 - for shaft insert, 2 - at camera ref pt
    reset = 1 # (0,1,2,3,4,5) - (dont, above dropoff, above pickup, only grasp, shaft dropoff1, shaft dropoff2)
    should_grasp = 0# (0,1,2,3)  - (dont change, open@width, close on obj, release)
    obj_width = 0.067
    ins = 0 # 0 dont 1 do  // bool!
    srv_msg, pos_, error_, ori_, stopped_, grasped_ = cartesian_move_real_panda(_T_,res=reset,should_grasp=should_grasp,obj_width=obj_width,insert_=ins, max_time=5.0, the_ee=the_ee, verbose=1)

    #
    # from evaluate_phase_1_without_2 import Evaluation_Paths, remove_current_scene_detections_from_csv
    # paths = Evaluation_Paths(without_2=False)
    # remove_current_scene_detections_from_csv(paths.real_unsorted_csv_path, paths.real_csv_path, 2)
    # picturetime(scene_number=13, image_index=1, current_reference_point_wrt_world=_T_, paths=paths, model_phase_1=None, model_phase_2=None, take_new_images=True, pipeline = None, without_2 = False)

    # #find_exact_pose(_T_, obj_last_descent)




