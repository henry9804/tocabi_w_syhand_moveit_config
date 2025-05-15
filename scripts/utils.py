import numpy as np

JOINT_NAMES = ["L_HipYaw_Joint",    "L_HipRoll_Joint",    "L_HipPitch_Joint",
               "L_Knee_Joint",      "L_AnklePitch_Joint", "L_AnkleRoll_Joint",
               "R_HipYaw_Joint",    "R_HipRoll_Joint",    "R_HipPitch_Joint",
               "R_Knee_Joint",      "R_AnklePitch_Joint", "R_AnkleRoll_Joint",
               "Waist1_Joint",      "Waist2_Joint",       "Upperbody_Joint",
               "L_Shoulder1_Joint", "L_Shoulder2_Joint",  "L_Shoulder3_Joint", "L_Armlink_Joint",
               "L_Elbow_Joint",     "L_Forearm_Joint",    "L_Wrist1_Joint",    "L_Wrist2_Joint",
               "Neck_Joint",        "Head_Joint",
               "R_Shoulder1_Joint", "R_Shoulder2_Joint",  "R_Shoulder3_Joint", "R_Armlink_Joint",
               "R_Elbow_Joint",     "R_Forearm_Joint",    "R_Wrist1_Joint",    "R_Wrist2_Joint"]

INIT_POSE = np.array([0.0,  0.0, -0.24, 0.6, -0.36, 0.0,
                      0.0,  0.0, -0.24, 0.6, -0.36, 0.0,
                      0.0,  0.0,  0.0,
                      0.3,  0.3,  1.5, -1.27, -1.0, 0.0, -1.0, 0.0,
                      0.0,  0.3,
                     -0.3, -0.3, -1.5,  1.27,  1.0, 0.0,  1.0, 0.0])

READY_POSE = np.array([0.0,  0.0, -0.24,  0.6, -0.36,  0.0,
                       0.0,  0.0, -0.24,  0.6, -0.36,  0.0,
                       0.0,  0.0,  0.0,
                       0.0, -0.3,  1.57, -1.2, -1.57,  1.5,  0.4, -0.2,
                       0.0,  0.3,
                       0.0,  0.3, -1.57,  1.2,  1.57, -1.5, -0.4,  0.2])

def cubic(time, time_0, time_f, x_0, x_f, x_dot_0, x_dot_f):
    if time < time_0:
        return x_0, x_dot_0
    elif time > time_f:
        return x_f,x_dot_f
    else:
        elapsed_time = time - time_0
        total_time = time_f - time_0
        total_x = x_f - x_0

        x_t = x_0 + x_dot_0 * elapsed_time \
              + (3 * total_x / total_time ** 2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time ** 2\
              + (-2 * total_x / total_time ** 3 + (x_dot_0 + x_dot_f) / total_time ** 2) * elapsed_time ** 3
        
        x_dot_t = x_dot_0 \
                  + 2 * (3 * total_x / total_time ** 2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time \
                  + 3 * (-2 * total_x / total_time ** 3 + (x_dot_0 + x_dot_f) / total_time ** 2) * elapsed_time ** 2
        
        return x_t, x_dot_t