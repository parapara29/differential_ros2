"""
This is the implementation of the OGN node defined in OgnRoboCamFollower.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy as np
from collections import deque

class OgnRoboCamFollower:
    """
         Node to make a custom camera follow the robot
    """
    ct =1
    radius = 0
    cam_prev_dist = np.zeros(2)
    camera_position = np.zeros(3)
    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""

        try:
            # Retrieve the robot position and orientation from the database as tuples
            robot_position_tuple = db.inputs.robot_position
            qw = db.inputs.qw
            qx = db.inputs.qx
            qy = db.inputs.qy
            qz = db.inputs.qz

            # Convert the tuples to numpy arrays
            robot_position = np.array(robot_position_tuple)

            # Retrieve the camera distance behind the robot from the database
            camera_distance_behind = np.array(db.inputs.cam_offset)
    
            # Convert the robot's orientation quaternion to a rotation matrix.
            Rz,yaw = quaternion_to_rotation_matrix(qw,qx,qy,qz)
            
            # Calculate the camera offset behind the robot.
            camera_offset = np.array([camera_distance_behind[0],camera_distance_behind[1], #initial position of the camera
                                      camera_distance_behind[2]]) 
    
            # Store the updated camera position in the database
            camera_position = robot_position - np.dot(Rz,camera_offset)
            #method 1 for smoothening starts here
            alpha = 0.2 #smoothning factor
            camera_position[0] = alpha * camera_position[0] + (1 - alpha) * camera_position[0]
            camera_position[1] = alpha * camera_position[1] + (1 - alpha) * camera_position[1]
            camera_position[2] = 0.5
            #end of method 1

            #method 2 of smoothening starts here
             # Apply smoothing to camera_position using a moving average filter
            # window_size = 5  # Adjust the window size as needed
            # if not hasattr(OgnRoboCamFollower, 'cam_prev_dist'):
            #     OgnRoboCamFollower.cam_prev_dist = []
            # OgnRoboCamFollower.cam_prev_dist.append(camera_position)
            # if len(OgnRoboCamFollower.cam_prev_dist) > window_size:
            #     OgnRoboCamFollower.cam_prev_dist.pop(0)
            
            # # Calculate the smoothed camera position for each component (x, y, z)
            # smoothed_camera_position = np.mean(OgnRoboCamFollower.cam_prev_dist, axis=0)

            # smoothed_camera_position[2] = 0.5

            db.outputs.cam_pos = camera_position
            # db.outputs.cam_pos = camera_position
            db.outputs.yaw = yaw
            pass

        except Exception as error:
            # If anything causes your compute to fail report the error and return False
            db.log_error(str(error))
            return False

        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True

def quaternion_to_rotation_matrix(qw,qx,qy,qz):
    """Convert a quaternion to a 3D rotation matrix"""

    Rz = np.array([[ 2*qx*qx - 1 + 2*qw*qw, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy],
                  [2*qx*qy + 2*qw*qz, 2*qw*qw -1 + 2*qy*qy, 2*qy*qz - 2*qw*qx],
                  [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 2*qw*qw -1 + 2*qz*qz]])
    
    yaw = np.arctan2(Rz[1, 0], Rz[0, 0])
    yaw = np.degrees(yaw)
    return Rz,yaw