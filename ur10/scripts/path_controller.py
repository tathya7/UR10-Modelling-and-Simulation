import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from sympy import *
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

x_plot, y_plot, z_plot = [], [], []
x_path_plot, y_path_plot, z_path_plot = [], [], []

def deg2rad(deg):
  rad = deg * pi / 180
  return rad

# Define symbolic variables
theta1, theta2, theta3, theta4, theta5, theta6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6')
a, d, alpha, theta = symbols('a d alpha theta')

dh_parameters_0_1 = {alpha: deg2rad(-90),a: 0, d: 0.128, theta: theta1}
dh_parameters_1_2 = {alpha: deg2rad(180), a: 0.6127 , d: 0.176, theta: theta2 - deg2rad(90)}
dh_parameters_2_3 = {alpha: deg2rad(180), a: 0.5716 ,  d: 0.1639, theta: theta3}
dh_parameters_3_4 = {alpha: deg2rad(90), a: 0, d: 0.1639, theta: theta4 + deg2rad(90)}
dh_parameters_4_5 = {alpha: deg2rad(-90), a: 0, d: 0.1157, theta: theta5 }
dh_parameters_5_n = {alpha: deg2rad(0), a: 0, d: 0.1922 , theta: theta6 }


A_matrix = Matrix([
    [cos(theta), -sin(theta) * cos(alpha),   sin(theta) * sin(alpha),    a * cos(theta)],
    [sin(theta), cos(theta) * cos(alpha),    -cos(theta) * sin(alpha),   a * sin(theta)],
    [0,sin(alpha), cos(alpha), d],
    [0,0,0,       1]
])

A_0_1 = N(A_matrix.subs(dh_parameters_0_1))
A_1_2 = N(A_matrix.subs(dh_parameters_1_2))
A_2_3 = N(A_matrix.subs(dh_parameters_2_3))
A_3_4 = N(A_matrix.subs(dh_parameters_3_4))
A_4_5 = N(A_matrix.subs(dh_parameters_4_5))
A_5_n = N(A_matrix.subs(dh_parameters_5_n))

A_0_2 = A_0_1 @ A_1_2
A_0_3 = A_0_2 @ A_2_3
A_0_4 = A_0_3 @ A_3_4
A_0_5 = A_0_4 @ A_4_5
A_0_n = A_0_5 @ A_5_n

jacobian_matrix = Array([
      [A_0_n[0, 3].diff(theta1), A_0_n[0, 3].diff(theta2), A_0_n[0, 3].diff(theta3), A_0_n[0, 3].diff(theta4), A_0_n[0, 3].diff(theta5), A_0_n[0, 3].diff(theta6)],
      [A_0_n[1, 3].diff(theta1), A_0_n[1, 3].diff(theta2), A_0_n[1, 3].diff(theta3), A_0_n[1, 3].diff(theta4), A_0_n[1, 3].diff(theta5), A_0_n[1, 3].diff(theta6)],
      [A_0_n[2, 3].diff(theta1), A_0_n[2, 3].diff(theta2), A_0_n[2, 3].diff(theta3), A_0_n[2, 3].diff(theta4), A_0_n[2, 3].diff(theta5), A_0_n[2, 3].diff(theta6)],
      [A_0_1[0, 2],              A_0_2[0, 2],              A_0_3[0, 2],              A_0_4[0, 2],              A_0_5[0, 2],              A_0_n[0, 2] ],
      [A_0_1[1, 2],              A_0_2[1, 2],              A_0_3[1, 2],              A_0_4[1, 2],              A_0_5[1, 2],              A_0_n[1, 2] ],
      [A_0_1[2, 2],              A_0_2[2, 2],              A_0_3[2, 2],              A_0_4[2, 2],              A_0_5[2, 2],              A_0_n[2, 2] ]
  ])


jacobian_function = lambdify([theta1,theta2,theta3,theta4,theta5,theta6],jacobian_matrix)
T_ = lambdify([theta1,theta2,theta3,theta4,theta5,theta6],A_0_n)

def End_Effector_Transformation_Matrix(q):
    return T_(q[0, 0], q[1, 0], q[2, 0], q[3, 0], q[4, 0], q[5, 0])

def Jacobian(q) :
  return jacobian_function(q[0, 0], q[1, 0], q[2, 0], q[3, 0], q[4, 0], q[5, 0])

def Jacobian_Inverse(q) :
  return np.linalg.pinv(jacobian_function(q[0, 0], q[1, 0], q[2, 0], q[3, 0], q[4, 0], q[5, 0]))


class MoveArm(Node):

    def __init__(self):
        # Initializing the parameters and creating publishers/subscribers
        delta = -1e-5

        self.q_ = np.array([delta, delta, delta, delta, delta, delta], dtype=np.float64).reshape((6, 1))

        super().__init__('move_arm_node')

        self.time_steps = 1000

        end_effector_pos = T_(self.q_[0, 0], self.q_[1, 0], self.q_[2, 0], self.q_[3, 0], self.q_[4, 0], self.q_[5, 0])

        # ref_traj = transform @ ref_traj.T
        self.x_desired = np.linspace(end_effector_pos[0, 3], 0.6, self.time_steps)
        self.y_desired = np.linspace(end_effector_pos[1, 3], 0.5, self.time_steps)
        self.z_desired = np.linspace(end_effector_pos[2, 3], 0.55, self.time_steps)

        # (0.6, 0.5, 0.5) to initial position
        self.x_desired = np.concatenate((self.x_desired, np.linspace(0.6, end_effector_pos[0, 3], self.time_steps)))
        self.y_desired = np.concatenate((self.y_desired, np.linspace(0.5, end_effector_pos[1, 3], self.time_steps)))
        self.z_desired = np.concatenate((self.z_desired, np.linspace(0.5, end_effector_pos[2, 3], self.time_steps)))

        #initial position to (0.6,-0.5, 0.5) which is the dropoff location
        self.x_desired = np.concatenate((self.x_desired, np.linspace(end_effector_pos[0, 3], 0.6, self.time_steps)))
        self.y_desired = np.concatenate((self.y_desired, np.linspace(end_effector_pos[1, 3], -0.5, self.time_steps)))
        self.z_desired = np.concatenate((self.z_desired, np.linspace(end_effector_pos[2, 3], 0.5, self.time_steps)))
        
        self.dt = 20/self.time_steps
        self.i = 0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.joint_states_sub = self.create_subscription(JointState, "/joint_states", self.joint_state_callback, qos_profile)
        
        self.publish_joint_trajectory = self.create_publisher(Float64MultiArray, "/position_controller/commands", 10)
        
        self.timer = self.create_timer(self.dt, self.move_arm)

    def move_arm(self):
        if self.i < len(self.x_desired) - 2:
            print("i: ", self.i)
            end_effector_pos = np.array(End_Effector_Transformation_Matrix(self.q_).tolist()).astype(np.float64)
            
            xi, yi, zi = np.around(end_effector_pos[0, 3], 3), np.around(end_effector_pos[1, 3], 3), np.around(end_effector_pos[2, 3], 3)
            
            

            Velocity = np.array([((self.x_desired[self.i+1] - xi)/self.dt, (self.y_desired[self.i+1] - yi)/self.dt, (self.z_desired[self.i+1] - zi)/self.dt, 0, 0, 0)]).astype(np.float64).T

            print("Velocity: ", np.around(Velocity.T, 3))
           
            q_dot = Jacobian_Inverse(self.q_) @ Velocity
            print("q_dot: ", np.around(q_dot.T, 3))

            self.q_ += q_dot * self.dt
            print("q_: ", np.around(self.q_.T, 3))

            x_plot.append(xi), y_plot.append(yi), z_plot.append(zi)
            # x_path_plot.append(self.x_desired[self.i]), y_path_plot.append(self.y_desired[self.i]), z_path_plot.append(self.z_desired[self.i]) 


            print("x, y, z: ", xi, yi, zi)
            print("x_desired, y_desired, z_desired: ", np.around(self.x_desired[self.i+1], 3), np.around(self.y_desired[self.i+1], 3), np.around(self.z_desired[self.i+1], 3))

            self.publish_joint_angles(self.q_)
            self.i += 1
            print("\n")
        else:

            fig = plt.figure()
            ax = fig.add_axes([0.1, 0.1, 0.8, 0.8], projection='3d')

            # Adjust the marker size (s) to reduce the circle size
            ax.scatter(x_plot, y_plot, z_plot, c='blue', label='End Effector Trajectory', linewidth=0.01, s=3)
            ax.set_xlabel('X-axis')
            ax.set_ylabel('Y-axis')
            ax.set_zlabel('Z-axis')
            ax.set_title('3D Plot of Trajectory')

            plt.legend()
            plt.show()
                

    def joint_state_callback(self, msg):
        # Callback function for joint state subscriber
        self.q_ = np.array([msg.position[2], msg.position[0], msg.position[1], msg.position[3], msg.position[4], msg.position[5]], dtype=np.float64).reshape((6, 1))

    def publish_joint_angles(self, joint_angles):
        angle_msg = Float64MultiArray(data=joint_angles.flatten().tolist())
        self.publish_joint_trajectory.publish(angle_msg)

        

def main(args=None):
    rclpy.init(args=args)
    node = MoveArm()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
