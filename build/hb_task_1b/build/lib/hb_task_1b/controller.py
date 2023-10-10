import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal

hb_y = 0.0
hb_x = 0.0
hb_theta = 0.0

def angular_vel_limit(w):
	# Angular Velocity Upper bound
	if(abs(w)>3):
		if (w<0): return -3
		else:	  return 3

	else : return w

def linear_vel_limit(v):
	# Linear Velocity Upper bound
	if(abs(v)>5):
		if (v<0): return -5
		else:     return 5
	else : return v
	


class HBTask1BController(Node):

	def __init__(self):
		super().__init__('hb_task1b_controller')
		self.PoseSuscirber = self.create_subscription(Odometry, '/odom', self.odometryCb,10 )
		self.TwistPublisher = self.create_publisher(Twist, '/cmd_vel',10)

		self.vel = Twist()

		self.rate = self.create_rate(100)

		# error in the pose
		self.x_err=0
		self.y_err=0
		self.theta_err=0
		
		# goal pose
		self.x_goal=0
		self.y_goal=0
		self.theta_goal=0

		# flags
		self.flag=0
		self.reached=0

		# client for the "next_goal" service
		self.cli = self.create_client(NextGoal, 'next_goal')      
		self.req = NextGoal.Request() 
		self.index = 0
		

	def send_request(self,index):
		self.req.request_goal=index
		self.future = self.cli.call_async(self.req)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()

	def odometryCb(self,msg:Odometry):
		global hb_x, hb_y, hb_theta
		orientation_q = msg.pose.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(roll, pitch, hb_theta) = euler_from_quaternion (orientation_list)
		(hb_x, hb_y, tempz)=(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z)
		self.x_err= self.x_goal-hb_x
		self.y_err=self.y_goal -hb_y
		self.theta_err = self.theta_goal - hb_theta

		if(abs(self.x_err)>= 0.1 or abs(self.y_err) >= 0.1 or abs(self.theta_err) >= 0.1 ):
			# self.get_logger().info(f"{linear_vel_limit(self.x_err*math.cos(hb_theta)+ self.y_err*math.sin(hb_theta))}")
			self.Twist_publisher_fun(linear_vel_limit( self.x_err*math.cos(hb_theta)+ self.y_err*math.sin(hb_theta)) * 5 ,
						   			 linear_vel_limit(-self.x_err*math.sin(hb_theta)+ self.y_err*math.cos(hb_theta)) * 5 , 
									 angular_vel_limit(self.theta_err) * 5 )

		else :
			self.reached=1
			self.Twist_publisher_fun(0,0,0)
		


	def Twist_publisher_fun(self,x,y,w):
		self.vel.linear.x  = float(x)
		self.vel.linear.y  = float(y)
		self.vel.linear.z  = float(0)
		self.vel.angular.x = float(0)
		self.vel.angular.y = float(0)
		self.vel.angular.z = float(w)
		self.TwistPublisher.publish(self.vel)



	def GotoGoal(self, x_goal, y_goal, theta_goal):
		self.x_goal    =x_goal 
		self.y_goal    =y_goal
		self.theta_goal=theta_goal
		self.x_err= x_goal-hb_x
		self.y_err=y_goal -hb_y
		self.theta_err = theta_goal - hb_theta

		

def main(args=None):
	rclpy.init(args=args)
	# Create an instance of the EbotController class
	ebot_controller = HBTask1BController()
	
	# Send an initial request with the index from ebot_controller.index
	ebot_controller.send_request(ebot_controller.index)
	
	# Main loop
	while rclpy.ok():
		# Check if the service call is done
		if ebot_controller.future.done():
			try:
				# response from the service call
				response = ebot_controller.future.result()
			except Exception as e:
				ebot_controller.get_logger().infselfo(
					'Service call failed %r' % (e,))
			else:
				#########           GOAL POSE             #########
				x_goal      = response.x_goal
				y_goal      = response.y_goal
				theta_goal  = response.theta_goal
				ebot_controller.flag = response.end_of_list
				####################################################

				# ******************actual FUCNTION***************
				ebot_controller.GotoGoal(x_goal, y_goal, theta_goal)


			if(ebot_controller.reached==1): 	
				# ############     DO NOT MODIFY THIS       #########
				ebot_controller.index += 1
				if ebot_controller.flag == 1 :
					ebot_controller.index = 0
				ebot_controller.send_request(ebot_controller.index)
				# ####################################################
				ebot_controller.reached=0

		# Spin once to process callbacks
		rclpy.spin_once(ebot_controller)
	# Destroy the node and shut down ROS
	ebot_controller.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()