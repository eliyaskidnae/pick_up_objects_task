#!/usr/bin/env python
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, TriggerResponse
from pick_up_objects_task.srv import Waypoint, WaypointResponse
from std_msgs.msg import MultiArrayLayout
import rospy
import random
import copy
import sys

class ManageObject():
    def __init__(self, models_path):
        # If stage 4 is used
        # self.locations = [(1.5, -1.2), (0.1, -1.8),
        #                   (-1, 2), (-2, 0.1), (0.5, 0.2)]
        # self.let_beer = (-1.7, -2)
        # self.let_coke = (2, 1.7)
        
        # If stage 3 is used

        self.locations = [ (1.25, 0.5),(1.25, -1.25), (0.0, -1.25),(-0.5, 1.25),(-1.25, 0.5)]
        self.let_beer = (-1.5, -1.5)
        self.let_coke = (1.5, 1.5)
        self.waypoints = [] 
        self.model_coke = models_path + '/models/coke_can/model.sdf'                          
        self.model_beer = models_path + '/models/beer/model.sdf'
        self.beer_loc = None
        self.coke_loc = None
        self.robot_pose = None
        self.beer_on_robot = False
        self.coke_on_robot = False

        if not self.setup_escenario():
            print("Error setting up scenario")
            exit()
            
        print("model started")
        self.pub_set_model_state = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=1)
        print("Service server started")
        
        server_check = rospy.Service('~check_object', Trigger,
                                     self.handle_check_object)
        
        server_take  =  rospy.Service('~get_object', Trigger,
                                    self.handle_take_object)
        
        server_let   =  rospy.Service('~let_object', Trigger,
                                   self.handle_let_object)
        
        server_waypoints = rospy.Service('~get_waypoint', Waypoint,
                                         self.handle_nextwaypoint)
        
        server_palce_waypoint = rospy.Service('~place_waypoint', Waypoint,
                                         self.handel_place_waypoint)
        
        subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.Timer(rospy.Duration(0.1), self.iterate)

    def spawn_model(self, model_name, model_xml, p):
        
        pose = Pose()
        pose.position.x = p[0]
        pose.position.y = p[1]
        pose.position.z = 0.1
        resp = None
        try:
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            spawn_model_client = rospy.ServiceProxy(
                '/gazebo/spawn_sdf_model', SpawnModel)
            
            req = SpawnModelRequest()
            
            req.model_name = model_name
            req.model_xml = open(
                model_xml, 'r').read()
            req.robot_namespace = '/'
            req.initial_pose = pose
            req.reference_frame = 'ground_plane'
            resp = spawn_model_client(req)
        except:
            print("Error spawning ", model_name)

        # print("success", resp.success)
        # print("status", resp.status_message)
        return resp.success
    ##

    def setup_escenario(self):
        self.beer_loc = self.locations[random.randint(
            0, len(self.locations)-1)]
        # self.beer_loc = self.locations[1]
        self.spawn_model('beer', self.model_beer, self.beer_loc)
        while self.coke_loc is None or self.coke_loc == self.beer_loc:
            self.coke_loc = self.locations[random.randint(
                0, len(self.locations)-1)]
            # self.coke_loc = self.locations[2]
        return self.spawn_model('coke', self.model_coke, self.coke_loc)
    

    def handel_place_waypoint(self, req ):
        print("handle palce object")
        res = WaypointResponse()

        if self.beer_on_robot:
            rospy.loginfo("Placing beer" )
            res.x = self.let_beer[0]
            res.y = self.let_beer[1]
            res.valid = True
            return res
        
        elif self.coke_on_robot:
            rospy.loginfo("Placing coke" )
            res.x = self.let_coke[0]
            res.y = self.let_coke[1]
            res.valid = True
            return res
        
        else:
            rospy.logwarn("No more object.")
            res.x = 0
            res.y = 0
            res.valid = False
            return res
        

    def handle_nextwaypoint(self,req):
        # Dummy implementation - replace with actual logic to get waypoints
    
        res = WaypointResponse()
        if self.locations:
           
            next_waypoint = self.locations.pop(0)
            print("Next waypoint: ", next_waypoint)
            res.x = next_waypoint[0]
            res.y = next_waypoint[1]
            res.valid = True
            return res
        
        else:

            rospy.logwarn("No more waypoints.")
            res.x = 0
            res.y = 0
            res.valid = False
            return res
        
    def handle_check_object(self, req):
        print("Checking object")
        ret = TriggerResponse()

        if self.distance(self.beer_loc, self.robot_pose) < 0.35:
            ret.success = True
            ret.message = 'beer'

        elif self.distance(self.coke_loc, self.robot_pose) < 0.35:
            ret.success = True
            ret.message = 'coke'
            
        else:
            ret.success = False
            ret.message = ''

        return ret
    
    def handle_take_object(self, req):
        ret = TriggerResponse()
        if self.distance(self.beer_loc, self.robot_pose) < 0.35:
            self.beer_on_robot = True
            ret.success = True
        elif self.distance(self.coke_loc, self.robot_pose) < 0.35:
            self.coke_on_robot = True
            ret.success = True
        else:
            print("Error! No objects close")
            ret.success = False
        return ret

    def handle_let_object(self, req):
        ret = TriggerResponse()
        print("Letting beer" , self.beer_on_robot)
        print("Letting cok" , self.coke_on_robot)
        if self.beer_on_robot:
            self.beer_on_robot = False
            ret.success = True
            model_state = ModelState()
            model_state.model_name = 'beer'
            model_state.pose.position.x = self.robot_pose[0] - 0.25
            model_state.pose.position.y = self.robot_pose[1] - 0.25
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            # self.coke_loc = copy.copy(self.robot_pose + 0.3)
            self.pub_set_model_state.publish(model_state)

        elif self.coke_on_robot:
            self.coke_on_robot = False
            ret.success = True
            model_state = ModelState()
            model_state.model_name = 'coke'
            model_state.pose.position.x = self.robot_pose[0] + 0.25
            model_state.pose.position.y = self.robot_pose[1] + 0.25
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            # self.coke_loc = copy.copy(self.robot_pose + 0.3)
            self.pub_set_model_state.publish(model_state)

        else:
            print("Error! No objects grasped.")
            ret.success = False
        return ret

    def odom_callback(self, data):
        self.robot_pose = (data.pose.pose.position.x,
                           data.pose.pose.position.y)

    def distance(self, p1, p2):
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5

    def iterate(self, event):
        if self.coke_on_robot:
            model_state = ModelState()
            model_state.model_name = 'coke'
            model_state.pose.position.x = self.robot_pose[0] 
            model_state.pose.position.y = self.robot_pose[1]
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            self.pub_set_model_state.publish(model_state)
            self.coke_loc = copy.copy(self.robot_pose)

        elif self.beer_on_robot:
            model_state = ModelState()
            model_state.model_name = 'beer'
            model_state.pose.position.x = self.robot_pose[0]
            model_state.pose.position.y = self.robot_pose[1]
            model_state.pose.position.z = 0.2
            model_state.reference_frame = 'ground_plane'
            self.pub_set_model_state.publish(model_state)
            self.beer_loc = copy.copy(self.robot_pose)


if __name__ == '__main__':
    models_path = './'
    print(sys.argv)
    if len(sys.argv) >= 2:
        models_path = sys.argv[1]
     
    print("Path: ", models_path)
    rospy.init_node('spawn_model')
    check_object = ManageObject(models_path)
    rospy.spin()
    