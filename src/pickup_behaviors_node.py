#!/usr/bin/env python
import py_trees
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from pick_up_objects_task.srv import Waypoint, WaypointRequest
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from time import sleep
from py_trees.composites import Sequence , Parallel , Selector
from py_trees import logging as log_tree 
from py_trees.decorators import Inverter , Retry , Timeout
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf
import numpy as np

# Behavior for calling `check_object` task and if True, store object name to Blackboard

class CheckObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckObject, self).__init__(name)
        self.current_pose = np.array([0, 0, 0])
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("object_name",
                                      access=py_trees.common.Access.WRITE)
        
    def distance(self, p1, p2):
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5
    

    def setup(self):
        self.logger.debug("  %s [CheckObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/check_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/check_object', Trigger)
            self.logger.debug(
                "  %s [CheckObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [CheckObject::setup() ERROR!]" % self.name)


    def initialise(self):
        self.logger.debug("  %s [CheckObject::initialise()]" % self.name)

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/check_object".format(self.name))
            # sleep(2)
            resp = self.server(TriggerRequest())
            # if object is found, store object name to blackboard
            if resp.success:
                self.blackboard.object_name = resp.message
                return py_trees.common.Status.SUCCESS 
            # if object is not found, return running
            else:   
                return py_trees.common.Status.FAILURE   
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/check_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [CheckObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `get_object`
class GetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(GetObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key( "let_beer" ,  access= py_trees.common.Access.READ)
        self.blackboard.register_key( "let_coke" , access= py_trees.common.Access.READ)
        self.blackboard.register_key( "object_name" , access= py_trees.common.Access.READ)
        self.blackboard.register_key( "next_waypoint" , access= py_trees.common.Access.WRITE)
    
    def setup(self):
        self.logger.debug("  %s [GetObject::setup()]" % self.name)

        rospy.wait_for_service('/manage_objects/get_object')
        self.move_goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/get_object', Trigger)
            self.logger.debug(
                "  %s [GetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [GetObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [GetObject::initialise()]" % self.name)
        self.object_name = self.blackboard.object_name
    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/get_object".format(self.name))
            sleep(2)
            resp = self.server(TriggerRequest())
            if resp.success:
                goal = PoseStamped()
                bool = self.blackboard.object_name == "coke"
                if( self.blackboard.object_name == "coke"):
                    goal.pose.position.x = self.blackboard.let_coke[0]
                    goal.pose.position.y = self.blackboard.let_coke[1]
                    self.blackboard.next_waypoint = self.blackboard.let_coke
                    print("Coke Found got ",self.blackboard.next_waypoint)
                    self.move_goal_pub.publish(goal)

                elif(self.object_name == "beer"):
                    print("Beer Found go to" , self.blackboard.let_beer[0] , self.blackboard.let_beer[1])
                    goal.pose.position.x = self.blackboard.let_beer[0]
                    goal.pose.position.y = self.blackboard.let_beer[1]
                    self.blackboard.next_waypoint =self.blackboard.let_beer
                    self.move_goal_pub.publish(goal)
                return py_trees.common.Status.SUCCESS
            
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/get_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [GetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# Behavior for calling `let_object`
class LetObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(LetObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key( "num_obj_coll", access= py_trees.common.Access.WRITE)
        self.blackboard.register_key( "num_obj_coll", access= py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
        rospy.wait_for_service('/manage_objects/let_object')
        try:
            self.server = rospy.ServiceProxy(
                '/manage_objects/let_object', Trigger)
            self.logger.debug(
                "  %s [LetObject::setup() Server connected!]" % self.name)
        except rospy.ServiceException as e:
            self.logger.debug("  %s [LetObject::setup() ERROR!]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)
        self.num_obj_coll = self.blackboard.num_obj_coll

    def update(self):
        try:
            self.logger.debug(
                "  {}: call service /manage_objects/let_object".format(self.name))
            resp = self.server(TriggerRequest())
            print("Object Status in let object " , resp)
            if resp.success:
                self.num_obj_coll = self.num_obj_coll + 1
                self.blackboard.num_obj_coll = self.num_obj_coll
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug(
                "  {}: Error calling service /manage_objects/let_object".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [LetObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

# TODO: Create any other required behavior like those to move the robot to a point, 
#       add or check elements in the blackboard, ...

# Behiviour to check if exploration is done or not
class Check_Exploration_Done(Behaviour):
  def __init__(self, name):
    super(Check_Exploration_Done, self).__init__(name)

    self.blackboard = self.attach_blackboard_client(name=self.name)
    self.blackboard.register_key( "exploration_done", access= py_trees.common.Access.READ)
    self.blackboard.register_key( "exploration_done", access= py_trees.common.Access.WRITE)

  def setup(self):
    self.logger.debug(f"Check_Exploration_Done::setup {self.name}")
    self.exploration_done = False
    self.blackboard.exploration_done = self.exploration_done

  def initialise(self):
    self.logger.debug(f"Check_Exploration_Done::initialise {self.name}")
    self.exploration_done = self.blackboard.exploration_done

  def update(self):
    self.logger.debug(f"Check_Exploration_Done::update {self.name}")
    if(self.exploration_done):
      print("Explore Done" , self.exploration_done)
      return Status.SUCCESS
    # check if miniman wavepoint is visited or not
    return Status.FAILURE

  def terminate(self, new_status):
    self.logger.debug(f"Check_Exploration_Done::terminate {self.name} to {new_status}")
# Behavior to explore the environment
class Explore(Behaviour):
  def __init__(self, name):
    super(Explore, self).__init__(name)

    self.blackboard = self.attach_blackboard_client(name=self.name)
    self.blackboard.register_key( "locations", access= py_trees.common.Access.WRITE) # list of waypoints

    self.blackboard.register_key( "next_waypoint", access= py_trees.common.Access.WRITE) # goal point 

    self.blackboard.register_key( "let_beer", access= py_trees.common.Access.WRITE)
    self.blackboard.register_key( "let_beer", access= py_trees.common.Access.READ)

    self.blackboard.register_key( "let_coke", access= py_trees.common.Access.READ)
    self.blackboard.register_key( "let_coke", access= py_trees.common.Access.WRITE)

    self.blackboard.register_key( "num_obj_explored", access= py_trees.common.Access.WRITE)
    self.blackboard.register_key( "num_obj_explored", access= py_trees.common.Access.READ)

    self.blackboard.register_key( "num_obj_coll", access= py_trees.common.Access.WRITE)
    self.blackboard.register_key( "num_obj_coll", access= py_trees.common.Access.READ)

    self.blackboard.register_key( "explor_index", access= py_trees.common.Access.WRITE)
    self.blackboard.register_key( "explor_index", access= py_trees.common.Access.READ)
  
  def setup(self):
    self.logger.debug(f"Explore::setup {self.name}")
    self.num_obj_explored  = 0
    self.num_obj_coll = 0 
    self.next_waypoint = []
    self.explor_index = 0
    self.blackboard.explor_index = self.explor_index
    self.locations = [ (1.25, 0.5), (1.25, -1.25), (0, -1.25),(-0.5, 1.25) ,(-1.25, 0.5)]
    self.let_beer = (-1.5, -1.5)
    self.let_coke = (1.5, 1.5)
    self.blackboard.locations = self.locations
    self.blackboard.let_beer = self.let_beer
    self.blackboard.let_coke = self.let_coke
    self.blackboard.num_obj_explored = self.num_obj_explored
    self.blackboard.num_obj_coll = self.num_obj_coll
    self.blackboard.next_waypoint = self.next_waypoint

    
    self.move_goal_pub = rospy.Publisher(
            '/move_base_simple/goal', PoseStamped, queue_size=10)
    self.logger.debug(
                "  %s [Explore::setup() Server connected!]" % self.name)



  def initialise(self):
    self.logger.debug(f"Explore::initialise {self.name}")
    self.num_obj_explored = self.blackboard.num_obj_explored
    self.num_obj_coll = self.blackboard.num_obj_coll

    
  def update(self):
    self.logger.debug(f"Explore::update {self.name}")
    sleep(2)
    if self.num_obj_explored >= len(self.locations) or self.num_obj_coll >= 2:
    
        print("Explore Done" )
        self.logger.debug("Exploration Finished all explored or 2 object are collected!")
        return Status.FAILURE
    
    else :
      goal = PoseStamped()
      self.next_waypoint = self.locations[self.explor_index]
      self.explor_index += 1
      self.blackboard.explor_index = self.explor_index
      self.blackboard.next_waypoint = self.next_waypoint ## input to path planner 
      self.num_obj_explored += 1
      self.blackboard.num_obj_explored = self.num_obj_explored

      goal.pose.position.x = self.next_waypoint[0]
      goal.pose.position.y = self.next_waypoint[1]

      self.move_goal_pub.publish(goal)
      return Status.SUCCESS

  def terminate(self, new_status):
    self.logger.debug(f"Explore::terminate {self.name} to {new_status}")
# Behavior to set exploration done
class SetExploration(Behaviour):
  def __init__(self, name):
    super(SetExploration, self).__init__(name)
    self.blackboard = self.attach_blackboard_client(name=self.name)
    self.blackboard.register_key( "exploration_done", access= py_trees.common.Access.WRITE)
    self.blackboard.register_key( "exploration_done", access= py_trees.common.Access.READ)
  def setup(self):
    self.logger.debug(f"Explore::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"Explore::initialise {self.name}")
    self.exploration_done = self.blackboard.exploration_done
    
  def update(self):
    self.logger.debug(f"Explore::update {self.name}")
    sleep(2)
    self.blackboard.exploration_done = True
    print("Explore Done" , self.exploration_done)
    self.logger.debug("Exploration Finished all explored or 2 object are collected!")
    return Status.FAILURE
    

  def terminate(self, new_status):
    self.logger.debug(f"Explore::terminate {self.name} to {new_status}")
# Behavior to follow the path
class Path_Follower(Behaviour):
  def __init__(self, name):
    super(Path_Follower, self).__init__(name)
    
    self.blackboard = self.attach_blackboard_client(name=self.name)

    self.blackboard.register_key( "next_waypoint", access= py_trees.common.Access.WRITE)
    self.blackboard.register_key( "next_waypoint", access= py_trees.common.Access.READ)

    self.blackboard.register_key( "num_obj_coll", access= py_trees.common.Access.WRITE)
    self.blackboard.register_key( "num_obj_coll", access= py_trees.common.Access.READ)

    self.blackboard.register_key( "num_obj_explored", access= py_trees.common.Access.WRITE)
    self.blackboard.register_key( "no_obj_cxp", access= py_trees.common.Access.READ)

    rospy.Subscriber('/odom', Odometry, self.odom_callback)
    self.distance_threshold = 0.15
  def odom_callback(self, data):
        self.robot_pose = (data.pose.pose.position.x,
                           data.pose.pose.position.y)

  def distance(self, p1, p2):
        return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**0.5
  
  def setup(self):
    self.logger.debug(f"Path_Follower::setup {self.name}")
    
  def initialise(self):
    self.logger.debug(f"Path_Follower::initialise {self.name}")
    self.next_waypoint = self.blackboard.next_waypoint 
  def update(self):
    self.logger.debug(f"Path_Follower::update {self.name}")
    
    try:
        # success if goal point is reached
        print("Goal Point", self.next_waypoint)
        if self.distance(self.robot_pose, self.next_waypoint) < self.distance_threshold:
            self.nex_waypoint = None
            print("Goal Point Reached") 
            return Status.SUCCESS
        # running while following the path
        else:   
            return Status.RUNNING
    except:
        self.logger.debug(
            "  {}: Error Following path".format(self.name))
        return py_trees.common.Status.FAILURE

  def terminate(self, new_status):
    self.logger.debug(f"Path_Follower::terminate {self.name} to {new_status}")
  
if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")

    # Create Behaviors
    check_object = CheckObject("check_object")
    get_object = GetObject("get_object")
    let_object = LetObject("let_object")

    explore_finish = Check_Exploration_Done("explore_finish")
    check_exploration = Inverter( "Invertor" , explore_finish )
    set_exploration = SetExploration("set_explo_always_fails")
    explore_list  =   Explore("explore_list")
    path_follower = Path_Follower("plan and follow path")
    path_follower2= Path_Follower("plan and follow_path2")
    # Create a root for the tree
    root = Sequence(name="Pick and Place", memory=True)
    log_tree.level = log_tree.Level.DEBUG
    root.add_children(
       
        [
            check_exploration,
            Selector("Explore_Selec", memory=True, children=[explore_list, set_exploration]),
            # parallel_behavior,
            
            Timeout(name ="path_follower" ,duration =  1000.0 , child =path_follower ), 
            check_object,
            
            Retry( "retry_get_obje" , get_object , 2),
            Timeout(name ="path_follower" ,duration =  1000.0 , child =path_follower2 ),
            let_object
        ]
    )

    root.setup_with_descendants() 
    py_trees.display.render_dot_tree(root)

    while check_exploration.status != Status.FAILURE:
        root.tick_once()
        sleep(2)

    print("Pick and Place Finished")
  

    

