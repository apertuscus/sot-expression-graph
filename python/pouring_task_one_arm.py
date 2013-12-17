import roslib
import rospy
import numpy

## Create the robot and the solver. only useful to test the whole system.
#example to check pytohn binding
from dynamic_graph.sot.core.meta_tasks_kine import *

# Create the robot romeo.
#from dynamic_graph.sot.hrp2.sot_hrp2_14_controller import *
#from dynamic_graph.sot.hrp2_14.prologue import *

### Create the robot romeo.
from dynamic_graph.sot.romeo.robot import *
robot = Robot('romeo', device=RobotSimu('romeo'))

# Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a simple kinematic solver.
#from dynamic_graph.sot.application.velocity.precomputed_meta_tasks import initialize
#from dynamic_graph.sot.application.velocity.solverkine_precomputed_tasks import initialize
from dynamic_graph.sot.dyninv import SolverKine
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
solver = initialize ( robot, SolverKine )
	
#-------------------------------------------------------------------------------
#----- MAIN LOOP ---------------------------------------------------------------
#-------------------------------------------------------------------------------
# define the macro allowing to run the simulation.
from dynamic_graph.sot.core.utils.thread_interruptible_loop import loopInThread,loopShortcuts
dt=5e-3
@loopInThread
def inc():
    robot.device.increment(dt)

runner=inc()
[go,stop,next,n]=loopShortcuts(runner)


# --- CONTACTS
# define contactLF and contactRF
for name,joint in [ ['LF','left-ankle'], ['RF','right-ankle' ] ]:
    contact = MetaTaskKine6d('contact'+name,robot.dynamic,name,joint)
    contact.feature.frame('desired')
    contact.gain.setConstant(10)
    contact.keep()
    locals()['contact'+name] = contact
# ---- TASKS -------------------------------------------------------------------




################################################################################
# Create the FeatureExpressionGraph and the corresponding task
from dynamic_graph import plug
from dynamic_graph.sot.core import *
from dynamic_graph.sot.expression_graph.expression_graph import *
from dynamic_graph.sot.dyninv import  TaskInequality


# Second set of task.
# rh_center_(x,y) + delta << bottle_(x,y) << rh_center_(x,y) + delta

# rh_center_(z) + delta << bottle_(z) << rh_center_(z) + delta

# angle //right hand == bottle
# -pi/2 << dot(bottle_normal, World_Z_axis) << pi/2


#def setFeatureOpPointModifier(feature, index, oppoint):
#  plug(robot.dynamic.signal(opPoint),    feature.signal('w_T_o'+index))
#  plug(robot.dynamic.signal('J'+opPoint),feature.signal('w_J_o'+index))

def setFeatureOpPoint2(feature, index, opmodif):
  plug(opmodif.position,    feature.signal('w_T_o'+index))
  plug(opmodif.jacobian,feature.signal('w_J_o'+index))


# operational point used
# TODO:
#r_gripper = OpPointModifier('r_gripper')
#opPoint = 'right-wrist'
#plug(robot.dynamic.signal(opPoint),     r_gripper.positionIN)
#plug(robot.dynamic.signal('J'+opPoint), r_gripper.jacobianIN)
#poseRPY = PoseRollPitchYawToMatrixHomo('poseRPY')
# coordinates of the repo
# poseRPY.sin.value =(0.09, 0, -0.02, -numpy.pi / 2., 0, 0)
#poseRPY.sin.value =(0.09, 0, 0.04, -numpy.pi / 2., 0, 0)
#poseRPY.sout.recompute(robot.dynamic.position.time + 1)
#r_gripper.setTransformation(poseRPY.sout.value)

r_gripper = robot.frames['rightGripper']

#######################################################""
## Angle of the "bottle" normal wrt world

# Get the position/Jacobian of the operational point for the dynamic entity
angleBottleZ = FeatureVersorToVersor('angleBottleZ')
setFeatureOpPoint2(angleBottleZ, '1', r_gripper)
#plug(robot.dynamic.signal(opPoint),    angleBottleZ.signal('w_T_o1'))
#plug(robot.dynamic.signal('J'+opPoint),angleBottleZ.signal('w_J_o1'))
angleBottleZ.v1.value = (1,0,0)

angleBottleZ.w_T_o2.value = ((1,0,0,0), (0,1,0,0), (0,0,1,0), (0,0,0,1))
angleBottleZ.v2.value = (0,0,1)

angleBottleZ.reference.value = 1.5

# Define a task
taskAngleBottle = Task('taskAngleBottle')
taskAngleBottle.controlGain.value = 1
taskAngleBottle.add(angleBottleZ.name)


#######################################################""
## The bottle can only rotate around its Y axis
planBottleY = FeatureAngleBtwPlaneAndVersor('planBottleY')

# Get the position/Jacobian of the operational point for the dynamic entity
planBottleY.w_T_o1.value = ((1,0,0,0), (0,1,0,0), (0,0,1,0), (0,0,0,1))
planBottleY.norm1.value = (0,0,1)
planBottleY.p1.value = (0,0,0)

#ground frame
setFeatureOpPoint2(planBottleY, '2', r_gripper)
planBottleY.v2.value = (0,1,0)
planBottleY.reference.value = 0


# Define a task
taskPlanBottleY = Task('taskPlanBottleY')
taskPlanBottleY.controlGain.value = 5
taskPlanBottleY.add(planBottleY.name)


#######################################################""
## position of the end effector (Z)
positionZ = FeaturePointToPoint('positionZ')

## Get the position/Jacobian of the operational point for the dynamic entity
setFeatureOpPoint2(positionZ, '1', r_gripper)
positionZ.p1.value = (0,0,0)
positionZ.signal('w_T_o2').value = ((1,0,0,0), (0,1,0,0), (0,0,1,0), (0,0,0,1))
positionZ.p2.value = (0, 0, 0.8)

positionZ.reference.value = (0,)
positionZ.selec.value ='100'

## Define a task
taskPositionZ = Task('taskPositionZ')

##define the gain
taskPositionZ.controlGain.value = 1

## Associate the feature to the task:
taskPositionZ.add(positionZ.name)


#######################################################""
## position of the end effector XY
positionXY = FeaturePointToPoint('positionXY')


## operational point used
## Get the position/Jacobian of the operational point for the dynamic entity
setFeatureOpPoint2(positionXY, '1', r_gripper)
positionXY.reference.value = (0,0,0)


positionXY.signal('w_T_o2').value = ((1,0,0,0), (0,1,0,0), (0,0,1,0), (0,0,0,1))
positionXY.p2.value = (0, 0, 0)

positionXY.selec.value = '011'
positionXY.reference.value = (-0.3,0.1)

## Define a task
taskPositionXY = Task('taskPositionXY')

## Associate the feature to the task:
taskPositionXY.add(positionXY.name)
taskPositionXY.controlGain.value = 1


## ADD The tasks.
solver.push(taskPositionXY)
solver.push(taskPositionZ)

solver.push(taskPlanBottleY)
solver.push(taskAngleBottle)

def pour():
  angleBottleZ.reference.value = 2.0


