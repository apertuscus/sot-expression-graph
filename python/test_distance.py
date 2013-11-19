import roslib
import rospy
import numpy

## Create the robot and the solver. only useful to test the whole system.
#example to check pytohn binding
from dynamic_graph.sot.core.meta_tasks_kine import *

## Create the robot romeo.
from dynamic_graph.sot.romeo.robot import *
robot = Robot('romeo', device=RobotSimu('romeo'))

## Binds with ROS. assert that roscore is running.
from dynamic_graph.ros import *
ros = Ros(robot)

# Create a simple kinematic solver.
from dynamic_graph.sot.application.velocity.precomputed_tasks import initialize
solver = initialize ( robot )


# Alternate visualization tool
# from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer
# addRobotViewer(robot.device,small=True,small_extra=24,verbose=False)
  
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

expg = FeatureExpressionFullGraph('expg')
expg.displaySignals()


# Define a task
taskExpg = Task('taskExpg')

#define the gain
taskExpg.controlGain.value = 1

# operational point used
#opPoint = 'left-wrist'
# Get the position/Jacobian of the operational point for the dynamic entity
#plug(robot.dynamic.signal('J'+opPoint),expg.signal('Jq'))
plug(robot.dynamic.signal('position'),expg.signal('joint'))

# Associate the feature to the task:
taskExpg.add(expg.name)

# Initial position of the ankles.
rightAnklePosition = (
(   1,   0,   0,   0.010260575628),
(   0,   1,   0,  -0.096000000000),
(   0,   0,   1,   0.066999500724),
(   0,   0,   0,   1.000000000000))

leftAnklePosition = (
(  1,  0,  0,  0.010260575628),
(  0,  1,  0,  0.096000000000),
(  0,  0,  1,  0.066999500724),
(  0,  0,  0,  1.000000000000))


# Define the distance to the reference frame
expg.positionRef.value = 0

# Remove the task corresponding to the right ankle.
solver.sot.remove('romeo_task_right-ankle')
solver.sot.remove ('romeo_task_com')






#constraint the position of the right foot
taskRF=MetaTaskKine6d('rf',robot.dynamic,'right-ankle','right-ankle')
taskRF.feature.frame('desired')
gotoNd(taskRF,(0,0,0),'111000',(4.9,0.9,0.01,0.9))



# add the task corresponding to the expression graph.
# note that the contacts have already been added (to have having a flying robot).
solver.push(taskRF.task)
solver.push(taskExpg)

from dynamic_graph.sot.core import Multiply_of_matrixHomo

# Express the desired position in the chosen frame.
invRefFrame = Inverse_of_matrixHomo("invRefFrame")
#plug(robot.leftAnkle.position, invRefFrame.sin)
invRefFrame.sin.value = leftAnklePosition

multHomo = Multiply_of_matrixHomo("multHomo")
plug(invRefFrame.sout, multHomo.sin1)
multHomo.sin2.value = leftAnklePosition
plug(multHomo.sout, expg.position_obj)

mId =(( 1, 0, 0, 0),
      ( 0, 1, 0, 0),
      ( 0, 0, 1, 0),
      ( 0, 0, 0, 1))

expg.position_obj.value = mId

expg.setChain("b2l", "r_ankle", "l_ankle")

