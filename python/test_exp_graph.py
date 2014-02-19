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
from dynamic_graph.sot.expression_graph.functions import *
from dynamic_graph.sot.expression_graph.types import *
robot.expressions = {}
createExpression(robot, PointElement('left-wrist', robot, 'left-wrist', position=(0,0,0)))
createExpression(robot, PointElement('right-wrist', robot, 'right-wrist', position=(0,0,0)))
createTask(robot, 'dist', 'left-wrist', 'right-wrist', 'distance', (0,), (0,))

#desired value
robot.features['dist'].reference.value = (0.1,)

# add the task corresponding to the expression graph.
# note that the contacts have already been added (to have having a flying robot).
solver.push(robot.tasks['dist'])

robot.tasks['dist'].displaySignals()
#type 'go'

