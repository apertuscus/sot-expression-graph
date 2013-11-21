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
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer
addRobotViewer(robot.device,small=True,small_extra=24,verbose=False)

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

expg = FeaturePointToPoint('expg')
expg.displaySignals()


# Define a task
taskExpg = Task('taskExpg')

#define the gain
taskExpg.controlGain.value = 1

# operational point used
opPoint1 = 'left-wrist' 
opPoint2 = 'right-wrist'
# Get the positions/Jacobians of the operational points for the dynamic entity
plug(robot.dynamic.signal(opPoint1),expg.signal('w_T_o1'))
plug(robot.dynamic.signal('J'+opPoint1),expg.signal('w_J_o1'))
plug(robot.dynamic.signal(opPoint2),expg.signal('w_T_o2'))
plug(robot.dynamic.signal('J'+opPoint2),expg.signal('w_J_o2'))

# Associate the feature to the task:
taskExpg.add(expg.name)

#desired value
expg.reference.value = 0.1 

# add the task corresponding to the expression graph.
# note that the contacts have already been added (to have having a flying robot).
solver.push(taskExpg)

expg.displaySignals()
#type 'go'

