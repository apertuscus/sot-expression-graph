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
from dynamic_graph.sot.expression_graph.expression_graph import *

# Define the point 2 point feature
expg = FeaturePointToPoint('expg')

# Define a task
taskExpg = Task('taskExpg')

#define the gain
taskExpg.controlGain.value = 1

# operational point used
opPoint1 = 'left-wrist'

# Get the position/Jacobian of the operational point for the dynamic entity
plug(robot.dynamic.signal(opPoint1),expg.signal('w_T_o1'))
plug(robot.dynamic.signal('J'+opPoint1),expg.signal('w_J_o1'))


# Associate the feature to the task:
taskExpg.add(expg.name)

# define the target position
m2= ((0.6625918136377457,  -0.5220206076684805,   0.5370908430327903,  0),
 (-0.12627042880607656,  0.6289750604880604,   0.7671024391130369,   0.3),
 (-0.7382600268938936,   -0.5760944874354131,   0.35083796008578705,   0.9),
 (0.0, 0.0, 0.0, 1.0))

# the distance towards this target position is 0
expg.reference.value = 0
expg.w_T_o2.value=m2

# add the task corresponding to the expression graph.
# note that the contacts have already been added (to avoid having a flying robot).
solver.push(taskExpg)

