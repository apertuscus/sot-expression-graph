import roslib
import rospy
import tf
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

# check what signals are defined now
# ... define the other signals position_obj and positionRef

# expg.position_obj.value = ((0.6625918136377457,  -0.5220206076684805,   0.5370908430327903,  0.10328669911264382),
#  (-0.12627042880607656,  0.6289750604880604,   0.7671024391130369,   0.24253855350188092),
#  (-0.7382600268938936,   -0.5760944874354131,   0.35083796008578705,   0.724250138032145),
#  (0.0, 0.0, 0.0, 1.0))
#m1= ((0.6625918136377457,  -0.5220206076684805,   0.5370908430327903,  0.3),
# (-0.12627042880607656,  0.6289750604880604,   0.7671024391130369,   0.24253855350188092),
# (-0.7382600268938936,   -0.5760944874354131,   0.35083796008578705,   1),
# (0.0, 0.0, 0.0, 1.0))
#m2= ((0.6625918136377457,  -0.5220206076684805,   0.5370908430327903,  0.1),
# (-0.12627042880607656,  0.6289750604880604,   0.7671024391130369,   0.24253855350188092),
# (-0.7382600268938936,   -0.5760944874354131,   0.35083796008578705,   1),
# (0.0, 0.0, 0.0, 1.0))

# initial position of the SoT.
#m3= (( 0.753093576084,  -0.379978278315,   0.537090843033,   0.103286699113),
#     (-0.248711481396,   0.591351373490,   0.767102439113,   0.242538553502),
#     (-0.609091671821,  -0.711280578310,   0.350837960086,   0.724250138032),
#     (              0,                0,                0,   1.000000000000))


#m3= (( 0.892233,   0.0545103,    0.448274, 0.229927),
#     ( 0.00330451,  0.991873,   -0.127189, -0.0126188),
#     ( -0.451564,   0.114964,    0.884801, 0.307811),
#     (              0,                0,                0,   1.000000000000))


m3= ((1,0,0,   0.010260575628),
     (0,1,0,  -0.096000000000),
     (0,0,1,  -0.106999500724),
     (0,0,0,   1.000000000000))

expg.position_obj.value = m3
#expg.position_obj.value = robot.rightAnkle.position.value
expg.positionRef.value = 0

# add the task corresponding to the expression graph.
# note that the contacts have already been added (to have having a flying robot).
#solver.sot.clear()
solver.push(taskExpg)

expg.displaySignals()

# expg.error.recompute(1)
# expg.jacobian.recompute(1)
# expg.jacobian.value
# go
# expg.error.value
#

from dynamic_graph.sot.core import Multiply_of_matrixHomo

# Express the desired position in the chosen frame.
invRefFrame = Inverse_of_matrixHomo("invRefFrame")
plug(robot.leftAnkle.position, invRefFrame.sin)

multHomo = Multiply_of_matrixHomo("multHomo")
plug(invRefFrame.sout, multHomo.sin1)
multHomo.sin2.value = m3
plug(multHomo.sout, expg.position_obj)

expg.setChain("b2l", "r_ankle", "l_ankle")


