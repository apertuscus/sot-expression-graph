from dynamic_graph import plug
from dynamic_graph.sot.expression_graph.expression_graph import *
from dynamic_graph.sot.dyninv import  TaskInequality
from dynamic_graph.sot.core import *
from dynamic_graph.sot.expression_graph.types import *

"""
...
"""
def plugSignalOrValue(val, in_signal):
  try:
    plug(val, in_signal)
  except:
    in_signal.value = val

""" 
plug operational point to a feature

opmodif: operational point of the robot
index: ...
feature: the destination feature
"""
def plugFeatureToRobotOpPoint(feature, index, opmodif):
	plug(opmodif.signal('position'), feature.signal('w_T_o'+index))
	try:
		plug(opmodif.jacobian,feature.signal('w_J_o'+index))
	except :
		plug(opmodif.Jq,feature.signal('w_J_o'+index))


def plugFeatureToOpPoint(feature, index, op):
  try:
	  plug(op.ref_frame, feature.signal('w_T_o'+index))
  except :
	  plugSignalOrValue(op.ref_frame, feature.signal('w_T_o'+index))


def plugOperationalPoint(feature, index, op):
	try:
		plugFeatureToRobotOpPoint(feature, index, op.ref_frame)
	except:
		plugFeatureToOpPoint(feature, index, op)



def createFeaturePlaneVersorAngle(name, elmt1, elmt2):
	assert elmt1.typename == 'plane'
	assert elmt2.typename == 'versor'

	feature = FeaturePlaneToVersorAngle(name)

	# Get the position/Jacobian of the operational point for the dynamic entity
	plugOperationalPoint(feature, '1', elmt1)
	plugSignalOrValue(elmt1.normal,   feature.norm1)
	plugSignalOrValue(elmt1.position, feature.p1)

	#ground frame
	plugOperationalPoint(feature, '2', elmt2)
	plugSignalOrValue(elmt2.versor, feature.v2)

	feature.reference.value = 0

	return feature


def createFeatureVersorVersorAngle(name, elmt1, elmt2):
	assert elmt1.typename == 'versor'
	assert elmt2.typename == 'versor'

	feature = FeatureVersorToVersorAngle(name)

	# Get the position/Jacobian of the operational point for the dynamic entity
	plugOperationalPoint(feature, '1', elmt1)
	plugSignalOrValue(elmt1.versor, feature.v1)

	#ground frame
	plugOperationalPoint(feature, '2', elmt2)
	plugSignalOrValue(elmt2.versor, feature.v2)

	feature.reference.value = 0

	return feature

""" Creates a point to point distance feature from two PointElement objects """
def createFeaturePointPointDistance(name, elmt1, elmt2):
	assert elmt1.typename == 'point'
	assert elmt2.typename == 'point'

	feature = FeaturePointToPointDistance(name)

	# Get the position/Jacobian of the operational point for the dynamic entity
	plugOperationalPoint(feature, '1', elmt1)
	plugSignalOrValue(elmt1.position, feature.p1)

	#ground frame
	plugOperationalPoint(feature, '2', elmt2)
	plugSignalOrValue(elmt2.position, feature.p2)

	return feature


def createFeaturePointToPoint(name, elmt1, elmt2):
	assert elmt1.typename == 'point'
	assert elmt2.typename == 'point'

	feature = FeaturePointToPoint(name)

	# Get the position/Jacobian of the operational point for the dynamic entity
	plugOperationalPoint(feature, '1', elmt1)
	plugSignalOrValue(elmt1.position, feature.p1)

	#ground frame
	plugOperationalPoint(feature, '2', elmt2)
	plugSignalOrValue(elmt2.position, feature.p2)

	return feature



def createTask(name, feature, equality):
	# Define a equality task
	if equality:
		task = Task('robot_task_'+name)
		task.controlGain.value = 1

	# Define a inequality task
	else:
		dt = 0.005
		task = TaskInequality('robot_task_'+name)
		task.dt.value=dt
		task.controlGain.value = 1 * dt
	task.add(feature.name)

	return task




def createTaskAndFeaturePointToPoint(name, elmt1, elmt2, equality=True):

	feature = createFeaturePointToPoint(name, elmt1, elmt2)
	task = createTask(name, feature, equality)
	return (task, feature)

"""
Main method to create the task and the feature
"""
def createTaskAndFeature(name, elmt1, elmt2, operation, equality=True):
	if operation == 'distance':
		# line / line
		if   elmt1.typename == 'line' and elmt2.typename == 'line':
			feature = createFeatureLineLineDistance(name, elmt1, elmt2)
		# point / surface
		elif elmt1.typename == 'point' and elmt2.typename == 'surface':
			feature = createFeatureSurfacePointDistance(name, elmt2, elmt1)
		elif elmt1.typename == 'surface' and elmt2.typename == 'point':
			feature = createFeatureSurfacePointDistance(name, elmt1, elmt2)
		# point / line
		elif elmt1.typename == 'line' and elmt2.typename == 'point':
			feature = createFeatureLinePointDistance(name, elmt1, elmt2)
		elif elmt1.typename == 'point' and elmt2.typename == 'line':
			feature = createFeatureLinePointDistance(name, elmt2, elmt1)
		# point / point
		elif elmt1.typename == 'point' and elmt2.typename == 'point':
			feature = createFeaturePointPointDistance(name, elmt1, elmt2)

		# not handled.
		else :
			raise TypeError('Unable to compute the angle between ' + elmt1.typename + '/' + elmt2.typename)

	elif operation == 'angle':
		# plane / plane
		if   elmt1.typename == 'plane' and elmt2.typename == 'plane':
			feature = createFeaturePlanePlaneAngle(name, elmt1, elmt2)
		# plane / versor
		elif elmt1.typename == 'plane' and elmt2.typename == 'versor':
			feature = createFeaturePlaneVersorAngle(name, elmt1, elmt2)
		elif elmt1.typename == 'versor' and elmt2.typename == 'plane':
			feature = createFeaturePlaneVersorAngle(name, elmt1, elmt2)
		# versor / versor
		elif elmt1.typename == 'versor' and elmt2.typename == 'versor':
			feature = createFeatureVersorVersorAngle(name, elmt1, elmt2)
		else:
			raise TypeError('Unable to compute the angle between ' + elmt1.typename  + '/' + elmt2.typename)

	else:
			raise TypeError("operation " +  operation + " unknown")

	task = createTask(name, feature, equality)
	return (task, feature)

