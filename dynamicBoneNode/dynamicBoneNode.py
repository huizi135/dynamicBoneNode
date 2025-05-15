import sys
import math
import maya.api.OpenMaya as om

PLUGIN_NODE_NAME = "dynamicBoneNode"

PLUGIN_NODE_ID = om.MTypeId()

def maya_useNewAPI():
    """
    The presence of this function tells maya what expect to be passed
    :return:
    """
    pass

#Node definition
class DynamicBoneNode(om.MPxNode):
    #class variables for attributes
    aimPoint = om.MObject()
    rootMatrix = om.MObject()
    targetOffsetMatrix = om.MObject()
    damping = om.MObject()
    stiffness = om.MObject()
    jiggleAmount = om.MObject()
    currentTime = om.MObject()
    targetPoint = om.MObject()
    
    def __init__(self):
        om.MPxNode.__init__(self)
        self.initialized = False
        self.currentPosition = om.MPoint()
        self.previousPosition = om.MPoint()
        self.previousTime = 0.0
        self.velocity = om.MVector()
        
    @staticmethod
    def creator():
        """Creator method"""
        return DynamicBoneNode()
    
    @staticmethod
    def initialize():
        uAttr = om.MFnUnitAttribute()
        nAttr = om.MFnNumericAttribute()
        xAttr = om.MFnMatrixAttribute()
        
        """Input Attribute"""
        DynamicBoneNode.aimPoint = nAttr.createPoint('aimPoint', 'ap')
        om.MPxNode.addAttribute(DynamicBoneNode.aimPoint)
        
        DynamicBoneNode.rootMatrix = xAttr.create('rootMatrix', 'rm', om.MFnMatrixAttribute.kDouble)
        om.MPxNode.addAttribute(DynamicBoneNode.rootMatrix)
        
        DynamicBoneNode.targetOffsetMatrix = xAttr.create('targetOffsetMatrix', 'tfm', om.MFnMatrixAttribute.kDouble)
        om.MPxNode.addAttribute(DynamicBoneNode.targetOffsetMatrix)
        
        xAttr.hidden = True
        xAttr.writable = False
        
        #dynamic params
        #damping
        DynamicBoneNode.damping = nAttr.create('damping', 'damping', om.MFnNumericData.kFloat, 1.0)
        om.MPxNode.addAttribute(DynamicBoneNode.damping)
        nAttr.setMin(0.0)
        nAttr.setMax(1.0)
        nAttr.keyable = True
        
        #stiffness
        DynamicBoneNode.stiffness = nAttr.create('stiffness', 'stiffness', om.MFnNumericData.kFloat, 1.0)
        om.MPxNode.addAttribute(DynamicBoneNode.stiffness)
        nAttr.keyable = True
        nAttr.setMin(0.0)
        nAttr.setMax(1.0)
        
        #jiggle amount
        DynamicBoneNode.jiggleAmount = nAttr.create('jiggleAmount', 'jiggleAmount', om.MFnNumericData.kFloat, 1.0)
        om.MPxNode.addAttribute(DynamicBoneNode.jiggleAmount)
        nAttr.keyable = True
        nAttr.setMin(0.0)
        nAttr.setMax(1.0)
        
        #time
        DynamicBoneNode.currentTime = uAttr.create('currentTime', 'ct', om.MFnUnitAttribute.kTime, 0.0)
        om.MPxNode.addAttribute(DynamicBoneNode.currentTime)
        
        #output
        DynamicBoneNode.targetPoint = nAttr.createPoint('targetPoint', 'tp')
        om.MPxNode.addAttribute(DynamicBoneNode.targetPoint)
        nAttr.writable = False
        nAttr.storable = False
        
        # attribute affects
        om.MPxNode.attributeAffects(DynamicBoneNode.rootMatrix, DynamicBoneNode.targetPoint)
        om.MPxNode.attributeAffects(DynamicBoneNode.stiffness, DynamicBoneNode.targetPoint)
        om.MPxNode.attributeAffects(DynamicBoneNode.damping, DynamicBoneNode.targetPoint)
        om.MPxNode.attributeAffects(DynamicBoneNode.jiggleAmount, DynamicBoneNode.targetPoint)
        om.MPxNode.attributeAffects(DynamicBoneNode.currentTime, DynamicBoneNode.targetPoint)
        
    def compute(self, plug, dataBlock):
        if (plug == DynamicBoneNode.targetPoint):
            aimHandle = dataBlock.inputValue(DynamicBoneNode.aimPoint)
            aimPosition = aimHandle.asFloatVector()
            aimPoint = om.MPoint(aimPosition)
            
            rootMatrixHandle = dataBlock.inputValue(DynamicBoneNode.rootMatrix)
            rootMatrix = rootMatrixHandle.asMatrix()
            
            targetOffsetMatrixHandle = dataBlock.inputValue(DynamicBoneNode.targetOffsetMatrix)
            targetOffsetMatrix = targetOffsetMatrixHandle.asMatrix()
            
            dampingHandle = dataBlock.inputValue(DynamicBoneNode.damping)
            dampingValue = dampingHandle.asFloat()
            
            stiffnessHandle = dataBlock.inputValue(DynamicBoneNode.stiffness)
            stiffnessValue = stiffnessHandle.asFloat()
            
            currentTimeHandle = dataBlock.inputValue(DynamicBoneNode.currentTime)
            currentTime = currentTimeHandle.asTime()
            
            jiggleAmountHandle = dataBlock.inputValue(DynamicBoneNode.jiggleAmount)
            jiggleAmountValue = jiggleAmountHandle.asFloat()
            
            #get aim point world space position
            #aimPoint = aimPoint * rootMatrix
            
            #get target point world space position
            targetMatrix = targetOffsetMatrix * rootMatrix
            transformMatrixFN = om.MTransformationMatrix(targetMatrix)
            targetPosition = transformMatrixFN.translation(om.MSpace.KWorld)
            targetPoint = om.MPoint(targetPosition)
            
            # reset simulation
            if currentTime.value - self.previousTime > 1.0 or currentTime.value < self.previousTime:
                self.initialized = False
                print('large time gap, simulation terminated.')
                
            if not self.initialized:
                self.previousTime = currentTime.value
                self.previousPosition = targetPoint
                self.currentPosition = targetPoint
                self.velocity = om.MVector(0, 0, 0)
                self.initialized = True
                
            #jiggle algorithm
            newPosition = self.calculateJiggle(targetPoint, dampingValue, stiffnessValue, jiggleAmountValue)
            #store node status for next computation
            self.previousPosition = om.MPoint(self.currentPosition)
            self.currentPosition = om.MPoint(newPosition)
            
            self.previousTime = currentTime.value
            
            outputHandle = dataBlock.outputValue(DynamicBoneNode.targetPoint)
            outputVector = om.MFloatVector(newPosition.x, newPosition.y, newPosition.z)
            outputHandle.setFloatVector(outputVector)
            outputHandle.setClean()
            
            dataBlock.setClean()
            
        else:
            return None
        
    def calculateJiggle(self, targetPoint, dampingValue, stiffnessValue, jiggleAmountValue):
        # jiggle point algorithm
        # semi-implicit Euler
        force = (targetPoint - self.currentPosition) * stiffnessValue
        dampingForce = self.velocity * dampingValue
        force = force -dampingForce
        self.velocity = self.velocity + force
        newPosition = self.currentPosition + self.velocity
        
        # Explicit Euler
        # velocity = (self.currentPosition - self.previousPosition) * (1 - dampingValue)
        # newPosition = self.currentPosition + velocity
        # position hold with stiffness
        newPosition += (targetPoint - newPosition) * stiffnessValue
    
        newPosition = targetPoint + (newPosition - targetPoint) * jiggleAmountValue
        return newPosition
    
def initializePlugin(mobject):
    plugin = om.MFnPlugin(mobject, 'HUIZI', '1.0')
    try:
        plugin.registerNode(PLUGIN_NODE_NAME, PLUGIN_NODE_ID, DynamicBoneNode.creator, DynamicBoneNode.initialize)
        om.MGlobal.displayInfo(f"Registered node: {PLUGIN_NODE_NAME}")
    except Exception as e:
        om.MGlobal.displayError(f"Failed to register node: {PLUGIN_NODE_NAME} - {e}")
        
# uninitialize the script plugin
def uninitializePlugin(plugin):
    pluginFn = om.MFnPlugin(plugin)
    try:
        pluginFn.deregisterNode(PLUGIN_NODE_ID)
        om.MGlobal.displayInfo(f"Deregistered command: {PLUGIN_NODE_NAME}")
    except Exception as e:
        om.MGlobal.displayError(f"Failed to deregister command: {PLUGIN_NODE_NAME} - {e}")





































    