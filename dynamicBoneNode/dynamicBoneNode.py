import sys
import math
import maya.api.OpenMaya as om
import dynamicBoneNode.lib as lib

importlib.reload(lib)

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
    rootMatrix = om.MObject()
    pivotOffsetMatrix = om.MObject()
    upOffsetMatrix = om.MObject()
    targetOffsetMatrix = om.MObject()
    aimOffsetMatrix = om.MObject()
    
    aimVectorX = om.MObject()
    aimVectorY = om.MObject()
    aimVectorZ = om.MObject()
    aimVector = om.MObject()
    
    upVectorX = om.MObject()
    upVectorY = om.MObject()
    upVectorZ = om.MObject()
    upVector = om.MObject()
    
    pivotJointOrientX = om.MObject()
    pivotJointOrientY = om.MObject()
    pivotJointOrientZ = om.MObject()
    pivotJointOrient = om.MObject()
    
    damping = om.MObject()
    stiffness = om.MObject()
    jiggleAmount = om.MObject()
    currentTime = om.MObject()
    
    maxAngle = om.MObject()
    
    # collision
    colIterations = om.MObject()
    boneColRad = om.MObject()
    # sphere collision
    sphereCollider = om.MObject()
    sphereColMatrix = om.MObject()
    sphereColRad = om.MObject()
    # capsule collision
    capsuleCollider = om.MObject()
    capsuleColMatrixA = om.MObject()
    capsuleColMatrixB = om.MObject()
    capsupleColRadA = om.MObject()
    capsupleColRadB = om.MObject()
    
    # output
    targetPoint = om.MObject()
    outputRotationX = om.MObject()
    outputRotationY = om.MObject()
    outputRotationZ = om.MObject()
    outputRotation = om.MObject()
    
    def __init__(self):
        om.MPxNode.__init__(self)
        self.initialized = False
        self.currentPosition = om.MPoint()
        self.previousPosition = om.MPoint()
        self.previousTime = 0.0
        self.velocity = om.MVector()
        self.boneLength = None
        
    @staticmethod
    def creator():
        """Creator method"""
        return DynamicBoneNode()
    
    @staticmethod
    def getMPoint(transformationMatrix):
        transformMatrixFN = om.MTransformationMatrix(transformationMatrix)
        translation = transformMatrixFN.translation(om.MSpace.kWorld)
        return om.MPoint(translation)
    
    @staticmethod
    def initialize():
        uAttr = om.MFnUnitAttribute()
        nAttr = om.MFnNumericAttribute()
        xAttr = om.MFnMatrixAttribute()
        cAttr = om.MFnCompoundAttribute()
        
        """Input Attribute"""
        DynamicBoneNode.rootMatrix = xAttr.create('rootMatrix', 'rm', om.MFnMatrixAttribute.kDouble)
        om.MPxNode.addAttribute(DynamicBoneNode.rootMatrix)
        
        DynamicBoneNode.pivotOffsetMatrix = xAttr.create('pivotOffsetMatrix', 'pom', om.MFnMatrixAttribute.kDouble)
        om.MPxNode.addAttribute(DynamicBoneNode.pivotOffsetMatrix)
        
        DynamicBoneNode.upOffsetMatrix = xAttr.create('upOffsetMatrix', 'ufm', om.MFnMatrixAttribute.kDouble)
        om.MPxNode.addAttribute(DynamicBoneNode.upOffsetMatrix)
        
        DynamicBoneNode.targetOffsetMatrix = xAttr.create('targetOffsetMatrix', 'tfm', om.MFnMatrixAttribute.kDouble)
        om.MPxNode.addAttribute(DynamicBoneNode.targetOffsetMatrix)
        
        DynamicBoneNode.aimOffsetMatrix = xAttr.create('aimOffsetMatrix', 'afm', om.MFnMatrixAttribute.kDouble)
        om.MPxNode.addAttribute(DynamicBoneNode.aimOffsetMatrix)
        
        DynamicBoneNode.aimVectorX = nAttr.create('aimVectorX', 'aimX', om.MFnNumericData.kFloat, 1.0)
        DynamicBoneNode.aimVectorY = nAttr.create('aimVectorY', 'aimY', om.MFnNumericData.kFloat, 0.0)
        DynamicBoneNode.aimVectorZ = nAttr.create('aimVectorZ', 'aimZ', om.MFnNumericData.kFloat, 0.0)
        DynamicBoneNode.aimVector = nAttr.create('aimVector', 'aim',
                                                 DynamicBoneNode.aimVectorX,
                                                 DynamicBoneNode.aimVectorY,
                                                 DynamicBoneNode.aimVectorZ)
        om.MPxNode.addAttribute(DynamicBoneNode.aimVector)
        
        DynamicBoneNode.upVectorX = nAttr.create('upVectorX', 'upX', om.MFnNumericData.kFloat, 0.0)
        DynamicBoneNode.upVectorY = nAttr.create('upVectorY', 'upY', om.MFnNumericData.kFloat, 1.0)
        DynamicBoneNode.upVectorZ = nAttr.create('upVectorZ', 'upZ', om.MFnNumericData.kFloat, 0.0)
        DynamicBoneNode.upVector = nAttr.create('upVector', 'up',
                                                DynamicBoneNode.upVectorX,
                                                DynamicBoneNode.upVectorY,
                                                DynamicBoneNode.upVectorZ)
        om.MPxNode.addAttribute(DynamicBoneNode.upVector)
        
        DynamicBoneNode.pivotJointOrientX = uAttr.create('jointOrientX', 'joX', om.MFnUnitAttribute.kAngle, 0.0)
        DynamicBoneNode.pivotJointOrientY = uAttr.create('jointOrientY', 'joY', om.MFnUnitAttribute.kAngle, 0.0)
        DynamicBoneNode.pivotJointOrientZ = uAttr.create('jointOrientZ', 'joZ', om.MFnUnitAttribute.kAngle, 0.0)
        DynamicBoneNode.pivotJointOrient = nAttr.create('jointOrient', 'jo',
                                                        DynamicBoneNode.pivotJointOrientX,
                                                        DynamicBoneNode.pivotJointOrientY,
                                                        DynamicBoneNode.pivotJointOrientZ)
        om.MPxNode.addAttribute(DynamicBoneNode.pivotJointOrient)
        
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
        
        # Angle constraint
        DynamicBoneNode.maxAngle = uAttr.create('maxAngle', 'ma', om.MFnUnitAttribute.kAngle, math.pi/2)
        om.MPxNode.addAttribute(DynamicBoneNode.maxAngle)
        uAttr.keyable = True
        uAttr.setMin(0.0)
        uAttr.setMax(math.pi/2)
        
        #time
        DynamicBoneNode.currentTime = uAttr.create('currentTime', 'ct', om.MFnUnitAttribute.kTime, 0.0)
        om.MPxNode.addAttribute(DynamicBoneNode.currentTime)
        
        # collision
        DynamicBoneNode.boneColRad = nAttr.create("boneColRadius", "bcrad", om.MFnNumericData.kFloat, 0.0)
        om.MPxNode.addAttribute(DynamicBoneNode.boneColRad)
        nAttr.keyable = False
        nAttr.setMin(0.0)
        
        DynamicBoneNode.colIterations = nAttr.create('colIterations', 'iter', om.MFnNumericData.kShort, 5)
        om.MPxNode.addAttribute(DynamicBoneNode.colIterations)
        nAttr.keyable = False
        nAttr.setMin(0.0)
        
        # sphere collider
        DynamicBoneNode.sphereColMatrix = xAttr.create('sphereColMatrix', 'scmtx')
        xAttr.keyable = False
        
        DynamicBoneNode.sphereColRad = nAttr.create('sphereColRadius', 'scrad', om.MFnNumericData.kFloat, 0.0)
        nAttr.keyable = False
        nAttr.setMin(0.0)
        
        DynamicBoneNode.sphereCollider = cAttr.create('sphereCollider', 'sc')
        om.MPxNode.addAttribute(DynamicBoneNode.sphereCollider)
        cAttr.array = True
        cAttr.addChild(DynamicBoneNode.sphereColMatrix)
        cAttr.addChild(DynamicBoneNode.sphereColRad)
        
        # capsule collider
        DynamicBoneNode.capsuleColMatrixA = xAttr.create('capsuleColMatrixA', 'ccmtxa')
        DynamicBoneNode.capsuleColMatrixB = xAttr.create('capsuleColMatrixB', 'ccmtxb')
        DynamicBoneNode.capsuleColRadA = nAttr.create('capsuleColRadA', 'ccrada', om.MFnNumericData.kFloat, 0.0)
        nAttr.setMin(0.0)
        DynamicBoneNode.capsuleColRadB = nAttr.create('capsuleColRadB', 'ccradb', om.MFnNumericData.kFloat, 0.0)
        nAttr.setMin(0.0)
        DynamicBoneNode.capsuleCollider = cAttr.create('capsuleCollider', 'cc')
        cAttr.array = True
        cAttr.addChild(DynamicBoneNode.capsuleColMatrixA)
        cAttr.addChild(DynamicBoneNode.capsuleColMatrixB)
        cAttr.addChild(DynamicBoneNode.capsuleColRadA)
        cAttr.addChild(DynamicBoneNode.capsuleColRadB)
        om.MPxNode.addAttribute(DynamicBoneNode.capsuleCollider)
        
        # position output
        DynamicBoneNode.targetPoint = nAttr.createPoint('targetPoint', 'tp')
        om.MPxNode.addAttribute(DynamicBoneNode.targetPoint)
        nAttr.writable = False
        nAttr.storable = False
        
        # rotation output
        DynamicBoneNode.outputRotationX = uAttr.create('outputRotationX', 'oroX', om.MFnUnitAttribute.kAngle, 0.0)
        DynamicBoneNode.outputRotationY = uAttr.create('outputRotationY', 'oroY', om.MFnUnitAttribute.kAngle, 0.0)
        DynamicBoneNode.outputRotationZ = uAttr.create('outputRotationZ', 'oroZ', om.MFnUnitAttribute.kAngle, 0.0)
        DynamicBoneNode.outputRotation = nAttr.create('outputRotation', 'oro',
                                                      DynamicBoneNode.outputRotationX,
                                                      DynamicBoneNode.outputRotationY,
                                                      DynamicBoneNode.outputRotationZ)
        nAttr.writable = False
        nAttr.storable = False
        om.MPxNode.addAttribute(DynamicBoneNode.outputRotation)
        
        # attribute affects
        om.MPxNode.attributeAffects(DynamicBoneNode.rootMatrix, DynamicBoneNode.targetPoint)
        om.MPxNode.attributeAffects(DynamicBoneNode.stiffness, DynamicBoneNode.targetPoint)
        om.MPxNode.attributeAffects(DynamicBoneNode.damping, DynamicBoneNode.targetPoint)
        om.MPxNode.attributeAffects(DynamicBoneNode.jiggleAmount, DynamicBoneNode.targetPoint)
        om.MPxNode.attributeAffects(DynamicBoneNode.maxAngle, DynamicBoneNode.targetPoint)
        om.MPxNode.attributeAffects(DynamicBoneNode.currentTime, DynamicBoneNode.targetPoint)
        
        om.MPxNode.attributeAffects(DynamicBoneNode.rootMatrix, DynamicBoneNode.outputRotation)
        om.MPxNode.attributeAffects(DynamicBoneNode.stiffness, DynamicBoneNode.outputRotation)
        om.MPxNode.attributeAffects(DynamicBoneNode.damping, DynamicBoneNode.outputRotation)
        om.MPxNode.attributeAffects(DynamicBoneNode.jiggleAmount, DynamicBoneNode.outputRotation)
        om.MPxNode.attributeAffects(DynamicBoneNode.maxAngle, DynamicBoneNode.outputRotation)
        om.MPxNode.attributeAffects(DynamicBoneNode.currentTime, DynamicBoneNode.outputRotation)
        
    def compute(self, plug, dataBlock):
        if (plug == DynamicBoneNode.targetPoint or plug == DynamicBoneNode.outputRotation):
            
            rootMatrixHandle = dataBlock.inputValue(DynamicBoneNode.rootMatrix)
            rootMatrix = rootMatrixHandle.asMatrix()
            
            pivotOffsetMatrixHandle = dataBlock.inputValue(DynamicBoneNode.pivotOffsetMatrix)
            pivotOffsetMatrix = pivotOffsetMatrixHandle.asMatrix()
            
            upOffsetMatrixHandle = dataBlock.inputValue(DynamicBoneNode.upOffsetMatrix)
            upOffsetMatrix = upOffsetMatrixHandle.asMatrix()
            
            targetOffsetMatrixHandle = dataBlock.inputValue(DynamicBoneNode.targetOffsetMatrix)
            targetOffsetMatrix = targetOffsetMatrixHandle.asMatrix()
            
            aimOffsetMatrixHandle = dataBlock.inputValue(DynamicBoneNode.aimOffsetMatrix)
            aimOffsetMatrix = aimOffsetMatrixHandle.asMatrix()
            
            dampingHandle = dataBlock.inputValue(DynamicBoneNode.damping)
            dampingValue = dampingHandle.asFloat()
            
            stiffnessHandle = dataBlock.inputValue(DynamicBoneNode.stiffness)
            stiffnessValue = stiffnessHandle.asFloat()
            
            maxAngleHandle = dataBlock.inputValue(DynamicBoneNode.maxAngle)
            maxAngle = maxAngleHandle.asAngle().asDegrees()
            
            currentTimeHandle = dataBlock.inputValue(DynamicBoneNode.currentTime)
            currentTime = currentTimeHandle.asTime()
            
            jiggleAmountHandle = dataBlock.inputValue(DynamicBoneNode.jiggleAmount)
            jiggleAmountValue = jiggleAmountHandle.asFloat()
            
            aimVectorHandle = dataBlock.inputValue(DynamicBoneNode.aimVector)
            aimVector = om.MVector(aimVectorHandle.asFloat3())
            
            upVectorHandle = dataBlock.inputValue(DynamicBoneNode.upVector)
            upVector = om.MVector(upVectorHandle.asFloat3())
            
            pivotJointOrientHandle = dataBlock.inputValue(DynamicBoneNode.pivotJointOrient)
            pivotJointOrientX = pivotJointOrientHandle.child(DynamicBoneNode.pivotJointOrientX).asAngle()
            pivotJointOrientY = pivotJointOrientHandle.child(DynamicBoneNode.pivotJointOrientY).asAngle()
            pivotJointOrientZ = pivotJointOrientHandle.child(DynamicBoneNode.pivotJointOrientZ).asAngle()
            pivotJointOrient = om.MEulerRotation(pivotJointOrientX.asRadians(),
                                                 pivotJointOrientY.asRadians(),
                                                 pivotJointOrientZ.asRadians(),
                                                 om.MEulerRotation.kXYZ)
            
            # collision
            boneColRadiusHandle = dataBlock.inputValue(DynamicBoneNode.boneColRad)
            boneColRadiusValue = boneColRadiusHandle.asFloat()
            
            colIterationsHandle = dataBlock.inputValue(DynamicBoneNode.colIterations)
            colIterationsValue = colIterationsHandle.asShort()
            
            sphereColArrayHandle = dataBlock.inputArrayValue(DynamicBoneNode.sphereCollider)
            scCount = sphereColArrayHandle.__len__()
            
            capsuleColArrayHandle = dataBlock.inputArrayValue(DynamicBoneNode.capsuleCollider)
            ccCount = capsuleColArrayHandle.__len__()
            
            # get pivot point world space position
            pivotPoint = self.getMPoint(pivotOffsetMatrix * rootMatrix)
            # get up point world space position
            upPoint = self.getMPoint(upOffsetMatrix * rootMatrix)
            # get target point world space position
            targetPoint = self.getMPoint(targetOffsetMatrix * rootMatrix)
            
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
                self.boneLength = (targetPoint - pivotPoint).length()
                
            # jiggle algorithm
            newPoint = self.calculateJiggle(targetPoint, dampingValue, stiffnessValue, jiggleAmountValue)
            
            # collision calculation, Iteratate multiple times
            for _ in range(colIterationsValue):
                """
                The iterations in the code are necessary because the system is resolving multiple constraints,
                and a single pass may not fully satisfy all of them.
                Iterative methods are common in constraint-based solvers to progressively refine the solution.
                """
                # set distance constraint
                newPoint = self.addDistanceConstraint(pivotPoint, newPoint, self.boneLength)
                
                for i in range(scCount):
                    #sphere
                    sphereColArrayHandle.jumpToPhysicalElement(i)
                    sphereCollider = sphereColArrayHandle.inputValue()
                    
                    # Get the sphere collider's transformation matrix
                    sphereColTfm = om.MTransformationMatrix(
                        sphereCollider.child(DynamicBoneNode.sphereColMatrix).asMatrix())
                    spherePosition = om.MPoint(sphereColTfm.translation(om.MSpace.kWorld))
                    # Get the scale and calculate the radius
                    sphereScale = sphereColTfm.scale(om.MSpace.kWorld)
                    sphereColRadius = sphereCollider.child(DynamicBoneNode.sphereColRad).asFloat() * sphereScale[2]
                    v = newPoint - spherePosition
                    r = sphereColRadius + boneColRadiusValue
                    if v.length() < r:
                        newPoint = spherePosition + (v.normal() * r)
                
                for i in range(ccCount):
                    # capsule
                    capsuleColArrayHandle.jumpToPhysicalElement(i)
                    capsuleCollider = capsuleColArrayHandle.inputValue()
                    capsuleColATfm = om.MTransformationMatrix(capsuleCollider.child(DynamicBoneNode.capsuleColMatrixA).asMatrix())
                    capsuleColAPos = om.MPoint(capsuleColATfm.translation(om.MSpace.kWorld))
                    
                    capsuleColBTfm = om.MTransformationMatrix(capsuleCollider.child(DynamicBoneNode.capsuleColMatrixB).asMatrix())
                    capsuleColBPos = om.MPoint(capsuleColBTfm.translation(om.MSpace.kWorld))
                    capsuleColAScale = capsuleColATfm.scale(om.MSpace.kWorld)
                    capsuleColARadius = capsuleCollider.child(DynamicBoneNode.capsuleColRadA).asFloat() * \
                                        capsuleColAScale[2]
                    capsuleColBScale = capsuleColBTfm.scale(om.MSpace.kWorld)
                    capsuleColBRadius = capsuleCollider.child(DynamicBoneNode.capsuleColRadB).asFloat() * \
                                        capsuleColBScale[2]
                    
                    # implement logic
                    abVector = capsuleColBPos - capsuleColAPos
                    h = abVector.length()
                    if h < 1e - 8:
                        print("Degenerate capsule")
                        continue
                    ab = abVector.normal()
                    t = ab * (newPoint - capsuleColAPos)
                    ratio = t / h
                    
                    if ratio <= 0:
                        # Sphere A
                        v = newPoint - capsuleColAPos
                        r = capsuleColARadius + boneColRadiusValue
                        if v.length() < r:
                            newPoint = capsuleColAPos + v.normal() * r
                    elif ratio >= 1:
                        # Sphere B
                        v = newPoint - capsuleColBPos
                        r = capsuleColBRadius + boneColRadiusValue
                        if v.length() < r:
                            newPoint = capsuleColBPos + v.normal() * r
                    else:
                        # Capsule body
                        q = capsuleColAPos + ab * t
                        v = newPoint - q
                        r = capsuleColARadius * (1 - ratio) + capsuleColBRadius * ratio + boneColRadiusValue
                        if v.length() < r:
                            newPoint = q + v.normal() * r
            
            newPoint = self.addAngleConstraint(pivotPoint, targetPoint, newPoint, maxAngle)
            
            # set distance constraint
            newPoint = self.addDistanceConstraint(pivotPoint, newPoint, self.boneLength)
            # store node status for next computation
            self.previousPosition = om.MPoint(self.currentPosition)
            
            self.currentPosition = om.MPoint(newPoint)
            
            self.previousTime = currentTime.value
            
            outputHandle = dataBlock.outputValue(DynamicBoneNode.targetPoint)
            outputVector = om.MFloatVector(newPoint.x, newPoint.y, newPoint.z)
            outputHandle.setMFloatVector(outputVector)
            outputHandle.setClean()
            
            # aim constraint
            aimQuat = self.addAimConstraint(pivotPoint, self.currentPosition, upPoint,
                                            aimVector=aimVector,
                                            upVector=upVector)
            
            transformMatrixFN = om.MTransformationMatrix(rootMatrix)
            rootQuat = transformMatrixFN.rotation(asQuaternion=True)
            
            localQuat = aimQuat * rootQuat.inverse()
            
            transformMatrixFN = om.MTransformationMatrix(aimOffsetMatrix)
            offsetQuat = transformMatrixFN.rotation(asQuaternion=True)
            
            jointOrientQuat = pivotJointOrient.asQuaternion()
            
            quaternion = offsetQuat.inverse() * localQuat * jointOrientQuat.inverse()
            
            # The original method seems only get the eulerRotation with rotate order XYZ
            rotationMatrix = quaternion.asMatrix()
            eulerRotation = om.MEulerRotation.decompose(rotationMatrix, om.MEulerRotation.kXYZ)
            
            rotation = eulerRotation.asVector()
            
            outputHandle = dataBlock.outputValue(DynamicBoneNode.outputRotation)
            rotX = om.MAngle(rotation.x, om.MAngle.kRadians)
            rotY = om.MAngle(rotation.y, om.MAngle.kRadians)
            rotZ = om.MAngle(rotation.z, om.MAngle.kRadians)
            outputHandle.child(DynamicBoneNode.outputRotationX).setMAngle(rotX)
            outputHandle.child(DynamicBoneNode.outputRotationY).setMAngle(rotY)
            outputHandle.child(DynamicBoneNode.outputRotationZ).setMAngle(rotZ)
            
            outputHandle.setClean()
            
            dataBlock.setClean(plug)
            
            dataBlock.setClean(plug)
            
        else:
            return None
            
        
    def calculateJiggle(self, targetPoint, dampingValue, stiffnessValue, jiggleAmountValue):
        # jiggle point algorithm
        # semi-implicit Euler
        # self.velocity = self.currentPosition - self.previousPosition
        # force = (targetPoint - self.currentPosition) * stiffnessValue
        # dampingForce = self.velocity * dampingValue
        # force = force - dampingForce
        # self.velocity = self.velocity + force
        # newPosition = self.currentPosition + self.velocity
        
        # Explicit Euler : verlet (position based)
        velocity = (self.currentPosition - self.previousPosition) * (1 - dampingValue)
        newPosition = self.currentPosition + velocity
        # position hold with stiffness
        newPosition += (targetPoint - newPosition) * stiffnessValue
    
        newPosition = targetPoint + (newPosition - targetPoint) * jiggleAmountValue
        
        return newPosition
    
    def addDistanceConstraint(self, pivot, target, distance):
        point = pivot + (target - pivot).normal() * distance
        return om.MPoint(point)
    
    def addAimConstraint(self, pivotPoint, targetPoint, upPoint,
                         aimVector=om.MVector.kXaxisVector,
                         upVector=om.MVector.kYaxisVector):
        aimTransformation = lib.createAimTransformation(pivotPoint, targetPoint, upPoint,
                                                        aimVector=aimVector,
                                                        upVector=upVector)
        # get the rotation component
        transformMatrixFN = om.MTransformationMatrix(aimTransformation)
        aimQuat = transformMatrixFN.rotation(asQuaternion=True)
        
        return aimQuat
    
    def addAngleConstraint(self, pivotPoint, targetPoint, newPoint, maxAngleValue):
        """
        add angle constraint to restrict target point motion
        :param pivotPoint: pivot point
        :param targetPoint: target point
        :param newPoint: new target point after computing jiggle algrithm
        :param maxAngleValue: max angle in degrees
        :return: vector4: new target point with angle constraint applied
        """
        # initiate constrainedPoint
        constrainedPoint = newPoint
        
        # get the target direction
        targetDirNorm = (targetPoint - pivotPoint).normal()
        # get the new direction
        newDir = newPoint - pivotPoint
        newDirNorm = (newDir - pivotPoint).normal()
        
        angleRadians = targetDirNorm.angle(newDirNorm)
        
        angleDegrees = lib.radiansToDegrees(angleRadians)
        
        axis = (targetDirNorm ^ newDirNorm).normal()
        
        if angleDegrees > maxAngleValue:
            angleOffset = maxAngleValue - angleDegrees
            
            # different method
            # rotatedVector = lib.rotateVectorAroundAxisAngle(axis, newDir, angleOffset)
            # constrainedPoint = pivotPoint + rotatedVector
            
        return constrainedPoint

    
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





































    