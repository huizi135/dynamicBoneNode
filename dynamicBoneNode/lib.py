import math
import maya.cmds as mc
import maya.api.OpenMaya as om

OFFSET = 5.0
VECTOR_MAPPING = [om.MVector.kXaxisVector,
                  om.MVector.kYaxisVector,
                  om.MVector.kZaxisVector,
                  om.MVector.kXnegAxisVector,
                  om.MVector.kYnegAxisVector,
                  om.MVector.kZnegAxisVector]

def createAimTransformation(aimPoint, targetPoint, upPoint,
                            aimVector=om.MVector.kZaxisVector,
                            upVector=om.MVector.kXaxisVector):
    """
    Add an aim constraint to the aim point, which convert the target point movement to aim point rotation
    :param aimPoint: aim point
    :param targetPoint: current target point
    :param upPoint: this point is used to generate the up vector for the aim point
    :param aimVector: aim vector
    :param upVector: up vector
    :return:
        Quaternion: Quaternion rotation for aim point to look at the target point
    """
    # aim vector
    uVector = (targetPoint - aimPoint).normal()
    # object up vector
    vVector = (upPoint - aimPoint).normal()
    #align the up vector with the object up, the othognoal issue will be fixed later
    wVector = (uVector * vVector).normal()
    # get the orthogonal v vector by cross product u and w, the cross product order matters
    vVector = wVector * uVector
    
    # construct TBN matrix
    TBNMatrix = om.MMatrix()
    UVWMatrix = om.MMatrix()
    # assign data to UVW matrix: base is forward/up/right
    UVWMatrix.setElement(0, 0, uVector.x)
    UVWMatrix.setElement(0, 1, uVector.y)
    UVWMatrix.setElement(0, 2, uVector.z)
    
    UVWMatrix.setElement(1, 0, vVector.x)
    UVWMatrix.setElement(1, 1, vVector.y)
    UVWMatrix.setElement(1, 2, vVector.z)
    
    UVWMatrix.setElement(2, 0, wVector.x)
    UVWMatrix.setElement(2, 1, wVector.y)
    UVWMatrix.setElement(2, 2, wVector.z)
    
    # assign data to TBN matrix
    # object's local coordinate system "forward vector", "up vector" and "right vector"
    # not saying where the object is currently facing the world, but how it is structured internally
    TBNMatrix.setElement(0, 0, aimVector.x)
    TBNMatrix.setElement(0, 1, aimVector.y)
    TBNMatrix.setElement(0, 2, aimVector.z)
    
    TBNMatrix.setElement(1, 0, upVector.x)
    TBNMatrix.setElement(1, 1, upVector.y)
    TBNMatrix.setElement(1, 2, upVector.z)
    
    TBNMatrix.setElement(2, 0, (aimVector * upVector).x)
    TBNMatrix.setElement(2, 1, (aimVector * upVector).y)
    TBNMatrix.setElement(2, 2, (aimVector * upVector).z)
    
    #swap row
    #TBN * transformationMatrix = UVMatrix (target world orientation you want the object to match)
    # transformationMatrix: M = TBN inverse * UVW (the rotation needed to bring the local orientation into world alignment)
    transformationMatrix = TBNMatrix.inverse() * UVWMatrix
    
    transformationMatrixFN = om.MTransformationMatrix(transformationMatrix)
    Translation = aimPoint - om.MPoint(0, 0, 0, 1)
    transformationMatrixFN.setTranslation(Translation, om.MSpace.kWorld)
    
    return transformationMatrixFN.asMatrix()

def getAimAndUpAxis(pivot, target):
    """
    Get the aim and up/right vector based on given aim joint and target joint. If upAxisIndex is defined, the given up
    axis is the major axis in elliptical cone and correspond to major angle
    :param pivot: name of the aim joint
    :param target: name of the target joint
    :return: aim and up axis index
    """
    targetPt = om.MPoint((mc.xform(target, translation=True, q=True, ws=True)))
    pivotPt = om.MPoint((mc.xform(pivot, translation=True, q=True, ws=True)))
    aimVector = om.MVector((targetPt - pivotPt))
    pivotMatrix = om.MMatrix(mc.getAttr('{0}.worldMatrix'.format(pivot)))
    # convert aim vector to the aimJoint's local space
    aimVectorLocal = (aimVector * pivotMatrix.inverse()).normalize()
    maxValue = max(max(abs(aimVectorLocal.x), abs(aimVectorLocal.y)), abs(aimVectorLocal.z))
    if maxValue in list(aimVectorLocal):
        aimAxisIndex = list(aimVectorLocal).index(maxValue)
    else:
        aimAxisIndex = [abs(aimVectorLocal.x), abs(aimVectorLocal.y), abs(aimVectorLocal.z)].index(maxValue) + 3
    
    secondaryValue = min(min(abs(aimVectorLocal.x), abs(aimVectorLocal.y)), abs(aimVectorLocal.z))
    if secondaryValue in list(aimVectorLocal):
        upAxisIndex = list(aimVectorLocal).index(secondaryValue)
    else:
        upAxisIndex = [abs(aimVectorLocal.x), abs(aimVectorLocal.y), abs(aimVectorLocal.z)].index(
            secondaryValue
        ) + 3
    return VECTOR_MAPPING[aimAxisIndex], VECTOR_MAPPING[upAxisIndex]

def getAimOffsetMatrix(aimNode, targetNode, aimVector=(1, 0, 0), upVector=(0, 1, 0)):
    """
    get the aim offset matrix.
    Maintain the aim offset when jiggle joint is not aimed to target joint.
    :param aimNode: name of the aim node
    :param targetNode: name of the target node
    :param aimVector: aim vector
    :param upVector: up vector
    :return: aim offset matrix
    """
    aimPoint = om.MPoint(mc.xform(aimNode, translation=True, q=True, ws=True))
    targetPoint = om.MPoint((mc.xform(targetNode, translation=True, q=True, ws=True)))
    # get up point
    transformationMatrixFN = om.MTransformationMatrix(om.MMatrix())
    transformationMatrixFN.setTranslation(OFFSET * om.MVector(upVector), om.MSpace.kObject)
    upPtMatrix = transformationMatrixFN.asMatrix() * om.MMatrix(mc.getAttr('{0}.worldMatrix'.format(aimNode)))
    upPoint = om.MPoint(om.MTransformationMatrix(upPtMatrix).translation(om.MSpace.kWorld))
    aimTransformationMatrix = createAimTransformation(aimPoint, targetPoint, upPoint,
                                                      aimVector=om.MVector(aimVector),
                                                      upVector = om.MVector(upVector))
    
    aimOffsetMatrix = aimTransformationMatrix * om.MMatrix(mc.getAttr('{0}.worldMatrix'.format(aimNode))).inverse()
    
    return aimOffsetMatrix

def getUpWorldMatrix(pivot, upVector=(0, 1, 0)):
    pivotWorldMatrix = om.MMatrix(mc.getAttr(f"{pivot}.worldMatrix[0]"))
    localMatrix = om.MMatrix()
    localTranslation = om.MVector(upVector) * OFFSET
    localMatrix.setElement(3, 0, localTranslation.x)
    localMatrix.setElement(3, 1, localTranslation.y)
    localMatrix.setElement(3, 2, localTranslation.z)
    upMatrix = localMatrix * pivotWorldMatrix
    return upMatrix

def radiansToDegrees(radians):
    return radians / math.pi * 180.0

def degreesToRadians(degrees):
    return degrees / math.pi * 180.0

def rotateVectorAroundAxisAngle(axis, v, theta):
    """
    return the vector rotated by axis given by angles
    :param axis: normalized axis which the given vector rotate by
    :param v: input vector
    :param theta: rotation angles (counterclockwise)
    :return:  rotated vector
    """
    # convert theta from degrees to radians
    angle = degreesToRadians(theta)
    return v * math.cos(angle) + axis * (axis * v) * (1 - math.cos(angle)) + (axis ^ v) * math.sin(angle)































