import os
import maya.cmds as mc
import maya.api.OpenMaya as om
from . import lib

import importlib
importlib.reload(lib)

pluginPath = os.path.join(os.path.dirname(__file__), "dynamicBoneNode.py")

#load necessary plugins
if not mc.pluginInfo(pluginPath, query=True, loaded=True):
    try:
        mc.loadPlugin(pluginPath)
    except:
        mc.warning(f"Could not load dynamic bone node plugin {pluginPath}")

def createDynamicBoneNode(root, pivot, target, damping=0.2, stiffness=0.2, jiggleAmount=1.0, nodeName="dynamicBoneNode"):
    aimAxis, upAxis = lib.getAimAndUpAxis(pivot, target)
    print('Applying aim vector {0} and up vector {1}'.format(str(aimAxis), str(upAxis)))
    
    dynamicBoneNode = mc.createNode('dynamicBoneNode', name=nodeName)
    
    rootMatrix = om.MMatrix(mc.getAttr(f"{root}.worldMatrix[0]"))
    
    targetMatrix = om.MMatrix(mc.getAttr(f"{target}.worldMatrix[0]"))
    targetOffsetMatrix = targetMatrix * rootMatrix.inverse()
    
    pivotMatrix = om.MMatrix(mc.getAttr(f"{pivot}.worldMatrix[0]"))
    pivotOffsetMatrix = pivotMatrix * rootMatrix.inverse()
    
    upMatrix = lib.getUpWorldMatrix(pivot, upVector = upAxis)
    upOffsetMatrix = upMatrix * rootMatrix.inverse()
    
    aimOffsetMatrix = lib.getAimOffsetMatrix(pivot, target, aimVector=aimAxis, upVector = upAxis)
    
    # build node connections
    mc.connectAttr(f"{root}.worldMatrix[0]", f"{dynamicBoneNode}.rootMatrix")
    mc.connectAttr("time1.outTime", f"{dynamicBoneNode}.currentTime")
    mc.connectAttr(f"{pivot}.jointOrient", f"{dynamicBoneNode}.jointOrient")
    
    # setup parameters
    mc.setAttr(f"{dynamicBoneNode}.damping", damping)
    mc.setAttr(f"{dynamicBoneNode}.stiffness", stiffness)
    mc.setAttr(f"{dynamicBoneNode}.jiggleAmount", jiggleAmount)
    mc.setAttr(f"{dynamicBoneNode}.targetOffsetMatrix", *targetOffsetMatrix, type="matrix")
    mc.setAttr(f"{dynamicBoneNode}.pivotOffsetMatrix", *pivotOffsetMatrix, type="matrix")
    mc.setAttr(f"{dynamicBoneNode}.upOffsetMatrix", *upOffsetMatrix, type="matrix")
    mc.setAttr(f"{dynamicBoneNode}.aimOffsetMatrix", *aimOffsetMatrix, type="matrix")
    mc.setAttr(f"{dynamicBoneNode}.aimVector", *aimAxis)
    mc.setAttr(f"{dynamicBoneNode}.upVector", *upAxis)
    
    # output
    # mc.connectAttr(f"{dynamicBoneNode}.targetPoint", f"{target}.translate")
    mc.connectAttr(f"{dynamicBoneNode}.outputRotation", f"{pivot}.rotate")
    return dynamicBoneNode