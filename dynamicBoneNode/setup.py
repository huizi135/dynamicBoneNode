import os
import maya.cmds as mc
import maya.api.OpenMaya as om

pluginPath = os.path.join(os.path.dirname(__file__), "dynamicBoneNode.py")

#load necessary plugins
if not mc.pluginInfo(pluginPath, query=True, loaded=True):
    try:
        mc.loadPlugin(pluginPath)
    except:
        mc.warning(f"Could not load dynamic bone node plugin {pluginPath}")

def createDynamicBoneNode(root, target, damping=0.2, jiggleAmount=1.0, nodeName="dynamicBoneNode"):
    dynamicBoneNode = mc.createNode('dynamicBoneNode', name=nodeName)
    
    # targetOffsetMatrix = getOffsetMatrix(endJoint, rootJoint)
    # dynamicBoneNode.targetOffsetMatrix.set(targetOffsetMatrix)
    
    targetMatrix = om.MMatrix(mc.getAttr(f"{target}.worldMatrix[0]"))
    rootMatrix = om.MMatrix(mc.getAttr(f"{root}.worldMatrix[0]"))
    targetOffsetMatrix = targetMatrix * rootMatrix.inverse()
    
    # build node connections
    mc.connectAttr(f"{root}.worldMatrix[0]", f"{dynamicBoneNode}.rootMatrix")
    mc.connectAttr("time1.outTime", f"{dynamicBoneNode}.currentTime")
    
    # setup parameters
    mc.setAttr(f"{dynamicBoneNode}.damping", damping)
    mc.setAttr(f"{dynamicBoneNode}.stiffness", stiffness)
    mc.setAttr(f"{dynamicBoneNode}.jiggleAmount", jiggleAmount)
    mc.setAttr(f"{dynamicBoneNode}.targetOffsetMatrix", *targetOffsetMatrix, type="matrix")
    
    # output
    mc.connectAttr(f"{dynamicBoneNode}.targetPoint", f"{target}.translate")
    
    return dynamicBoneNode