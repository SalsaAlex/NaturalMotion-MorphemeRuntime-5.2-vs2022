------------------------------------------------------------------------------------------------------------------------
-- Functions for exporting a rockstar ragdoll and additional euphoria data.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- include other required scripts
------------------------------------------------------------------------------------------------------------------------
require "rockstar/RockstarUtilities.lua"
require "rockstar/RockstarRagdollExport.lua"
require "rockstar/RockstarEuphoriaExport.lua"

------------------------------------------------------------------------------------------------------------------------
-- forward declaration for recursive function
------------------------------------------------------------------------------------------------------------------------
local exportRockstarRagdollPhysicsJoint = nil

------------------------------------------------------------------------------------------------------------------------
--| signature: table createExportWriter(string fileName, boolean debugging)
------------------------------------------------------------------------------------------------------------------------
local createExportWriter = function(fileName, debugging)
  local writer = {}
  if debugging then
    writer.write = function(self, depth, ...)
      print(string.rep("  ", depth), unpack(arg))
    end
  else
    writer.handle = io.open(fileName, "w")
    if not writer.handle then
      app.warning(string.format("createExportWriter() : could not open file '%s' for write", fileName))
      return false
    end

    writer.write = function(self, depth, ...)
      self.handle:write(string.rep("  ", depth), unpack(arg))
      self.handle:write("\n")
    end
  end

  return writer
end

------------------------------------------------------------------------------------------------------------------------
--| signature: boolean rockstar.exportPhysicsRig(
--|   string animationSetName,
--|   string ragdollragdollFileName,
--|   string euphoriaFileName)
--| brief:
------------------------------------------------------------------------------------------------------------------------
rockstar.exportPhysicsRig = function(animationSetName, ragdollFileName, euphoriaFileName, debugOutput)
  -- assert all arguments are valid
  --
  assert(type(animationSetName) == "string" and string.len(animationSetName) > 0, "bad argument #1 to 'rockstar.exportPhysicsRig'")
  assert(type(ragdollFileName) == "string" and string.len(ragdollFileName) > 0, "bad argument #2 to 'rockstar.exportPhysicsRig'")
  assert(type(euphoriaFileName) == "string" and string.len(euphoriaFileName) > 0, "bad argument #2 to 'rockstar.exportPhysicsRig'")
  debugOutput = debugOutput or false

  local application = nmx.Application.new()
  local scene = application:getSceneByName("AssetManager")

  local physicsRigRootTxInstance = anim.getPhysicsRigDataRoot(scene, animationSetName)
  if not physicsRigRootTxInstance then
    return false
  end
    
  -- get physics root joint   
  local rootPhysicsJoint = physicsRigRootTxInstance:getFirstChild(nmx.PhysicsJointNode.ClassTypeId())
  assert(rootPhysicsJoint, "could not find physics rig root PhysicsJointNode")    
  
  local rootPhysicsJointTxInstance = rootPhysicsJoint:getFirstInstance():getParent()
  assert(rootPhysicsJointTxInstance, "could not find physics rig root PhysicsJointNode")   
  
  local physicsRigWorldTxInstance = rootPhysicsJointTxInstance:getParent()
  assert(physicsRigWorldTxInstance, "could not find physics rig world transform")   
    
  

  -- open the output file
  --
  local ragdollExportWriter = createExportWriter(ragdollFileName, debugOutput)
  if not ragdollExportWriter then
    return ragdollExportWriter, string.format("could not open file '%s' for write access", ragdollFileName)
  end

  --Want x-down, y Forward, Z right
  -- adjust the position of the physics rig so the hips worldspace transform is set to identity
  -- wrap this change in a change block so it can be undone
  --
  local blockStatus, changeBlockReference = scene:beginChangeBlock(getCurrentFileAndLine())

  local inverseWorldMatrix = nmx.Matrix.new()
  inverseWorldMatrix:invert(rootPhysicsJointTxInstance:getWorldMatrix())

  local physicsRigOwningTxInstance = physicsRigWorldTxInstance:getParent()
  local localMatrixAttribute = physicsRigOwningTxInstance:findAttribute("LocalMatrix")
  localMatrixAttribute:disconnect(false)

  localMatrixAttribute:setMatrix(inverseWorldMatrix)
  
  scene:update()

  -- export the ragdoll
  --
  local exportedPhysicsJointTxInstances = {}
  local result, error = pcall(
    rockstar.writeRagdoll,
    ragdollExportWriter,
    rootPhysicsJointTxInstance,
    rootPhysicsJoint,
    exportedPhysicsJointTxInstances)
  if not result then
    scene:rollback(changeBlockReference)
    app.error(error)
    return false, error
  end

  if ragdollExportWriter.handle then
    ragdollExportWriter.handle:close()
  end

  --Want x-right, y Forward, Z up
  local capsuleOrientationTransform2 = nmx.Matrix.new()
  capsuleOrientationTransform2:set3x3ToYRotation(0.5 * math.pi)
  inverseWorldMatrix:multiply(capsuleOrientationTransform2)
  localMatrixAttribute:setMatrix(inverseWorldMatrix)
  scene:update()
  
  local depth = 0
  -- open the euphoria file
  --
  local euphoriaExportWriter = createExportWriter(euphoriaFileName, debugOutput)
  if not euphoriaExportWriter then
    scene:rollback(changeBlockReference)
    return false, string.format("could not open file '%s' for write access", euphoriaFileName)
  end

  result, error = pcall(rockstar.writeEuphoriaRig, euphoriaExportWriter, exportedPhysicsJointTxInstances)
  if not result then
    scene:rollback(changeBlockReference)
    app.error(error)
    return false, error
  end

  -- close the euphoria file
  --
  if euphoriaExportWriter.handle then
    euphoriaExportWriter.handle:close()
  end  

  scene:rollback(changeBlockReference)
  
  app.info("Succesfully exported rigs for: " .. animationSetName)   
  app.info("Ragdoll rig: " .. ragdollFileName )
  app.info("Euphoria rig: " .. euphoriaFileName)

  return true
end