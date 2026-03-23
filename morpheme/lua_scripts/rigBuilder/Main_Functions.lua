------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2012 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- include other required scripts
------------------------------------------------------------------------------------------------------------------------
require [[rigBuilder/Options.lua]]
require [[rigBuilder/Rig_Functions.lua]]
require [[rigBuilder/Character_Functions.lua]]
require [[rigBuilder/CollisionSets_Functions.lua]]


------------------------------------------------------------------------------
--  Copy all physics limbs
------------------------------------------------------------------------------
main_copyRigs = function(templateAnimationRigPath,templatePhysicsRigPath, newPhysicsRigPath , animSet)

 
  local application = nmx.Application.new()
  local scene = application:getSceneByName("AssetManager")
  local templateName = "Template"
  local typeId = application:lookupTypeId("CustomPhysicsEngineDataNode")

  -- create template rig to copy from
  if not(animsSetExists(templateName))then
    local templateAnimSet = createAnimSet{Name = templateName, Format = preferences.get("DefaultAnimationFormat"), Rig = templateAnimationRigPath}
  end

  local setTemplateCharacterType = anim.setAnimSetCharacterType(templateName, "Physics")  
  local set = anim.setPhysicsRigPath(templatePhysicsRigPath, templateName, true)

  local setTempalte = anim.setAnimSetTemplate(animSet, templatePath, defaultTemplateAnimRigPath, defaultTemplatePhysicsRigPath, false)
  autoMapAnimationTemplateMapping(animSet, 0)
  
  -- check to see what character type the anim set is
  local animSetType = anim.getAnimSetCharacterType(animSet)
  if(animSetType == "Animation")then
    local setPhysics = anim.setAnimSetCharacterType(animSet, "Physics")
  end
  
  app.info("Set anim set to physics")
  -- create new physics rig from the template rig and set the rig
  local created = anim.createPhysicsRigFromTemplate(animSet, newPhysicsRigPath, templatePhysicsRigPath, true)
  app.info("Created physics rig: " .. newPhysicsRigPath .. " from template: " .. templatePhysicsRigPath  )
  
  if created then
    local set = anim.setPhysicsRigPath(newPhysicsRigPath, animSet, false)
  else    
    print("Error: Unable to create physics rig!")
    return   
  end
  app.info("Set physics rig.")
  
  -- get new physics root, remove duplicate volumes and weld pelvis joints
  local physRoot =  anim.getPhysicsRigDataRoot(scene, animSet)
  local templatePhysRoot =  anim.getPhysicsRigDataRoot(scene, templateName)
  
  app.info("Get animSet physics root: " .. physRoot:getName())
  app.info("Get template physics root: " .. templatePhysRoot:getName())

  -- weld the joints
  local weld = weldPelvisJoints(application, scene , physRoot, weldSelectionList, animSet)
  
  app.info("Weld pelvis joint. Weld success: " .. tostring(weld))

  local mirrorMap = createMirrorMaps(animSet, mirrorMapping[1], mirrorMapping[2])  
  local setMirrorPlane = anim.setAnimSetJointMirrorPlane(animSet, mirrorPlane)
  
  removeSurplusBodies(application, scene, physRoot, animSet)
  app.info("Removed surplus bodies")
  removeSurplusJoints(application, scene , physRoot, animSet, skeletonPrefix )
  app.info("Removed surplus joints")

  resetMapping(scene, animSet)
  app.info("Reset mappings")

  -- build character limb definitions 
  local animSetRigDef = defineCharacter(scene, animSet, "Physics")
  app.info("Get " .. animSet .. " rig defintition.")
  local templateRigDef = defineCharacter(scene, templateName, "Physics")
  app.info("Get " .. templateName .. " rig defintition.")

  -- copy from one character to another
  local copied = copyAllPhysicsRig( application, scene, templateRigDef, animSetRigDef)
  app.info("Copy rigs. Copy succes: " .. tostring(copied))

  if(copied)then 
    print("Succesfully copied ", templateName,  " to ",  animSet)
  else 
    print("Copying error")
  end

  local list = getCollisionGroupsFromAnimSet(scene , templateName)
  app.info("Get collision list from anim set")
  
  local bodyList = getPhysicsBodyListFromAnimSet(scene, animSet)
  app.info("Get physics bodies from anim set")
  
  setCollisionGroups( application, scene, animSet, list, bodyList )
  app.info("Set collision groups")

  app.info("Start change block to delete template character")
  local status, cb = scene:beginChangeBlock(getCurrentFileAndLine())
  
    if(deleteAnimSet(templateName))then
      print("Template anim set deleted.")
    else
      print("Template anim set not deleted.")
    end
    
  scene:endChangeBlock(cb, changeBlockInfo("End copy Change")) 
  
  app.info("End change block to delete template character")

end
