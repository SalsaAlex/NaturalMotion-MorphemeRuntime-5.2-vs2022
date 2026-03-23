------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2012 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-------------------------------------------------------------------------
-- find if anim set exists
-------------------------------------------------------------------------
animsSetExists = function (name)
  local list = listAnimSets()  
  for i = 1, table.getn(list) do  
    if( name == list[i])then
      return true    
    end 
  end
  return false  
end

-------------------------------------------------------------------------
-- recursive physics joint by name
-------------------------------------------------------------------------
findPhysicsJointByName = function( root, jointName, list)
  
  local child = root:getFirstChild()
  local name 
  local parent
  local grandChild

  while child ~= nil do     
    if child:is(nmx.TransformNode.ClassTypeId())then      
        findPhysicsJointByName(child, jointName, list)
      
    elseif(child:is(nmx.PhysicsJointNode.ClassTypeId()))then
      name = child:getName() 
      if(name == jointName)then      
        parent = child:getFirstInstance():getParent()
        list:add(parent)
      end
      findPhysicsJointByName(child, jointName, list)     
      
    end      
    child = child:getNextSibling()  
  end 
end

-------------------------------------------------------------------------
-- weld pelvis/root/spine root physics joint together
-------------------------------------------------------------------------
weldPelvisJoints = function(application, scene, root, sel, animSet )  
  
  local pelvisSelection = nmx.SelectionList.new()  
  local newRootNode = anim.getPhysicsRigJoint(scene, animSet, sel[1])
  pelvisSelection:add(newRootNode)
  
  -- reorder physics rig to use Pelvis as new root
  local reorder = application:runCommand( "Physics Tools", "Set as Physics Rig Root", scene, pelvisSelection)

  -- create new selection
  local hipRootSelection = nmx.SelectionList.new()
  -- get joint nodes from weld list and add them to selection
  for i = 1, table.getn(sel) do
    print(sel[i])
    findPhysicsJointByName(root, sel[i], hipRootSelection)    
  end 
 
  -- run weld command with new hip selection list
  local weld = application:runCommand( "Physics Tools", "Weld To Parent", scene, hipRootSelection)
  if weld then
    print("--\nSuccessful weld!\n--")
    return true
  else
    print("--\nERROR: Unsuccessful weld!\n--")
    return false
  end

end

-------------------------------------------------------------------------
-- create mirror mapping
-------------------------------------------------------------------------
createMirrorMaps = function(set, firstFilter, secondFilter)

  -- generate a set for quick lookup of joint names for quickly checking if the mirrored joint name exists
  local jointNames = anim.getRigChannelNames(set)
  local jointLookupSet = { }
  for _, l in ipairs(jointNames) do
    jointLookupSet[l] = true
  end

  -- generate lookup for existing joint mirror mappings
  local jointAlreadyMapped = { }
  local currentJointMirrorMappings = anim.listAnimSetJointMirrorMappings(set)
  if currentJointMirrorMappings then
    for _, mirroring in ipairs(currentJointMirrorMappings) do
      jointAlreadyMapped[mirroring.first] = true
      jointAlreadyMapped[mirroring.second] = true
    end
  end

  local newMappings = { }
  -- for each joint now see if there is a first filter pattern match
  -- generate the second joint name using the second filter pattern
  -- if the generated name is a valid joint name then add the matches to the set's mirrored joint names
  for _, jointName in ipairs(jointNames) do
    local result = string.find(jointName, firstFilter)
    if result then
      local preFilterString = string.sub(jointName, 1, result - 1)
      local postFilterString = string.sub(jointName, result + string.len(firstFilter), string.len(jointName))
      local mirroredJointName = string.format("%s%s%s", preFilterString, secondFilter, postFilterString)

      if jointLookupSet[mirroredJointName] then
        -- adding a new mapping will fail if the joint or mirror joint is already mapped
        if not jointAlreadyMapped[jointName] and not jointAlreadyMapped[mirroredJointName] then
          table.insert(newMappings, { first = jointName, second = mirroredJointName })
        end
      end
    end
  end

  if table.getn(newMappings) > 0 then
    anim.addAnimSetJointMirrorMappings(set, newMappings)
  end
end



-------------------------------------------------------------------------
-- recursive physics volume search
-------------------------------------------------------------------------
--recursePhysicsVolumeSearch 
recursePhysicsVolumeSearch = function( root, list )
  
  local child = root:getFirstChild()
  local name 
  local parent

  while child ~= nil do 
  
    -- recurse if child is a transform, joint or body node
    if  child:is(nmx.TransformNode.ClassTypeId()) or
        child:is(nmx.PhysicsJointNode.ClassTypeId()) or
        child:is(nmx.PhysicsBodyNode.ClassTypeId()) then
      
        recursePhysicsVolumeSearch(child, list)
      
      elseif(child:is(nmx.PhysicsVolumeNode.ClassTypeId()))then      
        local isInTemplate = false
        name = child:getName()
        
        -- get the volume instance and name to add to the list
        local parent = child:getFirstInstance():getParent()
        local parentName = parent:getName()
        
        -- check for duplicate volumes
        local sibling = parent:getNextSibling()
        
        while sibling ~= nil do          
          print("Deleted extra volumes: ", sibling:getName())
          list:add(sibling)
          sibling = sibling:getNextSibling()
        end 
    end      
    child = child:getNextSibling()  
  end
end

------------------------------------------------------------------------------
-- Recurse through physics rig and remove duplicate bodies
------------------------------------------------------------------------------ 
removeSurplusBodies = function(application, scene,  physRoot, animSet )

  local deleteVolumeList = nmx.SelectionList.new()
  recursePhysicsVolumeSearch(physRoot, deleteVolumeList)
  
  if(deleteVolumeList:size() > 0)then
	local success = application:runCommand( "Core", "Delete", scene, deleteVolumeList) 
	  if success then
		print("--\nDeleted excess physics volumes!\n--")
	  else
		print("--\nERROR: Unsuccessfully deleted physics volumes!\n--")
	  end 		
  end  
end

------------------------------------------------------------------------------
-- Recursive physics joint search
------------------------------------------------------------------------------ 
--recursePhysicsJointSearch 
recursePhysicsJointSearch = function( root, list , prefix)
  
  local child = root:getFirstChild()
  local name 
  local parent
  local grandChild

  while child ~= nil do     
    if child:is(nmx.TransformNode.ClassTypeId())then      
        recursePhysicsJointSearch(child, list, prefix)
      
    elseif(child:is(nmx.PhysicsJointNode.ClassTypeId()))then

      name = child:getName()  
      
      -- check if joint has joint prefix in the name
      local same = string.find(name, prefix)
      
      -- check to see if joint is leaf joint (finger nub)
      local grandChild = child:getFirstChild("PhysicsJointNode")      
      
      if(same == nil or grandChild == nil)then      
        parent = child:getFirstInstance():getParent()
        list:add(parent)
      end       
      recursePhysicsJointSearch(child, list, prefix)      
    end      
    child = child:getNextSibling()  
  end 
end


------------------------------------------------------------------------------
-- Recurse through physics rig and remove unwanted joints
------------------------------------------------------------------------------ 
removeSurplusJoints = function(application, scene, physRoot, animSet, prefix )

  local deleteJointList = nmx.SelectionList.new()
  recursePhysicsJointSearch(physRoot, deleteJointList, prefix)

  local success = application:runCommand( "Core", "Delete", scene, deleteJointList)
  
  if success then
    print("--\nDeleted excess physics joints!\n--")
  else
    print("--\nERROR: Unsuccessfully deleted physics joints!\n--")
  end   
end


------------------------------------------------------------------------------
-- Create physics body list
------------------------------------------------------------------------------
--recursePhysicsBodySearch 
recursePhysicsBodySearch = function( root, list)
  
  local child = root:getFirstChild()
  local name 
  local parent

  while child ~= nil do 
  
    -- recurse if child is a transform, joint or body node
    if  child:is(nmx.TransformNode.ClassTypeId()) or
        child:is(nmx.PhysicsJointNode.ClassTypeId())then      
        recursePhysicsBodySearch(child, list)      
      elseif(child:is(nmx.PhysicsBodyNode.ClassTypeId()))then     
  
        -- get the volume instance and name to add to the list
        local parent = child:getFirstInstance():getParent()
        table.insert(list, parent)      
    end        
    child = child:getNextSibling()  
  end
end

------------------------------------------------------------------------------
-- Create physics body list
------------------------------------------------------------------------------
getPhysicsBodyListFromAnimSet = function(scene, animSet)

  local root = anim.getPhysicsRigDataRoot(scene, animSet)
  local physBodyList = {}
  recursePhysicsBodySearch(root, physBodyList)
  
  return physBodyList
  
end

