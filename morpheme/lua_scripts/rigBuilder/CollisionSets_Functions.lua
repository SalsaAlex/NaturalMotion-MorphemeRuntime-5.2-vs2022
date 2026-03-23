
------------------------------------------------------------------------------
-- build leg limb joint list
------------------------------------------------------------------------------
createCollisionGroup = function(application, scene, root, selection, groupName)

  -- create unnamed group with sgtransform body selection 
  local createGroup = application:runCommand( "PhysicsTools", "Create Collision Group", scene, selection)  
  -- get collision set node for that animation set
  local collisionSetNode = root:getFirstChild("CollisionSetNode")
  local collisionGroupNode = collisionSetNode:getFirstChild("CollisionGroupNode")  
  local i = 1
  local newName = nil
  
  while true do    
    if(i > 12)then
      break
    end    
     newName = string.find(collisionGroupNode:getName(), "Unnamed")
    if(newName ~= nil)then
      break
    else
      collisionGroupNode = collisionGroupNode:getNextSibling()
      i = i + 1
    end   
  end

  -- rename
  local status, cb = scene:beginChangeBlock(getCurrentFileAndLine())
  if(newName ~= nil)then  
    collisionGroupNode:setName(groupName)
  end
  scene:endChangeBlock(cb, changeBlockInfo("End copy Change")) 

end

------------------------------------------------------------------------------
-- build leg limb joint list
------------------------------------------------------------------------------
getCollisionGroupsFromAnimSet = function(scene, animSet)

  local collisionSetNode = anim.getPhysicsRigDataRoot(scene, animSet):getFirstChild("CollisionSetNode")
  local collisionGroupNode = collisionSetNode:getFirstChild("CollisionGroupNode")  
  local collisionGrpList = {}
  local i = 1
  
  while collisionGroupNode ~= nil do  
    collisionGrpList[i] = {collisionGroupNode:getName(), {}}
    
    local child = collisionGroupNode:getFirstChild()
    local j = 1
    
    while child ~= nil do      
      local name = string.gsub(child:getName(), "Ref", "")      
      collisionGrpList[i][2][j] = name    
      child = child:getNextSibling()      
      j = j + 1      
      if(j > 15)then
        break
      end    
    end    
    collisionGroupNode = collisionGroupNode:getNextSibling()
  
    i = i + 1
    
    if(i > 15)then
      break
    end  
  end 
  return collisionGrpList

end

------------------------------------------------------------------------------
-- build leg limb joint list
------------------------------------------------------------------------------
setCollisionGroups = function( application , scene, animSet, grpList, bodyList )

  local root = anim.getPhysicsRigDataRoot(scene, animSet) 
  -- cycle through each collision group
  for i = 1, table.getn(grpList) do    
    local grpName = grpList[i][1]
    local groupList = nmx.SelectionList.new()
    
    for j = 1, table.getn(grpList[i][2]) do    
      local bodyRefNodeName = grpList[i][2][j]
      -- check grpList against current body names and add to selection list if found
      for k = 1, table.getn(bodyList) do
        if(bodyRefNodeName == bodyList[k]:getName())then
          groupList:add(bodyList[k])
        end     
      end 
    end    
   createCollisionGroup(application, scene, root, groupList, grpName)    
  end 

end
