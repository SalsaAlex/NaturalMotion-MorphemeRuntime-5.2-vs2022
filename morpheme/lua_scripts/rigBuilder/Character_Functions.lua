------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2012 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------
-- build arm limb joint list
------------------------------------------------------------------------------
buildArm = function(templateList, spineEndIndex, armIndex)

  local arm = {}
  local currentNode = templateList[armIndex][2] -- {{ID} , {tagName, jointName}}

  -- check if arm has clavicle  
  parent = currentNode:getParent() 
  
  if( parent:getName() ~= templateList[spineEndIndex][2]:getName())then
    arm[1] = parent   
  else
    arm[1] = nil
  end  
        
  arm[2] = currentNode -- shoulder
  arm[3] = templateList[armIndex+1][2]  -- elbow
  arm[4] = templateList[armIndex+2][2]  -- wrist
      
  return arm   

end

------------------------------------------------------------------------------
-- build leg limb joint list
------------------------------------------------------------------------------
buildLeg = function(templateList, hipIndex)

  local leg = {}    
   
  leg[1] = templateList[hipIndex][2]  -- hip
  leg[2] = templateList[hipIndex+1][2]  --knee
  leg[3] = templateList[hipIndex+2][2]  --ankle
      
  return leg   

end

------------------------------------------------------------------------------
-- build spine limb joint list
------------------------------------------------------------------------------
buildSpine = function(templateList, spineEndIndex)

  local spine = {} 
  local spineRootName = templateList[1][2]:getName()
  local child = templateList[spineEndIndex][2]
  local i = 2
  local parent = nil  
  local maxSpineCount = 6
  
  spine[1] = child
  
  
  -- find root from spine end
  while true do  
    parent = child:getParent() 
    spine[i] = parent

    if(parent:getName() == spineRootName)then      
      break       
    end
    
    --break if i gets too big
    if( i > maxSpineCount)then
      break
    end    
    i = i + 1    
    child = parent  
  end
  
  return spine  

end



------------------------------------------------------------------------------
-- build spine limb joint list
------------------------------------------------------------------------------
buildHead = function(templateList, spineEndIndex, headIndex)

  local neck = {} 
  local headRootName = templateList[spineEndIndex][2]:getName()
  local child = templateList[headIndex][2]
  local parent = nil
  local i = 2
  local maxNeckCount = 6
  
  neck[1] = child
  
  -- find root from spine end
  while true do    
    parent = child:getParent() 
    neck[i] = parent
    
    if(parent:getName() == headRootName)then
      break       
    end 
    
    -- break if i gets too big   
    if( i > maxNeckCount)then
      break
    end
    
    i = i + 1    
    child = parent  
  end

  return neck 

end


------------------------------------------------------------------------------
--  Define character 
------------------------------------------------------------------------------ 
defineCharacter = function(scene, animSet, format)

  
  local charLimbDef = {Spine = {}, LeftArm = {}, RightArm = {}, LeftLeg = {}, RightLeg = {}, Head = {}}
  local spineEndIndex, leftArmIndex, rightArmIndex, leftLegIndex, rightLegIndex, neckIndex, headIndex = -1 


  local root = anim.getPhysicsRigDataRoot(scene, animSet)  
  local child = root:getFirstChild("BodyMappingNode")
  
  local templateList = {}
  if(child == nil)then
    print("Cannot find Body Mapping Node")
    return nil
  else
  -- cycle through body mapping list. Templates must be already made.
    for i=0, (child:getPartCount() - 1 ) do
    
      local part = child:getPart(i) 
      
      if(part == nil)then
        print("ERROR! Missing Templates. Make sure template is correct in the AnimSet Component.\n")
        return nil
        
      else
        local index = i + 1
        local tagName = nmx.BodyMappingNode.getPartPath(part) 
        local jointName = part:getName()
        local joint = anim.getPhysicsRigJoint(scene, animSet, jointName )  
        
        -- eg: {Hips, SKEL_ROOT}
        templateList[index] = {tagName, joint}      
              
        -- set indices for creating the limbs later
        if( tagName == "SpineEnd")then
          spineEndIndex = index        
        elseif( tagName == "LeftShoulder")then
          leftArmIndex = index        
        elseif( tagName == "RightShoulder")then
          rightArmIndex = index        
        elseif( tagName == "LeftHip")then
          leftLegIndex = index        
        elseif( tagName == "RightHip")then
          rightLegIndex = index        
        elseif( tagName == "Head")then
          headIndex = index
        end      
      end
    end
  
    
  charLimbDef.LeftArm = buildArm(templateList, spineEndIndex, leftArmIndex)
  charLimbDef.RightArm = buildArm(templateList , spineEndIndex, rightArmIndex)
  
  charLimbDef.LeftLeg = buildLeg(templateList, leftLegIndex)
  charLimbDef.RightLeg = buildLeg(templateList, rightLegIndex)
  
  charLimbDef.Spine = buildSpine(templateList, spineEndIndex)
  charLimbDef.Head = buildHead(templateList, spineEndIndex, headIndex) 
  
  return charLimbDef
  end
  

end

------------------------------------------------------------------------------
--  copy limbs from one character definition to another
------------------------------------------------------------------------------ 
copyLimbAttr = function(application , scene, copyLimb, pasteLimb, level)

  local copySize = table.getn(copyLimb)
  local pasteSize = table.getn(pasteLimb)
  
  local copyNodeList = {}
  local pasteNodeList = {}
 
  -- if same amount of object in arm -- todo: allow for different length arms to be copied across.
  if(copySize == pasteSize)then

    for i = 1, copySize do
    
      local copyPhysicsJoint = copyLimb[i]:getChildDataNode(nmx.PhysicsJointNode.ClassTypeId()) 
      local pastePhysicsJoint = pasteLimb[i]:getChildDataNode(nmx.PhysicsJointNode.ClassTypeId())

      if(level == "All" or level == "Limits")then         
 
        -- check parent is a physics joint, if not it's the root and no limits need to be calculated
        local parent = copyPhysicsJoint:getParent()
        
        if(parent:is(nmx.PhysicsJointNode.ClassTypeId()))then
        
          -- swing and twist limit attr
          copyNodeList[1] = copyPhysicsJoint:getFirstChild("TransformNode")
          copyNodeList[2] = copyNodeList[1]:getFirstChild("PhysicsTwistSwingNode")
          copyNodeList[3] = copyNodeList[2]:getFirstChild("CustomPhysicsEngineDataNode")
                  
          -- swing and twist limit attr
          pasteNodeList[1] = pastePhysicsJoint:getFirstChild("TransformNode")
          pasteNodeList[2] = pasteNodeList[1]:getFirstChild("PhysicsTwistSwingNode")
          pasteNodeList[3] = pasteNodeList[2]:getFirstChild("CustomPhysicsEngineDataNode")
        end
      end
      
      if(level == "All" or level == "Volumes")then
        local offset = 0
        local copyShape = "Capsule"
        if(level == "All")then 
          offset = 3
        end
        
        -- look into just copying all children from body        
        copyNodeList[offset+1] = copyPhysicsJoint:getFirstChild("PhysicsBodyNode")
        copyNodeList[offset+2] = copyNodeList[offset+1]:getFirstChild("CustomPhysicsEngineDataNode")
        copyNodeList[offset+3] = copyNodeList[offset+1]:getFirstChild("TransformNode")
        copyNodeList[offset+4] = copyNodeList[offset+3]:getFirstChild("PhysicsVolumeNode")
        copyNodeList[offset+5] = copyNodeList[offset+4]:getFirstChild("CustomPhysicsEngineDataNode")
        
        copyNodeList[offset+6] = copyNodeList[offset+3]:getFirstChild("CapsuleNode")
        if(copyNodeList[offset+6] == nil)then
          copyNodeList[offset+6] = copyNodeList[offset+3]:getFirstChild("BoxNode")
          copyShape = "Box"
        end
        
        -- look into just copying all children from body        
        pasteNodeList[offset+1] = pastePhysicsJoint:getFirstChild("PhysicsBodyNode")
        pasteNodeList[offset+2] = pasteNodeList[offset+1]:getFirstChild("CustomPhysicsEngineDataNode")
        pasteNodeList[offset+3] = pasteNodeList[offset+1]:getFirstChild("TransformNode")
        pasteNodeList[offset+4] = pasteNodeList[offset+3]:getFirstChild("PhysicsVolumeNode")
        pasteNodeList[offset+5] = pasteNodeList[offset+4]:getFirstChild("CustomPhysicsEngineDataNode")

        local pasteVolume = pasteNodeList[offset+4]:getFirstInstance():getParent() 
        
        local convertVolumeList = nmx.SelectionList.new() 
        convertVolumeList:add(pasteVolume)
        local convert = application:runCommand( "PhysicsTools", "Convert to", scene, convertVolumeList, copyShape)

        local shapeType = copyShape .. "Node"
        pasteNodeList[offset+6] = pasteNodeList[offset+3]:getFirstChild(shapeType)
     
      -- end if(level)
      end 
      
    --end for i
        -- start change block
        local status, cb = scene:beginChangeBlock(getCurrentFileAndLine())     
        
        -- cycle through nodes and search for attributes to copy and paste to
        for j = 1, (table.getn(copyNodeList)) do
          for k = 1, copyNodeList[j]:getNumElements() do
          
            local copyAttr = nil
            local pasteAttr = nil            
            local isArray = copyNodeList[j]:isAttributeArray(k-1)
            
            -- check if attribute array
            if(isArray)then
              copyAttr = copyNodeList[j]:getAttributeArray(k-1)
            else
             copyAttr = copyNodeList[j]:getAttribute(k-1)
            end
            
            local copyAttrName = copyAttr:getName()  
            
            if(isArray)then
              pasteAttr = pasteNodeList[j]:findAttributeArray(copyAttrName) 
            else
              pasteAttr = pasteNodeList[j]:findAttribute(copyAttrName) 
            end

            if(pasteAttr:isValid())then              
              pasteAttr:assignFrom(copyAttr)              
            end
          
          end     
        end
        
        -- end change block
        scene:endChangeBlock(cb, changeBlockInfo("End copy Change"))   
   end  
  else
    
    print("Different amount of ojects in", copyLimb[1]:getName(), " ", copySize, " in copyLimb" ,pasteLimb[1]:getName(), " ",  pasteSize ," in pasteLimb" )
  end
end

------------------------------------------------------------------------------
--  Copy all physics limbs
------------------------------------------------------------------------------ 
copyAllPhysicsRig = function(application, scene, copyRig, pasteRig )

  -- TO DO: HAVE 3rd PARAMETER TO DEFINE JUST LIMITS, JUST BODIES OR ALL  
  copyLimbAttr(application , scene, copyRig.LeftArm, pasteRig.LeftArm, "All")
  copyLimbAttr(application , scene, copyRig.RightArm, pasteRig.RightArm, "All") 
  
  copyLimbAttr(application , scene, copyRig.LeftLeg, pasteRig.LeftLeg, "All")
  copyLimbAttr(application , scene, copyRig.RightLeg, pasteRig.RightLeg, "All")
  
  copyLimbAttr(application , scene, copyRig.Spine, pasteRig.Spine, "All")
  copyLimbAttr(application , scene, copyRig.Head, pasteRig.Head, "All") 
  
  return true

end

------------------------------------------------------------------------------------
-- needed to reset the physics joint mapping to not use ROOT but use Pelvis instead
------------------------------------------------------------------------------------
resetMapping = function (scene, animSet)

  local root = anim.getPhysicsRigDataRoot(scene, animSet)
  
  local bodyMappingNode = root:getFirstChild("BodyMappingNode")
  local rootJoint = root:getFirstChild("PhysicsJointNode") 
  local newRootPart = rootJoint:getFirstInstance():getParent()  
 
  
  -- change block
  local status, cb = scene:beginChangeBlock(getCurrentFileAndLine())
  
    local clear = bodyMappingNode:clearPart(0)
    local set = bodyMappingNode:setPart(newRootPart, 0, true)
  
  scene:endChangeBlock(cb, changeBlockInfo("End copy Change")) 
  
end

