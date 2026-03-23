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

------------------------------------------------------------------------------------------------
-- populate combo boxes with list of anim sets.
------------------------------------------------------------------------------------------------
local populateAnimSetComboBox = function(comboBox)
  assert(comboBox)
  local list = listAnimSets()

  comboBox:setItems(list)
  local currentAnimSet = getSelectedAssetManagerAnimSet()
  comboBox:setSelectedItem(currentAnimSet)  

end --function

------------------------------------------------------------------------------------------------
-- get full path and file name to save to
------------------------------------------------------------------------------------------------
local getSavePhysicsRigPath = function()
  local physdl = ui.createFileDialog({name = "DefaultPhysicsPathControl",
                                      caption = "Save Physics Path",
                                      wildcard = "morpheme:connect physics rig files|mcprig"}) 
  physdl:show()
  local NewImportPath = physdl:getFullPath()
  local tempName = ui.getWindow("CreateDefaultPhysicsRigWindow|SavePhysicsRigBox")    
  tempName:setValue(NewImportPath)
  
end --function

------------------------------------------------------------------------------------------------
-- create defauly physics rig main function
------------------------------------------------------------------------------------------------
local createDefaultPhysicsRig = function(animSet, newPhysicsRigPath , VolumeScale)

  local application = nmx.Application.new()
  local scene = application:getSceneByName("AssetManager")
  
  local animSetType = anim.getAnimSetCharacterType(animSet)
  if(animSetType == "Animation")then
    local setPhysics = anim.setAnimSetCharacterType(animSet, "Physics")
  end
  
  local markUp = anim.getRigMarkupData(animSet)
  if(markUp.hipIndex == nil)then
    print("AnimSet tags are not set")
    return nil
  end
  local rootJoint = anim.getRigJoint(scene, animSet, markUp.hipIndex)
  local rootJointName = rootJoint:getName() 

  local created = anim.createPhysicsRig(animSet, rootJointName, newPhysicsRigPath, true)
  if created then
    print("Succesfully create physics rig")
    local set = anim.setPhysicsRigPath(newPhysicsRigPath, animSet, false)
  else    
    print("Error: Unable to create physics rig!") 
  end

  local physicsRoot = anim.getPhysicsRigDataRoot(scene, animSet)

  local hierarchy = physicsRoot:getFirstChild("HierarchyNode")
  local child = hierarchy:getFirstChild("PosQuatTransformNode")
  
  -- from Rig_Functions.lua - Remove all duplicate bodies from physics rig.
  removeSurplusBodies(application, scene, physicsRoot, animSet)

  while child ~= nil do
    local physicsJointTx = anim.getPhysicsRigJoint(scene, animSet, child:getName())
    local physicsJoint = physicsJointTx:getChildDataNode(nmx.PhysicsJointNode.ClassTypeId()) 

    local name = child:getName()
    local sibling = child:getNextSibling()  
    local scale = minVolumeScale
    if(sibling ~= nil)then
      scale = sibling:findAttribute("Translation"):asVector3():magnitude()
    end      
    local physicsJointTx = anim.getPhysicsRigJoint(scene, animSet, child:getName())
    local physicsJoint = physicsJointTx:getChildDataNode(nmx.PhysicsJointNode.ClassTypeId()) 
    local physicsBody = physicsJoint:getFirstChild("PhysicsBodyNode")
    
    
    if(physicsBody ~= nil)then
    
    local physicsBodyTx = physicsBody:getFirstChild("TransformNode")     
      if(physicsBodyTx ~= nil)then    
        local shapeNode = physicsBodyTx:getFirstChild("CapsuleNode")
        
        if(shapeNode == nil)then          
          local convertVolumeList = nmx.SelectionList.new()           
          convertVolumeList:add(physicsBodyTx:getFirstChild():getFirstInstance():getParent())
          local convert = application:runCommand( "PhysicsTools", "Convert to", scene, convertVolumeList, "Capsule")          
        end
        
        local status, cb = scene:beginChangeBlock(getCurrentFileAndLine()) 
        
          local position = nmx.Vector3.new((scale/2), 0.0 , 0.0)
          local rotationX = math.rad(90.0)
          local rotationY = math.rad(0.0)
          local rotationZ = math.rad(90.0)
          
          local setTx = physicsBodyTx:findAttribute("Translation"):setVector3(position)
          local setRx = physicsBodyTx:findAttribute("RotationX"):setFloat(rotationX)
          local setRy = physicsBodyTx:findAttribute("RotationY"):setFloat(rotationY)
          local setRz = physicsBodyTx:findAttribute("RotationZ"):setFloat(rotationZ)

          if(scale < minVolumeScale)then
            scale = minVolumeScale
          end    
          if(VolumeScale < minVolumeScale)then
            VolumeScale = minVolumeScale
          end
   
        
          local capsuleNode = physicsBodyTx:getFirstChild("CapsuleNode")
          if(capsuleNode ~= nil)then
            capsuleNode:findAttribute("Height"):setFloat(scale)
            capsuleNode:findAttribute("Radius"):setFloat(VolumeScale)
          end
          
        scene:endChangeBlock(cb, changeBlockInfo("End copy Change"))
        
      end      
    end
    
    child = child:getNextSibling()    
  end  
end


------------------------------------------------------------------------------------------------
-- set variables from createDefaultPhysics window
------------------------------------------------------------------------------------------------
local setPRigVariables = function()
 
  local animSetName = ui.getWindow("CreateDefaultPhysicsRigWindow|AnimSetComboBox2"):getValue() 
  local newRigPath = ui.getWindow("CreateDefaultPhysicsRigWindow|SavePhysicsRigBox"):getValue()  
  local volumeScale = tonumber(ui.getWindow("CreateDefaultPhysicsRigWindow|VolumeSizeTextBox"):getValue() )

    --createAnimRig( "set" animSetName, ".xmd file" XMDRig, ".mcarig file" newAnimRigPathName, ".mcarig file" tAnimRigPathName , ".mcprig file" tPhysicsRigPathName, "Options.lua tags" animTags)
    createDefaultPhysicsRig(animSetName, newRigPath, volumeScale)
    CreateDefaultPhysicsRigWindow:hide()


end --function



------------------------------------------------------------------------------------------------
-- create physics rig window
------------------------------------------------------------------------------------------------
createPhysicsRigWindowFunc = function()

  CreateDefaultPhysicsRigWindow = nil
  collectgarbage()
  
  CreateDefaultPhysicsRigWindow = ui.createModelessDialog
  { 
    caption = "Create Animation Rig for Morpheme:Connect 3.6.2", 
    size = {width = 400, height = 180}, 
    name = "CreateDefaultPhysicsRigWindow"
  }

  local btnWidth = 100
  local btnHeight = 50

  -- start main layout   
    CreateDefaultPhysicsRigWindow:beginVSizer()
    
      CreateDefaultPhysicsRigWindow:beginHSizer{flags = "expand"}
      
        local animSetName = CreateDefaultPhysicsRigWindow:addStaticText      
        {
          text = "Anim Set",
          name = "AnimSetText",
          size = { width = 170 },
        }  
        animSetName:setFont("bold")
        
      CreateDefaultPhysicsRigWindow:endSizer() 
      ----------------------------------------------------------
      CreateDefaultPhysicsRigWindow:beginHSizer{flags = "expand"}    
        local AnimSetComboBox2 = CreateDefaultPhysicsRigWindow:addComboBox
        {        
          name = "AnimSetComboBox2",
          onChanged = onComboChanged,
          size = { width = 170 }, 
        } 
        populateAnimSetComboBox(AnimSetComboBox2)
        
      CreateDefaultPhysicsRigWindow:endSizer() 
  

      CreateDefaultPhysicsRigWindow:beginHSizer{flags = "expand"}
          
            local SavePhysicsRigText = CreateDefaultPhysicsRigWindow:addStaticText
            {
              text = "Save Physics Rig\t",
              name = "SavePhysicsRigText",
              size = { width = 100 }
            }
          
            local SavePhysicsRigBox = CreateDefaultPhysicsRigWindow:addTextBox
            {
              flags = "expand",
              proportion = 1,
              setReadOnly = false,              
              name = "SavePhysicsRigBox"
            }
            local SavePhysicsRigButton = CreateDefaultPhysicsRigWindow:addButton
            {
              setReadOnly = false,
              label = "...",
              name = "SavePhysicsRigButton"
            }       
            SavePhysicsRigButton:setOnClick(getSavePhysicsRigPath)
            
      CreateDefaultPhysicsRigWindow:endSizer() 
      
            CreateDefaultPhysicsRigWindow:beginHSizer{flags = "expand"}
          
            local VolumeSizeText = CreateDefaultPhysicsRigWindow:addStaticText
            {
              text = "Volume Size\t",
              name = "VolumeSizeTextText",
              size = { width = 100 }
            }
          
            local VolumeSizeTextBox = CreateDefaultPhysicsRigWindow:addTextBox
            {
              --flags = "expand",
              proportion = 0,
              value = "0.1",
              size = { width = 50 },
              setReadOnly = false,               
              name = "VolumeSizeTextBox"
            }
            
      CreateDefaultPhysicsRigWindow:endSizer() 
      ----------------------------------------------------------
      -- splitter
      ----------------------------------------------------------
      CreateDefaultPhysicsRigWindow:beginHSizer{flags = "expand"}        
        local splitter = CreateDefaultPhysicsRigWindow:addSplitter      
        {
          flags = "expand",
          proportion = 1,
          size = { width = 170 },
        }  
        
      CreateDefaultPhysicsRigWindow:endSizer() 

      ----------------------------------------------------------
      CreateDefaultPhysicsRigWindow:beginHSizer()
      
        -- Add buttons to OK or Cancel this operation.
        local OKBox = CreateDefaultPhysicsRigWindow:addButton
        {
          flags = "expand",
          proportion = 1,
          label = "OK",
          name = "OKBox"
        }
        OKBox:setOnClick(setPRigVariables)

        local CancelBox = CreateDefaultPhysicsRigWindow:addButton
        {
          flags = "expand",
          proportion = 1,
          label = "Cancel",
          name = "CancelBox"
        }
         CancelBox:setOnClick(cancel)
       
      CreateDefaultPhysicsRigWindow:endSizer()
      ----------------------------------------------------------            
      CreateDefaultPhysicsRigWindow:beginHSizer{flags = "expand"}        
        local splitterHelpText = CreateDefaultPhysicsRigWindow:addSplitter      
        {
          flags = "expand",
          proportion = 1,
          size = { width = 170 },
        }  
        
      CreateDefaultPhysicsRigWindow:endSizer()
      CreateDefaultPhysicsRigWindow:beginHSizer{flags = "expand"}
          
            local FileName = CreateDefaultPhysicsRigWindow:addStaticText
            {
              text = "Help:   \n",
              name = "Help"
            }
            
      CreateDefaultPhysicsRigWindow:endSizer()
      
  CreateDefaultPhysicsRigWindow:endSizer()
  CreateDefaultPhysicsRigWindow:show()

end


--CreateDefaultPhysicsRigWindowFunc()

