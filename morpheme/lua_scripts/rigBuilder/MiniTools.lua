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

local col = {r = 0.1 , g = 0.1, b = 0.1}
local wContext = {rollUps = 5, layers = 9}
local buttonSize = {width = 145, height = 40}
local windowSize = {width = 170, height = 580}
local indent = 1.1

------------------------------------------------------------------------------
-- toggle mirror on selection
------------------------------------------------------------------------------
local mirrorToggle = function()

  local toggle = preferences.get("MirrorOnSelection")
  
  if(toggle)then
    preferences.set{ name = "MirrorOnSelection", value = false }
  else
    preferences.set{ name = "MirrorOnSelection", value = true }
  end
  
end

local meshToggle = function()

  local assetManagerWindow = ui.getWindow("MainFrame|LayoutManager|AssetManager|AssetManagerViewport")
  local application = nmx.Application.new()
  local classTypeId = application:lookupTypeId("MeshNode")
  local visible = assetManagerWindow:getShouldRenderType(classTypeId)
  if(visible)then
    assetManagerWindow:setShouldRenderType(classTypeId, false)
  else
    assetManagerWindow:setShouldRenderType(classTypeId, true)
  end
  
end

local volumeToggle = function()
  local assetManagerWindow = ui.getWindow("MainFrame|LayoutManager|AssetManager|AssetManagerViewport")
  local application = nmx.Application.new()
  local classTypeId = application:lookupTypeId("PhysicsVolumeNode")
  local visible = assetManagerWindow:getShouldRenderType(classTypeId)
  if(visible)then
    assetManagerWindow:setShouldRenderType(classTypeId, false)
  else
    assetManagerWindow:setShouldRenderType(classTypeId, true)
  end  
end

local increaseJointSize = function()
  local size, t = preferences.get("JointDisplayScale")
  local increment = jointSize[3] -- from Option.lua
  local maxSize = jointSize[2]-- from Option.lua
  
  t.value = t.value + increment  
  if(t.value > maxSize)then
    t.value = maxSize
  end   
  preferences.set(t)

end

local decreaseJointSize = function()
  local size, t = preferences.get("JointDisplayScale")
  local increment = jointSize[3] -- from Option.lua
  local minSize = jointSize[1]-- from Option.lua
  
  t.value = t.value - increment  
  if(t.value < minSize)then
    t.value = minSize
  end
  preferences.set(t)

end


------------------------------------------------------------------------------
-- save anim rig function
------------------------------------------------------------------------------
local saveAnimRig = function()

  local currentSet = getSelectedAssetManagerAnimSet()
  local hasRig = anim.isRigValid(currentSet)
  if(hasRig)then
    local save = anim.saveRig(currentSet)
    if(save)then
      app.info("Saved animatio rig for anim set: " .. currentSet)
    else
      app.info("Could not save animation rig for anim set: " .. currentSet)
    end
  else
    app.info("Invalid animation rig for anim set: " .. currentSet)
  end
end

------------------------------------------------------------------------------
-- save physics rig function
------------------------------------------------------------------------------
local savePhysicsRig = function()

  local currentSet = getSelectedAssetManagerAnimSet()
  local hasRig = anim.isPhysicsRigValid(currentSet)
  if(hasRig)then
    local save = anim.savePhysicsRig(currentSet)
    if(save)then
      app.info("Saved physics rig for anim set: " .. currentSet)
    else
      app.info("Could not save physics rig for anim set: " .. currentSet)
    end
  else
    app.info("Invalid physics rig for anim set: " .. currentSet)
  end
  
end

------------------------------------------------------------------------------
-- save current rig (rigType = "Animation" / "Physics" / "All")
------------------------------------------------------------------------------
local saveCurrentRig = function()
  local currentSet = getSelectedAssetManagerAnimSet()  
    saveAnimRig()
    savePhysicsRig()
end

------------------------------------------------------------------------------
-- save all rigs
------------------------------------------------------------------------------
local saveAllRigs = function()
  local animSets = listAnimSets()  
  for i, v in pairs(animSets)do  
    local hasAnimationRig = anim.isRigValid(v)
    if(hasAnimationRig)then
      saveAnimRig(v)
    end
    local hasPhysicsRig = anim.isPhysicsRigValid(v)
    if(hasPhysicsRig)then
      savePhysicsRig(v)    
    end
  end
end


------------------------------------------------------------------------------
-- find the closest vector to the world up vector
------------------------------------------------------------------------------
local fincClosestUpVector = function( WM, upAxis, negUpAxis) 

  local angles = {}
  WM:orthonormalise()

  angles[1] = math.abs(math.acos(WM:xAxis():dot(upAxis)))
  angles[2] = math.abs(math.acos(WM:yAxis():dot(upAxis)))
  angles[3] = math.abs(math.acos(WM:zAxis():dot(upAxis)))
  
  angles[4] = math.abs(math.acos(WM:xAxis():dot(negUpAxis)))
  angles[5] = math.abs(math.acos(WM:yAxis():dot(negUpAxis)))
  angles[6] = math.abs(math.acos(WM:zAxis():dot(negUpAxis)))

  local smallestAngle = 100
  local axisIndex = 0
  
  for i = 1, 6 do
    local QNAN_Test = string.find(tostring(angles[i]), "#QNAN")
    if(QNAN_Test ~= nil)then
      print(angles[i])
      angles[i] = 0.0
    end
    if(angles[i] < smallestAngle)then      
      smallestAngle = angles[i]
      axisIndex = i - 1
    end  
  end  
  return axisIndex   
end

------------------------------------------------------------------------------
-- align object function
------------------------------------------------------------------------------
local align = function()

  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local selection = scene:getSelectionList(0)
  local node = selection:getNode(1)
  if(node)then

    local upAxis = nmx.Vector3.new()
    local negUpAxis = nmx.Vector3.new()
    local worldUpAxis = preferences.get("WorldUpAxis")
    
    if(worldUpAxis == "X Axis") then
      upAxis:set(1.0, 0.0, 0.0)
      negUpAxis:set(-1.0, 0.0, 0.0)    
    elseif(worldUpAxis == "Y Axis") then
      upAxis:set(0.0, 1.0, 0.0)
      negUpAxis:set(0.0, -1.0, 0.0)    
    elseif(worldUpAxis == "Z Axis") then
      upAxis:set(0.0, 0.0, 1.0)
      negUpAxis:set(0.0, 0.0, -1.0)
    end

    local nodeLM = node:getLocalMatrix()
    local nodeWM = node:getWorldMatrix()
    local parentWMInvert = node:getParentWorldMatrix()
    
    parentWMInvert:invert()  

    -- find closest up axis local to the node returns index +X:0 +Y:1 +Z:2 -X:3 -Y:4 -Z:5
    local nodeUpAxisIndex = fincClosestUpVector(nodeWM, upAxis, negUpAxis)
    
    local nodeWxAxis = nodeWM:xAxis()
    local nodeWyAxis = nodeWM:yAxis()
    local nodeWzAxis = nodeWM:zAxis() 
    
    nodeWxAxis:normalise()
    nodeWyAxis:normalise()
    nodeWzAxis:normalise()
    
    -- set up axis to negative up axis if values is 3 or above
    if(nodeUpAxisIndex > 2)then
      upAxis:set(negUpAxis)
      nodeUpAxisIndex = nodeUpAxisIndex - 3
    end
    
    -- calculate new axis for the node
    if(nodeUpAxisIndex == 0)then
      nodeWxAxis:set(upAxis)   
      nodeWyAxis:cross( nodeWzAxis, nodeWxAxis)
      nodeWzAxis:cross( nodeWxAxis, nodeWyAxis)
    
    elseif(nodeUpAxisIndex == 1)then
      nodeWyAxis:set(upAxis)  
      nodeWxAxis:cross( nodeWyAxis, nodeWzAxis)
      nodeWzAxis:cross( nodeWxAxis, nodeWyAxis)
      
    elseif(nodeUpAxisIndex == 2)then
      nodeWzAxis:set(upAxis)  
      nodeWxAxis:cross( nodeWyAxis, nodeWzAxis)
      nodeWyAxis:cross( nodeWzAxis, nodeWxAxis)
    
    end
    
    nodeWxAxis:normalise()
    nodeWyAxis:normalise()
    nodeWzAxis:normalise()

    -- set new tranforms
    nodeWM:setRow(nodeWxAxis, 0)
    nodeWM:setRow(nodeWyAxis, 1)  
    nodeWM:setRow(nodeWzAxis, 2)
    
    -- LM = WM * -(PWM)
    nodeLM:multiply(nodeWM, parentWMInvert)  
    nodeLM:orthonormalise()
    
    -- set new local matrix for node
    local status, cb = scene:beginChangeBlock(getCurrentFileAndLine())     
      node:setLocalMatrix(nodeLM) 
    scene:endChangeBlock(cb, changeBlockInfo("End copy Change"))

  else
    print("Nothing selected.")
   
  end
end

------------------------------------------------------------------------------
-- build rig tools roll up
------------------------------------------------------------------------------
local wAddRigRollUp = function(panel)

  local buttonNum = 2
  local rollup1 = panel:addRollup {
        name = "RigRollUp",
        label = "   Rig",
        flags = "expand;mainSection",
      }
  rollup1:setHeaderBackgroundColour(col)
  rollup1:setOnExpandCollapse(function()  
    local expanded = rollup1:isExpanded()
    if(expanded)then
      windowSize.height = windowSize.height + ((buttonNum * buttonSize.height) * indent)
    else
      windowSize.height = windowSize.height - ((buttonNum * buttonSize.height) * indent)
    end  
    miniToolsWindow:setSize(windowSize)    
  end)
        
  local rollUpPanel = rollup1:getPanel()
  rollUpPanel:beginVSizer{ flags = "expand", proportion = 0 }
  
    rollUpPanel:beginHSizer{flags = "expand"}     
    local alignButton = rollUpPanel:addButton {
          --flags = "expand",
          proportion = 0,
          size = buttonSize,
          label = "Align With World",
          name = "AlignButton"
        }
    alignButton:setToolTip("Align objects with up vector such as foot boxes.")    
    alignButton:setOnClick(align)    
    rollUpPanel:endSizer() -- beginHSizer
    
    rollUpPanel:beginHSizer{flags = "expand"}     
    local mirrorToggleButton = rollUpPanel:addButton {
      proportion = 0,
      size = buttonSize,
      label = "Mirror Toggle",      
      name = "mirrorToggleButton"
    }
    mirrorToggleButton:setToolTip("Toggle ON/OFF Mirror on Selection. Mirror mappings need to be created to work.")
    mirrorToggleButton:setOnClick(mirrorToggle)     
    rollUpPanel:endSizer() -- beginHSizer
    
  rollUpPanel:endSizer() -- beginVSizer


end


------------------------------------------------------------------------------
-- build view tools roll up
------------------------------------------------------------------------------
local wAddViewRollUp = function(panel)

  local buttonNum = 2
  local rollup1 = panel:addRollup {
        name = "ViewRollUp",
        label = "   View",
        flags = "expand;mainSection",
      }
  rollup1:setHeaderBackgroundColour(col)
  rollup1:setOnExpandCollapse(function()
  
    local expanded = rollup1:isExpanded()
    if(expanded)then
      windowSize.height = windowSize.height + ((buttonNum * buttonSize.height) * indent)
    else
      windowSize.height = windowSize.height - ((buttonNum * buttonSize.height) * indent)
    end  
    miniToolsWindow:setSize(windowSize)
    
  end)
        
  local rollUpPanel = rollup1:getPanel()
  rollUpPanel:beginVSizer{ flags = "expand", proportion = 0 } 

    
    rollUpPanel:beginHSizer{flags = "expand"}     
    local meshToggleButton = rollUpPanel:addButton {
      proportion = 0,
      size = buttonSize,
      label = "Mesh Toggle", 
      name = "meshToggleButton"
    }
    meshToggleButton:setToolTip("Togggle ON/OFF the meshes in the Asset Manager viewport.")
    meshToggleButton:setOnClick(meshToggle)    
    rollUpPanel:endSizer() -- beginHSizer

    rollUpPanel:beginHSizer{flags = "expand"}     
    local volumeToggleButton = rollUpPanel:addButton {
      proportion = 0,
      size = buttonSize,
      label = "Volume Toggle", 
      name = "volumeToggleButton"
    }
    volumeToggleButton:setToolTip("Togggle ON/OFF the volumes in the Asset Manager viewport.")
    volumeToggleButton:setOnClick(volumeToggle)    
    rollUpPanel:endSizer() -- beginHSizer    
    
  rollUpPanel:endSizer() -- beginVSizer

end

------------------------------------------------------------------------------
-- build arm limb joint list
------------------------------------------------------------------------------
local wAddSaveRigRollUp = function(panel)

  local buttonNum = 2
  
  local rollup1 = panel:addRollup {
        name = "SaveRigRollUp",
        label = "   Save Rigs",
        flags = "expand;mainSection",
      }
  rollup1:setHeaderBackgroundColour(col)
  rollup1:setOnExpandCollapse(function()  
    local expanded = rollup1:isExpanded()
    if(expanded)then
      windowSize.height = windowSize.height + ((buttonNum * buttonSize.height) * indent)
    else
      windowSize.height = windowSize.height - ((buttonNum * buttonSize.height) * indent)
    end  
    miniToolsWindow:setSize(windowSize)
    
  end)
        
  local rollUpPanel = rollup1:getPanel()
  rollUpPanel:beginVSizer{ flags = "expand", proportion = 0 }
    rollUpPanel:beginHSizer{flags = "expand"} 
    
    local saveAnimRigButton = rollUpPanel:addButton {
          proportion = 0,
          size = buttonSize,
          label = "Anim",
          name = "AnimRig"
        }
    saveAnimRigButton:setToolTip("Save current animation rig.")
    saveAnimRigButton:setOnClick(saveAnimRig) 
    
    rollUpPanel:endSizer() -- beginHSizer    
    rollUpPanel:beginHSizer{flags = "expand"}    
        
    local savePhysicsRigButton = rollUpPanel:addButton {
          proportion = 0,
          size = buttonSize,
          label = "Physics",
          name = "PhysicsRig "
    }
    savePhysicsRigButton:setToolTip("Save current physics rig.")
    savePhysicsRigButton:setOnClick(savePhysicsRig)   

    rollUpPanel:endSizer() -- beginHSizer
  rollUpPanel:endSizer() -- beginVSizer


end

------------------------------------------------------------------------------
-- build arm limb joint list
------------------------------------------------------------------------------
local wAddSaveAnimSetRollUp = function(panel)

  local buttonNum = 2
  
  local rollup1 = panel:addRollup {
        name = "SaveAnimSetRollUp",
        label = "   Save Anim Sets",
        flags = "expand;mainSection",
      }
  rollup1:setHeaderBackgroundColour(col)
  rollup1:setOnExpandCollapse(function()
  
    local expanded = rollup1:isExpanded()
    if(expanded)then
      windowSize.height = windowSize.height + ((buttonNum * buttonSize.height) * indent)
    else
      windowSize.height = windowSize.height - ((buttonNum * buttonSize.height) * indent)
    end  
    miniToolsWindow:setSize(windowSize)
    
  end)
        
  local rollUpPanel = rollup1:getPanel()
  rollUpPanel:beginVSizer{ flags = "expand", proportion = 0 }
  
    rollUpPanel:beginHSizer{flags = "expand"} 
    local saveAnimSetButton = rollUpPanel:addButton {
          proportion = 0,
          size = buttonSize,
          label = "Current Set",
          name = "Current"
    }
    saveAnimSetButton:setToolTip("Save current animation and physics rig.")
    saveAnimSetButton:setOnClick(saveCurrentRig)
   
    rollUpPanel:endSizer() -- beginHSizer    
    rollUpPanel:beginHSizer{flags = "expand"} 
    
    local saveAllButton = rollUpPanel:addButton {
          proportion = 0,
          size = buttonSize,
          label = "All Anim Sets",
          name = "All"
    }
    saveAllButton:setToolTip("Save all Animation Set opened in this network.")
    saveAllButton:setOnClick(saveAllRigs)
    
    rollUpPanel:endSizer() -- beginHSizer
  rollUpPanel:endSizer() -- beginVSizer


end

------------------------------------------------------------------------------
-- build arm limb joint list
------------------------------------------------------------------------------
local wAddPrefRollUp = function(panel)

  local buttonNum = 1
  
  local rollup1 = panel:addRollup {
        name = "PreferencesRollUp",
        label = "   Preferences",
        flags = "expand;mainSection",
      }
  rollup1:setHeaderBackgroundColour(col)
  rollup1:setOnExpandCollapse(function()
  
    local expanded = rollup1:isExpanded()
    if(expanded)then
      windowSize.height = windowSize.height + ((buttonNum * buttonSize.height) * indent)
    else
      windowSize.height = windowSize.height - ((buttonNum * buttonSize.height) * indent)
    end  
    miniToolsWindow:setSize(windowSize)
    
  end)
        
  local rollUpPanel = rollup1:getPanel()
  rollUpPanel:beginVSizer{ flags = "expand", proportion = 0 }
  
    rollUpPanel:beginHSizer{flags = "expand"} 
    local decreaseSize = rollUpPanel:addButton {
          proportion = 0,
          size = {width = buttonSize.width/2, height = buttonSize.height},
          label = "Size (-)",
          name = "Decrease"
    }
    decreaseSize:setToolTip("Decrease size of limites and joints in the viewport.")
    decreaseSize:setOnClick(decreaseJointSize)
   
    --rollUpPanel:endSizer() -- beginHSizer    
    --rollUpPanel:beginHSizer{flags = "expand"} 
    
    local increaseSize = rollUpPanel:addButton {
          proportion = 0,
          size = {width = buttonSize.width/2, height = buttonSize.height},
          label = "Size (+)",
          name = "Increase"
    }
    increaseSize:setToolTip("Increase size of limites and joints in the viewport.")
    increaseSize:setOnClick(increaseJointSize)
    
    rollUpPanel:endSizer() -- beginHSizer
  rollUpPanel:endSizer() -- beginVSizer


end

------------------------------------------------------------------------------
-- build arm limb joint list
------------------------------------------------------------------------------
miniToolsWindowFunc = function()

  miniToolsWindow = nil
  collectgarbage()
  windowSize = {width = 170, height = 580}
  
  miniToolsWindow = ui.createModelessDialog
  { 
    caption = "Mini Tools", 
    size = windowSize, 
    name = "MiniTools"
  }
  
  miniToolsWindow:beginVSizer()

    local panel = miniToolsWindow:addRollupContainer{
          flags = "expand",
          proportion = 1,          
        }       
        -- rigging tools roll up
        wAddRigRollUp(panel)     
        -- rigging view roll up
        wAddViewRollUp(panel)         
        -- rigging save rigs roll up
        wAddSaveRigRollUp(panel)
        -- rigging save anim set roll up
        wAddSaveAnimSetRollUp(panel)
        -- prferences roll up
        wAddPrefRollUp(panel)
 
    
  miniToolsWindow:endSizer() --end VSizer   
  miniToolsWindow:show()

end

--miniToolsWindowFunc()


