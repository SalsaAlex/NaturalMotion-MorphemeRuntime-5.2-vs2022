------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local currentValue = "Good"
local limbRoot = app.loadImage("limbRoot.png")
local limbEnd = app.loadImage("limbEnd.png")
local reachLimit = app.loadImage("reachLimit.png")
local limbTypes = {"Arm","Leg","Head"}
local errorIcon = app.loadImage("ErrorIcon.png")

local addButton
local removeButton
local moveUpButton
local moveDownButton

-- a mapping from limb name to rollup
local limbSections = {}
local ikSolverAttributesSections = { }
local lastSelectedLimb = nil
local getCurrentLimb
local validateFunction

------------------------------------------------------------------------------------------------------------------------
-- Help Text
local kHelp = {
  ColourSwatchHelpText = "Colour of the physics volumes in this limb when displayed in the viewport in \"Limb Colour\" mode.",
  LimbVisibilityHelpText = "Set the visibility of all physics volumes in this limb.",
  LimbNameHelpText = "The name of the limb (right click to edit).",
  SelectLimbRootHelpText = "Select the limb's root locater in the view port.",
  SelectLimbEndHelpText = "Select the limb's end locater in the view port.",
  SelectLimbReachLimitHelpText = "Select the limb's reach limit in the view port.",
  LimbRootHelpText = "The root of the limb.",
  LimbEndHelpText = "The end of the limb.",
  LimbBaseHelpText = "The base of the limb. This is the point on the limb where most of the movement articulates, for an arm this would be the shoulder.",
  AdditionalBodiesHelpText = "Additional bodies are considered in internal Euphoria calculations, such as mass, but are not a part of the IK chain itself.",
  GuidePoseHelpText = "A stronger constraint on the IK solution brought about by modifying the input.",
  GuidePoseWeightHelpText = "Amount of the guide pose to blend onto the chosen joints before the IK solve.",
  GuidePoseJointsHelpText = "The joints affected by the input bias.",
  RedundancyControlHelpText = "Used to control the style of the IK solution for limb. For example controlling redundant degrees of freedom such as the elbow angle of the arms (swivel).",
  RedundancyControlWeightHelpText = "Weight of the guide pose that is used to determine the limbs mid-range (for redundancy control).",
  OrientationHelpText = "How much each joint should contribute to the orientation of the end effector. For an generally most contribution should come from joints at the end of the chain (eg. at the wrist).",
  OrientationModeHelpText = "Control how much each joint solves for the orientation constraint. \"Default\" is a linear distribution, \"Manual\" gives control over each joint.",
  OrientationWeighsHelpText = "Control how a particular joint solves for the orientation constraint.",
  PositionHelpText = "How much each joint should contribute to the position of the end effector. For an generally most contribution should come from joints at the end of the chain (eg. at the wrist).",
  PositionModesHelpText = "Control how much each joint solves for the position constraint. \"Default\" is a linear distribution, \"Manual\" gives control over each joint.",
  PositionWeightsHelpText = "Control how a particular joint solves for the position constraint.",
  CoupledLimitEnabledHelpText = "A hamstring like limit that restricts the combined articulation of the chain of joints in a limb.",
  CoupledLimitDistanceHelpText = "The maximum cumulative distance that an artculation represents before the joints are constrained.",
  CoupledLimitStiffnessHelpText = "Natural frequency of the spring in radians per second, larger is like using a thicker, stronger spring.",
  TwistWeightsHelpText = "Vector component in twist directon of a coupled limit.",
  Swing1Text = "Vector component in Swing1 directon of a coupled limit.",
  Swing2Text = "Vector component in Swing2 directon of a coupled limit.",
}

------------------------------------------------------------------------------------------------------------------------
local contains = function(theTable, element)
  local elementName = element:getName()
  for _, x in pairs(theTable) do
    if elementName == x:getName() then
      return true
    end
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
local getLimbJoints = function(limbObj)
  -- Todo: probably need to loop back to the spine root really... but this will do as a test...
  -- we return a table of sgShapeNodes
  local joints = { }
  local endObj = limbObj:findAttribute("End"):getInput():getNode()
  local baseObj = limbObj:findAttribute("Base"):getInput():getNode()

  return joints
end

------------------------------------------------------------------------------------------------------------------------
-- Returns the name of the currently selected limb and the currently selected object
local getCurrentLimb = function()
  for limbName, section in limbSections do
    if section.rollup:isSelected() then
      return limbName, section.limbObj
    end
  end
  
  return null
end

------------------------------------------------------------------------------------------------------------------------
local findLimbSetNode = function(selectedSet)
  if anim.isPhysicsRigValid(selectedSet) then
    local scene = nmx.Application.new():getSceneByName("AssetManager")
    local physRoot = anim.getPhysicsRigDataRoot(scene, selectedSet)
    if physRoot then
      local limbGroups =  physRoot:findChild("PhysicsLimbGroups", true)
      return limbGroups
    end
  end
  return nil
end

------------------------------------------------------------------------------------------------------------------------
local getSafeEndPhysicsJointNames = function(limb, tagMap) 
  local possibleJoints = tagMap:getSafeEndCandidates(limb)
  local numJoints = possibleJoints:size()
  local results = {}
  for i = 1, numJoints do 
    local joint = possibleJoints:at(i)
    results[joint:getName()] = true
  end
  return results
end

------------------------------------------------------------------------------------------------------------------------
local getPossibleArmAndHeadEndPhysicsJoints = function(limb, tagMap) 
  local ommit = getSafeEndPhysicsJointNames(limb, tagMap)
  local possibleJoints = tagMap:getAllArmAndHeadEndCandidates()
  local numJoints = possibleJoints:size()
  local results = {}
  for i = 1, numJoints do 
    local joint = possibleJoints:at(i)
    if not ommit[joint:getName()] then
      table.insert(results, joint)
    end
  end
  return results
end

------------------------------------------------------------------------------------------------------------------------

local getPossibleLegEndPhysicsJoints = function(limb, tagMap) 
  local ommit = getSafeEndPhysicsJointNames(limb, tagMap)
  local possibleJoints = tagMap:getAllLegEndCandidates()
  local numJoints = possibleJoints:size()
  local results = {}
  for i = 1, numJoints do 
    local joint = possibleJoints:at(i)
    if not ommit[joint:getName()] then
      table.insert(results, joint)
    end
  end
  return results
end

------------------------------------------------------------------------------------------------------------------------
local getSafeEndPhysicsJoints = function(limb, tagMap) 
  local possibleJoints = tagMap:getSafeEndCandidates(limb)
  local numJoints = possibleJoints:size()
  local results = {}
  for i = 1, numJoints do 
    table.insert(results, possibleJoints:at(i))
  end
  return results
end

------------------------------------------------------------------------------------------------------------------------
local listBasePhysicsJoints = function(limb, tagMap) 
  local baseJoints = limb:getSafeBaseCandidates()
  local numJoints = baseJoints:size()
  local results = {}
  for i = 1, numJoints do 
    table.insert(results, baseJoints:at(i))
  end
  return results
end

------------------------------------------------------------------------------------------------------------------------
local getEndJoints = function(limb, tagMap, selectedSet)
  local allPhysicsJoints = {}  
  local ommit = getSafeEndPhysicsJointNames(limb, tagMap)
  local scene = nmx.Application.new():getSceneByName("AssetManager")    
  local physRoot = anim.getPhysicsRigDataRoot(scene, selectedSet)
  local jointIt = nmx.NodeIterator.new(physRoot, nmx.Node.ClassTypeId())
  while jointIt:next() do
    local dataNode = jointIt:node()
    if dataNode and dataNode:is(nmx.PhysicsJointNode.ClassTypeId()) then
      if not ommit[dataNode] then
        table.insert(allPhysicsJoints, dataNode)
      end
    end
  end
  return allPhysicsJoints
end

------------------------------------------------------------------------------------------------------------------------
local getLimbValidityData = function(limb, limbSet, tagMap, selectedSet)
  local validityData = {}
  
  validityData.limb = limb
  validityData.limbName = limb:getName()
  validityData.limbType = "spine"
  
  local limbTypeMap = {}
  limbTypeMap[0] = "arm"
  limbTypeMap[1] = "leg"
  limbTypeMap[2] = "head"
  limbTypeMap[3] = "extra"
  limbTypeMap[4] = "spine"

  if limb:is("ExtremityNode") then
    validityData.limbType =  limbTypeMap[limb:findAttribute("Type"):asInt()]
  end

  validityData.valid =  limb:isValidLimb(tagMap)
  validityData.validBase = limb:isCurrentBaseValid(tagMap)

  validityData.safeEndJoints = getSafeEndPhysicsJoints(limb, tagMap)
  if validityData["limbType"] == "leg" then 
    validityData.possibleEndJoints = getPossibleLegEndPhysicsJoints(limb, tagMap)
  elseif validityData["limbType"] == "spine" then 
    validityData.possibleEndJoints = getEndJoints(limb, tagMap, selectedSet)
  else 
    validityData.possibleEndJoints = getPossibleArmAndHeadEndPhysicsJoints(limb, tagMap)
  end

  validityData.safeBaseJoints = listBasePhysicsJoints(limb, tagMap)
  return validityData
end

------------------------------------------------------------------------------------------------------------------------
local printLimbValidityData = function(validityData)
    local printPhysicsJointTable = function(jointTable)
      for i,v in ipairs(jointTable) do 
        print("   " .. v:getName())
      end
   end
   
   print("=============== " .. tostring(validityData.limbName) .. " ======================")
   print("   Type -  " .. tostring(validityData.limbType))
   print("   Valid - " .. tostring(validityData.valid))
   print("   Valid Base - " .. tostring(validityData.validBase))
   print("   ===  Safe End Joints === ")
   printPhysicsJointTable(validityData.safeEndJoints)
   print("   === Possible End Joints === ")
   printPhysicsJointTable(validityData.possibleEndJoints)
   print("   === Safe Base Joints === ")
   printPhysicsJointTable(validityData.safeBaseJoints)
end

------------------------------------------------------------------------------------------------------------------------
local createExtremityNode = function(selectedSet, extremityName, extremityType, endSelection)
  local app = nmx.Application.new()
  local scene = app:getSceneByName("AssetManager")
  local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

  local limbSet = findLimbSetNode(selectedSet)
  
  local sl = nmx.SelectionList.new()
  sl:add(endSelection)
  
  local commandReturn
  if extremityType == "Arm" then
    commandReturn = app:runCommand("Limb", "Create Limb", scene, sl, "Create Arm")

  elseif extremityType == "Leg" then
    commandReturn = app:runCommand("Limb", "Create Limb", scene, sl, "Create Leg")

  elseif extremityType == "Head" then
    commandReturn = app:runCommand("Limb", "Create Limb", scene, sl, "Create Head")

  else 
    app:logError("Couldn't find limb type " .. extremityType)
    return nil
  end

  -- If the command failed, bail out.
  if commandReturn:asString() ~= "kSuccess" then
    return nil
  end

  local newExtremity = limbSet:getLastChild()
  newExtremity:setLimbName(extremityName)

  scene:endChangeBlock(cbRef, changeBlockInfo("Create Extremity"))
  return newExtremity
end

------------------------------------------------------------------------------------------------------------------------
-- returns the Spine node
local findSpineNode = function(limbSetNode)
  local curObj = limbSetNode:getFirstChild()
  while curObj ~= nil and not curObj:is("SpineNode") do
    curObj = curObj:getNextSibling()
  end
  return curObj
end

------------------------------------------------------------------------------------------------------------------------
-- returns a table of the Extremity nodes
local findExtremityNodes = function(limbSetNode)
  local curObj = limbSetNode:getFirstChild()
  local extremityNodes = {}
  while curObj ~= nil do
    if curObj:is("ExtremityNode") then
      table.insert(extremityNodes, curObj)
    end
    curObj = curObj:getNextSibling()
  end
  return extremityNodes
end

------------------------------------------------------------------------------------------------------------------------
-- returns a table of the Extremity and Spine nodes
local findLimbNodes = function(limbSetNode)
  local limbNodes = {}
  local curObj = limbSetNode:getFirstChild()
  while curObj ~= nil do
    if curObj:is("ExtremityNode") or curObj:is("SpineNode") then
      table.insert(limbNodes, curObj)
    end
    curObj = curObj:getNextSibling()
  end
  return limbNodes
end

------------------------------------------------------------------------------------------------------------------------

local findPhysicsJointByName = function(selectedSet, name)
  local scene = nmx.Application.new():getSceneByName("AssetManager")    
  local physRoot = anim.getPhysicsRigDataRoot(scene, selectedSet)
  local jointIt = nmx.NodeIterator.new(physRoot, nmx.Node.ClassTypeId())
  while jointIt:next() do
    local dataNode = jointIt:node()
    if dataNode and dataNode:is(nmx.PhysicsJointNode.ClassTypeId()) and dataNode:getName() == name then
      return dataNode
    end
  end
  return nil
end

------------------------------------------------------------------------------------------------------------------------
local getSpineRootJoints = function(limb, selectedSet)
  -- This function currently returns all physics joints. The spine root can technically be anywhere, but only 
  -- a few places make sense
  local possibleRootPointNames = {}  
  local scene = nmx.Application.new():getSceneByName("AssetManager")    
  local physRoot = anim.getPhysicsRigDataRoot(scene, selectedSet)
  local jointIt = nmx.NodeIterator.new(physRoot, nmx.Node.ClassTypeId())
  while jointIt:next() do
    local dataNode = jointIt:node()
    if dataNode and dataNode:is(nmx.PhysicsJointNode.ClassTypeId()) then
      table.insert(possibleRootPointNames, dataNode)
    end
  end
  return possibleRootPointNames
end

------------------------------------------------------------------------------------------------------------------------
local createNewLimb = function(selectedSet, limbName, limbType, endSelection)
  return createExtremityNode(selectedSet, limbName, limbType, endSelection)
end

------------------------------------------------------------------------------------------------------------------------
local showNewLimbDialog = function(selectedSet)
  local dlg = ui.getWindow("NewLimbDialog")

  local okButton
  local dlgErrors = {}

  local updateOkButtonState = function ()
    -- check for any 'true' errors
    for k,v in pairs(dlgErrors) do
      if v == true then
        okButton:enable(false)
        return
      end
    end

    -- otherwise
    okButton:enable(true)
  end

  local limbTypeWidget
  local limbEndWidget
  local limbEndErrorLabel
  local updateEndCombo = function(self)
    local limbSet = findLimbSetNode(selectedSet)
    local tagMap = nmx.LimbTagMap.new()
    tagMap:initialise(limbSet)

    local endPosibilities
    if self:getValue() == "Leg" then
      endPosibilities = tagMap:getAllLegEndCandidates()
    else
      endPosibilities = tagMap:getAllArmAndHeadEndCandidates()
    end

    local endTable = { }
    for i = 1,endPosibilities:size() do
      table.insert(endTable, endPosibilities:at(i):getFirstInstance():getParentSgTransform():getName())
    end

    if table.getn(endTable) > 0 then
      dlgErrors["end"] = false
      limbEndErrorLabel:setShown(false)
      limbEndWidget:setShown(true)
    else
      dlgErrors["end"] = true
      limbEndErrorLabel:setShown(true)
      limbEndWidget:setShown(false)
    end

    limbEndWidget:setItems(endTable)
    updateOkButtonState()
  end

  local limbNameWidget
  local limbNameErrorLabel
  local onNameChanged = function(self)
    local limbSet = findLimbSetNode(selectedSet)
    local limbName = self:getValue()
    local existingNode = limbSet:findChild(limbName, true)

    if existingNode == nil then
      dlgErrors["name"] = false
      limbNameErrorLabel:setShown(false)
    else
      dlgErrors["name"] = true
      limbNameErrorLabel:setShown(true)
    end

    updateOkButtonState()
    dlg:doLayout()
  end

  local limbCreateErrorLabel

  if not dlg then
    dlg = ui.createModalDialog{name = "NewLimbDialog", caption = "Create Limb", centre = true, resize = false}
  end

  dlg:clear()

  dlg:beginVSizer()
    dlg:beginFlexGridSizer{ cols = 2, flags = "expand" }
      dlg:setFlexGridColumnExpandable(2)

      dlg:addStaticText{ text = "Name" }
      limbNameWidget = dlg:addTextBox{ name = "Name", flags = "expand", proportion = 1, onChanged = onNameChanged }

      dlg:addStaticText{ text = "Type" }
      limbTypeWidget = dlg:addComboBox{name = "Type", flags = "expand", items = limbTypes, onChanged = updateEndCombo }

      dlg:addStaticText{ text = "End" }
      dlg:beginHSizer{ flags = "expand" }
        limbEndWidget = dlg:addComboBox{ name = "End", flags = "expand", proportion = 1 }
        limbEndErrorLabel = dlg:addStaticText{ name = "EndErrorText", text = "No valid end parts" }
      dlg:endSizer()

      dlg:addStretchSpacer{ proportion = 1 }

      dlg:beginHSizer{ flags = "expand" }
        dlg:addStretchSpacer{ proportion = 1 }
        okButton = dlg:addButton{
          name = "Ok",
          label = "OK",
          onClick = function(self)
            local limbSet = findLimbSetNode(selectedSet)
            local tagMap = nmx.LimbTagMap.new()
            tagMap:initialise(limbSet)

            local endPosibilities
            if limbTypeWidget:getValue() == "Leg" then
              endPosibilities = tagMap:getAllLegEndCandidates()
            else
              endPosibilities = tagMap:getAllArmAndHeadEndCandidates()
            end

            local endSelection = { }
            for i = 1,endPosibilities:size() do
              if limbEndWidget:getValue() == endPosibilities:at(i):getFirstInstance():getParentSgTransform():getName() then
                endSelection = endPosibilities:at(i):getFirstInstance():getParentSgTransform()
                break
              end
            end

            local limbName = limbNameWidget:getValue()
            local existingNode = limbSet:findChild(limbName, true)
            if existingNode ~= nil then
              app.error("Limb name '" .. limbName .. "' already in use.")
              return
            end

            local newLimb = createNewLimb(selectedSet, limbNameWidget:getValue(), limbTypeWidget:getSelectedItem(), endSelection)

            if newLimb == nil then
              limbCreateErrorLabel:setShown(true)
              dlg:doLayout()
              return
            end

            componentEditor.reset()
            dlg:hide()
          end,
        }
        dlg:addButton{
          name = "CancelButton",
          label = "Cancel",
          onClick = function(self)
            dlg:hide()
          end,
        }
      dlg:endSizer()
    dlg:endSizer()

    limbNameErrorLabel = dlg:addStaticText{ name = "NameErrorText", text = "Limb name in use" }
    limbCreateErrorLabel = dlg:addStaticText{ name = "CreateErrorText", text = "Limb creation failed" }
    limbCreateErrorLabel:setShown(false)
  dlg:endSizer()
  
  local limbNameWidget = dlg:getChild("Name")
  limbNameWidget:setValue("Untitled")
  
  limbTypeWidget = dlg:getChild("Type")
  limbEndWidget = dlg:getChild("End")
  limbEndErrorLabel = dlg:getChild("EndErrorText")
  limbEndErrorLabel = dlg:getChild("EndErrorText")
  okButton = dlg:getChild("Ok")
  updateEndCombo(limbTypeWidget)
  onNameChanged(limbNameWidget)
  
  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
local headerFunction = function(selectedSet, panel)

    local toolbar = panel:addToolBar{
      name = "ToolBar",
    }

    addButton = toolbar:addButton{
      name = "AddButton",
      image = app.loadImage("additem.png"),
      helpText = "Add Limb",
      onClick = function(self) 
        showNewLimbDialog(selectedSet)
      end
    }

    removeButton = toolbar:addButton{
      name = "RemoveButton",
      image = app.loadImage("removeitem.png"),
      helpText = "Delete Limb",
      onClick = function(self)
        local app = nmx.Application.new()
        local scene = app:getSceneByName("AssetManager")

        local _, curObj = getCurrentLimb()
        local sl = nmx.SelectionList.new()
        sl:add(curObj)

        app:runCommand("Limb", "Delete limb", scene, sl)
        componentEditor.reset()
     end
    }

    moveUpButton = toolbar:addButton{
      name = "MoveUpButton",
      image = app.loadImage("moveup.png"),
      helpText = "Reorder Limb (move up)",
      onClick = function(self)
        local _, curObj = getCurrentLimb()
        if curObj and curObj:getPreviousSibling() then
          local scene = nmx.Application.new():getSceneByName("AssetManager")
          local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())
            curObj:moveUp()
          scene:endChangeBlock(cbRef, changeBlockInfo("Move Limb Up"))
          componentEditor.reset()
        end
      end
    }

    moveDownButton = toolbar:addButton{
      name = "MoveDownButton",
      image = app.loadImage("movedown.png"),
      helpText = "Reorder Limb (move down)",
      onClick = function(self)
        local _, curObj = getCurrentLimb()
        if curObj and curObj:getNextSibling() then
          local nextSibling = curObj:getNextSibling()
          local scene = nmx.Application.new():getSceneByName("AssetManager")
          local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())
            curObj:moveDown()
          scene:endChangeBlock(cbRef, changeBlockInfo("Move Limb Down"))
          componentEditor.reset()
        end
      end
    }
  
end

------------------------------------------------------------------------------------------------------------------------
local updateHeaderUI = function(selectedSet)
  local limbSet = findLimbSetNode(selectedSet)
  if limbSet == nil then
    moveDownButton:enable(false)
    removeButton:enable(false)
    moveUpButton:enable(false)
    addButton:enable(false)
  else
    local _, curObj = getCurrentLimb()
    
    addButton:enable(true)

    -- Can't delete the spine
    local currObjIsSpine = curObj ~= nil and curObj:is("SpineNode")
    removeButton:enable(curObj ~= nil and (not currObjIsSpine))
    
    -- Can't move the spine or past the spine
    local canMoveDown = (curObj ~= nil) 
                    and (not currObjIsSpine)
                    and (curObj:getNextSibling() ~= nil) 
                    and (not curObj:getNextSibling():is("SpineNode"))
    moveDownButton:enable(canMoveDown)
    
    -- Can't move the spine or past the spine
    local canMoveUp   = (curObj ~= nil) 
                    and (not currObjIsSpine)
                    and (curObj:getPreviousSibling() ~= nil) 
                    and (not curObj:getPreviousSibling():is("SpineNode"))
    moveUpButton:enable(canMoveUp)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- sets the currently selected limb
local setCurrentLimb = function(limbName, selectedSet)
  local currentLimb = getCurrentLimb()
  
  if currentLimb ~= limbName then
    if currentLimb then
      limbSections[currentLimb].rollup:setSelected(false)
    end
    
    if limbName then
      limbSections[limbName].rollup:setSelected(true)
    end
  end
  
  lastSelectedLimb = limbName
  updateHeaderUI(selectedSet)
end

----------------------------------------------------------------------------------------------------------------------
local updateIKSolverAttributes = function(limbObj)
  if limbObj ~= nil then
    local ikSection = ikSolverAttributesSections[limbObj:getName()]
    if ikSection ~= nil then
      local joints = nmx.sgTransformNodes.new()
      limbObj:getLimbTransforms(joints)
      local names = { }
      
      for i=1, joints:size() do
        -- insert at the front.
        table.insert(names, 1, joints:at(i):getName())
      end
      
      if table.getn(names) ~= ikSection["NumberOfJoints"] then
        ikSection["NumberOfJoints"] = table.getn(names)
        
        ikSection["OrientationWeights"]["Weights"]:setLabels(names)
        ikSection["PositionWeights"]["Weights"]:setLabels(names)
        ikSection["GuidePoseJoints"]:setLabels(names)    
        
        -- call these as if the size of the array has changed we may need to unhide the weight section
        ikSection["SizeChanged"]()
      end
    end
  end
end

----------------------------------------------------------------------------------------------------------------------
local updateCoupledLimitAttributes = function(limbObj)
  if limbObj ~= nil then
    local coupledLimitSection = CoupledLimitAttributesSections[limbObj:getName()]
    if coupledLimitSection ~= nil then
      local joints = nmx.sgTransformNodes.new()
      limbObj:getLimbTransforms(joints)
      local names = { }
      
      for i=1,joints:size() do
        -- insert at the front.
        table.insert(names, 1, joints:at(i):getName())
      end
      
      if table.getn(names) ~= coupledLimitSection["NumberOfJoints"] then
        coupledLimitSection["NumberOfJoints"] = table.getn(names)

        coupledLimitSection["TwistWeights"]:setLabels(names)
        coupledLimitSection["Swing1Weights"]:setLabels(names)
        coupledLimitSection["Swing2Weights"]:setLabels(names)    

        -- call these as if the size of the array has changed we may need to unhide the weight section
        coupledLimitSection["CoupledLimitEnabled"]:getOnChanged()()
      end
      
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local panelFunction = function(selectedSet, panel)

  limbSections = { }
  local limbSet = findLimbSetNode(selectedSet)
  if limbSet == nil then
    panel:addStaticText{ text = "Limbs require a physics rig." }
    updateHeaderUI(selectedSet)
    return
  end
  
  local spine = findSpineNode(limbSet)
  local extremityTable = findExtremityNodes(limbSet)

  local tagMap = nmx.LimbTagMap.new()
  tagMap:initialise(limbSet)
  
  ----------------------------------------------------------------------------------------------------------------------
  local addDetails = function(panel, limb)
    panel:beginVSizer{ flags = "expand", proportion = 1 }
    local limbSet = limb:getLimbSet()
    local limbIndex = limbSet:getExportIndex(limb)
    local exportIndexString = "Limb Index - " .. tostring(limbIndex)
    panel:addStaticText{ text = exportIndexString}
    panel:addStaticText{ text = "Limb Part Indices"}:setFont("bold")
    local limbBodies = nmx.PhysicsJointNodes.new()
    limb:getLimbPhysicsJoints(limbBodies, true)
    local index = 0
    for i=limbBodies:size(),1, -1 do
       panel:addStaticText{ text =("    " .. tostring(index) .." - " .. limbBodies:at(i):getName())}
       index = index + 1
     end
    panel:endSizer()
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local addAttitionalBodiesAttributes = function(panel, limb)
    local bodies = { }
    
    local limbEnd = limb:findAttribute("End")   
    local limbBase = limb:findAttribute("Base")   
    local it = nmx.NodeIterator.new(limb:getDatabase():getRoot(), nmx.PhysicsBodyNode.ClassTypeId())
    while it:next() do
      local instance = it:node():getFirstInstance()
      if instance ~= nil then
        local transform = instance:getParent()
        if transform ~= nil then
          if it:node() ~= limbEnd and it:node() ~= limbBase then
            table.insert(bodies, transform)
          end
        end
      end
    end
    
    panel:beginVSizer{ flags = "expand", proportion = 1 }
      local widget = panel:addAttributeWidget{ attributes = { limb:findAttributeArray("AdditionalBodies") }, flags = "expand", 
        references = bodies, resizable = true, uniqueReferences = true }
    panel:endSizer()
    
    return widget
  end

  ----------------------------------------------------------------------------------------------------------------------
  ikSolverAttributesSections = { }
  local addIKSolverAttributes = function(IkSolverPanel, limbObj)

    local addNMX2AttributeWidget = function( panel, node, attribute )
      return panel:addAttributeWidget{ attributes = { node:findAttribute(attribute) }, flags = "expand" }
    end

    local addWeightsSection = function( panel, node, modeAttr, weightsAttr)
      local returnValue = { }
      panel:beginFlexGridSizer{ cols = 2, flags = "expand" }
        panel:setFlexGridColumnExpandable(2)
        
        panel:addStaticText{ text = "Mode" }
        returnValue["Mode"] = addNMX2AttributeWidget( panel, node, modeAttr )
        
        local weightsText = panel:addStaticText{ text = "Weights" }
        returnValue["Weights"] = addNMX2AttributeWidget( panel, limbObj, weightsAttr )
        
        function refreshWeightsUI(noSizeChange)
          local shown = node:findAttribute(modeAttr):asInt() == 1 and -- 1 is manual, 0 is auto
            node:findAttribute(weightsAttr):asFloatArray():size() > 0
            
          weightsText:setShown(shown)
          returnValue["Weights"]:setShown(shown)
          
          if noSizeChange ~= true then
            panel:bestSizeChanged()
          end
        end
        
        refreshWeightsUI(true)
        returnValue["Mode"]:setOnChanged(refreshWeightsUI)
      panel:endSizer()
      return returnValue
    end

    local returnValue = { }
    IkSolverPanel:beginVSizer{ flags = "expand", proportion = 1 }
      
      local label = IkSolverPanel:addStaticText{ text = "Guide Pose (Style)", font = "bold" }
      componentEditor.bindHelpToWidget(label, kHelp.RedundancyControlHelpText)
      IkSolverPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
        IkSolverPanel:setFlexGridColumnExpandable(2)
        
        IkSolverPanel:addStaticText{ text = "Weight" }
        returnValue["NeutralPoseWeight"] = addNMX2AttributeWidget( IkSolverPanel, limbObj, "NeutralPoseWeight" )
        componentEditor.bindHelpToWidget(returnValue["NeutralPoseWeight"], kHelp.RedundancyControlWeightHelpText)
      IkSolverPanel:endSizer()

      local label = IkSolverPanel:addStaticText{ text = "Guide Pose (input bias)", font = "bold" }
      componentEditor.bindHelpToWidget(label, kHelp.GuidePoseHelpText)
      IkSolverPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
        IkSolverPanel:setFlexGridColumnExpandable(2)
      
        IkSolverPanel:addStaticText{ text = "Weight" }
        returnValue["GuidePoseWeight"] = addNMX2AttributeWidget( IkSolverPanel, limbObj, "GuidePoseWeight" )
        returnValue["GuidePoseJointsText"] = IkSolverPanel:addStaticText{ text = "Joints" }
        returnValue["GuidePoseJoints"] = addNMX2AttributeWidget( IkSolverPanel, limbObj, "GuidePoseJoints" )
        componentEditor.bindHelpToWidget(returnValue["GuidePoseWeight"], kHelp.GuidePoseWeightHelpText)
        componentEditor.bindHelpToWidget(returnValue["GuidePoseJoints"], kHelp.GuidePoseJointsHelpText)
      IkSolverPanel:endSizer() 

      componentEditor.addSeparator(IkSolverPanel);

      local orientationLabel = IkSolverPanel:addStaticText{ text = "Orientation", font = "bold" }
      componentEditor.bindHelpToWidget(orientationLabel, kHelp.OrientationHelpText)
      returnValue["OrientationWeights"] = addWeightsSection( IkSolverPanel, limbObj, "OrientationWeightMode", "OrientationWeights" )      
      componentEditor.bindHelpToWidget(returnValue["OrientationWeights"]["Mode"], kHelp.OrientationModeHelpText)
      componentEditor.bindHelpToWidget(returnValue["OrientationWeights"]["Weights"], kHelp.OrientationWeightsHelpText)

      local positionLabel = IkSolverPanel:addStaticText{ text = "Position", font = "bold" }
      componentEditor.bindHelpToWidget(positionLabel, kHelp.PositionHelpText)
      returnValue["PositionWeights"] = addWeightsSection( IkSolverPanel, limbObj, "PositionWeightMode", "PositionWeights" )
      componentEditor.bindHelpToWidget(returnValue["PositionWeights"]["Mode"], kHelp.PositionModesHelpText)
      componentEditor.bindHelpToWidget(returnValue["PositionWeights"]["Weights"], kHelp.PositionWeightsHelpText)
     
    IkSolverPanel:endSizer()

    returnValue["SizeChanged"] = function()
      -- hide array sections if the array size is 0
      returnValue["OrientationWeights"]["Mode"]:getOnChanged()()
      returnValue["PositionWeights"]["Mode"]:getOnChanged()()
      
      local shown = limbObj:findAttribute("GuidePoseJoints"):asBoolArray():size() > 0
      returnValue["GuidePoseJointsText"]:setShown(shown)
      returnValue["GuidePoseJoints"]:setShown(shown)
    end
    
    -- initiate the number of limb joints
    returnValue["NumberOfJoints"] = 0
    
    -- index by name because user data class wrappers change with scope
    ikSolverAttributesSections[limbObj:getName()] = returnValue
  end


  ----------------------------------------------------------------------------------------------------------------------
  CoupledLimitAttributesSections = { }
  local addCoupledLimitAttributes = function(CoupledLimitsPanel, limbObj)

    local addNMX2AttributeWidget = function( panel, node, attribute )
      return panel:addAttributeWidget{ attributes = { node:findAttribute(attribute) }, flags = "expand" }
    end

    
    local returnValue = { }
    CoupledLimitsPanel:beginVSizer{ flags = "expand", proportion = 1 }
      CoupledLimitsPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
        CoupledLimitsPanel:setFlexGridColumnExpandable(2)
      
        returnValue["CoupledLimitText"] = CoupledLimitsPanel:addStaticText{ text = "enabled" }
        returnValue["CoupledLimitEnabled"] = addNMX2AttributeWidget( CoupledLimitsPanel, limbObj, "CoupledLimitEnabled" )

        returnValue["DistanceText"] = CoupledLimitsPanel:addStaticText{ text = "distance" }
        returnValue["CoupledLimitDistance"] = addNMX2AttributeWidget( CoupledLimitsPanel, limbObj, "CoupledLimitDistance" )
        
        returnValue["StiffnessText"] = CoupledLimitsPanel:addStaticText{ text = "stiffness" }
        returnValue["CoupledLimitStiffness"] = addNMX2AttributeWidget( CoupledLimitsPanel, limbObj, "CoupledLimitStiffness" )
        
        returnValue["TwistText"] = CoupledLimitsPanel:addStaticText{ text = "Twist Weights" }
        returnValue["TwistWeights"] = addNMX2AttributeWidget( CoupledLimitsPanel, limbObj, "CoupledLimitTwistBendScale" )

   
        returnValue["Swing1Text"] = CoupledLimitsPanel:addStaticText{ text = "Swing 1 Weights" }
        returnValue["Swing1Weights"] = addNMX2AttributeWidget( CoupledLimitsPanel, limbObj, "CoupledLimitSwing1BendScale" )

   
        returnValue["Swing2Text"] = CoupledLimitsPanel:addStaticText{ text = "Swing 2 Weights" }
        returnValue["Swing2Weights"] = addNMX2AttributeWidget( CoupledLimitsPanel, limbObj, "CoupledLimitSwing2BendScale" )
        
        componentEditor.bindHelpToWidget(returnValue["CoupledLimitEnabled"], kHelp.CoupledLimitEnabledHelpText)
        componentEditor.bindHelpToWidget(returnValue["CoupledLimitText"], kHelp.CoupledLimitEnabledHelpText)
        componentEditor.bindHelpToWidget(returnValue["DistanceText"], kHelp.CoupledLimitDistanceHelpText)
        componentEditor.bindHelpToWidget(returnValue["CoupledLimitDistance"], kHelp.CoupledLimitDistanceHelpText)
        componentEditor.bindHelpToWidget(returnValue["StiffnessText"], kHelp.CoupledLimitStiffnessHelpText)
        componentEditor.bindHelpToWidget(returnValue["CoupledLimitStiffness"], kHelp.CoupledLimitStiffnessHelpText)

        componentEditor.bindHelpToWidget(returnValue["TwistText"], kHelp.TwistWeightsHelpText)
        componentEditor.bindHelpToWidget(returnValue["TwistWeights"], kHelp.TwistWeightsHelpText)
        componentEditor.bindHelpToWidget(returnValue["Swing1Text"], kHelp.Swing1Text)
        componentEditor.bindHelpToWidget(returnValue["Swing1Weights"], kHelp.Swing1Text)
        componentEditor.bindHelpToWidget(returnValue["Swing2Text"], kHelp.Swing2Text)
        componentEditor.bindHelpToWidget(returnValue["Swing2Weights"], kHelp.Swing2Text)

        function refreshCoupledLimitsUI()
          local shown = limbObj:findAttribute("CoupledLimitEnabled" ):asBool()
          returnValue["CoupledLimitDistance"]:setShown(shown)
          returnValue["CoupledLimitStiffness"]:setShown(shown)
          returnValue["TwistWeights"]:setShown(shown)
          returnValue["Swing1Weights"]:setShown(shown)
          returnValue["Swing2Weights"]:setShown(shown)
          
          returnValue["DistanceText"]:setShown(shown)
          returnValue["StiffnessText"]:setShown(shown)
          returnValue["TwistText"]:setShown(shown)
          returnValue["Swing1Text"]:setShown(shown)
          returnValue["Swing2Text"]:setShown(shown)
          
          if noSizeChange ~= true then
            CoupledLimitsPanel:bestSizeChanged()
          end
        end
        
        refreshCoupledLimitsUI(true)
        returnValue["CoupledLimitEnabled"]:setOnChanged(refreshCoupledLimitsUI)
        
      CoupledLimitsPanel:endSizer()

    CoupledLimitsPanel:endSizer()
    
    -- initiate the number of limb joints
    returnValue["NumberOfJoints"] = 0
    
    -- index by name because user data class wrappers change with scope
    CoupledLimitAttributesSections[limbObj:getName()] = returnValue
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local getLimbEndJoint = function(limb)
    local endAttribute = limb:findAttribute("End")
    if endAttribute:hasInputConnection() then 
      local inputConnection = endAttribute:getInput()
      local endJoint = inputConnection:getNode()
      return endJoint
    end
    return nil
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local getLimbBaseJoint = function(limb)
    local endAttribute = limb:findAttribute("Base")
    if endAttribute:hasInputConnection() then 
      local inputConnection = endAttribute:getInput()
      local baseJoint = inputConnection:getNode()
      return baseJoint
    end
    return nil
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local getSpineRootJoint = function(spine)
    local rootAttribute = spine:findAttribute("Root")
    if rootAttribute:hasInputConnection() then 
      local inputConnection = rootAttribute:getInput()
      local rootJoint = inputConnection:getNode()
      return rootJoint
    end
    return nil
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local addEndComboBox = function(limbObj, panel)
    return panel:addAttributeWidget{ attributes = { limbObj:findAttribute("End") }, flags = "expand"}
  end
   
  ----------------------------------------------------------------------------------------------------------------------
  local addBaseComboBox = function(limbObj, panel)
    return panel:addAttributeWidget{ attributes = { limbObj:findAttribute("Base") }, flags = "expand"}
  end
   
  ----------------------------------------------------------------------------------------------------------------------
  local addRootComboBox = function(limbObj, panel)
    return panel:addAttributeWidget{ attributes = { limbObj:findAttribute("Root") }, flags = "expand"}
  end

  ----------------------------------------------------------------------------------------------------------------------
  local addColourSwatch = function(limbObj, panel)
    return panel:addAttributeWidget{ attributes = { limbObj:findAttribute("Colour") }, size = { width = 18 ,  height = 18 } }
  end
   
  ----------------------------------------------------------------------------------------------------------------------
  local addVisibilityCheckBox = function(limbObj, panel) 
    panel:beginVSizer()
      panel:addVSpacer(2)
      local result = panel:addPanel{
        name = "VisibilityPanel",
        flags = "expand;parentBackground",
        proportion = 1
      }
    panel:endSizer()
    return result
  end
   
  ------------------------------------------------------------------------------------------------------------------------
  function addNameWidgets(limbObj, panel, headerColour)

    local limbName = limbObj:getName()
    local nameWidget, nameLabel, contextualMenu

    function editName(self)
      panel:freeze()
        nameWidget:setShown(true)
        nameLabel:setShown(false)
      panel:rebuild()
     
      nameWidget:setFocus{navigation = true}
    end

    function stopEditingName(self)
      panel:freeze()
         nameWidget:setShown(false)
         nameLabel:setShown(true)
      panel:rebuild()
    end

    function nameChanged(self)
      local oldName = limbName
      limbName = limbObj:getName()
      nameLabel:setLabel(limbName)

      -- update the list of rollups to reflect the new name
      if limbName ~= oldName then
         limbSections[limbName] = limbSections[oldName]
         limbSections[oldName] = nil
      end

      -- update the last selected rollup
      if lastSelectedLimb == oldName then
         lastSelectedLimb = limbName
      end
    end

    function clickOnName(self)
       setCurrentLimb(limbName, selectedSet) 
    end

    function rightClickOnName(self)
       contextualMenu:popup{x = 0, y = 0}
    end

    -- Add a the widget that will be responsible for the renaming, and hide it
    nameWidget = panel:addNameWidget{ nodes = { limbObj }, proportion = 1}
    nameWidget:setShown(false)
    nameWidget:setOnEnter(stopEditingName)
    nameWidget:setOnChanged(nameChanged)

    -- Add the button that is visible when we are not renaming
    nameLabel = panel:addStaticText{text = limbName, name=limbName, font = "largebold", labelAlign = "left", flags="disableTextHighlight", proportion = 1 }
    nameLabel:setOnLeftDown(clickOnName)
    nameLabel:setOnRightUp(rightClickOnName)
    nameLabel:setBackgroundColour(headerColour)

    -- Create the popum menu that is visble when you right click on the button
    contextualMenu = nameLabel:createPopupMenu()
    contextualMenu:addItem{ label = "Edit Name...", onClick = editName }
    
    return nameLabel;
  end

  ----------------------------------------------------------------------------------------------------------------------
  local selectLimbAttributeNode = function(scene, limb, attrName)
    local node = nmx.getAttrNodeInput(limb, attrName)
    if node ~= nil then
      sl = nmx.SelectionList.new()
      sl:add(node)
      scene:setSelectionList(sl)
      return true
    end
    return false
  end

  ----------------------------------------------------------------------------------------------------------------------
  mkIkSolverAttributesFunction = function(limbObj, rollPanel, section)
    return function(self)
      if self:isExpanded() and not section.hasIkSolverAttributesRollupBeenBuilt then
        section.hasIkSolverAttributesRollupBeenBuilt = true
        addIKSolverAttributes(rollPanel, limbObj)
        updateIKSolverAttributes(limbObj)
      end
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  mkAttitionalBodiesFunction = function(limbObj, rollPanel, section)
    return function(self)
      if self:isExpanded() and not section.hasAttitionalBodiesBeenBuilt then
        section.hasAttitionalBodiesBeenBuilt = true
        local additionalBodies = addAttitionalBodiesAttributes(rollPanel, limbObj)
        componentEditor.bindHelpToWidget(additionalBodies, kHelp.AdditionalBodiesHelpText)
      end
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  mkCoupledLimitAttributesFunction = function(limbObj, rollPanel, section)
    return function(self)
      if self:isExpanded() and not section.hasCoupledLimitAttributesBeenBuilt then
        section.hasCoupledLimitAttributesBeenBuilt = true
        addCoupledLimitAttributes(rollPanel, limbObj)
        updateCoupledLimitAttributes(limbObj)
      end
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  mkDetailsFunction = function(limbObj, rollPanel, section)
    return function(self)
      if self:isExpanded() and not section.hasDetailsBeenBuilt then
        section.hasDetailsBeenBuilt = true
        addDetails(rollPanel, limbObj)
      end
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  mkExpandCollapseSpineFunction = function(spineObj, rollPanel, section)
    return function(self)
      if self:isExpanded() and not section.hasBeenBuilt then
        section.hasBeenBuilt = true;
        rollPanel:setBackgroundColour("dialogDarkerTint")
        rollPanel:beginVSizer()

         rollPanel:beginFlexGridSizer{ cols = 4, flags = "expand" }
         rollPanel:setFlexGridColumnExpandable(4)

           rollPanel:addHSpacer(5)
           local selectRoot = rollPanel:addButton{image = limbRoot, size = { width = 16 ,  height = 16 },
             onClick = function ()
               selectLimbAttributeNode(scene, spineObj, "RootLocator")
             end
           }
           componentEditor.bindHelpToWidget(selectRoot, kHelp.SelectLimbRootHelpText)
           rollPanel:addStaticText{ text = "Root"}
           section.rootComboBox = addRootComboBox(spineObj, rollPanel)
           componentEditor.bindHelpToWidget(section.rootComboBox, kHelp.LimbRootHelpText)

           rollPanel:addHSpacer(5)
           local selectEnd = rollPanel:addButton{image = limbEnd, size = { width = 16 ,  height = 16 },
             onClick = function ()
               selectLimbAttributeNode(scene, spineObj, "EndLocator")
             end
           }
           componentEditor.bindHelpToWidget(selectEnd, kHelp.SelectLimbEndHelpText)
           rollPanel:addStaticText{ text = "End"}
           section.endComboBox = addEndComboBox(spineObj, rollPanel)
           componentEditor.bindHelpToWidget(section.endComboBox, kHelp.LimbEndHelpText)

           rollPanel:addHSpacer(5)
           local selectLimit = rollPanel:addButton{image = reachLimit, size = { width = 16 ,  height = 16 },
             onClick = function ()
               selectLimbAttributeNode(scene, spineObj, "ReachConnection")
             end
           }
           componentEditor.bindHelpToWidget(selectLimit, kHelp.SelectLimbReachLimitHelpText)
           rollPanel:addStaticText{ text = "Base"}
           section.baseComboBox = addBaseComboBox(spineObj, rollPanel)
           componentEditor.bindHelpToWidget(section.baseComboBox, kHelp.LimbBaseHelpText)
         rollPanel:endSizer()

         -- AttitionalBodies
         local additionalBodiesRollup = rollPanel:addRollup{label = "Additional Bodies", flags = "expand", name = "AdditionalBodies"}
         additionalBodiesRollup:expand(false)
         additionalBodiesRollup:setOnExpandCollapse(mkAttitionalBodiesFunction(spineObj, additionalBodiesRollup:getPanel(), section))

         -- IkSolverAttributes
         local IkSolverAttributesRollup  = rollPanel:addRollup{label = "IK Solver", flags = "expand", name = "IkSolver"}
         IkSolverAttributesRollup:expand(false)
         IkSolverAttributesRollup:setOnExpandCollapse(mkIkSolverAttributesFunction(spineObj, IkSolverAttributesRollup:getPanel(), section))

         -- CoupledLimits
         local CoupledLimitsRollup  = rollPanel:addRollup{label = "Coupled Limits", flags = "expand", name = "CoupledLimits"}
         CoupledLimitsRollup:expand(false)
         local CoupledLimitsPanel = CoupledLimitsRollup:getPanel()
         CoupledLimitsRollup:setOnExpandCollapse(mkCoupledLimitAttributesFunction(spineObj, CoupledLimitsRollup:getPanel(), section))
         
         -- Details
         local DetailsRollup  = rollPanel:addRollup{label = "Details", flags = "expand", name = "Details"}
         DetailsRollup:expand(false)
         local DetailsPanel = DetailsRollup:getPanel()
         DetailsRollup:setOnExpandCollapse(mkDetailsFunction(spineObj, DetailsRollup:getPanel(), section))
         
        rollPanel:endSizer()
        local selectedSet = getSelectedAssetManagerAnimSet()
        validateFunction(selectedSet, true)
      end
    end
  end
 
     
  ----------------------------------------------------------------------------------------------------------------------
  mkExpandCollapseExtremityFunction = function(extremityObj, rollPanel, section)
    return function(self)
      if self:isExpanded() and not section.hasBeenBuilt then
        section.hasBeenBuilt = true;
        rollPanel:beginVSizer()

        rollPanel:beginFlexGridSizer{ cols = 4, flags = "expand" }
          rollPanel:setFlexGridColumnExpandable(4)
          
            rollPanel:addHSpacer(5)
            local selectEnd = rollPanel:addButton{image = limbEnd, size = { width = 16 ,  height = 16 },
             onClick = function ()
               selectLimbAttributeNode(scene, extremityObj, "EndLocator")
             end
            }
            componentEditor.bindHelpToWidget(selectEnd, kHelp.SelectLimbEndHelpText)
            rollPanel:addStaticText{ text = "End"}
            section.endComboBox = addEndComboBox(extremityObj, rollPanel)
            componentEditor.bindHelpToWidget(section.endComboBox, kHelp.LimbEndHelpText)

            rollPanel:addHSpacer(5)
            local selectLimit = rollPanel:addButton{image = reachLimit, size = { width = 16 ,  height = 16 },
             onClick = function ()
               selectLimbAttributeNode(scene, extremityObj, "ReachConnection")
             end
            }
            componentEditor.bindHelpToWidget(selectLimit, kHelp.SelectLimbReachLimitHelpText)
            rollPanel:addStaticText{ text = "Base"}
            section.baseComboBox = addBaseComboBox(extremityObj, rollPanel)
            componentEditor.bindHelpToWidget(section.baseComboBox, kHelp.LimbBaseHelpText)
         rollPanel:endSizer()
         
         -- AttitionalBodies
         local additionalBodiesRollup = rollPanel:addRollup{label = "Additional Bodies", flags = "expand", name = "AdditionalBodies"}
         additionalBodiesRollup:expand(false)
         additionalBodiesRollup:setOnExpandCollapse(mkAttitionalBodiesFunction(extremityObj, additionalBodiesRollup:getPanel(), section))

         -- IkSolverAttributes
         local IkSolverAttributesRollup  = rollPanel:addRollup{label = "IK Solver", flags = "expand", name = "IkSolver"}
         IkSolverAttributesRollup:expand(false)
         IkSolverAttributesRollup:setOnExpandCollapse(mkIkSolverAttributesFunction(extremityObj, IkSolverAttributesRollup:getPanel(), section))

         -- CoupledLimits
         local CoupledLimitsRollup  = rollPanel:addRollup{label = "Coupled Limits", flags = "expand", name = "CoupledLimits"}
         CoupledLimitsRollup:expand(false)
         CoupledLimitsRollup:setOnExpandCollapse(mkCoupledLimitAttributesFunction(extremityObj, CoupledLimitsRollup:getPanel(), section))
         
         -- Details
         local DetailsRollup  = rollPanel:addRollup{label = "Details", flags = "expand", name = "Details"}
         DetailsRollup:expand(false)
         DetailsRollup:setOnExpandCollapse(mkDetailsFunction(extremityObj, DetailsRollup:getPanel(), section))
         
        rollPanel:endSizer()
       
        local selectedSet = getSelectedAssetManagerAnimSet()
        validateFunction(selectedSet, true)
      end
    end
  end

  ----------------------------------------------------------------------------------------------------------------------
  local addSpineSection = function(spineObj)
    local limbName = spineObj:getName()
    local section = {
      rollup = panel:addRollup{label = "", flags = "expand;advanced", name = limbName },
      limbObj = spineObj
    }
    -- Collapse it initally. The rebuild function only runs when the rollup changes
    -- from collapsed to expanded. If it starts expanded, the function wont run when
    -- expand(true) is called.
    section.rollup:expand(false)

    local scene = nmx.Application.new():getSceneByName("AssetManager")

    limbSections[limbName] = section
    local mainRollup = section.rollup
    local headerPanel = mainRollup:getHeader()
    local rollPanel = mainRollup:getPanel()

    headerPanel:beginHSizer()
      local colourSwatch = addColourSwatch(spineObj, headerPanel)
      section.visibilityCheckbox = addVisibilityCheckBox(spineObj, headerPanel)
      local limbName = addNameWidgets(spineObj, headerPanel, "controlBackground")
    headerPanel:endSizer()

    componentEditor.bindHelpToWidget(colourSwatch, kHelp.ColourSwatchHelpText)
    componentEditor.bindHelpToWidget(section.visibilityCheckbox, kHelp.LimbVisibilityHelpText)
    componentEditor.bindHelpToWidget(limbName, kHelp.LimbNameHelpText)

    mainRollup:setSelected(false)
    mainRollup:setHeaderBackgroundColour("controlBackground")
    mainRollup:setOnSelectionChanged(function() updateHeaderUI(selectedSet) end)
    mainRollup:setOnExpandCollapse(mkExpandCollapseSpineFunction(spineObj, rollPanel, section))
    mainRollup:expand(not spineObj:isValidLimb(tagMap))
  end

  ----------------------------------------------------------------------------------------------------------------------
  local addExtremitySection = function(extremityObj)
    local extremityName = extremityObj:getName()
    local section = {
      rollup = panel:addRollup{label = "", flags = "advanced;expand", name = extremityName },
      limbObj = extremityObj
    }
    -- Collapse it initally. The rebuild function only runs when the rollup changes
    -- from collapsed to expanded. If it starts expanded, the function wont run when
    -- expand(true) is called.
    section.rollup:expand(false)

    local scene = nmx.Application.new():getSceneByName("AssetManager")

    limbSections[extremityName] = section
    local mainRollup = limbSections[extremityName].rollup
    local headerPanel = mainRollup:getHeader()
    local rollPanel = mainRollup:getPanel()

    headerPanel:beginHSizer()
      local colourSwatch = addColourSwatch(extremityObj, headerPanel)
      section.visibilityCheckbox = addVisibilityCheckBox(extremityObj, headerPanel)
      local limbName = addNameWidgets(extremityObj, headerPanel, "dialogDarkerTint")
    headerPanel:endSizer()
    
    componentEditor.bindHelpToWidget(colourSwatch, kHelp.ColourSwatchHelpText)
    componentEditor.bindHelpToWidget(section.visibilityCheckbox, kHelp.LimbVisibilityHelpText)
    componentEditor.bindHelpToWidget(limbName, kHelp.LimbNameHelpText)

    mainRollup:setSelected(false)
    mainRollup:setOnSelectionChanged(function() updateHeaderUI(selectedSet) end)
    mainRollup:setOnExpandCollapse(mkExpandCollapseExtremityFunction(extremityObj, rollPanel, section))
    mainRollup:expand(not extremityObj:isValidLimb(tagMap))
  end

  ----------------------------------------------------------------------------------------------------------------------
  addSpineSection(spine)
  for extremityIndex, extremityObj in ipairs(extremityTable) do 
    addExtremitySection(extremityObj)
  end

  -- select the last selected limb if possible
  if limbSections[lastSelectedLimb] then
    limbSections[lastSelectedLimb].rollup:setSelected(true)
  end

  validateFunction(selectedSet, true)
  updateHeaderUI(selectedSet)
     
  local checkForStructuralChanges = function()
    local selectedSet = getSelectedAssetManagerAnimSet()
    local limbSet = findLimbSetNode(selectedSet)
    local limbNodes = findLimbNodes(limbSet)
    local sectionCount = 0
    
    for i,v in pairs(limbSections) do
      sectionCount = sectionCount + 1
    end
    
    if sectionCount ~= table.getn(limbNodes) then
      componentEditor.reset();
    end
  end
  
  componentEditor.addOnPhysicsRigChanged(checkForStructuralChanges)
end

------------------------------------------------------------------------------------------------------------------------
if not mcn.inCommandLineMode() then
  registerEventHandler("mcPhysicsRigFileOpenEnd", function()
    componentEditor.reset()
    componentEditor.updateComponentValidity("Limbs")
  end)
end

------------------------------------------------------------------------------------------------------------------------
validateFunction = function(selectedSet, isCurrentComponent)
  if not anim.isRigValid() then
    return "Inactive"
  end

  local limbSet = findLimbSetNode(selectedSet)
  local result = "Ok"
  if limbSet == nil then
    result = "Error"
  else
    local limbTable = findLimbNodes(limbSet)

    local tagMap = nmx.LimbTagMap.new()
    tagMap:initialise(limbSet) 

    for i, limbObj in ipairs(limbTable) do 
      local validityData = getLimbValidityData(limbObj, limbSet, tagMap, selectedSet)
      local limbName = limbObj:getName()
      local section = limbSections[limbName]

      if isCurrentComponent and section then
        local limbPanel = section.rollup:getPanel()
        limbPanel:suspendLayout()
        limbPanel:freeze()
    
        updateIKSolverAttributes(limbObj)
        updateCoupledLimitAttributes(limbObj)
        
        -- if there are no limb bodies, then add a dummy check box, otherwise add a checkbox linked to all the visible attributes
        -- of all the bodies transforms.
        --
        section.visibilityCheckbox:clear()
        local limbBodies = nmx.sgShapeNodes.new()
        limbObj:getLimbBodies(limbBodies, true)
        if limbBodies:size() then
          local limbBodyTransformVisibilityTable = { }
          for i=1,limbBodies:size() do
            table.insert(limbBodyTransformVisibilityTable, limbBodies:at(i):getParent():findAttribute("Visible"))
          end
          
          table.insert(limbBodyTransformVisibilityTable, limbObj:getLimbEndLocator():getParent():findAttribute("Visible"))
          table.insert(limbBodyTransformVisibilityTable, limbObj:getLimbReachNode():getParent():findAttribute("Visible"))
          
          if limbObj:is(nmx.SpineNode.ClassTypeId()) then
            table.insert(limbBodyTransformVisibilityTable, limbObj:getLimbRootLocator():getParent():findAttribute("Visible"))
          end
          
          section.visibilityCheckbox:beginVSizer{ flags = "expand", proportion = 1 }
            section.visibilityCheckbox:addAttributeWidget{ attributes = limbBodyTransformVisibilityTable }
          section.visibilityCheckbox:endSizer()
        else
          section.visibilityCheckbox:beginVSizer{ flags = "expand", proportion = 1 }
            section.visibilityCheckbox:addCheckBox{ checked = true }
          section.visibilityCheckbox:endSizer()
        end
        
        -- end
        if section.endComboBox then
          local joints
          if validityData.limbType == "spine" then
            -- temporary code
            joints = validityData.possibleEndJoints
            section.endComboBox:setReferences(joints)
            local attr = limbObj:findAttribute("End")
            section.endComboBox:setError( (not attr:hasInputConnection()) 
                                       or (not contains(joints, attr:getInput():getNode())))
          else
            joints =
            {  
              { name = "Safe", references = validityData.safeEndJoints}, 
              { name = "Possible", references = validityData.possibleEndJoints} 
            }
            section.endComboBox:setReferences(joints)
            local attr = limbObj:findAttribute("End")
            section.endComboBox:setError( (not attr:hasInputConnection()) 
                                       or (not contains(validityData.safeEndJoints, attr:getInput():getNode())))
          end
        end

        -- base
        if section.baseComboBox then
          local joints =  
          {  
            { name = "Safe", references = validityData.safeBaseJoints} 
          }
          section.baseComboBox:setReferences(joints)
          local attr = limbObj:findAttribute("Base")
          section.baseComboBox:setError( (not attr:hasInputConnection()) 
                                      or (not contains(validityData.safeBaseJoints, attr:getInput():getNode())))
        end
        
        -- root
        if section.rootComboBox then
          local joints = getSpineRootJoints(limbObj, selectedSet);
          section.rootComboBox:setReferences(joints)
          local attr = limbObj:findAttribute("Root")
          section.rootComboBox:setError( (not attr:hasInputConnection()) 
                                     or  (not contains(joints, attr:getInput():getNode())))
        end
        
        -- rollup
        if section.rollup then
          if validityData.valid then
            section.rollup:setHeaderIcon(nil)
          else
            section.rollup:setHeaderIcon(errorIcon)
          end
        end
        
        limbPanel:thaw()
        limbPanel:resumeLayout()
      end
   
      if not validityData.valid then
        result = "Error"
      end
    end
  end

  return result
end

components.register("Limbs", {"Euphoria"}, validateFunction, panelFunction, headerFunction,
  {
    CanUndo = function() return true end,
    CanRedo = function() return true end,
    Undo = function() nmx.Application.new():getSceneByName("AssetManager"):undo() end,
    Redo = function() nmx.Application.new():getSceneByName("AssetManager"):redo() end,
  })
