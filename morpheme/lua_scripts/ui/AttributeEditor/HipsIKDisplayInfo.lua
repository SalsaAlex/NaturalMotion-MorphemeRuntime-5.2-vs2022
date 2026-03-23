------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2012 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

-- Help Text
local kCalculateLeftKneeHelpText = "Automatically calculate the Left Knee Axis based on the normal to the plane of the leg bones (select a suitable frame of animation in the animation browser)."
local kCalculateRightKneeHelpText = "Automatically calculate the Right Knee Axis based on the normal to the plane of the leg bones (select a suitable frame of animation in the animation browser)."

------------------------------------------------------------------------------------------------------------------------
-- lookup tables

  
local kJointAttributes = { "HipsName", "LeftAnkleName", "LeftBallName", "RightAnkleName", "RightBallName" }
local kBallJointAttributes = { "LeftBallName", "RightBallName" }

local kBallToAnkleAttributes = {
  ["LeftBallName"] = "LeftAnkleName",
  ["RightBallName"] = "RightAnkleName"
}

------------------------------------------------------------------------------------------------------------------------
-- returns if a table contains an item
local contains = function(table, element)
  for _, value in pairs(table) do
    if value == element then
      return true
    end
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
-- returns a the sgTransform that represents a joint of a given name
local findJointByName = function(jointName, set)
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local sceneRoot = anim.getRigSceneRoot(scene, set)
  
  if sceneRoot then
    local it = nmx.NodeIterator.new(sceneRoot, nmx.sgTransformNode.ClassTypeId())
    it:reset(sceneRoot, nmx.sgTransformNode.ClassTypeId())
    while it:next() do
      local node = it:node()
      local joint = node:getChildDataNode(nmx.JointNode.ClassTypeId())
      if joint and joint:getTypeId() == nmx.JointNode.ClassTypeId() then
        if joint:getName() == jointName then
          return node
        end
      end
    end
  end
  
  return nil
end

------------------------------------------------------------------------------------------------------------------------
-- Returns a table with all the joint names
local buildJointNameTable = function(set, filterFunction)

  if filterFunction == nil then
    filterFunction = function()
      return true
    end 
  end
  
  local result = { }
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local sceneRoot = anim.getRigSceneRoot(scene, set)
  
  if sceneRoot then
    local it = nmx.NodeIterator.new(sceneRoot, nmx.sgTransformNode.ClassTypeId())
    it:reset(sceneRoot, nmx.sgTransformNode.ClassTypeId())
    while it:next() do
      local node = it:node()
      local joint = node:getChildDataNode(nmx.JointNode.ClassTypeId())
      
      if joint 
        and joint:getTypeId() == nmx.JointNode.ClassTypeId() 
        and filterFunction(joint) then

        table.insert(result, joint:getName())
      end
    end
  end
  return result
end

------------------------------------------------------------------------------------------------------------------------
-- Returns a table with all the joints thyah correspond to a given body part
local buildIKandGroundContactTable = function(set)
  local result = {IKTargets = {},
                  GroundContacts = {},
                  LeftGroundIkTargets = {},
                  RightGroundIkTargets= {},
                  TemplateRoot = nil}
                  
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local rootSceneNode = anim.getRigSceneRoot(scene, set)

  if rootSceneNode == nil then
    app.error("Failed to find rig scene root when trying to automatically assign hips IK joints")
    return
  end

  local bodyMap = nmx.BodyMappingNode.getBodyMappingNode(rootSceneNode)
  local bodyMappingInfo = nmx.BodyMappingInfoNode.getBodyMappingInfoNode(rootSceneNode)

  local parts = bodyMap:findAttributeArray("Parts")
  local parentIndexAttribute = bodyMap:findAttribute("ParentIndex"):asIntArray()

  if parts:isValid() then
    local partsSize = parts:size()
    for i = 1, partsSize do
    
      local attribute = parts:getAttribute(i)
      local parentIndex = nil
      parentIndex = parentIndexAttribute:at(i)
      
      local ikTarget =  bodyMappingInfo:isPartIKTarget(i -1)
      local isGroundContact = bodyMappingInfo:isPartGroundContact(i -1)
      local isLeft = bodyMappingInfo:isPartLeft(i -1)
      local isRight = bodyMappingInfo:isPartRight(i -1)
      
      local isRootOfTemplate = parentIndex == -1
      
      local outputAttribute = attribute:getLastOutput()
      
      if outputAttribute:getNode() then
        local joint = outputAttribute:getNode()
        if ikTarget then 
          table.insert(result.IKTargets, joint)
        end
        
        if isGroundContact then
          table.insert(result.GroundContacts, joint)
        end
        
        if isGroundContact and ikTarget and isLeft then 
          table.insert(result.LeftGroundIkTargets, joint)
        end
        
        if isGroundContact and ikTarget and isRight then 
          table.insert(result.RightGroundIkTargets, joint)
        end
        
        if isRootOfTemplate then 
          result.TemplateRoot = joint
        end
        
      end
    end
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
-- Return a table of all the canddate ball joints for an ankle joint
local findBallJointNames = function(ankleJoint, set)
  local result = { }
  
  if type(ankleJoint) == "string" then
    ankleJoint = findJointByName(ankleJoint, set)
  end
  
  if ankleJoint then
    local child = ankleJoint:getFirstChild()
    while child ~= nil do
      if child:is(nmx.sgTransformNode.ClassTypeId()) then
        local joint = child:getChildDataNode(nmx.JointNode.ClassTypeId())
        if joint and joint:getTypeId() == nmx.JointNode.ClassTypeId() then
          table.insert(result, joint:getName())
        end
      end
      child = child:getNextSibling()
    end
  end
  
  return result
end

------------------------------------------------------------------------------------------------------------------------
-- Return a table with "safe" and "possible" joints for a given ankle
local getBallJointsForWidget = function(objects, ankleAttribute, set)
  local ankleJoint = getCommonAttributeValue(objects, ankleAttribute, set)
  if ankleJoint then
    local ballJoints = findBallJointNames(ankleJoint, set)

    local filterFunction = function(joint)
      return not contains(ballJoints, joint:getName())
    end

    if table.getn(ballJoints) > 0 then
      return
      {  
        { name = "Safe", channels = ballJoints}, 
        { name = "Possible", channels = buildJointNameTable(set, filterFunction)} 
      }
    end
  end
  
  return nil
end

------------------------------------------------------------------------------------------------------------------------
-- Return the first ball joint for an ankle - of no joint can be found it will retuen ""
local findFirstBallJointName = function(ankleJoint)
  if ankleJoint then
    local child = ankleJoint:getFirstChild()
    while child ~= nil do
      if child:is(nmx.sgTransformNode.ClassTypeId()) then
        local joint = child:getChildDataNode(nmx.JointNode.ClassTypeId())
        if joint and joint:getTypeId() == nmx.JointNode.ClassTypeId() then
          return joint:getName()
        end
      end
      child = child:getNextSibling()
    end
  end
  
  return ""
end

------------------------------------------------------------------------------------------------------------------------
-- Automatically set all of the ball joint attributes
local autoSetJoints = function(objects, set)

  undoBlock(function()
    local ikTable = buildIKandGroundContactTable(set)
  
   local leftAnkleName = nil
   local rightAnkleName = nil
   
   if table.getn(ikTable.LeftGroundIkTargets) > 0 then 
     leftAnkleName = ikTable.LeftGroundIkTargets[1]
     setCommonAttributeValue(objects, "LeftAnkleName", leftAnkleName:getName(), set)
   else
     setCommonAttributeValue(objects, "LeftAnkleName", "", set)
   end 
   
   if table.getn(ikTable.RightGroundIkTargets) > 0 then 
     rightAnkleName = ikTable.RightGroundIkTargets[1]
     setCommonAttributeValue(objects, "RightAnkleName", rightAnkleName:getName(), set)
   else
     setCommonAttributeValue(objects, "RightAnkleName", "", set)
   end 
   
   if ikTable.TemplateRoot ~= nil then 
     setCommonAttributeValue(objects, "HipsName", ikTable.TemplateRoot:getName(), set)
   else
     setCommonAttributeValue(objects, "HipsName", "", set)
   end
    
    -- LeftBallName
    if leftAnkleName ~= nil then 
      local firstLeftBallJointName = findFirstBallJointName(leftAnkleName)
      if firstLeftBallJointName ~= nil then 
        setCommonAttributeValue(objects, "LeftBallName", firstLeftBallJointName, set)
      end 
    end
    
    if rightAnkleName ~= nil then 
      -- RightBallName
      local firstRightBallJointName = findFirstBallJointName(rightAnkleName)
      if firstRightBallJointName ~= nil then 
        setCommonAttributeValue(objects, "RightBallName", firstRightBallJointName, set)
      end
    end
    
  end)
end

------------------------------------------------------------------------------------------------------------------------
-- Clear all of the ball joint attributes
local clearJoints = function(objects, set)
  undoBlock(function()
    for _, attribute in kJointAttributes do
      setCommonAttributeValue(objects, attribute, "", set)
    end
  end)
end

------------------------------------------------------------------------------------------------------------------------
-- Validate a particular joint attribute
local validateJoint = function(objects, attribute, set)

  local ankleAttribute = kBallToAnkleAttributes[attribute]
  if ankleAttribute then
    local ankleValues = getAttributeValues(objects, ankleAttribute, set)
    local ballValues = getAttributeValues(objects, attribute, set)
    local size = table.getn(ankleValues)
    for i=1, size do 
      local ballJoint = ballValues[i]
      local ankleJoint = ankleValues[i]
      if ballJoint == "" then
        return false
      elseif ankleJoint == "" then
        return true
      end
      local ballJoints = findBallJointNames(ankleJoint, set)
      return contains(ballJoints, ballJoint)
    end
  end

  local values = getAttributeValues(objects, attribute, set)
  return not contains(values, "")
end

------------------------------------------------------------------------------------------------------------------------
local perAnimSetHipsIKJointsDisplayInfoSection = function(panel, objects, attributes, set)
  attributeEditor.logEnterFunc("perAnimSetHipsIKJointsDisplayInfoSection")
  
  local jointWidgets = { }
  local jointLabels = { }
  
  --------------------------------------------------------------------------------------------------------------------
  local syncInterface = function()
    local ballJoints = { }

    -- Ball Joints (need enabling/disabling)
    local useBallJoints = getCommonAttributeValue(objects, "UseBallJoints", set) == true
    for _, ballAttribute in kBallJointAttributes do
      local ankleAttribute = kBallToAnkleAttributes[ballAttribute]
      local widget = jointWidgets[ballAttribute]
      local label = jointLabels[ballAttribute]
      local valid = (not useBallJoints) or validateJoint(objects, ballAttribute, set)
      widget:enable(useBallJoints)
      label:enable(useBallJoints)
      widget:setError(not valid)
      ballJoints[ballAttribute] = true
      if useBallJoints then
        local joints = getBallJointsForWidget(objects, ankleAttribute, set)
        widget:setChannels(joints)
      end
    end

    -- Other Joints 
    for _, attribute in kJointAttributes do
      if not ballJoints[attribute] then
        local widget = jointWidgets[attribute]
        local valid = validateJoint(objects, attribute, set)
        widget:setError(not valid)
      end
    end
  end

  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(objects)
   
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:addHSpacer(6)
    panel:beginVSizer{flags = "expand", proportion = 1 }
      panel:setBorder(1)

      -- UseBallJoints
      panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 0 }
        panel:setFlexGridColumnExpandable(2)
        local attribute = "UseBallJoints"
        changeContext:addAttributeChangeEvent(attribute)
        attributeEditor.addAttributeLabel(panel, "Fix", objects, attribute)
        attributeEditor.addBoolAttributeCombo{
          panel = panel,
          objects = objects,
          attribute = attribute,
          falseValue = "Ankle Joint",
          trueValue = "Ball Joint",
          set = set,
        }
      panel:endSizer()
      
      attributeEditor.addSeparator(panel)

      -- Remaining Attributes
      panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

        for _, attribute in kJointAttributes do
          changeContext:addAttributeChangeEvent(attribute)
          local name = utils.getDisplayString(getAttributeDisplayName(objects[1], attribute))
          jointLabels[attribute] = attributeEditor.addAttributeLabel(panel, name, objects, attribute)
          jointWidgets[attribute] = attributeEditor.addAttributeWidget(panel, attribute, objects, set)
        end
      panel:endSizer()

      panel:beginHSizer{ flags = "right" }
        panel:addButton{
          name = "AutoButton",
          label = "Auto",
          size = { width = 74, height = -1 },
          onClick = function()
             autoSetJoints(objects, set)
          end
        }

        panel:addButton{
          name = "ClearButton",
          label = "Clear",
          size = { width = 74, height = -1 },
          onClick = function(self)
            clearJoints(objects, set)
          end
        }
      panel:endSizer()

    panel:endSizer()
  panel:endSizer()

  changeContext:setAttributeChangedHandler(syncInterface)
  syncInterface()

  attributeEditor.logExitFunc("perAnimSetHipsIKJointsDisplayInfoSection")
end
  
------------------------------------------------------------------------------------------------------------------------
local perAnimSetHipsIKKneeAxisDisplayInfoSection = function(panel, selection, attributes, set)
  attributeEditor.logEnterFunc("perAnimSetHipsIKKneeAxisDisplayInfoSection")

  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local leftKneeVectorLabel = nil
  local leftKneeVectorWidget = { }
  local leftKneeVectorCalculateButton = nil
  local rightKneeVectorLabel = nil
  local rightKneeVectorWidget = { }
  local rightKneeVectorCalculateButton = nil

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  -- first add the ui for the section
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:addHSpacer(6)
    panel:setBorder(1)

    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      -- Left Knee Axis and Calculate button
      leftKneeVectorLabel = attributeEditor.addAttributeLabel(panel, "Left Knee", selection, "LeftKneeRotationAxisX")
      local leftKneeAttrs = { "LeftKneeRotationAxisX", "LeftKneeRotationAxisY", "LeftKneeRotationAxisZ" }
      leftKneeVectorWidget = attributeEditor.addVectorAttributeWidget(panel, leftKneeAttrs, selection, set)

      panel:addHSpacer(6)

      local calculateButton = panel:addButton{ label = "Calculate" }
      attributeEditor.bindHelpToWidget(calculateButton, kCalculateLeftKneeHelpText)
      calculateButton:enable(not containsReference(selection))

      calculateButton:setOnClick(
        function(self)
          -- For multiselect to work we have to iterate through selected objects
          for i, selectedObject in selection do
            if getType(selectedObject) == "HipsIK" then
              local useBallJoints = getAttribute(selectedObject, "UseBallJoints", set)
              local ankleJointName = getAttribute(selectedObject, "LeftAnkleName", set)
              if useBallJoint ~= nil and useBallJoint then
                local ballJointName = getAttribute(selectedObject, "LeftBallName", set)
                if ballJointName ~= nil then
                  local ballIndex = anim.getRigChannelIndex(ballJointName, set)
                  local ankleIndex = anim.getParentBoneIndex(ballIndex, set)
                  ankleJointName = anim.getRigChannelNames(set)[ankleIndex + 1]
                end
              end
              if ankleJointName ~= nil then
                local axis = calculateHingeAxis(set, ankleJointName)
                setAttribute(selectedObject .. ".LeftKneeRotationAxisX", axis:getX(), set)
                setAttribute(selectedObject .. ".LeftKneeRotationAxisY", axis:getY(), set)
                setAttribute(selectedObject .. ".LeftKneeRotationAxisZ", axis:getZ(), set)
              end
            end
          end
        end
      )

      -- Right Knee Axis and Calculate button
      rightKneeVectorLabel = attributeEditor.addAttributeLabel(panel, "Right Knee", selection, "RightKneeRotationAxisX")
      local rightKneeAttrs = { "RightKneeRotationAxisX", "RightKneeRotationAxisY", "RightKneeRotationAxisZ" }
      rightKneeVectorWidget = attributeEditor.addVectorAttributeWidget(panel, rightKneeAttrs, selection, set)

      panel:addHSpacer(6)
      local calculateButton = panel:addButton { label = "Calculate", }
      attributeEditor.bindHelpToWidget(calculateButton, kCalculateRightKneeHelpText)
      calculateButton:enable(not containsReference(selection))

      calculateButton:setOnClick(
        function(self)
          -- For multiselect to work we have to iterate through selected objects
          for i, selectedObject in selection do
            if getType(selectedObject) == "HipsIK" then
              local useBallJoints = getAttribute(selectedObject, "UseBallJoints", set)
              local ankleJointName = getAttribute(selectedObject, "RightAnkleName", set)
              if useBallJoint ~= nil and useBallJoint then
                local ballJointName = getAttribute(selectedObject, "RightBallName", set)
                if ballJointName ~= nil then
                  local ballIndex = anim.getRigChannelIndex(ballJointName, set)
                  local ankleIndex = anim.getParentBoneIndex(ballIndex, set)
                  ankleJointName = anim.getRigChannelNames(set)[ankleIndex + 1]
                end
              end
              if ankleJointName ~= nil then
                local axis = calculateHingeAxis(set, ankleJointName)
                setAttribute(selectedObject .. ".RightKneeRotationAxisX", axis:getX(), set)
                setAttribute(selectedObject .. ".RightKneeRotationAxisY", axis:getY(), set)
                setAttribute(selectedObject .. ".RightKneeRotationAxisZ", axis:getZ(), set)
              end
            end
          end
        end
      )

    panel:endSizer()
  panel:endSizer()

  attributeEditor.logExitFunc("perAnimSetHipsIKKneeAxisDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a hipsIKKneeAxisDisplayInfoSection.
-- Used by HipsIK.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.HipsIKKneeAxisDisplayInfoSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = "hipsIKKneeAxisDisplayInfoSection"
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetHipsIKKneeAxisDisplayInfoSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a HipsIKJointsDisplayInfoSection.
-- Used by HipsIK.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.HipsIKJointsDisplayInfoSection = function(panel, displayInfo, selection)
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)

  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = "hipsIKJointAxisDisplayInfoSection"
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }

    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetHipsIKJointsDisplayInfoSection,
--      displayFunc = perAnimSetHipsIKKneeAxisDisplayInfoSection2,
      flags = "expand",
      proportion = 1,
    }

  rollPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the HipsIK general section to the attribute editor
------------------------------------------------------------------------------------------------------------------------
attributeEditor.hipsIKGeneralDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.hipsIKGeneralDisplayInfoSection")

  -- add the ui for the section
  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = "hipsIKGeneralDisplayInfoSection" 
  }
  local rollPanel = rollup:getPanel()

  -- add widgets
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:setBorder(1)

    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      -- InputRotationType
      local attribute = "InputRotationType"
      attributeEditor.addAttributeLabel(rollPanel, "Input rotation", selection, "InputRotationType")
      local options = {
        [1] = "Euler Angle",
        [2] = "Quaternion",
      }
      attributeEditor.addIntAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = attribute,
        values = options,
        set = set,
        helpText = getAttributeHelpText(selection[1], attribute)
      }

      -- Target Frame
      local attribute = "LocalReferenceFrame"
      attributeEditor.addAttributeLabel(rollPanel, "Hips Target Frame", selection, "LocalReferenceFrame")
      attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = attribute,
        trueValue = "Local Space",
        falseValue = "Character Space",
        helpText = getAttributeHelpText(selection[1], attribute)
      }

    rollPanel:endSizer()
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.hipsIKGeneralDisplayInfoSection")
end
