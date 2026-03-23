------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

------------------------------------------------------------------------------------------------------------------------
-- Sets the init smooth operator value to be the same as the Control Parameter's default value
------------------------------------------------------------------------------------------------------------------------
local SetDefaultCPValues = function(selection)
  undoBlock(function()
    for _, object in selection do
      local connections = listConnections{ Object = object, ResolveReferences = true }
      local cpNode = connections[1]
      
      if cpNode ~= nil and (string.find(getType(cpNode), "ControlParameter") ~= nil) then
        local defaultX, defaultY, defaultZ = getDefaultValue(cpNode)
        setAttribute(object .. ".InitValueX", defaultX)
        setAttribute(object .. ".InitValueY", defaultY)
        setAttribute(object .. ".InitValueZ", defaultZ)
      end
    end
  end)
end

------------------------------------------------------------------------------------------------------------------------
-- Check if the set button is available
------------------------------------------------------------------------------------------------------------------------
local getButtonEnabling = function(selection, button)
  -- check if the selection contains any referenced objects
  for i, object in ipairs(selection) do
    if isReferenced(object) then
      return false
    end
  end

  local shouldEnable = true
  for _, object in selection do
    local inputPin = string.format("%s.Input", object)
    local connections = listConnections{ Object = object, ResolveReferences = true }
    local cpNode = connections[1]

    if (isConnected(inputPin) == false) then
      return false
    elseif (string.find(getType(cpNode), "ControlParameter") == nil) then
      return false
    end
  end
  return true
end

------------------------------------------------------------------------------------------------------------------------
-- Adds an OperatorSmoothVector3 display info section.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.operatorSmoothVector3DisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.operatorSmoothVector3DisplayInfoSection")

  -- first add the ui for the section
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "operatorSmoothVector3DisplayInfoSection"  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      -- SmoothTime -------------------------------------------------------
      local attribute = "SmoothTime"
      attributeEditor.addAttributeLabel(rollPanel, "Smooth time", selection, attribute)
      attributeEditor.addAttributeWidget(rollPanel, attribute, selection, set)

      -- SmoothVelocity -----------------------------------------------------
      local attribute = "SmoothVelocity"
      attributeEditor.addAttributeLabel(rollPanel, "Smooth velocity", selection, attribute)
      local smoothVelComboBox = attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        flags = "expand",
        objects = selection,
        proportion = 1,
        trueValue = "Yes",
        falseValue = "No",
        attribute = attribute,
        helpText = getAttributeHelpText(selection[1], attribute)
      }
      

      -- Initial values attribute ----------------------------------------------------
      local initialLabel = attributeEditor.addAttributeLabel(rollPanel, "Initial value", selection, "InitValueX")
      local initalValueWidgets = attributeEditor.addVectorAttributeWidget(rollPanel, {"InitValueX", "InitValueY", "InitValueZ"}, selection)

      -- Use Control Parameter default value button ------------------------------------------
      local buttonHelp = "Use the Control Parameter default value as initial value."
      rollPanel:addHSpacer(0)
         
      local setInit = rollPanel:addButton{
        flags           = "expand",
        proportion      = 0,
        label           = "Copy Control Parameter Value",
        onClick         = function()
          SetDefaultCPValues(selection)
        end,
      }
      attributeEditor.bindHelpToWidget(setInit, buttonHelp)

      -- Use initial value attribute ----------------------------------------------------
      local attribute = "UseInitValueOnInit"
      attributeEditor.addAttributeLabel(rollPanel, "Use Initial Value on node init", selection, attribute)
      local useInitValComboBox = attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        flags = "expand",
        objects = selection,
        proportion = 1,
        trueValue = "Yes",
        falseValue = "No",
        attribute = attribute,
        helpText = getAttributeHelpText(selection[1], attribute)
      }
      
    rollPanel:endSizer()
  rollPanel:endSizer()

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  -- Ensure update editor
  attributeEditor.onFlowEdgeCreateDestroy = refreshUI

  ----------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.dataTypeSmoothVector3DisplayInfoSection")
end

