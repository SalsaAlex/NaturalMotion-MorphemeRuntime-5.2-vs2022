-----------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"


local perAnimSetJointName = function(panel, selection, attributes, set)
  panel:setBorder(1)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    attributeEditor.addAttributeWidget(panel, "JointName", selection, set)
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds an ExtractJointInfo display info section.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.extractJointInfoDisplayInfoSection = function(rollContainer, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.extractJointInfoDisplayInfoSection")

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  -- first add the ui for the section
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "extractJointInfoDisplayInfoSection"  }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

        local outputSpaceComboBox = nil
        local angleTypeComboBox = nil
        local unitComboBox  = nil
      
    -- ---------------------
    -- Output Space. 
        local outputSpaceHelp = getAttributeHelpText(selection[1], "OutputSpace")
        local outputSpaceLabel = rollPanel:addStaticText{
          text = "Output space",
          onMouseEnter = function()
            attributeEditor.setHelpText(outputSpaceHelp)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

        outputSpaceComboBox = attributeEditor.addBoolAttributeCombo{
          panel = rollPanel,
          objects = selection,
          attribute = "OutputSpace",
          proportion = 1,
          trueValue = "Object",
          falseValue = "Local",
          helpText = outputSpaceHelp
        }

    -- ---------------------
    -- Joint Name.
        local jointNameHelp = getAttributeHelpText(selection[1], "JointName")
        local outputSpaceLabel = rollPanel:addStaticText{
          text = "Joint name",
          onMouseEnter = function()
            attributeEditor.setHelpText(jointNameHelp)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

        local attrs = checkSetOrderAndBuildAttributeList(selection, {"JointName"})
        rollPanel:addAnimationSetWidget{
          attributes = attrs,
          displayFunc = perAnimSetJointName,
          flags = "expand",
          proportion = 1,
          helpText = jointNameHelp
        }

    -- ---------------------
    -- Angle Type.
        local angleTypeHelp = getAttributeHelpText(selection[1], "AngleType")
        local angleTypeLabel = rollPanel:addStaticText{
          text = "Angle type",
          onMouseEnter = function()
            attributeEditor.setHelpText(angleTypeHelp)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

        local angleTypeItems = { [0] = "Total", [1] = "EulerX", [2] = "EulerY", [3] = "EulerZ" }
        angleTypeComboBox = attributeEditor.addIntAttributeCombo{
          panel = rollPanel,
          flags = "expand",
          objects = selection,
          values = angleTypeItems,
          helpText = angleTypeHelp,
          attribute = "AngleType",
          order = { "Total", "EulerX", "EulerY", "EulerZ" }
        }    
        
    -- ---------------------
    -- Measure unit.
        local unitHelp = getAttributeHelpText(selection[1], "MeasureUnit")
        local unitLabel = rollPanel:addStaticText{
          text = "Measure unit",
          onMouseEnter = function()
            attributeEditor.setHelpText(unitHelp)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

        unitComboBox = attributeEditor.addBoolAttributeCombo{
          panel = rollPanel,
          objects = selection,
          attribute = "MeasureUnit",
          trueValue = "Radian",
          falseValue = "Degree",
          helpText = unitHelp
        }
            
     attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  if hasReference then
    outputSpaceComboBox:enable(false)
  end

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures that any changes that happen to selected
  -- attributes through script or undo redo are reflected in the custom ui.
  attributeEditor.log("creating data change context")
  local changeContext = attributeEditor.createChangeContext()

  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("OutputSpace")
  changeContext:addAttributeChangeEvent("AngleType")
  changeContext:addAttributeChangeEvent("MeasureUnits")

  ----------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.extractJointInfoDisplayInfoSection")
end

