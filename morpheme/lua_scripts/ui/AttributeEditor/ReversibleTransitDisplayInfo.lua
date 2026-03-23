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
-- Add's a reversibleTransitDisplayInfoSection.
-- Used by Transit, TransitAtEvent and TransitMatchEvents
------------------------------------------------------------------------------------------------------------------------
attributeEditor.reversibleTransitDisplayInfoSection = function(rollContainer, displayInfo, selection)

  if hasTransitionCategory(selection, "euphoria") then
    return
  end
  
  attributeEditor.logEnterFunc("attributeEditor.reversibleTransitDisplayInfoSection")

  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)
  
  local reversibleTransitCheckBox = nil
  local reverseControlParameterComboBox = nil

  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "reversibleTransitDisplayInfoSection" }
  local rollPanel = rollup:getPanel()
  rollup:expand(false)

  -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      local reversibleTransitAttrPaths = { }
      local reverseControlParameterAttrPaths = { }

      for i, object in ipairs(selection) do
        table.insert(reversibleTransitAttrPaths, string.format("%s.ReversibleTransit", object))
        table.insert(reverseControlParameterAttrPaths, string.format("%s.ReverseControlParameter", object))
      end

      reversibleTransitHelpText = "Allow the transition to be reversible."

      rollPanel:addStaticText{
        text = "Enable",
        onMouseEnter = function()
          attributeEditor.setHelpText(reversibleTransitHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      reversibleTransitCheckBox = rollPanel:addAttributeWidget{
        attributes = reversibleTransitAttrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(reversibleTransitHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      reverseControlParameterHelpText = "Choose the control parameter that will be used to control transition reversing."

      rollPanel:addStaticText{
        text = "Control parameter",
        onMouseEnter = function()
          attributeEditor.setHelpText(reverseControlParameterHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      reverseControlParameterComboBox = rollPanel:addAttributeWidget{
        attributes = reverseControlParameterAttrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(reverseControlParameterHelpText)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- enable ReverseControlParameter based on the values of ReversibleTransit
  ----------------------------------------------------------------------------------------------------------------------
  local syncUIWithAttributes = function()
    attributeEditor.logEnterFunc("syncUIWithAttributes")

    local usingReversibleTransit = getCommonAttributeValue(selection, "ReversibleTransit")

    if usingReversibleTransit == nil then
      --not all objects have the same value
    else
      -- all objects have the same value
      reverseControlParameterComboBox:enable(usingReversibleTransit and not hasReference)
    end

    attributeEditor.logExitFunc("syncUIWithAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen via script,
  -- or through undo redo, are reflected in the custom ui.
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("ReversibleTransit")

  ------------------------------------------------------------------------------------------------------------------------
  -- this function is called whenever ReversibleTransit is changed via script, or through undo redo.
  ------------------------------------------------------------------------------------------------------------------------
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")

      if enableContextEvents then
        enableUISetAttribute = false
        syncUIWithAttributes()
        enableUISetAttribute = true
      end

      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  -- set the initial state of the ui
  syncUIWithAttributes()

  ------------------------------------------------------------------------------------------------------------------------

  attributeEditor.logExitFunc("attributeEditor.reversibleTransitDisplayInfoSection")
end
