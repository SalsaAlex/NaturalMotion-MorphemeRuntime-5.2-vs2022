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
-- Add's a display info section containing acceleration limits attributes.
-- Use by SoftKeyFrame.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.accelerationLimitsDisplayInfoSection = function(rollContainer, displayInfo, selection)

  attributeEditor.logEnterFunc("attributeEditor.accelerationLimitsDisplayInfoSection")

  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "accelerationLimitsDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      local maxAccelerationAttrPaths = { }
      local useMaxAccelerationAttrPaths = { }
      local maxAngularAccelerationAttrPaths = { }
      local useMaxAngularAccelerationAttrPaths = { }

      for i, object in ipairs(selection) do
        table.insert(maxAccelerationAttrPaths, string.format("%s.MaxAcceleration", object))
        table.insert(useMaxAccelerationAttrPaths, string.format("%s.UseMaxAcceleration", object))
        table.insert(maxAngularAccelerationAttrPaths, string.format("%s.MaxAngularAcceleration", object))
        table.insert(useMaxAngularAccelerationAttrPaths, string.format("%s.UseMaxAngularAcceleration", object))
      end

      local useMaxAccelerationHelp = getAttributeHelpText(selection[1], "UseMaxAcceleration")

      rollPanel:addStaticText{
        text = "Linear",
        onMouseEnter = function()
          attributeEditor.setHelpText(useMaxAccelerationHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      rollPanel:addAttributeWidget{
        attributes = useMaxAccelerationAttrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(useMaxAccelerationHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local maxAccelerationHelp = getAttributeHelpText(selection[1], "MaxAcceleration")

      rollPanel:addStaticText{ }

      local maxAccelerationWidget = rollPanel:addAttributeWidget{
        attributes = maxAccelerationAttrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(maxAccelerationHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local useMaxAngularAccelerationHelp = getAttributeHelpText(selection[1], "UseMaxAngularAcceleration")

      rollPanel:addStaticText{
        text = "Angular",
        onMouseEnter = function()
          attributeEditor.setHelpText(useMaxAngularAccelerationHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      rollPanel:addAttributeWidget{
        attributes = useMaxAngularAccelerationAttrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(useMaxAngularAccelerationHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      local maxAngularAccelerationHelp = getAttributeHelpText(selection[1], "MaxAngularAcceleration")

      rollPanel:addStaticText{ }

      local maxAngularAccelerationWidget = rollPanel:addAttributeWidget{
        attributes = maxAngularAccelerationAttrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(maxAngularAccelerationHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  local hasReference = containsReference(selection)

  ----------------------------------------------------------------------------------------------------------------------
  -- update the interface to reflect the current attributes.
  ----------------------------------------------------------------------------------------------------------------------
  local syncInterface = function()
    attributeEditor.logEnterFunc("syncInterface")

    local useMaxAcceleration = getCommonAttributeValue(selection, "UseMaxAcceleration")

    if useMaxAcceleration ~= nil then
      if useMaxAcceleration then
        maxAccelerationWidget:enable(not hasReference)
      else
        maxAccelerationWidget:enable(false)
      end
    else
      maxAccelerationWidget:enable(false)
    end

    local useMaxAngularAcceleration = getCommonAttributeValue(selection, "UseMaxAngularAcceleration")

    if useMaxAngularAcceleration ~= nil then
      if useMaxAngularAcceleration then
        maxAngularAccelerationWidget:enable(not hasReference)
      else
        maxAngularAccelerationWidget:enable(false)
      end
    else
      maxAngularAccelerationWidget:enable(false)
    end

    attributeEditor.logExitFunc("syncInterface")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen via script, or through undo redo.
  ----------------------------------------------------------------------------------------------------------------------

  local changeContext = attributeEditor.createChangeContext()

  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("UseMaxAcceleration")
  changeContext:addAttributeChangeEvent("UseMaxAngularAcceleration")

  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")
      syncInterface()
      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )

  syncInterface()
  attributeEditor.logExitFunc("attributeEditor.accelerationLimitsDisplayInfoSection")
end

