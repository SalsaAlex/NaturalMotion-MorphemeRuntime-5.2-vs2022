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
-- Finds out if the transit is from another transit
------------------------------------------------------------------------------------------------------------------------
local transitFromTransit = function(object)
  local source = listConnections{
    Object = object[1],
    Upstream = true,
    Downstream = false,
    ResolveReferences = true
  }
  if table.getn(source) ~= 1 then
    return false
  end
  return (getType(source[1]) == "Transit")
end
  
------------------------------------------------------------------------------------------------------------------------
-- Adds a transitDurationInTimeSection.
-- Used by Transit.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.transitDurationInTimeSection = function(rollContainer, displayInfo, selection)
  if hasTransitionCategory(selection, "euphoria") then
    return
  end

  attributeEditor.standardDisplayInfoSection(rollContainer, displayInfo, selection)
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a transitDisplayInfoSection.
-- Used by Transit.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.transitDisplayInfoSection = function(rollContainer, displayInfo, selection)
  if hasTransitionCategory(selection, "euphoria") then
    return
  end
  
  attributeEditor.logEnterFunc("attributeEditor.transitDisplayInfoSection")
  -- check if the selection contains any referenced objects
  local hasReference = containsReference(selection)

  local destinationComboBox = nil
  local fractionWidget = nil
  local eventIndexComboBox = nil
  local eventIndexWidget = nil
  local eventFractionComboBox = nil
  local eventFractionWidget = nil
    
  attributeEditor.log("rollContainter:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "transitDisplayInfoSection" }
  local rollPanel = rollup:getPanel()

  -- first add the ui for the section
  attributeEditor.log("rollPanel:beginHSizer")
  rollPanel:beginHSizer{ flags = "expand", proportion = 1 }
    rollPanel:addHSpacer(6)
    rollPanel:setBorder(1)

    attributeEditor.log("rollPanel:beginFlexGridSizer")
    rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rollPanel:setFlexGridColumnExpandable(2)

      local destinationStartFractionAttrPaths = { }
      local destinationStartSyncEventIndexAttrPaths = { }
      local destinationStartSyncEventFractionAttrPaths = { }

      for i, object in ipairs(selection) do
        table.insert(destinationStartFractionAttrPaths, string.format("%s.DestinationStartFraction", object))
        table.insert(destinationStartSyncEventIndexAttrPaths, string.format("%s.DestinationStartSyncEventIndex", object))
        table.insert(destinationStartSyncEventFractionAttrPaths, string.format("%s.DestinationStartSyncEventFraction", object))
      end

local destinationComboHelp =
[[
Choose to use a specific fraction if you know a precise point to transition to in the destination.
Choose to use sync events if the position in the destination is relative to a sync event.
]]

local destinationTransitFromTransitComboHelp =
[[
Choose to use a specific fraction if you know a precise point to transition to in the destination.
]]

local fractionHelp =
[[
The start position in the destination as a fraction of its duration.
]]

      local falseValueTransit = "Use Sync Events"
      local helpTransit = destinationComboHelp
      if (transitFromTransit(selection) == true) then
        falseValueTransit = "Animation default start"
	helpTransit = destinationTransitFromTransitComboHelp
      end
	  
      rollPanel:addStaticText{
        text = "Destination",
        onMouseEnter = function()
          attributeEditor.setHelpText(helpTransit)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      destinationComboBox = attributeEditor.addBoolAttributeCombo{
        panel = rollPanel,
        objects = selection,
        attribute = "UseDestinationStartFraction",
        trueValue = "Specify Fraction",
        falseValue = falseValueTransit,
        helpText = helpTransit
      }

      rollPanel:addStaticText{ }

      fractionWidget = rollPanel:addAttributeWidget{
        attributes = destinationStartFractionAttrPaths,
        flags = "expand",
        proportion = 1,
        onMouseEnter = function(self)
          attributeEditor.setHelpText(fractionHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      if (transitFromTransit(selection) == false) then

        local eventIndexComboHelp =
[[
Choose to use a specific index if you know a precise point that you want to transition to in the destination.
Choose to match the source index when the source and destination have coordinated events and you do not need to target a particular event.
]]

        local eventIndexHelp =
[[
This controls the playback position that the destination will be initialised at on transition start.
It is the index of a synchronisation event within the destination.
]]
	  
        rollPanel:addStaticText{
          text = "Event index",
          onMouseEnter = function()
            attributeEditor.setHelpText(eventIndexComboHelp)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

        eventIndexComboBox = attributeEditor.addBoolAttributeCombo{
          panel = rollPanel,
          objects = selection,
          attribute = "UseDestinationStartSyncEventIndex",
          trueValue = "Specify",
          falseValue = "Match source",
          helpText = eventIndexComboHelp
        }

        rollPanel:addStaticText{ }

        eventIndexWidget = rollPanel:addAttributeWidget{
          attributes = destinationStartSyncEventIndexAttrPaths,
          flags = "expand",
          proportion = 1,
          onMouseEnter = function(self)
            attributeEditor.setHelpText(eventIndexComboHelp)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

        local eventFractionComboHelp =
[[
Choose to use a specific fraction if you know a precise point following the event that you want to transition to in the destination.
Choose to match the source fraction when the source and destination have coordinated events and you do not need to target a precise point.
]]

        local eventFractionHelp =
[[
This works in conjunction with destination start event index to allow the accurate specification of the start position in the destination.
]]

        local eventFractionHelpTransit = eventFractionComboHelp
        if (transitFromTransit(selection) == true) then
          eventFractionHelpTransit = eventFractionComboHelp
        end
	  
        rollPanel:addStaticText{
          text = "Fraction",
          onMouseEnter = function()
            attributeEditor.setHelpText(eventFractionComboHelp)
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }

        eventFractionComboBox = attributeEditor.addBoolAttributeCombo{
          panel = rollPanel,
          objects = selection,
          attribute = "UseDestinationStartSyncEventFraction",
          trueValue = "Specify",
          falseValue = "Match source",
          helpText = eventFractionComboHelp
        }

        rollPanel:addStaticText{ }

        eventFractionWidget = rollPanel:addAttributeWidget{
          attributes = destinationStartSyncEventFractionAttrPaths,
          flags = "expand",
          proportion = 1,
          onMouseEnter = function(self)
            attributeEditor.setHelpText(eventFractionComboHelp )
          end,
          onMouseLeave = function()
            attributeEditor.clearHelpText()
          end
        }
      end -- if (transitFromTransit(selection) == false) then

      attributeEditor.log("rollPanel:endSizer")
    rollPanel:endSizer()

    attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()

  if hasReference then
    destinationComboBox:enable(false)
    eventIndexComboBox:enable(false)
    eventFractionComboBox:enable(false)
  end

  -- these prevent callbacks firing off other callbacks causing infinite loops
  local enableContextEvents = true
  local enableUISetAttribute = true

  ----------------------------------------------------------------------------------------------------------------------
  -- enable fractionWidget when UseDestinationStartFraction is set.
  -- enable eventIndexWidget when UseDestinationStartSyncEventIndex is set.
  -- enable eventFractionWidget when UseDestinationStartSyncEventFraction is set.
  ----------------------------------------------------------------------------------------------------------------------
  local syncUIWithAttributes = function()
    attributeEditor.logEnterFunc("syncUIWithAttributes")

    local useDestinationStartFraction = getCommonAttributeValue(selection, "UseDestinationStartFraction")

    destinationComboBox:enable(not hasReference)

    if useDestinationStartFraction ~= nil then
      if useDestinationStartFraction then
        attributeEditor.log("all objects have UseDestinationStartFraction set, enabling the fractionWidget")
        fractionWidget:enable(not hasReference)
      else
        attributeEditor.log("all objects have UseDestinationStartFraction unset, disabling the fractionWidget")
        fractionWidget:enable(false)
      end
    else
      attributeEditor.log("not all objects have the same UseDestinationStartFraction value, disabling the fractionWidget")
      fractionWidget:enable(false)
      -- some objects have useDestinationStartFraction set
      -- setting useDestinationStartFraction will disabled the event based controls
      useDestinationStartFraction = true
    end
    
    if (transitFromTransit(selection) == false) then
      local useDestinationStartSyncEventIndex = getCommonAttributeValue(selection, "UseDestinationStartSyncEventIndex")

      eventIndexComboBox:enable(not hasReference and not useDestinationStartFraction)

      if useDestinationStartSyncEventIndex then
        attributeEditor.log("all objects have UseDestinationStartSyncEventIndex set, enabling the eventIndexWidget")
        eventIndexWidget:enable(not hasReference and not useDestinationStartFraction)
      else
        attributeEditor.log("not all objects have UseDestinationStartSyncEventIndex set, disabling the eventIndexWidget")
        eventIndexWidget:enable(false)
      end

      local useDestinationStartSyncEventFraction = getCommonAttributeValue(selection, "UseDestinationStartSyncEventFraction")

      eventFractionComboBox:enable(not hasReference and not useDestinationStartFraction)

      if useDestinationStartSyncEventFraction then
        attributeEditor.log("all objects have UseDestinationStartSyncEventFraction set, enabling the eventFractionWidget")
        eventFractionWidget:enable(not hasReference and not useDestinationStartFraction)
      else
        attributeEditor.log("not all objects have UseDestinationStartSyncEventFraction set, disabling the eventFractionWidget")
        eventFractionWidget:enable(false)
      end
    end -- if (transitFromTransit(selection) == false) then

    attributeEditor.logExitFunc("syncUIWithAttributes")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- this data change context ensures the ui reflects any changes that happen via script,
  -- or through undo redo, are reflected in the custom ui.
  local changeContext = attributeEditor.createChangeContext()

  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("UseDestinationStartSyncEventFraction")
  changeContext:addAttributeChangeEvent("UseDestinationStartSyncEventIndex")
  changeContext:addAttributeChangeEvent("UseDestinationStartFraction")

  ------------------------------------------------------------------------------------------------------------------------
  -- this function is called whenever UseDestinationStartSyncEventFraction, UseDestinationStartSyncEventIndex or
  -- UseDestinationStartFraction is is changed via script, or through undo redo.
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

  attributeEditor.logExitFunc("attributeEditor.transitDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Add a section for blend properties attributes on transitions
------------------------------------------------------------------------------------------------------------------------
attributeEditor.blendPropertiesDisplayInfoSection = function(rollContainer, displayInfo, selection)  
  if hasTransitionCategory(selection, "euphoria") then
    return
  end
  
  attributeEditor.logEnterFunc("attributeEditor.blendPropertiesDisplayInfoSection")
  local needsPhysicsBlendProperties = function(object)
    local source = listConnections{
      Object = object,
      Upstream = true,
      Downstream = false,
      ResolveReferences = true
    }
    local dest = listConnections{
      Object = object,
      Upstream = false,
      Downstream = true,
      ResolveReferences = true
    }
    
    if table.getn(source) ~= 1 then
      return false
    end
    
    for _,v in dest do
      local sourceType, sourceGroupType = getType(source[1])
      local destType, destGroupType = getType(dest[1])
      
      if sourceType == "PhysicsStateMachine" or destType == "PhysicsStateMachine" or sourceType == "PhysicsBlendTree" or destType == "PhysicsBlendTree" then
        return true
      end
      
      if sourceGroupType == "Transition" then
        local subSource = listConnections{
          Object = source[1],
          Upstream = true,
          Downstream = false,
          ResolveReferences = true
        }
        local subDest = listConnections{
          Object = source[1],
          Upstream = false,
          Downstream = true,
          ResolveReferences = true
        }
    
        if table.getn(subSource) ~= 1 then
          return false
        end
        
        for _,w in subDest do
          local subSourceType, subSourceGroupType = getType(subSource[1])
          local subDestType, subDestGroupType = getType(subDest[1])
          
          if subSourceType == "PhysicsStateMachine" or subDestType == "PhysicsStateMachine" or subSourceType == "PhysicsBlendTree" or subDestType == "PhysicsBlendTree" then
            return true
          end
        end
      end
    end
    
    return false
  end
  
  local selectionNeedsPhysicsBlendProperties = function(selection)
    local allPhysics = true
    for _,v in pairs(selection) do
      if not needsPhysicsBlendProperties(v) then
        allPhysics = false
        break
      end
    end
    return allPhysics
  end
  
  attributeEditor.log("rollContainer:addRollup")
  local rollup = rollContainer:addRollup{ label = displayInfo.title, flags = "mainSection", name = "blendPropertiesDisplayInfoSection" }
  local rollPanel = rollup:getPanel()
      
  local addAttributeSection = function(attr)
    attributeEditor.log("adding control for attribute \"%s\"", attr)
    local name = utils.getDisplayString(getAttributeDisplayName(selection[1], attr))
    attributeEditor.addAttributeLabel(rollPanel, name, selection, attr)
    attributeEditor.addAttributeWidget(rollPanel, attr, selection)
  end
  
  rollPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
    rollPanel:setFlexGridColumnExpandable(2)
    
    addAttributeSection("UseDeadReckoningWhenDeadBlending")
    
    if selectionNeedsPhysicsBlendProperties(selection) then
      addAttributeSection("BlendToDestinationPhysicsBones")    
    end

  attributeEditor.log("rollPanel:endSizer")
  rollPanel:endSizer()
  
  attributeEditor.logExitFunc("attributeEditor.blendPropertiesDisplayInfoSection")
end
