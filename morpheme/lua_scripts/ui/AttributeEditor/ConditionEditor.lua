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
-- adds a single condition to the condition editor panel
------------------------------------------------------------------------------------------------------------------------
local addConditionToPanel = function(panel, conditions)
  attributeEditor.logEnterFunc("addConditionToPanel")

  -- clear the used attribute array
  attributeEditor.usedAttributes = { }

  local firstCondition = conditions[1]
  local conditionCount = table.getn(conditions)
  local conditionPath, conditionName = splitNodePath(firstCondition)
  local rollup = panel:addRollup{ label = "", flags = "expand", name = conditionName }

  local rollupPanel = rollup:getPanel()
  rollupPanel:beginVSizer{ flags = "expand", proportion = 0 }
    local attributes = listAttributes(firstCondition)

    local conditionType, manifestType = getType(firstCondition)
    local displayInfo = attributeEditor.displayInfo[conditionType]

    if type(displayInfo) == "table" then
      attributeEditor.log("adding display info sections")
      for i, section in ipairs(displayInfo) do

        -- add the current sections used attributes to the global attributes table
        if type(section.usedAttributes) == "table" then
          for j, attr in ipairs(section.usedAttributes) do
            attributeEditor.log("marking attribute \"%s\" as used", attr)
            attributeEditor.usedAttributes[attr] = true
          end
        end

        if type(section.title) == "string" and string.len(section.title) > 0 then
          -- call the display function for the current section
          attributeEditor.log("attempting to call displayInfo.displayFunc")
          safefunc(section.displayFunc, rollupPanel, section, conditions)
        end
      end
    end

    for i, attr in ipairs(attributes) do
      if not attributeEditor.usedAttributes[attr] then
        rollupPanel:beginHSizer{ flags = "expand", proportion = 0 }

          local attrName = utils.getDisplayString(getAttributeDisplayName(firstCondition, attr))
          local attrHelp = getAttributeHelpText(firstCondition, attr)

          rollupPanel:addStaticText{
            text = attrName,
            onMouseEnter = function()
              attributeEditor.setHelpText(attrHelp)
            end,
            onMouseLeave = function()
              attributeEditor.clearHelpText()
            end
          }

          local attrPaths = { }
          for i, condition in ipairs(conditions) do
            table.insert(attrPaths, string.format("%s.%s", condition, attr))
          end

          local widget = rollupPanel:addAttributeWidget{
            attributes = attrPaths,
            flags = "expand",
            proportion = 1,
            onMouseEnter = function()
              attributeEditor.setHelpText(attrHelp)
            end,
            onMouseLeave = function()
              attributeEditor.clearHelpText()
            end
          }
        rollupPanel:endSizer()
      end
    end

  rollupPanel:endSizer()

  attributeEditor.log("adding delete button and condition name to the header panel")
  local headerPanel = rollup:getHeader()
  headerPanel:beginHSizer()

    local button = headerPanel:addButton{
      image = app.loadImage("deleteCross.png"),
      onClick = function()
        undoBlock(function()
          for i, condition in ipairs(conditions) do
            delete(condition)
          end
        end)
      end
    }

    headerPanel:addHSpacer(4)
    headerPanel:beginVSizer()
      headerPanel:addVSpacer(4)

      local conditionLabel = conditionName
      if conditionCount > 1 then
        conditionLabel = conditionType .. " (" .. conditionCount .. ")"
      end

      local textbox = headerPanel:addStaticText{
        text = conditionLabel,
        onMouseEnter = function()
          attributeEditor.setHelpText(getHelpText(conditionType))
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }
    headerPanel:endSizer()

    for i, condition in ipairs(conditions) do
      if isReferenced(condition) then
        button:enable(false)
        break
      end
    end

  headerPanel:endSizer()

  attributeEditor.logExitFunc("addConditionToPanel")
end

  ----------------------------------------------------------------------------------------------------------------------
  -- return a table of { condition type, { conditions } } for all the unique condition types in the passed transition
  ----------------------------------------------------------------------------------------------------------------------
local getConditions = function(transition)

  local typeTable = { }

  local uniqueTypes = { }
  local conditions = listChildren(transition)
  for i, condition in ipairs(conditions) do
    local conditionType, manifestType = getType(condition)
    if manifestType == "Condition" then
      local tableEntry = uniqueTypes[conditionType]
      if not tableEntry then
        tableEntry = { type = conditionType, conditions = { } }
        uniqueTypes[conditionType] = tableEntry
        table.insert(typeTable, tableEntry)
      end
      table.insert(tableEntry.conditions, condition)
    end
  end

  return typeTable
end

  ----------------------------------------------------------------------------------------------------------------------
  -- return a table of { condition type, { conditions } } for conditions common across all transitions
  ----------------------------------------------------------------------------------------------------------------------
local getCommonConditions = function(transitions)

  local typeTable = { }

  local transitionCount = table.getn(transitions)
  if transitionCount > 0 then
    local firstTypeTable = getConditions(transitions[1])
    for _, firstTypeEntry in ipairs(firstTypeTable) do
      local foundTypeInAllTransitions = true
      for i = 2, transitionCount do
        local foundTypeInTransition = false
        local checkTypeTable = getConditions(transitions[i])
        for _, checkTypeEntry in ipairs(checkTypeTable) do
          if firstTypeEntry.type == checkTypeEntry.type then
            for _, condition in checkTypeEntry.conditions do
              table.insert(firstTypeEntry.conditions, condition)
            end
            foundTypeInTransition = true
            break
          end
        end
        if not foundTypeInTransition then
          foundTypeInAllTransitions = false
          break
        end
      end
      if foundTypeInAllTransitions then
        table.insert(typeTable, firstTypeEntry)
      end
    end
  end

  return typeTable
end

------------------------------------------------------------------------------------------------------------------------
-- rebuilds the condition editor panel
------------------------------------------------------------------------------------------------------------------------
local rebuildConditionEditor = function(panel, selection)
  attributeEditor.logEnterFunc("rebuildConditionEditor")

  panel:beginVSizer{ flags = "expand", proportion = 0 }

    -- create add new condition combo
    local button = panel:addButton{
      label = "Create condition",
      flags = "left",
      onClick = function(self)
        showAddConditionDialog(selection)
      end
    }

    for i, object in ipairs(selection) do
      if isReferenced(object) then
        button:enable(false)
        break
      end
    end

    -- list current condition attributes
    local selectionCount = table.getn(selection)

    -- single selection
    if selectionCount == 1 then
      local conditions = listChildren(selection[1])
      for i, condition in ipairs(conditions) do
        local type, manifestType = getType(condition)
        if manifestType == "Condition" then
          addConditionToPanel(panel, { condition })
        end
      end

    -- multiple selection
    elseif selectionCount > 1 then
      local commonConditions = getCommonConditions(selection)
      for i, v in ipairs(commonConditions) do
        addConditionToPanel(panel, v.conditions)
      end
    end

  panel:endSizer()

  attributeEditor.logExitFunc("rebuildConditionEditor")
end

------------------------------------------------------------------------------------------------------------------------
-- adds the conditions editor panel to a roll container panel
------------------------------------------------------------------------------------------------------------------------
attributeEditor.addConditionEditor = function(panel, selection)
  attributeEditor.logEnterFunc("attributeEditor.addConditionEditor")

  local rollup = panel:addRollup{ name = "ConditionPanel", label = "Conditions", flags = "mainSection", name = "ConditionPanel" }
  local rollupPanel = rollup:getPanel()

  -- Ensure any condition changes update editor
  attributeEditor.onConditionCreateDestroy = function()
    local editor = attributeEditor.editorWindow
    editor:freeze()

    local currState = { }
    rollupPanel:storeState(currState, true)
    rollupPanel:clear()

    rebuildConditionEditor(rollupPanel, selection)
    rollupPanel:restoreState(currState, true, false)

    editor:rebuild()
  end

  rebuildConditionEditor(rollupPanel, selection)

  attributeEditor.logExitFunc("attributeEditor.addConditionEditor")
end

