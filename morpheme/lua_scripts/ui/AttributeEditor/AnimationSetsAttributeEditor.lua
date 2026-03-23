------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/MorphemeUnitAPI.lua"
require "ui/AttributeEditor/SkinsAttributeEditorSection.lua"

local getSelectionCount = function() return 1 end
local displayedCompressionFormat = nil
local selectedSkin = nil
local animationSetsTreeControl = nil
local animationSetsRollupHeading = nil
local addSetButton = nil
local removeSetButton = nil
local moveUpSetButton = nil
local moveDownSetButton = nil
local rollupHeadingLabel = nil
local mainScrollPanel = nil
local helpPanel = nil
local helpText = nil
local statusIcons = { app.loadImage("TickIcon.png"), app.loadImage("WarningIcon.png"), app.loadImage("ErrorIcon.png")}
local statusLookup = { Ok = 0, Warning = 0, Error = 3}

if not animationSetsAttributeEditor.registered then
  animationSetsAttributeEditor = { }

  if not mcn.inCommandLineMode() then
    --------------------------------------------------------------------------------------------------------------------
    -- Having a wrapper handler function ensures this script can be rerun
    -- without registering duplicate event handlers.
    --------------------------------------------------------------------------------------------------------------------
    local animationSetModifiedEventHandler = function(set)
      safefunc(animationSetsAttributeEditor.onAnimationSetModified, set)
    end
    local rigChangeHandler = function()
      safefunc(animationSetsAttributeEditor.onRigModified)
    end
    local physicsRigChangeHandler = function()
      safefunc(animationSetsAttributeEditor.onPhysicsRigModified)
    end

    registerEventHandler("mcAnimationSetModified", animationSetModifiedEventHandler)
    registerEventHandler("mcRigChange", rigChangeHandler)
    registerEventHandler("mcPhysicsRigChange", physicsRigChangeHandler)
  end
end

------------------------------------------------------------------------------------------------------------------------
animationSetsAttributeEditor.setHelpText = function(string)
  if helpText then
    if string then
      helpText:setValue(string)
    else
      helpText:setValue("")
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
animationSetsAttributeEditor.clearHelpText = function()
  if helpText then
    helpText:setValue("")
  end
end

------------------------------------------------------------------------------------------------------------------------
animationSetsAttributeEditor.bindHelpToWidget = function(widget, helpText)
 widget:setOnMouseEnter(
    function(self)
      animationSetsAttributeEditor.setHelpText(helpText)
    end
  )
  widget:setOnMouseLeave(
    function(self)
      animationSetsAttributeEditor.clearHelpText()
    end
  )
end

------------------------------------------------------------------------------------------------------------------------
-- canEditAnimationSet
------------------------------------------------------------------------------------------------------------------------
local canEditAnimationSet = function(selectedSet)
  if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
    local kDisplayEditAnimationSetWarning = "DisplayEditAnimationSetWarning"
    if not preferences.exists(kDisplayEditAnimationSetWarning) then
      preferences.set{
        name = kDisplayEditAnimationSetWarning,
        location = "RoamingUser",
        type = "boolean",
        value = true,
      }
    end

    local displayWarning = preferences.get(kDisplayEditAnimationSetWarning)
    if displayWarning then
      local selectedSetIsReferenced = anim.isAnimSetReferenced(selectedSet)
      if selectedSetIsReferenced then
        local message = "A referenced network is using the animation set '" .. selectedSet .. "'.\n"
        message = message .. "Changing the animation set can change the behaviour of the referenced network.\n"
        message = message .. "Are you sure you wish to continue?"
        local result, ignore = ui.showMessageBox(message, "yesno;ignore")
        if ignore then
          preferences.set{ name = kDisplayEditAnimationSetWarning, value = false }
        end
        if result == "no" then
          return false
        end
      end
    end

    return true
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
-- canDeleteAnimationSet
------------------------------------------------------------------------------------------------------------------------
local canDeleteAnimationSet = function(selectedSet)
  if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
    local kDisplayDeleteAnimationSetWarning = "DisplayDeleteAnimationSetWarning"
    if not preferences.exists(kDisplayDeleteAnimationSetWarning) then
      preferences.set{
        name = kDisplayDeleteAnimationSetWarning,
        location = "RoamingUser",
        type = "boolean",
        value = true,
      }
    end

    local displayWarning = preferences.get(kDisplayDeleteAnimationSetWarning)
    if displayWarning then
      local message = "Deleting an animation set can lose a lot of data.\n"
      message = message .. "Are you sure you wish to continue?"
      local result, ignore = ui.showMessageBox(message, "yesno;ignore")
      if ignore then
        preferences.set{ name = kDisplayDeleteAnimationSetWarning, value = false, }
      end
      if result ~= "yes" then
        return false
      end
    end

    return true
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
-- pop up the add animation set dialog box
------------------------------------------------------------------------------------------------------------------------
local poseJointMirroringDialog = function(oldFirst, oldSecond)
  local panel, firstListControl, secondListControl, mirroringOKButton
  local actionName, dialogCaption
  if oldFirst == nil then
    actionName = "Add"
  else
    actionName = "OK"
  end

  local onListControlChanged = function(self)
    if firstListControl and secondListControl and mirroringOKButton then
      local first = nil
      local firstRow = firstListControl:getSelectedRow()
      if firstRow then
        first = firstListControl:getItemValue(firstRow, 1)
      end

      local second = nil
      local secondRow = secondListControl:getSelectedRow()
      if secondRow then
        second = secondListControl:getItemValue(secondRow, 1)
      end

      local enable = false
      if type(first) == "string" and type(second) == "string" then
        enable = string.len(first) > 0 and string.len(second) > 0 and first ~= second
      end
      mirroringOKButton:enable(enable)
    end
  end

  local dlg = ui.getWindow("JointMirrorMapping")
  if not dlg then
    dlg = ui.createModalDialog{
      name = "JointMirrorMapping",
      caption = "Joint Mirroring",
      resize = true,
      size = { width = 200, height = -1 },
    }

    dlg:beginVSizer{ flags = "expand", proportion = 1 }
      dlg:beginHSizer{ flags = "expand", proportion = 1 }
        firstListControl = dlg:addListControl{
          name = "FirstListControl",
          columnNames = { "First" },
          flags = "expand",
          proportion = 1,
          onSelectionChanged = onListControlChanged,
        }

        secondListControl = dlg:addListControl{
          name = "SecondListControl",
          columnNames = { "Second" },
          flags = "expand",
          proportion = 1,
          onSelectionChanged = onListControlChanged,
        }
      dlg:endSizer()

      dlg:beginHSizer{ flags = "right", proportion = 0 }
        mirroringOKButton = dlg:addButton{
          name = "OKButton",
          label = "OK",
          size = { width = 74 },
        }

        dlg:addButton{
          name = "CancelButton",
          label = "Cancel",
          size = { width = 74 },
          onClick = function(self)
            dlg:hide()
          end,
        }
      dlg:endSizer()

    dlg:endSizer()
  else
    firstListControl = dlg:getChild("FirstListControl")
    secondListControl = dlg:getChild("SecondListControl")
    mirroringOKButton = dlg:getChild("OKButton")
  end

  dlg:freeze()

  mirroringOKButton:setLabel(actionName)
  mirroringOKButton:setOnClick(
    function(self)
      local first = firstListControl:getItemValue(firstListControl:getSelectedRow(), 1)
      local second = secondListControl:getItemValue(secondListControl:getSelectedRow(), 1)
      local set = animationSetsTreeControl:getSelectedItem()
      if oldFirst == nil or oldSecond == nil then
        anim.addAnimSetJointMirrorMappings(set, { { first = first, second = second } })
      else
        anim.replaceAnimSetJointMirrorMapping(set, { first = oldFirst, second = oldSecond }, { first = first, second = second })
      end
      dlg:hide()
    end
  )

  local set = animationSetsTreeControl:getSelectedItem()
  local joints = anim.getRigChannelNames(set)
  firstListControl:clearRows()
  secondListControl:clearRows()

  -- rebuild the list and select the joints
  local jointIndex = 1
  for _, joint in ipairs(joints) do
    firstListControl:addRow(joint)
    secondListControl:addRow(joint)
    if joint == oldFirst then
      firstListControl:selectRow(jointIndex)
      firstListControl:ensureVisible(jointIndex)
    end
    if joint == oldSecond then
      secondListControl:selectRow(jointIndex)
      secondListControl:ensureVisible(jointIndex)
    end
    jointIndex = jointIndex + 1
  end

  onListControlChanged()

  dlg:rebuild()
  dlg:setSize{ width = 400, height = -1 }
  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- simple function to auto generate mappings based on two filter strings ie "Left" -> "Right"
------------------------------------------------------------------------------------------------------------------------
local autoGenerateMirroredJointMappings = function(set, firstFilter, secondFilter)

  -- generate a set for quick lookup of joint names for quickly checking if the mirrored joint name exists
  local jointNames = anim.getRigChannelNames(set)
  local jointLookupSet = { }
  for _, l in ipairs(jointNames) do
    jointLookupSet[l] = true
  end

  -- generate lookup for existing joint mirror mappings
  local jointAlreadyMapped = { }
  local currentJointMirrorMappings = anim.listAnimSetJointMirrorMappings(set)
  if currentJointMirrorMappings then
    for _, mirroring in ipairs(currentJointMirrorMappings) do
      jointAlreadyMapped[mirroring.first] = true
      jointAlreadyMapped[mirroring.second] = true
    end
  end

  local newMappings = { }
  -- for each joint now see if there is a first filter pattern match
  -- generate the second joint name using the second filter pattern
  -- if the generated name is a valid joint name then add the matches to the set's mirrored joint names
  for _, jointName in ipairs(jointNames) do
    local result = string.find(jointName, firstFilter)
    if result then
      local preFilterString = string.sub(jointName, 1, result - 1)
      local postFilterString = string.sub(jointName, result + string.len(firstFilter), string.len(jointName))
      local mirroredJointName = string.format("%s%s%s", preFilterString, secondFilter, postFilterString)

      if jointLookupSet[mirroredJointName] then
        -- adding a new mapping will fail if the joint or mirror joint is already mapped
        if not jointAlreadyMapped[jointName] and not jointAlreadyMapped[mirroredJointName] then
          table.insert(newMappings, { first = jointName, second = mirroredJointName })
        end
      end
    end
  end

  if table.getn(newMappings) > 0 then
    anim.addAnimSetJointMirrorMappings(set, newMappings)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- pop up the add animation set dialog box
------------------------------------------------------------------------------------------------------------------------
local showAutoJointMirroringDialog = function()
  local panel, firstFilterTextBox, secondFilterTextBox, autoGenButton

  local onFiltersChanged = function()
    if firstFilterTextBox and secondFilterTextBox and autoGenButton then
      local enable =
        string.len(firstFilterTextBox:getValue()) > 0 and
        string.len(secondFilterTextBox:getValue()) > 0
      autoGenButton:enable(enable)
    end
  end

  local dlg = ui.getWindow("AutoJointMirrorMapping")
  if not dlg then
    dlg = ui.createModalDialog{
      name = "AutoJointMirrorMapping",
      caption = "Auto Generate Joint Mirroring",
      resize = false,
    }

    dlg:beginVSizer()
      dlg:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1, }
        dlg:addStaticText{
          text = "Search For"
        }

        firstFilterTextBox = dlg:addTextBox{
          name = "FirstFilter",
          proportion = 1,
          flags = "expand",
          onChanged = onFiltersChanged,
        }

        dlg:addStaticText{
          text = "Replace With"
        }

        secondFilterTextBox = dlg:addTextBox{
          name = "SecondFilter",
          proportion = 1,
          flags = "expand",
          onChanged = onFiltersChanged,
        }
      dlg:endSizer()

      dlg:beginHSizer{ flags = "right" }
        autoGenButton = dlg:addButton{
          name = "AddButton",
          label = "Generate",
          size = { width = 74 },
          onClick = function(self)
            local set = animationSetsTreeControl:getSelectedItem()
            local first = firstFilterTextBox:getValue()
            local second = secondFilterTextBox:getValue()
            autoGenerateMirroredJointMappings(set, first, second)
            dlg:hide()
          end,
        }

        dlg:addButton{
          name = "CancelButton",
          label = "Cancel",
          size = { width = 74 },
          onClick = function(self)
            dlg:hide()
          end,
        }
      dlg:endSizer()

    dlg:endSizer()
  else
    firstFilterTextBox = dlg:getChild("FirstFilter")
    secondFilterTextBox = dlg:getChild("SecondFilter")
    autoGenButton = dlg:getChild("AddButton")
  end

  firstFilterTextBox:setFocus{ navigation = true }
  onFiltersChanged()

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- pop up the add event mirroring dialog
------------------------------------------------------------------------------------------------------------------------
local showAddEventMirroringDialog = function(animationSetsTreeControl)
  local panel, firstUserDataTextBox, secondUserDataTextBox, mirroringOKButton

  -- only enable the add button if valid numbers are entered
  local onUserDataTextChanged = function(self)
    if firstUserDataTextBox and secondUserDataTextBox and mirroringOKButton then
      local value = firstUserDataTextBox:getValue()
      local enable = type(tonumber(value)) == "number"

      value = secondUserDataTextBox:getValue()
      enable = enable and type(tonumber(value)) == "number"

      mirroringOKButton:enable(enable)
    end
  end

  local dlg = ui.getWindow("EventMirrorMappingDialog")
  if not dlg then
    dlg = ui.createModalDialog{
      name = "EventMirrorMappingDialog",
      caption = "Add Event Mirroring",
      resize = false,
    }

    dlg:beginVSizer()
      dlg:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1, }
        dlg:addStaticText{
          text = "First User Data"
        }

        firstUserDataTextBox = dlg:addTextBox{
          name = "FirstUserDataTextBox",
          proportion = 1,
          flags = "expand",
          onChanged = onUserDataTextChanged,
        }

        dlg:addStaticText{
          text = "Second User Data"
        }

        secondUserDataTextBox = dlg:addTextBox{
          name = "SecondUserDataTextBox",
          proportion = 1,
          flags = "expand",
          onChanged = onUserDataTextChanged,
        }
      dlg:endSizer()

      dlg:beginHSizer{ flags = "right" }
        mirroringOKButton = dlg:addButton{
          name = "AddButton",
          label = "Add",
          size = { width = 74 }
        }

        dlg:addButton{
          name = "CancelButton",
          label = "Cancel",
          size = { width = 74 },
          onClick = function(self)
            dlg:hide()
          end,
        }
      dlg:endSizer()

    dlg:endSizer()
  else
    firstUserDataTextBox = dlg:getChild("FirstUserDataTextBox")
    secondUserDataTextBox = dlg:getChild("SecondUserDataTextBox")
    mirroringOKButton = dlg:getChild("AddButton")
  end
  
  -- update this each time the dialog is shown because the ui could have been rebuilt.
  local addButton = dlg:getChild("AddButton")
  addButton:setOnClick(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0
      then
        local first = tonumber(firstUserDataTextBox:getValue())
        local second = tonumber(secondUserDataTextBox:getValue())
        anim.addAnimSetEventUserdataMirrorMappings(selectedSet, { { first = first, second = second } })
        dlg:hide()
      end
    end
  )

  -- reset the controls
  firstUserDataTextBox:setFocus{ navigation = true }
  firstUserDataTextBox:setValue("")
  secondUserDataTextBox:setValue("")
  onUserDataTextChanged()

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- pop up the add event track mirroring dialog
------------------------------------------------------------------------------------------------------------------------
local showAddEventTrackMirroringDialog = function(animationSetsTreeControl)
  local panel, firstEventTrackTextBox, secondEventTrackTextBox, mirroringOKButton

  -- only enable the add button if valid numbers are entered
  local onEventTrackTextChanged = function(self)
    if firstEventTrackTextBox and secondEventTrackTextBox and mirroringOKButton then
      local value = firstEventTrackTextBox:getValue()
      local enable = type(tonumber(value)) == "number"

      value = secondEventTrackTextBox:getValue()
      enable = enable and type(tonumber(value)) == "number"

      mirroringOKButton:enable(enable)
    end
  end

  local dlg = ui.getWindow("EventTrackMirrorMappingDialog")
  if not dlg then
    dlg = ui.createModalDialog{
      name = "EventTrackMirrorMappingDialog",
      caption = "Add Event Track Mirroring",
      resize = false,
    }

    dlg:beginVSizer()
      dlg:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1, }
        dlg:addStaticText{
          text = "First Event Track ID"
        }

        firstEventTrackTextBox = dlg:addTextBox{
          name = "FirstEventTrackTextBox",
          proportion = 1,
          flags = "expand",
          onChanged = onEventTrackTextChanged,
        }

        dlg:addStaticText{
          text = "Second Event Track ID"
        }

        secondEventTrackTextBox = dlg:addTextBox{
          name = "SecondEventTrackTextBox",
          proportion = 1,
          flags = "expand",
          onChanged = onEventTrackTextChanged,
        }
      dlg:endSizer()

      dlg:beginHSizer{ flags = "right" }
        mirroringOKButton = dlg:addButton{
          name = "AddButton",
          label = "Add",
          size = { width = 74 },
          onClick = function(self)
            local selectedSet = animationSetsTreeControl:getSelectedItem()
            if type(selectedSet) == "string" and string.len(selectedSet) > 0
            then
              local first = tonumber(firstEventTrackTextBox:getValue())
              local second = tonumber(secondEventTrackTextBox:getValue())
              anim.addAnimSetEventTrackMirrorMappings(selectedSet, { { first = first, second = second } })
              dlg:hide()
            end
          end,
        }

        dlg:addButton{
          name = "CancelButton",
          label = "Cancel",
          size = { width = 74 },
          onClick = function(self)
            dlg:hide()
          end,
        }
      dlg:endSizer()

    dlg:endSizer()
  else
    firstEventTrackTextBox = dlg:getChild("FirstEventTrackTextBox")
    secondEventTrackTextBox = dlg:getChild("SecondEventTrackTextBox")
    mirroringOKButton = dlg:getChild("AddButton")
  end

  -- reset the controls
  firstEventTrackTextBox:setFocus{ navigation = true }
  firstEventTrackTextBox:setValue("")
  secondEventTrackTextBox:setValue("")
  onEventTrackTextChanged()

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the toolbar and buttons to the top of the animation sets dialog
------------------------------------------------------------------------------------------------------------------------
addToolbarAndButtons = function()

  local panel = animationSetsRollupHeading
  local moveup = app.loadImage("moveup.png")
  local movedown = app.loadImage("movedown.png")
  local removeitem = app.loadImage("removeitem.png")
  local additem = app.loadImage("additem.png")

  panel:setLabel("")
  panel:setBorder(0)

  panel:beginHSizer{ flags = "expand", proportion = 0 }
    rollupHeadingLabel = panel:addStaticText{ text = "Animation Sets", font = "bold", flags = "parentBackground;truncate;expand;decoration", proportion = 1 }

    moveUpSetButton = panel:addButton{
      name = "MoveUpButton", label = "",
      image = moveup, flags = "expand;parentBackground",
      helpText = "Reorder Animation Set",
      size = { width = moveup:getWidth(), height = moveup:getHeight() },
    }

    moveDownSetButton = panel:addButton{
      name = "MoveDownButton", label = "",
      image = movedown, flags = "expand;parentBackground",
      helpText = "Reorder Animation Set",
      size = { width = movedown:getWidth(), height = movedown:getHeight() },
    }

    panel:addHSpacer(2)

    removeSetButton = panel:addButton{
      name = "RemoveButton", label = "",
      image = removeitem, flags = "expand;parentBackground",
      helpText = "Delete Animation Set",
      size = { width = removeitem:getWidth(), height = removeitem:getHeight() },
    }
    removeSetButton:setToolTip("Delete Animation Set")

    addSetButton = panel:addButton{
      name = "AddButton", label = "",
      image = additem, flags = "expand;parentBackground",
      helpText = "Add Animation Set",
      size = { width = additem:getWidth(), height = additem:getHeight() },
    }
    addSetButton:setToolTip("Add Animation Set")

  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Sets the enable state for the rollupHeading header buttons
-- depending on the selection in the animation sets tree control.
------------------------------------------------------------------------------------------------------------------------
updateToolbarButtons = function()
  -- get the currently selected set
  local selectedSet = animationSetsTreeControl:getSelectedItem()
  local setIsValid = type(selectedSet) == "string" and string.len(selectedSet) > 0

  -- if the animation set is valid
  if setIsValid then
    local animationSetCount = table.getn(listAnimSets())
    local animationSetReferenced = anim.isAnimSetReferenced(selectedSet)
    -- enable the remove button if there is more than one animation set left
    -- and the selected set has not been used in a reference network.
    local removeEnabled = animationSetCount > 1 and not animationSetReferenced
      removeSetButton:enable(removeEnabled)
    if removeEnabled then
      removeSetButton:setToolTip("Delete Animation Set")
    else
      if animationSetReferenced then
        removeSetButton:setToolTip("Cannot remove Animation Sets used in reference files")    
      else
        removeSetButton:setToolTip("Cannot remove the last Animation Set")    
      end
    end

    -- get the index of the current set relative to it's other siblings
    local parentSet = getAnimSetParent(selectedSet)
    local nextSet = getAnimSetNextSibling(selectedSet)
    local previousSet = getAnimSetPreviousSibling(selectedSet)

    -- if it's not the first set or it has a parent set then enable the move up button
    moveUpSetButton:enable(type(previousSet) == "string" or type(parentSet) == "string")
    -- if it's not the last set enable the move down button
    moveDownSetButton:enable(type(nextSet) == "string")
  else
    removeSetButton:enable(false)
    removeSetButton:setToolTip("")
    moveUpSetButton:enable(false)
    moveDownSetButton:enable(false)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Sets the callbacks for the rollupHeading header buttons
------------------------------------------------------------------------------------------------------------------------
setupToolbarCallbacks = function(dlg)

  addSetButton:setOnClick(
    function(self)
      animationSetsRollupHeading:expand(true)
      local created, newName = newAnimationSetWizard()
      if created then
        createdItem = animationSetsTreeControl:findItem(newName)
        animationSetsTreeControl:selectItem(createdItem)
      end
    end
  )

  removeSetButton:setOnClick(
    function(self)
      -- get the currently selected set
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" then
        if canDeleteAnimationSet(selectedSet) then
          deleteAnimSet(selectedSet)
          updateToolbarButtons()
        end
      end
    end
  )

  moveUpSetButton:setOnClick(
    function(self)
      -- get the currently selected set
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        local parentSet = getAnimSetParent(selectedSet)
        local previousSet = getAnimSetPreviousSibling(selectedSet)
        if type(previousSet) == "string" then
          moveAnimSetBeforeSibling(selectedSet, previousSet)
          updateToolbarButtons()
        elseif type(parentSet) == "string" then
          local grandParentSet = getAnimSetParent(parentSet)
          setAnimSetParent(selectedSet, grandParentSet)
          updateToolbarButtons()
        end
      end
    end
  )

  moveDownSetButton:setOnClick(
    function(self)
      -- get the currently selected set
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        local nextSet = getAnimSetNextSibling(selectedSet)
        if type(nextSet) == "string" then
          moveAnimSetAfterSibling(selectedSet, nextSet)
          updateToolbarButtons()
        end
      end
    end
  )
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the rig settings rollup and all it's controls
------------------------------------------------------------------------------------------------------------------------
addRigSettingsRollup = function(panel)
  -- add rig settings controls
  -- add animation format options controls
  local rigSettingsRollup = panel:addRollup{
    name = "RigSettingsRollup",
    label = "Character",
    flags = "expand;mainSection",
  }

  local rigSettingsPanel = rigSettingsRollup:getPanel()
  rigSettingsPanel:beginVSizer()
    rigSettingsPanel:setBorder(2)
    
    if not mcn.isPhysicsDisabled() then
      rigSettingsPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
      rigSettingsPanel:setFlexGridColumnExpandable(2)

        local componentTypes = characterTypes.listNames();
        rigSettingsPanel:addStaticText{ text = "Type", font = "bold" }
        local characterTypeCombo = rigSettingsPanel:addComboBox{
          name = "characterTypeCombo",
          flags = "expand",
          proportion = 0,
          items = componentTypes,
        }
        animationSetsAttributeEditor.bindHelpToWidget(characterTypeCombo, "The type of character, Animation only, Animation with Physics or Animation with Physics and Euphoria.")
        
      rigSettingsPanel:endSizer()
    end
    
    rigSettingsPanel:beginHSizer{ cols = 2, flags = "expand"}
      rigSettingsPanel:addHSpacer(6)
      rigSettingsPanel:setBorder(1)
      rigSettingsPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      rigSettingsPanel:setFlexGridColumnExpandable(2)

        rigSettingsPanel:addStaticText{ text = "Animation Rig", name = "AnimRigLabel", }
        local animationRigFileControl = rigSettingsPanel:addFilenameControl{
          name = "AnimRigFileControl",
          flags = "expand",
          wildcard = "morpheme:connect animation rig files|mcarig",
          proportion = 1,
          onMacroize = utils.macroizeString,
          onDemacroize = utils.demacroizeString,
        }

        if not mcn.isPhysicsDisabled() then
          rigSettingsPanel:addStaticText{ text = "Physics Rig", name = "PhysicsRigLabel",  }
          local physicsRigFileControl = rigSettingsPanel:addFilenameControl{
            name = "PhysicsRigFileControl",
            flags = "expand",
            wildcard = "morpheme:connect physics rig files|mcprig",
            proportion = 1,
            onMacroize = utils.macroizeString,
            onDemacroize = utils.demacroizeString,
          }
        end

        rigSettingsPanel:addStaticText{ text = "Source Units" }
        local sourceUnitCombo = addUnitComboBox(rigSettingsPanel, "SourceUnitCombo", 1)
      rigSettingsPanel:endSizer()
    rigSettingsPanel:endSizer()
  rigSettingsPanel:endSizer()
  
end

------------------------------------------------------------------------------------------------------------------------
-- Sets the rig settings rollup control's callbacks
------------------------------------------------------------------------------------------------------------------------
setupRigSettingsCallbacks = function(dlg)
  local rigSettingsRollup = dlg:findDescendant("RigSettingsRollup")
  local rigSettingsPanel = rigSettingsRollup:getPanel()
  local characterTypeCombo = rigSettingsPanel:getChild("characterTypeCombo")
  local animRigFileControl = rigSettingsPanel:getChild("AnimRigFileControl")
  local physicsRigFileControl = rigSettingsPanel:getChild("PhysicsRigFileControl")
  local sourceUnitCombo = rigSettingsPanel:getChild("SourceUnitCombo")

  rigSettingsRollup:setOnExpandCollapse(
    function(self)
      dlg:freeze()
      dlg:rebuild()
    end
  )

  -- change the character type combo
  if characterTypeCombo then
    characterTypeCombo:setOnChanged(
      function(self)
        local selectedSet = animationSetsTreeControl:getSelectedItem()
        if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
          if not anim.setAnimSetCharacterType(selectedSet, self:getSelectedItem()) then
            local selectedTypeName = anim.getAnimSetCharacterType(selectedSet)
            characterTypeCombo:setSelectedItem(selectedTypeName)
          end
        end
      end
    )
  end
  
  -- ensure the animation rig file control only allows mcarigs and macroizes the string
  animRigFileControl:setOnChanged(
    function(self)
      -- get the currently selected set
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        if canEditAnimationSet(selectedSet) then
          local value = self:getValue()
          if string.len(value) > 0 then
            value = stripFilenameExtension(value)
            value = utils.macroizeString(string.format("%s.mcarig", value))
            self:setValue(value)
          end
          anim.setRigPath(value, selectedSet)
        else
          -- selection cancelled, reset old rig name
          local value = anim.getRigPath(selectedSet)
          self:setValue(value)
        end
      else
        self:clear()
      end
    end
  )

  if not mcn.isPhysicsDisabled() then
    -- ensure the physics rig file control only allows mcprigs and macroizes the string
    physicsRigFileControl:setOnChanged(
      function(self)
        -- get the currently selected set
        local selectedSet = animationSetsTreeControl:getSelectedItem()
        if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
          if canEditAnimationSet(selectedSet) then
            local value = self:getValue()
            if string.len(value) > 0 then
              value = stripFilenameExtension(value)
              value = utils.macroizeString(string.format("%s.mcprig", value))
              self:setValue(value)
            end
            anim.setPhysicsRigPath(value, selectedSet)
          else
            -- selection cancelled, reset old rig name
            local value = anim.getPhysicsRigPath(selectedSet)
            self:setValue(value)
          end
        else
          self:clear()
        end
      end
    )
  end

  sourceUnitCombo:setOnChanged(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        if canEditAnimationSet(selectedSet) then
          local selection = self:getSelectedItem()
          local unit = units.findByName(selection)
          units.setRigUnit(selectedSet, unit.name)
        else
          -- reset to the previous value
          local setRigUnitFactor = units.getRigUnit(selectedSet)
          self:setSelectedItem(setRigUnitFactor.longname)
        end
      end
    end
  )

  -- set the default location to point to the current default rig in preferences
  local defaultAnimationRigFile = preferences.get("DefaultAnimationRigFile")
  if type(defaultAnimationRigFile) == "string" then
    local defaultRigDir = splitFilePath(utils.demacroizeString(defaultAnimationRigFile))
    animRigFileControl:setDefaultDirectory(defaultRigDir, true)
  end

  if not mcn.isPhysicsDisabled() then
    local defaultPhysicsRigFile = preferences.get("DefaultPhysicsRigFile")
    if type(defaultPhysicsRigFile) == "string" then
      if string.len(defaultPhysicsRigFile) == 0 then
        defaultPhysicsRigFile = preferences.get("DefaultAnimationRigFile")
      end
    end

    if type(defaultPhysicsRigFile) == "string" then
      local defaultRigDir = splitFilePath(utils.demacroizeString(defaultPhysicsRigFile))
      physicsRigFileControl:setDefaultDirectory(defaultRigDir, true)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Sets the rig settings rollup control's enable state depending on the current animation set selection
------------------------------------------------------------------------------------------------------------------------
updateRigSettingsRollup = function(dlg)
  local rigSettingsRollup = dlg:findDescendant("RigSettingsRollup")
  local rigSettingsPanel = rigSettingsRollup:getPanel()
  local characterTypeCombo = rigSettingsPanel:getChild("characterTypeCombo")
  local animRigLabel = rigSettingsPanel:getChild("AnimRigLabel")
  local animRigFileControl = rigSettingsPanel:getChild("AnimRigFileControl")
  local physicsRigFileControl = rigSettingsPanel:getChild("PhysicsRigFileControl")
  local physicsRigLabel = rigSettingsPanel:getChild("PhysicsRigLabel")
  local sourceUnitCombo = rigSettingsPanel:getChild("SourceUnitCombo")

  local selectedSet = animationSetsTreeControl:getSelectedItem()
  local setIsValid = type(selectedSet) == "string" and string.len(selectedSet) > 0

  for _, child in ipairs(rigSettingsPanel:getChildren()) do
    child:enable(setIsValid)
  end

  if setIsValid then
    local selectedTypeName = anim.getAnimSetCharacterType(selectedSet)
    local supportsAnimationRig = characterTypes.supportsRig(selectedTypeName, "AnimationRig")
    local supportsPhysicsRig = characterTypes.supportsRig(selectedTypeName, "PhysicsRig")

    if characterTypeCombo then
      characterTypeCombo:setSelectedItem(selectedTypeName)
    end
    
    -- Animation Rig
    if supportsAnimationRig then
      local animationRigPath = anim.getRigPath(selectedSet)
      animRigFileControl:setValue(animationRigPath)

      -- set the default directory of the file control so it opens the browser in the right directory
      if string.len(animationRigPath) > 0 then
        local fullPath = utils.demacroizeString(animationRigPath)
        local defaultRigDir = splitFilePath(fullPath)
        -- the second parameter forces the control to use the default directory not the value of the control
        -- when opening the file browser
        animRigFileControl:setDefaultDirectory(defaultRigDir, true)
        if app.fileExists(fullPath) then
          animRigFileControl:setError(false)
          animationSetsAttributeEditor.bindHelpToWidget(animRigFileControl, "Animation rig.")
        else
          animRigFileControl:setError(true)
          animationSetsAttributeEditor.bindHelpToWidget(animRigFileControl, "The animation rig does not exist.")
        end
      else
        local defaultRigFile = preferences.get("DefaultAnimationRigFile")
        if type(defaultRigFile) == "string" then
          local defaultRigDir = splitFilePath(utils.demacroizeString(defaultRigFile))
          -- the second parameter forces the control to use the default directory not the value of the control
          -- when opening the file browser
          animRigFileControl:setDefaultDirectory(defaultRigDir, true)
        end
        animRigFileControl:setError(true)
        animationSetsAttributeEditor.bindHelpToWidget(animRigFileControl, "Animation rig - this needs to be specified.")
      end
    else
      animRigLabel:enable(false)
      animRigFileControl:setValue("")
      animRigFileControl:enable(false)
    end

    -- Physics Rig
    if not mcn.isPhysicsDisabled() then
      if supportsPhysicsRig then
        local physicsRigPath = anim.getPhysicsRigPath(selectedSet)
        physicsRigFileControl:setValue(physicsRigPath)

        -- set the default directory of the file control so it opens the browser in the right directory
        if string.len(physicsRigPath) > 0 then
          local fullPath = utils.demacroizeString(physicsRigPath)
          local defaultRigDir = splitFilePath(fullPath)
          -- the second parameter forces the control to use the default directory not the value of the control
          -- when opening the file browser
          physicsRigFileControl:setDefaultDirectory(defaultRigDir, true)
          physicsRigFileControl:setError(false)
          if app.fileExists(fullPath) then
            physicsRigFileControl:setError(false)
            animationSetsAttributeEditor.bindHelpToWidget(physicsRigFileControl, "Physics rig.")
          else
            physicsRigFileControl:setError(true)
            animationSetsAttributeEditor.bindHelpToWidget(animRigFileControl, "The physics rig does not exist.")
          end
       else
          local defaultRigFile = preferences.get("DefaultPhysicsRigFile")
          if type(defaultRigFile) == "string" then
            if string.len(defaultRigFile) == 0 then
              defaultRigFile = preferences.get("DefaultAnimationRigFile")
            end
          end

          if type(defaultRigFile) == "string" then
            local defaultRigDir = splitFilePath(utils.demacroizeString(defaultRigFile))
            -- the second parameter forces the control to use the default directory not the value of the control
            -- when opening the file browser
            physicsRigFileControl:setDefaultDirectory(defaultRigDir, true)
          end
          physicsRigFileControl:setError(true)
          animationSetsAttributeEditor.bindHelpToWidget(physicsRigFileControl, "Physics rig - this needs to be specified.")
        end
      else
        physicsRigFileControl:setValue("")
        physicsRigLabel:enable(false)
        physicsRigFileControl:enable(false)
      end
    end

    local unit = units.getRigUnit(selectedSet)
    if not unit then
      local runtimeAssetScaleFactor = preferences.get("RuntimeAssetScaleFactor")
      unit = units.findByScaleFactor(runtimeAssetScaleFactor)
    end
    sourceUnitCombo:setSelectedItem(unit.longname or unit.name)
  else
    animRigFileControl:setValue("")
    if not mcn.isPhysicsDisabled() then
      physicsRigFileControl:setValue("")
    end

    local runtimeAssetScaleFactor = preferences.get("RuntimeAssetScaleFactor")
    local unit = units.findByScaleFactor(runtimeAssetScaleFactor)
    sourceUnitCombo:setSelectedItem(unit.longname or unit.name)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Add Computed Channels
------------------------------------------------------------------------------------------------------------------------

addComputedChannelsRollup = function(panel)

 local channelsRollup = panel:addRollup{
    name = "computedChannels",
    label = "Channels",
    flags = "expand;mainSection",
  }
  
  local channelsPanel = channelsRollup:getPanel()
  channelsRollup:expand(false)
  
 
end

------------------------------------------------------------------------------------------------------------------------
-- Update the channels rollup
------------------------------------------------------------------------------------------------------------------------
updateChannelsRollup = function(panel)
  local channelsRollup = panel:findDescendant("computedChannels")
  local channelsPanel = channelsRollup:getPanel()
  channelsPanel:clear()

  local selectedSet = animationSetsTreeControl:getSelectedItem()
  local setIsValid = type(selectedSet) == "string" and string.len(selectedSet) > 0
 
  if setIsValid then
    channelsPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1}
    channelsPanel:setFlexGridColumnExpandable(2)

            channelsPanel:addStaticText{ name = "KinectJointLabel", text = "Joint" }
            channelsPanel:addStaticText{ name = "TargetJointLabel", text = "Calculate" }
            channelsPanel:addVSpacer(2)
            channelsPanel:addVSpacer(2)
    local specialIndices = {40, 50, 70, 1000}
    local curSpecialIndexOffset = 1
       local jointNames = anim.getRigChannelNames(selectedSet)
         for index, jointName in ipairs(jointNames) do
            
            channelsPanel:addStaticText{
                name = string.format("JointLabel%d", index),
                text = jointName
              }
            
            local curCombo = channelsPanel:addComboBox{
                name = jointName,
                flags = "expand",
                proportion = 1
              }
              
              curCombo:addItem("Orientation and Postion")
              curCombo:addItem("Orientation Only")
              curCombo:addItem("Bind Pose")
              
              if(specialIndices[curSpecialIndexOffset] == index) then
                  channelsPanel:addStaticText{
                    name = "LineBreak1",
                     text = string.format(" --- End of Compute Set %d",  (4- curSpecialIndexOffset))
                  }
              
                channelsPanel:addStaticText{
                    name = "LineBreak2",
                     text = "------------------"
                  }
                curSpecialIndexOffset = curSpecialIndexOffset + 1
              end
         end

     channelsPanel:endSizer()
   end

end

------------------------------------------------------------------------------------------------------------------------
-- Adds the format options rollup and all it's controls
------------------------------------------------------------------------------------------------------------------------
addFormatOptionsRollup = function(panel)
  -- add animation format options controls
  local formatOptionsRollup = panel:addRollup{
    name = "FormatOptionsRollup",
    label = "Animation Compression",
    flags = "expand;mainSection",
  }
  formatOptionsRollup:expand(false)

  local formatOptionsPanel = formatOptionsRollup:getPanel()
  formatOptionsPanel:beginVSizer{ flags = "expand", proportion = 1 }
    formatOptionsPanel:beginHSizer{ flags = "expand", proportion = 1 }
      formatOptionsPanel:addStaticText{ name = "FormatText", text = "Format" }

      local items = { }
      for i, value in ipairs(animfmt.ls()) do
        table.insert(items, value.format)
      end

      local formatCombo = formatOptionsPanel:addComboBox{
        name = "FormatCombo",
        flags = "expand",
        proportion = 1,
        items = items,
      }
      local defaultAnimationFormat = preferences.get("DefaultAnimationFormat")
      formatCombo:setSelectedItem(defaultAnimationFormat)
    formatOptionsPanel:endSizer()

    local formatSpecificPanel = formatOptionsPanel:addPanel{ name = "FormatSpecificPanel", flags = "expand" }
    formatSpecificPanel:beginVSizer()
    formatSpecificPanel:endSizer()

  formatOptionsPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Set the callbacks for the format combo box, the other controls are done via the callbacks
-- set when registering each animation compression format.
------------------------------------------------------------------------------------------------------------------------
setupFormatOptionsCallbacks = function(dlg)

  local formatOptionsRollup = dlg:findDescendant("FormatOptionsRollup")
  local formatOptionsPanel = formatOptionsRollup:getPanel()
  local formatCombo = formatOptionsPanel:getChild("FormatCombo")

  formatOptionsRollup:setOnExpandCollapse(
    function(self)
      dlg:freeze()
      dlg:rebuild()
    end
  )

  -- set the format combo changed function.
  formatCombo:setOnChanged(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        if canEditAnimationSet(selectedSet) then
          local formatString = self:getSelectedItem()
          anim.setAnimSetFormat(selectedSet, formatString)
          local options = animfmt.parseOptions(anim.getAnimSetOptions(selectedSet))
          local options = animfmt.removeInvalidOptions(formatString, options)
          anim.setAnimSetOptions(selectedSet, animfmt.compileOptions(options))
        else
          -- selection cancelled, reset format
          local formatString = anim.getAnimSetFormat(selectedSet)
          self:setSelectedItem(formatString)
        end
      end
    end
  )
end

------------------------------------------------------------------------------------------------------------------------
-- Update the ui for the format options rollup depending on the current animation set selection
-- also calls the update custom format options ui function that is registered with the animation
-- compression format.
------------------------------------------------------------------------------------------------------------------------
updateFormatOptionsRollup = function(panel)
  -- first get all the controls from the dialog
  local formatOptionsRollup = panel:findDescendant("FormatOptionsRollup")
  local formatOptionsPanel = formatOptionsRollup:getPanel()
  local formatCombo = formatOptionsPanel:getChild("FormatCombo")
  local formatSpecificPanel = formatOptionsPanel:getChild("FormatSpecificPanel")

  -- get the currently selected set and format
  local selectedSet = animationSetsTreeControl:getSelectedItem()
  local setIsValid = type(selectedSet) == "string" and string.len(selectedSet) > 0
  local format = animfmt.get(formatCombo:getSelectedItem())

  -- if the animation set is valid
  if setIsValid then
    -- get the format of the selected set and set the value of the combo
    local formatString = anim.getAnimSetFormat(selectedSet)
    format = animfmt.get(formatString)
    formatCombo:setSelectedItem(formatString)
  end

  -- set the enable state of everything in the panel
  for _, child in ipairs(formatOptionsPanel:getChildren()) do
    child:enable(setIsValid)
  end

  -- these functions should do nothing if there is no set selected.
  local getOptionsTable = function(index)
    local selectedSet = animationSetsTreeControl:getSelectedItem()
    if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
      local optionsString = anim.getAnimSetOptions(selectedSet)
      return animfmt.parseOptions(optionsString)
    end
    return { }
  end

  local shouldEnableControls = function()
    local selectedSet = animationSetsTreeControl:getSelectedItem()
    return type(selectedSet) == "string" and string.len(selectedSet) > 0
  end

  local setOptionsTable = nil
  setOptionsTable = function(index, options)
    local selectedSet = animationSetsTreeControl:getSelectedItem()
    if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
      if canEditAnimationSet(selectedSet) then
        local optionsString = animfmt.compileOptions(options)
        anim.setAnimSetOptions(selectedSet, optionsString)
      elseif type(format.updateFormatOptionsPanel) == "function" then
        -- the selection has been cancelled, so just update the panel
        format.updateFormatOptionsPanel(formatSpecificPanel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
      end
    end
  end

  if format.format ~= displayedCompressionFormat then
    -- set the format options section to display the controls for the currently selected format
    formatSpecificPanel:clear()
    formatSpecificPanel:beginVSizer{ flags = "expand" }
    if type(format.addFormatOptionsPanel) == "function" and type(format.updateFormatOptionsPanel) == "function" then
      -- add any format specific controls
      format.addFormatOptionsPanel(formatSpecificPanel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
      format.updateFormatOptionsPanel(formatSpecificPanel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
    end
    formatSpecificPanel:endSizer()
    displayedCompressionFormat = format.format
  elseif type(format.updateFormatOptionsPanel) == "function" then
    -- the displayed format hasn't changed so just update the panel
    format.updateFormatOptionsPanel(formatSpecificPanel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the mirror setings rollup and all it's controls
------------------------------------------------------------------------------------------------------------------------
addMirrorSettingsRollup = function(panel)
  -- add animation format options controls
  local mirrorSettingsRollup = panel:addRollup{
    name = "MirrorSettingsRollup",
    label = "Mirroring",
    flags = "expand;mainSection",
  }
  mirrorSettingsRollup:expand(false)

  local mirrorSettingsPanel = mirrorSettingsRollup:getPanel()
  mirrorSettingsPanel:setBorder(0)
  mirrorSettingsPanel:beginVSizer{ flags = "expand" }

    -- add the joint mirroring ui
    local jointMirroringRollup = mirrorSettingsPanel:addRollup{
      name = "JointMirroringRollup",
      label = "Joint Mirroring",
      flags = "expand",
    }
    jointMirroringRollup:expand(false)
    local jointMirroringPanel = jointMirroringRollup:getPanel()

    jointMirroringPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
    jointMirroringPanel:setFlexGridColumnExpandable(2)

      -- add the mirror plane text and combo box
      jointMirroringPanel:addStaticText{ name = "MirrorPlaneText", text = "Mirror Plane" }
      jointMirroringPanel:addComboBox{
        name = "MirrorPlaneCombo",
        items = {
          "XY",
          "XZ",
          "YZ",
        },
        flags = "expand",
        proportion = 1,
      }

      local mirrorJointsPanel = jointMirroringPanel:addPanel{
        name = "MirrorJointsPanel",
        flags = "expand",
        proportion = 1,
      }
      mirrorJointsPanel:beginVSizer{ }
        mirrorJointsPanel:addStaticText{ name = "MirrorJointText", text = "Mirrored Joints" }
        mirrorJointsPanel:addButton{
          name = "AddButton",
          label = "Add",
          size = { width = 74, height = -1 },
        }

        mirrorJointsPanel:addButton{
          name = "EditButton",
          label = "Edit",
          size = { width = 74, height = -1 },
        }

        mirrorJointsPanel:addButton{
          name = "AutoButton",
          label = "Auto",
          size = { width = 74, height = -1 },
        }

        mirrorJointsPanel:addButton{
          name = "RemoveButton",
          label = "Remove",
          size = { width = 74, height = -1 },
        }
      mirrorJointsPanel:endSizer()

      jointMirroringPanel:addListControl{
        name = "ListControl",
         size = { height = -1 },
         columnNames = {
          "first",
          "second",
        },
        flags = "sizeToContent;multiSelect;gridLines;expand",
      }

    jointMirroringPanel:endSizer()

    -- add the event mirroring ui
    local eventMirroringRollup = mirrorSettingsPanel:addRollup{
      name = "EventMirroringRollup",
      label = "Event Mirroring",
      flags = "expand",
    }
    eventMirroringRollup:expand(false)
    local eventMirroringPanel = eventMirroringRollup:getPanel()

    eventMirroringPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
    eventMirroringPanel:setFlexGridColumnExpandable(2)

      -- add the event mirroring text, list control and buttons
      eventMirroringPanel:addStaticText{ name = "EventUserDataText", text = "Event User Data" }

      local mirrorEventsPanel = eventMirroringPanel:addPanel{
        name = "MirrorEventsPanel",
        flags = "expand",
        proportion = 1,
      }

      mirrorEventsPanel:beginVSizer{ }
        mirrorEventsPanel:addListControl{
          name = "ListControl",
          columnNames = {
            "first",
            "second",
          },
          flags = "multiSelect;gridLines;expand",
        }

        mirrorEventsPanel:beginHSizer{ flags = "right" }
          mirrorEventsPanel:addButton{
            name = "AddButton",
            label = "Add",
            size = { width = 74, height = -1 },
          }

          mirrorEventsPanel:addButton{
            name = "RemoveButton",
            label = "Remove",
            size = { width = 74, height = -1 },
          }
        mirrorEventsPanel:endSizer()
      mirrorEventsPanel:endSizer()

      -- add the event track mirroring text, list control and buttons
      eventMirroringPanel:addStaticText{ name = "EventTrackText", text = "Event Tracks" }

      local mirrorEventTracksPanel = eventMirroringPanel:addPanel{
        name = "MirrorEventTracksPanel",
        flags = "expand",
        proportion = 1,
      }

      mirrorEventTracksPanel:beginVSizer{ }
        mirrorEventTracksPanel:addListControl{
          name = "ListControl",
          columnNames = {
            "first",
            "second",
          },
          flags = "multiSelect;gridLines;expand",
        }

        mirrorEventTracksPanel:beginHSizer{ flags = "right" }
          mirrorEventTracksPanel:addButton{
            name = "AddButton",
            label = "Add",
            size = { width = 74, height = -1 },
          }

          mirrorEventTracksPanel:addButton{
            name = "RemoveButton",
            label = "Remove",
            size = { width = 74, height = -1 },
          }
        mirrorEventTracksPanel:endSizer()
      mirrorEventTracksPanel:endSizer()

    eventMirroringPanel:endSizer()

  mirrorSettingsPanel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- Sets up the callbacks for the mirror settings controls
------------------------------------------------------------------------------------------------------------------------
setupMirrorSettingsCallbacks = function(dlg)
  local mirrorSettingsRollup = dlg:findDescendant("MirrorSettingsRollup")
  local mirrorSettingsPanel = mirrorSettingsRollup:getPanel()

  local jointMirroringRollup = mirrorSettingsPanel:getChild("JointMirroringRollup")
  local jointMirroringPanel = jointMirroringRollup:getPanel()
  local mirrorPlaneCombo = jointMirroringPanel:getChild("MirrorPlaneCombo")
  local mirrorJointsListControl = jointMirroringPanel:getChild("ListControl")

  local mirrorJointsPanel = jointMirroringPanel:getChild("MirrorJointsPanel")
  local mirrorJointsAddButton = mirrorJointsPanel:getChild("AddButton")
  local mirrorJointsEditButton = mirrorJointsPanel:getChild("EditButton")
  local mirrorJointsAutoButton = mirrorJointsPanel:getChild("AutoButton")
  local mirrorJointsRemoveButton = mirrorJointsPanel:getChild("RemoveButton")

  local eventMirroringRollup = mirrorSettingsPanel:getChild("EventMirroringRollup")
  local eventMirroringPanel = eventMirroringRollup:getPanel()

  local mirrorEventsPanel = eventMirroringPanel:getChild("MirrorEventsPanel")
  local mirrorEventsListControl = mirrorEventsPanel:getChild("ListControl")
  local mirrorEventsAddButton = mirrorEventsPanel:getChild("AddButton")
  local mirrorEventsRemoveButton = mirrorEventsPanel:getChild("RemoveButton")

  local mirrorEventTracksPanel = eventMirroringPanel:getChild("MirrorEventTracksPanel")
  local mirrorEventTracksListControl = mirrorEventTracksPanel:getChild("ListControl")
  local mirrorEventTracksAddButton = mirrorEventTracksPanel:getChild("AddButton")
  local mirrorEventTracksRemoveButton = mirrorEventTracksPanel:getChild("RemoveButton")

  local onRollupExpandCollapse = function(self)
    dlg:freeze()
    dlg:rebuild()
  end

  mirrorSettingsRollup:setOnExpandCollapse(onRollupExpandCollapse)
  jointMirroringRollup:setOnExpandCollapse(onRollupExpandCollapse)
  eventMirroringRollup:setOnExpandCollapse(onRollupExpandCollapse)

  mirrorPlaneCombo:setOnChanged(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        anim.setAnimSetJointMirrorPlane(selectedSet, self:getSelectedItem())
      end
    end
  )

  mirrorJointsListControl:setOnSelectionChanged(
    function(self)
     local selection = mirrorJointsListControl:getSelectedRows()
      local itemsInSelection = table.getn(selection)
      mirrorJointsEditButton:enable(itemsInSelection == 1)
      mirrorJointsRemoveButton:enable(itemsInSelection > 0)
    end
  )

  mirrorJointsAddButton:setOnClick(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        poseJointMirroringDialog()

        mirrorJointsListControl:clearRows()
        local mirroredJoints = anim.listAnimSetJointMirrorMappings(selectedSet)
        for _, mirroring in ipairs(mirroredJoints) do
          mirrorJointsListControl:addRow{ mirroring.first, mirroring.second }
        end
      end
    end
  )

  mirrorJointsEditButton:setOnClick(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        local row = mirrorJointsListControl:getSelectedRow()
        local first = nil
        if row then
          first = mirrorJointsListControl:getItemValue(row, 1)
        end

        local second = nil
        if row then
          second = mirrorJointsListControl:getItemValue(row, 2)
        end

        poseJointMirroringDialog(first, second)

        mirrorJointsListControl:clearRows()
        local mirroredJoints = anim.listAnimSetJointMirrorMappings(selectedSet)
        for _, mirroring in ipairs(mirroredJoints) do
          mirrorJointsListControl:addRow{ mirroring.first, mirroring.second }
        end
      end
    end
  )

  mirrorJointsAutoButton:setOnClick(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        showAutoJointMirroringDialog()

        mirrorJointsListControl:clearRows()
        local mirroredJoints = anim.listAnimSetJointMirrorMappings(selectedSet)
        for _, mirroring in ipairs(mirroredJoints) do
          mirrorJointsListControl:addRow{ mirroring.first, mirroring.second }
        end
     end
    end
  )

  mirrorJointsRemoveButton:setOnClick(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        -- remove the selected rows
        local rows = mirrorJointsListControl:getSelectedRows()
        anim.removeAnimSetJointMirrorMappings(selectedSet, rows)

        mirrorJointsListControl:clearRows()
        local mirroredJoints = anim.listAnimSetJointMirrorMappings(selectedSet)
        for _, mirroring in ipairs(mirroredJoints) do
          mirrorJointsListControl:addRow{ mirroring.first, mirroring.second }
        end
     end
    end
  )

  mirrorEventsAddButton:setOnClick(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        showAddEventMirroringDialog(animationSetsTreeControl)
     end
    end
  )

  mirrorEventsRemoveButton:setOnClick(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        -- remove the selected rows
        local rows = mirrorEventsListControl:getSelectedRows()
        anim.removeAnimSetEventUserdataMirrorMappings(selectedSet, rows)
      end
    end
  )

  mirrorEventTracksAddButton:setOnClick(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        showAddEventTrackMirroringDialog(animationSetsTreeControl)
      end
    end
  )

  mirrorEventTracksRemoveButton:setOnClick(
    function(self)
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      if type(selectedSet) == "string" and string.len(selectedSet) > 0 then
        -- remove the selected rows
        local rows = mirrorEventTracksListControl:getSelectedRows()
        anim.removeAnimSetEventTrackMirrorMappings(selectedSet, rows)
      end
    end
  )
end

------------------------------------------------------------------------------------------------------------------------
-- Updates the enable state of the mirror settings ui depending on the current animation sets
-- controls
------------------------------------------------------------------------------------------------------------------------
updateMirrorSettingsRollup = function(panel)
  local mirrorSettingsRollup = panel:findDescendant("MirrorSettingsRollup")
  local mirrorSettingsPanel = mirrorSettingsRollup:getPanel()

  local jointMirroringRollup = mirrorSettingsPanel:getChild("JointMirroringRollup")
  local jointMirroringPanel = jointMirroringRollup:getPanel()
  local mirrorPlaneCombo = jointMirroringPanel:getChild("MirrorPlaneCombo")
  local mirrorJointsListControl = jointMirroringPanel:getChild("ListControl")

  local mirrorJointsPanel = jointMirroringPanel:getChild("MirrorJointsPanel")
  local mirrorJointsAddButton = mirrorJointsPanel:getChild("AddButton")
  local mirrorJointsEditButton = mirrorJointsPanel:getChild("EditButton")
  local mirrorJointsAutoButton = mirrorJointsPanel:getChild("AutoButton")
  local mirrorJointsRemoveButton = mirrorJointsPanel:getChild("RemoveButton")

  local eventMirroringRollup = mirrorSettingsPanel:getChild("EventMirroringRollup")
  local eventMirroringPanel = eventMirroringRollup:getPanel()

  local mirrorEventsPanel = eventMirroringPanel:getChild("MirrorEventsPanel")
  local mirrorEventsListControl = mirrorEventsPanel:getChild("ListControl")

  local mirrorEventTracksPanel = eventMirroringPanel:getChild("MirrorEventTracksPanel")
  local mirrorEventTracksListControl = mirrorEventTracksPanel:getChild("ListControl")

  -- get the currently selected set and format
  local selectedSet = animationSetsTreeControl:getSelectedItem()
  local setIsValid = type(selectedSet) == "string" and string.len(selectedSet) > 0

  local rigIsValid = false
  if setIsValid then
    local scene = nmx.Application.new():getSceneByName("AssetManager")
    rigIsValid = anim.isRigValid(selectedSet) and (anim.getRigDataRoot(scene, selectedSet) ~= nil)
  end

  mirrorJointsListControl:clearRows()
  mirrorEventsListControl:clearRows()
  mirrorEventTracksListControl:clearRows()
  if setIsValid then
    if rigIsValid then
      local mirrorPlane = anim.getAnimSetJointMirrorPlane(selectedSet)
      -- if there is no rig loaded then mirrorPlane will be null
      if mirrorPlane then
        mirrorPlaneCombo:setSelectedItem(mirrorPlane)
      end

      local mirroredJoints = anim.listAnimSetJointMirrorMappings(selectedSet)
      -- if there is no rig loaded then mirrorPlane will be null
      if mirroredJoints then
        for _, mirroring in ipairs(mirroredJoints) do
          mirrorJointsListControl:addRow{ mirroring.first, mirroring.second }
        end
      end
    end

    local mirroredEvents = anim.listAnimSetEventUserdataMirrorMappings(selectedSet)
    for _, mirroring in ipairs(mirroredEvents) do
      mirrorEventsListControl:addRow{ tostring(mirroring.first), tostring(mirroring.second) }
    end

    local mirroredEvents = anim.listAnimSetEventTrackMirrorMappings(selectedSet)
    for _, mirroring in ipairs(mirroredEvents) do
      mirrorEventTracksListControl:addRow{ tostring(mirroring.first), tostring(mirroring.second) }
    end
  end

  for _, child in ipairs(mirrorSettingsPanel:getChildren()) do
    child:enable(setIsValid)
  end
  mirrorPlaneCombo:enable(setIsValid and rigIsValid)
  mirrorJointsAddButton:enable(setIsValid and rigIsValid)
  mirrorJointsEditButton:enable(false)
  mirrorJointsAutoButton:enable(setIsValid and rigIsValid)
  mirrorJointsRemoveButton:enable(false)

  for _, child in ipairs(mirrorEventsPanel:getChildren()) do
    child:enable(setIsValid)
  end
  for _, child in ipairs(mirrorEventTracksPanel:getChildren()) do
    child:enable(setIsValid)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- update all the parts of the animation sets dialog
------------------------------------------------------------------------------------------------------------------------
local updateAnimationSetsAttributes = function(panel, assetManager)
  panel:freeze()

  local selectedSet = animationSetsTreeControl:getSelectedItem()
  local setIsValid = type(selectedSet) == "string" and string.len(selectedSet) > 0
  mainScrollPanel:setShown(setIsValid)
  if setIsValid then
    updateToolbarButtons()
    updateRigSettingsRollup(panel)
    updateFormatOptionsRollup(panel)
    updateMirrorSettingsRollup(panel)
    updateSkinsRollup(panel, animationSetsTreeControl)
    --updateChannelsRollup(panel)
    safefunc(updateExtraAnimationSetAttributeEditorRollup, panel, assetManager)
  end

  panel:rebuild()
end

------------------------------------------------------------------------------------------------------------------------
-- validate an animation set
------------------------------------------------------------------------------------------------------------------------
local validateAnimationSet = function(selectedSet)
  local selectedTypeName = anim.getAnimSetCharacterType(selectedSet)
  local supportsAnimationRig = characterTypes.supportsRig(selectedTypeName, "AnimationRig")
  local supportsPhysicsRig = characterTypes.supportsRig(selectedTypeName, "PhysicsRig")

  -- Check animation rig
  if supportsAnimationRig then
    local animationRigPath = anim.getRigPath(selectedSet)
    if string.len(animationRigPath) > 0 then
      local fullPath = utils.demacroizeString(animationRigPath)
      if not app.fileExists(fullPath) then
        return "Error" -- file not found
      end
    else
      return "Error" -- no value set
    end
  end

  -- Check physics rig
  if not mcn.isPhysicsDisabled() and supportsPhysicsRig then
    local physicsRigPath = anim.getPhysicsRigPath(selectedSet)
    if string.len(physicsRigPath) > 0 then
      local fullPath = utils.demacroizeString(physicsRigPath)
      if not app.fileExists(fullPath) then
        return "Error" -- file not found
      end
    else
      return "Error" -- no value set
    end
  end

  return "Ok"
end

------------------------------------------------------------------------------------------------------------------------
-- validate all of the animation sets
------------------------------------------------------------------------------------------------------------------------
local validateAnimationSets = function()
  if animationSetsTreeControl then
    local summaryValidity = 0
    local animSets = listAnimSets()
    for _, animSetName in animSets do
      local valid = validateAnimationSet(animSetName)
      local treeItem = animationSetsTreeControl:findItem(animSetName)
      local validation = statusLookup[validateAnimationSet(animSetName)]
      if validation > summaryValidity then
        summaryValidity = validation
      end

      if treeItem then
        treeItem:setImageIndex(validation)
      end
    end
    
    if summaryValidity == 0 then  
      animationSetsRollupHeading:setIcon(nil)
    else
      animationSetsRollupHeading:setIcon(statusIcons[summaryValidity])
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Install the Attribute editor
------------------------------------------------------------------------------------------------------------------------
addAnimationSetAttributeEditor = function(contextualPanel, theContext, assetManager)
  displayedCompressionFormat = nil -- no format is displayed
  selectedSkin = nil
  local panel = contextualPanel:addPanel{ name = "AnimationSetAttributeEditor", forContext = theContext }

  animationSetsTreeControl = assetManager:getAnimationSetsTreeControl()
  animationSetsTreeControl:setImageList(statusIcons, statusIcons[1]:getWidth(), statusIcons[1]:getHeight(), true)
  animationSetsRollupHeading = assetManager:getAnimationSetsRollupHeading()

  animationSetsTreeControl:setOnSelectionChanged(
    function(self)
      updateAnimationSetsAttributes(panel, assetManager)
      return false
    end)
    
  animationSetsTreeControl:setOnRebuild(
    function(self)
      validateAnimationSets()
      updateAnimationSetsAttributes(panel, assetManager)
    end)

  panel:beginVSizer{ flags = "expand", proportion = 1 }

    addToolbarAndButtons()
    updateToolbarButtons()

    mainScrollPanel = panel:addScrollPanel{
      name = "ScrollPanel",
      flags = "expand;vertical",
      proportion = 8
    }

    mainScrollPanel:beginVSizer{ flags = "expand" }

      addRigSettingsRollup(mainScrollPanel)
      addSkinsRollup(mainScrollPanel, animationSetsTreeControl)
      --addComputedChannelsRollup(mainScrollPanel)
      addFormatOptionsRollup(mainScrollPanel)
      addMirrorSettingsRollup(mainScrollPanel)
      safefunc(addExtraAnimationSetAttributeEditorRollup, mainScrollPanel, assetManager)

    mainScrollPanel:endSizer()

    -- this panel contains the help text box
    helpPanel = panel:addPanel{
      name = "HelpPanel",
      flags = "expand",
      proportion = 2
    }
    helpPanel:beginVSizer{ flags = "expand", proportion = 1 }
      helpText = helpPanel:addTextControl{ name = "TextBox", flags = "expand;noscroll", proportion = 1 }
      helpText:setReadOnly(true)
    helpPanel:endSizer()
    
  panel:endSizer()

  setupToolbarCallbacks(panel)
  setupRigSettingsCallbacks(panel)
  setupFormatOptionsCallbacks(panel)
  setupMirrorSettingsCallbacks(panel)
  safefunc(setupExtraAnimationSetAttributeEditorCallbacks, panel, assetManager)

  animationSetsAttributeEditor.onAnimationSetModified = function(set)
    local panel = contextualPanel:getChild("AnimationSetAttributeEditor")
    if panel ~= nil then
      -- the onAnimationSetModified event passes the full path to the set ("AnimationSets|DefaultSet")
      -- and we only use the name so split the path first
      local _, setName = splitNodePath(set)
      -- get the currently selected set
      if setName == animationSetsTreeControl:getSelectedItem() then
        updateAnimationSetsAttributes(panel, assetManager)
      end
    end
    validateAnimationSets()
  end

  animationSetsAttributeEditor.onRigModified = function()
    local panel = contextualPanel:getChild("AnimationSetAttributeEditor")
    if panel ~= nil then
      updateAnimationSetsAttributes(panel, assetManager)
    end
    validateAnimationSets()
  end
  
  animationSetsAttributeEditor.forceSelectSet = function(setToSelect)
   local selectedSet = animationSetsTreeControl:getSelectedItem()
   animationSetsTreeControl:setSelectedItem(setToSelect)
  end
  
  animationSetsAttributeEditor.forceAttributeUpdate = function()
    local panel = contextualPanel:getChild("AnimationSetAttributeEditor")
    if panel ~= nil then
      updateAnimationSetsAttributes(panel, assetManager)
    end
  end
  
  updateAnimationSetsAttributes(panel, assetManager)
  validateAnimationSets()
  local animationSetsRollupHeading = assetManager:getAnimationSetsRollupHeading()
end
