------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/Components/RetargetingMappingSection.lua"

------------------------------------------------------------------------------------------------------------------------
-- util for adding all joints in a rig to a table.
local addChildJointsToList
addChildJointsToList = function (parent, jointList)
  local child = parent:getFirstChild()
  while child ~= nil do
    if child:is(nmx.sgTransformNode.ClassTypeId()) then
      local txChild = child:getFirstChild()
      while txChild ~= nil do
        if txChild:is(nmx.sgShapeNode.ClassTypeId()) then
          local dataNode = txChild:getDataNode()
          if dataNode ~= nil and dataNode:getTypeId() == nmx.JointNode.ClassTypeId() then
            table.insert(jointList, txChild)
            addChildJointsToList(child, jointList)
            break -- from txChild while loop
          end
        end
        txChild = txChild:getNextSibling()
      end
    end
    child = child:getNextSibling()
  end
end

------------------------------------------------------------------------------------------------------------------------
local getRetargetingStatus = function(selectedSet)
  if not anim.isRigValid(selectedSet) then
    return "NoTemplate"
  end

  if anim.getAnimSetTemplate(selectedSet) == "" then
    return "NoTemplate"
  end

  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local rootNode = anim.getRigSceneRoot(scene, selectedSet)
  local bodyMapping = nmx.BodyMappingNode.getBodyMappingNode(rootNode)
  local bodyMappingInfo = nmx.BodyMappingInfoNode.getBodyMappingInfoNode(rootNode)
  
  if bodyMappingInfo ~= nil then 
    if bodyMapping == nil or not bodyMapping:isComplete(bodyMappingInfo) then
      return "BadMapping"
    end
  end
  
  if not anim.hasOffsetFrames(selectedSet) then
    return "Calculate"
  end

  local srcs = anim.getPossibleRetargetingSources(selectedSet)
  if table.getn(srcs) == 0 then
    return "NoSources"
  end

  local retargetSource = anim.getActiveRetargetingSource(selectedSet)
  if retargetSource == "" then
    return "Disable Preview"
  end

  return "Active"
end

------------------------------------------------------------------------------------------------------------------------
-- Prefix for anim sets in the retargeting combo
local animSetPrefix = "from "

------------------------------------------------------------------------------------------------------------------------
local updateRetargetingComboBox = function(selectedSet)
  if selectedSet == nil or selectedSet == "" then
    selectedSet = selectedSet or getSelectedAssetManagerAnimSet()
  end

  local treeItem = componentEditor.getComponentListItem("Retargeting")
  if treeItem == nil then
    return
  end

  local panel = treeItem:getColumnPanel(1)
  if panel == nil then
    return
  end

  -- populate the combo box and select the correct item.
  local comboBox = panel:getChild("RetargetingCombo")
  if comboBox then
    local setNames = anim.getPossibleRetargetingSources(selectedSet)
    local numSources = table.getn(setNames)

    -- Append "from " to all possibly retargeting sources
    for i = 1, numSources do
      setNames[i] = animSetPrefix .. setNames[i]
    end
    -- Insert "Disable Preview" as the first item
    table.insert(setNames, 1, "Disable Preview")

    comboBox:setItems(setNames)
    -- Add the help items to the end of the list.
    if numSources == 0 then
      local state = getRetargetingStatus(selectedSet)
      if state == "NoTemplate" or state == "BadMapping" then
        comboBox:addItem("Templates Must Be Configured")
        comboBox:setEntryType(comboBox:countItems() - 1, 2)
      elseif state == "Calculate" then
        comboBox:addItem("Offsets Must Be Calculated")
        comboBox:setEntryType(comboBox:countItems() - 1, 2)
      elseif state == "NoSources" then
        comboBox:addItem("No Sources Containing Offsets")
        comboBox:setEntryType(comboBox:countItems() - 1, 2)
      end
    end

    -- select the correct retargeting source
    local activeSource = anim.getActiveRetargetingSource(selectedSet)
    if activeSource ~= "" then
      comboBox:setSelectedItem(animSetPrefix .. activeSource )
      local retargetColour =  colours.getActiveDisplayItemColour("Viewports", "Retarget Source")
      comboBox:setBackgroundColour(retargetColour)
    else
      comboBox:setSelectedItem("Disable Preview")
      comboBox:setBackgroundColour("controlBackground")
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local retargetingStateToValidity = function(selectedSet, state)
  local hasOffsets = anim.isRigValid(selectedSet) and anim.hasOffsetFrames(selectedSet)
  if (state == "NoTemplate" or state == "BadMapping") and hasOffsets then
    return "Warning"
  end

  if not hasOffsets then
    return "Inactive"
  end

  return "Ok"
end

------------------------------------------------------------------------------------------------------------------------
local validateFunction = function(selectedSet, isCurrentComponent)
  updateRetargetingComboBox(selectedSet)

  local state = getRetargetingStatus(selectedSet)
  return retargetingStateToValidity(selectedSet, state)
end

------------------------------------------------------------------------------------------------------------------------
-- image array used by calculate offsets dialog
local helpImageCache = { }

------------------------------------------------------------------------------------------------------------------------
local showSuccessPage = function (dlg)
  dlg:clear()
  dlg:beginVSizer{ flags = "expand" }
    dlg:beginHSizer{ flags = "expand" }
      dlg:addHSpacer(10)
      dlg:beginVSizer{ flags = "expand" }
        dlg:addVSpacer(10)
        dlg:addStaticText{ text = "Offsets have been setup for this rig." }
        dlg:addVSpacer(5)
        dlg:addStaticText{
          text = [[
Next Step:
  1.  Calculate offsets for a different animation set.
  2.  Select that animation set from the retargeting
       drop-down menu.]],
        }

        dlg:addVSpacer(15)

        local imagePath = utils.demacroizeString([[$(AppRoot)resources\images\ui\retargetingHelp.png]])
        if not helpImageCache[imagePath] then
          helpImageCache[imagePath] = ui.createImage(imagePath)
        end
        local helpImage = helpImageCache[imagePath]
        if helpImage then
          dlg:addStaticBitmap{ image = helpImage, flags = "center" }
        end

      dlg:endSizer()
      dlg:addHSpacer(10)
    dlg:endSizer()
    dlg:addVSpacer(15)

    dlg:addButton{
      label = "Ok",
      flags = "right",
      size = { width = 65 },
      onClick = function (self)
        dlg:hide()
      end,
      proportion = 0
    }
  dlg:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
local showCalculatePage = function (dlg, selectedSet)
  local errorTextCtrl

  local help = anim.getAnimSetRetargetingHelp(selectedSet)
  dlg:clear()
  dlg:beginVSizer{ flags = "expand" }
    dlg:addVSpacer(5)
    if anim.hasOffsetFrames(selectedSet) then
      dlg:addStaticText{ text = "Warning: This will replace the existing retargeting offsets.", font = "bold" }

      dlg:addVSpacer(10)
    end

    if help and anim.getAnimSetTemplate(selectedSet) ~= "" then
      local helpText
      if help.text ~= nil and help.text ~= "" then
        helpText = help.text
      end
      local helpImage
      if help.image ~= nil and help.image ~= "" then
        helpImage = help.image
      end

      if helpText ~= nil then
        dlg:addTextControl{
          value = help.text,
          size = { width = 256, height = 70 },
          flags = "readonly;expand;dialogColours", proportion = 1
        }
      end

      if helpImage ~= nil then
        local imagePath = utils.demacroizeString(help.image)

        if not helpImageCache[imagePath] then
          helpImageCache[imagePath] = ui.createImage(imagePath)
        end
        local helpImage = helpImageCache[imagePath]
        if helpImage then
          dlg:beginHSizer{ flags = "center" }
            dlg:addStaticBitmap{ image = helpImage, flags = "center" }
          dlg:endSizer()
        end
      end
    end

    dlg:beginHSizer{ flags = "right" }
      dlg:addButton{
        label = "Calculate",
        flags = "right",
        onClick = function (self)
          local app = nmx.Application.new()
          local scene = app:getSceneByName("AssetManager")
          local result = app:runCommand("Retargeting", "Calculate Retarget Offsets", scene)

          if result:asString() == "kSuccess" and anim.hasOffsetFrames(selectedSet) then
            componentEditor.updateComponentValidity("Retargeting")
            componentEditor.reset()
            showSuccessPage(dlg)
          else
            app:logError( "Retargeting offsets failed to be calculated" )
            if errorTextCtrl ~= nil then
              errorTextCtrl:setLabel( "Retargeting offsets failed to be calculated" )
              dlg:rebuild()
            end
          end
        end,
        proportion = 2,
      }
      dlg:addButton{
        label = "Cancel",
        flags = "right",
        onClick = function (self) dlg:hide() end,
        proportion = 2
      }
    dlg:endSizer()
    errorTextCtrl = dlg:addStaticText()
  dlg:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
local showCalculateDialog = function(selectedSet)
  local dlg = ui.getWindow("RetargetingOffsets")

  if not dlg then
    dlg = ui.createModelessDialog{
      name = "RetargetingOffsets",
      caption = "Retargeting Offsets",
      size = { width = 400, height = 600 },
      centre = true,
      flags = "dialogColours"
    }
  end

  showCalculatePage(dlg, selectedSet)

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
local addRecalculateRollup = function (panel, selectedSet, expand)
  local rollup = panel:addRollup{ label = "Recomputing offsets", flags = "expand" }
  local rollupPanel = rollup:getPanel()
  rollupPanel:beginVSizer{ flags = "expand" }
    rollupPanel:addStaticText{
      text = [[
If you didn't position your character correctly
before computing, then you may wish to Recalculate.]],
      font = "italic"
    }
    rollupPanel:addVSpacer(5)
    rollupPanel:beginHSizer{ flags = "right" }
      rollupPanel:addButton{
        label = "Recalculate",
        onClick = function(self)
          -- disable retargeting, recomputing from retargeting is a bad idea,
          -- if the user really wants it on, they can simply turn it back at the next stage.
          anim.setActiveRetargetingSource(selectedSet, "")
          showCalculateDialog(selectedSet)
        end
      }
    rollupPanel:endSizer()

 local deleteOffsetFrames = function (animSet)
    -- find the first offset frame transform instance and delete it.
    local scene = nmx.Application.new():getSceneByName("AssetManager")
    local rigRoot = anim.getRigDataRoot(scene, animSet)

    if rigRoot ~= nil then
      local it = nmx.NodeIterator.new(rigRoot, nmx.OffsetFrameNode.ClassTypeId())
      if it:next() then
        local kinectRoot = it:node():getFirstInstance():getParentSgTransform()

        if kinectRoot ~= nil then
          local sl = nmx.SelectionList.new()
          sl:add(kinectRoot)
          nmx.Application.new():runCommand("Core", "Generic Delete", scene, sl)
        end
      end
    end

    -- turn retargeting off when offsets are cleared.
    anim.setActiveRetargetingSource(animSet, "")
  end
  
    rollupPanel:addStaticText{
      text = [[
If recomputing offsets has not resolved the 
issue you may wish to delete the offset frames
and start over]],
      font = "italic"
    }
    rollupPanel:addVSpacer(5)
    rollupPanel:beginHSizer{ flags = "right" }
      rollupPanel:addButton{
        label = "Delete",
        onClick = function(self)
          deleteOffsetFrames(selectedSet)
        end
      }
    rollupPanel:endSizer()
    
  rollupPanel:endSizer()
  rollup:expand(expand)
end

------------------------------------------------------------------------------------------------------------------------
local updateItem = function (item, getFunct, offsetTx)
  if type(getFunct) == "function" then
    local trans = getFunct(offsetTx)
    item:setColumnValue(2, string.format("%.3f", trans:getX()))
    item:setColumnValue(3, string.format("%.3f", trans:getY()))
    item:setColumnValue(4, string.format("%.3f", trans:getZ()))
  end
end

------------------------------------------------------------------------------------------------------------------------
local mkChangeFunction = function(treeRoot, getFunct, itemOffsetMap)
  return function()
    local item = treeRoot:getChild(1)
    while item ~= nil do
      updateItem(item, getFunct, itemOffsetMap[item:getUserDataInt()])
      item = item:getNextSibling()
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- addManualOffsetEditorRollup{ panel, label, joints, getFunct, setFunct }
--
-- Adds a rollup containing a single table of vectors for each item in `joints`.
--
-- The table is populated using the `getFunct`, in the form `function (offsetTx)` where `offsetTx` is the
-- joints offset transform. The function should return a nmx.Vector3.
--
-- When an item is changed, the `setFunct` is called. This is in the form `function (offsetTx, vect)`. `vect` is
-- the new value on that row.
------------------------------------------------------------------------------------------------------------------------
local addManualOffsetEditorRollup = function (args)
  local rollup = args.panel:addRollup{ label = args.label, flags = "expand" }
  local rollupPanel = rollup:getPanel()
    rollupPanel:beginVSizer{ flags = "expand" }
      local itemOffsetMap = {}

      local editItem = function (item, column, value)
        if type(value) == "number" and type(args.setFunct) == "function" then
          local offsetTx = itemOffsetMap[ item:getUserDataInt() ]
          if offsetTx ~= nil then
            local vect
            if column == 2 then
              vect = nmx.Vector3.new(
                value,
                tonumber(item:getColumnValue(3)),
                tonumber(item:getColumnValue(4)))
            elseif column == 3 then
              vect = nmx.Vector3.new(
                tonumber(item:getColumnValue(2)),
                value,
                tonumber(item:getColumnValue(4)))
            elseif column == 4 then
              vect = nmx.Vector3.new(
                tonumber(item:getColumnValue(2)),
                tonumber(item:getColumnValue(3)),
                value)
            end

            args.setFunct(offsetTx, vect)
          end
        end
      end

      local tree = rollupPanel:addTreeControl{
        flags = "expand;sizeToContent;hideroot;hideExpansionBoxes",
        onShowInPlaceEditor = function (self, item, column)
          if column > 1 then
            self:popupEditor(item, column)
          end
        end,
        onEditColumn = function (self, item, column, value)
          local valNum = tonumber(value)
          if valNum ~= nil then
            editItem(item, column, valNum)
          end
        end,
        columnNames = { "Joint", "x", "y", "z" }
      }

      local treeRoot = tree:getRoot()
      for i, v in ipairs(args.joints) do
        local offsetTx = nmx.OffsetFrameTransformNode.getOffsetFrameTransform(v)
        local item = treeRoot:addChild(v:getName())
        itemOffsetMap[i] = offsetTx
        item:setUserDataInt(i)

        updateItem(item, args.getFunct, offsetTx)
      end

      componentEditor.setChangeFunction(args.label .. "_changeFunct",
        mkChangeFunction(treeRoot, args.getFunct, itemOffsetMap),
        {"mcRetargetingOffsetChange"}
      )
    rollupPanel:endSizer()
  rollup:expand(false)
end

------------------------------------------------------------------------------------------------------------------------
local addAdvancedRollup = function(panel, selectedSet, expand)
  local scene = nmx.Application.new():getSceneByName("AssetManager")

  local advancedRollup = panel:addRollup{ label = "Advanced", flags = "mainsection;expand", proportion = 1 }
  advancedPanel = advancedRollup:getPanel()
  advancedPanel:beginVSizer{ flags = "expand" }

    -------------------------------------------------------------------------
    -- Character scale slider
    --
    local rootNode = anim.getRigDataRoot(scene, selectedSet)
    local rigInfo = rootNode:getFirstChild(nmx.RigInfoNode.ClassTypeId())
    if rigInfo ~= nil then
      local retargetScale = rigInfo:findAttribute("RigRetargetScaleFactor")
      if retargetScale ~= nil then
        advancedPanel:beginHSizer{ flags = "expand" }
          advancedPanel:addStaticText{ text = "Character Scale" }
          advancedPanel:addAttributeWidget{
            attributes = { retargetScale },
            flags = "expand",
            min = 0.001,
            max = 10.0,
            proportion = 1
          }
        advancedPanel:endSizer()
      end
    end

    -------------------------------------------------------------------------
    -- Mapping section
    --
    local sourceSet = anim.getActiveRetargetingSource(selectedSet)
    local mappingRollup = advancedPanel:addRollup{ label = "Retarget Mapping", flags = "expand", proportion = 1 }
    mappingPanel = mappingRollup:getPanel()
      addRetargetMappingEditingUI(mappingPanel, selectedSet, sourceSet)

    -------------------------------------------------------------------------
    -- Manual editing sections
    --
    local kinematicJoints = { }
    if anim.isRigValid(selectedSet) then
      addChildJointsToList(anim.getRigSceneRoot(scene, selectedSet), kinematicJoints)
    end

    advancedPanel:beginVSizer{ flags = "expand" }

      -- Position Offsets table
      addManualOffsetEditorRollup{
        panel = advancedPanel,
        label = "Position Offsets",
        joints = kinematicJoints,
        getFunct = function (offsetTx)
          return offsetTx:getOffsetLocalTranslation()
        end,
        setFunct = function (offsetTx, vect)
          local scene = nmx.Application.new():getSceneByName("AssetManager")
          local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

          offsetTx:setOffsetLocalTranslation(vect)

          scene:endChangeBlock(cbRef, changeBlockInfo("Setting Offset Translation"))
        end
      }

      -- Rotation Offsets table
      addManualOffsetEditorRollup{
        panel = advancedPanel,
        label = "Rotation Offsets",
        joints = kinematicJoints,
        getFunct = function (offsetTx)
          local rot = offsetTx:getOffsetLocalOrientation()
          local vect = nmx.Vector3.new()
          rot:toEuler(vect, 0)
          -- Convert the vector of radians to degrees by mult pi/180
          vect:set(vect:getX()*0.01745329251 , vect:getY()*0.01745329251, vect:getZ()*0.01745329251)
          return vect
        end,
        setFunct =function (offsetTx, vect)
          local scene = nmx.Application.new():getSceneByName("AssetManager")
          local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

          -- Convert the vector of degrees to radians by mult 180/pi
          vect:set(vect:getX()*57.29577951, vect:getY()*57.29577951, vect:getZ()*57.29577951)

          local quat = nmx.Quat.new()
          quat:fromEuler(vect, 0)
          offsetTx:setOffsetLocalOrientation(quat)

          scene:endChangeBlock(cbRef, changeBlockInfo("Setting Offset Translation"))
        end
      }

      -- Post Rotation Offsets table
      addManualOffsetEditorRollup{
        panel = advancedPanel,
        label = "Post Rotation Offsets",
        joints = kinematicJoints,
        getFunct = function (offsetTx)
          local rot = offsetTx:getPostOrientOffset()
          local vect = nmx.Vector3.new()
          rot:toEuler(vect, 0)
          -- Convert the vector of radians to degrees by mult pi/180
          vect:set(vect:getX()*0.01745329251 , vect:getY()*0.01745329251, vect:getZ()*0.01745329251)
          return vect
        end,
        setFunct = function (offsetTx, vect)
          local scene = nmx.Application.new():getSceneByName("AssetManager")
          local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

          -- Convert the vector of degrees to radians by mult 180/pi
          vect:set(vect:getX()*57.29577951, vect:getY()*57.29577951, vect:getZ()*57.29577951)

          local quat = nmx.Quat.new()
          quat:fromEuler(vect, 0)
          offsetTx:setPostOrientOffset(quat)

          scene:endChangeBlock(cbRef, changeBlockInfo("Setting Offset Translation"))
        end
      }
    advancedPanel:endSizer()

  advancedPanel:endSizer()
  advancedRollup:expand(expand)
end

------------------------------------------------------------------------------------------------------------------------
local panelFunction = function(selectedSet, panel)
  local state = getRetargetingStatus(selectedSet)

  ----------------------------------------------------------------------------------------------------------------------
  local addTextPage = function(panel, title, text)
    panel:addStaticText{ text = title, font = "bold" }
    panel:addTextControl{
      value = text,
      size = { width = 275 },
      flags = "noscroll;readonly;dialogColours"
    }
  end

  ----------------------------------------------------------------------------------------------------------------------
  panel:beginVSizer{ flags = "expand" }
    if state == "NoTemplate" then
      addTextPage(
        panel,
        "Missing animation set template",
        "The animation set must have a template select to retarget."
      )

    elseif state == "BadMapping" then
      addTextPage(
        panel,
        "Animation mappings not configured",
        "The animation set must have a template with animation mappings to do retargeting."
      )

    elseif state == "Calculate" or state == "Recalculate" then
      panel:addTextControl{
        value = [[
To start using retargeting you must first calculate
transforms to the shared pose used by all the rigs that
you want to be able to retarget between.]],
        size = { width = 275, height = 50 },
        flags = "noscroll;readonly;dialogColours",  proportion = 0
      }

      panel:beginHSizer{ flags = "right" }
        panel:addButton{
          label = "Calculate",
          onClick = function()
            showCalculateDialog(selectedSet)
          end
        }
      panel:endSizer()

    elseif state == "NoSources" then
      panel:addStaticText{
        text = [[
This character is ready for retargeting but there
are no valid data sources.]],
      }
      panel:addVSpacer(10)

      panel:addStaticText{ text = "Next Step:", font = "largebold" }
      panel:addStaticText{
        text = [[
  1.  Calculate offsets for a different animation set.
  2.  Select that animation set from the retargeting
       drop-down menu.]],
      }

      panel:addVSpacer(10)

      addRecalculateRollup(panel, selectedSet, false)
      addAdvancedRollup(panel, selectedSet, false)

    elseif state == "Disable Preview" then
      -- Add the "select source" page
      panel:addStaticText{ text = "Ready to Retarget.", font = "largebold" }
      panel:addStaticText{
        text = [[
Select another animation set from the retargeting
drop-down menu.]],
      }

      panel:addVSpacer(10)

      addRecalculateRollup(panel, selectedSet, false)
      addAdvancedRollup(panel, selectedSet, false)

    elseif state == "Active" or state == "Disable Preview" then
      -- Add the settings page
      addRecalculateRollup(panel, selectedSet, true)
      addAdvancedRollup(panel, selectedSet, false)

    else
      addTextPage(panel, "!Error: Unknown retargeting state:", state)
    end
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
local headerFunction = function(selectedSet, panel)
  local validity = retargetingStateToValidity(selectedSet, state)
  local statusIcon = componentEditor.getStatusIcon(validity)
  panel:beginHSizer{ flags = "expand" }
    if statusIcon ~= nil then
      panel:addStaticBitmap{ image = statusIcon }
    end

    panel:addStaticText{ text = "Retargeting", font = "bold", flags = "expand" }

    local statusText = "No Offsets Calculated."
    if anim.isRigValid(selectedSet) and anim.hasOffsetFrames(selectedSet) then
      statusText = "Offsets Calculated."
    end
    panel:beginVSizer{ proportion = 1 }
      panel:addVSpacer(2)
      panel:addStaticText{ text = statusText, font = "italic", flags = "right" }
    panel:endSizer()
  panel:endSizer()
  panel:addVSpacer(5)
end

------------------------------------------------------------------------------------------------------------------------
local componentFunction = function(selectedSet, treeItem)
  -- Add a combo box to the second column section

  treeItem:setColumnToPanel(1)
  local panel = treeItem:getColumnPanel(1)

  panel:clear()

  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:setBorder(0)
    panel:addComboBox{
      name = "RetargetingCombo",
      flags = "expand;minimalBestSize;customBackground",
      proportion = 1,
      onChanged = function(self)
        local setName = self:getValue()
        if comboVal ~= "Disable Preview" then
          -- trim off "from "
          setName = string.sub(setName, string.len(animSetPrefix) + 1)
        end

        anim.setActiveRetargetingSource(selectedSet, setName)
        componentEditor.reset()

        updateRetargetingComboBox(selectedSet)
      end
    }
  panel:endSizer()

  updateRetargetingComboBox(selectedSet)

  componentEditor.setChangeFunction("rebuildRetargetingCombo",
    function ()
      updateRetargetingComboBox(getSelectedAssetManagerAnimSet())
    end,
    {
      "mcAnimationSetSelectionChange",
      "mcAnimationSetsChange"
    }
  )
end

components.register(
  "Retargeting",
  { "Animation", "Physics", "Euphoria" },
  validateFunction,
  panelFunction,
  headerFunction,
  componentFunction
)

------------------------------------------------------------------------------------------------------------------------
local onClearOffsetFrames = function(changedOffsets)
  local selectedSet = getSelectedAssetManagerAnimSet
  if anim.hasOffsetFrames(selectedSet) == false then
    componentEditor.reset()
    componentEditor.updateComponentValidity("Retargeting")
  end
end
registerEventHandler("mcRetargetingOffsetChange", onClearOffsetFrames)

------------------------------------------------------------------------------------------------------------------------
local onOffsetsCreatedOrDeleted = function(changedOffsets)
  componentEditor.reset()
  componentEditor.updateComponentValidity("Retargeting")
end
registerEventHandler("mcRetargetingOffsetsCreated", onOffsetsCreatedOrDeleted)
registerEventHandler("mcRetargetingOffsetsDeleted", onOffsetsCreatedOrDeleted)

------------------------------------------------------------------------------------------------------------------------
local broadcastRetargetOffsetChange = function(changedOffsets)
  local selectedSet =  getSelectedAssetManagerAnimSet()
  local retargetSource = anim.getActiveRetargetingSource(selectedSet)
  if retargetSource == "" then
    return 
  end
  
  anim.broadcastAssetManagerMessage(102,
    function(stream)
      local writeOffsetFrame = function(offsetTx)
        local rotate = offsetTx:getOffsetLocalOrientation()
        local translate = offsetTx:getOffsetLocalTranslation()
        local postRotation = offsetTx:getPostOrientOffset()

        local offsetInput = offsetTx:findAttribute("SiblingBindPoseLocalMatrix"):getInput()
        local offsetInputArray = offsetInput:getAttributeArray()
        local index = offsetInput:getId()

        if index < offsetInputArray:size() then
          stream:writeFloat(rotate:getX(), "offsetRotationX") -- 4
          stream:writeFloat(rotate:getY(), "offsetRotationY") -- 8
          stream:writeFloat(rotate:getZ(), "offsetRotationZ") -- 12
          stream:writeFloat(rotate:getW(), "offsetRotationW") -- 16
          stream:writeFloat(translate:getX(), "offsetTranslationX")-- 20
          stream:writeFloat(translate:getY(), "offsetTranslationY")-- 24
          stream:writeFloat(translate:getZ(), "offsetTranslationZ")-- 28
          stream:writeFloat(postRotation:getX(), "referenceRotationX")-- 32
          stream:writeFloat(postRotation:getY(), "referenceRotationY")-- 36
          stream:writeFloat(postRotation:getZ(), "referenceRotationZ")-- 40
          stream:writeFloat(postRotation:getW(), "referenceRotationW")-- 44

          stream:writeInt(index, "jointIndex") -- 48
        end
      end

      for _,v in pairs(changedOffsets) do
        writeOffsetFrame(v)
      end
    end
  )
end
registerEventHandler("mcRetargetingOffsetChange", broadcastRetargetOffsetChange)


------------------------------------------------------------------------------------------------------------------------
local broadcastRetargetScaleChange = function()
  local selectedSet =  getSelectedAssetManagerAnimSet()
  local retargetSource = anim.getActiveRetargetingSource(selectedSet)
  if retargetSource == "" then
    return 
  end
  
  anim.broadcastAssetManagerMessage(103,
    function(stream)
      local scene = nmx.Application.new():getSceneByName("AssetManager")
      local dataRoot = anim.getRigDataRoot(scene, getSelectedAssetManagerAnimSet())
      if dataRoot then
        local rigInfo = dataRoot:getFirstChild(nmx.RigInfoNode.ClassTypeId())
        if rigInfo then
          stream:writeFloat(rigInfo:getRigRetargetScaleFactor(), "scale")
        else
          nmx.Application.new():logError("No rig info found for set '" .. getSelectedAssetManagerAnimSet() .. "'")
        end
      else
        nmx.Application.new():logError("No animation rig found for set '" .. getSelectedAssetManagerAnimSet() .. "'")
      end
    end
  )
end
registerEventHandler("mcCharacterRetargetScaleChanged", broadcastRetargetScaleChange)
