------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/WizardAPI.lua"

------------------------------------------------------------------------------------------------------------------------
-- add a default scale preset for the given skin and set
------------------------------------------------------------------------------------------------------------------------
local scaleMessageType = "ScaleRig"
local scaleFactorsAttribute = "JointScale"

local addDefaultScalePreset = function(selectedSet, selectedName)
  local needsScale, scaleFactors = anim.generateScaleFactorsFromRigToSkin(selectedName, selectedSet)
  
  -- in general there is not a valid scale for the trajectory, so replace its scale factor with the hips
  local markup = anim.getRigMarkupData(selectedSet)
  if markup.hipIndex < table.getn(scaleFactors) and markup.trajectoryIndex < table.getn(scaleFactors) then
    scaleFactors[markup.trajectoryIndex+1] = scaleFactors[markup.hipIndex+1] 
  end
  
  local newPreset = create(scaleMessageType, "MessagePresets", "ScaleTo" .. selectedName)
  if not newPreset then
    app.error("Could not create a new message preset '" .. scaleMessageType .. "'")
  end
  
  local sfAttr = string.format("%s.%s", newPreset, scaleFactorsAttribute)
  local animSetAttr = string.format("%s.%s", newPreset, "AnimationSetHint")
  local advancedScaleAttr = string.format("%s.%s", newPreset, "AdvancedScale")
  setAttribute(sfAttr, scaleFactors)
  setAttribute(animSetAttr, selectedSet)
  setAttribute(advancedScaleAttr, true)
  
  return newPreset
end

local addIdentityScalePreset = function(selectedSet, selectedName)  
  local newPreset = create(scaleMessageType, "MessagePresets", "ScaleToOne")
  if not newPreset then
    app.error("Could not create a new message preset '" .. scaleMessageType .. "'")
  end
  
  local advScale = string.format("%s.%s", newPreset, "AdvancedScale")
  local speScale = string.format("%s.%s", newPreset, "SpeedScale")
  local ovScale = string.format("%s.%s", newPreset, "OverallScale")
  
  setAttribute(advScale, false)
  setAttribute(speScale, 1.0)
  setAttribute(ovScale, 1.0)
  
  return newPreset
end

local addMessage = function(name)  
  local newMessage = create(scaleMessageType, name)
  if not newMessage then
    app.error("Could not create a new message '" .. scaleMessageType .. "'")
  end
  
  return newMessage
end

local setMessageForSkin = function(selectedSet, selectedName, messageName)
  local skin = anim.getRigSkin(selectedName, selectedSet)
  
  if type(skin) ~= "table" then
    return
  end
  
  skin.RescaleInPreview = true
  skin.PreviewMessage = messageName
  
  anim.setSkinData(skin, selectedName, selectedSet)
end

local setScenePresetForSkin = function(selectedSet, selectedName, scalePreset, scene)  
  local skin = anim.getRigSkin(selectedName, selectedSet)
  
  if type(skin) ~= "table" then
    app.error("Invalid skin name" .. selectedName .. " for set " .. selectedSet .. " passed to setScenePresetForSkin")
    return
  end
  
  if scene == "AssetManager" then
    skin.RescaleInAssetManager = true
    skin.AssetManagerPreset = scalePreset
  else
    skin.RescaleInPreview = true
    skin.PreviewPreset = scalePreset
  end
  
  anim.setSkinData(skin, selectedName, selectedSet)
end

local setSceneClearPresetForSkin = function(selectedSet, selectedName, scalePreset, scene)  
  local skin = anim.getRigSkin(selectedName, selectedSet)
  
  if type(skin) ~= "table" then
    app.error("Invalid skin name" .. selectedName .. " for set " .. selectedSet .. " passed to setScenePresetForSkin")
    return
  end
  
  if scene == "AssetManager" then
    skin.RescaleInAssetManager = true
    skin.AssetManagerClearPreset = scalePreset
  else
    skin.RescaleInPreview = true
    skin.ClearPreset = scalePreset
  end
  
  anim.setSkinData(skin, selectedName, selectedSet)
end

local fileExists = function(filename)
  local demacro = utils.demacroizeString(filename)
  
  local handle = io.open(demacro, "r")
  if handle ~= nil then
    io.close(handle)
    return true
  end
  
  return false
end

local checkFileCanBeCreated = function(filePath)
  local directory, filename = splitFilePath(utils.demacroizeString(filePath))
  
  if filename == "" then
    return false
  end
  
  local testPath = directory .. "\\writeTest.tmp"
  local handle = io.open(testPath, "w")
  if handle ~= nil then
    io.close(handle)
    os.remove(testPath)
    return true
  end
  
  return false
end

local clearNewSkinWizard = function(data, panel)  
  panel:hide()
end

local cancelNewSkinWizard = function(data, panel)  
  if data.Result.Created then
    anim.removeSkin(data.SkinName)
  end

  clearNewSkinWizard(data, panel)
end

local isSkinNameUnique = function(selectedSet, name)
  local skins = anim.getRigSkins(selectedSet)
  return skins[name] == nil
end

local getUniqueSkinName = function(firstPart, selectedSet)
  local skins = anim.getRigSkins(selectedSet)
  
  local number = 1
  local skinName = firstPart
  while skins[skinName] ~= nil do
    skinName = firstPart .. tostring(number)
    number = number + 1
  end
  
  return skinName
  
end

local defaultSkinData = function(selectedSet)
  local result =
  {
    AnimationSet = selectedSet,
    SkinName = getUniqueSkinName("New Skin", selectedSet),
    SkinType = "New",
    
    -- the source of a new rig
    SourceXMD = "",
    NewMcSkinLocation = "",
    
    -- the source of an existing rig
    ExistingMcSkin = "",
    
    ScalePreset = "New",
    ScaleMessage = "New",
    
    NewScaleMessageName = "ScaleRig",
    ExistingScaleMessageName = "",
    ExistingScalePresetName = "",
    
    Result =
    {
      Created = false,
      CreatedName = ""
    }
  }
  
  return result
end

local buildNewSkinStart = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:setBorder(3)
    panel:addStaticText{ text = "What is the name of the new skin", flags = expand }
    local textbox = panel:addTextBox{
      name = "SkinName",
      flags = "expand"
    }
    
    panel:addVSpacer(20)
    
    
    panel:beginFlexGridSizer{ cols = 2, flags = "expand" }
    panel:setFlexGridColumnExpandable(2)
      local radioBox = panel:addCheckBox{
        name = "SkinType",
        flags = "expand",
        proportion = 1
      }
      panel:addStaticText{ text = "Create a new skin from existing morpheme:connect mcskin file?", flags = expand }
    panel:endSizer()
    
  panel:endSizer()
end

local updateNewSkinStart = function(data, panel, validate)
  local validatePanel = function()
    -- simple case, only this function affects the validation
    local valid = data.SkinName ~= ""

    if valid then
      local skinNameIsNotEmpty = data.SkinName ~= ""
      local iCreatedThisSkin = data.Result.Created and (data.Result.CreatedName == data.SkinName)
      
      valid = skinNameIsNotEmpty and (iCreatedThisSkin or isSkinNameUnique(data.AnimationSet, data.SkinName))
    end

    
    validate(valid)
    return valid
  end
  
  local skinName = panel:getChild("SkinName")
  skinName:setValue(data.SkinName)
  skinName:setError(not validatePanel())
  skinName:setOnChanged(function(self)
    data.SkinName = self:getValue()
    self:setError(not validatePanel())
    validatePanel()
    end)
  
  
  local skinType = panel:getChild("SkinType")
  skinType:setOnChanged(function(self)
    if self:getValue() == true then
      data.SkinType = "Existing"
    else
      data.SkinType = "New"
    end
  end)
    
  if data.SkinType == "New" then
    skinType:setValue(false)
  else
    skinType:setValue(true)
  end
end

local buildCreateNewSkin = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:setBorder(3)
    panel:addStaticText{ text = "Select the XMD or FBX file that contains your new skin" }
    panel:addFilenameControl{
      name      = "XMDSourceFile",
      caption   = "Pick the source file for the new skin",
      wildcard  = { { name = "XMD or FBX Files", extensions = { "xmd", "fbx" } } },
      flags = "expand",
      dialogStyle = "mustExist",
      proportion = 1,
      onMacroize = utils.macroizeString,
      onDemacroize = utils.demacroizeString,
    }
  panel:endSizer()
end

local updateCreateNewSkin = function(data, panel, validate)
  local validateNow = function()
    local isValid = fileExists(data.SourceXMD)
    validate(isValid)
    return isValid
  end
  
  local sourceFile = panel:getChild("XMDSourceFile")
  sourceFile:setValue(data.SourceXMD)
  sourceFile:setOnChanged(function(self)
    data.SourceXMD = utils.macroizeString(self:getValue())
    self:setValue(data.SourceXMD)
    
    self:setError(not validateNow())
  end)
  
  sourceFile:setError(not validateNow())
end

local buildWhereToSaveNewSkin = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:setBorder(3)
    panel:addStaticText{ text = "Select the location to save the new morpheme:connect skin" }
    panel:addFilenameControl{
      name      = "McSkinFile",
      caption   = "Pick the location to save the new skin",
      wildcard  = "Morpheme Skin|mcskin",
      flags = "expand",
      proportion = 1,
      onMacroize = utils.macroizeString,
      onDemacroize = utils.demacroizeString,
    }
  panel:endSizer()
end

local updateWhereToSaveNewSkin = function(data, panel, validate)
  local validateNow = function()
    local isValid = checkFileCanBeCreated(data.NewMcSkinLocation)
    validate(isValid)
    return isValid
  end
  
  local sourceFile = panel:getChild("McSkinFile")
  sourceFile:setValue(data.NewMcSkinLocation)
  sourceFile:setOnChanged(function(self)
    data.NewMcSkinLocation = utils.macroizeString(self:getValue())
    self:setValue(data.NewMcSkinLocation)
    
    self:setError(not validateNow())
  end)
  
  sourceFile:setError(not validateNow())
end

local buildLoadSkin = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:setBorder(3)
    panel:addStaticText{ text = "Select the morpheme:connect skin you want to add to the animation set" }
    panel:addFilenameControl{
      name      = "McSkinFile",
      caption   = "Pick the skin you wish to add",
      wildcard  = "Morpheme Skin|mcskin",
      flags = "expand",
      dialogStyle = "mustExist",
      proportion = 1,
      onMacroize = utils.macroizeString,
      onDemacroize = utils.demacroizeString,
    }
  panel:endSizer()
end

local updateLoadSkin = function(data, panel, validate)
  local validateNow = function()
    local isValid = fileExists(data.ExistingMcSkin)
    validate(isValid)
    return isValid
  end
  
  local sourceFile = panel:getChild("McSkinFile")
  sourceFile:setValue(data.ExistingMcSkin)
  sourceFile:setOnChanged(function(self)
    data.ExistingMcSkin = utils.macroizeString(self:getValue())
    self:setValue(data.ExistingMcSkin)
    
    self:setError(not validateNow())
  end)
  
  sourceFile:setError(not validateNow())
end

local buildAddPreset = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:addStaticText{ text = "A scale preset can be created to scale from the animation rig to the new skin." }
    panel:addRadioBox{
      name = "PresetType",
      flags = "expand"
    }
    
    panel:addVSpacer(20)
    
    local existingPresetPanel = panel:addPanel{ name = "ExistingPresetPanel", flags = "expand" }
    existingPresetPanel:beginVSizer{ flags = "expand" }
      existingPresetPanel:addStaticText{ text = "Which existing preset would you like to use" }
      
      existingPresetPanel:addComboBox{
        name = "ExistingPreset",
        flags = "expand",
        proportion = 0 
      }
    existingPresetPanel:endSizer()
  panel:endSizer()
end

local updateAddPreset = function(data, panel, validate)  
  local presetType = panel:getChild("PresetType")
  local existingPresetPanel = panel:getChild("ExistingPresetPanel")
  local existingPreset = existingPresetPanel:getChild("ExistingPreset")
  
  local availablePresetTypes = { "No Preset", "Generate a New Preset" }
  local scalePresets = { }
  
  local requestPresets = ls("MessagePresets")
  local foundCurrentPreset = false
  local alreadyFoundAScalePreset = false
  for i,v in pairs(requestPresets) do
    if getType(v) == scaleMessageType then
      if not alreadyFoundAScalePreset then
        alreadyFoundAScalePreset = true
        table.insert(availablePresetTypes, "Existing scale preset")
      end
      
      local path, name = splitNodePath(v)
      
      if data.ExistingScalePresetName == name then
        foundCurrentPreset = true
      end
      
      table.insert(scalePresets, name)
    end
  end
  
  if not foundCurrentPreset and table.getn(scalePresets) ~= 0 then
    data.ExistingScalePresetName = scalePresets[1]
  end
  
  local updateExistingPresetPanel = function()
    existingPreset:setItems(scalePresets)
    existingPreset:setSelectedItem(data.ExistingScalePresetName)
    existingPresetPanel:setShown(data.ScalePreset == "Existing")
    
    panel:bestSizeChanged()
    panel:getParent():getParent():getParent():doLayout()
  end
  
  presetType:setItems(availablePresetTypes)
  
  presetType:setOnChanged(function(self)
    if self:getSelectedIndex() == 1 then
      data.ScalePreset = "None"
    elseif self:getSelectedIndex() == 2 then
      data.ScalePreset = "New"
    else
      data.ScalePreset = "Existing"
    end
    
    updateExistingPresetPanel()
  end)
  
  
  if data.ScalePreset == "None" then
    presetType:setSelectedIndex(1)
  elseif data.ScaleMessage == "New" then
    presetType:setSelectedIndex(2)
  else
    presetType:setSelectedIndex(3)
  end
  
  existingPreset:setOnChanged(function(self)
    data.ExistingScalePresetName = self:getSelectedItem()
  end)
  
  updateExistingPresetPanel()
  validate(true)
end

local buildAddMessage = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }    
    panel:addStaticText{ text = "Would you like to send a message when the new skin is used" }
    panel:addRadioBox{
      name = "MessageType",
      flags = "expand"
    }
    
    panel:addVSpacer(20)
    
    local existingMessagePanel = panel:addPanel{ name = "ExistingMessagePanel", flags = "expand" }
    existingMessagePanel:beginVSizer{ flags = "expand" }
      existingMessagePanel:addStaticText{ text = "Which existing message would you like to use" }
      
      existingMessagePanel:addComboBox{
        name = "ExistingMessage",
        flags = "expand",
        proportion = 0 
      }
    existingMessagePanel:endSizer()
  panel:endSizer()
end

local updateAddMessage = function(data, panel, validate) 
  local messageType = panel:getChild("MessageType")
  local existingMessagePanel = panel:getChild("ExistingMessagePanel")
  local existingMessage = existingMessagePanel:getChild("ExistingMessage")
 
  local availableMessageTypes = { "No Message", "Create a New Message" }
  local scaleMessages = { }
  
  local requests = ls("Request")
  local alreadyFoundAScaleMessage = false
  local foundCurrentMessage = false
  for i,v in pairs(requests) do
    if getType(v) == scaleMessageType then
      if not alreadyFoundAScaleMessage then
        alreadyFoundAScaleMessage = true
        table.insert(availableMessageTypes, "Existing scale message")
      end
      local path, name = splitNodePath(v)
      
      if data.ExistingScaleMessageName == name then
        foundCurrentMessage = true
      end
      
      table.insert(scaleMessages, name)
    end
  end
  
  if not foundCurrentMessage and table.getn(scaleMessages) ~= 0 then
    data.ExistingScaleMessageName = scaleMessages[1]
  end
  
  local updateExistingMessagePanel = function()
    existingMessage:setItems(scaleMessages)
    existingMessage:setSelectedItem(data.ExistingScaleMessageName)
    existingMessagePanel:setShown(data.ScaleMessage == "Existing")
    
    panel:bestSizeChanged()
    panel:getParent():getParent():getParent():doLayout()
  end
  
  messageType:setItems(availableMessageTypes)
  
  messageType:setOnChanged(function(self)
    if self:getSelectedIndex() == 1 then
      data.ScaleMessage = "None"
    elseif self:getSelectedIndex() == 2 then
      data.ScaleMessage = "New"
    else
      data.ScaleMessage = "Existing"
    end
    
    updateExistingMessagePanel()
  end)
  
  if data.ScaleMessage == "None" then
    messageType:setSelectedIndex(1)
  elseif data.ScaleMessage == "New" then
    messageType:setSelectedIndex(2)
  else
    messageType:setSelectedIndex(3)
  end
  
  existingMessage:setOnChanged(function(self)
    data.ExistingScaleMessageName = self:getSelectedItem()
  end)
  
  updateExistingMessagePanel()
  validate(true)
end

local addSkinWizard = function(selectedSet)
  -- the data we are editing throughout the wizard.
  local skinData = defaultSkinData(selectedSet)
  
  -- grab the dialog, or create it.
  local dlg = ui.getWindow("NewSkinWizard")
  if not dlg then
    dlg = ui.createModelessDialog{
      name = "NewSkinWizard",
      caption = "New Skin",
      centre = true,
      resize = false
    }
    -- build ui which is not torn down each time we show a new stage, or end the wizard and start again
    buildWizardPersistantUI(dlg)
  end
  
  local cancelFn = function()
    cancelNewSkinWizard(skinData, dlg)
  end
  dlg:setOnClose(cancelFn)

  -- called when a new animation set (from fbx/xmd) is called
  local start, loadSkin, newSkin, whereToSaveNewSkin, addPresetForSkin

  addMessageForSkin = function(previous)
    local onNext = function()
      local messageName = ""
      if skinData.ScaleMessage == "New" then
        local createdMessage = addMessage("RigScaleChanged")
        local path, name = splitNodePath(createdMessage)
        messageName = name
      elseif skinData.ScaleMessage == "Existing" then
        messageName = skinData.ExistingScaleMessageName
      end
      
      if messageName ~= "" then
        setMessageForSkin(skinData.AnimationSet, skinData.SkinName, messageName)
      end
      
      clearNewSkinWizard(skinData, dlg)
    end
    
    showWizardStage("addMessage", skinData, dlg, buildAddMessage, updateAddMessage,
      "Should a message be sent when the new skin is used during preview",
      {
        Cancel = { onPress = cancelFn },
        Previous = { onPress = previous },
        Create = { onPress = onNext, disableWhenInvalid = true }
      })
  end
  
  addPresetForSkin = function(previous)
    -- pause stage, as loading the skij could take time.
    pauseStage(dlg, "waitWhileSkinIsLoaded", "Please wait whilst the skin is loaded", skinData, cancelFn, function()
      -- clear up old skin from the wizard
      if skinData.Result.Created then
        skinData.Result.Created = false
        anim.removeSkin(skinData.SkinName)
      end
      
      -- add the new skin
      local mcSkin = skinData.ExistingMcSkin
      if skinData.SkinType == "New" then
        mcSkin = skinData.NewMcSkinLocation
      end
      local result, errorMessage = anim.addSkin(skinData.SkinName, mcSkin, skinData.AnimationSet)
      
      -- failure report if the skin was wrong
      if not result then
        generateFailure = function()
          return "There was an error whilst loading the specified skin:\n\n  " .. errorMessage .. "\n\nPlease check the skin '" .. mcSkin .. "'\nis compatible with the animation set '" .. skinData.AnimationSet .. "'"
        end
        
        messageStage("errorLoadingSkin", "Error loading skin", dlg, generateFailure,
        {
          Cancel = { onPress = cancelFn },
          Previous = { onPress = function() loadSkin(previous) end },
          Next = { disabled = true }
        })
        return
      end
      skinData.Result.Created = true
      skinData.Result.CreatedName = skinData.SkinName
      
      -- check if the skin might need scaling
      local needsScale, scaleFactors = anim.generateScaleFactorsFromRigToSkin(skinData.SkinName, skinData.AnimationSet)
      if not needsScale then
        clearNewSkinWizard(skinData, dlg)        
      else
        local onNext = function()
          local scalePreset = ""
          if skinData.ScalePreset == "New" then
            local createdMessage = addDefaultScalePreset(skinData.AnimationSet, skinData.SkinName)
            local path, name = splitNodePath(createdMessage)
            scalePreset = name
          elseif skinData.ScalePreset == "Existing" then
            scalePreset = skinData.ExistingScalePresetName
          end
          
          if scalePreset ~= "" then
            setScenePresetForSkin(skinData.AnimationSet, skinData.SkinName, scalePreset, "AssetManager")
            setScenePresetForSkin(skinData.AnimationSet, skinData.SkinName, scalePreset, "Network")
          end
          
          addMessageForSkin(function() addPresetForSkin(previous) end)
        end
        
        showWizardStage("addPreset", skinData, dlg, buildAddPreset, updateAddPreset,
          "The new skin is a different size to the animation rig",
          {
            Cancel = { onPress = cancelFn },
            Previous = { onPress = previous },
            Next = { onPress = onNext, disableWhenInvalid = true }
          })
      end
    end)
  end

  whereToSaveNewSkin = function()
    -- the callback triggered when next is chosen
    local onNext = function()
      pauseStage(dlg, "waitWhileSkinIsCreated", "Please wait whilst the skin is created", skinData, cancelFn, function()
        local rigUnit = units.getRigUnit(skinData.AnimationSet)
        if anim.createSkin(skinData.SourceXMD, skinData.NewMcSkinLocation, rigUnit.scaleFactor) then
          -- make sure we pass in the correct previous so the users wizard path tracks with them
          addPresetForSkin(whereToSaveNewSkin)
        else
          generateFailure = function()
            return "There was an error whilst creating the specified skin.\n\nPlease check the skin '" .. skinData.SourceXMD .. "'\nis a valid source file for the '" .. skinData.AnimationSet .. "' animation set."
          end
          
          messageStage("errorCreatingSkin", "Error creating skin", dlg, generateFailure,
          {
            Cancel = { onPress = cancelFn },
            Previous = { onPress = function() whereToSaveNewSkin(previous) end },
            Next = { disabled = true }
          })
          return        
        end
      end)
    end
    
    showWizardStage("whereToSaveNewSkin", skinData, dlg, buildWhereToSaveNewSkin, updateWhereToSaveNewSkin,
      "Where should the new skin be saved",
      {
        Cancel = { onPress = cancelFn },
        Previous = { onPress = previous },
        Next = { onPress = onNext, disableWhenInvalid = true }
      })
  end

  newSkin = function()    
    showWizardStage("newSkin", skinData, dlg, buildCreateNewSkin, updateCreateNewSkin,
      "Create a skin from an FBX or XMD file",
      {
        Cancel = { onPress = cancelFn },
        Previous = { onPress = start },
        Next = { onPress = whereToSaveNewSkin, disableWhenInvalid = true }
      })
  end

  loadSkin = function()
    -- the callback triggered when next is chosen
    local onNext = function()
      -- make sure we pass in the correct previous so the users wizard path tracks with them
      addPresetForSkin(loadSkin)      
    end
    
    showWizardStage("loadSkin", skinData, dlg, buildLoadSkin, updateLoadSkin,
      "Load an existing morpheme:connect skin",
      {
        Cancel = { onPress = cancelFn },
        Previous = { onPress = start },
        Next = { onPress = onNext, disableWhenInvalid = true }
      })
  end
  
  -- function called to start the wizard off.
  start = function()
    -- the callback triggered when next is chosen
    local onNext = function()
      if skinData.SkinType == "New" then
        newSkin()
      else
        loadSkin()
      end
    end
    
    showWizardStage("start", skinData, dlg, buildNewSkinStart, updateNewSkinStart,
      "Create a new skin",
      {
        Cancel = { onPress = cancelFn },
        Next = { onPress = onNext, disableWhenInvalid = true }
      })
  end
  
  -- kick off the wizard at the start...
  start()
  dlg:show()  
end

------------------------------------------------------------------------------------------------------------------------
-- shows the dialog for adding a skin to the selected set
------------------------------------------------------------------------------------------------------------------------
showAddSkinDialog = function(currentSet)  
  local dlg = ui.getWindow("AddSkinDialog")
  if not dlg then
    dlg = ui.createModalDialog{ name = "AddSkinDialog", caption = "Add Skin", resize = false, size = { width = 240, height =100 } }
    dlg:beginVSizer()
    
      dlg:beginHSizer{ flags = "expand" }
       dlg:addStaticText{ text = "Name" }
       local nameTextBoxControl = dlg:addTextBox{ name = "NameText", flags = "expand", proportion = 1 }
      dlg:endSizer()
      
      dlg:beginHSizer{ flags = "expand" }
       dlg:addStaticText{ text = "Filename" }
       local skinFilenameControl = dlg:addFilenameControl{
          name = "AddSkinFileControl",
          flags = "expand",
          wildcard = "morpheme:connect skin files|mcskin",
          proportion = 1,
          onMacroize = utils.macroizeString,
          onDemacroize = utils.demacroizeString,
        }
      dlg:endSizer()

      dlg:beginHSizer{ flags = "expand" }
        dlg:addStretchSpacer{ proportion = 1 }
        dlg:addButton{ label = "OK", onClick = 
         function()
           local fileName = skinFilenameControl:getValue()
           local name =nameTextBoxControl:getValue()
           dlg:hide() 
           anim.addSkinPath(name, fileName, currentSet)
         end }
         
        dlg:addButton{ label = "Cancel", onClick = function()
                local dlg = ui.getWindow("AddSkinDialog")
                dlg:hide()
              end 
        }
        
      dlg:endSizer()
      
    dlg:endSizer()
  end
 dlg:show()

end

------------------------------------------------------------------------------------------------------------------------
-- Updates the enable state of the skins rollup
------------------------------------------------------------------------------------------------------------------------
registeredComboBoxHandlerFunction = nil
registeredAnimSetModifiedHandlerFunction = nil

updateSkinsRollup = function(panel, animationSetsTreeControl)  
  local skinsRollup = panel:findDescendant("skinsRollup")
  local skinsPanel = skinsRollup:getPanel()
  
  local skinsTreeControl = skinsPanel:getChild("skinsList")
  
  local pathControl = skinsPanel:getChild("Path")
  
  local rescaleInAssetManagerLabel = skinsPanel:getChild("RescaleInAssetManagerLabel")
  local rescaleInAssetManager = skinsPanel:getChild("RescaleInAssetManager")
  local assetManagerPresetLabel = skinsPanel:getChild("AMPresetLabel")
  local assetManagerPreset = skinsPanel:getChild("AssetManagerPreset")
  local assetManagerClearPresetLabel = skinsPanel:getChild("AMClearPresetLabel")
  local assetManagerClearPreset = skinsPanel:getChild("AssetManagerClearPreset")
  local createDefaultClearPresetAssetManager = skinsPanel:getChild("CreateDefaultClearPresetAssetManager")
  local sendSkinScaleNow = skinsPanel:getChild("SendSkinScalePreset")
  local createDefaultPresetAssetManager = skinsPanel:getChild("CreateDefaultPresetAssetManager")
  
  local rescaleInPreviewLabel = skinsPanel:getChild("RescaleInPreviewLabel")
  local rescaleInPreview = skinsPanel:getChild("RescaleInPreview")
  local previewMessageLabel = skinsPanel:getChild("PMessageLabel")
  local previewPresetLabel = skinsPanel:getChild("PPresetLabel")
  local previewMessage = skinsPanel:getChild("PreviewMessage")
  local assetManagerPresetLabel = skinsPanel:getChild("AMPresetLabel")
  local previewPreset = skinsPanel:getChild("PreviewPreset")
  local previewClearMessageLabel = skinsPanel:getChild("PClearMessageLabel")
  local clearPreset = skinsPanel:getChild("PClearPreset")
  local createDefaultPresetPreview = skinsPanel:getChild("CreateDefaultPresetPreview")
  local createDefaultClearPresetPreview = skinsPanel:getChild("CreateDefaultClearPresetPreview")
  
  
  local updateSkinSettingsUI = function(panel, selectedSkin)    
    if selectedSkin then
      pathControl:setValue(selectedSkin.Path)
      pathControl:enable(true)
      
      rescaleInAssetManagerLabel:enable(true)
      rescaleInAssetManager:setValue(selectedSkin.RescaleInAssetManager)
      rescaleInAssetManager:enable(true)
      
      local rescaleInAssetManagerValue = rescaleInAssetManager:getValue()
      assetManagerPresetLabel:enable(rescaleInAssetManagerValue)
      assetManagerPreset:setValue(selectedSkin.AssetManagerPreset)
      assetManagerPreset:enable(rescaleInAssetManagerValue)
      sendSkinScaleNow:enable(rescaleInAssetManagerValue)
      createDefaultPresetAssetManager:enable(rescaleInAssetManagerValue)
      
      assetManagerClearPresetLabel:enable(rescaleInAssetManagerValue)
      assetManagerClearPreset:setValue(selectedSkin.AssetManagerClearPreset)
      assetManagerClearPreset:enable(rescaleInAssetManagerValue)
      createDefaultClearPresetAssetManager:enable(rescaleInAssetManagerValue)
      
      if selectedSkin.AssetManagerPreset == "" then
        assetManagerPreset:setError(true)
      else
        assetManagerPreset:setError(false)
      end
      
      sendSkinScaleNow:setOnClick(
        function()
          if type(selectedSkin.AssetManagerPreset) == "string" then
            anim.broadcastAssetManagerMessage("MessagePresets|" .. selectedSkin.AssetManagerPreset)
            anim.setAssetManagerRescalingEnabled(true)
          end
        end
      )
      
      rescaleInPreviewLabel:enable(true)
      rescaleInPreview:setValue(selectedSkin.RescaleInPreview)
      rescaleInPreview:enable(true)
      
      local rescaleInPreviewValue = rescaleInPreview:getValue()
      previewMessageLabel:enable(rescaleInPreviewValue)
      previewMessage:setValue(selectedSkin.PreviewMessage)
      previewMessage:enable(rescaleInPreviewValue)
      previewPresetLabel:enable(rescaleInPreviewValue)
      previewPreset:setValue(selectedSkin.PreviewPreset)
      previewPreset:enable(rescaleInPreviewValue)
      createDefaultPresetPreview:enable(rescaleInPreviewValue)
      createDefaultClearPresetPreview:enable(rescaleInPreviewValue)
      
      previewClearMessageLabel:enable(rescaleInPreviewValue)
      clearPreset:setValue(selectedSkin.ClearPreset)
      clearPreset:enable(rescaleInPreviewValue)
      
      if selectedSkin.PreviewPreset == "" then
        previewPreset:setError(true)
      else
        previewPreset:setError(false)
      end
      
      if selectedSkin.PreviewMessage == "" then
        previewMessage:setError(true)
      else
        previewMessage:setError(false)
      end
      
      
    else
      pathControl:setValue("")
      pathControl:enable(false)
      
      rescaleInAssetManagerLabel:enable(false)
      rescaleInAssetManager:setValue(false)
      rescaleInAssetManager:enable(false)
      assetManagerPresetLabel:enable(false)
      assetManagerPreset:setValue("")
      assetManagerPreset:enable(false)
      assetManagerPreset:setError(false)
      sendSkinScaleNow:enable(false)
      createDefaultPresetAssetManager:enable(false)
      
      sendSkinScaleNow:setOnClick(nil)
      
      assetManagerClearPresetLabel:enable(false)
      assetManagerClearPreset:setValue("")
      assetManagerClearPreset:setError(false)
      assetManagerClearPreset:enable(false)
      createDefaultClearPresetAssetManager:enable(false)
      
      rescaleInPreviewLabel:enable(false)
      rescaleInPreview:setValue(false)
      rescaleInPreview:enable(false)
      previewMessageLabel:enable(false)
      previewMessage:setValue("")
      previewMessage:enable(false)
      previewMessage:setError(false)
      previewPresetLabel:enable(false)
      previewPreset:setValue("")
      previewPreset:enable(false)
      previewPreset:setError(false)
      previewClearMessageLabel:enable(false)
      clearPreset:setValue("")
      clearPreset:enable(false)
      clearPreset:setError(false)
      createDefaultPresetPreview:enable(false)
      createDefaultClearPresetPreview:enable(false)
    end
  end
  
  local updatePerSkinSettings = function()
    local selectedRow = skinsTreeControl:getSelectedItem()
    
    if selectedRow ~= nil then
      local selectedName = selectedRow:getValue()
      
      local items = anim.getRigSkins(selectedSet)
      
      updateSkinSettingsUI(panel, items[selectedName])
    else
      updateSkinSettingsUI(panel, nil)
    end
  end
  
  local rebuildRequestComboBoxes = function()    
    local selectedSkin = nil
    local selectedRow = skinsTreeControl:getSelectedItem()
      
    if selectedRow ~= nil then
      local selectedName = selectedRow:getValue()
      
      local items = anim.getRigSkins(selectedSet)
      
      selectedSkin = items[selectedName]
    end
    
    -- update the preview messages
    local networkMessages = {}
    for i,v in ipairs(ls("Request")) do
      if getType(v) == scaleMessageType then
        local path, name = splitNodePath(v)
        table.insert(networkMessages, name)
      end
    end
    table.insert(networkMessages, "")
   
    previewMessage:setItems(networkMessages)
    
    scalePresets = { }
    for i,v in ipairs(ls("MessagePresets")) do
      if getType(v) == scaleMessageType then
        local path, name = splitNodePath(v)
        table.insert(scalePresets, name)
      end
    end
    table.insert(scalePresets, "")
    
    -- update the preview messages   
    clearPreset:setItems(scalePresets)
    assetManagerClearPreset:setItems(scalePresets)
    
    assetManagerPreset:setItems(scalePresets)
    previewPreset:setItems(scalePresets)
    
    updatePerSkinSettings()
  end
  
  if registeredComboBoxHandlerFunction then
    unregisterEventHandler("mcRequestCreated", registeredComboBoxHandlerFunction)
    unregisterEventHandler("mcRequestDestroyed", registeredComboBoxHandlerFunction)
    unregisterEventHandler("mcRequestRenamed", registeredComboBoxHandlerFunction)
    
    unregisterEventHandler("mcRequestPresetCreated", registeredComboBoxHandlerFunction)
    unregisterEventHandler("mcRequestPresetDestroyed", registeredComboBoxHandlerFunction)
    unregisterEventHandler("mcRequestPresetRenamed", registeredComboBoxHandlerFunction)
  end
  registeredComboBoxHandlerFunction = rebuildRequestComboBoxes
  
  registerEventHandler("mcRequestCreated", rebuildRequestComboBoxes)
  registerEventHandler("mcRequestDestroyed", rebuildRequestComboBoxes)
  registerEventHandler("mcRequestRenamed", rebuildRequestComboBoxes)
  
  registerEventHandler("mcRequestPresetCreated", rebuildRequestComboBoxes)
  registerEventHandler("mcRequestPresetDestroyed", rebuildRequestComboBoxes)
  registerEventHandler("mcRequestPresetRenamed", rebuildRequestComboBoxes)
  
  local animSetModified = function(skinName)
    local selectedRow = skinsTreeControl:getSelectedItem()
    
    if selectedRow ~= nil then
      local selectedName = selectedRow:getValue()
      if selectedName == skinName then
        updatePerSkinSettings()
      end
    end
  end
  
  if registeredAnimSetModifiedHandlerFunction then
    unregisterEventHandler("mcAnimationSetSkinModified", registeredAnimSetModifiedHandlerFunction)
  end
  
  registeredAnimSetModifiedHandlerFunction = animSetModified
  -- update when the DB changes
  registerEventHandler("mcAnimationSetSkinModified", animSetModified)
  
  local setFromUI = function()
    local selectedRow = skinsTreeControl:getSelectedItem()
    
    if selectedRow ~= nil then
      local newData =
      {
        Path = pathControl:getValue(),
        RescaleInAssetManager = rescaleInAssetManager:getValue(),
        AssetManagerPreset = assetManagerPreset:getValue(),
        AssetManagerClearPreset = assetManagerClearPreset:getValue(),
        RescaleInPreview = rescaleInPreview:getValue(),
        PreviewMessage = previewMessage:getValue(),
        PreviewPreset = previewPreset:getValue(),
        ClearPreset = clearPreset:getValue()
      }
      
      local selectedName = selectedRow:getValue()
      
      local selectedSet = animationSetsTreeControl:getSelectedItem()
      anim.setSkinData(newData, selectedName, selectedSet)
    end
  end
  
  local createDefaultPreset = function(scene)
    local selectedRow = skinsTreeControl:getSelectedItem()
    if selectedRow ~= nil then
      local selectedName = selectedRow:getValue()
      local selectedSet = animationSetsTreeControl:getSelectedItem()
    
      local newPreset = addDefaultScalePreset(selectedSet, selectedName)
      rebuildRequestComboBoxes()
      
      local newPresetParentPath, newPresetName = splitNodePath(newPreset)
      setScenePresetForSkin(selectedSet, selectedName, newPresetName, scene)
    end
  end
  
  local createDefaultClearPreset = function(scene)
    local selectedRow = skinsTreeControl:getSelectedItem()
    if selectedRow ~= nil then
      local selectedName = selectedRow:getValue()
      local selectedSet = animationSetsTreeControl:getSelectedItem()
    
      local newPreset = addIdentityScalePreset(selectedSet, selectedName)
      rebuildRequestComboBoxes()
      
      local newPresetParentPath, newPresetName = splitNodePath(newPreset)
      setSceneClearPresetForSkin(selectedSet, selectedName, newPresetName, scene)
    end
  end
  
  createDefaultClearPresetAssetManager:setOnClick(function() createDefaultClearPreset("AssetManager") end)
  createDefaultClearPresetPreview:setOnClick(function() createDefaultClearPreset("Network") end)
  
  createDefaultPresetAssetManager:setOnClick(function() createDefaultPreset("AssetManager") end)
  createDefaultPresetPreview:setOnClick(function() createDefaultPreset("Network") end)
  
  skinsTreeControl:setOnSelectionChanged(updatePerSkinSettings)

  local updateSkinsList = function()
    local selectedSet = animationSetsTreeControl:getSelectedItem()
    local items = anim.getRigSkins(selectedSet)
    local root = skinsTreeControl:getRoot()

    root:clearChildren()

    for name,data in pairs(items) do
      root:addChild(name)
    end
  end

  skinsTreeControl:setOnItemRenamed(function(tree, item, oldName)
    local currentSkin = anim.getCurrentAssetManagerSkin(selectedSet)
    local setOnFinish = false -- set by default
    if type(currentSkin) == "table" then
      setOnFinish = oldName == currentSkin.Name
    end

    local skin = anim.getRigSkin(oldName, selectedSet)

    if type(skin) ~= "table" then
      return
    end

    local newName = item:getValue()

    anim.removeSkin(oldName, selectedSet)
    anim.addSkin(newName, skin.Path, selectedSet)
    anim.setSkinData(skin, newName, selectedSet)

    if setOnFinish then
      anim.setCurrentAssetManagerSkin(newName)
    end
    updateSkinsList()
  end)
  
  -- update UI on change
  pathControl:setOnChanged(setFromUI)
  
  rescaleInAssetManager:setOnChanged(setFromUI)
  assetManagerPreset:setOnChanged(setFromUI)
  assetManagerClearPreset:setOnChanged(setFromUI)

  rescaleInPreview:setOnChanged(setFromUI)
  previewMessage:setOnChanged(setFromUI)
  previewPreset:setOnChanged(setFromUI)
  clearPreset:setOnChanged(setFromUI)
   
  -- get the currently selected set and format
  local selectedSet = animationSetsTreeControl:getSelectedItem()
  local setIsValid = type(selectedSet) == "string" and string.len(selectedSet) > 0

  local rigIsValid = false
  if setIsValid then
    local scene = nmx.Application.new():getSceneByName("AssetManager")
    rigIsValid = anim.isRigValid(selectedSet) and (anim.getRigDataRoot(scene, selectedSet) ~= nil)
  end
  
  if setIsValid and rigIsValid then
    updateSkinsList()
    rebuildRequestComboBoxes()
    updatePerSkinSettings()
  end
  
end

------------------------------------------------------------------------------------------------------------------------
-- Adds the skins rollup and all it's controls
------------------------------------------------------------------------------------------------------------------------
addSkinsRollup = function(panel, animationSetsTreeControl)
  local skinsRollup = panel:addRollup{
    name = "skinsRollup",
    label = "Skins",
    flags = "expand;mainSection",
  }
  skinsRollup:expand(false)
  
  local skinsPanel = skinsRollup:getPanel()
  local treeControl = nil
  skinsPanel:beginVSizer{ flags = "expand", proportion = 1 }
    
    local toolbar = skinsPanel:addToolBar{
      name = "ToolBar",
    }

    addButton = toolbar:addButton{
      name = "AddButton",
      image = app.loadImage("additem.png"),
      helpText = "Add Skin",
      onClick = function(self)
        addSkinWizard(animationSetsTreeControl:getSelectedItem())
        updateSkinsRollup(panel, animationSetsTreeControl)
      end
    }

    removeButton = toolbar:addButton{
      name = "RemoveButton",
      image = app.loadImage("removeitem.png"),
      helpText = "Delete Skin",
      onClick = function(self)
        local selectedSet = animationSetsTreeControl:getSelectedItem()
        local selected = treeControl:getSelectedItems()
        for i,v in ipairs(selected) do
          local name = v:getValue()
          anim.removeSkin(name, selectedSet)
        end
        
        updateSkinsRollup(panel, animationSetsTreeControl)       
     end
    }


    treeControl = skinsPanel:addTreeControl{
      name = "skinsList",
      size = { height = -1 },
      flags = "sizeToContent;expand;hideRoot;rename",
    }
    
    skinsPanel:beginHSizer{ flags = "expand" }
      skinsPanel:addStaticText{ text = "Path" }
      skinsPanel:addFilenameControl{
        name = "Path",
        flags = "expand",
        wildcard = "morpheme:connect skin files|mcskin",
        proportion = 1,
        onMacroize = utils.macroizeString,
        onDemacroize = utils.demacroizeString,
     }
    skinsPanel:endSizer()
    
    skinsPanel:addStaticText{ name = "SkinScalingText", text = "Skin Scaling" }
    
    skinsPanel:beginVSizer{ label = "Asset Manager Rescaling", flags = "expand;group", proportion = 1 }
      skinsPanel:beginHSizer{ flags = "expand" }
         skinsPanel:addStaticText{ name = "RescaleInAssetManagerLabel",  text = "Rescale in asset manager" }
         local skinFilenameControl = skinsPanel:addCheckBox{
              name = "RescaleInAssetManager",
              proportion = 1,
              flags = "expand",
            }
      skinsPanel:endSizer()
      skinsPanel:beginHSizer{ flags = "expand" }
         skinsPanel:addStaticText{ name = "AMPresetLabel", text = "Preset to send" }
         local presetAssetManager = skinsPanel:addComboBox{
                  name = "AssetManagerPreset",
                  flags = "expand",
                  proportion = 1
                }
      
        skinsPanel:addButton{
          name = "CreateDefaultPresetAssetManager",
          label = "Add Default",
        }
      skinsPanel:endSizer()
      
      skinsPanel:beginHSizer{ flags = "expand" }
         skinsPanel:addStaticText{ name = "AMClearPresetLabel", text = "Clear Preset to send" }
         local presetAssetManager = skinsPanel:addComboBox{
                  name = "AssetManagerClearPreset",
                  flags = "expand",
                  proportion = 1
                }
      
        skinsPanel:addButton{
          name = "CreateDefaultClearPresetAssetManager",
          label = "Add Default",
        }
      skinsPanel:endSizer()
      
      skinsPanel:addButton{
        name = "SendSkinScalePreset",
        label = "Send skin scale preset",
      }
      
    skinsPanel:endSizer()
    
     skinsPanel:beginVSizer{ label = "Preview Rescaling", flags = "expand;group", proportion = 1 }
      skinsPanel:beginHSizer{ flags = "expand" }
         skinsPanel:addStaticText{ name = "RescaleInPreviewLabel",  text = "Rescale during preview" }
         local skinFilenameControl = skinsPanel:addCheckBox{
              name = "RescaleInPreview",
              proportion = 1,
              flags = "expand",
            }
      skinsPanel:endSizer()

      skinsPanel:beginHSizer{ flags = "expand" }
         skinsPanel:addStaticText{ name = "PMessageLabel", text = "Message to send" }
         local presetAssetManager = skinsPanel:addComboBox{
                  name = "PreviewMessage",
                  flags = "expand",
                  proportion = 1
                }
      skinsPanel:endSizer()
      
      skinsPanel:beginHSizer{ flags = "expand" }
         skinsPanel:addStaticText{ name = "PPresetLabel", text = "Preset to send" }
         local presetAssetManager = skinsPanel:addComboBox{
                  name = "PreviewPreset",
                  flags = "expand",
                  proportion = 1
                }
                
        skinsPanel:addButton{
          name = "CreateDefaultPresetPreview",
          label = "Add Default",
        }
      skinsPanel:endSizer()
      
      skinsPanel:beginHSizer{ flags = "expand" }
        skinsPanel:addStaticText{ name = "PClearMessageLabel", text = "Clear Preset to send" }
        skinsPanel:addComboBox{
          name = "PClearPreset",
          flags = "expand",
          proportion = 1
        }
                
        skinsPanel:addButton{
          name = "CreateDefaultClearPresetPreview",
          label = "Add Default",
        }
      skinsPanel:endSizer()
    skinsPanel:endSizer()
  
  skinsPanel:endSizer()
  
  updateSkinsRollup(panel, animationSetsTreeControl)
end
