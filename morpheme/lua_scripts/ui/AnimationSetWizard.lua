require("ui/Components/Template.lua")
require("ui/Components/Tags.lua")
require("ui/WizardAPI.lua")
require("luaAPI/MorphemeUnitAPI.lua")

local normalButtonTypes = { "Cancel", "Previous", "Next", "Create" }
local connectAppName = "euphoria"
if mcn.isEuphoriaDisabled() then
  connectAppName = "morpheme"
end

------------------------------------------------------------------------------------------------------------------------
local fileExists = function(filename)
  local demacro = utils.demacroizeString(filename)
  
  local handle = io.open(demacro, "r")
  if handle ~= nil then
    io.close(handle)
    return true
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
local checkFileCanBeCreated = function(filePath)
  -- Split before demacroizing, this ensures that "$(RootDir)blah" fails
  -- even though, after demacroizing, it might be a usable path.
  local directory, filename = splitFilePath(filePath)
  directory = utils.demacroizeString(directory)

  if filename == "" or directory == "" then
    return false
  end

  -- Check if it's directory can be wrote into
  local testPath = directory .. "\\writeTest.tmp"
  local handle = io.open(testPath, "w")
  if handle == nil then
    return false
  end
  io.close(handle)
  os.remove(testPath)

  -- Check the filename doesn't corrispond to a directory.
  testPath = string.format([[%s\%s\writeTest.tmp]], directory, filename)
  handle = io.open(testPath, "w")
  if handle ~= nil then
    io.close(handle)
    os.remove(testPath)
    return false
  end

  return true
end

------------------------------------------------------------------------------------------------------------------------
local guessFilename = function(directory, filename, reps, exists)
  local guess
  local guessFilename = filename
  for _,rep in ipairs(reps) do
    guessFilename = guessFilename:gsub(rep[1], rep[2])
    guess = directory .. "\\" .. guessFilename

    -- check if this file exists.
    if exists and fileExists(guess) then
      return guess
    end
  end

  if (not exists and fileExists(guess)) or (exists and not fileExists(guess)) then
    return ""
  end

  return guess
end

------------------------------------------------------------------------------------------------------------------------
local getFilenameWithoutDirectory = function(filePath)
  if filePath == nil or filePath == "" then
    return ""
  end

  local directory, filename = splitFilePath(filePath)
  return filename
end

------------------------------------------------------------------------------------------------------------------------
local guessOtherFileFromFile = function(path, exists)
  local directory, filename = splitFilePath(path)
  local strippedFilename = stripFilenameExtension(filename)
  local extension = getFilenameExtension(filename)

  local replacementsAnimation = {
    { "mcarig",  "mcprig" },
    { "Animation", "Physics" },
    { "animation", "physics" },
    { "Anim", "Phys" }
  }

  local replacementsPhysics = {
    { "mcprig", "mcarig" },
    { "Physics", "Animation" },
    { "physics", "animation" },
    { "Phys", "Anim" }
  }

  if extension == "mcprig" then
    return guessFilename(directory, filename, replacementsPhysics, exists)
  else
    return guessFilename(directory, filename, replacementsAnimation, exists)
  end
end

------------------------------------------------------------------------------------------------------------------------
local findAnimSet = function(name)
  local animSets = listAnimSets()
  for i,v in ipairs(animSets) do
    if v == name then
      return true
    end
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
local makeUniqueAnimSetName = function(mainName)
  local outputName = mainName
  local index = 1
  while findAnimSet(outputName) == true do
    outputName = mainName .. tostring(index)
    index = index + 1
  end
  return outputName
end

------------------------------------------------------------------------------------------------------------------------
local getBestMaleCharacterAnimationLocation = function()
  local filename = app.getCommonDocumentsDir() .. "\\samples\\Y-Up_Metres\\Characters\\MaleCharacter\\morphemeRigs\\maleCharacterAnimationRig.mcarig"

  if not fileExists(filename) then
    filename = preferences.get("DefaultAnimationRigFile")
  end

  return filename
end

------------------------------------------------------------------------------------------------------------------------
local getBestMaleCharacterPhysicsLocation = function()
  return guessOtherFileFromFile(getBestMaleCharacterAnimationLocation(), true)
end

------------------------------------------------------------------------------------------------------------------------
local builtInTemplates = {
  {
    Identifier = "NaturalMotion Humanoid",
    TemplateFile = string.format([[%s\Humanoid.mctmpl]], templateFolder),
    AnimationRig = getBestMaleCharacterAnimationLocation(),
    PhysicsRig = getBestMaleCharacterPhysicsLocation()
  }
  -- put other options here...
  }

------------------------------------------------------------------------------------------------------------------------
-- format and return the default wizard data
local defaultAnimSetData = function()
  local data = { }
  data.ExistingCharacterType = "Animation"
  data.NewCharacterType = "Animation"

  data.AnimationSetName = makeUniqueAnimSetName(preferences.get("DefaultAnimationSetName"))
  data.WizardMode = "New"

  -- only used for existing
  data.SourceXMD = "" -- blank by default, use the project defaults

  -- the location of existing files
  data.ExistingAnimationRig = preferences.get("DefaultAnimationRigFile")
  data.ExistingPhysicsRig = preferences.get("DefaultPhysicsRigFile")

  if data.ExistingPhysicsRig == "" then
    data.ExistingPhysicsRig = guessOtherFileFromFile(data.ExistingAnimationRig, true)
  end

  if data.ExistingAnimationRig == "" then
    data.ExistingAnimationRig = guessOtherFileFromFile(data.ExistingPhysicsRig, true)
  end

  -- the location of new existing files
  data.NewRigSourceScale = "Metres"
  data.NewAnimationRig = ""
  data.NewPhysicsRig = ""

  data.HasTemplate = true

  -- used for templated animation sets
  data.NoTemplate.Trajectory = ""
  data.NoTemplate.Hips = ""

  -- used otherwise
  data.Template.Identifier = "NaturalMotion Humanoid"
  data.Template.TemplateFile = string.format([[%s\Humanoid.mctmpl]], templateFolder)
  data.Template.SourceAnimationRig = getBestMaleCharacterAnimationLocation()
  data.Template.SourcePhysicsRig = getBestMaleCharacterPhysicsLocation()

  data.Template.BuiltInTemplateType = ""

  data.AnimationLocations = { }

  data.ShouldDeleteAnimCreatedAnimSetOnCancel = true

  -- used to determine how far along the process we got.
  data.Result.HasCreatedAnimSet = false
  data.Result.HadAutoMappedTemplate = 0
  data.Result.CreatedAnimationLocation = ""
  data.Result.NewPhysicsRigRootJointName = "" -- autodetected, hopefully.
  data.Result.CreatedAnimSet = ""

  return data
end

------------------------------------------------------------------------------------------------------------------------
-- display a change the animation set name and source type panel
local buildChooseRigType = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:setBorder(3)
    panel:addStaticText{ text = "New animation set name", flags = expand }
    local textbox = panel:addTextBox{
      name = "AnimSetName",
      flags = "expand"
    }

    panel:addVSpacer(20)
     panel:addStaticText{text = "Select rig(s) to use" }
     panel:addRadioBox{
        name = "WizardMode",
        flags = "expand",
        items = {"Create new rig(s)",
                 "Use existing rig(s)",
                 "Specify later (Advanced)"
                 }
      }
      

  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- display a change the animation set name and source type panel
local updateChooseRigType = function(data, panel, validate)
  local validatePanel = function()
    -- simple case, only this function affects the validation
    local valid = data.AnimationSetName ~= ""

    local animSets = listAnimSets()
    for i,v in ipairs(animSets) do
      if v == data.AnimationSetName and v ~= data.Result.CreatedAnimSet then
        valid = false
      end
    end

    validate(valid)
    return valid
  end

  local animSetName = panel:getChild("AnimSetName")
  animSetName:setValue(data.AnimationSetName)
  animSetName:setError(not validatePanel())
  animSetName:setOnChanged(function(self)
    data.AnimationSetName = self:getValue()
    self:setError(not validatePanel())
    validatePanel()
    end)


  local wizardMode = panel:getChild("WizardMode")
  wizardMode:setOnChanged(function(self)
    if self:getSelectedIndex() == 1 then 
       data.WizardMode = "New"
    elseif self:getSelectedIndex() == 2 then
       data.WizardMode = "Existing"
    elseif self:getSelectedIndex() == 3 then 
       data.WizardMode = "SetOnly"
    end
  end)

  if data.WizardMode == "New" then
    wizardMode:setSelectedIndex(1)
  elseif data.WizardMode == "Existing" then
    wizardMode:setSelectedIndex(2)
  elseif data.WizardMode == "SetOnly" then
    wizardMode:setSelectedIndex(3)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- display a source file panel
local buildChooseXMDSourceFile = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:setBorder(3)
    panel:addStaticText{ text = "Select the file you exported from your content creation package" }
    panel:addFilenameControl{
      name      = "XMDSourceFile",
      caption   = "Select exported file",
      wildcard  = { { name = "XMD or FBX Files", extensions = { "xmd", "fbx" } } },
      flags = "expand",
      onMacroize = utils.macroizeString,
      onDemacroize = utils.demacroizeString,
      dialogStyle = "mustExist"
    }

    panel:addVSpacer(20)

    panel:addStaticText{ text = "Select the source units for the input file" }
    addUnitComboBox(panel, "SourceUnits", "expand", 0)
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- display a source file panel
local updateChooseXMDSourceFile = function(data, panel, validate)
  local sourceFile = panel:getChild("XMDSourceFile")
  local sourceUnits = panel:getChild("SourceUnits")

  sourceFile:setValue(data.SourceXMD)
  sourceFile:setOnChanged(function(self)
    data.SourceXMD = utils.macroizeString(self:getValue())
    -- simple case, only this function affects the validation
    validate(fileExists(data.SourceXMD))
    self:setError(not fileExists(data.SourceXMD))
  end)

  sourceUnits:setSelectedItem(data.NewRigSourceScale)
  sourceUnits:setOnChanged(function(self)
    data.NewRigSourceScale = self:getSelectedItem()
  end)

  validate(fileExists(data.SourceXMD))
  sourceFile:setError(not fileExists(data.SourceXMD))
end

------------------------------------------------------------------------------------------------------------------------
-- display a source file panel
local buildChooseCharacterScale = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:setBorder(3)
    panel:addStaticText{ text = "Select the source units for the input file" }
    addUnitComboBox(panel, "SourceUnits", "expand", 0)
  panel:endSizer()
end

-- display a source file panel
local updateChooseCharacterScale = function(data, panel, validate)
  local sourceUnits = panel:getChild("SourceUnits")

  sourceUnits:setSelectedItem(data.NewRigSourceScale)
  sourceUnits:setOnChanged(function(self)
    data.NewRigSourceScale = self:getSelectedItem()
  end)

  validate(true)
end

------------------------------------------------------------------------------------------------------------------------
-- display a morpheme files and rig type panel
local buildChooseExistingMorphemeFilesAndSetType = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:setBorder(3)

    local availableSetTypes = { "Animation" }

    if not mcn.isPhysicsDisabled() then
      table.insert(availableSetTypes, "Physics")
    end

    if not mcn.isEuphoriaDisabled() then
      table.insert(availableSetTypes, "Euphoria")
    end

    if table.getn(availableSetTypes) > 1 then
      panel:addStaticText{ text = "What type of character would you like to create?", flags = "expand" }
      panel:addRadioBox{
        name = "CharacterType",
        flags = "expand",
        items = availableSetTypes
      }

      panel:addVSpacer(20)
    end


    panel:addStaticText{ text = "Animation rig" }
    local animRig = panel:addFilenameControl{
      name      = "AnimationRig",
      caption   = "Pick the animation rig for the new animation set",
      wildcard  = "Animation Rig|mcarig",
      dialogStyle = "mustExist",
      onMacroize = utils.macroizeString,
      onDemacroize = utils.demacroizeString,
      flags = "expand",
    }

    panel:addVSpacer(20)

    if not mcn.isPhysicsDisabled() or mcn.isEuphoriaDisabled() then
      local physicsRigPanel = panel:addPanel{ name = "PhysicsRigPanel", flags = "expand" }
      physicsRigPanel:beginVSizer{ flags = "expand", proportion = 1 }
        physicsRigPanel:setBorder(0)
        physicsRigPanel:addStaticText{ text = "Physics rig" }
        physicsRigPanel:addVSpacer(3)
        physicsRig = physicsRigPanel:addFilenameControl{
          name      = "PhysicsRig",
          caption   = "Pick the physics rig for the new animation set",
          wildcard  = "Physics Rig|mcprig",
          dialogStyle = "mustExist",
          onMacroize = utils.macroizeString,
          onDemacroize = utils.demacroizeString,
          flags = "expand",
        }
      physicsRigPanel:endSizer()
    end
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- display a morpheme files and rig type panel
local updateChooseExistingMorphemeFilesAndSetType = function(data, panel, validate)
  local validatePanel = function()
    local valid = true

    if not fileExists(data.ExistingAnimationRig) then
      valid = false
    end

    if data.ExistingCharacterType ~= "Animation" and not fileExists(data.ExistingPhysicsRig) then
      valid = false
    end

    validate(valid)
  end

  local characterType = panel:getChild("CharacterType")
  local physicsRigPanel = panel:getChild("PhysicsRigPanel")
  if characterType ~= nil then
    if data.ExistingCharacterType == "Euphoria" then
      characterType:setSelectedIndex(3)
    elseif data.ExistingCharacterType == "Physics" then
      characterType:setSelectedIndex(2)
    else
      characterType:setSelectedIndex(1)
    end

    characterType:setOnChanged(function(self)
      data.ExistingCharacterType = self:getSelectedItem()

      if data.ExistingCharacterType ~= "Animation" then
        physicsRigPanel:setShown(true)
      else
        physicsRigPanel:setShown(false)
      end
      -- our size has changes, so re-layout and update.
      panel:getParent():getParent():getParent():doLayout()
      validatePanel()
    end)
  end

  local physicsRig
  local animationRig = panel:getChild("AnimationRig")

  local updateTextBoxes = function()
    animationRig:setValue(data.ExistingAnimationRig)
    animationRig:setError(not fileExists(data.ExistingAnimationRig))

    if physicsRig then
      physicsRig:setValue(data.ExistingPhysicsRig)
      physicsRig:setError(not fileExists(data.ExistingPhysicsRig))
    end

    validatePanel()
  end

  animationRig:setOnChanged(function(self)
      data.ExistingAnimationRig = utils.macroizeString(self:getValue())

      if data.ExistingPhysicsRig == "" then
        data.ExistingPhysicsRig = guessOtherFileFromFile(data.ExistingAnimationRig, true)
      end

      updateTextBoxes()
    end)

  if physicsRigPanel ~= nil then
    physicsRig = physicsRigPanel:getChild("PhysicsRig")
    physicsRig:setOnChanged(function(self)
        data.ExistingPhysicsRig = utils.macroizeString(self:getValue())

        if data.ExistingAnimationRig == "" then
          data.ExistingAnimationRig = guessOtherFileFromFile(data.ExistingPhysicsRig, true)
        end

        updateTextBoxes()
      end)

    physicsRigPanel:setShown(data.ExistingCharacterType ~= "Animation")
  end

  updateTextBoxes()
end

------------------------------------------------------------------------------------------------------------------------
-- display a morpheme files and rig type panel
local buildChooseNewMorphemeFilesAndSetType = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:setBorder(3)

    local availableSetTypes = { "Animation" }

    if not mcn.isPhysicsDisabled() then
      table.insert(availableSetTypes, "Physics")
    end

    if not mcn.isEuphoriaDisabled() then
      table.insert(availableSetTypes, "Euphoria")
    end

    if table.getn(availableSetTypes) > 1 then
      panel:addStaticText{ text = "Character type", flags = "expand" }
      panel:addRadioBox{
        name = "CharacterType",
        flags = "expand",
        items = availableSetTypes
      }

      panel:addVSpacer(20)
    end


    panel:addStaticText{ text = "In which file do you want to save the animation rig?" }
    local animRig = panel:addFilenameControl{
      name      = "AnimationRig",
      caption   = "New animation rig location",
      wildcard  = "Animation Rig|mcarig",
      dialogStyle = "save;prompt",
      onMacroize = utils.macroizeString,
      onDemacroize = utils.demacroizeString,
      flags = "expand"
    }

    panel:addVSpacer(20)

    if not mcn.isPhysicsDisabled() or mcn.isEuphoriaDisabled() then
      local physicsRigPanel = panel:addPanel{ name = "PhysicsRigPanel", flags = "expand" }
      physicsRigPanel:beginVSizer{ flags = "expand", proportion = 1 }
        physicsRigPanel:setBorder(0)
        physicsRigPanel:addStaticText{ text = "In which file do you want to save the physics rig?" }
        physicsRigPanel:addVSpacer(3)
        physicsRig = physicsRigPanel:addFilenameControl{
          name      = "PhysicsRig",
          caption   = "New physics rig location",
          wildcard  = "Physics Rig|mcprig",
          onMacroize = utils.macroizeString,
          onDemacroize = utils.demacroizeString,
          dialogStyle = "save;prompt",
          flags = "expand"
        }
      physicsRigPanel:endSizer()
    end
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- display a morpheme files and rig type panel
local updateChooseNewMorphemeFilesAndSetType = function(data, panel, validate)
  local validatePanel = function()
    local valid = true

    if not checkFileCanBeCreated(data.NewAnimationRig) then
      valid = false
    end

    if data.NewCharacterType ~= "Animation" and not checkFileCanBeCreated(data.NewPhysicsRig) then
      valid = false
    end

    validate(valid)
  end

  local characterType = panel:getChild("CharacterType")
  local physicsRigPanel = panel:getChild("PhysicsRigPanel")
  if characterType ~= nil then
    if data.NewCharacterType == "Euphoria" then
      characterType:setSelectedIndex(3)
    elseif data.NewCharacterType == "Physics" then
      characterType:setSelectedIndex(2)
    else
      characterType:setSelectedIndex(1)
    end

    characterType:setOnChanged(function(self)
      data.NewCharacterType = self:getSelectedItem()

      if data.NewCharacterType ~= "Animation" then
        physicsRigPanel:setShown(true)
      else
        physicsRigPanel:setShown(false)
      end
      -- our size has changes, so re-layout and update.
      panel:getParent():getParent():getParent():doLayout()
      validatePanel()
    end)
  end

  local physicsRig
  local animationRig = panel:getChild("AnimationRig")

  local validateTextBoxes = function()
    animationRig:setError(not checkFileCanBeCreated(data.NewAnimationRig))
    if physicsRig then
      physicsRig:setError(not checkFileCanBeCreated(data.NewPhysicsRig))
    end
    validatePanel()
  end

  local updateTextBoxes = function()
    animationRig:setValue(data.NewAnimationRig)
    if physicsRig then
      physicsRig:setValue(data.NewPhysicsRig)
    end
    validateTextBoxes()
  end

  animationRig:setOnTextChanged(function(self)
    data.NewAnimationRig = utils.macroizeString(self:getValue())
    validateTextBoxes()
  end)

  animationRig:setOnChanged(function(self)
    data.NewAnimationRig = utils.macroizeString(self:getValue())

    if data.NewPhysicsRig == "" then
      data.NewPhysicsRig = guessOtherFileFromFile(data.NewAnimationRig, false)
    end

    updateTextBoxes()
  end)

  if physicsRigPanel ~= nil then
    physicsRig = physicsRigPanel:getChild("PhysicsRig")

    physicsRig:setOnTextChanged(function(self)
      data.NewPhysicsRig = utils.macroizeString(self:getValue())
      validateTextBoxes()
    end)

    physicsRig:setOnChanged(function(self)
      data.NewPhysicsRig = utils.macroizeString(self:getValue())

      if data.NewAnimationRig == "" then
        data.NewAnimationRig = guessOtherFileFromFile(data.NewPhysicsRig, false)
      end

      updateTextBoxes()
    end)

    physicsRigPanel:setShown(data.NewCharacterType ~= "Animation")
  end

  updateTextBoxes()
end

------------------------------------------------------------------------------------------------------------------------
-- display a morpheme files and rig type panel
local buildChooseTemplateOptions = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:setBorder(3)

    panel:addStaticText{
      text = string.format([[
A %s character contains a large number of settings.  By using a template this wizard
can copy the majority of these settings from an existing rig.]], connectAppName),
      flags = "expand"
    }

    panel:beginHSizer{ flags = "expand", proportion = 0 }
      panel:addStaticText{ text = "Preset: " }
      panel:addComboBox{
        name = "Templated",
        flags = "minimalBestSize",
        proportion = 1,
        items = { }
      }
    panel:endSizer()

    panel:addVSpacer(20)

    local templatePanelSwitcher = panel:addSwitchablePanel{ name = "TemplatedSwitcher", flags = "expand" }

    --------------------------------------------------------------
    local hasTemplatePanel = templatePanelSwitcher:addPanel{ name = "Templated", flags = "expand" }
    hasTemplatePanel:beginVSizer{ flags = "expand" }
      hasTemplatePanel:addCheckBox{
        name = "CustomTemplate",
        label = "Customise",
      }

      hasTemplatePanel:addVSpacer(10)

      hasTemplatePanel:addStaticText{ text = "Template file" }
      hasTemplatePanel:addVSpacer(3)
      hasTemplatePanel:addFilenameControl{
        name      = "TemplateFile",
        caption   = "Pick the template file to use",
        wildcard  = "Morpheme Connect template files|mctmpl",
        dialogStyle = "mustExist",
        onMacroize = utils.macroizeString,
        onDemacroize = utils.demacroizeString,
        directory = utils.demacroizeString(templateFolder),
        flags = "expand",
      }

      hasTemplatePanel:addVSpacer(10)

      hasTemplatePanel:addStaticText{ text = "Select the animation rig to copy data from" }
      hasTemplatePanel:addVSpacer(3)
      hasTemplatePanel:addFilenameControl{
        name      = "AnimationRig",
        caption   = "Pick the animation rig for the template to copy from",
        wildcard  = "Animation Rig|mcarig",
        dialogStyle = "mustExist",
        onMacroize = utils.macroizeString,
        onDemacroize = utils.demacroizeString,
        flags = "expand",
      }

      hasTemplatePanel:addVSpacer(10)

      local physicsRigPanel = hasTemplatePanel:addPanel{ name = "PhysicsRigPanel", flags = "expand" }
      physicsRigPanel:setBorder(0)
      physicsRigPanel:beginVSizer{ flags = "expand" }
        physicsRigPanel:addStaticText{ text = "Select the physics rig to copy data from" }
        physicsRigPanel:addVSpacer(3)
        physicsRigPanel:addFilenameControl{
          name      = "PhysicsRig",
          caption   = "Pick the physics rig for the template to copy from",
          wildcard  = "Physics Rig|mcprig",
          dialogStyle = "mustExist",
          onMacroize = utils.macroizeString,
          onDemacroize = utils.demacroizeString,
         flags = "expand",
        }
      physicsRigPanel:endSizer()
    hasTemplatePanel:endSizer()

    --------------------------------------------------------------
    local noTemplatePanel = templatePanelSwitcher:addPanel{ name = "NoTemplate", flags = "expand" }
    noTemplatePanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
    noTemplatePanel:setFlexGridColumnExpandable(2)

      noTemplatePanel:addStaticText{ text = "Character Root" }
      noTemplatePanel:addComboBox{ name = "CharacterRoot", flags = "expand" }

      noTemplatePanel:addStaticText{ text = "Trajectory" }
      noTemplatePanel:addComboBox{ name = "Trajectory", flags = "expand" }

    noTemplatePanel:endSizer()
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- display a morpheme files and rig type panel
local updateChooseTemplateOptions = function(data, panel, validate)
  local validatePanel = function()
    local valid = true

    if data.HasTemplate then
      if data.Template.TemplateFile == "" then
        valid = false
      end

      if not fileExists(data.Template.SourceAnimationRig) then
        valid = false
      end

      if not fileExists(data.Template.SourcePhysicsRig) and data.NewCharacterType ~= "Animation" then
        valid = false
      end
    else
      local markupData = anim.getRigMarkupData(data.Result.CreatedAnimSet)
      valid = markupData.hipIndex ~= 0 and markupData.trajectoryIndex ~= 0
    end

    validate(valid)
  end


  local isTemplated = panel:getChild("Templated")
  local switcher = panel:getChild("TemplatedSwitcher")

  local templatedPanel = switcher:findPanel("Templated")
  local noTemplatePanel = switcher:findPanel("NoTemplate")
  local customTemplate = templatedPanel:getChild("CustomTemplate")

  -- Fill template combo
  local templateItems = { }
  local validatedBuiltInTemplates = { }
  for i,v in ipairs(builtInTemplates) do
    -- validate the default templates...
    if fileExists(v.AnimationRig) and
       (fileExists(v.PhysicsRig) or (data.NewCharacterType == "Animation" and v.PhysicsRig == "")) and
       fileExists(v.TemplateFile) then
       
      -- cant use the new rig as a template...
      if v.AnimationRig ~= data.NewAnimationRig and v.AnimationRig ~= data.NewPhysicsRig and
            v.PhysicsRig ~= data.NewAnimationRig and v.PhysicsRig ~= data.NewPhysicsRig then
        table.insert(templateItems, v.Identifier)
        table.insert(validatedBuiltInTemplates, v)
      end
    end
  end
  
  if table.getn(validatedBuiltInTemplates) == 0 then
    table.insert(templateItems, "Custom template")    
  end
  
  table.insert(templateItems, "Do not use a template")
  isTemplated:setItems(templateItems)

  local updateTemplatedTextBoxes
  local resetTemplatedPanel = function()
    customTemplate:setChecked(true)
    local index = isTemplated:getSelectedIndex()
    if index ~= isTemplated:countItems() then
      local builtIn = validatedBuiltInTemplates[index]
      if builtIn then
        data.Template.Identifier = builtIn.Identifier
        data.Template.TemplateFile = builtIn.TemplateFile
        data.Template.SourceAnimationRig = builtIn.AnimationRig
        data.Template.SourcePhysicsRig = builtIn.PhysicsRig
        customTemplate:setChecked(false)
      end
    end
  end

  local selectCorrectBuiltInTemplate = function()
    isTemplated:setSelectedIndex(1)
    customTemplate:setChecked(true)

    -- Find the template with matching identifier
    for i,v in ipairs(validatedBuiltInTemplates) do
      if v.Identifier == data.Template.Identifier then
        isTemplated:setSelectedIndex(i)
        
        -- Check if it's a custom settings.
        if data.Template.TemplateFile == v.TemplateFile and
           data.Template.SourceAnimationRig == v.AnimationRig and
           data.Template.SourcePhysicsRig == v.PhysicsRig then
          customTemplate:setChecked(false)
        end
      end
    end
  end

  isTemplated:setOnChanged(function(self)
    if self:getSelectedIndex() ~= self:countItems() then
      data.HasTemplate = true
      resetTemplatedPanel()
      updateTemplatedTextBoxes()
      switcher:setCurrentPanel(templatedPanel)
    else
      data.HasTemplate = false
      switcher:setCurrentPanel(noTemplatePanel)
    end

    -- our size has changed, so re-layout and update.
    switcher:bestSizeChanged()
    panel:getParent():getParent():getParent():doLayout()
    validatePanel()
  end)

  -- Select the correct switcher panel
  if data.HasTemplate then
    selectCorrectBuiltInTemplate()
    switcher:setCurrentPanel(templatedPanel)
  else
    isTemplated:setSelectedIndex(isTemplated:countItems())
    switcher:setCurrentPanel(noTemplatePanel)
  end

  -- No Template UI
  local hipsCombo = noTemplatePanel:getChild("CharacterRoot")
  local trajectoryCombo = noTemplatePanel:getChild("Trajectory")

  hipsCombo:setOnChanged(
    function(self)
      local markup = { hipIndex = self:getSelectedIndex() - 1 }
      data.Result.NewPhysicsRigRootJointName = self:getSelectedItem()
      self:setError(markup.hipIndex == 0)
      anim.setRigMarkupData(markup, data.Result.CreatedAnimSet)

      validatePanel()
    end)

  trajectoryCombo:setOnChanged(
    function(self)
      local markup = { trajectoryIndex = self:getSelectedIndex() - 1 }
      self:setError(markup.trajectoryIndex == 0)
      anim.setRigMarkupData(markup, data.Result.CreatedAnimSet)

      validatePanel()
    end)

  local markupData = anim.getRigMarkupData(data.Result.CreatedAnimSet)
  local channelNames = anim.getRigChannelNames(data.Result.CreatedAnimSet)
  channelNames[1] = ""

  hipsCombo:setItems(channelNames)
  hipsCombo:setSelectedIndex(markupData.hipIndex + 1)
  hipsCombo:setError(markupData.hipIndex == 0)

  trajectoryCombo:setItems(channelNames)
  trajectoryCombo:setSelectedIndex(markupData.trajectoryIndex + 1)
  trajectoryCombo:setError(markupData.trajectoryIndex == 0)

  -- Template panel

  local templateFile
  local animationRigFile
  local physicsRigPanel
  local physicsRigFile
  updateTemplatedTextBoxes = function() -- declared local above
    local enableBoxes = customTemplate:getChecked()
    templateFile:setValue(data.Template.TemplateFile)
    templateFile:setError(not fileExists(data.Template.TemplateFile))
    templateFile:enable(enableBoxes)

    animationRigFile:setValue(data.Template.SourceAnimationRig)
    animationRigFile:setError(not fileExists(data.Template.SourceAnimationRig))
    animationRigFile:enable(enableBoxes)

    if physicsRigFile then
      physicsRigFile:setValue(data.Template.SourcePhysicsRig)
      physicsRigFile:setError(not fileExists(data.Template.SourcePhysicsRig))
      physicsRigFile:enable(enableBoxes)
    end

    physicsRigPanel:setShown(data.NewCharacterType ~= "Animation")
    validatePanel()
  end

  customTemplate:setOnChanged(function(self)
    if not self:getChecked() then
      resetTemplatedPanel()
    end
    updateTemplatedTextBoxes()
  end)

  templateFile = templatedPanel:getChild("TemplateFile")
  templateFile:setOnChanged(function(self)
    data.Template.TemplateFile = utils.macroizeString(self:getValue())
    updateTemplatedTextBoxes()
  end)

  animationRigFile = templatedPanel:getChild("AnimationRig")
  animationRigFile:setOnChanged(function(self)
    local newValue = utils.macroizeString(self:getValue())
    if newValue ~= data.NewAnimationRig and newValue ~= data.NewPhysicsRig then
      data.Template.SourceAnimationRig = newValue

      if data.Template.SourcePhysicsRig == "" then
        data.Template.SourcePhysicsRig = guessOtherFileFromFile(data.Template.SourceAnimationRig, true)
      end
    else
      ui.showMessageBox("The template rig cannot be the same as the created rig")
    end

    updateTemplatedTextBoxes()
  end)

  physicsRigPanel = templatedPanel:getChild("PhysicsRigPanel")
  if physicsRigPanel ~= nil then
    physicsRigFile = physicsRigPanel:getChild("PhysicsRig")
    physicsRigFile:setOnChanged(function(self)
      local newValue = utils.macroizeString(self:getValue())
      if newValue ~= data.NewAnimationRig and newValue ~= data.NewPhysicsRig then
        data.Template.SourcePhysicsRig = newValue

        if data.Template.SourceAnimationRig == "" then
          data.Template.SourceAnimationRig = guessOtherFileFromFile(data.Template.SourcePhysicsRig, true)
        end
      else
        ui.showMessageBox("The template rig cannot be the same as the created rig")
      end

      updateTemplatedTextBoxes()
    end)
  end

  -- these cannot be equal.
  if data.Template.SourceAnimationRig == data.NewAnimationRig or data.Template.SourceAnimationRig == data.NewPhysicsRig then
    data.Template.SourceAnimationRig = ""
  end

  if data.Template.SourcePhysicsRig == data.NewAnimationRig or data.Template.SourcePhysicsRig == data.NewPhysicsRig then
    data.Template.SourcePhysicsRig = ""
  end

  updateTemplatedTextBoxes()
end

------------------------------------------------------------------------------------------------------------------------
-- display a morpheme files and rig type panel
local buildChooseTemplateMapping = function(panel)
  panel:beginVSizer{ flags = "expand", proportion = 1 }

    panel:addScrollPanel{
      name = "ScrollPanel",
      flags = "expand;vertical",
      proportion = 1
    }

  -- this UI is dynamic, we build it in update.
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- display a morpheme files and rig type panel
local updateChooseTemplateMapping = function(data, panel, validate)
  local validatePanel = function()
    validate(isAnimationTemplateMappingComplete(animSetData.Result.CreatedAnimSet))
  end

  local mainScrollPanel = panel:getChild("ScrollPanel")
  mainScrollPanel:clear()

  mainScrollPanel:beginVSizer{ flags = "expand", proprtion = 1 }
    populateAnimationMapping(mainScrollPanel, animSetData.Result.CreatedAnimSet, validatePanel, TemplateOnlyRequiredTags, true)
  mainScrollPanel:endSizer()

  validate(isAnimationTemplateMappingComplete(animSetData.Result.CreatedAnimSet))
end

------------------------------------------------------------------------------------------------------------------------
-- display a morpheme files and rig type panel
local buildChooseOptionalTemplateMapping = function(panel)
  panel:beginFlexGridSizer{ cols = 1, flags = "expand", proportion = 1 }
  panel:setFlexGridRowExpandable(1)
  panel:setFlexGridColumnExpandable(1)

    panel:addScrollPanel{
      name = "ScrollPanel",
      flags = "expand;vertical",
      proportion = 1
    }

  -- this UI is dynamic, we build it in update.
  panel:endSizer()
end

-- display a morpheme files and rig type panel
local updateChooseOptionalTemplateMapping = function(data, panel, validate)
  local mainScrollPanel = panel:getChild("ScrollPanel")
  mainScrollPanel:clear()

  mainScrollPanel:beginVSizer{ flags = "expand", proprtion = 1 }
    populateAnimationMapping(mainScrollPanel, animSetData.Result.CreatedAnimSet, validatePanel, TemplateOnlyOptionalTags, true)
  mainScrollPanel:endSizer()

  validate(true)
end

------------------------------------------------------------------------------------------------------------------------
-- display a morpheme files and rig type panel
local buildAnimationLocations = function(panel)
  panel:beginVSizer{ flags = "expand", proprtion = 1 }

    panel:addCheckBox{
      name = "AddAnimaitonLocationCheckBox",
      label = "Add animation location(s)",
      checked = 0,
    }

    panel:setBorder(0)
    local animationPanel = panel:addPanel{ name = "AnimationLocationPanel", flags = "expand" }
    animationPanel:beginVSizer{ flags = "expand" }
      -- the top "toolbar"
      animationPanel:beginFlexGridSizer{ cols = 4, flags = "expand" }
        animationPanel:setFlexGridColumnExpandable(1)
        animationPanel:setBorder(3)

        animationPanel:addHSpacer(2)
        addImageButton(animationPanel, "AddLocationButton", "additem.png")
        animationPanel:addHSpacer(2)
        addImageButton(animationPanel, "RemoveLocationButton", "removeitem.png")
      animationPanel:endSizer()

      -- list control
      local listControl = animationPanel:addListControl{
        name = "AnimationLocationsListControl",
        flags = "expand;rename;vertical",
        columnNames = { "Animation location" }
      }
      listControl:setColumnTruncation(1, "pathElipsis")

      -- configuration of entries
      animationPanel:setBorder(3)
      local configPanel = animationPanel:addPanel{ name = "ConfigPanel", flags = "expand" }
      configPanel:beginVSizer{ flags = "expand" }

        configPanel:addStaticText{ text = " Directory containing source animations" }
        configPanel:addDirectoryControl{
          name = "AnimationSourceDirectoryControl",
          flags = "expand",
          onMacroize = utils.macroizeString,
          onDemacroize = utils.demacroizeString,
        }

        configPanel:beginHSizer{ flags = "expand" }
          configPanel:addHSpacer(2)
          configPanel:addCheckBox{
            name = "AnimationRecursiveCheckBox",
            label = "Add all subdirectories"
          }
        configPanel:endSizer()

        configPanel:addVSpacer(10)
        configPanel:addStaticText{ text = " Directory to output event markup files" }
        configPanel:addDirectoryControl{
          name = "AnimationMarkupDirectoryControl",
          flags = "expand",
          onMacroize = utils.macroizeString,
          onDemacroize = utils.demacroizeString,
        }
      configPanel:endSizer()

    animationPanel:endSizer()

  panel:endSizer()

  panel:doLayout()
end

------------------------------------------------------------------------------------------------------------------------
-- display a morpheme files and rig type panel
local updateAnimationLocations = function(data, panel, validate)
  local animationPanel = panel:getChild("AnimationLocationPanel")

  local listControl = animationPanel:getChild("AnimationLocationsListControl")
  local listControlSize = listControl:getSize()
  listControl:setColumnWidth(1, listControlSize.width -4)
  local updateListControlFromTable = function()
    listControl:clearRows()
    for i,v in ipairs(data.AnimationLocations) do
      listControl:addRow(v.Source)
    end
  end

  local configPanel = animationPanel:getChild("ConfigPanel")
  local source = configPanel:getChild("AnimationSourceDirectoryControl")
  local markup = configPanel:getChild("AnimationMarkupDirectoryControl")
  local recursive = configPanel:getChild("AnimationRecursiveCheckBox")

  local updateTextBoxes = function()
    local index = listControl:getSelectedRow()
    local hasSelection = data.AnimationLocations[index] ~= nil
    if hasSelection then
      source:setValue(data.AnimationLocations[index].Source)
      markup:setValue(data.AnimationLocations[index].Markup)
      recursive:setValue(data.AnimationLocations[index].Recursive)

      source:setError(data.AnimationLocations[index].Source == "")
      markup:setError(data.AnimationLocations[index].Markup == "")
      configPanel:enable(true)
    else
      source:setValue("")
      markup:setValue("")
      recursive:setValue(false)
      configPanel:enable(false)
    end
    source:enable(hasSelection)
    markup:enable(hasSelection)
    recursive:enable(hasSelection)
  end

  local addLocationCheckBox = panel:getChild("AddAnimaitonLocationCheckBox")
  addLocationCheckBox:setOnChanged(function(self)
    local showControls = self:getChecked()
    if showControls then
      data.AnimationLocations = { }
    end
    animationPanel:setShown(showControls)
    panel:getParent():getParent():getParent():doLayout()
  end)


  local showLocations
  local defaultDir = ""
  if data.WizardMode == "New" then
    showLocations = true
    defaultDir = splitFilePath(data.NewAnimationRig)
  else
    showLocations = (table.getn(data.AnimationLocations) ~= 0)
    defaultDir = splitFilePath(data.ExistingAnimationRig)
  end
  defaultDir = utils.demacroizeString(defaultDir)

  addLocationCheckBox:setChecked(showLocations)
  animationPanel:setShown(showLocations)

  animationPanel:getChild("AddLocationButton"):setOnClick(function(self)
    local addDlg = ui.createDirectoryDialog{
      parent = animationPanel,
      caption = "Add Animation Source Location"
    }
    addDlg:setPath(defaultDir)

    if addDlg:show() ~= false then
      local newDir = utils.macroizeString(addDlg:getPath())
      table.insert(data.AnimationLocations, { Source = newDir, Markup = "", Recursive = true })

      updateListControlFromTable()
      listControl:selectRow(table.getn(data.AnimationLocations))

      updateTextBoxes()
    end
  end)

  animationPanel:getChild("RemoveLocationButton"):setOnClick(function(self)
    local index = listControl:getSelectedRow()
    table.remove(data.AnimationLocations, index)
    updateListControlFromTable()
    updateTextBoxes()
  end)

  source:setDefaultDirectory(defaultDir)
  source:setOnChanged(function(self)
    local index = listControl:getSelectedRow()
    self:setValue(utils.macroizeString(self:getValue()))
    data.AnimationLocations[index].Source = self:getValue()
    listControl:setRow(index, data.AnimationLocations[index].Source)
    updateTextBoxes()
  end)

  markup:setDefaultDirectory(defaultDir)
  markup:setOnChanged(function(self)
    local index = listControl:getSelectedRow()
    self:setValue(utils.macroizeString(self:getValue()))
    data.AnimationLocations[index].Markup = self:getValue()
    updateTextBoxes()
  end)

  recursive:setOnChanged(function(self)
    local index = listControl:getSelectedRow()
    data.AnimationLocations[index].Recursive = self:getValue()
  end)

  listControl:setOnSelectionChanged(function(self)
    updateTextBoxes()
  end)

  updateListControlFromTable()
  listControl:selectRow(1)
  updateTextBoxes()
end

-----------------------------------------------------------------------------------------------------------------------
local selectFirstAddedAnimation = function()
  animationAddedCallback = function(resources)
    unregisterEventHandler("mcAnimationFileCreated", animationAddedCallback)
    local k,v = next(resources, nil)
    anim.selectTakeInAssetManager(k)
  end

  registerEventHandler("mcAnimationFileCreated", animationAddedCallback)
end
-- called from lots of places when create is triggered
local onCreateAnimationRig = function(animSetData)
  local ret = true
  undoBlock(function()
    local unit = units.findByName(animSetData.NewRigSourceScale)
    if anim.createRig(animSetData.SourceXMD, animSetData.NewAnimationRig, nil, nil, nil, unit.scaleFactor) then

      -- reset this flag because we just overwrote the rig.
      animSetData.Result.HadAutoMappedTemplate = 0

      if animSetData.Result.HasCreatedAnimSet == false then
        animSetData.Result.HasCreatedAnimSet = true

        -- create the new animation set
        local success, name = createAnimSet{ Name = animSetData.AnimationSetName, Format = preferences.get("DefaultAnimationFormat"), Rig = animSetData.NewAnimationRig }
        animSetData.Result.CreatedAnimSet = name
      else
        anim.setRigPath(animSetData.NewAnimationRig, animSetData.Result.CreatedAnimSet)
      end

      anim.setAnimSetCharacterType(animSetData.Result.CreatedAnimSet, animSetData.NewCharacterType)
      setSelectedAssetManagerAnimSet(animSetData.Result.CreatedAnimSet)
      anim.selectTakeInAssetManager(animSetData.SourceXMD)

      -- set up rig scale
      local unit = units.findByName(animSetData.NewRigSourceScale)
      units.setRigUnit(animSetData.Result.CreatedAnimSet, unit.name)

      local location, filename = splitFilePath(animSetData.SourceXMD)
      anim.setRigPosesLocation(animSetData.Result.CreatedAnimSet, location)

      local take = anim.getSelectedTakeInAssetManager()
      if take ~= nil then
        anim.setRigPose(animSetData.Result.CreatedAnimSet, "DefaultPose", take)
        anim.setRigPose(animSetData.Result.CreatedAnimSet, "GuidePose", take)
      end

      -- we will create the physics rig later, once the user has chosen the hips or template options
    else
      ret = false
    end
  end)
  return ret
end

------------------------------------------------------------------------------------------------------------------------
local onCreateExisting = function(animSetData)
  undoBlock(function()
    if animSetData.Result.HasCreatedAnimSet == true then
      deleteAnimSet(animSetData.Result.CreatedAnimSet)
      animSetData.Result.HasCreatedAnimSet = false
    end

    animSetData.Result.HasCreatedAnimSet = true
    local success, name = createAnimSet{ Name = animSetData.AnimationSetName, Format = preferences.get("DefaultAnimationFormat"), Rig = animSetData.ExistingAnimationRig }
    animSetData.Result.CreatedAnimSet = name

    if animSetData.WizardMode ~= "New"then
      anim.setAnimSetCharacterType(animSetData.Result.CreatedAnimSet, animSetData.ExistingCharacterType)

      if animSetData.ExistingCharacterType ~= "Animation" then
        anim.setPhysicsRigPath(animSetData.ExistingPhysicsRig, animSetData.Result.CreatedAnimSet)
      else
        anim.setPhysicsRigPath("", animSetData.Result.CreatedAnimSet)
      end
    end
  end)
end

------------------------------------------------------------------------------------------------------------------------
local onCreatePhysicsRig = function(animSetData)
  local ret = true
  undoBlock(function()
    if animSetData.NewCharacterType == "Animation" then
      -- no physics rig needed
      return
    end

    if animSetData.Result.HasCreatedAnimSet == false then
      app.warning("Cannot create a physics rig without the animation rig already created, and character root joint specified")
      return
    end

    if animSetData.HasTemplate and anim.createPhysicsRigFromTemplate(animSetData.Result.CreatedAnimSet, animSetData.NewPhysicsRig, animSetData.Template.SourcePhysicsRig, true) then
      anim.setPhysicsRigPath(animSetData.NewPhysicsRig, animSetData.Result.CreatedAnimSet, true)
      validatePhysicsRigAndInformUser(animSetData.Result.CreatedAnimSet)
      return
    end

    if anim.createPhysicsRig(animSetData.Result.CreatedAnimSet, animSetData.Result.NewPhysicsRigRootJointName, animSetData.NewPhysicsRig, true) then
      anim.setPhysicsRigPath(animSetData.NewPhysicsRig, animSetData.Result.CreatedAnimSet, true)
      validatePhysicsRigAndInformUser(animSetData.Result.CreatedAnimSet)
      return
    end
    
    ret = false
  end)
  return ret
end


------------------------------------------------------------------------------------------------------------------------
-- stage predefines, as we ahve to support previous etc...
local onNewSet
local onSourceFileChosen
local onNewRigLocationsChosen
local onAnimationLocations


------------------------------------------------------------------------------------------------------------------------
local clearDlg = function(animSetData, panel)
  if animSetData.Result.CreatedAnimationLocation ~= "" then
    anim.removeAnimationLocation(animSetData.Result.CreatedAnimationLocation)
  end

  panel:hide()
end

------------------------------------------------------------------------------------------------------------------------
local cancelDlg = function(animSetData, panel)
  if animSetData.Result.HasCreatedAnimSet == true and animSetData.ShouldDeleteAnimCreatedAnimSetOnCancel == true then
    deleteAnimSet(animSetData.Result.CreatedAnimSet)
    animSetData.Result.HasCreatedAnimSet = false
  end

  clearDlg(animSetData, panel)
end


------------------------------------------------------------------------------------------------------------------------
-- called when a new animation set (from fbx/xmd) is called and the source file is selected
onAnimationLocations = function(dlg, previous, animSetData)
  -- the callback triggered when next is chosen
  local onNext = function()
    generateSummary = function()
      local summaryText = ""
      local summaryToAppend = function(str)
        summaryText = string.format("%s-  %s\n\n", summaryText, str )
      end
      if animSetData.WizardMode ~= "New" then
        summaryToAppend("Creating an animation set of type \"" .. animSetData.ExistingCharacterType .. "\" called \"" .. animSetData.AnimationSetName .. "\" from existing " .. connectAppName .. "Connect files")

        summaryToAppend("Using animation rig: \"...\\" .. getFilenameWithoutDirectory(animSetData.ExistingAnimationRig) .. "\"")

        if animSetData.ExistingCharacterType ~= "Animation" then
          summaryText = summaryText .. "- Using physics rig: \"...\\" .. getFilenameWithoutDirectory(animSetData.ExistingPhysicsRig) .. "\"\n"
        end
      else
        summaryToAppend("Creating an animation set of type \"" .. animSetData.NewCharacterType .. "\" called \"" .. animSetData.AnimationSetName .. "\"")

        summaryToAppend("Using source file \"...\\".. getFilenameWithoutDirectory(animSetData.SourceXMD) .."\"")

        summaryToAppend("The source file is measured in " .. animSetData.NewRigSourceScale)

        summaryToAppend("Creating animation rig: \"...\\" .. getFilenameWithoutDirectory(animSetData.NewAnimationRig) .. "\"")

        if animSetData.NewCharacterType ~= "Animation" then
          summaryToAppend("Creating physics rig: \"...\\" .. getFilenameWithoutDirectory(animSetData.NewPhysicsRig) .. "\"")
        end

        if not animSetData.HasTemplate then
          summaryToAppend("The new animation set has no template mapping")
        else
          summaryToAppend("The new animation set is using a the template: \"...\\" .. getFilenameWithoutDirectory(animSetData.Template.TemplateFile) .. "\"")

          summaryToAppend("Using template animation rig: \"...\\" .. getFilenameWithoutDirectory(animSetData.Template.SourceAnimationRig) .. "\"")

          if animSetData.Template.SourcePhysicsRig ~= "" then
            summaryToAppend("Using template physics rig: \"...\\" .. getFilenameWithoutDirectory(animSetData.Template.SourcePhysicsRig) .. "\"")
          end
        end
      end
      return summaryText
    end

    local onCreate = function()
      clearDlg(animSetData, dlg)

      -- add animation location the user requested, and hide the dialog.
      if table.getn(animSetData.AnimationLocations) ~= 0 then
        selectFirstAddedAnimation()
      end

      for i,v in ipairs(animSetData.AnimationLocations) do
        anim.addAnimationLocation( v.Source, v.Markup, v.Recursive )
      end
      
      -- none of the anim set wizard should be undoable.
      -- so scrub the undo history.
      local scene = nmx.Application.new():getSceneByName("AssetManager")
      scene:clearUndoHistory()
    end

    messageStage("summary", "Summary", dlg, generateSummary,
    {
      Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
      Previous = { onPress = function() onAnimationLocations(dlg, previous, animSetData) end },
      Create = { onPress = onCreate, disableWhenInvalid = true }
    })
  end

  showWizardStage("animationLocations", animSetData, dlg, buildAnimationLocations, updateAnimationLocations,
    "Animation locations (optional)",
    {
      Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
      Previous = { onPress = previous },
      Next = { onPress = onNext, disableWhenInvalid = true }
    })
end

------------------------------------------------------------------------------------------------------------------------
onRequiredTemplateChosen = function(dlg, previous, animSetData)
  -- the callback triggered when next is chosen
  local onNext = function()
    -- the physics rig is new, there is a 1-1 mapping from physics to anim joints, so we can just copy the mapping across
    pauseStage(dlg, "newPhysicsRigWait", "Executing template algorithms", animSetData, function() cancelDlg(data, dlg) end, function()
      if not onCreatePhysicsRig(animSetData) then
        
        messageStage("errorCreatingPhsyicsRig", "Problem creating physics rig", dlg,
          function()
            return "There was a problem creating the physics rig, please check the log for more details"
          end,
          {
            Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
            Previous = { onPress = function() onRequiredTemplateChosen(dlg, previous, animSetData) end }
          })  
        return
      end
      
      -- update the scene, ensuring the joints world matrices have been updated before we run the template algorithms,
      local app = nmx.Application.new()
      local scene = app:getSceneByName("AssetManager")
      local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())
      
      local hierarchy = nil
      if animSetData.NewCharacterType ~= "Animation" then
        local dataRoot = anim.getPhysicsRigDataRoot(scene, animSetData.Result.CreatedAnimSet)
        local hierarchyType = app:lookupTypeId("HierarchyNode")
        hierarchy = dataRoot:getFirstChild(hierarchyType)
        if hierarchy then
          hierarchy:findAttribute("OutputBindPose"):setBool(true)
        else
          app.error("Couldn't find rig hierarchy node")
        end
      end
      
      scene:update(false)
      
      -- attempt to fill in the optional tags from the user filled in required tags.
      autoMapAnimationTemplateMapping(animSetData.Result.CreatedAnimSet, TemplateOnlyOptionalTags)
      
      -- run the template algorithms
      local algorithms = { "KinematicTags", "BodyGroups" }
      if animSetData.NewCharacterType ~= "Animation" then
        table.insert(algorithms, "PhysicsTags")
        table.insert(algorithms, "CollisionSets")
        if animSetData.NewCharacterType == "Euphoria" then
          table.insert(algorithms, "Limbs")
        end
      end

      local sourceAnimRig = animSetData.Template.SourceAnimationRig
      local sourcePhysicsRig = animSetData.Template.SourcePhysicsRig

      if animSetData.HasTemplate == "BuiltIn" then
        for i,v in ipairs(builtInTemplates) do
          if v.Identifier == animSetData.Template.BuiltInTemplateType then
            sourceAnimRig = v.AnimationRig
            sourcePhysicsRig = v.PhysicsRig
            break
          end
        end
      end

      anim.runAnimSetTemplateAlgorithm(animSetData.Result.CreatedAnimSet, algorithms, sourceAnimRig, sourcePhysicsRig, true)

      -- finally save all changes to the anim and physics rigs we created.
      anim.saveRig(animSetData.Result.CreatedAnimSet)
      if animSetData.NewCharacterType ~= "Animation" then
        anim.savePhysicsRig(animSetData.Result.CreatedAnimSet)
      end
      
      if hierarchy then
        hierarchy:findAttribute("OutputBindPose"):setBool(false)
        scene:update(false)
      end
      scene:endChangeBlock(cbRef, changeBlockInfo("Run template algorithms"))
      
      onAnimationLocations(dlg, function() onRequiredTemplateChosen(dlg, previous, animSetData) end, animSetData)
    end)
  end
  showWizardStage("templateMapping", animSetData, dlg, buildChooseTemplateMapping, updateChooseTemplateMapping,
    "Define the template mappings for the new animation rig",
    {
      Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
      Previous = { onPress = previous },
      Next = { onPress = onNext, disableWhenInvalid = true }
    })
end

------------------------------------------------------------------------------------------------------------------------

onNewRigLocationsChosen = function(dlg, previous, animSetData)
  -- the callback triggered when next is chosen
  local onNext = function()
    if animSetData.HasTemplate then
      local templateFilePath = animSetData.Template.TemplateFile

      anim.setAnimSetTemplate(animSetData.Result.CreatedAnimSet, templateFilePath)
      if animSetData.Result.HadAutoMappedTemplate ~= 1 then
        autoMapAnimationTemplateMapping(animSetData.Result.CreatedAnimSet, TemplateOnlyRequiredTags)
        animSetData.Result.HadAutoMappedTemplate = 1
      end
      onRequiredTemplateChosen(dlg, function() onNewRigLocationsChosen(dlg, previous, animSetData) end, animSetData)
    else
      pauseStage(dlg, "rigLocationsChosenNoTemplate", "Creating physics rig", animSetData, function() cancelDlg(data, dlg) end, function()
        onCreatePhysicsRig(animSetData)

        anim.saveRig(animSetData.Result.CreatedAnimSet)
        if animSetData.NewCharacterType ~= "Animation" then
          anim.savePhysicsRig(animSetData.Result.CreatedAnimSet)
        end

        onAnimationLocations(dlg, function() onNewRigLocationsChosen(dlg, previous, animSetData) end, animSetData)
      end)
    end
  end

  showWizardStage("templateOptions", animSetData, dlg, buildChooseTemplateOptions, updateChooseTemplateOptions,
    "Would you like to base this character on an existing template?",
    {
      Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
      Previous = { onPress = previous },
      Next = { onPress = onNext, disableWhenInvalid = true }
    })
end

------------------------------------------------------------------------------------------------------------------------
-- called when a new animation set (from fbx/xmd) is called and the source file is selected
onSourceFileBuffered = function(dlg, previous, animSetData)
  -- the callback triggered when next is chosen
  local onNext = function()
    -- we need to create the rig here, so we can access it in later steps.
    pauseStage(dlg, "sourceLocationChosen", "Creating animation rig", animSetData, function() cancelDlg(data, dlg) end, function()
      if onCreateAnimationRig(animSetData) then
        onNewRigLocationsChosen(dlg, function() onSourceFileBuffered(dlg, previous, animSetData) end, animSetData)
      else
        messageStage("errorCreatingRig", "Problem creating rig", dlg,
        function()
          return "There was a problem creating the animation rig, please check the log for more details"
        end,
        {
          Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
          Previous = { onPress = function() onSourceFileBuffered(dlg, previous, animSetData) end }
        })
      end
    end)
  end

  showWizardStage("newRigLocations", animSetData, dlg, buildChooseNewMorphemeFilesAndSetType, updateChooseNewMorphemeFilesAndSetType,
    "Where would you like to save the " .. connectAppName .. "Connect rig files?",
    {
      Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
      Previous = { onPress = previous },
      Next = { onPress = onNext, disableWhenInvalid = true }
    })
end


------------------------------------------------------------------------------------------------------------------------
onSourceFileChosen = function(dlg, previous, animSetData)
  -- add the animation location
  local directory, filename = splitFilePath(animSetData.SourceXMD)
  if not anim.isResource(animSetData.SourceXMD) then
    -- display a wait dialog while we wait for the files to buffer.
    showWizardStage("waitingForSourceFileLoad", animSetData, dlg, function(panel) end, function(data, panel, validate) validate(false) end,
      "Loading source file",
      {
        Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
        Previous = { onPress = function() end },
        Next = { onPress = function() end, disableWhenInvalid = true }
      })

    -- the callback triggered when next is chosen
    animationAddedCallback = function(resources)
      if anim.isResource(animSetData.SourceXMD) then
        local take = anim.getTakeName(animSetData.SourceXMD)
        if type(take) == "string" then
          -- success, the specirfied resource was loaded
          unregisterEventHandler("mcAnimationFileCreated", animationAddedCallback)
          onSourceFileBuffered(dlg, function() onSourceFileChosen(dlg, previous, animSetData) end, animSetData)
        end
      end
    end

    registerEventHandler("mcAnimationFileCreated", animationAddedCallback)
    anim.addAnimationLocation(directory, "", false)
    animSetData.Result.CreatedAnimationLocation = directory
  else
    local take = anim.getTakeName(animSetData.SourceXMD)
    if type(take) == "string" then
      -- success, the specirfied resource was loaded
      onSourceFileBuffered(dlg, previous, animSetData)
    else
      app.warning("Couldn't select " .. animSetData.SourceXMD .. " in the asset manager")
    end
  end
end

-----------------------------------------------------------------------------------------------------------------------

-- wizard logic, call this to start the wizard
newRigForSetWizard = function(animationSet, rigSource)
  -- the data we are editing throughout the wizard.
  local animSetData = defaultAnimSetData()

  animSetData.WizardMode = "Rig"
  animSetData.AnimationSetName = animationSet
  animSetData.SourceXMD = rigSource
  animSetData.ShouldDeleteAnimCreatedAnimSetOnCancel = false
  animSetData.HasCreatedAnimSet = true
  animSetData.Result.HasCreatedAnimSet = true
  animSetData.Result.CreatedAnimSet = animationSet

  -- grab the dialog, or create it.
  local dlg = ui.getWindow("NewRigForSetWizard")
  if not dlg then
    dlg = ui.createModalDialog{
      name = "NewRigForSetWizard",
      caption = "New rig for animation set",
      centre = true,
      resize = false
    }
    -- buidl ui which is not torn down each time we show a new stage, or end the wizard and start again
    buildWizardPersistantUI(dlg)
  end

  dlg:setOnClose(function()
    cancelDlg(animSetData, dlg)
  end)

  -- function called to start the wizard off.
  local start
  start = function()
    showWizardStage("chooseCharacterScale", animSetData, dlg, buildChooseCharacterScale, updateChooseCharacterScale,
      "Create a rig from file",
      {
        Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
        Next = { onPress = function()
            onSourceFileChosen(dlg, start, animSetData)
          end,
          disableWhenInvalid = true }
      })
  end

  -- kick off the wizard at the start...
  start()
  dlg:show()
  return animSetData.Result.HasCreatedAnimSet, data.Result.CreatedAnimSet
end

-- wizard logic, call this to start the wizard
newAnimationSetWizard = function()
  -- the data we are editing throughout the wizard.
  local animSetData = defaultAnimSetData()

  -- grab the dialog, or create it.
  local dlg = ui.getWindow("NewAnimationSetWizard")
  if not dlg then
    dlg = ui.createModalDialog{
      name = "NewAnimationSetWizard",
      caption = "New animation set",
      centre = true,
      resize = false
    }
    -- buidl ui which is not torn down each time we show a new stage, or end the wizard and start again
    buildWizardPersistantUI(dlg)
  end

  dlg:setOnClose(function()
    cancelDlg(animSetData, dlg)
  end)

  -- called when a new animation set (from fbx/xmd) is called
  local onNewSet
  local start
  onNewSet = function()
    -- the callback triggered when next is chosen
    local onNext = function()
      onSourceFileChosen(dlg, onNewSet, animSetData)
    end

    showWizardStage("newFiles", animSetData, dlg, buildChooseXMDSourceFile, updateChooseXMDSourceFile,
      "Create new rig(s)",
      {
        Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
        Previous = { onPress = start },
        Next = { onPress = onNext, disableWhenInvalid = true }
      })
  end

  -- called when a new animation set from existing files is called
  onExistingFiles = function()
    -- the callback triggered when next is chosen
    local onNext = function()
      pauseStage(dlg, "existingFilesChosen", "Creating rigs", animSetData, function() cancelDlg(data, dlg) end, function()
        onCreateExisting(animSetData)
        onAnimationLocations(dlg, onExistingFiles, animSetData)
      end)
    end

    showWizardStage("existingFiles", animSetData, dlg, buildChooseExistingMorphemeFilesAndSetType, updateChooseExistingMorphemeFilesAndSetType,
      "Use existing rig(s)",
      {
        Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
        Previous = { onPress = start },
        Next = { onPress = onNext, disableWhenInvalid = true }
      })
  end

  onExistingExit = function()
    undoBlock(function()
      local file = nil
      if fileExists(animSetData.ExistingAnimationRig) then
        file = animSetData.ExistingAnimationRig
      end
      local success, name = createAnimSet{ Name = animSetData.AnimationSetName, Format = preferences.get("DefaultAnimationFormat"), Rig = file }
      animSetData.Result.CreatedAnimSet = name
      anim.setAnimSetCharacterType(animSetData.Result.CreatedAnimSet, "Animation")
      anim.setPhysicsRigPath("", animSetData.Result.CreatedAnimSet)
      animSetData.Result.HasCreatedAnimSet = true
      end )
    
    undoBlock(function()
       setSelectedAssetManagerAnimSet(animSetData.Result.CreatedAnimSet)
       end )
        
    dlg:hide()
    animationSetsAttributeEditor.forceSelectSet(animSetData.Result.CreatedAnimSet)
    animationSetsAttributeEditor.forceAttributeUpdate()
  end
  
  -- function called to start the wizard off.
  start = function()
    -- the callback triggered when next is chosen
    local onNext = function()
      if animSetData.WizardMode == "Existing" then
        onExistingFiles()
      elseif animSetData.WizardMode == "New" then
        onNewSet()
      else
        onExistingExit()
      end
    end

    showWizardStage("start", animSetData, dlg, buildChooseRigType, updateChooseRigType,
      "Create an animation set",
      {
        Cancel = { onPress = function() cancelDlg(animSetData, dlg) end },
        Next = { onPress = onNext, disableWhenInvalid = true }
      })
  end

  -- kick off the wizard at the start...
  start()
  dlg:show()
  return animSetData.Result.HasCreatedAnimSet, data.Result.CreatedAnimSet
end
