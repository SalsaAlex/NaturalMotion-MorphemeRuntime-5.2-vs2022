local normalButtonTypes = { "Previous", "Next", "Create", "Cancel" }

buildWizardPersistantUI = function(panel)
  panel:setSize{ width = 500, height = 390 }
  panel:suspendLayout()
  panel:freeze()
  
  panel:beginFlexGridSizer{ cols = 1, flags = "expand", proportion = 1 }
    panel:setFlexGridColumnExpandable(1)
    panel:setFlexGridRowExpandable(2)
    panel:setBorder(0)
    
    -- set up the header bar with the title returned fromt the display function
    local headerPanel = panel:addPanel{ name = "Header", flags = "expand", proportion = 1 }
    headerPanel:setBackgroundColour("dialogDarkestTint")
    headerPanel:beginHSizer{ flags = "expand", proportion = 1 }
      headerPanel:setBorder(10)
      headerPanel:addStaticText{ name = "Label", flags = "expand;parentBackground", font = "largebold" }
    headerPanel:endSizer()
   
    local contents = panel:addSwitchablePanel{ name = "Contents", flags = "expand" }
    contents:addPanel{ name = "BlankPausePanel", flags = "expand" }
    
    local buttonPanel = panel:addPanel{ name = "Buttons", flags = "expand", proportion = 1 }
    buttonPanel:setBackgroundColour("dialogDarkTint")
    buttonPanel:beginFlexGridSizer{ cols = 1 + table.getn(normalButtonTypes), flags = "expand", proportion = 1 }
      buttonPanel:addVSpacer(1)
      buttonPanel:setFlexGridColumnExpandable(1)
      buttonPanel:setBorder(4)
      for index,butt in ipairs(normalButtonTypes) do
        local newButton = buttonPanel:addButton{
          name = butt,
          label = butt,
          size = { width = 74 }
        }
      end
    buttonPanel:endSizer()
  panel:endSizer()
  
  panel:resumeLayout()
  panel:thaw()
  panel:doLayout()
end

showWizardStage = function(stageName, data, panel, buildFunction, updateFunction, headerString, buttonData)
  panel:suspendLayout()
  panel:freeze()
  
  local label = panel:getChild("Header"):getChild("Label")
  label:setLabel(headerString)
  label:bestSizeChanged()
  
  local contents = panel:getChild("Contents")
  local buttons = panel:getChild("Buttons")
  
  local validateButtons = { }
  
  -- called when the valid state of the panel changes
  local validState = true
  local reValidate = function(valid)
    validState = valid
    for i,v in pairs(validateButtons) do
      v:enable(valid)
    end
  end

  -- Select the blank panel to ensure the page is cleared during the layout freeze
  contents:setCurrentPanel(contents:findPanel("BlankPausePanel"))

  local foundPanel = contents:findPanel(stageName)
  if foundPanel == nil then
    foundPanel = contents:addPanel{ name = stageName, flags = "expand" }
   
    foundPanel:beginFlexGridSizer{ cols = 1, flags = "expand", proportion = 1 }
      foundPanel:setFlexGridColumnExpandable(1)
      foundPanel:setFlexGridRowExpandable(1)
      -- big border around content
      foundPanel:setBorder(10)
      -- Result panel for the actual controls
      local resultContentPanel = foundPanel:addPanel{ name = "Result", flags = "expand", proportion = 1 }
      buildFunction(resultContentPanel)
      
    foundPanel:endSizer()
  end
  
  local resultPanel = foundPanel:getChild("Result")
  
  -- show or hide required buttons...
  for i,v in pairs(normalButtonTypes) do
    local button = buttons:getChild(v)
    if buttonData[v] ~= nil then
      local alwaysDisable = buttonData[v].disabled == true
      
      button:setShown(true)
      button:enable(not alwaysDisable)
      
      if type(buttonData[v].onPress) == "function" then
        local onPress = buttonData[v].onPress
        local needsValid = buttonData[v].disableWhenInvalid
        
        button:setOnClick(function(self)
          -- update first, this ensures the buttons are disabled if required.
          local canPress = not alwaysDisable
          if canPress and needsValid then
            updateFunction(data, resultPanel, reValidate)
            canPress = validState
          end
          
          if canPress then
            onPress(self)
          end
        end)
      elseif type(buttonData[v].onPress) ~= "nil" then
        app.warning("Non-function onPress callback for button ", v, " in ", stageName)
      end
      
      if buttonData[v].disableWhenInvalid == true then
        table.insert(validateButtons, button)
      end
    else
      button:setShown(false)
    end
  end
  buttons:doLayout()
  
  panel:resumeLayout()
  panel:thaw()
  panel:doLayout()
  panel:refresh()

  -- call after buttons added to catch the validity callbacks
  contents:setCurrentPanel(foundPanel)
  updateFunction(data, resultPanel, reValidate)
end

pauseStage = function(dlg, id, text, data, cancelFn, longFn)
  showWizardStage("BlankPausePanel", animSetData, dlg, function(panel) end, function(panel, data, validate) validate(false) end,
    text,
    {
      Cancel = { onPress = cancelFn },
      Previous = { onPress = function() end, disableWhenInvalid = true },
      Next = { onPress = function() end, disableWhenInvalid = true }
    })

  longFn()
end

messageStage = function(name, title, dlg, generateSummaryFn, buttonTable)
  -- display a summary of what the wizard has done
  local buildSummary = function(panel)
    panel:beginVSizer{ flags = "expand", proprtion = 1 }
      local summary = panel:addTextControl{ name = "Summary", flags = "expand;dialogColours", proprtion = 1, size = { height = 230 } }
      summary:setReadOnly(true)
    panel:endSizer()
  end

  local updateSummary = function(data, panel, validate)
    local summary = panel:getChild("Summary")
    
    local summaryText = generateSummaryFn()
    
    summary:setValue(summaryText)
  end
  
  showWizardStage(name, nil, dlg, buildSummary, updateSummary, title, buttonTable)    
end