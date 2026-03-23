------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- ScaleRIg request definition.
------------------------------------------------------------------------------------------------------------------------
registerMessage("ScaleRig",
  {
    helptext = "Orchestrates a shot performance.",
    version = 3,
    id = generateNamespacedId(idNamespaces.NaturalMotion, 104),
    --------------------------------------------------------------------------------------------------------------------
    attributes =
    {
      { name = "AnimationSetHint", input = true, array = false, type = "string" },
      { name = "SpeedScale", input = true, array = false, type = "float", value = 1 },
      { name = "OverallScale", input = true, array = false, type = "float", value = 1 },
      { name = "JointScale", input = true, array = false, type = "floatArray" },
      { name = "AdvancedScale", input = true, array = false, type = "bool", value = false },
    },
    supportsPresets = true,
  presets = {  },

--------------------------------------------------------------------------------------------------------------------
serialize = function(node, stream)

  local isAdvanced = getAttribute(string.format("%s.%s", node, "AdvancedScale"))

  local trajectoryScale = getAttribute(string.format("%s.%s", node, "SpeedScale"))

  stream:writeBool(isAdvanced, "PerJoint")
  if isAdvanced then
    local values = getAttribute(string.format("%s.%s", node, "JointScale"))

    stream:writeFloatArray(values, "PerJointScales")
  else
    local trajectoryScale = getAttribute(string.format("%s.%s", node, "SpeedScale"))
    local overallScale = getAttribute(string.format("%s.%s", node, "OverallScale"))

    stream:writeFloat(trajectoryScale, "SpeedScale")
    stream:writeFloat(overallScale, "CharacterScale")
  end
end,

--------------------------------------------------------------------------------------------------------------------
compareToPreset = function(node, preset)
  local attributesToCompare = {
    "SpeedScale",
    "OverallScale",
    "JointScale",
    "AdvancedScale",
  }

  for i, v in ipairs(attributesToCompare) do
    local nodeValue = getAttribute(node, v)
    local presetValue = getAttribute(preset, v)
    if type(nodeValue) == "table" then
      for i,v in ipairs(nodeValue) do
        if v ~= presetValue[i] then
          return false
        end
      end
    else
      if (nodeValue ~= presetValue) then
        return false
      end
    end
  end
  return true
end,

--------------------------------------------------------------------------------------------------------------------
displayFunction = function(panel, object)
  panel:beginFlexGridSizer{ cols = 1, flags = "expand" }
    panel:setFlexGridColumnExpandable(1)
    panel:setFlexGridRowExpandable(3)

    local overallScaleAttr = string.format("%s.%s", object, "OverallScale")
    local advancedScaleAttr = string.format("%s.%s", object, "AdvancedScale")
    local speedScaleAttr = string.format("%s.%s", object, "SpeedScale")
    local jointScaleAttr = string.format("%s.%s", object, "JointScale")
    local animHintAttr = string.format("%s.%s", object, "AnimationSetHint")

    local multiplyInOverallScale = function()
      local values = getAttribute(jointScaleAttr)
      local overall = getAttribute(overallScaleAttr)


      local set = getAttribute(animHintAttr)
      if set ~= "" then
        local markup = anim.getRigMarkupData()
        if type(markup.trajectoryIndex) == "number" and markup.trajectoryIndex > 0 and markup.trajectoryIndex <= table.getn(values) then
          values[markup.trajectoryIndex+1] = getAttribute(speedScaleAttr)
        end
      end

      setAttribute(jointScaleAttr, values)
    end

    local setGlobalScaleToMax = function()
      local values = getAttribute(jointScaleAttr)
      local maxVal = 1.0

      for i, v in ipairs(values) do
        if i == 1 then
          maxVal = v
        else
          if v > maxVal then
            maxVal = v
          end
        end
      end

      setAttribute(overallScaleAttr, maxVal)

      local set = getAttribute(animHintAttr)
      if set ~= "" then
        local markup = anim.getRigMarkupData(set)

        local values = getAttribute(jointScaleAttr)
        if type(markup.trajectoryIndex) == "number" and markup.trajectoryIndex > 0 and markup.trajectoryIndex < table.getn(values) then
          setAttribute(speedScaleAttr, values[markup.trajectoryIndex+1])
        end
      end
    end

    local updateScaleSection

    panel:beginFlexGridSizer{ cols = 2, flags = "expand" }
      panel:setFlexGridColumnExpandable(2)


      panel:addStaticText{text = "Scaling Mode", flags = "expand", proportion = 1}
      local typeCombo = panel:addComboBox{
        items = { "Basic", "Advanced" },
        flags = "expand",
        onChanged = function(self)
          local isAdvanced = self:getValue() == "Advanced"

          local wasAdvanced = getAttribute(advancedScaleAttr)

          if isAdvanced ~= wasAdvanced then
            if isAdvanced then
              multiplyInOverallScale()
            else
              setGlobalScaleToMax()
            end
          end

          setAttribute(advancedScaleAttr, isAdvanced)
          updateScaleSection(isAdvanced)
        end
      }

      local isAdvanced = getAttribute(advancedScaleAttr)
      if isAdvanced then
        typeCombo:setSelectedItem("Advanced")
      end

    panel:endSizer()
    panel:addVSpacer(10)

    local switch = panel:addSwitchablePanel{ flags = "expand" }

    local simplePanel = switch:addPanel()
    local advancedPanel = switch:addPanel()

  panel:endSizer()
  
  
    updateScaleSection = function(advanced)
      if advanced then
        switch:setCurrentPanel(advancedPanel)
      else
        switch:setCurrentPanel(simplePanel)
      end
      panel:bestSizeChanged()
      panel:rebuild()
    end
    updateScaleSection(isAdvanced)

    simplePanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
      simplePanel:setFlexGridColumnExpandable(2)
      simplePanel:setBorder(3)
      simplePanel:addStaticText{text = "Character Scale", flags = "expand", proportion = 1 }
      local overallScale = simplePanel:addAttributeWidget{
        attributes = { overallScaleAttr },
        flags = "expand"
      }

      simplePanel:addStaticText{text = "Character Speed Scale", flags = "expand", proportion = 1}
      simplePanel:addAttributeWidget{
        attributes = { speedScaleAttr },
        flags = "expand"
      }
    simplePanel:endSizer()

  local addPerBoneUI
  addPerBoneUI = function(perBonePanel)
    perBonePanel:clear()
    perBonePanel:freeze()
    perBonePanel:suspendLayout()
    perBonePanel:beginFlexGridSizer{ cols = 1, flags = "expand" }
      perBonePanel:setFlexGridColumnExpandable(1)
      perBonePanel:setFlexGridRowExpandable(4)
      perBonePanel:setBorder(0)


      local labels = {}
      local canEditPerBone = false
      local setHint = getAttribute(animHintAttr)
      if setHint ~= "" then
        local channelNames = anim.getRigChannelNames(setHint)
        labels = channelNames
        canEditPerBone = true
      else
        local channelNames = anim.getRigChannelNames()
        local values = getAttribute(jointScaleAttr)

        if table.getn(values) == table.getn(channelNames) then
          local channelNames = anim.getRigChannelNames()
          labels = channelNames
          canEditPerBone = true
        end
      end

      perBonePanel:beginHSizer{ flags = "expand" }
        local staticText = perBonePanel:addStaticText{text = "Animation set", flags = "expand", proportion = 1}

        -- padding for flex sizer for main layout
        local animSets = listAnimSets()
        table.insert(animSets, 1, "")

        local combo = perBonePanel:addComboBox{
          items = animSets,
          proportion = 1,
          flags = "expand",
          onChanged = function(self)
            local set = self:getValue()
            if set ~= "" then
              local channelNames = anim.getRigChannelNames(set)


              local values = getAttribute(jointScaleAttr)
              local numberOfChannels = table.getn(channelNames)


              if table.getn(values) < numberOfChannels then
                -- expand the table.

                if table.getn(values) == 0 then
                  local newScaleValue = getAttribute(overallScaleAttr)
                  local speedValue = getAttribute(speedScaleAttr)

                  local markup = anim.getRigMarkupData(set)

                  for i = table.getn(values), (numberOfChannels-1) do

                    if markup.trajectoryIndex ~= i then
                      table.insert(values, newScaleValue)
                    else
                      table.insert(values, speedValue)
                    end
                  end

                else
                  for i = table.getn(values), (numberOfChannels-1) do
                    table.insert(values, 1.0)
                  end
                end
              else
                -- trim the table
                while table.getn(values) > numberOfChannels do
                  table.remove(values)
                end
              end

              undoBlock(function()
                setAttribute(jointScaleAttr, values)
                setAttribute(animHintAttr, self:getValue())
              end)

            end
            
            addPerBoneUI(perBonePanel)
            perBonePanel:bestSizeChanged()
            perBonePanel:rebuild()
          end
        }
      local set = getAttribute(animHintAttr)
      combo:setSelectedItem(set)
      perBonePanel:endSizer()

      if canEditPerBone then
        local values = getAttribute(jointScaleAttr)

        perBonePanel:addVSpacer(10)

        local staticText = perBonePanel:addStaticText{ text = "Per bone scales", font = "bold" }
          
        local scrollPanel = perBonePanel:addScrollPanel{ flags = "expand;vertical" }
        
        scrollPanel:beginVSizer{ flags = "expand" }        

          local widget = scrollPanel:addAttributeWidget{
            attributes = { jointScaleAttr },
            flags = "expand",
            labels = labels,
            showAddRemove = false,
          }
          
        scrollPanel:endSizer()
      end

    perBonePanel:endSizer()
    perBonePanel:resumeLayout()
    perBonePanel:thaw()
    perBonePanel:doLayout()
  end

  addPerBoneUI(advancedPanel)
end,

------------------------------------------------------------------------------------------------------------------------
upgrade = function(node, version, pinLookupTable)

end,
  }
)


------------------------------------------------------------------------------------------------------------------------
-- End of ScaleRig request definition.
------------------------------------------------------------------------------------------------------------------------