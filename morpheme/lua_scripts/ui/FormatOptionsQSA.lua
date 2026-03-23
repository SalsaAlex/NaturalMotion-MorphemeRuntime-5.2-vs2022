------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/FormatOptionsCommon.lua"

------------------------------------------------------------------------------------------------------------------------
-- QSA format options panel
------------------------------------------------------------------------------------------------------------------------
local minimumCompressionRate = 1
local defaultCompressionRate = 50
local maximumCompressionRate = 300

local compressionRateCheckBoxHelp = "Turn off to specify the animation compression rate settings."
local compressionRateSliderHelp = "Set the compression rate of the animation (bytes per bone per second)."

------------------------------------------------------------------------------------------------------------------------
-- all the advanced QSA options dialog control helptext
------------------------------------------------------------------------------------------------------------------------
local framesPerKeyCheckBoxHelp = "Enable customisation of the frame rate between curve keyframes. Higher values produce lower quality approximations while allowing greater compression; a value of 6 usually generates a high quality approximation of the animation keyframes."
local framesPerKeySliderHelp = "The animation frame sample rate between curve keyframes. Higher values produce lower quality approximations while allowing greater compression; a value of 6 usually generates a high quality approximation of the animation keyframes."
local framesPerSectionCheckBoxHelp = "Enable customisation of the maximum number of frames per section. Reduce this value to potentially increase the quantisation quality of animations with a large range of motion. The default value is set to 60 frames."
local framesPerSectionSliderHelp = "The maximum number of frames per section. Reduce this value to potentially increase the quantisation quality of animations with a large range of motion. The default value is set to 60 frames."
local qualityFactorCheckBoxHelp = "Enable customisation of the number of quantisation sets that the compressor can use. This value is a fraction of the total number of animation channels, higher values should give better approximation. Typically, at least half of the number of quantisation sets are redundant in any animation."
local qualityFactorSliderHelp = "The number of quantisation sets that the compressor can use. This value is a fraction of the total number of animation channels, higher values should give better approximation. Typically, at least half of the number of quantisation sets are redundant in any animation."
local posChannelCompressionHelp =
[[
Apply user specified compression options to all position channels.

Channel compression methods:
* Quantised position samples (regularly sampled).
* Cubic poly-Bezier splines (lots of single Bezier segments joined together).
]]
local posChannelCompressionMethodHelp =
[[
* Sampled - All position channel samples are uniformly quantised.

* Spline - All position channel samples are approximated by spline curves and the control points subsequently uniformly quantised.
]]
local quatChannelCompressionHelp =
[[
Apply user specified compression options to all rotation channels.

Channel compression methods:
* Quantised rotation vector samples (regularly sampled).
* Cubic poly-Hermite quat splines (lots of single segments joined together).
]]
local quatChannelCompressionMethodHelp =
[[
* Sampled - All rotation channel quat samples are converted to rotation vectors and subsequently uniformly quantised.

* Spline - All rotation channel quat samples are approximated by quat spline curves and the control points subsequently uniformly quantised.
]]

------------------------------------------------------------------------------------------------------------------------
-- Get the QSA compression rate help string text based on a compression rate value
------------------------------------------------------------------------------------------------------------------------
local getQSACompressionHelpString = function(value)

  local helpString = {
    "Very high compression",
    "High compression",
    "Medium compression",
    "Medium quality",
    "Good quality",
    "High quality",
    "Very high quality"
  }

  local rateRanges = {
    0,
    50,
    75,
    100,
    115,
    130,
    200
  }

  local numArgs = table.getn(rateRanges)
  for i = 1, numArgs - 1 do
    if value < rateRanges[i + 1] then
      return helpString[i]
    end
  end
  return helpString[numArgs]

end

------------------------------------------------------------------------------------------------------------------------
-- syncs the qsa dialog ui with the current attribute values
------------------------------------------------------------------------------------------------------------------------
local syncAdvancedQSAOptionsDialog = function(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  local dlg = ui.getWindow("AdvancedQSAOptionsDialog")
  if dlg then
    local framesPerKeyCheckBox = dlg:getChild("framesPerKeyCheckBox")
    local framesPerKeySlider = dlg:getChild("framesPerKeySlider")
    local framesPerSectionCheckBox = dlg:getChild("framesPerSectionCheckBox")
    local framesPerSectionSlider = dlg:getChild("framesPerSectionSlider")
    local qualityFactorCheckBox = dlg:getChild("qualityFactorCheckBox")
    local qualityFactorSlider = dlg:getChild("qualityFactorSlider")
    local posChannelCompressionCheckBox = dlg:getChild("posChannelCompressionCheckBox")
    local posChannelCompressionMethodComboBox = dlg:getChild("posChannelCompressionMethodComboBox")
    local quatChannelCompressionCheckBox = dlg:getChild("quatChannelCompressionCheckBox")
    local quatChannelCompressionMethodComboBox = dlg:getChild("quatChannelCompressionMethodComboBox")

    local options = getOptionsTable(1)

    local framesPerKey = nil
    local framesPerSection = nil
    local qualityFactor = nil
    local posChannelCompressionMethod = nil
    local posChannelCompressionChannel = nil
    local quatChannelCompressionMethod = nil
    local quatChannelCompressionChannel = nil

    for i, option in options do
      if option.option == "fpk" then
        if table.getn(option.args) > 0 then
          framesPerKey = tonumber(option.args[1])
        end
      elseif option.option == "mfps" then
        if table.getn(option.args) > 0 then
          framesPerSection = tonumber(option.args[1])
        end
      elseif option.option == "qset" then
        if table.getn(option.args) > 0 then
          qualityFactor = tonumber(option.args[1])
        end
      elseif option.option == "pmethod" then
        if table.getn(option.args) > 1 then
          posChannelCompressionMethod = option.args[1]
          posChannelCompressionChannel = option.args[2]
        end
      elseif option.option == "qmethod" then
        if table.getn(option.args) > 1 then
          quatChannelCompressionMethod = option.args[1]
          quatChannelCompressionChannel = option.args[2]
        end
      end
    end

    local usingFramesPerKey = type(framesPerKey) == "number"
    framesPerKeyCheckBox:setChecked(usingFramesPerKey)
    framesPerKeySlider:enable(usingFramesPerKey)
    if usingFramesPerKey then
      framesPerKeySlider:setValue(framesPerKey)
    end

    local usingFramesPerSection = type(framesPerSection) == "number"
    framesPerSectionCheckBox:setChecked(usingFramesPerSection)
    framesPerSectionSlider:enable(usingFramesPerSection)
    if usingFramesPerSection then
      framesPerSectionSlider:setValue(framesPerSection)
    end

    local usingQualityFactor = type(qualityFactor) == "number"
    qualityFactorCheckBox:setChecked(usingQualityFactor)
    qualityFactorSlider:enable(usingQualityFactor)
    if usingQualityFactor then
      qualityFactorSlider:setValue(qualityFactor)
    end

    local posChannelCompression = (posChannelCompressionChannel == "*")
    posChannelCompressionCheckBox:setChecked(posChannelCompression)
    posChannelCompressionMethodComboBox:enable(posChannelCompression)
    if posChannelCompression then
      posChannelCompressionMethodComboBox:setSelectedItem(posChannelCompressionMethod)
    end

    local quatChannelCompression = (quatChannelCompressionChannel == "*")
    quatChannelCompressionCheckBox:setChecked(quatChannelCompression)
    quatChannelCompressionMethodComboBox:enable(quatChannelCompression)
    if quatChannelCompression then
      quatChannelCompressionMethodComboBox:setSelectedItem(quatChannelCompressionMethod)
    end

    dlg:rebuild()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- edit advanced QSA options
------------------------------------------------------------------------------------------------------------------------
local showAdvancedQSAOptionsDialog = function(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  -- Propogate up a modal dialog for controlling the QSA options.
  local framesPerKeyCheckBox = nil
  local framesPerKeySlider = nil
  local framesPerSectionCheckBox = nil
  local framesPerSectionSlider = nil
  local qualityFactorCheckBox = nil
  local qualityFactorSlider = nil
  local posChannelCompressionCheckBox = nil
  local posChannelCompressionMethodComboBox = nil
  local quatChannelCompressionCheckBox = nil
  local quatChannelCompressionMethodComboBox = nil

  local dlg = ui.getWindow("AdvancedQSAOptionsDialog")
  if not dlg then
    dlg = ui.createModalDialog{ name = "AdvancedQSAOptionsDialog", caption = "Advanced QSA options", resize = false, size = { width = 300, height = 400 } }

    dlg:beginVSizer{ flags = "expand" }

      local optionsValue = getCommonSubAttributeValue(selection, attribute, "options", set)

      local options = { }
      if type(optionsValue) == "string" and string.len(optionsValue) > 0 then
        options = animfmt.parseOptions(optionsValue)
      end

      local helptext = nil

      -- Wrap everything in a sizer that can fill out if the window is resized.
      dlg:beginVSizer{ flags = "expand", proportion = 2 }
          dlg:beginFlexGridSizer{ flags = "expand", rows = 2, cols = 2 }
            dlg:setFlexGridColumnExpandable(2)

            dlg:addStaticText{
              text = "Curve sample rate",
              onMouseEnter = function()
                helptext:setValue(framesPerKeyCheckBoxHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            framesPerKeyCheckBox = dlg:addCheckBox{
              name = "framesPerKeyCheckBox",
              onMouseEnter = function()
                helptext:setValue(framesPerKeyCheckBoxHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            dlg:addStaticText{ }

            local minimumFramesPerKeyValue = 4
            local defaultFramesPerKeyValue = 6
            local maximumFramesPerKeyValue = 12

            framesPerKeySlider = dlg:addIntSlider{
              name = "framesPerKeySlider",
              value = defaultFramesPerKeyValue,
              min = minimumFramesPerKeyValue,
              max = maximumFramesPerKeyValue,
              flags = "expand",
              proportion = 1,
              onMouseEnter = function()
                helptext:setValue(framesPerKeySliderHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            dlg:addStaticText{
              text = "Maximum frames per section",
              onMouseEnter = function()
                helptext:setValue(framesPerSectionCheckBoxHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            framesPerSectionCheckBox = dlg:addCheckBox{
              name = "framesPerSectionCheckBox",
              onMouseEnter = function()
                helptext:setValue(framesPerSectionCheckBoxHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            dlg:addStaticText{ }

            local minimumFramesPerSectionValue = 2
            local defaultFramesPerSectionValue = 60
            local maximumFramesPerSectionValue = 120

            framesPerSectionSlider = dlg:addIntSlider{
              name = "framesPerSectionSlider",
              value = defaultFramesPerSectionValue,
              min = minimumFramesPerSectionValue,
              max = maximumFramesPerSectionValue,
              flags = "expand",
              proportion = 1,
              onMouseEnter = function()
                helptext:setValue(framesPerSectionSliderHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            dlg:addStaticText{
              text = "Quantisation set quality",
              onMouseEnter = function()
                helptext:setValue(qualityFactorCheckBoxHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            qualityFactorCheckBox = dlg:addCheckBox{
              name = "qualityFactorCheckBox",
              onMouseEnter = function()
                helptext:setValue(qualityFactorCheckBoxHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            dlg:addStaticText{ }

            local minimumQualityFactorValue = 0.0
            local defaultQualityFactorValue = 0.333
            local maximumQualityFactorValue = 1.0

            qualityFactorSlider = dlg:addFloatSlider{
              name = "qualityFactorSlider",
              value = defaultQualityFactorValue,
              min = minimumQualityFactorValue,
              max = maximumQualityFactorValue,
              flags = "expand",
              proportion = 1,
              onMouseEnter = function()
                helptext:setValue(qualityFactorSliderHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            dlg:addStaticText{
              text = "Pos channel compression",
              onMouseEnter = function()
                helptext:setValue(posChannelCompressionHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            posChannelCompressionCheckBox = dlg:addCheckBox{
              name = "posChannelCompressionCheckBox",
              onMouseEnter = function()
                helptext:setValue(posChannelCompressionHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            dlg:addStaticText{ }

            local posChannelCompressionMethods = { "sampled", "spline" }

            posChannelCompressionMethodComboBox = dlg:addComboBox{
              name = "posChannelCompressionMethodComboBox",
              flags = "expand",
              proportion = 1,
              items = posChannelCompressionMethods,
              onMouseEnter = function()
                helptext:setValue(posChannelCompressionMethodHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            dlg:addStaticText{
              text = "Quat channel compression",
              onMouseEnter = function()
                helptext:setValue(quatChannelCompressionHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            quatChannelCompressionCheckBox = dlg:addCheckBox{
              name = "quatChannelCompressionCheckBox",
              onMouseEnter = function()
                helptext:setValue(quatChannelCompressionHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            dlg:addStaticText{ }

            local quatChannelCompressionMethods = { "sampled", "spline" }

            quatChannelCompressionMethodComboBox = dlg:addComboBox{
              name = "quatChannelCompressionMethodComboBox",
              flags = "expand",
              proportion = 1,
              items = quatChannelCompressionMethods,
              onMouseEnter = function()
                helptext:setValue(quatChannelCompressionMethodHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

          dlg:endSizer()
      dlg:endSizer()

      -- accept changes
      local acceptChanges = function()
        dlg:hide()
      end

    -- help text widget
      dlg:beginHSizer{ flags = "expand", proportion = 1 }
        helptext = dlg:addTextControl{ name = "TextBox", flags = "expand", proportion = 1 }
        helptext:setReadOnly(true)
      dlg:endSizer()

      -- ok cancel buttons
      dlg:beginHSizer{ flags = "right" }
        dlg:addButton{ label = "OK", onClick = acceptChanges }
      dlg:endSizer()

    dlg:endSizer()
  else
    framesPerKeyCheckBox = dlg:getChild("framesPerKeyCheckBox")
    framesPerKeySlider = dlg:getChild("framesPerKeySlider")
    framesPerSectionCheckBox = dlg:getChild("framesPerSectionCheckBox")
    framesPerSectionSlider = dlg:getChild("framesPerSectionSlider")
    qualityFactorCheckBox = dlg:getChild("qualityFactorCheckBox")
    qualityFactorSlider = dlg:getChild("qualityFactorSlider")
    posChannelCompressionCheckBox = dlg:getChild("posChannelCompressionCheckBox")
    posChannelCompressionMethodComboBox = dlg:getChild("posChannelCompressionMethodComboBox")
    quatChannelCompressionCheckBox = dlg:getChild("quatChannelCompressionCheckBox")
    quatChannelCompressionMethodComboBox = dlg:getChild("quatChannelCompressionMethodComboBox")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- function called when enabling frames per key or moving the frames per key slider
  ----------------------------------------------------------------------------------------------------------------------
  local updateFramesPerKeyOption = function(self)
    local framesPerKey = nil
    if framesPerKeyCheckBox:getChecked() then
      framesPerKey = framesPerKeySlider:getValue()
    end

    for index = 1, getSelectionCount() do
      local options = getOptionsTable(index)

      if type(framesPerKey) == "number" then
        animfmt.setOption(options, "fpk", framesPerKey)
      else
        animfmt.removeOption(options, "fpk")
      end

      setOptionsTable(index, options)
    end
  end

  framesPerKeyCheckBox:setOnChanged(updateFramesPerKeyOption)
  framesPerKeySlider:setOnChanged(updateFramesPerKeyOption)

  ----------------------------------------------------------------------------------------------------------------------
  -- function called when enabling frames per section or moving the frames per section slider
  ----------------------------------------------------------------------------------------------------------------------
  local updateFramesPerSectionOption = function(self)
    local framesPerSection = nil
    if framesPerSectionCheckBox:getChecked() then
      framesPerSection = framesPerSectionSlider:getValue()
    end

    for index = 1, getSelectionCount() do
      local options = getOptionsTable(index)

      if type(framesPerSection) == "number" then
        animfmt.setOption(options, "mfps", framesPerSection)
      else
        animfmt.removeOption(options, "mfps")
      end

      setOptionsTable(index, options)
    end
  end

  framesPerSectionCheckBox:setOnChanged(updateFramesPerSectionOption)
  framesPerSectionSlider:setOnChanged(updateFramesPerSectionOption)

  ----------------------------------------------------------------------------------------------------------------------
  -- function called when enabling quality factor or moving the quality factor slider
  ----------------------------------------------------------------------------------------------------------------------
  local updateQualityFactorOption = function(self)
    local qualityFactor = nil
    if qualityFactorCheckBox:getChecked() then
      qualityFactor = qualityFactorSlider:getValue()
    end

    for index = 1, getSelectionCount() do
      local options = getOptionsTable(index)

      if type(qualityFactor) == "number" then
        animfmt.setOption(options, "qset", qualityFactor)
      else
        animfmt.removeOption(options, "qset")
      end

      setOptionsTable(index, options)
    end
  end

  qualityFactorCheckBox:setOnChanged(updateQualityFactorOption)
  qualityFactorSlider:setOnChanged(updateQualityFactorOption)

  ----------------------------------------------------------------------------------------------------------------------
  -- function called when enabling pos channel compression or selecting the pos channel compression method
  ----------------------------------------------------------------------------------------------------------------------
  local updatePosChannelCompressionOption = function(self)
    local posChannelCompressionMethod = nil
    if posChannelCompressionCheckBox:getChecked() then
      posChannelCompressionMethod = posChannelCompressionMethodComboBox:getSelectedItem()
    end

    for index = 1, getSelectionCount() do
      local options = getOptionsTable(index)

      -- remove any existing pos channel compression
      animfmt.removeOption(options, "pmethod", "sampled", "*")
      animfmt.removeOption(options, "pmethod", "spline", "*")

      if type(posChannelCompressionMethod) == "string" and string.len(posChannelCompressionMethod) > 0 then
        animfmt.setOption(options, "pmethod", posChannelCompressionMethod, "*")
      end

      setOptionsTable(index, options)
    end
  end

  posChannelCompressionCheckBox:setOnChanged(updatePosChannelCompressionOption)
  posChannelCompressionMethodComboBox:setOnChanged(updatePosChannelCompressionOption)

  ----------------------------------------------------------------------------------------------------------------------
  -- function called when enabling quat channel compression or selecting the quat channel compression method
  ----------------------------------------------------------------------------------------------------------------------
  local updateQuatChannelCompressionOption = function(self)
    local quatChannelCompressionMethod = nil
    if quatChannelCompressionCheckBox:getChecked() then
      quatChannelCompressionMethod = quatChannelCompressionMethodComboBox:getSelectedItem()
    end

    for index = 1, getSelectionCount() do
      local options = getOptionsTable(index)

      -- remove any existing quat channel compression
      animfmt.removeOption(options, "qmethod", "sampled", "*")
      animfmt.removeOption(options, "qmethod", "spline", "*")

      if type(quatChannelCompressionMethod) == "string" and string.len(quatChannelCompressionMethod) > 0 then
        animfmt.setOption(options, "qmethod", quatChannelCompressionMethod, "*")
      end

      setOptionsTable(index, options)
    end
  end
  quatChannelCompressionCheckBox:setOnChanged(updateQuatChannelCompressionOption)
  quatChannelCompressionMethodComboBox:setOnChanged(updateQuatChannelCompressionOption)

  syncAdvancedQSAOptionsDialog(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- add's the QSA specific panel
------------------------------------------------------------------------------------------------------------------------
addFormatOptionsPanelQSA = function(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  addSampleRateFormatOptions(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)

  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:beginHSizer{ flags = "expand" }

      local compressionRateLabel = panel:addStaticText{
        text = "Use default compression rate",
        onMouseEnter = function()
          attributeEditor.setHelpText(compressionRateCheckBoxHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      panel:addHSpacer(5)

      local compressionRateCheckBox = panel:addCheckBox{
        name = "CompressionRateCheckBox",
        flags = "right",
        onMouseEnter = function()
          attributeEditor.setHelpText(compressionRateCheckBoxHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }
    panel:endSizer()

    local compressionRateSlider = panel:addIntSlider{
      name = "CompressionRateSlider",
      value = defaultCompressionRate,
      min = minimumCompressionRate,
      max = maximumCompressionRate,
      flags = "expand",
      proportion = 1,
      onMouseEnter = function()
        attributeEditor.setHelpText(compressionRateSliderHelp)
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end
    }

    panel:beginHSizer{ flags = "expand" }
      local advancedButton = panel:addButton{
        label = "Advanced",
        onClick = function()
          showAdvancedQSAOptionsDialog(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
        end,
        onMouseEnter = function()
          attributeEditor.setHelpText("Advanced QSA options")
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      panel:addHSpacer(5)

      local compressionRateInfoText = panel:addStaticText{
        name = "CompressionRateInfoText",
        flags = "expand",
        proportion = 1
      }
    panel:endSizer()

    compressionRateSlider:setOnChanging(
      function(self)
        local value = self:getValue()
        local helpText = getQSACompressionHelpString(value)
        compressionRateInfoText:setLabel(helpText)
      end
    )
  panel:endSizer()

  ----------------------------------------------------------------------------------------------------------------------
  -- function called when enabling compression or moving the compression rate slider with compression enabled
  ----------------------------------------------------------------------------------------------------------------------
  local updateCompressionRateOption = function(self)
    local compressionRate = nil
    if not compressionRateCheckBox:getChecked() then
      compressionRate = compressionRateSlider:getValue()
    end

    for index = 1, getSelectionCount() do
      local options = getOptionsTable(index)
      if type(compressionRate) == "number" then
        animfmt.setOption(options, "rate", compressionRate)
      else
        animfmt.removeOption(options, "rate")
      end
      setOptionsTable(index, options)
    end
  end

  compressionRateCheckBox:setOnChanged(updateCompressionRateOption)
  compressionRateSlider:setOnChanged(updateCompressionRateOption)
end

------------------------------------------------------------------------------------------------------------------------
-- syncs the ui with the current attribute values
------------------------------------------------------------------------------------------------------------------------
updateFormatOptionsPanelQSA = function(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  -- enable all the panel controls appropriately
  local enable = shouldEnableControls()
  for _, child in ipairs(panel:getChildren()) do
    child:enable(enable)
  end

  -- now override the previous enabling for the sample rate options
  updateSampleRateFormatOptions(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)

  local options = getOptionsTable(1)

  local compressionRate = nil
  for i, option in options do
    if option.option == "rate" then
      if table.getn(option.args) > 0 then
        compressionRate = tonumber(option.args[1])
      end
    end
  end

  local compressionRateCheckBox = panel:getChild("CompressionRateCheckBox")
  local compressionRateSlider = panel:getChild("CompressionRateSlider")
  local compressionRateInfoText = panel:getChild("CompressionRateInfoText")

  -- set the check state of the compression rate check box
  -- set the enable state and position for the compression rate slider
  local usingCompressionRate = type(compressionRate) == "number"
  compressionRateCheckBox:setChecked(not usingCompressionRate)
  compressionRateSlider:enable(enable and usingCompressionRate)
  compressionRateInfoText:enable(enable and usingCompressionRate)

  if usingCompressionRate then
    compressionRateSlider:setValue(compressionRate)
    compressionRateInfoText:setLabel(getQSACompressionHelpString(compressionRate))
  else
    compressionRateInfoText:setLabel("")
  end

  syncAdvancedQSAOptionsDialog(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
end
