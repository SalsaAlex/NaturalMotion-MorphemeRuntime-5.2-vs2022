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
-- syncs the nsa dialog ui with the current attribute values
------------------------------------------------------------------------------------------------------------------------
local syncAdvancedNSAOptionsDialog = function(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  local dlg = ui.getWindow("AdvancedNSAOptionsDialog")
  if dlg then
    local framesPerSectionCheckBox = dlg:getChild("framesPerSectionCheckBox")
    local framesPerSectionSlider = dlg:getChild("framesPerSectionSlider")
    local qualityFactorCheckBox = dlg:getChild("qualityFactorCheckBox")
    local qualityFactorSlider = dlg:getChild("qualityFactorSlider")

    local options = getOptionsTable(1)

    local framesPerSection = nil
    local qualityFactor = nil

    for i, option in options do
      if option.option == "mfps" then
        if table.getn(option.args) > 0 then
          framesPerSection = tonumber(option.args[1])
        end
      elseif option.option == "qset" then
        if table.getn(option.args) > 0 then
          qualityFactor = tonumber(option.args[1])
        end
      end
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

    dlg:rebuild()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- edit advanced NSA options
------------------------------------------------------------------------------------------------------------------------
local showAdvancedNSAOptionsDialog = function(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  -- Propagate up a modal dialog for controlling the NSA options.
  local framesPerSectionCheckBox = nil
  local framesPerSectionSlider = nil
  local qualityFactorCheckBox = nil
  local qualityFactorSlider = nil

  local dlg = ui.getWindow("AdvancedNSAOptionsDialog")
  if not dlg then
    dlg = ui.createModalDialog{ name = "AdvancedNSAOptionsDialog", caption = "Advanced NSA options", resize = false, size = { width = 300, height = 220 } }

    dlg:beginVSizer{ flags = "expand" }
      local helptext = nil

      -- Wrap everything in a sizer that can fill out if the window is resized.
      dlg:beginVSizer{ flags = "expand", proportion = 1 }
          dlg:beginFlexGridSizer{ flags = "expand", rows = 2, cols = 2 }
            dlg:setFlexGridColumnExpandable(2)

      -- Max Frames Per Section -----------------------------------------
            local framesPerSectionCheckBoxHelp = "Enable customisation of the maximum number of frames per section whenever the data needs to be sectioned. Sectioning occurs whenever the data is too large to fit into a predetermined byte budget. The default value is set to 30 frames."

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

            local framesPerSectionSliderHelp = "The maximum number of frames per section whenever the data needs to be sectioned. Sectioning occurs whenever the data is too large to fit into a predetermined byte budget. The default value is set to 30 frames."

            dlg:addStaticText{ }

            local minimumFramesPerSectionValue = 2
            local defaultFramesPerSectionValue = 30
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

      -- Quality factor -----------------------------------------
            local qualityFactorCheckBoxHelp = "Enable customisation of the number of quantisation sets that the compressor can use. This value is a fraction of the total number of animation channels, higher values should give better approximation. Typically, at least half of the number of quantisation sets are redundant in any animation."
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

            local qualityFactorSliderHelp = "The number of quantisation sets that the compressor can use. This value is a fraction of the total number of animation channels, higher values should give better approximation. Typically, at least half of the number of quantisation sets are redundant in any animation."

            dlg:addStaticText{ }

            local minimumQualityFactorValue = 0.0
            local defaultQualityFactorValue = 0.25
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

          dlg:endSizer()
      dlg:endSizer()

      -- help text widget
      dlg:beginHSizer{ flags = "expand", proportion = 1 }
        helptext = dlg:addTextControl{ name = "TextBox", flags = "expand", proportion = 1 }
        helptext:setReadOnly(true)
      dlg:endSizer()

      -- ok cancel buttons
      dlg:beginHSizer{ flags = "right" }
        dlg:addButton{
          label = "OK",
          onClick = function() dlg:hide() end,
        }
      dlg:endSizer()

    dlg:endSizer()
  else
    framesPerSectionCheckBox = dlg:getChild("framesPerSectionCheckBox")
    framesPerSectionSlider = dlg:getChild("framesPerSectionSlider")
    qualityFactorCheckBox = dlg:getChild("qualityFactorCheckBox")
    qualityFactorSlider = dlg:getChild("qualityFactorSlider")
  end

  ----------------------------------------------------------------------------------------------------------------------
  -- function called when enabling frames per section or moving the frames per section slider
  ----------------------------------------------------------------------------------------------------------------------
  local updateFramesPerSectionOption = function(self)
    local framesPerSection = nil
    if framesPerSectionCheckBox:getChecked() then
      framesPerSection = framesPerSectionSlider:getValue()
    end

    for i = 1, getSelectionCount() do
      local options = getOptionsTable(i)
      if type(framesPerSection) == "number" then
        animfmt.setOption(options, "mfps", framesPerSection)
      else
        animfmt.removeOption(options, "mfps")
      end
      setOptionsTable(i, options)
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

    for i = 1, getSelectionCount() do
      local options = getOptionsTable(i)
      if type(qualityFactor) == "number" then
        animfmt.setOption(options, "qset", qualityFactor)
      else
        animfmt.removeOption(options, "qset")
      end
      setOptionsTable(i, options)
    end
  end

  qualityFactorCheckBox:setOnChanged(updateQualityFactorOption)
  qualityFactorSlider:setOnChanged(updateQualityFactorOption)

  ----------------------------------------------------------------------------------------------------------------------
  syncAdvancedNSAOptionsDialog(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- add's the NSA specific panel
------------------------------------------------------------------------------------------------------------------------
addFormatOptionsPanelNSA = function(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  addSampleRateFormatOptions(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)

  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:beginHSizer{ flags = "expand" }

      local advancedButton = panel:addButton{
        name = "AdvancedButton",
        label = "Advanced",
        onClick = function()
          showAdvancedNSAOptionsDialog(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
        end,
        onMouseEnter = function()
          attributeEditor.setHelpText("Advanced NSA options")
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end
      }

      advancedButton:enable(shouldEnableControls())

    panel:endSizer()
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- syncs the ui with the current attribute values
------------------------------------------------------------------------------------------------------------------------
updateFormatOptionsPanelNSA = function(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  updateSampleRateFormatOptions(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)

  local advancedButton = panel:getChild("AdvancedButton")
  advancedButton:enable(shouldEnableControls())

  syncAdvancedNSAOptionsDialog(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
end