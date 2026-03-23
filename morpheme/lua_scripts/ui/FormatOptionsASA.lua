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
-- syncs the asa dialog ui with the current attribute values
------------------------------------------------------------------------------------------------------------------------
local syncAdvancedASAOptionsDialog = function(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  local dlg = ui.getWindow("AdvancedASAOptionsDialog")
  if dlg then
    local framesPerSectionCheckBox = dlg:getChild("framesPerSectionCheckBox")
    local framesPerSectionSlider   = dlg:getChild("framesPerSectionSlider")
    local numSubsectionCheckBox    = dlg:getChild("numSubsectionCheckBox")
    local numSubsectionSlider      = dlg:getChild("numSubsectionSlider")

    local options = getOptionsTable(1)

    local framesPerSection = nil
    local numSubsection    = nil

    for i, option in options do
      if option.option == "mfps" then
        if table.getn(option.args) > 0 then
          framesPerSection = tonumber(option.args[1])
        end
      elseif option.option == "nsub" then
        if table.getn(option.args) > 0 then
          numSubsection = tonumber(option.args[1])
        end
      end
    end

    local usingFramesPerSection = type(framesPerSection) == "number"
    framesPerSectionCheckBox:setChecked(usingFramesPerSection)
    framesPerSectionSlider:enable(usingFramesPerSection)
    if usingFramesPerSection then
      framesPerSectionSlider:setValue(framesPerSection)
    end

    local usingNumSubsection = type(numSubsection) == "number"
    numSubsectionCheckBox:setChecked(usingNumSubsection)
    numSubsectionSlider:enable(usingNumSubsection)
    if usingNumSubsection then
      numSubsectionSlider:setValue(numSubsection)
    end

    dlg:rebuild()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- edit advanced ASA options
------------------------------------------------------------------------------------------------------------------------
local showAdvancedASAOptionsDialog = function(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  -- Propogate up a modal dialog for controlling the ASA options.
  local framesPerSectionCheckBox = nil
  local framesPerSectionSlider   = nil
  local numSubsectionCheckBox    = nil
  local numSubsectionSlider      = nil

  local dlg = ui.getWindow("AdvancedASAOptionsDialog")
  if not dlg then
    dlg = ui.createModalDialog{ name = "AdvancedASAOptionsDialog", caption = "Advanced ASA options", resize = false, size = { width = 300, height = 220 } }

    dlg:beginVSizer{ flags = "expand" }
      local helptext = nil

      -- Wrap everything in a sizer that can fill out if the window is resized.
      dlg:beginVSizer{ flags = "expand", proportion = 1 }
          dlg:beginFlexGridSizer{ flags = "expand", rows = 2, cols = 2 }
            dlg:setFlexGridColumnExpandable(2)

      -- Max Frames Per Section -----------------------------------------
            local framesPerSectionCheckBoxHelp = "Enable customisation of the maximum number of frames per section. Reduce this value to potentially increase the quantisation quality of animations with a large range of motion. The default value is set to 60 frames."

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

            local framesPerSectionSliderHelp = "The maximum number of frames per section. Reduce this value to potentially increase the quantisation quality of animations with a large range of motion. The default value is set to 60 frames."

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

      -- Num Subsections -----------------------------------------
            local numSubsectionCheckBoxHelp = "Enable customisation of the number of subsections that the compressor can use. This value is the number of subsections in which each section will be divided. Use higher values when dealing with high bone count animations. The default value is set to 2 subsection per section."
            dlg:addStaticText{
              text = "Number of subsections",
              onMouseEnter = function()
                helptext:setValue(numSubsectionCheckBoxHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            numSubsectionCheckBox = dlg:addCheckBox{
              name = "numSubsectionCheckBox",
              onMouseEnter = function()
                helptext:setValue(numSubsectionCheckBoxHelp)
              end,
              onMouseLeave = function()
                helptext:setValue("")
              end
            }

            local numSubsectionSliderHelp = "Enable customisation of the number of subsections that the compressor can use. This value is the number of subsections in which each section will be divided. Use higher values when dealing with high bone count animations."

            dlg:addStaticText{ }

            local minimumNumSubsectionValue = 2
            local defaultNumSubsectionValue = 2
            local maximumNumSubsectionValue = 30

            numSubsectionSlider = dlg:addIntSlider{
              name = "numSubsectionSlider",
              value = defaultNumSubsectionValue,
              min = minimumNumSubsectionValue,
              max = maximumNumSubsectionValue,
              flags = "expand",
              proportion = 1,
              onMouseEnter = function()
                helptext:setValue(numSubsectionSliderHelp)
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
    framesPerSectionSlider   = dlg:getChild("framesPerSectionSlider")
    numSubsectionCheckBox    = dlg:getChild("numSubsectionCheckBox")
    numSubsectionSlider      = dlg:getChild("numSubsectionSlider")
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
  -- function called when enabling or moving the number of subsections slider
  ----------------------------------------------------------------------------------------------------------------------
  local updateNumSubsectionOption = function(self)
    local numSubsection = nil
    if numSubsectionCheckBox:getChecked() then
      numSubsection = numSubsectionSlider:getValue()
    end

    for i = 1, getSelectionCount() do
      local options = getOptionsTable(i)
      if type(numSubsection) == "number" then
        animfmt.setOption(options, "nsub", numSubsection)
      else
        animfmt.removeOption(options, "nsub")
      end
      setOptionsTable(i, options)
    end
  end

  numSubsectionCheckBox:setOnChanged(updateNumSubsectionOption)
  numSubsectionSlider:setOnChanged(updateNumSubsectionOption)

  syncAdvancedASAOptionsDialog(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
-- add's the ASA specific panel
------------------------------------------------------------------------------------------------------------------------
addFormatOptionsPanelASA = function(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  addSampleRateFormatOptions(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)

  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:beginHSizer{ flags = "expand" }

      local advancedButton = panel:addButton{
        name = "AdvancedButton",
        label = "Advanced",
        onClick = function()
          showAdvancedASAOptionsDialog(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
        end,
      }
      attributeEditor.bindHelpToWidget(advancedButton, "Advanced ASA options.")

      advancedButton:enable(shouldEnableControls())

    panel:endSizer()
  panel:endSizer()
end

------------------------------------------------------------------------------------------------------------------------
-- syncs the ui with the current attribute values
------------------------------------------------------------------------------------------------------------------------
updateFormatOptionsPanelASA = function(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  updateSampleRateFormatOptions(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)

  local advancedButton = panel:getChild("AdvancedButton")
  advancedButton:enable(shouldEnableControls())

  syncAdvancedASAOptionsDialog(getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
end