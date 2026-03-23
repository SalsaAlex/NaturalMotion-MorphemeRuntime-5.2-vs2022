------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- sample rate help text
local resampleCheckBoxHelp = "Turn off to override the sample rate of the source animation."
local resampleSliderHelp =
[[
Set the rate that the AssetCompiler will resample the source animation during processing.
Specifying values lower than the original FPS of the source animation will result in smaller compressed sizes.
]]
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Add the common controls for changing an animation formats sample rate
-- These options are used by all standard morpheme format types.
------------------------------------------------------------------------------------------------------------------------
addSampleRateFormatOptions = function(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  -- build the format resampling options ui
  panel:beginVSizer{ flags = "expand" }
    panel:beginHSizer{ flags = "expand" }
      local resampleLabel = panel:addStaticText{
        name = "ResampleText",
        text = "Use source sample rate",
        onMouseEnter = function()
          attributeEditor.setHelpText(resampleCheckBoxHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end,
      }

      panel:addHSpacer(5)

      local resampleCheckBox = panel:addCheckBox{
        name = "ResampleCheckBox",
        onMouseEnter = function()
          attributeEditor.setHelpText(resampleCheckBoxHelp)
        end,
        onMouseLeave = function()
          attributeEditor.clearHelpText()
        end,
      }
    panel:endSizer()

    local resampleSlider = panel:addIntSlider{
      name = "ResampleSlider",
      value = 30, min = 1, max = 120,
      flags = "expand",
      proportion = 1,
      onMouseEnter = function()
        attributeEditor.setHelpText(resampleSliderHelp)
      end,
      onMouseLeave = function()
        attributeEditor.clearHelpText()
      end,
    }
  panel:endSizer()

  -- set the update resampling options function
  local updateResamplingOption = function(self)
    local resampling = nil
    if not resampleCheckBox:getChecked() then
      resampling = resampleSlider:getValue()
    end

    for index = 1, getSelectionCount() do
      local options = getOptionsTable(index)
      if type(resampling) == "number" then
        animfmt.setOption(options, "resample", resampling)
      else
        animfmt.removeOption(options, "resample")
      end
      setOptionsTable(index, options)
    end
  end
  resampleCheckBox:setOnChanged(updateResamplingOption)
  resampleSlider:setOnChanged(updateResamplingOption)
end

------------------------------------------------------------------------------------------------------------------------
-- Update the common controls for changing an animation formats sample rate
------------------------------------------------------------------------------------------------------------------------
updateSampleRateFormatOptions = function(panel, getSelectionCount, getOptionsTable, setOptionsTable, shouldEnableControls)
  assert(type(getSelectionCount) == "function")
  assert(type(getOptionsTable) == "function")
  assert(type(setOptionsTable) == "function")
  assert(type(shouldEnableControls) == "function")

  local resampleLabel = panel:getChild("ResampleText")
  local resampleCheckBox = panel:getChild("ResampleCheckBox")
  local resampleSlider = panel:getChild("ResampleSlider")

  local enable = shouldEnableControls()
  resampleLabel:enable(enable)
  resampleCheckBox:enable(enable)

  local options = getOptionsTable(1)

  local samplingValue = nil
  for i, option in ipairs(options) do
    if option.option == "resample" then
      if table.getn(option.args) > 0 then
        samplingValue = tonumber(option.args[1])
      end
    end
  end

  -- set the check state of the resampling check box
  -- set the enable state and slider position for the resampling slider
  local resampling = type(samplingValue) == "number"
  resampleCheckBox:setChecked(not resampling)
  resampleSlider:enable(enable and resampling)
  if resampling then
    resampleSlider:setValue(samplingValue)
  end
end