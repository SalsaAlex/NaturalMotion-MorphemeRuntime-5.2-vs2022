------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- physicsVolumeSummaryCommand.lua
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Lazily create the nmxUtils table.
--
if nmxUtils == nil
then
  nmxUtils = { }
end

------------------------------------------------------------------------------------------------------------------------
-- helper functions
min = function(a, b)
  if a < b then return a end
  return b
end
max = function(a, b)
  if a > b then return a end
  return b
end
round = function(num, precision)
  local mult = math.pow(10, precision)
  return math.floor(num * mult) / mult
end
outputFormat = function(arg)
  return string.format('%f', round(arg, 5));
end

-- Create a lua string which implements the create command.
-- This essentially just delegates to a function.
--
local callbackImplementation = [[
  -- Create a new modal dialog for setting import options.
  local dlg = ui.createModalDialog{ caption = "Physics Summary" }

  -- function to update UI, defined later as it used local variabled defined later
  local updateUI = nil

  -- function to loop through all physics bodies in a list and call a command on them
  local loopThroughSelectionListCallEachPhysicsVolume = function(func, val)
    for i = 1, selectionList:size() do
      local node = selectionList:getNode(i)
      if node:is(nmx.sgTransformNode.ClassTypeId()) then
        local physicsVolume = nmx.PhysicsVolumeNode.getPhysicsVolume(node)

        if physicsVolume ~= nil then
          func(node, physicsVolume, val)
          
        else
        
          local physicsBody = nmx.PhysicsBodyNode.getPhysicsBody(node)
          if physicsBody ~= nil then
            local child = node:getFirstChild()
            while child do
              if child:is(nmx.sgTransformNode.ClassTypeId()) then
                local physicsVolume = nmx.PhysicsVolumeNode.getPhysicsVolume(child)

                if physicsVolume ~= nil then
                  func(child, physicsVolume, val)
                end
              end
              
              child = child:getNextSibling()
            end
          end
        end
        
      end
    end
  end

  -- clamp the min mass of a physics volume node
  clampMinMass = function(node, physicsVolume, minMass) 
    local vol = nmx.PhysicsVolumeNode.getVolume(node)

    if (vol*physicsVolume:getDensity()) < minMass then
      physicsVolume:setDensity(minMass/vol)
    end
  end
  
  -- clamp the max mass of a physics volume node
  clampMaxMass = function(node, physicsVolume, maxMass)
    local vol = nmx.PhysicsVolumeNode.getVolume(node)

    if physicsVolume:getDensity() > maxMass/vol then
      physicsVolume:setDensity(maxMass/vol)
    end
  end
  
  -- clamp the min density of a physics volume node
  clampMinDensity = function(node, physicsVolume, minDensity)

    if physicsVolume:getDensity() < minDensity then
      physicsVolume:setDensity(minDensity)
    end
  end
  
  -- clamp the max density of a physics volume node
  clampMaxDensity = function(node, physicsVolume, maxDensity)

    if physicsVolume:getDensity() > maxDensity then
      physicsVolume:setDensity(maxDensity)
    end
  end

  -- make grid sizer, and add local var controls to it
  dlg:beginFlexGridSizer{ cols = 4 }
    dlg:addStaticText{ text = " " }
    dlg:addStaticText{ text = "Mass" }
    dlg:addStaticText{ text = "Density" }
    dlg:addStaticText{ text = "Volume" }

    dlg:addStaticText{ text = "Maximum" }
    local maxMassTextBox = dlg:addTextBox{ value = "0.00000",
      onEnter = function(self)
        loopThroughSelectionListCallEachPhysicsVolume(clampMaxMass, tonumber(self:getValue()))
        updateUI()
      end
    }
    
    local maxDensityTextBox = dlg:addTextBox{ value = "0.00000",
      onEnter = function(self)
        loopThroughSelectionListCallEachPhysicsVolume(clampMaxDensity, tonumber(self:getValue()))
        updateUI()
      end
    }
    
    local maxVolumeTextBox = dlg:addTextBox{ value = "0.00000" }
    maxVolumeTextBox:setReadOnly(true)

    dlg:addStaticText{ text = "Average" }
    local avMassTextBox = dlg:addTextBox{ value = "0.00000" }
    local avDensityTextBox = dlg:addTextBox{ value = "0.00000" }
    local avVolumeTextBox = dlg:addTextBox{ value = "0.000000" }
    avMassTextBox:setReadOnly(true)
    avDensityTextBox:setReadOnly(true)
    avVolumeTextBox:setReadOnly(true)

    dlg:addStaticText{ text = "Minimum" }
    local minMassTextBox = dlg:addTextBox{ value = "0.00000",
      onEnter = function(self)
        loopThroughSelectionListCallEachPhysicsVolume(clampMinMass, tonumber(self:getValue()))
        updateUI()
      end
    }
    
    local minDensityTextBox = dlg:addTextBox{ value = "0.00000",
      onEnter = function(self)
        loopThroughSelectionListCallEachPhysicsVolume(clampMinDensity, tonumber(self:getValue()))
        updateUI()
      end
    }
    
    local minVolumeTextBox = dlg:addTextBox{ value = "0.000000" }
    minVolumeTextBox:setReadOnly(true)

    dlg:addStaticText{ text = "Total" }
    local totalMassTextBox = dlg:addTextBox{ value = "0.00000" }
    dlg:addStaticText{ text = " " }
    local totalVolumeTextBox = dlg:addTextBox{ value = "0.000000" }
    totalMassTextBox:setReadOnly(true)
    totalVolumeTextBox:setReadOnly(true)

  dlg:endSizer()

  -- re set the update UI function to actually update the newly defined UI
  updateUI = function()

    local totalVol = 0
    local minVol = 99999999999
    local avVol = 0
    local maxVol = -99999999999

    local minDen = 99999999999
    local avDen = 0
    local maxDen = -99999999999

    local totalMass = 0
    local minMass = 99999999999;
    local avMass = 0;
    local maxMass = -99999999999

    local selectionSize = 0

  
    -- clamp the min density of a physics volume node
    local gatherStats = function(node, physicsVolume)
      selectionSize = selectionSize + 1

      local vol = nmx.PhysicsVolumeNode.getVolume(node)
      local density = physicsVolume:getDensity()
      local mass = density*vol

      avDen = avDen + density

      totalVol = totalVol + vol
      totalMass = totalMass + mass

      minVol = min(minVol, vol)
      minDen = min(minDen, density)
      minMass = min(minMass, mass)

      maxVol = max(maxVol, vol)
      maxDen = max(maxDen, density)
      maxMass = max(maxMass, mass)
    end

    loopThroughSelectionListCallEachPhysicsVolume(gatherStats)

    avDen = avDen/selectionSize
    avVol = totalVol/selectionSize
    avMass = totalMass/selectionSize

    maxMassTextBox:setValue(outputFormat(maxMass))
    maxDensityTextBox:setValue(outputFormat(maxDen))
    maxVolumeTextBox:setValue(outputFormat(maxVol))

    avMassTextBox:setValue(outputFormat(avMass))
    avDensityTextBox:setValue(outputFormat(avDen))
    avVolumeTextBox:setValue(outputFormat(avVol))

    minMassTextBox:setValue(outputFormat(minMass))
    minDensityTextBox:setValue(outputFormat(minDen))
    minVolumeTextBox:setValue(outputFormat(minVol))

    totalMassTextBox:setValue(outputFormat(totalMass))
    totalVolumeTextBox:setValue(outputFormat(totalVol))
  end

  updateUI()
  dlg:show()
  dlg = nil
  ]]

-- Create a new callback object with the implementation provided by
-- callbackImplementation.
--
local app = nmx.Application.new()
local scriptManager = app:getScriptManager()
local callback, errorString = scriptManager:createCallback(
  "lua",
  nmx.Application.ScriptedCommandsSignature(),
  callbackImplementation
)

local associatedTypes = nmx.UIntArray.new()
associatedTypes:push_back(nmx.PhysicsVolumeNode.ClassTypeId())

local physicEnabled = not mcn.isPhysicsDisabled or not mcn.isPhysicsDisabled()
if physicEnabled then
  app:registerScriptedCommand(
    "PhysicsTools",
    "Physics Volume Summary",
    "Shows information about physics bodies",
    callback,
    associatedTypes,
    nmx.TransformNode.ClassTypeId()
    )
end
