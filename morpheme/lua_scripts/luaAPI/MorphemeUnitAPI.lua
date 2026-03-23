------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local morphemeUnits = { }

units = { }

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: PAGE
--| name: MorphemeUnitAPI
--| title: Morpheme Units
--| desc:
--|   Lua script functions related to registering and converting between unit types within
--|   morpheme:connect. Primarily used to deal with differing scale between Animation, Physics
--|   and the game world.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean units.add(string name, number scaleFactor, string longname = nil)
--| brief:
--|   Add a new morpheme unit type ie units.add("cm", 0.01, "centimeters") would register centimeters as a unit type.
--|   The value scaleFactor is the multiplier needed to convert from the unit being registered to the internal unit of metres.
--|
--| environments: GlobalEnv
--| page: MorphemeUnitAPI
------------------------------------------------------------------------------------------------------------------------
units.add = function(name, scaleFactor, longname)
  assert(type(morphemeUnits) == "table")

  -- validate the name parameter
  if type(name) ~= "string" then
    return false
  end
  if string.len(name) == 0 then
    return false
  end

  -- validate the scaleFactor parameter
  if type(scaleFactor) ~= "number" then
    return false
  end
  if scaleFactor <= 0.0 then
    return false
  end

  -- validate the optional longname parameter
  if type(longname) ~= "nil" then
    if type(longname) ~= "string" then
      return false
    end
    if string.len(longname) == 0 then
      return false
    end
  end

  -- add the new unit type or overwrite an existing one
  morphemeUnits[name] = {
    name = name,
    scaleFactor = scaleFactor,
    longname = longname,
  }

  return true
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table units.ls()
--| brief:
--|   List all the registered morpheme unit types. The table returned will be of the form
--|   { ["cm"] = { name = "cm", scaleFactor = 0.01, longname = "centimetres" }, ["m"] = { name = "m", scaleFactor = 1, longname = "metres" }, }.
--|
--| environments: GlobalEnv
--| page: MorphemeUnitAPI
------------------------------------------------------------------------------------------------------------------------
units.ls = function()
  return morphemeUnits
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: void units.addDefaults()
--| brief:
--|   Registers all the standard unit types supported by morpheme.
--|
--| environments: GlobalEnv
--| page: MorphemeUnitAPI
------------------------------------------------------------------------------------------------------------------------
units.addDefaults = function()
  -- metric units
  units.add("mm", 0.001, "Millimetres")
  units.add("cm", 0.01, "Centimetres")
  units.add("m", 1.0, "Metres")
  units.add("km", 1000.0, "Kilometres")

  -- imperial units
  units.add("in", 0.0254, "Inches")
  units.add("ft", 0.3048, "Feet")
  units.add("yd", 0.9144, "Yards")
  units.add("mi", 1609.3, "Miles")

  -- other application units
  units.add("XSI", 0.1, "XSI default") -- xsi assumed units are 1 xsi units is 10cm
  units.add("UE3", 0.02, "UE3 physics scale")
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table units.findByScaleFactor(number scaleFactor, number errorTolerance = nil)
--| brief:
--|   Finds a unit type by the scaleFactor need to convert to metres within a specified error tolerance.
--|   If no tolerance is specified then 0.001 is used by default. units.findByScaleFactor(0.001) would
--|   return the unit for millimetres: { name = "mm", scaleFactor = 0.001, longname = "millimetres" }.
--|
--| environments: GlobalEnv
--| page: MorphemeUnitAPI
------------------------------------------------------------------------------------------------------------------------
units.findByScaleFactor = function(scaleFactor, errorTolerance)
  errorTolerance = errorTolerance or 0.001

  for _, unit in pairs(morphemeUnits) do
    if unit.scaleFactor < scaleFactor + errorTolerance and
       unit.scaleFactor > scaleFactor - errorTolerance then
      return unit
    end
  end

  return nil
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table units.findByName(string name)
--| brief:
--|   Finds a unit type by the short or long name it was registered with. units.findByName("mm") would
--|   return the unit for millimetres: { name = "mm", scaleFactor = 0.001, longname = "millimetres" }.
--|
--| environments: GlobalEnv
--| page: MorphemeUnitAPI
------------------------------------------------------------------------------------------------------------------------
units.findByName = function(name)
  for _, unit in pairs(morphemeUnits) do
    if unit.name == name or unit.longname == name then
      return unit
    end
  end

  return nil
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table units.toWorldScale(number scale, string name)
--| brief:
--|   Convert a scalar from the given scale (passed by name or unit table form) to the viewport render scale
--|
--| environments: GlobalEnv
--| page: MorphemeUnitAPI
------------------------------------------------------------------------------------------------------------------------
units.toWorldScale = function(scale, unit)
  if type(unit) == "table" then
    unit = unit.name
  end

  -- the input scalar in meters
  local meterValue = units.convert(scale, unit, "m")

  -- create an nmx distance unit
  local distanceUnit = nmx.Distance.new(meterValue, nmx.Distance.Unit.kMetres)

  -- get its size in the current render units, in the current implementation of connect this will always be meters.
  local currentUnits = nmx.Application.new():getGlobalRoamingSettingsNode():getDistanceUnits()
  return distanceUnit:as(currentUnits)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: number units.convert(number sourceValue, string sourceUnit, string destUnit)
--| brief:
--|   Converts value source units to destination units ie convertMorphemeUnit(10, "cm", "yd")
--|   would convert from 10cms returning the new value in yards.
--|
--| environments: GlobalEnv
--| page: MorphemeUnitAPI
------------------------------------------------------------------------------------------------------------------------
units.convert = function(sourceValue, sourceUnit, destUnit)
  -- check the source unit type exists
  local sourceUnits = morphemeUnits[sourceUnit]
  if type(sourceUnits) ~= "table" then
    app.log(string.format("error convertMorphemeUnit : source unit type '%s' not found", sourceUnit))
    return nil
  end

  -- check the destination unit type exists
  local destUnits = morphemeUnits[destUnit]
  if type(destUnits) ~= "table" then
    app.log(string.format("error convertMorphemeUnit : destination unit type '%s' not found", destUnit))
    return nil
  end

  -- convert to the internal unit of metres
  local internalValue = sourceValue * sourceUnits.scaleFactor
  -- now convert from that unit to the desired unit
  local destValue = internalValue / destUnits.scaleFactor

  return destValue
end

-- check an animation set exists
local animSetExists = function(animationSet)
  local sets = listAnimSets()
  for _, set in ipairs(sets) do
    if set == animationSet then
      return true
    end
  end

  return false
end

-- validate animation set argument
local validateAnimationSetArgument = function(animationSet, funcName)
  if type(animationSet) ~= "string" then
    error(string.format("%s : Invalid parameter 1, expected 'string' got '%s'.", funcName, type(animationSet)))
  end

  if not animSetExists(animationSet) then
    app.log(string.format("%s : invalid animation set '%s' specified.", funcName, animationSet))
    return false
  end

  return true
end

-- validate all arguments to units.setRigScale
local validateSetRigUnitArguments = function(animationSet, unitName, customScaleFactor, funcName)
  if not validateAnimationSetArgument(animationSet, funcName) then
    return false
  end

  if type(unitName) ~= "string" then
    error(string.format("%s : Invalid parameter 2, expected 'string' got '%s'.", funcName, type(unitName)))
  end

  local scaleFactor = 1.0
  if unitName ~= "custom" then
    local u = units.findByName(unitName)
    if u == nil then
      error(string.format("%s : Could not find registered unit with name '%s', to register a custom rig unit use 'custom' unit name.", funcName, unitName))
    end

    scaleFactor = u.scaleFactor
  else
    if type(customScaleFactor) ~= "number" then
      error(string.format("%s : Invalid parameter 3, expected 'number' got '%s'.", funcName, type(customScaleFactor)))
    end

    if customScaleFactor <= 0.0 then
      app.log(string.format("%s : Invalid parameter 3, customScaleFactor must be greater than 0.0.", funcName))
      return false
    end

    scaleFactor = customScaleFactor
  end

  return true, scaleFactor
end

-- validate the root node of a rig and check for the rig info node with the rig scale attribute
local validateRigRoot = function(root, animationSet, funcName, rigType)
  if not root then
    app.log(string.format("%s : no valid %s loaded for animation set '%s'.", funcName, rigType, animationSet))
    return false
  end

  local it = nmx.NodeIterator.new(root, nmx.RigInfoNode.ClassTypeId())
  it:next()
  local rigInfo = it:node()
  if not rigInfo then
    app.log(string.format("%s : %s for animation set '%s' contains no RigInfo node.", funcName, rigType, animationSet))
    return false
  end

  local rigScaleAttr = rigInfo:findAttribute("RigScaleFactor")
  if not rigScaleAttr:isValid() then
    app.log(string.format("%s : RigInfo node for animation set '%s' contains no attribute 'RigScale'.", funcName, animationSet))
    return false
  end

  return true, rigScaleAttr
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: number units.getRigUnit(string animationSet)
--| brief:
--|   Get the unit that an animation rig was authored in so morpheme can work out how to scale
--|   the rig if necessary. Default is meters.
--|
--| environments: GlobalEnv
--| page: MorphemeUnitAPI
------------------------------------------------------------------------------------------------------------------------
units.getRigUnit = function(animationSet)
  if not validateAnimationSetArgument(animationSet, "units.getRigUnit") then
    return nil
  end

  if not anim.isRigValid(animationSet) then
    return nil
  end

  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local animRigRoot = anim.getRigDataRoot(scene, animationSet)

  local valid, animRigScaleAttr = validateRigRoot(animRigRoot, animationSet, "units.getRigUnit", "Rig")
  if not valid then
    return nil
  end

  local scaleFactor = animRigScaleAttr:asFloat()
  local unit = units.findByScaleFactor(scaleFactor)
  if type(unit) == "table" then
    return unit
  end

  return { name = "custom", scaleFactor = scaleFactor }
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: number units.setRigUnit(string animationSet, string unitName, number customScaleFactor = nil)
--| brief:
--|   Set the unit that an animation rig was authored in so morpheme can work out how to scale
--|   the rig if necessary. Default is meters.
--|
--| environments: GlobalEnv
--| page: MorphemeUnitAPI
------------------------------------------------------------------------------------------------------------------------
units.setRigUnit = function(animationSet, unitName, customScaleFactor)
  local valid, scaleFactor = validateSetRigUnitArguments(animationSet, unitName, customScaleFactor, "units.setRigUnit")

  if not valid then
    return false
  end

  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local animRigDataRoot = anim.getRigDataRoot(scene, animationSet)

  local valid, scaleAttr = validateRigRoot(animRigDataRoot, animationSet, "units.setRigUnit", "Rig")
  if not valid then
    return false
  end

  local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

  scaleAttr:setFloat(scaleFactor)

  -- Don't scale the physics rig, it is always in metres

  -- invalid skin attribute shouldn't make the operation fail
  local skinDataRoot = anim.getSkinDataRoot(scene, animationSet)
  valid, scaleAttr = validateRigRoot(skinDataRoot, animationSet, "units.setRigUnit", "Skin")
  if valid then
    scaleAttr:setFloat(scaleFactor)
  end

  scene:endChangeBlock(cbRef, changeBlockInfo("Setting AssetManager RigScaleFactor"))
  return true
end

if not mcn.inCommandLineMode() then
  addUnitComboBox = function(panel, name, flags, proportion)
    local items = { }
    local units = units.ls()
    for k, v in pairs(units) do
      table.insert(items, v.longname or v.name)
    end

    local comboBox = panel:addComboBox{
      name = name,
      flags = "expand",
      proportion = proportion,
      items = items,
    }

    return comboBox
  end
end

units.addDefaults()
