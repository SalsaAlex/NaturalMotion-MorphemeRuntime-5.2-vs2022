------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

nmx = nmx or {}

local nmxApplicationObj = nmx.Application.new()
------------------------------------------------------------------------------------------------------------------------
function nmx.getAttrFloatValue(node, attributeName)
  if node == nil then
    nmxApplicationObj:logError("Nil node passed to getAttrFloatValue()")
  end
   
  local attr = node:findAttribute(attributeName)
    
  if attr:isValid() ~= true then
    nmxApplicationObj:logError("Failed to find float attribute \"" .. attributeName .. "\" on " .. node:getName())
  end
  
  if attr:getTypeId() ~= nmx.AttributeTypeId.kFloat then
    nmxApplicationObj:logError("Unexpected attribute type! Expected attribute \"" .. attributeName .. "\" to be a float")
  end
  
  return attr:asFloat()
end

------------------------------------------------------------------------------------------------------------------------
function nmx.getAttrStringValue(node, attributeName)
  if node == nil then
    nmxApplicationObj:logError("Nil node passed to getAttrFloatValue()")
  end

  local attr = node:findAttribute(attributeName)

  if attr:isValid() ~= true then
    nmxApplicationObj:logError("Failed to find string attribute \"" .. attributeName .. "\" on " .. node:getName())
  end

  local attrTypeId = attr:getTypeId()
  if attrTypeId ~= nmx.AttributeTypeId.kString and attrTypeId ~= nmx.AttributeTypeId.kEnum then
    nmxApplicationObj:logError("Unexpected attribute type! Expected attribute \"" .. attributeName .. "\" to be a string")
  end

  return attr:asString()
end

------------------------------------------------------------------------------------------------------------------------
function nmx.getAttrMatrixValue(node, attributeName)
  if node == nil then
    nmxApplicationObj:logError("Nil node passed to getAttrFloatValue()")
  end

  local attr = node:findAttribute(attributeName)

  if attr:isValid() ~= true then
    nmxApplicationObj:logError("Failed to find matrix attribute \"" .. attributeName .. "\" on " .. node:getName())
  end

  if attr:getTypeId() ~= nmx.AttributeTypeId.kMatrix then
    nmxApplicationObj:logError("Unexpected attribute type! Expected attribute \"" .. attributeName .. "\" to be a matrix")
  end

  return attr:asMatrix()
end

------------------------------------------------------------------------------------------------------------------------
function nmx.getAttrNodeInput(node, attributeName)
  local attr = node:findAttribute(attributeName)
  if not attr:isValid() or not attr:hasInputConnection() then
    nmxApplicationObj:logError("Attribute \"" .. attributeName .. "\" doesn't have an input node")
  end

  return attr:getInput():getNode()
end

------------------------------------------------------------------------------------------------------------------------
-- Converts an nmx2 Vector3 to a table
function nmx.vector3ToTable(v)
  local t = { x = v:getX(), y = v:getY(), z = v:getZ(), w = v:getW(), }
  return t
end

------------------------------------------------------------------------------------------------------------------------
-- Converts an a table to an nmx2 Vector3
function nmx.tableToVector3(t)
  local v = nmx.Vector3.new(t.x, t.y, t.z, t.w)
  return v
end

------------------------------------------------------------------------------------------------------------------------
-- Converts an nmx2 matrix to a table
function nmx.matrixToTable(m)
  local r0 = m:R0()
  local r1 = m:R1()
  local r2 = m:R2()
  local r3 = m:R3()

  local t = {
    r0:getX(), r0:getY(), r0:getZ(), r0:getW(), -- Row 0
    r1:getX(), r1:getY(), r1:getZ(), r1:getW(), -- Row 1
    r2:getX(), r2:getY(), r2:getZ(), r2:getW(), -- Row 2
    r3:getX(), r3:getY(), r3:getZ(), r3:getW()  -- Row 3
  }
  return t
end

------------------------------------------------------------------------------------------------------------------------
-- Converts an a table to an nmx2 matrix
function nmx.tableToMatrix(t)
  local r0 = nmx.Vector3.new( t[1],  t[2],  t[3],  t[4])
  local r1 = nmx.Vector3.new( t[5],  t[6],  t[7],  t[8])
  local r2 = nmx.Vector3.new( t[9], t[10], t[11], t[12])
  local r3 = nmx.Vector3.new(t[13], t[14], t[15], t[16])

  local m = nmx.Matrix.new(r0, r1, r2, r3)
  return m
end

------------------------------------------------------------------------------------------------------------------------
-- Finds a scene by name and returns the scene and its selection list.
-- If no scene with the given name can be found then this function returns nil.
------------------------------------------------------------------------------------------------------------------------
function nmx.getSceneAndSelectionList(sceneName)
  local app = nmx.Application.new()
  local scene = app:getSceneByName(sceneName)

  if scene then
    local selectionList = nmx.SelectionList.new()
    scene:getSelectionList(selectionList)

    return scene, selectionList
  else
    return nil
  end
end
