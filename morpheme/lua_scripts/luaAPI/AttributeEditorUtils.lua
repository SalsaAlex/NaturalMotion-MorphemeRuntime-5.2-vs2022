------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/LuaUtils.lua"

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean checkSetOrderAndBuildAttributeList(table objects, table attributes)
--| signature: boolean checkSetOrderAndBuildAttributeList(table objects, ...)
--| brief:
--|   Checks to see if a list of attributes are per animation set attributes, and if
--|   these attributes have the same set order and attribute data for the same sets.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
checkSetOrderAndBuildAttributeList = function(selection, ...)
  local attributeNames = arg
  if table.getn(arg) == 1 then
    local t = type(arg[1])
    if t == "table" then
      attributeNames = arg[1]
    end
  end

  local selectionCount = table.getn(selection)
  local attributeNameCount = table.getn(attributeNames)
  local attributeCount = 0

  -- build the list of attributes
  local attributePaths = { }
  for i = 1, attributeNameCount do
    for j = 1, selectionCount do
      attributeCount = attributeCount + 1

      local current = string.format("%s.%s", selection[j], attributeNames[i])
      attributePaths[attributeCount] = current
    end
  end

  -- check for common order first
  if not haveCommonAnimSetOrderAndData(attributePaths) then
    attributeEditor.log("different animation set order detected in selection")
    return nil
  end

  return attributePaths
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table getAttributeValues(table objects, string attribute, string set = nil)
--| brief:
--|   Given a list of objects and an attribute name the function will return the
--|   a list of attribute values, one for each object.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
getAttributeValues = function(objects, attribute, set)
  local values = { }
  for i, object in ipairs(objects) do
    local value = getAttribute(object, attribute, set)
    table.insert(values, value)
  end
  return values
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: object, boolean getCommonAttributeValue(table objects, string attribute, string set = nil)
--| brief:
--|   Given a list of objects and an attribute name the function will return the
--|   common attribute value shared by all the objects. If there are any different
--|   values within the list of objects then nil will be returned, and the second
--|   return value will be false
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
getCommonAttributeValue = function(objects, attribute, set)
  if type(objects) ~= "table" then
    return nil
  end

  if type(attribute) ~= "string" or string.len(attribute) < 1 then
    return nil
  end

  -- early out for one object
  if table.getn(objects) == 1 then
    return getAttribute(objects[1], attribute, set)
  end

  local commonValue = nil
  for i, object in pairs(objects) do
    -- commonValue is not nil then the first attribute has been queried
    if commonValue ~= nil then
      local value = getAttribute(object, attribute, set)
      if commonValue ~= value then
        return nil, false
      end
    else
      commonValue = getAttribute(object, attribute, set)
    end
  end
  return commonValue, true
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean, boolean hasCommonAnimSetData(table objects, string attribute, string set)
--| brief:
--|   Given a list of objects and an attribute name the function will return the
--|   common if all the attributes have animation set data. If there are any different
--|   values within the list of objects then nil will be returned, and the second
--|   return value will be false
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
hasCommonAnimSetData = function(objects, attribute, set)
  if type(objects) ~= "table" then
    return nil
  end

  if type(attribute) ~= "string" or string.len(attribute) < 1 then
    return nil
  end

  -- early out for one object
  if table.getn(objects) == 1 then
    local attrPath = string.format("%s.%s", objects[1], attribute)
    return hasAnimSetData(attrPath, set)
  end

  local commonValue = nil
  for i, object in pairs(objects) do
    -- commonValue is not nil then the first attribute has been queried
    local attrPath = string.format("%s.%s", object, attribute)
    if commonValue ~= nil then
      local value = hasAnimSetData(attrPath, set)
      if commonValue ~= value then
        return nil, false
      end
    else
      commonValue = hasAnimSetData(attrPath, set)
    end
  end
  return commonValue, true
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: object getCommonArrayAttributeValueAtIndex(table objects, string attribute, integer index, string set = nil)
--| brief:
--|   Given a list of objects and an attribute name the function will return the
--|   common attribute value shared by all the array objects at a given index. If there are any different
--|   values within the list of objects (for athe given index) then nil will be returned
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
getCommonArrayAttributeValueAtIndex = function(objects, attribute, index, set)
  local attrValue
  if type(objects) ~= "table" then
    return nil
  end

  if type(attribute) ~= "string" or string.len(attribute) < 1 then
    return nil
  end

  -- early out for one object
  if table.getn(objects) == 1 then
    attrValue = getAttribute(objects[1], attribute, set)
    return attrValue[index]
  end

  local commonValue = nil
  for i, object in pairs(objects) do
    -- commonValue is not nil then the first attribute has been queried
    if commonValue then
      attrValue = getAttribute(object, attribute, set)
      local value = attrValue[index]
      if commonValue ~= value then
        return nil
      end
    else
      attrValue = getAttribute(object, attribute, set)
      commonValue = attrValue[index]
    end
  end
  return commonValue
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: destroyCommonAnimSetData(table objects, string attribute, string set)
--| brief:
--|   Given a list of objects and an attribute name the function will distroy any attribute data
--|   accross all the objects.
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
destroyCommonAnimSetData = function(selection, attribute, set)
  undoBlock(function()
    for i, object in pairs(selection) do
      local attrPath = string.format("%s.%s", object, attribute)
      if hasAnimSetData(attrPath, set) then
        destroyAnimSetData(attrPath, set)
      end
    end
 end)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: setCommonAttributeValue(table objects, string attribute, object value, string set = nil)
--| brief:
--|   Given a list of objects and an attribute name the function will set
--|   the attribute value accross all the objects.
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
setCommonAttributeValue = function(selection, attribute, value, set)
  undoBlock(function()
    for i, object in pairs(selection) do
      setAttribute(string.format("%s.%s", object, attribute), value, set)
    end
 end)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: setCommonArrayAttributeValueAtIndex(table objects, string attribute, object value, string set = nil)
--| brief:
--|   Given a list of objects and an attribute name the function will set
--|   the attribute value at a given index accross all the objects.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
setCommonArrayAttributeValueAtIndex = function(selection, attribute, value, index, set)
  undoBlock(function()
    for i, object in pairs(selection) do
      local currValue = getAttribute(object, attribute, set)
      currValue[index] = value
      setAttribute(string.format("%s.%s", object, attribute), currValue, set)
    end
  end)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: object getCommonSubAttributeValue(table objects, string attribute, string subattribute, string set = nil)
--| brief:
--|   Given a list of objects, an attribute name and a sub attribute name the function
--|   will return the common attribute value shared by all the objects. If there are any
--|   different values within the list of objects then nil will be returned.
--|   eg local commonValue = getCommonSubAttributeValue(selection, "AnimationTake", "filename")
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
getCommonSubAttributeValue = function(objects, attribute, subattribute, set)
  if type(objects) ~= "table" then
    return nil
  end

  if type(attribute) ~= "string" or string.len(attribute) < 1 then
    return nil
  end

  if type(subattribute) ~= "string" or string.len(subattribute) < 1 then
    return nil
  end

  -- early out for one object
  if table.getn(objects) == 1 then
    local attr = getAttribute(objects[1], attribute, set)
    return attr[subattribute]
  end

  local commonValue = nil
  for i, object in pairs(objects) do
    if commonValue then
      local attr = getAttribute(object, attribute, set)
      if commonValue ~= attr[subattribute] then
        return nil
      end
    else
      local attr = getAttribute(object, attribute, set)
      commonValue = attr[subattribute]
      if commonValue == nil then
        return nil
      end
    end
  end
  return commonValue
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: table getCommonAttributes(table objects, table attributes = nil)
--| brief:
--|   Given a list of objects and an optional list of attributes the function will return the
--|   table of common attributes shared by all nodes. If attributes are non nil it will check
--|   every object for common attributes that are also found in the attributes list. If attributes
--|   is nil or an empty table it will use the table of attributes returned by getAttributes(objects[1]).
--|   A common attribute exists for all nodes and must have the same name and type.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
getCommonAttributes = function(objects, attributes)
  -- check the first argument is a table and contain elements
  if type(objects) ~= "table" then
    return { }
  end

  local objectCount = table.getn(objects)
  if objectCount == 0 then
    return { }
  end

  -- store the first object as it will be used frequently
  local firstObject = objects[1]

  -- if the attributes parameter is a table then use this as the list
  -- of attributes to check, if it is not specified or emtpy then use
  -- all the attributes from the first object in the objects list as
  -- a starting basis for finding common attributes
  local attrs = nil
  if type(attributes) == "table" then
    attrs = attributes
  end

  if attrs == nil then
    attrs = listAttributes(firstObject)
  end

  local commonAttributes = { }

  for i, attr in ipairs(attrs) do
    -- check if the first object has the attribute
    if attributeExists(firstObject, attr) then
      -- store the type of the attribute on the first object
      local attrType = getAttributeType(firstObject, attr)

      -- for every other object in the list check that that attribute
      -- exists and is the same type as the first objects attribute of
      -- the same name
      local isCommonAttr = true
      for j = 2, objectCount do
        if attributeExists(objects[j], attr) then
          if attrType ~= getAttributeType(objects[j], attr) then
            -- if the type is different then it is not common
            isCommonAttr = false
            break
          end
        else
          -- if it doesn't exist for this object then it is not common
          isCommonAttr = false
          break
        end
      end

      -- if it was a common attribute add it to the table of common attributes
      if isCommonAttr then
        table.insert(commonAttributes, attr)
      end
    end
  end

  return commonAttributes
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: bindWidgetToArrayAttributeAtIndex(window widget, table selection, string attribute, integer index, string set = nil)
--| brief:
--|   This binds a widget to a particular index of an array attribute. What this means is that
--|   the attribute will be updated when the widget changes, and the widget will be updated when the attribute changes.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
bindWidgetToArrayAttributeAtIndex = function(widget, selection, attribute, index, set)
  local enableContextEvents = true
  local enableUISetAttribute = true
  local isReferencedList = containsReference(selection)

  -- Update Widget from the Attributes
  local updateWidget = function()
    enableUISetAttribute = false
    value = getCommonArrayAttributeValueAtIndex(selection, attribute, index, set)
    if value then
      widget:setIsIndeterminate(false)
      widget:setValue(value)
    else
      widget:setIsIndeterminate(true)
    end
    widget:enable(not isReferencedList)
    enableUISetAttribute = true
  end

  -- Update attributes from a widget
  local updateAttribute = function()
    return function(self)
      if enableUISetAttribute then
        -- prevent the change context callbacks from firing off
        enableContextEvents = false
        setCommonArrayAttributeValueAtIndex(selection, attribute, self:getValue(), index, set)
        enableContextEvents = true
      end
    end
  end

  -- create the change context
  local context = attributeEditor.createChangeContext()
  context:setObjects(selection)
  context:addAttributeChangeEvent(attribute)
  context:setAttributeChangedHandler(updateWidget)

  widget:setOnChanged(updateAttribute(chain.childIndex))

  -- and update the widget so that to start with it displays the current value
  -- and is enabled appropriately
  updateWidget()
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: object getAnimationAttribute(object obj, string attribute, string useDefaultAttribute, string set = nil)
--| brief:
--|   This will return an attribute value for an animation attribute that also appears in the animation
--|   take markup. "attribute" is the name of the attribute that appears both as an attribute in the object
--|   and in the animation markup. "useDefaultAttribute" is the object attribute (boolean) that if true means
--|   that the value will come from the markup, or false if the value in the object should override it
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
function buildGetAnimationAttribute()
  local getAnimationAttribute = function(object, attribute, useDefaultAttribute, set)
    if getAttribute(object, useDefaultAttribute, set) == true then
      local animationTake = getAttribute(object, "AnimationTake", set)
      local animPath = string.format("%s|%s", animationTake.filename, animationTake.takename)
      local defaultValue = anim.getAttribute(string.format("%s.%s", animPath, attribute))
      return defaultValue
    end
    return getAttribute(object, attribute, set)
  end
  return getAnimationAttribute
end
getAnimationAttribute = buildGetAnimationAttribute()

local environment = app.getLuaEnvironment("ValidateSerialize")
app.registerToEnvironment(buildGetAnimationAttribute(), "getAnimationAttribute", environment)

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: object getCommonAnimationAttributeValue(table objects, string attribute, string useDefaultAttribute, string set = nil)
--| brief:
--|   Given a list of objects and an attribute name the function will return the
--|   common animation attribute value shared by all the objects.  "attribute" is the name of the attribute that
--|   appears both as an attribute in the object and in the animation markup. "useDefaultAttribute" is the object
--|   attribute (boolean) that if true means that the value will come from the markup, or false if the value in
--|   the object should override it. If there are any different values within the list of objects then nil will be returned
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
getCommonAnimationAttributeValue = function(objects, attribute, useDefaultAttribute, set)
  if type(objects) ~= "table" then
    return nil
  end

  if type(attribute) ~= "string" or string.len(attribute) < 1 then
    return nil
  end

  -- early out for one object
  if table.getn(objects) == 1 then
    return getAnimationAttribute(objects[1], attribute, useDefaultAttribute, set)
  end

  local commonValue = nil
  for i, object in pairs(objects) do
    -- commonValue is not nil then the first attribute has been queried
    if commonValue ~= nil then
      local value = getAnimationAttribute(object, attribute, useDefaultAttribute, set)
      if commonValue ~= value then
        return nil
      end
    else
      commonValue = getAnimationAttribute(object, attribute, useDefaultAttribute, set)
    end
  end
  return commonValue
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: bindWidgetToAttribute(LuaWindow widget, table selection, string attribute, string set = nil, function setterFunction = nil, function getterFunction = nil)
--| brief:
--|   This binds a widget to an attribute. What this means is that
--|   the attribute will be updated when the widget changes, and the widget will be updated when the attribute changes.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
bindWidgetToAttribute = function(widget, selection, attribute, set, setterFunction, getterFunction)

  ----------------------------------------------------------------------------------------------------------------------
  if setterFunction == nil then
    setterFunction = function(widget, value)
      widget:setIsIndeterminate(value == nil)
      if value ~= nil then
        widget:setValue(value)
      end
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  if getterFunction == nil then
    getterFunction = function(widget)
      return widget:getValue()
    end
  end
  
  ----------------------------------------------------------------------------------------------------------------------
  local enableContextEvents = true
  local enableUISetAttribute = true
  local isReferencedList = containsReference(selection)
  local onChangedHandler = widget:getOnChanged()

  -- Update Widget from the Attributes
  local updateWidget = function(self)
    enableUISetAttribute = false
    setterFunction(widget, getCommonAttributeValue(selection, attribute, set))
    widget:enable(not isReferencedList)
    enableUISetAttribute = true
  end

  -- Update attributes from a widget
  local updateAttribute = function(self)
    return function(self)
      if onChangedHandler ~= nil then
        onChangedHandler(self)
      end

      if enableUISetAttribute then
        -- prevent the change context callbacks from firing off
        enableContextEvents = false
        setCommonAttributeValue(selection, attribute, getterFunction(self), set)
        enableContextEvents = true
      end
    end
  end

  -- create the change context
  local context = attributeEditor.createChangeContext()
  context:setObjects(selection)
  context:addAttributeChangeEvent(attribute)
  context:setAttributeChangedHandler(updateWidget)

  widget:setOnChanged(updateAttribute())

  -- and update the widget so that to start with it displays the current value
  -- and is enabled appropriately
  updateWidget()
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: bindWidgetEnablingToAttribute(LuaWindow widget, table selection, string attribute, string set = nil)
--| brief:
--|   This binds the enabling of a widget to a Boolean attribute.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
bindWidgetEnablingToAttribute = function(widget, selection, attribute, set)
 
  ----------------------------------------------------------------------------------------------------------------------
  local isReferencedList = containsReference(selection)

  -- Update Widget from the Attributes
  local updateWidget = function(self)
    local value= getCommonAttributeValue(selection, attribute, set)
    local doEnable = (not isReferencedList) and (value ~= nil) and value
    widget:enable(doEnable)
  end

  -- create the change context
  local context = attributeEditor.createChangeContext()
  context:setObjects(selection)
  context:addAttributeChangeEvent(attribute)
  context:setAttributeChangedHandler(updateWidget)

  -- and update the widget so that to start with it displays the current value
  -- and is enabled appropriately
  updateWidget()
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: bindWidgetToAttributeAsPercentage(LuaWindow widget, table selection, string attribute, string set = nil)
--| brief:
--|   This binds a widget to an attribute, displaying it as a percentage. 
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
bindWidgetToAttributeAsPercentage = function(widget, selection, attribute, set)

  local setterFunction = function(widget, value)
    widget:setIsIndeterminate(value == nil)
    if value ~= nil then
      widget:setValue(value * 100)
    end
  end
  
  local getterFunction = function(widget)
    return widget:getValue() / 100
  end
  
  bindWidgetToAttribute(widget, selection, attribute, set, setterFunction, getterFunction)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: bindWidgetToAnimationAttribute(object widgets, table selection, string attribute, string useDefaultAttribute, string set = nil)
--| brief:
--|  This binds a widget, or table of widgets, and possibly labels to an animation attribute. What this means is that
--|  the attribute will be updated when the widget changes, and the widget will be updated when the attribute changes.
--|  The widget will be disabled if the default value from the animation markup is used. In the case that there are
--|  one or more labels that need to be disabled pass a table of widgets. The "main widget" is expected to be the first
--|  item in the table.
--|
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
bindWidgetToAnimationAttribute = function(widgets, selection, attribute, useDefaultAttribute, set)
  local enableContextEvents = true
  local enableUISetAttribute = true
  local isReferencedList = containsReference(selection)

  -- get the main widget
  local mainWidget
  if type(widgets) == "table" then
    mainWidget = widgets[1]
  else
    mainWidget = widgets
    widgets = { mainWidget }
  end
  local onChangedHandler = mainWidget:getOnChanged()

  -- Update Widget from the Attributes
  local updateWidget = function(self)
    enableUISetAttribute = false
    isDefault = getCommonAttributeValue(selection, useDefaultAttribute, set)
    value = getCommonAnimationAttributeValue(selection, attribute, useDefaultAttribute, set)
    if value ~= nil then
      mainWidget:setIsIndeterminate(false)
      mainWidget:setValue(value)
    else
      mainWidget:setIsIndeterminate(true)
    end

    local enableWidgets = (not isReferencedList) and (isDefault == false)
    for _, widget in ipairs(widgets) do
      widget:enable(enableWidgets)
    end
    enableUISetAttribute = true
  end

  -- Update attributes from a widget
  local updateAttribute = function(self)
    if onChangedHandler ~= nil then
      onChangedHandler(self)
    end

    if enableUISetAttribute then
      -- prevent the change context callbacks from firing off
      enableContextEvents = false
      setCommonAttributeValue(selection, attribute, self:getValue(), set)
      enableContextEvents = true
    end
  end

  -- create the change context
  local context = attributeEditor.createChangeContext()
  context:setObjects(selection)
  context:addAttributeChangeEvent(attribute)
  context:addAttributeChangeEvent(useDefaultAttribute)
  context:setAttributeChangedHandler(updateWidget)
  attributeEditor.addOnMarkupChanged(updateWidget)

  mainWidget:setOnChanged(updateAttribute)

  -- and update the widget so that to start with it displays the current value
  -- and is enabled appropriately
  updateWidget()
end
