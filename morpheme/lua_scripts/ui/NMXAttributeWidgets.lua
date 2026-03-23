------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- adds a widget for manipulating a specific preference of type float
------------------------------------------------------------------------------------------------------------------------
local addNumericAttributeWidget = function(panel, attribute)
  assert(panel)
  assert(attribute)

  local attributeName = attribute:getName()

  local textBox = panel:addTextBox{
    name = string.format("%sTextBox", attributeName),
    flags = "numeric;expand",
    onEnter = function(self)

      value = tonumber(self:getValue())

      local db = attribute:getDatabase()
      local blockStatus, cbRef = db:beginChangeBlock(getCurrentFileAndLine())

      if attribute:getTypeId() == nmx.AttributeTypeId.kFloat then
        attribute:setFloat(value)
      elseif attribute:getTypeId() == nmx.AttributeTypeId.kInt then
        attribute:setInt(value)
      elseif attribute:getTypeId() == nmx.AttributeTypeId.kDouble then
        attribute:setDouble(value)
      end

      db:endChangeBlock(cbRef, changeBlockInfo("setting attribute %q", attribute:getName()))

      -- The attribute may have formatted the value in some way so read the value back into the text box
      newValue = 0
      if attribute:getTypeId() == nmx.AttributeTypeId.kFloat then
        newValue = attribute:asFloat()
      elseif attribute:getTypeId() == nmx.AttributeTypeId.kInt then
        newValue = attribute:asInt()
      elseif attribute:getTypeId() == nmx.AttributeTypeId.kDouble then
        newValue = attribute:asDouble()
      end

      self:setValue(string.format("%g", newValue))
    end,
  }

  return textBox
end

------------------------------------------------------------------------------------------------------------------------
-- updates a widget created by calling addNumberWidget by getting the current value from the attribute
------------------------------------------------------------------------------------------------------------------------
local updateNumericAttributeWidget = function(panel, attribute)
  assert(panel)
  assert(attribute)

  local attributeName = attribute:getName()

  local textBoxName = string.format("%sTextBox", attributeName)
  local textBox = panel:getChild(textBoxName)
  if textBox then

    local value = 0
    if attribute:getTypeId() == nmx.AttributeTypeId.kFloat then
      value = attribute:asFloat()
    elseif attribute:getTypeId() == nmx.AttributeTypeId.kInt then
      value = attribute:asInt()
    elseif attribute:getTypeId() == nmx.AttributeTypeId.kDouble then
      value = attribute:asDouble()
    end

    textBox:setValue(string.format("%g", value))

    return true
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
-- adds a widget for manipulating a specific preference of type bool
------------------------------------------------------------------------------------------------------------------------
local addBoolAttributeWidget = function(panel, attribute)
  assert(panel)
  assert(attribute)

  local checkBox = panel:addCheckBox{
    name = string.format("%sCheckBox", attribute:getName()),
    onChanged = function(self)
      local db = attribute:getDatabase()
      local blockStatus, cbRef = db:beginChangeBlock(getCurrentFileAndLine())

      attribute:setBool(self:getChecked())

      db:endChangeBlock(cbRef, changeBlockInfo("setting attribute %q", attribute:getName()))
    end,
    flags = "expand",
  }

  return checkBox
end

------------------------------------------------------------------------------------------------------------------------
-- updates a widget created by calling addNumberWidget by getting the current value from the node
------------------------------------------------------------------------------------------------------------------------
local updateBoolAttributeWidget = function(panel, attribute)
  assert(panel)
  assert(attribute)

  local attributeName = attribute:getName()

  local checkBoxName = string.format("%sCheckBox", attributeName)
  local checkBox = panel:getChild(checkBoxName)
  if checkBox then
    checkBox:setChecked(attribute:asBool())
    return true
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
-- adds a widget for manipulating a specific preference of type string
------------------------------------------------------------------------------------------------------------------------
local addStringAttributeWidget = function(panel, attribute)
  assert(panel)
  assert(attribute)

  local attributeName = attribute:getName()

  local textBox = panel:addTextBox{
    name = string.format("%sTextBox", attributeName),
    onEnter = function(self)
      local db = attribute:getDatabase()
      local blockStatus, cbRef = db:beginChangeBlock(getCurrentFileAndLine())

      attribute:setString(self:getValue())

      db:endChangeBlock(cbRef, changeBlockInfo("setting attribute %q", attribute:getName()))
    end,
    flags = "expand",
  }

  return textBox
end

------------------------------------------------------------------------------------------------------------------------
-- updates a widget created by calling addNumberWidget by getting the current value from the attribute
------------------------------------------------------------------------------------------------------------------------
local updateStringAttributeWidget = function(panel, attribute)
  assert(panel)
  assert(attribute)

  local attributeName = attribute:getName()

  local textBoxName = string.format("%sTextBox", attributeName)
  local textBox = panel:getChild(textBoxName)
  if textBox then
    textBox:setValue(attribute:asString())
    return true
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
-- adds a widget for manipulating a specific preference of type enum
------------------------------------------------------------------------------------------------------------------------
local addEnumAttributeWidget = function(panel, attribute)
  assert(panel)
  assert(attribute)

  local attributeName = attribute:getName()

  -- Create a table containing all the choices of the enum
  local enumChoicesNMXArray = attribute:getChoices()

  local choicesTable = { }
  for choiceIndex = 1, enumChoicesNMXArray:size() do
    table.insert(choicesTable, enumChoicesNMXArray:at(choiceIndex))
  end

  -- Create the combo box
  local attributeComboBox = panel:addComboBox{
    name = string.format("%sComboBox", attributeName),
    items = choicesTable,
    flags = "expand",
    proportion = 1,
    onChanged = function(self)
      local db = attribute:getDatabase()
      local blockStatus, cbRef = db:beginChangeBlock(getCurrentFileAndLine())

      attribute:setInt(self:getSelectedIndex() - 1)

      db:endChangeBlock(cbRef, changeBlockInfo("setting attribute %q", attribute:getName()))
    end,
  }

  return attributeComboBox
end

------------------------------------------------------------------------------------------------------------------------
-- updates a widget created by calling preferences.addEnumWidget by getting the current value from the preferences
------------------------------------------------------------------------------------------------------------------------
local updateEnumAttributeWidget = function(panel, attribute)
  assert(panel)
  assert(attribute)

  local comboBoxName = string.format("%sComboBox", attribute:getName())
  local comboBox = panel:getChild(comboBoxName)
  if comboBox then
    comboBox:setSelectedIndex(attribute:asInt() + 1)
    return true
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: widget nmx.hasAttributeWidgetForType(integer attributeTypeId)
--| brief:
--|   Returns true if nmx.addAttributeWidget can create a widget for this attribute type.
--|
--| environments: GlobalEnv
------------------------------------------------------------------------------------------------------------------------
nmx.hasAttributeWidgetForType = function(attributeTypeId)
  if attributeTypeId == nmx.AttributeTypeId.kFloat or
     attributeTypeId == nmx.AttributeTypeId.kInt or
     attributeTypeId == nmx.AttributeTypeId.kDouble or
     attributeTypeId == nmx.AttributeTypeId.kBool or
     attributeTypeId == nmx.AttributeTypeId.kString or
     attributeTypeId == nmx.AttributeTypeId.kEnum then
    return true
  else
    return false;
  end
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: widget nmx.addAttributeWidget()
--| brief:
--|   Creates an attribute widget for an nmx attribute, use nmx.updateAttributeWidget to update its value
--|
--| environments: GlobalEnv
------------------------------------------------------------------------------------------------------------------------
nmx.addAttributeWidget = function(panel, attribute)
  assert(attribute)

  local attributeTypeId = attribute:getTypeId()
  if not nmx.hasAttributeWidgetForType(attributeTypeId) then
    local message = string.format(
      "Cannot add attribute widget for attribute '%s' with type id '%d'",
      attribute:getName(),
      attributeTypeId)

    app.warning(message)
    return nil
  end

  if attributeTypeId == nmx.AttributeTypeId.kFloat or
     attributeTypeId == nmx.AttributeTypeId.kInt or
     attributeTypeId == nmx.AttributeTypeId.kDouble then
    return addNumericAttributeWidget(panel, attribute)
  elseif attributeTypeId == nmx.AttributeTypeId.kBool then
    return addBoolAttributeWidget(panel, attribute)
  elseif attributeTypeId == nmx.AttributeTypeId.kString then
    return addStringAttributeWidget(panel, attribute)
  elseif attributeTypeId == nmx.AttributeTypeId.kEnum then
    return addEnumAttributeWidget(panel, attribute)
  end

  return nil
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean nmx.addAttributeWidget()
--| brief:
--|   Updates an attribute widget created by nmx.addAttributeWidget.
--|
--| environments: GlobalEnv
------------------------------------------------------------------------------------------------------------------------
nmx.updateAttributeWidget = function(panel, attribute)
  local attributeTypeId = attribute:getTypeId()

  if attributeTypeId == nmx.AttributeTypeId.kFloat or
     attributeTypeId == nmx.AttributeTypeId.kInt or
     attributeTypeId == nmx.AttributeTypeId.kDouble then
    return updateNumericAttributeWidget(panel, attribute)
  elseif attributeTypeId == nmx.AttributeTypeId.kBool then
    return updateBoolAttributeWidget(panel, attribute)
  elseif attributeTypeId == nmx.AttributeTypeId.kString then
    return updateStringAttributeWidget(panel, attribute)
  elseif attributeTypeId == nmx.AttributeTypeId.kEnum then
    return updateEnumAttributeWidget(panel, attribute)
  end

  return false
end