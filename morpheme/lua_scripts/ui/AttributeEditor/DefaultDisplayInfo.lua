------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

------------------------------------------------------------------------------------------------------------------------
-- Adds a display info section with the title displayInfo.title
-- all attributes in the displayInfo.usedAttributes table.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.basicConditionInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.basicConditionInfoSection")

  attributeEditor.addBasicConditionInfoSection(
    panel,
    selection,
    displayInfo.usedAttributes)

  attributeEditor.logExitFunc("attributeEditor.basicConditionInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a display info section with the title displayInfo.title in a roll panel containing
-- all attributes in the displayInfo.usedAttributes table.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.standardDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.simpleDisplayInfoSection")

  attributeEditor.addSimpleAttributeSection(
    panel,
    displayInfo.title,
    selection,
    displayInfo.usedAttributes)

  attributeEditor.logExitFunc("attributeEditor.simpleDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Adds a display info section with the title displayInfo.title in a roll panel containing
-- all attributes in the displayInfo.usedAttributes table. If there is more than one animation
-- set in the network then the attributes are grouped in an animation set container.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.animSetDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.animSetDisplayInfoSection")

  attributeEditor.addCompoundAttributeSection(
    panel,
    displayInfo.title,
    selection,
    displayInfo.usedAttributes)

  attributeEditor.logExitFunc("attributeEditor.animSetDisplayInfoSection")
end

------------------------------------------------------------------------------------------------------------------------
-- Any attributes that should not be displayed in the UI can be assigned to the hiddenDisplayInfoSection
------------------------------------------------------------------------------------------------------------------------
attributeEditor.hiddenDisplayInfoSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.hiddenDisplayInfoSection")
  attributeEditor.logExitFunc("attributeEditor.hiddenDisplayInfoSection")
end
