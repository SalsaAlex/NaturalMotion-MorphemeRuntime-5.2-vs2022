------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

local perAnimSetSubsetSection = function(panel, selection, attributes, set)
  -- figure out which subset this applies to by looking at the attribute names
  local characterNo = string.find(attributes[1], ".") + 6
  local subsetNo = string.sub(attributes[1], characterNo, characterNo)
  local subsetName = nil
  if (subsetNo == "1") then subsetName = "First"  end
  if (subsetNo == "2") then subsetName = "Second" end
  if (subsetNo == "3") then subsetName = "Third"  end
  if (subsetNo == "4") then subsetName = "Fourth" end
  
  panel:setBorder(1)
  panel:beginHSizer{ flags = "expand", proportion = 1 }
    panel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(2)

      attributeEditor.addAttributeLabel(panel, subsetName.." subset root", selection, "Subset"..subsetNo.."Root")
      attributeEditor.addAttributeWidget(panel, "Subset"..subsetNo.."Root", selection, set)
      
      -- AffectsAllChildren
      attributeEditor.addAttributeLabel(panel, "Affect all children of root", selection, "Subset"..subsetNo.."AffectsAllChildren")
      attributeEditor.addAttributeWidget(panel, "Subset"..subsetNo.."AffectsAllChildren", selection, set)
      
      attributeEditor.addAttributeLabel(panel, subsetName.." subset leaf", selection, "Subset"..subsetNo.."Leaf")
      local leafWidget = attributeEditor.addAttributeWidget(panel, "Subset"..subsetNo.."Leaf", selection, set)
      
    panel:endSizer()
  panel:endSizer()
  
  local changeContext = attributeEditor.createChangeContext()
  changeContext:setObjects(selection)
  changeContext:addAttributeChangeEvent("Subset"..subsetNo.."AffectsAllChildren")

  local syncLeafWidget = function()
    attributeEditor.logEnterFunc("syncLeafWidget")
      local affectsAllChildren = getCommonAttributeValue(selection, "Subset"..subsetNo.."AffectsAllChildren", set)
      if affectsAllChildren then
        leafWidget:enable(false)
      else
        leafWidget:enable(true)
      end
    attributeEditor.logExitFunc("syncLeafWidget")
  end
  
  changeContext:setAttributeChangedHandler(
    function(object, attr)
      attributeEditor.logEnterFunc("changeContext attributeChangedHandler")
        syncLeafWidget()
      attributeEditor.logExitFunc("changeContext attributeChangedHandler")
    end
  )
  
  syncLeafWidget()     
end

attributeEditor.scaleCharacterDisplaySubsetSection = function(panel, displayInfo, selection)
  attributeEditor.logEnterFunc("attributeEditor.scaleCharacterDisplaySubsetSection")
  
  local attrs = checkSetOrderAndBuildAttributeList(selection, displayInfo.usedAttributes)
    
  local rollup = panel:addRollup{
    label = displayInfo.title,
    flags = "mainSection",
    name = string.format("scaleCharacterSubsetDisplayInfoSection")
  }
  local rollPanel = rollup:getPanel()

  rollPanel:beginVSizer{ flags = "expand" }
    rollPanel:addAnimationSetWidget{
      attributes = attrs,
      displayFunc = perAnimSetSubsetSection,
      flags = "expand",
      proportion = 1,
    }
  rollPanel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.scaleCharacterDisplaySubsetSection")  
end
