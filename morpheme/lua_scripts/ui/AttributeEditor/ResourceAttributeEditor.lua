------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "luaAPI/MorphemeUnitAPI.lua"

local animationSetTree = null;
local removeSettingButton = null;
local treeItemsToAnimationSets = { } -- a mapping from items in the anination tree to the animation set names

------------------------------------------------------------------------------------------------------------------------
local updateRemoveSettingButton = function()
  local selectedResources = animationFileTreeControl:getSelectedItems()
  local selection = animationSetTree:getSelectedItems()
  local settingCount = 0
  for _, treeItem in selection do
    local animSet = treeItemsToAnimationSets[treeItem];
    local included, inherited = getAnimSetFilePathsExclusions(animSet, selectedResources)

    if inherited == 0 or inherited == 2 then
      settingCount = settingCount + 1
    end
  end

  removeSettingButton:enable(settingCount > 0)
end

------------------------------------------------------------------------------------------------------------------------
local removeSettings = function()
  local selectedResources = animationFileTreeControl:getSelectedItems()
  local selection = animationSetTree:getSelectedItems()
  local settingCount = 0
  local animSets = { }
  for _, treeItem in selection do
    local animSet = treeItemsToAnimationSets[treeItem];
    table.insert(animSets, animSet)
  end

  clearAnimSetFilePathsExclusions(animSets, selectedResources)
end

------------------------------------------------------------------------------------------------------------------------
local updateFileSelection = function()
  local selectedResources = animationFileTreeControl:getSelectedItems()
  for treeItem, animSet in treeItemsToAnimationSets do
    local included, inherited = getAnimSetFilePathsExclusions(animSet, selectedResources)
    local mixed = included == 2 or inherited == 2
    treeItem:setColumnCheckbox(2, included)
    treeItem:setColumnIndeterminate(2, mixed)
    treeItem:setBold(inherited == 0)
  end

  updateRemoveSettingButton()
end

------------------------------------------------------------------------------------------------------------------------
local rebuildAChild = function(treeParent, animSetParent)
  local animSets = listAnimSetChildren(animSetParent)
  for _, animSet in animSets do
    local newChild = treeParent:addChild(animSet)
    treeItemsToAnimationSets[newChild] = animSet
    rebuildAChild(newChild, animSet)
  end
end

------------------------------------------------------------------------------------------------------------------------
local rebuildAnimationSetTree = function()
  -- Delete all the elements in the tree
  local rootItem = animationSetTree:getRoot()
  rootItem:clearChildren()
  treeItemsToAnimationSets = { }

  -- And rebuild
  rebuildAChild(rootItem, null)
  updateFileSelection()
end

------------------------------------------------------------------------------------------------------------------------
local animationSetTreeCheckboxChanged = function(tree, treeItem)
  local animSet = treeItem:getColumnValue(1)
  local checkboxValue = treeItem:getColumnCheckbox(2)
  local selectedResources = animationFileTreeControl:getSelectedItems()
  setAnimSetFilePathsExclusions(animSet, selectedResources, checkboxValue == 1)
end

------------------------------------------------------------------------------------------------------------------------
-- Install the Attribute editor
------------------------------------------------------------------------------------------------------------------------
addResourceAttributeEditor = function(contextualPanel, theContext, assetManager)

  local panel = contextualPanel:addPanel{ name = "ResourceAttributeEditor", forContext = theContext }

  animationFileTreeControl = assetManager:getAnimationFileTreeControl()

  animationFileTreeControl:setOnSelectionChanged(
    function(self)
      updateFileSelection(panel)
    end);

  panel:beginVSizer{ flags = "expand" }

   local pan = panel:addScrollPanel{
      name = "ScrollPanel",
      flags = "expand;vertical",
      proportion = 1
    }

    pan:beginVSizer{ flags = "expand" }
      animationSetTree = pan:addTreeControl{
        name = "AnimationSetFileTree",
        flags = "expand;sizeToContent;stripe;hideRoot",
        propotion = 1,
        numColumns = 2,
        treeControlColumn = 1,
        columnNames = { "Animation Set", "Include" },
        onCheckboxChanged = animationSetTreeCheckboxChanged,
        onSelectionChanged = updateRemoveSettingButton,
      }

      registerEventHandler("mcAnimationSetCreated", rebuildAnimationSetTree)
      registerEventHandler("mcAnimationSetDestroyed", rebuildAnimationSetTree)
      registerEventHandler("mcAnimationSetRenamed", rebuildAnimationSetTree)
      registerEventHandler("mcAnimationSetReparented", rebuildAnimationSetTree)
      registerEventHandler("mcAnimationSetModified", updateFileSelection)

      removeSettingButton = pan:addButton{
        label = "Clear Setting",
        flags = "right",
        onClick = removeSettings
      }
      rebuildAnimationSetTree()
    pan:endSizer()

  panel:endSizer()

end