------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local UngroupedName = "Ungrouped"
local MappingUtils = { }

-- find the union of groups between the two sets
MappingUtils.findRetargetingGroups = function(selectedSet, sourceSet)
  local selectedTempl = anim.getAnimSetTemplate(selectedSet)

  if sourceSet ~= "" and selectedTempl ~= anim.getAnimSetTemplate(sourceSet) then
    app.error("Mismatched templates in retarget mappings section")
  end

  local groups = { }
  for _,name in ipairs(listAnimSets()) do
    if not anim.getTemplatesIdentical(selectedSet, name) then
      return
    end
    
    local scene = nmx.Application.new():getSceneByName("AssetManager")
    local sceneRoot = anim.getRigSceneRoot(scene, name)
    if not sceneRoot then
      app.error("Couldn't find rig root for '" .. name .. "' anim set")
      return
    end
    
    local it = nmx.NodeIterator.new(sceneRoot, nmx.sgTransformNode.ClassTypeId())
    while it:next() do
      local joint = it:node():getChildDataNode(nmx.JointNode.ClassTypeId())
      if not joint or joint:getTypeId() ~= nmx.JointNode.ClassTypeId() then
        continue
      end
      
      local offsetFrameTransform = nmx.OffsetFrameTransformNode.getOffsetFrameTransform(joint:getFirstInstance())
      if not offsetFrameTransform then
        continue
      end
      
      local offsetFrame = offsetFrameTransform:getFirstChild(nmx.OffsetFrameNode.ClassTypeId())
      if not offsetFrame then
        continue
      end
      
      local mapping = offsetFrame:findAttribute("RetargetingTag"):asString()
      local found = false
      for i,v in ipairs(groups) do
        if v ~= mapping then
          continue
        end
        found = true
      end

      if mapping ~= "" and not found then
        table.insert(groups, mapping)
      end
    end
  end
  return groups
end

-- rename all offset nodes mappings in set called "oldName" to "newName"
MappingUtils.renameMappingForSet = function(scene, set, oldName, newName)
  local sceneRoot = anim.getRigSceneRoot(scene, set)
  if not sceneRoot then
    return
  end
  
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())
  
  local it = nmx.NodeIterator.new(sceneRoot, nmx.sgTransformNode.ClassTypeId())
  while it:next() do
    local node = it:node()
    if not node then
      continue
    end
    
    local joint = node:getChildDataNode(nmx.JointNode.ClassTypeId())
    if not joint or joint:getTypeId() ~= nmx.JointNode.ClassTypeId() then
      continue
    end
    
    local offsetFrameTransform = nmx.OffsetFrameTransformNode.getOffsetFrameTransform(joint:getFirstInstance())
    if not offsetFrameTransform then
      continue
    end
    
    local offsetFrame = offsetFrameTransform:getFirstChild(nmx.OffsetFrameNode.ClassTypeId())
    if not offsetFrame then
      continue
    end
    
    local mappingAttr = offsetFrame:findAttribute("RetargetingTag")
    local mapping = mappingAttr:asString()
    if mapping ~= oldName then
      continue
    end
    
    mappingAttr:setString(newName)
  end
  scene:endChangeBlock(cbRef, changeBlockInfo("Rename retarget mapping"))
end

-- rename a mapping in a group from old name to new name
MappingUtils.renameMapping = function(selectedSet, sourceSet, oldNameIn, newNameIn)
  local app = nmx.Application.new()
  local scene = app:getSceneByName("AssetManager")
  local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

  MappingUtils.renameMappingForSet(scene, selectedSet, oldNameIn, newNameIn)
  if sourceSet ~= "" then
    MappingUtils.renameMappingForSet(scene, sourceSet, oldNameIn, newNameIn)
  end

  scene:endChangeBlock(cbRef, changeBlockInfo("Rename retarget mapping"))
end

-- count the number of anim sets using a particular group name
MappingUtils.numberOfAnimationSetsUsingGroup = function(group)
  local setCount = 0
  local scene = nmx.Application.new():getSceneByName("AssetManager")

  for _,set in pairs(listAnimSets()) do
    local dataRoot = anim.getRigDataRoot(scene, set)

    local it = nmx.NodeIterator.new(dataRoot, nmx.OffsetFrameNode.ClassTypeId())
    while it:next() do
      local tag = it:node():findAttribute("RetargetingTag"):asString()
      if tag == group then
        setCount = setCount + 1
        break
      end
    end
  end

  return setCount
end

  -- set a mapping for an offset node named name
MappingUtils.setItemsMapping = function(set, name, newParent)
  if newParent == UngroupedName then
    newParent = ""
  end

  local scene = nmx.Application.new():getSceneByName("AssetManager")

  local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

  local sceneRoot = anim.getRigSceneRoot(scene, set)
  if sceneRoot then
    local it = nmx.NodeIterator.new(sceneRoot, nmx.sgTransformNode.ClassTypeId())

    it:reset(sceneRoot, nmx.sgTransformNode.ClassTypeId())
    while it:next() do
      local node = it:node()
      if not node or node:getName() ~= name then
        continue
      end
      
      local joint = node:getChildDataNode(nmx.JointNode.ClassTypeId())
      if not joint or joint:getTypeId() ~= nmx.JointNode.ClassTypeId() then
        continue
      end
      
      local offsetFrameTransform = nmx.OffsetFrameTransformNode.getOffsetFrameTransform(joint:getFirstInstance())
      if not offsetFrameTransform then
        continue
      end
      
      local offsetFrame = offsetFrameTransform:getFirstChild(nmx.OffsetFrameNode.ClassTypeId())
      if not offsetFrame then
        continue
      end
      
      offsetFrame:findAttribute("RetargetingTag"):setString(newParent)
      
      if newParent ~= "" then
        offsetFrame:findAttribute("ShowAxis"):setBool(true)
      else
        offsetFrame:findAttribute("ShowAxis"):setBool(false)
      
        -- ensure this frame has its axis shown if any of its children have their axis shown.
        local it2 = nmx.NodeIterator.new(node, nmx.sgTransformNode.ClassTypeId())
        while it2:next() do
          local node2 = it2:node()
          local joint2 = node2:getChildDataNode(nmx.JointNode.ClassTypeId())
          if not joint2 or joint2:getTypeId() ~= nmx.JointNode.ClassTypeId() then
            continue
          end
          
          local offsetFrameTransform2 = nmx.OffsetFrameTransformNode.getOffsetFrameTransform(joint2:getFirstInstance())
          if not offsetFrameTransform2 then
            continue
          end
          
          local offsetFrame2 = offsetFrameTransform2:getFirstChild(nmx.OffsetFrameNode.ClassTypeId())
          if not offsetFrame2 then
            continue
          end
          
          if offsetFrame2:findAttribute("RetargetingTag"):asString() == "" then
            continue
          end
          
          offsetFrame:findAttribute("ShowAxis"):setBool(true)
          break
        end
      end
      
      break
    end
  end

  scene:endChangeBlock(cbRef, changeBlockInfo("Rename retarget mapping"))
end

local MappingUiUtils = { }

-- add a tree control which is set up to display a retargeting mapping UI 
MappingUiUtils.addTree = function(panel, scene, set, primarySet, sourceSet, rebuild)
  local tree = panel:addTreeControl{
    flags = "expand;hideRoot;multiSelect;rename",
    proportion = 1,
    size = { width = -1, height = 200 },
    onItemRenamed = function(tree, item, oldName)
      MappingUtils.renameMapping(primarySet, sourceSet, oldName, item:getValue())
      rebuild()
    end,
    allowDrag = true,
    onDrop = function(self, dropItems, dropItem)
      if dropItem and dropItem:getParent() == self:getRoot() then
        for i,v in pairs(dropItems) do
          local currentGroup = v:getParent()
          if currentGroup and currentGroup:getParent() == self:getRoot() then
            if v:getTreeControl() == self then
              MappingUtils.setItemsMapping(set, v:getValue(), dropItem:getValue())
            end
          end
        end
      end
      rebuild()
    end
  }

  local onContextualMenu = function(menu, item)
    if item:getParent() ~= tree:getRoot() then
      local mappingsToRemove = tree:getSelection()

      local labelText = "Remove Mapping"
      if table.getn(mappingsToRemove) > 1 then
        labelText = "Remove Mappings"
      end

      -- delete item mapping
      menu:addItem{
        label = labelText,
        onClick = function(self)
          for _,v in pairs(mappingsToRemove) do
            MappingUtils.setItemsMapping(set, v:getValue(), "")
          end

          rebuild()
        end,
      }
    else
      local groupsToRemove = tree:getSelection()

      if table.getn(groupsToRemove) ~= 0 then

        local labelText = "Clear Group"
        if MappingUtils.numberOfAnimationSetsUsingGroup(groupsToRemove[1]:getValue()) == 1 then
          labelText = "Remove Group"
        end

        if table.getn(groupsToRemove) > 1 then
          labelText = labelText .. "s"
        end

        -- delete group
        menu:addItem{
          label = labelText,
          onClick = function(self)
            for _,v in pairs(groupsToRemove) do
              MappingUtils.renameMappingForSet(scene, set, v:getValue(), "")
            end

            rebuild()
          end,
        }
        
        menu:addItem{
          label = "Rename Mapping",
          onClick = function(self)
            item:getTreeControl():editItem(item)
          end,
        }
      end
    end
  end
  tree:setOnContextualMenu(onContextualMenu)

  return tree
end
  
-- update a tree from the passed animation set, and the table of group names
MappingUiUtils.updateTreeFromSet = function(tree, set, groups)
  local root = tree:getRoot()

  local state = { }
  for i=1,root:getNumChildren() do
    local child = root:getChild(i)
    state[child:getValue()] = not child:isCollapsed()
  end

  root:clearChildren()

  local items = { }
  local addItem = function(key, name)
    local item = root:addChild(name)

    if not state[name] then
      item:collapse()
    end
    item:setBold(true)

    items[key] = item
  end


  if groups ~= nil then
    for i,name in pairs(groups) do
      addItem(name, name)
    end
  end

  addItem("", UngroupedName)

  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local sceneRoot = anim.getRigSceneRoot(scene, set)
  if sceneRoot then
    local it = nmx.NodeIterator.new(sceneRoot, nmx.sgTransformNode.ClassTypeId())
    while it:next() do
      local node = it:node()
      if node then
        local joint = node:getChildDataNode(nmx.JointNode.ClassTypeId())
        if joint and joint:getTypeId() == nmx.JointNode.ClassTypeId() then
          local offsetFrameTransform = nmx.OffsetFrameTransformNode.getOffsetFrameTransform(joint:getFirstInstance())
          if offsetFrameTransform then
            local offsetFrame = offsetFrameTransform:getFirstChild(nmx.OffsetFrameNode.ClassTypeId())
            if offsetFrame then
              local mapping = offsetFrame:findAttribute("RetargetingTag"):asString()
              local item = items[mapping]

              if item ~= nil then
                item:addChild(node:getName())
              else
                app.error("Failed to find retargetting group for joint " .. node:getName() .. " with mapping '" .. mapping .. "'")
              end
            end
          end
        end
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
addRetargetMappingEditingUI = function(panel, selectedSet, sourceSet)
             
  -- whether we are displaying two sets or one set
  local secondSet = sourceSet ~= ""

  --
  local groups = { }
  local primaryTree = nil
  local secondaryTree = nil

  local rebuildAllTrees = function(groups)
    MappingUiUtils.updateTreeFromSet(primaryTree, selectedSet, groups)
    if secondSet then
      MappingUiUtils.updateTreeFromSet(secondaryTree, sourceSet, groups)
    end
  end

  -- used as a callback for ui to update
  local rebuildAllTreesAndGroups = function()
    groups = MappingUtils.findRetargetingGroups(selectedSet, sourceSet)
    rebuildAllTrees(groups)
  end
  
  panel:beginVSizer{ flags = "expand" }

    local columnCount = 1
    if secondSet then
      columnCount = 2
    end
    
    panel:beginFlexGridSizer{ cols = columnCount, flags = "expand", proportion = 1 }
      panel:setFlexGridColumnExpandable(1)
      panel:setFlexGridColumnExpandable(2)

      panel:addStaticText{ text = selectedSet }
      if secondSet then
        panel:addStaticText{ text = sourceSet }
      end

      local app = nmx.Application.new()
      local scene = app:getSceneByName("AssetManager")
      primaryTree = MappingUiUtils.addTree(panel, scene, selectedSet, selectedSet, sourceSet, rebuildAllTreesAndGroups)
      secondaryTree = nil
      if secondSet then
        secondaryTree = MappingUiUtils.addTree(panel, scene, sourceSet, selectedSet, sourceSet, rebuildAllTreesAndGroups)
      end

      panel:beginHSizer{ flags = "expand" }
        panel:addStretchSpacer{ proportion = 1 }
        panel:addButton{ label = "Reset",
          onClick = function()
            local app = nmx.Application.new()
            local scene = app:getSceneByName("AssetManager")
            local sceneRoot = anim.getRigSceneRoot(scene, selectedSet)

            local sl = nmx.SelectionList.new()
            sl:add(sceneRoot)

            app:runCommand("Retargeting", "Generate Dense Retarget Mapping", scene, sl)

            rebuildAllTreesAndGroups()
          end
        }
      panel:endSizer()

      if secondSet then
        panel:beginHSizer{ flags = "expand" }
          panel:addStretchSpacer{ proportion = 1 }
          panel:addButton{ label = "Reset",
            onClick = function()
              local app = nmx.Application.new()
              local scene = app:getSceneByName("AssetManager")
              local sceneRoot = anim.getRigSceneRoot(scene, sourceSet)

              local sl = nmx.SelectionList.new()
              sl:add(sceneRoot)

              app:runCommand("Retargeting", "Generate Dense Retarget Mapping", scene, sl)

              rebuildAllTreesAndGroups()
            end
          }
        panel:endSizer()
      end
    panel:endSizer()


    panel:beginVSizer{ flags = "expand" }
      panel:addTextControl{
        value = "Choose joints from one or both sections to add to a new group, then choose 'Create Group'.",
        size = { width = 275, height = 30 },
        flags = "readonly;noscroll;dialogColours;italic", proportion = 1
      }
      panel:beginHSizer{ flags = "expand" }
        panel:addStretchSpacer{ proportion = 1 }
        panel:addButton{ label = "Create Group",
          onClick = function()
            local baseGroupName = "NewGroup"
            local groupName = baseGroupName
            local num = 1
            local found = true
            while found do
              found = false
              for i,v in ipairs(groups) do
                if v == groupName then
                  groupName = baseGroupName .. tostring(num)
                  num = num + 1
                  found = true
                  break
                end
              end
            end

            local selection = primaryTree:getSelectedItems()
            for i,v in selection do
              if v:getParent() ~= primaryTree:getRoot() then
                MappingUtils.setItemsMapping(selectedSet, v:getValue(), groupName)
              end
            end

            if secondSet then
              local selection = secondaryTree:getSelectedItems()
              for i,v in selection do
                if v:getParent() ~= secondaryTree:getRoot() then
                  MappingUtils.setItemsMapping(sourceSet, v:getValue(), groupName)
                end
              end
            end

            rebuildAllTreesAndGroups()
          end
        }
      panel:endSizer()
    panel:endSizer()
  panel:endSizer()

  -- init UI
  rebuildAllTreesAndGroups()
end
