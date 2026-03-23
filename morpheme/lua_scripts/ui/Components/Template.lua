------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
local addImage = app.loadImage("additem.png")
local removeImage = app.loadImage("removeitem.png")
local saveImage = app.loadImage("save.png")
local orderUpImage = app.loadImage("moveup.png")
local orderDownImage = app.loadImage("movedown.png")
local newTemplateTreeControlName = "NewTemplateTreeControl"

local newTemplateOffsetHelpTextControl = "NewTemplateOffsetHelpTextControl"
local newTemplateOffsetImageTextControl = "NewTemplateOffsetImageTextControl"
local newTemplateHelpTextControl = "NewTemplateHelpTextControl"
local newTemplateImageTextControl = "NewTemplateImageTextControl"

local animationMappingRollUpName = "animationMappingPanel"
local physicsMappingRollUpName = "physicsMappingPanel"
local templateListComboBoxName = "templateListComboBox"
local componentEditorPath = "MainFrame|LayoutManager|AttributeEditor|ComponentAttributeEditor"
local noTemplateString = "No Template"
local templateUIElements = {}
local currentSourceAnimRigPath
local currentSourcePhysicsRigPath
templateFolder = "$(AppRoot)\\resources\\rigTemplates"

TemplateAllTags = 0
TemplateOnlyRequiredTags = 1
TemplateOnlyOptionalTags = 2

DebugAutoMapping = false

------------------------------------------------------------------------------------------------------------------------
local recursiveIsUniqueName
recursiveIsUniqueName = function(treeItem, name)
  local numChildren = treeItem:getNumChildren()
  for i = 1, numChildren do
    local childItem = treeItem:getChild(i)
    if childItem:getValue() == name or recursiveIsUniqueName(childItem, name) == false then
      return false
    end
  end

  return true
end

------------------------------------------------------------------------------------------------------------------------
local getFreeName = function(treeControl)
  local rootTreeItem = treeControl:getRoot()
  local name = "Tag"
  local iter = 1
  while recursiveIsUniqueName(rootTreeItem, name) == false do
    name = "Tag" .. iter
    iter = iter + 1
  end
  return name
end

------------------------------------------------------------------------------------------------------------------------
local onTemplateTreeItemRenamed = function(treeItem, oldName)
  -- Make sure the new name is unique, use the old one if this is not the case
  if not recursiveIsUniqueName(treeItem:getRoot(), treeItem:getValue()) then
    treeItem:setValue(oldName)
  end
end

------------------------------------------------------------------------------------------------------------------------
local onTemplateTreeItemActivated = function(treeItem)
  treeItem:editItem(treeItem:getSelectedItem())
end

------------------------------------------------------------------------------------------------------------------------
local addTemplateTreeItem = function(button)
  local treeControl = button:getParent():findDescendant(newTemplateTreeControlName)
  if treeControl ~= nil then
    local newItemName = getFreeName(treeControl)

    local newItemParent = treeControl:getSelectedItem()
    if newItemParent == nil then
      newItemParent = treeControl:getRoot()
    end
    local item = newItemParent:addChild(newItemName)

    item:setColumnCheckbox(2, false) -- Optional
    item:setColumnCheckbox(3, false) -- Left
    item:setColumnCheckbox(4, false) -- Right
    item:setColumnCheckbox(5, false) -- GroundContact
    item:setColumnCheckbox(6, false) -- IKTarget
    item:setColumnValue(7, "") -- RegularExpressions
  end
end

------------------------------------------------------------------------------------------------------------------------
local updateMappingComboBoxes = function(panel, bodyMappingNode, bodyMappingInfoNode, channelNames)
  if panel == nil or bodyMappingNode == nil  or bodyMappingInfoNode == nil then
    app.error("Failed to update mapping combo boxes, invalid arguements. - template.lua 102")
    return
  end

  local mappingArray = bodyMappingNode:findAttributeArray("Parts")
  local mappingArraySize = mappingArray:size()
  if not mappingArray:isValid() then
    app.error("Failed to update mapping combo boxes, invalid arguements. - template.lua 109")
    return
  end

  -- Update all the mapping combo box items and selection
  for i = 1, mappingArraySize do
    local attribute = mappingArray:getAttribute(i)
    local attributeName = attribute:asString()
    local comboBox = panel:findDescendant(attributeName)
    if comboBox ~= nil then

      -- Update the channel names
      local outputAttribute = attribute:getLastOutput()
      if channelNames ~= nil then
        local partNames = bodyMappingNode:getValidParts(attributeName, nmx.JointNode.ClassTypeId())
        if partNames:empty() and i == 1 then
          comboBox:setItems(channelNames)
        else
          local partNamesTable = {}

          table.insert(partNamesTable, "")
          if partNames:empty() and outputAttribute:isValid() then
            local possibleName = outputAttribute:getNode():getName()
            if possibleName ~= "" then
              table.insert(partNamesTable, possibleName)
            end
          else
            local partNameSize = partNames:size()
            for j = 1, partNameSize do
              local partString = partNames:at(j)
              table.insert(partNamesTable, partString)
            end
          end
          comboBox:setItems(partNamesTable)
        end
      end

      if outputAttribute:isValid() then
        comboBox:setSelectedItem(outputAttribute:getNode():getName())
      else
        comboBox:setSelectedIndex(1)
      end

      local result, partIndex = bodyMappingNode:findPart(attributeName)
      comboBox:setError(not bodyMappingNode:isValidPart(partIndex, bodyMappingInfoNode))
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local matchAgainstPatterns = function(bodyMapInfo, name, mapIndex)
  local patterns = bodyMapInfo:getPatterns(mapIndex)

  if DebugAutoMapping then
    app.info("# Checking against " .. tostring(patterns:size()) .. " patterns")
  end
  for i = 1,patterns:size() do
    local pattern = patterns:at(i)
    if string.find(string.lower(name), pattern) then
      return true
    end
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
local findBestParts = function(bodyMap, bodyMapInfo, mapIndex, root)
  local jointsAttribute = bodyMap:findAttributeArray("Parts")

  local parentPart, parentIndex = bodyMap:getParentPart(mapIndex)

  while parentIndex >= 0 and parentIndex < jointsAttribute:size() and parentPart == nil do
    parentPart, parentIndex = bodyMap:getParentPart(parentIndex)
  end

  if not parentPart then
    parentPart = root
  end

  local options = { }

  local it = nmx.NodeIterator.new(root, nmx.sgTransformNode.ClassTypeId())
  while it:next() do
    local transform = it:node()
    
    local joint = transform:getChildDataNode(nmx.JointNode.ClassTypeId())
    if joint and joint:getTypeId() ~= nmx.OffsetFrameNode.ClassTypeId() then
      if DebugAutoMapping then
        app.info("Checking node " .. transform:getName() .. " against patterns for " .. jointsAttribute:getAttribute(mapIndex+1):asString())
      end
      if matchAgainstPatterns(bodyMapInfo, transform:getName(), mapIndex) then
        table.insert(options, transform)
      end
    end
  end
  return options
end

------------------------------------------------------------------------------------------------------------------------
local findBestPartFromAlreadyMappedChildren = function(bodyMap, opts, luaMapIndex)
  local parentInfo = bodyMap:findAttribute("ParentIndex"):asIntArray()
  local lowestHitPart = nil

  -- first find the valid parts parented to this part
  local searchIndexes = { }

  if DebugAutoMapping then
    app.info("Looking for a correct mapping for " .. bodyMap:findAttributeArray("Parts"):getAttribute(luaMapIndex):asString() .. " based on already mapped children")
  end

  local loop = true
  searchIndexes[luaMapIndex-1] = true
  while loop do
    loop = false
    for i,v in pairs(searchIndexes) do
      local part = bodyMap:getPart(i)
      if DebugAutoMapping and i >= 0 and i < parentInfo:size() then
        app.info("Checking if " .. bodyMap:findAttributeArray("Parts"):getAttribute(i+1):asString() .. " is mapped correctly")
      end

      if not part then
        if DebugAutoMapping then
          app.info(bodyMap:findAttributeArray("Parts"):getAttribute(i+1):asString() .. " is not valid, searching its children to find a valid part")
        end
        -- this part isnt valid, search down and find valid children.
        for child=1,parentInfo:size() do
          local parent = parentInfo:at(child)
          if parent == i then
            searchIndexes[(child-1)] = true
            loop = true
          end
        end
        searchIndexes[i] = nil
      end
    end
  end

  if DebugAutoMapping then
    app.info("Valid child parts for " .. bodyMap:findAttributeArray("Parts"):getAttribute(luaMapIndex):asString() .. ":")
    for i,enable in pairs(searchIndexes) do
      if enable then
        local part = bodyMap:getPart(i)
        app.info(" -> " .. part:getName())
      end
    end
  end

  local modifiedOpts = { }

  for _,v in pairs(opts) do
    local failed = false
    for i,enable in pairs(searchIndexes) do
      if enable then
        local childPart = bodyMap:getPart(i)

        if not childPart:isDescendantOf(v) then
          failed = true
        end
      end
    end

    if not failed then
      table.insert(modifiedOpts, v)
    else
      if DebugAutoMapping then
        app.info("Discarding option " .. v:getName() .. " because it doesn't parent all required children")
      end
    end
  end

  if DebugAutoMapping then
    app.info("Choosing the lowest common root from the options:")
    for _,v in pairs(modifiedOpts) do
      app.info(" -> " .. v:getName())
    end
  end

  local findInOptions = function(part)
    for _,v in pairs(modifiedOpts) do
      if v:is(part) then
        return true
      end
    end
    return false;
  end

  -- find the lowest node that is parent to all the valid parts
  for i,enable in pairs(searchIndexes) do
    if enable then
      local part = bodyMap:getPart(i)

      while part and not findInOptions(part) do
        part = part:getParent()
      end

      if DebugAutoMapping and part then
        app.info("Ancestor of " .. bodyMap:getPart(i):getName() .. ", " .. part:getName() .. " is in options")
      end

      if part and (not lowestHitPart or lowestHitPart:isDescendantOf(part)) then
        lowestHitPart = part
      end
    end
  end

  return lowestHitPart
end

------------------------------------------------------------------------------------------------------------------------
local findBestPartFromChildrenAndDistance = function(bodyMap, luaMapIndex)
  local parentInfo = bodyMap:findAttribute("ParentIndex"):asIntArray()

  childIndex = luaMapIndex
  local childPart = nil
  local partsBelow = 0
  while not childPart and childIndex > 0 and childIndex <= parentInfo:size() do
    local foundNewChild = false
    for i=1,parentInfo:size() do
      if parentInfo:at(i) == (childIndex-1) then
        childIndex = i
        foundNewChild = true
        partsBelow = partsBelow + 1
        break
      end
    end
    if foundNewChild then
      childPart = bodyMap:getPart(childIndex-1)
    else
      break
    end
  end

  if childPart then
    local partsAbove = 0
    local parentPart = nil
    local parentIndex = luaMapIndex
    while not parentPart and parentIndex > 0 and parentIndex <= parentInfo:size() do
      parentIndex = parentInfo:at(parentIndex)+1
      partsAbove = partsAbove + 1
      parentPart = bodyMap:getPart(parentIndex-1)
    end

    if parentPart then
      local parts = { }
      local currentPart = childPart:getParent()
      while currentPart and not currentPart:is(parentPart) do
        table.insert(parts, currentPart)
        currentPart = currentPart:getParent()
      end

      if table.getn(parts) == 1 then
        return parts[1]
      end

      local fractionAlong = partsBelow / (partsBelow + partsAbove)

      -- find an index along the joint chain, counting the start and end in the fraction
      local index = math.floor(fractionAlong * (table.getn(parts) + 2))

      --local part = parts[table.getn(parts)-index]

      return parts[index]
    end
  end
  return nil
end

------------------------------------------------------------------------------------------------------------------------
local autoMapParts = function(bodyMap, bodyMapInfo, rootSceneNode, parts)
  local multipleOptions = { }
  
  for _,i in pairs(parts) do
    local bestParts = findBestParts(bodyMap, bodyMapInfo, i-1, rootSceneNode)
    if DebugAutoMapping then
      app.info("Searching for best matched parts for: " .. bodyMap:findAttributeArray("Parts"):getAttribute(i):asString() .. ", found " .. tostring(table.getn(bestParts)))
    end

    if table.getn(bestParts) == 1 then
      bodyMap:setPart(bestParts[1], i-1, true)
    elseif table.getn(bestParts) > 1 then
      if DebugAutoMapping then
        app.info(bodyMap:findAttributeArray("Parts"):getAttribute(i):asString() .. " has multiple possible tags:")
        for i,v in pairs(bestParts) do
          app.info(" -> " .. v:getName())
        end
      end

      multipleOptions[i] = bestParts
    end
  end

  -- reverse the mappings, so we can iterate over lower parts first, then parents are more likely to map correctly (as they use child info)
  local reverseIndexedTable = { }
  for i,v in pairs(multipleOptions) do
    table.insert(reverseIndexedTable, 1, i)
  end

  for _,i in ipairs(reverseIndexedTable) do
    local options = multipleOptions[i]

    if table.getn(options) == 1 then
      bodyMap:setPart(options[1], i-1, true)
    elseif table.getn(options) > 0 then
      local bestPart = findBestPartFromAlreadyMappedChildren(bodyMap, options, i)

      if DebugAutoMapping then
        local partName = ""
        if bestPart then
          partName =  bestPart:getName()
        end
        app.info("Finding best part from mapped children for " .. bodyMap:findAttributeArray("Parts"):getAttribute(i):asString() .. " returned '" .. partName .. "'")
      end

      if bestPart then
        bodyMap:setPart(bestPart, i-1, true)
      end
    end
  end

  -- now clean up by mapping things with no mapping, but a child mapping, by distance.
  for _,i in pairs(parts) do
    local part = bodyMap:getPart(i-1)
    if not part then
      local distancePart = findBestPartFromChildrenAndDistance(bodyMap, i)

      if DebugAutoMapping then
        local partName = ""
        if bestPart then
          partName =  distancePart:getName()
        end
        app.info("Finding best part based on distance between mapped hierachy for " .. bodyMap:findAttributeArray("Parts"):getAttribute(i):asString() .. " returned '" .. partName .. "'")
      end

      if distancePart then
        bodyMap:setPart(distancePart, i-1, true)
      end
    end
  end


  for _,i in pairs(parts) do
    local part = bodyMap:getPart(i-1)
    if not part then
      local highestPart = nil
      local options = multipleOptions[i]
      if options then
        for i,v in ipairs(options) do
          if not highestPart or highestPart:isDescendantOf(v) then
            highestPart = v
          end
        end

        if highestPart then
          bodyMap:setPart(highestPart, i-1, true)
        end
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local autoMapRequired = function(rootDataNode, rootSceneNode, channelNames)
  local selectedSet = getSelectedAssetManagerAnimSet()
  if type(selectedSet) ~= "string" or string.len(selectedSet) == 0 then
    return
  end

  local scene = nmx.Application.new():getSceneByName("AssetManager")

  if rootDataNode == nil or rootSceneNode == nil then
    app.error("Failed to find rig data root or rig scene root when auto mapping template tags")
    return
  end

  local bodyMap = nmx.BodyMappingNode.getBodyMappingNode(rootSceneNode)
  local bodyMappingInfo = nmx.BodyMappingInfoNode.getBodyMappingInfoNode(rootSceneNode)
  
  if bodyMap == nil then
    app.error("Failed to body map when auto mapping template tags")
    return
  end

  local mappingArray = bodyMap:findAttributeArray("Parts")
  local mappingArraySize = mappingArray:size()
  local mappingParentInfo = bodyMap:findAttribute("ParentIndex")
  if not mappingArray:isValid() and not mappingParentInfo:isValid() then
    return
  end

  local nodeIterator = nmx.NodeIterator.new(nmx.Node.ClassTypeId())
  local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

  bodyMap:clearMappings()

  local parts = { }

  for i = 1, mappingArraySize do
    if not bodyMappingInfo:isPartOptional(i-1) then
      table.insert(parts, i)
    end
  end

  autoMapParts(bodyMap,bodyMappingInfo, rootSceneNode, parts)

  scene:endChangeBlock(cbRef, changeBlockInfo("Auto map to template."))
end

------------------------------------------------------------------------------------------------------------------------
local autoMapOptional = function(rootDataNode, rootSceneNode, channelNames)
  local selectedSet = getSelectedAssetManagerAnimSet()
  if type(selectedSet) ~= "string" or string.len(selectedSet) == 0 then
    return
  end

  local scene = nmx.Application.new():getSceneByName("AssetManager")

  if rootDataNode == nil or rootSceneNode == nil then
    app.error("Failed to find rig data root or rig scene root when auto mapping template tags")
    return
  end
  
  local bodyMap = nmx.BodyMappingNode.getBodyMappingNode(rootSceneNode)
  local bodyMappingInfo = nmx.BodyMappingInfoNode.getBodyMappingInfoNode(rootSceneNode)
          
  if bodyMap == nil then
    app.error("Failed to body map when auto mapping template tags")
    return
  end

  local mappingArray = bodyMap:findAttributeArray("Parts")
  local mappingArraySize = mappingArray:size()
  local mappingParentInfo = bodyMap:findAttribute("ParentIndex")
  if not mappingArray:isValid() and not mappingParentInfo:isValid() then
    return
  end

  local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

  local parts = { }

  for i = 1, mappingArraySize do
    if bodyMappingInfo:isPartOptional(i-1) then
      table.insert(parts, i)
    end
  end

  autoMapParts(bodyMap,bodyMappingInfo, rootSceneNode, parts)

  scene:endChangeBlock(cbRef, changeBlockInfo("Auto map optional parts to template."))
end

------------------------------------------------------------------------------------------------------------------------
local autoMap = function(rootDataNode, rootSceneNode, channelNames, optional)
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

  if optional == TemplateOnlyRequiredTags then
    autoMapRequired(rootDataNode, rootSceneNode, channelNames, TemplateOnlyRequiredTags)
  elseif optional == TemplateOnlyOptionalTags then
    autoMapOptional(rootDataNode, rootSceneNode, channelNames, TemplateOnlyOptionalTags)
  else
    -- otherwise do both
    autoMapRequired(rootDataNode, rootSceneNode, channelNames, TemplateOnlyRequiredTags)
    autoMapOptional(rootDataNode, rootSceneNode, channelNames, TemplateOnlyOptionalTags)
  end

  scene:endChangeBlock(cbRef, changeBlockInfo("Auto map to template."))
end

------------------------------------------------------------------------------------------------------------------------
autoMapAnimationTemplateMapping = function(selectedSet, optional)
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local rootDataNode = anim.getRigDataRoot(scene, selectedSet)
  local rootSceneNode = anim.getRigSceneRoot(scene, selectedSet)

  local channelNames = anim.getRigChannelNames(selectedSet)
  table.insert(channelNames, 1, "")
  autoMap(rootDataNode, rootSceneNode, channelNames, optional)
end

------------------------------------------------------------------------------------------------------------------------
local getAnimSetTemplate = function(selectedSet)
  local templateFileName = anim.getAnimSetTemplate(selectedSet)
  local path, file = splitFilePath(templateFileName)
  return stripFilenameExtension(file)
end

------------------------------------------------------------------------------------------------------------------------
getTemplateList = function()

  local templateFilePaths = app.enumerateFiles(utils.demacroizeString(templateFolder), "*.mctmpl")
  local templateFiles = { }

  for i, v in templateFilePaths do
    local path, file = splitFilePath(v)
    table.insert(templateFiles, stripFilenameExtension(file))
  end

  return templateFiles
end

------------------------------------------------------------------------------------------------------------------------
local updateTemplateList = function(selectedSet, templateListComboBox)
  if selectedSet == nil or templateListComboBox == nil then
    app.error("Failed to update template combo box. Invalid arguements.")
    return
  end
  local templateList = getTemplateList()
  table.insert(templateList, 1, noTemplateString)
  templateListComboBox:setItems(templateList)
  templateListComboBox:setSelectedItem(getAnimSetTemplate(selectedSet))
end

------------------------------------------------------------------------------------------------------------------------
local clearMap = function(rootDataNode)
  local selectedSet = getSelectedAssetManagerAnimSet()
  if type(selectedSet) ~= "string" or string.len(selectedSet) == 0 then
    return
  end

  local scene = nmx.Application.new():getSceneByName("AssetManager")

  if rootDataNode then
    local bodyMap = rootDataNode:findChild("BodyMapping", true)
    if bodyMap == nil then
      return
    end
    bodyMap:clearMappings()
  end
end

------------------------------------------------------------------------------------------------------------------------
local removeTemplateTreeItem = function(button)
  local templateTreeControl = button:getParent():findDescendant(newTemplateTreeControlName)
  if templateTreeControl ~= nil then
    local selectedItem = templateTreeControl:getSelectedItem()
    if selectedItem ~= nil then
      selectedItem:remove()
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local orderUpTemplateTreeItem = function(button)
  local templateTreeControl = button:getParent():findDescendant(newTemplateTreeControlName)
  if templateTreeControl ~= nil then
    local selectedItem = templateTreeControl:getSelectedItem()
    if selectedItem ~= nil then
      local previousSibling = selectedItem:getPreviousSibling()
      if previousSibling then
        selectedItem:reorder(previousSibling)
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local orderDownTemplateTreeItem = function(button)
  local templateTreeControl = button:getParent():findDescendant(newTemplateTreeControlName)
  if templateTreeControl ~= nil then
    local selectedItem = templateTreeControl:getSelectedItem()
    if selectedItem ~= nil then
      local nextSibling = selectedItem:getNextSibling()
      if nextSibling then
        selectedItem:reorder(nextSibling:getNextSibling())
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local recursiveTreeControlToArrays

recursiveTreeControlToArrays = function(treeItem, tags, parentInfo, optional,left, right, groundContact, IKTarget, regularExpressions, parentIndex)
  local numChildren = treeItem:getNumChildren()
  for i = 1, numChildren do
    local childItem = treeItem:getChild(i)
    tags:push_back(childItem:getValue())
    parentInfo:push_back(parentIndex)

    optional:push_back(childItem:getColumnCheckbox(2) == 1)
    left:push_back(childItem:getColumnCheckbox(3) == 1)
    right:push_back(childItem:getColumnCheckbox(4) == 1)
    groundContact:push_back(childItem:getColumnCheckbox(5) == 1)
    IKTarget:push_back(childItem:getColumnCheckbox(6) == 1)
    regularExpressions:push_back(treeItem:getColumnValue(7))
    
    
    recursiveTreeControlToArrays(childItem, tags, parentInfo, optional,left, right, groundContact, IKTarget, regularExpressions, parentInfo:size() - 1)
  end
end

------------------------------------------------------------------------------------------------------------------------
local saveTemplate = function(self)

  -- Convert the tree control into a tag and parent info array
  local tags = nmx.StringArray.new()
  local parentInfo = nmx.IntArray.new()
  local optional = nmx.BoolArray.new()
  local left = nmx.BoolArray.new()
  local right = nmx.BoolArray.new()
  local groundContact = nmx.BoolArray.new()
  local IKTarget = nmx.BoolArray.new()
  
  local regularExpressions = nmx.StringArray.new()
  local templateTreeControl = self:getParent():findDescendant(newTemplateTreeControlName)

  local offsetHelpTextControl = self:getParent():findDescendant(newTemplateOffsetHelpTextControl)
  local offsetImageControl = self:getParent():findDescendant(newTemplateOffsetImageTextControl)
  local templateHelpTextControl = self:getParent():findDescendant(newTemplateHelpTextControl)
  local tempImageControl= self:getParent():findDescendant(newTemplateImageTextControl)
  
  local root = templateTreeControl:getRoot()
  recursiveTreeControlToArrays(root, tags, parentInfo, optional, left, right, groundContact, IKTarget,regularExpressions, -1)

  if tags:empty() then
    app.error("Can't save an empty template.");
    return
  end

  local saveDlg = ui.createFileDialog{
    style = "save",
    caption = "Save Template As",
    wildcard = "Morpheme Connect template files|mctmpl",
    directory = utils.demacroizeString(templateFolder)
  }

  if not saveDlg:show() then
    return
  end

  local savePath = saveDlg:getFullPath()

  -- Populate a body mapping node with the tags and parent info
  local nmxApp = nmx.Application.new()
  local newDB = nmxApp:createDatabase("TempBodyMappingDB", "", 0)
  local status, cbRef = newDB:beginChangeBlock(getCurrentFileAndLine())
  local bodyMapping = newDB:createNode(nmx.BodyMappingNode.ClassTypeId(), "BodyMapping", newDB:getRoot())
  local bodyMappingInfo = newDB:createNode(nmx.BodyMappingInfoNode.ClassTypeId(), "BodyMappingInfo", newDB:getRoot())
  
  bodyMapping:populateFromArrays(tags, parentInfo)
  
  if(bodyMappingInfo ~= nil) then 
    bodyMappingInfo:populateFromArrays( optional, left, right, groundContact, IKTarget, regularExpressions)
  end
  
  local templateNode = newDB:createNode(nmx.RetargetingTemplateNode.ClassTypeId(), "RetargetingTemplate", newDB:getRoot())
  templateNode:setCalculateOffsetsHelpText(offsetHelpTextControl:getValue())
  local macroizedOffsetImage   = utils.macroizeString(offsetImageControl:getValue())
  templateNode:setCalculateOffsetsHelpImage(macroizedOffsetImage)
  templateNode:setTemplateHelpText(templateHelpTextControl:getValue())
  local macroizedTemplateImage = utils.macroizeString(tempImageControl:getValue())
  templateNode:setTemplateHelpImage(macroizedTemplateImage)
 
  newDB:endChangeBlock(cbRef, changeBlockInfo("CreateBodyMapping"))
  newDB:saveFileAs(savePath, 0)
  nmxApp:destroyDatabase(newDB)

  
  self:getParent():hide()

  -- The template folder has a new template so update the list
  local selectedSet = getSelectedAssetManagerAnimSet()
  local attributeEditor = ui.getWindow(componentEditorPath)
  if type(selectedSet) == "string" and string.len(selectedSet) > 0 and attributeEditor ~= nil then
    local templateComboBox = attributeEditor:findDescendant(templateListComboBoxName)
    if templateComboBox ~= nil then
      updateTemplateList(selectedSet, templateComboBox)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- createTemplateDialogue
------------------------------------------------------------------------------------------------------------------------
createTemplateDialogue = function()
  local dlg = ui.getWindow("TemplateCreationDialogue")
  if not dlg then
    dlg = ui.createModalDialog{
      name = "TemplateCreationDialogue",
      caption = "Template Creation",
      size = { width = 450, height = 400 }
    }

    dlg:beginHSizer{ flags = "expand", proportion = 1 }
      dlg:beginVSizer{ flags = "expand", proportion = 1 }

        -- Top row of buttons
        dlg:beginHSizer()
          dlg:addButton{ image = addImage, onClick = addTemplateTreeItem }:setToolTip("Add template tag")
          dlg:addButton{ image = removeImage, onClick = removeTemplateTreeItem }:setToolTip("Remove template tag")
          dlg:addButton{ image = orderUpImage, onClick = orderUpTemplateTreeItem }:setToolTip("Move up")
          dlg:addButton{ image = orderDownImage, onClick = orderDownTemplateTreeItem }:setToolTip("Move down")
        dlg:endSizer()

        -- Tag tree control
        dlg:addTreeControl{
          name = newTemplateTreeControlName,
          flags = "expand;rename;hideRoot;stripe",
          proportion = 1,
          columnNames = { "Tag", "Optional", "Left", "Right", "GroundContact", "IKTarget","Regular Expressions" },
          preItemRenamed = onTemplateTreeItemRenamed,
          onItemActivated = onTemplateTreeItemActivated,
          allowDrag = true,
          onDrop = function(self, dropItems, dropItem)
            for i,v in pairs(dropItems) do
              if v:getTreeControl() == self then
                v:reparent(dropItem)
              end
            end
          end,
          onShowInPlaceEditor = function (self, item, column)
              if column > 2 then
                self:popupEditor(item, column)
              end
          end,
          onEditColumn = function (self, item, column, value)
             if column > 2 then
                item:setColumnValue(column, value)
              end
            end,
        }
        
        dlg:beginHSizer{ flags = "expand", proportion = 0 }
         dlg:addStaticText{ text = "Template Help" }
              dlg:addTextBox{
              name = newTemplateHelpTextControl,
              flags = "expand",
              proportion = 1
            }
        dlg:endSizer()

        dlg:beginHSizer{ flags = "expand", proportion = 0 }
         dlg:addStaticText{ text = "Template Image" }
                dlg:addHSpacer(3)
                dlg:addFilenameControl{
                  name      = newTemplateImageTextControl,
                  caption   = "Pick the image to show in the template UI component.",
                  wildcard  = "Portable Network Graphics|png",
                  dialogStyle = "mustExist",
                  directory = utils.demacroizeString(templateFolder),
                  flags = "expand",
                  proportion = 1
                }
        dlg:endSizer()

        dlg:beginHSizer{ flags = "expand", proportion = 0 }
         dlg:addStaticText{ text = "Compute Offsets Help" }
              dlg:addTextBox{
              name = newTemplateOffsetHelpTextControl,
              flags = "expand",
              proportion = 1
            }
        dlg:endSizer()

        dlg:beginHSizer{ flags = "expand", proportion = 0 }
         dlg:addStaticText{ text = "Compute Offsets Image" }
                dlg:addHSpacer(3)
                dlg:addFilenameControl{
                  name      = newTemplateOffsetImageTextControl,
                  caption   = "Pick the image to show in the compute offsets dialog.",
                  wildcard  = "Portable Network Graphics|png",
                  dialogStyle = "mustExist",
                  directory = utils.demacroizeString(templateFolder),
                  flags = "expand",
                  proportion = 1
                }
        dlg:endSizer()

        -- Bottom row of buttons
        dlg:beginHSizer{ flags = "right" }

          dlg:addButton{ label = "Save", onClick = saveTemplate }
          dlg:addButton{
            label = "Cancel",
            onClick =  function(self)
              dlg:hide()
            end
           }

        dlg:endSizer()
      dlg:endSizer()
    dlg:endSizer()
  end

  local newTemplateTreeControl = dlg:findDescendant(newTemplateTreeControlName)
  newTemplateTreeControl:getRoot():clearChildren()

  dlg:show()
end

------------------------------------------------------------------------------------------------------------------------
local validateFunction = function(selectedSet, isCurrentComponent)
  if not anim.isRigValid() then
    return "Inactive"
  end

  local templateState = "Inactive"
  -- No need to validate body maps if templates are not being used
  local template = getAnimSetTemplate(selectedSet)
  if template ~= noTemplateString and type(template) == "string" and string.len(template) > 0 then
    templateState = "Ok"

    -- Make sure the animation body map is complete
    local scene = nmx.Application.new():getSceneByName("AssetManager")
    local rootNode = anim.getRigSceneRoot(scene, selectedSet)
    if rootNode then
      local bodyMap = nmx.BodyMappingNode.getBodyMappingNode(rootNode)
      local bodyMappingInfo = nmx.BodyMappingInfoNode.getBodyMappingInfoNode(rootNode)

      if bodyMappingInfo ~= nil then 
        if not bodyMap:isComplete(bodyMappingInfo) then
          templateState = "Warning"
        end
      end
      
      local attributeEditor = ui.getWindow(componentEditorPath)
      if attributeEditor ~= nil then
        local animationMappingRollUp = attributeEditor:findDescendant(animationMappingRollUpName)
        if animationMappingRollUp ~= nil then
          local channelNames = anim.getRigChannelNames(selectedSet)
          table.insert(channelNames, 1, "")
          updateMappingComboBoxes(animationMappingRollUp:getPanel(), bodyMap,bodyMappingInfo,  channelNames)
        end
      end
    end


    -- Make sure the physics body map is complete
    local setType = anim.getAnimSetCharacterType(selectedSet)
    if characterTypes.supportsRig(setType, "PhysicsRig") then
      local physicsRigPath = anim.getPhysicsRigPath(selectedSet)
      if type(physicsRigPath) == "string" and string.len(physicsRigPath) > 0 and anim.isPhysicsRigValid(selectedSet) then
        local physicsRootNode = anim.getPhysicsRigSceneRoot(scene, selectedSet)
        if physicsRootDataNode then
          local physicsBodyMap = nmx.BodyMappingNode.getBodyMappingNode(physicsRootNode)
          local bodyMappingInfo = nmx.BodyMappingInfoNode.getBodyMappingInfoNode(physicsRootNode)
          if bodyMappingInfo ~= nil then 
            if not physicsBodyMap:isComplete(bodyMappingInfo) then
              templateState = "Warning"
            end
          end

          local attributeEditor = ui.getWindow(componentEditorPath)
          if attributeEditor ~= nil then
            local physicsMappingRollUp = attributeEditor:findDescendant(physicsMappingRollUpName)
            if physicsMappingRollUp ~= nil then
              local channelNames = anim.getPhysicsRigChannelNames(selectedSet)
              table.insert(channelNames, 1, "")
              updateMappingComboBoxes(physicsMappingRollUp, physicsBodyMap, bodyMappingInfo, channelNames)
            end
          end
        end
      end
    end

  end

  return templateState
end

------------------------------------------------------------------------------------------------------------------------
local getMapEntryDepth = function(bodyMapInfo, parentInfo, i, includeOptional)
  local depth = 0
  local parentIndex = parentInfo:at(i)
  while parentIndex ~= -1 do
    if includeOptional or not bodyMapInfo:isPartOptional(parentIndex) then
      depth = depth + 1
    end
    parentIndex = parentInfo:at(parentIndex + 1)
  end

  return depth
end

------------------------------------------------------------------------------------------------------------------------
local onMappingComboChanged = function(self, selectedSet)
  selectedSet = selectedSet or getSelectedAssetManagerAnimSet()
  local scene = nmx.Application.new():getSceneByName("AssetManager")

  local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

  rootDataNode = anim.getRigDataRoot(scene, selectedSet)
  local bodyMap = rootDataNode:findChild("BodyMapping", true)
  bodyMap:clearPart(self:getName())
  if self:getSelectedItem() ~= "" then
    bodyMap:setPart(anim.getRigJoint(scene, selectedSet, self:getSelectedItem()), self:getName(), true)
  end

  scene:endChangeBlock(cbRef, changeBlockInfo("Set body map part."))
end


------------------------------------------------------------------------------------------------------------------------
local onPhysicsMappingComboChanged = function(self, selectedSet)
  selectedSet = selectedSet or getSelectedAssetManagerAnimSet()
  local scene = nmx.Application.new():getSceneByName("AssetManager")

  local status, cbRef = scene:beginChangeBlock(getCurrentFileAndLine())

  rootDataNode = anim.getPhysicsRigDataRoot(scene, selectedSet)
  local bodyMap = rootDataNode:findChild("BodyMapping", true)
  bodyMap:clearPart(self:getName())
  if self:getSelectedItem() ~= "" then
    bodyMap:setPart(anim.getPhysicsRigJoint(scene, selectedSet, self:getSelectedItem()), self:getName(), true)
  end

  scene:endChangeBlock(cbRef, changeBlockInfo("Set body map part."))
end

local helpImageCache = { }
------------------------------------------------------------------------------------------------------------------------
local populateListControlFromMapping = function(panel, rootSceneNode, rootDataNode, channelNames, showOptional, physics, 
         changedCallback, selectedSet, layoutLeft)
  if rootDataNode == nil then
    return
  end

  local bodyMap =  nmx.BodyMappingNode.getBodyMappingNode(rootSceneNode)
  if bodyMap == nil then
    return
  end
  
  local bodyMapInfo = nmx.BodyMappingInfoNode.getBodyMappingInfoNode(rootSceneNode)

  local mappingArray = bodyMap:findAttributeArray("Parts")
  local mappingParentInfo = bodyMap:findAttribute("ParentIndex")
  if not mappingArray:isValid() and not mappingParentInfo:isValid() then
    return
  end

  panel:beginVSizer{ flags = "expand" }

    local help = anim.getAnimSetRetargetingHelp(selectedSet)
    local helpImage = nil

    if help and anim.getAnimSetTemplate(selectedSet) ~= "" then
      local helpText = ""
      if help.templateText ~= nil and help.templateText ~= "" then
        helpText = help.templateText
      end

      if help.templateImage ~= nil and help.templateImage ~= "" then
        helpImage = help.templateImage
      end

      if helpText ~= nil then
        panel:addTextControl{
          value = helpText,
          size = { width = 256, height = 30 },
          flags = "readonly;expand;dialogColours", proportion = 0
        }
      end

      if layoutLeft == true then
          panel:beginHSizer{flags = "expand" , proportion = 1}
      end

      if helpImage ~= nil then
        local imagePath = utils.demacroizeString(help.templateImage)

        if not helpImageCache[imagePath] then
          helpImageCache[imagePath] = ui.createImage(imagePath)
        end
        local helpImage = helpImageCache[imagePath]
        if helpImage then
          panel:beginHSizer{ flags = "center" }
            panel:addStaticBitmap{ image = helpImage, flags = "center" }
          panel:endSizer()
        end
      end
    end

    panel:setBorder(0)

    if layoutLeft ~= true then 
      panel:addVSpacer(10)
    else
      panel:addHSpacer(10)
      panel:beginVSizer{ flags = "expand", proportion = 1 }
    end
    
    panel:beginFlexGridSizer{ cols = 2, flags = "expand" }
    panel:setFlexGridColumnExpandable(2)

      panel:addStaticText{ text = "Tag" , font = "bold" }
      panel:addStaticText{ text = "Joint", font = "bold" }
      panel:addVSpacer(5)
      panel:addVSpacer(5)

      table.insert(channelNames, 1, "")
      local mappingArraySize = mappingArray:size()
      local mappingParentInfoArray = mappingParentInfo:asIntArray()
      for i = 1, mappingArraySize do
        local shouldShow = showOptional ~= TemplateOnlyRequiredTags or not bodyMapInfo:isPartOptional(i-1)
        if shouldShow then
          local shouldEnable = showOptional ~= TemplateOnlyOptionalTags or bodyMapInfo:isPartOptional(i-1)
          -- Indent the label by the mappings hierarchy depth
          local attribute = mappingArray:getAttribute(i)
          local outputAttribute = attribute:getLastOutput()
          local depth = getMapEntryDepth(bodyMapInfo, mappingParentInfoArray, i, showOptional ~= TemplateOnlyRequiredTags)
          local attributeName = attribute:asString()
          local strLen = string.len(attributeName)
          local stringFormat = string.format("%s%ss    ", "%", depth * 2 + strLen)
          local textLabel = string.format(stringFormat, attributeName)

          panel:addStaticText{ text = textLabel }
          local comboBox = panel:addComboBox{
            name = attributeName,
            flags = "expand",
            proportion = 1,
            onChanged = function(self)
              if not physics then
                onMappingComboChanged(self, selectedSet)
              else
                onPhysicsMappingComboChanged(self, selectedSet)
              end

              updateMappingComboBoxes(panel, bodyMap,bodyMapInfo, channelNames)
              if changedCallback ~= nil then
                changedCallback()
              end
            end
          }
          comboBox:enable(shouldEnable)
        end
      end

    updateMappingComboBoxes(panel, bodyMap,bodyMapInfo, channelNames)

    panel:endSizer()


    panel:setBorder(2)
    panel:addVSpacer(2)

    panel:beginHSizer{ flags = "right" }
      panel:addButton{
        name = "AutoButton",
        label = "Auto",
        size = { width = 74, height = -1 },
        onClick = function()
          autoMap(rootDataNode, rootSceneNode, channelNames, TemplateAllTags)
          updateMappingComboBoxes(panel, bodyMap,bodyMapInfo, channelNames)

          if changedCallback ~= nil then
            changedCallback()
          end
        end
      }

      panel:addButton{
        name = "ClearButton",
        label = "Clear",
        size = { width = 74, height = -1 },
        onClick = function(self)
          clearMap(rootDataNode)
          updateMappingComboBoxes(panel, bodyMap,bodyMapInfo, channelNames)

          if changedCallback ~= nil then
            changedCallback()
          end
        end
      }
    panel:endSizer()

    if layoutLeft == true and helpImage ~= nil then 
      panel:endSizer()
      panel:endSizer()
    end
    
  panel:endSizer()

end

------------------------------------------------------------------------------------------------------------------------
populateAnimationMapping = function(panel, selectedSet, changedCallback, showOptional, layoutLeft)
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local rootDataNode = anim.getRigDataRoot(scene, selectedSet)
  local rootSceneNode = anim.getRigSceneRoot(scene, selectedSet)
  populateListControlFromMapping(panel, rootSceneNode, rootDataNode, anim.getRigChannelNames(selectedSet), showOptional, false, changedCallback, selectedSet, layoutLeft)
end

------------------------------------------------------------------------------------------------------------------------
populatePhysicsMapping = function(listControl, selectedSet, showOptional, layoutLeft)
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local rootDataNode = anim.getPhysicsRigDataRoot(scene, selectedSet)
  local rootSceneNode = anim.getPhysicsRigSceneRoot(scene, selectedSet)
  populateListControlFromMapping(listControl, rootSceneNode, rootDataNode, anim.getPhysicsRigChannelNames(selectedSet), showOptional, true, changedCallback, selectedSet, layoutLeft)
end

------------------------------------------------------------------------------------------------------------------------
local onTemplateAlgorithmButtonPressed = function(button, selectedSet, algorithms, usesAnimRig, usesPhysicsRig)
  if ui.showMessageBox("Running template algorithms may overwrite data.\nAre you sure you want to continue?",
      "ok;cancel") == "ok" then

    local sourceAnimPath = ""
    if usesAnimRig then
      sourceAnimPath = button:getParent():findDescendant("SourceAnimRig"):getValue()
      if not app.fileExists(sourceAnimPath) then
        ui.showErrorMessageBox("Source animation rig does not exist.")
        return
      end
    end

    local sourcePhysicsPath = ""
    if usesPhysicsRig then
      sourcePhysicsPath = button:getParent():findDescendant("SourcePhysicsRig"):getValue()
      if not app.fileExists(sourcePhysicsPath) then
        ui.showErrorMessageBox("Source physics rig does not exist.")
        return
      end
    end

    anim.runAnimSetTemplateAlgorithm(selectedSet, algorithms, sourceAnimPath, sourcePhysicsPath)
  end
end

------------------------------------------------------------------------------------------------------------------------
local onUseExistingKinematicTemplate = function(self)

  local saveFileDialog = ui.createFileDialog{
    style = "save;prompt",
    caption = "Save Animation Rig Template",
    wildcard = "Template|mctmpl",
    directory = utils.demacroizeString(templateFolder)
  }

  if saveFileDialog:show() then
    local selectedSet = getSelectedAssetManagerAnimSet()
    local scene = nmx.Application.new():getSceneByName("AssetManager")
    local rootDataNode = anim.getRigDataRoot(scene, selectedSet)
    local bodyMapping = rootDataNode:findChild("BodyMapping", true)
    if bodyMapping then
      bodyMapping:saveToFile(saveFileDialog:getFullPath())
      updateTemplateList(selectedSet, self:getParent(templateListComboBoxName))
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local onUseExistingPhysicsTemplate = function()
  local saveFileDialog = ui.createFileDialog{
    style = "save;prompt",
    caption = "Save Physics Rig Template",
    wildcard = "Template|mctmpl",
    directory = utils.demacroizeString(templateFolder)
  }

  if saveFileDialog:show() then
    local selectedSet = getSelectedAssetManagerAnimSet()
    local scene = nmx.Application.new():getSceneByName("AssetManager")
    local rootDataNode = anim.getPhysicsRigDataRoot(scene, selectedSet)
    local bodyMapping = rootDataNode:findChild("BodyMapping", true)
    if bodyMapping then
      bodyMapping:saveToFile(saveFileDialog:getFullPath())
    end
    local attributeEditor = ui.getWindow(componentEditorPath)
    if attributeEditor ~= nil then
      local templateListComboBox = attributeEditor:findDescendant(templateListComboBoxName)
      if templateListComboBox then
        updateTemplateList(selectedSet, templateListComboBox)
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local onTemplateChanged = function(self)
  animSetTemplate = self:getSelectedItem()
  local selectedSet = getSelectedAssetManagerAnimSet()
  local uiPanel = self:getParent()

  local templateFilePath
  if animSetTemplate == noTemplateString then
    templateFilePath = ""
  else
    templateFilePath = string.format("%s\\%s.mctmpl", templateFolder, animSetTemplate)
  end

  if anim.setAnimSetTemplate(selectedSet, templateFilePath) then
    componentEditor.reset()
    componentEditor.updateAllComponentsValidity()
  else
    updateTemplateList(selectedSet, self)
  end
end

------------------------------------------------------------------------------------------------------------------------
isAnimationTemplateMappingComplete = function(animationSet)
  local scene = nmx.Application.new():getSceneByName("AssetManager")
  local rootNode = anim.getRigSceneRoot(scene, animationSet)
  local bodyMappingNode = nmx.BodyMappingNode.getBodyMappingNode(rootNode)
  local bodyMappingInfoNode = nmx.BodyMappingInfoNode.getBodyMappingInfoNode(rootNode)
  
  if bodyMappingNode == nil or bodyMappingInfoNode == nil then
    app.error("Failed to update mapping combo boxes, invalid arguements.")
    return
  end

  local mappingArray = bodyMappingNode:findAttributeArray("Parts")
  local mappingArraySize = mappingArray:size()
  if not mappingArray:isValid() then
    app.error("Failed to update mapping combo boxes, invalid arguements.")
    return
  end

  -- check all mappings are filled
  for i = 1, mappingArraySize do
    local attributeName = mappingArray:getAttribute(i):asString()

    local result, partIndex = bodyMappingNode:findPart(attributeName)
    if not bodyMappingNode:isValidPart(partIndex, bodyMappingInfoNode) then
      return false
    end
  end

  return true
end

------------------------------------------------------------------------------------------------------------------------
createUIFunction = function(selectedSet, panel)
  templateUIElements = {}
  local setType = anim.getAnimSetCharacterType(selectedSet)
  local characterSupportsPhysics = characterTypes.supportsRig(setType, "PhysicsRig")
  local characterSupportsEuphoria = setType == "Euphoria"

  local physicsRigPath = anim.getPhysicsRigPath()
  if physicsRigPath == nil or type(physicsRigPath) ~= "string" or string.len(physicsRigPath) == 0 then
    characterSupportsPhysics = false
  end

  panel:beginVSizer{ flags = "expand" }
    -- Template combo and buttons
    panel:beginHSizer{ flags = "expand" }
      panel:addStaticText{ text = "Template", font = "bold" }
      local templateListComboBox = panel:addComboBox{
        name = templateListComboBoxName,
        flags = "expand",
        proportion = 1 }

      updateTemplateList(selectedSet, templateListComboBox)

      -- TODO: cover case when the template isn't found
      local setTemplate = getAnimSetTemplate(selectedSet)
      if setTemplate == noTemplateString or setTemplate:len() == 0 then
        templateListComboBox:setSelectedItem(noTemplateString)
      else
        templateListComboBox:setSelectedItem(setTemplate)
      end

      -- Set the on changed property after the initial value has been set so that onTemplateChanged() isn't called
      templateListComboBox:setOnChanged(onTemplateChanged)

      local createTemplateButton = panel:addButton{
        image = addImage,
        size = { width = 16 ,  height = 16 },
        onClick = createTemplateDialogue
      }
      createTemplateButton:setToolTip("Create new template");
    panel:endSizer()

    if type(setTemplate) == "string" and string.len(setTemplate) > 0 and setTemplate ~= noTemplateString then

      panel:beginVSizer{ flags = "expand", center = true }

        -- Animation mapping list control
        local animationMappingRollup = panel:addRollup{ name = "AnimationMappingRollup", label = "Animation Mapping", flags = "expand;mainSection" }
        local animationMappingPanel = animationMappingRollup:getPanel()
        animationMappingRollUpName = animationMappingRollup:getName()
        populateAnimationMapping(animationMappingPanel, selectedSet, nil, TemplateAllTags, false)
        animationMappingRollup:expand(true)

        -- Physics mapping list control
        if characterSupportsPhysics and anim.isPhysicsRigValid(selectedSet) then
          local physicsMappingRollup = panel:addRollup{ name = "PhysicsMappingRollup", label = "Physics Mapping", flags = "expand;mainSection" }
          local physicsMappingPanel = physicsMappingRollup:getPanel()
          physicsMappingRollUpName = physicsMappingRollup:getName()
          populatePhysicsMapping(physicsMappingPanel, selectedSet, nil, TemplateAllTags, false)
          physicsMappingRollup:expand(true)
        end

        -- Source data
        local copyDataMappingRollup = panel:addRollup{ label = "Source Data", flags = "expand;mainSection" }
        local copyDataRollPanel = copyDataMappingRollup:getPanel()
        copyDataRollPanel:beginVSizer{ flags = "expand" }
          copyDataRollPanel:beginFlexGridSizer{ cols = 2, flags = "expand" }
          copyDataRollPanel:setFlexGridColumnExpandable(2)

            copyDataRollPanel:addStaticText{ text = "Animation Rig" }
            templateUIElements.kinematicRigFilePath = copyDataRollPanel:addFilenameControl{
              name = "SourceAnimRig",
              flags = "expand",
              wildcard = "morpheme:connect animation rig files|mcarig",
              value = currentsourceanimrigpath,
              onChanged = function(self)
                currentSourceAnimRigPath = self:getValue()
                if app.fileExists(currentSourceAnimRigPath) then
                  templateUIElements.kinematicTagsButton:enable(true)
                  templateUIElements.bodyGroupsButton:enable(true)
                  templateUIElements.allDataButton:enable(true)
                else
                  templateUIElements.kinematicTagsButton:enable(false)
                  templateUIElements.bodyGroupsButton:enable(false)
                  if not templateUIElements.physicsRigFilePath or (templateUIElements.physicsRigFilePath and
                    not app.fileExists(templateUIElements.physicsRigFilePath:getValue())) then
                    templateUIElements.allDataButton:enable(false)
                  end
                end
              end
            }

            if characterSupportsPhysics then
              copyDataRollPanel:addStaticText{ text = "Physics Rig" }
              templateUIElements.physicsRigFilePath = copyDataRollPanel:addFilenameControl{
                name = "SourcePhysicsRig",
                flags = "expand",
                wildcard = "morpheme:connect physics rig files|mcprig",
                value = currentSourcePhysicsRigPath,
                onChanged = function(self)
                  currentSourcePhysicsRigPath = self:getValue()
                  if app.fileExists(currentSourcePhysicsRigPath) then
                    templateUIElements.physicsTagsButton:enable(true)
                    templateUIElements.collisionSetsButton:enable(true)
                    templateUIElements.allDataButton:enable(true)
                    if templateUIElements.limbsButton ~= nil then
                      templateUIElements.limbsButton:enable(true)
                    end
                  else
                    templateUIElements.physicsTagsButton:enable(false)
                    templateUIElements.collisionSetsButton:enable(false)
                    if not app.fileExists(templateUIElements.kinematicRigFilePath:getValue()) then
                      templateUIElements.allDataButton:enable(false)
                    end
                    if templateUIElements.limbsButton ~= nil then
                      templateUIElements.limbsButton:enable(false)
                    end
                  end
                end
              }
            end

          copyDataRollPanel:endSizer()

          -- Copy buttons
          copyDataRollPanel:addVSpacer(10)
          copyDataRollPanel:beginVSizer{ center = true, flags = "center" }

            templateUIElements.allDataButton = copyDataRollPanel:addButton {
              name = "CopyAllDataButton",
              label = "Copy All Data",
              center = true,
              size = { width = 128 },
              onClick = function(self)
                local selectedSet = getSelectedAssetManagerAnimSet()
                local setType = anim.getAnimSetCharacterType(selectedSet)
                local characterSupportsPhysics = characterTypes.supportsRig(setType, "PhysicsRig")
                local characterSupportsEuphoria = setType == "Euphoria"

                local algorithms = {}
                local usesKinematicRig = false
                local usesPhysicsRig = false

                if app.fileExists(currentSourceAnimRigPath) then
                  usesKinematicRig = true
                  table.insert(algorithms, "KinematicTags")
                  table.insert(algorithms, "BodyGroups")
                end

                if characterTypes.supportsRig(setType, "PhysicsRig") and
                  currentSourcePhysicsRigPath ~= nil and
                  app.fileExists(currentSourcePhysicsRigPath) then
                  usesPhysicsRig = true
                  table.insert(algorithms, "PhysicsTags")
                  table.insert(algorithms, "CollisionSets")
                  if characterSupportsEuphoria then
                    table.insert(algorithms, "Limbs")
                  end
                end

                onTemplateAlgorithmButtonPressed(self, selectedSet, algorithms, usesKinematicRig, usesPhysicsRig)
              end
            }
            templateUIElements.kinematicTagsButton = copyDataRollPanel:addButton {
              name = "CopyTagsButton",
              label = "Copy Kinematic Tags",
              center = true,
              size = { width = 128 },
              onClick = function(self)
                local selectedSet = getSelectedAssetManagerAnimSet()
                onTemplateAlgorithmButtonPressed(self, selectedSet, "KinematicTags", true, false)
              end
            }
            templateUIElements.bodyGroupsButton = copyDataRollPanel:addButton {
              label = "Copy Body Groups",
              center = true,
              size = { width = 128 },
              onClick = function(self)
                local selectedSet = getSelectedAssetManagerAnimSet()
                onTemplateAlgorithmButtonPressed(self, selectedSet, "BodyGroups", true, false)
              end
            }
            if characterSupportsPhysics then
              templateUIElements.physicsTagsButton = copyDataRollPanel:addButton {
                name = "CopyPhysicsButton",
                label = "Copy Physics Tags",
                center = true,
                size = { width = 128 },
                onClick = function(self)
                  local selectedSet = getSelectedAssetManagerAnimSet()
                  onTemplateAlgorithmButtonPressed(self, selectedSet, "PhysicsTags", false, true)
                end
              }
              templateUIElements.collisionSetsButton = copyDataRollPanel:addButton {
                name = "CopyCollisionSetsButton",
                label = "Copy Collision Sets",
                center = true,
                size = { width = 128 },
                onClick = function(self)
                  local selectedSet = getSelectedAssetManagerAnimSet()
                  onTemplateAlgorithmButtonPressed(self, selectedSet, "CollisionSets", false, true)
                end
              }
            end

            if characterSupportsEuphoria then
              templateUIElements.limbsButton = copyDataRollPanel:addButton {
                name = "CopyLimbsButton",
                label = "Copy Limbs",
                center = true,
                size = { width = 128 },
                onClick = function(self)
                  local selectedSet = getSelectedAssetManagerAnimSet()
                  onTemplateAlgorithmButtonPressed(self, selectedSet, "Limbs", false, true)
                end
              }
            end

          copyDataRollPanel:endSizer()
        copyDataRollPanel:endSizer()
        copyDataMappingRollup:expand(false)
      panel:endSizer()
    end
  panel:endSizer()

  -- Call the on onChanged file path callbacks so thet the relevent buttons are enabled/disabled
  if templateUIElements.kinematicRigFilePath ~= nil then
    templateUIElements.kinematicRigFilePath:getOnChanged()(templateUIElements.kinematicRigFilePath)
  end
  if templateUIElements.physicsRigFilePath ~= nil then
    templateUIElements.physicsRigFilePath:getOnChanged()(templateUIElements.physicsRigFilePath)
  end

  componentEditor.addOnBodyMappingChanged(function()
    componentEditor.updateComponentValidity("Templates")
  end)
end

------------------------------------------------------------------------------------------------------------------------
components.register("Templates",  {"Animation", "Physics", "Euphoria"}, validateFunction, createUIFunction)
