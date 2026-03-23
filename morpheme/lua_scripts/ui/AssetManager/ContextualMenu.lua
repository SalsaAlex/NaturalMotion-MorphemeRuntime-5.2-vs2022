local getAssetManagerWindow = function()
  return ui.getWindow("MainFrame|LayoutManager|AssetManager")
end

------------------------------------------------------------------------------------------------------------------------
-- adds menu items for a resource that is an animation location
------------------------------------------------------------------------------------------------------------------------
local isRetargetingEnabled = function()
  local selectedSet = getSelectedAssetManagerAnimSet()
  local activeSource = anim.getActiveRetargetingSource(selectedSet)
  return activeSource ~= ""
end

------------------------------------------------------------------------------------------------------------------------
-- adds menu items for a resource that is an animation location
------------------------------------------------------------------------------------------------------------------------
local addAnimationLocationMenuItems = function(menu, resourceId, selectedResourceIds)
  menu:addItem{
    label = "Edit location...",
    onClick = function(self)
      local assetManager = getAssetManagerWindow()
      assetManager:editLocation(resourceId)
    end,
  }
  menu:addItem{
    label = "Remove location",
    onClick = function(self)
      local assetManager = getAssetManagerWindow()
      assetManager:removeLocation(resourceId)
    end,
  }
end

------------------------------------------------------------------------------------------------------------------------
-- adds menu items for a resource that is an animation file or animation take
------------------------------------------------------------------------------------------------------------------------
local addAnimationMenuItems = function(menu, resourceId, selectedResourceIds)
  if anim.getResourceIsMissing(resourceId) then
    menu:addItem{
      label = "Locate folder...",
      onClick = function(self)
        local assetManager = getAssetManagerWindow()
        local set = getSelectedAssetManagerAnimSet()
        for k,v in pairs(selectedResourceIds) do
          assetManager:locateMissingFile(v, set)
        end
      end,
    }
  else
    if anim.takeMarkupDataChanged(resourceId) then
      menu:addItem{
        label = "Save markup changes",
        onClick = function(self)
          for k,v in pairs(selectedResourceIds) do
            anim.saveMarkup(v)
          end
        end,
      }
    end

    local createRigSubMenu = menu:addSubMenu{
      label = "Create new rig for set",
    }

    -- Get the file resource id incase we've selected a take.
    local fileResourceId = anim.findFirstAncestor(resourceId, "file")

    -- add the per animation set create rig menu items
    local sets = listAnimSets()
    for _, set in ipairs(sets) do
      -- this copy is required as otherwise set is nil when used from within the onClick function below
      local current_set = set

      createRigSubMenu:addItem{
        label = set,
        onClick = function(self)
          newRigForSetWizard(current_set, anim.getResourcePath(fileResourceId))
        end,
      }
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local addResourceTypeSpecificMenuItems = function(menu, resourceId, selectedResourceIds)
  local resourceType = anim.getResourceType(resourceId)

  if anim.isAnimationLocation(resourceId) and not isRetargetingEnabled() then
    addAnimationLocationMenuItems(menu, resourceId, selectedResourceIds)
  elseif resourceType == "file" or resourceType == "take" then
    addAnimationMenuItems(menu, resourceId, selectedResourceIds)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Returns a table of animations sets key boolean pairs of animation sets with their character type
-- set to Euphoria.
-- local sets, containsEuphoriaSets = listEuphoriaAnimSets(listAnimSets())
-- if sets["FemaleCharacter"] then
--   -- FemaleCharacter is a euphoria animation set
-- end
------------------------------------------------------------------------------------------------------------------------
local listEuphoriaAnimSets = function(sets)
  local euphoriaSets = {}

  local containsEuphoriaSets = false
  for i, set in ipairs(sets) do
    if anim.getAnimSetCharacterType(set) == "Euphoria" then
      containsEuphoriaSets = true
      euphoriaSets[set] = true
    end
  end

  return euphoriaSets, containsEuphoriaSets
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local addSetAsPoseLocationMenuItem = function(menu, menuLabel, set, resourceId)
  if   (not anim.isResource(resourceId)) 
    or (not anim.isRigValid(set))
    or (anim.getResourceType(resourceId) ~= "directory") 
    or (anim.getResourceId(anim.getRigPosesLocation(set)) == resourceId) then
    return
  end

  local item = menu:addItem{
    label = menuLabel,
    onClick = function(self)
      local location = anim.getResourcePath(resourceId)
      anim.setRigPosesLocation(set, location)
    end,
  }
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
local addSetAsPoseMenuItem = function(menu, menuLabel, set, poseName, resourceId)
  if   (not anim.isResource(resourceId)) 
    or (not anim.isRigValid(set))
    or (anim.getResourceType(resourceId) ~= "take") then
    return
  end

  local fileResourceId = anim.findFirstAncestor(resourceId, "file")
  if fileResourceId == 0 then
    return
  end
  
  local pose = {
    filename = anim.getResourcePath(fileResourceId),
    takename = anim.getResourceName(resourceId),
  }

  -- Add if it has a valid rig and the
  -- take resource is a descendant of the poses location
  local location = anim.getRigPosesLocation(set)
  if anim.isResource(location) then
    local locationResourceId = anim.getResourceId(location)
    if anim.isResourceDescendant(resourceId, locationResourceId) then
      menu:addItem{
        label = menuLabel,
        onClick = function(self)
          anim.setRigPose(set, poseName, pose)
        end,
      }
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- 
------------------------------------------------------------------------------------------------------------------------
local addPoseSpecificMenuItems = function(menu, resourceId, selectedResourceIds)
  local set = getSelectedAssetManagerAnimSet()
  if (not mcn.isEuphoriaDisabled()) and (anim.getAnimSetCharacterType(set) == "Euphoria") then
    local resourceType = anim.getResourceType(resourceId)

    if resourceType == "directory" then
      addSetAsPoseLocationMenuItem(menu, "Set as poses location", set, resourceId)
    else
      local locationResourceId = nil
      local takeResourceId = nil
      if resourceType == "file" then
        locationResourceId = anim.getResourceParent(resourceId)
        local children = anim.getResourceChildren(resourceId)

        -- only set takeResourceId if there is one child take
        for childResourceId, _ in pairs(children) do
          if takeResourceId then
            takeResourceId = nil
            break
          else
            takeResourceId = childResourceId
          end
        end
      elseif resourceType == "take" then
        local fileResourceId = anim.getResourceParent(resourceId)
        locationResourceId = anim.getResourceParent(fileResourceId)

        takeResourceId = resourceId
      end

      if anim.isResource(takeResourceId) and anim.getResourceType(takeResourceId) == "take" then
        addSetAsPoseMenuItem(menu, "Set as default pose", set, "DefaultPose", takeResourceId)
        addSetAsPoseMenuItem(menu, "Set as guide pose", set, "GuidePose", takeResourceId)
      end

      if anim.isResource(locationResourceId) and anim.getResourceType(locationResourceId) == "directory" then
        addSetAsPoseLocationMenuItem(menu, "Set folder as poses location", set, locationResourceId)
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- adds menu items to show the resource and associated files in an explorer window
------------------------------------------------------------------------------------------------------------------------
local addShowInExplorerMenuItems = function(menu, resourceId, selectedResourceIds)
  if not anim.getResourceIsMissing(resourceId) then
    menu:addItem{
      label = "Show in explorer",
      onClick = function(self)
        anim.showResourceInExplorer(resourceId)
      end,
    }

    local resourceType = anim.getResourceType(resourceId)
    if resourceType == "file" or resourceType == "take" then
      menu:addItem{
        label = "Show markup file in explorer",
        onClick = function(self)
          anim.showResourceMarkupInExplorer(resourceId)
        end,
      }
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
addResourceContextMenuItems = function(menu, resourceId, selectedResourceIds)
  local menuTitle = string.format("%s (%i selected)", anim.getResourceName(resourceId), table.getn(selectedResourceIds))
  menu:addItem{ label = menuTitle, style = "title", enable = false }
  addResourceTypeSpecificMenuItems(menu, resourceId, selectedResourceIds)
  addPoseSpecificMenuItems(menu, resourceId, selectedResourceIds)
  addShowInExplorerMenuItems(menu, resourceId, selectedResourceIds)
  
  if anim.hasAnyMarkupChanged() then
    if menu:countItems() > 0 then
      menu:addSeparator()
    end
  
    menu:addItem{
      label = "Save all markup changes",
      onClick = function(self)
        anim.saveAllMarkup()
      end
    }
  end
end

------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------
onAssetChooserContextualMenu = function(menu, resourceId, selectedResourceIds)
  if resourceId then

    local resourceType = anim.getResourceType(resourceId)

    if resourceType == "take" then
      local parent = anim.getResourceParent(resourceId)
      if parent ~= nil then
        local children = anim.getResourceChildren(parent)
        -- see if we only have a single take in this file
        local numTakes = 0
        for animationResourceId in pairs(children) do
          numTakes = numTakes + 1
          
          if (numTakes > 1) then
            -- we have more than one take
            break
          end
        end
      end
      
      if numTakes == 1 then
        resourceId = parent
      end
    end

    addResourceContextMenuItems(menu, resourceId, selectedResourceIds)
    
    if resourceType == "file" or resourceType == "take" then
      menu:addItem{
        label = "Reveal in tree",
        onClick = function(self)
          local assetManager = getAssetManagerWindow()
          assetManager:revealResourceInTree(resourceId)
        end,
      }
    end
    
  end
end

------------------------------------------------------------------------------------------------------------------------
onAssetTreeContextualMenu = function(menu, resourceId, selectedResourceIds)
  if resourceId then
    addResourceContextMenuItems(menu, resourceId, selectedResourceIds)

    if menu:countItems() > 0 then
      menu:addSeparator()
    end

    if not isRetargetingEnabled() then 
      menu:addItem{
        label = "Add new location...",
        onClick = function()
          local assetManager = getAssetManagerWindow()
          assetManager:addNewLocation()
        end
      }
      local missingLocations = anim.listMissingLocations()
      if table.getn(missingLocations) > 0 then
        menu:addItem{
          label = "Add missing locations",
          onClick = function()
            local assetManager = getAssetManagerWindow()
            assetManager:addMissingLocations()
          end
        }
      end
    end
  end
end