------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AddAttributeDialog.lua"
require "ui/KeyboardNetworkNavigator.lua"

local kBakeReferenceMessage = [[
Are you sure you want to bake this reference?
]]

------------------------------------------------------------------------------------------------------------------------
-- Reset the state of the network navigation buttons
------------------------------------------------------------------------------------------------------------------------
local resetNetworkButtonState = function()
  local networkPanel = ui.getWindow("MainFrame|LayoutManager|Network")
  local networkView = ui.getWindow("MainFrame|LayoutManager|Network|Network")

  if networkPanel ~= nil then
    local navigateUpButton = ui.getWindow("MainFrame|LayoutManager|Network|ToolBar|NavigateUp")
    local networkLocation = ui.getWindow("MainFrame|LayoutManager|Network|ToolBar|CurrentLocation")

    local currentGraph = getCurrentGraph()

    local eventTimelineButton = ui.getWindow("MainFrame|LayoutManager|Network|ToolBar|ToggleEventTimelineDisplay")
    local cpTracingButton = ui.getWindow("MainFrame|LayoutManager|Network|ToolBar|ToggleControlParamTracing")
    local cpTracingButtonLive = ui.getWindow("MainFrame|LayoutManager|Network|ToolBar|toggleControlParamTracingLive")
    local cpTracingButtonProp = ui.getWindow("MainFrame|LayoutManager|Network|ToolBar|ToggleControlParamTracePropagate")

    eventTimelineButton:setChecked(networkView:getEventTimelineDisplay())
    cpTracingButton:setChecked(networkView:getControlParamTracing())
    cpTracingButtonLive:setChecked(networkView:getControlParamTracingLive())
    cpTracingButtonProp:setChecked(networkView:getControlParamTracePropagate())

    networkLocation:setValue(currentGraph)
    navigateUpButton:enable(getParent(currentGraph) ~= nil)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Navigate active network up
------------------------------------------------------------------------------------------------------------------------
local navigateUp = function()
  local currentGraph = getCurrentGraph()
  local parentGraph = getParent(currentGraph)

  if parentGraph ~= nil then
    setCurrentGraph(parentGraph)
    safefunc(resetNetworkButtonState)
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Zoom network to extents
------------------------------------------------------------------------------------------------------------------------
local zoomToExtents = function()
  local networkView = ui.getWindow("MainFrame|LayoutManager|Network|Network")

  if networkView ~= nil then
    networkView:frameAll()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Zoom network to selection
------------------------------------------------------------------------------------------------------------------------
local zoomToSelection = function()
  local networkView = ui.getWindow("MainFrame|LayoutManager|Network|Network")

  if networkView ~= nil then
    networkView:frameSelection()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Zoom network to region
------------------------------------------------------------------------------------------------------------------------
local zoomToRegion = function()
  local networkView = ui.getWindow("MainFrame|LayoutManager|Network|Network")

  if networkView ~= nil then
    networkView:dragSelectZoom()
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Toggle event timeline display
------------------------------------------------------------------------------------------------------------------------
local toggleEventTimelineDisplay = function(self)
  local networkView = ui.getWindow("MainFrame|LayoutManager|Network|Network")

  if networkView ~= nil then
    self:setChecked(not self:getChecked())
    networkView:setEventTimelineDisplay(self:getChecked())
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Toggle CP tracing
------------------------------------------------------------------------------------------------------------------------
local toggleControlParamTracing = function(self)
  local networkView = ui.getWindow("MainFrame|LayoutManager|Network|Network")

  if networkView ~= nil then
    self:setChecked(not self:getChecked())
    networkView:setControlParamTracing(self:getChecked())
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Toggle CP tracing 'live' - while network is running
------------------------------------------------------------------------------------------------------------------------
local toggleControlParamTracingLive = function(self)
  local networkView = ui.getWindow("MainFrame|LayoutManager|Network|Network")

  if networkView ~= nil then
    self:setChecked(not self:getChecked())
    networkView:setControlParamTracingLive(self:getChecked())
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Toggle CP trace propogation - copying values from CP outputs to their connected inputs to help visualize value flow
------------------------------------------------------------------------------------------------------------------------
local toggleControlParamTracePropagate = function(self)
  local networkView = ui.getWindow("MainFrame|LayoutManager|Network|Network")

  if networkView ~= nil then
    self:setChecked(not self:getChecked())
    networkView:setControlParamTracePropagate(self:getChecked())
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Toggle active state following
------------------------------------------------------------------------------------------------------------------------
local toggleActiveStateFollowing = function()
  local networkView = ui.getWindow("MainFrame|LayoutManager|Network|Network")
  local toggleButton = ui.getWindow("MainFrame|LayoutManager|Network|ToolBar|ToggleActiveStateFollowing")

  if networkView ~= nil then
    local isEnabled = networkView:isActiveStateFollowingEnabled()
    if (isEnabled == true) then
      networkView:disableActiveStateFollowing()
      toggleButton:setChecked(false)
    else
    networkView:enableActiveStateFollowing()
      toggleButton:setChecked(true)
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Get selected transitions
------------------------------------------------------------------------------------------------------------------------
local getSelectedTransitions = function(clickedItemPath)
  local selectedTransitions = { }

  local selection = ls("Selection")
  if clickedItemPath and string.len(clickedItemPath) > 0 then
    local nodeType, manifestType = getType(clickedItemPath)
    if nodeType == "TransitionGroup" or manifestType == "Transition" then
      table.insert(selection, clickedItemPath)
    else
      return selectedTransitions
    end
  end

  for i, path in ipairs(selection) do
    local nodeType, manifestType = getType(path)
    if nodeType == "TransitionGroup" then
      local groupPaths = listTransitionGroupMembers(path)
      for i, groupPath in ipairs(groupPaths) do
        table.insert(selectedTransitions, groupPath)
      end
    elseif manifestType == "Transition" then
      table.insert(selectedTransitions, path)
    end
  end

  return selectedTransitions
end

------------------------------------------------------------------------------------------------------------------------
-- Align selected nodes
------------------------------------------------------------------------------------------------------------------------
alignSelectedNodes = function(NetworkWindow, direction)
  local networkView = ui.getWindow(NetworkWindow)

  if direction == 0 then
    networkView:alignNodesToLeft()
  elseif direction == 1 then
    networkView:alignNodesToRight()
  elseif direction == 2 then
    networkView:alignNodesToTop()
  elseif direction == 3 then
    networkView:alignNodesToBottom()
  end
end

------------------------------------------------------------------------------------------------------------------------
local graphSizes;
local validManifestTypes = { StateMachine = true, BlendNode = true, StateMachineNode = true }

local getEntry = function(path, type)
  local entry
  if type == "ControlParameters" then
    local x, y = getControlParametersNodePosition(path)
    local width, height = getControlParametersNodeSize(path)
    entry = { path = path, x = x, y = y, width = width, height = height, type = "ControlParameters" }
  elseif type == "EmittedControlParameters" then
    local x, y = getEmittedControlParametersNodePosition(path)
    local width, height = getEmittedControlParametersNodeSize(path)
    entry = { path = path, x = x, y = y, width = width, height = height, type = "EmittedControlParameters" }
  elseif type == "Output" then
    local x, y = getOutputNodePosition(path)
    local width, height = getOutputNodeSize(path)
    entry = { path = path, x = x, y = y, width = width, height = height, type = "Output" }
  else
    local x, y = getNodePosition(path)
    local width, height = getNodeSize(path)
    entry = { path = path, x = x, y = y, width = width, height = height, type = manifestType }
  end
  
  return entry
end

------------------------------------------------------------------------------------------------------------------------
local setPosition = function(path, type, x, y)
  local entry
  if type == "ControlParameters" then
    setControlParametersNodePosition(path, x, y)
  elseif type == "EmittedControlParameters" then
    setEmittedControlParametersNodePosition(path, x, y)
  elseif type == "Output" then
    setOutputNodePosition(path, x, y)
  else
    setNodePosition(path, x, y)
  end
end

------------------------------------------------------------------------------------------------------------------------
local beforeGraphChanged = function(path)
  graphSizes = { }
  local children = listChildren(path)
  for _, path in ipairs(children) do
    local nodeType, manifestType = getType(path)
    if manifestType and validManifestTypes[manifestType] then
      table.insert(graphSizes, getEntry(path, manifestType))
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local afterGraphChanged = function(path)
  local nodesHaveGrown = false
  local vSortedNodes = { }
  local hSortedNodes = { }

  if table.getn(graphSizes) == 0 then
    return
  end

  for _, entry in ipairs(graphSizes) do
    local newEntry = getEntry(entry.path, entry.type)
    if (newEntry.width > entry.width) or (newEntry.height > entry.height) then
      nodesHaveGrown = true
    end
    table.insert(vSortedNodes, newEntry)
    table.insert(hSortedNodes, newEntry)
  end

  -- if the size of the nodes has grown then see if any overlap and reposition them  
  if nodesHaveGrown then
    local kHResolution = 4.0
    local kVResolution = 4.0
    local kVSeparation = 10.0
    local kHSeparation = 10.0

    local manifestType, nodeType = getType(path)
    if (manifestType == "BlendTree" or manifestType == "PhysicsBlendTree") and (nodeType ~= "StateMachine" and nodeType ~= "PhysicsStateMachine") then
      local otherNodes = {"ControlParameters", "EmittedControlParameters", "Output"}
      for _, nodeType in pairs(otherNodes) do
        local newEntry = getEntry(path, nodeType)
        table.insert(vSortedNodes, newEntry)
        table.insert(hSortedNodes, newEntry)
      end
    end
    
    table.sort(hSortedNodes, function(a, b)
      return a.x < b.x
    end)
    table.sort(vSortedNodes, function(a, b)
      return a.y < b.y
    end)
    
    -- shuffle vertically any vertical overlaps
    local maxDepth = { }
    for _, node in ipairs(vSortedNodes) do
      local nodeMin = math.floor(node.x/kVResolution);
      local nodeMax = math.ceil((node.x + node.width)/kVResolution);
      
      -- find the first free top position
      local top = node.y;
      for i = nodeMin, nodeMax do
        if (maxDepth[i] ~= nil) and (maxDepth[i] > top) then
          top = maxDepth[i]
        end
      end

      -- fill in maxDepth with the bottom of the item
      local bottom = top + node.height;
      for i = nodeMin, nodeMax do
        maxDepth[i] = bottom + kVSeparation;
      end
      
      node.y = top
    end

    -- shuffle horizontaly any horizontal overlaps
    local maxWidth = { }
    for _, node in ipairs(hSortedNodes) do

      local nodeMin = math.floor(node.y/kHResolution);
      local nodeMax = math.ceil((node.y + node.height)/kHResolution);
      local left = node.x;
      for i = nodeMin, nodeMax do
        if (maxWidth[i] ~= nil) and (maxWidth[i] > left) then
          left = maxWidth[i]
        end
      end
      
      local right = left + node.width;
      for i = nodeMin, nodeMax do
        maxWidth[i] = right + kHSeparation;
      end
      node.x = left
    end

    -- and finally actually move the nodes
    for _, node in ipairs(vSortedNodes) do
      setPosition(node.path, node.type, node.x, node.y)
    end
  end 
end

------------------------------------------------------------------------------------------------------------------------
local findHint
findHint = function(hintToFind, pinInfo)
  local hint = pinInfo.hint

  if hint == hintToFind then
    return pinInfo
  end

  if pinInfo.members then
    for _, item in ipairs(pinInfo.members) do
      local foundHint = findHint(hintToFind, item)
      if foundHint then
        return foundHint
      end
    end
  end

  return nil
end

------------------------------------------------------------------------------------------------------------------------
local findUnconnectedHint
findUnconnectedHint = function(hintToFind, pinInfo)
  local hint = pinInfo.hint

  if hint == hintToFind and pinInfo.unconnected > 0 then
    return pinInfo
  end

  if pinInfo.members and pinInfo.unconnected > 0 then
    for _, item in ipairs(pinInfo.members) do
      local foundHint = findUnconnectedHint(hintToFind, item)
      if foundHint then
        return foundHint
      end
    end
  end

  return nil
end

------------------------------------------------------------------------------------------------------------------------
local connectByHints
connectByHints = function(fromPins, toPins)
  local hint = fromPins.hint
  local connectionsMade = 0
  
  if hint then
    local toConnection = findUnconnectedHint(hint, toPins)
    if toConnection and toConnection.unconnected > 0 then
    
      -- if we have found an unconected pin then make the connection
      if fromPins.pin and toConnection.pin then
        if connect(fromPins.pin, toConnection.pin) then
          connectionsMade = connectionsMade + 1
          fromPins.connected = fromPins.connected + 1
          fromPins.unconnected = fromPins.unconnected - 1
          toConnection.connected = toConnection.connected + 1
          toConnection.unconnected = toConnection.unconnected - 1
        end
      -- otherwise attempt to connect the children
      else 
        for _, item in ipairs(fromPins.members) do
          connectionsMade = connectionsMade + connectByHints(item, toConnection)
        end
      end
      
    end
  elseif fromPins.members then
    for _, item in ipairs(fromPins.members) do
      connectionsMade = connectionsMade + connectByHints(item, toPins)
    end
  end
  return connectionsMade
end

------------------------------------------------------------------------------------------------------------------------
local canConnectByHints
canConnectByHints = function(fromPins, toPins)
  local hint = fromPins.hint
  if hint then
    local toConnection = findHint(hint, toPins)
    if toConnection and toConnection.unconnected > 0 then
      if fromPins.pin and toConnection.pin then
        return true
      else
        for _, item in ipairs(fromPins.members) do
          if canConnectByHints(item, toConnection) then
            return true
          end
        end
      end
    end
  elseif fromPins.members then
    for _, item in ipairs(fromPins.members) do
      if canConnectByHints(item, toPins) then
        return true
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
local canConnectCable = function(fromPins, toPins)
  return canConnectByHints(fromPins, toPins)
end

------------------------------------------------------------------------------------------------------------------------
local onConnectCable = function(fromPins, toPins)
  connectByHints(fromPins, toPins)
end

------------------------------------------------------------------------------------------------------------------------
-- Network panel right click menus
------------------------------------------------------------------------------------------------------------------------
onNetworkContextMenu = function(menu, container, path)
  if string.len(container) > 0 and string.len(path) == 0 then
    if canRemoveParentGraph(container) then
      local manifestType, nodeType = getType(getParent(container))

      local containerTypeDisplayName = "Blend Tree"
      if manifestType == "PhysicsBlendTree" then
        containerTypeDisplayName = "Physics Blend Tree"
      elseif manifestType == "StateMachine" then
        containerTypeDisplayName = "State Machine"
      elseif manifestType == "PhysicsStateMachine" then
        containerTypeDisplayName = "Physics State Machine"
      end

      menu:addItem{
        label = string.format("Remove Parent %s", containerTypeDisplayName),
        onClick = function()
          removeParentGraph(container)
        end
      }
    end
  end

  if string.len(path) > 0 then
    if canCollapseToNxMNode(path) then
      menu:addItem{
        label = "Collapse to BlendNxM",
        onClick = function()
          collapseNxMNode(path)
        end
      }
    elseif canExpandNxMNode(path) then
      menu:addItem{
        label = "Expand BlendNxM",
        onClick = function()
          expandNxMNode(path)
        end
      }
    end

    local manifestType, nodeType = getType(path)
    if nodeType ~= nil then
      if nodeType == "BlendNode" or nodeType == "StateMachineNode" then
        menu:addItem{
          label = "Add Attribute",
          onClick = function()
            showAddAttributeDialog(path)
          end
        }
      end
      
      if nodeType == "BlendNode" or nodeType == "StateMachine" then
        local referenceFile = getReferenceFile(path)
        if string.len(referenceFile) > 0 then
        
          menu:addSeparator()

          -- This is a reference node
          if not isReferenced(container) then
            menu:addItem{
              label = "Bake Reference",
              onClick = function(self)
                if ui.showMessageBox(kBakeReferenceMessage, "yesno") == "yes" then
                  bakeReference(path)
                end
              end
            }
          end

          -- This is a reference node
          menu:addItem{
            label = "Show Reference File in Explorer",
            onClick = function(self)
              local fullPath = utils.demacroizeString(referenceFile)
              app.execute(string.format("explorer.exe /select, %s", fullPath), false, false)
            end
          }

          menu:addCheckedItem{
            label = "Auto Update Reference on File Open",
            onClick = function(self)
              local autoUpdate = self:isChecked()
              setReferenceIsAutoUpdating{ Path = path, AutoUpdate = not autoUpdate }
            end,
            checked = getReferenceIsAutoUpdating(path)
          }
        end
      end
    end
  end

  local type, manifestType = getType(container)
  if type == "StateMachine" or manifestType == "StateMachine" or type == "StateMachineNode" or type == "PhysicsStateMachine" then
    local transitions = getSelectedTransitions(path)
    if table.getn(transitions) > 0 then
      menu:addItem{
        label = "Group Selected Transitions",
        onClick = function()
          groupTransitions(transitions)
        end
      }
      menu:addItem{
        label = "Ungroup Selected Transitions",
        onClick = function()
          groupTransitions(transitions, false)
        end
      }
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Group selected transitions
------------------------------------------------------------------------------------------------------------------------
onGroupTransitions = function()
  local graph = getCurrentGraph()
  if graph then
    local type = getType(graph)
    if type == "StateMachine" or type == "PhysicsStateMachine" then
      local transitions = getSelectedTransitions()
      if table.getn(transitions) > 0 then
        groupTransitions(transitions)
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Ungroup selected transitions
------------------------------------------------------------------------------------------------------------------------
onUngroupTransitions = function()
  local graph = getCurrentGraph()
  if graph then
    local type = getType(graph)
    if type == "StateMachine" or type == "PhysicsStateMachine" then
      local transitions = getSelectedTransitions()
      if table.getn(transitions) > 0 then
        groupTransitions(transitions, false)
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
-- Network panel
------------------------------------------------------------------------------------------------------------------------
addNetwork = function(layoutManager)
  local network = layoutManager:addPanel{ name = "Network", caption = "Network" }

  network:beginVSizer{ proportion = 1, flags = "expand" }
    local toolBar = network:addToolBar{ name = "ToolBar", proportion = 0, flags = "expand" }

    toolBar:addButton{
      name = "AlignLeft",
      image = app.loadImage("alignleft.png"),
      onClick = "alignSelectedNodes(\"MainFrame|LayoutManager|Network|Network\", 0)",
      helpText = "Align to left" }

    toolBar:addButton{
      name = "AlignRight",
      image = app.loadImage("alignright.png"),
      onClick = "alignSelectedNodes(\"MainFrame|LayoutManager|Network|Network\", 1)",
      helpText = "Align to right" }

    toolBar:addButton{
      name = "AlignTop",
      image = app.loadImage("aligntop.png"),
      onClick = "alignSelectedNodes(\"MainFrame|LayoutManager|Network|Network\", 2)",
      helpText = "Align to top" }

    toolBar:addButton{
      name = "AlignBottom",
      image = app.loadImage("alignbottom.png"),
      onClick = "alignSelectedNodes(\"MainFrame|LayoutManager|Network|Network\", 3)",
      helpText = "Align to bottom" }

    toolBar:addButton{
      name = "ZoomToExtents",
      image = app.loadImage("zoomtoextents.png"),
      onClick = zoomToExtents,
      helpText = "Zoom to extents" }

    toolBar:addButton{
      name = "ZoomToSelection",
      image = app.loadImage("zoomtoselection.png"),
      onClick = zoomToSelection,
      helpText = "Zoom to selection" }

    toolBar:addButton{
      name = "ZoomToRegion",
      image = app.loadImage("zoomtoregion.png"),
      onClick = zoomToRegion,
      helpText = "Zoom to region" }

    toolBar:addButton{
      name = "ToggleActiveStateFollowing",
      image = app.loadImage("toggleactivestatefollowing.png"),
      onClick = toggleActiveStateFollowing,
      helpText = "Toggle active state following" }

    toolBar:addSeparator();

    toolBar:addButton{
      name = "ToggleEventTimelineDisplay",
      image = app.loadImage("eventTimeline.png"),
      onClick = toggleEventTimelineDisplay,
      helpText = "Toggle display of event debug timelines" }

    toolBar:addButton{
      name = "ToggleControlParamTracing",
      image = app.loadImage("cptrace.png"),
      onClick = toggleControlParamTracing,
      helpText = "Toggle display of Control Parameter value tracing" }

    toolBar:addButton{
      name = "toggleControlParamTracingLive",
      image = app.loadImage("cptracelive.png"),
      onClick = toggleControlParamTracingLive,
      helpText = "Toggle display of Control Parameter tracing during network update, not just playback" }

    toolBar:addButton{
      name = "ToggleControlParamTracePropagate",
      image = app.loadImage("cptraceprop.png"),
      onClick = toggleControlParamTracePropagate,
      helpText = "Toggle propagation of Control Parameter traces to connected pins" }

    toolBar:addSeparator();

    toolBar:addButton{
      name = "NavigateUp",
      image = app.loadImage("navigateUp.png"),
      onClick = navigateUp,
      helpText = "Navigate up" }

    local locationText = toolBar:addTextBox{
      name = "CurrentLocation",
      helpText = "Current location",
      proportion = 1,
      flags = "expand" }

    local networkNavChange = function()
      local path = locationText:getValue()
      if (objectExists(path)) then
        local type = getType(path)
        local isBlendTree = (type == "BlendTree" or type == "PhysicsBlendTree")
        local isStateMachine = (type == "StateMachine" or type == "PhysicsStateMachine")
        if (isBlendTree or isStateMachine) then
          setCurrentGraph(path)
        end
      end
      locationText:setValue(getCurrentGraph())
    end

    locationText:setOnEnter(networkNavChange)

    local networkStockWindow = network:addStockWindow{ name = "Network", type = "Network", proportion = 1, flags = "expand" }

    -- set the right click menus.
    if networkStockWindow then
      networkStockWindow:setOnContextMenuFunction(onNetworkContextMenu)
      networkStockWindow:setCanConnectCableFunction(canConnectCable)
      networkStockWindow:setOnConnectCableFunction(onConnectCable)
      networkStockWindow:setBeforeGraphChangedFunction(beforeGraphChanged)
      networkStockWindow:setAfterGraphChangedFunction(afterGraphChanged)
      networkStockWindow:addAccelerator("Ctrl+G", onGroupTransitions)
      networkStockWindow:addAccelerator("Ctrl+Shift+G", onUngroupTransitions)
    end

    -- set the network navigation keyboard shortcuts
    if networkStockWindow then
      setupNetworkNavigator()
    end

  network:endSizer()

  if type(resetNetworkButtonState) == "function" then
    registerEventHandler("mcCurrentGraphChange", resetNetworkButtonState)
    resetNetworkButtonState()
  end

  return network
end

