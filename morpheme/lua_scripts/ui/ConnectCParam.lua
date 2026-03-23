------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- a table of currently valid connections
local validConnections = { }

------------------------------------------------------------------------------------------------------------------------
-- Dialog to choose which connections to force a reconnect of
------------------------------------------------------------------------------------------------------------------------
local getConnectionsToBreak = function(alreadyConnected, dataPin, controlParameter)
  local choiceCancelled = false

  local dlg = ui.createModalDialog{
    name = "ChooseForceReconnect",
    caption = string.format("Force Connection to '%s'", controlParameter),
    resize = true,
    size = { width = 400, height = 300 }
  }

  dlg:beginVSizer{ }
    dlg:addStaticText{ text = string.format("Some nodes already have a connection to: %s", dataPin) }
    dlg:addStaticText{ text = "Choose which nodes you would like to force a connection to" }
    local listControl = dlg:addListControl{
      flags = "expand;multiSelect",
      proportion = 1,
      numColumns = 2
    }

    dlg:beginHSizer{ flags = "expand", proportion = 0 }
      dlg:addButton{
        label = "Select All",
        proportion = 0,
        onClick = function(self)
          for i = 1, listControl:getNumRows() do
            listControl:setItemValue(i, 1, true)
          end
        end
      }

      dlg:addButton{
        label = "Select None",
        proportion = 0,
        onClick = function()
          for i = 1, listControl:getNumRows() do
            listControl:setItemValue(i, 1, false)
          end
        end
      }

      dlg:addStretchSpacer{ proportion = 1 }

      dlg:addButton{
        label= "OK",
        onClick = function()
          choiceCancelled = false
          dlg:hide()
        end
      }

      dlg:addButton{
        label = "Cancel",
        proportion = 0,
        onClick = function()
          choiceCancelled = true
          dlg:hide()
        end
      }
    dlg:endSizer()
  dlg:endSizer()

  for _, n in ipairs(alreadyConnected) do
    listControl:addRow{ false, n }
  end

  dlg:show()

  if choiceCancelled then
    -- dispose of the dlg
    dlg = nil
    collectgarbage()
    return false
  end

  local breakConnection = { }
  for i = 1, listControl:getNumRows() do
    if listControl:getItemValue(i, 1) then
      table.insert(breakConnection, listControl:getItemValue(i, 2))
    end
  end

  -- dispose of the dlg
  dlg = nil
  collectgarbage()

  return true, breakConnection
end

------------------------------------------------------------------------------------------------------------------------
-- Connect CParam Window
------------------------------------------------------------------------------------------------------------------------
local updateValidConnections = function()
  local validConnectionList = { }
  validConnections = { }
  -- Generate a list of which pins can connect to another pin
  for _, node in ipairs(ls("Selection")) do

    for _, pin in ipairs(listPins(node)) do
      -- cache the path to this pin
      local pinPath = string.format("%s.%s", node, pin)

      for _, controlParam in ipairs(ls("ControlParameters")) do

        -- can connect must have flag for ignoring current conections.
        if canConnect(string.format("%s.Result", controlParam), pinPath, true) then

          -- add it to the list of valid connections
          local _, controlParamName = splitNodePath(controlParam)

          if validConnectionList[pin] == nil then
            validConnectionList[pin] = { controlParamName }
          else
            -- insert if not already in table
            local found = false
            for _, c in ipairs(validConnectionList[pin]) do
              if c == controlParamName then
                found = true
                break
              end
            end

            if not found then
              table.insert(validConnectionList[pin], controlParamName)
            end
          end
        end
      end
    end
  end

  -- sort the validConnections
  for i, v in pairs(validConnectionList) do
    -- here v is the list of cparams for this pin, so sort them
    table.sort(v, function(a, b) return a < b end)
    table.insert(validConnections, { dataPin = i, controlParams = v })
  end

  table.sort(validConnections, function(a, b) return a.dataPin < b.dataPin end)
end

------------------------------------------------------------------------------------------------------------------------
-- refreshes the control parameter window if it exists
------------------------------------------------------------------------------------------------------------------------
local controlParamList = nil
local connectButton = nil
local dataPinList = nil

local refreshWindow = function()
  if not dataPinList or not controlParamList or not connectButton then
    return
  end

  dataPinList:clearRows()
  controlParamList:clearRows()
  connectButton:enable(false)

  -- Update the list of valid connections
  updateValidConnections()

  -- update the list boxes
  if (table.getn(validConnections) > 0) then
    -- Some connections are possible
    local i, v
    for i, v in ipairs(validConnections) do
      dataPinList:addRow(v.dataPin)
    end
    dataPinList:enable(true)

    -- This will always be empty until a pin is selected
    controlParamList:addRow("No Input Pin Selected")
    controlParamList:enable(false)
  else
    -- No valide connections so nothing in here
    dataPinList:addRow("No Connections Possible")
    dataPinList:enable(false)
    controlParamList:addRow("No Connections Possible")
    controlParamList:enable(false)
  end

end

------------------------------------------------------------------------------------------------------------------------
-- add Connect Control Parameter Window
------------------------------------------------------------------------------------------------------------------------
addConnectControlParamWindow = function(layoutManager)
  -- create the connect control parameter panel and add it to the layout manager
  local panel = layoutManager:addPanel{
    name = "ConnectControlParams",
    caption = "Connect Control Parameter",
    flags = "expand",
    proportion = 1
  }

  panel:beginVSizer{ flags = "expand", proportion = 1 }
    panel:addStaticText{ text = "Input Pin" }
    dataPinList = panel:addListControl{ flags = "expand", proportion = 1 }
    panel:addStaticText{ text = "Control Parameter" }
    controlParamList = panel:addListControl{ flags = "expand", proportion = 1 }
    connectButton = panel:addButton{ label = "Connect", onClick = doConnect, flags = "right", size = { width = 74 } }
  panel:endSizer()

  refreshWindow()

  -- set the selection changed function for the data pins list control
  dataPinList:setOnSelectionChanged(
    function()
      controlParamList:clearRows()
      connectButton:enable(false)

      local connection = validConnections[dataPinList:getSelectedRow()]
      -- connection will be nil if nothing is selected.
      if connection ~= nil then
        -- Update list of pins to connect to
        for i, v in ipairs(connection.controlParams) do
          controlParamList:addRow(v)
        end
        controlParamList:enable(true)
      else
        controlParamList:addRow("No connections possible")
        controlParamList:enable(false)
      end
    end
  )

  -- set the on click function for the connect button
  connectButton:setOnClick(
    function()
      -- enable the connection button if selection > 0
      if not controlParamList:getSelectedRow() or not dataPinList:getSelectedRow() then
        return
      end

      local selectedDataPinRow = dataPinList:getSelectedRow()

      -- see if we allready have a connection to the data pin
      local dataPin = validConnections[selectedDataPinRow].dataPin
      local controlParam = validConnections[selectedDataPinRow].controlParams[controlParamList:getSelectedRow()]

      -- get a list of nodes that have the selected data pin
      local nodeList = { }
      for _, node in ipairs(ls("Selection")) do
        for _, pin in ipairs(listPins(node)) do
          if pin == dataPin then
            table.insert(nodeList, node)
            break
          end
        end
      end

      -- see if we already have connections
      local alreadyConnected = { }
      for _, node in ipairs(nodeList) do
        if isConnected(string.format("%s.%s", node, dataPin)) then
          table.insert(alreadyConnected, node)
        end
      end

      if table.getn(alreadyConnected) > 0 then
        -- display a dialog working out which pins to force a re-connection on.
        local success, connectionsToBreak = getConnectionsToBreak(alreadyConnected, dataPin, controlParam)

        if success == false then
          -- user cancelled, do nothing
          return false
        end

        -- disconnect pins that should be reconnected
        for _, node in ipairs(connectionsToBreak) do
          local pin = string.format("%s.%s", node, dataPin)
          if isConnected(pin) then
            local otherPins = listConnections{ Pins = true, Object = pin }
            if table.getn(otherPins) > 1 then
              warning(string.format("More than one connection to %s. Skipping pin", pin))
            else
              breakConnection(otherPins[1], pin)
            end
          end
        end
      end

      -- by now, all pins that need a connection will not have a current connection
      for _, node in ipairs(nodeList) do
        local controlParamPin = string.format("ControlParameters|%s.Result", controlParam)
        local nodePin = string.format("%s.%s", node, dataPin)
        if canConnect(controlParamPin, nodePin, false) then
          connect(controlParamPin, nodePin)
        end
      end

    end
  )

  -- set the selection changed function for the control parameters list control
  controlParamList:setOnSelectionChanged(
    function(self)
      -- enable the connection button if selection > 0
      if controlParamList:getSelectedRow() then
        connectButton:enable(true)
      else
        connectButton:enable(false)
      end
    end
  )
end

if not mcn.inCommandLineMode() then
  registerEventHandler("mcSelectionChange", refreshWindow)
end
