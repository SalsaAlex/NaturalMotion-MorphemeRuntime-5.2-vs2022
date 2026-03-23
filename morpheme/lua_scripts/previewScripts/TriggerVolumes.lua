------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: PAGE
--| name: TriggerVolumes
--| title: Trigger Volumes API
--| brief: Test if a character is in a given region of the scene
--| desc:
--|   Trigger Volumes allow you to easily write update scripts that test if a character is in a given region of the scene.  
--|   They also provide tracking information to make it simpler to test for various scenarios about the character such as 
--|   if the entire body has entered the volume or just a small portion.  
--|
--| LUAHELP: NAMESPACE
--| name: triggerVolumes
--| groupdef: Init functions that will typically be used in init scripting
--| groupdef: Update funcitons that will typically be used in update scripting
----------------------------------------------------------------------------------------------------------------------


------------------------------------------------------------------------------------------------------------------------
--  Usage notes 
------------------------------------------------------------------------------------------------------------------------
-- The simplest way to use trigger volumes is to add a few calls in your init script describing actions that will occur on 
-- entry or exit.  For example, in a scene using the volumes "TriggerSphere1" and "TriggerBox2" we may have: 
--   triggerVolumes.cparamOnEnter("TriggerSphere1","speed", 0.5)
--   triggerVolumes.requestOnEnter("TriggerSphere1", "RUN")
--
--   triggerVolumes.cparamOnExit("TriggerSphere1", "speed", 0.0)
--   triggerVolumes.requestOnExit("TriggerSphere1", "WALK")
--
--   triggerVolumes.requestOnEnter("TriggerBox2", "RAGDOLL")
--
-- You may prefer to use update scripting to test the volumes directly.  In this case make use of the functions listed
-- under update scripting.  For example
--
--    if triggerVolumes.fullyEnteredVolume("MyVolume") then
--      sendRequest("Stop")
--    end
--
-- Note that the enteredVolume and fullyEnteredVolume functions are only true for a single frame. 
-- For control paramters that are set every frame you can use the inside outside test functions
--
--   if triggerVolumes.fullyInVolume("MyVolume") then 
--      setControlParmater("speed", 0.5)
--    end


------------------------------------------------------------------------------------------------------------------------
-- Implementation Details: 
------------------------------------------------------------------------------------------------------------------------
-- Connect automatically conmstructs and maintains a table that contains the triggerVolumes in the scene.  
-- If we have two trigger volumes called "TriggerStop" and "TriggerSphere1" and instance IDS 15, and 18 connect would 
-- create the following structure for the case where instsance 18 was in triggerStop and instance 15 is in TriggerSphere1
--
--  __triggerVolumes = {
--        TriggerStop = {
--             15 = { inVolume = false, enteredVolume = false, leftVolume = false, fullyEnteredVolume = false, 
--                    inVolumeRatio = 0, updatesInVolume = 0}, 
--             18 = { inVolume = true, enteredVolume = false, leftVolume = false, fullyEnteredVolume = false, 
--                    inVolumeRatio = 0.60, updatesInVolume = 4}
--                      },
--        TriggerSphere1 = {
--             15 = { inVolume = true, enteredVolume = false, leftVolume = false, fullyEnteredVolume = true, 
--                    inVolumeRatio = 1.0, updatesInVolume = 10}, 
--             18 = { inVolume = false, enteredVolume = false, leftVolume = false, fullyEnteredVolume = false, 
--                    inVolumeRatio = 0.0, updatesInVolume = 0}
--                      },
--     }
------------------------------------------------------------------------------------------------------------------------

triggerVolumes = {}
triggerVolumes._triggerEnterRequestTable = {}
triggerVolumes._triggerExitRequestTable = {}
triggerVolumes._triggerEnterCparamTable = {}
triggerVolumes._triggerExitCparamTable = {}

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: triggerVolumes.update()
--| brief:
--|   Tests the trigger volumes for any commands set through the cparamOnEnter, cparamOnExit, requestOnEnter and requestOnExit
--|  functions.  If this command is not called during update the automatic commands will not be sent. 
--|
--| groups: Update
--| page: TriggerVolumes
------------------------------------------------------------------------------------------------------------------------
triggerVolumes.update = function()

  if triggerVolumes.data == nil then 
    triggerVolumes.data = __triggerVolumes
  end

  for volumeName, volumeInstanceData in pairs(triggerVolumes.data) do
    for instance, volumeData in pairs(volumeInstanceData) do
      if volumeData.enteredVolume == true then
        if(triggerVolumes._triggerEnterRequestTable[volumeName] ~= nil) then
          for index, requestTable in ipairs(triggerVolumes._triggerEnterRequestTable[volumeName] ) do
            if(requestTable.sendOnPartialEntry == true) then 
              broadcastRequest(requestTable.request, instance)
            end
          end
        end
            
        if(triggerVolumes._triggerEnterCparamTable[volumeName] ~= nil) then
          for index, cparamTable in ipairs(triggerVolumes._triggerEnterCparamTable[volumeName] ) do
            if(cparamTable.sendOnPartialEntry == true) then
              setControlParam(cparamTable.cparam, cparamTable.value)
            end
          end
        end
      end
         
      if volumeData.fullyEnteredVolume == true then
        if(triggerVolumes._triggerEnterRequestTable[volumeName] ~= nil) then
          for index, requestTable in ipairs(triggerVolumes._triggerEnterRequestTable[volumeName] ) do
            if(requestTable.sendOnPartialEntry ~= true) then 
              broadcastRequest(requestTable.request, instance)
            end
          end
        end
            
        if(triggerVolumes._triggerEnterCparamTable[volumeName] ~= nil) then
          for index, cparamTable in ipairs(triggerVolumes._triggerEnterCparamTable[volumeName] ) do
            if(cparamTable.sendOnPartialEntry ~= true) then 
              setControlParam(cparamTable.cparam, cparamTable.value)
            end
          end
        end
        volumeData.wasFullyInsideLastUpdate = true
      end
          
      if volumeData.leftVolume == true then 
        if(triggerVolumes._triggerExitRequestTable[volumeName] ~= nil) then
          for index, requestTable in ipairs(triggerVolumes._triggerExitRequestTable[volumeName] ) do 
            local request = requestTable.request
            if not requestTable.onlySendOnFullyEnteredAndExited or volumeData.wasFullyInsideLastUpdate then
              broadcastRequest(request, instance)
            end
          end
        end
            
        if(triggerVolumes._triggerExitCparamTable[volumeName] ~= nil) then
          for index, cparamTable in ipairs(triggerVolumes._triggerExitCparamTable[volumeName] ) do 
            setControlParam(cparamTable.cparam, cparamTable.value)
          end
        end
        volumeData.wasFullyInsideLastUpdate = false
      end      
    end
  end  
end
  
------------------------------------------------------------------------------------------------------------------------
-- Init Scripting
------------------------------------------------------------------------------------------------------------------------


------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: triggerVolumes.requestOnEnter(string volumeName, string request, boolean sendOnPartialEntry)
--| brief:
--|   When any instance enters the given volume send the specified request.  For  this behaviour to work correctly the
--|   function triggerVolumes.update() must be called in the update script. 
--|
--| param: volumeName string the name of a triggerVolume in the navigator
--| param: request string the name of the request
--| param:
--|   sendOnPartialEntry boolean set this parameter to true if the request should be sent when any portion of the character enters 
--|   the volume.  By default the reqyest will only be sent when every bone of the character is in the volume
--| groups: Init
--| page: TriggerVolumes
------------------------------------------------------------------------------------------------------------------------
triggerVolumes.requestOnEnter = function(volumeName, request, sendOnPartialEntry)
  if(triggerVolumes._triggerEnterRequestTable[volumeName] == nil) then
    triggerVolumes._triggerEnterRequestTable[volumeName] = {}
  end
  local requestTable = {}
  requestTable.request = request
  requestTable.sendOnPartialEntry = sendOnPartialEntry
  table.insert(triggerVolumes._triggerEnterRequestTable[volumeName], requestTable)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: triggerVolumes.requestOnExit(string volumeName, string request, boolean onlySendOnFullyEnteredAndExited)
--| brief:
--|   When any instance fully exits the given volume send the specified request. 
--|   For this behaviour to work correctly the function triggerVolumes.update() must be called in the update script. 
--|
--| param: volumeName string the name of a triggerVolume in the navigator
--| param: request string the name of the request
--| param: onlySendOnFullyEnteredAndExited whether the request should only be sent after being fully entered then exited.
--| groups: Init
--| page: TriggerVolumes
------------------------------------------------------------------------------------------------------------------------
triggerVolumes.requestOnExit = function(volumeName, request, onlySendOnFullyEnteredAndExited)
  if(triggerVolumes._triggerExitRequestTable[volumeName] == nil) then
    triggerVolumes._triggerExitRequestTable[volumeName] = {}
  end
  
  local requestTable = {}
  requestTable.request = request
  if onlySendOnFullyEnteredAndExited == nil then
    requestTable.onlySendOnFullyEnteredAndExited = false
  else
    requestTable.onlySendOnFullyEnteredAndExited = onlySendOnFullyEnteredAndExited
  end
  
  
  table.insert(triggerVolumes._triggerExitRequestTable[volumeName], requestTable)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: triggerVolumes.cparamOnEnter(string volumeName, string cparam, object value, boolean sendOnPartialEntry)
--| brief:
--|   When any instance enters the given volume set the control parameter with the given name to the value specified.  For  
--|   this behaviour to work correctly the function triggerVolumes.update() must be called in the update script. 
--|
--| param: volumeName string the name of a triggerVolume in the navigator
--| param: cparam string the name of the control parameter
--| param: value object the value to set the control paramater.  see setControlParam notes for details on how to construct these values for complex types
--| param: sendOnPartialEntry boolean set this parameter to true if the cparam should be set when any portion of the character enters the volume.  By default the cparam will only be set when every bone of the character is in the 
--|                      volume
--| groups: Init
--| page: TriggerVolumes
------------------------------------------------------------------------------------------------------------------------
triggerVolumes.cparamOnEnter = function(volumeName, cparam, value, sendOnPartialEntry)
    if(triggerVolumes._triggerEnterCparamTable[volumeName] == nil) then
      triggerVolumes._triggerEnterCparamTable[volumeName] = {}
    end
    local cparamTable = {}
    cparamTable.cparam = cparam
    cparamTable.value = value
    cparamTable.sendOnPartialEntry = sendOnPartialEntry
    
    table.insert(triggerVolumes._triggerEnterCparamTable[volumeName], cparamTable)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: triggerVolumes.cparamOnExit(string volumeName, string cparam, object value)
--| brief:
--|   When any instance fully exits the given volume set the control parameter with the given name to the value specified. 
--|   For this  behaviour to work correctly the function triggerVolumes.update() must be called in the update script. 
--|
--| param: volumeName string the name of a triggerVolume in the navigator
--| param: cparam string the name of the control parameter
--| param: value object the value to set the control paramater.  see setControlParam notes for details on how to construct these values for complex types
--| groups: Init
--| page: TriggerVolumes
------------------------------------------------------------------------------------------------------------------------
triggerVolumes.cparamOnExit = function(volumeName, cparam, value)
    if(triggerVolumes._triggerExitCparamTable[volumeName] == nil) then
      triggerVolumes._triggerExitCparamTable[volumeName] = {}
    end
    local cparamTable = {}
    cparamTable.cparam = cparam
    cparamTable.value = value
    
    table.insert(triggerVolumes._triggerExitCparamTable[volumeName], cparamTable)
  
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean triggerVolumes.enteredVolume(string volumeName, integer instanceID)
--| brief:
--|   Returns true the first frame when any joint of the character enters the volume.  Useful
--|   for sending requests and setting control parameter values if you need a single shot action
--|   This could trigger multiple times if the character brushes the volume edge
--|
--| param: volumeName string the name of a triggerVolume in the navigator
--| param: instanceID integer optional parameter for the instance to check.  The active instance will be used if this is not provided
--| groups: Update
--| page: TriggerVolumes
------------------------------------------------------------------------------------------------------------------------
triggerVolumes.enteredVolume = function(volumeName, instanceID)
  if triggerVolumes.data == nil then 
     triggerVolumes.data = __triggerVolumes
  end

  if(instanceID == nil) then 
    instanceID = getActiveInstance()
  end
  
  for curVolName, volumeInstanceData in pairs(triggerVolumes.data) do
     if curVolName == volumeName then
        for instance, volumeData in pairs(volumeInstanceData) do
    if(instance == instanceID) then
      if volumeData.enteredVolume == true then
        return true, instance
      end
    end
  end
     end
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean triggerVolumes.fullyEnteredVolume(string volumeName, integer instanceID)
--| brief:
--|   Returns true the first frame when all joints of the character enters the volume.  Useful
--|   for sending requests and setting control parameter values if you need a single shot action
--|   This is less likely to trigger multiple times if the character brushes the volume edge
--|
--| param: volumeName string the name of a triggerVolume in the navigator
--| param: instanceID integer optional parameter for the instance to check.  The active instance will be used if this is not provided
--| groups: Update
--| page: TriggerVolumes
------------------------------------------------------------------------------------------------------------------------
triggerVolumes.fullyEnteredVolume = function(volumeName, instanceID)
  if triggerVolumes.data == nil then 
     triggerVolumes.data = __triggerVolumes
  end

  if(instanceID == nil) then 
    instanceID = getActiveInstance()
  end
  
  for curVolName, volumeInstanceData in pairs(triggerVolumes.data) do
     if curVolName == volumeName then
        for instance, volumeData in pairs(volumeInstanceData) do
     if(instance == instanceID) then
             if volumeData.fullyEnteredVolume == true then
         return true, instance
       end
     end
  end
     end
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean triggerVolumes.leftVolume(string volumeName, integer instanceID)
--| brief:
--|   Returns true the first frame when all joints of the character leave the volume if any joint had been in the volume.  
--|   This is likely to trigger multiple times if the character brushes the volume edge
--|
--| param: volumeName string the name of a triggerVolume in the navigator
--| param: instanceID integer optional parameter for the instance to check.  The active instance will be used if this is not provided
--| groups: Update
--| page: TriggerVolumes
------------------------------------------------------------------------------------------------------------------------
triggerVolumes.leftVolume = function(volumeName, instanceID)
  if triggerVolumes.data == nil then 
     triggerVolumes.data = __triggerVolumes
  end

  if(instanceID == nil) then 
    instanceID = getActiveInstance()
  end
  
  for curVolName, volumeInstanceData in pairs(triggerVolumes.data) do
     if curVolName == volumeName then
        for instance, volumeData in pairs(volumeInstanceData) do
    if(instance == instanceID) then 
      if volumeData.leftVolume == true then
        return true, instance
      end
    end
  end
     end
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean triggerVolumes.inVolume(string volumeName, integer instanceID)
--| brief:
--|   Returns true if any joint of the character is in the trigger volume 
--|
--| param: volumeName string the name of a triggerVolume in the navigator
--| param: instanceID integer optional parameter for the instance to check.  The active instance will be used if this is not provided
--| groups: Update
--| page: TriggerVolumes
------------------------------------------------------------------------------------------------------------------------
triggerVolumes.inVolume = function(volumeName, instanceID)
  if triggerVolumes.data == nil then 
     triggerVolumes.data = __triggerVolumes
  end

  if(instanceID == nil) then 
    instanceID = getActiveInstance()
  end
  
  for curVolName, volumeInstanceData in pairs(triggerVolumes.data) do
     if curVolName == volumeName then
        for instance, volumeData in pairs(volumeInstanceData) do
    if(instance == instanceID) then 
      if volumeData.inVolume == true then
        return true, instance
      end
    end
  end
     end
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: boolean triggerVolumes.fullyInVolume(string volumeName, integer instanceID)
--| brief:
--|   Returns true if every joint of the character is in the trigger volume 
--|
--| param: volumeName string the name of a triggerVolume in the navigator
--| param: instanceID integer optional parameter for the instance to check.  The active instance will be used if this is not provided
--| groups: Update
--| page: TriggerVolumes
------------------------------------------------------------------------------------------------------------------------
triggerVolumes.fullyInVolume = function(volumeName, instanceID)
  if triggerVolumes.data == nil then 
     triggerVolumes.data = __triggerVolumes
  end

  if(instanceID == nil) then 
    instanceID = getActiveInstance()
  end
  
  for curVolName, volumeInstanceData in pairs(triggerVolumes.data) do
     if curVolName == volumeName then
        for instance, volumeData in pairs(volumeInstanceData) do
    if(instance == instanceID) then
            if volumeData.inVolume == true and volumeData.inVolumeRatio >= 1.0 then
        return true, instance
      end
    end
  end
     end
  end
  return false
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: float triggerVolumes.ratioInVolume(string volumeName, integer instanceID)
--| brief:
--|   Returns the ratio of the joints of the character that are in the trigger volume versus the total number of character
--|   joints (jointInVolume / totalJoints)
--|
--| param: volumeName string the name of a triggerVolume in the navigator
--| param: instanceID integer optional parameter for the instance to check. The active instance will be used if this is not provided
--| groups: Update
--| page: TriggerVolumes
------------------------------------------------------------------------------------------------------------------------
triggerVolumes.ratioInVolume = function(volumeName, instanceID)
  if triggerVolumes.data == nil then 
     triggerVolumes.data = __triggerVolumes
  end
  
  if(instanceID == nil) then 
    instanceID = getActiveInstance()
  end
  
  for curVolName, volumeInstanceData in pairs(triggerVolumes.data) do
     if curVolName == volumeName then
        for instance, volumeData in pairs(volumeInstanceData) do
    if(instance == instanceID) then
      return volumeData.inVolumeRatio
    end
  end
     end
  end
  return 0.0
end