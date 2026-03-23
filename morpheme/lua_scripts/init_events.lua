
------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: PAGE
--| name: Events
--| title: Event Functions
--| desc: Supports LUA callbacks from connect when events occur.
------------------------------------------------------------------------------------------------------------------------

__IDREGISTEREDEVENTS = { }

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil registerEventHandler(string event, function handlerFunction)
--| signature: nil registerEventHandler(string event, function handlerFunction, string replacementIdentifier)
--| brief:
--|   Register an event handler function for an event, the handler function will then be called every time the
--|   event occurs.
--|   For more information on which events are available see "Registering event handlers" in the User Guide.
--|
--| param: string event the event type to register a handler for
--| param: function handlerFunction then function to be called when the event occurs
--| param: string replacementIdentifier an identifier used to ensure only one event is registered for a specific
--|               component, for example registering and reregistering events when ui is rebuilt.
--|
--| environments: GlobalEnv
--| page: Events
------------------------------------------------------------------------------------------------------------------------
registerEventHandler = function(event, func, replacementId)
  if EVENTS[event] == nil then
    error("Attempting to register a handler for an unknown event: "..event)
  end
  
  if type(replacementId) == "string" then
    if type(__IDREGISTEREDEVENTS[replacementId]) == "function" then
      unregisterEventHandler(event, __IDREGISTEREDEVENTS[replacementId], replacementId)
    end
    
    __IDREGISTEREDEVENTS[replacementId] = func
  end
  
  table.insert(EVENTS[event], func)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: nil unregisterEventHandler(string event, function handlerFunction)
--| signature: nil unregisterEventHandler(string event, function handlerFunction, string replacementIdentifier)
--| brief:
--|   Unregister an event handler function for an event.
--|   For more information on which events are available see "Registering event handlers" in the User Guide.
--|
--| param: string event the event type to register a handler for
--| param: function handlerFunction then function to be called when the event occurs
--| param: string replacementIdentifier an identifier used to ensure only one event is registered for a specific
--|               component, for example registering and reregistering events when ui is rebuilt.
--|
--| environments: GlobalEnv
--| page: Events
------------------------------------------------------------------------------------------------------------------------
unregisterEventHandler = function(event, func, replacementId)
  if EVENTS[event] == nil then
    error("Attempting to unregister a handler for an unknown event: "..event)
  end
  
  if type(replacementId) == "string" then
    __IDREGISTEREDEVENTS[replacementId] = nil    
  end

  for i, v in ipairs(EVENTS[event]) do
    if v == func then
      table.remove(EVENTS[event], i)
      return true
    end
  end

  return false
end