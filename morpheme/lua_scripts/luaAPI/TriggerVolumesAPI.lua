------------------------------------------------------------------------------------------------------------------------
local onExecutePreviewScripts = function()

  local script = [[require([[previewScripts\TriggerVolumes.lua]])]]
  local result = mcn.executeRuntimeScriptCommand(script)
end

registerEventHandler("mcExecutePreviewScripts", onExecutePreviewScripts)