------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

local appScriptsDir = app.getAppScriptsDir()
-- load base
require "manifest/base.lua"
require "manifest/ManifestChange.lua"

-- load manifest
executeAllScripts(appScriptsDir .. "manifest/nodes")
executeAllScripts(appScriptsDir .. "manifest/transitions")
executeAllScripts(appScriptsDir .. "manifest/conditions")
executeAllScripts(appScriptsDir .. "manifest/extras")
executeAllScripts(appScriptsDir .. "manifest/notes")
executeAllScripts(appScriptsDir .. "manifest/messages")
executeAllScripts(appScriptsDir .. "manifest/assetManagerNetwork")
-- load any reference manifests
executeAllScripts(appScriptsDir .. "references/")

if not mcn.isPhysicsDisabled() then
  local physicsExportFiles = app.enumerateFiles(string.format("%smanifest/export", appScriptsDir), "Physics*.lua")
  for i = 1, table.getn(physicsExportFiles) do
    require(physicsExportFiles[i])
  end
end

if not mcn.isEuphoriaDisabled() then
  require "manifest/export/EuphoriaBodyExport.lua"
end

-- if morpheme:connect is in command line the there is no point running the ui scripts
if not mcn.inCommandLineMode() then
  require "ui/FormatOptionsCommon.lua"
  require "ui/FormatOptionsASA.lua"
  require "ui/FormatOptionsNSA.lua"
  require "ui/FormatOptionsQSA.lua"
end

-- register default animation formats
-- the function arguments to formats.register will be nil if morpheme:connect is in command line mode.
animfmt.register("mba", { "genDeltas", "resample" }, addSampleRateFormatOptions, updateSampleRateFormatOptions)
animfmt.register("asa", { "genDeltas", "resample", "mfps", "nsub" }, addFormatOptionsPanelASA, updateFormatOptionsPanelASA)
animfmt.register("nsa", { "genDeltas", "resample", "mfps", "qset" }, addFormatOptionsPanelNSA, updateFormatOptionsPanelNSA)
animfmt.register("qsa", { "genDeltas", "resample", "mfps", "qset", "rate", "fpk", "pmethod", "qmethod" }, addFormatOptionsPanelQSA, updateFormatOptionsPanelQSA)

registerEventHandler(
  "mcAdjustAnimOptions",
  function(node, option)
    -- Adjust gen deltas animation option for nodes with flag set
    if getType(node) == "AnimWithEvents" then
      if getAttribute(node, "GenerateAnimationDeltas") then
        -- Try to preserve existing options if set by delimiting with a space
        if string.len(option) == 0 then
          return "genDeltas"
        else
          return string.format("%s genDeltas", option)
        end
      end
    end
    return option
  end
)

-- Event handler to modify the default name of transitions to be Source_Dest
registerEventHandler(
  "mcEdgeCreated",
  function(nodePath)
  
    if(mcn.isUndoing() or mcn.isRedoing()) then 
      return 
    end
    
    -- ensure the transit has the default name of TransitionType ThenNumber
    -- otherwise we dont want to rename it
    local type = getType(nodePath)
    local _, nodeName = splitNodePath(nodePath)
    if string.find(nodeName, type) ~= 1 then
      return 
    end
    
    local _, source, dest = nil, "Unknown", "Unknown"

    local sourcePath = listConnections{ Object = nodePath, Upstream = true, Downstream = false }
    assert(table.getn(sourcePath) == 1)
    _, source = splitNodePath(sourcePath[1])

    local destPath = listConnections{ Object = nodePath, Upstream = false, Upstream = false }
    -- We may have transitions from transitions as desitnations, make sure that we select the state
    local k, v
    for k, v in pairs(destPath) do
      local destType
      _, destType = getType(v)
      if (destType == "StateMachineNode") then
        _, dest = splitNodePath(v)
      end
    end

    rename(nodePath, source .. "_"..dest)
  end
)