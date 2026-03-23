------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------
require "ui/tools/ToolSettingsPages.lua"
require "ui/tools/ToolSettingsPanel.lua"
require "ui/NMXAttributeWidgets.lua"

removeToolSettingsPage("ApplyForceTool")
addToolSettingsPage(
  "ApplyForceTool",
  {
    title = "Impulse Tool Settings",
    attributes = {
      {
        name = "ImpulseSize",
        displayName = "Impulse size",
      },
    },
    settingsNode = "|ApplyForceToolSettings",
    create = defaultToolSettingsPageCreateFunction,
    update = defaultToolSettingsPageUpdateFunction,
  }
)