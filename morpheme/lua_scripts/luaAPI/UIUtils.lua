------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: Button addImageButton(Panel panel, string name, string imageName)
--| brief: Adds a button with an image to a panel.
--| environments: GlobalEnv
--| page: ScriptUtilsAPI
------------------------------------------------------------------------------------------------------------------------
addImageButton = function(panel, name, imageName)
  local image = app.loadImage(imageName)

  local button = panel:addButton{
    name = name,
    label = "",
    image = image,
    flags = "expand;parentBackground",
    size = {
      width = image:getWidth(),
      height = image:getHeight()
    },
  }

  return button
end