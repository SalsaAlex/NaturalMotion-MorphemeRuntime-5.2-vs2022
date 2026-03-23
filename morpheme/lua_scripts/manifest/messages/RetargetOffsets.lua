------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------
-- RetargetOffsets message definition.
------------------------------------------------------------------------------------------------------------------------
registerMessage("RetargetOffsets",
  {
    helptext = "Internal message to update the retargeting system",
    version = 1,
    id = generateMessageId(idNamespaces.NaturalMotion, 102),
    supportsPresets = false,
    --------------------------------------------------------------------------------------------------------------------
    attributes =
    {
    },
    presets = 
    {
    },
    --------------------------------------------------------------------------------------------------------------------
    serialize = function(node, Stream)
      -- The function writeAttributesInOrder defined in ManifestUtils.lua, and writes the attributes in the exact order 
      -- they are declared in the attribute table above. Because the runtime must interpret the binary structure sent 
      -- from connect it is important to keep the runtime structure and the attributes declared in messages synchronized
      
      writeAttributesInOrder(node, Stream)
    end,
    --------------------------------------------------------------------------------------------------------------------
    compareToPreset = function(node, preset)
      return false
    end,

    --------------------------------------------------------------------------------------------------------------------
    displayFunction = function(panel, object)
    end,

    upgrade = function(node, version, pinLookupTable)
    end,
  }
)

registerMessage("RetargetCharacterScale",
  {
    helptext = "Internal message to update the retargeting system's characterScale",
    version = 1,
    id = generateMessageId(idNamespaces.NaturalMotion, 103),
    supportsPresets = false,
    --------------------------------------------------------------------------------------------------------------------
    attributes =
    {
    },
    presets =
    {
    },
    --------------------------------------------------------------------------------------------------------------------
    serialize = function(node, Stream)
      -- The function writeAttributesInOrder defined in ManifestUtils.lua, and writes the attributes in the exact order
      -- they are declared in the attribute table above. Because the runtime must interpret the binary structure sent
      -- from connect it is important to keep the runtime structure and the attributes declared in messages synchronized

      writeAttributesInOrder(node, Stream)
    end,
    --------------------------------------------------------------------------------------------------------------------
    compareToPreset = function(node, preset)
      return false
    end,

    --------------------------------------------------------------------------------------------------------------------
    displayFunction = function(panel, object)
    end,

    upgrade = function(node, version, pinLookupTable)
    end,
  }
)
