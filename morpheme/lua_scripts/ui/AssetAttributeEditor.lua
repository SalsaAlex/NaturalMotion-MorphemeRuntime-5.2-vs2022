------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- Object creation functions.
------------------------------------------------------------------------------------------------------------------------
addAssetAttributeEditor = function(contextualPanel, forContext)
  local splitter = contextualPanel:addSplitter{ name = "AssetAttributes", forContext = forContext, flags = "expand", proportion = 1, sash = 200, dominant = 2 }
  splitter:addStockWindow{ type = "AssetAttributeEditor", name = "AssetAttributes" }
  return panel
end