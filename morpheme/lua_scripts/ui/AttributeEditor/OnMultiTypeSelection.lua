------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

require "ui/AttributeEditor/AttributeEditor.lua"

------------------------------------------------------------------------------------------------------------------------
-- handle selection of multiple types, type filter table is an array containing
-- entry table, each of which has a 'type' string field and a 'nodes' table which is
-- an array of all the nodes of that type.
------------------------------------------------------------------------------------------------------------------------
attributeEditor.doMultiTypeSelection = function(panel, selection, typeTable)
  attributeEditor.logEnterFunc("attributeEditor.doMultiTypeMultiSelection()")

  panel:beginVSizer{ flags = "expand" }

    for i, v in ipairs(typeTable) do
      attributeEditor.log("adding attributes for node type \"%s\"", v.type)
      attributeEditor.doSingleTypeSelection(panel, v.nodes)

      if i < table.getn(typeTable) then
        local separator = panel:addPanel{ flags = "expand", proportion = 0 }
        separator:beginHSizer{ flags = "expand;group", proportion = 1 }
        separator:endSizer()
      end
    end

  panel:endSizer()

  attributeEditor.logExitFunc("attributeEditor.doMultiTypeMultiSelection()")
end

