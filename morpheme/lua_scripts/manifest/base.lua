------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

 -- register custom interfaces used by different node types
 -- the interfaces "Transforms", "Time" and "Events" are built in interface types
customInterfaces =
{
  {name = "Activity", colour = {170, 170, 100} },
}

for _, interface in pairs(customInterfaces) do
  registerCustomInterface(interface.name, interface.colour or { 125, 125, 125, })
end

generateNamespacedId = function(namespace, id)
  return ((namespace * 2 ^ 16) + id)
end

generateMessageId = function(namespace, id)
  return generateNamespacedId(namespace, id)
end


idNamespaces =
{
  NaturalMotion = 0,
}