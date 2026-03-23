------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

onExport = function(exportDir)
  -- Place any logic you would like before export begins here.  Please note that
  -- runtime IDs have not been assigned when the mcFileExportBegin event is called.

end

registerEventHandler("mcFileExportBegin", onExport)
