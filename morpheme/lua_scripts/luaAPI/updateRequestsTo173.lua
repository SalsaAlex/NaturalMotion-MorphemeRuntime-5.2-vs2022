------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

updateRequestsTo173 = function()

  local requestObjects = ls{ Type = "Request" }
  local requestStrings = { }

  local curIndex = 1
  for i, v in ipairs(requestObjects) do
    local curReqStr = getAttribute(v, "RequestString")
    local addItem = true
    for j, k in ipairs(requestStrings) do
       if k == curReqStr then
          addItem = false
        end
    end
    if addItem == true then
      requestStrings[curIndex] = curReqStr
      curIndex = curIndex + 1
    end
  end

  for i, v in ipairs(requestStrings) do
    create("Request", requestStrings[i])
  end

  for i, v in requestObjects do
    local parentTrans = getParent(v)
    local newRequestCondPath = create("RequestCondition", parentTrans)
    local curReqStr = getAttribute(v, "RequestString")
    setAttribute((newRequestCondPath .. ".Request"), ("Requests|" .. curReqStr))
    delete(v)
  end
end

