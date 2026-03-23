------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2012 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- include other required scripts
------------------------------------------------------------------------------------------------------------------------
require [[rigBuilder/Main_Functions.lua]]

------------------------------------------------------------------------------------------------
-- cancel button function - hide window
------------------------------------------------------------------------------------------------
local cancel = function()  
  copyRockStarWindow:hide()  
end

------------------------------------------------------------------------------------------------
-- populate combo boxes with list of anim sets.
------------------------------------------------------------------------------------------------
local populateRigComboBox = function(comboBox)
  assert(comboBox)
  local list = listAnimSets()  

  comboBox:setItems(list)
  local currentAnimSet = getSelectedAssetManagerAnimSet()
  comboBox:setSelectedItem(currentAnimSet)  

end --function

------------------------------------------------------------------------------------------------
-- create error dialogue if something goes wrong
------------------------------------------------------------------------------------------------
local errorDialog = function(errorText)

  errorWindow = nil
  collectgarbage()
  
  errorWindow = ui.createModelessDialog
  { 
    caption = "Errors", 
    size = {width = 300, height = 150}, 
    name = "ErrorWindow"
  }
  errorWindow:beginVSizer()    
    errorWindow:beginHSizer{flags = "expand"}    
      local errorName = errorWindow:addStaticText
      {
         text = errorText,
         name = "Help"
      }              
    errorWindow:endSizer()      
  errorWindow:endSizer()
  
  errorWindow:show() 

end

------------------------------------------------------------------------------------------------
-- create window to tell user that rig is being made
------------------------------------------------------------------------------------------------
local wipDialog = function(toggle)

  wipWindow = nil
  collectgarbage()
  
  wipWindow = ui.createModelessDialog
  { 
    caption = "Copying....", 
    size = {width = 100, height = 100}, 
    name = "Copying"
  }
  wipWindow:beginVSizer()
    wipWindow:beginHSizer{flags = "expand"}    
      local message = wipWindow:addStaticText
      {
         text = "Currently creating physics rig...",
         name = "Help"
      }              
    wipWindow:endSizer()      
  wipWindow:endSizer()
  
  if(toggle)then
    wipWindow:show() 
  else
    wipWindow:hide()
  end

end

------------------------------------------------------------------------------------------------
-- get full path and file name to save to
------------------------------------------------------------------------------------------------
local getAnimRigPath = function()
  local animdl = ui.createFileDialog({name = "PhysicsPathControl",
                                      caption = "Template Physics Path",
                                      wildcard = "morpheme:connect animation rig files|mcarig"} )
  animdl:show()  
  local NewImportPath = animdl:getFullPath() 
  local tempName = ui.getWindow("CreateCopyRigWindow|TemplateAnimRigBox")    
  tempName:setValue(NewImportPath)
  
end --function

------------------------------------------------------------------------------------------------
-- get full path and file name to save to
------------------------------------------------------------------------------------------------
local getPhysicsRigPath = function()
  local physdl = ui.createFileDialog({name = "PhysicsPathControl",
                                      caption = "Template Physics Path",
                                      wildcard = "morpheme:connect physics rig files|mcprig"}) 
  physdl:show()
  local NewImportPath = physdl:getFullPath()
  local tempName = ui.getWindow("CreateCopyRigWindow|TemplatePhysicsRigBox")    
  tempName:setValue(NewImportPath)
  
end --function

------------------------------------------------------------------------------------------------
-- get full path and file name to save to
------------------------------------------------------------------------------------------------
local getSavePhysicsRigPath = function()
  local physdl = ui.createFileDialog({name = "PhysicsPathControl",
                                      caption = "Save Physics Path",
                                      wildcard = "morpheme:connect physics rig files|mcprig"}) 
  physdl:show()
  local NewImportPath = physdl:getFullPath()
  local tempName = ui.getWindow("CreateCopyRigWindow|SavePhysicsRigBox")    
  tempName:setValue(NewImportPath)
  
end --function

-------------------------------------------------------------------------------------------------
-- set variables from copyPhysicsRigWindow() and call main_copyRigs()
------------------------------------------------------------------------------------------------
local setVariables = function()

  local checkList = true
  local errorMessage = "\n"  
 
  local animSetName = ui.getWindow("CreateCopyRigWindow|CopyAnimSetComboBox"):getValue()
  local newPhysicsRigPathName = ui.getWindow("CreateCopyRigWindow|SavePhysicsRigBox"):getValue()
  local tAnimRigPathName = ui.getWindow("CreateCopyRigWindow|TemplateAnimRigBox"):getValue()
  local tPhysicsRigPathName = ui.getWindow("CreateCopyRigWindow|TemplatePhysicsRigBox"):getValue()

  if(newPhysicsRigPathName == "")then
    errorMessage = errorMessage .. "Export path for new physics rig was not selected.\n"
    errorDialog(errorMessage)
    checkList = false
  end
  if(tAnimRigPathName == "")then
    errorMessage = errorMessage .. "Template animation rig path was not selected.\n"
    errorDialog(errorMessage)
    checkList = false
  end
  if(tAnimRigPathName == "")then
    errorMessage = errorMessage .. "Template physics rig path was not selected.\n"
    errorDialog(errorMessage)
    checkList = false
  end  
  
  if(checkList) then
    wipDialog(true)
    main_copyRigs(tAnimRigPathName,tPhysicsRigPathName, newPhysicsRigPathName , animSetName)
    wipDialog(false)
    copyRockStarWindow:hide()
  end

end --function



------------------------------------------------------------------------------------------------
-- create copy rig window
------------------------------------------------------------------------------------------------
createCopyRigWindow = function()

  copyRockStarWindow = nil
  collectgarbage()
  
  copyRockStarWindow = ui.createModelessDialog
  { 
    caption = "Copy Physics Rig for Morpheme:Connect 3.6.2", 
    size = {width = 500, height = 250}, 
    name = "CreateCopyRigWindow"
  }

  local btnWidth = 100
  local btnHeight = 50

  -- start main layout   
    copyRockStarWindow:beginVSizer()
      ----------------------------------------------------------
      copyRockStarWindow:beginHSizer{flags = "expand"}
      
        local copyFromText = copyRockStarWindow:addStaticText      
        {
          text = "Anim Set",
          name = "CopyFromAnimSetText",
          size = { width = 170 },
        }  
        copyFromText:setFont("bold")
        
      copyRockStarWindow:endSizer() 
      ----------------------------------------------------------
      copyRockStarWindow:beginHSizer{flags = "expand"}    
        local CopyAnimSetComboBox = copyRockStarWindow:addComboBox
        {        
          name = "CopyAnimSetComboBox",
          onChanged = onComboChanged,
          size = { width = 170 }, 
        } 
        populateRigComboBox(CopyAnimSetComboBox)
        
      copyRockStarWindow:endSizer()
      ----------------------------------------------------------
      copyRockStarWindow:beginHSizer{flags = "expand"}
          
            local SavePhysicsRig = copyRockStarWindow:addStaticText
            {
              text = "Save Physics Rig\t",
              name = "SavePhysicsRig",
              size = { width = 80 }
            }
          
            local SavePhysicsRigBox = copyRockStarWindow:addTextBox
            {
              flags = "expand",
              proportion = 1,
              setReadOnly = false,              
              name = "SavePhysicsRigBox"
            }
            local SavePhysicsRigButton = copyRockStarWindow:addButton
            {
              setReadOnly = false,
              label = "...",
              name = "SavePhysicsRigButton"
            }       
            SavePhysicsRigButton:setOnClick(getSavePhysicsRigPath)
            
      copyRockStarWindow:endSizer() 
      ----------------------------------------------------------
      -- splitter
      ----------------------------------------------------------
      copyRockStarWindow:beginHSizer{flags = "expand"}        
        local splitter = copyRockStarWindow:addSplitter      
        {
          flags = "expand",
          proportion = 1,
          size = { width = 170 },
        }  
        
      copyRockStarWindow:endSizer()
      ----------------------------------------------------------
      copyRockStarWindow:beginHSizer{flags = "expand"}
      
        local copyFromText = copyRockStarWindow:addStaticText      
        {
          text = "Template Rigs",
          name = "TemplateRigText",
          size = { width = 170 },
        }  
        copyFromText:setFont("bold"
        )
      copyRockStarWindow:endSizer() 
      ----------------------------------------------------------
      copyRockStarWindow:beginHSizer{flags = "expand"}
          
            local TemplateAnimRig = copyRockStarWindow:addStaticText
            {
              text = "Animation Rig\t",
              name = "TemplateAnimRig",
              size = { width = 80 }
            }
          
            local TemplateAnimRigBox = copyRockStarWindow:addTextBox
            {
              flags = "expand",
              proportion = 1,
              value = defaultTemplateAnimRigPath,
              setReadOnly = false,
              name = "TemplateAnimRigBox"
            }
            local AnimPathButton = copyRockStarWindow:addButton
            {
              setReadOnly = false,
              label = "...",
              name = "AnimPathButton"
            }       
            AnimPathButton:setOnClick(getAnimRigPath)
            
      copyRockStarWindow:endSizer()   
      ----------------------------------------------------------
      copyRockStarWindow:beginHSizer{flags = "expand"}
          
            local TemplatePhysicsRig = copyRockStarWindow:addStaticText
            {
              text = "Physics Rig\t",
              name = "TemplatePhysicsRig",
              size = { width = 80 }
            }
          
            local TemplatePhysicsRigBox = copyRockStarWindow:addTextBox
            {
              flags = "expand",
              proportion = 1,
              value = defaultTemplatePhysicsRigPath,
              setReadOnly = false,
              name = "TemplatePhysicsRigBox"
            }
            local PhysicsPathButton = copyRockStarWindow:addButton
            {
              setReadOnly = false,
              label = "...",
              name = "PhysicsPathButton"
            }       
            PhysicsPathButton:setOnClick(getPhysicsRigPath)
            
      copyRockStarWindow:endSizer()
      ----------------------------------------------------------
      copyRockStarWindow:beginHSizer()
      
        -- Add buttons to OK or Cancel this operation.
        local OKBox = copyRockStarWindow:addButton
        {
          flags = "expand",
          proportion = 1,
          label = "OK",
          name = "OKBox"
        }
        OKBox:setOnClick(setVariables)

        local CancelBox = copyRockStarWindow:addButton
        {
          flags = "expand",
          proportion = 1,
          label = "Cancel",
          name = "CancelBox"
        }
         CancelBox:setOnClick(cancel)
       
      copyRockStarWindow:endSizer()
      ----------------------------------------------------------            
      copyRockStarWindow:beginHSizer{flags = "expand"}        
        local splitterHelpText = copyRockStarWindow:addSplitter      
        {
          flags = "expand",
          proportion = 1,
          size = { width = 170 },
        }  
        
      copyRockStarWindow:endSizer()
      copyRockStarWindow:beginHSizer{flags = "expand"}
          
            local FileName = copyRockStarWindow:addStaticText
            {
              text = "Help: ",
              name = "Help"
            }
            
      copyRockStarWindow:endSizer()
      
  copyRockStarWindow:endSizer()
  copyRockStarWindow:show()
  
end --function

--createCopyRigWindow()
