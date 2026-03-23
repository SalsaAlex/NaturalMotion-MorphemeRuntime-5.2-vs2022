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
require [[rigBuilder/Options.lua]]

local id = nil

------------------------------------------------------------------------------------------------
-- populate combo boxes with list of anim sets.
------------------------------------------------------------------------------------------------
local populateRigComboBox = function(comboBox)
  assert(comboBox)
  local list = listAnimSets()  

  comboBox:setItems(list)
  comboBox:setSelectedItem(list[1])  

end --function


------------------------------------------------------------------------------------------------
-- cancel button function - hide window
------------------------------------------------------------------------------------------------
local cancel = function()  
  creatAnimRigWindow:hide()  
end

------------------------------------------------------------------------------------------------
-- get full source xmd path and set file in box
------------------------------------------------------------------------------------------------
local getXMDRigPath = function()
  local animdl = ui.createFileDialog({name = "XMDRigControl",
                                      caption = "Template Physics Path",
                                      wildcard = "xmd files|xmd"} )
  animdl:show()  
  local NewImportPath = animdl:getFullPath() 
  local tempName = ui.getWindow("CreatAnimRigWindow|XMDRigBox")    
  tempName:setValue(NewImportPath)
  
end --function

------------------------------------------------------------------------------------------------
-- get full path and file name to save to
------------------------------------------------------------------------------------------------
local getAnimRigPath = function()
  local animdl = ui.createFileDialog({name = "AnimPathControl",
                                      caption = "Template Physics Path",
                                      wildcard = "morpheme:connect animation rig files|mcarig"} )
  animdl:show()  
  local NewImportPath = animdl:getFullPath() 
  local tempName = ui.getWindow("CreatAnimRigWindow|TemplateAnimRigBox")    
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
  local tempName = ui.getWindow("CreatAnimRigWindow|TemplatePhysicsRigBox")    
  tempName:setValue(NewImportPath)
  
end --function


------------------------------------------------------------------------------------------------
-- get full path and file name to save to
------------------------------------------------------------------------------------------------
local getSaveAnimRigPath = function()
  local physdl = ui.createFileDialog({name = "AnimationPathControl",
                                      caption = "Save Animation Path",
                                      wildcard = "morpheme:connect animation rig files|mcarig"}) 
  physdl:show()
  local NewImportPath = physdl:getFullPath()
  local tempName = ui.getWindow("CreatAnimRigWindow|SaveAnimRigBox")    
  tempName:setValue(NewImportPath)
  
end --function

------------------------------------------------------------------------------------------------

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
    ----------------------------------------------------------
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
-- select animation on creation of new rig
------------------------------------------------------------------------------------------------
local selectAnim = nil  
selectAnim = function()
  local selectAnimTake = anim.selectTakeInAssetManager(id)
  unregisterEventHandler("mcAnimationSetSelectionChange",selectAnim) 
end

------------------------------------------------------------------------------------------------
-- find the index of a current joint name
------------------------------------------------------------------------------------------------
local findJointIndexByName = function(jointName, animSetName)
  local index = anim.getRigChannelIndex(jointName, animSetName)  
  return index  
end

------------------------------------------------------------------------------------------------
-- create morpheme animation rig main function
------------------------------------------------------------------------------------------------
--local createAnimRig (string animSetName, string rigPath, string templateAnim, string templatePhysics, table tags )
createAnimRig = function (animSetName, xmdPath, animRigPath, templateAnim, templatePhysics, tags )
  
  local dir = splitFilePath(xmdPath)
  anim.addAnimationLocation(dir)
  anim.waitForAnimationsToLoad()
  id = anim.getResourceId(xmdpath)

  local animRig = anim.createRig{sourceFile = xmdPath, destFile = animRigPath, hipBoneName = tags[1], trajBoneName  = tags[2]}

  local newAnimationSet = createAnimSet{Name = animSetName, Format = preferences.get("DefaultAnimationFormat"), Rig = animRigPath}
  local root = findJointIndexByName(tags[1], animSetName)
  local trajectory = findJointIndexByName(tags[2], animSetName)
  local markUp = anim.getRigMarkupData(animSetName)

  markUp.hipIndex = root
  markUp.trajectoryIndex = trajectory

  local setMarkUp = anim.setRigMarkupData(markUp, animSetName)
  local setTempalte = anim.setAnimSetTemplate(animSetName, templatePath, defaultTemplateAnimRigPath, defaultTemplatePhysicsRigPath, false)

  autoMapAnimationTemplateMapping(animSetName, 0)
  registerEventHandler("mcAnimationSetSelectionChange", selectAnim)

  local set = setSelectedAssetManagerAnimSet(animSetName)
  
  return root, trajectory

end

-------------------------------------------------------------------------------------------------
-- set variables from createAnimRigWindow() and call createAnimRig()
------------------------------------------------------------------------------------------------
local setVariables = function()

  local checkList = true
  local errorMessage = "\n"  
 
  local animSetName = ui.getWindow("CreatAnimRigWindow|AnimSetNameBox"):getValue()
  local XMDRig = ui.getWindow("CreatAnimRigWindow|XMDRigBox"):getValue()
  local newAnimRigPathName = ui.getWindow("CreatAnimRigWindow|SaveAnimRigBox"):getValue()
  local tAnimRigPathName = ui.getWindow("CreatAnimRigWindow|TemplateAnimRigBox"):getValue()
  local tPhysicsRigPathName = ui.getWindow("CreatAnimRigWindow|TemplatePhysicsRigBox"):getValue()

  
  if(XMDRig == "")then
    errorMessage = errorMessage .. "Export path for new animation rig was not selected.\n"
    errorDialog(errorMessage)
    checkList = false
  end
  if(newAnimRigPathName == "")then
    errorMessage = errorMessage .. "Export path for new animation rig was not selected.\n"
    errorDialog(errorMessage)
    checkList = false
  end
  if(tAnimRigPathName == "")then
    errorMessage = errorMessage .. "Template animation rig path was not selected.\n"
    errorDialog(errorMessage)
    checkList = false
  end
  if(tPhysicsRigPathName == "")then
    errorMessage = errorMessage .. "Template physics rig path was not selected.\n"
    errorDialog(errorMessage)
    checkList = false
  end  
  
  if(checkList) then
    --createAnimRig( "set" animSetName, ".xmd file" XMDRig, ".mcarig file" newAnimRigPathName, ".mcarig file" tAnimRigPathName , ".mcprig file" tPhysicsRigPathName, "Options.lua tags" animTags)
    createAnimRig(animSetName, XMDRig, newAnimRigPathName, tAnimRigPathName ,tPhysicsRigPathName, animTags)
    creatAnimRigWindow:hide()
  end

end --function


------------------------------------------------------------------------------------------------
-- window function.
------------------------------------------------------------------------------------------------
creatAnimRigWindowFunc = function()

  creatAnimRigWindow = nil
  collectgarbage()
  
  creatAnimRigWindow = ui.createModelessDialog
  { 
    caption = "Create Animation Rig for Morpheme:Connect 3.6.2", 
    size = {width = 500, height = 250}, 
    name = "CreatAnimRigWindow"
  }

  local btnWidth = 100
  local btnHeight = 50


  -- start main layout   
    creatAnimRigWindow:beginVSizer()
      ----------------------------------------------------------
      creatAnimRigWindow:beginHSizer{flags = "expand"}
      
        local copyFromText = creatAnimRigWindow:addStaticText      
        {
          text = " New Animation Set",
          name = "NewAnimSetText",
          size = { width = 200 },
        }  
        copyFromText:setFont("bold")
        
      creatAnimRigWindow:endSizer() 
      ----------------------------------------------------------
      creatAnimRigWindow:beginHSizer{flags = "expand"}    
          local AnimSetNameText = creatAnimRigWindow:addStaticText
            {
              text = "Animation Set Name\t",
              name = "AnimSetNameText",
              size = { width = 120 }
            }
          AnimSetNameText:setToolTip("New name of the animation set to be created")
          
          local AnimationSetName = creatAnimRigWindow:addTextBox
          {
            flags = "expand",
            proportion = 1,
            size = { width = 150 },
            setReadOnly = false,              
            name = "AnimSetNameBox"
          }         
        
      creatAnimRigWindow:endSizer()
      ----------------------------------------------------------
      creatAnimRigWindow:beginHSizer{flags = "expand"}
          
            local XMDRigText = creatAnimRigWindow:addStaticText
            {
              text = "Source XMD Rig\t",
              name = "XMDRigText",
              size = { width = 120 }
            }
            XMDRigText:setToolTip("Source xmd rig file exported from 3DS Max or Maya")
          
            local XMDRigBox = creatAnimRigWindow:addTextBox
            {
              flags = "expand",
              proportion = 1,
              setReadOnly = false,              
              name = "XMDRigBox"
            }
            local XMDRigButton = creatAnimRigWindow:addButton
            {
              setReadOnly = false,
              label = "...",
              name = "XMDRigButton"
            }       
            XMDRigButton:setOnClick(getXMDRigPath)
            
      creatAnimRigWindow:endSizer() 
      ----------------------------------------------------------
      creatAnimRigWindow:beginHSizer{flags = "expand"}
          
            local SaveAnimRigText = creatAnimRigWindow:addStaticText
            {
              text = "Save Morpheme Anim Rig\t",
              name = "SaveAnimRigText",
              size = { width = 120 }
            }
            SaveAnimRigText:setToolTip("Newly created morpheme animatoin rig. (*.mcarig)")
            local SaveAnimRigBox = creatAnimRigWindow:addTextBox
            {
              flags = "expand",
              proportion = 1,
              setReadOnly = false,              
              name = "SaveAnimRigBox"
            }
            local SaveAnimRigButton = creatAnimRigWindow:addButton
            {
              setReadOnly = false,
              label = "...",
              name = "SaveAnimRigButton"
            }       
            SaveAnimRigButton:setOnClick(getSaveAnimRigPath)
            
      creatAnimRigWindow:endSizer() 
      ----------------------------------------------------------
      -- splitter
      ----------------------------------------------------------
      creatAnimRigWindow:beginHSizer{flags = "expand"}        
        local splitter = creatAnimRigWindow:addSplitter      
        {
          flags = "expand",
          proportion = 1,
          size = { width = 170 },
        }  
        
      creatAnimRigWindow:endSizer()
      ----------------------------------------------------------
      creatAnimRigWindow:beginHSizer{flags = "expand"}
      
        local copyFromText = creatAnimRigWindow:addStaticText      
        {
          text = "Template Rigs",
          name = "TemplateRigText",
          size = { width = 170 },
        }  
        copyFromText:setFont("bold"
        )
      creatAnimRigWindow:endSizer() 
      ----------------------------------------------------------
      creatAnimRigWindow:beginHSizer{flags = "expand"}
          
            local TemplateAnimRig = creatAnimRigWindow:addStaticText
            {
              text = "Animation Rig\t",
              name = "TemplateAnimRig",
              size = { width = 80 }
            }
            TemplateAnimRig:setToolTip("Morpheme animation rig to copy data from. Will not be used if not matched.")
            local TemplateAnimRigBox = creatAnimRigWindow:addTextBox
            {
              flags = "expand",
              proportion = 1,
              value = defaultTemplateAnimRigPath,
              setReadOnly = false,
              name = "TemplateAnimRigBox"
            }
            local AnimPathButton = creatAnimRigWindow:addButton
            {
              setReadOnly = false,
              label = "...",
              name = "AnimPathButton"
            }       
            AnimPathButton:setOnClick(getAnimRigPath)
            
      creatAnimRigWindow:endSizer()   
      ----------------------------------------------------------
      creatAnimRigWindow:beginHSizer{flags = "expand"}
          
            local TemplatePhysicsRig = creatAnimRigWindow:addStaticText
            {
              text = "Physics Rig\t",
              name = "TemplatePhysicsRig",
              size = { width = 80 }
            }
            TemplatePhysicsRig:setToolTip("Morpheme physics rig to copy data from. Will not be used if not matched.")
            local TemplatePhysicsRigBox = creatAnimRigWindow:addTextBox
            {
              flags = "expand",
              proportion = 1,
              value = defaultTemplatePhysicsRigPath;
              setReadOnly = false,
              name = "TemplatePhysicsRigBox"
            }
            local PhysicsPathButton = creatAnimRigWindow:addButton
            {
              setReadOnly = false,
              label = "...",
              name = "PhysicsPathButton"
            }       
            PhysicsPathButton:setOnClick(getPhysicsRigPath)
            
      creatAnimRigWindow:endSizer()
      ----------------------------------------------------------
      creatAnimRigWindow:beginHSizer()
      
        -- Add buttons to OK or Cancel this operation.
        local OKBox = creatAnimRigWindow:addButton
        {
          flags = "expand",
          proportion = 1,
          label = "OK",
          name = "OKBox"
        }
        OKBox:setOnClick(setVariables)

        local CancelBox = creatAnimRigWindow:addButton
        {
          flags = "expand",
          proportion = 1,
          label = "Cancel",
          name = "CancelBox"
        }
         CancelBox:setOnClick(cancel)
       
      creatAnimRigWindow:endSizer()
      ----------------------------------------------------------            
      creatAnimRigWindow:beginHSizer{flags = "expand"}        
        local splitterHelpText = creatAnimRigWindow:addSplitter      
        {
          flags = "expand",
          proportion = 1,
          size = { width = 170 },
        }  
        
      creatAnimRigWindow:endSizer()
      creatAnimRigWindow:beginHSizer{flags = "expand"}
          
            local FileName = creatAnimRigWindow:addStaticText
            {
              text = "Help:   \n",
              name = "Help"
            }
            
      creatAnimRigWindow:endSizer()
      
  creatAnimRigWindow:endSizer()
  creatAnimRigWindow:show()


end

--creatAnimRigWindowFunc()