------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2012 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

-- final physics root name must be first - keep SKEL_Pelvis first as it will be set as the new root.
weldSelectionList = {"SKEL_Pelvis", "SKEL_ROOT", "SKEL_Spine_Root"}
skeletonPrefix = "SKEL"
mirrorMapping = {"_L_", "_R_"}
mirrorPlane = "YZ"
animTags = {"SKEL_ROOT", "mover"}
jointSize = {0.2, 1.25, 0.2}
minVolumeScale = 0.001
characterScale = 0.05

defaultTemplateAnimRigPath = "$(RootDir)\\TemplateRigs\\MaleNorth\\NorthCharacter.mcarig"
defaultTemplatePhysicsRigPath = "$(RootDir)\\TemplateRigs\\MaleNorth\\NorthCharacterReorder.mcprig"
templatePath = "$(AppRoot)\\resources\\rigTemplates\\Humanoid.mctmpl"


