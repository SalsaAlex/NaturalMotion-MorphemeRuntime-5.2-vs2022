------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2010 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------
  local mainShotAttributes = {}
  local priorityAttributes, reachAttributes, animAttributes
  local lookAttributes, deathTriggerAttributes, impulseAttributes
  local bodyStrengthAndBalanceAttributes, forcedFalldownAttributes, impactAttributes
------------------------------------------------------------------------------------------------------------------------
  local helpWindow
  local clearHelpText, setHelpText, bindHelpToWidget, bindAttributeHelpToWidget
  
-- Shot message definition.
------------------------------------------------------------------------------------------------------------------------
registerMessage("Shot",
  {
    helptext = "Orchestrates a shot performance.",
    version = 5,
    id = generateMessageId(idNamespaces.NaturalMotion, 101),
    supportsPresets = true,
    --------------------------------------------------------------------------------------------------------------------
    attributes =
    {
      -- rig part that got hit
      {
        name = "RigPartIndex",
        input = true,
        array = false,
        type = "int",
      },
      
      -- geometry of the hit
      { name = "HitPointLocalX", input = true, array = false, type = "float", },
      { name = "HitPointLocalY", input = true, array = false, type = "float", },
      { name = "HitPointLocalZ", input = true, array = false, type = "float", },
      
      { name = "HitNormalLocalX", input = true, array = false, type = "float", },
      { name = "HitNormalLocalY", input = true, array = false, type = "float", },
      { name = "HitNormalLocalZ", input = true, array = false, type = "float", },
      
      { name = "HitDirectionLocalX", input = true, array = false, type = "float", },
      { name = "HitDirectionLocalY", input = true, array = false, type = "float", },
      { name = "HitDirectionLocalZ", input = true, array = false, type = "float", },

      { name = "HitDirectionWorldX", input = true, array = false, type = "float", },
      { name = "HitDirectionWorldY", input = true, array = false, type = "float", },
      { name = "HitDirectionWorldZ", input = true, array = false, type = "float", },

      { name = "SourcePointWorldX", input = true, array = false, type = "float", },
      { name = "SourcePointWorldY", input = true, array = false, type = "float", },
      { name = "SourcePointWorldZ", input = true, array = false, type = "float", },
      
      {
        name = "Priority",
        displayName = "Priority",
        helptext = "Prioritisation can be used to arbitrate responses to multiple hits. The character's hit reaction will be interrupted and retargeted to respond (eg. reach for wound) on a shot with higher priority than the current one. So the character may reach for a head wound interrupting a reach for torso but not the other way around.",
        input = true,
        array = false,
        type = "int",
      },
      {
        name = "ReachDelay",
        displayName = "Reach delay",
        helptext = "The time delay before a reach for wound.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ReachDuration",
        displayName = "Reach duration",
        helptext = "This is the time the character should spend reaching for a wound.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "MaxReachSpeed",
        displayName = "Reach speed limit",
        helptext = "The maximimum speed the hand should attain whilst reaching. In m/s on the standard character",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ReflexAnimStart",
        displayName = "Delay",
        helptext = "The time delay between the arrival of a hit and the animated reflex response.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ReflexAnimRampDuration",
        displayName = "Ramp-up duration",
        helptext = "Duration of blend in to reflex animation.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ReflexAnimFullDuration",
        displayName = "Time at full strength",
        helptext = "Time spent holding the reflex animation at full strength.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ReflexAnimDerampDuration",
        displayName = "Ramp-down time",
        helptext = "Duration of blend back from reflex animation.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ReflexAnimMaxWeight",
        displayName = "Full strength level",
        helptext = "The maximum weight to be applied to the reflex animation (on a scale of 0 to 1).",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ReflexLookDuration",
        displayName = "Reflex look duration",
        helptext = "This is the time the character should spend looking at either the wound or the shot source.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "DeathTriggerDelay",
        displayName = "Death relax delay",
        helptext = "Delay between setting of the death trigger and initiation of the death response (strength ramp-down).",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "DeathRelaxDuration",
        displayName = "Death relax duration",
        helptext = "Duration of the death relax (strength ramp-down).",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ShotPartImpactMagnitude",
        displayName = "Part impulse magnitude",
        helptext = "Magnitude of bullet impulse to be applied locally to the rig part. In kg.m/s on the standard character.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ShotBodyImpactMagnitude",
        displayName = "Body impulse magnitude",
        helptext = "Magnitude of bullet impulse to be applied globally to the rig as a whole. In kg.m/s on the standard character.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ShotTorqueMultiplier",
        displayName = "Body impulse torque multiplier",
        helptext = "Multiplier to allow modulation of angular response. The response to the bullet impact can be exaggerated (using values > 1) or attenuated (using a value between 0 and 1)",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ShotLiftBoost",
        displayName = "Body lift impulse",
        helptext = "A vertical impulse most commonly used to reduce contact and the resulting friction between the character and the ground. Allows a shot impulse to flip or spin the character more easily. In kg.m/s on the standard character.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ShotImpulseYield",
        displayName = "Yield to impulse amount",
        helptext = "A value between between 0 and 1 sets the spine damping to be in effect when the bullet impulse is applied. A value closer to zero will make the spine react more to a given impulse, while a value closer to 1 will make it react less.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ImpulseYieldDuration",
        displayName = "Yield to impulse duration",
        helptext = "The duration of the spine yield following application of the shot impulse.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ImpulseBalanceErrorMagnitude",
        displayName = "Stagger on hit velocity",
        helptext = "Stagger velocity applicable at point of impact. Can be used to exaggerate response to the shot by making the character step. In m/s on the standard character.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ImpulseBalanceErrorDuration",
        displayName = "Stagger on hit duration.",
        helptext = "Duration of the stagger due to bullet impact. Will effect how long the character steps for after the impact.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "BalanceAssistance",
        displayName = "Balance assistance amount",
        helptext = "Level of auxilliary balance assistance. A means to increase the likelyhood of the character remaining standing despite the impact of a shot. Small values between 0 and 1.5 can be useful in extreme cases but higher values may not look natural.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ImpulseLegStrengthReduction",
        displayName = "Leg strength reduction on hit",
        helptext = "Amount between 0 and 1 to degrade the leg strength (balance and stepping capability) when the shot hits the particular leg.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ImpulseLegStrengthReductionDuration",
        displayName = "Leg strength reduction duration",
        helptext = "Duration of leg weakening due to shot.",
        input = true,
        array = false,
        type = "float",
      },
      
      {
        name = "ShotDeathBalanceErrorMagnitude",
        displayName = "Stagger on death velocity",
        helptext = "Stagger velocity applicable as death relax begins. Induces stepping, which can be useful to produce a good fall and avoid ragdoll-like crumpling.  In m/s on the standard character.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ShotDeathBalanceStrength",
        displayName = "Balance strength for death relax",
        helptext = "Overall body strength at start of death relax.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "ReachSku",
        displayName = "Reach mode",
        helptext = "Determines which arms reach for wound: 0 for left arm only, 1 for right arm only, 2 for both.",
        input = true,
        array = false,
        type = "int",
      },
      {
        name = "ReflexAnim",
        displayName = "Reflex Anim Id",
        helptext = "Id of the reflex animation applicable for this type of shot.",
        input = true,
        array = false,
        type = "int",
      },
      {
        name = "ReachSwitch",
        displayName = "Reach Reflex switch",
        helptext = "Enables or disables reach for wound reflex.",
        input = true,
        array = false,
        type = "bool",
      },
      {
        name = "ReflexAnimSwitch",
        displayName = "Reflex Anim switch",
        helptext = "Enables or disables animated reflex.",
        input = true,
        array = false,
        type = "bool",
      },
      {
        name = "ReflexLookSwitch",
        displayName = "Reflex Look Switch",
        helptext = "Enables or disables look at wound or shot source reflex.",
        input = true,
        array = false,
        type = "bool",
      },
      
      {
        name = "ForcedFalldownSwitch",
        displayName = "Forced fall down switch",
        helptext = "Enables or diables a forced fall down (to guarantee that the character falls over as part of it response to the shot).",
        input = true,
        array = false,
        type = "bool",
      },
      {
        name = "TargetTimeBeforeFalldown",
        displayName = "Target time before fall down",
        helptext = "Target amount of time before the character should fall down in response to the hit.",
        input = true,
        array = false,
        type = "float",
      },
      {
        name = "TargetNumberOfStepsBeforeFalldown",
        displayName = "Target step count before fall down",
        helptext = "Target number of steps the character should be allowed to take before falling down in response to the hit.",
        input = true,
        array = false,
        type = "int",
      },
      {
        name = "ImpulseDirWorldOrLocal",
        displayName = "Impulse direction fixed or follows character",
        helptext = "Direction of impulse determines direction of any induced stepping responses. When this flag is set direction is fixed relative to the world and character staggers uniformly. Otherwise, direction follows the part that was hit and stepping follows changes in orientation of the part. The former is more commonly used while the latter may be used to simulate loss of sense of equilibrium due to a head impact.",
        input = true,
        array = false,
        type = "bool",
      },  
      {
        name = "LookAtWoundOrShotSource",
        displayName = "Reflex look at wound or shot source",
        helptext = "If the look reflex is enabled this toggles the look target. If set the character looks at the wound, otherwise the character looks at the source of the shot.",
        input = true,
        array = true,
        type = "bool",
      },  
      {
        name = "DeathTrigger",
        displayName = "Killshot",
        helptext = "Enables or disables death response to the hit.",
        input = true,
        array = false,
        type = "bool",
      },
      {
        name = "DeathTriggerOnHeadshot",
        displayName = "Killshot for head hit",
        helptext = "Enables or disables death response for hits to the head.",
        input = true,
        array = false,
        type = "bool",
      },
    },
  presets = {
    Handgun = {
      Priority = 1,
      ReachDelay = 0,
      ReachDuration = 4,
      MaxReachSpeed = 20,
      ReflexAnimStart = 0,
      ReflexAnimRampDuration = 0.25,
      ReflexAnimFullDuration = 2,
      ReflexAnimDerampDuration = 0.75,
      ReflexAnimMaxWeight = 1.0,
      ReflexLookDuration = 4,
      DeathTriggerDelay = 1.5,
      DeathRelaxDuration = 1.5,
      ShotPartImpactMagnitude = 10,
      ShotBodyImpactMagnitude = 30,
      ShotTorqueMultiplier = 1,
      ShotLiftBoost = 1.0,
      ShotImpulseYield = 1.0,
      ImpulseYieldDuration = 0.4,
      ImpulseBalanceErrorMagnitude = 1.0,
      ImpulseBalanceErrorDuration = 0.4,
      BalanceAssistance = 0.0,
      ImpulseLegStrengthReduction = 0.5,
      ImpulseLegStrengthReductionDuration = 10,
      ShotDeathBalanceErrorMagnitude = 1.0,
      ShotDeathBalanceStrength = 0.0,
      ReachSku = 2,
      ReflexAnim = 0,
      ReachSwitch = true,
      ReflexAnimSwitch = true,
      ReflexLookSwitch = true,
      ForcedFalldownSwitch = false,
      TargetTimeBeforeFalldown = 100,
      TargetNumberOfStepsBeforeFalldown = 100,
      ImpulseDirWorldOrLocal = true,
      LookAtWoundOrShotSource = true,
      DeathTrigger = false,
      DeathTriggerOnHeadShot = false,
    },

    Shotgun =
    {
      Priority = 1,
      ReachDelay = 0,
      ReachDuration = 4,
      MaxReachSpeed = 20,
      ReflexAnimStart = 0,
      ReflexAnimRampDuration = 0.25,
      ReflexAnimFullDuration = 2,
      ReflexAnimDerampDuration = 0.75,
      ReflexAnimMaxWeight = 1.0,
      ReflexLookDuration = 4,
      DeathTriggerDelay = 1.5,
      DeathRelaxDuration = 1.5,
      ShotPartImpactMagnitude = 10,
      ShotBodyImpactMagnitude = 70,
      ShotTorqueMultiplier = 1,
      ShotLiftBoost = 1.0,
      ShotImpulseYield = 1.0,
      ImpulseYieldDuration = 0.4,
      ImpulseBalanceErrorMagnitude = 1.0,
      ImpulseBalanceErrorDuration = 0.4,
      BalanceAssistance = 0.0,
      ImpulseLegStrengthReduction = 0.75,
      ImpulseLegStrengthReductionDuration = 10,
      ShotDeathBalanceErrorMagnitude = 1.0,
      ShotDeathBalanceStrength = 0.0,
      ReachSku = 2,
      ReflexAnim = 0,
      ReachSwitch = true,
      ReflexAnimSwitch = true,
      ReflexLookSwitch = true,
      ForcedFalldownSwitch = false,
      TargetTimeBeforeFalldown = 100,
      TargetNumberOfStepsBeforeFalldown = 100,
      ImpulseDirWorldOrLocal = true,
      LookAtWoundOrShotSource = true,
      DeathTrigger = false,
      DeathTriggerOnHeadshot = false,
    },
  },
  
  --------------------------------------------------------------------------------------------------------------------
serialize = function(node, Stream)

  -- The function writeAttributesInOrder defined in ManifestUtils.lua, and writes the attributes in the exact order they
  -- are declared in the attribute table above. Because the runtime must interpret the binary structure sent from
  -- connect it is important to keep the runtime structure and the attributes declared in messages synchronized
  
  writeAttributesInOrder(node, Stream)
end,
--------------------------------------------------------------------------------------------------------------------

compareToPreset = function(node, preset)

  -- the mainShotAttributes is shared with the display function and are declared below

  for i, v in ipairs(mainShotAttributes) do
    local nodeValue = getAttribute(node, v)
    local presetValue = getAttribute(preset, v)
    if (nodeValue ~= presetValue) then
      return false
    end
  end
  return true
end,

--------------------------------------------------------------------------------------------------------------------

displayFunction = function(panel, object, helpPanel)
 
  local displayPanelAttribs = function(obj, displayPanel, attr)
    local label = utils.getDisplayString(getAttributeDisplayName(obj, attr))  
    local staticText = displayPanel:addStaticText{ name = string.format("%sText", attr), text = label}
    bindAttributeHelpToWidget(staticText, {object} , attr)
    local attrPaths = { }
    table.insert(attrPaths, string.format("%s.%s", obj, attr))
    local widget = displayPanel:addAttributeWidget
    {
      name = string.format("%sWidget", attr),
      attributes = attrPaths,
      flags = "expand",
      proportion = 1, 
    }
    bindAttributeHelpToWidget(widget, {object} , attr)
  end
  
  local displayAttribsRollup = function(obj, rollCnt, name, attr, expand, helptext)
    -- add a rollcontainer and get it's panel
    local rollup = rollCnt:addRollup{
      name = name,
      label = name,
      flags = "expand;mainSection",
    }
    bindHelpToWidget(rollup:getHeader(), helptext)
    local mainValuesPanel = rollup:getPanel()
    mainValuesPanel:beginFlexGridSizer{ cols = 2, flags = "expand", proportion = 1 }
    mainValuesPanel:setFlexGridColumnExpandable(2)

    -- display the attriubutes
    for i, attribute in ipairs(attr) do
      displayPanelAttribs(obj, mainValuesPanel, attribute)
    end
    mainValuesPanel:endSizer()
    rollup:expand(expand)
  end

  panel:beginVSizer{ flags = "expand" }
    local splitter = panel:addSplitter{ name = "Contents", flags = "expand", absolute = true, proportion = 1, sash = 60, dominant = 2 }
    local scrollPanel = splitter:addScrollPanel{flags = "expand;vertical", proportion = 1 }
    scrollPanel:beginVSizer{ flags = "expand" }
      local rollContainer = scrollPanel:addRollupContainer{ flags = "expand" }  
      
      -- display the shot message parameters grouped in categories
      displayAttribsRollup(object, rollContainer, "Hit prioritisation", priorityAttributes, true, "Determine a priority order on different hit messages.")
      displayAttribsRollup(object, rollContainer, "Impulse properties", impulseAttributes, true, "Bullet impact parameters.")
      displayAttribsRollup(object, rollContainer, "Reach for wound response", reachAttributes, true, "Reach-for-wound properties.")
      displayAttribsRollup(object, rollContainer, "Head look response", lookAttributes, true, "Head look properties.")
      displayAttribsRollup(object, rollContainer, "Stagger responses", stepAttributes, true, "Stagger velocites and durations.")
      displayAttribsRollup(object, rollContainer, "Reflex Animation", animAttributes, true, "Weight and duration of blended animation response.")
      displayAttribsRollup(object, rollContainer, "Limb strengths and balance", bodyStrengthAndBalanceAttributes, true, "Limb strength and yield to impact.")
      displayAttribsRollup(object, rollContainer, "Death shot", deathTriggerAttributes, true, "Death response parameters.")
      displayAttribsRollup(object, rollContainer, "Forced fall down",forcedFalldownAttributes , true, "Force fall down response parameters.")
      
      -- display the hit data (not expanded by default)
      displayAttribsRollup(object, rollContainer, "Hit Data", impactAttributes, false, "Details of ray-cast geometry captured by the shot and passed to runtime.")
     scrollPanel:endSizer()

    -- this panel contains the help text box
    helpWindow = splitter:addPanel{
      name = "HelpPanel",
      flags = "expand",
      proportion = 2
    }    
    helpWindow:beginVSizer{ flags = "expand", proportion = 1 }
      local helptext = helpWindow:addTextControl{ name = "TextBox", flags = "expand;noscroll", proportion = 1 }
      helptext:setReadOnly(true)
    helpWindow:endSizer()
  panel:endSizer()  
      
end,

--------------------------------------------------------------------------------------------------------------------

upgrade = function(node, version, pinLookupTable)

  if version < 2 then
    -- version 2: ImpulseTorqueMin/Max removed and ShotImpulseMagnitude renamed to ShotPartImpactMagnitude

    local value = getAttribute(node, "deprecated_ShotTorqueMin")
    removeAttribute(node, "deprecated_ShotTorqueMin")
    local value = getAttribute(node, "deprecated_ShotTorqueMax")
    removeAttribute(node, "deprecated_ShotTorqueMax")
    
    local value = getAttribute(node, "deprecated_ShotImpulseMagnitude")
    setAttribute(string.format("%s.ShotPartImpactMagnitude", node), value)
    removeAttribute(node, "deprecated_ShotImpulseMagnitude") 
    
  end
  
  if version < 3 then
    -- version 3: ImpulseLegStrengthReduction and dudration added
    -- set a non-zero leg strength reduction
    setAttribute(string.format("%s.ImpulseLegStrengthReduction", node), 0.3)
    -- set the duration equal to the duration of the balance error
    local value = getAttribute(node, "ImpulseBalanceErrorDuration")
    setAttribute(string.format("%s.ImpulseLegStrengthReductionDuration", node), value)
  end
  
  if version < 4 then
  
    -- set the balance assistance level
    setAttribute(string.format("%s.BalanceAssistance", node), 0.0)
    
  end
  
  if version < 5 then
    -- set the balance assistance level
    setAttribute(string.format("%s.ForcedFalldownSwitch", node), false)
    setAttribute(string.format("%s.TargetTimeBeforeFalldown", node), 100.0)
    setAttribute(string.format("%s.TargetNumberOfStepsBeforeFalldown", node), 100)
  end
  
end,
  }
)

------------------------------------------------------------------------------------------------------------------------
-- End of Shot message definition.
------------------------------------------------------------------------------------------------------------------------

mainShotAttributes =  {
  "Priority",
  
  "ReachSwitch",
  "ReachSku", 
  "ReachDelay",
  "ReachDuration",
  "MaxReachSpeed",
  
  "ReflexAnimSwitch",
  "ReflexAnim",
  "ReflexAnimStart",
  "ReflexAnimRampDuration",
  "ReflexAnimFullDuration",
  "ReflexAnimDerampDuration",
  "ReflexAnimMaxWeight",
  
  "ReflexLookSwitch",
  "LookAtWoundOrShotSource",
  "ReflexLookDuration", 
  
  "DeathTrigger", 
  "DeathTriggerOnHeadshot",
  "DeathTriggerDelay", 
  "DeathRelaxDuration",
  "ShotDeathBalanceErrorMagnitude",
  "ShotDeathBalanceStrength",
 
  "ImpulseDirWorldOrLocal",
  "ShotPartImpactMagnitude",
  "ShotBodyImpactMagnitude",
  "ShotTorqueMultiplier",
  "ShotLiftBoost",
 
  "ShotImpulseYield",
  "ImpulseYieldDuration",
  
  "ImpulseBalanceErrorMagnitude",
  "ImpulseBalanceErrorDuration",
  "BalanceAssistance",
  "ImpulseLegStrengthReduction",
  "ImpulseLegStrengthReductionDuration",
  
  "ForcedFalldownSwitch",
  "TargetTimeBeforeFalldown",
  "TargetNumberOfStepsBeforeFalldown"
  }
  
priorityAttributes =  {
  "Priority"
}
  
reachAttributes =  {
  "ReachSwitch",
  "ReachSku", 
  "ReachDelay",
  "ReachDuration",
  "MaxReachSpeed"
}
  
animAttributes = {
  "ReflexAnimSwitch",
  "ReflexAnim",
  "ReflexAnimStart",
  "ReflexAnimRampDuration",
  "ReflexAnimFullDuration",
  "ReflexAnimDerampDuration",
  "ReflexAnimMaxWeight"
}

lookAttributes = {
  "ReflexLookSwitch",
  "LookAtWoundOrShotSource",
  "ReflexLookDuration"
  }
  
deathTriggerAttributes = {
  "DeathTrigger",
  --"DeathTriggerOnHeadshot",   hide this parameter for 3.5.x to be removed entirely for subsequent releases
  "DeathTriggerDelay",
  "DeathRelaxDuration",
  "ShotDeathBalanceErrorMagnitude",
  "ShotDeathBalanceStrength"
 }
 
 stepAttributes = {
  "ImpulseDirWorldOrLocal",
  "ImpulseBalanceErrorMagnitude",
  "ImpulseBalanceErrorDuration",  
  }
  
 impulseAttributes = {  
  "ShotPartImpactMagnitude",
  "ShotBodyImpactMagnitude",
  "ShotTorqueMultiplier",
  "ShotLiftBoost"
  }
 
 
 bodyStrengthAndBalanceAttributes = {
  "ShotImpulseYield",
  "ImpulseYieldDuration",  
  "BalanceAssistance",
  "ImpulseLegStrengthReduction",
  "ImpulseLegStrengthReductionDuration"
  }
  
 forcedFalldownAttributes = {  
  "ForcedFalldownSwitch",
  "TargetTimeBeforeFalldown",
  "TargetNumberOfStepsBeforeFalldown"
  }

 impactAttributes = {
    "HitPointLocalX",  "HitPointLocalY", "HitPointLocalZ",
    "HitNormalLocalX", "HitNormalLocalY", "HitNormalLocalZ",
    "HitDirectionLocalX", "HitDirectionLocalY", "HitDirectionLocalZ",
    "HitDirectionWorldX", "HitDirectionWorldY", "HitDirectionWorldZ",
    "SourcePointWorldX", "SourcePointWorldY", "SourcePointWorldZ"
  }
  
------------------------------------------------------------------------------------------------------------------------
clearHelpText = function()
  if helpWindow then
    local helptext = helpWindow:getChild("TextBox")
    if helptext then
      helptext:setValue("")
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
setHelpText = function(string)
  if helpWindow then
    local helptext = helpWindow:getChild("TextBox")
    if helptext then
      if string then
        helptext:setValue(string)
      else
        helptext:setValue("")
      end
    end
  end
end

------------------------------------------------------------------------------------------------------------------------
bindHelpToWidget = function(widget, helpText)
   widget:setOnMouseEnter(
      function(self)
        setHelpText(helpText)
      end
    )
    widget:setOnMouseLeave(
      function(self)
        clearHelpText()
      end
    )
end

------------------------------------------------------------------------------------------------------------------------
bindAttributeHelpToWidget = function(widget, selection, attribute)
  local helpText = getAttributeHelpText(selection[1], attribute)
  bindHelpToWidget(widget, helpText)
end
