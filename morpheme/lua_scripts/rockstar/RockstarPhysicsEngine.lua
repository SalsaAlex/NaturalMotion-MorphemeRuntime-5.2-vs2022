------------------------------------------------------------------------------------------------------------------------
-- Functions for exporting a rockstar ragdoll and additional euphoria data.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- include other required scripts
------------------------------------------------------------------------------------------------------------------------
require "rockstar/RockstarUtilities.lua"

------------------------------------------------------------------------------------------------------------------------
--| signature: CustomPhysicsEngineDataNode createRockstarPhysicsData(Scene scene, Node parent)
------------------------------------------------------------------------------------------------------------------------
local createRockstarPhysicsEngineDataNode = function(scene, parent)
  local requiredClassTypeId = nmx.CustomPhysicsEngineDataNode.ClassTypeId()

  -- make sure a CustomPhysicsEngineDataNode for this PhysicsEngine doesn't already exist.
  --
  local rockstarData = parent:getFirstChild(requiredClassTypeId)
  while rockstarData do
    if rockstarData:is(requiredClassTypeId) and
       rockstarData:getPhysicsEngineName() == "Rockstar" then
      return rockstarData
    end

    rockstarData = rockstarData:getNextSibling()
  end

  rockstarData = scene:createNode(requiredClassTypeId, "RockstarProperties", parent)

  local physicsEngineNameAttr = rockstarData:findAttribute("PhysicsEngineName")
  physicsEngineNameAttr:setString("Rockstar")
  physicsEngineNameAttr:setEditable(false)

  return rockstarData
end

------------------------------------------------------------------------------------------------------------------------
--| signature: boolean createRockstarProperty(CustomPhysicsEngineDataNode rockstarPhysicsData, string propertyName, number propertyValue)
--| signature: boolean createRockstarProperty(CustomPhysicsEngineDataNode rockstarPhysicsData, string propertyName, boolean propertyValue)
------------------------------------------------------------------------------------------------------------------------
local createRockstarProperty = function(rockstarPhysicsData, propertyName, propertyValue)
  -- assert all arguments are valid
  --
  assert(type(rockstarPhysicsData) == "userdata", "bad argument #1 to 'createRockstarProperty'")
  assert(rockstarPhysicsData:is(nmx.CustomPhysicsEngineDataNode.ClassTypeId()), "bad argument #1 to 'createRockstarProperty'")
  assert(rockstarPhysicsData:getPhysicsEngineName() == "Rockstar", "bad argument #1 to 'createRockstarProperty'")
  assert(type(propertyName) == "string", "bad argument #2 to 'createRockstarProperty'")
  assert(propertyName:len() > 0, "bad argument #2 to 'createRockstarProperty'")

  local attribute = rockstarPhysicsData:findAttribute(propertyName)

  local propertyType = type(propertyValue)
  if propertyType == "number" then
    if attribute and attribute:isValid() then
      local attributeTypeId = attribute:getTypeId()
      if attributeTypeId == nmx.AttributeTypeId.kInt or
         attributeTypeId == nmx.AttributeTypeId.kFloat then
        return true
      else
        -- attribute already exists but has the wrong type
        --
        return false
      end
    end

    local options = nmx.FloatOptions.new()
    options:defaultValue(propertyValue)
    rockstarPhysicsData:addFloatAttribute(propertyName, options)

    return true
  elseif propertyType == "boolean" then
    if attribute and attribute:isValid() then
      local attributeTypeId = attribute:getTypeId()
      if attributeTypeId == nmx.AttributeTypeId.kBool then
        return true
      else
        -- attribute already exists but has the wrong type
        --
        return false
      end
    end

    local options = nmx.BoolOptions.new()
    options:defaultValue(propertyValue)
    rockstarPhysicsData:addBoolAttribute(propertyName, options)

    return true
  elseif propertyType == "table" then
    if attribute and attribute:isValid() then
      local attributeTypeId = attribute:getTypeId()
      if attributeTypeId == nmx.AttributeTypeId.kEnum then
        return true
      else
        -- attribute already exists but has the wrong type
        --
        return false
      end
    end

    local options = nmx.EnumOptions.new()
    options:defaultValue(1)
    for i = 1, table.getn(propertyValue) do
      options:choice(i, propertyValue[i])
    end
    rockstarPhysicsData:addEnumAttribute(propertyName, options)

    return true
  else
    assert(false, "bad argument #3 to 'createRockstarProperty'")
  end

  return false
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil createPhysicsJointData(Scene scene, PhysicsJointNode physicsJoint)
------------------------------------------------------------------------------------------------------------------------
local createPhysicsJointData = function(scene, physicsJoint)
  -- nothing to do for physics joints
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil createPhysicsBodyData(Scene scene, PhysicsBodyNode physicsBody)
------------------------------------------------------------------------------------------------------------------------
local createPhysicsBodyData = function(scene, physicsBody)
  local rockstarData = createRockstarPhysicsEngineDataNode(scene, physicsBody)

  createRockstarProperty(rockstarData, "PartBouyancyMultiplier", 1.0)
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil createPhysicsVolumeData(Scene scene, PhysicsVolumeNode physicsJoint)
------------------------------------------------------------------------------------------------------------------------
local createPhysicsVolumeData = function(scene, physicsVolume)
  local rockstarData = createRockstarPhysicsEngineDataNode(scene, physicsVolume)
  
  createRockstarProperty(rockstarData, "Mass", 1.0)
end

------------------------------------------------------------------------------------------------------------------------
--| signature: nil createPhysicsJointLimitData(Scene scene, PhysicsJointLimitNode physicsJointLimit)
------------------------------------------------------------------------------------------------------------------------
local createPhysicsJointLimitData = function(scene, physicsJointLimit)
  if physicsJointLimit:is(nmx.PhysicsTwistSwingNode.ClassTypeId()) then
    local rockstarData = createRockstarPhysicsEngineDataNode(scene, physicsJointLimit)

    -- add some rockstar properties
    --
    createRockstarProperty(rockstarData, "EffectorName", rockstar.euphoriaEffectorNames)

    createRockstarProperty(rockstarData, "LimitEnabled", true)
    createRockstarProperty(rockstarData, "CreateEffector", true)

    createRockstarProperty(rockstarData, "ReverseFirstLeanMotor", false)
    createRockstarProperty(rockstarData, "ReverseSecondLeanMotor", false)
    createRockstarProperty(rockstarData, "ReverseTwistMotor", false)

    createRockstarProperty(rockstarData, "SoftLimitFirstLeanMultiplier", 1.0)
    createRockstarProperty(rockstarData, "SoftLimitSecondLeanMultiplier", 1.0)
    createRockstarProperty(rockstarData, "SoftLimitTwistMultiplier", 1.0)

    createRockstarProperty(rockstarData, "DefaultLeanForceCap", 300)
    createRockstarProperty(rockstarData, "DefaultTwistForceCap", 300)

    createRockstarProperty(rockstarData, "DefaultMuscleStiffness", 0.5)
    createRockstarProperty(rockstarData, "DefaultMuscleStrength", 100)
    createRockstarProperty(rockstarData, "DefaultMuscleDamping", 20)
  elseif physicsJointLimit:is(nmx.PhysicsHingeNode.ClassTypeId()) then
    local rockstarData = createRockstarPhysicsEngineDataNode(scene, physicsJointLimit)

    -- add some rockstar properties
    --
    createRockstarProperty(rockstarData, "EffectorName", rockstar.euphoriaEffectorNames)

    createRockstarProperty(rockstarData, "DefaultLeanForceCap", 300)

    createRockstarProperty(rockstarData, "DefaultMuscleStiffness", 0.5)
    createRockstarProperty(rockstarData, "DefaultMuscleStrength", 100)
    createRockstarProperty(rockstarData, "DefaultMuscleDamping", 20)
  end
end

-- now register the rockstar physics engine
--
local rockstar = {
  supportedPhysicsJointLimitTypeNames = {
    "PhysicsTwistSwingNode",
    "PhysicsHingeNode",
  },
  createPhysicsJointData = createPhysicsJointData,
  createPhysicsBodyData = createPhysicsBodyData,
  createPhysicsVolumeData = createPhysicsVolumeData,
  createPhysicsJointLimitData = createPhysicsJointLimitData,
}

if getPhysicsEngine("Rockstar") then
  unregisterPhysicsEngine("Rockstar")
end
registerPhysicsEngine("Rockstar", rockstar)