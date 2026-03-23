-- Natural Motion Sample Character Controller script functions
EVENTS["mcCharacterControllerExport"] = { }

local function onControllerExport(target, controllerPath, shapePath, sceneIndex)
  --
  -- Default controller export
  --
  local function defaultControllerExport(target)
    target:setStringAttribute("Shape", "Capsule")
    target:setDoubleAttribute("Height", 1.04749)
    target:setDoubleAttribute("Radius", 0.196)
    target:setDoubleAttribute("SkinWidth", 0.01)
    target:setVector3Attribute("Colour",  {x = 0.5, y = 0.5, z = 1.0})
    target:setBoolAttribute("Visible", true)

    target:setDoubleAttribute("StepHeight", 0.25)
    target:setDoubleAttribute("MaxPushForce", 0.0)
    target:setDoubleAttribute("MaxSlopeAngle", 45.0)
  end

  local nmx_app = nmx.Application.new()
  local scene = nmx_app:getScene(sceneIndex)
  
  if scene == nil then
    app.warning("Invalid scene, couldn't export character controller.  A default character controller has been exported instead.")
    defaultControllerExport(target)
    return
  end
  
  local controllerData = scene:getNodeFromPath(controllerPath)
  local shapeData = scene:getNodeFromPath(shapePath)
  
  if controllerData == nil or shapeData == nil then
    app.warning("Invalid controller data, couldn't export character controller.  A default character controller has been exported instead.")
    defaultControllerExport(target)
    return
  end
  
  -- Capsule
  --
  if(shapeData:is("CapsuleNode"))
  then
    target:setStringAttribute("Shape", "Capsule")
    target:setDoubleAttribute("Height", shapeData:getHeight())
    target:setDoubleAttribute("Radius", shapeData:getRadius())
    target:setVector3Attribute("Colour",  {x = 0.5, y = 0.5, z = 1.0})
    target:setBoolAttribute("Visible", true)
    
    target:setDoubleAttribute("SkinWidth", controllerData:findAttribute("SkinWidth"):asFloat())
    target:setDoubleAttribute("MaxPushForce", controllerData:findAttribute("MaxPushForce"):asFloat())
    target:setDoubleAttribute("MaxSlopeAngle", controllerData:findAttribute("MaxSlopeAngle"):asFloat())
  
  -- Box
  --
  elseif( shapeData:is("BoxNode"))
  then
    target:setStringAttribute("Shape", "Box")
    target:setDoubleAttribute("Height", shapeData:getHeight())
    target:setDoubleAttribute("Width", shapeData:getWidth())
    target:setDoubleAttribute("Depth", shapeData:getDepth())
    target:setVector3Attribute("Colour",  {x = 0.5, y = 0.5, z = 1.0})
    target:setBoolAttribute("Visible", true)
    
    target:setDoubleAttribute("SkinWidth", controllerData:findAttribute("SkinWidth"):asFloat())
    target:setDoubleAttribute("MaxPushForce", controllerData:findAttribute("MaxPushForce"):asFloat())
    target:setDoubleAttribute("MaxSlopeAngle", controllerData:findAttribute("MaxSlopeAngle"):asFloat())

  else
    
    app.warning("Character controller is not a supported shape. A default character controller has been exported instead.")
    defaultControllerExport(target)
    return
  end
  
  if not mcn.isPhysicsDisabled() then
    local physicsEngine = preferences.get("PhysicsEngine")
    local exporter = getPhysicsDriverExporter(physicsEngine)
    if type(exporter.exportController) == "function" then
      exporter.exportController(target, controllerData)
    end
  end

end

-- register the function to handle the controller export event
registerEventHandler("mcCharacterControllerExport", onControllerExport)