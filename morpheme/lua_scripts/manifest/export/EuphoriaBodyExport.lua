require "manifest/ExportUtils.lua"
require "NMX2/Utils.lua"
require "luaApi/IKCalculationAPI.lua"

-- Natural Motion Sample Euphoria Body Exporter script functions
EVENTS["mcBodyExport"] = { }

--target:getLimbs() 										: returns list of limbdefexport
--target:createLimb() 										: arg1 is of type string. returns a limbdefexport
--target:setRootLimb(arg1) 									: arg1 is of type string.
--target:createSelfAvoidanceExport() 						: returns a selfavoidanceshapedef
--target:createAnimationPose(arg1) 							: arg1 is of type string. returns a animationposedef
			
--limbdefexport:

	--limbdefexport:getName() 								: returns string
	--limbdefexport:getCoupledLimit() 						: returns a coupledlimitdef (aka HamstringExport)
	--limbdefexport:setRootPart(arg1) 						: arg1 is of type string. should be the name of a physics part
	--limbdefexport:setEndPart(arg1) 						: arg1 is of type string. should be the name of a physics part
	--limbdefexport:setBasePart(arg1) 						: arg1 is of type string. should be the name of a physics part
	--limbdefexport:setType(arg1) 							: arg1 is of type string. types: "Leg" "Arm" "Head" "Spine"
	--limbdefexport:setReachDir(arg1)						: arg1 is of type vector3.
	--limbdefexport:setReachAngleX(arg1)					: arg1 is of type float.		(replaced by ReachCone in 5.2 ?)		
	--limbdefexport:setReachAngleY(arg1)					: arg1 is of type float.		(replaced by ReachCone in 5.2 ?)		
	--limbdefexport:setReachDistance(arg1)					: arg1 is of type float.
	--limbdefexport:setReachOffset(arg1)					: arg1 is of type matrix34.
	--limbdefexport:setEndOffset(arg1)						: arg1 is of type matrix34.
	--limbdefexport:setRootOffset(arg1)						: arg1 is of type matrix34.
	--limbdefexport:setExtraParts(arg1)						: arg1 is of type string (?) table.
	--limbdefexport:setNeutralPoseWeight(arg1)				: arg1 is of type string.
	--limbdefexport:setGuidePoseWeight(arg1)				: arg1 is of type float.
	--limbdefexport:setGuidePoseJoints(arg1)				: arg1 is of type bool (?) table. table size is number of joints in the limb chain + one
	--limbdefexport:setPositionWeights(arg1)				: arg1 is of type float (?) table. table size is number of joints in the limb chain + one
	--limbdefexport:setOrientationWeights(arg1)				: arg1 is of type float (?) table. table size is number of joints in the limb chain + one
		
	--limbdefexport:setDoubleAttribute(arg1, arg2)			: arg1 is of type string; arg2 is of type double.
	--limbdefexport:setIntAttribute(arg1, arg2)				: arg1 is of type string; arg2 is of type double.
	--limbdefexport:setUIntAttribute(arg1, arg2)			: arg1 is of type string; arg2 is of type double.
	--limbdefexport:setStringAttribute(arg1, arg2)			: arg1 is of type string; arg2 is of type double.
	--limbdefexport:setBoolAttribute(arg1, arg2)			: arg1 is of type string; arg2 is of type double.
	--limbdefexport:setVector3Attribute(arg1, arg2)			: arg1 is of type string; arg2 is of type double.
	--limbdefexport:setMatrix34Attribute(arg1, arg2)		: arg1 is of type string; arg2 is of type double.
		
	--limbdefexport:getDoubleAttribute(arg1)				: arg1 is of type string. return type is double.
	--limbdefexport:getIntAttribute(arg1)					: arg1 is of type string. return type is int.
	--limbdefexport:getUIntAttribute(arg1)					: arg1 is of type string. return type is uint.
	--limbdefexport:getStringAttribute(arg1)				: arg1 is of type string. return type is string.
	--limbdefexport:getBoolAttribute(arg1)					: arg1 is of type string. return type is bool.
	--limbdefexport:getVector3Attribute(arg1)				: arg1 is of type string. return type is vector3.
	--limbdefexport:getMatrix34Attribute(arg1)				: arg1 is of type string. return type is matrix34.
	
--coupledlimit:	
	--coupledlimitdef:setDistance(arg1)						: arg1 is of type float.
	--coupledlimitdef:setStiffness(arg1)					: arg1 is of type float.
	--coupledlimitdef:setEnabled(arg1)						: arg1 is of type bool.
	--coupledlimitdef:setTwistBendScaleWeights(arg1)		: arg1 is of type float (?) table.
	--coupledlimitdef:setSwing1BendScaleWeights(arg1)		: arg1 is of type float (?) table.
	--coupledlimitdef:setSwing2BendScaleWeights(arg1)		: arg1 is of type float (?) table.
	
	--coupledlimitdef:setDoubleAttribute(arg1, arg2)		: arg1 is of type string; arg2 is of type double.
	--coupledlimitdef:setIntAttribute(arg1, arg2)			: arg1 is of type string; arg2 is of type double.
	--coupledlimitdef:setUIntAttribute(arg1, arg2)			: arg1 is of type string; arg2 is of type double.
	--coupledlimitdef:setStringAttribute(arg1, arg2)		: arg1 is of type string; arg2 is of type double.
	--coupledlimitdef:setBoolAttribute(arg1, arg2)			: arg1 is of type string; arg2 is of type double.
	--coupledlimitdef:setVector3Attribute(arg1, arg2)		: arg1 is of type string; arg2 is of type double.
	--coupledlimitdef:setMatrix34Attribute(arg1, arg2)		: arg1 is of type string; arg2 is of type double.
	
	--coupledlimitdef:getDoubleAttribute(arg1)				: arg1 is of type string. return type is double.
	--coupledlimitdef:getIntAttribute(arg1)					: arg1 is of type string. return type is int.
	--coupledlimitdef:getUIntAttribute(arg1)				: arg1 is of type string. return type is uint.
	--coupledlimitdef:getStringAttribute(arg1)				: arg1 is of type string. return type is string.
	--coupledlimitdef:getBoolAttribute(arg1)				: arg1 is of type string. return type is bool.
	--coupledlimitdef:getVector3Attribute(arg1)				: arg1 is of type string. return type is vector3.
	--coupledlimitdef:getMatrix34Attribute(arg1)			: arg1 is of type string. return type is matrix34.



--selfavoidanceshapedef:
	--selfavoidanceshapedef:setRootRadiusMultiplier(arg1)	: arg1 is of type float.
	--selfavoidanceshapedef:setEndExtension(arg1)			: arg1 is of type float.
	--selfavoidanceshapedef:setEndFrontRadius(arg1)			: arg1 is of type float.
	--selfavoidanceshapedef:setEndSideRadius(arg1)			: arg1 is of type float.
	--selfavoidanceshapedef:getRootRadiusMultiplier()		: return type is of type float
	--selfavoidanceshapedef:getEndExtension()				: return type is of type float
	--selfavoidanceshapedef:getEndFrontRadius()				: return type is of type float
	--selfavoidanceshapedef:getEndSideRadius()				: return type is of type float


--animationposedef:
	--animationposedef:setName(arg1)						: arg1 is of type string.
	--animationposedef:setAnimationFile(arg1)				: arg1 is of type string.
	--animationposedef:setAnimationTake(arg1)				: arg1 is of type string.
	--animationposedef:setPoseFrameIndex(arg1)				: arg1 is of type number
	--animationposedef:getName()							: return type is string.
	--animationposedef:getAnimationFile()					: return type is string.
	--animationposedef:getAnimationTake()					: return type is string.
	--animationposedef:getPoseFrameIndex()					: return type is number

local function onBodyExport(target, physicsRigDataSceneRootPath, physicsRigSceneRootPath)
  --
  -- Default body export. this is where we will just write info about the body's limbs. i think.
  --
  
	local limbs = target:getLimbs()
	
	app.warning("exporting bodydef")
	
	app.warning("physicsRigDataSceneRootPath" .. physicsRigDataSceneRootPath)
	app.warning("physicsRigSceneRootPath" .. physicsRigSceneRootPath)
	
	local nmx_app = nmx.Application.new()
	local scene = nmx_app:getScene(0)

	local physxrigdata = scene:getNodeFromPath(physicsRigDataSceneRootPath)
	local physicsRigRootTxInstance = scene:getNodeFromPath(physicsRigSceneRootPath)
	
	local limbGroups =  physxrigdata:findChild("PhysicsLimbGroups", true)
	
	local errorsReturn = { errors = { }, warnings = { } }
	
	local jointtransforms = getPhysicsRigJointsForExport(physicsRigRootTxInstance, false, errorsReturn)
	
	preparePhysicsRigForExport(scene, jointtransforms)
	
    local inverseRootMatrix = physicsRigRootTxInstance:getWorldMatrix()
	inverseRootMatrix:invert()
	
	app.warning("inverserootmatrix")
	for key, value in pairs(nmx.matrixToTable(inverseRootMatrix)) do
		app.warning("," .. value)
	end
	
	--for k, v in pairs(jointtransforms) do
	--	local transform = nmx.Matrix.new(v:getWorldMatrix())
    --    --transform:multiply(inverseRootMatrix)
    --    transform = nmx.matrixToTable(transform)
	--	
	--	app.warning("part transform")
	--	for key, value in pairs(transform) do
	--		app.warning("," .. value)
	--	end
	--end
	
	local limbTypeMap = {}
	limbTypeMap[0] = "arm"
	limbTypeMap[1] = "leg"
	limbTypeMap[2] = "head"
	limbTypeMap[3] = "extra"
	limbTypeMap[4] = "spine"
	
	local numheads = 0
	local numarms = 0
	local numlegs = 0
	local numspines = 0
	
	local curObj = limbGroups:getFirstChild()
	while curObj ~= nil do
		local limbtype = "spine"
		if curObj:is("ExtremityNode") then
			limbtype =  limbTypeMap[curObj:findAttribute("Type"):asInt()]
		end
		
		for k, v in pairs(getmetatable(curObj).__index) do
			app.warning("method: " .. tostring(k))
		end
		
		local name = "Spine_" .. numspines
		if limbtype == "arm" then
			name = "Arm_" .. numarms
			numarms = numarms + 1
		elseif limbtype == "leg" then
			name = "Leg_" .. numlegs
			numlegs = numlegs + 1
		elseif limbtype == "head" then
			name = "Head_" .. numheads
			numheads = numheads + 1
		else
			numspines = numspines + 1
		end
		
		local limbdef = target:createLimb(name)
		
		if limbtype == "arm" then
			app.warning("arm limb")
			limbdef:setType("Arm")
		elseif limbtype == "leg" then
			app.warning("leg limb")
			limbdef:setType("Leg")
		elseif limbtype == "head" then
			app.warning("head limb")
			limbdef:setType("Head")
		else
			app.warning("spine limb")
			limbdef:setType("Spine")
			target:setRootLimb(name)
		end
		
		local basepart = curObj:getLimbBase()
		local endpart = curObj:getLimbEnd()
		local rootpart = curObj:getLimbRoot()
		
		local endjointtransform = nil
		local basejointtransform = nil
		local rootjointtransform = nil
		
		for k, v in pairs(jointtransforms) do
			if v:getName() == endpart:getName() then
				endjointtransform = v
				end
			if v:getName() == basepart:getName() then
				basejointtransform = v
				end
			if v:getName() == rootpart:getName() then
				rootjointtransform = v
			end
			
			if endjointtransform ~= nil and rootjointtransform ~= nil and basejointtransform ~= nil then
				break
			end
		end
		
		app.warning( "basepart : " ..  basepart:getName() )
		app.warning( "endpart : " .. endpart:getName() )
		app.warning( "rootpart : " .. rootpart:getName() )
		
		limbdef:setBasePart( basepart:getName() )
		limbdef:setEndPart( endpart:getName() )
		limbdef:setRootPart( rootpart:getName() )

		if limbtype == "spine" then
			limbdef:setStringAttribute( "midPartName", "_NULL" )
		elseif limbtype == "head" then
			limbdef:setStringAttribute( "midPartName", "_NULL" )
		else
			limbdef:setStringAttribute( "midPartName", endpart:getParent():getName() ) --FIX PLS
		end
		
		local arraytotable = function(array)
			local t = {}
			local n = array:size()
			
			for i = 1, n do
				t[i] = array:at(i)
			end
			
			return t
		end
		
		local r0 = nmx.Vector3.new( 0.000000, 0.000000, 1.000000 )
		local r1 = nmx.Vector3.new( 0.000000, 1.000000, 0.000000 )
		local r2 = nmx.Vector3.new( -1.000000, 0.000000, 0.000000 )
		local r3 = nmx.Vector3.new( 0.000000, 0.000000, 0.000000 )
				
		local dummyoffset = nmx.Matrix.new(r0, r1, r2, r3)
		
		limbdef:setRootOffset(nmx.matrixToTable(dummyoffset))
		
		local reachnode = curObj:getLimbReachNode()
		
		limbdef:setReachAngleX(reachnode:getDataNode():findAttribute("AngleX"):asFloat() )
		limbdef:setReachAngleY(reachnode:getDataNode():findAttribute("AngleY"):asFloat() )
		limbdef:setDoubleAttribute( "reachConeAngle", reachnode:getDataNode():findAttribute("AngleY"):asDouble() )
		limbdef:setReachDistance(reachnode:getDataNode():findAttribute("ReachDistance"):asFloat() )
		limbdef:setReachOffset( nmx.matrixToTable(reachnode:getParentSgTransform():getTransformNode():getLocalMatrix()) )
		limbdef:setReachDir( nmx.vector3ToTable(reachnode:getParentSgTransform():getTransformNode():getLocalMatrix():getColumn(0)) )
		
		local locatornode = curObj:getLimbEndLocator():getParentSgTransform():getTransformNode()
		
		local locatoroffset = nmx.Vector3.new(locatornode:getTranslation())
		
		local endoffset = nmx.Matrix.new(locatornode:getLocalMatrix())
		dummyoffset:multiply(dummyoffset)
		
		limbdef:setEndOffset(nmx.matrixToTable(endoffset))
	
		limbdef:setGuidePoseWeight( curObj:findAttribute("GuidePoseWeight"):asFloat() )
		limbdef:setNeutralPoseWeight( curObj:findAttribute("GuidePoseWeight"):asFloat() )
		limbdef:setGuidePoseJoints( arraytotable( curObj:findAttribute("GuidePoseJoints"):asBoolArray() ) )
		limbdef:setPositionWeights( arraytotable( curObj:findAttribute("PositionWeights"):asFloatArray() ) )
		limbdef:setOrientationWeights( arraytotable( curObj:findAttribute("OrientationWeights"):asFloatArray() ) )
		
		local hamstring = limbdef:getCoupledLimit()
		
		if hamstring ~= nil then
			local hamstringenabled = curObj:findAttribute("CoupledLimitEnabled"):asBool()
			hamstring:setEnabled( hamstringenabled )
			
			if hamstringenabled then
				hamstring:setDistance( curObj:findAttribute("CoupledLimitDistance"):asFloat() )
				hamstring:setStiffness( curObj:findAttribute("CoupledLimitStiffness"):asFloat() )
				hamstring:setTwistBendScaleWeights( arraytotable( curObj:findAttribute("CoupledLimitTwistBendScale"):asFloatArray() ) )
				hamstring:setSwing1BendScaleWeights( arraytotable( curObj:findAttribute("CoupledLimitSwing1BendScale"):asFloatArray() ) )
				hamstring:setSwing2BendScaleWeights( arraytotable( curObj:findAttribute("CoupledLimitSwing2BendScale"):asFloatArray() ) )
				end
				
			end
		
		curObj = curObj:getNextSibling()
	end
	
	local animpose1 = target:createAnimationPose("DefaultPose")
	
	local rigpose1 = anim.getRigPose(getSelectedAssetManagerAnimSet(), "DefaultPose")
	
	animpose1:setAnimationFile(rigpose1.filename)
	animpose1:setAnimationTake(rigpose1.takename)
	animpose1:setPoseFrameIndex(0)
	
	local animpose2 = target:createAnimationPose("GuidePose")
	
	local rigpose2 = anim.getRigPose(getSelectedAssetManagerAnimSet(), "GuidePose")
	
	animpose2:setAnimationFile(rigpose2.filename)
	animpose2:setAnimationTake(rigpose2.takename)
	animpose2:setPoseFrameIndex(0)
	
	

	--for i, limb in ipairs(limbs) do
	--	app.warning("limb", limb:getName())
	--	end
  
	return 1

end

-- register the function to handle the controller export event
registerEventHandler("mcBodyExport", onBodyExport)