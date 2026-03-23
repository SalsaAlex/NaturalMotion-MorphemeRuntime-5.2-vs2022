require "previewScripts/Viewport.lua"
------------------------------------------------------------------------------------------------------------------------
--
------------------------------------------------------------------------------------------------------------------------

viewport.debugDraw = {}
viewport.debugDraw._items = {}

------------------------------------------------------------------------------------------------------------------------
-- Debug Draw Functions
------------------------------------------------------------------------------------------------------------------------
viewport.debugDraw.update = function()
  local transforms = getWorldSpaceTransforms()
  local animSet = getAnimSet()
  
  for _,line in ipairs(viewport.debugDraw._items.lines) do
    if line.animSet == animSet and isNodeActive(line.owner) then
      local lineStart = {}
      local lineEnd = {}
      if (line.bone == -1) then
        -- this is a line in world space
        lineStart = line.offset
        lineEnd = vecAdd(line.offset, line.vector)
      else
        local offset = line.offset
        -- Instead of specifying an offset we can specify another bone to be the location where the
        -- line should start
        if type(line.offset) == "number" then
          local baseTM = transforms[line.bone]
          local refTM = transforms[line.offset]
          local baseInvQuat = { x=-baseTM.quat[1], y=-baseTM.quat[2], z=-baseTM.quat[3], w=baseTM.quat[4] }
          local refRelativePos = { x=(refTM.pos[1] - baseTM.pos[1]), y=(refTM.pos[2]-baseTM.pos[2]), z=(refTM.pos[3]-baseTM.pos[3]) }
          offset = vecRotate(refRelativePos, baseInvQuat)
        end
        -- transform the start and end points into local space
        local t = transforms[line.bone]
        local pos = {x=t.pos[1], y=t.pos[2], z=t.pos[3]}
        local quat = {x=t.quat[1], y=t.quat[2], z=t.quat[3], w=t.quat[4]}
        lineStart = vecTransform(offset, pos, quat)
        lineEnd = vecAdd(lineStart, vecRotate(line.vector, quat))
      end
      
      addDebugLine(lineStart, lineEnd, line.colour)
    end
  end
  
  for _,point in ipairs(viewport.debugDraw._items.points) do
    if point.animSet == animSet and isNodeActive(point.owner) then
      local pt = {} -- to store the point in world space
      
      if (point.bone == -1) then
        -- this is a point in world space
        pt = point.offset
      else
        -- transform the point into local space
        local t = transforms[point.bone]
        local pos = { x=t.pos[1], y=t.pos[2], z=t.pos[3] }
        local quat = { x=t.quat[1], y=t.quat[2], z=t.quat[3], w=t.quat[4] }
        pt = vecTransform(point.offset, pos, quat)

      end

      local sz = point.scale
      local startx = { x = pt.x-sz, y = pt.y, z = pt.z }
      local endx =   { x = pt.x+sz, y = pt.y, z = pt.z }
      local starty = { x = pt.x, y = pt.y-sz, z = pt.z }
      local endy =   { x = pt.x, y = pt.y+sz, z = pt.z }
      local startz = { x = pt.x, y = pt.y, z = pt.z-sz }
      local endz =   { x = pt.x, y = pt.y, z = pt.z+sz }
                            
      addDebugLine( startx, endx, point.colour )
      addDebugLine( starty, endy, point.colour )
      addDebugLine( startz, endz, point.colour )
    end
  end
end

-------------------------
-- Maths
-------------------------
local pi = 3.14159265

------------------------------------------------------------------------------------------------------------------------
-- Vector Maths functions
------------------------------------------------------------------------------------------------------------------------
vecAdd = function(v1, v2)
  return {x = v1.x+v2.x, y = v1.y+v2.y, z = v1.z+v2.z}
end

------------------------------------------------------------------------------------------------------------------------
vecSubtract = function(v1, v2)
  return {x = v1.x-v2.x, y = v1.y-v2.y, z = v1.z-v2.z}
end

------------------------------------------------------------------------------------------------------------------------
vecDot = function(v1, v2)
  return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z
end

------------------------------------------------------------------------------------------------------------------------
vecCross = function(v1, v2)
  return {x = v1.y*v2.z-v1.z*v2.y, y = v1.z*v2.x-v1.x*v2.z, z = v1.x*v2.y-v1.y*v2.x}
end

------------------------------------------------------------------------------------------------------------------------
vecMagnitude = function(v)
  return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
end

------------------------------------------------------------------------------------------------------------------------
vecScale = function(v, k)
  return {x = v.x*k, y = v.y*k, z = v.z*k}
end

------------------------------------------------------------------------------------------------------------------------
vecNormalise = function(v)
  local mag = vecMagnitude(v)
  if (mag > 0) then
    return vecScale(v, 1/mag)
  else
    return {x = v.x, y = v.y, z = v.z}
  end
end

------------------------------------------------------------------------------------------------------------------------
vecTransform = function(v, p, q)
  return vecAdd(vecRotate(v, q), p)
end

------------------------------------------------------------------------------------------------------------------------
vecRotate = function(v, q)
  local result
  local qv = {x=q.x, y=q.y, z=q.z}
  
  result = vecCross(qv,v)
  result = vecScale(result, q.w*2)
  
  result = vecAdd(result, vecScale(v, 2 * (q.w*q.w) - 1))
  result = vecAdd(result, vecScale(qv, vecDot(qv,v) * 2))

  return result;
end

------------------------------------------------------------------------------------------------------------------------
-- Quaternion maths functions
------------------------------------------------------------------------------------------------------------------------
quatMultiply = function(q1, q2)
  return { x = q2.w*q1.x + q2.x*q1.w + q1.y*q2.z - q1.z*q2.y,
           y = q2.w*q1.y + q2.y*q1.w + q1.z*q2.x - q1.x*q2.z,
           z = q2.w*q1.z + q2.z*q1.w + q1.x*q2.y - q1.y*q2.x,
           w = q2.w*q1.w - q2.x*q1.x - q2.y*q1.y - q2.z*q1.z }
end

------------------------------------------------------------------------------------------------------------------------
quatFromEulerXYZ = function(eulerX, eulerY, eulerZ)
  local chx = math.cos(eulerX / 2)
  local chy = math.cos(eulerY / 2)
  local chz = math.cos(eulerZ / 2)
  local shx = math.sin(eulerX / 2)
  local shy = math.sin(eulerY / 2)
  local shz = math.sin(eulerZ / 2)
  return { x = shy*shz*chx + chy*chz*shx,
           y = shy*chz*chx + chy*shz*shx,
           z = chy*shz*chx - shy*chz*shx,
           w = chx*chy*chz - shx*shy*shz }
end

------------------------------------------------------------------------------------------------------------------------
quatToR = function(q)
  local q0t2 = q.w*2
  local q1t2 = q.x*2
  local q0sq = q.w*q.w
  local q1sq = q.x*q.x
  local q2sq = q.y*q.y
  local q3sq = q.z*q.z
  local q0q1 = q0t2*q.x
  local q0q2 = q0t2*q.y
  local q0q3 = q0t2*q.z
  local q1q2 = q1t2*q.y
  local q1q3 = q1t2*q.z
  local q2q3 = q.y*q.z*2
  return { {x = q0sq + q1sq - q2sq - q3sq, y = q1q2 + q0q3, z = -q0q2 + q1q3},
           {x = q1q2 - q0q3, y = q0sq - q1sq + q2sq - q3sq, z = q0q1 + q2q3},
           {x = q0q2 + q1q3, y = -q0q1 + q2q3, z = q0sq - q1sq - q2sq + q3sq} }
end

------------------------------------------------------------------------------------------------------------------------
quatFromRotationVector = function(r)
  local mag2 = r.x*r.x + r.y*r.y + r.z*r.z
  local mag = math.sqrt(mag2)
  if mag2 < 1e-8 then
    return { x = 0, y = 0, z = 0, w = 1 }
  else
    local phi = 0.5 * mag
    local fac = math.sin(phi) / mag
    return { x = r.x*fac, y = r.y*fac, z = r.z*fac, w = math.cos(phi) }
  end
end


------------------------------------------------------------------------------------------------------------------------
-- Drawing functions
------------------------------------------------------------------------------------------------------------------------
--[[
local globalColour = { r = 255, g = 0, b = 0 }
local globalSize = 0.1

local drawCircle = function(centre, axis, radial, detail, col)
  if col == nil then col = globalColour end
  if detail == nil then detail = 10 end
  
  local points = {}
  table.setn(points, detail+1)
  
  local pi = 3.14159265
  local perp = vecCross(radial, vecNormalise(axis))
  local start = vecAdd(centre, radial)
  local endp = {x = 0, y = 0, z = 0}
  for i = 0,detail do
    local theta = i*2*pi/detail
    endp = vecAdd(centre, vecAdd(vecScale(radial, math.cos(theta)), vecScale(perp, math.sin(theta))))
    addDebugLine(start, endp, col)
    start = endp
  end
end

------------------------------------------------------------------------------------------------------------------------
local drawSphere = function(centre, R, radius, detail, col)
  if col == nil then col = globalColour end
  if detail == nil then detail = 10 end
  if radius == nil then radius = globalSize end
  if R == nil then R = {{x=1,y=0,z=0},{x=0,y=1,z=0},{x=0,y=0,z=1}} end
  
  local pi = 3.14159265
  local axis = R[1]
  local perp1 = R[2]
  local perp2 = R[3]
  for i = 0,detail do
    local theta = i*2*pi/detail
    drawCircle(centre, vecAdd(vecScale(perp1, math.cos(theta)), vecScale(perp2, math.sin(theta))), vecScale(axis, radius), detail, col)
  end
end
--]]
