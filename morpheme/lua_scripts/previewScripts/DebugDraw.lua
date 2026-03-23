------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2011 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------
require "previewScripts/VectorMath.lua"
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- these colours mimic the constants found in NMColour.h
------------------------------------------------------------------------------------------------------------------------
colour = {
  -- shades of white
  --
  black          = { r = 0.00, g = 0.00, b = 0.00, },
  darkGrey       = { r = 0.30, g = 0.30, b = 0.30, },
  lightGrey      = { r = 0.75, g = 0.75, b = 0.75, },
  white          = { r = 1.00, g = 1.00, b = 1.00, },

  -- shades of red
  --
  darkRed        = { r = 0.50, g = 0.00, b = 0.00, },
  red            = { r = 1.00, g = 0.00, b = 0.00, },
  lightRed       = { r = 1.00, g = 0.60, b = 0.60, },

  -- shades of green
  --
  darkGreen      = { r = 0.00, g = 0.50, b = 0.00, },
  green          = { r = 0.00, g = 1.00, b = 0.00, },
  lightGreen     = { r = 0.60, g = 1.00, b = 0.60, },

  -- shades of blue
  --
  darkBlue       = { r = 0.00, g = 0.00, b = 0.50, },
  blue           = { r = 0.00, g = 0.00, b = 1.00, },
  lightBlue      = { r = 0.60, g = 0.60, b = 1.00, },

  -- shades of yellow
  --
  darkYellow     = { r = 0.50, g = 0.50, b = 0.00, },
  yellow         = { r = 1.00, g = 1.00, b = 0.00, },
  lightYellow    = { r = 1.00, g = 1.00, b = 0.60, },

  -- shades of purple
  --
  darkPurple     = { r = 0.50, g = 0.00, b = 0.50, },
  purple         = { r = 1.00, g = 0.00, b = 1.00, },
  lightPurple    = { r = 1.00, g = 0.60, b = 1.00, },

  -- shades of turquoise
  --
  darkTurquoise  = { r = 0.00, g = 0.50, b = 0.50, },
  turquoise      = { r = 0.00, g = 1.00, b = 1.00, },
  lightTurquoise = { r = 0.60, g = 1.00, b = 1.00, },

  -- shades of orange
  --
  darkOrange     = { r = 0.50, g = 0.25, b = 0.00, },
  orange         = { r = 1.00, g = 0.50, b = 0.00, },
  lightOrange    = { r = 1.00, g = 0.75, b = 0.50, },
}

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: void addDebugVector(table position, table direction, float size, table colour)
--| signature: void addDebugVector(table position, table direction, float size, table colour, integer lifespan)
--| brief: Draws a vector from position along direction of length size. Assumes direction is normalised.
--| page: Control
------------------------------------------------------------------------------------------------------------------------
addDebugVector = function(position, direction, size, colour, lifespan)
  lifespan = lifespan or 1

  addDebugLine(
    position,
    vector.add(position, vector.multiply(direction, size)),
    colour,
    lifespan)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: void addDebugLocator(table position, float halfSize, table colour)
--| signature: void addDebugLocator(table position, float halfSize, table colour, integer lifespan)
--| brief: Draws a three axis cross at a given point.
--| page: Control
------------------------------------------------------------------------------------------------------------------------
addDebugLocator = function(position, size, colour, lifespan)
  lifespan = lifespan or 1

  addDebugLine(
    vector.add(position, vector.multiply(vector.xaxis, size)),
    vector.subtract(position, vector.multiply(vector.xaxis, size)),
    colour,
    lifespan)

  addDebugLine(
    vector.add(position, vector.multiply(vector.yaxis, size)),
    vector.subtract(position, vector.multiply(vector.yaxis, size)),
    colour,
    lifespan)

  addDebugLine(
    vector.add(position, vector.multiply(vector.zaxis, size)),
    vector.subtract(position, vector.multiply(vector.zaxis, size)),
    colour,
    lifespan)
end

------------------------------------------------------------------------------------------------------------------------
-- draws an arrow head
------------------------------------------------------------------------------------------------------------------------
local addDebugArrowHead = function(position, forward, side, size, colour, lifespan)
  --    leftArrowPoint
  --  /
  -- /_ arrowTipBegin
  -- \
  --  \ rightArrowPoint

  local arrowTipBegin = vector.subtract(position, vector.multiply(forward, size))

  local halfSize = 0.5 * size

  local leftArrowPoint = vector.add(arrowTipBegin, vector.multiply(side, halfSize))
  addDebugLine(position, leftArrowPoint, colour, lifespan)

  local rightArrowPoint = vector.add(arrowTipBegin, vector.multiply(vector.negate(side), halfSize))
  addDebugLine(position, rightArrowPoint, colour, lifespan)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: FUNCTION
--| signature: void addDebugArrow(table position, table direction, float length, float size, table colour)
--| signature: void addDebugArrow(table position, table direction, float length, float size, table colour, integer lifespan)
--| brief: Draws an arrow starting from position along direction of length length. Assumes direction is normalised.
--| page: Control
------------------------------------------------------------------------------------------------------------------------
addDebugArrow = function(position, direction, length, size, colour, lifespan)
  lifespan = lifespan or 1
  
  local tip = vector.add(position, vector.multiply(direction, length))
  addDebugLine(position, tip, colour, lifespan)
  
  local side = vector.cross(vector.yaxis, direction)
  addDebugArrowHead(tip, direction, side, size, colour, lifespan)

  local up = vector.cross(direction, side)
  addDebugArrowHead(tip, direction, up, size, colour, lifespan)
end