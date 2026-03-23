------------------------------------------------------------------------------------------------------------------------
-- Copyright (c) 2011 NaturalMotion.  All Rights Reserved.
-- Not to be copied, adapted, modified, used, distributed, sold, licensed or commercially exploited in any manner
-- without the written consent of NaturalMotion.
--
-- All non public elements of this software are the confidential information of NaturalMotion and may not be disclosed
-- to any person nor used for any purpose not expressly approved by NaturalMotion in writing.
------------------------------------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------------------------------------
-- vector functions
------------------------------------------------------------------------------------------------------------------------
local assertvector = function(vec)
  assert(type(vec) == "table")
  assert(type(vec.x) == "number")
  assert(type(vec.y) == "number")
  assert(type(vec.z) == "number")
end

------------------------------------------------------------------------------------------------------------------------
local printvector = function(vec)
  if type(vec) == "table" then
    if type(vec.x) == "number" and
       type(vec.y) == "number" and
       type(vec.z) == "number" then
      print(vec.x, " ", vec.y, " ", vec.z)
      return
    end
  end
  
  print("invalid")
end

------------------------------------------------------------------------------------------------------------------------
local newvector = function(x, y, z)
  if type(x) == "table" then
    assertvector(x)
    assert(y == nil)
    assert(z == nil)
    return {
      x = x.x,
      y = x.y,
      z = x.z,
    }
  else
    assert(type(x) == "number")
    assert(type(y) == "number")
    assert(type(z) == "number")
    return {
      x = x,
      y = y,
      z = z,
    }
  end
end

------------------------------------------------------------------------------------------------------------------------
local negatevector = function(vec)
  assertvector(vec)
  return {
    x = -vec.x,
    y = -vec.y,
    z = -vec.z,
  }
end

------------------------------------------------------------------------------------------------------------------------
local addvector = function(lhs, rhs)
  if type(lhs) == "table" then
    assertvector(lhs)

    if type(rhs) == "table" then
      -- vector + vector
      --
      return {
        x = lhs.x + rhs.x,
        y = lhs.y + rhs.y,
        z = lhs.z + rhs.z,
      }
    else
      -- vector + number
      --
      assert(type(rhs) == "number")
      return {
        x = lhs.x + rhs,
        y = lhs.y + rhs,
        z = lhs.z + rhs,
      }
    end
  else
    -- number + vector
    --
    assert(type(lhs) == "number")
    assertvector(rhs)
    return {
      x = lhs + rhs.x,
      y = lhs + rhs.y,
      z = lhs + rhs.z,
    }
  end
end

------------------------------------------------------------------------------------------------------------------------
local subtractvector = function(lhs, rhs)
  if type(lhs) == "table" then
    assertvector(lhs)

    if type(rhs) == "table" then
      -- vector - vector
      --
      return {
        x = lhs.x - rhs.x,
        y = lhs.y - rhs.y,
        z = lhs.z - rhs.z,
      }
    else
      -- vector - number
      --
      assert(type(rhs) == "number")
      return {
        x = lhs.x - rhs,
        y = lhs.y - rhs,
        z = lhs.z - rhs,
      }
    end
  else
    -- number - vector
    --
    assert(type(lhs) == "number")
    assertvector(rhs)
    return {
      x = lhs - rhs.x,
      y = lhs - rhs.y,
      z = lhs - rhs.z,
    }
  end
end

------------------------------------------------------------------------------------------------------------------------
local multiplyvector = function(lhs, rhs)
  if type(lhs) == "table" then
    assertvector(lhs)

    if type(rhs) == "table" then
      -- vector * vector
      --
      return {
        x = lhs.x * rhs.x,
        y = lhs.y * rhs.y,
        z = lhs.z * rhs.z,
      }
    else
      -- vector * number
      --
      assert(type(rhs) == "number")
      return {
        x = lhs.x * rhs,
        y = lhs.y * rhs,
        z = lhs.z * rhs,
      }
    end
  else
    -- number * vector
    --
    assert(type(lhs) == "number")
    assertvector(rhs)
    return {
      x = lhs * rhs.x,
      y = lhs * rhs.y,
      z = lhs * rhs.z,
    }
  end
end

------------------------------------------------------------------------------------------------------------------------
local dividevector = function(lhs, rhs)
  assertvector(lhs)

  if type(rhs) == "table" then
    -- vector / vector
    --
    return {
      x = lhs.x / rhs.x,
      y = lhs.y / rhs.y,
      z = lhs.z / rhs.z,
    }
  else
    -- vector / number
    --
    assert(type(rhs) == "number")
    return {
      x = lhs.x / rhs,
      y = lhs.y / rhs,
      z = lhs.z / rhs
    }
  end
end

------------------------------------------------------------------------------------------------------------------------
local lengthsquaredvector = function(lhs)
  assertvector(lhs)
  return (lhs.x * lhs.x) + (lhs.y * lhs.y) + (lhs.z * lhs.z)
end

------------------------------------------------------------------------------------------------------------------------
local lengthvector = function(lhs)
  assertvector(lhs)
  return math.sqrt((lhs.x * lhs.x) + (lhs.y * lhs.y) + (lhs.z * lhs.z))
end

------------------------------------------------------------------------------------------------------------------------
local normalisevector = function(lhs)
  local length = lengthvector(lhs)
  return {
    x = lhs.x / length,
    y = lhs.y / length,
    z = lhs.z / length,
  }
end

------------------------------------------------------------------------------------------------------------------------
local dotvector = function(lhs, rhs)
  assertvector(lhs)
  assertvector(rhs)
  return (lhs.x * rhs.x) + (lhs.y * rhs.y) + (lhs.z * rhs.z)
end

------------------------------------------------------------------------------------------------------------------------
local crossvector = function(lhs, rhs)
  assertvector(lhs)
  assertvector(rhs)
  return {
    x = (lhs.y * rhs.z) - (lhs.z * rhs.y),
    y = (lhs.z * rhs.x) - (lhs.x * rhs.z),
    z = (lhs.x * rhs.y) - (lhs.y * rhs.x),
  }
end

------------------------------------------------------------------------------------------------------------------------
-- quaternion functions
------------------------------------------------------------------------------------------------------------------------
local assertquat = function(quat)
  assert(type(quat) == "table")
  assert(type(quat.x) == "number")
  assert(type(quat.y) == "number")
  assert(type(quat.z) == "number")
  assert(type(quat.w) == "number")
end

------------------------------------------------------------------------------------------------------------------------
local printquat = function(vec)
  if type(vec) == "table" then
    if type(vec.x) == "number" and
       type(vec.y) == "number" and
       type(vec.z) == "number" and
       type(vec.w) == "number" then
      print(vec.x, " ", vec.y, " ", vec.z, " ", vec.w)
      return
    end
  end

  print("invalid")
end

------------------------------------------------------------------------------------------------------------------------
local newquat = function(x, y, z, w)
  if type(x) == "table" then
    assertquat(x)
    assert(y == nil)
    assert(z == nil)
    assert(w == nil)
    return {
      x = x.x,
      y = x.y,
      z = x.z,
      w = x.w,
    }
  else
    assert(type(x) == "number")
    assert(type(y) == "number")
    assert(type(z) == "number")
    assert(type(w) == "number")
    return {
      x = x,
      y = y,
      z = z,
      w = w,
    }
  end
end

------------------------------------------------------------------------------------------------------------------------
local multiplyquat = function(lhs, rhs)
  assertvector(lhs)
  assertquat(rhs)
  return {
    x = (rhs.w * lhs.x) + (rhs.x * lhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
    y = (rhs.w * lhs.y) + (rhs.y * lhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
    z = (rhs.w * lhs.z) + (rhs.z * lhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x),
    w = (rhs.w * lhs.w) - (rhs.x * lhs.x) - (rhs.y * lhs.y) - (rhs.z * lhs.z),
  }
end

------------------------------------------------------------------------------------------------------------------------
-- other maths functions
------------------------------------------------------------------------------------------------------------------------
local rotatevectorquat = function(vec, rotation)
  assertvector(vec)
  assertquat(rotation)

  local result = crossvector(rotation, vec)
  result = multiplyvector(result, rotation.w * 2.0)

  result = addvector(result, multiplyvector(vec, 2 * (rotation.w * rotation.w) - 1))
  return addvector(result, multiplyvector(rotation, dotvector(rotation, vec) * 2))
end

------------------------------------------------------------------------------------------------------------------------
local transformvector = function(vec, rotation, translation)
  assertvector(vec)
  assertquat(rotation)
  assertvector(translation)

  local rotated = rotatevectorquat(vec, quat)
  return addvector(rotated, translation)
end

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: NAMESPACE
--| name: vector
--| title: Vector math functions
--| brief: Morpheme:Connect vector commands are used to manipulate vectors in preview scripts.
------------------------------------------------------------------------------------------------------------------------
vector = {
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table vector.print(table vec)
  --| brief: Prints a vector. prints "invalid" if vec is not a valid vector table.
  ----------------------------------------------------------------------------------------------------------------------
  print = printvector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table vector.new(table vec)
  --| signature: table vector.new(number x, number y, number z)
  --| brief: Creates a new vector table.
  ----------------------------------------------------------------------------------------------------------------------
  new = newvector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table vector.negate(table vec)
  --| brief: Negates a vector returning the result.
  ----------------------------------------------------------------------------------------------------------------------
  negate = negatevector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table vector.add(table lhs, table rhs)
  --| signature: table vector.add(table lhs, number rhs)
  --| signature: table vector.add(number lhs, table rhs)
  --| brief: Adds two vectors or a vector and a scalar returning the result.
  ----------------------------------------------------------------------------------------------------------------------
  add = addvector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table vector.subtract(table lhs, table rhs)
  --| signature: table vector.subtract(table lhs, number rhs)
  --| signature: table vector.subtract(number lhs, table rhs)
  --| brief: Subtracts two vectors or a vector and a scalar returning the result.
  ----------------------------------------------------------------------------------------------------------------------
  subtract = subtractvector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table vector.multiply(table lhs, table rhs)
  --| signature: table vector.multiply(number lhs, table rhs)
  --| signature: table vector.multiply(table lhs, number rhs)
  --| brief: Multiplies two vectors or a vector and a scalar returning the result.
  ----------------------------------------------------------------------------------------------------------------------
  multiply = multiplyvector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table vector.divide(table lhs, table rhs)
  --| signature: table vector.divide(table lhs, number rhs)
  --| brief: Divides two vectors or a vector and a scalar returning the result.
  ----------------------------------------------------------------------------------------------------------------------
  divide = dividevector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: number vector.lengthsquared(table lhs)
  --| brief: Returns the squared lengthgth of a vector.
  ----------------------------------------------------------------------------------------------------------------------
  lengthsquared = lengthsquaredvector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: number vector.length(table lhs)
  --| brief: Returns the length of a vector.
  ----------------------------------------------------------------------------------------------------------------------
  length = lengthvector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table vector.normalise(table lhs)
  --| brief: Normalises a vector returning the result.
  ----------------------------------------------------------------------------------------------------------------------
  normalise = normalisevector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: number vector.dot(table lhs, table rhs)
  --| brief: Returns the dot product of two vectors.
  ----------------------------------------------------------------------------------------------------------------------
  dot = dotvector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table vector.cross(table lhs, table rhs)
  --| brief: Returns the cross product of two vectors.
  ----------------------------------------------------------------------------------------------------------------------
  cross = crossvector,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table vector.rotate(table vec, table rotation)
  --| brief: Rotates vector vec by quaternion rotation returning the rotated vector result.
  ----------------------------------------------------------------------------------------------------------------------
  rotate = rotatevectorquat,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table vector.transform(table vec, table rotation, table translation)
  --| brief: Transforms vector vec by quaternion rotation and vector translation returning the transformed vector result.
  ----------------------------------------------------------------------------------------------------------------------
  transform = transformvector,

  -- some vector constants
  --
  zero = newvector(0.0, 0.0, 0.0),
  one = newvector(1.0, 1.0, 1.0),
  xaxis = newvector(1.0, 0.0, 0.0),
  yaxis = newvector(0.0, 1.0, 0.0),
  zaxis = newvector(0.0, 0.0, 1.0),
}

------------------------------------------------------------------------------------------------------------------------
--| LUAHELP: NAMESPACE
--| name: vector
--| title: Quaternion math functions
--| brief: Morpheme:Connect quaternion commands are used to manipulate quaternions in preview scripts.
------------------------------------------------------------------------------------------------------------------------
quat = {
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table quat.print(table q)
  --| brief: Prints a quaternion. prints "invalid" if q is not a valid quaternion table.
  ----------------------------------------------------------------------------------------------------------------------
  print = printquat,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table quat.new(table rhs)
  --| signature: table quat.new(number x, number y, number z, number w)
  --| brief: Creates a new quaternion table.
  ----------------------------------------------------------------------------------------------------------------------
  new = newquat,
  ----------------------------------------------------------------------------------------------------------------------
  --| LUAHELP: FUNCTION
  --| signature: table quat.multiply(table lhs, table rhs)
  --| brief: Multiplies two quaternions together returning the result.
  ----------------------------------------------------------------------------------------------------------------------
  multiply = multiplyquat,
}