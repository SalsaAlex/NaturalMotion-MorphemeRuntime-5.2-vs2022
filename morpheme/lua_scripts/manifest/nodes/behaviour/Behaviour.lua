------------------------------------------------------------------------------------------------------------------------
-- Behaviour node definition.
------------------------------------------------------------------------------------------------------------------------
registerNode("Behaviour",
  {
    helptext = "Behaviour node",
    group = "Behaviours",
    image = "Behaviour.png",
    id = generateNamespacedId(idNamespaces.NaturalMotion, 128),
    version = 1,

    --------------------------------------------------------------------------------------------------------------------
    functionPins =
    {
      ["Result"] = {
        input = false,
        passThrough = false,
        interfaces =
        {
          required = { "Transforms", "Time", "Events", },
          optional = { },
        },
      },
    },

    pinOrder =
    {
      "Result",
    },

    --------------------------------------------------------------------------------------------------------------------
    attributes =
    {
    },

    --------------------------------------------------------------------------------------------------------------------
    validate = function(node)
      return true
    end,

    --------------------------------------------------------------------------------------------------------------------
    serialize = function(node, stream)
		stream:writeUInt(0, "NumMessageSlots")
		stream:writeInt(29, "BehaviourID")
    end,

    --------------------------------------------------------------------------------------------------------------------
    getTransformChannels = function(node)
      local resultChannels = setUnion()
      return resultChannels
    end,

    --------------------------------------------------------------------------------------------------------------------
    upgrade = function(node, version, pinLookupTable)
      -- for future use
    end,
  }
)

------------------------------------------------------------------------------------------------------------------------
if not mcn.inCommandLineMode() then
  -- attributeEditor.registerDisplayInfo(
  -- "Behaviour",
  -- {
  -- add custom UI here
  -- }
  -- )
end
