registerNote("Note",
{
    helptext = "A standard note",
    group = "Notes",
    image = "Note.png",
    
     attributes =
    {
      {
        name = "Message", type = "string",
        value = "",
        helptext = "Type the Note to display in the Graph here"
      },
    },
    
    -- return the text shown when the user is not editing the box
    getNoteDisplayText = function(node)
        local message =  getAttribute(node, "Message")
        if message == "" then 
          return "Double click here to edit text"
        else
          return message
        end
    end,
    
    -- return the text shown when the user is editing the box
    getNoteEditText = function(node)
        return getAttribute(node, "Message")
    end,
    
    -- transforms the text when the user hits enter or clicks outside the box
    setNoteEditText = function(node, inputText)
        setAttribute(string.format("%s.Message", node), inputText)
        return true
    end,
    
}
)
