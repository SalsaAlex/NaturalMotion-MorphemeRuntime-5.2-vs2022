registerNote("ScriptNote",
{
    helptext = "A script note",
    group = "Notes",
    image = "ScriptNote.png",
    
     attributes =
    {
      {
        name = "Script", type = "string",
        value = "return 'Type a script here'",
        helptext = "Script to be executed and displayed. The script should return a string (or example 'return ls()')"
      },
    },
    
    -- return the text shown when the user is not editing the box
    getNoteDisplayText = function(node)
        local script =  getAttribute(node, "Script")
        local fullScript = string.format("return function (node) %s end", script)
        
        local func, errorMsg = loadstring(fullScript)()
        
        if not func then 
          return errorMsg
        end
        
        local success, v1, v2 = pcall(func, node)
        if success then 
          local stringResult = ""
          if(type(v1) == "table") then
            stringResult = table.concat(v1, "  ")
          else
            stringResult = tostring(v1)
          end
          return stringResult
        else
          return v1
        end
      

    end,
    
    -- return the text shown when the user is editing the box
    getNoteEditText = function(node)
        return getAttribute(node, "Script")
    end,
    
    -- transforms the text when the user hits enter or clicks outside the box
    setNoteEditText = function(node, inputText)
        local function trim (s)
          -- trim whitespace
          return (string.gsub(s, "^%s*(.-)%s*$", "%1"))
        end
        setAttribute(string.format("%s.Script", node),trim(inputText))
        return true
    end,
    
}
)

