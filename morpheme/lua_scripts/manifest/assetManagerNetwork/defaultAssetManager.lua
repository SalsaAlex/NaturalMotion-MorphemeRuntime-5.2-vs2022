registerAssetManagerNetwork("DefaultNetwork", 
 {
   fileName = "$(Approot)resources\\assetManagerNetwork\\assetManagerNetworkExport.xml",
   animationNode  = "AnimationSourceNode",
   retargetNode = "RetargetNode",
   scaleNode = "ScaleCharacterNode",
   
   -- messages that may be sent to the assetManager
   retargetCharacterOffsetsMessage = "UpdateRetargetOffsets",
   retargetCharacterScaleMessage = "UpdateRetargetScale",
   scaleCharacterMessage = "ScaleRig",
 }
 )