// Copyright (c) 2009 NaturalMotion.  All Rights Reserved.
// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

//----------------------------------------------------------------------------------------------------------------------
// includes in various places, notably packet.h, to define and iterate over
// the various internal packet types.
//----------------------------------------------------------------------------------------------------------------------

#define MORPHEME_3_6_2

#ifndef MORPHEME_3_6_2

PACKET(IdentificationCmd)
PACKET(IdentificationReply)

PACKET(BeginFrame)
PACKET(EndFrame)
PACKET(BeginFrameSegment)
PACKET(EndFrameSegment)
PACKET(BeginInstanceData)
PACKET(EndInstanceData)
PACKET(BeginInstanceSection)
PACKET(EndInstanceSection)

// BeginFrame
//   BeginFrameSegment(Instances)
//     BeginInstanceData(id)
//       BeginInstanceSection(morphemeSectionId)
//         (section data)
//       EndInstanceSection(morphemeSectionId)
//       ...
//       (other sections: physics, euphoria, etc...)
//     EndInstanceData(id)
//     ...
//     (other instances)
//   EndFrameSegment(Instances)
//   ...
//   (other segments: scene objects, etc)
// EndFrame

PACKET(AddStringToCache)

PACKET(AnimRigDef)
PACKET(TransformBuffer)

// Morpheme network descriptor
PACKET(NetworkDef)
PACKET(NodeDef)
PACKET(MessageDef)
PACKET(SceneObjectDef)

PACKET(BeginPersistent)
PACKET(EndPersistent)

PACKET(ConnectActiveInstance)
PACKET(NetworkInstance)
PACKET(NetworkCreatedReply)
PACKET(NetworkDestroyedReply)
PACKET(NetworkDefLoadedReply)
PACKET(NetworkDefDestroyedReply)

PACKET(ID32s)
PACKET(ID16s)
PACKET(ActiveNodes)
PACKET(NodeEventMessages)
PACKET(ActiveInstances)
PACKET(RootStateMachineNodes)
PACKET(NodeOutputData)
PACKET(PersistentData)
PACKET(FrameData)
PACKET(ScatterBlendWeights)

PACKET(ActiveSceneObjects)

PACKET(ConnectFrameData)

// dispatcher debugging
PACKET(BeginDispatcherTaskExecute)
PACKET(AddDispatcherTaskExecuteParameter)
PACKET(EndDispatcherTaskExecute)
PACKET(ProfilePointTiming)
PACKET(FrameNodeTimings)

// Recording messages sent to state machines
PACKET(StateMachineMessageEventMsg)

PACKET(SceneObjectUpdate)

// Attribute packets are used to describe scene objects
PACKET(AttributeDef)
PACKET(AttributeUpdate)

// List of commands. These packets go from app to runtime.

PACKET(PingCmd)
PACKET(PingData)

PACKET(ClearCachedDataCmd)

PACKET(StartSessionCmd)
PACKET(PauseSessionCmd)
PACKET(StopSessionCmd)

PACKET(StepModeCmd)

// Network management
PACKET(LoadNetworkCmd)
PACKET(CreateNetworkInstanceCmd)
PACKET(DestroyNetworkInstanceCmd)
PACKET(DestroyNetworkDefinitionCmd)
PACKET(ReferenceNetworkInstanceCmd)
PACKET(UnreferenceNetworkInstanceCmd)
PACKET(ReferenceNetworkInstanceReply)

PACKET(NetworkDefinitionDestroyedCtrl)
PACKET(TargetStatusCtrl)

// Debugging commands
PACKET(SetAnimationSetCmd)
PACKET(SetAssetManagerAnimationSetCmd)
PACKET(SetControlParameterCmd)
PACKET(SendRequestCmd)
PACKET(BroadcastRequestCmd)
PACKET(MessageBufferCmd)
PACKET(SetCurrentStateCmd)
PACKET(ExecuteCommandCmd)
PACKET(EnableOutputDataCmd)
PACKET(SetDebugOutputFlagsCmd)
PACKET(SetDebugOutputOnNodesCmd)
PACKET(StepCmd)
PACKET(SetRootTransformCmd)

// Scene object and Environment commands.
PACKET(SetAttributeCmd)
PACKET(SetEnvironmentAttributeCmd)

PACKET(DestroySceneObjectCmd)

PACKET(SceneObjectDestroyed)

PACKET(BeginSceneObjectCmd)
PACKET(EndSceneObjectCmd)
PACKET(AttributeCmd)

// Physics manipulation commands
PACKET(CreateConstraintCmd)
PACKET(MoveConstraintCmd)
PACKET(RemoveConstraintCmd)
PACKET(ApplyForceCmd)


// Fileserver packets.
PACKET(Filename)
PACKET(FileSizeRequest)
PACKET(FileRequest)
PACKET(FileSize)
PACKET(File)


PACKET(ConnectScriptCommand)
PACKET(DownloadFrameDataCmd)
PACKET(DownloadSceneObjectsCmd)
PACKET(DownloadGlobalDataCmd)
PACKET(DownloadNetworkDefinitionCmd)

PACKET(Reply)

PACKET(PreviewChannels)

#else

PACKET(IdentificationCmd)					//17
PACKET(IdentificationReply)					//18

PACKET(BeginFrame)							//19
PACKET(EndFrame)							//20
PACKET(BeginFrameSegment)					//21
PACKET(EndFrameSegment)						//22
PACKET(BeginInstanceData)					//23
PACKET(EndInstanceData)						//24
PACKET(BeginInstanceSection)				//25
PACKET(EndInstanceSection)					//26

// BeginFrame
//   BeginFrameSegment(Instances)
//     BeginInstanceData(id)
//       BeginInstanceSection(morphemeSectionId)
//         (section data)
//       EndInstanceSection(morphemeSectionId)
//       ...
//       (other sections: physics, euphoria, etc...)
//     EndInstanceData(id)
//     ...
//     (other instances)
//   EndFrameSegment(Instances)
//   ...
//   (other segments: scene objects, etc)
// EndFrame

PACKET(AddStringToCache)					//27

PACKET(AnimRigDef)							//28
PACKET(TransformBuffer)						//29

// Morpheme network descriptor
PACKET(NetworkDef)							//30
PACKET(NodeDef)								//31
PACKET(MessageDef)							//32
PACKET(SceneObjectDef)						//33

PACKET(BeginPersistent)						//34
PACKET(EndPersistent)						//35

PACKET(ConnectActiveInstance)				//36
PACKET(NetworkInstance)						//37
PACKET(NetworkCreatedReply)					//38
PACKET(NetworkDestroyedReply)				//39
PACKET(NetworkDefLoadedReply)				//40
PACKET(NetworkDefDestroyedReply)			//41

PACKET(ID32s)								//42
PACKET(ID16s)								//43
PACKET(ActiveNodes)							//44
//PACKET(NodeEventMessages)					
PACKET(ActiveInstances)						//45
PACKET(RootStateMachineNodes)				//46
PACKET(NodeOutputData)						//47
PACKET(PersistentData)						//48
PACKET(FrameData)							//49
//PACKET(ScatterBlendWeights)				

PACKET(ActiveSceneObjects)					//50

PACKET(ConnectFrameData)					//51

// dispatcher debugging
PACKET(BeginDispatcherTaskExecute)			//52
PACKET(AddDispatcherTaskExecuteParameter)	//53
PACKET(EndDispatcherTaskExecute)			//54
PACKET(ProfilePointTiming)					//55
PACKET(FrameNodeTimings)					//56

// Recording messages sent to state machines
PACKET(StateMachineMessageEventMsg)			//57

PACKET(SceneObjectUpdate)					//58

// Attribute packets are used to describe scene objects
PACKET(AttributeDef)						//59
PACKET(AttributeUpdate)						//60

// List of commands. These packets go from app to runtime.

PACKET(PingCmd)								//61
PACKET(PingData)							//62

PACKET(ClearCachedDataCmd)					//63

PACKET(StartSessionCmd)						//64
PACKET(PauseSessionCmd)						//65
PACKET(StopSessionCmd)						//66

PACKET(StepModeCmd)							//67 SIXSVENVENEVEEENV

// Network management
PACKET(LoadNetworkCmd)						//68
PACKET(CreateNetworkInstanceCmd)			//69
PACKET(DestroyNetworkInstanceCmd)			//70
PACKET(DestroyNetworkDefinitionCmd)			//71
PACKET(ReferenceNetworkInstanceCmd)			//72
PACKET(UnreferenceNetworkInstanceCmd)		//73
PACKET(ReferenceNetworkInstanceReply)		//74

PACKET(NetworkDefinitionDestroyedCtrl)		//75
PACKET(TargetStatusCtrl)					//76

// Debugging commands
PACKET(SetAnimationSetCmd)					//77
PACKET(SetAssetManagerAnimationSetCmd)		//78
PACKET(SetControlParameterCmd)				//79
PACKET(SendRequestCmd)						//80
PACKET(BroadcastRequestCmd)					//81
PACKET(MessageBufferCmd)					//82
PACKET(SetCurrentStateCmd)					//83
PACKET(ExecuteCommandCmd)					//84
PACKET(EnableOutputDataCmd)					//85
PACKET(SetDebugOutputFlagsCmd)				//86
PACKET(SetDebugOutputOnNodesCmd)			//87
PACKET(StepCmd)								//88
PACKET(SetRootTransformCmd)					//89

// Scene object and Environment commands.
PACKET(SetAttributeCmd)						//90
PACKET(SetEnvironmentAttributeCmd)			//91

PACKET(DestroySceneObjectCmd)				//92

PACKET(SceneObjectDestroyed)				//93

PACKET(BeginSceneObjectCmd)					//94
PACKET(EndSceneObjectCmd)					//95
PACKET(AttributeCmd)						//96

// Physics manipulation commands
PACKET(CreateConstraintCmd)					//97
PACKET(MoveConstraintCmd)					//98
PACKET(RemoveConstraintCmd)					//99
PACKET(ApplyForceCmd)						//100


// Fileserver packets.
PACKET(Filename)							//101
PACKET(FileSizeRequest)						//102
PACKET(FileRequest)							//103
PACKET(FileSize)							//104
PACKET(File)								//105


//PACKET(ConnectScriptCommand)				
PACKET(DownloadFrameDataCmd)				//106
PACKET(DownloadSceneObjectsCmd)				//107
PACKET(DownloadGlobalDataCmd)				//108
PACKET(DownloadNetworkDefinitionCmd)		//109

PACKET(Reply)								//110

PACKET(PreviewChannels)						//111

#endif