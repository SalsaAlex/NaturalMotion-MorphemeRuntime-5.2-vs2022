# Morpheme Runtime 5.2 project dictionary (by holypinko)

Dense map of this repository: what each major module is for, where headers live, and how the runtime pieces connect. It does not replace the headers (function contracts, parameter lists, and edge cases stay in source). Symbols and paths are written for the layout in this zip.

Version constants are in `morpheme/SDK/sharedDefines/mVersion.h`:

- `MR_VERSION_MAJOR` 5, `MR_VERSION_MINOR` 2, `MR_VERSION_RELEASE` 0, `MR_VERSION_REVISION` 0
- `MR_VERSION_RUNTIME_BINARY` 24 (increment when any serialised asset binary layout changes)
- `MR_VERSION_CHECK(Major, Minor, Release, Revision)` and `MR_VERSION_CHECK_ERROR` for compile-time checks
- `MR_DEPRECATED_MESSAGE` pairs deprecation messages with version gates

Namespaces: `MR::` is almost all Morpheme runtime. `NMP::` is the NaturalMotion platform layer from `common/NMPlatform` (math, memory, sockets, containers). Game samples sometimes use their own namespaces. Euphoria uses `er` prefixes and headers under `euphoria/`.

---

## How the runtime is structured

A **network** is a compiled behaviour graph: states, transitions, blend trees, and control parameters. `MR::NetworkDef` is the baked definition; `MR::Network` is the per-instance state (which nodes are active, frame counter, attribute storage, queued work).

**Manager** (`mrManager.h`) is the global registry. One instance is intended. It holds:

- Registered assets (rig, maps, event tracks, physics defs, network defs, etc.) addressed by `ObjectID`
- Tables of task queuing functions, output CP tasks, node init/delete, message handlers, attribute semantics, attrib data types, transit condition types, prediction models, and plugin asset loaders
- Animation load policy: `RequestAnimFn` / `ReleaseAnimFn` you supply so the engine can resolve `RuntimeAnimAssetID` to `AnimSourceBase*`

**Update flow** (conceptually): active nodes get `UpdateNodeConnections` first (child activation, immediate CP updates). Then tasks are queued per node via `QueueAttrTaskFn` into a `TaskQueue`. The **Dispatcher** runs tasks; each task receives `Dispatcher::TaskParameters` with methods like `getInputAttrib`, `createOutputAttrib`, `createOutputCPAttrib` to read/write `AttribData` blobs. Semantics (`AttribDataSemantic`) describe what a pin means; the manager maps semantics to "create reference" task IDs and registration metadata.

**Locate / dislocate**: binary assets are often loaded as raw buffers. Types register `locate` and `dislocate` functions so pointers and endianness are fixed for the target platform after load or before network transfer.

---

## Library bootstrap (core)

From `MR::Manager` in `mrManager.h`:

1. `MR::Manager::initMorphemeLib()` creates the manager and runs core registration (`registerCore*` from `mrNodes.h` and friends) in the "compute registry requirements" phase where applicable.
2. `allocateRegistry()` allocates internal tables sized by those requirements. On PS3 SPU, `allocateRegistrySPU` exists to pass known bounds from PPU.
3. `finaliseInitMorphemeLib()` performs registration that requires allocated storage (attribute semantics, etc.).
4. `setAnimFileHandlingFunctions(RequestAnimFn*, ReleaseAnimFn*)` wires animation loading.
5. `termMorphemeLib()` tears down.

You still register concrete assets with `registerObject(ptr, typeID, objectID)` and use refcounts via `incObjectRefCount` / `decObjectRefCount` / `getObjectRefCount`.

`getTargetPlatformAssetFmt()` returns a bitmask of `PlatformFormat` (`kPlatform_BigEndian`, `kPlatform_LittleEndian`, `kPlatform_32Bit`, `kPlatform_64Bit`). `getRuntimeBinaryVersion()` returns `MR_VERSION_RUNTIME_BINARY`.

---

## `MR::Manager::AssetType` (enum)

Values from `mrManager.h` (used when registering objects):

| Value | Name |
|-------|------|
| 1 | `kAsset_Rig` |
| 2 | `kAsset_RigToAnimMap` |
| 3 | `kAsset_EventTrackDiscrete` |
| 4 | `kAsset_EventTrackDuration` |
| 5 | `kAsset_EventTrackCurve` |
| 6 | `kAsset_PhysicsRigDef` |
| 7 | `kAsset_CharacterControllerDef` |
| 8 | `kAsset_InteractionProxyDef` |
| 9 | `kAsset_BodyDef` |
| 10 | `kAsset_NetworkDef` |
| 11 | `kAsset_NetworkPredictionDef` |
| 12 | `kAsset_PluginList` |

`kAsset_NumAssetTypes` is the count; `kAsset_Invalid` is `0xFFFFFFFF`.

---

## Node wiring typedefs (`mrNode.h`)

These are the function-pointer shapes the manager stores for node types:

- `QueueAttrTaskFn`: given `NodeDef*`, `TaskQueue*`, `Network*`, and a dependent `TaskParameter*`, enqueue zero or more `Task*` objects that compute attributes.
- `OutputCPTask`: immediate evaluation for a control parameter output pin (`PinIndex`), returns new `AttribData*` for that pin on the network.
- `InitNodeInstance` / `DeleteNodeInstance`: per-instance setup and teardown of node-owned attrib data in the network.
- `UpdateNodeConnections`: first pass each frame; decides active children and updates CP wiring; must recurse as required by the node type.
- `FindGeneratingNodeForSemanticFn`: traces which connected node produces a given `AttribDataSemantic` (used when queuing depends on graph topology).
- `MessageHandlerFn`: optional handler for `Message` delivery to a node.

Registration goes through `MR::Manager` (`registerTaskQueuingFn`, `registerOutputCPTask`, `registerInitNodeInstanceFn`, `registerDeleteNodeInstanceFn`, `registerUpdateNodeConnectionsFn`, `registerFindGeneratingNodeForSemanticFn`, `registerMessageHandlerFn`, plus attrib and message type tables). Names are stored for debugging (`getTaskQueuingFnName`, etc.).

---

## Core registration entry points (`morpheme/SDK/core/include/morpheme/Nodes/mrNodes.h`)

- `registerCoreQueuingFnsAndOutputCPTasks()`
- `registerCoreAttribSemantics(bool computeRegistryRequirements)` two-phase: `true` counts required slots, `false` fills them after `allocateRegistry`
- `registerCorePredictionModelTypes()`
- `registerCoreNodeInitDatas()`
- `registerCoreAttribDataTypes()`
- `registerCoreTransitConditions()`
- `registerCoreAssets()`

Custom nodes and plugins register additional entries in the same tables.

---

## Limits (`mrDefines.h`)

Examples (full list in header):

- `MAX_NUM_MESSAGE_TYPES` 256
- `MAX_NUM_NODE_TYPES` 256
- `MAX_NUM_QUEUING_FNS` 1024
- `MAX_NUM_IMMEDIATE_FNS` 1024
- `MAX_NUM_NODE_INIT_DATA_TYPES` 16
- `MAX_NUM_ATTR_DATA_TYPES` 144
- `MAX_NUM_TRANSIT_COND_TYPES` 256
- `MAX_NUM_TRANSIT_DEADBLEND_TYPES` 16
- `MAX_NUM_PREDICTION_MODEL_TYPES` 256
- `MAX_NUM_SYNC_EVENTS` 16
- `MAX_NUM_DURATION_EVENT_TRACKS_PER_SET` 16

`REG_FUNC_ARGS` / `REG_FUNC_ARGS_COMPUTE` macros wrap registration parameters for SPU builds (`NMP_NULL_ON_SPU` for string names on SPU).

---

## IDs and shared types (`mSharedDefines.h`)

Core typedefs (all documented in the header):

- `TaskID`, `NodeID` (`INVALID_NODE_ID`, `NETWORK_NODE_ID` = 0 root)
- `StateID`, `ConditionIndex`, `StateConditionIndex`, `INVALID_CONDITION_INDEX`
- `LimbIndex`, `INVALID_LIMB_INDEX`
- `PinIndex`, `INVALID_PIN_INDEX`, `CONTROL_PARAMETER_NODE_PIN_0`
- `MessageID`, `MessageType`, `TransitConditType`, `NodeType`
- `AnimSetIndex`, `ANIMATION_SET_ANY`
- `FrameCount`, `NOT_FRAME_UPDATED`
- `AttribDataType` and large `AttribDataTypeEnum` (bool, int, float, vectors, arrays, transform buffers, trajectory, velocity, event tracks, rig, source anim, rig-to-anim map, and many specialised types; some entries guarded by `MORPHEME_CONNECT_362`)

**Customer IDs**: `GEN_NAMESPACED_ID_32`, `GEN_NAMESPACED_ID_16`, `GEN_NODE_TYPE_ID`, `GEN_TRANSITCONDIT_TYPE_ID` pack customer and type bits so Connect and runtime stay consistent.

**Node type macros**: long list of `NODE_TYPE_*` values (state machine, control params, blend nodes, IK, physics-related grouper types, operators, transit, euphoria-adjacent types, etc.). Do not renumber NaturalMotion slots without syncing Connect.

**Transit condition type macros**: `TRANSCOND_ON_MESSAGE_ID`, `TRANSCOND_DISCRETE_EVENT_TRIGGERED_ID`, `TRANSCOND_CROSSED_DURATION_FRACTION_ID`, through physics, ray, CP comparisons, bool set, and the rest through `TRANSCOND_CONTROL_PARAM_BOOL_SET_ID`.

**Dead blend**: `TRANSDEADBLEND_DEFAULT_ID`.

**Debug**: `MR_OUTPUT_DEBUGGING` unless `MR_DISABLE_OUTPUT_DEBUGGING` or SPU. `MR_OUTPUT_DEBUG_ARG`, `MR_NULL_NO_OUTPUT_DEBUGGING`, `MR_USED_FOR_OUTPUT_DEBUGGING` strip or keep debug parameters. `MR_ENABLE_ATTRIB_DEBUG_BUFFERING` / `MR_ATTRIB_DEBUG_BUFFERING` for extra attrib history for tools.

---

## Repository layout (top level)

- `common/`: portable C++ libraries (platform, numerics, XML, XMD, scripting, qhull, widgets, etc.)
- `docs/`: `CHANGES.txt`, `UPGRADE.txt`, `KNOWN_ISSUES.txt`
- `morpheme/`: SDK sources, samples, tools, utils
- `README.txt`: original NaturalMotion readme (solution locations, support contacts)

---

## Bundled or referenced third-party style code

| Location | Notes |
|----------|--------|
| `common/tinyxml` | TinyXML-derived XML, NM wrappers (`NMTinyXML.h`, `NMTinyStr.h`, `NMTinyMMIO.h`, `NMTinyFastHeap.h`). |
| `common/qhull` | Qhull; `CMakeLists.txt` present. |
| `common/NMSquirrel` | Squirrel VM and stdlib (`include/squirrel`, `include/sqstdlib`, `src/squirrel`). |
| `common/XMD` | XMD format and `XMU` mesh utilities. |
| `common/NMTL/include/MinHook` | MinHook API for hooking. |
| PhysX 2 / 3 | Integrated in physics SKU; headers under `morpheme/SDK/physics`. SDK not shipped in this tree in full. |
| FBX | Asset compiler `FBX/` plugin; Autodesk SDK expected on build machine. |

---

## `common/NMPlatform/include/NMPlatform` (file index)

Rough roles; see each header for exact API:

| Header | Role |
|--------|------|
| `NMPlatform.h` | Umbrella includes and platform detection. |
| `NMMemory.h`, `NMMemoryAllocator.h`, `NMFreelistMemoryAllocator.h`, `NMLoggingMemoryAllocator.h`, `NMStaticFreeListAllocator.h` | Allocators and memory formats. |
| `NMFastHeapAllocator.h`, `NMFastFreeList.h`, `NMStaticFreeList.h` | Fast small-object heaps and freelists. |
| `NMVector3.h`, `NMVector.h`, `NMQuat.h`, `NMMatrix.h`, `NMMatrix34.h`, `NMNorm.h`, `NMMathUtils.h`, `NMMathPlatform.h` | Math primitives. |
| `NMBuffer.h`, `NMBitArray.h`, `NMBitStreamCoder.h` | Binary buffers and packing. |
| `NMHash.h`, `NMHashMap.h`, `NMMapContainer.h`, `NMStringTable.h`, `NMString.h` | Hashing, maps, strings. |
| `NMHierarchy.h` | Hierarchy utilities. |
| `NMVectorContainer.h` | Vector container helper (used in debug paths in core). |
| `NMTimer.h`, `NMSync.h`, `NMAtomic.h`, `NMSystem.h` | Time and sync primitives. |
| `NMBasicLogger.h`, `NMPrioritiesLogger.h`, `NMProfiler.h` | Logging and simple profiling. |
| `NMDebugDrawManager.h` | Debug draw aggregation. |
| `NMSocket.h`, `NMSocketWrapper.h` | Socket abstraction (NMP socket namespace noted in CHANGES for 5.1). |
| `NMRingBuffer.h` | Ring buffer (API updated in 5.2 per CHANGES). |
| `NMRNG.h` | Random number generation. |
| `NMFile.h`, `NMCommandLineProcessor.h`, `NMKeyboardHarness.h`, `NMPadHarness.h` | File IO, CLI, input test hooks. |
| `NMColour.h`, `NMFlags.h`, `NMSeh.h`, `NMStlUtils.h`, `NMvpu.h` | Misc helpers. |

---

## `common/NMNumerics/include/NMNumerics`

Offline and analysis numerics used in tools and some runtime helpers. Grouped by theme:

**Linear algebra / decomposition**: `NMBandDiagMatrix.h`, `NMBandDiagSolverCholesky.h`, `NMBandDiagSolverLU.h`, `NMBidiagonalizer.h`, `NMGivens.h`, `NMHouseholder.h`, `NMEigenSystemSym.h`, `NMSolverCholesky.h`, `NMSolverLU.h`, `NMSolverQR.h`, `NMSolverTriDiag.h`, `NMSVD.h`, `NMQR.h`, `NMRigidMotionTMJacobian.h`.

**Splines / curves**: `NMCurveBase.h`, `NMBSplineCurve.h`, `NMBSplineSolver.h`, `NMCSplineInterpolator.h`, `NMCSplineSmoother.h`, `NMPosSpline.h`, `NMPosSplineFitterBase.h`, `NMPosSplineFitterTangents.h`, `NMPPolyCurve.h`, `NMPolyLine.h`, `NMQuatSpline.h`, `NMQuatSplineFitterAngleTol.h`, `NMQuatSplineFitterBase.h`, `NMQuatSplineFitterContinuityC0.h`, `NMQuatSplineFitterContinuityC1.h`, `NMQuatSplineFitterInsertKnot.h`, `NMQuatSplineFitterRemoveKnot.h`, `NMQuatSplineFitterSmoothedC1.h`, `NMQuatSplineFitterTangents.h`, `NMSimpleKnotVector.h`, `NMSimpleSplineFittingUtils.h`, `NMSplineUtils.h`, `NMQuatUtils.h`.

**Transforms / geometry**: `NMScrew.h`, `NMSimplexGenerator3D.h`, `NMTriangulation2D.h`, `NMVector3Utils.h`.

**Signal / image**: `NMDiscreteWaveletTransform.h`, `NMDiscreteWaveletTransform2D.h`, `NMImage.h`.

**Stats / fitting**: `NMLinearRegression.h`, `NMMoments2D.h`, `NMNormalDistribution2D.h`, `NMNonLinearOptimiser.h`, `NMNonLinearOptimiserBandDiag.h`, `NMNonLinearOptimiserBase.h`, `NMNonLinearOptimiserCholesky.h`.

**Other**: `NMNumericUtils.h`, `NMUniformQuantisation.h`.

---

## `common/NMRuntimeUtils`

Includes `NMSplineUtils/NMSplinePath.h` and `NMIK/NMHybridIK.h` (inverse kinematics used in runtime tooling paths). Projects: `NMHybridIK_WIN32.sln`, `NMHybridIK_*.vcxproj`.

---

## `common/NMUtils/include/NMutils`

Cross-cutting helpers: `NMTypes.h`, `NMCustomMemory.h`, `MemoryStream.h`, `ClientPlatformIncludes.h`, `NMUtils_utils.h`, `NMUtils_Time.h`, `NMUtils_NMPoint.h`, `NMUtils_NMPoint2D.h`, `NMUtils_NMColourRGBA.h`, `NMUtils_BasicLogger.h`, `NMUtils_XMLWriter.h`, `TypeUtils.h`.

---

## `common/XMD/include/XMU`

Mesh and animation processing: `VertexArray.h`, `Normals.h`, `Tangents.h`, `BlendShapeOptimiser.h`, `SkinPruner.h`, `KeyStripper.h`, `FileSystem.h`, `Upgrading.h`.

---

## `morpheme/SDK/sharedDefines`

| File | Role |
|------|------|
| `mVersion.h` | Version and binary format id. |
| `mSharedDefines.h` | Shared IDs, attrib types, debug switches (see above). |
| `mCoreDebugInterface.h`, `mAnimDebugInterface.h`, `mPhysicsDebugInterface.h`, `mEuphoriaDebugInterface.h` | Debug channel interfaces for tools. |
| `mDebugDrawTessellator.h` | Debug draw tessellation helper. |

---

## Core public headers (`morpheme/SDK/core/include/morpheme/*.h`)

| File | Notes |
|------|--------|
| `mrAnimationSourceHandle.h` | Handle type tying network animation references to loaded sources. |
| `mrAttribAddress.h` | Locates an attrib in the network's attrib store (node, semantic, anim set, lifetime). |
| `mrAttribData.h` | Base `AttribData` and typed subclasses; creation, locate/dislocate; semantic enums. |
| `mrBlendOps.h` | Scalar and SIMD blend paths, feathered blends, partial poses. |
| `mrCharacterControllerAttribData.h` | CC-related attrib payloads. |
| `mrCharacterControllerDef.h` | CC definition baked from assets. |
| `mrCharacterControllerInterface.h` | Runtime CC API used during network update. |
| `mrCharacterControllerInterfaceBase.h` | Shared CC interface pieces. |
| `mrCommonTaskQueuingFns.h` | Shared queuing functions for common attrib semantics. |
| `mrCommonTasks.h` | Task implementations referenced by core task IDs. |
| `mrCompressedDataBufferQuat.h` | Packed quat channels for attrib compression paths. |
| `mrCompressedDataBufferVector3.h` | Packed vector3 channels. |
| `mrCoreTaskIDs.h` | Enumerates built-in `TaskID` values for core tasks. |
| `mrDebugClient.h` | Connect-side debug client hooks. |
| `mrDebugMacros.h` | Asserts and MR-specific debug macros. |
| `mrDebugManager.h` | Central debug state for output-capable builds. |
| `mrDefines.h` | Limits and registration macros. |
| `mrDispatcher.h` | `Dispatcher`, `TaskParameters`, task execution entry points. |
| `mrEventTrackBase.h` | Shared event track infrastructure. |
| `mrEventTrackCurve.h` | Curve-sampled events. |
| `mrEventTrackDiscrete.h` | Instant events. |
| `mrEventTrackDuration.h` | Duration windows; sets and bounds. |
| `mrEventTrackSync.h` | Sync events for locomotion alignment. |
| `mrFootCyclePrediction.h` | Foot cycle prediction helpers. |
| `mrInstanceDebugInterface.h` | Per-network debug interface implementation hooks. |
| `mrJointControlUtilities.h` | Joint-level utilities shared by IK nodes. |
| `mrManager.h` | `MR::Manager` full API. |
| `mrMessage.h` | `Message` struct, types, queues. |
| `mrMessageDistributor.h` | Routes messages to node handlers. |
| `mrMirroredAnimMapping.h` | Mirroring mapping for left/right channel remaps. |
| `mrNetwork.h` | `MR::Network` (large file): update, attrib queries, state machine driver, debug. |
| `mrNetworkControlSerialiser.h` | Serialises control state for inspection or replay tooling. |
| `mrNetworkDef.h` | `MR::NetworkDef`: compiled network, anim sets, state machine defs. |
| `mrNetworkLogger.h` | Optional logging around network execution. |
| `mrNetworkRestorePoint.h` | Save/restore network snapshots for debugging. |
| `mrNode.h` | Node function typedefs (see section above). |
| `mrNodeBin.h` | `NodeBin` storage for active node runtime entries. |
| `mrNodeDef.h` | `NodeDef`: pins, task indices, child counts, node type. |
| `mrNodeParentingMap.h` | Parent indices for node hierarchy in def. |
| `mrNodeTagTable.h` | Tag strings to node ID maps for tooling lookups. |
| `mrPackedArrayUint32.h` | Compact uint32 arrays in defs. |
| `mrRig.h` | `Rig` hierarchy, bind pose, names. |
| `mrRigRetargetMapping.h` | Retarget tables between rigs. |
| `mrRigToAnimMap.h` | Channel index mapping rig to anim. |
| `mrRuntimeNodeInspector.h` | Introspection API for active nodes (debug). |
| `mrSPUDefines.h` | Cell SPU compilation toggles for core. |
| `mrSyncEventPos.h` | Evaluating playback position in sync event space. |
| `mrTask.h` | `Task`, `TaskParameter`, dependency wiring. |
| `mrTaskQueue.h` | Queue of tasks for one update. |
| `mrTaskUtilities.h` | Helpers to build parameters and enqueue. |
| `mrTrajectoryPrediction.h` | Trajectory prediction for transitions and planting. |
| `mrTransitDeadBlend.h` | Dead blend parameters and runtime type. |
| `mrUnevenTerrainIK.h` | Uneven terrain node data shared with IK solve. |
| `mrUnevenTerrainUtilities.h` | Foot placement and height sampling helpers. |
| `mrUtils.h` | Small shared utilities (frame time, validation helpers, etc.). |

### `AnimSource/`

| File | Notes |
|------|--------|
| `mrAnimSource.h` | `AnimSourceBase` interface; section iteration, duration, format id. |
| `mrAnimSourceASA.h`, `mrAnimSourceMBA.h`, `mrAnimSourceNSA.h`, `mrAnimSourceQSA.h` | Four packaged formats (ASA, MBA, NSA, QSA). |
| `mrAnimSectionASA.h`, `mrAnimSectionMBA.h`, `mrAnimSectionNSA.h`, `mrAnimSectionQSA.h` | Section chunks per format. |
| `mrTrajectorySourceBase.h`, `mrTrajectorySourceASA.h`, `mrTrajectorySourceMBA.h`, `mrTrajectorySourceNSA.h`, `mrTrajectorySourceQSA.h` | Root motion trajectories per format. |
| `mrChannelQuat.h`, `mrChannelRotVecQuantised.h`, `mrChannelPos.h`, `mrChannelPosQuantised.h` | Channel layouts. |
| `mrAnimSourceUtils.h` | Shared decoding and evaluation helpers. |

### `TransitConditions/`

Implements `TransitConditType` handlers: base `mrTransitCondition.h`, false, message, node active, sync and duration windows, discrete and curve crossings, fraction of duration, ray hit, and the full set of control-parameter comparisons (float, int, uint, bool, ranges). File names match the condition types in Connect.

### `Prediction/`

| File | Notes |
|------|--------|
| `mrNetworkPredictionDef.h` | Baked prediction model references inside `NetworkDef`. |
| `mrPredictionModelNDMesh.h` | ND mesh model evaluation. |
| `mrNDMesh.h`, `mrNDMeshQueryData.h`, `mrNDMeshAPResampleMap.h`, `mrNDMeshAPSearchMap.h` | Mesh data and acceleration structures. |
| `mrScatteredDataUtils.h` | Interpolation over scattered samples. |

### `Nodes/`

Beyond `mrNodes.h`, this folder is one header per node type or helper: all blend variants (2, 2x2, N, NxM, all, feather, subtractive, scatter 1D/2D with sync-event variants), control parameter nodes (bool, int, uint, float, vector3, vector4), IK (two-bone, lock foot, head look, hips, gun aim), transforms (filter, mirror, smooth, scale character, retarget, modify joint, modify trajectory), state machine container (`mrNodeStateMachine.h`), transit (`mrNodeTransit`, sync variants), sequence, switch, single frame, freeze, closest anim, uneven terrain, character controller override, operators listed earlier, utilities `mrSharedNodeFunctions.h`, `mrEmittedControlParamNodeUtils.h`, scatter projection utilities. Physics-specific node headers live under the physics SDK, not here.

---

## Physics SDK (`morpheme/SDK/physics/include/physics/`)

Core: `mrPhysics.h`, `mrPhysicsScene.h`, `mrPhysicsRig.h`, `mrPhysicsRigDef.h`, `mrPhysicsAttribData.h`, `mrPhysicsTasks.h`, `mrPhysicsSerialisationBuffer.h`, `mrCCOverrideBasics.h`.

PhysX 2 subtree: `PhysX2/mrPhysX2*.h`, scene, rig, driver data, character controller interface.

PhysX 3 subtree: `PhysX3/mrPhysX3*.h`, jointed and articulation rigs, scene, driver data, CC interface, `mrPhysX3Deprecated.h` for migration notes.

`Nodes/`: physics grouper, transit, impulses, joint limits, non-physics transform set, control param physics object pointer, SK deviation transit condition, `mrPhysicsNodes.h` aggregate include.

---

## Euphoria (`morpheme/SDK/euphoriaCore/include/euphoria/`)

Character and body: `erCharacter.h`, `erCharacterDef.h`, `erBody.h`, `erBodyDef.h`, `erBehaviour.h`, `erModule.h`, `erNetworkInterface.h`.

Limbs: `erLimb.h`, `erLimbDef.h`, `erLimbIK.h`, `erLimbInterface.h`, `erLimbTransforms.h`.

Interaction and motion: `erInteractionProxy.h`, `erInteractionProxyDef.h`, `erJunction.h`, `erPath.h`, `erPinInterface.h`, `erRigConstraint.h`, `erEndConstraint.h`, `erReachLimit.h`.

Collision and hits: `erCollisionProbes.h`, `erContactFeedback.h`, `erHitReaction.h`, `erHitUtils.h`.

Scaling and gravity: `erDimensionalScaling.h`, `erDimensionalScalingHelpers.h`, `erGravityCompensation.h`.

Support: `erDefines.h`, `erSharedEnums.h`, `erAttribData.h`, `erDebugDraw.h`, `erDebugControls.h`, `erEuphoriaLogger.h`, `erEuphoriaUserData.h`, `erValueValidators.h`, `erValuePostCombiners.h`, `erSPU.h`, `erSPUDefines.h`.

`Nodes/`: `erNodes.h`, behaviour grouper, limb info, performance behaviour, trajectory override, operators (contact, hit, fall over wall, roll down stairs, orientation in free fall, physical constraint, etc.).

`morpheme/SDK/euphoriaCoreBehaviours/` contains generated behaviour API headers (`AutoGenerated/`) and `Implementation/Helpers/` (Aim, Step, StandingSupport, Spin, etc.).

---

## Asset processor (`morpheme/SDK/assetProcessor/include/assetProcessor/`)

Builders mirroring runtime: `AssetProcessor.h`, `AnalysisProcessor.h`, `AnimationPoseBuilder.h`, `NetworkDefBuilder.h`, `NodeBuilder.h`, `NodeBuilderUtils.h`, `BlendNodeBuilderUtils.h`, `TransitConditionDefBuilder.h`, `TransitDeadBlendDefBuilder.h`, `NodeTransitBaseBuilder.h`, `MessageBuilder.h`, `PredictionModelBuilder.h`, `AssetProcessorUtils.h`, `acAnimInfo.h`.

`AnimSource/` mirrors compression and channel pipeline: `AnimSourceCompressor*.h`, `AnimSectionCompressor*.h`, `Channel*Builder.h`, `QuantisationSetQSA.h`, `TrajectorySourceCompressor*.h`, `TransformsAccumulator.h`, `Vector3QuantisationTableBuilder.h`, and related QSA/NSA/ASA paths.

---

## Export (`morpheme/SDK/export/include/export/`)

Read Connect-exported data: `mcExport.h`, `mcExportXml.h`, `mcExportBody.h`, `mcExportBodyXml.h`, `mcExportControllerXml.h`, `mcExportPhysics.h`, `mcExportPhysicsXml.h`, `mcExportInteractionProxyXml.h`, `mcExportMessagePresets.h`, `mcExportMessagePresetsXml.h`, `mcAnimInfo.h`, `mcAnimInfoXml.h`, `mcXML.h`, `apExport.h`, `apExportLUA.h`.

---

## MDF (`morpheme/SDK/MDF/`)

Parser for Morpheme Definition Format: `ParserEngine.h`, `ParserBase.h`, `ParserDefs.h`, `ParserMemory.h`, `MDFModuleGrammar.h`, `MDFModuleDefs.h`, `MDFModuleAnalysis.h`, `MDFTypesGrammar.h`, `MDFTypesDefs.h`, `MDFTypesAnalysis.h`, `PString.h`, `StringBuilder.h`, `Utils.h`, `MDFPrecompiled.h`.

---

## Tools (`morpheme/tools/`)

| Directory | Role |
|-----------|------|
| `assetCompiler` | Main offline compiler EXEs per SKU (NoPhysics, PhysX2, PhysX3, Euphoria). Subfolders: `Core/include/core` node builders, `Physics/include/Physics`, `Plugins/Euphoria`, `Plugins/PhysX2`, `Plugins/PhysX3`, `FBX`, configs copied post-build per CHANGES. |
| `assetExporter` | C# Morpheme Asset Exporter solution. |
| `MDFCodeGen` | `MDFCodeGen.cpp`, `EmitCPP_Module.cpp`, `EmitCPP_Desc.cpp`, `EmitCPP_Common.cpp`, `EmitCPP_Types.cpp`, `EmitXmlSummary.cpp`, `CodeWriter.cpp`, `FileUtils.cpp`, `NetworkStats.cpp`, Windows UI resource. |
| `NodeWizard` | C# wizard for new nodes. |
| `runtimeTarget` | Reference host: PhysX variants, Euphoria target, NoPhysics; pairs with tutorials. |
| `scriptedRuntime` | `scriptedRuntimeApp`, `srScripting`, `srScene`, `srNetwork`, `srEnvironment`. |
| `behaviourTuner` | `GA.cpp`, `Configuration.cpp` for parameter search. |
| `debugging` | Visual Studio macro files for studio-side debugging. |

---

## Utils (`morpheme/utils/`)

**comms2** (`include/comms/`): `mcomms.h` umbrella; `commsServer.h`, `commsServerModule.h`, `coreCommsServerModule.h`, `connection.h`, `connectionManager.h`, `simpleConnectionManager.h`, `packet.h`, `corePackets.h`, `debugPackets.h`, `assetManagerPackets.h`, `commandsHandler.h`, `coreCommandsHandler.h`, `assetManagerCommandsHandler.h`, `commsDebugClient.h`, `runtimeTargetInterface.h`, `runtimeTargetNull.h`, `morphemeCommsTarget.h`, `attribute.h`, `attributeHelpers.h`, `networkDataBuffer.h`, `networkManagementUtils.h`, `liveLinkDataManager.h`, `liveLinkNetworkManager.h`, `liveLinkSceneObjectManager.h`, `sceneObject.h`, `simpleDataManager.h`, `simpleEnvironmentManager.h`, `simpleAnimBrowserManager.h`, `simpleBundleUtils.h`, `debugDrawObjects.h`, `scopedMemory.h`.

**physicsComms2**, **euphoriaComms2**: same pattern with physics or euphoria command sets.

**simpleBundle** (`include/simpleBundle/`): `simpleBundle.h` defines `SimpleBundleHeader` (version, platform fmt, `AssetType`, `RuntimeAssetID`, guid, `NMP::Memory::Format` for payload), `SimpleBundleWriter`, reader side in same header family; `simpleAnimRegistry.h`, `simpleAnimRuntimeIDtoFilenameLookup.h`, `simpleAnimRuntimeIDtoFilenameLookupBuilder.h` map runtime anim IDs to files when loading from bundles.

**gameManagement**: sample solutions for managing characters in larger apps.

---

## Samples (`morpheme/samples/tutorials`)

Multiple `.sln` files for VS2008 and newer Win32/X64, split by physics backend (NoPhysics, PhysX2, PhysX3, Euphoria). Under `WIN/`, per-tutorial vcxproj folders (for example `RuntimeSimple_Euphoria`). Use the solution that matches your linked middleware.

---

## Hooking (`morpheme/hooking/`)

`morpheneConnect_launcher.sln` and related projects for launching Connect-related tooling.

---

## Legal and support

Copyright NaturalMotion; see per-file headers. Support: https://support.naturalmotion.com/

For authoritative behaviour of any function, read the `.h` and `.cpp` with the same base name under `morpheme/SDK` or `common` as appropriate.
