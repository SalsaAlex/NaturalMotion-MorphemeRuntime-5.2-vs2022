one should compile the behaviour definitions with MDFCodeGen, this will generate the C++ code required (modules, behaviours and types) for compiling euphoriaCoreBehaviours and will also generate the lua scripts for the behaviour nodes that shall be used in morphemeConnect.

observations:

behaviours dont hold any logic, but rather serve simply as a controller. they only describe attributes, inputs, control parameters, outputs.. and enable a group of modules. all the input processing and output generation is defined in the "Implementation" folder. This folder implements the logic for all modules and types.
example: if you want to implement a behaviour, you should make two entries for it in "Definition/Behaviours": 
-the behaviour (will compile into ER::Behaviour) and the interface (will compile into ER::Module).
in short, the behaviour holds the data, and the interface (module) processes the ins and outs.

example:

	-ArmsWindmill.behaviour

		--------
		|behaviour_def ArmsWindmill
		|{
		|interface:
		|	version = 5;
		|
		|modules: ///list modules to enable
		|	armsWindmillBehaviourInterface;
		|	upperBody;
		|	arms[*].spin; /// "*" basically means "enable the "spin" module of all arms".
		|	
		|attributes:
		|	//attributes are defined in "groups" just so they can be separated in groups on the lua connect node manifest of this node.
		|	group("Spin")
		|	{
		|		/// 	min and max are optional, one can simply only define the min value or none at all.
		|		///
		|		/// 				default		min			max
		|		float MaxAngSpeed = 3.0f, 		0.0f, 		5.0f     __per_anim_set__  __title__("MaximumAngularSpeed"); 
		|		
		|		float MaxRadius = 	0.6f, 		0.0f, 		2.0f        __per_anim_set__  __title__("MaximumRadius"); 
		|		
		|		bool Synchronised = true                  __per_anim_set__ __title__("SynchroniseArms");
		|	}
		|
		|
		|controlParams:
		|	{
		|		vector3 TargetRotationDelta = (0.0f, 0.0f, 90.0f) __title__("Rotation");
		|		float   RotationTime = 0.5f, 0.0f, 5.0f;
		|		loat  ImportanceForArm[networkMaxNumArms] = 1.0f, 0.0f, 1.0f __title__("Weight");
		|	}
		|}
		--------