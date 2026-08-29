# Asset Exporter Documentation
The Asset Exporter is a tool that facilitates batch processing of morpheme networks. The user can create a list of networks in a .lst file, and create a configuration file in a .cfg file to specify where the networks are exported.


# Config Files
The user should begin by opening the sample config (Samples/SampleConfig.cfg) via the Open Config button. This can then be modified to suit the user's project's requirements, and saved out as a new config file.

The Edit Config Macros... button allows the user to create new macros that can be used as environment variables to give a level of indirection to allow different users to use the same macro, but make that macro point to different local paths.

The macros also allow the user to process networks to paths based on the name or directory of the network. For example, by adding $(MCN_NAME) to the "Process sub path" in the Platforms table will process each network into a directory with the network's name.

- "Runtime SDK root" should point to where the morpheme runtime is installed.

- "Export root" should point to where the root directory of where the mcns will be exported.

- "Process sub path" is a per asset compiler setting and should point to where, within the "Export root" the network should be processed. For example, Win32/$(MCN_NAME).

- “Asset Compiler Executable” points to the asset compiler. In versions precedent to 3.6 the name of the executable fully identifies the platform, the physics engine and additional components like Move or Kinect (eg: win32/vc9/debug/morphemeAssetCompiler_Kinect_PhysX2_target_pc.exe is the asset compiler for morpheme 3.0.x, platform pc-debug with PhysX2 and Kinect support). The asset compiler can be selected either by typing the executable name in the text box of double clicking on the box to open a file browser.

The user can disable each asset compiler for testing purposes via the "Use" check box. This is not saved out in the config file. Unwanted asset compilers can be removed using the context menu.


# Network List Files
Network files (.mcns) can be added to the MCN List table once a list file has been created via the Create List button.  The user will see the "Add networks" dialog appear immediately after creating the list file. .mcn files can be multi-selected from this dialog or drag and dropped into the application.

A context menu allows the user to remove network files from the list.

The “Asset Compiler” column allows the user to specify which asset compiler to use for a specific network. This is useful to process non-physics networks together with physx2, physx3, etc networks in the same job. This feature is not applicable to versions precedent to morpheme 3.6, where the asset compiler executable name contains the necessary information to identify the platform, the physics engine, etc. Morpheme 3.6 introduces the ‘dynamic asset compiler plugins’ system. With this new system, only one executable per platform and per configuration is necessary (example: morphemeAssetCompiler_target_pc_debug.exe), while all the extra components are managed by dynamically loaded plugins (example: acPluginPhysX2, acPluginKinect) that are specified on the command line or in the asset compiler’s configuration file.


# Export and Process
Once the config has been setup and the network list has been generated then the user can press the Export and Process button. The morpheme networks will be batch exported via connect in no-gui mode, and will then be processed in parallel with the Asset Compilers setup in the Platforms table in the config. 

The Asset Exporter tool can be run from the command line. See Samples/ SampleCommandLineExport.bat.