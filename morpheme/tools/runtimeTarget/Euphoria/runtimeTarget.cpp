// Copyright (c) 2011 NaturalMotion.  All Rights Reserved.
// Not to be copied, adapted, modified, used, distributed, sold,
// licensed or commercially exploited in any manner without the
// written consent of NaturalMotion.
//
// All non public elements of this software are the confidential
// information of NaturalMotion and may not be disclosed to any
// person nor used for any purpose not expressly approved by
// NaturalMotion in writing.

//----------------------------------------------------------------------------------------------------------------------
#include "NMPlatform/NMPlatform.h"
#include "NMPlatform/NMSocket.h"
#include "NMPlatform/NMTimer.h"
#include "NMPlatform/NMCommandLineProcessor.h"

#include "morpheme/mrDebugManager.h"

#include "comms/commsDebugClient.h"
#include "comms/commsServer.h"
#include "comms/euphoriaCommsServerModule.h"
#include "comms/physicsCommsServerModule.h"
#include "comms/runtimeTargetNull.h"
#include "comms/simpleAnimBrowserManager.h"
#include "comms/simpleConnectionManager.h"
#include "comms/simpleEnvironmentManager.h"

#include "defaultDataManager.h"
#include "defaultPhysicsManipMgr.h"
#include "defaultNetworkManager.h"
#include "defaultSceneObjectMgr.h"
#include "networkDefRecordManager.h"
#include "networkInstanceRecordManager.h"
#include "runtimeTargetContext.h"
#include "sceneObjectRecordManager.h"

#include "erLiveLinkData.h"

#include "defaultERAssetMgr.h"
#include "defaultERPhysicsMgr.h"
#include "defaultERPhysicsDataMgr.h"
#include "defaultEuphoriaDataMgr.h"
#include "runtimeTargetLogger.h"

#if defined(NM_HOST_X86LINUX)
  #include <sys/time.h>
  #include <sys/timer.h>
#elif defined(NM_HOST_CELL_PPU)
  #include <cell/sysmodule.h>
  #include <sys/timer.h>
#endif

#ifdef NM_HOST_CELL_PPU
  #include "euphoria/erSPU.h"
  #include "morpheme/mrDispatcherPS3.h"
#endif

#ifdef NM_HOST_ANDROID
  #include <android_native_app_glue.h>
#endif // NM_HOST_ANDROID

//enable for help in tweaking jolt physics for euphoria
//#define JOLTPHYS_TWEAKING

//----------------------------------------------------------------------------------------------------------------------
static void pause(float fps)
{
  static NMP::Timer loopTimer(true);

  float frameMaxTime = 1000.0f / fps;
  float elapsedTime = loopTimer.stop();

#if defined(NM_HOST_WIN32) || defined(NM_HOST_WIN64) || defined(NM_HOST_X360)
  // Use a time accumulation approach so that we can smooth the inaccuracies caused by the usage of the Sleep(msecs)
  // command. In average the frame rate should be quite accurate.
  static float accumulatedTime = 0;
  if (elapsedTime < frameMaxTime)
  {
    uint32_t sleepTime = (uint32_t)(accumulatedTime + frameMaxTime - elapsedTime);
    accumulatedTime = NMP::maximum(frameMaxTime - elapsedTime - sleepTime, 0.0f);
    NMP_ASSERT(accumulatedTime >= 0.0f);
    if (sleepTime > 0)
    {
      Sleep(sleepTime);
    }
  }
  else
  {
    accumulatedTime = 0;
  }
#elif defined(NM_HOST_CELL_PPU) || defined(NM_HOST_X86LINUX)
  if (elapsedTime < frameMaxTime)
  {
    uint32_t sleepTimeMicroSec = (uint32_t)(1000.0f * (frameMaxTime - elapsedTime));
    if (sleepTimeMicroSec > 0)
    {
  #if defined(NM_HOST_CELL_PPU)
      sys_timer_usleep(sleepTimeMicroSec);
  #else
  usleep(sleepTimeMicroSec);
  #endif
    }
  }
#else
  (void)fps;
  (void)elapsedTime;
  (void)frameMaxTime;
#endif

  loopTimer.start();
}

//----------------------------------------------------------------------------------------------------------------------
static void update(
  DefaultPhysicsMgr& physicsMgr,
  DefaultDataManager& dataManager,
  MCOMMS::NetworkManagementInterface& NMP_UNUSED(networkMgr))
{
  MCOMMS::CommsServer* const commsServer = MCOMMS::CommsServer::getInstance();

  if (commsServer)
  {
    NMP_MSG("//");
    NMP_MSG("//pre-commsserver update");
    NMP_MSG("//");
    if (commsServer->update())
    {
      commsServer->scheduleStep(true);
    }
    NMP_MSG("//");
    NMP_MSG("//post-commsserver update");
    NMP_MSG("//");
  }

  if (dataManager.isSimulating())
  {
    // Update frame index, get step interval and wait for a little while.
    int32_t frameIndex = dataManager.getFrameIndex();
    dataManager.setFrameIndex(frameIndex + 1);

    const float delta = dataManager.getStepInterval();
    const float fps = dataManager.getFrameRate();

    pause(fps);

    NMP_MSG("//");
    NMP_MSG("//pre-begin frame");
    NMP_MSG("//");

    if (commsServer)
    {
      commsServer->beginFrame(delta);
    }

    NMP_MSG("//");
    NMP_MSG("pre-physics update");
    NMP_MSG("//");

    // Update all our network instances
    physicsMgr.update(delta);

    NMP_MSG("//");
    NMP_MSG("//pre-end frame update");
    NMP_MSG("//");

    if (commsServer)
    {
      commsServer->endFrame(delta);
    }

    NMP_MSG("//");
    NMP_MSG("//post-end frame update");
    NMP_MSG("//");
  }
}

//----------------------------------------------------------------------------------------------------------------------
class DefaultPlaybackMgr : public MCOMMS::PlaybackManagementInterface
{
  virtual bool canStepInSynch() const NM_OVERRIDE
  {
    return true;
  }

  virtual bool canPause() const NM_OVERRIDE
  {
    return true;
  }

  virtual bool canChangeDeltaTime() const NM_OVERRIDE
  {
    return true;
  }
};

//----------------------------------------------------------------------------------------------------------------------
static void initMorphemeRegistration(DefaultPhysicsMgr& physicsManager)
{
  // Dispatchers
  uint32_t numDispatchers = 0;
  MR::Dispatcher* dispatchers[2];

  dispatchers[numDispatchers] = MR::DispatcherBasic::createAndInit();
  ++numDispatchers;
#ifdef NM_HOST_CELL_PPU
  dispatchers[numDispatchers] = MR::DispatcherPS3::createAndInit();
  ++numDispatchers;
#endif // NM_HOST_CELL_PPU

  // Initialise the physics core by registering the task queueing functions,
  // attribute data and the task functions with the dispatchers.
  physicsManager.initializePhysicsCore(numDispatchers, dispatchers);

  // Finalize semantic and core task registration
  MR::Manager::getInstance().allocateRegistry();
  for (uint32_t i = 0; i < numDispatchers; ++i)
    MR::CoreTaskIDs::registerNMCoreTasks(dispatchers[i]);

  MR::Manager::getInstance().finaliseInitMorphemeLib();
  physicsManager.finaliseInitPhysicsCore();

  // Release the dispatcher instances since we have finished initialising the static function tables
  for (uint32_t i = 0; i < numDispatchers; ++i)
    dispatchers[i]->releaseAndDestroy();
}

//----------------------------------------------------------------------------------------------------------------------
class PluginValidator : public MCOMMS::IPluginValidator
{
public:
  virtual bool validatePluginList(const NMP::OrderedStringTable& pluginList) NM_OVERRIDE;

  NM_INLINE void setPhysicsInterface(IPhysicsMgr* physicsManager);

private:
  IPhysicsMgr* m_physicsManager;
};

//----------------------------------------------------------------------------------------------------------------------
void PluginValidator::setPhysicsInterface(IPhysicsMgr* physicsManager)
{
  m_physicsManager = physicsManager;
}

//----------------------------------------------------------------------------------------------------------------------
bool PluginValidator::validatePluginList(const NMP::OrderedStringTable& pluginList)
{
  if (!m_physicsManager->validatePluginList(pluginList))
  {
    return false;
  }

  const char* euphoriaPlugin = "acPluginEuphoria_target_" NM_PLATFORM_FORMAT_STRING;
  const char* debugEuphoriaPlugin = "acPluginEuphoria_target_" NM_PLATFORM_FORMAT_STRING"_debug";

  if (pluginList.getNumEntries() >= 2)
  {
    const char* pluginListEntry = pluginList.getEntryString(1);
    if ((NMP_STRCMP(pluginListEntry, euphoriaPlugin) == 0 || NMP_STRCMP(pluginListEntry, debugEuphoriaPlugin) == 0))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  return true;
}

#ifdef JOLTPHYS_TWEAKING
#define ID_SLIDER_1 1001
#define ID_SLIDER_2 1002
#define ID_SLIDER_3 1003
#define ID_SLIDER_4 1004
#define ID_SLIDER_5 1005
#define ID_SLIDER_6 1006
HWND g_slider1;
HWND g_slider2;
HWND g_slider3;
HWND g_slider4;
HWND g_slider5;
HWND g_slider6;

extern float JPH_JOINTSTRENGTH_SCALE;
extern float JPH_JOINTDAMP_SCALE;
extern float JPH_JOINTDRIVESTRENGTH_SCALE;
extern float JPH_JOINTDRIVEDAMPING_SCALE;
extern float JPH_JOINTDRIVECOMPENSATION_SCALE;
extern float JPH_EXTERNALCOMPLIANCE_SCALE;

#pragma comment(lib, "comctl32.lib")
#include <commctrl.h>
LRESULT CALLBACK WindowProc(
    HWND hwnd,
    UINT uMsg,
    WPARAM wParam,
    LPARAM lParam)
{
    switch (uMsg)
    {
    case WM_CREATE:
    {
        // Slider 1

         CreateWindowEx(
            0,
            L"STATIC",
            L"jointstrength_scale",
            WS_CHILD | WS_VISIBLE,
            20, 0,
            150, 20,
            hwnd,
            NULL,
            GetModuleHandle(NULL),
            NULL
        );
        g_slider1 = CreateWindowEx(
            0,
            TRACKBAR_CLASS,
            NULL,
            WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
            20, 30, 
            300, 20,
            hwnd,
            (HMENU)ID_SLIDER_1,
            GetModuleHandle(NULL),
            NULL
        );

        // Slider 2
        CreateWindowEx(
            0,
            L"STATIC",
            L"jointdamp_scale",
            WS_CHILD | WS_VISIBLE,
            20, 60,
            150, 20,
            hwnd,
            NULL,
            GetModuleHandle(NULL),
            NULL
        );
        g_slider2 = CreateWindowEx(
            0,
            TRACKBAR_CLASS,
            NULL,
            WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
            20, 90, 
            300, 20,
            hwnd,
            (HMENU)ID_SLIDER_2,
            GetModuleHandle(NULL),
            NULL
        );

        // Slider 3
        CreateWindowEx(
            0,
            L"STATIC",
            L"jointdrivestrength_scale",
            WS_CHILD | WS_VISIBLE,
            20, 120,
            150, 20,
            hwnd,
            NULL,
            GetModuleHandle(NULL),
            NULL
        );
        g_slider3 = CreateWindowEx(
            0,
            TRACKBAR_CLASS,
            NULL,
            WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
            20, 150, 
            300, 20,
            hwnd,
            (HMENU)ID_SLIDER_3,
            GetModuleHandle(NULL),
            NULL
        );
        // Slider 4
        CreateWindowEx(
            0,
            L"STATIC",
            L"jointdrivedamp_scale",
            WS_CHILD | WS_VISIBLE,
            20, 180,
            150, 20,
            hwnd,
            NULL,
            GetModuleHandle(NULL),
            NULL
        );
        g_slider4 = CreateWindowEx(
            0,
            TRACKBAR_CLASS,
            NULL,
            WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
            20, 210, 
            300, 20,
            hwnd,
            (HMENU)ID_SLIDER_4,
            GetModuleHandle(NULL),
            NULL
        );
        // Slider 5
        CreateWindowEx(
            0,
            L"STATIC",
            L"jointdrivecompensation_scale",
            WS_CHILD | WS_VISIBLE,
            20, 240,
            150, 20,
            hwnd,
            NULL,
            GetModuleHandle(NULL),
            NULL
        );
        g_slider5 = CreateWindowEx(
            0,
            TRACKBAR_CLASS,
            NULL,
            WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
            20, 270, 
            300, 20,
            hwnd,
            (HMENU)ID_SLIDER_5,
            GetModuleHandle(NULL),
            NULL
        );
        // Slider 6
        CreateWindowEx(
            0,
            L"STATIC",
            L"externalcompliance_scale",
            WS_CHILD | WS_VISIBLE,
            20, 300,
            150, 20,
            hwnd,
            NULL,
            GetModuleHandle(NULL),
            NULL
        );
        g_slider6 = CreateWindowEx(
            0,
            TRACKBAR_CLASS,
            NULL,
            WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
            20, 330,
            300, 20,
            hwnd,
            (HMENU)ID_SLIDER_6,
            GetModuleHandle(NULL),
            NULL
        );

        // Set ranges: 0 -> 100
        SendMessage(g_slider1, TBM_SETRANGE, TRUE, MAKELPARAM(0, 100));
        SendMessage(g_slider2, TBM_SETRANGE, TRUE, MAKELPARAM(0, 100));
        SendMessage(g_slider3, TBM_SETRANGE, TRUE, MAKELPARAM(0, 100));
        SendMessage(g_slider4, TBM_SETRANGE, TRUE, MAKELPARAM(0, 100));
        SendMessage(g_slider5, TBM_SETRANGE, TRUE, MAKELPARAM(0, 100));
        SendMessage(g_slider6, TBM_SETRANGE, TRUE, MAKELPARAM(0, 100));

        // Initial positions
        SendMessage(g_slider1, TBM_SETPOS, TRUE, JPH_JOINTSTRENGTH_SCALE * 100);
        SendMessage(g_slider2, TBM_SETPOS, TRUE, JPH_JOINTDAMP_SCALE * 100);
        SendMessage(g_slider3, TBM_SETPOS, TRUE, JPH_JOINTDRIVESTRENGTH_SCALE * 100);
        SendMessage(g_slider4, TBM_SETPOS, TRUE, JPH_JOINTDRIVEDAMPING_SCALE * 100);
        SendMessage(g_slider5, TBM_SETPOS, TRUE, JPH_JOINTDRIVECOMPENSATION_SCALE * 100);
        SendMessage(g_slider6, TBM_SETPOS, TRUE, JPH_EXTERNALCOMPLIANCE_SCALE * 100);

        break;
    }

    case WM_HSCROLL:
    {
        // A slider was moved
        HWND slider = (HWND)lParam;
        float value = SendMessage(slider, TBM_GETPOS, 0, 0);
        if (value != 0)
            value /= 100;

       if (slider == g_slider1)
       {
           JPH_JOINTSTRENGTH_SCALE = value;
       }
       else if (slider == g_slider2)
       {
           JPH_JOINTDAMP_SCALE = value;
       }
       else if (slider == g_slider3)
       {
           JPH_JOINTDRIVESTRENGTH_SCALE = value;
       }
       else if (slider == g_slider4)
       {
           JPH_JOINTDRIVEDAMPING_SCALE = value;
       }
       else if (slider == g_slider5)
       {
           JPH_JOINTDRIVECOMPENSATION_SCALE = value;
       }
       else if (slider == g_slider6)
       {
           JPH_EXTERNALCOMPLIANCE_SCALE = value;
       }
    }

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }

    return DefWindowProc(hwnd, uMsg, wParam, lParam);
}
static void create_tweaking_window()
{
    INITCOMMONCONTROLSEX icc = {};
    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_BAR_CLASSES;
    InitCommonControlsEx(&icc);

    WNDCLASS wc = {};
    wc.lpfnWndProc = WindowProc;
    wc.hInstance = GetModuleHandle(NULL);
    wc.lpszClassName = L"MyWindowClass";
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);

    RegisterClass(&wc);

    HWND hwnd = CreateWindowEx(
        0,
        L"MyWindowClass",
        L"Slider Example",
        WS_OVERLAPPEDWINDOW,
        CW_USEDEFAULT, CW_USEDEFAULT,
        500, 400,
        NULL,
        NULL,
        GetModuleHandle(NULL),
        NULL
    );

    ShowWindow(hwnd, 1);
}

#endif

//----------------------------------------------------------------------------------------------------------------------
static int runtimeTarget(const NMP::CommandLineProcessor& commandLineArguments)
{
  // Initialize runtime target components.
  static const uint32_t MAX_SIMULATANEOUS_CONNECTIONS = MCOMMS::ConnectionManager::MAX_NUMBER_CONNECTIONS;
  static const uint32_t MAX_ENVIRONMENT_ATTRIBUTES = 40;

  PluginValidator pluginValidator;

  NetworkDefRecordManager networkDefRecordManager;
  NetworkInstanceRecordManager networkInstanceRecordManager;
  SceneObjectRecordManager sceneObjectRecordManager;
  RuntimeTargetContext context(&networkDefRecordManager, &networkInstanceRecordManager, &sceneObjectRecordManager);

  MCOMMS::SimpleConnectionManager connectionManager;
  DefaultNetworkManager networkManager(&context);
  DefaultERAssetMgr assetManager(&context, &pluginValidator);

  // The animation browser depends on MR_ATTRIB_DEBUG_BUFFERING being enabled so if it is not enabled
  // we must fall back to the null implementation. By default this is only disabled in profile builds.
#if defined(MR_ATTRIB_DEBUG_BUFFERING)
  MCOMMS::SimpleAnimBrowserManager animBrowserMgr(MAX_SIMULATANEOUS_CONNECTIONS, RTT_LOGGER, RTT_MESSAGE_PRIORITY, NULL);
#else
  MCOMMS::AnimationBrowserNull animBrowserMgr;
#endif

  MCOMMS::SimpleEnvironmentManager environmentManagment(MAX_ENVIRONMENT_ATTRIBUTES);

  DefaultERPhysicsMgr physicsManager(&context, &assetManager, commandLineArguments);
  DefaultDataManager dataManager(&context, &assetManager, &physicsManager);
  DefaultPhysicsDataManager physicsDataManager(&context, &physicsManager);
  DefaultEuphoriaDataManager euphoriaDataManager(&context);
  DefaultSceneObjectMgr sceneObjectManager(&context, &physicsManager);
  DefaultPhysicsManipMgr physicsManipManager(&physicsManager);

  DefaultPlaybackMgr playbackManager;

  pluginValidator.setPhysicsInterface(&physicsManager);

  initMorphemeRegistration(physicsManager);

  // Start COMMS server.
  MCOMMS::RuntimeTargetInterface* const runtimeTarget = new MCOMMS::RuntimeTargetInterface(
    &connectionManager,
    &animBrowserMgr,
    &networkManager,
    &assetManager,
    &assetManager,
    &environmentManagment,
    &sceneObjectManager,
    &dataManager,
    &physicsDataManager,
    &euphoriaDataManager,
    &playbackManager,
    &physicsManipManager);

  // Setup COMMS server.
  uint32_t port = 4567;
  commandLineArguments.getOptionValue("-port", &port);
  MCOMMS::CommsServer* const commsServer = new MCOMMS::CommsServer(runtimeTarget, port);

  MCOMMS::PhysicsCommsServerModule physicsCommsServerModule;
  commsServer->registerModule(&physicsCommsServerModule);

  MCOMMS::EuphoriaCommsServerModule euphoriaCommsServerModule;
  commsServer->registerModule(&euphoriaCommsServerModule);

  // Add the COMMS debug client so debug data is sent back to connect.
  MR::DebugManager* debugManager = MR::DebugManager::getInstance();
  MCOMMS::CommsDebugClient debugClient(commsServer);
  debugManager->addDebugClient(&debugClient);

  {
    if (!commsServer->startServer(true))
    {
      return EXIT_FAILURE;
    }

    NMP_ASSERT(MCOMMS::CommsServer::getInstance());

    // Let the outside world know the IP address that they can connect to us on.
    const char* const networkAddress = NMP::GetNetworkAddress();
    NMP_MSG("... COMMS initialized on IP %s:%i ...\n", networkAddress, port);
  }

  bool autoKillTarget = false;
  commandLineArguments.getOptionValue("-autoKillTarget", &autoKillTarget);

#ifdef JOLTPHYS_TWEAKING
  create_tweaking_window();
#endif

  bool finished = false;
  while (!finished)
  {
    debugManager->clearDebugDraw();

    update(physicsManager, dataManager, networkManager);

    finished = (connectionManager.connectionsReducedToZero() && autoKillTarget);

#ifdef JOLTPHYS_TWEAKING
    MSG msg;
    while(PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
    {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
#endif
  }

  // Remove the COMMS debug client.
  debugManager->removeDebugClient(&debugClient);

  // Shutdown COMMS server.
  commsServer->stopServer();

  commsServer->unregisterModule(&euphoriaCommsServerModule);
  commsServer->unregisterModule(&physicsCommsServerModule);

  delete commsServer;

#ifdef NM_HOST_CELL_PPU
  ER::SPU::term();
#endif // NM_HOST_CELL_PPU

  // ...
  return EXIT_SUCCESS;
}

//----------------------------------------------------------------------------------------------------------------------
#ifdef NM_HOST_CUSTOM_ENTRY_POINT
int NMEntryPoint(
#else
int main(
#endif
  int    argc,
  char** argv)
{
#ifdef NM_HOST_CELL_PPU
  // PS3 Specific initialization
  int moduleLoadResult = cellSysmoduleLoadModule(CELL_SYSMODULE_SYSUTIL);
  NMP_ASSERT_MSG(moduleLoadResult == CELL_OK, "Failed to load module SYSUTIL.");
  moduleLoadResult = cellSysmoduleLoadModule(CELL_SYSMODULE_IO);
  NMP_ASSERT_MSG(moduleLoadResult == CELL_OK, "Failed to load module IO.");
  moduleLoadResult = cellSysmoduleLoadModule(CELL_SYSMODULE_FS);
  NMP_ASSERT_MSG(moduleLoadResult == CELL_OK, "Failed to load module FS.");
  moduleLoadResult = cellSysmoduleLoadModule(CELL_SYSMODULE_SPURS);
  NMP_ASSERT_MSG(moduleLoadResult == CELL_OK, "Failed to load module SPURS.");
  moduleLoadResult = cellSysmoduleLoadModule(CELL_SYSMODULE_SYNC);
  NMP_ASSERT_MSG(moduleLoadResult == CELL_OK, "Failed to load module sync.");
#endif

  // Initialise morpheme and the NMP memory management system.
  // The default memory management functions can be overridden by the provision of a NMP::Memory::Config structure to
  //  this init function.
  NMP::Memory::init();

  // Uncomment these lines to output to log files.
  // NET_LOG_INIT("MorphUpdate.log");
  // PROFILING_LOG_INIT("MorphProfiling.log")

  // Uncomment this line to log everything.
  // NET_LOG_ADD_PRIORITY_RANGE(NMP::LOG_MIN_PRIORITY, NMP::LOG_MAX_PRIORITY);

  // Uncomment this line to log queued parameters.
  // NET_LOG_ADD_PRIORITY_RANGE(98, 100);

  // Uncomment this line to log queue contents when not empty at end of execution (including all task params).
  // NET_LOG_ADD_PRIORITY_RANGE(101, 107);

  // Uncomment this line to log queue contents when not empty at end of execution (Only include unevaluated task params).
  // NET_LOG_ADD_PRIORITY_RANGE(102, 103);

  // Uncomment this line to list executing tasks.
  // NET_LOG_ADD_PRIORITY_RANGE(107, 107);

  // Uncomment this line to list executing task parameters.
  // NET_LOG_ADD_PRIORITY_RANGE(108, 108);

  // Uncomment this line to list task names against their registered IDs.
  // NET_LOG_ADD_PRIORITY_RANGE(109, 109);

  MR::Manager::initMorphemeLib();

  int result = 0;
  {
    NMP::CommandLineProcessor commandLineArguments;
    commandLineArguments.registerFlagOption("-autoKillTarget");
    commandLineArguments.registerUInt32Option("-port");
    commandLineArguments.registerStringOption("-pvdFilename");

    if (!commandLineArguments.parseCommandLine(argc, argv))
    {
      NMP_ASSERT_FAIL_MSG("%s", commandLineArguments.getParseErrorString());
    }

    result = runtimeTarget(commandLineArguments);
  }

  // Shut down morpheme and the NMP memory management system.
  MR::DispatcherBasic::term();
#ifdef NM_HOST_CELL_PPU
  MR::DispatcherPS3::term();
#endif // NM_HOST_CELL_PPU
  MR::Manager::termMorphemeLib();
  NMP::Memory::shutdown();

  // ...
  return result;
}

//----------------------------------------------------------------------------------------------------------------------
#ifdef NM_HOST_ANDROID
// This will be the entry point from the NativeActivity.
void android_main(struct android_app* state)
{
  // Make sure glue isn't stripped.
  app_dummy();

  static char* appName = (char*)"morphemeRuntimeTarget.exe";
  main(1, &appName);

  return;
}
#endif // NM_HOST_ANDROID
//----------------------------------------------------------------------------------------------------------------------
