//function offsets

#define IDAPRO_IMAGEBASE 0x67370000

constexpr uint32_t g_mcc__isEuphoriaEnabled = 0x67649560 - IDAPRO_IMAGEBASE;
constexpr uint32_t g_mcc__isKinectEnabled = 0x67649550 - IDAPRO_IMAGEBASE;
constexpr uint32_t g_mcc__isPhysicsEnabled = 0x67649540 - IDAPRO_IMAGEBASE;

