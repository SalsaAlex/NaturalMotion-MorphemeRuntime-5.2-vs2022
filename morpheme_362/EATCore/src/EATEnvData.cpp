
static inline uint32_t ROL32(uint32_t value, int shift)
{
	return (value << shift) | (value >> (32 - shift));
}
static struct StoragePropertyQuery
{
	DWORD PropertyId;         // StorageDeviceProperty = 0
	DWORD QueryType;          // PropertyStandardQuery  = 0
	BYTE  AdditionalParameters[4];
};
#pragma pack(push, 1)
static struct StorageDeviceDescriptor
{
	DWORD   Version;              // +0
	DWORD   Size;                 // +4
	BYTE    DeviceType;           // +8
	BYTE    DeviceTypeModifier;   // +9
	BOOLEAN RemovableMedia;       // +10  ← checked for == 0
	BOOLEAN CommandQueueing;      // +11
	DWORD   VendorIdOffset;       // +12  ← XOR'd into hash
	DWORD   ProductIdOffset;      // +16  ← used to walk product string
	DWORD   ProductRevisionOffset;// +20
	DWORD   SerialNumberOffset;   // +24  ← XOR'd into hash
	DWORD   BusType;              // +28  ← validated against whitelist
	DWORD   RawPropertiesLength;  // +32
	BYTE    RawDeviceProperties[1];
};
#pragma pack(pop)

namespace EAT
{
	
	EnvData::EnvData(void* pBuildStampData, 
			const wxString& productId, 
			const wxString& appDisplayName, 
			int argc, wchar_t** argv, 
			int versionmajor, int versionminor, int versionrelease, int versionrevision, 
			wxString versionSpecialString, 
			const wxString& unknown1, 
			const wxString& vendor,
			unsigned long unknown2)
	{
		m_productId = productId;
		m_appDisplayName = appDisplayName;
		
		m_versionMajor = versionmajor;
		m_versionMinor = versionminor;
		m_versionRevision = versionrevision;
		m_versionRelease = versionrevision;
		m_versionSpecialString = versionSpecialString;
		
		m_appConfig = nullptr;
		
		m_unknown13 = unknown2;
		
		
		//incomplete
		
	}				
	
	
	wxString EnvData::getPluginConfigKey(int key)
	{
		wxString configkey;
		configkey.Format("Plugins/%d/", key);
		return configkey;
	}
	
	
	
	//
	//machine id (not tested)
	//
	
	static uint32_t EnvData::calculateUniqueMachineID()
	{
		
		int cpuRegs[4];
	
		__cpuid(cpuRegs, 0);
		const uint32_t vendorHash = static_cast<uint32_t>(cpuRegs[1])   // EBX
								^ static_cast<uint32_t>(cpuRegs[2])   // ECX
								^ static_cast<uint32_t>(cpuRegs[3]);  // EDX
	
		__cpuid(cpuRegs, 1);
		const uint32_t cpuFeatures    = static_cast<uint32_t>(cpuRegs[3]); // EDX (stored but not used further)
		const uint32_t processorModel = static_cast<uint32_t>(cpuRegs[0]) & 0xFFF;
	
		uint32_t hash = (processorModel << 16) ^ vendorHash ^ 0xF2B001D3u;
		
		static const DWORD VALID_BUS_TYPES[] = {
			1,  // BusTypeScsi
			2,  // BusTypeAtapi
			3,  // BusTypeAta
			8,  // BusTypeRAID
			9,  // BusTypeiScsi
			10, // BusTypeSas
			11, // BusTypeSata
		};
		
		StoragePropertyQuery query    = {};
		uint8_t              rawBuf[0x2000];
		bool                 foundDriveInfo = false;
	
		for (int i = 0; i < 16; ++i)
		{
			char drivePath[128];
			sprintf_s(drivePath, sizeof(drivePath), "\\\\.\\PhysicalDrive%d", i);
	
			HANDLE hDrive = CreateFileA(
				drivePath,
				/*dwDesiredAccess=*/  0,
				/*dwShareMode=*/      FILE_SHARE_READ | FILE_SHARE_WRITE,
				/*lpSecurityAttrs=*/  nullptr,
				/*dwCreationDisposition=*/ OPEN_EXISTING,
				/*dwFlagsAndAttributes=*/ 0,
				/*hTemplateFile=*/    nullptr
			);
	
			if (hDrive == INVALID_HANDLE_VALUE)
				continue;
	
			DWORD bytesReturned = 0;
			memset(rawBuf, 0, sizeof(rawBuf));
	
			const BOOL ok = DeviceIoControl(
				hDrive,
				/*dwIoControlCode=*/ 0x2D1400u, // IOCTL_STORAGE_QUERY_PROPERTY
				/*lpInBuffer=*/      &query,
				/*nInBufferSize=*/   sizeof(query),
				/*lpOutBuffer=*/     rawBuf,
				/*nOutBufferSize=*/  sizeof(rawBuf),
				&bytesReturned,
				/*lpOverlapped=*/    nullptr
			);
	
			if (ok)
			{
				auto* desc = reinterpret_cast<StorageDeviceDescriptor*>(rawBuf);
	
				// Validate: non-removable media on a recognised bus type
				bool validBusType = false;
				for (DWORD bt : VALID_BUS_TYPES)
					if (desc->BusType == bt) { validBusType = true; break; }
	
				if (!desc->RemovableMedia && validBusType)
				{
					// Mix vendor and serial offset values into hash
					hash ^= desc->VendorIdOffset ^ desc->SerialNumberOffset;
	
					// Walk the null-terminated product-ID string (if present)
					if (desc->ProductIdOffset != 0)
					{
						const char* str = reinterpret_cast<const char*>(rawBuf)
										+ desc->ProductIdOffset;
	
						while (*str)
						{
							hash = ROL32(hash ^ static_cast<uint8_t>(*str), 6);
							++str;
						}
	
						foundDriveInfo = true;
					}
				}
			}
			CloseHandle(hDrive);
		}
		
		if (!foundDriveInfo)
		{
			// Reuse rawBuf as the IP_ADAPTER_INFO output buffer
			auto* adapters   = reinterpret_cast<IP_ADAPTER_INFO*>(rawBuf);
			ULONG bufferSize = sizeof(rawBuf);
	
			const DWORD adapterResult = GetAdaptersInfo(adapters, &bufferSize);
	
			if (adapterResult != ERROR_SUCCESS)
			{
				// GetAdaptersInfo failed → fall back to computer name
				char  computerName[16];
				DWORD nameLen = static_cast<DWORD>(sizeof(computerName));
	
				if (GetComputerNameA(computerName, &nameLen) && nameLen > 0)
				{
					for (DWORD k = 0; k < nameLen; ++k)
					{
						const int shifted = static_cast<int>(computerName[k])
										<< (k % 25u); // 0x19 == 25
						hash ^= shifted;
					}
				}
			}
			else
			{
				// GetAdaptersInfo succeeded → XOR all MAC address bytes in
	
				// First adapter
				for (UINT n = 0; n < adapters->AddressLength; ++n)
					hash = ROL32(hash ^ adapters->Address[n], 6);
	
				// Remaining adapters
				for (IP_ADAPTER_INFO* adp = adapters->Next; adp != nullptr; adp = adp->Next)
				{
					for (UINT n = 0; n < adp->AddressLength; ++n)
						hash = ROL32(hash ^ adp->Address[n], 6);
				}
			}
		}
		
		return hash;
	}
	
	void EnvData::readSessionID(long &l1, long &l2, long &l3)
	{
		wxString configsessionid;
		l1 = l2 = l3 = 0;
		
		if( configRead("sessionID", configsessionid) )
		{
			wxStringList list;
			nmui::splitDelimitedString(configsessionid, list, '.');
			if(list.GetCount() == 3)
			{
				list[0].ToLong(l1, 16);
				list[1].ToLong(l2, 16);
				list[2].ToLong(l3, 16);
			}
		}
	}
	
	wxString EnvData::getSessionIDString()
	{
		wxString sessionId;
		sessionId.reserve(64);
		sessionId += wxString::Format("%x", calculateUniqueMachineID());
		
        unsigned long long1, long2, long3;
		readSessionID(long1, long2, long3);
		
		sessionId = (
			sessionId + 
			wxString::Format(".%lx", long1) +
			wxString::Format(".%lx", long2) +
			wxString::Format(".%lx", long3) 
			);
		
		return sessionId;
	}
	
} // namespace EAT