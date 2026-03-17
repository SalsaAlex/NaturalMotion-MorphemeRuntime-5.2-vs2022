
namespace mcc
{
	
	class RigExporter
	{
	public:
		RigExporter(const RigExporter& rigexporter);
		RigExporter(XMD::XModel& model);
		~RigExporter();
		
		void setTrajectoryJointIndex(uint index);
		void setHipJointIndex(uint index);
		void setDefaultBodyGroups(const nmx::StringArray& stringarray);
		void setBlendFrame(const nmx::Vector3 pos, const nmx::Quat rot);
		void setInputRigScaleFactor(float factor);
		void reset(void);
		void enableExport(XMD::XBone* bone);
		void assignRuntimeID(XMD::XBone *bone, int& runtimeid);
		void buildHierarchyNode(void);
		
		void buildOutputAndHierarchy(const wchar_t* string1, const wchar_t* string2, 
						nmx::sgTransformNode *&sgtransformnode, nmx::TransformNode *&transformnode,
						nmx::RuntimeOutputNode *&runtimeoutputnode, nmx::HierarchyNode *&hierarchynode);
						
		void addMeshData(nmx::RuntimeOutputNode* runtimeoutputnode,
						const XMU::XVertexArray& vertarray,
						const XMD::XGeometry &xmdgeometry,
						const nmx::Node* pnode,
						nmx::sgTransformNode* psgtransformnode1,
						nmx::MeshNode** meshnode,
						nmx::sgTransformNode** ppsgtransformnode1,
						nmx::sgTransformNode** ppsgtransformnode2);
						
		void setupForExport(void);
		
		
		bool buildKinematicGraph(void);
		bool buildSkinGraph(void);
		
		bool saveRig(const std::wstring& filename);
		bool saveSkin(const std::wstring& filename);
		
		
		static bool exportRigAndSkin(const std::wstring& string1, 
						const std::wstring& string2,
						const nmx::StringArray& unusedstringarray,
						const std::wstring& string3,
						const std::wstring& string4,
						const nmx::Vector3& unusedpos,
						const nmx::Quat& unusedrot,
						float unusedfloat);
		static bool exportSkin(const std::wstring& filename);
		
		
	private:
		XM2::XBoneList m_pBoneList; //(char *)this + 4
		float m_fInputRigScaleFactor; //*((float *)this + 36)
		uint m_uiHipJointIndex; //*((_DWORD *)this + 14)
		uint m_uiTrajectJointIndex; //*((_DWORD *)this + 13)
		
	}
	
	void RigExporter::enableExport(XMD::XBone* bone)
	{
		if(!bone)
			return;
		
		if(bone->GetAttribute("export"))
			return;
		
		XMD::XAttribute* attrib = bone->CreateAttribute("export");
		attrib->SetType(XFn::Type::BoolAttribute);
		attrib->Set(1, 0);
		
		while(bone && bone->GetParent())
		{
			bone = bone->GetParent();
			if(bone->GetAttribute("export"))
				continue;
			
			XMD::XAttribute* attrib = bone->CreateAttribute("export");
			attrib->SetType(XFn::Type::BoolAttribute);
			attrib->Set(1, 0);
		}
	}
	
	void RigExporter::assignRuntimeID(XMD::XBone *bone, int& runtimeid)
	{
		if(!bone)
			return;
		XMD::XAttribute* exportattrib;
		if( !(exportattrib = bone->GetAttribute("export")) )
			return;
		
		bool exportenabled;
		exportattrib->Get(&exportenabled, 0);
		if(!exportenabled)
			return;
		
		XMD::XAttribute* runtimeidattrib = bone->CreateAttribute("runtimeid");
		runtimeidattrib->SetType(XFn::Type::IntAttribute);
		runtimeidattrib->Set(runtimeid++, 0);
		
		m_pBoneList.push_back(a2);
		
		XM2::XBoneList childlist;
		
		bone->GetChildren(childlist);
		
		// now duplicate each child and parent to this bone
		for (XM2::XBoneList::iterator it = bones.begin();it != bones.end(); ++it)
			assignRuntimeID(it, runtimeid);
	}
	
	void RigExporter::setupForExport(void)
	{
		
	}

} //namespace mcc