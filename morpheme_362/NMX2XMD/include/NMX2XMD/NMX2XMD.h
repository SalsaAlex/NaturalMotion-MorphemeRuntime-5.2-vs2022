
namespace NMX2XMD
{
	class SkinningResults
	{
	public:
		void* placeholder;
	}
	
	bool applySkinning(
		const XMD::XModel& model,
		const XMD::XGeometry& geometry,
		const XMU::XVertexArray& vertarray,
		nmx::Scene& scene,
		nmx::GeometryNode& geometrynode,
		nmx::Node& node,
		SkinningResults& result);
	
	nmx::MeshNode *createMesh(const wchar_t* name, const XMU::XVertexArray& vertarray, nmx::Scene& scene);
	
	
	class NMX2HierarchyConverter
	{
	public:
		NMX2HierarchyConverter(const NMX2HierarchyConverter& converter);
		NMX2HierarchyConverter(XMD::XModel& model);
		
		void convertHierarchyToJoints(const nmx::sgTransformNode& hierarchy, uint32_t num, XMD::XJoint* joints);
		void recursiveSaveHierarchy(const nmx::sgTransformNode& hierarchy, uint32_t num, XMD::XJoint* joints);
		void recursiveSavePoseToCycle(const nmx::sgTransformNode& pose, uint32_t num, XMD::XAnimCycle* animcycle, float unknownfloat1);
		void recursiveSavePoseToTake(const nmx::sgTransformNode& pose, uint32_t num, XMD::XAnimationTake* animtake, float unknownfloat1);
		void savePoseAsAnimation(const nmx::sgTransformNode& pose, uint32_t num, const char* data, float unknownfloat1, bool unknownbool1);
	}
} // namespace NMX2XMD