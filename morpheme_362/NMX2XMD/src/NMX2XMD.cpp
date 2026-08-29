#include "NMX2XMD/NMX2XMD.h"


namespace NMX2XMD
{
	bool applySkinning(
		const XMD::XModel& model,
		const XMD::XGeometry& geometry,
		const XMU::XVertexArray& vertarray,
		nmx::Scene& scene,
		nmx::GeometryNode& geometrynode,
		nmx::Node& node,
		SkinningResults& result)
	{
		
	}
	
	nmx::MeshNode *createMesh(const wchar_t* name, const XMU::XVertexArray& vertarray, nmx::Scene& scene)
	{
		
	}
	
	NMX2HierarchyConverter::NMX2HierarchyConverter(const NMX2HierarchyConverter& converter)
	{
		
	}
	NMX2HierarchyConverter::NMX2HierarchyConverter(XMD::XModel& model)
	{
		
	}
	
	void NMX2HierarchyConverter::convertHierarchyToJoints(const nmx::sgTransformNode& hierarchy, uint32_t num, XMD::XJoint* joints)
	{
		
	}
	void NMX2HierarchyConverter::recursiveSaveHierarchy(const nmx::sgTransformNode& hierarchy, uint32_t num, XMD::XJoint* joints)
	{
		
	}
	void NMX2HierarchyConverter::recursiveSavePoseToCycle(const nmx::sgTransformNode& pose, uint32_t num, XMD::XAnimCycle* animcycle, float unknownfloat1
	{
		
	}
	void NMX2HierarchyConverter::recursiveSavePoseToTake(const nmx::sgTransformNode& pose, uint32_t num, XMD::XAnimationTake* animtake, float unknownfloat1)
	{
		
	}
	void NMX2HierarchyConverter::savePoseAsAnimation(const nmx::sgTransformNode& pose, uint32_t num, const char* data, float unknownfloat1, bool unknownbool1)
	{
		
	}
	
} // namespace NMX2XMD