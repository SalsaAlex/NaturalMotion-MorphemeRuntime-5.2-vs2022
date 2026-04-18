
namespace nmx
{
	
	class PhysicsVolumeWrapper
	{
	public:
		PhysicsVolumeWrapper(void) = default;
		PhysicsVolumeWrapper(
			nmx::sgTransformNode *sgtransformnode,
			nmx::TransformNode *transformnode,
			nmx::sgShapeNode *sgshapenode1,
			nmx::PhysicsVolumeNode *physicsvolumenode,
			nmx::sgShapeNode *sgshapenode2,
			nmx::ShapeNode *shapenode
			);
		
		bool isValid(void);
		
		void reset(void);
		
		static int kInvalidWrapper;
		
	private:
		nmx::sgTransformNode *m_sgtransformnode = nullptr; // (_DWORD *)this
		nmx::TransformNode *m_transformnode = nullptr; // (_DWORD *)this + 1
		nmx::sgShapeNode *m_sgshapenode1 = nullptr; // (_DWORD *)this + 2
		nmx::PhysicsVolumeNode *m_physicsvolumenode = nullptr; // (_DWORD *)this + 3
		nmx::sgShapeNode *m_sgshapenode2 = nullptr; // (_DWORD *)this + 4
		nmx::ShapeNode *m_shapenode = nullptr; // (_DWORD *)this + 5
	}
	
	void deletePhysicsJoint(nmx::Database& database, nmx::PhysicsJointWrapper& jointwrapper);
	
} // namespace nmx