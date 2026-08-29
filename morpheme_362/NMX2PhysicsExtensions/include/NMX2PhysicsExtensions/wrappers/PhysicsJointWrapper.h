
namespace nmx
{
	
	class PhysicsJointWrapper
	{
	public:
		PhysicsJointWrapper(void) = default;
		PhysicsJointWrapper(
			nmx::sgTransformNode *sgtransformnode,
			nmx::sgShapeNode *sgshapenode,
			nmx::TransformBaseNode *transformbasenode,
			nmx::PhysicsJointNode *physicsjointnode
			);
		
		bool hasPhysicsJointLimit(void);
		bool hasPhysicsSoftJointLimit(void);
		bool isValid(void);
		
		void reparent(nmx::PhysicsJointWrapper jointwrapper);
		void reset(void);
		
		static int kInvalidWrapper;
		
	private:
		nmx::sgTransformNode* m_sgtransformnode = nullptr; // (_DWORD *)this
		nmx::sgShapeNode* m_sgshapenode = nullptr; // (_DWORD *)this + 1
		nmx::TransformBaseNode* m_transformbasenode = nullptr; // (_DWORD *)this + 2
		nmx::PhysicsJointNode* m_physicsjointnode = nullptr; // (_DWORD *)this + 3
	}
	
	void deletePhysicsJoint(nmx::Database& database, nmx::PhysicsJointWrapper& jointwrapper);
	
} // namespace nmx