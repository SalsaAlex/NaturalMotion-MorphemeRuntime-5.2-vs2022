
namespace nmx
{
	//to discover:
	// does getattribute & getAttributeArray return a pointer (*) or a reference (&) ? (it seems to return reference..)
	void sgTransformNode::connectChildBounds(nmx::sgNode* node)
	{
		nmx::ChangeBlock("Connect Child Bounds"); //obs1: incomplete. obs2: make macro
		
		nmx::AttributeArray* attribarray1 = node->getAttributeArray(10);
		attribarray1->resize(array->size() + 1);
		
		nmx::Attribute* attrib1 = node->getAttribute(7);
		//incomplete
	}
	
	void sgTransformNode::ConnectChildTransform(nmx::sgNode* node)
	{
		nmx::ChangeBlock("Connect Child Transforms"); //obs1: incomplete. obs2: make macro
		
		nmx::Attribute* attrib1 = node->getAttribute(16);
		nmx::Attribute* attrib2 = node->getAttribute(15);
		
		attrib2->connect(attrib1, true, false);
		
		nmx::Attribute* attrib3 = node->getAttribute(17);
		nmx::Attribute* attrib4 = node->getAttribute(18);
		
		attrib4->connect(attrib3, true, false);
		connectChildBounds(node);
	}
	
} //namespace nmx