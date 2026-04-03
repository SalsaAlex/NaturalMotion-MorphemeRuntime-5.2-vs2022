
namespace nmx
{
	
	bool CameraSettingsNode::setActiveCamera(nmx::CameraNode* cam_node, nmx::CameraMode cam_mode)
	{
		if(!cam_node->unknownfunc1()) //is this really a class function ?
			return false;
		
		nmx::ChangeBlock("set active camera"); //obs1: incomplete. obs2: make macro
		
		if(getActiveCamera() != cam_node)
		{
			nmx::ConstAttribute* attrib1 = getAttribute(6);
			nmx::LegacyAttribute* attrib2 = attrib1->getInput();
			
			if(attrib2->getNode())
				attrib1->disconnect(false);
			
			nmx::Attribute* attrib2 = cam_node->getAttribute(13);
			attrib1->connect(attrib2, false);
			getAttribute(7)->setInt(cam_mode);
		}
		return true;
	}
	
} //namespace nmx