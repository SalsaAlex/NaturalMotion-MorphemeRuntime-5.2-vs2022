
class morphemeConnectApp : public EAT::EATCommonApp
{
public:
	bool init() override;
	
private:
	
	uint8_t unknown[36];
	int m_argc; //(int)this + 40
	wchar_t **m_argv; //(int)this + 44
	
	void* 
	
}