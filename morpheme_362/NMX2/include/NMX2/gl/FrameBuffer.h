
namespace nmx
{
	
namespace gl
{
	
	//sizeof(FrameBuffer) == 60
	class FrameBuffer
	{
	public:
		~FrameBuffer(void);
		
		void bind(void);
		void unbind(void);
		
		bool writeDepthTGA(const char* filename);
		bool writeDepthRAW(const char* filename);
		bool writeColourTGA(const char* filename);
		
		uint32_t textureId(void);
		
		bool getDepthPixels(float* dst);
		bool getColourPixels(float* dst);
		
		void draw(void);
		
		void create(
			uint32_t int_unknown1, 
			uint32_t int_unknown2, 
			const nmx::gl::ColourBufferFormat& colorformat,
			const nmx::gl::DepthBufferFormat& depthformat,
			const nmx::gl::MultiSampleMode& multisamplemode,
			bool bool_unknown);
		void copy(const nmx::gl::FrameBuffer& src);
		void destroy(void);
		
		
		
	private:
		int m_unknown1 = 2; // (char* )this
		int m_unknown2 = 0; // (char* )this + 4 || (_DWORD* )this + 1
		int m_unknown3 = 6; // (char* )this + 8 || (_DWORD* )this + 2
		uint8_t m_unknown4 = 0; // (char* )this + 12 || (_DWORD* )this + 3
		uint8_t m_unknown5 = 0; // (char* )this + 13
		uint8_t m_unknown6 = 0; // (char* )this + 14
		uint8_t m_unknown7 = 0; // (char* )this + 15
		int m_unknown8 = 0; // (char* )this + 16 || (_DWORD* )this + 4
		int m_unknown9 = 0; // (char* )this + 20 || (_DWORD* )this + 5
		int m_unknown10 = 0; // (char* )this + 24 || (_DWORD* )this + 6
		int m_unknown11 = 0; // (char* )this + 28 || (_DWORD* )this + 7
		int m_unknown12 = 0; // (char* )this + 32 || (_DWORD* )this + 8
		uint32_t m_textureId = 0; // (char* )this + 36 || (_DWORD* )this + 9
		int m_unknown13 = 0; // (char* )this + 40 || (_DWORD* )this + 10
		int m_unknown14 = 0; // (char* )this + 44 || (_DWORD* )this + 11
		int m_unknown15 = 0; // (char* )this + 48 || (_DWORD* )this + 12
		int m_unknown16 = 0; // (char* )this + 52 || (_DWORD* )this + 13
		uint8_t m_unknown17 = 0;  // (char* )this + 56 || (_DWORD* )this + 14
		
	}
	
} // namespace gl
	
} // namespace nmx