
namespace db2
{
	class XMLWriter
	{
	public:
		XMLWriter(db2::OutputStream &outstream);
		~XMLWriter(void);
	
	private:
		NMutils::FastStringStack* m_stringstack;
	}
} // namespace NMX2XMD