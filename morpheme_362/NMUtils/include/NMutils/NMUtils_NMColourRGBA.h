
namespace NMutils
{
	class __declspec(dllexport) NMColourRGBA
	{
	public:
		NMColourRGBA(void);
		NMColourRGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a);

		void set(uint8_t r, uint8_t g, uint8_t b, uint8_t a);
		void zero();

		static uint8_t clampAdd(uint8_t col1, uint8_t col2);
		static uint8_t clampSubtract(uint8_t col1, uint8_t col2);

		NMColourRGBA& operator += (const NMColourRGBA& rgba);
		NMColourRGBA operator + (const NMColourRGBA& rgba);
		NMColourRGBA& operator -= (const NMColourRGBA& rgba);
		NMColourRGBA operator - (const NMColourRGBA& rgba);

	private:
		uint8_t m_col[4];
	};
} //namepsace NMutils