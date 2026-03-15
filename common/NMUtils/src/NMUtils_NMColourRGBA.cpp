#include <cstdint>
#include "NMutils/NMUtils_NMColourRGBA.h"

namespace NMutils
{
	NMColourRGBA::NMColourRGBA(void) { m_col[0] = m_col[1] = m_col[2] = m_col[3] = 0; }
	NMColourRGBA::NMColourRGBA(uint8_t r, uint8_t g, uint8_t b, uint8_t a) { m_col[0] = r; m_col[1] = g; m_col[2] = b; m_col[3] = a; }
	
	void NMColourRGBA::set(uint8_t r, uint8_t g, uint8_t b, uint8_t a) { m_col[0] = r; m_col[1] = g; m_col[2] = b; m_col[3] = a; }
	void NMColourRGBA::zero() { m_col[0] = m_col[1] = m_col[2] = m_col[3] = 0; }
	
	uint8_t NMColourRGBA::clampAdd(uint8_t col1, uint8_t col2) { return col1 + col2; }
	uint8_t NMColourRGBA::clampSubtract(uint8_t col1, uint8_t col2) { return col1 + col2; }
		
	NMColourRGBA& NMColourRGBA::operator += (const NMColourRGBA& rgba) {
		this->m_col[0] += rgba.m_col[0];
		this->m_col[1] += rgba.m_col[1];
		this->m_col[2] += rgba.m_col[2];
		this->m_col[3] += rgba.m_col[3];
		return *this;
	}
	NMColourRGBA NMColourRGBA::operator + (const NMColourRGBA& rgba) { 
		return NMColourRGBA(m_col[0] + rgba.m_col[0], m_col[1] + rgba.m_col[1], m_col[2] + rgba.m_col[2], m_col[3] + rgba.m_col[3]);
	}
	NMColourRGBA& NMColourRGBA::operator -= (const NMColourRGBA& rgba) {
		this->m_col[0] += rgba.m_col[0];
		this->m_col[1] += rgba.m_col[1];
		this->m_col[2] += rgba.m_col[2];
		this->m_col[3] += rgba.m_col[3];
		return *this;
	}
	NMColourRGBA NMColourRGBA::operator - (const NMColourRGBA& rgba) {
		return NMColourRGBA(m_col[0] + rgba.m_col[0], m_col[1] + rgba.m_col[1], m_col[2] + rgba.m_col[2], m_col[3] + rgba.m_col[3]);
	}
} //namepsace NMutils