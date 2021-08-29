/*
 * crc_compute.h
 *
 *  Created on: 2019年4月1日
 *      Author: Henry Zhu
 */

#ifndef MATH_CRC_COMPUTE_H_
#define MATH_CRC_COMPUTE_H_

#include <QMainWindow>
#include <stdint.h>

template <typename TYPE> class CRC
{
public:
    CRC();
    CRC(TYPE polynomial, TYPE init_remainder, TYPE final_xor_value);
    void build(TYPE polynomial, TYPE init_remainder, TYPE final_xor_value);
    /**
     * Compute the CRC checksum of a binary message block.
     * @para message, 用来计算的数据
     * @para nBytes, 数据的长度
     */
    TYPE crcCompute(uint8_t * message, unsigned int nBytes);
    TYPE crcCompute(uint8_t * message, unsigned int nBytes, bool reinit);
protected:
    TYPE m_polynomial;
    TYPE m_initial_remainder;
    TYPE m_final_xor_value;
    TYPE m_remainder;
    TYPE crcTable[256];
    int m_width;
    int m_topbit;
    /**
     * Initialize the CRC lookup table.
     * This table is used by crcCompute() to make CRC computation faster.
     */
    void crcInit(void);
};

template <typename TYPE>
CRC<TYPE>::CRC()
{
    m_width = 8 * sizeof(TYPE);
    m_topbit = 1 << (m_width - 1);
}

template <typename TYPE>
CRC<TYPE>::CRC(TYPE polynomial, TYPE init_remainder, TYPE final_xor_value)
{
    m_width = 8 * sizeof(TYPE);
    m_topbit = 1 << (m_width - 1);
    m_polynomial = polynomial;
    m_initial_remainder = init_remainder;
    m_final_xor_value = final_xor_value;

    crcInit();
}

template <typename TYPE>
void CRC<TYPE>::build(TYPE polynomial, TYPE init_remainder, TYPE final_xor_value)
{
    m_polynomial = polynomial;
    m_initial_remainder = init_remainder;
    m_final_xor_value = final_xor_value;

    crcInit();
}

template <typename TYPE>
TYPE CRC<TYPE>::crcCompute(uint8_t * message, unsigned int nBytes)
{
    unsigned int offset;
    unsigned char byte;
    TYPE remainder = m_initial_remainder;
    /* Divide the message by the polynomial, a byte at a time. */
    for( offset = 0; offset < nBytes; offset++)
    {
        byte = (remainder >> (m_width - 8)) ^ message[offset];
        remainder = crcTable[byte] ^ (remainder << 8);
    }
    /* The final remainder is the CRC result. */
    return (remainder ^ m_final_xor_value);
}

template <typename TYPE>
TYPE CRC<TYPE>::crcCompute(uint8_t * message, unsigned int nBytes, bool reinit)
{
    unsigned int offset;
    unsigned char byte;
    if(reinit)
    {
        m_remainder = m_initial_remainder;
    }
    /* Divide the message by the polynomial, a byte at a time. */
    for( offset = 0; offset < nBytes; offset++)
    {
        byte = (m_remainder >> (m_width - 8)) ^ message[offset];
        m_remainder = crcTable[byte] ^ (m_remainder << 8);
    }
    /* The final remainder is the CRC result. */
    return (m_remainder ^ m_final_xor_value);
}

class CRC8 : public CRC<uint8_t>
{
public:
    enum CRC8_TYPE {eCRC8, eAUTOSAR, eCDMA2000, eDARC, eDVB_S2, eEBU, eAES, eGSM_A, eGSM_B, eI_CODE,
                    eITU, eLTE, eMAXIM, eOPENSAFETY, eROHC, eSAE_J1850, eWCDMA};
    CRC8(CRC8_TYPE type);
    CRC8(uint8_t polynomial, uint8_t init_remainder, uint8_t final_xor_value)
        :CRC<uint8_t>(polynomial, init_remainder, final_xor_value){}
};

class CRC16 : public CRC<uint16_t>
{
public:
    enum CRC16_TYPE {eCCITT, eKERMIT, eCCITT_FALSE, eIBM, eARC, eLHA, eSPI_FUJITSU,
                     eBUYPASS, eVERIFONE, eUMTS, eCDMA2000, eCMS, eDDS_110, eDECT_R,
                     eDECT_X, eDNP, eEN_13757, eGENIBUS, eEPC, eDARC, eI_CODE, eGSM,
                     eLJ1200, eMAXIM, eMCRF4XX, eOPENSAFETY_A, eOPENSAFETY_B, ePROFIBUS,
                     eIEC_61158_2, eRIELLO, eT10_DIF, eTELEDISK, eTMS37157, eUSB,
                     eCRC_A, eMODBUS, eX_25, eCRC_B, eISO_HDLC, eIBM_SDLC, eXMODEM,
                     eZMODEM, eACORN, eLTE};
    CRC16(CRC16_TYPE type);
    CRC16(uint16_t polynomial, uint16_t init_remainder, uint16_t final_xor_value)
        :CRC<uint16_t>(polynomial, init_remainder, final_xor_value){}
};

class CRC32 : public CRC<uint32_t>
{
public:
    enum CRC32_TYPE {eADCCP, ePKZIP, eCRC32, eAAL5, eDECT_B, eB_CRC32, eBZIP2, eAUTOSAR,
                     eCRC32C, eCRC32D, eMPEG2, ePOSIX, eCKSUM, eCRC32Q, eJAMCRC, eXFER};
    CRC32(CRC32_TYPE type);
};
#endif /* MATH_CRC_COMPUTE_H_ */
