/*
 * crc_compute.cpp
 *
 *  Created on: 2019年4月1日
 *      Author: Henry Zhu
 */

#include <./Common/Math/crc_compute.h>

template <typename TYPE>
void CRC<TYPE>::crcInit(void)
{
    TYPE remainder;
    uint16_t dividend;
    uint8_t bit;
    /* Perform binary long division, a bit at a time. */
    for(dividend = 0; dividend < 256; dividend++)
    {
        /* Initialize the remainder.  */
        remainder = dividend << (m_width - 8);
        /* Shift and XOR with the polynomial.   */
        for(bit = 0; bit < 8; bit++)
        {
            /* Try to divide the current data bit.  */
            if(remainder & m_topbit)
            {
                remainder = (remainder << 1) ^ m_polynomial;
            }
            else
            {
                remainder = remainder << 1;
            }
        }
        /* Save the result in the table. */
        crcTable[dividend] = remainder;
    }
}

CRC8::CRC8(CRC8_TYPE type)
{
    switch (type)
    {
    case eCRC8:
        m_polynomial = 0x07; //http://reveng.sourceforge.net/crc-catalogue/all.htm
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
    case eAUTOSAR:
        m_polynomial = 0x2f;
        m_initial_remainder = 0xff;
        m_final_xor_value = 0xff;
        break;
    case eCDMA2000:
        m_polynomial = 0x9b;
        m_initial_remainder = 0xFF;
        m_final_xor_value = 0x00;
        break;
    case eDARC:
        m_polynomial = 0x39;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
    case eDVB_S2:
        m_polynomial = 0xd5;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
    case eEBU:
    case eAES:
        m_polynomial = 0x1d;
        m_initial_remainder = 0xFF;
        m_final_xor_value = 0x00;
        break;
    case eGSM_A:
        m_polynomial = 0x1d;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
    case eGSM_B:
        m_polynomial = 0x49;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0xFF;
        break;
    case eI_CODE:
        m_polynomial = 0x1d;
        m_initial_remainder = 0xFD;
        m_final_xor_value = 0x00;
        break;
    case eITU:
        m_polynomial = 0x07;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x55;
        break;
    case eLTE:
        m_polynomial = 0x9b;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
    case eMAXIM:
        m_polynomial = 0x31;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
    case eOPENSAFETY:
        m_polynomial = 0x2f;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
    case eROHC:
        m_polynomial = 0x07;
        m_initial_remainder = 0xff;
        m_final_xor_value = 0x00;
        break;
    case eSAE_J1850:
        m_polynomial = 0x1d;
        m_initial_remainder = 0xff;
        m_final_xor_value = 0xff;
        break;
    case eWCDMA:
        m_polynomial = 0x9b;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
    default:
        m_polynomial = 0x07;
        m_initial_remainder = 0x00;
        m_final_xor_value = 0x00;
        break;
    }
    crcInit();

}

CRC16::CRC16(CRC16_TYPE type)
{
    switch (type)
    {
    case eCCITT_FALSE:
    case eMCRF4XX:
        m_polynomial = 0x1021;
        m_initial_remainder = 0xFFFF;
        m_final_xor_value = 0x0000;
        break;
    case eIBM:
    case eARC:
    case eLHA:
    case eBUYPASS:
    case eVERIFONE:
    case eUMTS:
        m_polynomial = 0x8005;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
    case eSPI_FUJITSU:
        m_polynomial = 0x1021;
        m_initial_remainder = 0x1d0f;
        m_final_xor_value = 0x0000;
        break;
    case eCCITT:
    case eKERMIT:
    case eXMODEM:
    case eZMODEM:
    case eACORN:
    case eLTE:
        m_polynomial = 0x1021;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
    case eCDMA2000:
        m_polynomial = 0xc867;
        m_initial_remainder = 0xffff;
        m_final_xor_value = 0x0000;
        break;
    case eCMS:
    case eMODBUS:
        m_polynomial = 0x8005;
        m_initial_remainder = 0xffff;
        m_final_xor_value = 0x0000;
        break;
    case eDDS_110:
        m_polynomial = 0x8005;
        m_initial_remainder = 0x800d;
        m_final_xor_value = 0x0000;
        break;
    case eDECT_R:
        m_polynomial = 0x0589;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0001;
        break;
    case eDECT_X:
        m_polynomial = 0x0589;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
    case eDNP:
    case eEN_13757:
        m_polynomial = 0x3d65;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0xffff;
        break;
    case eGENIBUS:
    case eEPC:
    case eDARC:
    case eI_CODE:
    case eX_25:
    case eCRC_B:
    case eISO_HDLC:
    case eIBM_SDLC:
        m_polynomial = 0x1021;
        m_initial_remainder = 0xffff;
        m_final_xor_value = 0xffff;
        break;
    case eGSM:
        m_polynomial = 0x1021;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0xffff;
        break;
    case eLJ1200:
        m_polynomial = 0x6f63;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
    case eMAXIM:
        m_polynomial = 0x8005;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0xffff;
        break;
    case eOPENSAFETY_A:
        m_polynomial = 0x5935;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
    case eOPENSAFETY_B:
        m_polynomial = 0x755b;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
    case ePROFIBUS:
    case eIEC_61158_2:
        m_polynomial = 0x1dcf;
        m_initial_remainder = 0xffff;
        m_final_xor_value = 0xffff;
        break;
    case eRIELLO:
        m_polynomial = 0x1021;
        m_initial_remainder = 0xb2aa;
        m_final_xor_value = 0x0000;
        break;
    case eT10_DIF:
        m_polynomial = 0x8bb7;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
    case eTELEDISK:
        m_polynomial = 0xa097;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
    case eTMS37157:
        m_polynomial = 0x1021;
        m_initial_remainder = 0x89ec;
        m_final_xor_value = 0x0000;
        break;
    case eUSB:
        m_polynomial = 0x8005;
        m_initial_remainder = 0xffff;
        m_final_xor_value = 0xffff;
        break;
    case eCRC_A:
        m_polynomial = 0x1021;
        m_initial_remainder = 0xc6c6;
        m_final_xor_value = 0x0000;
        break;
    default:
        m_polynomial = 0x8005;
        m_initial_remainder = 0x0000;
        m_final_xor_value = 0x0000;
        break;
    }
    crcInit();
}


CRC32::CRC32(CRC32_TYPE type)
{
    switch (type)
    {
    case eADCCP:
    case ePKZIP:
    case eCRC32:
    case eBZIP2:
    case eAAL5:
    case eDECT_B:
    case eB_CRC32:
        m_polynomial = 0x04c11db7;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0xFFFFFFFF;
        break;
    case eAUTOSAR:
        m_polynomial = 0xf4acfb13;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0xFFFFFFFF;
        break;
    case eCRC32C:
        m_polynomial = 0x1edc6f41;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0xFFFFFFFF;
        break;
    case eCRC32D:
        m_polynomial = 0xa833982b;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0xFFFFFFFF;
        break;
    case eMPEG2:
    case eJAMCRC:
        m_polynomial = 0x04c11db7;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0x00000000;
        break;
    case ePOSIX:
    case eCKSUM:
        m_polynomial = 0x04c11db7;
        m_initial_remainder = 0x00000000;
        m_final_xor_value = 0xFFFFFFFF;
        break;
    case eCRC32Q:
        m_polynomial = 0x814141ab;
        m_initial_remainder = 0x00000000;
        m_final_xor_value = 0x00000000;
        break;
    case eXFER:
        m_polynomial = 0x000000af;
        m_initial_remainder = 0x00000000;
        m_final_xor_value = 0x00000000;
        break;
    default:
        m_polynomial = 0x04C11DB7;
        m_initial_remainder = 0xFFFFFFFF;
        m_final_xor_value = 0xFFFFFFFF;
        break;
    }
    crcInit();
}



