/**@@@+++@@@@******************************************************************
**
** Microsoft Windows Media
** Copyright (C) Microsoft Corporation. All rights reserved.
**
***@@@---@@@@******************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define     IN_CRC32

/******************************************************************************/
#define CRC16_CCITT         0x1021		// CRC ������ʽ.
#define CRC32_CRC32         0x04C10DB7
////////////////////////////////////////////////////////////////////////////////////////
//#include "crctable.c"
/******************************************************************************/


/******************************************************************************/
// ע�⣺�����λһ��Ϊ"1"������ȥ 
//const INT16U cnCRC_16 = 0x8005; 
// CRC-16 = X16 + X15 + X2 + X0 
//const INT16U cnCRC_CCITT = 0x1021; 
// CRC-CCITT = X16 + X12 + X5 + X0����˵��� 16 λ CRC ����ʽ����һ��Ҫ�� 

//const INT32U cnCRC_32 = 0x04C10DB7; 
// CRC-32 = X32 + X26 + X23 + X22 + X16 + X11 + X10 + X8 + X7 + X5 + X4 + X2 + X1 + X0 

//unsigned long Table_CRC[256]; // CRC �� 

extern unsigned long gTable_Crc32[256];

// ���� 32 λ CRC-32 ֵ 
unsigned long CRC_32( unsigned char * aData, unsigned long aSize ) 
{ 
    unsigned long i; 
    unsigned long nAccum = 0; 
//    unsigned long crc32Table[256];
    
//    CRCBuildTable32( CRC32_CRC32 , crc32Table ); 
    for ( i = 0; i < aSize; i++ ) 
        nAccum = ( nAccum << 8 ) ^ gTable_Crc32[( nAccum >> 24 ) ^ *aData++]; 
    return nAccum; 
} 

// ���� 32 λ CRC-32 ֵ 
// CMY: ����ԭ��CRC�����ϼ�������crc
unsigned long CRC_32_NEW( unsigned char * aData, unsigned long aSize, unsigned long prev_crc ) 
{ 
    unsigned long i; 
    unsigned long nAccum = prev_crc; 

    for ( i = 0; i < aSize; i++ ) 
        nAccum = ( nAccum << 8 ) ^ gTable_Crc32[( nAccum >> 24 ) ^ *aData++]; 
    return nAccum; 
} 

//#ifdef NOT_IN_LOADER
unsigned long CRC_32_FILE(const char* file)
{
    unsigned long nAccum = 0;
    unsigned char rd=0;
    FILE *fp=NULL;

    if( (fp=fopen(file, "rb"))==NULL )
    {
        return 0;
    }
    fseek(fp, 0, SEEK_SET);
    while(fread(&rd, 1, 1, fp)>0)
    {
        nAccum = ( nAccum << 8 ) ^ gTable_Crc32[( nAccum >> 24 ) ^ rd];
    }
    fclose(fp);
    return nAccum;
}
//#endif

unsigned long CRC_32_CheckFile(const char* file)
{
    unsigned long nAccum = 0;
    unsigned char rd=0;
    FILE *fp=NULL;
	unsigned int size = 0;
	unsigned int crc = 0;

    if( (fp=fopen(file, "rb"))==NULL )
    {
        return 0;
    }
    fseek(fp, 0, SEEK_END);
	size = ftell(fp);

    fseek(fp, 0, SEEK_SET);

	while(size-- > 4)
	{
		fread(&rd, 1, 1, fp);
		nAccum = ( nAccum << 8 ) ^ gTable_Crc32[( nAccum >> 24 ) ^ rd];
	}
//    while(fread(&rd, 1, 1, fp)>0)
//    {
//        nAccum = ( nAccum << 8 ) ^ gTable_Crc32[( nAccum >> 24 ) ^ rd];
//    }
	fread(&crc, 1, 4, fp);
    fclose(fp);

    return (nAccum==crc?crc:0);
}

// ��� BUFFER CRC У���Ƿ� �д���.���� ��� 4 �� BYTE �� CRC32 ��У��ֵ.
// CMY:
// ��CRCУ��ʧ�ܣ�����0
// ��CRCУ��ɹ�������CRCֵ
unsigned long CRC_32CheckBuffer( unsigned char * aData, unsigned long aSize )
{
    unsigned long crc = 0;
	int i=0;
    if( aSize <= 4 )
        {
        return 0;
        }
    aSize -= 4;

	// CMY:���ǵ�4Bytes����
	for(i=3; i>=0; i--)
		crc = (crc<<8)+(*(aData+aSize+i));

    if( CRC_32( aData , aSize ) == crc )
        return crc;
	
    return 0;
}

/////////////////////////////////////////////////////////////

