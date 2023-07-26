#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>


/*************************************************************************/
/*  SSD general data types                                               */
/*************************************************************************/

#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE (!FALSE)
#endif

/*
typedef unsigned char BOOL;

typedef signed char INT8;
typedef unsigned char UINT8;
typedef volatile signed char VINT8;
typedef volatile unsigned char VUINT8;

typedef signed short INT16;
typedef unsigned short UINT16;
typedef volatile signed short VINT16;
typedef volatile unsigned short VUINT16;

<<<<<<< HEAD
typedef signed long INT32;
typedef unsigned long UINT32;
typedef volatile signed long VINT32;
typedef volatile unsigned long VUINT32;
=======
typedef signed int INT32;
typedef unsigned int UINT32;
typedef volatile signed int VINT32;
typedef volatile unsigned int VUINT32;
>>>>>>> origin/spc5

#define ASM_KEYWORD __asm
*/
/*************************************************************************/
/*                  Offsets of NVM Control Registers                     */
/*************************************************************************/
#define NVM_MCR             0x0000       /* Module Configuration Register */
#define NVM_MCRX            0x0004       /* Module Configuration Extended */
#define NVM_MCRE            0x0008       /* Extended Module Configuration Register */
#define NVM_LOCK0           0x0010       /* Lock 0 Register */
#define NVM_LOCK1           0x0014       /* Lock 1 Register */
#define NVM_LOCK2           0x0018       /* Lock 2 Register */
#define NVM_LOCK3           0x001C       /* Lock 3 Register */
#define NVM_LOCK0A          0x0028       /* Alternate Lock 0 Register */
#define NVM_LOCK1A          0x002C       /* Alternate Lock 1 Register */
#define NVM_SEL0            0x0038       /* Select 0 Register */
#define NVM_SEL1            0x003C       /* Select 1 Register */
#define NVM_SEL2            0x0040       /* Select 2 Register */
#define NVM_SEL3            0x0044       /* Select 3 Register */
#define NVM_OPP0            0x0080       /* Over-Program Protection 0 Register */
#define NVM_OPP1            0x0084       /* Over-Program Protection 1 Register */
#define NVM_OPP2            0x0088       /* Over-Program Protection 2 Register */
#define NVM_OPP3            0x008C       /* Over-Program Protection 3 Register */
#define NVM_EXTSTAT         0x0100       /* Extended Mode Status Register */


/*************************************************************************/
/*              NVM Module Control Registers Field Definitions           */
/*************************************************************************/
/* Module Configuration Register */
#define NVM_MCR_EER         0x00008000   /* ECC Event Error */
#define NVM_MCR_RWE         0x00004000   /* Read While Write Event Error */
#define NVM_MCR_SBC         0x00002000   /* Single Bit Correction Error */
#define NVM_MCR_DONE        0x00000400   /* State Machine Status */
#define NVM_MCR_PEG         0x00000200   /* Program/Extended Good */
#define NVM_MCR_FERS        0x00000080   /* Factory Mode */
#define NVM_MCR_PGM         0x00000010   /* Program */
#define NVM_MCR_PSUS        0x00000008   /* Program Suspend */
#define NVM_MCR_ERS         0x00000004   /* Extended Operations */
#define NVM_MCR_ESUS        0x00000002   /* Extended operation Suspend */
#define NVM_MCR_EHV         0x00000001   /* Enable High Voltage */

/* Extended Mode Status Register */
#define NVM_EXTSTAT_EXTSTAT  0x0000FFFF   /* Extended Mode Status Register */
#define NVM_EXTSTAT_SWAPSTAT 0xFFFF0000   /* Swap Mode Status Register */


/* Module Configuration Extended */
#define NVM_MCRX_EXT        0x00000001   /* Address Extension */
#define NVM_MCRX_SWAP       0x00000002   /* Address Swap */
#define NVM_MCRX_RFS        0x00000010   /* Refresh */
#define NVM_MCRX_EXTIE      0x00000020   /* Extended Interrupt Enable */
#define NVM_MCRX_DIR        0x00000100   /* Direct array is selected for extended command */
#define NVM_MCRX_AOTA       0x00001000   /* Assisted over-the-air */
#define NVM_MCRX_AOTAR      0x00002000   /* Assisted over-the-air reset*/
#define NVM_MCRX_OTAW       0x10000000   /* Over-the-air ready-to-write */

/*************************************************************************/
/*                   Return Codes for SSD functions                      */
/*************************************************************************/
#define NVM_OK                      0x00000000   /* Indicates successful completion of operation */
#define NVM_ERROR_ALIGNMENT         0x00000001   /* Alignment error */
#define NVM_ERROR_ENABLE            0x00000002   /* it's impossible to enable an operation */
#define NVM_ERROR_BUSY              0x00000003   /* New program cannot be preformed while previous high voltage operation in progress */
#define NVM_ERROR_PGOOD             0x00000004   /* The program operation is unsuccessful */
#define NVM_ERROR_EGOOD             0x00000005   /* The extended operation is unsuccessful */
#define NVM_ERROR_VERIFY            0x00000006   /* There is a mismatch between the source data and content in the checked flash memory */
#define NVM_ERROR_ALTERNATE         0x00000007   /* The operation is unsupported via alternate interface */
#define NVM_ERROR_FACTORY_OP        0x00000008   /* Factory program cannot be performed */
#define NVM_ERROR_BLOCK_INDICATOR   0x00000009   /* Invalid block indicator */
#define NVM_DONE                    0x00000010   /* The operation has been done and there is no more this operation requested on
                                                    FlashCheckStatus function. */
#define NVM_INPROGRESS              0x00000011   /* The operation is in progress and user need call FlashCheckStatus more times finish this
                                                    operation */
#define NVM_ERROR_EXTFUNC_INDICATOR 0x00000012   /* Invalid Extended Function indicator */
#define NVM_ERROR_MODE_OP           0x0000000E   /* Invalid mode op */


/*************************************************************************/
/*            Predefined values for flags, options variables             */
/*************************************************************************/

/* Checking mode used in FlashCheckStatus */
#define NVM_MODE_OP_PROGRAM         0x00
#define NVM_MODE_OP_PROGRAM_VERIFY  0x01
#define NVM_MODE_OP_CHECK_SUM       0x02
#define NVM_MODE_OP_USER_TEST_CHECK 0x03
#define NVM_MODE_OP_EXT             0x04
#define NVM_MODE_OP_EXT_DISABLE     0x05
#define NVM_MODE_OP_SWAP            0x06
#define NVM_MODE_OP_SWAP_DISABLE    0x07
#define NVM_MODE_OP_RFS             0x08
#define NVM_MODE_OP_RFS_DISABLE     0x09
#define NVM_MODE_OP_AOTA_SETUP      0x0A
#define NVM_MODE_OP_AOTA_END        0x0B

/* Indicators for getting/setting block lock state */
#define NVM_BLOCK_LOW               0x00   /* Block lock protection of low address space */
#define NVM_BLOCK_MID               0x01   /* Block lock protection of mid address space */
#define NVM_BLOCK_HIGH              0x02   /* Block lock protection of high address space */
#define NVM_BLOCK_256_FIRST         0x03   /* Block lock protection of first 32 blocks in 256K address space */
#define NVM_BLOCK_256_SECOND        0x04   /* Block lock protection of next blocks in 256K address space */
#define NVM_BLOCK_UTEST             0x05   /* Block lock protection of UTest address space */

/* Declarations for pcm suspend operation */
#define NVM_PGM_WRITE               0x11   /* A program sequence in interlock write stage. */
#define NVM_ERS_WRITE               0x12   /* An extended sequence in interlock write stage. */
#define NVM_ERS_SUS_PGM_WRITE       0x13   /* An extended-suspend program sequence in interlock write stage. */
#define NVM_PGM_SUS                 0x14   /* The program operation is in suspend state */
#define NVM_ERS_SUS                 0x15   /* The extended operation is in suspend state */
#define NVM_ERS_SUS_PGM_SUS         0x16   /* The extended-suspended program operation is in suspend state */



/*************************************************************************/
/*                   Other Macros for SSD functions                      */
/*************************************************************************/
#define NVM_WORD_SIZE               4      /* size of a word in byte */
#define NVM_DWORD_SIZE              8      /* size of a double word in byte */
#define NVM_RFS_DIRECT              0      /* refresh will be performed to the Direct array */
#define NVM_RFS_COMPLEMENTARY       1      /* refresh will be performed to the Complementary array */
#define NVM_AOTA_DIRECT             0      /* the Direct array is marked as over-the-air*/
#define NVM_AOTA_COMPLEMENTARY      1      /* the Complementary array is marked as over-the-air*/


#define NULL_POINTER                ((void *)0xFFFFFFFF)

#define NVM_REG_BIT_SET(address, mask)   \
     (*(VUINT32*)(address) |= (mask)); \
  	  asm ( "dmb" );
	 
#define NVM_REG_BIT_CLEAR(address, mask) \
      (*(VUINT32*)(address) &= ~(mask)); \
  	  asm ( "dmb" );

#define NVM_REG_BIT_TEST(address, mask)       (*(VUINT32*)(address) & (mask))

#define NVM_REG_WRITE(address, value)  \
     (*(VUINT32*)(address) = (value)); \
  	  asm ( "dmb" );

#define NVM_REG_READ(address)                 ((UINT32)(*(VUINT32*)(address)))


/* Set/Clear NVM-MCR bits without affecting MCR-EER, MCR-RWE, and MCR-SBC */
#define NVM_MCR_BIT_SET(MCRAddress, mask)      \
    NVM_REG_WRITE(MCRAddress, ((mask | NVM_REG_READ(MCRAddress)) & (~(NVM_MCR_EER | NVM_MCR_RWE | NVM_MCR_SBC)))); \
  	  asm ( "dmb" );
	

#define NVM_MCR_BIT_CLEAR(MCRAddress, mask)    \
    NVM_REG_WRITE(MCRAddress, (((~mask) & NVM_REG_READ(MCRAddress)) & (~(NVM_MCR_EER | NVM_MCR_RWE | NVM_MCR_SBC)))); \
  	  asm ( "dmb" );

#define NVM_MCR_EER_RWE_SBC_CLEAR(MCRAddress)    \
    NVM_REG_WRITE(MCRAddress, (NVM_REG_READ(MCRAddress) | NVM_MCR_EER | NVM_MCR_RWE | NVM_MCR_SBC)); \
  	  asm ( "dmb" );

	
#define NVM_LOW_BLOCK_NUM(pSSDConfig)     \
    (pSSDConfig->lowBlockInfo.n16KBlockNum + pSSDConfig->lowBlockInfo.n32KBlockNum +  \
    pSSDConfig->lowBlockInfo.n64KBlockNum)
	
#define NVM_MID_BLOCK_NUM(pSSDConfig)     \
    (pSSDConfig->midBlockInfo.n16KBlockNum + pSSDConfig->midBlockInfo.n32KBlockNum +  \
    pSSDConfig->midBlockInfo.n64KBlockNum)

#define NVM_HIGH_BLOCK_NUM(pSSDConfig)     \
    (pSSDConfig->highBlockInfo.n16KBlockNum + pSSDConfig->highBlockInfo.n32KBlockNum +  \
    pSSDConfig->highBlockInfo.n64KBlockNum)
	
    
/*************************************************************************/
/*                   SSD Configuration Structure                         */
/*************************************************************************/

/* Block information for an address space */
typedef struct _nvm_block_info
{
    UINT32 n16KBlockNum;         /* Number of 16K blocks */
    UINT32 n32KBlockNum;         /* Number of 32K blocks */
    UINT32 n64KBlockNum;         /* Number of 64K blocks */
	
} BLOCK_INFO, *PBLOCK_INFO;

/* SSD Configuration Structure */
typedef struct _nvm_ssd_config
{
    UINT32 NVMRegBase;           /* NVM control register base */    
	UINT32 mainArrayBase;        /* base of main array */   
	BLOCK_INFO lowBlockInfo;     /* blocks info of low address space */
    BLOCK_INFO midBlockInfo;     /* blocks info of mid address space */
    BLOCK_INFO highBlockInfo;    /* blocks info of high address space */   
	UINT32 n256BlockNum;         /* number of blocks in 256K address space */ 
	
    BOOL mainInterfaceFlag;      /* interface flag indicate main or alternate interface */
    UINT32 programmableSize;     /* programmable size */	
	BOOL BDMEnable;              /* debug mode selection */
} SSD_CONFIG, *PSSD_CONFIG;

/* MISR structure */
typedef struct _nvm_misr
{
    UINT32 W0;
    UINT32 W1;
    UINT32 W2;
    UINT32 W3;
    UINT32 W4;
    UINT32 W5;
    UINT32 W6;
    UINT32 W7;
    UINT32 W8;
    UINT32 W9;
} MISR, *PMISR;

/* Structure data for the context values */
typedef struct _nvm_context_data
{
    UINT32 dest;
    UINT32 size;
    UINT32 source;
    UINT32 *pFailedAddress;
    UINT32 *pFailedData;
    UINT32 *pFailedSource;
    UINT32 *pSum;
    PMISR  pMisr;
    void*  pReqCompletionFn;
} CONTEXT_DATA, *PCONTEXT_DATA;

/* Block select structure for 256 address space */
typedef struct _nvm_n256_block_sel
{
    UINT32 first256BlockSelect;
    UINT32 second256BlockSelect;
} N256_BLOCK_SEL, *PN256_BLOCK_SEL;

#define MAIN_REG_BASE_C0                0x711BC000
#define MAIN_REG_BASE_C1                0x717BC000
#define MAIN_REG_BASE_C2                0x711C4000

#define MAIN_ARRAY_BASE                 0x28000000 // Main array B0F0_START_ADDR


#define UTEST_ARRAY_BASE                0x29F80000
#define UTEST_ARRAY_SIZE                0x00008000

#define C55_PROGRAMMABLE_SIZE           0x80

#define BUFFER_SIZE_BYTE                0x1000

#define WRITE_STACK_SIZE                0x100

/* Lock State */
#define LOCK_ALL_BLOCKS                 0xFFFFFFFF
#define UNLOCK_ALL_BLOCKS               0x00000000

/* FLASH sectorization */

// Low Flash Blocks

/* Low space block 0 */
#define FLS_OFFSET_LOW_16K_BLOCK0	0x00FC4000	/* Offset of low block 0 */
#define FLS_SIZE_LOW_16K_BLOCK0		0x4000		/* 16KB size */

/* Low space block 1 */
#define FLS_OFFSET_LOW_16K_BLOCK1	0x00FCC000	/* Offset of low block 1 */
#define FLS_SIZE_LOW_16K_BLOCK1		0x4000		/* 16KB size */		

/* Low space block 2 */
#define FLS_OFFSET_LOW_16K_BLOCK2	0x00FC0000	/* Offset of low block 2 */
#define FLS_SIZE_LOW_16K_BLOCK2		0x4000		/* 16KB size */		

/* Low space block 3 */
#define FLS_OFFSET_LOW_16K_BLOCK3	0x00FC8000	/* Offset of low block 3 */
#define FLS_SIZE_LOW_16K_BLOCK3		0x4000		/* 16KB size */		

/* Low space block 4 */
#define FLS_OFFSET_LOW_16K_BLOCK4	0x0060C000	/* Offset of low block 4 */
#define FLS_SIZE_LOW_16K_BLOCK4		0x4000		/* 16KB size */		

/* Low space block 5 */
#define FLS_OFFSET_LOW_32K_BLOCK5	0x00FD0000	/* Offset of low block 5 */
#define FLS_SIZE_LOW_32K_BLOCK5		0x8000		/* 32KB size */

/* Low space block 6 */
#define FLS_OFFSET_LOW_32K_BLOCK6	0x00FD8000	/* Offset of low block 6 */
#define FLS_SIZE_LOW_32K_BLOCK6		0x8000		/* 32KB size */

/* Low space block 7 */
#define FLS_OFFSET_LOW_64K_BLOCK7	0x00FE0000	/* Offset of low block 7 */
#define FLS_SIZE_LOW_64K_BLOCK7		0x10000		/* 64KB size */	

/* Low space block 8 */
#define FLS_OFFSET_LOW_64K_BLOCK8	0x00FF0000	/* Offset of low block 8 */
#define FLS_SIZE_LOW_64K_BLOCK8		0x10000		/* 64KB size */	

/* Low space block 9 */
#define FLS_OFFSET_LOW_64K_BLOCK9	0x00610000	/* Offset of low block 9 */
#define FLS_SIZE_LOW_64K_BLOCK9		0x10000		/* 64KB size */	

/* Low space block 10 */
#define FLS_OFFSET_LOW_64K_BLOCK10	0x00620000	/* Offset of low block 10 */
#define FLS_SIZE_LOW_64K_BLOCK10	0x10000		/* 64KB size */	





/* Large space block 0 */
#define FLS_OFFSET_256K_BLOCK0    0x01000000  /* Offset of Large block 0 */
#define FLS_SIZE_256K_BLOCK0      0x20000     /* 128KB size */

/* Large space block 1 */
#define FLS_OFFSET_256K_BLOCK1    0x010200000  /* Offset of Large block 1 */    
#define FLS_SIZE_256K_BLOCK1      0x20000     /* 128KB size */

/* Large space block 2 */
#define FLS_OFFSET_256K_BLOCK2    0x01040000  /* Offset of Large block 2 */
#define FLS_SIZE_256K_BLOCK2      0x40000     /* 256KB size */

/* Large space block 3 */
#define FLS_OFFSET_256K_BLOCK3    0x01080000  /* Offset of Large block 3 */
#define FLS_SIZE_256K_BLOCK3      0x40000     /* 256KB size */

/* Large space block 4 */
#define FLS_OFFSET_256K_BLOCK4    0x010C0000  /* Offset of Large block 4 */
#define FLS_SIZE_256K_BLOCK4      0x40000     /* 256KB size */

/* Large space block 5 */
#define FLS_OFFSET_256K_BLOCK5    0x01100000  /* Offset of Large block 5 */
#define FLS_SIZE_256K_BLOCK5      0x40000     /* 256KB size */

/* Large space block 6 */
#define FLS_OFFSET_256K_BLOCK6    0x01140000  /* Offset of Large block 6 */
#define FLS_SIZE_256K_BLOCK6      0x40000     /* 256KB size */

/* Large space block 7 */
#define FLS_OFFSET_256K_BLOCK7    0x01180000  /* Offset of Large block 7 */
#define FLS_SIZE_256K_BLOCK7      0x40000     /* 256KB size */

/* Large space block 8 */
#define FLS_OFFSET_256K_BLOCK8    0x011C0000  /* Offset of Large block 8 */
#define FLS_SIZE_256K_BLOCK8      0x40000     /* 256KB size */

/* Large space block 9 */
#define FLS_OFFSET_256K_BLOCK9    0x01200000  /* Offset of Large block 9 */
#define FLS_SIZE_256K_BLOCK9      0x40000     /* 256KB size */

/* Large space block 10 */
#define FLS_OFFSET_256K_BLOCK10    0x01240000  /* Offset of Large block 10 */
#define FLS_SIZE_256K_BLOCK10      0x40000     /* 256KB size */

/* Large space block 11 */
#define FLS_OFFSET_256K_BLOCK11    0x01280000  /* Offset of Large block 11 */
#define FLS_SIZE_256K_BLOCK11      0x40000     /* 256KB size */

/* Large space block 12 */
#define FLS_OFFSET_256K_BLOCK12    0x012C0000  /* Offset of Large block 12 */
#define FLS_SIZE_256K_BLOCK12      0x40000     /* 256KB size */

/* Large space block 13 */
#define FLS_OFFSET_256K_BLOCK13    0x01300000  /* Offset of Large block 13 */
#define FLS_SIZE_256K_BLOCK13      0x40000     /* 256KB size */

/* Large space block 14 */
#define FLS_OFFSET_256K_BLOCK14    0x01340000  /* Offset of Large block 14 */
#define FLS_SIZE_256K_BLOCK14      0x40000     /* 256KB size */

/* Large space block 15 */
#define FLS_OFFSET_256K_BLOCK15    0x01380000  /* Offset of Large block 15 */
#define FLS_SIZE_256K_BLOCK15      0x40000     /* 256KB size */

/* Large space block 16 */
#define FLS_OFFSET_256K_BLOCK16    0x013C0000  /* Offset of Large block 16 */
#define FLS_SIZE_256K_BLOCK16      0x40000     /* 256KB size */

/* Large space block 17 */
#define FLS_OFFSET_256K_BLOCK17    0x01400000  /* Offset of Large block 17 */
#define FLS_SIZE_256K_BLOCK17      0x40000     /* 256KB size */

/* Large space block 18 */
#define FLS_OFFSET_256K_BLOCK18    0x01440000  /* Offset of Large block 18 */
#define FLS_SIZE_256K_BLOCK18      0x40000     /* 256KB size */

/* Large space block 19 */
#define FLS_OFFSET_256K_BLOCK19    0x01480000  /* Offset of Large block 19 */
#define FLS_SIZE_256K_BLOCK19      0x40000     /* 256KB size */

/* Large space block 20 */
#define FLS_OFFSET_256K_BLOCK20    0x014CC000  /* Offset of Large block 20 */
#define FLS_SIZE_256K_BLOCK20      0x40000     /* 256KB size */

/* Large space block 21 */
#define FLS_OFFSET_256K_BLOCK21    0x01500000  /* Offset of Large block 21 */
#define FLS_SIZE_256K_BLOCK21      0x40000     /* 256KB size */

/* Large space block 22 */
#define FLS_OFFSET_256K_BLOCK22    0x01540000  /* Offset of Large block 22 */
#define FLS_SIZE_256K_BLOCK22      0x40000     /* 256KB size */

/* Large space block 23 */
#define FLS_OFFSET_256K_BLOCK23    0x01580000  /* Offset of Large block 23 */
#define FLS_SIZE_256K_BLOCK23      0x40000     /* 256KB size */

/* Large space block 24 */
#define FLS_OFFSET_256K_BLOCK24    0x015C0000  /* Offset of Large block 24 */
#define FLS_SIZE_256K_BLOCK24      0x40000     /* 256KB size */

/* Large space block 25 */
#define FLS_OFFSET_256K_BLOCK25    0x01600000  /* Offset of Large block 25 */
#define FLS_SIZE_256K_BLOCK25      0x40000     /* 256KB size */

/* Large space block 26 */
#define FLS_OFFSET_256K_BLOCK26    0x01640000  /* Offset of Large block 26 */
#define FLS_SIZE_256K_BLOCK26      0x40000     /* 256KB size */

/* Large space block 27 */
#define FLS_OFFSET_256K_BLOCK27    0x01680000  /* Offset of Large block 27 */
#define FLS_SIZE_256K_BLOCK27      0x40000     /* 256KB size */

/* Large space block 28 */
#define FLS_OFFSET_256K_BLOCK28    0x016C0000  /* Offset of Large block 28 */
#define FLS_SIZE_256K_BLOCK28      0x40000     /* 256KB size */

/* Large space block 29 */
#define FLS_OFFSET_256K_BLOCK29    0x01700000  /* Offset of Large block 29 */
#define FLS_SIZE_256K_BLOCK29      0x40000     /* 256KB size */

/* Large space block 30 */
#define FLS_OFFSET_256K_BLOCK30    0x01740000  /* Offset of Large block 30 */
#define FLS_SIZE_256K_BLOCK30      0x40000     /* 256KB size */

/* Large space block 31 */
#define FLS_OFFSET_256K_BLOCK31    0x01780000  /* Offset of Large block 31 */
#define FLS_SIZE_256K_BLOCK31      0x40000     /* 256KB size */

/* Large space block 32 */
#define FLS_OFFSET_256K_BLOCK32    0x017C0000  /* Offset of Large block 32 */
#define FLS_SIZE_256K_BLOCK32      0x40000     /* 256KB size */

/* Large space block 33 */
#define FLS_OFFSET_256K_BLOCK33    0x01800000  /* Offset of Large block 33 */
#define FLS_SIZE_256K_BLOCK33      0x40000     /* 256KB size */

/* Large space block 34 */
#define FLS_OFFSET_256K_BLOCK34    0x01840000  /* Offset of Large block 34 */
#define FLS_SIZE_256K_BLOCK34      0x40000     /* 256KB size */

/* Large space block 35 */
#define FLS_OFFSET_256K_BLOCK35    0x01880000  /* Offset of Large block 35 */
#define FLS_SIZE_256K_BLOCK35      0x40000     /* 256KB size */

/* Large space block 36 */
#define FLS_OFFSET_256K_BLOCK36    0x018C0000  /* Offset of Large block 36 */
#define FLS_SIZE_256K_BLOCK36      0x40000     /* 256KB size */

/* Large space block 37 */
#define FLS_OFFSET_256K_BLOCK37    0x01900000  /* Offset of Large block 37 */
#define FLS_SIZE_256K_BLOCK37      0x40000     /* 256KB size */

/* Large space block 38 */
#define FLS_OFFSET_256K_BLOCK38    0x01940000  /* Offset of Large block 38 */
#define FLS_SIZE_256K_BLOCK38      0x40000     /* 256KB size */

/* Large space block 39 */
#define FLS_OFFSET_256K_BLOCK39    0x01980000  /* Offset of Large block 39 */
#define FLS_SIZE_256K_BLOCK39      0x40000     /* 256KB size */




//Data Flash Blocks

/* High space block 0 */
#define FLS_OFFSET_HIGH_64K_BLOCK0     0x00800000  /* Offset of high block 0 */
#define FLS_SIZE_HIGH_64K_BLOCK0       0x10000     /* 64KB size */

/* High space block 1 */
#define FLS_OFFSET_HIGH_64K_BLOCK1     0x00810000  /* Offset of high block 1 */
#define FLS_SIZE_HIGH_64K_BLOCK1       0x10000     /* 64KB size */

/* High space block 2 */
#define FLS_OFFSET_HIGH_64K_BLOCK2     0x00820000  /* Offset of high block 2 */
#define FLS_SIZE_HIGH_64K_BLOCK2       0x10000     /* 64KB size */

/* High space block 3 */
#define FLS_OFFSET_HIGH_64K_BLOCK3     0x00830000  /* Offset of high block 3 */
#define FLS_SIZE_HIGH_64K_BLOCK3       0x10000     /* 64KB size */


/* Mid space block 0 */
#define FLS_OFFSET_MID_16K_BLOCK0     0x00680000  /* Offset of mid block 0 */
#define FLS_SIZE_MID_16K_BLOCK0       0x4000      /* 16KB size */

/* Mid space block 1 */
#define FLS_OFFSET_MID_16K_BLOCK1     0x00684000  /* Offset of mid block 0 */  
#define FLS_SIZE_MID_16K_BLOCK1       0x4000      /* 16KB size */

#define FLASH_ARRAY_SIZE	  0x200000 /* Flash total size */

#define MIS_ALIGNED_VALUE               0x07
#define FlashOutRangeValue              0x00500000

#define FAIL  0
#define PASS  (!FAIL)

// BOOL Test_Result = FAIL;

/* Prototype of error trap funciton */
// static void ErrorTrap(UINT32 returnCode);

/* Typedef for null callback */
typedef void (*tpfNullCallback)(void);
