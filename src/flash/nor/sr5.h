#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdbool.h>
#include <stdint.h>
#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/cortex_m.h>


/*************************************************************************/
/*                  Offsets of C55 Control Registers                     */
/*************************************************************************/
#define C55_MCR             0x0000       /* Module Configuration Register */
#define C55_MCRA            0x0004       /* Alternate Module Configuration Register */
#define C55_MCRE            0x0008       /* Extended Module Configuration Register */
#define C55_LOCK0           0x0010       /* Lock 0 Register */
#define C55_LOCK1           0x0014       /* Lock 1 Register */
#define C55_LOCK2           0x0018       /* Lock 2 Register */
#define C55_LOCK3           0x001C       /* Lock 3 Register */
#define C55_LOCK0A          0x0028       /* Alternate Lock 0 Register */
#define C55_LOCK1A          0x002C       /* Alternate Lock 1 Register */
#define C55_SEL0            0x0038       /* Select 0 Register */
#define C55_SEL1            0x003C       /* Select 1 Register */
#define C55_SEL2            0x0040       /* Select 2 Register */
#define C55_SEL3            0x0044       /* Select 3 Register */
#define C55_ADR             0x0050       /* Address Register */
#define C55_UT0             0x0054       /* User Test 0 register */
#define C55_UM0             0x0058       /* User MISR 0 Register */
#define C55_UM1             0x005C       /* User MISR 1 Register */
#define C55_UM2             0x0060       /* User MISR 2 Register */
#define C55_UM3             0x0064       /* User MISR 3 Register */
#define C55_UM4             0x0068       /* User MISR 4 Register */
#define C55_UM5             0x006C       /* User MISR 5 Register */
#define C55_UM6             0x0070       /* User MISR 6 Register */
#define C55_UM7             0x0074       /* User MISR 7 Register */
#define C55_UM8             0x0078       /* User MISR 8 Register */
#define C55_UM9             0x007C       /* User MISR 9 Register */
#define C55_OPP0            0x0080       /* Over-Program Protection 0 Register */
#define C55_OPP1            0x0084       /* Over-Program Protection 1 Register */
#define C55_OPP2            0x0088       /* Over-Program Protection 2 Register */
#define C55_OPP3            0x008C       /* Over-Program Protection 3 Register */

/*************************************************************************/
/*              C55 Module Control Registers Field Definitions           */
/*************************************************************************/
/* Module Configuration Register */
#define C55_MCR_RVE         0x80000000   /* Read Voltage Error */
#define C55_MCR_RRE         0x40000000   /* Read Reference Error */
#define C55_MCR_AEE         0x20000000   /* Address Encode Error */
#define C55_MCR_EEE         0x10000000   /* ECC after ECC Error */
#define C55_MCR_EER         0x00008000   /* ECC Event Error */
#define C55_MCR_RWE         0x00004000   /* Read While Write Event Error */
#define C55_MCR_SBC         0x00002000   /* Single Bit Correction Error */
#define C55_MCR_PEAS        0x00000800   /* Program/Erase Access Space */
#define C55_MCR_DONE        0x00000400   /* State Machine Status */
#define C55_MCR_PEG         0x00000200   /* Program/Erase Good */
#define C55_MCR_PECIE       0x00000100   /* Program/Erase Complete Interrupt Enable */
#define C55_MCR_FERS        0x00000080   /* Factory Erase */
#define C55_MCR_PGM         0x00000010   /* Program */
#define C55_MCR_PSUS        0x00000008   /* Program Suspend */
#define C55_MCR_ERS         0x00000004   /* Erase */
#define C55_MCR_ESUS        0x00000002   /* Erase Suspend */
#define C55_MCR_EHV         0x00000001   /* Enable High Voltage */

/* User Test 0 Register */
#define C55_UT0_UTE         0x80000000
#define C55_UT0_SBCE        0x40000000
#define C55_UT0_NAIBP       0x00000200
#define C55_UT0_AIBPE       0x00000100
#define C55_UT0_AISUS       0x00000040
#define C55_UT0_MRE         0x00000020
#define C55_UT0_MRV         0x00000010
#define C55_UT0_AIS         0x00000004
#define C55_UT0_AIE         0x00000002
#define C55_UT0_AID         0x00000001

/*************************************************************************/
/*                   Return Codes for SSD functions                      */
/*************************************************************************/

#define C55_OK                      0x00000000
#define C55_ERROR_ALIGNMENT         0x00000001   /* Alignment error */
#define C55_ERROR_ENABLE            0x00000002   /* it's impossible to enable an operation */
#define C55_ERROR_BUSY              0x00000004   /* New program/erase cannot be preformed while previous high voltage operation in progress */
#define C55_ERROR_PGOOD             0x00000008   /* The program operation is unsuccessful */
#define C55_ERROR_EGOOD             0x00000010   /* The erase operation is unsuccessful */
#define C55_ERROR_NOT_BLANK         0x00000020   /* There is non-blank location in the checked flash memory */
#define C55_ERROR_VERIFY            0x00000040   /* There is a mismatch between the source data and content in the checked flash memory */
#define C55_ERROR_BLOCK_INDICATOR   0x00000080   /* Invalid block indicator */
#define C55_ERROR_ALTERNATE         0x00000100   /* The operation is unsupported via alternate interface */
#define C55_ERROR_FACTORY_OP        0x00000200   /* Factory erase/program cannot be performed */
#define C55_ERROR_MISMATCH          0x00000400   /* The MISR generated by the AIC hardware doesnot match the MISR passed by the user */
#define C55_ERROR_NO_BLOCK          0x00000800   /* No blocks have been enabled for Array Integrity check */
#define C55_ERROR_ADDR_SEQ          0x00001000   /* Invalid address sequence */
#define C55_ERROR_MARGIN_LEVEL      0x00002000   /* Invalid margin level */
#define C55_ERROR_ERASE_OPTION      0x00004000   /* Invalid erase option */
#define C55_ERROR_MODE_OP           0x00008000   /* Invalid mode op */

#define C55_DONE                    0x00010000   /* Done status */
#define C55_INPROGRESS              0x00020000   /* InProgress status */

/*************************************************************************/
/*            Predefined values for flags, options variables             */
/*************************************************************************/

/* Checking mode used in FlashCheckStatus */
#define C55_MODE_OP_PROGRAM         0x00
#define C55_MODE_OP_ERASE           0x01
#define C55_MODE_OP_PROGRAM_VERIFY  0x02
#define C55_MODE_OP_BLANK_CHECK     0x03
#define C55_MODE_OP_CHECK_SUM       0x04
#define C55_MODE_OP_USER_TEST_CHECK 0x05

/* Indicators for getting/setting block lock state */
#define C55_BLOCK_LOW               0x00   /* Block lock protection of low address space */
#define C55_BLOCK_MID               0x01   /* Block lock protection of mid address space */
#define C55_BLOCK_HIGH              0x02   /* Block lock protection of high address space */
#define C55_BLOCK_LARGE_FIRST       0x03   /* Block lock protection of first 32 blocks in 256K address space */
#define C55_BLOCK_LARGE_SECOND      0x04   /* Block lock protection of next 16 blocks in 256K address space */
#define C55_BLOCK_UTEST             0x05   /* Block lock protection of UTest address space */

/* Declarations for flash suspend operation */
#define C55_SUS_NOTHING             0x10   /* No program/erase operation */
#define C55_PGM_WRITE               0x11   /* A program sequence in interlock write stage. */
#define C55_ERS_WRITE               0x12   /* An erase sequence in interlock write stage. */
#define C55_ERS_SUS_PGM_WRITE       0x13   /* An erase-suspend program sequence in interlock write stage. */
#define C55_PGM_SUS                 0x14   /* The program operation is in suspend state */
#define C55_ERS_SUS                 0x15   /* The erase operation is in suspend state */
#define C55_ERS_SUS_PGM_SUS         0x16   /* The erase-suspended program operation is in suspend state */
#define C55_USER_TEST_SUS           0x17   /* The UTest check operation is in suspend state */

/* Declarations for flash resume operation */
#define C55_RES_NOTHING             0x20   /* No suspended program/erase operation */
#define C55_RES_PGM                 0x21   /* The program operation is resumed */
#define C55_RES_ERS                 0x22   /* The erase operation is resumed */
#define C55_RES_ERS_PGM             0x23   /* The erase-suspended program operation is resumed */
#define C55_RES_USER_TEST           0x24   /* The UTest check operation is resumed */

#define C55_USER_TEST_BREAK_SBC     0x30   /* The UTest check operation is broken by Single bit correction */
#define C55_USER_TEST_BREAK_DBD     0x31   /* The UTest check operation is broken by Double bit detection */

/* Declarations of margin levels */
#define C55_MARGIN_LEVEL_PROGRAM    0x00
#define C55_MARGIN_LEVEL_ERASE      0x01

/* Declarations of address sequences */
#define C55_ADDR_SEQ_PROPRIETARY    0x00
#define C55_ADDR_SEQ_LINEAR         0x01

/* Declarations of break options */
#define C55_BREAK_NONE              0x00   /* No break at all */
#define C55_BREAK_ON_DBD            0x01   /* Break on Double bit detection */
#define C55_BREAK_ON_DBD_SBC        0x02   /* Break on bot Double bit detection and Single bit correction */

/* Declarations of erase options */
#define C55_ERASE_MAIN              0x00   /* Perform normal erase on main array */
#define C55_ERASE_MAIN_FERS         0x01   /* Perform factory erase on main array */
#define C55_ERASE_UTEST             0x02   /* Perform erase on UTest array */
#define C55_ERASE_UTEST_FERS        0x03   /* Perform factory erase on UTest array */

/*************************************************************************/
/*                   Other Macros for SSD functions                      */
/*************************************************************************/

#define C55_WORD_SIZE               4      /* size of a word in byte */
#define C55_DWORD_SIZE              8      /* size of a double word in byte */

#define C55_USER_TEST_ENABLE_PASSWORD      0xF9F99999
#define C55_FACTORY_ERASE_DIARY_LOCATION   0x0020

#define NULL_POINTER                ((void *)0xFFFFFFFF)

/* Macros for Accessing the Registers */
#define C55_REG_BIT_SET(address, mask)        (*(uint32_t*)(address) |= (mask))
#define C55_REG_BIT_CLEAR(address, mask)      (*(uint32_t*)(address) &= ~(mask))
#define C55_REG_BIT_TEST(address, mask)       (*(uint32_t*)(address) & (mask))
#define C55_REG_WRITE(address, value)         (*(uint32_t*)(address) = (value))
#define C55_REG_READ(address)                 ((uint32_t)(*(uint32_t*)(address)))
#define C55_GET_BIT(value, position)          (uint8_t)(((value) >> (position)) & 0x01)

/* Set/Clear C55-MCR bits without affecting MCR-EER, MCR-RWE, and MCR-SBC */
#define C55_MCR_BIT_SET(MCRAddress, mask)      \
    C55_REG_WRITE(MCRAddress, ((mask | C55_REG_READ(MCRAddress)) & (~(C55_MCR_EER | C55_MCR_RWE | C55_MCR_SBC))))

#define C55_MCR_BIT_CLEAR(MCRAddress, mask)    \
    C55_REG_WRITE(MCRAddress, (((~mask) & C55_REG_READ(MCRAddress)) & (~(C55_MCR_EER | C55_MCR_RWE | C55_MCR_SBC))))

#define C55_MCR_EER_CLEAR(MCRAddress)    \
    C55_REG_WRITE(MCRAddress, (C55_REG_READ(MCRAddress) | C55_MCR_RWE | C55_MCR_SBC))

#define C55_MCR_RWE_CLEAR(MCRAddress)    \
    C55_REG_WRITE(MCRAddress, (C55_REG_READ(MCRAddress) | C55_MCR_EER | C55_MCR_SBC))

#define C55_MCR_SBC_CLEAR(MCRAddress)    \
    C55_REG_WRITE(MCRAddress, (C55_REG_READ(MCRAddress) | C55_MCR_EER | C55_MCR_RWE))

#define C55_MCR_EER_RWE_SBC_CLEAR(MCRAddress)    \
    C55_REG_WRITE(MCRAddress, (C55_REG_READ(MCRAddress) | C55_MCR_EER | C55_MCR_RWE | C55_MCR_SBC))



#define C55_MAIN_ARRAY_SIZE(pSSDConfig)    \
    ((((pSSDConfig->lowBlockInfo.n16KBlockNum + pSSDConfig->midBlockInfo.n16KBlockNum + \
    pSSDConfig->highBlockInfo.n16KBlockNum) << 4 /*16*/) + \
    ((pSSDConfig->lowBlockInfo.n32KBlockNum + pSSDConfig->midBlockInfo.n32KBlockNum + \
    pSSDConfig->highBlockInfo.n32KBlockNum) << 5 /*32*/) + \
    ((pSSDConfig->lowBlockInfo.n64KBlockNum + pSSDConfig->midBlockInfo.n64KBlockNum + \
    pSSDConfig->highBlockInfo.n64KBlockNum) << 6 /*64*/) + \
    ((pSSDConfig->lowBlockInfo.n128KBlockNum + pSSDConfig->midBlockInfo.n128KBlockNum + \
    pSSDConfig->highBlockInfo.n128KBlockNum) << 7 /*64*/) + \
    (pSSDConfig->nLargeBlockNum << 8)) << 10 /*1024*/)



#define C55_LOW_BLOCK_NUM(pSSDConfig)     \
    (pSSDConfig->lowBlockInfo.n16KBlockNum + pSSDConfig->lowBlockInfo.n32KBlockNum +  \
    pSSDConfig->lowBlockInfo.n64KBlockNum + pSSDConfig->lowBlockInfo.n128KBlockNum)

#define C55_MID_BLOCK_NUM(pSSDConfig)     \
    (pSSDConfig->midBlockInfo.n16KBlockNum + pSSDConfig->midBlockInfo.n32KBlockNum +  \
    pSSDConfig->midBlockInfo.n64KBlockNum + pSSDConfig->midBlockInfo.n128KBlockNum)

#define C55_HIGH_BLOCK_NUM(pSSDConfig)     \
    (pSSDConfig->highBlockInfo.n16KBlockNum + pSSDConfig->highBlockInfo.n32KBlockNum +  \
    pSSDConfig->highBlockInfo.n64KBlockNum + pSSDConfig->highBlockInfo.n128KBlockNum)


/*************************************************************************/
/*                   SSD Configuration Structure                         */
/*************************************************************************/

/* Block information for an address space */

typedef struct _c55_block_info
{
    uint32_t n16KBlockNum;         /* Number of 16K blocks */
    uint32_t n32KBlockNum;         /* Number of 32K blocks */
    uint32_t n64KBlockNum;         /* Number of 64K blocks */
    uint32_t n128KBlockNum;        /* Number of 128K blocks */

} BLOCK_INFO, *PBLOCK_INFO;


/* SSD Configuration Structure */
typedef struct _c55_ssd_config
{
    uint32_t c55RegBase;           /* C55 control register base */
    uint32_t mainArrayBase;        /* base of main array */
    BLOCK_INFO lowBlockInfo;     /* blocks info of low address space */
    BLOCK_INFO midBlockInfo;     /* blocks info of mid address space */
    BLOCK_INFO highBlockInfo;    /* blocks info of high address space */

	uint32_t nLargeBlockNum;        /* number of blocks in Large address space */

	uint32_t uTestArrayBase;       /* base of UTEST array */
    uint8_t mainInterfaceFlag;      /* interface flag indicate main or alternate interface */
    uint32_t programmableSize;     /* programmable size */
    uint8_t BDMEnable;              /* debug mode selection */
} SSD_CONFIG, *PSSD_CONFIG;


/* MISR structure */
typedef struct _c55_misr
{
    uint32_t W0;
    uint32_t W1;
    uint32_t W2;
    uint32_t W3;
    uint32_t W4;
    uint32_t W5;
    uint32_t W6;
    uint32_t W7;
    uint32_t W8;
    uint32_t W9;
} MISR, *PMISR;

/* Structure data for the context values */
typedef struct _c55_context_data
{
    uint32_t dest;
    uint32_t size;
    uint32_t source;
    uint32_t *pFailedAddress;
    uint32_t *pFailedData;
    uint32_t *pFailedSource;
    uint32_t *pSum;
    PMISR pMisr;
    void* pReqCompletionFn;
} CONTEXT_DATA, *PCONTEXT_DATA;

/* Block select structure for Large address space */
typedef struct _c55_nLarge_block_sel
{
    uint32_t firstLargeBlockSelect;
    uint32_t secondLargeBlockSelect;
} NLARGE_BLOCK_SEL, *PNLARGE_BLOCK_SEL;


typedef uint32_t (*PFLASHPROGRAM) ( PSSD_CONFIG pSSDConfig,
                    bool factoryPgmFlag,
                    uint32_t dest,
                    uint32_t size,
                    uint32_t source,
                    PCONTEXT_DATA pCtxData
                    );

#define MAIN_REG_BASE                    0x40018000

#define MAIN_ARRAY_BASE                 0x18000000 // Main array B0F0_START_ADDR


#define UTEST_ARRAY_BASE                0x1FF80000
#define UTEST_ARRAY_SIZE                0x00004000

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

// bool Test_Result = FAIL;

/* Prototype of error trap funciton */
// static void ErrorTrap(uint32_t returnCode);

/* Typedef for null callback */
typedef void (*tpfNullCallback)(void);
