
#ifndef COMMON_INC_CONFIG_H
#define COMMON_INC_CONFIG_H

#define PI                      3.14159265
#define BLOCK_SIZE              4
#define MB_BLOCK_SIZE           16
#define DC_BITDEPTH             10
// Macros for encoder/decoder
#define WHOLEFLOW               1
#define AC_QSTEP                1
#define RESIDUAL_CODING         1
#define SKIP_MODE               1
#define MODE_DECISION           1
#define INTEGER_DCT             1
#define HARDWARE_FLOW           1
#define HARDWARE_QUANTIZATION   1
#define HARDWARE_CMS            1
#define HARDWARE_OPT            1
#define BIDIRECT_REFINEMENT     0
#define SI_REFINEMENT           1
#define ISADAPTIVE				0
#define HEAPSORT				0
#define ACOVERLAP				0.0
#define CRC						1					//Whether CRC validation is used
#if CRC
#define CRCLENGTH				1584
#define CRCWRONGWEIGHT			-50
#endif // CRC
#define MYNODECOUNT				2048				//Use to set the size of nodecount for decoding
/*==============================The following parameters are adjusted for different video sequences===========*/
#define ISCHANGE				0					//Whether overlap is variable
#define PSOTION					2					//The size of overlap change, and the last X+1 bit is AC algorithm
#define OVERLAP					0.02				//For different sequence, overlap is different!!!
#define HIGHMOTACT				1					//For high motion active (like soccer sequence)

/* For test sequence

Hall Monitor 
 ISCHANGE				0					
 PSOTION				2					
 OVERLAP				0.08				
 HIGHMOTACT				0

 Coast Guard
 ISCHANGE				0
 PSOTION				2
 OVERLAP				0.10
 HIGHMOTACT				0

 Foreman
 ISCHANGE				1
 PSOTION				2
 OVERLAP				0.025
 HIGHMOTACT				0

 Soccer
 ISCHANGE				0
 PSOTION				2
 OVERLAP				0.02
 HIGHMOTACT				1
*/

// Macros for encoder only
#ifdef ENCODER
# define TESTPATTERN            1
# define DEBUG                  0
#endif

// Macros for decoder only
#ifdef DECODER
# define INVERSE_MATRIX         1
#endif

#include "types.h"
#endif // COMMON_INC_CONFIG_H

