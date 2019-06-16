/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _CUSTOM_FMT_H_
#define _CUSTOM_FMT_H_

/*********************************************************************
 * INCLUES
 */
#include <string.h>

#include <xdc/std.h>


/*********************************************************************
 * FUNCTIONS
 */

/*
 * @brief   Formats a U32 value to ASCII, and appends a str.
 *          The final format is stored in pOut.
 *
 * @param[out]  pOut   The memory area to store to.
 * @param[in]   value  U32 value to format to ASCII.
 * @param[in]   pStr   String to append after value.
 *
 * @return  None.
 */
static inline
void itoaAppendStr(char *pOut, uint32_t value, char *pStr)
{
    // 10 is the max number of digits in a 32-bit value
    #define _MAXSIZE 10
    char pLine[_MAXSIZE], *pCurr = pLine + _MAXSIZE;
    uint8_t i = 0;

    // Value to str fmt, going from right to left in pLine
    do {
        *--pCurr = (char)((value % 10UL) + '0');
        i++;
    } while ((value /= 10) != 0);

    // Copy fmt to pOut
    memcpy(pOut,     pCurr, i);                // Digit
    memcpy(pOut + i, pStr,  strlen(pStr) + 1); // Unit
    #undef _MAXSIZE
}

#endif /* _CUSTOM_FMT_H_ */
