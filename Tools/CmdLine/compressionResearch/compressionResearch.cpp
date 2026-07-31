/*
  File:     compressionResearch.cpp

  Contains:   Console app to test compression of profile tags
    Output is meant to go in a spreadsheet, and may be too large to view in your terminal/console.

  Version:  V1

  Copyright:  (c) see below
*/

/*
 * The ICC Software License, Version 0.2
 *
 *
 * Copyright (c) 2003-2026 The International Color Consortium. All rights
 * reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 *
 * 3. In the absence of prior written permission, the names "ICC" and "The
 *  International Color Consortium" must not be used to imply that the
 *  ICC organization endorses or promotes products derived from this
 *  software.
 *
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE INTERNATIONAL COLOR CONSORTIUM OR
 * ITS CONTRIBUTING MEMBERS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 * ====================================================================
 *
 * This software consists of voluntary contributions made by many
 * individuals on behalf of the The International Color Consortium.
 *
 *
 * Membership in the ICC is encouraged when this software is used for
 * commercial purposes.
 *
 *
 * For more information on The International Color Consortium, please
 * see <http://www.color.org/>.
 *
 *
 */

#include <cstdio>
#include <cstdarg>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cmath>
#include <limits>
#include <string>
#include <memory>
#include <algorithm>
#include "IccProfile.h"
#include "IccTag.h"
#include "IccUtil.h"
#include "IccProfLibVer.h"
#include "../IccCmdLineUtil.h"

#ifdef ICC_USE_ZLIB
#include "zlib.h"
#endif

/*
NOTE
Profiling: 94.8% in deflate
            3.0% in all predictors
            1.9% in FindTag/LoadTag

NEXT: test on many, many profiles, find best predictors

FUTURE: test LZMA compressor

 */

/******************************************************************************/

// calling types for predictors, will have different wrapper function for each type
enum predictor_type {
    PREDICTOR_TYPE_NULL = 0,
    PREDICTOR_TYPE_1D = 1,
    PREDICTOR_TYPE_2D = 2,
    PREDICTOR_TYPE_3D = 3,      // may be too much hassle for small benefit
};


typedef void func_predictor( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                            size_t size1, size_t size2, size_t size3,
                            size_t colStep, size_t rowStep, size_t planeStep );

/******************************************************************************/

// forward declarations of predictor functions
// 1D
static func_predictor null_forward;
static func_predictor null_reverse;
static func_predictor prev_forward;
static func_predictor prev_reverse;
static func_predictor bytesplitPrev_forward;
static func_predictor bytesplitPrev_reverse;

// 2D
static func_predictor up_forward;
static func_predictor up_reverse;
static func_predictor prev2D_forward;
static func_predictor prev2D_reverse;
static func_predictor min_forward;
static func_predictor min_reverse;
static func_predictor avgUpLeft_forward;
static func_predictor avgUpLeft_reverse;
static func_predictor median3_forward;
static func_predictor median3_reverse;
static func_predictor MED_forward;
static func_predictor MED_reverse;
static func_predictor Paeth_forward;
static func_predictor Paeth_reverse;
static func_predictor MinGrad_forward;
static func_predictor MinGrad_reverse;

// 3D
static func_predictor min3D_forward;
static func_predictor min3D_reverse;
static func_predictor median3D_forward;
static func_predictor median3D_reverse;

// utilities
static void bytesplit16( const uint8_t *input, uint8_t *output, size_t count );
static void byteunsplit16( const uint8_t *input, uint8_t *output, size_t count );
void LogAnError(FILE *stream, const char* format, ...);

/******************************************************************************/

template<func_predictor fun>
void splitwrap(const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t size3,
                size_t colStep, size_t rowStep, size_t planeStep)
{
    if (bitDepth == 8)
        fun(input,output,8,channels,size1,size2,size3,colStep,rowStep,planeStep);

    if (bitDepth == 16) {
      if ((int)colStep != channels) {
        LogAnError(stderr,"ERROR - split forward cannot be used on non-linear steps\n" );
        // at least not without a lot more code and testing...
      }
      size2 = size2 ? size2 : 1;
      size3 = size3 ? size3 : 1;
      size_t half = size1*size2*size3*channels;
      std::vector<uint8_t> temp( 2*half );
      fun(input,&temp[0],16,channels,size1,size2,size3,colStep,rowStep,planeStep);
      bytesplit16( &temp[0], output, half );
    }
}

template<func_predictor fun>
void unsplitwrap(const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t size3,
                size_t colStep, size_t rowStep, size_t planeStep)
{
    if (bitDepth == 8)
        fun(input,output,8,channels,size1,size2,size3,colStep,rowStep,planeStep);

    if (bitDepth == 16) {
      if ((int)colStep != channels) {
        LogAnError(stderr,"ERROR - split reverse cannot be used on non-linear steps\n" );
      }
      size2 = size2 ? size2 : 1;
      size3 = size3 ? size3 : 1;
      size_t half = size1*size2*size3*channels;
      std::vector<uint8_t> temp( 2*half );
      byteunsplit16( input, &temp[0], half );
      fun(&temp[0],output,16,channels,size1,size2,size3,colStep,rowStep,planeStep);
    }
}

/******************************************************************************/

struct predictor_desc {
    const char *name;
    predictor_type type;
    func_predictor *forward;
    func_predictor *reverse;
};

std::vector<predictor_desc> predictorList =
{
 { "None", PREDICTOR_TYPE_NULL, null_forward, null_reverse },

#if 0 && !defined(NDEBUG)
// these test the outer loops of the predictors
 { "None1", PREDICTOR_TYPE_1D, null_forward, null_reverse },
 { "None2", PREDICTOR_TYPE_2D, null_forward, null_reverse },
 { "None3", PREDICTOR_TYPE_3D, null_forward, null_reverse },
#endif

 { "Previous", PREDICTOR_TYPE_1D, prev_forward, prev_reverse },


 { "Up", PREDICTOR_TYPE_2D, up_forward, up_reverse },
 { "Previous2D", PREDICTOR_TYPE_2D, prev2D_forward, prev2D_reverse },
 { "Min", PREDICTOR_TYPE_2D, min_forward, min_reverse },
 { "AvgUpLeft", PREDICTOR_TYPE_2D, avgUpLeft_forward, avgUpLeft_reverse },
 { "Median", PREDICTOR_TYPE_2D, median3_forward, median3_reverse },
 { "MED", PREDICTOR_TYPE_2D, MED_forward, MED_reverse },
 { "Paeth", PREDICTOR_TYPE_2D, Paeth_forward, Paeth_reverse },
 { "MinGrad", PREDICTOR_TYPE_2D, MinGrad_forward, MinGrad_reverse },

// splits should only be used if depth > 8
 { "SplitPrev", PREDICTOR_TYPE_1D, bytesplitPrev_forward, bytesplitPrev_reverse },
 { "PrevSplit", PREDICTOR_TYPE_1D, splitwrap<prev_forward>, unsplitwrap<prev_reverse> },
 { "UpByteSplit", PREDICTOR_TYPE_2D, splitwrap<up_forward>, unsplitwrap<up_reverse> },
 { "Prev2DByteSplit", PREDICTOR_TYPE_2D, splitwrap<prev2D_forward>, unsplitwrap<prev2D_reverse> },
 { "MinByteSplit", PREDICTOR_TYPE_2D, splitwrap<min_forward>, unsplitwrap<min_reverse> },
 { "AvgUpLeftSplit", PREDICTOR_TYPE_2D, splitwrap<avgUpLeft_forward>, unsplitwrap<avgUpLeft_reverse> },
 { "MedianSplit", PREDICTOR_TYPE_2D, splitwrap<median3_forward>, unsplitwrap<median3_reverse> },
 { "MEDSplit", PREDICTOR_TYPE_2D, splitwrap<MED_forward>, unsplitwrap<MED_reverse> },
 { "PaethSplit", PREDICTOR_TYPE_2D, splitwrap<Paeth_forward>, unsplitwrap<Paeth_reverse> },
 { "MinGradSplit", PREDICTOR_TYPE_2D, splitwrap<MinGrad_forward>, unsplitwrap<MinGrad_reverse> },

 { "Min3D", PREDICTOR_TYPE_3D, min3D_forward, min3D_reverse },
 { "Median3D", PREDICTOR_TYPE_3D, median3D_forward, median3D_reverse },

// splits should only be used if depth > 8
 { "Min3DSplit", PREDICTOR_TYPE_3D, splitwrap<min3D_forward>, unsplitwrap<min3D_reverse> },
 { "Median3DSplit", PREDICTOR_TYPE_3D, splitwrap<median3D_forward>, unsplitwrap<median3D_reverse> },
 
 

// on old notes: pred then bytesplit usually compresses better
// except on floating point data (32 bit integer diff fails)

// TODO - should bytesplit reorder data to planar?
// TODO - gamut specific binary encoding?  LZ should already do that.
};

/******************************************************************************/
/******************************************************************************/

// command line option to disable warning and error reports
bool gRunSilent = false;

// internal option - should only be used by client code that wants to display errors separately
bool gLogErrorsToString = false;

// global storage of accumulated error reports
// abstracted below because this can easily become more complicated in the future
std::string gErrorLogs;

/******************************************************************************/

void ClearErrorLogs()
{
  gErrorLogs.clear();
}

/******************************************************************************/

std::string &GetErrorLogs()
{
  return gErrorLogs;
}

/******************************************************************************/

// currently just used by LogAnError, but could be exported if needed
static
void AddErrorStringToLog(const std::string &input)
{
  gErrorLogs += input;
}

/******************************************************************************/

void LogAnError(FILE *stream, const char* format, ...)
{
  try {
    if (gLogErrorsToString) {
      std::va_list args;
      va_start(args, format);
      const size_t bufSize = 4096;      // could also print twice to get size, but this is simpler
      char buf [ bufSize ];
      auto len = std::vsnprintf(buf, bufSize, format, args);
      if (len > 0)
        AddErrorStringToLog( buf );
      else
        AddErrorStringToLog( "Internal buffer error while formatting: \"" + std::string(format) + "\"\n" );
      va_end(args);
      return;
    }

    // are we running silent (but not deep)?
    if (gRunSilent)
      return;

    // else normal output
    std::va_list args;
    va_start(args, format);
    (void)std::vfprintf(stream, format, args);
    va_end(args);
  }
  catch(...) {
    // don't let any exceptions escape, don't rethrow
  }

}

/******************************************************************************/
/******************************************************************************/

static
bool inflateBuffer( uint8_t *input, uint8_t *output, size_t in_bytes, size_t &out_bytes )
{

#ifndef ICC_USE_ZLIB
  return false;
#else
  int zstat;
  z_stream zstr;
  memset(&zstr, 0, sizeof(zstr));

  zstat = inflateInit(&zstr);
  if (zstat != Z_OK) {
    return false;
  }

  zstat = inflateReset(&zstr);
  if (zstat != Z_OK) {
    inflateEnd(&zstr);
    return false;
  }

  zstr.next_in = input;
  zstr.avail_in = (uInt) in_bytes;

  do {
    zstr.next_out = output;
    zstr.avail_out = (uInt) out_bytes;

    zstat = inflate(&zstr, Z_SYNC_FLUSH);
    if (zstat != Z_OK && zstat != Z_STREAM_END) {
      inflateEnd(&zstr);
      return false;
    }

  } while (zstat != Z_STREAM_END && zstr.avail_in > 0);

  out_bytes = zstr.total_out;

  inflateEnd(&zstr);
#endif

  return true;
}

/******************************************************************************/

static
bool deflateBuffer( uint8_t *input, uint8_t *output, size_t in_bytes, size_t &out_bytes, int level = 9 )
{

#ifndef ICC_USE_ZLIB
  return false;
#else
  int zstat;
  z_stream zstr;
  memset(&zstr, 0, sizeof(zstr));

  zstat = deflateInit(&zstr, level);
  if (zstat != Z_OK) {
    return false;
  }

  zstat = deflateReset(&zstr);
  if (zstat != Z_OK) {
    deflateEnd(&zstr);
    return false;
  }

  zstr.next_in = input;
  zstr.avail_in = (uInt) in_bytes;

  do {
    zstr.next_out = output;
    zstr.avail_out = (uInt) out_bytes;

    zstat = deflate(&zstr, Z_SYNC_FLUSH);
    if (zstat != Z_OK && zstat != Z_STREAM_END) {
      inflateEnd(&zstr);
      return false;
    }

  } while (zstat != Z_STREAM_END && zstr.avail_in > 0);

  out_bytes = zstr.total_out;

  deflateEnd(&zstr);
#endif

  return true;
}

/******************************************************************************/
/******************************************************************************/

static
uint8_t ClipU8( const icFloatNumber &input )
{
  if (std::isnan(input))
    return 0;
  if (std::isinf(input))
    return 255;
  if (input < 0)
    return 0;
  if (input > 255)
    return 255;
  return (uint8_t)input;
}

/******************************************************************************/

static
uint16_t ClipU16( const icFloatNumber &input )
{
  if (std::isnan(input))
    return 0;
  if (std::isinf(input))
    return 65535;
  if (input < 0)
    return 0;
  if (input > 65535)
    return 65535;
  return (uint16_t)input;
}

/******************************************************************************/

static
icFloatNumber ClipFloat( const icFloatNumber &input )
{
  if (std::isnan(input))
    return 0;
  if (std::isinf(input))
    return 1000.0;
  return input;
}

/******************************************************************************/

// sort so y is the median
// which version is faster depends on the compiler and vectorization
template <typename T>
T median3( T x, T y, T z )
{
#if 0
  if (x > y)
    std::swap(x,y);

  // now y >= x
  if (y > z) {
    std::swap(y,z);

    // now z >= y  (but we changed y, so recompare with x)
    if (x > y)
      std::swap(x,y);
  }
  return y;
#else
  return std::max(std::min(x, y), std::min(std::max(x, y), z));
#endif
}

/******************************************************************************/
/******************************************************************************/

void apply1DPredictor( const predictor_desc &pred, const uint8_t *input, uint8_t *output,
                        uint32_t *dimArray, uint8_t nDimensions, int bitDepth,
                        int channels, size_t pixelCount, bool reverse )
{
  func_predictor *predFunc = reverse ? pred.reverse : pred.forward;
  
  // shortcut the simplest case
  if (nDimensions == 1) {
    predFunc( input, output, bitDepth, channels, pixelCount, 0, 0, channels, 0, 0 );
    return;
  }

  size_t tiles = 1;
  for (int i = (int)nDimensions-1; i > 0; --i) {
    size_t temp = dimArray[i];
    tiles *= temp;
  }

  size_t width = dimArray[ 0 ];

  if (tiles * width != pixelCount) {
    LogAnError(stderr,"ERROR - 1D tile and pixel counts do not match (%zu, %zu)\n", pixelCount, tiles*width );
  }

  size_t increment = width * (bitDepth/8) * channels;

  for (size_t k = 0; k < tiles; ++k) {
    predFunc( input, output, bitDepth, channels, width, 0, 0, channels, 0, 0 );
    input += increment;
    output += increment;
  }

}

/******************************************************************************/

void apply2DPredictor( const predictor_desc &pred, const uint8_t *input, uint8_t *output,
                        uint32_t *dimArray, uint8_t nDimensions, uint8_t bitDepth,
                        uint8_t channels, size_t pixelCount, bool reverse )
{
  func_predictor *predFunc = reverse ? pred.reverse : pred.forward;

  size_t tiles = 1;
  for (int i = (int)nDimensions-1; i > 1; --i) {
    size_t temp = dimArray[i];
    tiles *= temp;
  }

  size_t width = dimArray[ 0 ];
  size_t height = (nDimensions > 1) ? dimArray[ 1 ] : 1;

  if (tiles * width * height != pixelCount) {
    LogAnError(stderr,"ERROR - 2D tile and pixel counts do not match (%zu, %zu)\n", pixelCount, tiles * width * height);
  }

  size_t increment = width * height * (bitDepth/8) * channels;

  for (size_t k = 0; k < tiles; ++k) {
    predFunc( input, output, bitDepth, channels, width, height, 0, channels, channels*width, 0 );
    input += increment;
    output += increment;
  }

}

/******************************************************************************/

void apply3DPredictor( const predictor_desc &pred, const uint8_t *input, uint8_t *output,
                        uint32_t *dimArray, uint8_t nDimensions, uint8_t bitDepth,
                        uint8_t channels, size_t pixelCount, bool reverse )
{
  func_predictor *predFunc = reverse ? pred.reverse : pred.forward;

  size_t tiles = 1;
  for (int i = (int)nDimensions-1; i > 2; --i) {
    size_t temp = dimArray[i];
    tiles *= temp;
  }

  size_t width = dimArray[ 0 ];
  size_t height = (nDimensions > 1) ? dimArray[ 1 ] : 1;
  size_t depth = (nDimensions > 2) ? dimArray[ 2 ] : 1;

  if (tiles * width * height * depth != pixelCount) {
    LogAnError(stderr,"ERROR - 3D tile and pixel counts do not match (%zu, %zu)\n", pixelCount, tiles * width * height * depth);
  }

  size_t increment = width * height * depth * (bitDepth/8) * channels;

  for (size_t k = 0; k < tiles; ++k) {
    predFunc( input, output, bitDepth, channels, width, height, depth, channels, channels*width, channels*width*height );
    input += increment;
    output += increment;
  }
}

/******************************************************************************/

// call different wrapper functions based on each type
static
void applyOnePredictor( const predictor_desc &pred, const uint8_t *input, uint8_t *output,
                        uint32_t *dimArray, uint8_t nDimensions, int bitDepth, int channels,
                        size_t pixelCount, bool reverse = false )
{
  func_predictor *predFunc = pred.forward;
  if (reverse)
    predFunc = pred.reverse;

  switch (pred.type) {
    case PREDICTOR_TYPE_NULL:
      predFunc(input,output,bitDepth,channels,pixelCount,0,0,channels,0,0);      // needs no wrapper, just copy the whole array
      break;

    case PREDICTOR_TYPE_1D:
      apply1DPredictor( pred, input, output, dimArray, nDimensions, bitDepth, channels, pixelCount, reverse );
      break;

    case PREDICTOR_TYPE_2D:
      apply2DPredictor( pred, input, output, dimArray, nDimensions, bitDepth, channels, pixelCount, reverse );
      break;

    case PREDICTOR_TYPE_3D:
      apply3DPredictor( pred, input, output, dimArray, nDimensions, bitDepth, channels, pixelCount, reverse );
      break;
    
    default:
      LogAnError(stderr,"%s: ERROR - unknown or unimplemented predictor type %d\n", pred.type );
      break;
  }

}

/******************************************************************************/

static
void bytesplit16( const uint8_t *input, uint8_t *output, size_t count )
{
  // rearrange bytes: assumes little endian byte order, reorganized to big endian (sort of)
  for (size_t i = 0; i < count; ++i)
    output[i] = input[2*i+1];
  for (size_t i = 0; i < count; ++i)
    output[count+i] = input[2*i+0];
}

/******************************************************************************/

static
void byteunsplit16( const uint8_t *input, uint8_t *output, size_t count )
{
  // rearrange bytes: assumes little endian byte order, reorganized from big endian (sort of)
  for (size_t i = 0; i < count; ++i) {
    output[2*i+0] = input[count+i];
    output[2*i+1] = input[i];
  }
}

/******************************************************************************/

static
void null_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t size3,
                size_t /*colStep*/, size_t /*rowStep*/, size_t /*planeStep*/ )
{
  size2 = size2 ? size2 : 1;
  size3 = size3 ? size3 : 1;
  size_t totalBytes = size1*size2*size3*(bitDepth/8)*channels;
  memcpy( output, input, totalBytes );
}

/******************************************************************************/

static
void null_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t size3,
                size_t /*colStep*/, size_t /*rowStep*/, size_t /*planeStep*/ )
{
  size2 = size2 ? size2 : 1;
  size3 = size3 ? size3 : 1;
  size_t totalBytes = size1*size2*size3*(bitDepth/8)*channels;
  memcpy( output, input, totalBytes );
}

/******************************************************************************/

static
void prev_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t /*size2*/, size_t /*size3*/,
                size_t colStep, size_t /*rowStep*/, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    for (int c = 0; c < channels; ++c) {    // copy first pixel
      output[c] = input[c];
    }
    for (size_t x = 1; x < size1; ++x) {
      const uint8_t *in = input + x*colStep;
      const uint8_t *prev = input + (x-1)*colStep;
      uint8_t *out = output + x*colStep;
      for (int c = 0; c < channels; ++c) {
        out[c] = in[c] - prev[c];   // overflow/underflow is intentional
      }
    }
  }

  if (bitDepth == 16) {
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (int c = 0; c < channels; ++c) {    // copy first pixel
      output16[c] = input16[c];
    }
    for (size_t x = 1; x < size1; ++x) {
      const uint16_t *in = input16 + x*colStep;
      const uint16_t *prev = input16 + (x-1)*colStep;
      uint16_t *out = output16 + x*colStep;
      for (int c = 0; c < channels; ++c) {
        out[c] = in[c] - prev[c];   // overflow/underflow is intentional
      }
    }
  }
}

/******************************************************************************/

static
void prev_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t /*size2*/, size_t /*size3*/,
                size_t colStep, size_t /*rowStep*/, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    for (int c = 0; c < channels; ++c) {    // copy first pixel
      output[c] = input[c];
    }
    for (size_t x = 1; x < size1; ++x) {
      const uint8_t *in = input + x*colStep;
      const uint8_t *prev = output + (x-1)*colStep;
      uint8_t *out = output + x*colStep;
      for (int c = 0; c < channels; ++c) {
        out[c] = in[c] + prev[c];   // overflow/underflow is intentional
      }
    }
  }

  if (bitDepth == 16) {
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (int c = 0; c < channels; ++c) {    // copy first pixel
      output16[c] = input16[c];
    }
    for (size_t x = 1; x < size1; ++x) {
      const uint16_t *in = input16 + x*colStep;
      const uint16_t *prev = output16 + (x-1)*colStep;
      uint16_t *out = output16 + x*colStep;
      for (int c = 0; c < channels; ++c) {
        out[c] = in[c] + prev[c];   // overflow/underflow is intentional
      }
    }
  }
}

/******************************************************************************/

static
void bytesplitPrev_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t /*size2*/, size_t /*size3*/,
                size_t colStep, size_t /*rowStep*/, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    prev_forward(input,output,8,channels,size1,0,0,colStep,0,0);
    return;
  }

  if (bitDepth == 16) {
    if ((int)colStep != channels) {
      LogAnError(stderr,"ERROR - bytesplit reverse cannot be used on non-linear steps\n" );
      // at least not without a lot more code and testing...
    }
    // rearrange bytes: assumes little endian byte order, reorganized into big endian (sort of)
    // NOTE - this operation cannot be done inplace
    //          because prev is not designed to run inplace (else we could write to output)
    size_t half = size1*channels;
    std::vector<uint8_t> temp( 2*half );
    bytesplit16( input, &temp[0], half );
    prev_forward( &temp[0],output,8,channels,2*size1,0,0,colStep,0,0);
  }
}

/******************************************************************************/

static
void bytesplitPrev_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t /*size2*/, size_t /*size3*/,
                size_t colStep, size_t /*rowStep*/, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    prev_reverse(input,output,8,channels,size1,0,0,colStep,0,0);
    return;
  }

  if (bitDepth == 16) {
    if ((int)colStep != channels) {
      LogAnError(stderr,"ERROR - bytesplit reverse cannot be used on non-linear steps\n" );
    }
    // NOTE - this operation cannot be done inplace
    //          because prev is not designed to run inplace (else we could write to input)
    size_t half = size1*channels;
    std::vector<uint8_t> temp( 2*half );
    prev_reverse(input,&temp[0],8,channels,2*size1,0,0,colStep,0,0);
    byteunsplit16( &temp[0], output, half );
  }
}

/******************************************************************************/

// for normal call: colStep = channels, rowStep = size1*channels, planeStep = size1*size2*channels
static
void up_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_forward( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *prevY = input + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      for (size_t x = 0; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = prevY + x*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] - prevX[c];   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_forward( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *prevY = input16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;
      for (size_t x = 0; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = prevY + x*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] - prevX[c];   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

static
void up_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_reverse( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *prevY = output + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      for (size_t x = 0; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = prevY + x*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] + prevX[c];   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_reverse( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *prevY = output16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;
      for (size_t x = 0; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = prevY + x*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] + prevX[c];   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

// for normal call: colStep = channels, rowStep = size1*channels, planeStep = size1*size2*channels
static
void prev2D_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    for (size_t y = 0; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // up diff first pixel (copying very first)
      if (y == 0) {
        for (int c = 0; c < channels; ++c)
          outY[c] = inY[c];
      }
      else {
        const uint8_t *prevY = input + (y-1)*rowStep;
        for (int c = 0; c < channels; ++c)
          outY[c] = inY[c] - prevY[c];   // overflow/underflow is intentional
      }
      
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = inY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] - prevX[c];   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 0; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      uint16_t *outY = output16 + y*rowStep;
      
      // up diff first pixel (copying very first)
      if (y == 0) {
        for (int c = 0; c < channels; ++c)
          outY[c] = inY[c];
      }
      else {
        const uint16_t *prevY = input16 + (y-1)*rowStep;
        for (int c = 0; c < channels; ++c)
          outY[c] = inY[c] - prevY[c];   // overflow/underflow is intentional
      }
      
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = inY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] - prevX[c];   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

static
void prev2D_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    for (size_t y = 0; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // up diff first pixel (copying very first)
      if (y == 0) {
        for (int c = 0; c < channels; ++c)
          outY[c] = inY[c];
      }
      else {
        const uint8_t *prevY = output + (y-1)*rowStep;
        for (int c = 0; c < channels; ++c)
          outY[c] = inY[c] + prevY[c];   // overflow/underflow is intentional
      }
      
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = outY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] + prevX[c];   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 0; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      uint16_t *outY = output16 + y*rowStep;
      
      // up diff first pixel (copying very first)
      if (y == 0) {
        for (int c = 0; c < channels; ++c)
          outY[c] = inY[c];
      }
      else {
        const uint16_t *prevY = output16 + (y-1)*rowStep;
        for (int c = 0; c < channels; ++c)
          outY[c] = inY[c] + prevY[c];   // overflow/underflow is intentional
      }
      
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = outY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] + prevX[c];   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

// for normal call: colStep = channels, rowStep = size1*channels, planeStep = size1*size2*channels
static
void min_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_forward( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // min3 diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = input + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = inY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        const uint8_t *upXP = upY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint8_t minVal = std::min( upX[c], std::min( upXP[c], prevX[c] ) );
          outX[c] = inX[c] - minVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_forward( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // min3 diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = input16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;

      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = inY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        const uint16_t *upXP = upY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint16_t minVal = std::min( upX[c], std::min( upXP[c], prevX[c] ) );
          outX[c] = inX[c] - minVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

static
void min_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_reverse( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // min3 diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = output + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = outY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        const uint8_t *upXP = upY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint8_t minVal = std::min( upX[c], std::min( upXP[c], prevX[c] ) );
          outX[c] = inX[c] + minVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_reverse( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // min3 diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = output16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = outY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        const uint16_t *upXP = upY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint16_t minVal = std::min( upX[c], std::min( upXP[c], prevX[c] ) );
          outX[c] = inX[c] + minVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

// for normal call: colStep = channels, rowStep = size1*channels, planeStep = size1*size2*channels
static
void avgUpLeft_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_forward( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = input + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = inY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint8_t avgVal = (uint8_t)(((uint16_t)upX[c] + (uint16_t)prevX[c]) / 2);
          outX[c] = inX[c] - avgVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_forward( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = input16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;

      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = inY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint16_t avgVal = (uint16_t)(((uint32_t)upX[c] + (uint32_t)prevX[c]) / 2);
          outX[c] = inX[c] - avgVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

static
void avgUpLeft_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_reverse( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = output + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = outY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint8_t avgVal = (uint8_t)(((uint16_t)upX[c] + (uint16_t)prevX[c]) / 2);
          outX[c] = inX[c] + avgVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_reverse( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = output16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = outY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint16_t avgVal = (uint16_t)(((uint32_t)upX[c] + (uint32_t)prevX[c]) / 2);
          outX[c] = inX[c] + avgVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

// for normal call: colStep = channels, rowStep = size1*channels, planeStep = size1*size2*channels
static
void median3_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_forward( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = input + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = inY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        const uint8_t *upXP = upY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint8_t medVal = median3( upX[c], upXP[c], prevX[c] );
          outX[c] = inX[c] - medVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_forward( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = input16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;

      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = inY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        const uint16_t *upXP = upY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint16_t medVal = median3( upX[c], upXP[c], prevX[c] );
          outX[c] = inX[c] - medVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

static
void median3_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_reverse( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = output + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = outY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        const uint8_t *upXP = upY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint8_t medVal = median3( upX[c], upXP[c], prevX[c] );
          outX[c] = inX[c] + medVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_reverse( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = output16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = outY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        const uint16_t *upXP = upY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint16_t medVal = median3( upX[c], upXP[c], prevX[c] );
          outX[c] = inX[c] + medVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

// for normal call: colStep = channels, rowStep = size1*channels, planeStep = size1*size2*channels
static
void MED_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_forward( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = input + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = inY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        const uint8_t *upXP = upY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint8_t VX = prevX[c];
          uint8_t VY = upX[c];
          uint8_t VZ = upXP[c];
          auto medVal = median3( VX, VY, (uint8_t)(VX+VY-VZ) );
          outX[c] = inX[c] - medVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_forward( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = input16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;

      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = inY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        const uint16_t *upXP = upY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint16_t VX = prevX[c];
          uint16_t VY = upX[c];
          uint16_t VZ = upXP[c];
          auto medVal = median3( VX, VY, (uint16_t)(VX+VY-VZ) );
          outX[c] = inX[c] - medVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

static
void MED_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_reverse( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = output + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = outY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        const uint8_t *upXP = upY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint8_t VX = prevX[c];
          uint8_t VY = upX[c];
          uint8_t VZ = upXP[c];
          auto medVal = median3( VX, VY, (uint8_t)(VX+VY-VZ) );
          outX[c] = inX[c] + medVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_reverse( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = output16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = outY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        const uint16_t *upXP = upY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          uint16_t VX = prevX[c];
          uint16_t VY = upX[c];
          uint16_t VZ = upXP[c];
          auto medVal = median3( VX, VY, (uint16_t)(VX+VY-VZ) );
          outX[c] = inX[c] + medVal;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

// for normal call: colStep = channels, rowStep = size1*channels, planeStep = size1*size2*channels
static
void Paeth_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_forward( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = input + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = inY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        const uint8_t *upXP = upY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          int VX = prevX[c];
          int VY = upX[c];
          int VZ = upXP[c];
          int p = VX + VY - VZ;
          int pa = std::abs(p - VX);	// y - z, horizontal gradient
          int pb = std::abs(p - VY);	// x - z, vertical gradient
          int pc = std::abs(p - VZ);	// x + y - 2*z, (x-z)+(y-z), sum of vert and horiz gradients
          if (pa <= pb && pa <= pc)
            p = VX;		// horizontal gradient is smallest
          else if (pb <= pc)
            p = VY;		// vertical gradient is smallest
          else
            p = VZ;		// diagonal gradient is smallest
          outX[c] = inX[c] - p;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_forward( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = input16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;

      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = inY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        const uint16_t *upXP = upY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          int VX = prevX[c];
          int VY = upX[c];
          int VZ = upXP[c];
          int p = VX + VY - VZ;
          int pa = std::abs(p - VX);	// y - z, horizontal gradient
          int pb = std::abs(p - VY);	// x - z, vertical gradient
          int pc = std::abs(p - VZ);	// x + y - 2*z, (x-z)+(y-z), sum of vert and horiz gradients
          if (pa <= pb && pa <= pc)
            p = VX;		// horizontal gradient is smallest
          else if (pb <= pc)
            p = VY;		// vertical gradient is smallest
          else
            p = VZ;		// diagonal gradient is smallest
          outX[c] = inX[c] - p;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

static
void Paeth_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_reverse( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = output + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = outY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        const uint8_t *upXP = upY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          int VX = prevX[c];
          int VY = upX[c];
          int VZ = upXP[c];
          int p = VX + VY - VZ;
          int pa = std::abs(p - VX);	// y - z, horizontal gradient
          int pb = std::abs(p - VY);	// x - z, vertical gradient
          int pc = std::abs(p - VZ);	// x + y - 2*z, (x-z)+(y-z), sum of vert and horiz gradients
          if (pa <= pb && pa <= pc)
            p = VX;		// horizontal gradient is smallest
          else if (pb <= pc)
            p = VY;		// vertical gradient is smallest
          else
            p = VZ;		// diagonal gradient is smallest
          outX[c] = inX[c] + p;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_reverse( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = output16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = outY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        const uint16_t *upXP = upY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          int VX = prevX[c];
          int VY = upX[c];
          int VZ = upXP[c];
          int p = VX + VY - VZ;
          int pa = std::abs(p - VX);	// y - z, horizontal gradient
          int pb = std::abs(p - VY);	// x - z, vertical gradient
          int pc = std::abs(p - VZ);	// x + y - 2*z, (x-z)+(y-z), sum of vert and horiz gradients
          if (pa <= pb && pa <= pc)
            p = VX;		// horizontal gradient is smallest
          else if (pb <= pc)
            p = VY;		// vertical gradient is smallest
          else
            p = VZ;		// diagonal gradient is smallest
          outX[c] = inX[c] + p;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

// for normal call: colStep = channels, rowStep = size1*channels, planeStep = size1*size2*channels
static
void MinGrad_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_forward( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = input + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = inY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        const uint8_t *upXP = upY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          int VX = prevX[c];
          int VY = upX[c];
          int VZ = upXP[c];
          int pa = std::abs(VY-VZ);	// horizontal gradient
          int pb = std::abs(VX-VZ);	// vertical gradient
          int p = (pa < pb) ? VX : VY;
          outX[c] = inX[c] - p;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_forward( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = input16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;

      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] - upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = inY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        const uint16_t *upXP = upY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          int VX = prevX[c];
          int VY = upX[c];
          int VZ = upXP[c];
          int pa = std::abs(VY-VZ);	// horizontal gradient
          int pb = std::abs(VX-VZ);	// vertical gradient
          int p = (pa < pb) ? VX : VY;
          outX[c] = inX[c] - p;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

static
void MinGrad_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t /*size3*/,
                size_t colStep, size_t rowStep, size_t /*planeStep*/ )
{
  if (bitDepth == 8) {
    // forward diff first row
    prev_reverse( input, output, 8, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    for (size_t y = 1; y < size2; ++y) {
      const uint8_t *inY = input + y*rowStep;
      const uint8_t *upY = output + (y-1)*rowStep;
      uint8_t *outY = output + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint8_t *inX = inY + x*colStep;
        const uint8_t *prevX = outY + (x-1)*colStep;
        const uint8_t *upX = upY + x*colStep;
        const uint8_t *upXP = upY + (x-1)*colStep;
        uint8_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          int VX = prevX[c];
          int VY = upX[c];
          int VZ = upXP[c];
          int pa = std::abs(VY-VZ);	// horizontal gradient
          int pb = std::abs(VX-VZ);	// vertical gradient
          int p = (pa < pb) ? VX : VY;
          outX[c] = inX[c] + p;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 8 bit

  if (bitDepth == 16) {
    // forward diff first row
    prev_reverse( input, output, 16, channels, size1, 0, 0, colStep, 0, 0 );
    
    // vertical diff remaining rows
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t y = 1; y < size2; ++y) {
      const uint16_t *inY = input16 + y*rowStep;
      const uint16_t *upY = output16 + (y-1)*rowStep;
      uint16_t *outY = output16 + y*rowStep;
      
      // first pixel just does up
      for (int c = 0; c < channels; ++c) {
          outY[c] = inY[c] + upY[c];   // overflow/underflow is intentional
      }
      for (size_t x = 1; x < size1; ++x) {
        const uint16_t *inX = inY + x*colStep;
        const uint16_t *prevX = outY + (x-1)*colStep;
        const uint16_t *upX = upY + x*colStep;
        const uint16_t *upXP = upY + (x-1)*colStep;
        uint16_t *outX = outY + x*colStep;
        for (int c = 0; c < channels; ++c) {
          int VX = prevX[c];
          int VY = upX[c];
          int VZ = upXP[c];
          int pa = std::abs(VY-VZ);	// horizontal gradient
          int pb = std::abs(VX-VZ);	// vertical gradient
          int p = (pa < pb) ? VX : VY;
          outX[c] = inX[c] + p;   // overflow/underflow is intentional
        }
      }  // end x loop
    }   // end y loop
  } // end 16 bit

}

/******************************************************************************/

// for normal call: colStep = channels, rowStep = size1*channels, planeStep = size1*size2*channels
static
void min3D_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t size3,
                size_t colStep, size_t rowStep, size_t planeStep )
{
  if (bitDepth == 8) {
    // 2D first plane
    min_forward( input, output, 8, channels, size1, size2, 0, colStep, rowStep, 0 );

    for (size_t z = 1; z < size3; ++z) {
      const uint8_t *inZ = input + z*planeStep;
      const uint8_t *belowZ = input + (z-1)*planeStep;
      uint8_t *outZ = output + z*planeStep;
      
      // below first row
      for (size_t x = 0; x < size1; ++x) {
        const uint8_t *inX = inZ + x*colStep;
        const uint8_t *belowX = belowZ + x*colStep;
        uint8_t *outX = outZ + x*colStep;
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] - belowX[c];   // overflow/underflow is intentional
        }
      }  // end x loop first row

      // minimum remaining rows
      for (size_t y = 1; y < size2; ++y) {
        const uint8_t *inY = inZ + y*rowStep;
        const uint8_t *upY = inZ + (y-1)*rowStep;
        const uint8_t *belowY = belowZ + y*rowStep;
        const uint8_t *belowUpY = belowZ + (y-1)*rowStep;
        uint8_t *outY = outZ + y*rowStep;
        
        // below first pixel
        for (int c = 0; c < channels; ++c) {
            auto minVal = std::min( { upY[c], belowY[c], belowUpY[c]  } );
            outY[c] = inY[c] - minVal;   // overflow/underflow is intentional
        }
        for (size_t x = 1; x < size1; ++x) {
          const uint8_t *inX = inY + x*colStep;
          const uint8_t *prevX = inY + (x-1)*colStep;
          const uint8_t *upX = upY + x*colStep;
          const uint8_t *upXP = upY + (x-1)*colStep;
          const uint8_t *belowX = belowY + x*colStep;
          const uint8_t *belowPrevX = belowY + (x-1)*colStep;
          const uint8_t *belowUpX = belowUpY + x*colStep;
          const uint8_t *belowUpPrevX = belowUpY + (x-1)*colStep;
          uint8_t *outX = outY + x*colStep;
          
          for (int c = 0; c < channels; ++c) {
            auto minVal = std::min( { upX[c], upXP[c], prevX[c], belowX[c], belowPrevX[c], belowUpX[c], belowUpPrevX[c]  } );
            outX[c] = inX[c] - minVal;   // overflow/underflow is intentional
          }
        }  // end x loop
      }   // end y loop
    } // end z loop
  } // end 8 bit

  if (bitDepth == 16) {
    // 2D first plane
    min_forward( input, output, 16, channels, size1, size2, 0, colStep, rowStep, 0 );
    
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t z = 1; z < size3; ++z) {
      const uint16_t *inZ = input16 + z*planeStep;
      const uint16_t *belowZ = input16 + (z-1)*planeStep;
      uint16_t *outZ = output16 + z*planeStep;
      
      // below first row
      for (size_t x = 0; x < size1; ++x) {
        const uint16_t *inX = inZ + x*colStep;
        const uint16_t *belowX = belowZ + x*colStep;
        uint16_t *outX = outZ + x*colStep;
          
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] - belowX[c];   // overflow/underflow is intentional
        }
      }  // end x loop

      // minimum remaining rows
      for (size_t y = 1; y < size2; ++y) {
        const uint16_t *inY = inZ + y*rowStep;
        const uint16_t *upY = inZ + (y-1)*rowStep;
        const uint16_t *belowY = belowZ + y*rowStep;
        const uint16_t *belowUpY = belowZ + (y-1)*rowStep;
        uint16_t *outY = outZ + y*rowStep;
        
        // below first pixel
        for (int c = 0; c < channels; ++c) {
            auto minVal = std::min( { upY[c], belowY[c], belowUpY[c]  } );
            outY[c] = inY[c] - minVal;   // overflow/underflow is intentional
        }
        for (size_t x = 1; x < size1; ++x) {
          const uint16_t *inX = inY + x*colStep;
          const uint16_t *prevX = inY + (x-1)*colStep;
          const uint16_t *upX = upY + x*colStep;
          const uint16_t *upXP = upY + (x-1)*colStep;
          const uint16_t *belowX = belowY + x*colStep;
          const uint16_t *belowPrevX = belowY + (x-1)*colStep;
          const uint16_t *belowUpX = belowUpY + x*colStep;
          const uint16_t *belowUpPrevX = belowUpY + (x-1)*colStep;
          uint16_t *outX = outY + x*colStep;
          
          for (int c = 0; c < channels; ++c) {
            auto minVal = std::min( { upX[c], upXP[c], prevX[c], belowX[c], belowPrevX[c], belowUpX[c], belowUpPrevX[c]  } );
            outX[c] = inX[c] - minVal;   // overflow/underflow is intentional
          }
        }  // end x loop
      }   // end y loop
    } // end z loop
  } // end 16 bit

}

/******************************************************************************/

static
void min3D_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t size3,
                size_t colStep, size_t rowStep, size_t planeStep )
{
  if (bitDepth == 8) {
    // first plane
    min_reverse( input, output, 8, channels, size1, size2, 0, colStep, rowStep, 0 );

    for (size_t z = 1; z < size3; ++z) {
      const uint8_t *inZ = input + z*planeStep;
      const uint8_t *belowZ = output + (z-1)*planeStep;
      uint8_t *outZ = output + z*planeStep;
      
      // below first row
      for (size_t x = 0; x < size1; ++x) {
        const uint8_t *inX = inZ + x*colStep;
        const uint8_t *belowX = belowZ + x*colStep;
        uint8_t *outX = outZ + x*colStep;
          
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] + belowX[c];   // overflow/underflow is intentional
        }
      }  // end x loop

      // minimum remaining rows
      for (size_t y = 1; y < size2; ++y) {
        const uint8_t *inY = inZ + y*rowStep;
        const uint8_t *upY = outZ + (y-1)*rowStep;
        const uint8_t *belowY = belowZ + y*rowStep;
        const uint8_t *belowUpY = belowZ + (y-1)*rowStep;
        uint8_t *outY = outZ + y*rowStep;
        
        // below first pixel
        for (int c = 0; c < channels; ++c) {
            auto minVal = std::min( { upY[c], belowY[c], belowUpY[c]  } );
            outY[c] = inY[c] + minVal;   // overflow/underflow is intentional
        }
        for (size_t x = 1; x < size1; ++x) {
          const uint8_t *inX = inY + x*colStep;
          const uint8_t *prevX = outY + (x-1)*colStep;
          const uint8_t *upX = upY + x*colStep;
          const uint8_t *upXP = upY + (x-1)*colStep;
          const uint8_t *belowX = belowY + x*colStep;
          const uint8_t *belowPrevX = belowY + (x-1)*colStep;
          const uint8_t *belowUpX = belowUpY + x*colStep;
          const uint8_t *belowUpPrevX = belowUpY + (x-1)*colStep;
          uint8_t *outX = outY + x*colStep;
          
          for (int c = 0; c < channels; ++c) {
            auto minVal = std::min( { upX[c], upXP[c], prevX[c], belowX[c], belowPrevX[c], belowUpX[c], belowUpPrevX[c]  } );
            outX[c] = inX[c] + minVal;   // overflow/underflow is intentional
          }
        }  // end x loop
      }   // end y loop
    } // end z loop
  } // end 8 bit

  if (bitDepth == 16) {
    // 2D first plane
    min_reverse( input, output, 16, channels, size1, size2, 0, colStep, rowStep, 0 );
    
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t z = 1; z < size3; ++z) {
      const uint16_t *inZ = input16 + z*planeStep;
      const uint16_t *belowZ = output16 + (z-1)*planeStep;
      uint16_t *outZ = output16 + z*planeStep;
      
      // below first row
      for (size_t x = 0; x < size1; ++x) {
        const uint16_t *inX = inZ + x*colStep;
        const uint16_t *belowX = belowZ + x*colStep;
        uint16_t *outX = outZ + x*colStep;
          
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] + belowX[c];   // overflow/underflow is intentional
        }
      }  // end x loop

      // minimum remaining rows
      for (size_t y = 1; y < size2; ++y) {
        const uint16_t *inY = inZ + y*rowStep;
        const uint16_t *upY = outZ + (y-1)*rowStep;
        const uint16_t *belowY = belowZ + y*rowStep;
        const uint16_t *belowUpY = belowZ + (y-1)*rowStep;
        uint16_t *outY = outZ + y*rowStep;
        
        // below first pixel
        for (int c = 0; c < channels; ++c) {
            auto minVal = std::min( { upY[c], belowY[c], belowUpY[c]  } );
            outY[c] = inY[c] + minVal;   // overflow/underflow is intentional
        }
        for (size_t x = 1; x < size1; ++x) {
          const uint16_t *inX = inY + x*colStep;
          const uint16_t *prevX = outY + (x-1)*colStep;
          const uint16_t *upX = upY + x*colStep;
          const uint16_t *upXP = upY + (x-1)*colStep;
          const uint16_t *belowX = belowY + x*colStep;
          const uint16_t *belowPrevX = belowY + (x-1)*colStep;
          const uint16_t *belowUpX = belowUpY + x*colStep;
          const uint16_t *belowUpPrevX = belowUpY + (x-1)*colStep;
          uint16_t *outX = outY + x*colStep;
          
          for (int c = 0; c < channels; ++c) {
            auto minVal = std::min( { upX[c], upXP[c], prevX[c], belowX[c], belowPrevX[c], belowUpX[c], belowUpPrevX[c]  } );
            outX[c] = inX[c] + minVal;   // overflow/underflow is intentional
          }
        }  // end x loop
      }   // end y loop
    } // end z loop
  } // end 16 bit

}

/******************************************************************************/

template<typename T>
void cmpswap(T&a, T&b) { if (b < a) { std::swap(a,b); } }

template <typename T>
inline T median7( std::initializer_list<T> input ) {
  std::vector<T> p( input );

// NOTE - Google AI got the sorting network completely wrong 19/20 tries
//  and got the performance analysis wrong 20/20 tries.
//  and the one that worked, was not minimal (it was a full sort)
// NOTE - Claude got the minimal blind sorting network (13 compares) correct

  std::nth_element(p.begin(), p.begin()+3, p.end());
  return p[3];
}

template <typename T>
inline T median( std::initializer_list<T> input ) {
  std::vector<T> p( input );
  size_t half = p.size() >> 1;
  std::nth_element(p.begin(), p.begin()+half, p.end());
  return p[half];
}

template <typename T>
inline T median( std::vector<T> p ) {
  size_t count = p.size();
  size_t half = count >> 1;
  if ((count & 1) != 0) {
    std::nth_element(p.begin(), p.begin()+half, p.end());
    return p[half];
  } else {
    std::nth_element(p.begin(), p.begin()+half, p.end());
    auto val1 = p[half];
    std::nth_element(p.begin(), p.begin()+half-1, p.end());
    return (p[half-1]+val1)/2;
  }
}

// This is used for verification, not production code
template<typename T>
T bruteForceMedian( std::initializer_list<T> input )
{
  std::vector<T> p( input );
  size_t count = p.size();
  size_t half = count >> 1;
  std::sort(p.begin(),p.end());
  if ( (count & 1) != 0 )
    return p[ half ];
  else
   return (p[half-1] + p[half])/2;
}

// This is used for verification, not production code
template<typename T>
T bruteForceMedian( std::vector<T> p )
{
  size_t count = p.size();
  size_t half = count >> 1;
  std::sort(p.begin(),p.end());
  if ( (count & 1) != 0 )
    return p[ half ];
  else
   return (p[half-1] + p[half])/2;
}


// for normal call: colStep = channels, rowStep = size1*channels, planeStep = size1*size2*channels
static
void median3D_forward( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t size3,
                size_t colStep, size_t rowStep, size_t planeStep )
{
  if (bitDepth == 8) {
    // 2D first plane
    median3_forward( input, output, 8, channels, size1, size2, 0, colStep, rowStep, 0 );

    for (size_t z = 1; z < size3; ++z) {
      const uint8_t *inZ = input + z*planeStep;
      const uint8_t *belowZ = input + (z-1)*planeStep;
      uint8_t *outZ = output + z*planeStep;
      
      // below first row
      for (size_t x = 0; x < size1; ++x) {
        const uint8_t *inX = inZ + x*colStep;
        const uint8_t *belowX = belowZ + x*colStep;
        uint8_t *outX = outZ + x*colStep;
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] - belowX[c];   // overflow/underflow is intentional
        }
      }  // end x loop first row

      // minimum remaining rows
      for (size_t y = 1; y < size2; ++y) {
        const uint8_t *inY = inZ + y*rowStep;
        const uint8_t *upY = inZ + (y-1)*rowStep;
        const uint8_t *belowY = belowZ + y*rowStep;
        const uint8_t *belowUpY = belowZ + (y-1)*rowStep;
        uint8_t *outY = outZ + y*rowStep;
        
        // below and median first pixel
        for (int c = 0; c < channels; ++c) {
            auto medVal = median3( upY[c], belowY[c], belowUpY[c] );
            outY[c] = inY[c] - medVal;   // overflow/underflow is intentional
        }
        for (size_t x = 1; x < size1; ++x) {
          const uint8_t *inX = inY + x*colStep;
          const uint8_t *prevX = inY + (x-1)*colStep;
          const uint8_t *upX = upY + x*colStep;
          const uint8_t *upXP = upY + (x-1)*colStep;
          const uint8_t *belowX = belowY + x*colStep;
          const uint8_t *belowPrevX = belowY + (x-1)*colStep;
          const uint8_t *belowUpX = belowUpY + x*colStep;
          const uint8_t *belowUpPrevX = belowUpY + (x-1)*colStep;
          uint8_t *outX = outY + x*colStep;
          
          for (int c = 0; c < channels; ++c) {
            auto medVal = median7( { upX[c], upXP[c], prevX[c], belowX[c], belowPrevX[c], belowUpX[c], belowUpPrevX[c]  } );
            outX[c] = inX[c] - medVal;   // overflow/underflow is intentional
          }
        }  // end x loop
      }   // end y loop
    } // end z loop
  } // end 8 bit

  if (bitDepth == 16) {
    // 2D first plane
    median3_forward( input, output, 16, channels, size1, size2, 0, colStep, rowStep, 0 );
    
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t z = 1; z < size3; ++z) {
      const uint16_t *inZ = input16 + z*planeStep;
      const uint16_t *belowZ = input16 + (z-1)*planeStep;
      uint16_t *outZ = output16 + z*planeStep;
      
      // below first row
      for (size_t x = 0; x < size1; ++x) {
        const uint16_t *inX = inZ + x*colStep;
        const uint16_t *belowX = belowZ + x*colStep;
        uint16_t *outX = outZ + x*colStep;
          
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] - belowX[c];   // overflow/underflow is intentional
        }
      }  // end x loop

      // minimum remaining rows
      for (size_t y = 1; y < size2; ++y) {
        const uint16_t *inY = inZ + y*rowStep;
        const uint16_t *upY = inZ + (y-1)*rowStep;
        const uint16_t *belowY = belowZ + y*rowStep;
        const uint16_t *belowUpY = belowZ + (y-1)*rowStep;
        uint16_t *outY = outZ + y*rowStep;
        
        // below and median first pixel
        for (int c = 0; c < channels; ++c) {
            auto medVal = median3( upY[c], belowY[c], belowUpY[c] );
            outY[c] = inY[c] - medVal;   // overflow/underflow is intentional
        }
        for (size_t x = 1; x < size1; ++x) {
          const uint16_t *inX = inY + x*colStep;
          const uint16_t *prevX = inY + (x-1)*colStep;
          const uint16_t *upX = upY + x*colStep;
          const uint16_t *upXP = upY + (x-1)*colStep;
          const uint16_t *belowX = belowY + x*colStep;
          const uint16_t *belowPrevX = belowY + (x-1)*colStep;
          const uint16_t *belowUpX = belowUpY + x*colStep;
          const uint16_t *belowUpPrevX = belowUpY + (x-1)*colStep;
          uint16_t *outX = outY + x*colStep;
          
          for (int c = 0; c < channels; ++c) {
            auto medVal = median7( { upX[c], upXP[c], prevX[c], belowX[c], belowPrevX[c], belowUpX[c], belowUpPrevX[c]  } );
            outX[c] = inX[c] - medVal;   // overflow/underflow is intentional
          }
        }  // end x loop
      }   // end y loop
    } // end z loop
  } // end 16 bit

}

/******************************************************************************/

static
void median3D_reverse( const uint8_t *input, uint8_t *output, int bitDepth, int channels,
                size_t size1, size_t size2, size_t size3,
                size_t colStep, size_t rowStep, size_t planeStep )
{
  if (bitDepth == 8) {
    // first plane
    median3_reverse( input, output, 8, channels, size1, size2, 0, colStep, rowStep, 0 );

    for (size_t z = 1; z < size3; ++z) {
      const uint8_t *inZ = input + z*planeStep;
      const uint8_t *belowZ = output + (z-1)*planeStep;
      uint8_t *outZ = output + z*planeStep;
      
      // below first row
      for (size_t x = 0; x < size1; ++x) {
        const uint8_t *inX = inZ + x*colStep;
        const uint8_t *belowX = belowZ + x*colStep;
        uint8_t *outX = outZ + x*colStep;
          
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] + belowX[c];   // overflow/underflow is intentional
        }
      }  // end x loop

      // minimum remaining rows
      for (size_t y = 1; y < size2; ++y) {
        const uint8_t *inY = inZ + y*rowStep;
        const uint8_t *upY = outZ + (y-1)*rowStep;
        const uint8_t *belowY = belowZ + y*rowStep;
        const uint8_t *belowUpY = belowZ + (y-1)*rowStep;
        uint8_t *outY = outZ + y*rowStep;
        
        // below and median first pixel
        for (int c = 0; c < channels; ++c) {
            auto medVal = median3( upY[c], belowY[c], belowUpY[c] );
            outY[c] = inY[c] + medVal;   // overflow/underflow is intentional
        }
        for (size_t x = 1; x < size1; ++x) {
          const uint8_t *inX = inY + x*colStep;
          const uint8_t *prevX = outY + (x-1)*colStep;
          const uint8_t *upX = upY + x*colStep;
          const uint8_t *upXP = upY + (x-1)*colStep;
          const uint8_t *belowX = belowY + x*colStep;
          const uint8_t *belowPrevX = belowY + (x-1)*colStep;
          const uint8_t *belowUpX = belowUpY + x*colStep;
          const uint8_t *belowUpPrevX = belowUpY + (x-1)*colStep;
          uint8_t *outX = outY + x*colStep;
          
          for (int c = 0; c < channels; ++c) {
            auto medVal = median7( { upX[c], upXP[c], prevX[c], belowX[c], belowPrevX[c], belowUpX[c], belowUpPrevX[c]  } );
            outX[c] = inX[c] + medVal;   // overflow/underflow is intentional
          }
        }  // end x loop
      }   // end y loop
    } // end z loop
  } // end 8 bit

  if (bitDepth == 16) {
    // 2D first plane
    median3_reverse( input, output, 16, channels, size1, size2, 0, colStep, rowStep, 0 );
    
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (size_t z = 1; z < size3; ++z) {
      const uint16_t *inZ = input16 + z*planeStep;
      const uint16_t *belowZ = output16 + (z-1)*planeStep;
      uint16_t *outZ = output16 + z*planeStep;
      
      // below first row
      for (size_t x = 0; x < size1; ++x) {
        const uint16_t *inX = inZ + x*colStep;
        const uint16_t *belowX = belowZ + x*colStep;
        uint16_t *outX = outZ + x*colStep;
          
        for (int c = 0; c < channels; ++c) {
          outX[c] = inX[c] + belowX[c];   // overflow/underflow is intentional
        }
      }  // end x loop

      // minimum remaining rows
      for (size_t y = 1; y < size2; ++y) {
        const uint16_t *inY = inZ + y*rowStep;
        const uint16_t *upY = outZ + (y-1)*rowStep;
        const uint16_t *belowY = belowZ + y*rowStep;
        const uint16_t *belowUpY = belowZ + (y-1)*rowStep;
        uint16_t *outY = outZ + y*rowStep;
        
        // below and median first pixel
        for (int c = 0; c < channels; ++c) {
            auto medVal = median3( upY[c], belowY[c], belowUpY[c] );
            outY[c] = inY[c] + medVal;   // overflow/underflow is intentional
        }
        for (size_t x = 1; x < size1; ++x) {
          const uint16_t *inX = inY + x*colStep;
          const uint16_t *prevX = outY + (x-1)*colStep;
          const uint16_t *upX = upY + x*colStep;
          const uint16_t *upXP = upY + (x-1)*colStep;
          const uint16_t *belowX = belowY + x*colStep;
          const uint16_t *belowPrevX = belowY + (x-1)*colStep;
          const uint16_t *belowUpX = belowUpY + x*colStep;
          const uint16_t *belowUpPrevX = belowUpY + (x-1)*colStep;
          uint16_t *outX = outY + x*colStep;
          
          for (int c = 0; c < channels; ++c) {
            auto medVal = median7( { upX[c], upXP[c], prevX[c], belowX[c], belowPrevX[c], belowUpX[c], belowUpPrevX[c]  } );
            outX[c] = inX[c] + medVal;   // overflow/underflow is intentional
          }
        }  // end x loop
      }   // end y loop
    } // end z loop
  } // end 16 bit

}

/******************************************************************************/
/******************************************************************************/

static
bool describe1DLUT( CIccTagCurve *curve, std::string &description,
                    const std::string &sigDesc, const std::string &filename )
{
  std::string path(":");
  path += sigDesc;
  std::string report;
  if (curve->Validate(path, report, NULL) > icValidateWarning) {
    LogAnError(stderr,"%s: WARNING - curve failed validation:\n%s\n", filename.c_str(), report.c_str() );
    description = "simpleCurve";
    return true;
  }

  auto size = curve->GetSize();
  if (size == 0) {
    description += "Y = X";
  } else if (size == 1) {
    icFloatNumber value0 = (*curve)[0];
    icFloatNumber dGamma = (icFloatNumber)(value0 * 256.0f);
    description += "Y = X ^ " + std::to_string(dGamma);
  } else {
    description += "LookupTable[" + std::to_string(size) + "]";
  }

  return false;
}

/******************************************************************************/

static
bool describe3DLUT( CIccMBB *curve, CIccProfile *pIcc, std::string &description,
                    const std::string &sigDesc, const std::string &filename )
{
  std::string path(":");
  path += sigDesc;
  std::string report;
  description = "MBBLut";
  if (curve->Validate(path, report, pIcc ) > icValidateWarning) {
    LogAnError(stderr,"%s: WARNING - 3D table failed validation:\n%s\n", filename.c_str(), report.c_str() );
    return true;
  }
//  curve->Describe( description, 100 );     // no longer used, and SLOW
  return false;
}

/******************************************************************************/

static
void test1DLUT( CIccTagCurve *curve, const std::string &name,
                const std::string &description )
{
  const uint32_t kMaxCurveSize = 16384;

  // read curve into input buffer
  uint32_t steps = curve->GetSize();
  if (steps == 0 || steps > kMaxCurveSize) {
    LogAnError(stderr, "%s: WARNING - curve size invalid for %s\n", name.c_str(), description.c_str() );
    return;
  }
  
  // this won't compress - don't bother
  if (steps <= 4)
    return;
  
  std::unique_ptr<uint16_t[]> inputBuffer( new uint16_t[ steps ] );
  std::unique_ptr<uint16_t[]> outputBuffer( new uint16_t[ steps ] );
  uint16_t *input = inputBuffer.get();
  uint16_t *output = outputBuffer.get();

#if 1
  std::unique_ptr<uint16_t[]> verifyBuffer( new uint16_t[ steps ] );
  uint16_t *verify = verifyBuffer.get();
#endif

  // iterate the curve and copy into input
  for (uint32_t i = 0; i < steps; ++i ) {
    // bitDepth is always 8.8
    // but stored as float
    auto value = *( curve->GetData(i) );
    input[i] = ClipU16( value * 256.0 );
  }
  
  // print name of the data object and base size
  printf("%s\t%u", name.c_str(), steps*2 );

  uint32_t dimensionArray[2] = {(uint32_t)steps,0};
  
  // iterate over all predictors
  for (const auto &pred : predictorList) {
    
    // apply forward predictor
    applyOnePredictor( pred, (uint8_t*)input, (uint8_t*)output,
                       dimensionArray, 1, 16, 1, steps, false );

#if 1
    // apply reverse predictor for verification
    applyOnePredictor( pred, (uint8_t*)output, (uint8_t*)verify,
                       dimensionArray, 1, 16, 1, steps, true );
    if ( memcmp(input,verify,steps*2) != 0 ) {
        LogAnError(stderr, "%s: WARNING - %s predictor reverse failed %s\n",
                    name.c_str(), pred.name, description.c_str() );
    }
#endif

    // allow extra room, just in case
    std::unique_ptr<uint16_t[]> compressedBuffer( new uint16_t[ 2*steps ] );
    uint16_t *compressed = compressedBuffer.get();

    // compress
    size_t inSize = steps*2;
    size_t outBytes = steps*4;
    if ( !deflateBuffer( (uint8_t*)output, (uint8_t*)compressed, inSize, outBytes, 9 ) ) {
      LogAnError(stderr, "%s: ERROR - could not deflate %s\n", name.c_str(), description.c_str() );
    }

    // report size
    printf("\t%zu", outBytes );
  }

  printf("\n"); // and finish the line of results

}

/******************************************************************************/

// output graphic representation of 1D LUTs
static
void process1DLUT(CIccProfile * /* pIcc */, CIccTag *tag, const std::string &sigDesc,
                const std::string &filename )
{
  const size_t bufSize = 64;
  char buf[bufSize];

  if (!tag) {
    LogAnError(stderr, "%s: ERROR - missing data for %s\n", filename.c_str(), sigDesc.c_str() );
    return;
  }

  icTagTypeSignature typeSig = tag->GetType();

  switch(typeSig) {
    case icSigCurveType:
      // continuous sampled curve - we can compress this
      {
      CIccTagCurve *curve = dynamic_cast<CIccTagCurve*> (tag);
      if (curve) {
        std::string description;
        if (describe1DLUT(curve, description, sigDesc, filename)) {
          return;
        }
        test1DLUT( curve, sigDesc, description );
        }
      }
      break;

    case icSigParametricCurveType:
        // ignore, only parameters
      break;

    case icSigSegmentedCurveType:
      // ignore, not continuous data
      break;

    default:
      // unknown
      LogAnError(stderr,"%s: Unknown 1D LUT type %s for tag %s\n",
         filename.c_str(),
         icGetSig(buf, bufSize, typeSig), sigDesc.c_str() );
      break;

  }   // end switch by type

}   // end process1DLUT()

/******************************************************************************/

static
std::string channelName(int index, bool isInputMatrix, icColorSpaceSignature inputSpace,
                        icColorSpaceSignature outputSpace,
                        int inputChannels, int outputChannels)
{
  const size_t bufSize = 128;
  char buf[bufSize];
  icColorIndexName(buf, bufSize, isInputMatrix ? inputSpace :  outputSpace,
                  index, isInputMatrix ? inputChannels : outputChannels,
                  isInputMatrix ? "In" : "Out");
  return std::string(buf);
}

/******************************************************************************/

static
void testCLUT(CIccProfile */*pIcc*/, CIccCLUT *clut, const std::string &sigDesc,
                const std::string &basename )
{
  int inputChannels = clut->GetInputDim();
  int outputChannels = clut->GetOutputChannels();
  
  int bytes = clut->GetPrecision();    // currently only 1 or 2
  if (bytes > 2 || bytes < 1) {
    LogAnError(stderr,"%s: ERROR - bad clut precision for tag '%s'\n",
              basename.c_str(), sigDesc.c_str() );
    return;
  }

  int gridPoints = clut->GridPoints(); // gridSize[0]
  int tiles = gridPoints;
  if (gridPoints <= 0) {
    LogAnError(stderr, "%s: Skipping %s: invalid CLUT grid\n", basename.c_str(), sigDesc.c_str());
    return;
  }
  
  int tileWidth = 1;
  int tileHeight = 1;

  if (inputChannels >= 2) {
    tileWidth = clut->GridPoint(1);
    if (tileWidth <= 0) {
      LogAnError(stderr, "%s: Skipping %s: invalid CLUT width\n", basename.c_str(), sigDesc.c_str());
      return;
    }
  }

  if (inputChannels >= 3) {
    tileHeight = clut->GridPoint(2);
    if (tileHeight <= 0) {
      LogAnError(stderr, "%s: Skipping %s: invalid CLUT height\n", basename.c_str(), sigDesc.c_str());
      return;
    }
  }

  if (inputChannels > 3) {
    for (int i = 3; i < inputChannels; ++i) {
      int extraGridPoints = clut->GridPoint(i);
      if (extraGridPoints <= 0) {
        LogAnError(stderr, "%s: Skipping %s: invalid CLUT tile count\n", basename.c_str(), sigDesc.c_str());
        return;
      }
      tiles *= extraGridPoints;
    }
  }

  // special case for single dimensional LUT
  if (inputChannels == 1) {
    tileWidth = tiles;
    tiles = 1;
    tileHeight = 1;
  }

  // special case for 2 dimensional LUT
  if (inputChannels == 2) {
    tileHeight = tiles;
    tiles = 1;
  }

  if (tiles <= 0) {
    LogAnError(stderr,"%s: WARNING - tile count overflow.\n", basename.c_str() );
    tiles = 1;
  }

  icUInt32Number numPoints = clut->NumPoints();
  size_t byteSize = outputChannels * numPoints * bytes;
  
  std::unique_ptr<uint8_t[]> inputBuffer( new uint8_t[ byteSize ] );
  std::unique_ptr<uint8_t[]> outputBuffer( new uint8_t[ byteSize ] );
  uint8_t *input8 = inputBuffer.get();
  uint16_t *input16 = (uint16_t*)inputBuffer.get();
  uint8_t *output = outputBuffer.get();

#if 1
  std::unique_ptr<uint8_t[]> verifyBuffer( new uint8_t[ byteSize ] );
  uint8_t *verify = verifyBuffer.get();
#endif

  // convert float buffer to int
  for (uint32_t i = 0; i < numPoints; ++i ) {
    // bitDepth is always 8.8
    // but stored as float
    auto value = *( clut->GetData(i) );
    if (bytes == 1)
      input8[i] = ClipU8( value * 255.0 );
    else
      input16[i] = ClipU16( value * 65535.0 );
  }
  
  // print name of the data object and base size
  printf("%s\t%zu", sigDesc.c_str(), byteSize );

  uint32_t dimensionArray[16];
  int i;
  for (i = 0; i < inputChannels; ++i) {
    dimensionArray[i] = clut->GridPoint(i);
  }
  for (; i < 16; ++i) {
    dimensionArray[i] = 0;
  }

  // iterate over all predictors
  for (const auto &pred : predictorList) {
    
    // apply forward predictor
    applyOnePredictor( pred, input8, output,
                       dimensionArray, inputChannels, 8*bytes, outputChannels, numPoints, false );

#if 1
    // apply reverse predictor for verification
    applyOnePredictor( pred, output, verify,
                       dimensionArray, inputChannels, 8*bytes, outputChannels, numPoints, true );
    if ( memcmp(input8,verify,byteSize) != 0 ) {
        LogAnError(stderr, "%s: WARNING - %s predictor reverse failed\n",
                    sigDesc.c_str(), pred.name );
    }
#endif

    // allow extra room, just in case
    std::unique_ptr<uint8_t[]> compressedBuffer( new uint8_t[ 2*byteSize ] );
    uint8_t *compressed = compressedBuffer.get();

    // compress
    size_t inSize = byteSize;
    size_t outBytes = 2*byteSize;
    if ( !deflateBuffer( output, compressed, inSize, outBytes, 9 ) ) {
      LogAnError(stderr, "%s: ERROR - could not deflate\n", sigDesc.c_str() );
    }

    // report size
    printf("\t%zu", outBytes );
  }

  printf("\n"); // and finish the line of results

}

/******************************************************************************/

static
void processMBBType(CIccProfile *pIcc, CIccTag *tag, const std::string &sigDesc,
                const std::string &basename )
{
  const size_t bufSize = 128;
  char buf[bufSize];

  icTagTypeSignature typeSig = tag->GetType();

  CIccMBB *lut = dynamic_cast<CIccMBB*> (tag);
  if (!lut) {
    LogAnError(stderr, "%s: Skipping %s: unable to convert LUT\n", basename.c_str(), sigDesc.c_str());
    return;
  }

  std::string description;
  if (describe3DLUT( lut, pIcc, description, sigDesc, basename)) {
    return;
  }

  // output input and output curves
  CIccCurve **curveA = lut->GetCurvesA();
  CIccCurve **curveB = lut->GetCurvesB();
  CIccCurve **curveM = lut->GetCurvesM();
  std::string curveDesc = sigDesc + ": ";

  int inputChannels = lut->InputChannels();
  int outputChannels = lut->OutputChannels();
  icColorSpaceSignature inputSpace = lut->GetCsInput();
  icColorSpaceSignature outputSpace = lut->GetCsOutput();
  bool isInputMatrix = lut->IsInputMatrix();

  if (inputChannels <= 0 || outputChannels <= 0) {
    LogAnError(stderr, "%s: Skipping %s: invalid channel count\n", basename.c_str(), sigDesc.c_str());
    return;
  }

  if (curveA) {
    int curveACount = isInputMatrix ? outputChannels : inputChannels;
    for (int i = 0; i < curveACount; ++i) {
      if (curveA[i]) {
        std::string channel = channelName( i, !isInputMatrix,
                  inputSpace, outputSpace, inputChannels, outputChannels );
        std::string channelDesc = curveDesc + "curveA[ " + channel + " ]";
        process1DLUT( pIcc, curveA[i], channelDesc, basename );
      }
    }
  }

  if (curveB) {
    int curveBCount = isInputMatrix ? inputChannels : outputChannels;
    for (int i = 0; i < curveBCount; ++i) {
      if (curveB[i]) {
        std::string channel = channelName( i, isInputMatrix,
                  inputSpace, outputSpace, inputChannels, outputChannels );
        std::string channelDesc = curveDesc + "curveB[ " + channel + " ]";
        process1DLUT( pIcc, curveB[i], channelDesc, basename );
      }
    }
  }

  if (curveM) {
    int curveMCount = isInputMatrix ? inputChannels : outputChannels;
    for (int i = 0; i < curveMCount; ++i) {
      if (curveM[i]) {
        std::string channel = channelName( i, isInputMatrix,
                  inputSpace, outputSpace, inputChannels, outputChannels );
        std::string channelDesc = curveDesc + "curveM[ " + channel + " ]";
        process1DLUT( pIcc, curveM[i], channelDesc, basename );
      }
    }
  }


  // process CLUT
  int bytes = lut->GetPrecision();    // currently only 1 or 2
  if (bytes > 2 || bytes < 1) {
    std::string typeDesc = icGetSigStr(buf, bufSize, typeSig);
    LogAnError(stderr,"%s: ERROR - bad clut precision for tag '%s' of type '%s'\n",
              basename.c_str(), sigDesc.c_str(), typeDesc.c_str() );
    return;
  }
  
  
  CIccCLUT *clut = lut->GetCLUT();
  if (!clut) {
    // clut is optional in mAB and mBA tags - only report if it isn't one of those
    if ( !(typeSig == icSigLutAtoBType || typeSig == icSigLutBtoAType) ) {
      std::string typeDesc = icGetSigStr(buf, bufSize, typeSig);
      LogAnError(stderr,"%s: ERROR - clut data could not be read for tag '%s' of type '%s'\n",
              basename.c_str(), sigDesc.c_str(), typeDesc.c_str() );
    }
    return;
  }

  // validate is called back before the Describe call
  clut->Begin();  // initialize some grid information

  testCLUT( pIcc, clut, sigDesc + " table", basename );

}

/******************************************************************************/

// output graphic representation of nD LUTs
// return count of output objects created, 0 if none
static
void process3DLUT( CIccProfile *pIcc, CIccTag *tag, const std::string &sigDesc,
        const std::string &basename )
{
  const size_t bufSize = 128;
  char buf[bufSize];

  if (!tag) {
    LogAnError(stderr, "%s: Skipping %s: unable to load tag\n", basename.c_str(), sigDesc.c_str());
    return;
  }

  icTagTypeSignature typeSig = tag->GetType();
  switch(typeSig) {

    // these are all subclases of CIccMBB, and can share most of the code
    case icSigLut8Type:   // CIccTagLut8
    case icSigLut16Type:  // CIccTagLut16
    case icSigLutAtoBType:  // CIccTagLutAtoB
    case icSigLutBtoAType:  // CIccTagLutBtoA
      processMBBType( pIcc, tag, sigDesc, basename );
      break;

    case icSigMultiProcessElementType:
      // do nothing for now, because we don't know how to render the Multiprocess elements
      break;

    default:
      LogAnError(stderr,"%s: Unknown nD LUT type %s for tag %s\n",
         basename.c_str(),
         icGetSig(buf, bufSize, typeSig),
         sigDesc.c_str() );
      break;

  }   // end switch by type

}   // end process3DLUT()

/******************************************************************************/

static
std::string remove_extension( const std::string& filename )
{
  size_t lastdot = filename.find_last_of(".");
  if (lastdot == std::string::npos || lastdot == 0) {
    return filename;
  }
  return filename.substr(0, lastdot);
}

/******************************************************************************/

inline
std::string remove_path( const std::string& filename )
{
  size_t lastPath = filename.find_last_of("/");
  if (lastPath == std::string::npos || lastPath == 0) {
    lastPath = filename.find_last_of("\\");
  }
  if (lastPath == std::string::npos || lastPath == 0) {
    return filename;
  }
  return filename.substr(lastPath+1, filename.size() );
}

/******************************************************************************/

// create graphic representation of LUTs, named colors, etc.
static
void processProfile( CIccProfile *pIcc, const std::string &basename )
{
  const size_t bufSize = 64;
  char buf1[bufSize];
  
  
  printf("name\toriginal"); // start label line
  for (const auto &pred : predictorList) {
    printf("\t%s", pred.name );
  }
  printf("\n"); // and finish the labels line
  

  for ( auto &tag: pIcc->m_Tags ) {
    icTagSignature sig = tag.TagInfo.sig;
    //icTagTypeSignature typeSig = tag.pTag->GetType(); // unused

// Switching by data type is easier from a programmming standpoint.
// But name will limit us to known tags and ignore bogus tags.

    switch (sig) {

      // 1D LUTs
      case icSigRedTRCTag:
      case icSigGreenTRCTag:
      case icSigBlueTRCTag:
      case icSigGrayTRCTag:
      case icSigAppleAltRedTRC:
      case icSigAppleAltGreenTRC:
      case icSigAppleAltBlueTRC:
        {
        const char *sigDesc = icGetSigStr(buf1, bufSize, sig);
        CIccTag *pTag = pIcc->FindTag(tag); // load if needed
        process1DLUT(pIcc, pTag, sigDesc, basename );
        }
        break;

      // nD LUTs
      case icSigAToB0Tag:
      case icSigAToB1Tag:
      case icSigAToB2Tag:
      case icSigAToB3Tag:
      case icSigBToA0Tag:
      case icSigBToA1Tag:
      case icSigBToA2Tag:
      case icSigBToA3Tag:
      case icSigGamutTag:
      case icSigPreview0Tag:
      case icSigPreview1Tag:
      case icSigPreview2Tag:
        {
        std::string sigDesc = icGetSigStr(buf1, bufSize, sig);
        CIccTag *pTag = pIcc->FindTag(tag); // load if needed
        process3DLUT(pIcc, pTag, sigDesc, basename );
        }
        break;

      // ignore everything else
      default:
        break;

    }   // end switch over tag signatures
  }   // end loop over tags

}   // end processProfile()

/******************************************************************************/

#ifndef NDEBUG
static
void unitTestPredictorInner( const predictor_desc &pred,
                        uint16_t *input, uint16_t *output, uint16_t *verify,
                        size_t pixelCount, uint8_t dimensions, uint32_t *dimArray,
                        uint8_t bitDepth, uint8_t channels )
{
  memset( output, 0, pixelCount*2 );
  memset( verify, 2, pixelCount*2 );
  applyOnePredictor( pred, (uint8_t*)input, (uint8_t*)output,
                   dimArray, dimensions, bitDepth, channels, pixelCount, false );
  applyOnePredictor( pred, (uint8_t*)output, (uint8_t*)verify,
                   dimArray, dimensions, bitDepth, channels, pixelCount, true );
  if ( memcmp(input,verify,pixelCount) != 0 ) {
    LogAnError(stderr, "ERROR - predictor %s reverse %dbits %dchannels %ddimensions failed unit test\n",
                pred.name, bitDepth, channels, dimensions );
  }
}
#endif      // DEBUG

/******************************************************************************/

#ifndef NDEBUG
static
void unitTestPredictorMiddle( const predictor_desc &pred,
                        uint16_t *input, uint16_t *output, uint16_t *verify,
                        size_t pixelCount, uint8_t dimensions, uint32_t *dimArray )
{
  unitTestPredictorInner( pred,input,output,verify,pixelCount,dimensions,dimArray, 8, 1 );
  unitTestPredictorInner( pred,input,output,verify,pixelCount,dimensions,dimArray, 16, 1 );
  unitTestPredictorInner( pred,input,output,verify,pixelCount,dimensions,dimArray, 16, 3 );
  unitTestPredictorInner( pred,input,output,verify,pixelCount,dimensions,dimArray, 16, 3 );
}
#endif      // DEBUG

/******************************************************************************/

// sanity check!
static
void unitTestPredictors(void)
{
#ifndef NDEBUG
  const uint32_t testLen = 5;
  const uint32_t testDim1 = 1;
  const uint32_t testDim2 = 2;
  const uint32_t testDim3 = 3;
  const uint32_t testDim4 = 8;
  const uint32_t testDim5 = 4;
  const uint32_t testMaxChannels = 3;
  uint32_t dimensionArray1[testDim1] = {testLen};
  uint32_t dimensionArray2[testDim2] = {testLen,testLen};
  uint32_t dimensionArray3[testDim3] = {testLen,testLen,testLen};
  uint32_t dimensionArray4[testDim4] = {testLen,testLen,testLen,testLen,testLen,testLen,testLen,testLen};
  uint32_t dimensionArray5[testDim5] = {5,3,2,4};       // uneven to test apply loops and offsets

  uint32_t pixelCount1 = testLen;
  uint32_t pixelCount2 = (uint32_t) pow( testLen, testDim2 );   // 25
  uint32_t pixelCount3 = (uint32_t) pow( testLen, testDim3 );   // 125
  uint32_t pixelCount4 = (uint32_t) pow( testLen, testDim4 );   // 390625
  uint32_t pixelCount5 = 1; // 120
  for (uint32_t i = 0; i < testDim5; ++i)
    pixelCount5 *= dimensionArray5[i];
  
  
  std::unique_ptr<uint16_t[]> inputBuffer(  new uint16_t[ testMaxChannels * pixelCount4 ] );
  std::unique_ptr<uint16_t[]> outputBuffer( new uint16_t[ testMaxChannels * pixelCount4 ] );
  std::unique_ptr<uint16_t[]> verifyBuffer( new uint16_t[ testMaxChannels * pixelCount4 ] );
  uint16_t *input = inputBuffer.get();
  uint16_t *output = outputBuffer.get();
  uint16_t *verify = verifyBuffer.get();


  // fill the input with an odd pattern
#if 0
  memset( input, 0xA5, 2*pixelCount4 ); // pattern for DEBUGGING
#else
  for (uint32_t i = 0; i < pixelCount4; ++i) {
    input[i] = (uint16_t)((41*i) & 0xFFFF);
  }
#endif

  // test median code
  srandom(0x424242);
  for (int i = 0; i < 20; ++i) {
    const int kMaxMedianCount = 4;
    std::vector<uint16_t> values(kMaxMedianCount);
    for (int k = 0; k < kMaxMedianCount; ++k)
      values[k] = random();
    
     auto result = median3(values[0],values[1],values[2]);
     auto verifyMedian = bruteForceMedian( {values[0],values[1],values[2]} );
    
     if (result != verifyMedian) {
       LogAnError(stderr, "ERROR - median3 failed on pass %d\n", i );
       break;
     }
  }
  
  for (int i = 0; i < 20; ++i) {
    const int kMaxMedianCount = 7;
    std::vector<uint16_t> values(kMaxMedianCount);
    for (int k = 0; k < kMaxMedianCount; ++k)
      values[k] = random();
    
     auto result = median7( {values[0],values[1],values[2],values[3],values[4],values[5],values[6]} );
     auto verifyMedian = bruteForceMedian( {values[0],values[1],values[2],values[3],values[4],values[5],values[6]} );
    
     if (result != verifyMedian) {
       LogAnError(stderr, "ERROR - median7 failed on pass %d\n", i );
       break;
     }
  }

#if 0
// test more sizes, but this is SLOW
  for (int j = 2; j < 22; ++j) {
    for (int i = 0; i < 20; ++i) {
      std::vector<uint16_t> values(j);
      for (int k = 0; k < j; ++k)
        values[k] = random();
      
       auto result = median( values );
       auto verifyMedian = bruteForceMedian( values );
      
       if (result != verifyMedian) {
         LogAnError(stderr, "ERROR - median%d failed on pass %d\n", j, i );
         break;
       }
    }
  }
#endif


  // simple compress and decompress to validate zlib
  size_t compSize = pixelCount3*2;
  if (!deflateBuffer( (uint8_t*)input, (uint8_t*)output, pixelCount3, compSize, 9 )) {
    LogAnError(stderr, "ERROR - zlib deflate failed\n" );
  }
  size_t fullSize = pixelCount3*2;
  if (!inflateBuffer( (uint8_t*)output, (uint8_t*)verify, compSize, fullSize )) {
    LogAnError(stderr, "ERROR - zlib inflate failed\n" );
  }
  if (fullSize != pixelCount3) {
    LogAnError(stderr, "ERROR - zlib failed unit test size\n" );
  }
  if (memcmp( input, verify, pixelCount3) != 0) {
    LogAnError(stderr, "ERROR - zlib failed unit test comparison\n" );
  }


  // iterate over all predictors, 8 and 16 bit
  for (const auto &pred : predictorList) {
    unitTestPredictorMiddle( pred, input, output, verify, pixelCount1, testDim1, dimensionArray1 );
    unitTestPredictorMiddle( pred, input, output, verify, pixelCount2, testDim2, dimensionArray2 );
    unitTestPredictorMiddle( pred, input, output, verify, pixelCount3, testDim3, dimensionArray3 );
    unitTestPredictorMiddle( pred, input, output, verify, pixelCount4, testDim4, dimensionArray4 );
    unitTestPredictorMiddle( pred, input, output, verify, pixelCount5, testDim5, dimensionArray5 );
  }
#endif      // DEBUG

}

/******************************************************************************/

static
void printUsage(void)
{
  printf("Usage: compressionResearch <args> input_profiles\n");
  printf("\t-silent         don't output any warnings or errors.\n");
  printf("\t-V              print usage and version.\n");
  printf("\t-help           print usage and version.\n");
  printf("compressionResearch built with IccProfLib version " ICCPROFLIBVER "\n\n");
}

/******************************************************************************/

typedef std::vector<std::string> filename_list;

static
filename_list parse_arguments( int argc, char *argv[] )
{
  filename_list filenames;

  for ( int c = 1; c < argc; ++c ) {

    if ( (strcasecmp( argv[c], "-silent" ) == 0 ) ) {
      gRunSilent = true;
    }
    else if ( strcasecmp( argv[c], "-V" ) == 0
            || strcasecmp( argv[c], "--V" ) == 0
            || strcasecmp( argv[c], "-help" ) == 0
            || strcasecmp( argv[c], "--help" ) == 0
            || strcasecmp( argv[c], "-version" ) == 0
            ) {
      printUsage();
      exit (0);
    }
    else if (argv[c][0] == '-') {
      // unrecognized switch
      printUsage();
      exit (1);
    }
    else {
      // not a switch, treat it as an input file
      filenames.push_back( argv[c] );
    }

  } // end loop over arguments


  if (filenames.size() == 0) {
      printUsage();
      exit (0);
  }

  return filenames;
}

/******************************************************************************/

int main(int argc, char* argv[])
{

#ifndef ICC_USE_ZLIB
  printf("ERROR - %s requires ZLIB!\n", argv[0] );
  return 1;
#endif

  if (argc <= 1) {
    printUsage();
    return 0;
  }
  
  filename_list fileList = parse_arguments(argc,argv);
  
  unitTestPredictors();
  
  for (auto &file : fileList) {
    std::string sanitizedFile = icSanitizeFileName( file );

    try {
      ClearErrorLogs(); // NOTE - this is so we can get logs per input file

      printf("Processing profile '%s'\n", file.c_str() );
      CIccProfile *pIcc = OpenIccProfile( file.c_str() );
      if (!pIcc) {
        LogAnError(stderr,"Unable to parse '%s' as ICC profile!\n", sanitizedFile.c_str() );
        continue;
      }

      std::string basename = remove_extension( sanitizedFile );

      processProfile( pIcc, basename );

      delete pIcc;
    }   // end try
    catch (const std::exception& e) {
      LogAnError(stderr, "%s: ERROR exception: '%s'\n", sanitizedFile.c_str() , e.what() );
    }
    catch (...) {
      LogAnError(stderr, "%s: ERROR: unknown exception\n", sanitizedFile.c_str() );
    }

    // NOTE - consume error logs here if needed, so exceptions are included

  } // end for file list

  return 0;
}

/******************************************************************************/
/******************************************************************************/
