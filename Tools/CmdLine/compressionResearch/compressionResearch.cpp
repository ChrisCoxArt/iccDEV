/*
  File:     compressionResearch.cpp

  Contains:   Console app to test compression of profile tags

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


/******************************************************************************/

// calling types for predictors, will have different wrapper function for each type
enum predictor_type {
    PREDICTOR_TYPE_NULL = 0,
    PREDICTOR_TYPE_1D = 1,
    PREDICTOR_TYPE_2D = 2,
    PREDICTOR_TYPE_3D = 3,      // may be too much hassle
    PREDICTOR_TYPE_1DITER = 4,  // likely going to be worse than 1D
};

typedef void func_predictor( const uint8_t *input, uint8_t *output, int depth, uint8_t channels, size_t size1, size_t size2, size_t size3 );

struct predictor_desc {
    const char *name;
    predictor_type type;
    func_predictor *forward;
    func_predictor *reverse;
};

/******************************************************************************/

// forward declarations of predictor functions
static func_predictor null_forward;
static func_predictor null_reverse;
static func_predictor prev_forward;
static func_predictor prev_reverse;

/******************************************************************************/

std::vector<predictor_desc> predictorList =
{
 { "None", PREDICTOR_TYPE_NULL, null_forward, null_reverse },
 { "Prev", PREDICTOR_TYPE_1D, prev_forward, prev_reverse },

// byte shuffle, forward 1D

// TODO - WRITE ME!
// up, 2D
// Paeth, 2D
// MED, 2D

// prevIter, 1DIterated - calls prev

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

  inflateEnd(&zstr);

  return true;
}

/******************************************************************************/

static
bool deflateBuffer( uint8_t *input, uint8_t *output, size_t in_bytes, size_t &out_bytes, int level = 9 )
{
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
/******************************************************************************/

void apply1DPredictor( const predictor_desc &pred, const uint8_t *input, uint8_t *output,
                        uint32_t */*dimArray*/, uint8_t /*nDimensions*/, int depth,
                        uint8_t channels, size_t pixelCount, bool reverse )
{
  func_predictor *predFunc = pred.forward;
  if (reverse)
    predFunc = pred.reverse;

// WRITE ME!
// temp, 1D simple case
  predFunc( input, output, depth, channels, pixelCount, 0, 0 );

}

/******************************************************************************/

void apply1DIteratedPredictor( const predictor_desc &pred, const uint8_t */*input*/, uint8_t */*output*/,
                        uint32_t */*dimArray*/, uint8_t /*nDimensions*/, uint8_t /*depth*/,
                        uint8_t /*channels*/, size_t /*pixelCount*/, bool reverse )
{
  func_predictor *predFunc = pred.forward;
  if (reverse)
    predFunc = pred.reverse;
// WRITE ME!
}

/******************************************************************************/

void apply2DPredictor( const predictor_desc &pred, const uint8_t */*input*/, uint8_t */*output*/,
                        uint32_t */*dimArray*/, uint8_t /*nDimensions*/, uint8_t /*depth*/,
                        uint8_t /*channels*/, size_t /*pixelCount*/, bool reverse )
{
  func_predictor *predFunc = pred.forward;
  if (reverse)
    predFunc = pred.reverse;
// WRITE ME!
}

/******************************************************************************/

void apply3DPredictor( const predictor_desc &pred, const uint8_t */*input*/, uint8_t */*output*/,
                        uint32_t */*dimArray*/, uint8_t /*nDimensions*/, uint8_t /*depth*/,
                        uint8_t /*channels*/, size_t /*pixelCount*/, bool reverse )
{
  func_predictor *predFunc = pred.forward;
  if (reverse)
    predFunc = pred.reverse;
// WRITE ME!
}

/******************************************************************************/

// call different wrapper functions based on each type
static
void applyOnePredictor( const predictor_desc &pred, const uint8_t *input, uint8_t *output,
                        uint32_t *dimArray, uint8_t nDimensions, int depth, uint8_t channels,
                        size_t pixelCount, bool reverse = false )
{
  func_predictor *predFunc = pred.forward;
  if (reverse)
    predFunc = pred.reverse;

  switch (pred.type) {
    case PREDICTOR_TYPE_NULL:
      predFunc(input,output,depth,channels,pixelCount,0,0);      // needs no wrapper, just copy the whole array
      break;

    case PREDICTOR_TYPE_1D:
      apply1DPredictor( pred, input, output, dimArray, nDimensions, depth, channels, pixelCount, reverse );
      break;

    case PREDICTOR_TYPE_2D:
      if (nDimensions > 1)
        apply2DPredictor( pred, input, output, dimArray, nDimensions, depth, channels, pixelCount, reverse );
      break;

    case PREDICTOR_TYPE_3D:
      if (nDimensions > 2)
        apply3DPredictor( pred, input, output, dimArray, nDimensions, depth, channels, pixelCount, reverse );
      break;

    case PREDICTOR_TYPE_1DITER:
      if (nDimensions > 1)
        apply1DIteratedPredictor( pred, input, output, dimArray, nDimensions, depth, channels, pixelCount, reverse );
      break;
    
    default:
      LogAnError(stderr,"%s: ERROR - unknown or unimplemented predictor type %d\n", pred.type );
      break;
  }

}

/******************************************************************************/

static
void null_forward( const uint8_t *input, uint8_t *output, int depth, uint8_t channels, size_t size1, size_t /*size2*/, size_t /*size3*/ )
{
  memcpy( output, input, size1*(depth/8)*channels );
}

static
void null_reverse( const uint8_t *input, uint8_t *output, int depth, uint8_t channels, size_t size1, size_t /*size2*/, size_t /*size3*/ )
{
  memcpy( output, input, size1*(depth/8)*channels );
}

/******************************************************************************/

static
void prev_forward( const uint8_t *input, uint8_t *output, int depth, uint8_t channels, size_t size1, size_t /*size2*/, size_t /*size3*/ )
{
  if (depth == 8) {
    for (int c = 0; c < channels; ++c) {
      output[c] = input[c];
    }
    for (size_t i = 1; i < size1; ++i) {
      const uint8_t *in = input + i*channels;
      const uint8_t *prev = input + (i-1)*channels;
      uint8_t *out = output + i*channels;
      for (int c = 0; c < channels; ++c) {
        out[c] = in[c] - prev[c];   // overflow/underflow is intentional
      }
    }
  }
  
  if (depth == 16) {
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (int c = 0; c < channels; ++c) {
      output16[c] = input16[c];
    }
    for (size_t i = 1; i < size1; ++i) {
      const uint16_t *in = input16 + i*channels;
      const uint16_t *prev = input16 + (i-1)*channels;
      uint16_t *out = output16 + i*channels;
      for (int c = 0; c < channels; ++c) {
        out[c] = in[c] - prev[c];   // overflow/underflow is intentional
      }
    }
  }
}

static
void prev_reverse( const uint8_t *input, uint8_t *output, int depth, uint8_t channels, size_t size1, size_t /*size2*/, size_t /*size3*/ )
{
  if (depth == 8) {
    for (int c = 0; c < channels; ++c) {
      output[c] = input[c];
    }
    for (size_t i = 1; i < size1; ++i) {
      const uint8_t *in = input + i*channels;
      const uint8_t *prev = output + (i-1)*channels;
      uint8_t *out = output + i*channels;
      for (int c = 0; c < channels; ++c) {
        out[c] = in[c] + prev[c];   // overflow/underflow is intentional
      }
    }
  }
  
  if (depth == 16) {
    uint16_t *input16 = (uint16_t*)input;
    uint16_t *output16 = (uint16_t*)output;
    for (int c = 0; c < channels; ++c) {
      output16[c] = input16[c];
    }
    for (size_t i = 1; i < size1; ++i) {
      const uint16_t *in = input16 + i*channels;
      const uint16_t *prev = output16 + (i-1)*channels;
      uint16_t *out = output16 + i*channels;
      for (int c = 0; c < channels; ++c) {
        out[c] = in[c] + prev[c];   // overflow/underflow is intentional
      }
    }
  }
}

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
  if (curve->Validate(path, report, pIcc ) > icValidateWarning) {
    LogAnError(stderr,"%s: WARNING - 3D table failed validation:\n%s\n", filename.c_str(), report.c_str() );
    description = "MBBLut";
    return true;
  }
  curve->Describe( description, 100 );
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
    // depth is always 8.8
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

    // allow extra room just in case
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
void testCLUT(CIccProfile */*pIcc*/, CIccCLUT */*clut*/, const std::string &/*sigDesc*/,
                const std::string &/*basename*/ )
{
// WRITE ME!
    // iterate over all predictors, compress, report size


    // optionally reverse for testing?      or do unit tests on all predictors beforehand?
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

  testCLUT( pIcc, clut, sigDesc, basename );

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

// sanity check!
static
void unitTestPredictors(void)
{
  const uint32_t testLen = 5;
  const uint32_t testDim2 = 3;
  const uint32_t testDim3 = 8;
  uint32_t dimensionArray1[2] = {testLen,0};
  uint32_t dimensionArray2[testDim2] = {testLen,testLen,testLen};
  uint32_t dimensionArray3[testDim3] = {testLen,testLen,testLen,testLen,testLen,testLen,testLen,testLen};
  
  uint32_t pixelCount1 = testLen;
  uint32_t pixelCount2 = 125; // (uint32_t) pow( testLen, 3 )
  uint32_t pixelCount3 = 390625; // (uint32_t) pow( testLen, 8 )
  
  std::unique_ptr<uint16_t[]> inputBuffer( new uint16_t[ pixelCount3 ] );
  std::unique_ptr<uint16_t[]> outputBuffer( new uint16_t[ pixelCount3 ] );
  std::unique_ptr<uint16_t[]> verifyBuffer( new uint16_t[ pixelCount3 ] );
  uint16_t *input = inputBuffer.get();
  uint16_t *output = outputBuffer.get();
  uint16_t *verify = verifyBuffer.get();


  // iterate over all predictors, 8 and 16 bit
  for (const auto &pred : predictorList) {
  
    memset( output, 0, pixelCount1*2 );
    applyOnePredictor( pred, (uint8_t*)input, (uint8_t*)output,
                       dimensionArray1, 1, 8, 1, pixelCount1, false );
    applyOnePredictor( pred, (uint8_t*)output, (uint8_t*)verify,
                       dimensionArray1, 1, 8, 1, pixelCount1, true );
    if ( memcmp(input,verify,pixelCount1) != 0 ) {
        LogAnError(stderr, "ERROR - predictor %s reverse8 failed unit test1\n",
                    pred.name );
    }
  
    memset( output, 0, pixelCount1*2 );
    applyOnePredictor( pred, (uint8_t*)input, (uint8_t*)output,
                       dimensionArray1, 1, 16, 1, pixelCount1, false );
    applyOnePredictor( pred, (uint8_t*)output, (uint8_t*)verify,
                       dimensionArray1, 1, 16, 1, pixelCount1, true );
    if ( memcmp(input,verify,pixelCount1*2) != 0 ) {
        LogAnError(stderr, "ERROR - predictor %s reverse16 failed unit test1\n",
                    pred.name );
    }
    
// TODO - full 3 dimension test
// TODO - full 8 dimension test
  }



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

printf("Processing profile '%s'\n", file.c_str() );     // DEBUGGING
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
