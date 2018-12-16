
#ifndef COMMON_INC_CODEC_H
#define COMMON_INC_CODEC_H

#include "config.h"
#include <iostream>

class Bitstream;

class Codec
{
public:
  Codec() {};
 

  int     getFrameWidth()      { return _frameWidth; };
  int     getFrameHeight()     { return _frameHeight; };
  int     getBitPlaneLength()  { return _bitPlaneLength; };
  int     getQp()              { return _qp; };
  int     getKeyQp()           { return _keyQp; };
  int     getNumChnCodeBands() { return _numChnCodeBands; };

  double* getAverage()         { return _average; };
  double* getAlpha()           { return _alpha; };
  double* getSigma()           { return _sigma; };

  int     getQuantMatrix(int qp, int x, int y) { return QuantMatrix[qp][y][x]; };
  int     getQuantStep(int x, int y) { return _quantStep[y][x]; };

  Bitstream* getBitstream() { return _bs; };





protected:
  const static int  ResidualBlockSize;
  const static int  SkipBlockSize;

  const static int  QuantMatrix[8][4][4];
  const static int  BitPlaneNum[8];
  const static int  MaxBitPlane[4][4];
# if RESIDUAL_CODING
  const static int  MinQStepSize[8][4][4];
# endif

  const static int  ScanOrder[16][2];
  const static int  HuffmanCodeValue[4][3][16];
  const static int  HuffmanCodeLength[4][3][16];
  
  int               _quantStep[4][4];

  int               _frameWidth;
  int               _frameHeight;
  int               _frameSize;
  int               _numFrames;
  int               _bitPlaneLength;
  int               _qp;
  int               _keyQp;
  int               _gopLevel;
  int               _gop;
  int               _numChnCodeBands;

  bool*             _parity;
  double*           _dParity;
  unsigned char*    _crc;
  double*           _average;
  double*           _alpha;
  double*           _sigma;

  Bitstream*        _bs;
  Bitstream*		_s;
#if CRC
  Bitstream*        _c;
#endif

#if ISCHANGE
  int				_Postion;
#endif

};

#endif // COMMON_INC_CODEC_H

