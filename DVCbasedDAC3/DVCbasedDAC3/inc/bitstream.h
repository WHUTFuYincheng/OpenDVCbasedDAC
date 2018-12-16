
#ifndef ENCODER_INC_BITSTREAM_H
#define ENCODER_INC_BITSTREAM_H

#include <cstdio>

#include "config.h"

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
class Bitstream
{
public:
  Bitstream(int bufferCapacity, FILE* fileHandle);

  ~Bitstream() { delete [] _streamBuffer; };

  void write(int value, int numBits);

  int read(int numBits);

  bool read1Bit();

  void flush();

  int getBitCount() { return _bitCount; };

private:
  void write1Bit(int bitValue);

  FILE*   _fileHandle;      //!< Output file handle

  int     _numEmptyBits;    //!< Byte buffer empty bit number
  imgpel  _byteBuffer;      //!< Byte buffer

  int     _bufferCapacity;  //!< Internal buffer capacity
  int     _bufferSize;
  int     _byteCount;       //!< Internal buffer usage byte count
  int     _bitCount;
  imgpel* _streamBuffer;    //!< Internal buffer
};

#endif // ENCODER_INC_BITSTREAM_H

