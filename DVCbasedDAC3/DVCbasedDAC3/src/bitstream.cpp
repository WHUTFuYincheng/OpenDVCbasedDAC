
#include "bitstream.h"

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Bitstream::Bitstream(int bufferCapacity, FILE* fileHandle)
{
  _fileHandle     = fileHandle;

  _numEmptyBits   = 8;
  _byteBuffer     = 0;

  _bufferCapacity = bufferCapacity;
  _bufferSize     = 0;
  _byteCount      = 0;
  _bitCount       = 0;
  _streamBuffer   = new imgpel[_bufferCapacity];
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Bitstream::write(int value, int numBits)
{
  unsigned int bitPosition;

  if (numBits <= 32) {
    bitPosition = 1 << (numBits - 1);

    for (int i = 0; i < numBits; i++) {
      if (value & bitPosition)
        write1Bit(1);
      else
        write1Bit(0);

      bitPosition >>= 1;
    }
  }
  else {
    // Pad zeros
    for (int i = 0; i < numBits-32; i++)
      write1Bit(0);

    bitPosition = 1 << 31;

    for (int i = 0; i < 32; i++) {
      if (value & bitPosition)
        write1Bit(1);
      else
        write1Bit(0);

      bitPosition >>= 1;
    }
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int Bitstream::read(int numBits)
{
  int data = 0;

  for (int i = numBits-1; i >= 0; i--)
    if (read1Bit())
      data |= 1<<i;

  _bitCount += numBits;

  return data;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Bitstream::write1Bit(int bitValue)
{
  _byteBuffer <<= 1;
  _byteBuffer |= bitValue;

  // If the byte buffer is ready
  if (--_numEmptyBits == 0) {
    _numEmptyBits = 8;

    _streamBuffer[_byteCount++] = _byteBuffer;

    _byteBuffer = 0;

    // If the stream buffer is full
    if (_byteCount == _bufferCapacity) {
      fwrite(_streamBuffer, 1, _bufferCapacity, _fileHandle);

      _byteCount = 0;
    }
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
bool Bitstream::read1Bit()
{
  if (_byteCount == _bufferSize) {
    _bufferSize = fread(_streamBuffer, 1, _bufferCapacity, _fileHandle);
    _byteCount = 0;

    _byteBuffer = _streamBuffer[_byteCount];
    _numEmptyBits = 0;
  }

  bool bit = _byteBuffer & (1 << 7);

  _byteBuffer <<= 1;

  if (++_numEmptyBits == 8) {
    _numEmptyBits = 0;

    _byteBuffer = _streamBuffer[++_byteCount];
  }

  return bit;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Bitstream::flush()
{
  if (_numEmptyBits < 8 && _numEmptyBits > 0)
    _streamBuffer[_byteCount++] = (_byteBuffer << _numEmptyBits);

  if (_byteCount != 0)
    fwrite(_streamBuffer, 1, _byteCount, _fileHandle);

  _byteCount    = 0;

  _numEmptyBits = 8;
  _byteBuffer   = 0;
}

