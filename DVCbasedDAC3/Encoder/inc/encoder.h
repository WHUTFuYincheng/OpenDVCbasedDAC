
#ifndef ENCODER_INC_ENCODER_H
#define ENCODER_INC_ENCODER_H

#include "config.h"
#include "codec.h"

class FileManager;
class FrameBuffer;
class Transform;
class CavlcEnc;
class LdpcaEnc;
class Codec;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
class Encoder : public Codec
{
public:
  Encoder(char** argv);
  ~Encoder() { /* TODO Remember to free memory space */ };

  void encodeKeyFrame();
  void encodeWzFrame();

private:
  void initialize();

  void encodeWzHeader();

  void computeResidue(int* residue);
  int  computeSad(imgpel* blk1, imgpel* blk2, int width1, int width2, int step1, int step2, int blockSize);

  void updateMaxValue(int* block);

  void computeQuantStep();

  void selectCodingMode(int* frame);

  void generateSkipMask();

  int encodeSkipMask();
  int getHuffmanCode(int qp, int type, int symbol, int& code, int& length);

  void encodeFrameDac(int* frame);
  void setupLdpcaSource(int* frame, int* source, int offsetX, int offsetY, int bitPosition);
  void setupDACSource(int* frame, int*source, int offsetX, int offsetY, int bitPosition);		//Dac
  void encodeDac(int* source, bool*dacParity);
  void Write_to_file_all0(bool* dacParity);

  void computeCRC(int* data, const int length, unsigned char* crc);

  void report();

private:
  const static int  Scale[3][8];

  FileManager*      _files;

  FrameBuffer*      _fb;

  Transform*        _trans;

# if RESIDUAL_CODING | MODE_DECISION
  int               _rcBitPlaneNum;
  int               _rcQuantMatrix[4][4];
# endif

  int               _maxValue[4][4];
  int*              _skipMask;
  int               _prevMode;
  int               _prevType;

  int               _modeCounter[4];

  int				_p0Num;
  int				_xNum;
  int				_overLapMode;
  int				_FrameNo;
};

#endif // ENCODER_INC_ENCODER_H

