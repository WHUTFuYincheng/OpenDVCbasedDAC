
#include <iostream>
#include <sstream>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <cmath>

#include "encoder.h"
#include "transform.h"
#include "fileManager.h"
#include "frameBuffer.h"
#include "bitstream.h"
#include "time.h"

#include "arithmetic_codec.h"

using namespace std;

# if RESIDUAL_CODING
const int Encoder::Scale[3][8] = {
  {8, 6, 6, 4, 4, 3, 2, 1},
  {8, 8, 8, 4, 4, 4, 2, 1},
  {4, 4, 4, 4, 3, 2, 2, 1}
};
# endif // RESIDUAL_CODING

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Encoder::Encoder(char** argv)
{
  _files = FileManager::getManager();

  _qp          = atoi(argv[1]);
  _keyQp       = atoi(argv[2]);
  _frameWidth  = atoi(argv[3]);
  _frameHeight = atoi(argv[4]);
  _numFrames   = atoi(argv[5]);
  _gopLevel    = atoi(argv[6]);


  _files->addFile("src", argv[7])->openFile("rb");
  _files->addFile("wz",  argv[8])->openFile("wb");
  _files->addFile("key", argv[9]);

  _bs = new Bitstream(1024, _files->getFile("wz")->getFileHandle());

  _files->addFile("source", "source.dat")->openFile("wb");
  _files->addFile("crc", "crc.dat")->openFile("wb");
  _s = new Bitstream(1024, _files->getFile("source")->getFileHandle());
#if CRC
  _c = new Bitstream(1024, _files->getFile("crc")->getFileHandle());			//for crc code
#endif
  initialize();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::initialize()
{
  _frameSize        = _frameWidth * _frameHeight;
  _bitPlaneLength   = _frameSize / 16;
  _gop              = 1 << _gopLevel;

  _numChnCodeBands  = 16;

  _parity           = new bool[_bitPlaneLength * BitPlaneNum[_qp]];


  if (_bitPlaneLength == 6336)
    _crc            = new unsigned char[BitPlaneNum[_qp] * 4];
  else
    _crc            = new unsigned char[BitPlaneNum[_qp]];


  _average          = new double[16];
  _alpha            = new double[_frameSize];
  _sigma            = new double[16];

  _skipMask         = new int[_bitPlaneLength];

  _prevMode         = 0;
  _prevType         = 0;

  for (int i = 0; i < 4; i++)
    _modeCounter[i] = 0;

  _fb = new FrameBuffer(_frameWidth, _frameHeight);

  _trans = new Transform(this);


}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::encodeKeyFrame()
{
  string srcFileName = _files->getFile("src")->getFileName();
  string keyFileName = _files->getFile("key")->getFileName();

  cout << "Running JM to encode key frames" << endl;

  stringstream cmd(stringstream::in | stringstream::out);

  //cmd << "cd jm; ";
  cmd << "jm\\lencod.exe -d jm\\encoder_intra_main.cfg ";
  cmd << "-p InputFile= \"" << srcFileName << "\" ";
  cmd << "-p ReconFile=" << "\"" << keyFileName << "\" ";
  cmd << "-p FramesToBeEncoded=" << ((_numFrames + _gop/2)/_gop) << " ";
  cmd << "-p QPISlice=" << _keyQp << " ";
  cmd << "-p FrameSkip=" << _gop-1 << " ";
  cmd << "-p SourceWidth=" << _frameWidth << " ";
  cmd << "-p SourceHeight=" << _frameHeight << " ";
  cmd << " > jm.log";

  system(cmd.str().c_str());

  cout << "Done encoding key frames" << endl << endl;

  _files->getFile("key")->openFile("rb");
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::encodeWzHeader()
{
  // Encode width and height in terms of macroblock
  _bs->write(_frameWidth/16, 8);
  _bs->write(_frameHeight/16, 8);
  _bs->write(_qp, 8);
  _bs->write(_numFrames, 16);
  _bs->write(_gopLevel, 2);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::encodeWzFrame()
{
  FILE* fReadPtr    = _files->getFile("src")->getFileHandle();
  FILE* fKeyReadPtr = _files->getFile("key")->getFileHandle();

  File* patternFile;
  FILE* patternFh;

  clock_t timeStart, timeEnd;
  double cpuTime;

  imgpel* currFrame     = _fb->getCurrFrame();
  int*    dctFrame      = _fb->getDctFrame();
  int*    quantDctFrame = _fb->getQuantDctFrame();

  int*    residue       = new int[_frameSize];

  timeStart = clock();

  encodeWzHeader();

  // Main loop
  // ---------------------------------------------------------------------------
  for (int keyFrameNo = 0; keyFrameNo < _numFrames/_gop; keyFrameNo++) {
    // Read previous key frame from the reconstructed key frame file
    fseek(fKeyReadPtr, (3*(keyFrameNo)*_frameSize)>>1, SEEK_SET);
    fread(_fb->getPrevFrame(), _frameSize, 1, fKeyReadPtr);

    // Read next key frame from the reconstructed key frame file
    fseek(fKeyReadPtr, (3*(keyFrameNo+1)*_frameSize)>>1, SEEK_SET);
    fread(_fb->getNextFrame(), _frameSize, 1, fKeyReadPtr);

    for (int il = 0; il < _gopLevel; il++) {
      int frameStep = _gop / ((il+1)<<1);
      int idx = frameStep;

      // Start encoding the WZ frame
      while (idx < _gop) {
        int wzFrameNo = keyFrameNo*_gop + idx;

        cout << "Encoding frame " << wzFrameNo << " (Wyner-Ziv frame)" << endl;
		_FrameNo = wzFrameNo;
        // Read current frame from the source file
        fseek(fReadPtr, (3*wzFrameNo*_frameSize)>>1, SEEK_SET);
        fread(currFrame, _frameSize, 1, fReadPtr);

        // ---------------------------------------------------------------------
        // STAGE 1 - Residual coding & DCT
        // ---------------------------------------------------------------------
# if RESIDUAL_CODING
        computeResidue(residue);
        _trans->dctTransform(residue, dctFrame);
# else // if !RESIDUAL_CODING
        _trans->dctTransform(currFrame, dctFrame);
# endif // RESIDUAL_CODING

        updateMaxValue(dctFrame);
        // ---------------------------------------------------------------------
        // STAGE 2 - Calculate quantization step size
        // ---------------------------------------------------------------------
        computeQuantStep();
        // ---------------------------------------------------------------------
        // STAGE 3 - Quantization
        // ---------------------------------------------------------------------
        _trans->quantization(dctFrame, quantDctFrame);
		
        // ---------------------------------------------------------------------
        // STAGE 4 - Mode decision
        // ---------------------------------------------------------------------
# if MODE_DECISION
        selectCodingMode(quantDctFrame);
# endif // MODE_DECISION

        // ---------------------------------------------------------------------
        // STAGE 5 - Skip mode
        // ---------------------------------------------------------------------
# if SKIP_MODE
        generateSkipMask();
        // TODO SKIP PATTERN
        encodeSkipMask();
# endif // SKIP_MODE

        // ---------------------------------------------------------------------
        // STAGE 6 - Encode (Channel/Entropy)
        // ---------------------------------------------------------------------
        int numBands = 0;

        _rcBitPlaneNum = 0;

        for (int bandNo = 0; bandNo < 16; bandNo++) {
          int x = ScanOrder[bandNo][0];
          int y = ScanOrder[bandNo][1];

          if (bandNo < _numChnCodeBands) {
            _rcQuantMatrix[y][x] = QuantMatrix[_qp][y][x];
            _rcBitPlaneNum += _rcQuantMatrix[y][x];
          }

          if (QuantMatrix[_qp][y][x] > 0)
            numBands++;
        }

        int bits = 0;

# if HARDWARE_FLOW
        if (bits%32 != 0) {
          int dummy = 32 - (bits%32);
          _bs->write(0, dummy);
        }
# endif // HARDWARE_FLOW

		encodeFrameDac(quantDctFrame);
        // ---------------------------------------------------------------------
        // STAGE 7 - Write parity and CRC bits to the bitstream
        // ---------------------------------------------------------------------
# if RESIDUAL_CODING
        for (int i = 0; i < _rcBitPlaneNum; i++)
# else // if !RESIDUAL_CODING
        for (int i = 0; i < BitPlaneNum[_qp]; i++)
# endif // RESIDUAL_CODING
        {
          for (int j = 0; j < _bitPlaneLength; j++)
            _bs->write(int(_parity[j+i*_bitPlaneLength]), 1);

# if !HARDWARE_FLOW
#   if HARDWARE_LDPC
          if (_bitPlaneLength == 6336)
            for (int n = 0; n < 4; n++)
              _bs->write(_crc[i*4+n], 8);
          else
            _bs->write(_crc[i], 8);
#   else // if !HARDWARE_LDPC
          _bs->write(_crc[i], 8);
#   endif // HARDWARE_LDPC
# endif // !HARDWARE_FLOW
        }

        idx += 2*frameStep;
      } // Finish encoding the WZ frame
    }
  }

  _bs->flush();
  _s->flush();
#if CRC
  _c->flush();
#endif
  timeEnd = clock();
  cpuTime = (timeEnd - timeStart) / CLOCKS_PER_SEC;

  cout << endl;
  cout << "--------------------------------------------------" << endl;
  cout << "Encode statistics" << endl;
  cout << "--------------------------------------------------" << endl;

  report();

  cout << "Total   encoding time: " << cpuTime << "(s)" << endl;
  cout << "Average encoding time: " << cpuTime/_numFrames << "(s)" << endl;
  cout << "--------------------------------------------------" << endl;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::computeResidue(int* residue)
{
  int     blockCount = 0;
  imgpel* refFrame;
  imgpel* backwardRefFrame = _fb->getPrevFrame();
  imgpel* forwardRefFrame  = _fb->getNextFrame();
  imgpel* currentFrame     = _fb->getCurrFrame();

  int sum = 0;

# if !HARDWARE_OPT
  int* dirList = new int[_frameSize/64];

  memset(dirList, 0, 4*_frameSize/64);
# endif

  for (int j = 0; j < _frameHeight; j += ResidualBlockSize)
    for (int i = 0; i < _frameWidth; i += ResidualBlockSize) {
# if HARDWARE_OPT
      refFrame = backwardRefFrame;
# else // if !HARDWARE_OPT
      int bckDist; // backward distortion
      int fwdDist; // forward distortion

      bckDist = computeSad(backwardRefFrame + i + j*_frameWidth,
                           currentFrame + i + j*_frameHeight,
                           _frameWidth, _frameWidth, 1, 1, ResidualBlockSize);

      fwdDist = computeSad(forwardRefFrame + i + j*_frameWidth,
                           currentFrame + i + j*_frameHeight,
                           _frameWidth, _frameWidth, 1, 1, ResidualBlockSize);

      dirList[blockCount] = (bckDist <= fwdDist) ? 0 : 1;

      refFrame = (dirList[blockCount] == 0) ? backwardRefFrame : forwardRefFrame;
# endif // HARDWARE_OPT
	  
      for (int y = 0; y < ResidualBlockSize; y++)
        for (int x = 0; x < ResidualBlockSize; x++) {
          int idx = (i+x) + (j+y)*_frameWidth;

          residue[idx] = currentFrame[idx] - refFrame[idx];

		  sum += abs(residue[idx]);
        }
	  
      blockCount++;
    }
  cout << "[residue=" << sum << "] ";
  cout << "blockCount=" << blockCount << endl;

# if !HARDWARE_FLOW
    // Encode motion vector
    for (int i = 0; i < _frameSize/64; i++)
      _bs->write(dirList[i], 1);
# endif
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int Encoder::computeSad(imgpel* blk1, imgpel* blk2, int width1, int width2, int step1, int step2, int blockSize)
{
  int sad = 0;

  for (int y = 0; y < blockSize; y++)
    for (int x = 0; x < blockSize; x++) {
      imgpel pel1 = *(blk1 + step1*x + step1*y*width1);
      imgpel pel2 = *(blk2 + step2*x + step2*y*width2);

      sad += abs(pel1 - pel2);
    }

  return sad;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::updateMaxValue(int* block)
{
  for (int y = 0; y < 4; y++)
    for (int x = 0; x < 4; x++)
      _maxValue[y][x] = 0;

  for (int y = 0; y < _frameHeight; y += 4)
    for (int x = 0; x < _frameWidth; x += 4)
      for (int j = 0; j < 4; j++)
        for (int i = 0; i < 4; i++)
          if (abs(_maxValue[j][i]) < abs(block[(x+i) + (y+j)*_frameWidth]))
            _maxValue[j][i] = block[(x+i) + (y+j)*_frameWidth];
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::computeQuantStep()
{
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 4; i++) {
# if RESIDUAL_CODING

#   if !HARDWARE_FLOW
      _bs->write(abs(_maxValue[j][i]), 11);
#   endif

      if (QuantMatrix[_qp][j][i] != 0) {
#   if HARDWARE_QUANTIZATION
        _quantStep[j][i] = 1 << (MaxBitPlane[j][i]+1-QuantMatrix[_qp][j][i]);
#   else // if !HARDWARE_QUANTIZATION
        int iInterval = 1 << QuantMatrix[_qp][j][i];

        _quantStep[j][i] = (int)ceil(double(2*abs(_maxValue[j][i]))/double(iInterval-1));
        _quantStep[j][i] = Max(_quantStep[j][i], MinQStepSize[_qp][j][i]);
#   endif // HARDWARE_QUANTIZATION
      }
      else
        _quantStep[j][i] = 1;

# else // if !RESIDUAL_CODING

      if (i != 0 || j != 0) {
        _bs->write(abs(_maxValue[j][i]), 11);

#   if HARDWARE_QUANTIZATION
        if (QuantMatrix[_qp][j][i] != 0)
          _quantStep[j][i] = 1 << (MaxBitPlane[j][i]+1-QuantMatrix[_qp][j][i]);
        else
          _quantStep[j][i] = 0;
#   else // if !HARDWARE_QUANTIZATION
        int iInterval = 1 << (QuantMatrix[_qp][j][i]);

#     if AC_QSTEP
        if (QuantMatrix[_qp][j][i] != 0) {
          _quantStep[j][i] = (int)ceil(double(2*abs(_maxValue[j][i]))/double(iInterval-1));

          if (_quantStep[j][i] < 0)
            _quantStep[j][i] = 0;
        }
        else
          _quantStep[j][i] = 1;
#     else // if !AC_QSTEP
        if (QuantMatrix[_qp][j][i] != 0)
          _quantStep[j][i] = ceil(double(2*abs(_maxValue[j][i]))/double(iInterval));
        else
          _quantStep[j][i] = 1;
#     endif // AC_QSTEP

#   endif // HARDWARE_QUANTIZATION
      }
      else
        _quantStep[j][i] = 1 << (DC_BITDEPTH-QuantMatrix[_qp][j][i]);
# endif // RESIDUAL_CODING
    }
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::selectCodingMode(int* frame)
{
  int numBands = 0;
  int mode;
  int codingMode;

  memset(_average, 0x00, 16*sizeof(double));

  for (int bandNo = 0; bandNo < 16; bandNo++) {
    int i = ScanOrder[bandNo][0];
    int j = ScanOrder[bandNo][1];

    if (QuantMatrix[_qp][j][i] > 0)
      numBands++;
  }

  for (int j = 0; j < _frameHeight; j++)
    for (int i = 0; i < _frameWidth; i++) {
      int mask, data;

      mask = (0x1 << (QuantMatrix[_qp][j%4][i%4]-1)) - 1;
      data = frame[i + j*_frameWidth] & mask;

      _average[(i%4) + (j%4)*4] += data;
    }

  for (int i = 0; i < 16; i++)
    _average[i] /= _bitPlaneLength;

  mode = 0;

  _modeCounter[mode]++;

# if HARDWARE_CMS
  codingMode = _prevMode;
  _prevMode  = mode;
# else // if !HARDWARE_CMS
  codingMode = mode;
# endif // HARDWARE_CMS

  _bs->write(codingMode, 2);

  if (codingMode == 0) _numChnCodeBands = 16; else
  if (codingMode == 1) _numChnCodeBands =  3; else
  if (codingMode == 2) _numChnCodeBands =  6; else
                       _numChnCodeBands =  0;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::generateSkipMask()
{
# if HARDWARE_OPT
  int threshold = 2;
# else // if !HARDWARE_OPT
  int threshold = 5;
# endif // HARDWARE_OPT

# if RESIDUAL_CODING
  int* frame    = _fb->getQuantDctFrame();
# else // if !RESIDUAL_CODING
  int* frame    = new int[_frameSize];
  int* frameDct = new int[_frameSize];

  for (int i = 0; i < _frameSize; i++)
    frame[i] = _fb->getPrevFrame()[i] - _fb->getCurrFrame()[i];

  _trans->dctTransform(frame, frameDct);
  _trans->quantization(frameDct, frame);
# endif // !RESIDUAL_CODING

  memset(_skipMask, 0x0, sizeof(int)*_bitPlaneLength);

  for (int j = 0; j < _frameHeight; j += SkipBlockSize)
    for (int i = 0; i < _frameWidth; i += SkipBlockSize) {
      int distortion = 0;
      int blockIndex = i/SkipBlockSize + (j/SkipBlockSize)*(_frameWidth/SkipBlockSize);

      for (int y = 0; y < SkipBlockSize; y++)
        for (int x = 0; x < SkipBlockSize; x++) {
          int mask, data;

          mask = (0x1 << (QuantMatrix[_qp][y][x]-1)) - 1;
          data = frame[(i+x) + (j+y)*_frameWidth] & mask;

# if HARDWARE_OPT
          distortion += data;
# else // if !HARDWARE_OPT
          distortion += data * data;
# endif // HARDWARE_OPT
        }

      _skipMask[blockIndex] = (distortion < threshold) ? 1 : 0;
    }

# if !RESIDUAL_CODING
  delete [] frame;
  delete [] frameDct;
# endif // !RESIDUAL_CODING
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int Encoder::encodeSkipMask()
{
  int n0 = 0; // number of non-skipped blocks
  int diff;
  int bitCount;
  int sign;
  int code;
  int length;
  int run = 0;
  int index = 0;

# if HARDWARE_FLOW
  // Pad zero
  int dummy = 20;
  _bs->write(0, dummy);
# endif // HARDWARE_FLOW

  for (int i = 0; i < _bitPlaneLength; i++)
    if (_skipMask[i] == 0)
      n0++;

  diff = abs(n0 - _bitPlaneLength/2);

# if HARDWARE_OPT
  int type = _prevType;
  int nextType = diff / (_bitPlaneLength/6);
  _prevType = nextType;
# else // if !HARDWARE_OPT
  int type = diff / (_bitPlaneLength/6);
# endif // HARDWARE_OPT

  if (type > 2) type = 2;

  _bs->write(type, 2);

  bitCount = 2;

  // Directly output first bit to bitstream
  sign = _skipMask[0];

  _bs->write(sign, 1);

  bitCount++;
  run++;
  index++;

# if TESTPATTERN
  File* patternFile;
  FILE* patternFh;

  patternFile = _files->addFile("pattern_rlc", "pattern_rlc.dat");
  patternFile->openFile("w");
  patternFh = patternFile->getFileHandle();

  for (int idx = 1; idx <= 8; idx++) {
    int data = (sign >> (32-idx*4)) & 0xf;
    fprintf(patternFh, "%x", data);
  }
  fprintf(patternFh, "\n");
# endif // TESTPATTERN

  // Huffman code for other bits
  while (index < _bitPlaneLength) {
    if (_skipMask[index] == sign) {
      run++;

      if (run == 16) { // reach maximum run length
        bitCount += getHuffmanCode(_qp, type, run-1, code, length);

        _bs->write(code, length);

        run = 1;

# if TESTPATTERN
        for (int idx = 1; idx <= 8; idx++) {
          int data = (code >> (32-idx*4)) & 0xf;
          fprintf(patternFh, "%x", data);
        }
        fprintf(patternFh, "\n");
# endif // TESTPATTERN
      }
    }
    else {
      bitCount += getHuffmanCode(_qp, type, run-1, code, length);

      _bs->write(code, length);

      sign = _skipMask[index];
      run = 1;

# if TESTPATTERN
      for (int idx = 1; idx <= 8; idx++) {
        int data = (code >> (32-idx*4)) & 0xf;
        fprintf(patternFh, "%x", data);
      }
      fprintf(patternFh, "\n");
# endif // TESTPATTERN
    }

    index++;
  }

  if (run != 0) {
    bitCount += getHuffmanCode(_qp, type, run-1, code, length);

    _bs->write(code, length);

# if TESTPATTERN
    for (int idx = 1; idx <= 8; idx++) {
      int data = (code >> (32-idx*4)) & 0xf;
      fprintf(patternFh, "%x", data);
    }
    fprintf(patternFh, "\n");
# endif // TESTPATTERN
  }

# if TESTPATTERN
  patternFile->closeFile();
# endif // TESTPATTERN

# if HARDWARE_FLOW
  if (bitCount%32 != 0) { // pad zero
    int dummy = 32 - (bitCount%32);
    _bs->write(0, dummy);
  }
# endif // HARDWARE_FLOW

  return bitCount;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int Encoder::getHuffmanCode(int qp, int type, int symbol, int& code, int& length)
{
  int table = qp / 2;

  code   = HuffmanCodeValue [table][type][symbol];
  length = HuffmanCodeLength[table][type][symbol];

  return length;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::encodeFrameDac(int* frame)
{
  int   bitPosition;
  unsigned char* crc=new unsigned char[1];
#if CRC
  int* buffer = new int[CRCLENGTH + 8];
#endif
  int*  dacSource = new int[_bitPlaneLength + 8];

  bool* dacValue = _parity;

  int count = 0;

  for (int band = 0; band < _numChnCodeBands; band++) {
    int i = ScanOrder[band][0];
    int j = ScanOrder[band][1];

# if RESIDUAL_CODING
    for (bitPosition = _rcQuantMatrix[j][i]-1; bitPosition >= 0; bitPosition--)
# else // if !RESIDUAL_CODING
    for (bitPosition = QuantMatrix[_qp][j][i]-1; bitPosition >= 0; bitPosition--)
# endif // RESIDUAL_CODING
    {		
#if ISCHANGE
		_Postion = bitPosition;

#endif // ISCHANGE
	  setupDACSource(frame, dacSource, i, j, bitPosition);
	  count += 1584 - _p0Num;
	  for (int count = 0; count < 1584; count++)
	  {
		  _s->write(int(dacSource[count]), 1);
	  }
#if CRC
	  for (int iCount = 0; iCount < _bitPlaneLength; iCount += CRCLENGTH)
	  {
		  memcpy(buffer, dacSource + iCount, CRCLENGTH * sizeof(int));
		  computeCRC(buffer, CRCLENGTH, crc);
		  for (int iy = 0; iy < 8; iy++)
		  {
			  int iValue = (int)((crc[0] >> (7 - iy)) & 0x01);
			  _c->write(iValue, 1);
		  }
	  }
#endif
	  
      if (_bitPlaneLength == 6336) {
		  _p0Num /= 4;
		  _xNum /= 4;
		  for (int n = 0; n < 4; n++) {
			  encodeDac(dacSource + n * 1584, dacValue);
			  
			  dacValue += _bitPlaneLength / 4;
		  }

		  cout << ".";
      }
      else {
		  encodeDac(dacSource, dacValue);

		  cout << ".";
		  dacValue += _frameSize / 16;

      }

    }
	//cout << endl;
  }
  cout << " count=" << count;
  cout << endl;

  delete[] dacSource;
#if CRC
  delete[] buffer;
#endif
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::setupLdpcaSource(int* frame, int* source, int offsetX, int offsetY, int bitPosition)
{
  for (int y = 0; y < _frameHeight; y = y+4)
    for (int x = 0; x < _frameWidth; x = x+4) {
      int blockIdx = (x/4) + (y/4)*(_frameWidth/4);
      int frameIdx = (x+offsetX) + (y+offsetY)*_frameWidth;

# if SKIP_MODE

      if (_skipMask[blockIdx] == 1)
        source[blockIdx] = 0;
      else
        source[blockIdx] = (frame[frameIdx] >> bitPosition) & 0x1;

# else // if !SKIP_MODE
      source[blockIdx] = (frame[frameIdx] >> bitPosition) & 0x1;
# endif // SKIP_MODE

    }
}
void Encoder::setupDACSource(int* frame, int*source, int offsetX, int offsetY, int bitPosition)
{
	int count = 0, xCount = 0;
	for (int y = 0; y < _frameHeight; y = y + 4)
		for (int x = 0; x < _frameWidth; x = x + 4) 
		{
			int blockIdx = (x / 4) + (y / 4)*(_frameWidth / 4);
			int frameIdx = (x + offsetX) + (y + offsetY)*_frameWidth;
			xCount++;
# if SKIP_MODE
			
			if (_skipMask[blockIdx] == 1)
				source[blockIdx] = 0, count++;
			else
			{
				source[blockIdx] = (frame[frameIdx] >> bitPosition) & 0x1;
				if (source[blockIdx] == 0)
					count++;
			}

# else // if !SKIP_MODE
			source[blockIdx] = (frame[frameIdx] >> bitPosition) & 0x1;
			if (source[blockIdx] == 0)
				count++;

# endif // SKIP_MODE
			
		}
	_xNum = xCount;
	_p0Num = count;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::computeCRC(int* data, const int length, unsigned char* crc)
{
  // CRC8 110011011
  //const int code[9] = { 1, 1, 0, 0, 1, 1, 0, 1, 1 };
	
  const int code[9] = { 1,1,0,0,1,1,0,1,1 };

  int* buffer = data;

  for (int i = length; i < length+8; i++)
    buffer[i] = 0;

  for (int i = 0; i < length; i++)
    if (buffer[i] == 1)
      for (int j = 0; j < 9; j++)
        buffer[i+j] = code[j] ^ buffer[i+j];

  *crc = 0;

  for (int i = 0; i < 8; i++)
    *crc |= buffer[length+i] << (7-i);

}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Encoder::report()
{
# if MODE_DECISION
  cout << "Mode usage: ";

  for (int i = 0; i < 4; i++) {
    float usage = (float)_modeCounter[i]/75.0 * 100.0;
    cout << usage << " ";
  }

  cout << endl;
# endif // MODE_DECISION
}

void Encoder::encodeDac(int* source, bool*dacParity)
{	
	double probability;
#if ISCHANGE	
	double overlap_input = OVERLAP;							//set overlap interval
	if (_Postion <= PSOTION)
		overlap_input = 0.0;
#else
	double overlap_input = OVERLAP;							//set overlap interval
#endif // ISCHANGE	
#if ADAPTIVEDAC
	if (_FrameNo == 35 || _FrameNo == 37 || _FrameNo == 47)
		overlap_input = 0.0;
#endif // ADAPTIVEDAC	

	unsigned BufferSize = 1584;								//Set bufferSize to 1584
	unsigned BufferCount = 1;								//Set bufferCount to 1
	unsigned TERMINATION = 15;
	unsigned char * data = new unsigned char[1584];
	unsigned nb, bytes = 0, crc = 0;						 // compute CRC (cyclic check) of file
	
	if (_xNum == _p0Num)
		probability = 0.999;
	else
		probability = _p0Num / (double)_xNum;

	bytes = BufferSize*BufferCount;

	// set data models
#if ISADAPTIVE
	Adaptive_Bit_Model abm;
	double tempOverlapInput = overlap_input;		//In order to keep the original overlap value unchanged while TERMINATION
#else
	Static_Bit_Model dm;										//Generate the object dm: that is, the data models data schema, which stores the probability of P(X=0), with the default of 0.5
	dm.set_probability_0(probability);							//Set the probability of P(X=0) in dm

#endif // ISADAPTIVE
	Arithmetic_Codec encoder(BufferSize);            // set encoder buffer
	do {
		nb = (bytes < BufferSize ? bytes : BufferSize);			//Each encoding length nb takes the size of bufferSize or when the bytes are not enough
		//if (fread(data, 1, nb, data_file) != nb) Error(R_MSG);   // read file data
		for (unsigned c = 0; c != nb; c++)
		{
			if (source[0])
				data[c] = 0x01;
			else
				data[c] = 0x00;
			source++;
		}		
		
		encoder.start_encoder();
#if ISADAPTIVE
		tempOverlapInput = overlap_input;
#else
		encoder.setoverlap(overlap_input, dm);
#endif // ISADAPTIVE
		for (unsigned p = 0; p < nb; p++)
		{
			// compress data
			if (p == (nb - TERMINATION))				//When nb reaches the terminator, set the probability to the original probability
			{
#if ISADAPTIVE
				tempOverlapInput = 0.0;
#else
				encoder.setoverlap(0.0, dm);
#endif // ISADAPTIVE
			}
#if ISADAPTIVE
			encoder.setoverlap(tempOverlapInput, abm);
			encoder.encode(data[p], abm);
#else
			encoder.encode(data[p], dm);			//Encoder.code_buffer encoder.code_buffer encoder.code_buffer encoder.code_buffer encoder.code_buffer encoder.code_buffer encoder.code_buffer
#endif // ISADAPTIVE
		}
		//encoder.write_to_file(code_file);  // stop encoder & write compressed data
		encoder.write_to_file(dacParity, _p0Num);
	} while (bytes -= nb);


	delete[] data;
}