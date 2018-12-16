
#ifndef DECODER_INC_SIDEINFORMATION_H
#define DECODER_INC_SIDEINFORMATION_H

#include "config.h"

class CorrModel;
//class Codec;
class Decoder;

class SideInformation
{
public:
  //SideInformation(Codec* codec, CorrModel* model);
  SideInformation(Decoder* codec, CorrModel* model);

# if SI_REFINEMENT
  void getRefinedSideInfo(imgpel *imgPrevKey,imgpel *imgNextKey,imgpel *imgCurrFrame,imgpel *imgTmpRec,imgpel *imgRefined,int iMode);
# endif

  void createSideInfo(imgpel* imgPreKey,imgpel* imgNextKey,imgpel* imgCurrFrame);

# if RESIDUAL_CODING
  void getResidualFrame(imgpel* bRefFrame, imgpel* fRefFrame, imgpel* currFrame, int* residue, int* dirList);
  void getRecFrame(imgpel *imgFReference,imgpel *imgBReference,int *iResidue,imgpel *imgRec,int *iList);
# endif

private:
  void lowpassFilter(imgpel* src, imgpel* dst, const int boxSize);
  void forwardME(imgpel* prev, imgpel* curr, mvinfo* candidate,const int range);
  void bidirectME(imgpel* prev, imgpel* next, mvinfo* candidate,const int iPadSize,const int range);
  void MC(imgpel* imgPrev, imgpel* imgNext, imgpel* imgDst ,imgpel* mc1,imgpel* mc2, mvinfo* candidate, mvinfo* candidate2, const int iPadSize, const int range, const int mode);

  void spatialSmooth(imgpel* imgPrev,imgpel* imgNext,mvinfo* varCandidate,const int iBlockSize,const int iPadSize);

  int calcSAD(imgpel* blk1, imgpel* blk2,int px,int py,int rx,int ry,const int blocksize,int width,int height);
  int calcSAD(imgpel* blk1, imgpel* blk2,const int blocksize,const int iPadSize);
  int calcSAD(imgpel* blk1, imgpel* blk2, int width1, int width2, int s1,int s2,int blocksize);
  void pad(imgpel *src,imgpel *dst, const int iPadSize);

  void getSkippedRecFrame(imgpel* imgPrevKey,imgpel * imgWZFrame, int* skipMask);

# if SI_REFINEMENT
  void createSideInfoProcess(imgpel* imgPrevKey,imgpel* imgNextKey,imgpel* imgMCForward,imgpel* imgMCBackward,int iMode);
  void getRefinedSideInfoProcess(imgpel* imgPrevBuffer,imgpel* imgTmpRec,imgpel* imgSI,imgpel* imgRefined,mvinfo* varList,int iMode);
# else
  void createSideInfoProcess(imgpel* imgPrevKey,imgpel* imgNextKey,imgpel* imgMCForward,imgpel* imgMCBackward);
# endif

  //Codec*      _codec;
  Decoder*      _codec;

  CorrModel*  _model;

  mvinfo*     _varList0;
  mvinfo*     _varList1;

# if SI_REFINEMENT
  int*        _refinedMask;
# endif
};

void bilinear(imgpel *source,imgpel *buffer,int buffer_w,int buffer_h,int picwidth,int picheight,int px,int py);

#if SI_REFINEMENT
float getDCValue(imgpel* img,int iStep,int iStripe,int iBlock);
int calcDist(imgpel* blk1, imgpel* blk2, int width1, int width2, int s1,int s2,int blocksize);
#endif

#endif // DECODER_INC_SIDEINFORMATION_H

