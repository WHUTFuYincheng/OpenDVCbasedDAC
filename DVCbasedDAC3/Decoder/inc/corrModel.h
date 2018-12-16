
#ifndef DECODER_INC_CORRMODEL_H
#define DECODER_INC_CORRMODEL_H

#include "config.h"

class Transform;
class Codec;

class CorrModel
{
public:
  CorrModel(Codec* codec, Transform* trans) { _codec = codec; _trans = trans; };

# if SI_REFINEMENT
  void updateCNM(imgpel* imgForward,imgpel* imgBackward,int *g_iRefinedMask);
# endif

  void correlationNoiseModeling(imgpel* imgMCForward,imgpel *imgMCBackward);

  double getSoftInput(int* si,int* skipMask,int iCurrPos,int *decoded,double* LLR,int x,int y,int mode);

private:
  Codec*      _codec;

  Transform*  _trans;
};

double getLaplacian(int iLowerBound,int iUpperBound,double dAlpha,int iSideInfo,int iQuantSize,int iMode);
double getCondProb(int iBit,int iBand,int iBitLength,int iCurrPos,int iDecodedBit,double dAlpha,int iQuantSize,int iMode);

#endif // DECODER_INC_CORRMODEL_H

