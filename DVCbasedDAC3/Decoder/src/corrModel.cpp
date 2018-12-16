
#include <cmath>

#include "corrModel.h"
#include "transform.h"
#include "codec.h"

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double getLaplacian(int iLowerBound,int iUpperBound,double dAlpha,int iSideInfo,int iQuantSize,int iMode){
  double dProb = 0.0;
  double dx = iLowerBound;
  if(iMode==1)
  {
    do
    {
      dProb+=exp(-1*dAlpha*iQuantSize*fabs(dx-iSideInfo));
      dx+=1.0;
    }while(dx<iUpperBound);
  }
  else if(iMode==2)
  {
    if(iUpperBound!=iLowerBound)
    {
      if(iUpperBound>= iSideInfo && iSideInfo>= iLowerBound)
      {
        dProb=1.0-(exp(-1.0*dAlpha*(double(iSideInfo-iLowerBound)))+exp(-1.0*dAlpha*(double(iUpperBound-iSideInfo))))/2.0;
      }
      else if(iLowerBound>=iSideInfo)
      {
        dProb=(exp(-1.0*dAlpha*(double(iLowerBound-iSideInfo)))-exp(-1.0*dAlpha*(double(iUpperBound-iSideInfo))))/2.0;
      }
      else if(iSideInfo>=iUpperBound)
      {
        dProb=(exp(dAlpha*(double(iUpperBound-iSideInfo)))-exp(dAlpha*(double(iLowerBound-iSideInfo))))/2.0;
      }
    }
    else
    {
      dProb=0.5*dAlpha*exp(-1*dAlpha*fabs(double(iUpperBound-iSideInfo)));
    }
  }
  return dProb;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double getCondProb(int iBit,int iBand,int iBitLength,int iCurrPos,int iDecodedBit,double dAlpha,int iQuantSize,int iMode){

  int iSign;
  int c;
  int lower;
  int upper;
  unsigned int value=0;

  if(iCurrPos==iBitLength-1)//msb
  {
    iSign=(iBit==1)?-1:1;
    c=(iSign==1)?0:-1*int(pow(2.0,double(iBitLength-1)));
    if(iSign==1)
    {
      upper=int(pow(2.0,double(iBitLength-1)))-1;
      lower=0;
    }
    else
    {
      upper=0;
      lower=-1*int(pow(2.0,double(iBitLength-1)))+1;
    }
    return getLaplacian(lower,upper,dAlpha,iBand,iQuantSize,iMode);
  }
  else
  {
    for(int i=iBitLength-2;i>iCurrPos;i--)
    {
      int b=(iDecodedBit>>i)&0x01;
      value|=(b*(0x01<<i));
    }
    value|=(iBit<<iCurrPos);
    iSign=((iDecodedBit>>(iBitLength-1))&0x01)?-1:1;
    c=(iSign==1)?0:-1*int(pow(2.0,double(iBitLength-1)));

    if(iSign==1)//positive
    {
      upper=(value+int(pow(2.0,double(iCurrPos)))-1);
      lower=(value);
    }
    else
    {
      lower=-1*(value+int(pow(2.0,double(iCurrPos)))-1);
      upper=-1*(value);
    }

    if(iMode==2)
    {
      if(iSign == 1)
      {
        upper=(value+int(pow(2.0,double(iCurrPos))))*iQuantSize-1;
        lower=(value)*iQuantSize;
      }
      else
      {
        lower=-1*((value+int(pow(2.0,double(iCurrPos))))*iQuantSize-1);
        upper=-1*(value)*iQuantSize;
      }
    }
    return getLaplacian(lower,upper,dAlpha,iBand,iQuantSize,iMode);
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double CorrModel::getSoftInput(int* si,int* skipMask,int iCurrPos,int *iDecoded,double* dLLR,int x,int y,int iMode){
  double p0,p1;
  double p=0.0;
  double tp=0.0;
  double entropy=0.0;
  int iIndex,iQuantSize,iBitLength;
  int iWidth,iHeight,iBitplaneLength;
  int iSideInfo,iQP;
  double* dAlpha;

  iWidth = _codec->getFrameWidth();
  iHeight= _codec->getFrameHeight();
  iBitplaneLength = _codec->getBitPlaneLength();
  dAlpha = _codec->getAlpha();
  iQP = _codec->getQp();
  int mask;

  for(int j=0;j<iHeight/4;j++)
    for(int i=0;i<iWidth/4;i++)
    {
      iIndex=i+j*(iWidth/4);
      iQuantSize=_codec->getQuantStep(x, y);
#if RESIDUAL_CODING
      iBitLength= _codec->getQuantMatrix(iQP, x, y);

      if(iMode==2)
        iSideInfo = si[(i*4+x)+(j*4+y)*iWidth];
      else
      {
        mask = (0x1<<(_codec->getQuantMatrix(iQP, x, y)-1))-1;
        int sign = (si[(i*4+x)+(j*4+y)*iWidth]>>(_codec->getQuantMatrix(iQP, x, y)-1))&0x1;
        int value = si[(i*4+x)+(j*4+y)*iWidth] & mask;

        iSideInfo = (sign==0)?value:-1*value;
      }
#else
      iBitLength=(x==0 && y==0) ? _codec->getQuantMatrix(iQP, x, y)+1 : _codec->getQuantMatrix(iQP, x, y);

      if(iMode==2 || (x==0 && y==0))
        iSideInfo = si[(i*4+x)+(j*4+y)*iWidth];
      else
      {
        mask = (0x1<<(_codec->getQuantMatrix(iQP, x, y)-1))-1;
        int sign = (si[(i*4+x)+(j*4+y)*iWidth]>>(_codec->getQuantMatrix(iQP, x, y)-1))&0x1;
        int value = si[(i*4+x)+(j*4+y)*iWidth] & mask;

        iSideInfo = (sign==0)?value:-1*value;
      }
#endif

      p0=getCondProb(0,iSideInfo,iBitLength,iCurrPos,iDecoded[(i)+(j)*iWidth/4],dAlpha[(i*4+x)+(j*4+y)*iWidth],iQuantSize,iMode);
      p1=getCondProb(1,iSideInfo,iBitLength,iCurrPos,iDecoded[(i)+(j)*iWidth/4],dAlpha[(i*4+x)+(j*4+y)*iWidth],iQuantSize,iMode);

#if SKIP_MODE
      if(skipMask[iIndex]==1)//skip
        dLLR[iIndex]=500;
      else
#endif
      dLLR[iIndex]=log(p0/p1);


	  p=p1/(p1+p0);
      entropy+= p*(log10(1.0/p)/log10(2.0))+(1.0-p)*(log10(1.0/(1-p))/log10(2.0));
      tp+=(p>0.5)?(1.0-p):(p);
      
    }

  entropy/=(iBitplaneLength);
  tp/=(iBitplaneLength);
  return 0.5*(entropy)*exp(entropy)+sqrt(tp*(1-tp));
}

/*
*update the sigma values for all dct bands
*Param
*/
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void CorrModel::correlationNoiseModeling(imgpel *imgMCFoward,imgpel* imgMCBackward){
  double e,e2,r;
  int iIndex;
  int iWidth,iHeight,iBitplaneLength;
  double* dAlpha;						//¦ÁÁ£×Ó
  double* dSigma;
  double* dAverage;
  iWidth = _codec->getFrameWidth();
  iHeight= _codec->getFrameHeight();
  iBitplaneLength = _codec->getBitPlaneLength();
  dAlpha = _codec->getAlpha();
  dSigma = _codec->getSigma();
  dAverage = _codec->getAverage();

  float *residue  = new float[iWidth*iHeight];
  float *residue_b  = new float[iWidth*iHeight];

  for(int j=0;j<iHeight;j++)
    for(int i=0;i<iWidth;i++)
    {
      residue[i+j*iWidth]=float(imgMCFoward[i+j*iWidth]-imgMCBackward[i+j*iWidth])/2;
    }
  _trans->dctTransform(residue,residue_b);

  for(int j=0;j<4;j++)
    for(int i=0;i<4;i++)
    {
      e=e2=0.0;
      for(int y=0;y<iHeight;y=y+4)
        for(int x=0;x<iWidth;x=x+4)
        {
          iIndex=(x+i)+(y+j)*iWidth;
          r=fabs((double)residue_b[iIndex]);
          e+=r;
          e2+=r*r;
        }

      e  /=iBitplaneLength;
      e2 /=iBitplaneLength;
      dSigma[i+j*4]=(e2-e*e);
      dAverage[i+j*4]=e;
    }
  for(int j=0;j<iHeight;j=j+4)
    for(int i=0;i<iWidth;i=i+4)
    {
      for(int y=0;y<4;y++)
        for(int x=0;x<4;x++)
        {
          iIndex=(x+i)+(y+j)*iWidth;
          double d=fabs((double)residue_b[iIndex])-dAverage[x+y*4];
          int iQuantStep=_codec->getQuantStep(x, y);
          if(d*d<=dSigma[x+y*4])
          {
            dAlpha[iIndex]=sqrt(2/(dSigma[x+y*4]+0.1*iQuantStep*iQuantStep));
            //dAlpha[iIndex]=sqrt(2/(dSigma[x+y*4]));
          }
          else
          {
            dAlpha[iIndex]=sqrt(2/(d*d+0.1*iQuantStep*iQuantStep));
            //dAlpha[iIndex]=sqrt(2/(d*d));
          }
        }
    }
  delete [] residue;
  delete [] residue_b;

}


#if SI_REFINEMENT
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void CorrModel::updateCNM(imgpel* imgForward,imgpel* imgBackward,int *refinedMask)
{
  int iIndex;
  int iWidth,iHeight,iBitplaneLength;
  double* dAlpha;
  double* dSigma;
  double* dAverage;
  iWidth = _codec->getFrameWidth();
  iHeight= _codec->getFrameHeight();
  iBitplaneLength = _codec->getBitPlaneLength();
  dAlpha = _codec->getAlpha();
  dSigma = _codec->getSigma();
  dAverage = _codec->getAverage();

  float *residue  = new float[iWidth*iHeight];
  float *residue_b  = new float[iWidth*iHeight];

  for(int j=0;j<iHeight;j=j+4)
    for(int i=0;i<iWidth;i=i+4)
    {
      if(refinedMask[i/4+(j/4)*iWidth/4])
      {
        for(int y=0;y<4;y++)
          for(int x=0;x<4;x++)
          {
            if(refinedMask[i/4+(j/4)*iWidth/4]==2)
              residue[i+x+(j+y)*iWidth]=float(imgForward[i+x+(j+y)*iWidth]-imgBackward[i+x+(j+y)*iWidth])/2;
            else
              residue[i+x+(j+y)*iWidth]=float(imgForward[i+x+(j+y)*iWidth]-imgBackward[i+x+(j+y)*iWidth]);
          }

        _trans->dct4x4(residue,residue_b,i,j);

        for(int y=0;y<4;y++)
          for(int x=0;x<4;x++)
          {
            iIndex=(x+i)+(y+j)*iWidth;
            double d=fabs((double)residue_b[iIndex])-dAverage[x+y*4];
            int iQuantStep=_codec->getQuantStep(x, y);
            if(d*d<=dSigma[x+y*4])
            {
              dAlpha[iIndex]=sqrt(2/(dSigma[x+y*4]+0.1*iQuantStep*iQuantStep));
            }
            else
            {
              dAlpha[iIndex]=sqrt(2/(d*d+0.1*iQuantStep*iQuantStep));
            }
          }
      }
    }
  delete [] residue;
  delete [] residue_b;
}
#endif

