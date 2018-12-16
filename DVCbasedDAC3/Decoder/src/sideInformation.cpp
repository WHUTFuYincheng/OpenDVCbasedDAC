
#include <vector>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

#include "sideInformation.h"
#include "corrModel.h"
#include "codec.h"
#include "decoder.h"



// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
//SideInformation::SideInformation(Codec* codec, CorrModel* model)
SideInformation::SideInformation(Decoder* codec, CorrModel* model)
{
  _codec = codec;
  _model = model;

  int width  = _codec->getFrameWidth();
  int height = _codec->getFrameHeight();

  _varList0 = new mvinfo[width * height / 64];
  _varList1 = new mvinfo[width * height / 64];

# if SI_REFINEMENT
  _refinedMask = new int[width * height / 16];
# endif
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SideInformation::lowpassFilter(imgpel* src, imgpel* dst, const int boxSize)
{
  int width  = _codec->getFrameWidth();
  int height = _codec->getFrameHeight();

  const int left  = boxSize/2;
  const int right = boxSize - (left + 1);

  for (int y = 0; y < height; y++)
    for (int x = 0; x < width; x++) {
      int sum   = 0;
      int total = 0;

      // Sum up pixels within the box
      for (int j = y-left; j <= y+right; j++)
        for (int i = x-left; i <= x+right; i++)
          if (i >= 0 && i < width &&
              j >= 0 && j < height) {
            sum += src[i+j*width];
            total++;
          }

      dst[x+y*width] = (imgpel)(sum/total);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
/*
*get approximately true motion field
*/
void SideInformation::forwardME(imgpel* prev, imgpel* curr, mvinfo* candidate, const int iRange)
{
  int iWidth,iHeight;
  iWidth  = _codec->getFrameWidth();
  iHeight = _codec->getFrameHeight();
  int px,py;
  float fDist;
  float fMinDist = -1.0;
  float fRatio;
  int iMinx=0,iMiny=0;
  int iIndex;
  int iMVx,iMVy;

  mvinfo *mv= new mvinfo[iWidth*iHeight/(iRange*iRange)];

  //half pixel buffer
  imgpel* buffer= new imgpel[iRange*iRange*4];


  for(int y=0;y<iHeight;y=y+iRange)
    for(int x=0;x<iWidth;x=x+iRange)
    {
      iIndex    = x/iRange+(y/iRange)*(iWidth/(iRange));
      fMinDist = -1;
      //first iteration: using 16x16 block size

      for(int pos=0;pos<(32+1)*(32+1);pos++)
        {
          iMVx=static_cast<Decoder*>(_codec)->getSpiralHpelSearchX()[pos];
          iMVy=static_cast<Decoder*>(_codec)->getSpiralHpelSearchY()[pos];

          px=iMVx+x;
          py=iMVy+y;

          fRatio=float(1.0+0.05*sqrtf((float)iMVx*iMVx+(float)iMVy*iMVy));
          if(px>=0 && px<iWidth && py>=0 && py<iHeight)
          {
            fDist=(float)calcSAD(prev,curr,px,py,x,y,iRange,iWidth,iHeight);
            fDist*=fRatio;

            if(fMinDist<0 || fDist<fMinDist)
            {
              fMinDist=fDist;
              iMinx=iMVx;
              iMiny=iMVy;
            }
          }
        }

      //}
      //half pixel ME
      //bilinear intepolation
      bilinear(prev,buffer,iRange,iRange,iWidth,iHeight,x+iMinx,y+iMiny);
      fMinDist=-1;

      for(int j=0;j<2;j++)
        for(int i=0;i<2;i++)
        {
          fDist=0;
          fDist=(float)calcSAD((buffer+i+j*(2*iRange)),(curr+x+y*iWidth),iRange*2, iWidth,2,1,iRange);
          if(fMinDist<0 || fDist<fMinDist)
          {
            fMinDist = fDist;
            mv[iIndex].iMvx=(2*iMinx+i)/2;
            mv[iIndex].iMvy=(2*iMiny+j)/2;
            mv[iIndex].iCx=x+iMinx/2;
            mv[iIndex].iCy=y+iMiny/2;
          }
        }

    }

  //select suitable motion vector for each block
  for(int j=0;j<iHeight/(iRange);j++)
    for(int i=0;i<iWidth/(iRange);i++)
    {
      iIndex=i+j*(iWidth/(iRange));
      candidate[iIndex].iCx=i*iRange;
      candidate[iIndex].iCy=j*iRange;
      candidate[iIndex].iMvx=mv[iIndex].iMvx;
      candidate[iIndex].iMvy=mv[iIndex].iMvy;

      int max_area=0;
      //select closest motion vector
      for(int k=0;k<(iWidth*iHeight/(iRange*iRange));k++)
      {
        int x=candidate[iIndex].iCx;
        int y=candidate[iIndex].iCy;
        int w=(x>mv[k].iCx)?(mv[k].iCx-x+iRange):(x-mv[k].iCx+iRange);
        int h=(y>mv[k].iCy)?(mv[k].iCy-y+iRange):(y-mv[k].iCy+iRange);

        if(w>=0 && h>=0 && w*h>=0.5*iRange*iRange){
          if(max_area==0 || w*h>max_area)
          {
            candidate[iIndex].iMvx=mv[k].iMvx;
            candidate[iIndex].iMvy=mv[k].iMvy;
          }
        }
      }
    }

  delete [] mv;
  delete [] buffer;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int SideInformation::calcSAD(imgpel* blk1, imgpel* blk2,int px,int py,int rx,int ry,const int blocksize,int width,int height){
  int sad=0;
  int cx,cy;
  for(int y=0;y<blocksize;y++)
    for(int x=0;x<blocksize;x++)
    {
      cx=px+x;
      cy=py+y;
      if(cx<=0)cx=0;
      if(cx>width-1)cx=width-1;
      if(cy<=0)cy=0;
      if(cy>height-1)cy=height-1;

      imgpel pel1=*(blk1+cx+cy*width);
      imgpel pel2=*(blk2+(rx+x)+(ry+y)*width);
      sad+=abs(pel1-pel2);
    }
  return sad;
}

int SideInformation::calcSAD(imgpel* blk1, imgpel* blk2, int width1, int width2, int s1,int s2,int blocksize){
  int sad=0;
  for(int y=0;y<blocksize;y++)
    for(int x=0;x<blocksize;x++)
    {
      imgpel pel1=*(blk1+s1*x+s1*y*width1);
      imgpel pel2=*(blk2+s2*x+s2*y*width2);
      sad+=abs(pel1-pel2);
    }
  return sad;
}

int SideInformation::calcSAD(imgpel* blk1, imgpel* blk2,const int blocksize,const int iPadSize){
  int iWidth;
  iWidth  = _codec->getFrameWidth();
  int sad=0;
  for(int y=0;y<blocksize;y++)
    for(int x=0;x<blocksize;x++)
    {
      imgpel pel1=*(blk1+x+y*(iWidth+2*iPadSize));
      imgpel pel2=*(blk2+x+y*(iWidth+2*iPadSize));
      sad+=abs(pel1-pel2);
    }
  return sad;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
/*
* create side information of the current frame
*/
void SideInformation::createSideInfo(imgpel* imgPrevKey, imgpel* imgNextKey, imgpel* imgCurrFrame)
{
  int width  = _codec->getFrameWidth();
  int height = _codec->getFrameHeight();

  imgpel* mc1 = new imgpel[width*height];
  imgpel* mc2 = new imgpel[width*height];

  imgpel* mc1_f = new imgpel[width*height];
  imgpel* mc1_b = new imgpel[width*height];
  imgpel* mc2_f = new imgpel[width*height];
  imgpel* mc2_b = new imgpel[width*height];

# if SI_REFINEMENT
  createSideInfoProcess(imgPrevKey, imgNextKey, mc1_f, mc1_b, 0);
  createSideInfoProcess(imgNextKey, imgPrevKey, mc2_b, mc2_f, 1);
# else
  createSideInfoProcess(imgPrevKey, imgNextKey, mc1_f, mc1_b);
  createSideInfoProcess(imgNextKey, imgPrevKey, mc2_b, mc2_f);
# endif

  for (int iy = 0; iy < height; iy++)
    for (int ix = 0; ix < width; ix++) {
      int iIdx = ix+iy*(width);

      imgCurrFrame[iIdx] = (mc1_f[iIdx] + mc1_b[iIdx] + mc2_f[iIdx] + mc2_b[iIdx] + 2)/4;

      mc1[iIdx] = (mc1_f[iIdx] + mc2_f[iIdx] + 1)/2;
      mc2[iIdx] = (mc1_b[iIdx] + mc2_b[iIdx] + 1)/2;
    }

  _model->correlationNoiseModeling(mc1, mc2);

  delete [] mc1;
  delete [] mc2;
  delete [] mc1_f;
  delete [] mc1_b;
  delete [] mc2_f;
  delete [] mc2_b;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
/*
* main process of creating side information
*/
# if SI_REFINEMENT
void SideInformation::createSideInfoProcess(imgpel* imgPrevKey, imgpel* imgNextKey, imgpel* imgMCForward, imgpel* imgMCBackward, int iMode)
# else
void SideInformation::createSideInfoProcess(imgpel* imgPrevKey, imgpel* imgNextKey, imgpel* imgMCForward, imgpel* imgMCBackward)
# endif
{
  int width  = _codec->getFrameWidth();
  int height = _codec->getFrameHeight();
  int iRange = 16;

  imgpel *imgPrevLowPass  = new imgpel[width*height];
  imgpel *imgNextLowPass  = new imgpel[width*height];

  imgpel *imgPrevKeyPadded  = new imgpel[(width+80)*(height+80)];
  imgpel *imgNextKeyPadded  = new imgpel[(width+80)*(height+80)];

  mvinfo *varCandidate        = new mvinfo[width*height/(iRange*iRange)];
# if SI_REFINEMENT
  mvinfo *varCandidate_iter2  = (iMode == 0) ? _varList0 : _varList1;
# else
  mvinfo *varCandidate_iter2  = new mvinfo[width*height/(iRange*iRange)*4];
# endif

  //apply lowpass filter to both key frames
  lowpassFilter(imgPrevKey , imgPrevLowPass, 3);
  lowpassFilter(imgNextKey , imgNextLowPass, 3);

  forwardME(imgPrevLowPass, imgNextLowPass, varCandidate , iRange);

  pad(imgPrevKey, imgPrevKeyPadded, 40);
  pad(imgNextKey, imgNextKeyPadded, 40);

  for (int iter = 0; iter < 2; iter++)
    spatialSmooth(imgPrevKeyPadded, imgNextKeyPadded, varCandidate, iRange, 40);

  //copy mv
  for (int y = 0; y < height; y += iRange)
    for (int x = 0; x < width; x += iRange) {
      int iIndex=(x/iRange)+(y/iRange)*(width/iRange);

      for (int j = 0; j < iRange; j += iRange/2)
        for (int i = 0; i < iRange; i += iRange/2) {
          int index2=(x+i)/(iRange/2)+(y+j)/(iRange/2)*(width/(iRange/2));

          varCandidate_iter2[index2].iMvx = varCandidate[iIndex].iMvx;
          varCandidate_iter2[index2].iMvy = varCandidate[iIndex].iMvy;
          varCandidate_iter2[index2].iCx  = x+i;
          varCandidate_iter2[index2].iCy  = y+j;
        }
    }

# if BIDIRECT_REFINEMENT
  bidirectME(imgPrevKeyPadded, imgNextKeyPadded, varCandidate_iter2, 40, iRange/2);
# endif

  for (int iter = 0; iter < 3; iter++)
    spatialSmooth(imgPrevKeyPadded, imgNextKeyPadded, varCandidate_iter2, iRange/2, 40);

  MC(imgPrevKeyPadded, imgNextKeyPadded, NULL, imgMCForward, imgMCBackward, varCandidate_iter2, NULL, 40, iRange/2, 0);

  delete [] imgPrevLowPass;
  delete [] imgNextLowPass;
  delete [] imgPrevKeyPadded;
  delete [] imgNextKeyPadded;
  delete [] varCandidate;
# if !SI_REFINEMENT
  delete [] varCandidate_iter2;
# endif
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
//get bidirectional motion field based on candidate mvs from previous stage (forward ME)
void SideInformation::bidirectME(imgpel* imgPrev, imgpel* imgNext, mvinfo* varCandidate,const int iPadSize,const int iRange){
  int iWidth,iHeight;
  iWidth  = _codec->getFrameWidth();
  iHeight = _codec->getFrameHeight();
  float sad,min_sad;
  float r;
  int iPosx[2]={0};
  int iPosy[2]={0};
  imgpel imgPel[2];
  int iMinx,iMiny;
  int yu,yb,xl,xr;
  int iIdx[4];
  bool bRefineFlag;

  imgpel *imgPrevBuffer,*imgNextBuffer;
  imgPrevBuffer=new imgpel[(iWidth+2*iPadSize)*(iHeight+2*iPadSize)*4];
  imgNextBuffer=new imgpel[(iWidth+2*iPadSize)*(iHeight+2*iPadSize)*4];

  bilinear(imgPrev,imgPrevBuffer,iWidth+2*iPadSize,iHeight+2*iPadSize,iWidth+2*iPadSize,iHeight+2*iPadSize,0,0);
  bilinear(imgNext,imgNextBuffer,iWidth+2*iPadSize,iHeight+2*iPadSize,iWidth+2*iPadSize,iHeight+2*iPadSize,0,0);
  for(int iIndex=0;iIndex<iWidth*iHeight/(iRange*iRange);iIndex++)
  {
    bRefineFlag=true;
    for(int i=0;i<4;i++)
    {
      iIdx[i]=-1;
    }
    int p_x=varCandidate[iIndex].iCx/iRange;
    int p_y=varCandidate[iIndex].iCy/iRange;
    if(p_y>0)                iIdx[0] = p_x+(p_y-1)*(iWidth/iRange);
    if(p_x>0)                iIdx[1] = p_x-1+p_y*(iWidth/iRange);
    if(p_x<iWidth/iRange-1)  iIdx[2] = p_x+1+p_y*(iWidth/iRange);
    if(p_y<iHeight/iRange-1) iIdx[3] = p_x+(p_y+1)*(iWidth/iRange);

    for(int i=0;i<4;i++)
    {
      if(iIdx[i]==-1)
      {
        bRefineFlag=false;
        break;
      }
    }

    if(bRefineFlag==true)
    {
      min_sad=-1;
      yu=varCandidate[iIdx[0]].iMvy;
      yb=varCandidate[iIdx[3]].iMvy;
      xl=varCandidate[iIdx[1]].iMvx;
      xr=varCandidate[iIdx[2]].iMvx;

      iMinx=iMiny=0;
      if(yu<=yb && xl<=xr)
      {
        for(int j=yu;j<=yb;j++)
          for(int i=xl;i<=xr;i++)
          {
            iPosx[0]=2*(varCandidate[iIndex].iCx+iPadSize)+i;
            iPosy[0]=2*(varCandidate[iIndex].iCy+iPadSize)+j;
            iPosx[1]=2*(varCandidate[iIndex].iCx+iPadSize)-i;
            iPosy[1]=2*(varCandidate[iIndex].iCy+iPadSize)-j;
            sad=0;

            r=float(1.0+0.05*sqrtf(float(i*i+j*j))/2.0);
            for(int y=0;y<iRange;y++)
            {
              for(int x=0;x<iRange;x++)
              {
                imgPel[0]=imgPrevBuffer[(iPosx[0]+2*x)+(iPosy[0]+2*y)*2*(2*iPadSize+iWidth)];
                imgPel[1]=imgNextBuffer[(iPosx[1]+2*x)+(iPosy[1]+2*y)*2*(2*iPadSize+iWidth)];
                sad+=abs(imgPel[0]-imgPel[1]);
              }
            }
            sad*=r;
            if(min_sad<0 || sad<min_sad){
              iMinx=i;
              iMiny=j;
              varCandidate[iIndex].fDist=sad;
              min_sad=sad;
            }
          }
        varCandidate[iIndex].iMvx=iMinx;
        varCandidate[iIndex].iMvy=iMiny;
      }
    }
  }
  delete [] imgPrevBuffer;
  delete [] imgNextBuffer;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
/*
* Spatial Smoothing
*/
void SideInformation::spatialSmooth(imgpel* imgPrev, imgpel* imgNext, mvinfo* varCandidate, const int iBlockSize, const int iPadSize)
{
  int iWidth,iHeight;
  iWidth  = _codec->getFrameWidth();
  iHeight = _codec->getFrameHeight();
  int iIndex[9];
  double dWeight[9];
  int iSAD[9];
  double dMinWeight;
  int iBestMVx=0,iBestMVy=0;
  int iPx[2],iPy[2];
  int iBestIdx=0;

  imgpel *imgPrevBuffer,*imgBufferNext;
  mvinfo *varRefine = new mvinfo[iWidth*iHeight/(iBlockSize*iBlockSize)];
  imgPrevBuffer     = new imgpel[(iWidth+2*iPadSize)*(iHeight+2*iPadSize)*4];
  imgBufferNext     = new imgpel[(iWidth+2*iPadSize)*(iHeight+2*iPadSize)*4];

  bilinear(imgPrev,imgPrevBuffer,iWidth+2*iPadSize,iHeight+2*iPadSize,iWidth+2*iPadSize,iHeight+2*iPadSize,0,0);
  bilinear(imgNext,imgBufferNext,iWidth+2*iPadSize,iHeight+2*iPadSize,iWidth+2*iPadSize,iHeight+2*iPadSize,0,0);

  for (int j = 0; j < iHeight/iBlockSize; j++)
    for (int i = 0; i < iWidth/iBlockSize; i++) {
      for (int k = 0; k < 9; k++)
        iIndex[k] = -1;

      if(j>0)                           iIndex[0]=i+(j-1)*(iWidth/iBlockSize);
      if(i>0)                           iIndex[1]=i-1+j*(iWidth/iBlockSize);
      if(i<iWidth/iBlockSize-1)         iIndex[2]=i+1+j*(iWidth/iBlockSize);
      if(j<iHeight/iBlockSize-1)        iIndex[3]=i+(j+1)*(iWidth/iBlockSize);
      if(i>0 && j>0)                    iIndex[5]=(i-1)+(j-1)*(iWidth/iBlockSize);
      if(i>0 && j<iHeight/iBlockSize-1) iIndex[6]=(i-1)+(j+1)*(iWidth/iBlockSize);
      if(i<iWidth/iBlockSize-1 && j>0)  iIndex[7]=(i+1)+(j-1)*(iWidth/iBlockSize);
      if(i<iWidth/iBlockSize-1 && j<iHeight/iBlockSize-1) iIndex[8]=(i+1)+(j+1)*(iWidth/iBlockSize);

      iIndex[4]=i+j*(iWidth/iBlockSize);

      varRefine[iIndex[4]].iMvx = varCandidate[iIndex[4]].iMvx;
      varRefine[iIndex[4]].iMvy = varCandidate[iIndex[4]].iMvy;
      varRefine[iIndex[4]].iCx  = varCandidate[iIndex[4]].iCx;
      varRefine[iIndex[4]].iCy  = varCandidate[iIndex[4]].iCy;

      for (int k = 0; k < 9; k++) {
        if (iIndex[k] != -1) {
          iPx[0]=2*(varCandidate[iIndex[4]].iCx+iPadSize)+varCandidate[iIndex[k]].iMvx;
          iPy[0]=2*(varCandidate[iIndex[4]].iCy+iPadSize)+varCandidate[iIndex[k]].iMvy;
          iPx[1]=2*(varCandidate[iIndex[4]].iCx+iPadSize)-varCandidate[iIndex[k]].iMvx;
          iPy[1]=2*(varCandidate[iIndex[4]].iCy+iPadSize)-varCandidate[iIndex[k]].iMvy;

          iSAD[k] = 0;
          iSAD[k] = calcSAD(imgPrevBuffer+iPx[0]+iPy[0]*2*(2*iPadSize+iWidth),
                            imgBufferNext+iPx[1]+iPy[1]*2*(2*iPadSize+iWidth),
                            2*(iWidth+2*iPadSize), 2*(iWidth+2*iPadSize), 2, 2, iBlockSize);
        }
      }

      dMinWeight = -1;

      for (int il = 0; il < 9; il++) {
        dWeight[il] = 0;

        if (iIndex[il] != -1) {
          for (int im = 0; im < 9; im++) {
            if (il != im && iIndex[im] != -1)
              dWeight[il] += (double)(iSAD[il]/std::max<double>(iSAD[im],0.0001))
                               *( abs(varCandidate[iIndex[il]].iMvx-varCandidate[iIndex[im]].iMvx)
                                 +abs(varCandidate[iIndex[il]].iMvy-varCandidate[iIndex[im]].iMvy));
			//	dWeight[il] += (double)(iSAD[il] / std::max<double>(iSAD[im], 0.0001))
			//		*(abs(varCandidate[iIndex[il]].iMvx - varCandidate[iIndex[im]].iMvx)
			//			+ abs(varCandidate[iIndex[il]].iMvy - varCandidate[iIndex[im]].iMvy));
          }
        }
        else
          dWeight[il]=-1;

        if ((dMinWeight<0 || dWeight[il]<=dMinWeight) && dWeight[il]>=0) {
          iBestMVx = varCandidate[iIndex[il]].iMvx;
          iBestMVy = varCandidate[iIndex[il]].iMvy;
          dMinWeight = dWeight[il];
          iBestIdx = il;
        }
      }

      varRefine[iIndex[4]].iMvx = iBestMVx;
      varRefine[iIndex[4]].iMvy = iBestMVy;
      varRefine[iIndex[4]].fDist = (float)iSAD[iBestIdx];
    }

  for (int iIndex = 0; iIndex < iWidth*iHeight/(iBlockSize*iBlockSize); iIndex++) {
    varCandidate[iIndex].iMvx = varRefine[iIndex].iMvx;
    varCandidate[iIndex].iMvy = varRefine[iIndex].iMvy;
    varCandidate[iIndex].iCx  = varRefine[iIndex].iCx;
    varCandidate[iIndex].iCy  = varRefine[iIndex].iCy;
    varCandidate[iIndex].fDist= varRefine[iIndex].fDist;
  }

  delete [] imgPrevBuffer;
  delete [] imgBufferNext;
  delete [] varRefine;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
/*
* this function support two modes
* mode 0 bilateral motion compensation
*        require only one motion vector set
* Param :candidate
* mode 1 bidirectional motion compensation
       require two motion vector sets (forward & backward)
* Param :candidate, candiate2
*/
void SideInformation::MC(imgpel* imgPrev, imgpel* imgNext, imgpel* imgDst ,imgpel* imgMCf,imgpel* imgMCb, mvinfo* varCandidate, mvinfo* varCandidate2, const int iPadSize, const int iRange, const int iMode){
  int iWidth,iHeight;
  iWidth  = _codec->getFrameWidth();
  iHeight = _codec->getFrameHeight();
  int px[2],py[2];
  imgpel pel[2];
  imgpel *imgPrevBuffer,*imgNextBuffer;
  imgPrevBuffer=new imgpel[(iWidth+2*iPadSize)*(iHeight+2*iPadSize)*4];
  imgNextBuffer=new imgpel[(iWidth+2*iPadSize)*(iHeight+2*iPadSize)*4];

  bilinear(imgPrev,imgPrevBuffer,iWidth+2*iPadSize,iHeight+2*iPadSize,iWidth+2*iPadSize,iHeight+2*iPadSize,0,0);
  bilinear(imgNext,imgNextBuffer,iWidth+2*iPadSize,iHeight+2*iPadSize,iWidth+2*iPadSize,iHeight+2*iPadSize,0,0);

  for(int iIndex=0;iIndex<iWidth*iHeight/(iRange*iRange);iIndex++)
  {

    for(int j=0;j<iRange;j++)
      for(int i=0;i<iRange;i++)
      {
        if(iMode==0)
        {
          px[0]=2*(varCandidate[iIndex].iCx+i+iPadSize)+varCandidate[iIndex].iMvx;
          py[0]=2*(varCandidate[iIndex].iCy+j+iPadSize)+varCandidate[iIndex].iMvy;
          px[1]=2*(varCandidate[iIndex].iCx+i+iPadSize)-varCandidate[iIndex].iMvx;
          py[1]=2*(varCandidate[iIndex].iCy+j+iPadSize)-varCandidate[iIndex].iMvy;
        }
        else
        {
          px[0]=2*(varCandidate[iIndex].iCx+i+iPadSize)+varCandidate[iIndex].iMvx;
          py[0]=2*(varCandidate[iIndex].iCy+j+iPadSize)+varCandidate[iIndex].iMvy;
          px[1]=2*(varCandidate2[iIndex].iCx+i+iPadSize)+varCandidate2[iIndex].iMvx;
          py[1]=2*(varCandidate2[iIndex].iCy+j+iPadSize)+varCandidate2[iIndex].iMvy;
        }
        pel[0]=imgPrevBuffer[px[0]+py[0]*2*(2*iPadSize+iWidth)];
        pel[1]=imgNextBuffer[px[1]+py[1]*2*(2*iPadSize+iWidth)];

        int pos=(varCandidate[iIndex].iCx+i)+(varCandidate[iIndex].iCy+j)*(iWidth);
        if(imgDst!=NULL)
        {
          imgDst[pos] = (pel[0]+pel[1]+1)/2;
        }
        imgMCf[pos] = pel[0];
        imgMCb[pos] = pel[1];
      }
  }
  delete [] imgPrevBuffer;
  delete [] imgNextBuffer;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SideInformation::pad(imgpel* src, imgpel* dst, const int iPadSize)
{
  int width  = _codec->getFrameWidth();
  int height = _codec->getFrameHeight();

  int paddedWidth  = width  + 2*iPadSize;
  int paddedHeight = height + 2*iPadSize;

  // Upper left
  for (int y = 0; y < iPadSize; y++)
    for (int x = 0; x < iPadSize; x++)
      dst[x+y*paddedWidth] = src[0];

  // Upper
  for (int y = 0; y < iPadSize; y++)
    for (int x = iPadSize; x < iPadSize+width; x++)
      dst[x+y*paddedWidth] = src[x-iPadSize];

  // Upper right
  for (int y = 0; y < iPadSize; y++)
    for (int x = iPadSize+width; x < paddedWidth; x++)
      dst[x+y*paddedWidth] = src[width-1];

  // Left
  for (int y = iPadSize; y < iPadSize+height; y++)
    for (int x = 0; x < iPadSize; x++)
      dst[x+y*paddedWidth] = src[(y-iPadSize)*width];

  // Middle
  for (int y = iPadSize; y < iPadSize+height; y++)
    for (int x = iPadSize; x < iPadSize+width; x++)
      dst[x+y*paddedWidth] = src[x-iPadSize+(y-iPadSize)*width];

  // Right
  for (int y = iPadSize; y < iPadSize+height; y++)
    for (int x = iPadSize+width; x < paddedWidth; x++)
      dst[x+y*paddedWidth] = src[(y-iPadSize+1)*width-1];

  // Bottom left
  for (int y = iPadSize+height; y < paddedHeight; y++)
    for (int x = 0; x < iPadSize; x++)
      dst[x+y*paddedWidth] = src[(height-1)*width];

  // Bottom
  for (int y = iPadSize+height; y < paddedHeight; y++)
    for (int x = iPadSize; x < iPadSize+width; x++)
      dst[x+y*paddedWidth] = src[(x-iPadSize)+(height-1)*width];

  // Bottom right
  for (int y = iPadSize+height; y < paddedHeight; y++)
    for (int x = iPadSize+width; x < paddedWidth; x++)
      dst[x+y*paddedWidth] = src[height*width-1];

}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void bilinear(imgpel *source,imgpel *buffer,int buffer_w,int buffer_h,int picwidth,int picheight,int px,int py){
  for(int j=0;j<buffer_h;j++)
    for(int i=0;i<buffer_w;i++)
    {
      int buffer_r=2*buffer_w;
      int a,b,c,d;

      int x=px+i;
        int y=py+j;
      if(x>picwidth-1)x=picwidth-1;
      if(x<0)x=0;
      if(y>picheight-1)y=picheight-1;
      if(y<0)y=0;
      a=source[(x)+(y)*picwidth];

      if((x+1)<picwidth) b=source[(x+1)+(y)*picwidth];
      else b=a;

      if((y+1)<picheight) c=source[(x)+(y+1)*picwidth];
      else c=a;

      if((x+1)<picwidth && (y+1)<picheight) d=source[(x+1)+(y+1)*picwidth];
      else d=a;

      buffer[2*i+(2*j)*buffer_r]=a;
      buffer[(2*i+1)+(2*j)*buffer_r]=(a+b)/2;
      buffer[2*i+(2*j+1)*buffer_r]=(a+c)/2;
      buffer[(2*i+1)+(2*j+1)*buffer_r]=(a+b+c+d)/4;
    }
}

#if RESIDUAL_CODING
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SideInformation::getResidualFrame(imgpel* bRefFrame, imgpel* fRefFrame, imgpel* currFrame, int* residue, int* dirList)
{
  int width  = _codec->getFrameWidth();
  int height = _codec->getFrameHeight();
  int iBlock = 8;
  int blockCount = 0;

  for (int j = 0; j < height; j += iBlock)
    for (int i = 0; i < width; i += iBlock) {
      imgpel* refFrame = (dirList[blockCount] == 0) ? bRefFrame : fRefFrame;

      for (int y = 0; y < iBlock; y++)
        for (int x = 0; x < iBlock; x++) {
          int idx = (i+x) + (j+y)*width;

          residue[idx] = currFrame[idx] - refFrame[idx];
        }

      blockCount++;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SideInformation::getRecFrame(imgpel *imgFReference,imgpel *imgBReference,int *iResidue,imgpel *imgRec,int *iList)
{
  int iWidth,iHeight;
  iWidth  = _codec->getFrameWidth();
  iHeight = _codec->getFrameHeight();
  int iBlock = 8;
  int iIndex=0;
  int iPos;
  imgpel* imgRef;
  for(int j=0;j<iHeight;j+=iBlock)
    for(int i=0;i<iWidth;i+=iBlock)
    {
      imgRef = (iList[iIndex]==0)? imgFReference:imgBReference;

      for(int y=0;y<iBlock;y++)
        for(int x=0;x<iBlock;x++)
        {
          iPos = (i+x) + (j+y)*iWidth;
          imgRec[iPos]=Clip(0,255,iResidue[iPos]+imgRef[iPos]);
        }
      iIndex++;
    }

}

#endif

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SideInformation::getSkippedRecFrame(imgpel* imgPrevKey,imgpel * imgWZFrame, int* skipMask)
{
  int iWidth,iHeight;
  iWidth  = _codec->getFrameWidth();
  iHeight = _codec->getFrameHeight();

  for(int ij=0;ij<iHeight;ij+=4)
    for(int ii=0;ii<iWidth;ii+=4)
    {
      int idx= ii/4 + ij/4*(iWidth/4);
      if(skipMask[idx]==1)//skip
      {
        for(int iy=0;iy<4;iy++)
          for(int ix=0;ix<4;ix++)
          {
            imgWZFrame[(ii+ix)+(ij+iy)*iWidth]=imgPrevKey[(ii+ix)+(ij+iy)*iWidth];
          }
      }
    }
}

#if SI_REFINEMENT
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SideInformation::getRefinedSideInfo(imgpel *imgPrevKey,imgpel *imgNextKey,imgpel *imgCurrFrame,imgpel* imgTmpRec,imgpel *imgRefined,int iMode)
{
  int iWidth,iHeight;
  iWidth           = _codec->getFrameWidth();
  iHeight          = _codec->getFrameHeight();
  int iBlock       = 8;
  int iPadSize     = 40;
  int iStripe      = iWidth+2*iPadSize;

  imgpel *imgForward,*imgBackward;
  imgpel *imgPrevBuffer,*imgNextBuffer,*imgPrevPadded,*imgNextPadded;
  imgForward    = new imgpel[iWidth*iHeight];
  imgBackward   = new imgpel[iWidth*iHeight];
  imgPrevPadded = new imgpel[(iWidth+2*iPadSize)*(iHeight+2*iPadSize)];
  imgNextPadded = new imgpel[(iWidth+2*iPadSize)*(iHeight+2*iPadSize)];

  mvinfo * mvList0 = new mvinfo[iWidth*iHeight/16];
  mvinfo * mvList1 = new mvinfo[iWidth*iHeight/16];

  pad(imgPrevKey,imgPrevPadded,iPadSize);
  pad(imgNextKey,imgNextPadded,iPadSize);

  imgPrevBuffer = new imgpel[(iWidth+2*iPadSize)*(iHeight+2*iPadSize)*4];
  imgNextBuffer = new imgpel[(iWidth+2*iPadSize)*(iHeight+2*iPadSize)*4];

  bilinear(imgPrevPadded,imgPrevBuffer,iWidth+2*iPadSize,iHeight+2*iPadSize,iWidth+2*iPadSize,iHeight+2*iPadSize,0,0);
  bilinear(imgNextPadded,imgNextBuffer,iWidth+2*iPadSize,iHeight+2*iPadSize,iWidth+2*iPadSize,iHeight+2*iPadSize,0,0);

  memset(imgForward,0x0,iWidth*iHeight);
  memset(imgBackward,0x0,iWidth*iHeight);

  for(int y=0;y<iHeight;y+=iBlock)
    for(int x=0;x<iWidth;x+=iBlock)
    {
      int iIndex=(x/iBlock)+(y/iBlock)*(iWidth/iBlock);
      for(int j=0;j<iBlock;j+=iBlock/2)
        for(int i=0;i<iBlock;i+=iBlock/2)
        {
          int index2=(x+i)/(iBlock/2)+(y+j)/(iBlock/2)*(iWidth/(iBlock/2));
          mvList0[index2].iMvx = _varList0[iIndex].iMvx;
          mvList0[index2].iMvy = _varList0[iIndex].iMvy;
          mvList0[index2].iCx  = x+i;
          mvList0[index2].iCy  = y+j;

          mvList1[index2].iMvx = _varList1[iIndex].iMvx;
          mvList1[index2].iMvy = _varList1[iIndex].iMvy;
          mvList1[index2].iCx  = x+i;
          mvList1[index2].iCy  = y+j;
        }
    }

  memset(_refinedMask,0x00,4*(iWidth*iHeight/(16)));

  getRefinedSideInfoProcess(imgPrevBuffer,imgTmpRec,imgCurrFrame,imgForward,mvList0,iMode);
  getRefinedSideInfoProcess(imgNextBuffer,imgTmpRec,imgCurrFrame,imgBackward,mvList1,iMode);

  for(int iy=0;iy<iHeight;iy++)
    for(int ix=0;ix<iWidth;ix++)
    {
      int iIdx = ix+iy*(iWidth);
      imgRefined[iIdx]=(imgForward[iIdx]+imgBackward[iIdx]+1)/2;
    }
  _model->updateCNM(imgForward,imgBackward,_refinedMask);

  delete [] mvList0;
  delete [] mvList1;
  delete [] imgForward;
  delete [] imgBackward;
  delete [] imgPrevPadded;
  delete [] imgNextPadded;
  delete [] imgPrevBuffer;
  delete [] imgNextBuffer;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
float getDCValue(imgpel* img,int iStep,int iStripe,int iBlock)
{
  float fDCValue;
  int sum = 0;
  for(int j=0;j<iBlock;j++)
    for(int i=0;i<iBlock;i++)
    {
      sum += *(img + i*iStep + j*iStep*iStripe);
    }
  fDCValue = (float)sum/float(iBlock*iBlock);
  return fDCValue;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void SideInformation::getRefinedSideInfoProcess(imgpel* imgPrevBuffer,imgpel* imgTmpRec,imgpel* imgSI,imgpel* imgRefined,mvinfo* varList,int iMode)
{
  int iWidth,iHeight;
  iWidth           = _codec->getFrameWidth();
  iHeight          = _codec->getFrameHeight();
  int iBlock       = 4;
  int x,y,i,j;
  int iCx,iCy;
  int iPox,iPoy;
  int iPadSize     = 40;
  int iSearchRange = 8;
  int iStripe      = iWidth+2*iPadSize;
  float fDist      = 0.0;
  float fSIDist    = 0.0;
  float fThreshold = 3.0;
  float fRefinedTH;
  float fRefinedTH_dc = (float)2.0*_codec->getQuantStep(0, 0);
  float fRefinedTH_ac = 100.0;

  fRefinedTH = (!iMode)?fRefinedTH_dc:fRefinedTH_ac;
  fThreshold = (!iMode)?   (float)4.0:fRefinedTH_ac;

  vector< pair<int,int> > varMVList;
  vector<float>         varWeight;
  float                 fWeightSum;
  float                 fTmp;

  //do ME again to find the better MV
  for(int iIndex=0;iIndex<(iHeight*iWidth)/(iBlock*iBlock);iIndex++)
    {
      varMVList.clear();
      varWeight.clear();
      fWeightSum = 0.0;
      x = 2*(varList[iIndex].iCx+iPadSize)+varList[iIndex].iMvx;
      y = 2*(varList[iIndex].iCy+iPadSize)+varList[iIndex].iMvy;
      iCx = varList[iIndex].iCx;
      iCy = varList[iIndex].iCy;

      float fRecDC = getDCValue(imgTmpRec+iCx+iCy*iWidth,1,iWidth,iBlock);
      float fDCValue;
      if(iMode==0)//DC
      {
        fDCValue = getDCValue(imgSI+iCx+iCy*iWidth,1,iWidth,iBlock);
        fSIDist  = fabs(fDCValue - fRecDC);
      }
      else
      {
        fSIDist  = (float)calcDist((imgSI+iCx+iCy*iWidth),(imgTmpRec+iCx+iCy*iWidth),iWidth,iWidth,1,1,iBlock);
      }
      if(fSIDist>fRefinedTH)
      {
        for(int pos=0;pos<(2*iSearchRange+1)*(2*iSearchRange+1);pos++)
        {
            i=static_cast<Decoder*>(_codec)->getSpiralSearchX()[pos];
            j=static_cast<Decoder*>(_codec)->getSpiralSearchY()[pos];

            if(iMode == 0)//DC
            {
              fDCValue = getDCValue(imgPrevBuffer+(x+i)+(y+j)*iStripe*2,2,iStripe*2,iBlock);
              fDist  =  fabs(fDCValue - fRecDC);
            }
            else
            {
              fDist  = (float)calcDist((imgPrevBuffer+(x+i)+(y+j)*iStripe*2),(imgTmpRec+iCx+iCy*iWidth),iStripe*2,iWidth,2,1,iBlock);
            }

            if(pos==0 && iMode!=0) fThreshold = fDist;

            if( fDist < 0.8*fThreshold)
            {
              varMVList.push_back(pair<int,int>(i,j));
              varWeight.push_back(1/(fDist+(float)0.001));
              fWeightSum += (1/(fDist+(float)0.001));
            }
        }
      }
      //weighted mean
      for(int ij=0;ij<iBlock;ij++)
        for(int ii=0;ii<iBlock;ii++)
        {
          fTmp = 0.0;
          if(varMVList.size()!=0)
          {
            for(size_t iList = 0;iList<varMVList.size();iList++)
            {
              iPox = x + 2*ii + varMVList[iList].first ;
              iPoy = y + 2*ij + varMVList[iList].second;
              fTmp += (*(imgPrevBuffer + iPox + iPoy*iStripe*2))*varWeight[iList];
            }
            imgRefined[(iCx+ii)+(iCy+ij)*iWidth] = (imgpel) (fTmp / fWeightSum);
          }
          else
          {
            imgRefined[(iCx+ii)+(iCy+ij)*iWidth] = imgSI[(iCx+ii)+(iCy+ij)*iWidth];
          }
        }

      if(varMVList.size()!=0)
      {
        _refinedMask[iIndex]++;
      }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int calcDist(imgpel* blk1, imgpel* blk2, int width1, int width2, int s1,int s2,int blocksize){
  int iDist=0;
  for(int y=0;y<blocksize;y++)
    for(int x=0;x<blocksize;x++)
    {
      imgpel pel1=*(blk1+s1*x+s1*y*width1);
      imgpel pel2=*(blk2+s2*x+s2*y*width2);
      iDist+=(pel1-pel2)*(pel1-pel2);
    }
  return iDist;
}
#endif

