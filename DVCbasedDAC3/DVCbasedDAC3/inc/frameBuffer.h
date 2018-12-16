
#ifndef ENCODER_INC_FRAMEBUFFER_H
#define ENCODER_INC_FRAMEBUFFER_H

#include "config.h"

class FrameBuffer
{
public:
  FrameBuffer(int width, int height, int gop = 0)
  {
    _prevFrame        = new imgpel[width*height];
    _currFrame        = new imgpel[width*height];
    _nextFrame        = new imgpel[width*height];
    _sideInfoFrame    = new imgpel[width*height];

    if (gop != 0) {
      _recFrames      = new imgpel*[gop-1];

      for (int i = 0; i < gop-1; i++)
        _recFrames[i] = new imgpel[width*height];
    }

    _dctFrame         = new int[width*height];
    _quantDctFrame    = new int[width*height];
    _decFrame         = new int[width*height];
    _invQuantDecFrame = new int[width*height];
  };

  imgpel*  getPrevFrame()        { return _prevFrame; };
  imgpel*  getCurrFrame()        { return _currFrame; };
  imgpel*  getNextFrame()        { return _nextFrame; };
  imgpel*  getSideInfoFrame()    { return _sideInfoFrame; };
  imgpel** getRecFrames()        { return _recFrames; };
  int*     getDctFrame()         { return _dctFrame; };
  int*     getQuantDctFrame()    { return _quantDctFrame; };
  int*     getDecFrame()         { return _decFrame; };
  int*     getInvQuantDecFrame() { return _invQuantDecFrame; };

private:
  imgpel*  _prevFrame;
  imgpel*  _currFrame;
  imgpel*  _nextFrame;
  imgpel*  _sideInfoFrame;
  imgpel** _recFrames;
  int*     _dctFrame;
  int*     _quantDctFrame;
  int*     _decFrame;
  int*     _invQuantDecFrame;
};

#endif // ENCODER_INC_FRAMEBUFFER_H

