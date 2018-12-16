
#ifndef COMMON_INC_TYPES_H
#define COMMON_INC_TYPES_H

typedef unsigned char byte;
typedef unsigned char imgpel;

// Contain 4 rows by 4 columns of 4x4 blocks
typedef struct _mb {
  int             I4;                 // 1:is I4 0:is I16
  int             nnz[4][4];          // num of nonzero coef for luma
  int             nnz_chroma[2][4];   // num of nonzero coef for chroma
  int             coef_ldc[16];       // luma dc
  int             coef_lac[4][4][16]; // luma ac
  int             coef_cdc[2][4];     // chroma dc
  int             coef_cac[2][4][16]; // chroma ac [mode][b4][data]
  int             pred_mode_I4[16];   // I4 intra pred mode
  int             pred_mode_I16;      // I16 intra pred mode
  int             pred_mode_uv;       // chroma intra pred mode
  int             level[16];
  int             Ones[3];
  int             iRun[16];
  unsigned char   cbp;                // coded block pattern
} mb;

typedef struct _mvinfo {
  int   iCx;     //current block position
  int   iCy;     //
  int   iMvx;    //x-dir motion vector
  int   iMvy;    //y-dir motion vector
  float fDist;
  bool  bOMBCFlag;
} mvinfo;

enum CAVLCBlockTypes {
  LUMA              =  0,
  LUMA_INTRA16x16DC =  1,
  LUMA_INTRA16x16AC =  2,
  CHROMA_DC         =  3,
  CHROMA_AC         =  4
};

template <typename T> inline T Min(T a, T b) { return (a < b) ? a : b; }
template <typename T> inline T Max(T a, T b) { return (a > b) ? a : b; }
template <typename T> inline T Clip(T lowBound, T upBound, T value)
{
  value = Max(value, lowBound);
  value = Min(value, upBound);

  return value;
}

template <typename T> T** AllocArray2D(int width, int height)
{
  T** img = new T*[height];

  for (int i = 0; i < height; i++)
    img[i] = new T[width];

  return img;
}

#endif // COMMON_INC_TYPES_H

