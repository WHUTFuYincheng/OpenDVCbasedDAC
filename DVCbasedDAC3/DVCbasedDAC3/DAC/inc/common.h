#ifndef _COMMON_H_
#define _COMMON_H_

#ifndef MYINLINE
#    ifdef _MSC_VER
#        ifdef __cplusplus
#            define MYINLINE __inline
#            define FORCEINLINE __forceinline
#        else
#            define MYINLINE _inline
#            define FORCEINLINE MYINLINE
#        endif
#    else
#        define MYINLINE inline
#        define FORCEINLINE MYINLINE
#    endif
#endif /* MYINLINE */

#ifdef WIN32
#    define GTK_CB __declspec(dllexport)
#else
#    define GTK_CB
#endif

#ifdef WIN32
/* XXX This is from Win32's <windef.h> */
#ifndef APIENTRY
#define GLUT_APIENTRY_DEFINED
#if (_MSC_VER >= 800) || defined(_STDCALL_SUPPORTED)
#define APIENTRY    __stdcall
#else
#define APIENTRY
#endif
#endif
/* XXX This is from Win32's <winnt.h> */
#ifndef CALLBACK
#if (defined(_M_MRX000) || defined(_M_IX86) || defined(_M_ALPHA) || defined(_M_PPC)) && !defined(MIDL_PASS)
#define CALLBACK __stdcall
#else
#define CALLBACK
#endif
#endif
/* XXX This is from Win32's <wingdi.h> and <winnt.h> */
#ifndef WINGDIAPI
#define GLUT_WINGDIAPI_DEFINED
#define WINGDIAPI __declspec(dllimport)
#endif
/* XXX This is from Win32's <ctype.h> */
#if !defined( _WCHAR_T_DEFINED) & !defined(MINGW)
typedef unsigned short wchar_t;
#define _WCHAR_T_DEFINED
#endif
#endif /* WIN32 */

#ifndef NULL
#define NULL 0
#endif

#if !defined(_WINDEF_) & !defined(_WINDEF_H)

#define CONST               const
typedef unsigned long       DWORD;
typedef int                 BOOL;
typedef char                CHAR;
typedef unsigned char       BYTE;
typedef unsigned short      WORD;
typedef float               FLOAT;
typedef double              DOUBLE;
typedef int                 INT;
typedef unsigned int        UINT;
typedef char*               SZ;
typedef void                VOID;
typedef long                LONG;
#ifdef _MSC_VER
typedef __int64             LONGLONG;
#else
typedef long long           LONGLONG;
#endif

typedef struct _LARGE_INTEGER {
    LONGLONG QuadPart;
} LARGE_INTEGER;

typedef struct tagRECT
{
    LONG    left;
    LONG    top;
    LONG    right;
    LONG    bottom;
} RECT, *PRECT;

#ifndef TRUE
#  define TRUE                1
#  define FALSE               0
#endif /* TRUE */

#endif /* _WINDEF_ */

typedef char*               SZ;

#define ROK                 0x00000000
#define RERR                0x10000000

#define OK(x)               (!(x & RERR))
#define ERR(x)              (x & RERR)

#ifndef SAFE_DELETE
#  define SAFE_DELETE(p)       { if(p) { delete (p);     (p)=NULL; } }
#  define SAFE_DELETE_ARRAY(p) { if(p) { delete[] (p);   (p)=NULL; } }
#  define SAFE_RELEASE(p)      { if(p) { (p)->Release(); (p)=NULL; } }
#endif /* SAFE_DELETE */

#ifdef WIN32
#define strcasecmp stricmp
#endif

#if !defined(REAL) | defined(REAL_IS_DOUBLE)
#undef REAL
#define REAL DOUBLE
#ifndef REAL_IS_DOUBLE
#define REAL_IS_DOUBLE
#endif
#endif

#endif
