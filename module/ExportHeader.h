#ifndef _EXPORT

#if (WIN32)
#define _EXPORT __declspec(dllexport)
#else
#define _EXPORT
#endif

#endif
