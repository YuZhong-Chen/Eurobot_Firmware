#ifndef _DEBUGMODE_H_
#define _DEBUGMODE_H_

// NOTE : Uncomment the define below to enable debugmode.
//#define DEBUGGER_MODE

#ifdef DEBUGGER_MODE

class DEBUGMODE {
public:
	void DebuggingMode();
};

extern DEBUGMODE DebugMode;

#endif

#endif
