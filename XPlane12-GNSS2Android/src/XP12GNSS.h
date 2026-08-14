// XP12GNSS.h - X-Plane 12 plugin: read aircraft state and broadcast it to an
// Android phone so it can inject the position as a mock GPS location.
//
// Entry points exported to the XPLM (X-Plane plugin manager).
#ifndef XP12_GNSS_H
#define XP12_GNSS_H

#include "XPLMDefs.h"

#ifdef __cplusplus
extern "C" {
#endif

PLUGIN_API int  XPluginStart(char* outName, char* outSig, char* outDesc);
PLUGIN_API void XPluginStop(void);
PLUGIN_API void XPluginDisable(void);
PLUGIN_API int  XPluginEnable(void);
PLUGIN_API void XPluginReceiveMessage(int inFromWho, int inMessage, void* inParam);

#ifdef __cplusplus
}
#endif

#endif // XP12_GNSS_H
