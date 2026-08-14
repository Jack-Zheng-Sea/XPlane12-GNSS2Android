// XP12GNSS.cpp - X-Plane 12 plugin "GNSS to Android".
//
// Reads the user aircraft state directly from X-Plane 12 datarefs (via the
// XPLM SDK) and pushes it over UDP to the Android app from the
// MSFS-SimGPStoAndroid project, which then injects the position as an Android
// mock GPS location. The wire format is identical to the MSFS PC server so the
// existing Android APK works unchanged:
//
//   frame = "%.6f,%.6f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f\n"
//           lat, lon, altitude_ft, heading, pitch, roll, groundSpeed_mps, airSpeed_mps
//
// Configuration datarefs (editable at runtime with DataRefTool / X-Plane):
//   sim/GNSS2Android/udp_port             int  (default 36666)
//   sim/GNSS2Android/refresh_interval_ms  int  (default 50)
//   sim/GNSS2Android/enabled              int  (default 1)
// Values can also be preset with an INI file next to the plugin.

#include "GpsUdpServer.h"   // pulls in winsock2.h before XPLM headers
#include "XP12GNSS.h"

#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>

// ---------------------------------------------------------------------------
// Configuration (mirrors the MSFS PC server defaults)
// ---------------------------------------------------------------------------
static int g_cfgPort            = 36666;   // UDP port for the Android app
static int g_cfgIntervalMs      = 50;      // PC-side data refresh interval
static int g_cfgEnabled         = 1;       // 1 = broadcast to phones
static int g_lastStartedPort    = -1;      // port currently bound by the server

static XPLMDataRef g_cfgPortRef    = NULL;
static XPLMDataRef g_cfgIntervalRef = NULL;
static XPLMDataRef g_cfgEnabledRef = NULL;

// ---------------------------------------------------------------------------
// Aircraft datarefs (looked up once at plugin start, read every loop)
// ---------------------------------------------------------------------------
static XPLMDataRef gRefLat  = NULL;   // double, degrees
static XPLMDataRef gRefLon  = NULL;   // double, degrees
static XPLMDataRef gRefElev = NULL;   // double, meters MSL  -> converted to ft
static XPLMDataRef gRefPsi  = NULL;   // float, true heading (fallback)
static XPLMDataRef gRefTruePsi = NULL; // float, true heading (XP11+)
static XPLMDataRef gRefTheta = NULL;  // float, pitch  (deg, nose up = +)
static XPLMDataRef gRefPhi   = NULL;  // float, roll   (deg, right wing down = +)
static XPLMDataRef gRefGS    = NULL;  // float, ground speed m/s
static XPLMDataRef gRefIAS   = NULL;  // float, indicated airspeed m/s

static GpsUdpServer g_server;
static long long    g_lastSendMs = 0;

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
static long long NowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static void LogLine(const std::string& text)
{
    std::string msg = "XP12-GNSS2Android: " + text + "\n";
    XPLMDebugString(msg.c_str());
}

// ---------------------------------------------------------------------------
// Config dataref accessors
// ---------------------------------------------------------------------------
static int CfgGetInt(void* inRefcon) { return *static_cast<int*>(inRefcon); }
static void CfgSetInt(void* inRefcon, int inValue) { *static_cast<int*>(inRefcon) = inValue; }

static void RegisterConfigDataRefs()
{
    g_cfgPortRef    = XPLMRegisterDataAccessor("sim/GNSS2Android/udp_port",
        xplmType_Int, 1, CfgGetInt, CfgSetInt,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        &g_cfgPort, &g_cfgPort);
    g_cfgIntervalRef = XPLMRegisterDataAccessor("sim/GNSS2Android/refresh_interval_ms",
        xplmType_Int, 1, CfgGetInt, CfgSetInt,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        &g_cfgIntervalMs, &g_cfgIntervalMs);
    g_cfgEnabledRef = XPLMRegisterDataAccessor("sim/GNSS2Android/enabled",
        xplmType_Int, 1, CfgGetInt, CfgSetInt,
        NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
        &g_cfgEnabled, &g_cfgEnabled);
}

static void UnregisterConfigDataRefs()
{
    if (g_cfgPortRef)    { XPLMUnregisterDataAccessor(g_cfgPortRef);   g_cfgPortRef    = NULL; }
    if (g_cfgIntervalRef){ XPLMUnregisterDataAccessor(g_cfgIntervalRef); g_cfgIntervalRef = NULL; }
    if (g_cfgEnabledRef) { XPLMUnregisterDataAccessor(g_cfgEnabledRef); g_cfgEnabledRef = NULL; }
}

// ---------------------------------------------------------------------------
// INI config (optional file next to the plugin, e.g. "XP12-GNSS2Android.ini")
// ---------------------------------------------------------------------------
static std::string ConfigDir()
{
    char path[512] = { 0 };
    XPLMGetPluginInfo(XPLMGetMyID(), NULL, path, NULL, NULL);
    std::string dir = path;
    // Strip everything up to and including the last path separator.
    size_t slash = dir.find_last_of("/\\");
    if (slash != std::string::npos)
        dir.erase(slash + 1);
    return dir;
}

static void ReadConfigFile()
{
    std::string ini = ConfigDir() + "XP12-GNSS2Android.ini";
    FILE* f = std::fopen(ini.c_str(), "r");
    if (!f)
        return;
    char line[256];
    while (std::fgets(line, sizeof(line), f))
    {
        char key[128], value[128];
        if (std::sscanf(line, "%127[^=]=%127s", key, value) != 2)
            continue;
        if (std::strcmp(key, "udp_port") == 0)
        {
            int v = std::atoi(value);
            if (v > 0 && v <= 65535) g_cfgPort = v;
        }
        else if (std::strcmp(key, "refresh_interval_ms") == 0)
        {
            int v = std::atoi(value);
            if (v >= 5) g_cfgIntervalMs = (v > 5000) ? 5000 : v;
        }
        else if (std::strcmp(key, "enabled") == 0)
        {
            g_cfgEnabled = (std::atoi(value) != 0) ? 1 : 0;
        }
    }
    std::fclose(f);
    LogLine("Read config from " + ini);
}

// ---------------------------------------------------------------------------
// Dataref lookup
// ---------------------------------------------------------------------------
static void FindAircraftDataRefs()
{
    gRefLat     = XPLMFindDataRef("sim/flightmodel/position/latitude");
    gRefLon     = XPLMFindDataRef("sim/flightmodel/position/longitude");
    gRefElev    = XPLMFindDataRef("sim/flightmodel/position/elevation");  // meters MSL
    gRefTruePsi = XPLMFindDataRef("sim/flightmodel/position/true_psi");
    gRefPsi     = XPLMFindDataRef("sim/flightmodel/position/psi");        // fallback
    gRefTheta   = XPLMFindDataRef("sim/flightmodel/position/theta");
    gRefPhi     = XPLMFindDataRef("sim/flightmodel/position/phi");
    gRefGS      = XPLMFindDataRef("sim/flightmodel/position/groundspeed");  // m/s
    gRefIAS     = XPLMFindDataRef("sim/flightmodel/position/indicated_airspeed"); // m/s

    if (!gRefLat || !gRefLon || !gRefElev || !gRefGS || !gRefIAS || !gRefTheta || !gRefPhi)
        LogLine("WARNING: not all datarefs found. Is X-Plane 12 running?");
}

// ---------------------------------------------------------------------------
// Read state and broadcast one frame
// ---------------------------------------------------------------------------
static void ReadAndBroadcast()
{
    if (!gRefLat || !gRefLon || !gRefElev)
        return;

    double lat  = XPLMGetDatad(gRefLat);
    double lon  = XPLMGetDatad(gRefLon);
    double altFt = XPLMGetDatad(gRefElev) * 3.28083989501;  // meters -> feet

    // True heading: prefer true_psi (XP11+), fall back to psi.
    float heading = gRefTruePsi ? XPLMGetDataf(gRefTruePsi) : 0.0f;
    if (!gRefTruePsi && gRefPsi) heading = XPLMGetDataf(gRefPsi);
    float pitch = gRefTheta ? XPLMGetDataf(gRefTheta) : 0.0f;
    float roll  = gRefPhi   ? XPLMGetDataf(gRefPhi)   : 0.0f;
    float gs    = gRefGS    ? XPLMGetDataf(gRefGS)    : 0.0f;
    float ias   = gRefIAS   ? XPLMGetDataf(gRefIAS)   : 0.0f;

    char msg[256];
    int written = std::snprintf(msg, sizeof(msg),
        "%.6f,%.6f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
        lat, lon, altFt, heading, pitch, roll, gs, ias);
    if (written > 0)
        g_server.broadcast(msg);
}

// ---------------------------------------------------------------------------
// Flight loop
// ---------------------------------------------------------------------------
static float FlightLoop(float inElapsedSinceLastCall,
                        float inElapsedTimeSinceLastFlightLoop,
                        int   inCounter,
                        void* inRefcon)
{
    // Re-bind automatically if the port config changed (thread-safe enough:
    // stop() joins the old thread; start() opens the new listener).
    if (g_lastStartedPort != g_cfgPort)
    {
        if (g_cfgPort >= 1 && g_cfgPort <= 65535)
        {
            g_server.start(static_cast<uint16_t>(g_cfgPort));
            g_lastStartedPort = g_cfgPort;
            LogLine("UDP server listening on port " + std::to_string(g_cfgPort));
        }
    }

    if (g_cfgEnabled != 0)
    {
        long long now = NowMs();
        if (now - g_lastSendMs >= g_cfgIntervalMs)
        {
            g_lastSendMs = now;
            if (g_server.clientCount() > 0)
                ReadAndBroadcast();
        }
    }
    return -1.0f; // call again every flight loop
}

// ---------------------------------------------------------------------------
// XPLM entry points
// ---------------------------------------------------------------------------
PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc)
{
    std::strcpy(outName, "X-Plane 12 GNSS to Android");
    std::strcpy(outSig, "mcp.gnss2android.xp12");
    std::strcpy(outDesc, "Broadcast X-Plane 12 position/speed/heading/altitude "
                         "to an Android phone for mock GPS injection.\n");

#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

    ReadConfigFile();
    FindAircraftDataRefs();
    RegisterConfigDataRefs();

    LogLine("Plugin loaded (udp_port=" + std::to_string(g_cfgPort) +
            ", interval=" + std::to_string(g_cfgIntervalMs) +
            ", enabled=" + std::to_string(g_cfgEnabled) + ")");
    return 1;
}

PLUGIN_API int XPluginEnable(void)
{
    g_lastStartedPort = g_cfgPort;
    g_server.start(static_cast<uint16_t>(g_cfgPort));
    LogLine("UDP server listening on port " + std::to_string(g_cfgPort));

    XPLMRegisterFlightLoopCallback(FlightLoop, -1.0f, NULL);
    return 1;
}

PLUGIN_API void XPluginDisable(void)
{
    XPLMUnregisterFlightLoopCallback(FlightLoop, NULL);
    g_server.stop();
    g_lastStartedPort = -1;
    LogLine("Plugin disabled, UDP server stopped");
}

PLUGIN_API void XPluginStop(void)
{
    g_server.stop();
    UnregisterConfigDataRefs();
#ifdef _WIN32
    WSACleanup();
#endif
    LogLine("Plugin stopped");
}

PLUGIN_API void XPluginReceiveMessage(int, int, void*)
{
    // Nothing to do.
}