#ifndef __EFFECTS_H__
#define __EFFECTS_H__

#include "Tools.h"

/*extern void effectIcosphereInit(void);
extern void effectIcosphereRender(C3D_RenderTarget* targetLeft, C3D_RenderTarget* targetRight, float iod, float time, float escalate);
extern void effectIcosphereExit(void);

extern void effectMetaballsInit();
extern void effectMetaballsRender(C3D_RenderTarget* targetLeft, C3D_RenderTarget* targetRight, float iod, float time, float escalate);
extern void effectMetaballsExit(void);

extern void effectNordlichtInit();
extern void effectNordlichtRender(C3D_RenderTarget* targetLeft, C3D_RenderTarget* targetRight, float iod, float time, float escalate);
extern void effectNordlichtExit(void);

extern void effectTunnelInit();
extern void effectTunnelRender(C3D_RenderTarget* targetLeft, C3D_RenderTarget* targetRight, float iod, float time, float escalate);
extern void effectTunnelExit(void);

extern void effectGreetsInit();
extern void effectGreetsRender(C3D_RenderTarget* targetLeft, C3D_RenderTarget* targetRight, float iod, float time, float escalate);
extern void effectGreetsExit(void);*/

extern void effectTunnelInit();
extern void effectTunnelRender(C3D_RenderTarget* targetLeft, C3D_RenderTarget* targetRight, float iod, float time, float escalate);
extern void effectTunnelExit(void);

extern void effectScrollerInit();
extern void effectScrollerRender(C3D_RenderTarget* targetLeft, C3D_RenderTarget* targetRight, float iod, float time, float escalate);
extern void effectScrollerExit(void);

#endif