#include "Tools.h"
#include "music_bin.h"

#include "Effects.h"

#define CLEAR_COLOR 0x555555FF

C3D_Tex fade_tex;
static Pixel* fadePixels;
static Bitmap fadeBitmap;
float fadeVal;

#define min(a, b) (((a)<(b))?(a):(b))
#define max(a, b) (((a)>(b))?(a):(b))

// void soundFill(void *audioBuffer,size_t offset, size_t size, int frequency ) {
//     u32 *dest = (u32*)audioBuffer;
// 
//     for (int i=0; i<size; i++) {
//         s16 sample = INT16_MAX * sin(frequency*(2*M_PI)*(offset+i)/SAMPLERATE);
//         dest[i] = (sample<<16) | (sample & 0xffff);
//     }
// 
//     DSP_FlushDataCache(audioBuffer,size);
// 
// }

int main() {
    bool DUMPFRAMES = false;
    
    // Initialize graphics
    gfxInit(GSP_RGBA8_OES, GSP_BGR8_OES, false);
    gfxSet3D(true);
    consoleInit(GFX_BOTTOM, NULL);
    C3D_Init(C3D_DEFAULT_CMDBUF_SIZE);

    // Initialize the render target
    C3D_RenderTarget* targetLeft = C3D_RenderTargetCreate(240, 400, GPU_RB_RGBA8, GPU_RB_DEPTH24_STENCIL8);
    C3D_RenderTarget* targetRight = C3D_RenderTargetCreate(240, 400, GPU_RB_RGBA8, GPU_RB_DEPTH24_STENCIL8);
    C3D_RenderTargetSetClear(targetLeft, C3D_CLEAR_ALL, CLEAR_COLOR, 0);
    C3D_RenderTargetSetClear(targetRight, C3D_CLEAR_ALL, CLEAR_COLOR, 0);
    C3D_RenderTargetSetOutput(targetLeft, GFX_TOP, GFX_LEFT,  DISPLAY_TRANSFER_FLAGS);
    C3D_RenderTargetSetOutput(targetRight, GFX_TOP, GFX_RIGHT, DISPLAY_TRANSFER_FLAGS);

    fadePixels = (Pixel*)linearAlloc(SCREEN_TEXTURE_WIDTH * SCREEN_TEXTURE_HEIGHT * sizeof(Pixel));
    InitialiseBitmap(&fadeBitmap, SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT, BytesPerRowForWidth(SCREEN_TEXTURE_WIDTH), fadePixels);
    C3D_TexInit(&fade_tex, SCREEN_TEXTURE_HEIGHT, SCREEN_TEXTURE_WIDTH, GPU_RGBA8);
    
    // Sound on
    // csndInit();
    ndspInit();
    
    ndspSetOutputMode(NDSP_OUTPUT_STEREO);

    ndspChnSetInterp(0, NDSP_INTERP_LINEAR);
    ndspChnSetRate(0, 32000);
    ndspChnSetFormat(0, NDSP_FORMAT_MONO_PCM16);
    
    float mix[12];
    memset(mix, 0, sizeof(mix));
    mix[0] = 1.0;
    mix[1] = 1.0;    
    ndspChnSetMix(0, mix);

    u32 *audioBuffer = (u32*)linearAlloc(music_bin_size);
    memcpy(audioBuffer, music_bin, music_bin_size);
    DSP_FlushDataCache(audioBuffer, music_bin_size);
    
    ndspWaveBuf waveBuf;    
    memset(&waveBuf,0, sizeof(ndspWaveBuf));
    waveBuf.data_vaddr = &audioBuffer[0];
    waveBuf.nsamples = music_bin_size / sizeof(uint16_t);
    
    // Play music
    ndspChnWaveBufAdd(0, &waveBuf);
    //csndPlaySound(0x8, SOUND_ONE_SHOT | SOUND_FORMAT_16BIT, 32000, 1.0, 0.0, (u32*)music_bin, NULL, music_bin_size);
    
    int64_t startTick = svcGetSystemTick();
    
    // Start up first effect
    //effectTunnelInit();
    effectScrollerInit();
    // effectMetaballsInit();
   // effectNordlichtInit();
    
    int currentSwitchTick = 0;
    int64_t switchTicks[8];
    
    float fudge = -17570.0;
    float songOrigSamples = 4056096.0;
    float songOrigRate = 48000.0;
    float songSeconds = songOrigSamples / songOrigRate;
    float halfSectionSeconds = songSeconds / 10.0;
    float microfudge = 450.0;
    float fadeTime = 0.3;
    float cpuHz = 268123480.0;
    float fadeTicks = fadeTime * cpuHz;

    int64_t tunnelFade = (800985.0 / songOrigRate) * cpuHz - fadeTicks;
    
    // Tunnel
    switchTicks[0] = ((826036.0 - microfudge) / songOrigRate) * cpuHz;
    
    // Iso
    switchTicks[1] = ((1226732 - microfudge)/ songOrigRate) * cpuHz;
    switchTicks[2] = ((1627428.0 - microfudge) / songOrigRate) * cpuHz;
    
    // Meta
    switchTicks[3] = ((2028124 - microfudge) / songOrigRate) * cpuHz;
    switchTicks[4] = ((2428821.0 - microfudge) / songOrigRate) * cpuHz;
    
    // Greet
    switchTicks[5] = ((3230215.0 - microfudge) / songOrigRate) * cpuHz;
    
    // Nordlicht
    switchTicks[6] = ((4088448.0 - microfudge) / songOrigRate) * cpuHz;
    
    // Nada
    switchTicks[7] = (halfSectionSeconds * 40.0 + fudge / songOrigRate) * cpuHz;
    
    int64_t finalFade = switchTicks[5] + 3 * (200348.0  / songOrigRate) * cpuHz;
    
    // Main loop
    int escalate = 0;
    int haveEscalated = 0;
    int curEscalate = escalate;
    float effectStart = 0.0;
    int fc = 0;
    int64_t fakeTick = startTick;
    
    //fakeTick = startTick + ((3230215.0 - microfudge) / songOrigRate) * cpuHz;
    currentSwitchTick = 0;
    escalate = 0;
    curEscalate = escalate;
    
    int64_t tickAdvance = (int64_t)((float)cpuHz / 60.0);
    
    while (aptMainLoop()) {
        int64_t currentTick;
        if(!DUMPFRAMES) {
            currentTick = svcGetSystemTick() - startTick;
        }
        else {
            currentTick = fakeTick - startTick;
        }
        fakeTick = fakeTick + tickAdvance;
        float time = currentTick * 0.000001;
            
        fadeVal = 0.0;
        /*for(int i = 2; i < 8; i++) {
            int64_t epsilon = switchTicks[i] - currentTick;
            if(epsilon > fadeTicks) {
                epsilon = fadeTicks;
            }
            if(epsilon < -fadeTicks) {
                epsilon = -fadeTicks;
            }
            fadeVal = max(fadeVal, 1.0 - (((float)abs(epsilon)) / ((float)fadeTicks)));
        }*/
        
        if(currentTick < cpuHz * 3) {
            fadeVal = 1.0 - ((float)(currentTick)) / ((float)cpuHz * 3);
        }
        
        if(currentTick > finalFade) {
            fadeVal = (float)(currentTick - finalFade) / ((200348.0  / songOrigRate) * cpuHz);
        }
        
        /*if(currentTick <= switchTicks[0] + fadeTicks / 6) {
            if(currentTick > tunnelFade) {
                fadeVal = (float)(currentTick - tunnelFade) / (float)fadeTicks;
                fadeVal = min(1.0, fadeVal);
            }
            FillBitmap(&fadeBitmap, RGBAf(0.0, 0.0, 0.0, fadeVal));
        }
        else {
            FillBitmap(&fadeBitmap, RGBAf(1.0, 1.0, 1.0, fadeVal));
        }
        */
        
        FillBitmap(&fadeBitmap, RGBAf(1.0, 1.0, 1.0, fadeVal));
        GSPGPU_FlushDataCache(fadePixels, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(Pixel));
        GX_DisplayTransfer((u32*)fadePixels, GX_BUFFER_DIM(SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT), (u32*)fade_tex.data, GX_BUFFER_DIM(SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT), TEXTURE_TRANSFER_FLAGS);
        gspWaitForPPF();
        
        hidScanInput();
        
        // Respond to user input
        u32 kDown = hidKeysDown();
        if (kDown & KEY_START) {
            break; // break in order to return to hbmenu
        }
        
//         //if (kDown & KEY_A && haveEscalated == 0) {
        /*if(currentTick > switchTicks[currentSwitchTick]) {
            currentSwitchTick += 1;
            escalate += 1;
            haveEscalated = 1;
            // printf("%lld\n", currentTick);
        }
        else {
            haveEscalated = 0;
        }
        
        // Init / Deinit
        if(escalate == 1 && haveEscalated) {
            curEscalate = escalate;
            effectStart = time;
            effectTunnelExit();
            effectScrollerInit();
        }*/
        
        /*if(escalate == 3 && haveEscalated) {
            curEscalate = escalate;
            effectIcosphereExit();
            effectMetaballsInit();
        }
        
        if(escalate == 5 && haveEscalated) {
            curEscalate = escalate;
            effectMetaballsExit();
            effectGreetsInit();
        }
        
        if(escalate == 6 && haveEscalated) {
            curEscalate = escalate;
            effectGreetsExit();
            effectNordlichtInit();
        }*/
//         
        float slider = osGet3DSliderState();
        float iod = slider / 3.0;
        
        // Render the scene
        //if(escalate < 1) {
        //    effectTunnelRender(targetLeft, targetRight, iod, time, escalate - curEscalate);
        //} else {
            effectScrollerRender(targetLeft, targetRight, iod, time - effectStart, escalate - curEscalate);
        //}
            /*} else if(escalate < 5) {
                effectMetaballsRender(targetLeft, targetRight, iod, time, escalate - curEscalate);
            } else if(escalate < 6) {
                effectGreetsRender(targetLeft, targetRight, iod, time, escalate - curEscalate);
            } else if(escalate < 7) {
                effectNordlichtRender(targetLeft, targetRight, iod, time, escalate - curEscalate);
            }
            else {
                break;
            }*/
//             effectMetaballsRender(targetLeft, targetRight, iod, time, 1);
        
        gspWaitForP3D();
        gspWaitForPPF();
        
        if(DUMPFRAMES) {
            u8* fbl = gfxGetFramebuffer(GFX_TOP, GFX_LEFT, NULL, NULL);
            //u8* fbr = gfxGetFramebuffer(GFX_TOP, GFX_RIGHT, NULL, NULL);
            
            char fname[255];
            sprintf(fname, "fb_left_%08d.raw", fc);
            
            FILE* file = fopen(fname,"w");
            fwrite(fbl, sizeof(int32_t), SCREEN_HEIGHT * SCREEN_WIDTH, file);
            fflush(file);
            fclose(file);
            
            /*sprintf(fname, "fb_right_%08d.raw", fc);            
            file = fopen(fname,"w");
            fwrite(fbr, sizeof(int32_t), SCREEN_HEIGHT * SCREEN_WIDTH, file);
            fflush(file);
            fclose(file);*/
        }
        
        fc++;   
    }
    
    // Clean up
    // effectNordlichtExit();
    //effectMetaballsExit();
    
    linearFree(fadePixels);
    
    // Sound off
    ndspExit();
    linearFree(audioBuffer);
    
    // Deinitialize graphics
    C3D_Fini();
    gfxExit();
    return 0;
}
