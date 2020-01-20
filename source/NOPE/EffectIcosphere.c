#include "Effects.h"

#include "vshader_shbin.h"
#include "svatg_bin.h"
#include "scroller_bin.h"
#include "Icosphere.h"

#define RING_MAX_VERTS 1000

#include "Font.h"
#include "MonoFont.h"

static DVLB_s* vshader_dvlb;
static shaderProgram_s program;

static int uLoc_projection, uLoc_modelView;
static C3D_Mtx projection;

static C3D_Tex sphere_tex;
static C3D_LightEnv lightEnv;
static C3D_Light light;
static C3D_LightLut lut_Phong;
static C3D_LightLut lut_shittyFresnel;

static Pixel* screenPixels;
static Bitmap screen;
static C3D_Tex screen_tex;
static C3D_Tex logo_tex;

static Pixel* scrollPixels;
static Bitmap scroller;
static C3D_Tex scroll_tex;

static vertex* vboVertsSphere;
static vertex* vboVertsRing;

extern Font OL16Font; 

#define SCROLLERTEXT "                                          hello nordlicht! welcome to our first prod on the 3DS! what a weird cross between a modern gpu and hardware straight from the 90s this is. in any case, we hope you will continue to enjoy wonderful SVatG products in the future. love, halcy."

void effectIcosphereInit(void) {
    // Load the vertex shader, create a shader program and bind it
    vshader_dvlb = DVLB_ParseFile((u32*)vshader_shbin, vshader_shbin_size);
    shaderProgramInit(&program);
    shaderProgramSetVsh(&program, &vshader_dvlb->DVLE[0]);

    C3D_TexInit(&sphere_tex, 256, 256, GPU_RGBA8);
    C3D_TexInit(&screen_tex, SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT, GPU_RGBA8);
    C3D_TexInit(&scroll_tex, 512, 512, GPU_RGBA8);
    
    screenPixels = (Pixel*)linearAlloc(SCREEN_TEXTURE_WIDTH * SCREEN_TEXTURE_HEIGHT * sizeof(Pixel));
    InitialiseBitmap(&screen, SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT, BytesPerRowForWidth(SCREEN_TEXTURE_WIDTH), screenPixels);
    
    scrollPixels = (Pixel*)linearAlloc(512 * 512 * sizeof(Pixel));
    InitialiseBitmap(&scroller, 512, 512, BytesPerRowForWidth(512), scrollPixels);
    
    vboVertsSphere = (vertex*)linearAlloc(sizeof(vertex) * icosphereNumFaces * 3);
    vboVertsRing = (vertex*)linearAlloc(sizeof(vertex) * RING_MAX_VERTS);
    
    C3D_TexInit(&logo_tex, SCREEN_TEXTURE_HEIGHT, SCREEN_TEXTURE_WIDTH, GPU_RGBA8);
    C3D_TexUpload(&logo_tex, scroller_bin);
    C3D_TexSetFilter(&logo_tex, GPU_LINEAR, GPU_NEAREST);
}

static void renderSphere(float iod, float time, float escalate) {
    C3D_BindProgram(&program);

    // Get the location of the uniforms
    uLoc_projection = shaderInstanceGetUniformLocation(program.vertexShader, "projection");
    uLoc_modelView = shaderInstanceGetUniformLocation(program.vertexShader, "modelView");

    // Configure attributes for use with the vertex shader
    C3D_AttrInfo* attrInfo = C3D_GetAttrInfo();
    AttrInfo_Init(attrInfo);
    AttrInfo_AddLoader(attrInfo, 0, GPU_FLOAT, 3); // v0=position
    AttrInfo_AddLoader(attrInfo, 1, GPU_FLOAT, 2); // v1=texcoord
    AttrInfo_AddLoader(attrInfo, 2, GPU_FLOAT, 3); // v2=normal

    // Compute the projection matrix
    Mtx_PerspStereoTilt(&projection, 65.0f*M_PI/180.0f, 400.0f/240.0f, 0.01f, 1000.0f, iod, 2.0f);
    
    int32_t* normalIndices = (int32_t*)malloc(sizeof(int32_t) * icosphereNumVertices);
    for(int i = 0; i < icosphereNumVertices; i++) {
        normalIndices[i] = -1;
    }
    for(int f = 0; f < icosphereNumFaces; f++) {
        for(int v = 0; v < 3; v++) {
            // Set up vertex
            uint32_t vertexIndex = icosphereFaces[f].v[v];
            vboVertsSphere[f * 3 + v].position[0] = icosphereVertices[vertexIndex].x;
            vboVertsSphere[f * 3 + v].position[1] = icosphereVertices[vertexIndex].y;
            vboVertsSphere[f * 3 + v].position[2] = icosphereVertices[vertexIndex].z;
            
            // Set normal to face normal
            vboVertsSphere[f * 3 + v].normal[0] = icosphereNormals[icosphereFaces[f].v[3]].x;
            vboVertsSphere[f * 3 + v].normal[1] = icosphereNormals[icosphereFaces[f].v[3]].y;
            vboVertsSphere[f * 3 + v].normal[2] = icosphereNormals[icosphereFaces[f].v[3]].z;
            
            // Displace
            float shiftVal = noise_at(
                (2.0 + icosphereVertices[vertexIndex].x) * 4.0, 
                (1.0 + icosphereVertices[vertexIndex].y) * 4.0, 
                (2.0 + icosphereVertices[vertexIndex].z) * 4.0 + time * 0.005
            ) / 6.0;
            shiftVal = fmax(shiftVal, 0.0) * escalate;
            
            int32_t normalIndex = icosphereFaces[f].v[3];
            if(normalIndices[vertexIndex] == -1) {
                normalIndices[vertexIndex] = normalIndex;
            }
            else {
                normalIndex = normalIndices[vertexIndex];
            }
            
            vboVertsSphere[f * 3 + v].position[0] += shiftVal * icosphereNormals[normalIndex].x;
            vboVertsSphere[f * 3 + v].position[1] += shiftVal * icosphereNormals[normalIndex].y;
            vboVertsSphere[f * 3 + v].position[2] += shiftVal * icosphereNormals[normalIndex].z;
            
            // Set texcoords
            vboVertsSphere[f * 3 + v].texcoord[0] = v & 1;
            vboVertsSphere[f * 3 + v].texcoord[1] = (v >> 1) & 1;
        }
    }
    free(normalIndices);
    
    // Recompute normals
    for(int f = 0; f < icosphereNumFaces; f++) {
        vec3_t a = vec3(vboVertsSphere[f * 3 + 0].position[0], vboVertsSphere[f * 3 + 0].position[1], vboVertsSphere[f * 3 + 0].position[2]);
        vec3_t b = vec3(vboVertsSphere[f * 3 + 1].position[0], vboVertsSphere[f * 3 + 1].position[1], vboVertsSphere[f * 3 + 1].position[2]);
        vec3_t c = vec3(vboVertsSphere[f * 3 + 2].position[0], vboVertsSphere[f * 3 + 2].position[1], vboVertsSphere[f * 3 + 2].position[2]);
        vec3_t n = vec3norm(vec3cross(vec3norm(vec3sub(b, a)), vec3norm(vec3sub(c, a))));
        for(int v = 0; v < 3; v++) {
            vboVertsSphere[f * 3 + v].normal[0] = n.x;
            vboVertsSphere[f * 3 + v].normal[1] = n.y;
            vboVertsSphere[f * 3 + v].normal[2] = n.z;
        }
    }
    
    // Configure buffers
    C3D_BufInfo* bufInfo = C3D_GetBufInfo();
    BufInfo_Init(bufInfo);
    BufInfo_Add(bufInfo, vboVertsSphere, sizeof(vertex), 3, 0x210);

    // Load the texture and bind it to the first texture unit
    C3D_TexUpload(&sphere_tex, svatg_bin);
    C3D_TexSetFilter(&sphere_tex, GPU_LINEAR, GPU_NEAREST);
    C3D_TexBind(0, &sphere_tex);
    
    // Calculate the modelView matrix
    C3D_Mtx modelView;
    Mtx_Identity(&modelView);
    Mtx_Translate(&modelView, 0.0, 0.0, -1.5);
    Mtx_RotateX(&modelView, time * 0.004, true);
    Mtx_RotateY(&modelView, time * 0.002, true);

    // Update the uniforms
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_projection, &projection);
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_modelView,  &modelView);

    C3D_TexEnv* env = C3D_GetTexEnv(0);
    C3D_TexEnvSrc(env, C3D_RGB, GPU_FRAGMENT_PRIMARY_COLOR, GPU_FRAGMENT_SECONDARY_COLOR, 0);
    C3D_TexEnvOp(env, C3D_RGB, 0, 0, 0);
    C3D_TexEnvFunc(env, C3D_RGB, GPU_ADD);

    C3D_TexEnv* env2 = C3D_GetTexEnv(1);
    C3D_TexEnvSrc(env2, C3D_RGB, GPU_TEXTURE0, GPU_PREVIOUS, 0);
    C3D_TexEnvOp(env2, C3D_RGB, 0, 0, 0);
    C3D_TexEnvFunc(env2, C3D_RGB, GPU_MODULATE);
    
    C3D_TexEnv* env3 = C3D_GetTexEnv(2);
    C3D_TexEnvSrc(env3, C3D_RGB, GPU_FRAGMENT_SECONDARY_COLOR, GPU_PREVIOUS, 0);
    C3D_TexEnvOp(env3, C3D_RGB, GPU_TEVOP_RGB_SRC_ALPHA , 0, 0);
    C3D_TexEnvFunc(env3, C3D_RGB, GPU_ADD);
    
    static const C3D_Material lightMaterial =
    {
        { 0.2f, 0.0f, 0.0f }, //ambient
        { 0.4f, 0.4f, 0.4f }, //diffuse
        { 0.8f, 0.8f, 0.8f }, //specular0
        { 0.0f, 0.0f, 0.0f }, //specular1
        { 0.0f, 0.0f, 0.0f }, //emission
    };

    C3D_LightEnvInit(&lightEnv);
    C3D_LightEnvBind(&lightEnv);
    C3D_LightEnvMaterial(&lightEnv, &lightMaterial);

    LightLut_Phong(&lut_Phong, 3.0);
    C3D_LightEnvLut(&lightEnv, GPU_LUT_D0, GPU_LUTINPUT_LN, false, &lut_Phong);
    
    LightLut_FromFunc(&lut_shittyFresnel, badFresnel, 0.6, false);
    C3D_LightEnvLut(&lightEnv, GPU_LUT_FR, GPU_LUTINPUT_NV, false, &lut_shittyFresnel);
    C3D_LightEnvFresnel(&lightEnv, GPU_PRI_SEC_ALPHA_FRESNEL);
    C3D_FVec lightVec = { { 0.0, 0.0, 0.5, 0.0 } };

    C3D_LightInit(&light, &lightEnv);
    C3D_LightColor(&light, 1.0, 1.0, 1.0);
    C3D_LightPosition(&light, &lightVec);
    
    // Depth test is back
    C3D_DepthTest(true, GPU_GEQUAL, GPU_WRITE_ALL);
    
    // To heck with culling
    //C3D_CullFace(GPU_CULL_NONE);
    
    // Draw the VBO
    C3D_DrawArrays(GPU_TRIANGLES, 0, icosphereNumFaces * 3);
    
    C3D_LightEnvBind(0);
    //TexEnv_Init(&env);
    //TexEnv_Init(&env2);
    //TexEnv_Init(&env3);
}

static void renderRing(float iod, float time, float escalate) {
    C3D_BindProgram(&program);

    // Get the location of the uniforms
    uLoc_projection = shaderInstanceGetUniformLocation(program.vertexShader, "projection");
    uLoc_modelView = shaderInstanceGetUniformLocation(program.vertexShader, "modelView");

    // Configure attributes for use with the vertex shader
    C3D_AttrInfo* attrInfo = C3D_GetAttrInfo();
    AttrInfo_Init(attrInfo);
    AttrInfo_AddLoader(attrInfo, 0, GPU_FLOAT, 3); // v0=position
    AttrInfo_AddLoader(attrInfo, 1, GPU_FLOAT, 2); // v1=texcoord
    AttrInfo_AddLoader(attrInfo, 2, GPU_FLOAT, 3); // v2=normal

    // Compute the projection matrix
    Mtx_PerspStereoTilt(&projection, 65.0f*M_PI/180.0f, 400.0f/240.0f, 0.01f, 1000.0f, iod, 2.0f);
    
    int ringVertCount = 0;
    float ringSects = 30;
    for(float sect = 0.0; sect < ringSects; sect += 1.0) {
        float ringAngleStartSin = sin((sect / ringSects) * 3.1415 * 2.0);
        float ringAngleStartCos = cos((sect / ringSects) * 3.1415 * 2.0);
        float ringAngleStopSin = sin(((sect + 1.0) / ringSects) * 3.1415 * 2.0);
        float ringAngleStopCos = cos(((sect + 1.0) / ringSects) * 3.1415 * 2.0);
        ringVertCount += buildQuad(
            &(vboVertsRing[ringVertCount]),
            vec3mul(vec3(ringAngleStartSin, 0.0, ringAngleStartCos), 0.7),
            vec3mul(vec3(ringAngleStopSin, 0.0, ringAngleStopCos), 0.7),
            vec3mul(vec3(ringAngleStopSin, 0.2, ringAngleStopCos), 0.7),
            vec3mul(vec3(ringAngleStartSin, 0.2, ringAngleStartCos), 0.7),
            vec2(0.03125 * (sect), 1.0 - 0.03125),
            vec2(0.03125 * (1 + sect), 1.0 - 0.03125),
            vec2(0.03125 * (sect), 1.0),
            vec2(0.03125 * (1 + sect), 1.0)
        );
    }
    
    // Recompute normals
    for(int f = 0; f < ringVertCount; f++) {
        vec3_t a = vec3(vboVertsRing[f * 3 + 0].position[0], vboVertsRing[f * 3 + 0].position[1], vboVertsRing[f * 3 + 0].position[2]);
        vec3_t b = vec3(vboVertsRing[f * 3 + 1].position[0], vboVertsRing[f * 3 + 1].position[1], vboVertsRing[f * 3 + 1].position[2]);
        vec3_t c = vec3(vboVertsRing[f * 3 + 2].position[0], vboVertsRing[f * 3 + 2].position[1], vboVertsRing[f * 3 + 2].position[2]);
        vec3_t n = vec3norm(vec3cross(vec3norm(vec3sub(b, a)), vec3norm(vec3sub(c, a))));
        for(int v = 0; v < 3; v++) {
            vboVertsRing[f * 3 + v].normal[0] = n.x;
            vboVertsRing[f * 3 + v].normal[1] = n.y;
            vboVertsRing[f * 3 + v].normal[2] = n.z;
        }
    }
    
    // Configure buffers
    C3D_BufInfo* bufInfo = C3D_GetBufInfo();
    BufInfo_Init(bufInfo);
    BufInfo_Add(bufInfo, vboVertsRing, sizeof(vertex), 3, 0x210);

    // Load the texture and bind it to the first texture unit
    C3D_TexBind(0, &scroll_tex);
    C3D_TexSetFilter(&scroll_tex, GPU_NEAREST, GPU_NEAREST);
    
    // Calculate the modelView matrix
    C3D_Mtx modelView;
    Mtx_Identity(&modelView);
    Mtx_Translate(&modelView, 0.0, 0.0, -1.5);
    Mtx_RotateX(&modelView, -0.3, true);
    Mtx_RotateZ(&modelView, -0.3, true);
    Mtx_RotateY(&modelView, -3.14, true);
    
    // Update the uniforms
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_projection, &projection);
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_modelView,  &modelView);

    C3D_TexEnv* env = C3D_GetTexEnv(0);
    C3D_TexEnvSrc(env, C3D_Both, GPU_TEXTURE0, 0, 0);
    C3D_TexEnvOp(env, C3D_Both, 0, 0, 0);
    C3D_TexEnvFunc(env, C3D_Both, GPU_REPLACE);
    
    C3D_LightEnvBind(0);
    
    // Depth test is back
    C3D_DepthTest(true, GPU_GEQUAL, GPU_WRITE_ALL);
    
    // To heck with culling
    C3D_CullFace(GPU_CULL_NONE);
    
    // Draw the VBO
    C3D_DrawArrays(GPU_TRIANGLES, 0, ringVertCount);
}

void effectIcosphereRenderSingle(float iod, float time, float escalate) {
    renderSphere(iod, time, escalate);
    renderRing(iod, time, escalate);
}

void effectIcosphereRender(C3D_RenderTarget* targetLeft, C3D_RenderTarget* targetRight, float iod, float time, float escalate) {
    // Render some 2D stuff
    float lobes = 3.0;
    for(int x = 0; x < SCREEN_WIDTH; x += 10) {
        for(int y = 0; y < SCREEN_HEIGHT; y += 10) {
            float posX = ((float)x / (float)SCREEN_WIDTH) - 0.5;
            float posY = ((float)y / (float)SCREEN_HEIGHT) - 0.5;
            float posR = sqrt(posX * posX + posY * posY);
            float posA = atan2(posX, posY);
            float dVal = sin(posA * lobes + time / 120.0) + cos(posX * sin(posY + time/400.0) * 5.0);
            
            float dvalAdd = fmin(0.8 - (posR - dVal), 0.9);
            Pixel colorPrimary = RGBAf(0.1 + dvalAdd * 0.1, 0.1 + dvalAdd * 0.2, 0.1 + dvalAdd * 0.4, 1.0);
            Pixel colorSecondary = RGBAf(dvalAdd * 0.05, dvalAdd * 0.2, dvalAdd * 0.3, 1.0);
            
            if((((x / 10) & 1) ^ ((y / 10) & 1)) == 0) {
                Pixel colorTemp = colorPrimary;
                colorPrimary = colorSecondary;
                colorSecondary = colorTemp;
            }
            DrawFilledRectangle(&screen, x, y, 10, 10, colorPrimary);
            DrawFilledRectangle(&screen, x + 2, y + 2, 6, 6, colorSecondary);
        }
    }
    
    GSPGPU_FlushDataCache(screenPixels, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(Pixel));
    GX_DisplayTransfer((u32*)screenPixels, GX_BUFFER_DIM(SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT), (u32*)screen_tex.data, GX_BUFFER_DIM(SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT), TEXTURE_TRANSFER_FLAGS);
    gspWaitForPPF();
    
    float sshift = -time * 0.57;
    FillBitmap(&scroller, RGBAf(0.0, 0.0, 0.0, 0.0));
    DrawSimpleString(&scroller, &OL16Font, sshift, 0, RGBAf(1.0, 1.0, 1.0, 1.0), SCROLLERTEXT);
    
    GSPGPU_FlushDataCache(scrollPixels, 512 * 512 * sizeof(Pixel));
    GX_DisplayTransfer((u32*)scrollPixels, GX_BUFFER_DIM(512, 512), (u32*)scroll_tex.data, GX_BUFFER_DIM(512, 512), TEXTURE_TRANSFER_FLAGS);
    gspWaitForPPF();
    
    // Left eye
    C3D_FrameDrawOn(targetLeft);
    
    // Fullscreen quad
    fullscreenQuad(screen_tex, -iod, 1.0 / 10.0);
    
    // Actual scene
    effectIcosphereRenderSingle(-iod, time, escalate);

    // Overlay
    fullscreenQuad(logo_tex, 0.0, 0.0);
    
    // Fade
    fade();
    
    if(iod > 0.0) {
        // Right eye
        C3D_FrameDrawOn(targetRight);
    
        // Fullscreen quad
        fullscreenQuad(screen_tex, iod, 1.0 / 10.0);
    
        // Actual scene
        effectIcosphereRenderSingle(iod, time, escalate);
        
        // Overlay
        fullscreenQuad(logo_tex, 0.0, 0.0);
        
        // Fade
        fade();
    }
}

void effectIcosphereExit(void) {
    // Free the texture
    C3D_TexDelete(&sphere_tex);
    C3D_TexDelete(&screen_tex);

    // Free the VBOs
    linearFree(vboVertsSphere);
    linearFree(vboVertsRing);
    
    // Free the shader program
    shaderProgramFree(&program);
    DVLB_Free(vshader_dvlb);
    
    // Free the screen bitmap
    linearFree(screenPixels);
    linearFree(scrollPixels);
}
