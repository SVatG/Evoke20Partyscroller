// Nordlicht demoparty
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "Tools.h"
#include "vshader_shbin.h"
#include "svatg3_bin.h"
#include "logo_bin.h"

#include "Lighthouse.h"

#define NORDLICHT_MAX_VERTS 20000

static DVLB_s* vshader_dvlb;
static shaderProgram_s program;

static int uLoc_projection, uLoc_modelView;
static C3D_Mtx projection;

static C3D_Tex sphere_tex;
static C3D_Tex logo_tex;

static C3D_LightEnv lightEnv;
static C3D_Light light;
static C3D_LightLut lut_Phong;
static C3D_LightLut lut_shittyFresnel;

int32_t vertCount;
static vertex* vboVerts;
static vertex* vboVertsLighthouse;
float* valueGrid;

float wave(float x) {
    return(x - (tanh(((x + 0.5) - floor(x + 0.5) - 0.5) * 8.0) / (2.0 * tanh(0.5 * 8.0)) + floor(x + 0.5) - 0.5));
}


static float waveTable[512];
float waveFromTable(float x) {
    return(waveTable[(int32_t)((x + 2000.0) * 512.0) % 512]);
}

// a -- b     .
// |\   |\    .
// d -- c \   .
//  \ e -\ f  .
//   \|   \|  .
//    h -- g  .
static int buildCube(vertex* vert, vec3_t cp, float r, int32_t s, float time, float hh) {
        // Texcoords
        vec2_t t1 = vec2(0, hh);
        vec2_t t2 = vec2(0, hh + 0.05);
        vec2_t t3 = vec2(1, hh);
        vec2_t t4 = vec2(1, hh + 0.05);
        
        srand(s);
        vec3_t axis = vec3norm(vec3(rand(), rand(), rand()));
        mat3x3_t rot = mat3x3rotate(s * 100.0 + time * 0.005, axis);
        
        // Corners
        vec3_t a = vec3add(cp, mat3x3transform(rot, vec3(-r, -r, -r)));
        vec3_t b = vec3add(cp, mat3x3transform(rot, vec3( r, -r, -r)));
        vec3_t c = vec3add(cp, mat3x3transform(rot, vec3( r, -r,  r)));
        vec3_t d = vec3add(cp, mat3x3transform(rot, vec3(-r, -r,  r)));
        vec3_t e = vec3add(cp, mat3x3transform(rot, vec3(-r,  r, -r)));
        vec3_t f = vec3add(cp, mat3x3transform(rot, vec3( r,  r, -r)));
        vec3_t g = vec3add(cp, mat3x3transform(rot, vec3( r,  r,  r)));
        vec3_t h = vec3add(cp, mat3x3transform(rot, vec3(-r,  r,  r)));
        
        // Faces
        vert += buildQuad(vert, a, b, c, d, t1, t2, t3, t4);
        vert += buildQuad(vert, d, c, g, h, t1, t2, t3, t4);
        vert += buildQuad(vert, h, g, f, e, t1, t2, t3, t4);
        vert += buildQuad(vert, e, f, b, a, t1, t2, t3, t4);
        vert += buildQuad(vert, b, f, g, c, t1, t2, t3, t4);
        vert += buildQuad(vert, e, a, d, h, t1, t2, t3, t4);
        
        // Done
        return(6 * 6);
}

// Für den echten Partybezug
void effectNordlichtInit() {
    // Load default shader
    vshader_dvlb = DVLB_ParseFile((u32*)vshader_shbin, vshader_shbin_size);
    shaderProgramInit(&program);
    shaderProgramSetVsh(&program, &vshader_dvlb->DVLE[0]);

    C3D_TexInit(&sphere_tex, 256, 256, GPU_RGBA8);
    C3D_TexInit(&logo_tex, SCREEN_TEXTURE_HEIGHT, SCREEN_TEXTURE_WIDTH, GPU_RGBA8);
    
    vboVerts = (vertex*)linearAlloc(sizeof(vertex) * NORDLICHT_MAX_VERTS);
    vboVertsLighthouse = (vertex*)linearAlloc(sizeof(vertex) * lighthouseNumFaces * 3);
    
    for(int f = 0; f < lighthouseNumFaces; f++) {
        for(int v = 0; v < 3; v++) {
            // Set up vertex
            uint32_t vertexIndex = lighthouseFaces[f].v[v];
            vboVertsLighthouse[f * 3 + v].position[0] = -lighthouseVertices[vertexIndex].z * 3.5;
            vboVertsLighthouse[f * 3 + v].position[1] = -lighthouseVertices[vertexIndex].x * 3.5;
            vboVertsLighthouse[f * 3 + v].position[2] = lighthouseVertices[vertexIndex].y * 3.5;
            
            // Set texcoords
            vboVertsLighthouse[f * 3 + v].texcoord[0] = v & 1;
            vboVertsLighthouse[f * 3 + v].texcoord[1] = (v >> 1) & 1;
        }
    }
    
    // Recompute normals
    for(int f = 0; f < lighthouseNumFaces; f++) {
        vec3_t a = vec3(vboVertsLighthouse[f * 3 + 0].position[0], vboVertsLighthouse[f * 3 + 0].position[1], vboVertsLighthouse[f * 3 + 0].position[2]);
        vec3_t b = vec3(vboVertsLighthouse[f * 3 + 1].position[0], vboVertsLighthouse[f * 3 + 1].position[1], vboVertsLighthouse[f * 3 + 1].position[2]);
        vec3_t c = vec3(vboVertsLighthouse[f * 3 + 2].position[0], vboVertsLighthouse[f * 3 + 2].position[1], vboVertsLighthouse[f * 3 + 2].position[2]);
        vec3_t n = vec3norm(vec3cross(vec3norm(vec3sub(b, a)), vec3norm(vec3sub(c, a))));
        for(int v = 0; v < 3; v++) {
            vboVertsLighthouse[f * 3 + v].normal[0] = n.x;
            vboVertsLighthouse[f * 3 + v].normal[1] = n.y;
            vboVertsLighthouse[f * 3 + v].normal[2] = n.z;
        }
    }
    
    for(int i = 0; i < 512; i++) {
        waveTable[i] = wave(((float)i) / 512.0);
    }
    
    // Load the texture and bind it to the first texture unit
    C3D_TexUpload(&sphere_tex, svatg3_bin);
    C3D_TexSetFilter(&sphere_tex, GPU_LINEAR, GPU_NEAREST);
    
    C3D_TexUpload(&logo_tex, logo_bin);
    C3D_TexSetFilter(&logo_tex, GPU_LINEAR, GPU_NEAREST);
}

void effectNordlichtUpdate(float time, float escalate) {
    vertCount = 0;
    for(int x = -10; x < 5; x++) {
        for(int y = -7; y < 5; y++) {
            float xx = x * 0.15;
            float yy = y * 0.15;
            float h = waveFromTable(x * 0.1 - time * 0.001) * 0.25;
            vertCount += buildCube(&(vboVerts[vertCount]), vec3(xx, yy, h), 0.08, (x * 1212) % 2311 + (y * 2133) % 881, time, h * 5.0);
        }
    }
    
    for(int f = 0; f < vertCount / 3; f++) {
        vec3_t a = vec3(vboVerts[f * 3 + 0].position[0], vboVerts[f * 3 + 0].position[1], vboVerts[f * 3 + 0].position[2]);
        vec3_t b = vec3(vboVerts[f * 3 + 1].position[0], vboVerts[f * 3 + 1].position[1], vboVerts[f * 3 + 1].position[2]);
        vec3_t c = vec3(vboVerts[f * 3 + 2].position[0], vboVerts[f * 3 + 2].position[1], vboVerts[f * 3 + 2].position[2]);
        vec3_t n = vec3norm(vec3cross(vec3norm(vec3sub(b, a)), vec3norm(vec3sub(c, a))));
        for(int v = 0; v < 3; v++) {
            vboVerts[f * 3 + v].normal[0] = n.x;
            vboVerts[f * 3 + v].normal[1] = n.y;
            vboVerts[f * 3 + v].normal[2] = n.z;
        }
    }
}

// Draw 
void effectNordlichtDraw(float iod, float time, float escalate) {
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

    // Load the texture and bind it to the first texture unit
    C3D_TexUpload(&sphere_tex, svatg3_bin);
    C3D_TexSetFilter(&sphere_tex, GPU_LINEAR, GPU_NEAREST);
    C3D_TexBind(0, &sphere_tex);
    
    // Calculate the modelView matrix
    C3D_Mtx modelView;
    Mtx_Identity(&modelView);
    Mtx_Translate(&modelView, 0.0, 0.0, -1.5);
    Mtx_RotateX(&modelView,  0.5, true);
    Mtx_RotateZ(&modelView, -0.3, true);
    //Mtx_RotateY(&modelView, time * 0.002, true);
    
    // Update the uniforms
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_projection, &projection);
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_modelView,  &modelView);

    C3D_TexEnv* env = C3D_GetTexEnv(0);
    C3D_TexEnvSrc(env, C3D_Both, GPU_TEXTURE0, GPU_FRAGMENT_PRIMARY_COLOR, 0);
    C3D_TexEnvOp(env, C3D_Both, 0, 0, 0);
    C3D_TexEnvFunc(env, C3D_Both, GPU_MODULATE);
    
    C3D_TexEnv* env2 = C3D_GetTexEnv(1);
    C3D_TexEnvSrc(env2, C3D_Alpha, GPU_TEXTURE0, 0, 0);
    C3D_TexEnvOp(env2, C3D_Alpha, 0, 0, 0);
    C3D_TexEnvFunc(env2, C3D_Alpha, GPU_REPLACE);
    
    C3D_TexEnv* env3 = C3D_GetTexEnv(2);
    C3D_TexEnvSrc(env3, C3D_RGB, GPU_FRAGMENT_SECONDARY_COLOR, GPU_PREVIOUS, 0);
    C3D_TexEnvOp(env3, C3D_RGB, GPU_TEVOP_RGB_SRC_ALPHA , 0, 0);
    C3D_TexEnvFunc(env3, C3D_RGB, GPU_ADD);
    
    static const C3D_Material lightMaterial =
    {
        { 0.6f, 0.6f, 0.6f }, //ambient
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
    
    C3D_FVec lightVec = { { 2.0, 2.0, -2.0, 0.0 } };

    C3D_LightInit(&light, &lightEnv);
    C3D_LightColor(&light, 1.0, 1.0, 1.0);
    C3D_LightPosition(&light, &lightVec);
    
    // Depth test is back
    C3D_DepthTest(true, GPU_GEQUAL, GPU_WRITE_ALL);
    
    // To heck with culling
    C3D_CullFace(GPU_CULL_NONE);

    
    // Draw the VBO
    C3D_TexBind(0, &sphere_tex);    
    C3D_BufInfo* bufInfo = C3D_GetBufInfo();
    
    BufInfo_Init(bufInfo);
    BufInfo_Add(bufInfo, vboVerts, sizeof(vertex), 3, 0x210);
    
    if(vertCount > 0) {
        C3D_DrawArrays(GPU_TRIANGLES, 0, vertCount);
    }
    
    C3D_LightEnvBind(0);
}

void effectNordlichtDrawLighthouse(float iod, float time, float escalate) {
    // Lighthouse
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

    // Calculate the modelView matrix
    C3D_Mtx modelView;
    Mtx_Identity(&modelView);
    Mtx_Translate(&modelView, 0.1 + 0.48, -1.6 + 0.42, -3.5 + 0.78 + 0.06);
    Mtx_RotateX(&modelView,  0.5, true);
    Mtx_RotateZ(&modelView, -0.3, true);
    //Mtx_RotateY(&modelView, time * 0.002, true);
    
    // Update the uniforms
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_projection, &projection);
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_modelView,  &modelView);

    C3D_TexEnv* env = C3D_GetTexEnv(0);
    C3D_TexEnvSrc(env, C3D_Both, GPU_FRAGMENT_PRIMARY_COLOR, 0, 0);
    C3D_TexEnvOp(env, C3D_Both, 0, 0, 0);
    C3D_TexEnvFunc(env, C3D_Both, GPU_REPLACE);

    C3D_TexEnv* env2 = C3D_GetTexEnv(1);
    TexEnv_Init(env2);
    C3D_TexEnv* env3 = C3D_GetTexEnv(2);
    TexEnv_Init(env3);
//     C3D_TexEnv* env2 = C3D_GetTexEnv(1);
//     C3D_TexEnvSrc(env2, C3D_RGB, GPU_TEXTURE0, GPU_PREVIOUS, 0);
//     C3D_TexEnvOp(env2, C3D_RGB, 0, 0, 0);
//     C3D_TexEnvFunc(env2, C3D_RGB, GPU_MODULATE);
//     
//     C3D_TexEnv* env3 = C3D_GetTexEnv(2);
//     C3D_TexEnvSrc(env3, C3D_RGB, GPU_FRAGMENT_SECONDARY_COLOR, GPU_PREVIOUS, 0);
//     C3D_TexEnvOp(env3, C3D_RGB, GPU_TEVOP_RGB_SRC_ALPHA , 0, 0);
//     C3D_TexEnvFunc(env3, C3D_RGB, GPU_ADD);
    
    static const C3D_Material lightMaterial =
    {
        { 0.2f, 0.2f, 0.2f }, //ambient
        { 0.4f, 0.4f, 0.4f }, //diffuse
        { 0.8f, 0.8f, 0.8f }, //specular0
        { 0.0f, 0.0f, 0.0f }, //specular1
        { 0.0f, 0.0f, 0.0f }, //emission
    };

    C3D_LightEnvInit(&lightEnv);
    C3D_LightEnvBind(&lightEnv);
    C3D_LightEnvMaterial(&lightEnv, &lightMaterial);

//     LightLut_Phong(&lut_Phong, 3.0);
//     C3D_LightEnvLut(&lightEnv, GPU_LUT_D0, GPU_LUTINPUT_LN, false, &lut_Phong);
    
//     LightLut_FromFunc(&lut_shittyFresnel, badFresnel, 0.6, false);
//     C3D_LightEnvLut(&lightEnv, GPU_LUT_FR, GPU_LUTINPUT_NV, false, &lut_shittyFresnel);
//     C3D_LightEnvFresnel(&lightEnv, GPU_PRI_SEC_ALPHA_FRESNEL);
    
    C3D_FVec lightVec = { { 2.0, 2.0, -2.0, 0.0 } };

    C3D_LightInit(&light, &lightEnv);
    C3D_LightColor(&light, 1.0, 1.0, 1.0);
    C3D_LightPosition(&light, &lightVec);
    
    C3D_CullFace(GPU_CULL_NONE);
    
    // Depth test is back
    C3D_DepthTest(true, GPU_GEQUAL, GPU_WRITE_ALL);
    
    C3D_BufInfo* bufInfo = C3D_GetBufInfo();
    BufInfo_Init(bufInfo);
    BufInfo_Add(bufInfo, vboVertsLighthouse, sizeof(vertex), 3, 0x210);
    C3D_DrawArrays(GPU_TRIANGLES, 0, lighthouseNumFaces * 3);
    
    C3D_LightEnvBind(0);    
}

void effectNordlichtRender(C3D_RenderTarget* targetLeft, C3D_RenderTarget* targetRight, float iod, float time, float escalate) {
    // Render some 2D stuff
    effectNordlichtUpdate(time, escalate);
    
    // Left eye
    C3D_FrameDrawOn(targetLeft);
    
    // Actual scene
    effectNordlichtDraw(iod, time, escalate);
    effectNordlichtDrawLighthouse(iod, time, escalate);
    fullscreenQuad(logo_tex, 0.0, 0.0);
    
    fade();
    
    if(iod > 0.0) {
        // Right eye
        C3D_FrameDrawOn(targetRight);
    
        // Actual scene
        effectNordlichtDraw(-iod, time, escalate);
        effectNordlichtDrawLighthouse(-iod, time, escalate);
        fullscreenQuad(logo_tex, 0.0, 0.0);
        
        fade();
    }
}

void effectNordlichtExit() {
    // Free the texture
    C3D_TexDelete(&sphere_tex);
    C3D_TexDelete(&logo_tex);
    
    // Free the VBOs
    linearFree(vboVerts);
    linearFree(vboVertsLighthouse);
    
    // Free the shader program
    shaderProgramFree(&program);
    DVLB_Free(vshader_dvlb);
}
