// Nordlicht demoparty
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "Tools.h"
#include <vshader_shbin.h>
#include "tunnel_bin.h"
#include "gridbg_bin.h"
#include "logo_bin.h"
#include "Perlin.h"

#include "Lighthouse.h"

// Simple marching cubs impl.
#include "MarchingCubes.h"

#define NORDLICHT_MAX_VERTS 20000

#define GRID_X 11
#define GRID_Y 11
#define GRID_Z 11
#define GRID_STEP 0.08
#define GRID_EXTENT ((float)GRID_X * GRID_STEP)
#define GRID_OFFSET (GRID_EXTENT / 2.0)
#define GRID(x, y, z) ((x) + (y) * GRID_X + (z) * GRID_X * GRID_Y)

static DVLB_s* vshader_dvlb;
static shaderProgram_s program;

static int uLoc_projection, uLoc_modelView;
static C3D_Mtx projection;

static Pixel* screenPixels;
static Bitmap screen;
static C3D_Tex screen_tex;
static C3D_Tex logo_tex;

static C3D_LightEnv lightEnv;
static C3D_Light light;
static C3D_LightLut lut_Phong;
static C3D_LightLut lut_shittyFresnel;

int32_t vertCount;
int32_t vertCountMetaballs;
static vertex* vboVerts;
float* valueGrid;

#define SEGMENTS 15

#include "Font.h"
#include "MonoFont.h"

extern Font OL16Font; 

#define SCROLLERTEXT "                                                                              hi! halcy on the keys! unfortunately, SVatG once again didn't make anything before the party and then ran out of time, so this is only a simple scroller, but you can't very well NOT release something for evoke 20, right? the code and graphics are by halcy and the music was made by SunSpire. Greets fly out to desire, Rabenauge, k2, Nuance, mercury, Alcatraz, Riku55 and Titan, Poo-Brain, virgill, gaspode, h0fmann, Medo, Rene, DaTucker, kaomau, RbR and everyone here at the party. wishing you all a great evoke!                                  the scroller will now repeat.                                          hi! halcy on the keys! unfortunately, SVatG once again didn't make anything before the party and then ran out of time, so this is only a simple scroller, but you can't very well NOT release something for evoke 20, right? the code and graphics are by halcy and the music was made by SunSpire. Greets fly out to desire, Rabenauge, k2, Nuance, mercury, Alcatraz, Riku55 and Titan, Poo-Brain, virgill, gaspode, h0fmann, Medo, Rene, DaTucker, kaomau, RbR and everyone here at the party. wishing you all a great evoke!                                  the scroller will now repeat.                                          hi! halcy on the keys! unfortunately, SVatG once again didn't make anything before the party and then ran out of time, so this is only a simple scroller, but you can't very well NOT release something for evoke 20, right? the code and graphics are by halcy and the music was made by SunSpire. Greets fly out to desire, Rabenauge, k2, Nuance, mercury, Alcatraz, Riku55 and Titan, Poo-Brain, virgill, gaspode, h0fmann, Medo, Rene, DaTucker, kaomau, RbR and everyone here at the party. wishing you all a great evoke!                                  the scroller will now repeat.                                          hi! halcy on the keys! unfortunately, SVatG once again didn't make anything before the party and then ran out of time, so this is only a simple scroller, but you can't very well NOT release something for evoke 20, right? the code and graphics are by halcy and the music was made by SunSpire. Greets fly out to desire, Rabenauge, k2, Nuance, mercury, Alcatraz, Riku55 and Titan, Poo-Brain, virgill, gaspode, h0fmann, Medo, Rene, DaTucker, kaomau, RbR and everyone here at the party. wishing you all a great evoke!                                  the scroller will now repeat.                                          "

static Pixel* scrollPixels;
static Bitmap scroller;
static C3D_Tex scroll_tex;

static const C3D_Material lightMaterial = {
    { 0.1f, 0.1f, 0.1f }, //ambient
    { 0.2f, 0.8f, 0.4f }, //diffuse
    { 0.8f, 0.8f, 0.9f }, //specular0
    { 0.0f, 0.0f, 0.0f }, //specular1
    { 0.0f, 0.1f, 0.0f }, //emission
};

// Ohne tunnel geht eben nicht
void effectScrollerInit() {
    // Load default shader
    vshader_dvlb = DVLB_ParseFile((u32*)vshader_shbin, vshader_shbin_size);
    shaderProgramInit(&program);
    shaderProgramSetVsh(&program, &vshader_dvlb->DVLE[0]);
    
    // Scroller texture
    C3D_TexInit(&scroll_tex, 512, 512, GPU_RGBA8);
    scrollPixels = (Pixel*)linearAlloc(512 * 512 * sizeof(Pixel));
    InitialiseBitmap(&scroller, 512, 512, BytesPerRowForWidth(512), scrollPixels);
    
    // BG texture
    C3D_TexInit(&screen_tex, SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT, GPU_RGBA8);    
    screenPixels = (Pixel*)linearAlloc(SCREEN_TEXTURE_WIDTH * SCREEN_TEXTURE_HEIGHT * sizeof(Pixel));
    InitialiseBitmap(&screen, SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT, BytesPerRowForWidth(SCREEN_TEXTURE_WIDTH), screenPixels);
    
    vboVerts = (vertex*)linearAlloc(sizeof(vertex) * NORDLICHT_MAX_VERTS);
    valueGrid = malloc(sizeof(float) * GRID_X * GRID_Y * GRID_Z);
    for(int i = 0; i < GRID_X * GRID_Y * GRID_Z; i++) {
        valueGrid[i] = 0.0;
    }
    
    C3D_TexInit(&logo_tex, SCREEN_TEXTURE_HEIGHT, SCREEN_TEXTURE_WIDTH, GPU_RGBA8);
    C3D_TexUpload(&logo_tex, logo_bin);
    C3D_TexSetFilter(&logo_tex, GPU_LINEAR, GPU_NEAREST);
}


// A single metaball call.
// function:
// f(x,y,z) = 1 / ((x − x0)^2 + (y − y0)^2 + (z − z0)^2)
static inline float metaball(float x, float y, float z, float cx, float cy, float cz) {
    float dx = x - cx;
    float dy = y - cy;
    float dz = z - cz;
    float xs = dx * dx;
    float ys = dy * dy;
    float zs = dz * dz;
    return(1.0 / sqrt(xs + ys + zs));
}

static inline float field(float xx, float yy, float zz, float* xa, float* ya, float* za, int bc, float time) {
    float val = 0.0;
    for(int i = 0; i < bc; i++) {
        val += metaball(xx, yy, zz, xa[i], ya[i], za[i]);
    }
    
    float border = fmax(fmax(
        abs((xx - (float)GRID_OFFSET) * 3.1), 
        abs((yy - (float)GRID_OFFSET) * 3.1)), 
        abs((zz - (float)GRID_OFFSET) * 3.1));
    //val = fmax(-(val - 16.0 * (sin(time * 0.05) * 0.5)), border - 0.5);
    val = fmax(-(val - 4.0), border - 0.5);
    val += fmod(xx + yy * 0.3, GRID_STEP * 1.7) * 5.0;
    return(val);
}

static int effectMetaballsPrepareBalls(int offset, float time, float escalate) {    
    // Movement.
    float movescale = 0.01;
    
    float xpos = GRID_OFFSET + cos(time * 0.3 * movescale) / 3.0;
    float ypos = GRID_OFFSET + sin(time * 0.2 * movescale) / 2.0;
    float zpos = GRID_OFFSET + sin(time * 0.36 * movescale) / 2.5;

    float xpos2 = GRID_OFFSET + sin(time * 0.22 * movescale) / 2.0;
    float ypos2 = GRID_OFFSET + cos(time * 0.07 * movescale) / 3.0;
    float zpos2 = GRID_OFFSET + cos(time * 0.19 * movescale) / 2.4;

    float xa[2] = {xpos, xpos2};
    float ya[2] = {ypos, ypos2};
    float za[2] = {zpos, zpos2};
    for(int x = 0; x < GRID_X; x++) {
        for(int y = 0; y < GRID_Y; y++) {
            for(int z = 0; z < GRID_Z; z++) {
                float xx = (float)x * GRID_STEP;
                float yy = (float)y * GRID_STEP;
                float zz = (float)z * GRID_STEP;
                
                valueGrid[GRID(x, y, z)] = field(xx, yy, zz, xa, ya, za, 2, time);
            }
        }
    }
    
    vertCountMetaballs = 0;
    vec3_t corners[8];
    float values[8];
    for(int x = 0; x < GRID_X - 1; x++) {
        for(int y = 0; y < GRID_Y - 1; y++) {
            for(int z = 0; z < GRID_Z - 1; z++) {
                int32_t xx = x + 1;
                int32_t yy = y + 1;
                int32_t zz = z + 1;
                
                float ax = x * GRID_STEP;
                float ay = y * GRID_STEP;
                float az = z * GRID_STEP;
                
                float axx = xx * GRID_STEP;
                float ayy = yy * GRID_STEP;
                float azz = zz * GRID_STEP;

                corners[0] = vec3(ax, ay, azz);
                corners[1] = vec3(axx, ay, azz);
                corners[2] = vec3(axx, ay, az);
                corners[3] = vec3(ax, ay, az);
                corners[4] = vec3(ax, ayy, azz);
                corners[5] = vec3(axx, ayy, azz);
                corners[6] = vec3(axx, ayy, az);
                corners[7] = vec3(ax, ayy, az);
                
                values[0] = valueGrid[GRID(x, y, zz)];
                values[1] = valueGrid[GRID(xx, y, zz)];
                values[2] = valueGrid[GRID(xx, y, z)];
                values[3] = valueGrid[GRID(x, y, z)];
                values[4] = valueGrid[GRID(x, yy, zz)];
                values[5] = valueGrid[GRID(xx, yy, zz)];
                values[6] = valueGrid[GRID(xx, yy, z)];
                values[7] = valueGrid[GRID(x, yy, z)];
                
                vertCountMetaballs += polygonise(corners, values, 0.0, &(vboVerts[vertCountMetaballs + offset]));
            }
        }
    }
    
    for(int f = offset; f < offset + vertCountMetaballs / 3; f++) {
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
    
//     // Vertex normals
//     for(int i = 0; i < vertCount; i++) {
//         float xn = field(vboVerts[i].position[0] - NORMAL_STEP, vboVerts[i].position[1], vboVerts[i].position[2], xa, ya, za, 2, time);
//         float xp = field(vboVerts[i].position[0] + NORMAL_STEP, vboVerts[i].position[1], vboVerts[i].position[2], xa, ya, za, 2, time);
//         float yn = field(vboVerts[i].position[0], vboVerts[i].position[1] - NORMAL_STEP, vboVerts[i].position[2], xa, ya, za, 2, time);
//         float yp = field(vboVerts[i].position[0], vboVerts[i].position[1] + NORMAL_STEP, vboVerts[i].position[2], xa, ya, za, 2, time);
//         float zn = field(vboVerts[i].position[0], vboVerts[i].position[1], vboVerts[i].position[2] - NORMAL_STEP, xa, ya, za, 2, time);
//         float zp = field(vboVerts[i].position[0], vboVerts[i].position[1], vboVerts[i].position[2] + NORMAL_STEP, xa, ya, za, 2, time);
//         
//         float nx = (xp - xn);
//         float ny = (yp - yn);
//         float nz = (zp - zn);
//         
//         float nn = sqrt(nx * nx + ny * ny + nz * nz);
//         
//         vboVerts[i].normal[0] = xn / nn;
//         vboVerts[i].normal[1] = yn / nn;
//         vboVerts[i].normal[2] = zn / nn;
//     }
    return(vertCountMetaballs);
}

// Draw balls 
void effectMetaballsRenderBalls(int offset, float iod, float time, float escalate) {    
    // Configure buffers
    C3D_BufInfo* bufInfo = C3D_GetBufInfo();
    BufInfo_Init(bufInfo);
    BufInfo_Add(bufInfo, vboVerts, sizeof(vertex), 3, 0x210);
    
//     C3D_TexUpload(&sphere_tex, svatg2_bin);
//     C3D_TexSetFilter(&sphere_tex, GPU_LINEAR, GPU_NEAREST);
    //C3D_TexBind(0, &sphere_tex);
    
    // Calculate the modelView matrix
    C3D_Mtx modelView;
    Mtx_Identity(&modelView);
    Mtx_Translate(&modelView, 0.0, 0.0, -1.5, false);
    Mtx_RotateX(&modelView, -0.3, true);
    Mtx_RotateZ(&modelView, -0.3, true);
    Mtx_RotateY(&modelView, time * 0.002, true);
    Mtx_Translate(&modelView, -GRID_OFFSET, -GRID_OFFSET, -GRID_OFFSET, true);
    
    // Update the uniforms
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_projection, &projection);
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_modelView,  &modelView);

    C3D_TexEnv* env = C3D_GetTexEnv(0);
    C3D_TexEnvSrc(env, C3D_Both, GPU_FRAGMENT_PRIMARY_COLOR, GPU_FRAGMENT_SECONDARY_COLOR, 0);
    C3D_TexEnvOp(env, C3D_Both, 0, 0, 0);
    C3D_TexEnvFunc(env, C3D_Both, GPU_ADD);
    
    env = C3D_GetTexEnv(1);
    C3D_TexEnvColor(env, RGBAf(1.0, 1.0, 1.0, 1.0));
    C3D_TexEnvSrc(env, C3D_Alpha, GPU_CONSTANT, GPU_CONSTANT, 0);
    C3D_TexEnvOp(env, C3D_Alpha, 0, 0, 0);
    C3D_TexEnvFunc(env, C3D_Alpha, GPU_REPLACE);

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
    C3D_DepthTest(true, GPU_GREATER, GPU_WRITE_ALL);
    
    // To heck with culling
    C3D_CullFace(GPU_CULL_BACK_CCW);
    
    // Draw the VBO
    if(vertCountMetaballs > 0) {
        C3D_DrawArrays(GPU_TRIANGLES, offset, vertCountMetaballs);
    }
    
    C3D_LightEnvBind(0);
}

static int drawSingleRing(int offset, float time, float radVal, float idVal) {
    float xshift = cos(time * 0.003) * 0.1;
    float yshift = sin(time * 0.001) * 0.1;
    
    int ringVertCount = 0;
    float ringSects = 30;
    for(float sect = 0.0; sect < ringSects - ringSects / 3; sect += 1.0) {
        float ringAngleStartSin = sin((sect / ringSects) * 3.1415 * 2.0);
        float ringAngleStartCos = cos((sect / ringSects) * 3.1415 * 2.0);
        float ringAngleStopSin = sin(((sect + 1.0) / ringSects) * 3.1415 * 2.0);
        float ringAngleStopCos = cos(((sect + 1.0) / ringSects) * 3.1415 * 2.0);
        ringVertCount += buildQuad(
            &(vboVerts[offset + ringVertCount]),
            vec3mul(vec3(ringAngleStartSin, 0.0, ringAngleStartCos), radVal),
            vec3mul(vec3(ringAngleStopSin, 0.0, ringAngleStopCos), radVal),
            vec3mul(vec3(ringAngleStopSin, 0.2, ringAngleStopCos), radVal),
            vec3mul(vec3(ringAngleStartSin, 0.2, ringAngleStartCos), radVal),
            vec2(0.03125 * (sect), 1.0 - 0.03125 - 0.03125 * idVal),
            vec2(0.03125 * (1 + sect), 1.0 - 0.03125 - 0.03125 * idVal),
            vec2(0.03125 * (sect), 1.0 - 0.03125 * idVal),
            vec2(0.03125 * (1 + sect), 1.0 - 0.03125 * idVal)
        );
    }
    
    // Configure buffers
    C3D_BufInfo* bufInfo = C3D_GetBufInfo();
    BufInfo_Init(bufInfo);
    BufInfo_Add(bufInfo, vboVerts, sizeof(vertex), 3, 0x210);

    // Load the texture and bind it to the first texture unit
    C3D_TexBind(0, &scroll_tex);
    C3D_TexSetFilter(&scroll_tex, GPU_NEAREST, GPU_NEAREST);
    
    // Calculate the modelView matrix
    C3D_Mtx modelView;
    Mtx_Identity(&modelView);
    
    if(idVal != 3) {
        Mtx_Translate(&modelView, xshift, yshift, -1.5, false);
    }
    else {
        Mtx_Translate(&modelView, xshift, yshift - 0.2, -1.5, false);
    }
    if(idVal != 3) {
        Mtx_RotateX(&modelView, -0.001 * time + idVal * 0.7, true);
        Mtx_RotateZ(&modelView, -0.002 * time + idVal * 1.0, true);
    }
    Mtx_RotateY(&modelView, -3.14 + idVal * 2.0, true);
    
    // Update the uniforms
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_projection, &projection);
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_modelView,  &modelView);

    C3D_TexEnv* env = C3D_GetTexEnv(0);
    C3D_TexEnvSrc(env, C3D_Both, GPU_TEXTURE0, 0, 0);
    C3D_TexEnvOp(env, C3D_Both, 0, 0, 0);
    C3D_TexEnvFunc(env, C3D_Both, GPU_REPLACE);
    
    env = C3D_GetTexEnv(1);
    C3D_TexEnvSrc(env, C3D_Both, GPU_PREVIOUS, 0, 0);
    C3D_TexEnvOp(env, C3D_Both, 0, 0, 0);
    C3D_TexEnvFunc(env, C3D_Both, GPU_REPLACE);
    
    C3D_LightEnvBind(0);
    
    // Depth test is back
    C3D_DepthTest(true, GPU_GEQUAL, GPU_WRITE_ALL);
    
    // To heck with culling
    C3D_CullFace(GPU_CULL_NONE);
    
    // Draw the VBO
    C3D_DrawArrays(GPU_TRIANGLES, offset, ringVertCount);
    
    return(ringVertCount);
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

static int drawCubeSet(int offset, float time, float radVal, float cubeStart) {
    float xshift = cos(time * 0.003) * 0.1;
    float yshift = sin(time * 0.001) * 0.1;
    
    float ringSects = 30.0;
    int cubeVertCount = 0;
    cubeStart += time * 0.06;
    for(float sect = cubeStart; sect < cubeStart + 4; sect += 1.0) {
        float ringAngleStartSin = sin((sect / ringSects) * 3.1415 * 2.0);
        float ringAngleStartCos = cos((sect / ringSects) * 3.1415 * 2.0);
        cubeVertCount += buildCube(
            &(vboVerts[offset + cubeVertCount]),
            vec3mul(vec3(ringAngleStartSin, 0.0, ringAngleStartCos), radVal),
            0.05 + 0.01 * (sect - cubeStart),
            (int)sect - cubeStart,
            time,
            0.0
        );
    }
    
    // Recompute normals
    for(int f = offset; f < offset + cubeVertCount; f++) {
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
    
    // Configure buffers
    C3D_BufInfo* bufInfo = C3D_GetBufInfo();
    BufInfo_Init(bufInfo);
    BufInfo_Add(bufInfo, vboVerts, sizeof(vertex), 3, 0x210);
    
    // Calculate the modelView matrix
    C3D_Mtx modelView;
    Mtx_Identity(&modelView);
    Mtx_Translate(&modelView, xshift, yshift, -1.5, false);
    Mtx_RotateX(&modelView, -0.001 * time + radVal * 10.8, true);
    Mtx_RotateZ(&modelView, -0.002 * time + radVal * 49.2, true);
    Mtx_RotateY(&modelView, -3.14 + radVal * 66.3, true);
    
    // Update the uniforms
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_projection, &projection);
    C3D_FVUnifMtx4x4(GPU_VERTEX_SHADER, uLoc_modelView,  &modelView);

    C3D_TexEnv* env = C3D_GetTexEnv(0);
    C3D_TexEnvSrc(env, C3D_Both, GPU_FRAGMENT_PRIMARY_COLOR, GPU_FRAGMENT_SECONDARY_COLOR, 0);
    C3D_TexEnvOp(env, C3D_Both, 0, 0, 0);
    C3D_TexEnvFunc(env, C3D_Both, GPU_ADD);
    
    env = C3D_GetTexEnv(1);
    C3D_TexEnvColor(env, RGBAf(1.0, 1.0, 1.0, 1.0));
    C3D_TexEnvSrc(env, C3D_Alpha, GPU_CONSTANT, GPU_CONSTANT, 0);
    C3D_TexEnvOp(env, C3D_Alpha, 0, 0, 0);
    C3D_TexEnvFunc(env, C3D_Alpha, GPU_REPLACE);

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
    C3D_CullFace(GPU_CULL_BACK_CCW);
    
    // Draw the VBO
    //printf("%d %d\n", offset, cubeVertCount);
    C3D_DrawArrays(GPU_TRIANGLES, offset, cubeVertCount);
    
    return(cubeVertCount);
}

static void effectScrollerDraw(int offset, float iod, float time, float escalate) {
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
    Mtx_PerspStereoTilt(&projection, 65.0f*M_PI/180.0f, 400.0f/240.0f, 0.2f, 30.0f, iod, 2.0f, false);
    
    effectMetaballsRenderBalls(0, iod, time, escalate);
    for(int i = 2; i < 5; i++) {
        offset += drawCubeSet(offset, time, 0.17 + 0.2 * i, i);
    }
    
    for(int i = 3; i < 6; i++) {
        float scaleVal = ((float)i) / 4.0;
        offset += drawSingleRing(offset, time, scaleVal, i - 1);
    }
}

inline double wrapAngle( double angle ) {
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
}

void effectScrollerRender(C3D_RenderTarget* targetLeft, C3D_RenderTarget* targetRight, float iod, float time, float escalate) {
    int offset = 0;
    offset += effectMetaballsPrepareBalls(offset, time, escalate);
    
    float xshift = cos(time * 0.0003) * 0.1;
    float yshift = sin(time * 0.0001) * 0.1;
    
    // Render some 2D stuff
    FillBitmap(&screen, RGBAf(0.1 * 0.5, 0.15 * 0.5, 0.15 * 0.5, 1.0));
    for(int x = 0; x < SCREEN_WIDTH; x += 10) {
        for(int y = 0; y < SCREEN_HEIGHT; y += 10) {
            float posX = ((float)(x - SCREEN_WIDTH / 2) / (float)SCREEN_WIDTH) + xshift * 0.1;
            float posY = ((float)(y - SCREEN_HEIGHT / 2) / (float)SCREEN_WIDTH) - yshift * 0.1;
            float posR = sqrt(posX * posX + posY * posY);
            float posA = atan2(posX, posY);
            
            bool dVal1 = fmod(posR, 0.15) > 0.15 / 2.0;
            float timeMul = fmod(((int)(posR / 0.15)) * 12982342.12932, 2.0);
            timeMul = timeMul * (dVal1 ? 1 : -1);
            
            bool dVal2 = wrapAngle(posA + 3.14 + time * 0.01 * timeMul) > 3.14;
            
            Pixel colorPrimary = RGBAf(
                0.3 * (0.1 + 0.1 * dVal1), 
                0.3 * (0.35 + (dVal2 * 0.5) * (!dVal1)), 
                0.3 * (0.35 + (dVal2 * 0.5) * dVal1), 
                1.0
            );
            
            float lines = fmod(posX + posY * 0.3 + time * 0.0001 + 10.0, 0.3) > 0.15 ? 0.15 : 0.05;
            Pixel colorSecondary = RGBAf(lines, lines, lines, 1.0);
                
            DrawFilledRectangle(&screen, x, y, 10, 10, colorSecondary);
            DrawFilledCircle(&screen, x + 5, y + 5, 3, colorPrimary);
        }
    }
    
    GSPGPU_FlushDataCache(screenPixels, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(Pixel));
    GX_DisplayTransfer((u32*)screenPixels, GX_BUFFER_DIM(SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT), (u32*)screen_tex.data, GX_BUFFER_DIM(SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT), TEXTURE_TRANSFER_FLAGS);
    gspWaitForPPF();
    
    // Scroller draw
    float sshift = -time * 0.4;
    FillBitmap(&scroller, RGBAf(1.0, 1.0, 1.0, 0.1));
    for(int i = 3; i < 5; i++) {
        DrawSimpleString(&scroller, &OL16Font, sshift, i * 16, RGBAf(0.3, 0.6 + i * 0.1, 1.0 - i * 0.3, 1.0), SCROLLERTEXT);
    }
    
    GSPGPU_FlushDataCache(scrollPixels, 512 * 256 * sizeof(Pixel));
    GX_DisplayTransfer((u32*)scrollPixels, GX_BUFFER_DIM(512, 256), (u32*)scroll_tex.data, GX_BUFFER_DIM(512, 256), TEXTURE_TRANSFER_FLAGS);
    gspWaitForPPF();
    
    C3D_FrameBegin(C3D_FRAME_SYNCDRAW);
    
    // Left eye
    C3D_FrameDrawOn(targetLeft);
    
    // Background
    fullscreenQuad(screen_tex, -iod, 1.0 / 10.0);
    
    // Actual scene
    effectScrollerDraw(offset, -iod, time, escalate);
    
    // Overlay
    fullscreenQuad(logo_tex, 0.0, 0.0);
    
    fade();
    
    if(iod > 0.0) {
        // Right eye
        C3D_FrameDrawOn(targetRight);
    
        // Background
        fullscreenQuad(screen_tex, iod, 1.0 / 10.0);
        
        // Actual scene
        effectScrollerDraw(offset, iod, time, escalate);
        
        // Overlay
        fullscreenQuad(logo_tex, 0.0, 0.0);
        
        fade();
    }
    
    C3D_FrameEnd(0);
}

void effectScrollerExit() {
    // Free the texture
    C3D_TexDelete(&scroll_tex);
    C3D_TexDelete(&screen_tex);
    
    // Free the VBOs
    linearFree(vboVerts);
    
    // Free pixel data
    linearFree(scrollPixels);
    linearFree(screenPixels);
    
    // Free the shader program
    shaderProgramFree(&program);
    DVLB_Free(vshader_dvlb);
}
