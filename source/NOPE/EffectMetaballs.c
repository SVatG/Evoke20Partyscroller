// Pretty meta-balls.
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "Tools.h"
#include "vshader_shbin.h"
#include "svatg2_bin.h"
#include "cube_bin.h"

// Simple marching cubs impl.
#include "MarchingCubes.h"

#define METABALLS_MAX_VERTS 20000

static DVLB_s* vshader_dvlb;
static shaderProgram_s program;

static int uLoc_projection, uLoc_modelView;
static C3D_Mtx projection;

static Pixel* texPixels;
static Bitmap texture;
static C3D_Tex sphere_tex;

static C3D_LightEnv lightEnv;
static C3D_Light light;
static C3D_LightLut lut_Phong;
static C3D_LightLut lut_shittyFresnel;

static Pixel* screenPixels;
static Bitmap screen;
static C3D_Tex screen_tex;
static C3D_Tex logo_tex;

int32_t vertCount;
static vertex* vboVerts;
float* valueGrid;

#define GRID_X 15
#define GRID_Y 15
#define GRID_Z 15
#define GRID_STEP 0.05
#define GRID_EXTENT ((float)GRID_X * GRID_STEP)
#define GRID_OFFSET (GRID_EXTENT / 2.0)
#define GRID(x, y, z) ((x) + (y) * GRID_X + (z) * GRID_X * GRID_Y)

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

// Metablobbies
void effectMetaballsInit() {
    // Load default shader
    vshader_dvlb = DVLB_ParseFile((u32*)vshader_shbin, vshader_shbin_size);
    shaderProgramInit(&program);
    shaderProgramSetVsh(&program, &vshader_dvlb->DVLE[0]);

    C3D_TexInit(&sphere_tex, 256, 256, GPU_RGBA8);
    C3D_TexInit(&screen_tex, SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT, GPU_RGBA8);
    
    screenPixels = (Pixel*)linearAlloc(SCREEN_TEXTURE_WIDTH * SCREEN_TEXTURE_HEIGHT * sizeof(Pixel));
    InitialiseBitmap(&screen, SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT, BytesPerRowForWidth(SCREEN_TEXTURE_WIDTH), screenPixels);
    
    texPixels = (Pixel*)linearAlloc(256 * 256 * sizeof(Pixel));
    InitialiseBitmap(&texture, 256, 256, BytesPerRowForWidth(256), texPixels);
    
    vboVerts = (vertex*)linearAlloc(sizeof(vertex) * METABALLS_MAX_VERTS);
    valueGrid = malloc(sizeof(float) * GRID_X * GRID_Y * GRID_Z);
    for(int i = 0; i < GRID_X * GRID_Y * GRID_Z; i++) {
        valueGrid[i] = 0.0;
    }
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
    return(val);
}

void effectMetaballsPrepareBalls(float time, float escalate) {    
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
    
    vertCount = 0;
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
                
                vertCount += polygonise(corners, values, 0.0, &(vboVerts[vertCount]));
            }
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

    C3D_TexInit(&logo_tex, SCREEN_TEXTURE_HEIGHT, SCREEN_TEXTURE_WIDTH, GPU_RGBA8);
    C3D_TexUpload(&logo_tex, cube_bin);
    C3D_TexSetFilter(&logo_tex, GPU_LINEAR, GPU_NEAREST);
}

// Draw balls 
void effectMetaballsRenderBalls(float iod, float time, float escalate) {    
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
    
    // Configure buffers
    C3D_BufInfo* bufInfo = C3D_GetBufInfo();
    BufInfo_Init(bufInfo);
    BufInfo_Add(bufInfo, vboVerts, sizeof(vertex), 3, 0x210);
    
//     C3D_TexUpload(&sphere_tex, svatg2_bin);
//     C3D_TexSetFilter(&sphere_tex, GPU_LINEAR, GPU_NEAREST);
    C3D_TexBind(0, &sphere_tex);
    
    // Calculate the modelView matrix
    C3D_Mtx modelView;
    Mtx_Identity(&modelView);
    Mtx_Translate(&modelView, 0.0, 0.0, -1.4);
    Mtx_RotateX(&modelView, -0.3, true);
    Mtx_RotateZ(&modelView, -0.3, true);
    Mtx_RotateY(&modelView, time * 0.002, true);
    Mtx_Translate(&modelView, -GRID_OFFSET, -GRID_OFFSET, -GRID_OFFSET);
    
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
        { 0.2f, 0.2f, 0.2f }, //ambient
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
    C3D_LightTwoSideDiffuse(&light, true);
    
    // Depth test is back
    C3D_DepthTest(true, GPU_GREATER, GPU_WRITE_ALL);
    
    // To heck with culling
    C3D_CullFace(GPU_CULL_NONE);
    
    // Draw the VBO
    if(vertCount > 0) {
        C3D_DrawArrays(GPU_TRIANGLES, 0, vertCount);
    }
    
    C3D_LightEnvBind(0);
}

void effectMetaballsRender(C3D_RenderTarget* targetLeft, C3D_RenderTarget* targetRight, float iod, float time, float escalate) {
    
    // Load the texture and bind it to the first texture unit
    FillBitmap(&texture, RGBA(129, 97, 0, 255));
    if(escalate >= 1) {
        int32_t offset = (int32_t)(time * 0.2) % 30;
        for(int y = 0; y < 256; y += 30) {
            DrawFilledRectangle(&texture, 0, y + offset, 256, 15, RGBA(0, 0, 0, 0));
        }
    }
    
    GSPGPU_FlushDataCache(texPixels, 256 * 256 * sizeof(Pixel));
    GX_DisplayTransfer((u32*)texPixels, GX_BUFFER_DIM(256, 256), (u32*)sphere_tex.data, GX_BUFFER_DIM(256, 256), TEXTURE_TRANSFER_FLAGS);
    gspWaitForPPF();
    
    // Render some 2D stuff
    // Blocky noise rotozoom
    FillBitmap(&screen, RGBAf(0.0, 0.0, 0.0, 1.0));        
    for(int x = 0; x < SCREEN_WIDTH; x += 10) {
        for(int y = 0; y < SCREEN_HEIGHT; y += 10) {
            float ra = time / 3000.0;
            float cx = (x - SCREEN_WIDTH / 2) * (cos(time / 2000.0) + 1.5);
            float cy = (y - SCREEN_HEIGHT / 2) * (cos(time / 2000.0) + 1.5);;
            float rx = cx * sin(ra) + cy * cos(ra);
            float ry = cx * -cos(ra) + cy * sin(ra);
            float coolVal = nnnoise(rx + time * 0.001, ry + time * 0.001, 6) / 70000.0;
            DrawFilledCircle(&screen, x + 5, y + 5, 4, RGBAf(coolVal, coolVal / 2.0, coolVal, 1.0));
        }
    }
    
    GSPGPU_FlushDataCache(screenPixels, SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(Pixel));
    GX_DisplayTransfer((u32*)screenPixels, GX_BUFFER_DIM(SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT), (u32*)screen_tex.data, GX_BUFFER_DIM(SCREEN_TEXTURE_WIDTH, SCREEN_TEXTURE_HEIGHT), TEXTURE_TRANSFER_FLAGS);
    gspWaitForPPF();
    
    // Calculate ball vertices
    effectMetaballsPrepareBalls(time, escalate);
    
    // Left eye
    C3D_FrameDrawOn(targetLeft);
    
    // Fullscreen quad
    fullscreenQuad(screen_tex, -iod, 1.0 / 10.0);
    
    // Actual scene
    effectMetaballsRenderBalls(-iod, time, escalate);
    
    // Overlay
    fullscreenQuad(logo_tex, 0.0, 0.0);
    
    fade();
    
    if(iod > 0.0) {
        // Right eye
        C3D_FrameDrawOn(targetRight);
    
        // Fullscreen quad
        fullscreenQuad(screen_tex, iod, 1.0 / 10.0);
    
        // Actual scene
        effectMetaballsRenderBalls(iod, time, escalate);
        
        // Overlay
        fullscreenQuad(logo_tex, 0.0, 0.0);
        
        fade();
    }
}

void effectMetaballsExit() {
    // Free the texture
    C3D_TexDelete(&sphere_tex);
    C3D_TexDelete(&screen_tex);

    // Free the VBOs
    linearFree(vboVerts);
    
    // Free the shader program
    shaderProgramFree(&program);
    DVLB_Free(vshader_dvlb);
    
    // Free the bitmaps
    linearFree(screenPixels);
    linearFree(texPixels);
//     free(valueGrid); // "delayed free"
}
