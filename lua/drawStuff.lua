--====================================================================
local ffi =require("ffi");
--====================================================================
ffi.cdef(
[[

/* high byte is major version, low byte is minor version */
//#define DS_VERSION 0x0002
enum
{
    DS_VERSION = 0x0002
};

enum 
{
    DS_NONE = 0,       
    DS_WOOD,
    DS_CHECKERED,
    DS_GROUND,
    DS_SKY
};
//====================================================================
enum
{
 DS_POLYFILL  =0
,DS_WIREFRAME =1
};
//#define DS_POLYFILL  0
//#define DS_WIREFRAME 1
//====================================================================
typedef struct dsFunctions {
  int version;            
  void (*start)();        
  void (*step) (int pause);    
  void (*command) (int cmd);    
  void (*stop)();        
  const char *path_to_textures;    
} dsFunctions;
//====================================================================
void dsSimulationLoop (int argc, char **argv,
                       int window_width, int window_height,
                       struct dsFunctions *fn);

void dsError (const char *msg, ...);
void dsDebug (const char *msg, ...);
void dsPrint (const char *msg, ...);
void dsSetViewpoint (float xyz[3], float hpr[3]);
void dsGetViewpoint (float xyz[3], float hpr[3]);
void dsStop();
double dsElapsedTime();
void dsSetTexture (int texture_number);
void dsSetColor (float red, float green, float blue);
void dsSetColorAlpha (float red, float green, float blue, float alpha);
//====================================================================
void dsSetSphereQuality (int n);        
void dsSetCapsuleQuality (int n);        
void dsSetDrawMode(int mode);
//====================================================================
void dsDrawBox (const float pos[3], const float R[12], const float sides[3]);
void dsDrawSphere (const float pos[3], const float R[12], float radius);
void dsDrawTriangle (const float pos[3], const float R[12], const float *v0, const float *v1, const float *v2, int solid);
void dsDrawCylinder (const float pos[3], const float R[12], float length, float radius);
void dsDrawCapsule (const float pos[3], const float R[12],float length, float radius);
void dsDrawLine (const float pos1[3], const float pos2[3]);
void dsDrawConvex(const float pos[3], 
                  const float R[12],
                  float *_planes,
                  unsigned int _planecount,
                  float *_points,
                  unsigned int _pointcount,
                  unsigned int *_polygons);
//====================================================================
void dsDrawBoxD (const double pos[3], const double R[12],const double sides[3]);
void dsDrawSphereD (const double pos[3], const double R[12],const float radius);
void dsDrawTriangleD (const double pos[3], const double R[12],const double *v0, const double *v1, const double *v2, int solid);
void dsDrawCylinderD (const double pos[3], const double R[12],float length, float radius);
void dsDrawCapsuleD (const double pos[3], const double R[12],float length, float radius);
void dsDrawLineD (const double pos1[3], const double pos2[3]);
void dsDrawConvexD(const double pos[3], 
                   const double R[12],
                   double *_planes,
                   unsigned int _planecount,
                   double *_points,
                   unsigned int _pointcount,
                   unsigned int *_polygons);
//====================================================================                  

//#define dsDrawCappedCylinder dsDrawCapsule
//#define dsDrawCappedCylinderD dsDrawCapsuleD
//#define dsSetCappedCylinderQuality dsSetCapsuleQuality
]]);
--====================================================================
return ffi.load("drawstuff.dll");
--====================================================================


