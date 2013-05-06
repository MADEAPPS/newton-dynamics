//
//
//
//                MilkShape 3D 1.8.2 File Format Specification
//
//
//                  This specifcation is written in C style.
//
//
// The data structures are defined in the order as they appear in the .ms3d file.
//
//
//
//
//



//
// max values
//
#define MAX_VERTICES    65534
#define MAX_TRIANGLES   65534
#define MAX_GROUPS      255
#define MAX_MATERIALS   128
#define MAX_JOINTS      128



//
// flags
//
#define SELECTED        1
#define HIDDEN          2
#define SELECTED2       4
#define DIRTY           8



//
// types
//
#ifndef byte
typedef unsigned char byte;
#endif // byte

#ifndef word
typedef unsigned short word;
#endif // word


// force one byte alignment
#include <pshpack1.h>

//
// First comes the header (sizeof(ms3d_header_t) == 14)
//
typedef struct
{
    char    id[10];                                     // always "MS3D000000"
    int     version;                                    // 4
} ms3d_header_t;

//
// Then comes the number of vertices
//
word nNumVertices; // 2 bytes

//
// Then come nNumVertices times ms3d_vertex_t structs (sizeof(ms3d_vertex_t) == 15)
//
typedef struct
{
    byte    flags;                                      // SELECTED | SELECTED2 | HIDDEN
    float   vertex[3];                                  //
    char    boneId;                                     // -1 = no bone
    byte    referenceCount;
} ms3d_vertex_t;

//
// Then comes the number of triangles
//
word nNumTriangles; // 2 bytes

//
// Then come nNumTriangles times ms3d_triangle_t structs (sizeof(ms3d_triangle_t) == 70)
//
typedef struct
{
    word    flags;                                      // SELECTED | SELECTED2 | HIDDEN
    word    vertexIndices[3];                           //
    float   vertexNormals[3][3];                        //
    float   s[3];                                       //
    float   t[3];                                       //
    byte    smoothingGroup;                             // 1 - 32
    byte    groupIndex;                                 //
} ms3d_triangle_t;

//
// Then comes the number of groups
//
word nNumGroups; // 2 bytes

//
// Then come nNumGroups times groups (the sizeof a group is dynamic, because of triangleIndices is numtriangles long)
//
typedef struct
{
    byte            flags;                              // SELECTED | HIDDEN
    char            name[32];                           //
    word            numtriangles;                       //
    word            triangleIndices[numtriangles];      // the groups group the triangles
    char            materialIndex;                      // -1 = no material
} ms3d_group_t;

//
// number of materials
//
word nNumMaterials; // 2 bytes

//
// Then come nNumMaterials times ms3d_material_t structs (sizeof(ms3d_material_t) == 361)
//
typedef struct
{
    char            name[32];                           //
    float           ambient[4];                         //
    float           diffuse[4];                         //
    float           specular[4];                        //
    float           emissive[4];                        //
    float           shininess;                          // 0.0f - 128.0f
    float           transparency;                       // 0.0f - 1.0f
    char            mode;                               // 0, 1, 2 is unused now
    char            texture[128];                        // texture.bmp
    char            alphamap[128];                       // alpha.bmp
} ms3d_material_t;

//
// save some keyframer data
//
float fAnimationFPS; // 4 bytes
float fCurrentTime; // 4 bytes
int iTotalFrames; // 4 bytes

//
// number of joints
//
word nNumJoints; // 2 bytes

//
// Then come nNumJoints joints (the size of joints are dynamic, because each joint has a differnt count of keys
//
typedef struct // 16 bytes
{
    float           time;                               // time in seconds
    float           rotation[3];                        // x, y, z angles
} ms3d_keyframe_rot_t;

typedef struct // 16 bytes
{
    float           time;                               // time in seconds
    float           position[3];                        // local position
} ms3d_keyframe_pos_t;

typedef struct
{
    byte            flags;                              // SELECTED | DIRTY
    char            name[32];                           //
    char            parentName[32];                     //
    float           rotation[3];                        // local reference matrix
    float           position[3];

    word            numKeyFramesRot;                    //
    word            numKeyFramesTrans;                  //

    ms3d_keyframe_rot_t keyFramesRot[numKeyFramesRot];      // local animation matrices
    ms3d_keyframe_pos_t keyFramesTrans[numKeyFramesTrans];  // local animation matrices
} ms3d_joint_t;

//
// Then comes the subVersion of the comments part, which is not available in older files
//
int subVersion; // subVersion is = 1, 4 bytes

// Then comes the numer of group comments
unsigned int nNumGroupComments; // 4 bytes

//
// Then come nNumGroupComments times group comments, which are dynamic, because the comment can be any length
//
typedef struct
{
	int index;											// index of group, material or joint
	int commentLength;									// length of comment (terminating '\0' is not saved), "MC" has comment length of 2 (not 3)
	char comment[commentLength];						// comment
} ms3d_comment_t;

// Then comes the number of material comments
int nNumMaterialComments; // 4 bytes

//
// Then come nNumMaterialComments times material comments, which are dynamic, because the comment can be any length
//

// Then comes the number of joint comments
int nNumJointComments; // 4 bytes

//
// Then come nNumJointComments times joint comments, which are dynamic, because the comment can be any length
//

// Then comes the number of model comments, which is always 0 or 1
int nHasModelComment; // 4 bytes

//
// Then come nHasModelComment times model comments, which are dynamic, because the comment can be any length
//


// Then comes the subversion of the vertex extra information like bone weights, extra etc.
int subVersion;		// subVersion is = 2, 4 bytes

// ms3d_vertex_ex_t for subVersion == 1
typedef struct
{
	char boneIds[3];									// index of joint or -1, if -1, then that weight is ignored, since subVersion 1
	byte weights[3];									// vertex weight ranging from 0 - 255, last weight is computed by 1.0 - sum(all weights), since subVersion 1
	// weight[0] is the weight for boneId in ms3d_vertex_t
	// weight[1] is the weight for boneIds[0]
	// weight[2] is the weight for boneIds[1]
	// 1.0f - weight[0] - weight[1] - weight[2] is the weight for boneIds[2]
} ms3d_vertex_ex_t;

// ms3d_vertex_ex_t for subVersion == 2
typedef struct
{
	char boneIds[3];									// index of joint or -1, if -1, then that weight is ignored, since subVersion 1
	byte weights[3];									// vertex weight ranging from 0 - 100, last weight is computed by 1.0 - sum(all weights), since subVersion 1
	// weight[0] is the weight for boneId in ms3d_vertex_t
	// weight[1] is the weight for boneIds[0]
	// weight[2] is the weight for boneIds[1]
	// 1.0f - weight[0] - weight[1] - weight[2] is the weight for boneIds[2]
	unsigned int extra;									// vertex extra, which can be used as color or anything else, since subVersion 2
} ms3d_vertex_ex_t;

// Then comes nNumVertices times ms3d_vertex_ex_t structs (sizeof(ms3d_vertex_ex_t) == 10)

// Then comes the subversion of the joint extra information like color etc.
int subVersion;		// subVersion is = 2, 4 bytes

// ms3d_joint_ex_t for subVersion == 1
typedef struct
{
	float color[3];	// joint color, since subVersion == 1
} ms3d_joint_ex_t;

// Then comes nNumJoints times ms3d_joint_ex_t structs (sizeof(ms3d_joint_ex_t) == 12)

// Then comes the subversion of the model extra information
int subVersion;		// subVersion is = 1, 4 bytes

// ms3d_model_ex_t for subVersion == 1
typedef struct
{
	float jointSize;	// joint size, since subVersion == 1
	int transparencyMode; // 0 = simple, 1 = depth buffered with alpha ref, 2 = depth sorted triangles, since subVersion == 1
	float alphaRef; // alpha reference value for transparencyMode = 1, since subVersion == 1
} ms3d_model_ex_t;

#include <poppack.h>


//
// Mesh Transformation:
// 
// 0. Build the transformation matrices from the rotation and position
// 1. Multiply the vertices by the inverse of local reference matrix (lmatrix0)
// 2. then translate the result by (lmatrix0 * keyFramesTrans)
// 3. then multiply the result by (lmatrix0 * keyFramesRot)
//
// For normals skip step 2.
//
//
//
// NOTE:  this file format may change in future versions!
//
//
// - Mete Ciragan
//
