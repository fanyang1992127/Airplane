

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <malloc.h>
#include <gl/glut.h>
#include "Vector3D.h"
#include "QuadMesh.h"
#include "CubeMesh.h"

const int meshSize = 16;    // Default Mesh Size
static int vWidth = 1300;     // Viewport width in pixels
static int vHeight = 500;    // Viewport height in pixels
static int currentButton, lmpx, lmpy;
static int cv = 0;
static unsigned char currentKey;
static GLfloat mouseX, mouseY, tmpX, tmpZ;
static GLfloat tx = 0.0;
static GLfloat ty= 0.0;
static GLfloat tz = 0.0;
static GLfloat sx = 1.0;
static GLfloat sy = 1.0;
static GLfloat sz = 1.0;
static GLfloat e = 0.0;
static GLfloat rang = 0.0;
static GLfloat angleX = 0.0;
static GLfloat angleY = 30 * 0.01745329251;
static GLfloat spin = 0.0;
static GLfloat fangle = 0.0;   // front angle
static GLfloat fangle2 = 0.0;
static GLfloat cangle = 30.0;   // camera angle
static GLfloat ax = 10.0;  // cood 1st drone
static GLfloat ay = 5.0;
static GLfloat az = 0.0;
static GLfloat bx = -10.0;  // cood 2nd drone
static GLfloat by = 6.0;
static GLfloat bz = 0.0;
static GLfloat spd = 0.0;
static GLfloat mx = 0.0;  // cood missle
static GLfloat my = 70.0;
static GLfloat mz = 0.0;
static GLfloat mspd = 2.0; // missle speed
static GLfloat tspd = 0.0;
static GLfloat tangle = 0.0;
static GLboolean flying = GL_FALSE;
static GLboolean crush = GL_FALSE;
static GLboolean hit = GL_FALSE;
static FILE *fp;
static bool transfer, scale, height, newCub;
static GLfloat cubeAttr[6000];
static GLfloat fh = 0.5;

typedef unsigned char  byte;
typedef unsigned short word;
typedef unsigned long  dword;
typedef unsigned short ushort;
typedef unsigned long  ulong;

typedef struct RGB
{
	byte r, g, b;
} RGB;

typedef struct RGBpixmap
{
	int nRows, nCols;
	RGB *pixel;
} RGBpixmap;

RGBpixmap pix[6];

// Structure defining a bounding box
typedef struct BBox {
	Vector3D min;
	Vector3D max;
} BBox;

BBox bbox[18];  

// Lighting/shading and material properties for drone - upcoming lecture - just copy for now

// Light properties
static GLfloat light_position0[] = { -6.0F, 12.0F, 0.0F, 1.0F };
static GLfloat light_position1[] = { 6.0F, 12.0F, 0.0F, 1.0F };
static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_ambient[] = { 0.2F, 0.2F, 0.2F, 1.0F };

// Material properties
static GLfloat drone_mat_ambient[] = { 0.4F, 0.2F, 0.0F, 1.0F };
static GLfloat drone_mat_specular[] = { 0.1F, 0.1F, 0.0F, 1.0F };
static GLfloat drone_mat_diffuse[] = { 0.9F, 0.5F, 0.0F, 1.0F };
static GLfloat drone_mat_shininess[] = { 0.0F };

// A quad mesh representing the ground
static QuadMesh groundMesh;



// Prototypes for functions in this module
void initOpenGL(int w, int h);
void display(void);
void reshape(int w, int h);
void mouse(int button, int state, int x, int y);
void mouseMotionHandler(int xMouse, int yMouse);
void keyboard(unsigned char key, int x, int y);
void specialKey(int key, int x, int y);
void keyUp(unsigned char key, int x, int y);
void specialKeyUp(int key, int x, int y);
Vector3D ScreenToWorld(int x, int y);


void fskip(FILE *fp, int num_bytes)
{
	int i;
	for (i = 0; i < num_bytes; i++)
		fgetc(fp);
}


ushort getShort(FILE *fp) //helper function
{ //BMP format uses little-endian integer types
  // get a 2-byte integer stored in little-endian form
	char ic;
	ushort ip;
	ic = fgetc(fp); ip = ic;  //first byte is little one 
	ic = fgetc(fp);  ip |= ((ushort)ic << 8); // or in high order byte
	return ip;
}
//<<<<<<<<<<<<<<<<<<<< getLong >>>>>>>>>>>>>>>>>>>
ulong getLong(FILE *fp) //helper function
{  //BMP format uses little-endian integer types
   // get a 4-byte integer stored in little-endian form
	ulong ip = 0;
	char ic = 0;
	unsigned char uc = ic;
	ic = fgetc(fp); uc = ic; ip = uc;
	ic = fgetc(fp); uc = ic; ip |= ((ulong)uc << 8);
	ic = fgetc(fp); uc = ic; ip |= ((ulong)uc << 16);
	ic = fgetc(fp); uc = ic; ip |= ((ulong)uc << 24);
	return ip;
}


void readBMPFile(RGBpixmap *pm, char *file)
{
	FILE *fp;
	long index;
	int k, row, col, numPadBytes, nBytesInRow;
	char ch1, ch2;
	ulong fileSize;
	ushort reserved1;    // always 0
	ushort reserved2;     // always 0 
	ulong offBits; // offset to image - unreliable
	ulong headerSize;     // always 40
	ulong numCols; // number of columns in image
	ulong numRows; // number of rows in image
	ushort planes;      // always 1 
	ushort bitsPerPixel;    //8 or 24; allow 24 here
	ulong compression;      // must be 0 for uncompressed 
	ulong imageSize;       // total bytes in image 
	ulong xPels;    // always 0 
	ulong yPels;    // always 0 
	ulong numLUTentries;    // 256 for 8 bit, otherwise 0 
	ulong impColors;       // always 0 
	long count;
	char dum;

	/* open the file */
	if ((fp = fopen(file, "rb")) == NULL)
	{
		printf("Error opening file %s.\n", file);
		exit(1);
	}

	/* check to see if it is a valid bitmap file */
	if (fgetc(fp) != 'B' || fgetc(fp) != 'M')
	{
		fclose(fp);
		printf("%s is not a bitmap file.\n", file);
		exit(1);
	}

	fileSize = getLong(fp);
	reserved1 = getShort(fp);    // always 0
	reserved2 = getShort(fp);     // always 0 
	offBits = getLong(fp); // offset to image - unreliable
	headerSize = getLong(fp);     // always 40
	numCols = getLong(fp); // number of columns in image
	numRows = getLong(fp); // number of rows in image
	planes = getShort(fp);      // always 1 
	bitsPerPixel = getShort(fp);    //8 or 24; allow 24 here
	compression = getLong(fp);      // must be 0 for uncompressed 
	imageSize = getLong(fp);       // total bytes in image 
	xPels = getLong(fp);    // always 0 
	yPels = getLong(fp);    // always 0 
	numLUTentries = getLong(fp);    // 256 for 8 bit, otherwise 0 
	impColors = getLong(fp);       // always 0 

	if (bitsPerPixel != 24)
	{ // error - must be a 24 bit uncompressed image
		printf("Error bitsperpixel not 24\n");
		exit(1);
	}
	//add bytes at end of each row so total # is a multiple of 4
	// round up 3*numCols to next mult. of 4
	nBytesInRow = ((3 * numCols + 3) / 4) * 4;
	numPadBytes = nBytesInRow - 3 * numCols; // need this many
	pm->nRows = numRows; // set class's data members
	pm->nCols = numCols;
	pm->pixel = (RGB *)malloc(3 * numRows * numCols);//make space for array
	if (!pm->pixel) return; // out of memory!
	count = 0;

	for (row = 0; row < numRows; row++) // read pixel values
	{
		for (col = 0; col < numCols; col++)
		{
			int r, g, b;
			b = fgetc(fp); g = fgetc(fp); r = fgetc(fp); //read bytes
			pm->pixel[count].r = r; //place them in colors
			pm->pixel[count].g = g;
			pm->pixel[count++].b = b;
		}
		fskip(fp, numPadBytes);
	}
	fclose(fp);
}

void setTexture(RGBpixmap *p, GLuint textureID)
{
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, p->nCols, p->nRows, 0, GL_RGB,
		GL_UNSIGNED_BYTE, p->pixel);
}


static GLfloat vertices[1000][3] = { {-1.0, 0.0,-1.0},
						 { 1.0, 0.0,-1.0},
						 { 1.0, 2.0,-1.0},
						 {-1.0, 2.0,-1.0},
						 {-1.0, 0.0, 1.0},
						 { 1.0, 0.0, 1.0},
						 { 1.0, 2.0, 1.0},
						 {-1.0, 2.0, 1.0},
};


static GLubyte quads[1000] = { 0, 1, 5, 4,	// bottom face
				   0, 3, 2, 1,	// back face
				   2, 3, 7, 6,	// top face
				   0, 4, 7, 3,  // left face
				   1, 2, 6, 5,  // right face
				   4, 5, 6, 7  // front face
};

static GLfloat quadNormals[1000][3] = { { 0.0,-1.0,  0.0} , 	// Bottom Face
						  { 0.0, 0.0, -1.0},	// Back Face 
						  { 0.0, 1.0,  0.0},	// Top Face
						  {-1.0, 0.0,  0.0},	// Left Face
						  { 1.0, 0.0,  0.0},	// Right Face
						  { 0.0, 0.0,  1.0}	// Front Face
};

CubeMesh newCube(GLfloat tx, GLfloat ty, GLfloat tz, GLfloat sfx, GLfloat sfy, GLfloat sfz, GLfloat e)
{
	CubeMesh cube;
	cube.angle = 0.0;
	cube.sfx = sfx;
	cube.sfy = sfy;
	cube.sfz = sfz;
	cube.tx = tx;
	cube.ty = ty;
	cube.tz = tz;
	cube.e = e;
	cube.f = 4 * sfy;
	cube.highlightMat_ambient[0] = 1.0;
	cube.highlightMat_ambient[1] = 1.00;
	cube.highlightMat_ambient[2] = 1.0;
	cube.highlightMat_ambient[3] = 1.0;
	cube.highlightMat_specular[0] = 1.0;
	cube.highlightMat_specular[1] = 1.0;
	cube.highlightMat_specular[2] = 1.0;
	cube.highlightMat_specular[3] = 1.0;
	cube.highlightMat_diffuse[0] = 1.0;
	cube.highlightMat_diffuse[1] = 1.0;
	cube.highlightMat_diffuse[2] = 1.0;
	cube.highlightMat_diffuse[3] = 1.0;
	cube.highlightMat_shininess[0] = 0.0;
	if (cube.e == 0.0| cube.e >= 3.0)
		update(&cube);
	else
		extrude(&cube);
	return cube;
}

void drawCube(CubeMesh *cube)
{

	// Setup the material and lights used for selected cube
	glMaterialfv(GL_FRONT, GL_AMBIENT, cube->highlightMat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, cube->highlightMat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, cube->highlightMat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, cube->highlightMat_shininess);


	if (cube->e == 1.0 | cube->e == 2.0) {
		glBindTexture(GL_TEXTURE_2D, 0);
		glBegin(GL_QUADS);
	}
	else if (cube->e == 3.0)
	{
		glBindTexture(GL_TEXTURE_2D, 2);
		glBegin(GL_QUADS);
	}
	else if (cube->e == 0.0)
	{
		glBindTexture(GL_TEXTURE_2D, 1);
		glBegin(GL_QUADS);
	}
	else if (cube->e == 4.0)
	{
		glBindTexture(GL_TEXTURE_2D, 3);
		glBegin(GL_QUADS);
	}
	else if (cube->e == 5.0)
	{
		glBindTexture(GL_TEXTURE_2D, 4);
		glBegin(GL_QUADS);
	}

	// Buttom Face
	glNormal3f(quadNormals[0][0], quadNormals[0][1], quadNormals[0][2]);
	glVertex3f(vertices[quads[0]][0], vertices[quads[0]][1], vertices[quads[0]][2]);
	glVertex3f(vertices[quads[1]][0], vertices[quads[1]][1], vertices[quads[1]][2]);
	glVertex3f(vertices[quads[2]][0], vertices[quads[2]][1], vertices[quads[2]][2]);
	glVertex3f(vertices[quads[3]][0], vertices[quads[3]][1], vertices[quads[3]][2]);
	// Back Face
	if (cube->e == 3.0)
	{
		glEnd();
		glBindTexture(GL_TEXTURE_2D, 3);
		glBegin(GL_QUADS);
	}
	glNormal3f(quadNormals[1][0], quadNormals[1][1], quadNormals[1][2]);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(vertices[quads[4]][0], vertices[quads[4]][1], vertices[quads[4]][2]);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(vertices[quads[5]][0], vertices[quads[5]][1], vertices[quads[5]][2]);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(vertices[quads[6]][0], vertices[quads[6]][1], vertices[quads[6]][2]);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(vertices[quads[7]][0], vertices[quads[7]][1], vertices[quads[7]][2]);
	// Top Face
	if (cube->e == 3.0)
	{
		glEnd();
		glBindTexture(GL_TEXTURE_2D, 2);
		glBegin(GL_QUADS);
	}

	glNormal3f(quadNormals[2][0], quadNormals[2][1], quadNormals[2][2]);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(vertices[quads[8]][0], vertices[quads[8]][1], vertices[quads[8]][2]);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(vertices[quads[9]][0], vertices[quads[9]][1], vertices[quads[9]][2]);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(vertices[quads[10]][0], vertices[quads[10]][1], vertices[quads[10]][2]);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(vertices[quads[11]][0], vertices[quads[11]][1], vertices[quads[11]][2]);
	// Left Face
	glNormal3f(quadNormals[3][0], quadNormals[3][1], quadNormals[3][2]);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(vertices[quads[12]][0], vertices[quads[12]][1], vertices[quads[12]][2]);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(vertices[quads[13]][0], vertices[quads[13]][1], vertices[quads[13]][2]);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(vertices[quads[14]][0], vertices[quads[14]][1], vertices[quads[14]][2]);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(vertices[quads[15]][0], vertices[quads[15]][1], vertices[quads[15]][2]);
	// Right Face
	glNormal3f(quadNormals[4][0], quadNormals[4][1], quadNormals[4][2]);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(vertices[quads[16]][0], vertices[quads[16]][1], vertices[quads[16]][2]);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(vertices[quads[17]][0], vertices[quads[17]][1], vertices[quads[17]][2]);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(vertices[quads[18]][0], vertices[quads[18]][1], vertices[quads[18]][2]);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(vertices[quads[19]][0], vertices[quads[19]][1], vertices[quads[19]][2]);
	// Front Face
	if (cube->e == 3.0)
	{
		glEnd();
		glBindTexture(GL_TEXTURE_2D, 3);
		glBegin(GL_QUADS);
	}
	glNormal3f(quadNormals[5][0], quadNormals[5][1], quadNormals[5][2]);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(vertices[quads[20]][0], vertices[quads[20]][1], vertices[quads[20]][2]);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(vertices[quads[21]][0], vertices[quads[21]][1], vertices[quads[21]][2]);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(vertices[quads[22]][0], vertices[quads[22]][1], vertices[quads[22]][2]);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(vertices[quads[23]][0], vertices[quads[23]][1], vertices[quads[23]][2]);
	
	int k = 6;
	if (cube->f > 1.0f)
		for (int j = 24; j < 20 * cube->f + 4; j = j + 20)
		{
			glNormal3f(quadNormals[k][0], quadNormals[k][1], quadNormals[k][2]);
			k++;
			glTexCoord2f(0.0, 0.0);
			glVertex3f(vertices[quads[j]][0], vertices[quads[j]][1], vertices[quads[j]][2]);
			glTexCoord2f(0.0, 1.0);
			glVertex3f(vertices[quads[j + 1]][0], vertices[quads[j + 1]][1], vertices[quads[j + 1]][2]);
			glTexCoord2f(1.0, 1.0);
			glVertex3f(vertices[quads[j + 2]][0], vertices[quads[j + 2]][1], vertices[quads[j + 2]][2]);
			glTexCoord2f(1.0, 0.0);
			glVertex3f(vertices[quads[j + 3]][0], vertices[quads[j + 3]][1], vertices[quads[j + 3]][2]);
			// Back Face

			glEnd();
			glBindTexture(GL_TEXTURE_2D, 1);
			glBegin(GL_QUADS);

			glNormal3f(quadNormals[k][0], quadNormals[k][1], quadNormals[k][2]);
			k++;
			glTexCoord2f(0.0, 0.0);
			glVertex3f(vertices[quads[j + 4]][0], vertices[quads[j + 4]][1], vertices[quads[j + 4]][2]);
			glTexCoord2f(0.0, 1.0);
			glVertex3f(vertices[quads[j + 5]][0], vertices[quads[j + 5]][1], vertices[quads[j + 5]][2]);
			glTexCoord2f(1.0, 1.0);
			glVertex3f(vertices[quads[j + 6]][0], vertices[quads[j + 6]][1], vertices[quads[j + 6]][2]);
			glTexCoord2f(1.0, 0.0);
			glVertex3f(vertices[quads[j + 7]][0], vertices[quads[j + 7]][1], vertices[quads[j + 7]][2]);
			// Top Face
			glEnd();
			glBindTexture(GL_TEXTURE_2D, 0);
			glBegin(GL_QUADS);
			
			glNormal3f(quadNormals[k][0], quadNormals[k][1], quadNormals[k][2]);
			k++;
			glTexCoord2f(0.0, 0.0);
			glVertex3f(vertices[quads[j + 8]][0], vertices[quads[j + 8]][1], vertices[quads[j + 8]][2]);
			glTexCoord2f(0.0, 1.0);
			glVertex3f(vertices[quads[j + 9]][0], vertices[quads[j + 9]][1], vertices[quads[j + 9]][2]);
			glTexCoord2f(1.0, 1.0);
			glVertex3f(vertices[quads[j + 10]][0], vertices[quads[j + 10]][1], vertices[quads[j + 10]][2]);
			glTexCoord2f(1.0, 0.0);
			glVertex3f(vertices[quads[j + 11]][0], vertices[quads[j + 11]][1], vertices[quads[j + 11]][2]);
			// Left Face

			glNormal3f(quadNormals[k][0], quadNormals[k][1], quadNormals[k][2]);
			k++;
			glTexCoord2f(0.0, 0.0);
			glVertex3f(vertices[quads[j + 12]][0], vertices[quads[j + 12]][1], vertices[quads[j + 12]][2]);
			glTexCoord2f(0.0, 1.0);
			glVertex3f(vertices[quads[j + 13]][0], vertices[quads[j + 13]][1], vertices[quads[j + 13]][2]);
			glTexCoord2f(1.0, 1.0);
			glVertex3f(vertices[quads[j + 14]][0], vertices[quads[j + 14]][1], vertices[quads[j + 14]][2]);
			glTexCoord2f(1.0, 0.0);
			glVertex3f(vertices[quads[j + 15]][0], vertices[quads[j + 15]][1], vertices[quads[j + 15]][2]);
			// Right Face
			glNormal3f(quadNormals[k][0], quadNormals[k][1], quadNormals[k][2]);
			k++;
			glTexCoord2f(0.0, 0.0);
			glVertex3f(vertices[quads[j + 16]][0], vertices[quads[j + 16]][1], vertices[quads[j + 16]][2]);
			glTexCoord2f(0.0, 1.0);
			glVertex3f(vertices[quads[j + 17]][0], vertices[quads[j + 17]][1], vertices[quads[j + 17]][2]);
			glTexCoord2f(1.0, 1.0);
			glVertex3f(vertices[quads[j + 18]][0], vertices[quads[j + 18]][1], vertices[quads[j + 18]][2]);
			glTexCoord2f(1.0, 0.0);
			glVertex3f(vertices[quads[j + 19]][0], vertices[quads[j + 19]][1], vertices[quads[j + 19]][2]);
			// Front Face

		}
	glEnd();
	glFlush();
}
void update(CubeMesh *cube)
{

	int i = 0;
	vertices[i][0] = cube->tx - cube->sfx; //{-1.0, -1.0, -1.0},
	vertices[i][1] = cube->ty - cube->sfy + 1;
	vertices[i][2] = cube->tz - cube->sfz;
	vertices[i + 1][0] = cube->tx + cube->sfx;//{ 1.0, -1.0,-1.0 },
	vertices[i + 1][1] = cube->ty - cube->sfy + 1;
	vertices[i + 1][2] = cube->tz - cube->sfz;
	vertices[i + 2][0] = cube->tx + cube->sfx;//{ 1.0,  1.0,-1.0 },
	vertices[i + 2][1] = cube->ty + cube->sfy + 1;
	vertices[i + 2][2] = cube->tz - cube->sfz;
	vertices[i + 3][0] = cube->tx - cube->sfx;//{ -1.0,  1.0,-1.0 },
	vertices[i + 3][1] = cube->ty + cube->sfy + 1;
	vertices[i + 3][2] = cube->tz - cube->sfz;
	vertices[i + 4][0] = cube->tx - cube->sfx;//{ -1.0, -1.0, 1.0 },
	vertices[i + 4][1] = cube->ty - cube->sfy + 1;
	vertices[i + 4][2] = cube->tz + cube->sfz;
	vertices[i + 5][0] = cube->tx + cube->sfx;//{ 1.0, -1.0, 1.0 },
	vertices[i + 5][1] = cube->ty - cube->sfy + 1;
	vertices[i + 5][2] = cube->tz + cube->sfz;
	vertices[i + 6][0] = cube->tx + cube->sfx;//{ 1.0,  1.0, 1.0 },
	vertices[i + 6][1] = cube->ty + cube->sfy + 1;
	vertices[i + 6][2] = cube->tz + cube->sfz;
	vertices[i + 7][0] = cube->tx - cube->sfx;//{ -1.0,  1.0, 1.0 },
	vertices[i + 7][1] = cube->ty + cube->sfy + 1;
	vertices[i + 7][2] = cube->tz + cube->sfz;


}


void extrude(CubeMesh *cube)
{
	for (int i = 0; i < 8 * cube->f; i = i + 8)
	{


		if (cube->e == 2.0)
		{
			if (i / 8 % 4 == 0) {
				cube->tx = cube->tx - 0.1;
				cube->tz = cube->tz - 0.1;
			}
			else if (i / 8 % 4 == 1) {
				cube->tx = cube->tx + 0.1;
				cube->tz = cube->tz - 0.1;
			}
			else if (i / 8 % 4 == 2) {
				cube->tx = cube->tx + 0.1;
				cube->tz = cube->tz + 0.1;
			}
			else {
				cube->tx = cube->tx - 0.1;
				cube->tz = cube->tz + 0.1;
			}
		}


		vertices[i][0] = cube->tx - cube->sfx; //{-1.0, -1.0, -1.0},
		vertices[i][1] = i / 8 * 0.5;
		vertices[i][2] = cube->tz - cube->sfz;
		vertices[i + 1][0] = cube->tx + cube->sfx;//{ 1.0, -1.0,-1.0 },
		vertices[i + 1][1] = i / 8 * 0.5;
		vertices[i + 1][2] = cube->tz - cube->sfz;
		vertices[i + 2][0] = cube->tx + cube->sfx;//{ 1.0,  1.0,-1.0 },
		vertices[i + 2][1] = (i / 8 + 1) * 0.5;
		vertices[i + 2][2] = cube->tz - cube->sfz;
		vertices[i + 3][0] = cube->tx - cube->sfx;//{ -1.0,  1.0,-1.0 },
		vertices[i + 3][1] = (i / 8 + 1) * 0.5;
		vertices[i + 3][2] = cube->tz - cube->sfz;
		vertices[i + 4][0] = cube->tx - cube->sfx;//{ -1.0, -1.0, 1.0 },
		vertices[i + 4][1] = i / 8 * 0.5;
		vertices[i + 4][2] = cube->tz + cube->sfz;
		vertices[i + 5][0] = cube->tx + cube->sfx;//{ 1.0, -1.0, 1.0 },
		vertices[i + 5][1] = i / 8 * 0.5;
		vertices[i + 5][2] = cube->tz + cube->sfz;
		vertices[i + 6][0] = cube->tx + cube->sfx;//{ 1.0,  1.0, 1.0 },
		vertices[i + 6][1] = (i / 8 + 1) * 0.5;
		vertices[i + 6][2] = cube->tz + cube->sfz;
		vertices[i + 7][0] = cube->tx - cube->sfx;//{ -1.0,  1.0, 1.0 },
		vertices[i + 7][1] = (i / 8 + 1) * 0.5;
		vertices[i + 7][2] = cube->tz + cube->sfz;
	}
	if (cube->f > 1.0f)
	{
		for (int j = 24; j < 20 * cube->f + 4; j = j + 20)
		{

			quads[j] = quads[j - 20] + 8;
			quads[j + 1] = quads[j - 19] + 8;
			quads[j + 2] = quads[j - 18] + 8;
			quads[j + 3] = quads[j - 17] + 8;
			quads[j + 4] = quads[j - 16] + 8;
			quads[j + 5] = quads[j - 15] + 8;
			quads[j + 6] = quads[j - 14] + 8;
			quads[j + 7] = quads[j - 13] + 8;
			quads[j + 8] = quads[j - 12] + 8;
			quads[j + 9] = quads[j - 11] + 8;
			quads[j + 10] = quads[j - 10] + 8;
			quads[j + 11] = quads[j - 9] + 8;
			quads[j + 12] = quads[j - 8] + 8;
			quads[j + 13] = quads[j - 7] + 8;
			quads[j + 14] = quads[j - 6] + 8;
			quads[j + 15] = quads[j - 5] + 8;
			quads[j + 16] = quads[j - 4] + 8;
			quads[j + 17] = quads[j - 3] + 8;
			quads[j + 18] = quads[j - 2] + 8;
			quads[j + 19] = quads[j - 1] + 8;
			quads[j + 20] = quads[j - 0] + 8;

		}
		if (cube->e == 3.0f) {
			quadNormals[4][0] = 0.7071068967;//right face
			quadNormals[4][2] = 0.7071068967;
			quadNormals[5][0] = 0.7071068967;//front face
			quadNormals[5][2] = 0.7071068967;
		}
		for (int l = 6; l < 5 * cube->f + 6; l = l + 5)
		{
			quadNormals[l][0] = 0.0;//back face
			quadNormals[l][1] = 0.0;
			quadNormals[l][2] = -1.0;
			quadNormals[l + 1][0] = 0.0;//top face
			quadNormals[l + 1][1] = 1.0;
			quadNormals[l + 1][2] = 0.0;
			quadNormals[l + 2][0] = -1.0;//left face
			quadNormals[l + 2][1] = 0.0;
			quadNormals[l + 2][2] = 0.0;
			quadNormals[l + 3][0] = 1.0;//right face
			quadNormals[l + 3][1] = 0.0;
			quadNormals[l + 3][2] = 0.0;
			quadNormals[l + 4][0] = 0.0;//front face
			quadNormals[l + 4][1] = 0.0;
			quadNormals[l + 4][2] = 1.0;
			if (cube->e == 3.0f) {
				quadNormals[l + 3][0] = 0.7071068967;//right face
				quadNormals[l + 3][2] = 0.7071068967;
				quadNormals[l + 4][0] = 0.7071068967;//front face
				quadNormals[l + 4][2] = 0.7071068967;
			}
		}

	}
}


int main(int argc, char **argv)
{
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(vWidth, vHeight);
    glutInitWindowPosition(200, 30);
    glutCreateWindow("Assignment 3");

    // Initialize GL
    initOpenGL(vWidth, vHeight);

    // Register callbacks
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutMouseFunc(mouse);
    glutMotionFunc(mouseMotionHandler);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKey);
	glutKeyboardUpFunc(keyUp);
	glutSpecialUpFunc(specialKeyUp);
	readBMPFile(&pix[0], "glass.bmp");  
	setTexture(&pix[0], 0);
	readBMPFile(&pix[1], "top.bmp");
	setTexture(&pix[1], 1);
	readBMPFile(&pix[2], "drone.bmp");
	setTexture(&pix[2], 2);
	readBMPFile(&pix[3], "drone2.bmp");
	setTexture(&pix[3], 3);
	readBMPFile(&pix[4], "road.bmp");
	setTexture(&pix[4], 4);
    glutMainLoop();
	
    return 0;
}


// Set up OpenGL. For viewport and projection setup see reshape(). */
void initOpenGL(int w, int h)
{
    // Set up and enable lighting
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);


    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
	glEnable(GL_TEXTURE_2D);

    // Other OpenGL setup
    glEnable(GL_DEPTH_TEST);   // Remove hidded surfaces
    glShadeModel(GL_SMOOTH);   // Use smooth shading, makes boundaries between polygons harder to see 
    glClearColor(0.6F, 0.6F, 0.6F, 0.0F);  // Color and depth for glClear
    glClearDepth(1.0f);

    glEnable(GL_NORMALIZE);    // Renormalize normal vectors 
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);   // Nicer perspective

    // Set up ground quad mesh
    Vector3D origin = NewVector3D(-32.0f, 0.0f, 32.0f);
    Vector3D dir1v = NewVector3D(1.0f, 0.0f, 0.0f);
    Vector3D dir2v = NewVector3D(0.0f, 0.0f, -1.0f);
    groundMesh = NewQuadMesh(meshSize);

    InitMeshQM(&groundMesh, meshSize, origin, 64.0, 64.0, dir1v, dir2v);

    Vector3D ambient = NewVector3D(0.2f, 1.0f, 0.2f);
    Vector3D diffuse = NewVector3D(0.2f, 1.0f, 0.2f);
    Vector3D specular = NewVector3D(0.04f, 0.04f, 0.04f);
    SetMaterialQM(&groundMesh, ambient, diffuse, specular, 0.2);
	tmpX = 0;
	tmpZ = 0;
	printf("Shoot down the enemy drone, be careful not to crush into ground buildings or ground!\nDrag mouse to move the city view camera angle\nPress f6 for instructions");
}


// Callback, called whenever GLUT determines that the window should be redisplayed
// or glutPostRedisplay() has been called.
void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, (GLsizei)vWidth/2, (GLsizei)vHeight);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLdouble)vWidth / (2 * vHeight), 0.2, 40.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	gluLookAt(0, 6, 20, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0); 
	glRotatef(angleX, 0.f, 1.f, 0.f);
	glRotatef(angleY, cos(angleX*0.01745329251), 0 , sin(angleX*0.01745329251));
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	DrawMeshQM(&groundMesh, meshSize);
	
	

	cv = 0;
	//read file.txt
	fp = fopen("file.txt", "r");
	while (fscanf(fp, "%f", &cubeAttr[cv]) != EOF) {
		cv++;
	}
	fclose(fp);
	for (int i = 0; i < cv; i=i+7)
	{	
		glPushMatrix();
		CubeMesh tmp = newCube(cubeAttr[i], cubeAttr[i + 1], cubeAttr[i + 2], cubeAttr[i + 3], cubeAttr[i + 4], cubeAttr[i + 5], cubeAttr[i+6]);
		drawCube(&tmp);
		glPopMatrix();
		// Set up the bounding box of the scene
		Set(&bbox[i/7].min, cubeAttr[i] - cubeAttr[i + 3], cubeAttr[i + 1] + 1 - cubeAttr[i + 4], cubeAttr[i + 2] - cubeAttr[i + 5]);
		Set(&bbox[i/7].max, cubeAttr[i] + cubeAttr[i + 3], cubeAttr[i + 1] + 1 + cubeAttr[i + 4], cubeAttr[i + 2] + cubeAttr[i + 5]);
	}
	glPushMatrix();
	glRotatef(90, 0.0, 1.0, 0.0);
	CubeMesh r = newCube(0.0, -0.98, 0.0, 32.0, 0.02, 1.0, 5.0);
	drawCube(&r);
	glPopMatrix();

	


	
	if (crush == GL_FALSE) {

		glPushMatrix();
		glTranslatef(mx, my + 0.5, mz);
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glScalef(0.1, 0.1, 1.0);
		glutSolidCube(1.0);
		glPopMatrix();


		glPushMatrix();
		glTranslatef(ax, ay, az);
		glRotatef(45, 0.0, 1.0, 0.0);


		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glRotatef(90, 0.0, 1.0, 0.0);
		CubeMesh q1 = newCube(0.0, 0.0, 0.0, 0.1, 0.1, 1.0, 3.0);
		drawCube(&q1);
		glPopMatrix();

		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		CubeMesh q2 = newCube(0.0, 0.0, 0.0, 0.1, 0.1, 1.0, 3.0);
		drawCube(&q2);
		glPopMatrix();

		


		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glTranslatef(0.93, 0.0, 0.0);
		glRotatef(spin, 0.0, 1.0, 0.0);
		glRotatef(50, 0.0, 1.0, 0.0);
		CubeMesh q3 = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
		drawCube(&q3);
		glPopMatrix();


		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glTranslatef(-0.93, 0.0, 0.0);
		glRotatef(spin, 0.0, 1.0, 0.0);
		glRotatef(30, 0.0, 1.0, 0.0);
		CubeMesh q4 = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
		drawCube(&q4);
		glPopMatrix();


		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glTranslatef(0.0, 0.0, 0.93);
		glRotatef(spin, 0.0, 1.0, 0.0);
		glRotatef(45, 0.0, 1.0, 0.0);
		CubeMesh q5 = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
		drawCube(&q5);
		glPopMatrix();

		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glTranslatef(0.0, 0.0, -0.93);
		glRotatef(spin, 0.0, 1.0, 0.0);
		CubeMesh q6 = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
		drawCube(&q6);
		glPopMatrix();
		glPopMatrix();
	}

	glTranslatef(bx, by, bz);
	glRotatef(45, 0.0, 1.0, 0.0);

	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	glRotatef(90, 0.0, 1.0, 0.0);
	CubeMesh q1a = newCube(0.0, 0.0, 0.0, 0.1, 0.1, 1.0, 3.0);
	drawCube(&q1a);
	glPopMatrix();

	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	CubeMesh q2a = newCube(0.0, 0.0, 0.0, 0.1, 0.1, 1.0, 3.0);
	drawCube(&q2a);
	glPopMatrix();


	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	glTranslatef(0.93, 0.0, 0.0);
	glRotatef(spin, 0.0, 1.0, 0.0);
	glRotatef(50, 0.0, 1.0, 0.0);
	CubeMesh q3a = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
	drawCube(&q3a);
	glPopMatrix();


	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	glTranslatef(-0.93, 0.0, 0.0);
	glRotatef(spin, 0.0, 1.0, 0.0);
	glRotatef(30, 0.0, 1.0, 0.0);
	CubeMesh q4a = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
	drawCube(&q4a);
	glPopMatrix();


	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.0, 0.93);
	glRotatef(spin, 0.0, 1.0, 0.0);
	glRotatef(45, 0.0, 1.0, 0.0);
	CubeMesh q5a = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
	drawCube(&q5a);
	glPopMatrix();

	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.0, -0.93);
	glRotatef(spin, 0.0, 1.0, 0.0);
	CubeMesh q6a = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
	drawCube(&q6a);
	glPopMatrix();

	bbox[17].max.x = bx + 1;
	bbox[17].min.x = bx - 1;
	bbox[17].max.y = by + 1.5;
	bbox[17].min.y = by + 1;
	bbox[17].max.z = bz + 1;
	bbox[17].min.z = bz - 1;

	glViewport((GLsizei)vWidth / 2, 0, (GLsizei)vWidth / 2, (GLsizei)vHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLdouble)vWidth / (2 * vHeight), 0.2, 40.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	
	gluLookAt(ax, ay - rang * sin(cangle*0.01745329251) + 0.9, az - rang * cos(cangle*0.01745329251), ax, ay - 20 * sin(cangle*0.01745329251), az - 20 * cos(cangle*0.01745329251), 0.0, 1.0, 0.0);
	glTranslatef(ax, ay, az);
	glRotatef(-fangle, 0.f, 1.f, 0.f);
	glTranslatef(-ax, -ay, -az);

	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	DrawMeshQM(&groundMesh, meshSize);


	cv = 0;
	//read file.txt
	fp = fopen("file.txt", "r");
	while (fscanf(fp, "%f", &cubeAttr[cv]) != EOF) {
		cv++;
	}
	fclose(fp);
	for (int i = 0; i < cv; i = i + 7)
	{
		glPushMatrix();
		CubeMesh tmp = newCube(cubeAttr[i], cubeAttr[i + 1], cubeAttr[i + 2], cubeAttr[i + 3], cubeAttr[i + 4], cubeAttr[i + 5], cubeAttr[i + 6]);
		drawCube(&tmp);
		glPopMatrix();
	}
	glPushMatrix();
	glRotatef(90, 0.0, 1.0, 0.0);
	CubeMesh r2 = newCube(0.0, -0.98, 0.0, 32.0, 0.02, 1.0, 5.0);
	drawCube(&r2);
	glPopMatrix();

	glPushMatrix();
	glRotatef(90, 0.0, 1.0, 0.0);
	CubeMesh r3 = newCube(0.0, -0.98, 0.0, 32.0, 0.02, 1.0, 5.0);
	drawCube(&r3);
	glPopMatrix();

	if (crush == GL_FALSE) {

		glPushMatrix();

		glTranslatef(mx, my + 0.5, mz);
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glScalef(0.1, 0.1, 1.0);
		glutSolidCube(1.0);
		glPopMatrix();


		glPushMatrix();
		glTranslatef(ax, ay, az);
		glRotatef(45, 0.0, 1.0, 0.0);
		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glRotatef(90, 0.0, 1.0, 0.0);
		CubeMesh q7 = newCube(0.0, 0.0, 0.0, 0.1, 0.1, 1.0, 3.0);
		drawCube(&q7);
		glPopMatrix();

		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		CubeMesh q8 = newCube(0.0, 0.0, 0.0, 0.1, 0.1, 1.0, 3.0);
		drawCube(&q8);
		glPopMatrix();


		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glTranslatef(0.93, 0.0, 0.0);
		glRotatef(spin, 0.0, 1.0, 0.0);
		glRotatef(50, 0.0, 1.0, 0.0);
		CubeMesh q9 = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
		drawCube(&q9);
		glPopMatrix();


		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glTranslatef(-0.93, 0.0, 0.0);
		glRotatef(spin, 0.0, 1.0, 0.0);
		glRotatef(30, 0.0, 1.0, 0.0);
		CubeMesh q10 = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
		drawCube(&q10);
		glPopMatrix();


		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glTranslatef(0.0, 0.0, 0.93);
		glRotatef(spin, 0.0, 1.0, 0.0);
		glRotatef(45, 0.0, 1.0, 0.0);
		CubeMesh q11 = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
		drawCube(&q11);
		glPopMatrix();

		glPushMatrix();
		glRotatef(fangle, 0.0, 1.0, 0.0);
		glTranslatef(0.0, 0.0, -0.93);
		glRotatef(spin, 0.0, 1.0, 0.0);
		CubeMesh q12 = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
		drawCube(&q12);
		glPopMatrix();
		glPopMatrix();
	}

	glTranslatef(bx, by, bz);
	glRotatef(45, 0.0, 1.0, 0.0);
	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	glRotatef(90, 0.0, 1.0, 0.0);
	CubeMesh q7a = newCube(0.0, 0.0, 0.0, 0.1, 0.1, 1.0, 3.0);
	drawCube(&q7a);
	glPopMatrix();

	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	CubeMesh q8a = newCube(0.0, 0.0, 0.0, 0.1, 0.1, 1.0, 3.0);
	drawCube(&q8a);
	glPopMatrix();


	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	glTranslatef(0.93, 0.0, 0.0);
	glRotatef(spin, 0.0, 1.0, 0.0);
	glRotatef(50, 0.0, 1.0, 0.0);
	CubeMesh q9a = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
	drawCube(&q9a);
	glPopMatrix();


	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	glTranslatef(-0.93, 0.0, 0.0);
	glRotatef(spin, 0.0, 1.0, 0.0);
	glRotatef(30, 0.0, 1.0, 0.0);
	CubeMesh q10a = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
	drawCube(&q10a);
	glPopMatrix();


	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.0, 0.93);
	glRotatef(spin, 0.0, 1.0, 0.0);
	glRotatef(45, 0.0, 1.0, 0.0);
	CubeMesh q11a = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
	drawCube(&q11a);
	glPopMatrix();

	glPushMatrix();
	glRotatef(fangle2, 0.0, 1.0, 0.0);
	glTranslatef(0.0, 0.0, -0.93);
	glRotatef(spin, 0.0, 1.0, 0.0);
	CubeMesh q12a = newCube(0.0, 0.1, 0.0, 0.3, 0.01, 0.04, 4.0);
	drawCube(&q12a);
	glPopMatrix();
	


	glFlush();
    glutSwapBuffers();   // Double buffering, swap buffers
}


// Callback, called at initialization and whenever user resizes the window.
void reshape(int w, int h)
{
	vHeight = h;
	vWidth = w;
}




// Mouse button callback - use only if you want to 
void mouse(int button, int state, int x, int y)
{
    currentButton = button;

    switch (button)
    {
    case GLUT_LEFT_BUTTON:
        if (state == GLUT_DOWN)
        {
			lmpx = x;
			lmpy = y;
        }
        break;
    case GLUT_RIGHT_BUTTON:
        if (state == GLUT_DOWN)
        {
            
        }
        break;
    default:
        break;
    }

    display();   // Trigger a window redisplay
}


// Mouse motion callback
void mouseMotionHandler(int xMouse, int yMouse)
{
    if (currentButton == GLUT_LEFT_BUTTON)
    {
		mouseX = xMouse - lmpx;
		mouseY = yMouse - lmpy;
		angleX = angleX + mouseX * 0.01;
		if (angleX < 0)
			angleX += 360;
		else if (angleX > 360)
			angleX -= 360;
		angleY = angleY + mouseY * 0.01;
		if (angleY < 0)
			angleY += 360;
		else if (angleY > 360)
			angleY -= 360;
		
    }
	
    display();   // Trigger a window redisplay
}




void tr(void)
{
	if (flying == GL_TRUE)
	{
		tspd += 0.01;
		fangle -= tspd;
		if (fangle < 0.0)
			fangle += 360.0;
		glutPostRedisplay();
	}
}


void tl(void)
{
	if (flying == GL_TRUE) {
		tspd += 0.01;
		fangle += tspd;
		if (fangle > 360.0)
			fangle -= 360.0;
		glutPostRedisplay();
	}
}

void spinDisplay(void)
{
	flying = GL_TRUE;
	spin += 5;
	if (spin > 360.0)
		spin -= 360.0;
	mx -= sin(fangle / 57.295780) * mspd;
	mz -= cos(fangle / 57.295780) * mspd;
	
	if ((bx - tmpX) < -0.5) {
		bx += 0.02;
	}
	else if ((bx - tmpX) > 0.5) {
		bx -= 0.02;
	}
	else if ((bz - tmpZ) > 0.5) {
		bz -= 0.02;
	}
	else if ((bz - tmpZ) < -0.5) {
		bz += 0.02;
	}
	else {
		tmpX = (rand() % 38) - 19;
		tmpZ = (rand() % 38) - 19;
	}


	if (my > bbox[17].min.y - 1.0 && my < bbox[17].max.y - 1.0 && mx < bbox[17].max.x && mx > bbox[17].min.x &&  mz < bbox[17].max.z &&  mz > bbox[17].min.z) {
		printf("hit\n");
		bx = (rand() % 56) - 28;
		bz = (rand() % 56) - 28;
	}
	glutPostRedisplay();
}





void f(void)
{
	if (flying == GL_TRUE) {
		for (int i = 0; i < 17; i++) {
			if (ay < bbox[i].max.y - 1.0 && ax < bbox[i].max.x && ax > bbox[i].min.x &&  az < bbox[i].max.z &&  az > bbox[i].min.z) {
				crush = GL_TRUE;
				break;
			}
		}
		spd += 0.005;
		ax -= sin(fangle / 57.295780) * spd;
		az -= cos(fangle / 57.295780) * spd;

		glutPostRedisplay();
	}
}

void b(void)
{
	if (flying == GL_TRUE) {
		for (int i = 0; i < 17; i++) {
			if (ay < bbox[i].max.y - 1.0 && ax < bbox[i].max.x && ax > bbox[i].min.x &&  az < bbox[i].max.z &&  az > bbox[i].min.z) {
				crush = GL_TRUE;
				break;
			}
		}
		spd += 0.005;
		ax += sin(fangle / 57.295780) * spd;
		az += cos(fangle / 57.295780) * spd;
		glutPostRedisplay();
	}
}

void u(void)
{
	if (flying == GL_TRUE) {
		ay += 0.1;
		glutPostRedisplay();
	}
}

void d(void)
{
	if (flying == GL_TRUE) {
		for (int i = 0; i < 17; i++) {
			if (ay < bbox[i].max.y - 1.0 && ax < bbox[i].max.x && ax > bbox[i].min.x &&  az < bbox[i].max.z &&  az > bbox[i].min.z)  {
				crush = GL_TRUE;
				break;
			}
		}
		if (ay >= -0.5)
			ay -= 0.1;
		else
			crush = GL_TRUE;
		glutPostRedisplay();
	}
}

void keyboard(unsigned char key, int x, int y)
{
	if (crush == GL_FALSE)
	switch (key)
	{
	case 's':
		glutIdleFunc(spinDisplay);
		break;
	case 'f':
		f();
		break;
	case 'b':
		b();
		break;
	default:
		break;
	}
}

void specialKey(int key, int x, int y)
{
	if (crush == GL_FALSE)
	switch (key)
	{
	case GLUT_KEY_UP:
		u();
		break;
	case GLUT_KEY_DOWN:
		d();
		break;
	case GLUT_KEY_LEFT:
		tl();
		break;
	case GLUT_KEY_RIGHT:
		tr();
		break;
	case GLUT_KEY_F1:
		rang ++;
		if (rang > 5)
			rang = 5;
		break;
	case GLUT_KEY_F2:
		rang --;
		if (rang < 0)
			rang = 0;
		break;
	case GLUT_KEY_F3:
		cangle++;
		if (cangle > 90)
			cangle = 90;
		break;
	case GLUT_KEY_F4:
		cangle--;
		if (cangle < 0)
			cangle = 0;
		break;
	case GLUT_KEY_F5:
		mx = ax;
		my = ay;
		mz = az;
		break;
	case GLUT_KEY_F6:
		printf("%s", "\nPress 's' to start\nPress 'f' to move foward\nPress 'b' to move backward\nPress up key to move up\nPress down key to move down\nPress left key to turn left\nPress right key to turn right\nPress f1 to zoom in the camera\nPress f2 to zoom out the camera\nPress f3 to tilt down the camera angle\nPress f4 to tilt up the camera angle\nPress f5 to shoot the missle");
		break;
	}glutPostRedisplay();
}
void keyUp(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'f':
		spd = 0.0;
		tangle = 0.0;
		break;
	case 'b':
		spd = 0.0;
		tangle = 0.0;
		break;
	}
}

void specialKeyUp(int key, int x, int y) {
	switch (key)
	{
	case GLUT_KEY_LEFT:
		tspd = 0.0;
		break;
	case GLUT_KEY_RIGHT:
		tspd = 0.0;
		break;
	}

	glutPostRedisplay();
}