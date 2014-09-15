// Real-Time Visual Feedback Display
#define _CRT_SECURE_NO_WARNINGS 1;
#include "stdafx.h"

#include "RTProtocol.h"
//#define QTM_RT_SERVER_BASE_PORT 22222
int QTM_RT_SERVER_BASE_PORT = 22222;

#include <windows.h>  // for MS Windows
#include <GL/glut.h>  // GLUT, include glu.h and gl.h
#include <math.h>     // Needed for sin, cos'
#include <float.h>
#include <stdio.h>
#include <conio.h>
#include <string>
#include <iostream>
#include <ctime>
#include <nuiapi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>
#include <NuiSkeleton.h>
#include <objbase.h>
#include <stdlib.h>



#include <cstdlib> 

class Kinect {
	public: INuiSensor * sensor;
	public:	int initStatus;
	public:	HANDLE skeletonEvent;
	public: HANDLE colorEvent;
	public: HANDLE colorHandler;

};
HRESULT skeletonFrame;
_NUI_SKELETON_FRAME skelFrame;
Vector4 kinectZero;
Vector4 curLeft;
Vector4 curRight;
float zeroXL;
float zeroXR;
float zeroYL;
float zeroYR;


float MAX_SPEED = 2.0;
float DEFAULT_SPEED = 1.0;
float DEFAULT_NSTEPS = 100;
bool RECORDING = true;

bool rePause = false;


std::clock_t start;
double duration;

#define PI 3.14159265f
const int font=(int)GLUT_BITMAP_TIMES_ROMAN_24;

int Score = 0;

#include "Sequence.h"
#include "treadmill-remote.h"

char title[] = "";
int windowWidth  = 640;     // Windowed mode's width
int windowHeight = 480;     // Windowed mode's height
int windowPosX   = 50;      // Windowed mode's top-left corner x
int windowPosY   = 50;      // Windowed mode's top-left corner y

bool pause = 0;
bool visible = 1; //Endpoint feedback visible
bool targetVisible = 1; //Target visible

CRTProtocol				poRTProtocol;
CRTPacket*				pRTPacket;
CRTPacket::EPacketType	eType;
CRTPacket::EEvent       eEvent;		

    unsigned int                    nCaptureFrequency;
    float                           fCaptureTime;
    bool                            bStartOnExtTrig;
    CRTProtocol::EProcessingActions eProcessingActions;

	unsigned int nCount = 0;
	float  fX[14], fY[14], fZ[14];
	float  fX2[14], fY2[14], fZ2[14]; //no label
	unsigned int nId = 0;
	int nFrameNumber = 0;

GLfloat Speed     = 2.0f;          //Treadmill speed
GLfloat VSpeed[2] = {0,0};   //Virtual treadmill belt speeds (left, right)
double acc		  = .5;		   //Treadmill acceleration 


int MAX_NSTEPS = 100;

GLfloat EndPointRadius = 0.04f;  
static GLfloat EndPointX[2] = {0.0f,0.0f};
static GLfloat EndPointY[2] = {0.0f,0.0f};
static GLfloat EndPointVel = 100.0f;
GLfloat TargetSize = 0.08f;
GLfloat TargetX = 0.15f;
int nTarget = 2;
GLfloat *TargetY = new GLfloat[nTarget];
int refreshInterval = 10;      // Refresh period in milliseconds
float widthScale = 0.7;

FILE *stream;
FILE *stream2;

enum Foot
{
	RIGHT_FOOT = 1,
	LEFT_FOOT  = 0
};
Foot eFoot = RIGHT_FOOT;

enum MarkerSet 
{
	TWO_MARKER      = 0,
	TWELVE_MARKER   = 1,
	FOURTEEN_MARKER = 2
};

#include <time.h>
enum Level
{
	Easy    = -1,
	Normal  =  0,
	Hard    =  1,
	Hardest =  2
};
Level eLevel = Normal;

float errorMarginX = 0.06f;
float errorMarginY = 0.06f;

float ElaspedTimeInZone = 0.0f;
float ElaspedTimeTargetDisplay = 0.0f;
float ElaspedTimeFootContact = 0.0f;
float errorX = 0.0f;
float errorY = 0.0f;
static bool inZone = false;

int mySequence[2] = {3,1};
static CSequence sequence = 'a';
static int nStep = -5;
float stepLength = 0.5;

bool VARIABLE = true;

// Projection clipping area
GLdouble clipAreaXLeft, clipAreaXRight, clipAreaYBottom, clipAreaYTop;

bool fullScreenMode = false; // Full-screen or windowed mode?

/* Initialize OpenGL Graphics */
void initGL() {
	glClearColor(0.0, 0.0, 0.0, 1.0); // Set background (clear) color to black
}

void SortMarkers(float *anArray, float *anArray2, float *anArray3, int nSize)
{
    using namespace std;
    for (int nStartIndex= 0; nStartIndex < nSize; nStartIndex++)
    {
        int nBestIndex = nStartIndex;
 
        // Search through every element starting at nStartIndex+1
        for (int nCurrentIndex = nStartIndex + 1; nCurrentIndex < nSize; nCurrentIndex++)
        {
            // Note that we are using the user-defined comparison here
            if (anArray[nCurrentIndex] < anArray[nBestIndex]) // COMPARISON DONE HERE
				//if (abs(anArray[nCurrentIndex]) < 1500)
					//if (abs(anArray3[nCurrentIndex]) < 100)
                nBestIndex = nCurrentIndex;
        }
 
        // Swap our start element with our best element
        swap(anArray[nStartIndex], anArray[nBestIndex]);
		swap(anArray2[nStartIndex], anArray2[nBestIndex]);   
		swap(anArray3[nStartIndex], anArray3[nBestIndex]);
    }
}

void kinectGetEndpoint()
{
	float tempPos = 0;
	switch(eFoot) 
	{
	case RIGHT_FOOT:
		tempPos = EndPointY[1];
		break;
	case LEFT_FOOT:
		tempPos = EndPointY[0];
		break;
	}

		
	

		if (!(curRight.z != curRight.z)) //not is NaN		
		{
			EndPointY[0] = -(curRight.z-zeroYR) * VSpeed[0];	
			EndPointX[0] = -abs(TargetX); //(fY[7]-285)/1000/widthScale;
		}
		//else
		//{
		//	findLToe(fX2,fY2,fZ2,nSize);
		//	EndPointY[0] = -fX2[0]/1000 * VSpeed[0];;
		//	EndPointX[0] = -abs(TargetX);
		//}
		
		if (!(curLeft.z != curLeft.z)) //not is NaN
		{
			EndPointY[1] = -(curLeft.z-zeroYL) * VSpeed[1];	
			EndPointX[1] =  abs(TargetX); //(fY[14]-285)/1000/widthScale; 
		}		
		//else
		//{
		//	findRToe(fX2,fY2,fZ2,nSize);
		//	EndPointY[1] = -fX2[0]/1000 * VSpeed[0];;
		//	EndPointX[1] = abs(TargetX);
		//}
	


	switch(eFoot) {
	case RIGHT_FOOT:
		EndPointVel = EndPointY[1] - tempPos + VSpeed[1]*Speed*0.0002778*refreshInterval*2;
		break;
	case LEFT_FOOT:
		EndPointVel = EndPointY[0] - tempPos + VSpeed[0]*Speed*0.0002778*refreshInterval*2;
		break;
	}

	if(abs(EndPointVel) < .005)
		ElaspedTimeFootContact = ElaspedTimeFootContact + refreshInterval;
	else
		ElaspedTimeFootContact = 0;
}


/* Fetch Real Time Marker Data */
void GetEndpoint() {

	float tempPos = 0;
	switch(eFoot) 
	{
	case RIGHT_FOOT:
		tempPos = EndPointY[1];
		break;
	case LEFT_FOOT:
		tempPos = EndPointY[0];
		break;
	}

	bool bDataAvailable; //added May 2014

	poRTProtocol.Read3DSettings(bDataAvailable);
	//float  fX[14], fY[14], fZ[14];
	//float  fX2[14], fY2[14], fZ2[14]; //no label
	//unsigned int nId = 0;
	//int nFrameNumber = pRTPacket->GetFrameNumber();
	nCount = poRTProtocol.Get3DLabeledMarkerCount();
	if (nCount == 0) //no AIM
	{
		poRTProtocol.GetCurrentFrame(CRTProtocol::cComponent3dNoLabels);
		if ((poRTProtocol.ReceiveRTPacket(eType, true)) && (CRTPacket::PacketData))
		{
			pRTPacket = poRTProtocol.GetRTPacket();
			for (unsigned int i = 0; i < pRTPacket->Get3DNoLabelsMarkerCount(); i++)
			{
				pRTPacket->Get3DNoLabelsMarker(i, fX[i], fY[i], fZ[i], nId);
			}
			nFrameNumber = pRTPacket->GetFrameNumber();

			int nSize = pRTPacket->Get3DNoLabelsMarkerCount();
			SortMarkers(fY, fX, fZ, nSize);
			EndPointY[1] = -fX[nSize-1]/1000 * VSpeed[1];	
			EndPointX[1] =  abs(TargetX); //(fY[nSize-1]-285)/1000/widthScale; 
			EndPointY[0] = -fX[0]/1000 * VSpeed[0];	
			EndPointX[0] = -abs(TargetX); //(fY[0]-285)/1000/widthScale; 
		}
	}
	else if (nCount == 14) //14 marker set
	{
		poRTProtocol.GetCurrentFrame(CRTProtocol::cComponent3d);
		if ((poRTProtocol.ReceiveRTPacket(eType, true)) && (CRTPacket::PacketData))
		{
			pRTPacket = poRTProtocol.GetRTPacket();
			for (unsigned int i = 0; i < pRTPacket->Get3DMarkerCount(); i++)
			{
				pRTPacket->Get3DMarker(i, fX[i], fY[i], fZ[i]);
			}
			nFrameNumber = pRTPacket->GetFrameNumber();
		}

		//pRTPacket = poRTProtocol.GetRTPacket();
		//for (unsigned int i = 0; i < pRTPacket->Get3DNoLabelsMarkerCount(); i++)
		//{
		//	pRTPacket->Get3DNoLabelsMarker(i, fX2[i], fY2[i], fZ2[i], nId);
		//}
		//int nSize = pRTPacket->Get3DNoLabelsMarkerCount();

		if (!(fY[6] != fY[6])) //not is NaN		
		{
			EndPointY[0] = -fX[6]/1000 * VSpeed[0];	
			EndPointX[0] = -abs(TargetX); //(fY[7]-285)/1000/widthScale;
		}
		//else
		//{
		//	findLToe(fX2,fY2,fZ2,nSize);
		//	EndPointY[0] = -fX2[0]/1000 * VSpeed[0];;
		//	EndPointX[0] = -abs(TargetX);
		//}
		
		if (!(fY[13] != fY[13])) //not is NaN
		{
			EndPointY[1] = -fX[13]/1000 * VSpeed[1];	
			EndPointX[1] =  abs(TargetX); //(fY[14]-285)/1000/widthScale; 
		}		
		//else
		//{
		//	findRToe(fX2,fY2,fZ2,nSize);
		//	EndPointY[1] = -fX2[0]/1000 * VSpeed[0];;
		//	EndPointX[1] = abs(TargetX);
		//}
	}


	switch(eFoot) {
	case RIGHT_FOOT:
		EndPointVel = EndPointY[1] - tempPos + VSpeed[1]*Speed*0.0002778*refreshInterval*2;
		break;
	case LEFT_FOOT:
		EndPointVel = EndPointY[0] - tempPos + VSpeed[0]*Speed*0.0002778*refreshInterval*2;
		break;
	}

	if(abs(EndPointVel) < .005)
		ElaspedTimeFootContact = ElaspedTimeFootContact + refreshInterval;
	else
		ElaspedTimeFootContact = 0;
}

void kinectPrintResults()
{
	fprintf( stream2, "%d\t", nStep);
	fprintf( stream2, "%d\t", sequence.getElement(nStep));
	fprintf( stream2, "%d\t", (int) eFoot);
	fprintf( stream2, "%1.4f\t", TargetY[0]);
	fprintf( stream2, "%1.4f\t", EndPointY[eFoot]);
	fprintf( stream2, "%1.4f\t", EndPointY[!eFoot]);
	fprintf( stream2, "%1.4f\t", errorY);
	fprintf( stream2, "%1.1f\t", VSpeed[0]*Speed);
	fprintf( stream2, "%1.1f\t", VSpeed[1]*Speed);
	fprintf( stream2, "%d\t", Score);
	fprintf( stream2, "%d\n", eLevel);
}

void printResults()
{	
	fprintf( stream2, "%d\t", nStep);
	fprintf( stream2, "%d\t", sequence.getElement(nStep));
	fprintf( stream2, "%d\t", (int) eFoot);
	fprintf( stream2, "%1.4f\t", TargetY[0]);
	fprintf( stream2, "%1.4f\t", EndPointY[eFoot]);
	fprintf( stream2, "%1.4f\t", EndPointY[!eFoot]);
	fprintf( stream2, "%1.4f\t", errorY);
	fprintf( stream2, "%1.1f\t", VSpeed[0]*Speed);
	fprintf( stream2, "%1.1f\t", VSpeed[1]*Speed);
	fprintf( stream2, "%d\t", Score);
	fprintf( stream2, "%d\n", eLevel);
}

void printData()
{
	//fprintf(stream, "%d,", nFrameNumber);
	//fprintf(stream, "%d,", nStep);
	//fprintf(stream, "%d,", sequence.getElement(nStep));
	fprintf(stream, "%d,", Score);
	fprintf(stream, "%1.4f,", TargetX);
	fprintf(stream, "%1.4f,", TargetY[0]);
	fprintf(stream, "%1.4f,", TargetY[1]);
	fprintf(stream, "%1.4f,", EndPointX[eFoot]);
	fprintf(stream, "%1.4f,", EndPointY[eFoot]);
	fprintf(stream, "%1.4f,", EndPointX[!eFoot]);
	fprintf(stream, "%1.4f,", EndPointY[!eFoot]);
	fprintf(stream, "%1.1f,", VSpeed[0]*Speed);
	fprintf(stream, "%1.1f,", VSpeed[1]*Speed);
	for (unsigned int i = 0; i < nCount; i++)
	{
		fprintf(stream, "%1.5f,%1.5f,%1.5f,", fX[i], fY[i], fZ[i]);
	}
	fprintf(stream, "\n");
}

void updateTarget() {	

	float scale;

	if ((nStep < 1) & (ElaspedTimeTargetDisplay == 0))
	{
		GetEndpoint();
		TargetY[0] = EndPointY[eFoot] + 0.0f;
		for (int i=0; i < nTarget; ++i)
		{
			switch (sequence.getElement(nStep+i))
			{
			case 1:
				scale = 0.7; //short
				break;
			case 2:
				scale = 1.0; //normal
				break;
			case 3:
				scale = 1.2; //long
				break;			
			}
			TargetY[i+1] = TargetY[i] + scale*stepLength*(VSpeed[(i+1) % 2]/abs(VSpeed[(i+1) % 2]));
		}
	}

	ElaspedTimeTargetDisplay = ElaspedTimeTargetDisplay + refreshInterval;

	bool TargetVisible = 0;
	if ((TargetY[0] < 0.5) && TargetY[0] > -0.5)
		TargetVisible = 1;

	float bodyPos = (EndPointY[0] + EndPointY[1])/2;

	if ((ElaspedTimeInZone > 20)  ||  (TargetY[0]/VSpeed[eFoot] < -0.6)  ||  //successful hit or too far back
		((ElaspedTimeTargetDisplay > 450) & (ElaspedTimeFootContact > 50) & (TargetY[0] - EndPointY[!eFoot] < .5*stepLength)) || 
		((VSpeed[eFoot] > 0) & (abs(EndPointY[!eFoot]) < 0.5) & (TargetY[0] < EndPointY[!eFoot])) )   
	//if((ElaspedTimeInZone > 50) ||  ((TargetY[0]/VSpeed[eFoot])<-0.6) || (TargetVisible & (ElaspedTimeTargetDisplay > 450) & (ElaspedTimeFootContact > 100)))
	{
		printResults();

		nStep++;
		switch (eFoot) 
		{
		case RIGHT_FOOT:
			eFoot = LEFT_FOOT;
			break;
		case LEFT_FOOT: 
			eFoot = RIGHT_FOOT;
			break;
		}
		TargetX = -TargetX;			

		for (int i = 0; i < nTarget-1 ; i++)
			TargetY[i] = TargetY[i+1];
		switch (sequence.getElement(nStep))
		{
		case 1:
			scale = 0.7; //short
			break;
		case 2:
			scale = 1.0; //normal
			break;
		case 3:
			scale = 1.2; //long
			break;
		}

		float d = 0;
		if (VARIABLE)
		{
			 d = (float)(rand() % 20 - 10) / 100;
		}
		
		TargetY[nTarget-1] = TargetY[nTarget-1] + (scale+d)*stepLength*(VSpeed[eFoot]/VSpeed[!eFoot]);
		

		inZone = false;
		ElaspedTimeInZone = 0;
		ElaspedTimeTargetDisplay = 0;

		poRTProtocol.GetState(eEvent);
		if (eEvent == CRTPacket::EventCaptureStarted) {
			char eventID[12];
			//switch (sequence.getElement(nStep))
			//{
			//case 1:
			//	strcpy_s(eventID, "short"); //short
			//	break;
			//case 2:
			//	strcpy_s(eventID, "normal"); //normal
			//	break;
			//case 3:
			//	strcpy_s(eventID, "long"); //long
			//	break;
			//}
			strcpy_s(eventID, "TargetOff"); //next Target
			poRTProtocol.SetQTMEvent(eventID);
		}
	}

	for (int i = 0; i < nTarget; i++) 
	{
		GLfloat s;
		if (eFoot)
			s = Speed * VSpeed[(i+1) % 2];
		else
			s = Speed * VSpeed[ i    % 2];
		TargetY[i] = TargetY[i] - s*0.0002778*refreshInterval*2;
	}

}

void kinectUpdateTarget() {	

	float scale;

	if ((nStep < 1) & (ElaspedTimeTargetDisplay == 0))
	{
		kinectGetEndpoint();
		TargetY[0] = EndPointY[eFoot] + 0.0f;
		for (int i=0; i < nTarget; ++i)
		{
			switch (sequence.getElement(nStep+i))
			{
			case 1:
				scale = 0.7; //short
				break;
			case 2:
				scale = 1.0; //normal
				break;
			case 3:
				scale = 1.2; //long
				break;			
			}
			TargetY[i+1] = TargetY[i] + scale*stepLength*(VSpeed[(i+1) % 2]/abs(VSpeed[(i+1) % 2]));
		}
	}

	ElaspedTimeTargetDisplay = ElaspedTimeTargetDisplay + refreshInterval;

	bool TargetVisible = true;
	

	float bodyPos = (EndPointY[0] + EndPointY[1])/2;

	if ((ElaspedTimeInZone > 20)  ||  (TargetY[0]/VSpeed[eFoot] < -0.6)  ||  //successful hit or too far back
		((ElaspedTimeTargetDisplay > 450) & (ElaspedTimeFootContact > 50) & (TargetY[0] - EndPointY[!eFoot] < .5*stepLength)) || 
		((VSpeed[eFoot] > 0) & (abs(EndPointY[!eFoot]) < 0.5) & (TargetY[0] < EndPointY[!eFoot])) )   
	//if((ElaspedTimeInZone > 50) ||  ((TargetY[0]/VSpeed[eFoot])<-0.6) || (TargetVisible & (ElaspedTimeTargetDisplay > 450) & (ElaspedTimeFootContact > 100)))
	{
		printResults();

		nStep++;
		switch (eFoot) 
		{
		case RIGHT_FOOT:
			eFoot = LEFT_FOOT;
			break;
		case LEFT_FOOT: 
			eFoot = RIGHT_FOOT;
			break;
		}
		TargetX = -TargetX;			

		for (int i = 0; i < nTarget-1 ; i++)
			TargetY[i] = TargetY[i+1];
		switch (sequence.getElement(nStep))
		{
		case 1:
			scale = 0.7; //short
			break;
		case 2:
			scale = 1.0; //normal
			break;
		case 3:
			scale = 1.2; //long
			break;
		}

		float d = 0;
		if (VARIABLE)
		{
			 d = (float)(rand() % 20 - 10) / 100;
		}
		
		TargetY[nTarget-1] = TargetY[nTarget-1] + (scale+d)*stepLength*(VSpeed[eFoot]/VSpeed[!eFoot]);
		

		inZone = false;
		ElaspedTimeInZone = 0;
		ElaspedTimeTargetDisplay = 0;

		//poRTProtocol.GetState(eEvent);
		if (eEvent == CRTPacket::EventCaptureStarted) {
			char eventID[12];
			//switch (sequence.getElement(nStep))
			//{
			//case 1:
			//	strcpy_s(eventID, "short"); //short
			//	break;
			//case 2:
			//	strcpy_s(eventID, "normal"); //normal
			//	break;
			//case 3:
			//	strcpy_s(eventID, "long"); //long
			//	break;
			//}
			strcpy_s(eventID, "TargetOff"); //next Target
			poRTProtocol.SetQTMEvent(eventID);
		}
	}

	for (int i = 0; i < nTarget; i++) 
	{
		GLfloat s;
		if (eFoot)
			s = Speed * VSpeed[(i+1) % 2];
		else
			s = Speed * VSpeed[ i    % 2];
		TargetY[i] = TargetY[i] - s*0.0002778*refreshInterval*2;
	}

}

void updateScore() {
	errorX = EndPointX[eFoot] - TargetX;
	errorY = EndPointY[eFoot] - TargetY[0];
	if (abs(errorX) < errorMarginX && abs(errorY) < errorMarginY && EndPointVel<0.01)
	{
		inZone = true;
		ElaspedTimeInZone = ElaspedTimeInZone + refreshInterval;
	}
	else
	{
		inZone = false;
		ElaspedTimeInZone = 0;
	}

	if (ElaspedTimeInZone > 20) // changed from 50 to 20 in v.1.4 for faster walking speed
		++Score;
}

void drawStrokeText(char *string, float x, float y, float z){
	char *c;
	glPushMatrix();
	glScalef(0.001f,0.001f,0.0f);
	glTranslatef(x*1000, y*1000, z*1000);
	for (c=string; *c != '\0'; c++)
	{
		glutStrokeCharacter(GLUT_STROKE_ROMAN, *c);
	}
	glPopMatrix();
} 

void drawStrokeTextLarge(char *string, float x, float y, float z){
	char *c;
	glPushMatrix();
	glScalef(0.01f,0.01f,0.0f);
	glTranslatef(x*1000, y*1000, z*1000);
	for (c=string; *c != '\0'; c++)
	{
		glutStrokeCharacter(GLUT_STROKE_ROMAN, *c);
	}
	glPopMatrix();
} 

/* Callback handler for window re-paint event */
void kinectDisplay() {
	if(rePause)
	{
		TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
		rePause = !rePause;
	}
	
	glClear(GL_COLOR_BUFFER_BIT);  // Clear the color buffer
	glMatrixMode(GL_MODELVIEW);    // To operate on the model-view matrix
	glLoadIdentity();   // Reset model-view matrix
	bool trackOK = false;
	skeletonFrame = NuiSkeletonGetNextFrame(5000, &skelFrame);
		if(SUCCEEDED(skeletonFrame))
		{
			for(int i = 0; i < NUI_SKELETON_COUNT; i++)
			{
				
				NUI_SKELETON_DATA dat = skelFrame.SkeletonData[i];
				if(dat.eTrackingState==NUI_SKELETON_TRACKED)
				{
					std::cout<<"\n Found a skellington";
					Vector4 left = dat.SkeletonPositions[18];
					Vector4 right = dat.SkeletonPositions[14];
					curLeft = left;
					curRight = right;
					std::cout<<left.x;
					std::cout<<"\n";
					std::cout<<right.x;
					trackOK=true;
				}
			}
		}
		if(!trackOK)
		{
			std::cout<<"Kinect tracking error";
			scanf("%f");
		}
		
	
	if (nStep > MAX_NSTEPS) {
		// FINISHED
		glColor3f(0,1,0);
		drawStrokeText("Finished. Your score is", -0.8, 0.0, 0);
		glColor3f(0,1,0);
		char buffer[33];
		_itoa_s(Score,buffer,10);
		drawStrokeText(buffer, -0.05, -0.2, 0);
	}
	else if (nStep < 0) {
		// COUNTDOWN
		glColor3f(1,1,1);
		char buffer[33];
		_itoa_s(-nStep,buffer,10);
		drawStrokeTextLarge(buffer, -0.05, -0.05, 0);	
		duration = (double) (( std::clock() - start ) / CLOCKS_PER_SEC);
		if (duration > 1)
		{
			nStep++;
			start = std::clock();			
			duration = (double) (( std::clock() - start ) / CLOCKS_PER_SEC);
		}
	}
	else if (pause){
		// PAUSED
		glColor3f(0,1,0);
		drawStrokeText("PAUSED", -0.15, 0.0, 0);
		
		TREADMILL_setSpeed(0,0,acc);
		rePause = true;

	}
	else {
		// Treadmill
		glBegin(GL_QUADS);
		glColor3f(0.2f, 0.2f, 0.2f); 
		glVertex2f(-0.50f, -0.9f);     
		glVertex2f(-0.02f, -0.9f );     
		glVertex2f(-0.02f,  1.0f );
		glVertex2f(-0.50f,  1.0f);
		glVertex2f(-0.50f, -0.9f);
		glEnd();

		glBegin(GL_QUADS);
		glColor3f(0.2f, 0.2f, 0.2f); 
		glVertex2f( 0.02f, -0.9f);     
		glVertex2f( 0.50f, -0.9f );     
		glVertex2f( 0.50f,  1.0f );
		glVertex2f( 0.02f,  1.0f);
		glVertex2f( 0.02f, -0.9f);
		glEnd();

		// Target
		if (true){
			for (int i = 0; i < nTarget; i++)
			{
				glPushMatrix();
				glTranslatef(TargetX*pow(-1,i), TargetY[i], 0.0f);
				glBegin(GL_LINE_STRIP);
				if (i == 0)
				{
					if (!inZone)
						glColor3f(1.0f, 0.0f, 0.0f); 
					else {
						if (ElaspedTimeInZone > 100)
							glColor3f(0.0f, 0.0f, 0.0f); 
						else
							glColor3f(1.0f, 1.0f, 1.0f);
					}
				}
				else 
					glColor3f(0.5f, 0.5f, 0.5f); 
				glVertex2f(-1.0f * TargetSize, -1.0f * TargetSize);     
				glVertex2f(1.0f * TargetSize, -1.0f * TargetSize);     
				glVertex2f(1.0f * TargetSize, 1.0f * TargetSize);
				glVertex2f(-1.0f * TargetSize, 1.0f * TargetSize);
				glVertex2f(-1.0f * TargetSize, -1.0f * TargetSize);
				glEnd();
				glPopMatrix();				
			}
		}
		
		// Fetch Real Time Marker Data
		kinectGetEndpoint();
		if (true) {
			glPushMatrix();
			glTranslatef(EndPointX[eFoot], EndPointY[eFoot], 0.0f);  // Translate EndPoint to (xPos, yPos)
			// Use triangular segments to form a circle
			glBegin(GL_TRIANGLE_FAN);
			glColor3f(0.0f, 0.0f, 1.0f);  // Blue
			glVertex2f(0.0f, 0.0f);       // Center of circle
			int numSegments = 100;
			GLfloat angle;
			for (int i = 0; i <= numSegments; i++) { // Last vertex same as first vertex
				angle = i * 2.0f * PI / numSegments;  // 360 deg for all segments
				glVertex2f(cos(angle) * EndPointRadius, sin(angle) * EndPointRadius);
			}
			glEnd();
			glPopMatrix();
		}
				
		glColor3f(0,1,0);
		char buffer[33];
		_itoa_s(Score,buffer,10);
		if (targetVisible & visible) {
			drawStrokeText(buffer, 0.8, 0.8, 0);
		}

		glColor3f(0.5,0.5,0.5);
		int n;
		n=sprintf_s(buffer, "%1.1f", Speed * VSpeed[1]);
		drawStrokeText(buffer, 0.8, -0.8, 0);

		drawStrokeText("Level ", -0.95, 0.8, 0);
		n=sprintf_s(buffer, "%1d", eLevel+2);
		drawStrokeText(buffer, -0.6, 0.8, 0);

		updateScore();
		kinectUpdateTarget();
		//printData();
	}
	glutSwapBuffers();  // Swap front and back buffers (of double buffered mode)
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT);  // Clear the color buffer
	glMatrixMode(GL_MODELVIEW);    // To operate on the model-view matrix
	glLoadIdentity();   // Reset model-view matrix
	
	
	if (nStep > MAX_NSTEPS) {
		// FINISHED
		glColor3f(0,1,0);
		drawStrokeText("Finished. Your score is", -0.8, 0.0, 0);
		glColor3f(0,1,0);
		char buffer[33];
		_itoa_s(Score,buffer,10);
		drawStrokeText(buffer, -0.05, -0.2, 0);
	}
	else if (nStep < 0) {
		// COUNTDOWN
		glColor3f(1,1,1);
		char buffer[33];
		_itoa_s(-nStep,buffer,10);
		drawStrokeTextLarge(buffer, -0.05, -0.05, 0);	
		duration = (double) (( std::clock() - start ) / CLOCKS_PER_SEC);
		if (duration > 1)
		{
			nStep++;
			start = std::clock();			
			duration = (double) (( std::clock() - start ) / CLOCKS_PER_SEC);
		}
	}
	else if (pause){
		// PAUSED
		glColor3f(0,1,0);
		drawStrokeText("PAUSED", -0.15, 0.0, 0);
	}
	else {
		// Treadmill
		glBegin(GL_QUADS);
		glColor3f(0.2f, 0.2f, 0.2f); 
		glVertex2f(-0.50f, -0.9f);     
		glVertex2f(-0.02f, -0.9f );     
		glVertex2f(-0.02f,  1.0f );
		glVertex2f(-0.50f,  1.0f);
		glVertex2f(-0.50f, -0.9f);
		glEnd();

		glBegin(GL_QUADS);
		glColor3f(0.2f, 0.2f, 0.2f); 
		glVertex2f( 0.02f, -0.9f);     
		glVertex2f( 0.50f, -0.9f );     
		glVertex2f( 0.50f,  1.0f );
		glVertex2f( 0.02f,  1.0f);
		glVertex2f( 0.02f, -0.9f);
		glEnd();

		// Target
		if (targetVisible) {
			for (int i = 0; i < nTarget; i++)
			{
				glPushMatrix();
				glTranslatef(TargetX*pow(-1,i), TargetY[i], 0.0f);
				glBegin(GL_LINE_STRIP);
				if (i == 0)
				{
					if (!inZone)
						glColor3f(1.0f, 0.0f, 0.0f); 
					else {
						if (ElaspedTimeInZone > 100)
							glColor3f(0.0f, 0.0f, 0.0f); 
						else
							glColor3f(1.0f, 1.0f, 1.0f);
					}
				}
				else 
					glColor3f(0.5f, 0.5f, 0.5f); 
				glVertex2f(-1.0f * TargetSize, -1.0f * TargetSize);     
				glVertex2f(1.0f * TargetSize, -1.0f * TargetSize);     
				glVertex2f(1.0f * TargetSize, 1.0f * TargetSize);
				glVertex2f(-1.0f * TargetSize, 1.0f * TargetSize);
				glVertex2f(-1.0f * TargetSize, -1.0f * TargetSize);
				glEnd();
				glPopMatrix();				
			}
		}
		
		// Fetch Real Time Marker Data
		GetEndpoint();
		if (visible) {
			glPushMatrix();
			glTranslatef(EndPointX[eFoot], EndPointY[eFoot], 0.0f);  // Translate EndPoint to (xPos, yPos)
			// Use triangular segments to form a circle
			glBegin(GL_TRIANGLE_FAN);
			glColor3f(0.0f, 0.0f, 1.0f);  // Blue
			glVertex2f(0.0f, 0.0f);       // Center of circle
			int numSegments = 100;
			GLfloat angle;
			for (int i = 0; i <= numSegments; i++) { // Last vertex same as first vertex
				angle = i * 2.0f * PI / numSegments;  // 360 deg for all segments
				glVertex2f(cos(angle) * EndPointRadius, sin(angle) * EndPointRadius);
			}
			glEnd();
			glPopMatrix();
		}
				
		glColor3f(0,1,0);
		char buffer[33];
		_itoa_s(Score,buffer,10);
		if (targetVisible & visible) {
			drawStrokeText(buffer, 0.8, 0.8, 0);
		}

		glColor3f(0.5,0.5,0.5);
		int n;
		n=sprintf_s(buffer, "%1.1f", Speed * VSpeed[1]);
		drawStrokeText(buffer, 0.8, -0.8, 0);

		drawStrokeText("Level ", -0.95, 0.8, 0);
		n=sprintf_s(buffer, "%1d", eLevel+2);
		drawStrokeText(buffer, -0.6, 0.8, 0);

		updateScore();
		updateTarget();
		printData();
	}
	glutSwapBuffers();  // Swap front and back buffers (of double buffered mode)
}
/* Call back when the windows is re-sized */
void reshape(GLsizei width, GLsizei height) {
	// Compute aspect ratio of the new window
	if (height == 0) height = 1;                // To prevent divide by 0
	GLfloat aspect = (GLfloat)width / (GLfloat)height;

	// Set the viewport to cover the new window
	glViewport(0, 0, width, height);

	// Set the aspect ratio of the clipping area to match the viewport
	glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
	glLoadIdentity();             // Reset the projection matrix
	if (width >= height) {
		clipAreaXLeft   = -1.0 * aspect;
		clipAreaXRight  = 1.0 * aspect;
		clipAreaYBottom = -1.0;
		clipAreaYTop    = 1.0;
	} else {
		clipAreaXLeft   = -1.0;
		clipAreaXRight  = 1.0;
		clipAreaYBottom = -1.0 / aspect;
		clipAreaYTop    = 1.0 / aspect;
	}
	gluOrtho2D(clipAreaXLeft, clipAreaXRight, clipAreaYBottom, clipAreaYTop);
}

/* Called back when the timer expired */
void Timer(int value) {
	glutPostRedisplay();    // Post a paint request to activate display()
	glutTimerFunc(refreshInterval, Timer, 0); // subsequent timer call at milliseconds
}

void kinectTheEnd(){
	time_t now = time(0); 
	char* dt = ctime(&now);
	fprintf(stream, "--------------------\n");
	fprintf(stream, "[Finished] %s\n", dt);
	fclose( stream );
	fclose( stream2 );
}

void theEnd() {	
	if (eEvent == CRTPacket::EventCaptureStarted)
		poRTProtocol.StopCapture();
	poRTProtocol.Disconnect();
	poRTProtocol.ReleaseControl();
	time_t now = time(0); 
	char* dt = ctime(&now);
	fprintf(stream, "--------------------\n");
	fprintf(stream, "[Finished] %s\n", dt);
	fclose( stream );
	fclose( stream2 );
}

/* Callback handler for normal-key event */
void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case 'p':  // (P)ause
		pause = !pause;
		break;
	case'q':  //force quit
		MAX_NSTEPS = nStep;
		break;
	case '+':  // increase step length
		stepLength += 0.025;
		VSpeed[0] = .5;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
		break;
	case '-':  // decrease step length
		stepLength -= 0.025;
		break;
	case 'a':
		
				if(VSpeed[0]<3)
				{
					VSpeed[0] += .05;
					TREADMILL_setSpeed(VSpeed[0], VSpeed[0], acc);
				}
				break;

	case 's':
		if(VSpeed[0]>0)
				{
					VSpeed[0] -= .05;
					TREADMILL_setSpeed(VSpeed[0], VSpeed[0], acc);
				}
		break;

	
	case 27:   // ESC key
		theEnd();
		exit(0);
		break;
	}
}

/* Callback handler for special-key event */
void specialKeys(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_HOME:  // HOME: Toggle between full-screen and windowed mode
		fullScreenMode = !fullScreenMode;         
		if (fullScreenMode) {                     
			windowPosX   = glutGet(GLUT_WINDOW_X); 
			windowPosY   = glutGet(GLUT_WINDOW_Y);
			windowWidth  = glutGet(GLUT_WINDOW_WIDTH);
			windowHeight = glutGet(GLUT_WINDOW_HEIGHT);
			glutFullScreen();                             
		} else {                                          
			glutReshapeWindow(windowWidth, windowHeight); 
			glutPositionWindow(windowPosX, windowPosX);   
		}
		
	case GLUT_KEY_PAGE_UP:  // Page-Up: increase speed
		Speed += 0.1f;
		break;
	case GLUT_KEY_PAGE_DOWN: // Page-Down: decrease speed
		Speed -= 0.1f;
		break;
	case GLUT_KEY_LEFT:
		
		

		if(GetKeyState(0x31) & 0x8000)
			{
				VSpeed[0] = 0;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x32) & 0x8000)
			{
				VSpeed[0] = .5;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x33) & 0x8000)
			{
				VSpeed[0] = 1;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x34) & 0x8000)
			{
				VSpeed[0] = 1.5;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x35) & 0x8000)
			{
				VSpeed[0] = 2;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x36) & 0x8000)
			{
				VSpeed[0] = 2.5;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x37) & 0x8000)
			{
				VSpeed[0] = 3;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			
			else if(GetKeyState(GLUT_KEY_UP) & 0x8000)
			{
				if(VSpeed[0]<3)
				{
					VSpeed[0] += .05;
					TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
				}
			}
			else if(GetKeyState(GLUT_KEY_DOWN) & 0x8000)
			{
				if(VSpeed[0]>0)
				{
					VSpeed[0] -= .05;
					TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
				}
			}
			break;
	case GLUT_KEY_RIGHT:
		
		if(GetKeyState(0x31) & 0x8000)
			{
				VSpeed[1] = 0;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x32) & 0x8000)
			{
				VSpeed[1] = .5;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x33) & 0x8000)
			{
				VSpeed[1] = 1;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x34) & 0x8000)
			{
				VSpeed[1] = 1.5;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x35) & 0x8000)
			{
				VSpeed[1] = 2;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x36) & 0x8000)
			{
				VSpeed[1] = 2.5;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x37) & 0x8000)
			{
				VSpeed[1] = 3;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			
			else if(GetKeyState(GLUT_KEY_UP) & 0x8000)
			{
				if(VSpeed[1]<3)
				{
					VSpeed[1] += .05;
					TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
				}
			}
			else if(GetKeyState(GLUT_KEY_DOWN) & 0x8000)
			{
				if(VSpeed[1]>0)
				{
					VSpeed[1] -= .05;
					TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
				}
			}
			break;
	case VK_SHIFT:
		if(GetKeyState(VK_UP) & 0x8000)
		{
			if(acc < 3.0)
			{
				acc += .05;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
		}
		else if(GetKeyState(VK_DOWN) & 0x8000)
		{
			if(acc > 0)
			{
				acc-=.05;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
		}
		break;
	
	case VK_CONTROL:
			if(GetKeyState(0x31) & 0x8000)
			{
				VSpeed[0] = 0;
				VSpeed[1] = 0;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x32) & 0x8000)
			{
				VSpeed[0] = .5;
				VSpeed[1] = .5;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x33) & 0x8000)
			{
				VSpeed[0] = 1;
				VSpeed[1] = 1;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x34) & 0x8000)
			{
				VSpeed[0] = 1.5;
				VSpeed[1] = 1.5;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x35) & 0x8000)
			{
				VSpeed[0] = 2;
				VSpeed[1] = 2;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x36) & 0x8000)
			{
				VSpeed[0] = 2.5;
				VSpeed[1] = 2.5;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			else if(GetKeyState(0x37) & 0x8000)
			{
				VSpeed[0] = 3;
				VSpeed[1] = 3;
				TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
			}
			
			else if(GetKeyState(VK_UP) & 0x8000)
			{
				if(VSpeed[0]<3 && VSpeed[1] < 3)
				{
					VSpeed[0] += .05;
					VSpeed[1] += .05;
					TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
				}
			}
			else if(GetKeyState(VK_DOWN) & 0x8000)
			{
				if(VSpeed[0]>0 && VSpeed[1] > 0)
				{
					VSpeed[0] -= .05;
					VSpeed[1] -= .05;
					TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
				}
			}
			break;
	case GLUT_KEY_F1: 
		eLevel = Easy;
		errorMarginX = 0.08f;
		errorMarginY = 0.08f;
		break;
	case GLUT_KEY_F2: 
		eLevel = Normal;
		errorMarginX = 0.06f;
		errorMarginY = 0.06f;
		break;	
	case GLUT_KEY_F3: 
		eLevel = Hard;
		errorMarginX = 0.04f;
		errorMarginY = 0.04f;
		break;
	case GLUT_KEY_F4: 
		eLevel = Hardest;
		errorMarginX = 0.02f;
		errorMarginY = 0.02f;
		break;
	case GLUT_KEY_END: 
		visible = !visible;   
		break;
	case GLUT_KEY_INSERT: 
		targetVisible = !targetVisible;   
		break;
	}
}

char WaitForKey()
{
	return _getch();
}

void kinectPrintHeader()
{
	fprintf(stream, "VRTask: Sequence A, version 1.4\n");
	fprintf(stream, "===================\n");
	fprintf(stream, "------Kinect-------\n");
	fprintf(stream, "===================\n");

	// current date/time based on current system
	time_t now = time(0);   
	char* dt = ctime(&now);
	fprintf(stream, "[Start] %s", dt);
	fprintf(stream, "------------------------------------------------------------------------------------------------------------------------------\n");
    fprintf(stream,"Frame, nStep, stepLength, Score, TargetX, TargetY-0, TargetY-1, EndpointX-0, EndpointY-0, EndpointX-1, EndpointY-1, LBelt, RBelt, \n");
	fprintf(stream, "------------------------------------------------------------------------------------------------------------------------------\n");
}

void printHeader()
{	
	fprintf(stream, "VRTask: Sequence A, version 1.4\n");
	fprintf(stream, "===================\n");
	// 3D settings
	char* pTmpStr = poRTProtocol.Get3DCalibrated();
	if (pTmpStr[0] == 0)
	{
		fprintf(stream ,"Calibration Time: Not Calibrated\n");
	}
	else
	{
		fprintf(stream, "Calibration Time: %s\n", poRTProtocol.Get3DCalibrated());
	}
    poRTProtocol.GetGeneral(nCaptureFrequency, fCaptureTime, bStartOnExtTrig, eProcessingActions);
	fprintf(stream, "Capture frequency: %d\n", nCaptureFrequency);

	nCount = poRTProtocol.Get3DLabeledMarkerCount();
	for (unsigned int iLabel = 0; iLabel < nCount; iLabel++)
	{
		fprintf(stream, "Marker %2d: %s\n", iLabel + 1, poRTProtocol.Get3DLabelName(iLabel));
	}
	fprintf(stream,"\n");
	
	// current date/time based on current system
	time_t now = time(0);   
	char* dt = ctime(&now);
	fprintf(stream, "[Start] %s", dt);
	fprintf(stream, "------------------------------------------------------------------------------------------------------------------------------\n");
    fprintf(stream,"Frame, nStep, stepLength, Score, TargetX, TargetY-0, TargetY-1, EndpointX-0, EndpointY-0, EndpointX-1, EndpointY-1, LBelt, RBelt, \n");
	fprintf(stream, "------------------------------------------------------------------------------------------------------------------------------\n");
}

void printHeader2()
{	
	fprintf(stream2, "VRTask: Sequence A, version 1.4\n");
	fprintf(stream2, "===================\n");
	fprintf(stream2,"nStep, seqID, footID, TargetY, EndpointY, StanceY, ErrorY, LBelt, RBelt, Score, Level \n");
}


Kinect setup_Kinect(Kinect k)
{
	int sID = 0;
	INuiSensor * sensor = NULL;
	HRESULT sOK = NuiGetSensorCount(&sID);
	std::cout<<sID;
	if(sOK == S_OK)
	{
		std::cout<< "Kinect Sensors Found! \n Connecting...";
	
		HRESULT cOK = NuiCreateSensorByIndex(0, &sensor);
		if(cOK == S_OK)
		{
			std::cout<< "Connection to Kinect Established on channel: " + sID;
			DWORD initFlags = NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_SKELETON;
			if(sensor -> NuiInitialize(initFlags) == S_OK)
			{
				std::cout<<"Kinect Streams Initialized Good To Go...";
				k.sensor = sensor;
				k.initStatus = 0;
				
				HANDLE k_skeleton_event = CreateEventW(NULL, TRUE, FALSE, NULL);
				k.skeletonEvent = k_skeleton_event;
				HRESULT hr = k.sensor->NuiSkeletonTrackingEnable(k.skeletonEvent, 0);
				if(!SUCCEEDED(hr))
				{
					std::cout<<"Couldn't open skeleton stream";
					k.initStatus = 5;
				}
				else
				{
					HANDLE k_Color_event = CreateEventW(NULL, TRUE, FALSE, NULL);
					HANDLE k_Color_handler = NULL;
					k.colorEvent = k_Color_event;
					k.colorHandler = k_Color_handler;
					HRESULT hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_1280x960, 0x00020000, 0,k.colorEvent,&k.colorHandler);
				}

			}
			else
			{
				std::cout<<"Failed to initialize kinect streams, exiting...";
				k.sensor = sensor;
				k.initStatus = 4;
			}
		}
		else
		{
			std::cout<< "Connection to kinect sensor could not be established exiting...";
			
			k.sensor = sensor;
			k.initStatus = 2;
		}

	}
	else
	{
		std::cout<< "No Kinect Sensors found";
		k.sensor = sensor;
		k.initStatus = 3;
	}
	return k;
}

void secondaryController()
{
	if(GetKeyState(0x41)&0x8000)
			{
				VSpeed[0] += .05;
				VSpeed[1] += .05;
				
			}
			if(GetKeyState(0x53) & 0x8000)
			{
				VSpeed[0] -= .05;
				VSpeed[1] -= .05;
				
			}
			if(GetKeyState(VK_ESCAPE) & 0x8000)
			{
				TREADMILL_setSpeed(0,0,acc);
				exit(0);
			}
			TREADMILL_setSpeed(VSpeed[0],VSpeed[1],acc);
}

/* Main function: GLUT runs as a console application starting at main() */
int main(int argc, char** argv)
{
	HANDLE      hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	
	// By default assume you want to connect to QTM at the same machine - just for testing
	char pServerAddr[32] = "localhost";
	int	nMajorVersion = 1;
    int nMinorVersion = 12;
	
	//// Check the command line for the server address
	if (argc > 1)
	{
		strcpy(pServerAddr, argv[1]);
	}

	//// The base port (as entered in QTM, TCP/IP port number, in the RT output tab of the workspace options.
	int nBasePort = QTM_RT_SERVER_BASE_PORT;	// Use default port if there is no argument
	if (argc > 2)
	{
		nBasePort = strtoul(argv[2], NULL, 10);
	}

	const char * tempPort = std::to_string(QTM_RT_SERVER_BASE_PORT).c_str();
	char * port = "";
	//strcpy(port, tempPort);

	// Check the command line for task settings
	if (argc > 1)
	{
		Speed = strtod (argv[1], NULL);
	}

	if (argc > 2)
	{
		stepLength = strtod (argv[2], NULL);
	}
	
	
	char treadHolder;
	bool treadMode = false;
	std::cout<<"Treadmill Controller only? (y/n)";
	std::cin>>treadHolder;
	if(treadHolder =='y')
	{
		treadMode = true;
	}
	if(treadMode)
	{
		std::cout<<"Treadmill control now active";
		start = std::clock();
		duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

	if(!TREADMILL_initialize("127.0.0.1","4000")==0)
	{
		std::cout<<"Couldn't connect to treadmill using default connection settings";
		exit(1);
	}
	VSpeed[0] = 0;
	VSpeed[1] = 0;
	//TREADMILL_setSpeed(DEFAULT_SPEED, DEFAULT_SPEED, acc);
		while(true)
		{
			Sleep(100);
			secondaryController();
		}

	}
	
	std::cout<<"Enter start speed: ";
	float temp = 0;
	std::cin>>temp;
	DEFAULT_SPEED = temp;
	VSpeed[0] = DEFAULT_SPEED;
	VSpeed[1] = DEFAULT_SPEED;
	std::cout<<"Enter Max Speed (default is 2.0): ";
	float temp2 = 0;
	std::cin>>temp2;

	
	MAX_SPEED = temp2;
	
	

	std::cout<<"Enter Num Steps (default is 100, 0 for infinite): ";
	int temp3 = 0;
	std::cin>>temp3;

	
	if(temp3 == 0)
	{
		DEFAULT_NSTEPS = 100000;
	}
	else
	{
		DEFAULT_NSTEPS = temp3;
	}
	std::cout<< "Record Qualisys info? (1 for y, 0 for n)";
	int temp4 = 0;
	std::cin>>temp4;
	

	
	void static (*pFcn1)() = kinectDisplay;

	printf("Enter File Name : ");
	char pStr[256];
	gets_s(pStr, sizeof(pStr));
	stream = fopen(pStr, "a+");	
	char pStr2[256];
	strcpy(pStr2, pStr);
	strcat(pStr2, "_summary");
	stream2 = fopen(pStr2, "a+");
	
	start = std::clock();
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

	if(!TREADMILL_initialize("127.0.0.1","4000")==0&&!treadMode)
	{
		std::cout<<"Couldn't connect to treadmill using default connection settings";
		exit(1);
	}
	Kinect *k = new Kinect();
	bool treadin = true;

	setup_Kinect(*k);
	
	if(k->initStatus == 0)
	{
		bool treadin = true;
		std::cout<<"Okay to start Treadmill...";
	}
	else
	{
		std::cout<< "Entering Qualisys Mode";
		pFcn1 = display;
		if (poRTProtocol.Connect(pServerAddr, QTM_RT_SERVER_BASE_PORT, 0, nMajorVersion, nMinorVersion)) {
		poRTProtocol.TakeControl();
		if(RECORDING)
		{
			poRTProtocol.NewMeasurement();
		}
		poRTProtocol.GetState(eEvent);
		while (eEvent != CRTPacket::EventConnected)
			poRTProtocol.GetState(eEvent);
		poRTProtocol.StartCapture();
		GetEndpoint();
		printHeader();
		printHeader2();
		
		glutInit(&argc, argv);                          
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);    
		glutInitWindowSize(windowWidth, windowHeight);  
		glutInitWindowPosition(windowPosX, windowPosY); 
		glutCreateWindow(title);                        
		glutDisplayFunc(*pFcn1);                        
		glutReshapeFunc(reshape);                       
		glutTimerFunc(0, Timer, 0);                     
		glutSpecialFunc(specialKeys);                   
		glutKeyboardFunc(keyboard);                     
		//glutFullScreen();                              
		initGL();                                       
		glutMainLoop();  
	}
		
	}
	

	bool zeroed = false;

	
			
					
					
			

	while(!zeroed)
	{
		skeletonFrame = NuiSkeletonGetNextFrame(5000, &skelFrame);
		if(SUCCEEDED(skeletonFrame))
		{
			
			
			if(NUI_SKELETON_COUNT!=0 )
			{
				
				
				
				for(int i = 0; i<NUI_SKELETON_COUNT; i++)
				{
					NUI_SKELETON_DATA dat = skelFrame.SkeletonData[i];
					
					if(dat.eTrackingState==NUI_SKELETON_TRACKED)
					{
						
						std::cout<<"Skeleton Framed";
						
						NUI_SKELETON_DATA dat = skelFrame.SkeletonData[i];
						Vector4 tempLeft = dat.SkeletonPositions[18];
						Vector4 tempRight = dat.SkeletonPositions[14];

						zeroXL = tempRight.x;
						zeroXR = tempLeft.x;
						zeroYL = tempRight.z+.08;
						zeroYR = tempLeft.z+.08;
						zeroed = true;
		
						VSpeed[0] = 1.5;
						VSpeed[1] = 1.5;
						
						TREADMILL_setSpeed(VSpeed[0], VSpeed[1], acc);
						
					}
				}
				
				
			}
		}
	}

	//glutInit(&argc, argv);                          
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);    
		glutInitWindowSize(windowWidth, windowHeight);  
		glutInitWindowPosition(windowPosX, windowPosY); 
		glutCreateWindow(title);                        
		glutDisplayFunc(*pFcn1);                        
		glutReshapeFunc(reshape);                       
		glutTimerFunc(0, Timer, 0);                     
		glutSpecialFunc(specialKeys);                   
		glutKeyboardFunc(keyboard);                     
		//glutFullScreen();                              
		initGL();                                       
		glutMainLoop();  
	//glutCreateWindow("TEST");
		
	
	
	

	// Get settings from QTM		
	
	
	return 0;

}