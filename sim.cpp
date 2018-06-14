// sim.cpp
#include "sim.h"
#include "normal.h"
#include "shortCut.h"
#include "UCFL_Utils2.h"
#include "mainCode.h"
#include "math.h"
/*
the direction for angles is as follows 

				270 degrees
                 |
180 degrees <----+----> 0 degrees
		         |
                90 degrees
*/
const int maxX=600;
const int maxY=600;
const GLfloat obstacleRadius = 8;
const GLfloat moveRadius = 8;
int ticks=0;
int countFail=0,countWalls=0, countCrash=0;
int run_num=0;
bool stop_sim=true;
bool shortCut_sim=false;
bool sugenoAcum=false;

bool showAccumSets=false;
bool showRulesSets=false;
bool showInitSets=false;
bool showCLines=false;
bool showStat=true;

const int numObstacle=10;
const int Corners=4;
simObject obstacle[numObstacle+Corners];
int rndArry[numObstacle][2];

float speedDilation=1;

const int numMove=120;
simMoveObject move[numMove];
int rndArryM[numMove][2];
float calculatedDistance, predectedDistance,calculatedAngle;
float dr,dl,db,dt;
GLfloat drDelta, dlDelta, dbDelta ,dtDelta ;
double fuzzifySpeed, fuzzifyDirection;

CBitmap *bmpObstacle;
CBitmap *bmpMove;

float allowed_distance = 0.1;
float emergency_angle = 8;
float emergency_reset_value=-368;

int runs_count=0;
int runs_allowed=500;
bool batch_run=true;
time_t time_start, time_end;
int minFPS, maxFPS, FPS;

float sign(float val)
{
	if (val < 0) return -1;
	return 1;
}

void moveByAngleDist(GLfloat x,GLfloat y, GLfloat angleInRadians, GLfloat distance,GLfloat *nx,GLfloat *ny)
{
	//angleInRadians = degToRad(angleInRadians);
	*nx=x + (distance * (GLfloat)cos(angleInRadians));
	*ny=y - (distance * (GLfloat)sin(angleInRadians));
}

bool willCrashWall(simMoveObject *moveObj)
{
	if ((moveObj->nextX+moveObj->radius >= maxX)||
		(moveObj->nextY+moveObj->radius >= maxY)||
		(moveObj->nextX-moveObj->radius < 0)||
		(moveObj->nextY-moveObj->radius < 0)) return true;
return false;
}

bool willCrashStationary(simMoveObject *moveObj)
{
//bool retv=false;
	for (int i=0; i<numObstacle; i++)
	{
	//calculater the predicted distance
	predectedDistance = dist((float)moveObj->nextX,(float)moveObj->nextY,
		(float)obstacle[i].x,(float)obstacle[i].y);
	predectedDistance = predectedDistance-(moveObj->radius+obstacle[i].radius);
	
	if(predectedDistance<allowed_distance) {return true;}
	}
	return false;
}

bool willCrashMoving(simMoveObject *moveObj)
{
	for (int i=0; i<numMove; i++)
	{
	//calculater the predicted distance
	if (moveObj==(&move[i])) continue;
	predectedDistance = dist((float)moveObj->nextX,(float)moveObj->nextY,
		(float)move[i].x,(float)move[i].y);
	predectedDistance = predectedDistance-(moveObj->radius+move[i].radius);
	
	if(predectedDistance<allowed_distance) {return true;}
	}
	return false;
}

void initCal(simMoveObject *moveObj, int i, int j)
{
	int N=i-(numObstacle+Corners);
	if(j==N) return;
	
	GLfloat posX, posY, currRadius;
	GLfloat nextX, nextY; 
	moveByAngleDist(moveObj->x,moveObj->y, degToRad(moveObj->angle), moveObj->speed*speedDilation,&nextX,&nextY);
	
	if(i<numObstacle+Corners)
	{ 
		posX=obstacle[i].x;
		posY=obstacle[i].y;
		currRadius=obstacle[i].radius;
	}
	else if(i>=numObstacle+Corners)
	{ 
		posX=move[N].x;
		posY=move[N].y;
		currRadius=move[N].radius;
	}
	//calculate distance between player and objects
	calculatedDistance = dist((float)moveObj->x,(float)moveObj->y,posX,posY);
	calculatedDistance =calculatedDistance-(moveObj->radius+currRadius);
			
	////calculater the predicted distance
	//predectedDistance = dist((float)nextX,(float)nextY,(float)posX,(float)posY);
	//predectedDistance = predectedDistance-(moveObj->radius+currRadius);
	//
	//if(predectedDistance<allowed_distance)

	float gama = (float)(atan2(-(moveObj->y-posY),(moveObj->x-posX)));
	gama = radToDeg(gama)+180;
	float g = (gama - moveObj->angle) ; //theta - gama;
	while (g > 360) g = g - 360;
	while (g < -360) g = g + 360;
	if (g > 180) g = g - 360;
	calculatedAngle = g;
	
	
	//calculate walls distance 
	dr = maxX - moveObj->x;
	dl = moveObj->x;
	db = moveObj->y;
	dt = maxY - moveObj->y;

	GLfloat drnext = maxX - nextX;
	GLfloat dlnext = nextX;
	GLfloat dbnext = nextY;
	GLfloat dtnext = maxY - nextY;

	//allowed_distance+=moveObj->radius;
	////walls emergency exit
	//if(drnext<=allowed_distance || dlnext<=allowed_distance || dbnext<=allowed_distance || dtnext<=allowed_distance )//while
	//{
	//	if(!moveObj->emergency) countWalls++;
	//	moveObj->emergency=true;
	//}
	//else 
	//	moveObj->emergency=false;

	drDelta = dr-drnext;
	dlDelta = dl-dlnext;
	dbDelta = db-dbnext;
	dtDelta = dt-dtnext;
}
void moveIt(simMoveObject *moveObj)
{	
	moveObj->speed=fuzzifySpeed;
	moveObj->angle=moveObj->angle+fuzzifyDirection;

	// now predict if we will crash with another creature or ovbject
	moveByAngleDist(moveObj->x,moveObj->y, degToRad(moveObj->angle), moveObj->speed*speedDilation,&moveObj->nextX,&moveObj->nextY);

	if (willCrashWall(moveObj))
	{
		// are in emergency
		if (!moveObj->emergency)countWalls++;
		moveObj->emergency=true;
		moveObj->emergency_cnt=moveObj->emergency_cnt+emergency_angle;
		if (moveObj->emergency_cnt > (-1)*emergency_reset_value) moveObj->emergency_cnt=emergency_reset_value;
		moveObj->speed=0;
	 	moveObj->angle=moveObj->angle+sign(moveObj->emergency_cnt)*emergency_angle;
		if(moveObj->angle>360) moveObj->angle-=360;
		if(moveObj->angle<-360) moveObj->angle+=360;
	}
	if (willCrashStationary(moveObj))
	{
		// are in emergency	
		if (!moveObj->emergency)countFail++;
		moveObj->emergency=true;
		moveObj->emergency_cnt=moveObj->emergency_cnt+emergency_angle;
		if (moveObj->emergency_cnt > (-1)*emergency_reset_value) moveObj->emergency_cnt=emergency_reset_value;
		moveObj->speed=0;
	 	moveObj->angle=moveObj->angle+sign(moveObj->emergency_cnt)*emergency_angle;

		if(moveObj->angle>360) moveObj->angle-=360;
		if(moveObj->angle<-360) moveObj->angle+=360;
	}
	if (willCrashMoving(moveObj))
	{
		// are in emergency	
		if (!moveObj->emergency)countCrash++;
		moveObj->emergency=true;
		moveObj->emergency_cnt=moveObj->emergency_cnt+emergency_angle;
		if (moveObj->emergency_cnt > (-1)*emergency_reset_value) moveObj->emergency_cnt=emergency_reset_value;
		moveObj->speed=0;
	 	moveObj->angle=moveObj->angle+sign(moveObj->emergency_cnt)*emergency_angle;
		if(moveObj->angle>360) moveObj->angle-=360;
		if(moveObj->angle<-360) moveObj->angle+=360;
	}
	if (moveObj->speed > 0)	moveObj->emergency=false;
 
	//printf("[%d] Dist=%2.2f  speed=%2.2f  angle=%2.2f  deltaD=%2.2f   CalAng=%2.2f\n",j,(double)calculatedDistance,(double)move[j].speed,(double)move[j].angle, fuzzifyDirection, calculatedAngle);
	moveByAngleDist(moveObj->x,moveObj->y, degToRad(moveObj->angle), moveObj->speed*speedDilation,&moveObj->x,&moveObj->y);
	//else 
	if ((moveObj->x >=maxX)||(moveObj->y >=maxY)||(moveObj->x < 0)||(moveObj->y < 0))
	{
		countWalls++; 
		//simReset();
		if(moveObj->x >=maxX) moveObj->x=0; 
		if(moveObj->y >=maxY) moveObj->y=0;
		if(moveObj->x < 0) moveObj->x=maxX;
		if(moveObj->y < 0) moveObj->y=maxY;
	}
	
}


void simRender()
{	
	UcPoint place(maxX-60,maxY-10);
	char txt[40]="";

	if (stop_sim) 
	{
		place.x=maxX/2; place.y=maxY/2;
		UcDrawstr(place.x, place.y, font_style0, "PAUSE"); 
	}

	for (int i=0; i<numObstacle+Corners; i++)
	{
    RenderRotate(bmpObstacle,
		obstacle[i].x-obstacle[i].radius,obstacle[i].y+obstacle[i].radius,
		obstacle[i].x+obstacle[i].radius,obstacle[i].y-obstacle[i].radius,
              true, 0,
			  obstacle[i].x,obstacle[i].y ,
			  degToRad(obstacle[i].gama+90),
			  UcColour::White, GL_FILL
			  );
	}
	for (int j=0; j<numMove; j++)
	{	
		RenderRotate(bmpMove,
				  move[j].x-move[j].radius, move[j].y+move[j].radius,
				  move[j].x+move[j].radius, move[j].y-move[j].radius,
				  true, 0,
				  move[j].x, move[j].y,
				  degToRad(move[j].angle-90),
				  //degToRad(80),
				  UcColour(UcColour::White), GL_FILL
				  );
		if(showCLines)
		{
			//FAR distance area
			UcDrawCircleLine(UcPoint(move[j].x,move[j].y), UcColour::Brown , 70, 21);
			UcDrawCircleLine(UcPoint(move[j].x,move[j].y), UcColour::Brown , 50, 21);
			//close distance area
			UcDrawCircleLine(UcPoint(move[j].x,move[j].y), UcColour::Silver , 40, 21);
			UcDrawCircleLine(UcPoint(move[j].x,move[j].y), UcColour::Silver , 30, 21);
			//very close distance area
			UcDrawCircleLine(UcPoint(move[j].x,move[j].y), UcColour::Brown , 20, 11);

			//very dangAngle left & right
			//UcDrawLine(UcPoint(move.x,move.y), UcColour::Red, 100, degToRad(-5+move.angle));
			//UcDrawLine(UcPoint(move.x,move.y), UcColour::Red, 100, degToRad(5+move.angle));
			//dangAngle left & right
			UcDrawLine(UcPoint(move[j].x,move[j].y), UcColour::Blue, 70, degToRad(-15+move[j].angle));
			UcDrawLine(UcPoint(move[j].x,move[j].y), UcColour::Blue, 70, degToRad(15+move[j].angle));
			//close Angle left & right
			UcDrawLine(UcPoint(move[j].x,move[j].y), UcColour::Black, 40, degToRad(-30+move[j].angle));
			UcDrawLine(UcPoint(move[j].x,move[j].y), UcColour::Black, 40, degToRad(30+move[j].angle));
			//far Angle left & right
			UcDrawLine(UcPoint(move[j].x,move[j].y), UcColour::Green, 20, degToRad(-60+move[j].angle));
			UcDrawLine(UcPoint(move[j].x,move[j].y), UcColour::Green, 20, degToRad(60+move[j].angle));
		}
	}
			

	if(!shortCut_sim)
		normalRender();
	else 
		shortCutRender();	
	
	if(showStat)
	{	
		UcColour(UcColour::Black).setGlColour();

		place.x=maxX-100; place.y=maxY-20;
		sprintf(txt,"FPS=%i",framePerSecond);
		UcDrawstr(place.x, place.y, font_style4, txt);
		place.y -= 16;

		place.x=maxX-100; place.y=maxY-30;
		sprintf(txt,"Crach Moveing=%i",countCrash);
		UcDrawstr(place.x, place.y, font_style4, txt); 

		place.x=maxX-100; place.y=maxY-40;
		sprintf(txt,"Crash Stationary=%i",countFail);
		UcDrawstr(place.x, place.y, font_style4, txt); 
		
		place.x=maxX-100; place.y=maxY-50;
		sprintf(txt,"Crash Walls=%i",countWalls);
		UcDrawstr(place.x, place.y, font_style4, txt); 
				
		place.x=maxX-100; place.y=maxY-60;
		sprintf(txt,"runs_count=%i",runs_count);
		UcDrawstr(place.x, place.y, font_style4, txt); 
				
		place.x=maxX-100; place.y=maxY-70;
		sprintf(txt,"runs=%i",run_num);
		UcDrawstr(place.x, place.y, font_style4, txt); 

		place.x=maxX/2; place.y=maxY-30;
		if(!shortCut_sim)
			UcDrawstr(place.x-24, place.y, font_style0, "Normal Fuzzy Set");
		else 
			UcDrawstr(place.x-24, place.y, font_style0, "Short Cut Fuzzy Set"); 
		if(sugenoAcum)
			UcDrawstr(place.x-24, place.y-20, font_style0, "With Sugeno Acumilator"); 

	}
	
	/*show timing 
	
    place.x= 8;
    place.y = maxY - 24;

	sprintf(txt,"countsPerSec=%I64d",countsPerSec);
    UcDrawstr(place.x, place.y, font_style4, txt);
     place.y -= 16;
	
	sprintf(txt,"prevTimeStamp=%I64d",prevTimeStamp);
    UcDrawstr(place.x, place.y, font_style4, txt);
     place.y -= 16;
	
	sprintf(txt,"secsPerCount=%6.6f",secsPerCount);
    UcDrawstr(place.x, place.y, font_style4, txt);
     place.y -= 16;
	
	sprintf(txt,"currTimeStamp=%I64df",currTimeStamp);
   UcDrawstr(place.x, place.y, font_style4, txt);
     place.y -= 16;

	sprintf(txt,"deltatime=%6.6f",deltatime);
    UcDrawstr(place.x, place.y, font_style4, txt);
     place.y -= 16;
	*/
	 
}
void simReset()
{
	//places of obstacles seperated from each other so the Moveing can get through

	ticks=0;
    countFail=0;
	countWalls=0;
	countCrash=0;
	runs_count=0;
    run_num++;

	float minDist, rx, ry;
	int chosen = 1;
	minDist = (2 * obstacleRadius + 2 * moveRadius);

	for (int i=0; i<numObstacle; i++)
	{			
		while(chosen){
			rx=(rand()%maxX);
			ry=(rand()%maxY);
			chosen = 0;
			for (int j = 0; j < i; j++){
				float dd = dist(rx,ry,rndArry[j][0],rndArry[j][1]);
				if( dd <= minDist)
					chosen = 1;
			}
		}
		rndArry[i][0]=rx;
		rndArry[i][1]=ry;
		chosen = 1;
	}
	for (int i=0; i<numObstacle; i++)
	{
		obstacle[i].x=(rndArry[i][0]);
		obstacle[i].y=(rndArry[i][1]);
		obstacle[i].radius=obstacleRadius;
	}

	//make the middle of the walls and the corners as fixed obsticals
	//top left corner
	obstacle[numObstacle].x=0;
	obstacle[numObstacle].y=0;
	obstacle[numObstacle].radius=obstacleRadius;
	//top right corner
	obstacle[numObstacle+1].x=0;
	obstacle[numObstacle+1].y=maxY;
	obstacle[numObstacle+1].radius=obstacleRadius;
	//bottom left corner
	obstacle[numObstacle+2].x=maxX;
	obstacle[numObstacle+2].y=0;
	obstacle[numObstacle+2].radius=obstacleRadius;
	//bottom right corner
	obstacle[numObstacle+3].x=maxX;
	obstacle[numObstacle+3].y=maxY;
	obstacle[numObstacle+3].radius=obstacleRadius;

	//place for moving Objects so they are not created on the obstacles or on each other!
	float minDistOb, minDistMv;
	int chosenM = 1;
	minDistOb = (obstacleRadius + moveRadius);
	minDistMv = (2 * moveRadius);

	for (int i=0; i<numMove; i++)
	{			
		chosen = true;
		while(chosen){
			rx= moveRadius +(rand()%(int)(maxX-(moveRadius+0.5)*2));
			ry= moveRadius +(rand()%(int)(maxY-(moveRadius+0.5)*2));
			chosen = false;
			for (int j = 0; j < i; j++) // test for collisions with other movers
			    {
				float dd = dist(rx,ry,rndArryM[j][0],rndArryM[j][1]);
				if( dd <= minDistMv)
					chosen = true;
			    }
			if(!chosen)
				for (int j = 0; j < numObstacle; j++) // test for collisions with obstacle
				    {
					float dd = dist(rx,ry,obstacle[j].x,obstacle[j].y);
					if( dd <= minDistOb)
						chosen = true;
				    }

		}
		rndArryM[i][0]=rx;
		rndArryM[i][1]=ry;
		chosen = 1;
	}
	for (int j=0; j<numMove; j++)
	{	
		move[j].x=(rndArryM[j][0]);
		move[j].y=(rndArryM[j][1]);
		move[j].angle=rand()%360;
		move[j].radius=moveRadius;
		move[j].speed=0.0001;
//		move[j].crashed=false;
		move[j].emergency=false;
		move[j].emergency_cnt=emergency_reset_value;
	}

	if(!shortCut_sim)
		normalReset();
	else 
		shortCutReset();	
}

void simInit()
{
	srand ( time(NULL) ); // init random
	
	minFPS=100;
	maxFPS=0;
    bmpObstacle = new CBitmap(findFile("Arrow4.bmp"));
    bmpMove = new CBitmap(findFile("Arrow3.bmp"));
	makeCBitmapTrans(bmpObstacle, 255, 255, 255, 0);
	makeCBitmapTrans(bmpMove, 0, 0, 0, 0);
	simReset();
	//if(!shortCut_sim)
		normalInit();
	//else 
		shortCutInit();		
}

void simKey(unsigned char key, int x, int y)
{
	if (key == ' ') stop_sim=!stop_sim;
	if(key=='s' || key=='S'){
		shortCut_sim=!shortCut_sim;	
		run_num=0;
		simReset();
	}
	if(key=='x' || key=='X'){
		sugenoAcum=!sugenoAcum;		
		run_num=0;
		simReset();
	}

	if (key == '1') showAccumSets=!showAccumSets;
	if (key == '2') showRulesSets=!showRulesSets;
	if (key == '3') showInitSets=!showInitSets;
	if (key == '4') showCLines=!showCLines;
	if (key == '5') showStat=!showStat;
	
	if (key == 'u' || key == 'U' ) move[0].y=move[0].y+5; 
	if (key == 'd' || key == 'D' ) move[0].y=move[0].y-5; 
	if (key == 'l' || key == 'L' ) move[0].x=move[0].x-5; 
	if (key == 'r' || key == 'R' ) move[0].x=move[0].x+5; 
	if (key == 'c' || key == 'C' ) move[0].angle = move[0].angle + 5; 
	if (key == 'a' || key == 'A' ) move[0].angle = move[0].angle - 5;  

	if(!shortCut_sim)
		normalKey(key, x, y);
	else 
		shortCutKey(key, x, y);
}

void simTimer(int value)
{
}

void record_results()
{
	char temp[256];
	
	if(shortCut_sim) sprintf(temp,"ShortCut");
	else sprintf(temp,"Normal");
	if(sugenoAcum) sprintf(temp,"%s With Sugeno",temp);

	FILE * hFile;
	hFile = fopen("results.txt","a+");
	if (hFile == NULL)
	{
	printf("Error>> results.txt file not able to be opened");
	return;
	}
	else
	{
    time_t rawtime;
    struct tm * timeinfo;
	double dif;
	dif = difftime(time_end,time_start);
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    //printf ( "Current local time and date: %s", asctime (timeinfo) );
	
    fprintf(hFile,"%s, %.24s, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
	       temp, asctime (timeinfo),run_num,runs_count,numObstacle,numMove,countFail,countCrash,countWalls,(int)dif,minFPS,maxFPS);
	fclose(hFile);
	}
}

void simIdle()
{	
	if (!stop_sim)
	{
		runs_count++;
		if (runs_count==1) time (&time_start);
		if (runs_count > 20 && framePerSecond < 19 )
		{
			if(framePerSecond > maxFPS) maxFPS=framePerSecond;
			if(framePerSecond < minFPS) minFPS=framePerSecond;
		}
	}
	if (runs_count > runs_allowed) 
		{
			stop_sim = true;
			time (&time_end);
			record_results();
			if (batch_run)
			{
			simReset();
            stop_sim = false;
			}

		}
	if(!shortCut_sim)
		normalIdle();
	else 
		shortCutIdle();
	simHasRun=true;
}

void simExit()
{
}

// end
