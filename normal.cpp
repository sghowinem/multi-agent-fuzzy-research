// sim.cpp

#include "normal.h"
#include "UCFL_Utils2.h"
#include "mainCode.h"
#include "math.h"
#include "Sim.h"

CFLS *gFSList;
int scs;
FuzzySet *fs;

// Normal Fuzzy sets 
//fuzzy sets for distance from obj
float L,H; //low and high values

//distanse from obsticals sets
FuzzySet *veryClose, *close, *far ;

//fuzzy sets for angle from obj
//very dangerous angle 
FuzzySet *veryDangAngleLeft ,*veryDangAngleRight ;
//dangerous angle 
FuzzySet *dangAngleLeft , *dangAngleRight ;
//close angle 
FuzzySet *closeAngleLeft,*closeAngleRight;
//far angle 
FuzzySet *farAngleLeft, *farAngleRight;


//fuzzy sets for speed
FuzzySet *slow, *medSpeed, *fast ;

//fuzzy sets for direction
//shrp turn
FuzzySet *sharpLeft ,*sharpRight;
//turn 
FuzzySet *turnleft, *turnright;
//slowTurn 
FuzzySet *slowTurnLeft, *slowTurnRight;

//fuzzy sets for distance of walls
FuzzySet *closeToWall, *closishToWall ,*wallGettingCloser;

//Sugeno sets for speed
Sugeno *sugSlow, *sugMedSpeed, *sugFast ;
//Sugeno sets for direction
Sugeno *sugSharpLeft ,*sugSharpRight;
Sugeno *sugTurnleft, *sugTurnright;
Sugeno *sugSlowTurnLeft, *sugSlowTurnRight;
Sugeno *sugNoMove;

//acumilating sets
FuzzySet *fSetSpeed ;
FuzzySet *fSetDirection;
//temproray rules sets
FuzzySet *r1s,*r2s,*r3s,*r4s,*r5s,*r6s,*r7s,*r8s,*r9s,*r10s,*r11s,*r12s;
FuzzySet *r13s,*r14s,*r15s,*r16s,*r17s,*r18s,*r19s,*r20s,*r21s;
FuzzySet *r1d,*r2d,*r3d,*r4d,*r5d,*r6d,*r7d,*r8d,*r9d,*r10d,*r11d,*r12d;
FuzzySet *r13d,*r14d,*r15d,*r16d,*r17d,*r18d,*r19d,*r20d,*r21d;

//acumilating sets
Sugeno *sugfSetSpeed[3];
Sugeno *sugfSetDirection[7];

//temproray rules sets
Sugeno *sugR1s,*sugR2s,*sugR3s,*sugR4s,*sugR5s,*sugR6s,*sugR7s,*sugR8s,*sugR9s,*sugR10s,*sugR11s,*sugR12s;
Sugeno *sugR13s,*sugR14s,*sugR15s,*sugR16s,*sugR17s,*sugR18s,*sugR19s,*sugR20s,*sugR21s;
Sugeno *sugR1d,*sugR2d,*sugR3d,*sugR4d,*sugR5d,*sugR6d,*sugR7d,*sugR8d,*sugR9d,*sugR10d,*sugR11d,*sugR12d;
Sugeno *sugR13d,*sugR14d,*sugR15d,*sugR16d,*sugR17d,*sugR18d,*sugR19d,*sugR20d,*sugR21d;


FuzzySet *one;
FuzzySet *zero;

float theta,gama, g;

const float speedRangeH = 3.0;
const float speedRangeL = 0.0;


/**
Fuzzy rule with two antecedants with AND and two result
The folloing rule evaluates as follows
IF 	distanceVlue IS in distanceSet AND angleVlue IS angleSet THEN 
    acumilateSpeedSet <-IS speedSet AND_ALSO acumilateDirectionSet <-IS directionSet

*/
void fuzzyRule2and2(double distanceVlue,FuzzySet *distanceSet, 
			   double angleVlue,FuzzySet *angleSet,
			   FuzzySet *acumilateSpeedSet,FuzzySet *speedSet,
			   FuzzySet *acumilateDirectionSet,FuzzySet *directionSet,
			   FuzzySet *tempSpeed, FuzzySet *tempDirection, float weighting=1)
{
	double fuzzyDistance = distanceSet->fuzzify((double) distanceVlue);	
	double fuzzyAngle = angleSet->fuzzify((double) angleVlue);	

	double membership = min(fuzzyDistance,fuzzyAngle)*weighting; //AND = min in fuzzy logic

	if (membership > 0)
	{
		//if there is member ship creat temp sets
		FuzzySet *tempSpeed2, *tempDirection2;
		tempSpeed2 = new FuzzySet("tempSpeed2",speedSet->getLowRange(),speedSet->getHighRange());
		tempDirection2 = new FuzzySet("tempDirection2",-7,7);
		tempSpeed2->clearVal(0);
		tempDirection2->clearVal(0);

		//acumilate Speed
		FuzzySet::copyFS(tempSpeed2, speedSet);
		FuzzySet::scaleFS(tempSpeed2,membership);
		FuzzySet::unionFSsafe(acumilateSpeedSet,tempSpeed2, acumilateSpeedSet);
		FuzzySet::unionFSsafe(tempSpeed,tempSpeed2, tempSpeed);

		//acumilate Distance
		FuzzySet::copyFS(tempDirection2, directionSet);
		FuzzySet::scaleFS(tempDirection2,membership);
		FuzzySet::unionFSsafe(acumilateDirectionSet,tempDirection2, acumilateDirectionSet);
		FuzzySet::unionFSsafe(tempDirection,tempDirection2, tempDirection);
		
		//distroy them later
		tempSpeed2->~FuzzySet();
		tempDirection2->~FuzzySet();
	}
}
/**
Fuzzy rule with two antecedants with AND and two result
The folloing rule evaluates as follows
IF 	distanceVlue IS in distanceSet AND angleVlue IS angleSet AND v3 IS set3 THEN 
    acumilateSpeedSet <-IS speedSet AND_ALSO acumilateDirectionSet <-IS directionSet

*/
void fuzzyRule3and2(double distanceVlue,FuzzySet *distanceSet, 
			   double angleVlue,FuzzySet *angleSet,
			   double v3,FuzzySet *set3,
			   FuzzySet *acumilateSpeedSet,FuzzySet *speedSet,
			   FuzzySet *acumilateDirectionSet,FuzzySet *directionSet,
			   FuzzySet *tempSpeed, FuzzySet *tempDirection, float weighting=1)
{
	double fuzzyDistance = distanceSet->fuzzify((double) distanceVlue);	
	double fuzzyAngle = angleSet->fuzzify((double) angleVlue);	
	double a3 = set3->fuzzify((double) v3);	

	double membership = min(fuzzyDistance,fuzzyAngle); //AND = min in fuzzy logic
	membership = min(a3,membership)*weighting; //AND = min in fuzzy logic

	if (membership > 0)
	{
		//if there is member ship creat temp sets
		FuzzySet *tempSpeed2, *tempDirection2;
		tempSpeed2 = new FuzzySet("tempSpeed2",speedSet->getLowRange(),speedSet->getHighRange());
		tempDirection2 = new FuzzySet("tempDirection2",-7,7);
		tempSpeed2->clearVal(0);
		tempDirection2->clearVal(0);

		//acumilate Speed
		FuzzySet::copyFS(tempSpeed2, speedSet);
		FuzzySet::scaleFS(tempSpeed2,membership);
		FuzzySet::unionFSsafe(acumilateSpeedSet,tempSpeed2, acumilateSpeedSet);
		FuzzySet::unionFSsafe(tempSpeed,tempSpeed2, tempSpeed);

		//acumilate Distance
		FuzzySet::copyFS(tempDirection2, directionSet);
		FuzzySet::scaleFS(tempDirection2,membership);
		FuzzySet::unionFSsafe(acumilateDirectionSet,tempDirection2, acumilateDirectionSet);
		FuzzySet::unionFSsafe(tempDirection,tempDirection2, tempDirection);
		
		//distroy them later
		tempSpeed2->~FuzzySet();
		tempDirection2->~FuzzySet();
	}
}


void sugfuzzyRule2and2(double distanceVlue,FuzzySet *distanceSet, 
			   double angleVlue,FuzzySet *angleSet,
			   Sugeno *acumilateSpeedSet[3],Sugeno *speedSet,
			   Sugeno *acumilateDirectionSet[7],Sugeno *directionSet,
			   Sugeno *tempSpeed, Sugeno *tempDirection, float weighting=1)
{
	double fuzzyDistance = distanceSet->fuzzify((double) distanceVlue);	
	double fuzzyAngle = angleSet->fuzzify((double) angleVlue);	

	double membership = min(fuzzyDistance,fuzzyAngle)*weighting; //AND = min in fuzzy logic

	if (membership > 0)
	{
		//acumilate Speed
		tempSpeed->setWorldVal(speedSet->getWorldVal());
		tempSpeed->setSetValue(speedSet->getSetValue()*membership);
		for(int l=0;l<3;l++)
			acumilateSpeedSet[l]->sugenoMin(*tempSpeed);//was Max? why

		//acumilate Distance
		tempDirection->setWorldVal(directionSet->getWorldVal());
		tempDirection->setSetValue(directionSet->getSetValue()*membership);
		for(int l=0;l<6;l++)
			acumilateDirectionSet[l]->sugenoMin(*tempDirection);//was Max? why
	}
}
/**
Fuzzy rule with two antecedants with AND and two result
The folloing rule evaluates as follows
IF 	distanceVlue IS in distanceSet AND angleVlue IS angleSet AND v3 IS set3 THEN 
    acumilateSpeedSet <-IS speedSet AND_ALSO acumilateDirectionSet <-IS directionSet

*/
void sugfuzzyRule3and2(double distanceVlue,FuzzySet *distanceSet, 
			   double angleVlue,FuzzySet *angleSet,
			   double v3,FuzzySet *set3,
			   Sugeno *acumilateSpeedSet[3],Sugeno *speedSet,
			   Sugeno *acumilateDirectionSet[7],Sugeno *directionSet,
			   Sugeno *tempSpeed, Sugeno *tempDirection, float weighting=1)
{
	double fuzzyDistance = distanceSet->fuzzify((double) distanceVlue);	
	double fuzzyAngle = angleSet->fuzzify((double) angleVlue);	
	double a3 = set3->fuzzify((double) v3);	

	double membership = min(fuzzyDistance,fuzzyAngle); //AND = min in fuzzy logic
	membership = min(a3,membership)*weighting; //AND = min in fuzzy logic

	if (membership > 0)
	{
		//acumilate Speed
		tempSpeed->setSetValue(speedSet->getSetValue()*membership);
		tempSpeed->setWorldVal(speedSet->getWorldVal());
		for(int l=0;l<3;l++)
			acumilateSpeedSet[l]->sugenoMin(*tempSpeed);//was Max why?

		//acumilate Distance
		tempDirection->setWorldVal(directionSet->getWorldVal());
		tempDirection->setSetValue(directionSet->getSetValue()*membership);
		for(int l=0;l<6;l++)
			acumilateDirectionSet[l]->sugenoMin(*tempDirection);//was Max why?
	}
}



void renderAcSet()
{		
	UcPoint place(maxX-60,maxY-10);
	char txt[40]="";
	if(!sugenoAcum)
	{
		gFSList = new CFLS();

		scs=gFSList->add(r1s);	
		scs=gFSList->add(r2s);
		scs=gFSList->add(r3s);
		scs=gFSList->add(r4s);
		scs=gFSList->add(r5s);
		scs=gFSList->add(r6s);
		scs=gFSList->add(r7s);
		scs=gFSList->add(r8s);
		scs=gFSList->add(r9s);
		scs=gFSList->add(r10s);
		scs=gFSList->add(r11s);
		scs=gFSList->add(r12s);
		scs=gFSList->add(r13s);	
		scs=gFSList->add(r14s);
		scs=gFSList->add(r15s);
		scs=gFSList->add(r16s);
		scs=gFSList->add(r17s);
		scs=gFSList->add(r18s);
		scs=gFSList->add(r19s);
		scs=gFSList->add(r20s);
		scs=gFSList->add(r21s);
		scs=gFSList->add(fSetSpeed);	

		scs=gFSList->add(r1d);	
		scs=gFSList->add(r2d);
		scs=gFSList->add(r3d);
		scs=gFSList->add(r4d);
		scs=gFSList->add(r5d);
		scs=gFSList->add(r6d);
		scs=gFSList->add(r7d);
		scs=gFSList->add(r8d);
		scs=gFSList->add(r9d);
		scs=gFSList->add(r10d);
		scs=gFSList->add(r11d);
		scs=gFSList->add(r12d);
		scs=gFSList->add(r13d);	
		scs=gFSList->add(r14d);
		scs=gFSList->add(r15d);
		scs=gFSList->add(r16d);
		scs=gFSList->add(r17d);
		scs=gFSList->add(r18d);
		scs=gFSList->add(r19d);
		scs=gFSList->add(r20d);
		scs=gFSList->add(r21d);
		scs=gFSList->add(fSetDirection);
	}//end if not Sugeno

	if(showRulesSets)
	{
		FsFrame f1(10, 590, 200, 100);
		FsFrame f2(220, 590, 200, 100);	
		
		FsFrame f3(10, 470, 200, 100);
		FsFrame f4(220, 470, 200, 100);

		FsFrame f5(10, 350, 200, 100);	
		FsFrame f6(220, 350, 200, 100);	

		FsFrame f7(10, 230, 200, 100);
		FsFrame f8(220, 230, 200, 100);		
		if(!sugenoAcum)
		{
			f1.setTitle(90, -12, "Obsticals speed Sets");
			f1.graphFS((FuzzySet*)gFSList->get(0), UcColour(UcColour::LightRed));
			f1.graphFS((FuzzySet*)gFSList->get(1), UcColour(UcColour::Green));
			f1.graphFS((FuzzySet*)gFSList->get(2), UcColour(UcColour::Blue));
			f1.graphFS((FuzzySet*)gFSList->get(3), UcColour(UcColour::Yellow));
			f1.graphFS((FuzzySet*)gFSList->get(4), UcColour(UcColour::Magenta));
			f1.graphFS((FuzzySet*)gFSList->get(5), UcColour(UcColour::Cyan));
			

			f3.setTitle(90, -12, "Right wall speed Sets");
			f3.graphFS((FuzzySet*)gFSList->get(6), UcColour(UcColour::DarkGrey));
			f3.graphFS((FuzzySet*)gFSList->get(7), UcColour(UcColour::Gold));

			f5.setTitle(90, -12, "Left wall speed Sets");
			f5.graphFS((FuzzySet*)gFSList->get(8), UcColour(UcColour::Pink));
			f5.graphFS((FuzzySet*)gFSList->get(9), UcColour(UcColour::Brown));
			
			f7.setTitle(90, -12, "Top & Bottom wall speed Sets");
			f7.graphFS((FuzzySet*)gFSList->get(10), UcColour(UcColour::MediumBlue));
			f7.graphFS((FuzzySet*)gFSList->get(11), UcColour(UcColour::LightGreen));
			f7.graphFS((FuzzySet*)gFSList->get(12), UcColour(UcColour::Red));
			f7.graphFS((FuzzySet*)gFSList->get(13), UcColour(UcColour::LightRed));
			
			
			f2.setTitle(90, -12, "Obsticals Direction sets");
			f2.graphFS((FuzzySet*)gFSList->get(22), UcColour(UcColour::Brown));
			f2.graphFS((FuzzySet*)gFSList->get(23), UcColour(UcColour::MediumBlue));
			f2.graphFS((FuzzySet*)gFSList->get(24), UcColour(UcColour::LightGreen));
			f2.graphFS((FuzzySet*)gFSList->get(25), UcColour(UcColour::Red));
			f2.graphFS((FuzzySet*)gFSList->get(26), UcColour(UcColour::Gold));
			f2.graphFS((FuzzySet*)gFSList->get(27), UcColour(UcColour::Cyan));

			f4.setTitle(90, -12, "Right wall Direction sets");
			f4.graphFS((FuzzySet*)gFSList->get(28), UcColour(UcColour::Magenta));
			f4.graphFS((FuzzySet*)gFSList->get(29), UcColour(UcColour::Brown));


			f6.setTitle(90, -12, "Left wall Direction sets");
			f6.graphFS((FuzzySet*)gFSList->get(30), UcColour(UcColour::MediumBlue));
			f6.graphFS((FuzzySet*)gFSList->get(31), UcColour(UcColour::LightGreen));
			
			f8.setTitle(90, -12, "Top & Bottom wall Direction sets");
			f8.graphFS((FuzzySet*)gFSList->get(32), UcColour(UcColour::Red));
			f8.graphFS((FuzzySet*)gFSList->get(33), UcColour(UcColour::Gold));
			f8.graphFS((FuzzySet*)gFSList->get(34), UcColour(UcColour::Cyan));
			f8.graphFS((FuzzySet*)gFSList->get(35), UcColour(UcColour::Magenta));
		}
		else
		{						
			f1.setTitle(90, -12, "Obsticals speed Sets");
			f1.graphFS(sugR1s, UcColour(UcColour::LightRed),1,"sugR1s");
			f1.graphFS(sugR2s, UcColour(UcColour::Green),1,"sugR2s");
			f1.graphFS(sugR3s, UcColour(UcColour::Blue),1,"sugR3s");
			f1.graphFS(sugR4s, UcColour(UcColour::Yellow),1,"sugR4s");
			f1.graphFS(sugR5s, UcColour(UcColour::Magenta),1,"sugR5s");
			f1.graphFS(sugR6s, UcColour(UcColour::Cyan),1,"sugR6s");
			

			f3.setTitle(90, -12, "Right wall speed Sets");
			f3.graphFS(sugR7s, UcColour(UcColour::DarkGrey),1,"sugR7s");
			f3.graphFS(sugR8s, UcColour(UcColour::Gold),1,"sugR8s");

			f5.setTitle(90, -12, "Left wall speed Sets");
			f5.graphFS(sugR9s, UcColour(UcColour::Pink),1,"sugR9s");
			f5.graphFS(sugR10s, UcColour(UcColour::Brown),1,"sugR10s");
			
			f7.setTitle(90, -12, "Top & Bottom wall speed Sets");
			f7.graphFS(sugR11s, UcColour(UcColour::MediumBlue),1,"sugR11s");
			f7.graphFS(sugR12s, UcColour(UcColour::LightGreen),1,"sugR12s");
			f7.graphFS(sugR13s, UcColour(UcColour::Red),1,"sugR13s");
			f7.graphFS(sugR14s, UcColour(UcColour::LightRed),1,"sugR14s");
			
			//column 2 the Direction Rules Sets
			f2.setTitle(90, -12, "Obsticals Direction sets");
			f2.graphFS(sugR1d, UcColour(UcColour::Brown),2,"sugR1d");
			f2.graphFS(sugR2d, UcColour(UcColour::MediumBlue),2,"sugR2d");
			f2.graphFS(sugR3d, UcColour(UcColour::LightGreen),2,"sugR3d");
			f2.graphFS(sugR4d, UcColour(UcColour::Red),2,"sugR4d");
			f2.graphFS(sugR5d, UcColour(UcColour::Gold),2,"sugR5d");
			f2.graphFS(sugR6d, UcColour(UcColour::Cyan),2,"sugR6d");

			f4.setTitle(90, -12, "Right wall Direction sets");
			f4.graphFS(sugR7d, UcColour(UcColour::Magenta),2,"sugR7d");
			f4.graphFS(sugR8d, UcColour(UcColour::Brown),2,"sugR8d");


			f6.setTitle(90, -12, "Left wall Direction sets");
			f6.graphFS(sugR9d, UcColour(UcColour::MediumBlue),2,"sugR9d");
			f6.graphFS(sugR10d, UcColour(UcColour::LightGreen),2,"sugR10d");
			
			f8.setTitle(90, -12, "Top & Bottom wall Direction sets");
			f8.graphFS(sugR11d, UcColour(UcColour::Red),2,"sugR11d");
			f8.graphFS(sugR12d, UcColour(UcColour::Gold),2,"sugR12d");
			f8.graphFS(sugR13d, UcColour(UcColour::Cyan),2,"sugR13d");
			f8.graphFS(sugR14d, UcColour(UcColour::Magenta),2,"sugR14d");
		}
	}

	if(showAccumSets)
	{
		FsFrame f9(10, 110, 200, 100);	
		FsFrame f10(220, 110, 200, 100);		
		if(!sugenoAcum)
		{
			f9.setTitle(90, -12, "acumilated speed Sets");
			f9.graphFS((FuzzySet*)gFSList->get(21), UcColour(UcColour::Red));

			f10.setTitle(90, -12, "acumilated Direction sets");
			f10.graphFS((FuzzySet*)gFSList->get(43), UcColour(UcColour::Red));	UcColour(UcColour::Black).setGlColour();
		}
		else
		{
			for(int k=0;k<3;k++)
			{
				f9.setTitle(90, -12, "acumilated speed Sets");
				f9.graphFS(sugfSetSpeed[k], UcColour(UcColour::Red),1,"");
			}
			for(int k=0;k<7;k++)
			{
				f10.setTitle(90, -12, "acumilated Direction sets");
				f10.graphFS(sugfSetDirection[k], UcColour(UcColour::Red),2,"");
			}
		}
	
		/*
		sprintf(txt,"Speed=%6.3f",move[0].speed);
		UcDrawstr(place.x, place.y, font_style4, txt); 
	
		place.x=maxX-60; place.y=maxY-20;
		sprintf(txt,"Angle=%2.1f",move[j].angle);
		UcDrawstr(place.x, place.y, font_style4, txt); 
			

		place.x=maxX-150; place.y=maxY-10;
		sprintf(txt,"dr=%1.2f",dr);
		UcDrawstr(place.x, place.y, font_style4, txt); 

		place.x=maxX-150; place.y=maxY-20;
		sprintf(txt,"dt=%1.2f",dt);
		UcDrawstr(place.x, place.y, font_style4, txt); 

		place.x=maxX-150; place.y=maxY-30;
		sprintf(txt,"dl=%1.2f",dl);
		UcDrawstr(place.x, place.y, font_style4, txt); 

		place.x=maxX-150; place.y=maxY-40;
		sprintf(txt,"db=%1.2f",db);
		UcDrawstr(place.x, place.y, font_style4, txt); 
		*/
	}
}




void normalRender()
{
	renderAcSet();
}

void normalReset(){	
	
	// constant sets
	one = new FuzzySet("r1s",0,10); one->clearVal(1);
	zero = new FuzzySet("r1s",0,10); zero->clearVal(0);

	//acumilating sets
	fSetSpeed  = new FuzzySet("fSetSpeed",speedRangeL,speedRangeH);
	fSetSpeed->clearVal(0);
	fSetDirection = new FuzzySet("fSetDirection",-7,7);
	fSetDirection->clearVal(0);

	//temprory rules sets 	
	//speed
	r1s  = new FuzzySet("r1s",speedRangeL,speedRangeH); r1s->clearVal(0);
	r2s  = new FuzzySet("r2s",speedRangeL,speedRangeH); r2s->clearVal(0);
	r3s  = new FuzzySet("r3s",speedRangeL,speedRangeH); r3s->clearVal(0);
	r4s  = new FuzzySet("r4s",speedRangeL,speedRangeH); r4s->clearVal(0);
	r5s  = new FuzzySet("r5s",speedRangeL,speedRangeH); r5s->clearVal(0);
	r6s  = new FuzzySet("r6s",speedRangeL,speedRangeH); r6s->clearVal(0);
	r7s  = new FuzzySet("r7s",speedRangeL,speedRangeH); r7s->clearVal(0);
	r8s  = new FuzzySet("r8s",speedRangeL,speedRangeH); r8s->clearVal(0);
	r9s  = new FuzzySet("r9s",speedRangeL,speedRangeH); r9s->clearVal(0);
	r10s  = new FuzzySet("r10s",speedRangeL,speedRangeH); r10s->clearVal(0);
	r11s  = new FuzzySet("r11s",speedRangeL,speedRangeH); r11s->clearVal(0);
	r12s  = new FuzzySet("r12s",speedRangeL,speedRangeH); r12s->clearVal(0);
	r13s  = new FuzzySet("r13s",speedRangeL,speedRangeH); r13s->clearVal(0);
	r14s  = new FuzzySet("r14s",speedRangeL,speedRangeH); r14s->clearVal(0);
	r15s  = new FuzzySet("r15s",speedRangeL,speedRangeH); r15s->clearVal(0);
	r16s  = new FuzzySet("r16s",speedRangeL,speedRangeH); r16s->clearVal(0);
	r17s  = new FuzzySet("r17s",speedRangeL,speedRangeH); r17s->clearVal(0);
	r18s  = new FuzzySet("r18s",speedRangeL,speedRangeH); r18s->clearVal(0);
	r19s  = new FuzzySet("r19s",speedRangeL,speedRangeH); r19s->clearVal(0);
	r20s  = new FuzzySet("r20s",speedRangeL,speedRangeH); r20s->clearVal(0);
	r21s  = new FuzzySet("r21s",speedRangeL,speedRangeH); r21s->clearVal(0);
	
	//direction
	r1d  = new FuzzySet("r1d",-7,7); r1d->clearVal(0);
	r2d  = new FuzzySet("r2d",-7,7); r2d->clearVal(0);
	r3d  = new FuzzySet("r3d",-7,7); r3d->clearVal(0);
	r4d  = new FuzzySet("r4d",-7,7); r4d->clearVal(0);
	r5d  = new FuzzySet("r5d",-7,7); r5d->clearVal(0);
	r6d  = new FuzzySet("r6d",-7,7); r6d->clearVal(0);
	r7d  = new FuzzySet("r7d",-7,7); r7d->clearVal(0);
	r8d  = new FuzzySet("r8d",-7,7); r8d->clearVal(0);
	r9d  = new FuzzySet("r9d",-7,7); r9d->clearVal(0);
	r10d = new FuzzySet("r10d",-7,7); r10d->clearVal(0);
	r11d = new FuzzySet("r11d",-7,7); r11d->clearVal(0);
	r12d = new FuzzySet("r12d",-7,7); r12d->clearVal(0);
	r13d  = new FuzzySet("r13d",-7,7); r13d->clearVal(0);
	r14d  = new FuzzySet("r14d",-7,7); r14d->clearVal(0);
	r15d  = new FuzzySet("r15d",-7,7); r15d->clearVal(0);
	r16d  = new FuzzySet("r16d",-7,7); r16d->clearVal(0);
	r17d  = new FuzzySet("r17d",-7,7); r17d->clearVal(0);
	r18d  = new FuzzySet("r18d",-7,7); r18d->clearVal(0);
	r19d  = new FuzzySet("r19d",-7,7); r19d->clearVal(0);
	r20d  = new FuzzySet("r20d",-7,7); r20d->clearVal(0);
	r21d  = new FuzzySet("r21d",-7,7); r21d->clearVal(0);
	
	//sugeno Reset
	
	//acumilating sets	
	sugfSetSpeed[0]  = new Sugeno(speedRangeL,0);//slowSpeed Sugeno acumilator
	sugfSetSpeed[1]  = new Sugeno(speedRangeH/2,0);//medSpeed Sugeno acumilator
	sugfSetSpeed[2]  = new Sugeno(speedRangeH,0);//fastSpeed Sugeno acumilator

	sugfSetDirection[0] = new Sugeno(-4.5,0);//sharpLeft Sugeno acumilator
	sugfSetDirection[1] = new Sugeno(-2,0);//turnleft Sugeno acumilator
	sugfSetDirection[2] = new Sugeno(-1,0);//slowTurnRight Sugeno acumilator
	sugfSetDirection[3] = new Sugeno(1,0);//slowTurnLeft Sugeno acumilator
	sugfSetDirection[4] = new Sugeno(2,0);//turnright Sugeno acumilator
	sugfSetDirection[5] = new Sugeno(4.5,0);//sharpRight Sugeno acumilator
	sugfSetDirection[6] = new Sugeno(0,0);//noMove Sugeno acumilator

	//temprory rules sets 	
	//speed
	sugR1s  = new Sugeno();
	sugR2s  = new Sugeno();
	sugR3s  = new Sugeno();
	sugR4s  = new Sugeno();
	sugR5s  = new Sugeno();
	sugR6s  = new Sugeno();
	sugR7s  = new  Sugeno();
	sugR8s  = new Sugeno();
	sugR9s  = new Sugeno();
	sugR10s  = new Sugeno();
	sugR11s  = new Sugeno();
	sugR12s  = new Sugeno();
	sugR13s  = new Sugeno();
	sugR14s  = new Sugeno();
	sugR15s  = new Sugeno();
	sugR16s  = new Sugeno();
	sugR17s  = new Sugeno();
	sugR18s  = new Sugeno();
	sugR19s  = new Sugeno();
	sugR20s  = new Sugeno();
	sugR21s  = new Sugeno();

	//direction
	sugR1d  = new Sugeno();
	sugR2d  = new Sugeno();
	sugR3d  = new Sugeno();
	sugR4d  = new Sugeno();
	sugR5d  = new Sugeno();
	sugR6d  = new Sugeno();
	sugR7d  = new Sugeno();
	sugR8d  = new Sugeno();
	sugR9d  = new Sugeno();
	sugR10d  = new Sugeno();
	sugR11d  = new Sugeno();
	sugR12d  = new Sugeno();
	sugR13d  = new Sugeno();
	sugR14d  = new Sugeno();
	sugR15d  = new Sugeno();
	sugR16d  = new Sugeno();
	sugR17d  = new Sugeno();
	sugR18d  = new Sugeno();
	sugR19d  = new Sugeno();
	sugR20d  = new Sugeno();
	sugR21d  = new Sugeno();
}
void distroyAllTemp(){	
	//temprory rules sets 	
	//speed
	r1s->~FuzzySet();
	r2s->~FuzzySet();
	r3s->~FuzzySet();
	r4s->~FuzzySet();
	r5s->~FuzzySet(); 
	r6s->~FuzzySet(); 
	r7s->~FuzzySet(); 
	r8s->~FuzzySet(); 
	r9s->~FuzzySet(); 
	r10s->~FuzzySet();
	r11s->~FuzzySet();
	r12s->~FuzzySet();
	r13s->~FuzzySet();
	r14s->~FuzzySet();
	r15s->~FuzzySet();
	r16s->~FuzzySet();
	r17s->~FuzzySet();
	r18s->~FuzzySet();
	r19s->~FuzzySet();
	r20s->~FuzzySet();
	r21s->~FuzzySet();
	//direction
	r1d->~FuzzySet(); 
	r2d->~FuzzySet(); 
	r3d->~FuzzySet(); 
	r4d->~FuzzySet(); 
	r5d->~FuzzySet(); 
	r6d->~FuzzySet(); 
	r7d->~FuzzySet(); 
	r8d->~FuzzySet(); 
	r9d->~FuzzySet(); 
	r10d->~FuzzySet();
	r11d->~FuzzySet();
	r12d->~FuzzySet();
	r13d->~FuzzySet();
	r14d->~FuzzySet();
	r15d->~FuzzySet();
	r16d->~FuzzySet();
	r17d->~FuzzySet();
	r18d->~FuzzySet();
	r19d->~FuzzySet();
	r20d->~FuzzySet();
	r21d->~FuzzySet();
}
void clearAllTempSets()
{
	r1s->clearVal(0);
	r2s->clearVal(0);
	r3s->clearVal(0);
	r4s->clearVal(0);
	r5s->clearVal(0);
	r6s->clearVal(0);
	r7s->clearVal(0);
	r8s->clearVal(0);
	r9s->clearVal(0);
	r10s->clearVal(0);
	r11s->clearVal(0);
	r12s->clearVal(0);
	r13s->clearVal(0);
	r14s->clearVal(0);
	r15s->clearVal(0);
	r16s->clearVal(0);
	r17s->clearVal(0);
	r18s->clearVal(0);
	r19s->clearVal(0);
	r20s->clearVal(0);
	r21s->clearVal(0);

	
	r1d->clearVal(0);
	r2d->clearVal(0);
	r3d->clearVal(0);
	r4d->clearVal(0);
	r5d->clearVal(0);
	r6d->clearVal(0);
	r7d->clearVal(0);
	r8d->clearVal(0);
	r9d->clearVal(0);
	r10d->clearVal(0);
	r11d->clearVal(0);
	r12d->clearVal(0);
	r13d->clearVal(0);
	r14d->clearVal(0);
	r15d->clearVal(0);
	r16d->clearVal(0);
	r17d->clearVal(0);
	r18d->clearVal(0);
	r19d->clearVal(0);
	r20d->clearVal(0);
	r21d->clearVal(0);
}

void setInit()
{
	//normal fuzzy set initilaization 
	//fuzzy sets for distance from obj
	L=0;H=70; //low and high values
	//very close
	veryClose = new FuzzySet("veryClose",L,H);
	veryClose->addPoint(L,1);
	veryClose->addPoint(20,1);
	veryClose->addPoint(30,0);
	veryClose->addPoint(H,0);

	//close
	close = new FuzzySet("close",L,H);
	close->addPoint(L,0);
	close->addPoint(20,0);
	close->addPoint(30,1);
	close->addPoint(40,1);
	close->addPoint(50,0);
	close->addPoint(H,0);

	//far
	far = new FuzzySet("far",L,H);
	far->addPoint(L,0);
	far->addPoint(40,0);
	far->addPoint(50,1);
	far->addPoint(H,1);

	//fuzzy sets for angle from obj
	L=-90;H=90; //low and high values

	//dangerous angle Left
	dangAngleLeft = new FuzzySet("dangAngleLeft",L,H);
	dangAngleLeft->addPoint(L,0);
	dangAngleLeft->addPoint(-20,0);
	dangAngleLeft->addPoint(-15,1);
	dangAngleLeft->addPoint(5,0);
	dangAngleLeft->addPoint(H,0);
	//dangerous angle Right
	dangAngleRight = new FuzzySet("dangAngleRight",L,H);
	dangAngleRight->addPoint(L,0);
	dangAngleRight->addPoint(-5,0);
	dangAngleRight->addPoint(15,1);
	dangAngleRight->addPoint(20,0);
	dangAngleRight->addPoint(H,0);

	//close angle Left
	closeAngleLeft = new FuzzySet("closeAngleLeft",L,H);
	closeAngleLeft->addPoint(L,0);
	closeAngleLeft->addPoint(-40,0);
	closeAngleLeft->addPoint(-30,1);
	closeAngleLeft->addPoint(5,0);
	closeAngleLeft->addPoint(H,0);
	//close angle Right
	closeAngleRight = new FuzzySet("closeAngleRight",L,H);
	closeAngleRight->addPoint(L,0);
	closeAngleRight->addPoint(-5,0);
	closeAngleRight->addPoint(30,1);
	closeAngleRight->addPoint(40,0);
	closeAngleRight->addPoint(H,0);

	//far angle Left
	farAngleLeft = new FuzzySet("farAngleLeft",L,H);
	farAngleLeft->addPoint(L,1);
	farAngleLeft->addPoint(-60,1);
	farAngleLeft->addPoint(5,0);
	farAngleLeft->addPoint(H,0);

	//far angle Right
	farAngleRight = new FuzzySet("farAngleRight",L,H);
	farAngleRight->addPoint(L,0);
	farAngleRight->addPoint(-5,0);
	farAngleRight->addPoint(60,1);
	farAngleRight->addPoint(H,1);

	//fuzzy sets for speed
	L=speedRangeL; H=speedRangeH; //low and high values
	//slow
	slow = new FuzzySet("slow",L,H);
	slow->addPoint(L,1);
	slow->addPoint(speedRangeH*0.4,0);
	slow->addPoint(H,0);

	//med
	medSpeed = new FuzzySet("medSpeed",L,H);
	medSpeed->addPoint(L,0);
	medSpeed->addPoint(speedRangeH*0.3,0);
	medSpeed->addPoint(speedRangeH*0.5,1);
	medSpeed->addPoint(speedRangeH*0.7,0);
	medSpeed->addPoint(H,0);

	//fast
	fast = new FuzzySet("fast",L,H);
	fast->addPoint(L,0);
	fast->addPoint(speedRangeH*0.6,0);
	fast->addPoint(H,1);

	//fuzzy sets for direction
	L=-7;H=7; //low and high values
	//shrp left
	sharpLeft = new FuzzySet("sharpLeft",L,H);
	sharpLeft->addPoint(L,0);
	sharpLeft->addPoint(-5.5,1);
	sharpLeft->addPoint(-2,0);
	sharpLeft->addPoint(H,0);

	//left 
	turnleft = new FuzzySet("turnleft",L,H);
	turnleft->addPoint(L,0);
	turnleft->addPoint(-4,0);
	turnleft->addPoint(-2,1);
	turnleft->addPoint(0,0);
	turnleft->addPoint(H,0);

	//slowTurnLeft 
	slowTurnLeft = new FuzzySet("slowTurnLeft",L,H);
	slowTurnLeft->addPoint(L,0);
	slowTurnLeft->addPoint(-2,0);
	slowTurnLeft->addPoint(-1,1);
	slowTurnLeft->addPoint(0.5,0);
	slowTurnLeft->addPoint(H,0);

	//slowTurnRight  
	slowTurnRight = new FuzzySet("slowTurnRight",L,H);
	slowTurnRight->addPoint(L,0);
	slowTurnRight->addPoint(-0.5,0);
	slowTurnRight->addPoint(1,1);
	slowTurnRight->addPoint(2,0);
	slowTurnRight->addPoint(H,0);

	//right 
	turnright = new FuzzySet("turnright",L,H);
	turnright->addPoint(L,0);
	turnright->addPoint(0,0);
	turnright->addPoint(2,1);
	turnright->addPoint(4,0);
	turnright->addPoint(H,0);

	//shrp Right
	sharpRight = new FuzzySet("sharpRight",L,H);
	sharpRight->addPoint(L,0);
	sharpRight->addPoint(2.5,0);
	sharpRight->addPoint(4.5,1);
	sharpRight->addPoint(H,0);

	//distance from wall sets
	//closeToWall 
	L=0;H=max(maxX,maxY);
	closeToWall = new FuzzySet("closeToWall",L,H);
	closeToWall->addPoint(L,1);
	//closeToWall->addPoint(20,1);
	closeToWall->addPoint(speedRangeH*15,0);
	closeToWall->addPoint(H,0);

	//closeishToWall  
	closishToWall = new FuzzySet("closishToWall",L,H);
	closishToWall->addPoint(L,1);
	closishToWall->addPoint(H/2+50,0);
	closishToWall->addPoint(H,0);

	//wallGettingCloser
	wallGettingCloser = new FuzzySet("wallGettingCloser",-H,H);
	wallGettingCloser->addPoint(-H,0);
	wallGettingCloser->addPoint(-0.5,0);
	wallGettingCloser->addPoint(0,1);
	wallGettingCloser->addPoint(speedRangeH*1.0,0);
	wallGettingCloser->addPoint(H,0);

	//sugeno init sets for speed
	sugSlow = new Sugeno(0,1);
	sugMedSpeed = new Sugeno(speedRangeH/2,1);
	sugFast = new Sugeno(speedRangeH,1);
	//Sugeno init sets for direction
	sugSharpLeft = new Sugeno(-4.5,1);//sharpLeft Sugeno acumilator
	sugTurnleft = new Sugeno(-2,1);//turnleft Sugeno acumilator
	sugSlowTurnLeft = new Sugeno(-1,1);//slowTurnRight Sugeno acumilator
	sugSlowTurnRight = new Sugeno(1,1);//slowTurnLeft Sugeno acumilator
	sugTurnright = new Sugeno(2,1);//turnright Sugeno acumilator
	sugSharpRight = new Sugeno(4.5,1);//sharpRight Sugeno acumilator
	sugNoMove = new Sugeno(0,1);//noMove Sugeno acumilator

}
void normalInit()
{
	setInit();
	normalReset();
}

void normalKey(unsigned char key, int x, int y)
{
}

void normalTimer(int value)
{
}

void JustNormalIdle()
{
	ticks++;
	clearAllTempSets();
	for (int j=0; j<numMove; j++)
	{		

		fSetSpeed->clearVal(0); 
		fSetSpeed->addPoint(fSetSpeed->getHighRange()*0.8,0.0);
		fSetSpeed->addPoint(fSetSpeed->getHighRange()*0.9,0.1);
		
		fSetDirection->clearVal(0);
		fSetDirection->addPoint(0,0.1);

		//avoid obsticals , Corners and Moving objects
		for (int i=0; i<numObstacle+Corners+numMove; i++)
		{	
			initCal(&move[j], i, j);

			//normal set rules
			//very close distance  rules
			//IF distance IS very close AND angle IS far Left THEN speed IS slow	AND direction IS sharp right
			fuzzyRule2and2((double)calculatedDistance, veryClose, calculatedAngle, farAngleLeft, fSetSpeed, slow, fSetDirection, sharpRight,r1s,r1d);
			//IF distance IS very close AND angle IS far Right THEN speed IS slow	AND direction IS sharp left
			fuzzyRule2and2((double)calculatedDistance, veryClose, calculatedAngle, farAngleRight, fSetSpeed, slow, fSetDirection, sharpLeft,r2s,r2d);

			// close distance rules
			//IF distance IS close AND angle IS close Left THEN speed IS mod AND direction IS  right
			fuzzyRule2and2((double)calculatedDistance, close, calculatedAngle, closeAngleLeft, fSetSpeed, medSpeed, fSetDirection, turnright,r3s,r3d);
			//IF distance IS close		AND angle IS close	Right		THEN speed IS mod	AND direction IS left 
			fuzzyRule2and2((double)calculatedDistance, close, calculatedAngle, closeAngleRight, fSetSpeed, medSpeed, fSetDirection, turnleft,r4s,r4d);

			// far distance rules
			//IF distance IS far AND angle IS dang Left THEN speed IS fast AND direction IS slow right
			fuzzyRule2and2((double)calculatedDistance, far, calculatedAngle, dangAngleLeft, fSetSpeed, fast, fSetDirection, slowTurnRight,r5s,r5d);
			//IF distance IS far AND angle IS dang Right THEN speed IS fast	AND direction IS slow left 
			fuzzyRule2and2((double)calculatedDistance, far, calculatedAngle, dangAngleRight, fSetSpeed, fast, fSetDirection, slowTurnLeft,r6s,r6d);
	
		}
		//IF right distance IS close AND top distance IS closish THEN speed IS slow turn sharp right
		fuzzyRule3and2(dr, closeToWall, dt, closishToWall, drDelta, wallGettingCloser , fSetSpeed, slow, fSetDirection, sharpRight,r7s,r7d);
		//IF right distance IS close AND bottom distance IS closish THEN speed IS slow  turn sharp left
		fuzzyRule3and2(dr, closeToWall, db, closishToWall, drDelta, wallGettingCloser , fSetSpeed, slow, fSetDirection, sharpLeft,r8s,r8d);

		//IF left distance IS close AND top distance IS closish THEN speed IS slow  turn sharp left
		fuzzyRule3and2(dl, closeToWall, dt, closishToWall, dlDelta, wallGettingCloser, fSetSpeed, slow, fSetDirection, sharpLeft,r9s,r9d);
		//IF left distance IS close AND bottom distance IS closish THEN speed IS slow  turn sharp right
		fuzzyRule3and2(dl, closeToWall, db, closishToWall, dlDelta, wallGettingCloser, fSetSpeed, slow, fSetDirection, sharpRight,r10s,r10d);

		//IF top distance IS close AND right distance IS closish THEN speed IS slow  turn sharp left
		fuzzyRule3and2(dt, closeToWall, dr, closishToWall, dtDelta, wallGettingCloser, fSetSpeed, slow, fSetDirection, sharpLeft,r11s,r11d);
		//IF top distance IS close AND left distance IS closish THEN speed IS slow  turn sharp right
		fuzzyRule3and2(dt, closeToWall, dl, closishToWall, dtDelta, wallGettingCloser, fSetSpeed, slow, fSetDirection, sharpRight,r12s,r12d);

		//IF bottom distance IS close right  distance IS closish THEN speed IS slow  turn sharp left
		fuzzyRule3and2(db, closeToWall, dr, closishToWall, dbDelta, wallGettingCloser, fSetSpeed, slow, fSetDirection, sharpRight,r13s,r13d);
		//IF bottom distance IS close left  distance IS closish THEN speed IS slow  turn sharp right
		fuzzyRule3and2(db, closeToWall, dl, closishToWall, dbDelta, wallGettingCloser, fSetSpeed, slow, fSetDirection, sharpLeft,r14s,r14d);

		distroyAllTemp();
		//fuzzify the result set
		fuzzifySpeed=FuzzySet::deFuzzifyCOG(fSetSpeed,20);//move.speed; 
		fuzzifyDirection=FuzzySet::deFuzzifyCOG(fSetDirection,20);//move.angle;

		moveIt(&move[j]);
	}
}

void SugNormalIdle()
{
	ticks++;
	clearAllTempSets();
	for (int j=0; j<numMove; j++)
	{					
		//acumilating sets	
		sugfSetSpeed[0]  = new Sugeno(speedRangeL,0);//slowSpeed Sugeno acumilator
		sugfSetSpeed[1]  = new Sugeno(speedRangeH/2,0);//medSpeed Sugeno acumilator
		sugfSetSpeed[2]  = new Sugeno(speedRangeH,0.1);//fastSpeed Sugeno acumilator

		sugfSetDirection[0] = new Sugeno(-4.5,0);//sharpLeft Sugeno acumilator
		sugfSetDirection[1] = new Sugeno(-2,0);//turnleft Sugeno acumilator
		sugfSetDirection[2] = new Sugeno(-1,0);//slowTurnRight Sugeno acumilator
		sugfSetDirection[3] = new Sugeno(1,0);//slowTurnLeft Sugeno acumilator
		sugfSetDirection[4] = new Sugeno(2,0);//turnright Sugeno acumilator
		sugfSetDirection[5] = new Sugeno(4.5,0);//sharpRight Sugeno acumilator
		sugfSetDirection[6] = new Sugeno(0,0.1);//noMove Sugeno acumilator
		theta = move[j].angle;

		//avoid obsticals
		for (int i=0; i<numObstacle+Corners+numMove; i++)
		{	
			initCal(&move[j], i, j);
			//normal set rules
			//very close distance  rules
			//IF distance IS very close AND angle IS far Left THEN speed IS slow	AND direction IS sharp right
			sugfuzzyRule2and2((double)calculatedDistance, veryClose, calculatedAngle, farAngleLeft, sugfSetSpeed, sugSlow, sugfSetDirection, sugSharpRight,sugR1s,sugR1d);
			//IF distance IS very close AND angle IS far Right THEN speed IS slow	AND direction IS sharp left
			sugfuzzyRule2and2((double)calculatedDistance, veryClose, calculatedAngle, farAngleRight, sugfSetSpeed, sugSlow, sugfSetDirection, sugSharpLeft,sugR2s,sugR2d);

			// close distance rules
			//IF distance IS close AND angle IS close Left THEN speed IS mod AND direction IS  right
			sugfuzzyRule2and2((double)calculatedDistance, close, calculatedAngle, closeAngleLeft, sugfSetSpeed, sugMedSpeed, sugfSetDirection, sugTurnright,sugR3s,sugR3d);
			//IF distance IS close		AND angle IS close	Right		THEN speed IS mod	AND direction IS left 
			sugfuzzyRule2and2((double)calculatedDistance, close, calculatedAngle, closeAngleRight, sugfSetSpeed, sugMedSpeed, sugfSetDirection, sugTurnleft,sugR4s,sugR4d);

			// far distance rules
			//IF distance IS far AND angle IS dang Left THEN speed IS fast AND direction IS slow right
			sugfuzzyRule2and2((double)calculatedDistance, far, calculatedAngle, dangAngleLeft, sugfSetSpeed, sugFast, sugfSetDirection, sugSlowTurnRight,sugR5s,sugR5d);
			//IF distance IS far AND angle IS dang Right THEN speed IS fast	AND direction IS slow left 
			sugfuzzyRule2and2((double)calculatedDistance, far, calculatedAngle, dangAngleRight, sugfSetSpeed, sugFast, sugfSetDirection, sugSlowTurnLeft,sugR6s,sugR6d);
		}

		//IF right distance IS close AND top distance IS closish THEN speed IS slow turn sharp right
		sugfuzzyRule3and2(dr, closeToWall, dt, closishToWall, drDelta, wallGettingCloser , sugfSetSpeed, sugSlow, sugfSetDirection, sugSharpRight,sugR7s,sugR7d);
		//IF right distance IS close AND bottom distance IS closish THEN speed IS slow  turn sharp left
		sugfuzzyRule3and2(dr, closeToWall, db, closishToWall, drDelta, wallGettingCloser , sugfSetSpeed, sugSlow, sugfSetDirection, sugSharpLeft,sugR8s,sugR8d);

		//IF left distance IS close AND top distance IS closish THEN speed IS slow  turn sharp left
		sugfuzzyRule3and2(dl, closeToWall, dt, closishToWall, dlDelta, wallGettingCloser, sugfSetSpeed, sugSlow, sugfSetDirection, sugSharpLeft,sugR9s,sugR9d);
		//IF left distance IS close AND bottom distance IS closish THEN speed IS slow  turn sharp right
		sugfuzzyRule3and2(dl, closeToWall, db, closishToWall, dlDelta, wallGettingCloser, sugfSetSpeed, sugSlow, sugfSetDirection, sugSharpRight,sugR10s,sugR10d);

		//IF top distance IS close AND right distance IS closish THEN speed IS slow  turn sharp left
		sugfuzzyRule3and2(dt, closeToWall, dr, closishToWall, dtDelta, wallGettingCloser, sugfSetSpeed, sugSlow, sugfSetDirection, sugSharpLeft,sugR11s,sugR11d);
		//IF top distance IS close AND left distance IS closish THEN speed IS slow  turn sharp right
		sugfuzzyRule3and2(dt, closeToWall, dl, closishToWall, dtDelta, wallGettingCloser, sugfSetSpeed, sugSlow, sugfSetDirection, sugSharpRight,sugR12s,sugR12d);

		//IF bottom distance IS close right  distance IS closish THEN speed IS slow  turn sharp left
		sugfuzzyRule3and2(db, closeToWall, dr, closishToWall, dbDelta, wallGettingCloser, sugfSetSpeed, sugSlow, sugfSetDirection, sugSharpRight,sugR13s,sugR13d);
		//IF bottom distance IS close left  distance IS closish THEN speed IS slow  turn sharp right
		sugfuzzyRule3and2(db, closeToWall, dl, closishToWall, dbDelta, wallGettingCloser, sugfSetSpeed, sugSlow, sugfSetDirection, sugSharpLeft,sugR14s,sugR14d);

		distroyAllTemp();

		//fuzzify the result set
		fuzzifySpeed=Sugeno::sugenoDefuzzify(sugfSetSpeed[0],sugfSetSpeed[1],sugfSetSpeed[2]);//move.speed; 
		fuzzifyDirection=Sugeno::sugenoDefuzzify(sugfSetDirection[0],sugfSetDirection[1],sugfSetDirection[2],sugfSetDirection[3],sugfSetDirection[4],sugfSetDirection[5],sugfSetDirection[6]);//move.angle;
		//scfuzzifyDirection=Sugeno::sugenoDefuzzify(SCfSetDirection);//move.angle;
		
		moveIt(&move[j]);
	}
}

void normalIdle()
{
	if (stop_sim) return;
	if(!sugenoAcum) JustNormalIdle();
	else SugNormalIdle();
}


void normalExit()
{
}


// end
