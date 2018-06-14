// sim.cpp

#include "normal.h"
#include "UCFL_Utils2.h"
#include "mainCode.h"
#include "math.h"
#include "Sim.h"
#include "shortcut.h"

CFLS *scFSList;
int scs2;
FuzzySet *scfs;

float scdr,scdl,scdb,scdt;

// Normal Fuzzy sets 
//fuzzy sets for distance from obj
float scL,scH; //low and high values

//distanse from obsticals sets
scFuzzySet *SCveryClose, *SCclose, *SCfar ;

//fuzzy sets for angle from obj
//very dangerous angle 
scFuzzySet *SCveryDangAngleLeft ,*SCveryDangAngleRight ;
//dangerous angle 
scFuzzySet *SCdangAngleLeft , *SCdangAngleRight ;
//close angle 
scFuzzySet *SCcloseAngleLeft,*SCcloseAngleRight;
//far angle 
scFuzzySet *SCfarAngleLeft, *SCfarAngleRight;


//fuzzy sets for speed
scFuzzySet *SCslow, *SCmedSpeed, *SCfast ;
Sugeno *sugSCslow, *sugSCmedSpeed, *sugSCfast ;

//fuzzy sets for direction
//shrp turn
scFuzzySet *SCsharpLeft ,*SCsharpRight;
Sugeno *sugSCsharpLeft ,*sugSCsharpRight;
//turn 
scFuzzySet *SCturnleft, *SCturnright;
Sugeno *sugSCturnleft, *sugSCturnright;
//slowTurn 
scFuzzySet *SCslowTurnLeft, *SCslowTurnRight;
Sugeno *sugSCslowTurnLeft, *sugSCslowTurnRight;
//no Move
scFuzzySet *SCnoMove;
Sugeno *sugSCnoMove;

//fuzzy sets for distance of walls
scFuzzySet *SCcloseToWall, *SCclosishToWall ,*SCwallGettingCloser;

//acumilating sets
scFuzzySet *SCfSetSpeed ;
scFuzzySet *SCfSetDirection;
//temproray rules sets
scFuzzySet *r1s,*r2s,*r3s,*r4s,*r5s,*r6s,*r7s,*r8s,*r9s,*r10s,*r11s,*r12s;
scFuzzySet *r13s,*r14s,*r15s,*r16s,*r17s,*r18s,*r19s,*r20s,*r21s;
scFuzzySet *r1d,*r2d,*r3d,*r4d,*r5d,*r6d,*r7d,*r8d,*r9d,*r10d,*r11d,*r12d;
scFuzzySet *r13d,*r14d,*r15d,*r16d,*r17d,*r18d,*r19d,*r20d,*r21d;

//sugeno sets
//acumilating sets
Sugeno *sugSCfSetSpeed[3];
Sugeno *sugSCfSetDirection[7];

//temproray rules sets
Sugeno *segNR1s,*segNR2s,*segNR3s,*segNR4s,*segNR5s,*segNR6s,*segNR7s,*segNR8s,*segNR9s,*segNR10s,*segNR11s,*segNR12s;
Sugeno *segNR13s,*segNR14s,*segNR15s,*segNR16s,*segNR17s,*segNR18s,*segNR19s,*segNR20s,*segNR21s;
Sugeno *segNR1d,*segNR2d,*segNR3d,*segNR4d,*segNR5d,*segNR6d,*segNR7d,*segNR8d,*segNR9d,*segNR10d,*segNR11d,*segNR12d;
Sugeno *segNR13d,*segNR14d,*segNR15d,*segNR16d,*segNR17d,*segNR18d,*segNR19d,*segNR20d,*segNR21d;

scFuzzySet *one;
scFuzzySet *zero;

float sccalculatedDistance,sctheta,scgama, scg;
double sccalculatedAngle, scfuzzifySpeed, scfuzzifyDirection;

const float scspeedRangeH =3.0;
const float scspeedRangeL = 0.0;

const float scDirectionRangeH =7;
const float scDirectionRangeL=-7;
/**
Fuzzy rule with two antecedants with AND and two result
The folloing rule evaluates as follows
IF 	distanceVlue IS in distanceSet AND angleVlue IS angleSet THEN 
    acumilateSpeedSet <-IS speedSet AND_ALSO acumilateDirectionSet <-IS directionSet

*/
void sugscfuzzyRule2and2(double distanceVlue,scFuzzySet *distanceSet, 
			   double angleVlue,scFuzzySet *angleSet,
			   Sugeno *acumilateSpeedSet[3],Sugeno *speedSet,
			   Sugeno *acumilateDirectionSet[7],Sugeno *directionSet,
			   Sugeno *tempSpeed, Sugeno *tempDirection, float weighting=1)
{
	double fuzzyDistance = distanceSet->scfuzzify((double) distanceVlue);	
	double fuzzyAngle = angleSet->scfuzzify((double) angleVlue);	

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
void sugscfuzzyRule3and2(double distanceVlue,scFuzzySet *distanceSet, 
			   double angleVlue,scFuzzySet *angleSet,
			   double v3,scFuzzySet *set3,
			   Sugeno *acumilateSpeedSet[3],Sugeno *speedSet,
			   Sugeno *acumilateDirectionSet[7],Sugeno *directionSet,
			   Sugeno *tempSpeed, Sugeno *tempDirection, float weighting=1)
{
	double fuzzyDistance = distanceSet->scfuzzify((double) distanceVlue);	
	double fuzzyAngle = angleSet->scfuzzify((double) angleVlue);	
	double a3 = set3->scfuzzify((double) v3);	

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


/**
Fuzzy rule with two antecedants with AND and two result
The folloing rule evaluates as follows
IF 	distanceVlue IS in distanceSet AND angleVlue IS angleSet THEN 
    acumilateSpeedSet <-IS speedSet AND_ALSO acumilateDirectionSet <-IS directionSet

*/
void scfuzzyRule2and2(double distanceVlue,scFuzzySet *distanceSet, 
			   double angleVlue,scFuzzySet *angleSet,
			   scFuzzySet *acumilateSpeedSet,scFuzzySet *speedSet,
			   scFuzzySet *acumilateDirectionSet,scFuzzySet *directionSet,
			   scFuzzySet *tempSpeed, scFuzzySet *tempDirection, float weighting=1)
{
	double fuzzyDistance = distanceSet->scfuzzify((double) distanceVlue);	
	double fuzzyAngle = angleSet->scfuzzify((double) angleVlue);	

	double membership = min(fuzzyDistance,fuzzyAngle)*weighting; //AND = min in fuzzy logic

	if (membership > 0)
	{
		//if there is member ship creat temp sets
		scFuzzySet *tempSpeed2, *tempDirection2;
		tempSpeed2 = new scFuzzySet("tempSpeed2",speedSet->getLowRange(),speedSet->getHighRange());
		tempDirection2 = new scFuzzySet("tempDirection2",-7,7);
		tempSpeed2->clearVal(0);
		tempDirection2->clearVal(0);

		//acumilate Speed
		scFuzzySet::copy(tempSpeed2, speedSet);
		scFuzzySet::scaleFS(tempSpeed2,membership);
		scFuzzySet::unionFSsafe(acumilateSpeedSet,tempSpeed2, acumilateSpeedSet);
		scFuzzySet::unionFSsafe(tempSpeed,tempSpeed2, tempSpeed);

		//acumilate Distance
		scFuzzySet::copy(tempDirection2, directionSet);
		scFuzzySet::scaleFS(tempDirection2,membership);
		scFuzzySet::unionFSsafe(acumilateDirectionSet,tempDirection2, acumilateDirectionSet);
		scFuzzySet::unionFSsafe(tempDirection,tempDirection2, tempDirection);
		
		//distroy them later
		tempSpeed2->~scFuzzySet();
		tempDirection2->~scFuzzySet();
	}
}
/**
Fuzzy rule with two antecedants with AND and two result
The folloing rule evaluates as follows
IF 	distanceVlue IS in distanceSet AND angleVlue IS angleSet AND v3 IS set3 THEN 
    acumilateSpeedSet <-IS speedSet AND_ALSO acumilateDirectionSet <-IS directionSet

*/
void scfuzzyRule3and2(double distanceVlue,scFuzzySet *distanceSet, 
			   double angleVlue,scFuzzySet *angleSet,
			   double v3,scFuzzySet *set3,
			   scFuzzySet *acumilateSpeedSet,scFuzzySet *speedSet,
			   scFuzzySet *acumilateDirectionSet,scFuzzySet *directionSet,
			   scFuzzySet *tempSpeed, scFuzzySet *tempDirection, float weighting=1)
{
	double fuzzyDistance = distanceSet->scfuzzify((double) distanceVlue);	
	double fuzzyAngle = angleSet->scfuzzify((double) angleVlue);	
	double a3 = set3->scfuzzify((double) v3);	

	double membership = min(fuzzyDistance,fuzzyAngle); //AND = min in fuzzy logic
	membership = min(a3,membership)*weighting; //AND = min in fuzzy logic

	if (membership > 0)
	{
		//if there is member ship creat temp sets
		scFuzzySet *tempSpeed2, *tempDirection2;
		tempSpeed2 = new scFuzzySet("tempSpeed2",speedSet->getLowRange(),speedSet->getHighRange());
		tempDirection2 = new scFuzzySet("tempDirection2",-7,7);
		tempSpeed2->clearVal(0);
		tempDirection2->clearVal(0);

		//acumilate Speed
		scFuzzySet::copy(tempSpeed2, speedSet);
		scFuzzySet::scaleFS(tempSpeed2,membership);
		scFuzzySet::unionFSsafe(acumilateSpeedSet,tempSpeed2, acumilateSpeedSet);
		scFuzzySet::unionFSsafe(tempSpeed,tempSpeed2, tempSpeed);

		//acumilate Distance
		scFuzzySet::copy(tempDirection2, directionSet);
		scFuzzySet::scaleFS(tempDirection2,membership);
		scFuzzySet::unionFSsafe(acumilateDirectionSet,tempDirection2, acumilateDirectionSet);
		scFuzzySet::unionFSsafe(tempDirection,tempDirection2, tempDirection);
		
		//distroy them later
		tempSpeed2->~scFuzzySet();
		tempDirection2->~scFuzzySet();
	}
}




void scRenderAcSet()
{
	UcPoint place(maxX-60,maxY-10);
	char txt[40]="";
	
	if(!sugenoAcum)
	{
		scFSList = new CFLS();
		
		scfs =new FuzzySet("r1s",scspeedRangeL,scspeedRangeH); r1s->fuzzySet(scfs);scs2=scFSList->add(scfs); 	
		scfs =new FuzzySet("r2s",scspeedRangeL,scspeedRangeH); r2s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r3s",scspeedRangeL,scspeedRangeH); r3s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r4s",scspeedRangeL,scspeedRangeH); r4s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r5s",scspeedRangeL,scspeedRangeH); r5s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r6s",scspeedRangeL,scspeedRangeH); r6s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r7s",scspeedRangeL,scspeedRangeH); r7s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r8s",scspeedRangeL,scspeedRangeH); r8s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r9s",scspeedRangeL,scspeedRangeH); r9s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r10s",scspeedRangeL,scspeedRangeH); r10s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r11s",scspeedRangeL,scspeedRangeH); r11s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r12s",scspeedRangeL,scspeedRangeH); r12s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r13s",scspeedRangeL,scspeedRangeH); r13s->fuzzySet(scfs);scs2=scFSList->add(scfs);	
		scfs =new FuzzySet("r14s",scspeedRangeL,scspeedRangeH); r14s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r15s",scspeedRangeL,scspeedRangeH); r15s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r16s",scspeedRangeL,scspeedRangeH); r16s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r17s",scspeedRangeL,scspeedRangeH); r17s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r18s",scspeedRangeL,scspeedRangeH); r18s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r19s",scspeedRangeL,scspeedRangeH); r19s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r20s",scspeedRangeL,scspeedRangeH); r20s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r21s",scspeedRangeL,scspeedRangeH); r21s->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("SCfSetSpeed",scspeedRangeL,scspeedRangeH); SCfSetSpeed->fuzzySet(scfs);scs2=scFSList->add(scfs);	

		scfs =new FuzzySet("r1d",-7,7); r1d->fuzzySet(scfs);scs2=scFSList->add(scfs);	
		scfs =new FuzzySet("r2d",-7,7); r2d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r3d",-7,7); r3d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r4d",-7,7); r4d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r5d",-7,7); r5d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r6d",-7,7); r6d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r7d",-7,7); r7d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r8d",-7,7); r8d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r9d",-7,7); r9d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r10d",-7,7); r10d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r11d",-7,7); r11d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r12d",-7,7); r12d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r13d",-7,7); r13d->fuzzySet(scfs);scs2=scFSList->add(scfs);	
		scfs =new FuzzySet("r14d",-7,7); r14d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r15d",-7,7); r15d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r16d",-7,7); r16d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r17d",-7,7); r17d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r18d",-7,7); r18d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r19d",-7,7); r19d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r20d",-7,7); r20d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("r21d",-7,7); r21d->fuzzySet(scfs);scs2=scFSList->add(scfs);
		scfs =new FuzzySet("SCfSetDirection",-7,7); SCfSetDirection->fuzzySet(scfs);scs2=scFSList->add(scfs);	

	}
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
		
		if(sugenoAcum)
		{
			f1.setTitle(90, -12, "Obsticals speed Sets");
			f1.graphFS(segNR1s, UcColour(UcColour::LightRed),1,"segNR1s");
			f1.graphFS(segNR2s, UcColour(UcColour::Green),1,"segNR2s");
			f1.graphFS(segNR3s, UcColour(UcColour::Blue),1,"segNR3s");
			f1.graphFS(segNR4s, UcColour(UcColour::Yellow),1,"segNR4s");
			f1.graphFS(segNR5s, UcColour(UcColour::Magenta),1,"segNR5s");
			f1.graphFS(segNR6s, UcColour(UcColour::Cyan),1,"segNR6s");
			

			f3.setTitle(90, -12, "Right wall speed Sets");
			f3.graphFS(segNR7s, UcColour(UcColour::DarkGrey),1,"segNR7s");
			f3.graphFS(segNR8s, UcColour(UcColour::Gold),1,"segNR8s");

			f5.setTitle(90, -12, "Left wall speed Sets");
			f5.graphFS(segNR9s, UcColour(UcColour::Pink),1,"segNR9s");
			f5.graphFS(segNR10s, UcColour(UcColour::Brown),1,"segNR10s");
			
			f7.setTitle(90, -12, "Top & Bottom wall speed Sets");
			f7.graphFS(segNR11s, UcColour(UcColour::MediumBlue),1,"segNR11s");
			f7.graphFS(segNR12s, UcColour(UcColour::LightGreen),1,"segNR12s");
			f7.graphFS(segNR13s, UcColour(UcColour::Red),1,"segNR13s");
			f7.graphFS(segNR14s, UcColour(UcColour::LightRed),1,"segNR14s");
			
			//column 2 the Direction Rules Sets
			f2.setTitle(90, -12, "Obsticals Direction sets");
			f2.graphFS(segNR1d, UcColour(UcColour::Brown),2,"segNR1d");
			f2.graphFS(segNR2d, UcColour(UcColour::MediumBlue),2,"segNR2d");
			f2.graphFS(segNR3d, UcColour(UcColour::LightGreen),2,"segNR3d");
			f2.graphFS(segNR4d, UcColour(UcColour::Red),2,"segNR4d");
			f2.graphFS(segNR5d, UcColour(UcColour::Gold),2,"segNR5d");
			f2.graphFS(segNR6d, UcColour(UcColour::Cyan),2,"segNR6d");

			f4.setTitle(90, -12, "Right wall Direction sets");
			f4.graphFS(segNR7d, UcColour(UcColour::Magenta),2,"segNR7d");
			f4.graphFS(segNR8d, UcColour(UcColour::Brown),2,"segNR8d");


			f6.setTitle(90, -12, "Left wall Direction sets");
			f6.graphFS(segNR9d, UcColour(UcColour::MediumBlue),2,"segNR9d");
			f6.graphFS(segNR10d, UcColour(UcColour::LightGreen),2,"segNR10d");
			
			f8.setTitle(90, -12, "Top & Bottom wall Direction sets");
			f8.graphFS(segNR11d, UcColour(UcColour::Red),2,"segNR11d");
			f8.graphFS(segNR12d, UcColour(UcColour::Gold),2,"segNR12d");
			f8.graphFS(segNR13d, UcColour(UcColour::Cyan),2,"segNR13d");
			f8.graphFS(segNR14d, UcColour(UcColour::Magenta),2,"segNR14d");
		}
		else
		{	
			f1.setTitle(90, -12, "Obsticals speed Sets");
			f1.graphFS((FuzzySet*)scFSList->get(0), UcColour(UcColour::LightRed));
			f1.graphFS((FuzzySet*)scFSList->get(1), UcColour(UcColour::Green));
			f1.graphFS((FuzzySet*)scFSList->get(2), UcColour(UcColour::Blue));
			f1.graphFS((FuzzySet*)scFSList->get(3), UcColour(UcColour::Yellow));
			f1.graphFS((FuzzySet*)scFSList->get(4), UcColour(UcColour::Magenta));
			f1.graphFS((FuzzySet*)scFSList->get(5), UcColour(UcColour::Cyan));
			

			f3.setTitle(90, -12, "Right wall speed Sets");
			f3.graphFS((FuzzySet*)scFSList->get(6), UcColour(UcColour::DarkGrey));
			f3.graphFS((FuzzySet*)scFSList->get(7), UcColour(UcColour::Gold));

			f5.setTitle(90, -12, "Left wall speed Sets");
			f5.graphFS((FuzzySet*)scFSList->get(8), UcColour(UcColour::Pink));
			f5.graphFS((FuzzySet*)scFSList->get(9), UcColour(UcColour::Brown));
			
			f7.setTitle(90, -12, "Top & Bottom wall speed Sets");
			f7.graphFS((FuzzySet*)scFSList->get(10), UcColour(UcColour::MediumBlue));
			f7.graphFS((FuzzySet*)scFSList->get(11), UcColour(UcColour::LightGreen));
			f7.graphFS((FuzzySet*)scFSList->get(12), UcColour(UcColour::Red));
			f7.graphFS((FuzzySet*)scFSList->get(13), UcColour(UcColour::LightRed));
			
			f2.setTitle(90, -12, "Obsticals Direction sets");
			f2.graphFS((FuzzySet*)scFSList->get(22), UcColour(UcColour::Brown));
			f2.graphFS((FuzzySet*)scFSList->get(23), UcColour(UcColour::MediumBlue));
			f2.graphFS((FuzzySet*)scFSList->get(24), UcColour(UcColour::LightGreen));
			f2.graphFS((FuzzySet*)scFSList->get(25), UcColour(UcColour::Red));
			f2.graphFS((FuzzySet*)scFSList->get(26), UcColour(UcColour::Gold));
			f2.graphFS((FuzzySet*)scFSList->get(27), UcColour(UcColour::Cyan));

			f4.setTitle(90, -12, "Right wall Direction sets");
			f4.graphFS((FuzzySet*)scFSList->get(28), UcColour(UcColour::Magenta));
			f4.graphFS((FuzzySet*)scFSList->get(29), UcColour(UcColour::Brown));


			f6.setTitle(90, -12, "Left wall Direction sets");
			f6.graphFS((FuzzySet*)scFSList->get(30), UcColour(UcColour::MediumBlue));
			f6.graphFS((FuzzySet*)scFSList->get(31), UcColour(UcColour::LightGreen));
			
			f8.setTitle(90, -12, "Top & Bottom wall Direction sets");
			f8.graphFS((FuzzySet*)scFSList->get(32), UcColour(UcColour::Red));
			f8.graphFS((FuzzySet*)scFSList->get(33), UcColour(UcColour::Gold));
			f8.graphFS((FuzzySet*)scFSList->get(34), UcColour(UcColour::Cyan));
			f8.graphFS((FuzzySet*)scFSList->get(35), UcColour(UcColour::Magenta));
		}
	}

	if(showAccumSets)
	{
		FsFrame f9(10, 110, 200, 100);	
		FsFrame f10(220, 110, 200, 100);

		if(sugenoAcum)
		{
			for(int k=0;k<3;k++)
			{
				f9.setTitle(90, -12, "acumilated speed Sets");
				f9.graphFS(sugSCfSetSpeed[k], UcColour(UcColour::Red),1,"");
			}
			for(int k=0;k<7;k++)
			{
				f10.setTitle(90, -12, "acumilated Direction sets");
				f10.graphFS(sugSCfSetDirection[k], UcColour(UcColour::Red),2,"");
			}
		}
		else
		{			
			f9.setTitle(90, -12, "acumilated speed Sets");
			f9.graphFS((FuzzySet*)scFSList->get(21), UcColour(UcColour::Red));

			f10.setTitle(90, -12, "acumilated Direction sets");
			f10.graphFS((FuzzySet*)scFSList->get(43), UcColour(UcColour::Red));
		}
		/*
		sprintf(txt,"Speed=%2.1f",move[0].speed);
		UcDrawstr(place.x, place.y, font_style4, txt); 

		place.x=maxX-60; place.y=maxY-20;
		sprintf(txt,"Angle=%2.1f",move[0].angle);
		UcDrawstr(place.x, place.y, font_style4, txt); 


		place.x=maxX-150; place.y=maxY-10;
		sprintf(txt,"dr=%1.2f",scdr);
		UcDrawstr(place.x, place.y, font_style4, txt); 

		place.x=maxX-150; place.y=maxY-20;
		sprintf(txt,"dt=%1.2f",scdt);
		UcDrawstr(place.x, place.y, font_style4, txt); 

		place.x=maxX-150; place.y=maxY-30;
		sprintf(txt,"dl=%1.2f",scdl);
		UcDrawstr(place.x, place.y, font_style4, txt); 

		place.x=maxX-150; place.y=maxY-40;
		sprintf(txt,"db=%1.2f",scdb);
		UcDrawstr(place.x, place.y, font_style4, txt); 
		*/		
	}

}


void shortCutRender()
{
	scRenderAcSet();
}


void shortCutReset()
{		
	
	//acumilating sets	
	SCfSetSpeed  = new scFuzzySet("SCfSetSpeed",scspeedRangeL,scspeedRangeH);
	SCfSetSpeed->clearVal(0);
	SCfSetDirection = new scFuzzySet("SCfSetDirection",-7,7);
	SCfSetDirection->clearVal(0);

	//temprory rules sets 	
	//speed
	r1s  = new scFuzzySet("r1s",scspeedRangeL,scspeedRangeH); r1s->clearVal(0);
	r2s  = new scFuzzySet("r2s",scspeedRangeL,scspeedRangeH); r2s->clearVal(0);
	r3s  = new scFuzzySet("r3s",scspeedRangeL,scspeedRangeH); r3s->clearVal(0);
	r4s  = new scFuzzySet("r4s",scspeedRangeL,scspeedRangeH); r4s->clearVal(0);
	r5s  = new scFuzzySet("r5s",scspeedRangeL,scspeedRangeH); r5s->clearVal(0);
	r6s  = new scFuzzySet("r6s",scspeedRangeL,scspeedRangeH); r6s->clearVal(0);
	r7s  = new scFuzzySet("r7s",scspeedRangeL,scspeedRangeH); r7s->clearVal(0);
	r8s  = new scFuzzySet("r8s",scspeedRangeL,scspeedRangeH); r8s->clearVal(0);
	r9s  = new scFuzzySet("r9s",scspeedRangeL,scspeedRangeH); r9s->clearVal(0);
	r10s  = new scFuzzySet("r10s",scspeedRangeL,scspeedRangeH); r10s->clearVal(0);
	r11s  = new scFuzzySet("r11s",scspeedRangeL,scspeedRangeH); r11s->clearVal(0);
	r12s  = new scFuzzySet("r12s",scspeedRangeL,scspeedRangeH); r12s->clearVal(0);
	r13s  = new scFuzzySet("r13s",scspeedRangeL,scspeedRangeH); r13s->clearVal(0);
	r14s  = new scFuzzySet("r14s",scspeedRangeL,scspeedRangeH); r14s->clearVal(0);
	r15s  = new scFuzzySet("r15s",scspeedRangeL,scspeedRangeH); r15s->clearVal(0);
	r16s  = new scFuzzySet("r16s",scspeedRangeL,scspeedRangeH); r16s->clearVal(0);
	r17s  = new scFuzzySet("r17s",scspeedRangeL,scspeedRangeH); r17s->clearVal(0);
	r18s  = new scFuzzySet("r18s",scspeedRangeL,scspeedRangeH); r18s->clearVal(0);
	r19s  = new scFuzzySet("r19s",scspeedRangeL,scspeedRangeH); r19s->clearVal(0);
	r20s  = new scFuzzySet("r20s",scspeedRangeL,scspeedRangeH); r20s->clearVal(0);
	r21s  = new scFuzzySet("r21s",scspeedRangeL,scspeedRangeH); r21s->clearVal(0);

	//direction
	r1d  = new scFuzzySet("r1d",-7,7); r1d->clearVal(0);
	r2d  = new scFuzzySet("r2d",-7,7); r2d->clearVal(0);
	r3d  = new scFuzzySet("r3d",-7,7); r3d->clearVal(0);
	r4d  = new scFuzzySet("r4d",-7,7); r4d->clearVal(0);
	r5d  = new scFuzzySet("r5d",-7,7); r5d->clearVal(0);
	r6d  = new scFuzzySet("r6d",-7,7); r6d->clearVal(0);
	r7d  = new scFuzzySet("r7d",-7,7); r7d->clearVal(0);
	r8d  = new scFuzzySet("r8d",-7,7); r8d->clearVal(0);
	r9d  = new scFuzzySet("r9d",-7,7); r9d->clearVal(0);
	r10d  = new scFuzzySet("r10d",-7,7); r10d->clearVal(0);
	r11d  = new scFuzzySet("r11d",-7,7); r11d->clearVal(0);
	r12d  = new scFuzzySet("r12d",-7,7); r12d->clearVal(0);
	r13d  = new scFuzzySet("r13d",-7,7); r13d->clearVal(0);
	r14d  = new scFuzzySet("r14d",-7,7); r14d->clearVal(0);
	r15d  = new scFuzzySet("r15d",-7,7); r15d->clearVal(0);
	r16d  = new scFuzzySet("r16d",-7,7); r16d->clearVal(0);
	r17d  = new scFuzzySet("r17d",-7,7); r17d->clearVal(0);
	r18d  = new scFuzzySet("r18d",-7,7); r18d->clearVal(0);
	r19d  = new scFuzzySet("r19d",-7,7); r19d->clearVal(0);
	r20d  = new scFuzzySet("r20d",-7,7); r20d->clearVal(0);
	r21d  = new scFuzzySet("r21d",-7,7); r21d->clearVal(0);

	// constant sets
	one = new scFuzzySet("r1s",0,10); one->setValues(10,10,5,1,0);
	zero = new scFuzzySet("r1s",0,10); zero->setValues(10,10,5,0,0);

	//acumilating sets	
	sugSCfSetSpeed[0]  = new Sugeno(SCslow->getCenter(),0);//slowSpeed Sugeno acumilator
	sugSCfSetSpeed[1]  = new Sugeno(SCmedSpeed->getCenter(),0);//medSpeed Sugeno acumilator
	sugSCfSetSpeed[2]  = new Sugeno(SCfast->getCenter(),0);//fastSpeed Sugeno acumilator

	sugSCfSetDirection[0] = new Sugeno(SCsharpLeft->getCenter(),0);//sharpLeft Sugeno acumilator
	sugSCfSetDirection[1] = new Sugeno(SCturnleft->getCenter(),0);//turnleft Sugeno acumilator
	sugSCfSetDirection[2] = new Sugeno(SCslowTurnRight->getCenter(),0);//slowTurnRight Sugeno acumilator
	sugSCfSetDirection[3] = new Sugeno(SCslowTurnLeft->getCenter(),0);//slowTurnLeft Sugeno acumilator
	sugSCfSetDirection[4] = new Sugeno(SCturnright->getCenter(),0);//turnright Sugeno acumilator
	sugSCfSetDirection[5] = new Sugeno(SCsharpRight->getCenter(),0);//sharpRight Sugeno acumilator
	sugSCfSetDirection[6] = new Sugeno(SCnoMove->getCenter(),0);//noMove Sugeno acumilator

	//temprory rules sets 	
	//speed
	segNR1s  = new Sugeno();
	segNR2s  = new Sugeno();
	segNR3s  = new Sugeno();
	segNR4s  = new Sugeno();
	segNR5s  = new Sugeno();
	segNR6s  = new Sugeno();
	segNR7s  = new Sugeno();
	segNR8s  = new Sugeno();
	segNR9s  = new Sugeno();
	segNR10s  = new Sugeno();
	segNR11s  = new Sugeno();
	segNR12s  = new Sugeno();
	segNR13s  = new Sugeno();
	segNR14s  = new Sugeno();
	segNR15s  = new Sugeno();
	segNR16s  = new Sugeno();
	segNR17s  = new Sugeno();
	segNR18s  = new Sugeno();
	segNR19s  = new Sugeno();
	segNR20s  = new Sugeno();
	segNR21s  = new Sugeno();

	//direction
	segNR1d  = new Sugeno();
	segNR2d  = new Sugeno();
	segNR3d  = new Sugeno();
	segNR4d  = new Sugeno();
	segNR5d  = new Sugeno();
	segNR6d  = new Sugeno();
	segNR7d  = new Sugeno();
	segNR8d  = new Sugeno();
	segNR9d  = new Sugeno();
	segNR10d  = new Sugeno();
	segNR11d  = new Sugeno();
	segNR12d  = new Sugeno();
	segNR13d  = new Sugeno();
	segNR14d  = new Sugeno();
	segNR15d  = new Sugeno();
	segNR16d  = new Sugeno();
	segNR17d  = new Sugeno();
	segNR18d  = new Sugeno();
	segNR19d  = new Sugeno();
	segNR20d  = new Sugeno();
	segNR21d  = new Sugeno();
}
void scdistroyAllTemp(){	
	//temprory rules sets 	
	//speed
	r1s->~scFuzzySet();
	r2s->~scFuzzySet();
	r3s->~scFuzzySet();
	r4s->~scFuzzySet();
	r5s->~scFuzzySet(); 
	r6s->~scFuzzySet(); 
	r7s->~scFuzzySet(); 
	r8s->~scFuzzySet(); 
	r9s->~scFuzzySet(); 
	r10s->~scFuzzySet();
	r11s->~scFuzzySet();
	r12s->~scFuzzySet();
	r13s->~scFuzzySet();
	r14s->~scFuzzySet();
	r15s->~scFuzzySet();
	r16s->~scFuzzySet();
	r17s->~scFuzzySet();
	r18s->~scFuzzySet();
	r19s->~scFuzzySet();
	r20s->~scFuzzySet();
	r21s->~scFuzzySet();
	//direction
	r1d->~scFuzzySet(); 
	r2d->~scFuzzySet(); 
	r3d->~scFuzzySet(); 
	r4d->~scFuzzySet(); 
	r5d->~scFuzzySet(); 
	r6d->~scFuzzySet(); 
	r7d->~scFuzzySet(); 
	r8d->~scFuzzySet(); 
	r9d->~scFuzzySet(); 
	r10d->~scFuzzySet();
	r11d->~scFuzzySet();
	r12d->~scFuzzySet();
	r13d->~scFuzzySet();
	r14d->~scFuzzySet();
	r15d->~scFuzzySet();
	r16d->~scFuzzySet();
	r17d->~scFuzzySet();
	r18d->~scFuzzySet();
	r19d->~scFuzzySet();
	r20d->~scFuzzySet();
	r21d->~scFuzzySet();
}
void scclearAllTempSets()
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

void scSetInit(){
	//ShortCut fuzzy set initilaization 
	//fuzzy sets for distance from obj
	scL=0;scH=70; //low and high values
	//very close
	SCveryClose = new scFuzzySet("SCveryClose",scL,scH);
	SCveryClose->setValues(20,30,0,1,0);

	//close	
	SCclose = new scFuzzySet("SCclose",scL,scH);
	SCclose->setValues(5,15,35,1,0);

	//far
	SCfar = new scFuzzySet("SCfar",scL,scH);
	SCfar->setValues(20,30,scH,1,0);

	//fuzzy sets for angle from obj
	scL=-90;scH=90; //low and high values

	//dangerous angle Left
	SCdangAngleLeft = new scFuzzySet("SCdangAngleLeft",scL,scH);
	SCdangAngleLeft->setValues(0,12.5,-7.5,1,0);

	//dangerous angle Right
	SCdangAngleRight = new scFuzzySet("SCdangAngleRight",scL,scH);
	SCdangAngleRight->setValues(0,12.5,7.5,1,0);

	//close angle Left
	SCcloseAngleLeft = new scFuzzySet("SCcloseAngleLeft",scL,scH);
	SCcloseAngleLeft->setValues(0,22.5,-17.5,1,0);

	//close angle Right
	SCcloseAngleRight = new scFuzzySet("SCcloseAngleRight",scL,scH);
	SCcloseAngleRight->setValues(0,22.5,17.5,1,0);

	//far angle Left
	SCfarAngleLeft = new scFuzzySet("SCfarAngleLeft",scL,scH);
	SCfarAngleLeft->setValues(30,95,-90,1,0);

	//far angle Right
	SCfarAngleRight = new scFuzzySet("SCfarAngleRight",scL,scH);
	SCfarAngleRight->setValues(30,95,90,1,0);

	//fuzzy sets for speed
	scL=scspeedRangeL; scH=scspeedRangeH;  //low and high values
	//slow
	SCslow = new scFuzzySet("SCslow",scL,scH);
	SCslow->setValues(0,scspeedRangeH*0.4,0,1,0);
	sugSCslow = new Sugeno(SCslow);

	//med
	SCmedSpeed = new scFuzzySet("SCmedSpeed",scL,scH);
	SCmedSpeed->setValues(0,scspeedRangeH*0.2,scspeedRangeH*0.5,1,0);
	sugSCmedSpeed = new Sugeno(SCmedSpeed);

	//fast
	SCfast = new scFuzzySet("SCfast",scL,scH);
	SCfast->setValues(0,scspeedRangeH*0.4,scH,1,0);
	sugSCfast = new Sugeno(SCfast);

	//fuzzy sets for direction
	scL=-7;scH=7; //low and high values
	//sharp left
	SCsharpLeft = new scFuzzySet("SCsharpLeft",scL,scH);
	SCsharpLeft->setValues(0,2.5,-4.5,1,0);
	sugSCsharpLeft = new Sugeno(SCsharpLeft);


	//left 
	SCturnleft = new scFuzzySet("SCturnleft",scL,scH);
	SCturnleft->setValues(0,2,-2,1,0);
	sugSCturnleft = new Sugeno(SCturnleft);

	//slowTurnLeft 	
	SCslowTurnLeft = new scFuzzySet("SCslowTurnLeft",scL,scH);
	SCslowTurnLeft->setValues(0,1.25,-1,1,0);
	sugSCslowTurnLeft = new Sugeno(SCslowTurnLeft);

	//slowTurnRight  
	SCslowTurnRight = new scFuzzySet("SCslowTurnRight",scL,scH);
	SCslowTurnRight->setValues(0,1.25,1,1,0);
	sugSCslowTurnRight = new Sugeno(SCslowTurnRight);

	//right 
	SCturnright = new scFuzzySet("SCturnright",scL,scH);
	SCturnright->setValues(0,2,2,1,0);
	sugSCturnright = new Sugeno(SCturnright);

	//shrp Right
	SCsharpRight = new scFuzzySet("SCsharpRight",scL,scH);
	SCsharpRight->setValues(0,2.5,4.5,1,0);
	sugSCsharpRight = new Sugeno(SCsharpRight);

	//no Moving angle
	SCnoMove = new scFuzzySet("SCnoMove",scL,scH);
	SCnoMove->setValues(0,0.5,0,1,0);
	sugSCnoMove = new Sugeno(SCnoMove);

	//distance from wall sets
	scL=0;scH=max(maxX,maxY);
	//closeToWall 
	SCcloseToWall = new scFuzzySet("SCcloseToWall",scL,scH);
	SCcloseToWall->setValues(0,scspeedRangeH*15,0,1,0);

	//closeishToWall  
	SCclosishToWall = new scFuzzySet("SCclosishToWall",scL,scH);
	SCclosishToWall->setValues(0,(scH/2+50),0,1,0);

	//wallGettingCloser
	SCwallGettingCloser = new scFuzzySet("SCwallGettingCloser",-scH,scH);
	SCwallGettingCloser->setValues(0,scspeedRangeH/1.9,scspeedRangeH/2,1,0);
}
void shortCutInit()
{
	scSetInit();
	shortCutReset();
}

void shortCutKey(unsigned char key, int x, int y)
{
	
}

void shortCutTimer(int value)
{
}

void JustShortCutIdle()
{
	ticks++;
	scclearAllTempSets();
	for (int j=0; j<numMove; j++)
	{		
		SCfSetSpeed->clearVal(0); 
		SCfSetSpeed->setValues(0,SCfSetSpeed->getHighRange()*0.05,SCfSetSpeed->getHighRange()*0.85,0.1,0);

		SCfSetDirection->clearVal(0);
		SCfSetDirection->setValues(0,SCfSetDirection->getHighRange()*0.5,0,0.1,0);

		//avoid obsticals
		for (int i=0; i<numObstacle+Corners+numMove; i++)
		{		
			initCal(&move[j], i, j);

			//Short Cut set rules
			//very close distance  rules
			//IF distance IS very close AND angle IS far Left THEN speed IS slow	AND direction IS sharp right
			scfuzzyRule2and2((double)calculatedDistance, SCveryClose, calculatedAngle, SCfarAngleLeft, SCfSetSpeed, SCslow, SCfSetDirection, SCsharpRight,r1s,r1d);
			//IF distance IS very close AND angle IS far Right THEN speed IS slow	AND direction IS sharp left
			scfuzzyRule2and2((double)calculatedDistance, SCveryClose, calculatedAngle, SCfarAngleRight, SCfSetSpeed, SCslow, SCfSetDirection, SCsharpLeft,r2s,r2d);

			// close distance rules
			//IF distance IS close AND angle IS close Left THEN speed IS mod AND direction IS  right
			scfuzzyRule2and2((double)calculatedDistance, SCclose, calculatedAngle, SCcloseAngleLeft, SCfSetSpeed, SCmedSpeed, SCfSetDirection, SCturnright,r3s,r3d);
			//IF distance IS close		AND angle IS close	Right		THEN speed IS mod	AND direction IS left 
			scfuzzyRule2and2((double)calculatedDistance, SCclose, calculatedAngle, SCcloseAngleRight, SCfSetSpeed, SCmedSpeed, SCfSetDirection, SCturnleft,r4s,r4d);

			// far distance rules
			//IF distance IS far AND angle IS dang Left THEN speed IS fast AND direction IS slow right
			scfuzzyRule2and2((double)calculatedDistance, SCfar, calculatedAngle, SCdangAngleLeft, SCfSetSpeed, SCfast, SCfSetDirection, SCslowTurnRight,r5s,r5d);
			//IF distance IS far AND angle IS dang Right THEN speed IS fast	AND direction IS slow left 
			scfuzzyRule2and2((double)calculatedDistance, SCfar, calculatedAngle, SCdangAngleRight, SCfSetSpeed, SCfast, SCfSetDirection, SCslowTurnLeft,r6s,r6d);
	
		}//end of obstical for loop 

		//IF right distance IS close AND top distance IS closish THEN speed IS slow turn sharp right
		scfuzzyRule3and2(dr, SCcloseToWall, dt, SCclosishToWall, drDelta, SCwallGettingCloser , SCfSetSpeed, SCslow, SCfSetDirection, SCsharpRight,r7s,r7d);
		//IF right distance IS close AND bottom distance IS closish THEN speed IS slow  turn sharp left
		scfuzzyRule3and2(dr, SCcloseToWall, db, SCclosishToWall, drDelta, SCwallGettingCloser , SCfSetSpeed, SCslow, SCfSetDirection, SCsharpLeft,r8s,r8d);

		//IF left distance IS close AND top distance IS closish THEN speed IS slow  turn sharp left
		scfuzzyRule3and2(dl, SCcloseToWall, dt, SCclosishToWall, dlDelta, SCwallGettingCloser, SCfSetSpeed, SCslow, SCfSetDirection, SCsharpLeft,r9s,r9d);
		//IF left distance IS close AND bottom distance IS closish THEN speed IS slow  turn sharp right
		scfuzzyRule3and2(dl, SCcloseToWall, db, SCclosishToWall, dlDelta, SCwallGettingCloser, SCfSetSpeed, SCslow, SCfSetDirection, SCsharpRight,r10s,r10d);

		//IF top distance IS close AND right distance IS closish THEN speed IS slow  turn sharp left
		scfuzzyRule3and2(dt, SCcloseToWall, dr, SCclosishToWall, dtDelta, SCwallGettingCloser, SCfSetSpeed, SCslow, SCfSetDirection, SCsharpLeft,r11s,r11d);
		//IF top distance IS close AND left distance IS closish THEN speed IS slow  turn sharp right
		scfuzzyRule3and2(dt, SCcloseToWall, dl, SCclosishToWall, dtDelta, SCwallGettingCloser, SCfSetSpeed, SCslow, SCfSetDirection, SCsharpRight,r12s,r12d);

		//IF bottom distance IS close right  distance IS closish THEN speed IS slow  turn sharp left
		scfuzzyRule3and2(db, SCcloseToWall, dr, SCclosishToWall, dbDelta, SCwallGettingCloser, SCfSetSpeed, SCslow, SCfSetDirection, SCsharpLeft,r13s,r13d);
		//IF bottom distance IS close left  distance IS closish THEN speed IS slow  turn sharp right
		scfuzzyRule3and2(db, SCcloseToWall, dl, SCclosishToWall, dbDelta, SCwallGettingCloser, SCfSetSpeed, SCslow, SCfSetDirection, SCsharpRight,r14s,r14d);

		scdistroyAllTemp();
		//fuzzify the result set
		fuzzifySpeed=scFuzzySet::scDefuzzify(SCfSetSpeed);//move.speed; 
		fuzzifyDirection=scFuzzySet::scDefuzzify(SCfSetDirection);//move.angle;
		
		moveIt(&move[j]);
	}//moveing objects for loop
}//function end



void SugShortCutIdle()
{	
	if (stop_sim) return;
	ticks++;
	scclearAllTempSets();
	for (int j=0; j<numMove; j++)
	{		
		//acumilating sets	
		sugSCfSetSpeed[0]  = new Sugeno(SCslow->getCenter(),0);//slowSpeed Sugeno acumilator
		sugSCfSetSpeed[1]  = new Sugeno(SCmedSpeed->getCenter(),0);//medSpeed Sugeno acumilator
		sugSCfSetSpeed[2]  = new Sugeno(SCfast->getCenter(),0.1);//fastSpeed Sugeno acumilator

		sugSCfSetDirection[0] = new Sugeno(SCsharpLeft->getCenter(),0);//sharpLeft Sugeno acumilator
		sugSCfSetDirection[1] = new Sugeno(SCturnleft->getCenter(),0);//turnleft Sugeno acumilator
		sugSCfSetDirection[2] = new Sugeno(SCslowTurnRight->getCenter(),0);//slowTurnRight Sugeno acumilator
		sugSCfSetDirection[3] = new Sugeno(SCslowTurnLeft->getCenter(),0);//slowTurnLeft Sugeno acumilator
		sugSCfSetDirection[4] = new Sugeno(SCturnright->getCenter(),0);//turnright Sugeno acumilator
		sugSCfSetDirection[5] = new Sugeno(SCsharpRight->getCenter(),0);//sharpRight Sugeno acumilator
		sugSCfSetDirection[6] = new Sugeno(SCnoMove->getCenter(),0.1);//no change angle Sugeno acumilator

		//avoid obsticals
		for (int i=0; i<numObstacle+Corners+numMove; i++)
		{		
			initCal(&move[j], i, j);
			//Short Cut set rules
			//very close distance  rules
			//IF distance IS very close AND angle IS far Left THEN speed IS slow	AND direction IS sharp right
			sugscfuzzyRule2and2((double)calculatedDistance, SCveryClose, calculatedAngle, SCfarAngleLeft, sugSCfSetSpeed, sugSCslow, sugSCfSetDirection, sugSCsharpRight,segNR1s,segNR1d);
			//IF distance IS very close AND angle IS far Right THEN speed IS slow	AND direction IS sharp left
			sugscfuzzyRule2and2((double)calculatedDistance, SCveryClose, calculatedAngle, SCfarAngleRight, sugSCfSetSpeed, sugSCslow, sugSCfSetDirection, sugSCsharpLeft,segNR2s,segNR2d);

			// close distance rules
			//IF distance IS close AND angle IS close Left THEN speed IS mod AND direction IS  right
			sugscfuzzyRule2and2((double)calculatedDistance, SCclose, calculatedAngle, SCcloseAngleLeft, sugSCfSetSpeed, sugSCmedSpeed, sugSCfSetDirection, sugSCturnright,segNR3s,segNR3d);
			//IF distance IS close		AND angle IS close	Right		THEN speed IS mod	AND direction IS left 
			sugscfuzzyRule2and2((double)calculatedDistance, SCclose, calculatedAngle, SCcloseAngleRight, sugSCfSetSpeed, sugSCmedSpeed, sugSCfSetDirection, sugSCturnleft,segNR4s,segNR4d);

			// far distance rules
			//IF distance IS far AND angle IS dang Left THEN speed IS fast AND direction IS slow right
			sugscfuzzyRule2and2((double)calculatedDistance, SCfar, calculatedAngle, SCdangAngleLeft, sugSCfSetSpeed, sugSCfast, sugSCfSetDirection, sugSCslowTurnRight,segNR5s,segNR5d);
			//IF distance IS far AND angle IS dang Right THEN speed IS fast	AND direction IS slow left 
			sugscfuzzyRule2and2((double)calculatedDistance, SCfar, calculatedAngle, SCdangAngleRight, sugSCfSetSpeed, sugSCfast, sugSCfSetDirection, sugSCslowTurnLeft,segNR6s,segNR6d);
		
		}//end of obstical for loop 

		//IF right distance IS close AND top distance IS closish THEN speed IS slow turn sharp right
		sugscfuzzyRule3and2(dr, SCcloseToWall, dt, SCclosishToWall, drDelta, SCwallGettingCloser , sugSCfSetSpeed, sugSCslow, sugSCfSetDirection, sugSCsharpRight,segNR7s,segNR7d);
		//IF right distance IS close AND bottom distance IS closish THEN speed IS slow  turn sharp left
		sugscfuzzyRule3and2(dr, SCcloseToWall, db, SCclosishToWall, drDelta, SCwallGettingCloser , sugSCfSetSpeed, sugSCslow, sugSCfSetDirection, sugSCsharpLeft,segNR8s,segNR8d);

		//IF left distance IS close AND top distance IS closish THEN speed IS slow  turn sharp left
		sugscfuzzyRule3and2(dl, SCcloseToWall, dt, SCclosishToWall, dlDelta, SCwallGettingCloser, sugSCfSetSpeed, sugSCslow, sugSCfSetDirection, sugSCsharpLeft,segNR9s,segNR9d);
		//IF left distance IS close AND bottom distance IS closish THEN speed IS slow  turn sharp right
		sugscfuzzyRule3and2(dl, SCcloseToWall, db, SCclosishToWall, dlDelta, SCwallGettingCloser, sugSCfSetSpeed, sugSCslow, sugSCfSetDirection, sugSCsharpRight,segNR10s,segNR10d);

		//IF top distance IS close AND right distance IS closish THEN speed IS slow  turn sharp left
		sugscfuzzyRule3and2(dt, SCcloseToWall, dr, SCclosishToWall, dtDelta, SCwallGettingCloser, sugSCfSetSpeed, sugSCslow, sugSCfSetDirection, sugSCsharpLeft,segNR11s,segNR11d);
		//IF top distance IS close AND left distance IS closish THEN speed IS slow  turn sharp right
		sugscfuzzyRule3and2(dt, SCcloseToWall, dl, SCclosishToWall, dtDelta, SCwallGettingCloser, sugSCfSetSpeed, sugSCslow, sugSCfSetDirection, sugSCsharpRight,segNR12s,segNR12d);

		//IF bottom distance IS close right  distance IS closish THEN speed IS slow  turn sharp left
		sugscfuzzyRule3and2(db, SCcloseToWall, dr, SCclosishToWall, dbDelta, SCwallGettingCloser, sugSCfSetSpeed, sugSCslow, sugSCfSetDirection, sugSCsharpRight,segNR13s,segNR13d);
		//IF bottom distance IS close left  distance IS closish THEN speed IS slow  turn sharp right
		sugscfuzzyRule3and2(db, SCcloseToWall, dl, SCclosishToWall, dbDelta, SCwallGettingCloser, sugSCfSetSpeed, sugSCslow, sugSCfSetDirection, sugSCsharpLeft,segNR14s,segNR14d);

		scdistroyAllTemp();
		//fuzzify the result set
		fuzzifySpeed=Sugeno::sugenoDefuzzify(sugSCfSetSpeed[0],sugSCfSetSpeed[1],sugSCfSetSpeed[2]);//move.speed; 
		fuzzifyDirection=Sugeno::sugenoDefuzzify(sugSCfSetDirection[0],sugSCfSetDirection[1],sugSCfSetDirection[2],sugSCfSetDirection[3],sugSCfSetDirection[4],sugSCfSetDirection[5],sugSCfSetDirection[6]);//move.angle;
		//scfuzzifyDirection=Sugeno::sugenoDefuzzify(SCfSetDirection);//move.angle;
		
		moveIt(&move[j]);

	}//moveing objects for loop
}//function end

void shortCutIdle()
{		
	if (stop_sim) return;
	if(!sugenoAcum) JustShortCutIdle();
	else SugShortCutIdle();	
}//function end
void shortCutExit()
{
}


// end
