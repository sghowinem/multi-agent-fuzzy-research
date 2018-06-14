// MainCode.h

#include "mainCode.h"
#include "Sim.h"
#include "normal.h"
#include "shortCut.h"
#include "BlitAndSprite.h"

CFLS *FSList;
int ss;
// ****************************** FsFrame ******************************************* //

FsFrame::FsFrame(float leftZ, float topZ, float widthZ, float heightZ)
{
  count=0;
  lowR=0;
  highR=0;

  r.setValues(leftZ, topZ, leftZ+widthZ, topZ-heightZ);

  rightBorder=2;
  leftBorder=20;
  topBorder=42;
  bottomBorder=15;

  textOffsetTitle.x=3;
  textOffsetTitle.y=-24;
  
  textOffsetSets.x=3;
  textHeight =12;
  textOffsetSets.y=0;
  
  title[0]=0;

  frameColour.namedColour(UcColour::Black);
  axisColour.namedColour(UcColour::Brown);
  titleColour.namedColour(UcColour::MediumBlue);

}

void FsFrame::drawFrame()
{
  computeBounds();

  r.draw(frameColour);
  axisColour.setGlColour();
  glBegin(GL_LINES);
    glVertex2f(lowX,lowY-1);
    glVertex2f(highX,lowY-1);
    glVertex2f(lowX-1,lowY);
 	glVertex2f(lowX-1,highY);
  glEnd();
  UcPoint place(r.left+textOffsetTitle.x,r.top+textOffsetTitle.y);
  UcDrawstr(place.x, place.y, font_style3, title);
}

void FsFrame::reset()
{
	count=0;
}

bool FsFrame::graphFS(scFuzzySet *fs, UcColour c)
{
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  if (count==0) drawFrame(); 

  computeBounds(); 

return true;
}

void FsFrame::computeBounds()
{
  lowX = r.left+leftBorder;
  highX = r.right-rightBorder;
  lowY= r.bottom+bottomBorder;
  highY= r.top-topBorder;
}

/**
* Graphs a fuzzy set return true if it succeeds
*/

bool FsFrame::graphFS(FuzzySet *fs, UcColour c)
{
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glDisable(GL_TEXTURE_2D);

  if (count==0){
	  drawFrame();

	  //drowing Y axces values
	  float yl_x1,yl_y1,yl_x2;
	  yl_x1=lowX-2;
	  yl_x2=lowX+1;
	  yl_y1=lowY;
	  float yInc=abs(lowY-highY)/2;
	  glBegin(GL_LINES);
	   for (int yi=0;yi<2;yi++)
		   {
				yl_y1+=yInc;
				glVertex2f(yl_x1,yl_y1);
				glVertex2f(yl_x2,yl_y1);  
		   }
	  glEnd();

	  UcPoint place1(lowX-18,lowY);	  
	  char ytxt[40]="";
	  float yLInc=0.5;
  	  for (int yti=0;yti<=2;yti++)
	  {
		sprintf(ytxt,"%1.1f",(yLInc*yti));
		UcDrawstr(place1.x, place1.y, font_style4, ytxt); 
		place1.y+=yInc;				  
	  }
	  //drowing X axces values
	  float xl_x,xl_y1,xl_y2;
	  xl_x=lowX;
	  xl_y1=lowY+1;
	  xl_y2=lowY-4;
	  float xInc=abs(lowX-highX)/6;
	  glBegin(GL_LINES);
	   for (int xi=0;xi<6;xi++)
		   {
				xl_x+=xInc;
				glVertex2f(xl_x,xl_y1);
				glVertex2f(xl_x,xl_y2);  
		   }
	  glEnd();

	  UcPoint place2(lowX-15,lowY-13);
	  char xtxt[40]="";
	  float xLInc=abs(fs->getLowRange()-fs->getHighRange())/6;
  	  for (int xi=0;xi<=6;xi++)
	  {
		  if(fs->getHighRange()<=1)
			sprintf(xtxt,"%3.2f",fs->getLowRange()+(xLInc*xi));
		  else
			sprintf(xtxt,"%3.1f",fs->getLowRange()+(xLInc*xi));
		UcDrawstr(place2.x, place2.y, font_style4, xtxt); 
		place2.x+=xInc;				  
	  }
  }
 
  computeBounds();

  c.setGlColour();
  UcPoint place(r.left+textOffsetSets.x,r.top-textOffsetSets.y-((count+1)*textHeight));
  UcDrawstr(place.x, place.y, font_style3, fs->getName());

  count++;
  
  int n=fs->getNumPoints();
  if (n<=1) return false;

  c.setGlColour();
  int i;
  float p1x, p1y, p2x, p2y;
  double w1,w2; // world val
  double s1,s2; // set val 

  double lowR = fs->getLowRange();
  double highR = fs->getHighRange();
  float lerpV1, lerpV2;
  
  glBegin(GL_LINES);		
        w1 = fs->getWorldVal(0);
		s1 = fs->getSetVal(0);
        s1 = UcClamp(s1,0,1); // just in case
		w1 = UcClamp(w1,lowR,highR);
  for (i=1; i<n; i++)
	  {
		w2 = fs->getWorldVal(i);
		s2 = fs->getSetVal(i);
        s2 = UcClamp(s2,0,1); // just in case
		w2 = UcClamp(w2,lowR,highR);
		lerpV1 = (w1 - lowR)/(highR-lowR); 
		lerpV2 = (w2 - lowR)/(highR-lowR);

		p1x = UcLerp(lowX,highX,lerpV1); 
		p1y = UcLerp(lowY,highY,s1); 
		p2x = UcLerp(lowX,highX,lerpV2);
		p2y = UcLerp(lowY,highY,s2);

		glVertex2f(p1x,p1y);
        glVertex2f(p2x,p2y);
		s1=s2;
		w1=w2;
	  }
  glEnd();
return true;
}

bool FsFrame::graphFS(Sugeno *fs, UcColour c, int b, char* nameSet)
{
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
  glDisable(GL_TEXTURE_2D);
  float xLInc, highRange,lowRange;
  if(b==1){
	  xLInc=abs(scspeedRangeH-scspeedRangeL)/6;
	  highRange = scspeedRangeH;
	  lowRange = scspeedRangeL;
  }
  else if(b==2){
	  xLInc=abs(scDirectionRangeH-scDirectionRangeL)/6;
	  highRange = scDirectionRangeH;
	  lowRange = scDirectionRangeL;
  }
  else{
	  xLInc=0.5;
	  highRange = 10;
	  lowRange = 0;
  }
  if (count==0)
  {
	  drawFrame();

	  //drowing Y axces values
	  float yl_x1,yl_y1,yl_x2;
	  yl_x1=lowX-2;
	  yl_x2=lowX+1;
	  yl_y1=lowY;
	  float yInc=abs(lowY-highY)/2;
	  glBegin(GL_LINES);
	   for (int yi=0;yi<2;yi++)
		   {
				yl_y1+=yInc;
				glVertex2f(yl_x1,yl_y1);
				glVertex2f(yl_x2,yl_y1);  
		   }
	  glEnd();

	  UcPoint place1(lowX-18,lowY);	  
	  char ytxt[40]="";
	  float yLInc=0.5;
  	  for (int yti=0;yti<=2;yti++)
	  {
		sprintf(ytxt,"%1.1f",(yLInc*yti));
		UcDrawstr(place1.x, place1.y, font_style4, ytxt); 
		place1.y+=yInc;				  
	  }
	  //drowing X axces values
	  float xl_x,xl_y1,xl_y2;
	  xl_x=lowX;
	  xl_y1=lowY+1;
	  xl_y2=lowY-4;
	  float xInc=abs(lowX-highX)/6;
	  glBegin(GL_LINES);
	   for (int xi=0;xi<6;xi++)
		   {
				xl_x+=xInc;
				glVertex2f(xl_x,xl_y1);
				glVertex2f(xl_x,xl_y2);  
		   }
	  glEnd();

	  UcPoint place2(lowX-15,lowY-13);
	  char xtxt[40]="";
  	  for (int xi=0;xi<=6;xi++)
	  {
		  if(highRange<=1)
			sprintf(xtxt,"%3.2f",lowRange+(xLInc*xi));
		  else
			sprintf(xtxt,"%3.1f",lowRange+(xLInc*xi));
		UcDrawstr(place2.x, place2.y, font_style4, xtxt); 
		place2.x+=xInc;				  
	  }
  }
 
  computeBounds();
  count++;
	//sprintf(nameSet,"%i",count);
  c.setGlColour();
  UcPoint place(r.left+textOffsetSets.x,r.top-textOffsetSets.y-((count)*textHeight));
  UcDrawstr(place.x, place.y, font_style3, nameSet);

  
  c.setGlColour();
  float p1x, p1y, p2x, p2y;
  double w1,w2; // world val
  double s1,s2; // set val 

  double lowR = lowRange;//fs->getLowRange();
  double highR = highRange;//fs->getHighRange();
  float lerpV1, lerpV2;
  
  glBegin(GL_LINES);		
        w1 = fs->getWorldVal();//getWorldVal(0);
		s1 = 0;//fs->getSetValue();//getSetVal(0);
        s1 = UcClamp(s1,0,1); // just in case
		w1 = UcClamp(w1,lowR,highR);

		w2 = fs->getWorldVal();
		s2 = fs->getSetValue();
        s2 = UcClamp(s2,0,1); // just in case
		w2 = UcClamp(w2,lowR,highR);

		lerpV1 = (w1 - lowR)/(highR-lowR); 
		lerpV2 = (w2 - lowR)/(highR-lowR);

		p1x = UcLerp(lowX,highX,lerpV1); 
		p1y = UcLerp(lowY,highY,s1); 
		p2x = UcLerp(lowX,highX,lerpV2);
		p2y = UcLerp(lowY,highY,s2);

		glVertex2f(p1x,p1y);
        glVertex2f(p2x,p2y);
  glEnd();

return true;
}

void FsFrame::setTitle(float offsetX, float offsetY, char* titleZ)
{
	strcpy(title,titleZ);
	textOffsetTitle.x=offsetX;
	textOffsetTitle.y=offsetY;
}

// +++++++++++++++++++++++++ main code below ++++++++++++++++++++++++++++++++ //
Sugeno mySeg ;
void mainCode()
{
	mySeg = new Sugeno(2,1);
	FuzzySet *fs;
	FSList = new CFLS(); // global
	//normal Sets
	ss=FSList->add(veryClose); //0
	ss=FSList->add(close); //1
	ss=FSList->add(far); //2
	ss=FSList->add(dangAngleLeft); //3
	ss=FSList->add(dangAngleRight); //4
	ss=FSList->add(closeAngleLeft); //5
	ss=FSList->add(closeAngleRight); //6
	ss=FSList->add(farAngleLeft); //7
	ss=FSList->add(farAngleRight); //8

	ss=FSList->add(slow); //9
	ss=FSList->add(medSpeed); //10
	ss=FSList->add(fast); //11

	ss=FSList->add(sharpLeft); //12
	ss=FSList->add(turnleft); //13
	ss=FSList->add(slowTurnLeft);//14
	ss=FSList->add(slowTurnRight);//15
	ss=FSList->add(turnright);//16
	ss=FSList->add(sharpRight);//17

	ss=FSList->add(closeToWall);//18
	ss=FSList->add(closishToWall);//19
	ss=FSList->add(wallGettingCloser);//20

	//ShortCut fuzzy set initilaization 
	//fuzzy sets for distance from obj
	int L=0; int H=70; //low and high values
	//very close
	fs =new FuzzySet("SCveryClose",L,H); 
	SCveryClose->fuzzySet(fs);
	ss=FSList->add(fs); // 21

	//close	
	fs =new FuzzySet("SCclose",L,H); 
	SCclose->fuzzySet(fs);
	ss=FSList->add(fs); // 22

	//far
	fs =new FuzzySet("SCfar",L,H); 
	SCfar->fuzzySet(fs);
	ss=FSList->add(fs); // 23

	//fuzzy sets for angle from obj
	L=-90;H=90; //low and high values

	//dangerous angle Left
	fs =new FuzzySet("SCdangAngleLeft",L,H); 
	SCdangAngleLeft->fuzzySet(fs);
	ss=FSList->add(fs); // 24

	//dangerous angle Right
	fs =new FuzzySet("SCdangAngleRight",L,H); 
	SCdangAngleRight->fuzzySet(fs);
	ss=FSList->add(fs); // 25

	//close angle Left
	fs =new FuzzySet("SCcloseAngleLeft",L,H); 
	SCcloseAngleLeft->fuzzySet(fs);
	ss=FSList->add(fs); // 26

	//close angle Right
	fs =new FuzzySet("SCcloseAngleRight",L,H); 
	SCcloseAngleRight->fuzzySet(fs);
	ss=FSList->add(fs); // 27

	//far angle Left
	fs =new FuzzySet("SCfarAngleLeft",L,H); 
	SCfarAngleLeft->fuzzySet(fs);
	ss=FSList->add(fs); // 28

	//far angle Right
	fs =new FuzzySet("SCfarAngleRight",L,H); 
	SCfarAngleRight->fuzzySet(fs);
	ss=FSList->add(fs); // 29

	//fuzzy sets for speed
	L=0;H=5; //low and high values
	//slow
	fs =new FuzzySet("SCslow",L,H); 
	SCslow->fuzzySet(fs);
	ss=FSList->add(fs); // 30

	//med
	fs =new FuzzySet("SCmedSpeed",L,H); 
	SCmedSpeed->fuzzySet(fs);
	ss=FSList->add(fs); // 31

	//fast
	fs =new FuzzySet("SCfast",L,H); 
	SCfast->fuzzySet(fs);
	ss=FSList->add(fs); // 32

	//fuzzy sets for direction
	L=-7;H=7; //low and high values
	//sharp left
	fs =new FuzzySet("SCsharpLeft",L,H); 
	SCsharpLeft->fuzzySet(fs);
	ss=FSList->add(fs); // 33

	//left 
	fs =new FuzzySet("SCturnleft",L,H); 
	SCturnleft->fuzzySet(fs);
	ss=FSList->add(fs); // 34

	//slowTurnLeft 	
	fs =new FuzzySet("SCslowTurnLeft",L,H); 
	SCslowTurnLeft->fuzzySet(fs);
	ss=FSList->add(fs); // 35

	//slowTurnRight  
	fs =new FuzzySet("SCslowTurnRight",L,H); 
	SCslowTurnRight->fuzzySet(fs);
	ss=FSList->add(fs); // 36

	//right 
	fs =new FuzzySet("SCturnright",L,H); 
	SCturnright->fuzzySet(fs);
	ss=FSList->add(fs); // 37

	//shrp Right
	fs =new FuzzySet("SCsharpRight",L,H); 
	SCsharpRight->fuzzySet(fs);
	ss=FSList->add(fs); // 38

	//distance from wall sets
	L=0;H=600;
	//closeToWall 
	fs =new FuzzySet("SCcloseToWall",L,H); 
	SCcloseToWall->fuzzySet(fs);
	ss=FSList->add(fs); // 39

	//closeishToWall  
	fs =new FuzzySet("SCclosishToWall",L,H); 
	SCclosishToWall->fuzzySet(fs);
	ss=FSList->add(fs); // 40

	//wallGettingCloser
	fs =new FuzzySet("SCwallGettingCloser",-H,H); 
	SCwallGettingCloser->fuzzySet(fs);
	ss=FSList->add(fs); // 41
}


void mainRender()
{

	if(!showInitSets) return;
	UcDrawCross(UcPoint(10,10), UcColour(UcColour::Pink), 3);
	FsFrame f1(10, 590, 250, 100);
	FsFrame f2(10, 470, 250, 100);
	FsFrame f4(10, 350, 250, 100);
	FsFrame f5(10, 230, 250, 100);	
	FsFrame f6(10, 110, 250, 100);		
	
	f1.setTitle(90, -12, "Distance Sets");
	f1.graphFS((FuzzySet*)FSList->get(0), UcColour(UcColour::DarkGrey));
	f1.graphFS((FuzzySet*)FSList->get(1), UcColour(UcColour::Red));
	f1.graphFS((FuzzySet*)FSList->get(2), UcColour(UcColour::Green));
	
	
	f2.setTitle(120, -12, "Angle sets");
	f2.graphFS((FuzzySet*)FSList->get(3), UcColour(UcColour::DarkGrey));
	f2.graphFS((FuzzySet*)FSList->get(4), UcColour(UcColour::Red));
	f2.graphFS((FuzzySet*)FSList->get(5), UcColour(UcColour::Green));
	f2.graphFS((FuzzySet*)FSList->get(6), UcColour(UcColour::Blue));
	f2.graphFS((FuzzySet*)FSList->get(7), UcColour(UcColour::Pink));
	f2.graphFS((FuzzySet*)FSList->get(8), UcColour(UcColour::LightGreen));

	
	
	f4.setTitle(90, -12,  "Speed Sets");
	f4.graphFS((FuzzySet*)FSList->get(9), UcColour(UcColour::LightBlue));
	f4.graphFS((FuzzySet*)FSList->get(10), UcColour(UcColour::DarkGrey));
	f4.graphFS((FuzzySet*)FSList->get(11), UcColour(UcColour::Red));

	f5.setTitle(90, -12,  "Direction Sets");
	f5.graphFS((FuzzySet*)FSList->get(12), UcColour(UcColour::Green));
	f5.graphFS((FuzzySet*)FSList->get(13), UcColour(UcColour::Blue));
	f5.graphFS((FuzzySet*)FSList->get(14), UcColour(UcColour::Magenta));
	f5.graphFS((FuzzySet*)FSList->get(15), UcColour(UcColour::Red));
	f5.graphFS((FuzzySet*)FSList->get(16), UcColour(UcColour::DarkGrey));
	f5.graphFS((FuzzySet*)FSList->get(17), UcColour(UcColour::Green));
	
	f6.setTitle(90, -12,  "Distance From Walls");
	f6.graphFS((FuzzySet*)FSList->get(18), UcColour(UcColour::Green));
	f6.graphFS((FuzzySet*)FSList->get(19), UcColour(UcColour::Blue));
	f6.graphFS((FuzzySet*)FSList->get(20), UcColour(UcColour::Magenta));

	//Short Cut Sets
	
	UcDrawCross(UcPoint(10,10), UcColour(UcColour::Pink), 3);
	FsFrame f7(270, 590, 250, 100);
	FsFrame f8(270, 470, 250, 100);
	FsFrame f9(270, 350, 250, 100);
	FsFrame f10(270, 230, 250, 100);	
	FsFrame f11(270, 110, 250, 100);		
	
	f7.setTitle(90, -12, "Distance Sets");
	f7.graphFS((FuzzySet*)FSList->get(21), UcColour(UcColour::DarkGrey));
	f7.graphFS((FuzzySet*)FSList->get(22), UcColour(UcColour::Red));
	f7.graphFS((FuzzySet*)FSList->get(23), UcColour(UcColour::Green));
	
	
	f8.setTitle(120, -12, "Angle sets");
	f8.graphFS((FuzzySet*)FSList->get(24), UcColour(UcColour::DarkGrey));
	f8.graphFS((FuzzySet*)FSList->get(25), UcColour(UcColour::Red));
	f8.graphFS((FuzzySet*)FSList->get(26), UcColour(UcColour::Green));
	f8.graphFS((FuzzySet*)FSList->get(27), UcColour(UcColour::Blue));
	f8.graphFS((FuzzySet*)FSList->get(28), UcColour(UcColour::Pink));
	f8.graphFS((FuzzySet*)FSList->get(29), UcColour(UcColour::LightGreen));

	
	/* swiched to Sugeno
	f9.setTitle(90, -12,  "Speed Sets");
	f9.graphFS((FuzzySet*)FSList->get(30), UcColour(UcColour::LightBlue));
	f9.graphFS((FuzzySet*)FSList->get(31), UcColour(UcColour::DarkGrey));
	f9.graphFS((FuzzySet*)FSList->get(32), UcColour(UcColour::Red));
	*/
	f9.setTitle(90, -12,  "Speed Sets");
	f9.graphFS(sugSCslow, UcColour(UcColour::Red),1,"sugSCslow");
	f9.graphFS(sugSCmedSpeed, UcColour(UcColour::LightBlue),1,"sugSCmedSpeed");
	f9.graphFS(sugSCfast, UcColour(UcColour::DarkGrey),1,"sugSCfast");

	/* swiched to Sugeno
	f10.setTitle(90, -12,  "Direction Sets");
	f10.graphFS((FuzzySet*)FSList->get(33), UcColour(UcColour::Green));
	f10.graphFS((FuzzySet*)FSList->get(34), UcColour(UcColour::Blue));
	f10.graphFS((FuzzySet*)FSList->get(35), UcColour(UcColour::Magenta));
	f10.graphFS((FuzzySet*)FSList->get(36), UcColour(UcColour::Red));
	f10.graphFS((FuzzySet*)FSList->get(37), UcColour(UcColour::DarkGrey));
	f10.graphFS((FuzzySet*)FSList->get(38), UcColour(UcColour::Green));
	*/
	f10.setTitle(90, -12,  "Direction Sets");
	f10.graphFS(sugSCsharpLeft, UcColour(UcColour::Black),2,"sugSCsharpLeft");
	f10.graphFS(sugSCsharpRight, UcColour(UcColour::Blue),2,"sugSCsharpRight");
	f10.graphFS(sugSCturnleft, UcColour(UcColour::Magenta),2,"sugSCturnleft");
	f10.graphFS(sugSCturnright, UcColour(UcColour::Red),2,"sugSCturnright");
	f10.graphFS(sugSCslowTurnLeft, UcColour(UcColour::DarkGrey),2,"sugSCslowTurnLeft");
	f10.graphFS(sugSCslowTurnRight, UcColour(UcColour::Green),2,"sugSCslowTurnRight");
	f10.graphFS(sugSCnoMove, UcColour(UcColour::LightBlue),2,"sugSCnoMove");

	f11.setTitle(90, -12,  "Distance From Walls");
	f11.graphFS((FuzzySet*)FSList->get(39), UcColour(UcColour::Green));
	f11.graphFS((FuzzySet*)FSList->get(40), UcColour(UcColour::Blue));
	f11.graphFS((FuzzySet*)FSList->get(41), UcColour(UcColour::Magenta));
	
}


void mainTimer(int value)
{
}
void mainIdle()
{
}

void key_generic(unsigned char key)
{
}


// end