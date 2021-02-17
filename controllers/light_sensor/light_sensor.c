#include <stdio.h>
#include <stdlib.h>

#include <math.h>


#include <webots/light_sensor.h>

#include <webots/distance_sensor.h>

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/supervisor.h>


#define MAX_SPEED 10
#define SPEED 6
#define TIME_STEP 64

#define B 0.18//09

#define LENGTH 2.2
#define HEIGHT 1.42

typedef struct
{
  double left,right;
} DifSpeed;

typedef struct
{
  double vel,rot;
} Speed;

typedef struct
{
    double x;
    double y;
    double z;


} Point3d;

double distance (Point3d a, Point3d b)
{
  return sqrt ( powl(a.x-b.x,2)+powl(a.y-b.y,2)+powl(a.z-b.z,2));
}

double distance2D (Point3d a, Point3d b)
{
  return sqrt ( powl(a.x-b.x,2)+powl(a.y-b.y,2));
}

double length (Point3d p)
{
  return sqrt (p.x*p.x+p.y*p.y+p.z*p.z);
}


double wrapToPi(double a)
{
  while (a>M_PI)
     a-=M_PI*2;
  while (a<=-M_PI)
     a+=M_PI*2;
  return a;
}

double wrapTo2Pi(double a)
{
  while (a>2*M_PI)
     a-=M_PI*2;
  while (a<=0)
     a+=M_PI*2;
  return a;
}


double min (double a, double b)
{
    return a<b?a:b;
}
DifSpeed normalToDifferential (Speed s, bool normalize)
{
    DifSpeed r;
    r.left = s.vel - s.rot*B/2.0;
    r.right = s.vel + s.rot*B/2.0;

    const double MAXSPEED =0.3;

    double max = r.left;

    if (fabs(r.right)> fabs(r.left))
        max = r.right;

    if (/*max!=0 &&*/ normalize)
    {
        double factor = MAXSPEED/fabs(max);

       // printf ("before: %f %f\n",r.left,r.right);

        r.left*=factor;
        r.right*=factor;

    //    printf ("after: %f %f\n",r.left,r.right);

    }


    return r;
}

Speed moveToGoal (Point3d goal)
{
    double fg =  atan2(-goal.y,goal.x) ;//- M_PI/2;
    Speed s;
    s.rot = 3*fg;
    s.vel = 1;
    return s;
}

Speed followCurvature (WbDeviceTag sonar[], int size, bool clockwise)
{
    Speed result;
    int min = 0;
    double minRange = wb_distance_sensor_get_value(sonar[0]);

    for (int i=0;i<size;i++)
    {
      double range = wb_distance_sensor_get_value(sonar[i]);
 //     printf ("Obstacle %d in %f\n",i,range);
      if (range< minRange)
      {
          minRange = range;
          min=i;
      }
    }

 //   printf ("%d\n",min);




    if (wb_distance_sensor_get_value(sonar[min]) >= wb_distance_sensor_get_max_value(sonar[min]))
       printf("Blind\n");

    double radius = 0.2;
//    printf ("Minimum obstacle at %fmm at %d degrees\n",minRange,min*45);
    double v =radius+minRange;
    double Px = v*cos(min* M_PI/4.0);
    double Pz = v*sin(min* M_PI/4.0);

    double phLin;

    int next =(min+1)%size;
    int prev = (size+min-1)%size;
 //   printf ("next is %d = %f\n",next, wb_distance_sensor_get_value(sonar[next]));
  //  printf ("prev is %d = %f\n",prev, wb_distance_sensor_get_value(sonar[prev]));
 //   printf("NEXT\n");
    if (wb_distance_sensor_get_value(sonar[next]) < wb_distance_sensor_get_max_value(sonar[next]))
    {
        double v2=radius+wb_distance_sensor_get_value(sonar[next]);
        double Px2 = v2*cos(next* M_PI/4.0);
        double Pz2 = v2*sin(next* M_PI/4.0);

        if (clockwise)
            phLin = -atan2(Pz-Pz2,Px2-Px);
        else
            phLin = -atan2(Pz2-Pz,Px-Px2);

   /*     if (phLin>M_PI/2)
            phLin-=M_PI;
        if (phLin<-M_PI/2)
            phLin+=M_PI;
     */

  //      printf ("Two with next\n");

    //    printf ("points: (%f,%f) and (%f,%f)\n",Px,Pz,Px2,Pz2);
   // printf ("phlin is %f (%fC)\n",phLin,phLin*180/M_PI);


    }
    else if (wb_distance_sensor_get_value(sonar[prev]) < wb_distance_sensor_get_max_value(sonar[prev]))
    {
        double v2=radius+wb_distance_sensor_get_value(sonar[prev]);
        double Px2 = v2*cos(prev* M_PI/4.0);
        double Pz2 = v2*sin(prev* M_PI/4.0);

        if (!clockwise)
            phLin = -atan2(Pz-Pz2,Px2-Px);
        else
            phLin = -atan2(Pz2-Pz,Px-Px2);

      //  printf ("Two with prev\n");
   //     printf ("points: (%f,%f) and (%f,%f)\n",Px,Pz,Px2,Pz2);
   // printf ("phlin is %f (%fC)\n",phLin,phLin*180/M_PI);

    }
    else

    {
        double vx,vz;
        if (clockwise)
        {
          vx = -Pz;
          vz = Px;
        }
        else
        {
          vx = Pz;
          vz = -Px;
        }
        phLin = 3*atan2(vz,vx);
        //printf ("Single one\n");
  //  printf ("phlin is %f (%fC)\n",phLin,phLin*180/M_PI);

    }

    //printf ("phlin is %f (%fC)\n",phLin,phLin*180/M_PI);


  //  printf("distance is %f should be 0.4\n",v);

    double phRot = atan(2.0*(v-0.6));


    if (min==0 || min==4)
      phRot=0;

    if (min>4 && min<8)
      phRot = -phRot;



    if (clockwise)
      phRot=-phRot;

   // printf ("Correction is %f\n",phRot);
    double phRef = wrapToPi(phLin+phRot) / M_PI;

/*    while (phRef>M_PI/2)
        phRef-=M_PI;
    while (phRef<-M_PI/2)
        phRef+=M_PI;
  */



   // printf ("v = %f , phrot = %f\n",v,phRot);

   // printf ("phref is %f (%fC)\n",phRef,phRef*180/M_PI);


    result.rot = 10*phRef;
    result.vel=1*cos(phRef*M_PI/2);

 //   printf ("rot = %f vel = %f\n",result.rot,result.vel);

    return result;
}


bool solve (double a, double b, double c,double *root1, double *root2)
{
    float d;
    d = b * b - 4 * a * c;

    if(d < 0)
        return false;
    else if(d==0)
    {
        *root1 = -b /(2* a);
        *root2 = -b /(2* a);
        return true;
    }
    else
    {
        *root1 = ( -b + sqrt(d)) / (2* a);
        *root2 = ( -b - sqrt(d)) / (2* a);
        return true;
    }
}

Point3d SimpleFindLight (double El23,double Er23,double Ef23,double E023, double r)
{
    double K = (1.0/Er23 + 1.0/El23 - 2.0/E023);
    double b = (2*r*r)/K;

    Point3d result;
    result.x = (r*r - b*(1/Ef23-1/E023))/(2*r);
    result.y =- ( b*(1/Er23-1/El23))/(4*r);
    result.z = sqrt(b/E023 - result.x*result.x - result.y*result.y);

    return result;

}

bool FindSource (WbDeviceTag lsL,WbDeviceTag lsR, WbDeviceTag lsF,WbDeviceTag lsC,WbDeviceTag lsB, double r,Point3d *res)
{
    double EC = powl(wb_light_sensor_get_value(lsC),2.0/3);
    double EF = powl(wb_light_sensor_get_value(lsF),2.0/3);
    double ER = powl(wb_light_sensor_get_value(lsR),2.0/3);
    double EL = powl(wb_light_sensor_get_value(lsL),2.0/3);
    double EB = powl(wb_light_sensor_get_value(lsB),2.0/3);




    Point3d p[4];

    p[0] = SimpleFindLight(EL,ER,EF,EC,r);

    p[1] = SimpleFindLight(EF,EB,ER,EC,r);

    double t = p[1].x;
    p[1].x = -p[1].y;
    p[1].y = t;

    p[2]= SimpleFindLight(ER,EL,EB,EC,r);
    p[2].x = -p[2].x;
    p[2].y=-p[2].y;


    p[3]= SimpleFindLight(EB,EF,EL,EC,r);
    t = p[3].x;
    p[3].x = p[3].y;
    p[3].y = -t;


    int i;

 //   for (i=0;i<4;i++)
   //     printf ("%d: %f,%f,%f\n",i,p[i].x,p[i].y,p[i].z);


    res->x=0;
    res->y=0;
    res->z=0;
    int tx=0, ty=0,tz=0;

    for (i=0;i<4;i++)
    {
        if (!isnan(p[i].x))
        {
            res->x += p[i].x;
            tx++;
        }
        if (!isnan(p[i].y))
        {
            res->y += p[i].y;
            ty++;
        }
        if (!isnan(p[i].z))
        {
            res->z += p[i].z;
            tz++;
        }

    }

    if (tx>0)
        res->x /=tx;

    if (ty>0)
        res->y /=ty;

    if (tz>0)
        res->z /=tz;

  //  printf ("%f (%d), %f(%d), %f (%d)\n",res.x,tx,res.y,ty,res.z,tz);


  //  if (!tx && !ty && !tz)
     if (!tx || !ty)// || !tz)

         return false;
  //  {
   // printf ("000\n");
    //    return res;
   // }

    return true;
}



Point3d findLight(WbDeviceTag lsL,WbDeviceTag lsR, WbDeviceTag lsF,WbDeviceTag lsC,WbDeviceTag lsB, double r)
{
    double E023 = powl(wb_light_sensor_get_value(lsC),2.0/3);
    double Ef23 = powl(wb_light_sensor_get_value(lsF),2.0/3);
    double Er23 = powl(wb_light_sensor_get_value(lsR),2.0/3);
    double El23 = powl(wb_light_sensor_get_value(lsL),2.0/3);
    double Eb23 = powl(wb_light_sensor_get_value(lsB),2.0/3);


   /*
    double r1,r2;

    double Fa,Fb,Fc;

    Fa = Ef23 + Eb23 -El23 - Er23;

    Fb =  2*Ef23*Eb23 -2*El23*Er23;//(2*Ef23*Eb23 - 2*El23*Er23)*(El23+Er23)*(Ef23+Eb23);      //2*El23*Er23-2*Ef23*Eb23;
    Fc =  (El23+Er23)*(Ef23*Eb23)-(Ef23+Eb23)*(El23*Er23);     //El23*Ef23*Eb23 + Er23*Ef23*Eb23 - Ef23*El23*Er23 - Eb23*El23*Er23;

    printf ("Solving %f*x^2 + %f*x + %f\n",Fa,Fb,Fc);

    if (solve(Fa,Fb,Fc,&r1,&r2))
    {
        double correction = r1;

        if (fabs(r2)<fabs(r1))
          correction = r2;

        double dif = 1/El23 + 1/Er23 -1/Ef23 - 1/Eb23;

        printf ("Before change %f\n",dif);

        El23+=correction;
        Er23+=correction;
        Ef23+=correction;
        Eb23+=correction;
        dif = 1/El23 + 1/Er23 -1/Ef23 - 1/Eb23;

        printf ("After change %f\n",dif);



        printf ("%f or %f\n",r1,r2);
    }

    */



    double K = (1.0/Er23 + 1.0/El23 - 2.0/E023);
    double mb23 = (2*r*r)/K;

    double KV = (1.0/Ef23 + 1.0/Eb23 - 2.0/E023);
    double mb23V = (2*r*r)/KV;

  //  printf ("mb23 is %f\n",mb23);
  //  printf ("mb23V is %f\n",mb23V);



    double b = mb23;//*/ (mb23+mb23V)/2.0;

    Point3d result;

  //  printf ("K is %f\n",K);
 //   printf ("1 DIA = %f\n",(1/Ef23-1/E023));

  //  result.x = (r*r - mb23*(1/Ef23-1/E023))/(2*r);

    //printf ("old x= %f\n",result.x);
/*
    result.x = (mb23*(1/Eb23-1/Ef23))/(4*r);
//    printf ("new x= %f\n",result.x);

    result.x = (mb23V*(1/Eb23-1/Ef23))/(4*r);
//    printf ("newV x= %f\n",result.x);
  */
    result.x = ( b*(1/Eb23-1/Ef23))/(4*r);
//    printf ("new avg x= %f\n",result.x);



/*
    result.y = (r*r - mb23*(1/El23-1/E023))/(2*r);

//    printf ("old y= %f\n",result.y);

    result.y = (mb23*(1/Er23-1/El23))/(4*r);
  //  printf ("new y= %f\n",result.y);

    result.y = (mb23V*(1/Er23-1/El23))/(4*r);
  //  printf ("newV y= %f\n",result.y);
*/
    result.y =- ( b*(1/Er23-1/El23))/(4*r);
  //  printf ("new avg y= %f\n",result.y);



 //   result.y = (mb23*(1/Ef23 - 1/Er23)+2*r*result.x)/(2*r);
    result.z = sqrt(mb23/E023 - result.x*result.x - result.y*result.y);

  //  printf("E0 = %f Ef = %f Er = %f El=%f\n",E023,Ef23,Er23,El23);
 /*
    if (isnan(result.x))
    {
      printf ("Nan x\n");
      printf ("K is %f\n",K);
      printf ("mb23 is %f\n",mb23);

    }
    if (isnan(result.y))
    {
      printf ("Nan y\n");
      printf ("K is %f\n",K);
      printf ("mb23 is %f\n",mb23);
    }
    if (isnan(result.z))
    {
      printf ("Nan z\n");
      printf ("K is %f\n",K);
      printf ("mb23 is %f\n",mb23);
    }
    */
   //   printf ("inside sqrt is %f\n",(mb23/E023 - result.x*result.x - result.y*result.y));

    return result;
}

Speed Bug0(WbDeviceTag sonar[], int size, WbDeviceTag lsL,WbDeviceTag lsR, WbDeviceTag lsF,WbDeviceTag lsC, double r)
{
   Speed result;
   const double ZERO = 0.1;
   Point3d lg;//findLight(lsL,lsR,lsF,lsC,r);
   double f2g = atan2(-lg.y,lg.x);
   double minDist=100;
   double dist = length(lg);

   //printf("Min dist is %f\n",dist);

   if (dist<ZERO)
   {
       result.rot=0;
       result.vel=0;
   }
   else
   {
       for (int i=0;i<size;i++)
       {
          double ph = wrapToPi(i * M_PI/4.0);
          if (abs(ph-f2g)<=M_PI/2)
              minDist= min(minDist,wb_distance_sensor_get_value(sonar[i]));
       }
    //   printf("Min dist is %f\n",minDist);
       if (minDist>0.3)    // MOVE TO GOAL
          result= moveToGoal(findLight(lsL,lsR,lsF,lsC, lsC,r)); //ERRRORROROROOROROROOR
       else
          result = followCurvature(sonar,8,true);
   }
   return result;
}


Speed pBug(bool enhanced, WbDeviceTag sonar[], int size, WbDeviceTag lsL,WbDeviceTag lsR, WbDeviceTag lsF,WbDeviceTag lsC, WbDeviceTag lsB, double r)
{
   static Point3d lastG;
   static bool first=true;



   static double initial=1000;
   Speed result;
   const double ZERO = 0.1;
 //  Point3d lg = findLight(lsL,lsR,lsF,lsC,lsB,r);

  // Point3d lg = SimpleFindLight(lsL,lsR,lsF,lsC,/*lsB,*/r);

   Point3d lg;
    if (!FindSource(lsL,lsR,lsF,lsC,lsB,r,&lg))
    {
        result.rot=0/0;
        result.vel =0/0;
    //    printf ("should\n");
        return result;
    }


    printf ("goal at %f,%f,%f\n",lg.x,lg.y,lg.z);

    double ang = atan2(-lg.x,lg.y);
    printf ("Angle is %f\n",ang*180/3.14);


  //  printf ("Original: %f,%f,%f\n",lg.x,lg.y,lg.z);
    if (!first && enhanced)
    {
        lg.x = (lg.x+9*lastG.x)/10.0;
        lg.y = (lg.y+9*lastG.y)/10.0;
        lg.z = (lg.z+9*lastG.z)/10.0;
    }
 //   printf ("Updated: %f,%f,%f\n",lg.x,lg.y,lg.z);

   first = false;


   double kt = atan2(0,0);
  //printf ("0 is %f\n",kt);


   double f2g = atan2(-lg.y,lg.x);
   double minDist=100;
   double dist = length(lg);
   static bool state = false; //false is move to goal

  // printf ("Goal is at %f,%f\n",lg.x,lg.y);

//   result.rot=0;
//   result.vel = 5;
//   return result;


  // printf("state is %d Min dist is %f\n",state,dist);

   if (dist<ZERO)
   {
       result.rot=0;
       result.vel=0;
   }
   else
   {
       for (int i=0;i<size;i++)
       {
     //      printf ("sensor %d gives %f\n",i,wb_distance_sensor_get_value(sonar[i]));
          double ph = wrapToPi(i * M_PI/4.0);
          if (abs(ph-f2g)<=M_PI/6)
          {
              double read = wb_distance_sensor_get_value(sonar[i]);
              if (!isnan(read) && read<minDist)
                  minDist= read;
          }
       }
   //   printf("Min dist is %f dist is %f init %f\n",minDist,dist,initial);


       if ((minDist>0.6) && (!state || dist<initial))    // MOVE TO GOAL
       {
       printf("moving to goal with distance  %f<%f\n",dist,initial);
   //    if (state==true)
  //       printf("Change to false\n");
          state = false;
          result= moveToGoal(lg);


     //     printf ("rot:%f\n",result.rot);
       }
       else
       {
       if (state==false)
       {
       printf ("curvature");
         if (!isnan(dist))
             initial = dist;
  //       printf("Change to true initial is %f\n",initial);
       }
          state = true;
   //       printf ("rotati\n");
          result = followCurvature(sonar,8,false);
       }
   }

   return result;
}

double rotSpeed (WbDeviceTag lsL,WbDeviceTag lsR,WbDeviceTag lsF)
{
    double R = wb_light_sensor_get_value(lsR);
    double L = wb_light_sensor_get_value(lsL);
    double F = wb_light_sensor_get_value(lsF);


    //printf ("L=%f, R=%f, F=%f\n",L,R,F);

    double r = 50*(L-R);

//    if  (F<L && F<R)
    {
//        if (L<R)
//          r=(R-L)*5;
  //      else
    //      r=5;
    }
  //  else
    {

    }

    if (r>5)
      r=5;
    if (r<-5)
      r=-5;


    return r;
}
bool faceTower2(WbDeviceTag ir[])
{
    int maxPos = 0;
    double maxVal=0;
    for (int k=0;k<10;k++)
    {
      maxVal += wb_light_sensor_get_value(ir[0]);
    //  printf ("%d: %f\n",k,wb_light_sensor_get_value(ir[0]));
    }
    for (int i=1;i<12;i++)
    {
        double cur=0;

        for (int k=0;k<10;k++)
          cur+= wb_light_sensor_get_value(ir[i]);

        if (cur>maxVal)
        {
            maxVal = cur;
            maxPos = i;
        }

    }

  //  printf("Light at %i\n",maxPos);
    return maxPos==9;

}

double getI (WbDeviceTag lsC)
{
    return wb_light_sensor_get_value(lsC);
}

bool atTower (WbDeviceTag lsL,WbDeviceTag lsR, WbDeviceTag lsF,WbDeviceTag lsC, double r)
{
   const double ZERO = 0.1;
   Point3d lg ;//= findLight(lsL,lsR,lsF,lsC,r);
   double dist = length(lg);
   return dist<=ZERO;
}

bool atObstacle (WbDeviceTag sonar[], int size)
{
    double minDist=100;
    for (int i=0;i<size;i++)
        minDist= min(minDist,wb_distance_sensor_get_value(sonar[i]));

    minDist = wb_distance_sensor_get_value(sonar[7]);
    minDist= min(minDist,wb_distance_sensor_get_value(sonar[0]));
    minDist= min(minDist,wb_distance_sensor_get_value(sonar[1]));


    return minDist<0.2;
}

bool localMax (double ik[], int N)
{
    int group = N/3;
    double pp=0,p=0,c=0;

    int i;
    double min,max;

    min = 1000;
    max =0;
    for (i=0;i<group;i++)
    {
      pp+=ik[i];
      min=ik[i]<min?ik[i]:min;
      max=ik[i]>max?ik[i]:max;
    }
    if (group>3)
    {
        pp=pp-min-max;
        pp/=(group-2);
    }
    else
      pp/=group;

    min=1000;
    max=0;
    for (i=group;i<2*group;i++)
    {
      p+=ik[i];
      min=ik[i]<min?ik[i]:min;
      max=ik[i]>max?ik[i]:max;
    }
    if (group>3)
    {
        p=p-min-max;
        p/=(group-2);
    }
    else
      p/=group;

    min=1000;
    max=0;
    for (i=2*group;i<N;i++)
    {
      c+=ik[i];
      min=ik[i]<min?ik[i]:min;
      max=ik[i]>max?ik[i]:max;
    }
    if (group>3)
    {
        c=c-min-max;
        c/=(group-2);
    }
    else
      c/=group;

    if ((p - pp > 0.001) && (p - c > 0.001))
    {
        return true;
    }
    return false;

}

bool localMax2 (double ik, double ik1, double ik2)
{
    if (ik1-ik2>0.0002 && ik1-ik>0.0002)
 //    if (ik1>=ik2 && ik1>=ik)
      {
     //   printf ("Current: %f, Previous: %f, Last: %f\n",ik,ik1,ik2);
     //   printf("Local max\n");
        return true;
    }
    return false;//ik1-ik2>0.1 && ik1-ik>0.1;
}

bool Reached (Point3d r, Point3d g)
{
 // printf ("distance: %f\n",distance2D(r,g));
  return (distance2D(r,g)<0.4);


}
Speed iBug(WbDeviceTag sonar[], int size, WbDeviceTag ir[], /*WbDeviceTag lsL,WbDeviceTag lsR, WbDeviceTag lsF,*/WbDeviceTag lsC, double r)
{
   static int leave = 0;
   Speed result;
   const double ZERO = 0.1;

   double minDist=100;
 //  double dist = length(lg);
   static bool state = false; //false is move to goal
//  static bool rotating = false;
  // printf("state is %d Min dist is %f\n",state,dist);


   static double ik[30] = {0};
   static int ikSize = sizeof(ik)/sizeof(ik[0]);

//   static double ik=0, ik2=0, ik1=0;

   static double iH=0,iL=1000;


   for (int i=0;i<ikSize-1;i++)
   {
       ik[i] = ik[i+1];
   }
   ik[ikSize-1]= getI(lsC);

//   ik2 = ik1;
//   ik1 = ik;
 //  ik = getI(lsC);
  bool lmax = localMax(ik,ikSize);
 // if (lmax)
  //   printf ("ik = %f iH is %f\n",ik,iH);
   if  (state && /*atObstacle(sonar,size) &&*/lmax  && ik[ikSize-1]>iH)
   {
       iL = ik[ikSize-1];
       state = false;
    //   printf("Leave point\n");
    //   for (int i=0;i<ikSize;i++)
      //     printf ("%f ",ik[i]);
      // printf ("\n");
      // printf ("Leave is %d\n",++leave);
   }
   else   if  (atObstacle(sonar,size) && !state)
   {
       state = true;
       iH = ik[ikSize-1];
    //   printf("Hit point at %f\n",ik);
//       ik=0;
//       ik1=0;
//       ik2=0;
   }
/*
   if (atTower(lsL,lsR, lsF,lsC,r))
   {
     printf ("Stop\n");
       result.rot=0;
       result.vel=0;
   }
   else*/ if (state)
   {
     // printf ("Circular\n");
      result = followCurvature(sonar,8,false);
   }
   else if (!faceTower2(ir))
   {

//       result.rot = rotSpeed(lsL,lsR,lsF);
   //    printf ("Rotating\n");
       result.rot = 0.5;
       result.vel = 0;
   }
   else
   {
  // printf("Approaching\n");
         result.rot=0;
         result.vel = 1;
   }


/*
   if (atTower(lsL,lsR, lsF,lsC,r))
   {
     printf ("Stop\n");
       result.rot=0;
       result.vel=0;
   }
   else if  (atObstacle(sonar,size) && (!localMax(ik,ik1,ik2) || ik<=iH))
   {
       printf ("Circular\n");
       result = followCurvature(sonar,8,false);
   }
   else if (!faceTower(lsL,lsR,lsF))
   {
       printf ("Rotating\n");
       result.rot = 2;
       result.vel = 0;
   }
   else
   {
   printf("Approaching\n");
         result.rot=0;
         result.vel = 5;
   }
  */

   return result;
}



int main() {
  WbDeviceTag  ir[12], or1, or2,lsL, lsR,lsF,lsB, lsC, left_motor, right_motor, sonar[8];

  wb_robot_init();

  WbNodeRef robot_node = wb_supervisor_node_get_from_def("ROBOT");
  WbFieldRef trans_field1 = wb_supervisor_node_get_field(robot_node, "translation");

  WbNodeRef light_node = wb_supervisor_node_get_from_def("LAMP");
  WbFieldRef trans_field2 = wb_supervisor_node_get_field(light_node, "translation");


  printf("Begin\n");

  /* get a handler to the distance sensors. */
  lsL = wb_robot_get_device("lsL");
  lsR = wb_robot_get_device("lsR");
  lsC = wb_robot_get_device("lsC");
  lsF = wb_robot_get_device("lsF");
  lsB = wb_robot_get_device("lsB");
//  or1 = wb_robot_get_device("lsOr1");
//  or2 = wb_robot_get_device("lsOr2");


  ir[0] = wb_robot_get_device("Ori0");
  ir[1] = wb_robot_get_device("Ori1");
  ir[2] = wb_robot_get_device("Ori2");
  ir[3] = wb_robot_get_device("Ori3");
  ir[4] = wb_robot_get_device("Ori4");
  ir[5] = wb_robot_get_device("Ori5");
  ir[6] = wb_robot_get_device("Ori6");
  ir[7] = wb_robot_get_device("Ori7");
  ir[8] = wb_robot_get_device("Ori8");
  ir[9] = wb_robot_get_device("Ori9");
  ir[10] = wb_robot_get_device("Ori10");
  ir[11] = wb_robot_get_device("Ori11");



  sonar[0] = wb_robot_get_device("N");
  sonar[1] = wb_robot_get_device("NW");
  sonar[2] = wb_robot_get_device("W");
  sonar[3] = wb_robot_get_device("SW");
  sonar[4] = wb_robot_get_device("S");
  sonar[5] = wb_robot_get_device("SE");
  sonar[6] = wb_robot_get_device("E");
  sonar[7] = wb_robot_get_device("NE");

  int SAMPLING_PERIOD = 64;


  for (int i=0;i<8;i++)
    wb_distance_sensor_enable (sonar[i], TIME_STEP);

  wb_light_sensor_enable(lsL, SAMPLING_PERIOD);
  wb_light_sensor_enable(lsR, SAMPLING_PERIOD);
  wb_light_sensor_enable(lsC, SAMPLING_PERIOD);
  wb_light_sensor_enable(lsF, SAMPLING_PERIOD);
  wb_light_sensor_enable(lsB, SAMPLING_PERIOD);

  for (int i=0;i<12;i++)
    wb_light_sensor_enable(ir[i], SAMPLING_PERIOD);

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  int counter =0;
int   mycount=0;
  double r = 0.2;

  double total =0;


  FILE *fp;
  fp = fopen("log.txt","w");
  fprintf(fp,"step,x,y,trans,rot,Lodo,Rodo\n");

    const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field1);
    const double *trans2 = wb_supervisor_field_get_sf_vec3f(trans_field2);

    Point3d real;
    real.x = -trans2[2];
    real.y = trans2[0];
    real.z = trans2[1];

    Point3d rob;
    rob.x = -trans[2];
    rob.y = trans[0];
    rob.z = trans[1];


int count=0;

    double leftRad =0;
    double rightRad =0;

    double lastL=0;
    double lastR=0;



  while (wb_robot_step(TIME_STEP) != 1) {
  count++;

    trans = wb_supervisor_field_get_sf_vec3f(trans_field1);
    trans2 = wb_supervisor_field_get_sf_vec3f(trans_field2);

//Point3d calc= findLight(lsL,lsR,lsF,lsC,lsB, r);
/*
    Point3d calc0 = findLight(lsL,lsR,lsF,lsC, r);
    Point3d calc1 = findLight(lsR,lsL,lsB,lsC, r);
    Point3d calc2 = findLight(lsF,lsB,lsR,lsC, r);
    Point3d calc3 = findLight(lsB,lsF,lsL,lsC, r);

    calc1.x=-calc1.x;
    calc1.y=-calc1.y;

    double temp = calc2.x;
    calc2.x = -calc2.y;
    calc2.y=temp;

    temp = -calc3.x;
    calc3.x=calc3.y;
    calc3.y=temp;




    calc.x=0;
    calc.y=0;
    calc.z=0;
    int c=0;

    if (!isnan(calc0.x) && !isnan(calc0.y) && !isnan(calc0.z))
    {
      calc.x+=calc0.x;
      calc.y+=calc0.y;
      calc.z+=calc0.z;
      c++;
    }
    if (!isnan(calc1.x) && !isnan(calc1.y) && !isnan(calc1.z))
    {
      calc.x+=calc1.x;
      calc.y+=calc1.y;
      calc.z+=calc1.z;
      c++;
    }
    if (!isnan(calc2.x) && !isnan(calc2.y) && !isnan(calc2.z))
    {
      calc.x+=calc2.x;
      calc.y+=calc2.y;
      calc.z+=calc2.z;
      c++;
    }
    if (!isnan(calc3.x) && !isnan(calc3.y) && !isnan(calc3.z))
    {
      calc.x+=calc3.x;
      calc.y+=calc3.y;
      calc.z+=calc3.z;
      c++;
    }
  //  if (c!=0)
    {
      calc.x/=c;
      calc.y/=c;
      calc.z/=c;
    }

  */

   // printf("My calc1 light is at position: %f %f %f\n", calc0.x, calc0.y, calc0.z);


   // printf("My calc2 light is at position: %f %f %f\n", calc1.x, calc1.y, calc1.z);
   // printf("My calc3 light is at position: %f %f %f\n", calc2.x, calc2.y, calc2.z);
   // printf("My calc4 light is at position: %f %f %f\n", calc3.x, calc3.y, calc3.z);

 //   printf("My average light is at position: %f %f %f\n", calc.x, calc.y, calc.z);



    //printf("My calc light is at position: %f %f %f\n", calc.x, calc.y, calc.z);



    real;
    real.x = -trans2[2];
    real.y = trans2[0];
    real.z = trans2[1];

    rob;
    rob.x = -trans[2];
    rob.y = trans[0];
    rob.z = trans[1];




   // printf ("Error is %f\n",distance(calc,real));

    //printf ("Error is %f\n",distance(calc,real)/length(real));


   // double calcDist = length(calc);
   // double calcDist0 = length(calc0);
   // double realDist = distance(real,rob);


//    printf("length is %f\n",calcDist);
  //  printf("Real length is %f\n",realDist);

   // printf("Real error0 is %f\n",fabs(calcDist0-realDist)/realDist);
  //  printf("Real error is %f\n",fabs(calcDist-realDist)/realDist);

/*    counter++;
    double f = fabs(calcDist-realDist)/realDist;
    if (isnan(f))
      f=0;
     else
     mycount++;
    total+=f;
*/
/*    Speed s = moveToGoal(findLight(lsL,lsR,lsF,lsC,r));
    DifSpeed r = normalToDifferential (s);
    s = followCurvature(sonar,8,false);
*/
DifSpeed rs;
rs.left=0;
rs.right=0;



bool LADY = true;



Speed s;

    if (!Reached(rob,real))
    {
      if (LADY)
      {
        s = pBug(true,sonar, 8, lsL, lsR, lsF, lsC,lsB, r);
        rs = normalToDifferential (s,false);
      }
      else
      {
        s = iBug(sonar, 8, ir, lsC, r);
        rs = normalToDifferential (s,false);
      }


        if (!isnan(rs.left))
            lastL= rs.left;
        else
        {
    //    printf ("correctd\n");
            rs.left = lastL/2;
            lastL= rs.left;
        }


            leftRad+=fabs(rs.left*TIME_STEP/1000);

        if (!isnan(rs.right))
            lastR=rs.right;
        else
        {
    //    printf ("correctd\n");
            rs.right =lastR/2;
            lastR=rs.right;


        }
            rightRad+=fabs(rs.right*TIME_STEP/1000);
        fprintf(fp,"%d,%f,%f,%f,%f,%f,%f\n",count,rob.y, rob.x,s.vel,s.rot,rs.left*TIME_STEP/1000,rs.right*TIME_STEP/1000);

    }
    else
    {
    printf ("Left: %f rad, Right: %f rad.\n",leftRad,rightRad);
    fclose(fp);
     // printf ("Reach goal\n");
      wb_motor_set_velocity(left_motor, 0);
      wb_motor_set_velocity(right_motor, 0);
     // break;
    }

    if (!isnan(rs.left))
        wb_motor_set_velocity(left_motor, rs.left);
    if (!isnan(rs.right))
        wb_motor_set_velocity(right_motor, rs.right);
   // wb_motor_set_velocity(left_motor, 0);
   // wb_motor_set_velocity(right_motor, 0);
  }

//printf ("count is %d Error is %f\n",mycount,total/mycount);
  return 0;
}
