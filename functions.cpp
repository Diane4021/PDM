#include "functions.h"


// rdm class, for gentaring random flot numbers
rdm::rdm() {i=time(0);}
float rdm::randomize() { i=i+1;  srand (i);  return float(rand())/float(RAND_MAX);}



//Norm function 
float Norm(std::vector<float> x1,std::vector<float> x2)
{
return pow(	(pow((x2[0]-x1[0]),2)+pow((x2[1]-x1[1]),2))	,0.5);//norm 2 to compute the distance of 2 points
}


//sign function
float sign(float n)
{
if (n<0.0){return -1.0;}
else{return 1.0;}
}


//Nearest function,return the nearst point
std::vector<float> Nearest(  std::vector< std::vector<float>  > V, std::vector<float>  x){

float min=Norm(V[0],x);//the initial point of the point set 
int min_index;
float temp;

for (int j=0;j<V.size();j++)//search all the points
{
temp=Norm(V[j],x);
if (temp<=min){
min=temp;
min_index=j;}//remember the index of nearest point

}

return V[min_index];
}



//Steer function,eta step length
std::vector<float> Steer(  std::vector<float> x_nearest , std::vector<float> x_rand, float eta){
std::vector<float> x_new;//generate a new point

if (Norm(x_nearest,x_rand)<=eta){
x_new=x_rand;
}
else{


float m=(x_rand[1]-x_nearest[1])/(x_rand[0]-x_nearest[0]);

x_new.push_back(  (sign(x_rand[0]-x_nearest[0]))* (   sqrt( (pow(eta,2)) / ((pow(m,2))+1) )   )+x_nearest[0] );
x_new.push_back(  m*(x_new[0]-x_nearest[0])+x_nearest[1] );

if(x_rand[0]==x_nearest[0]){
x_new[0]=x_nearest[0];
x_new[1]=x_nearest[1]+eta;
}



}
return x_new;// the new point
}





//gridValue function known the point position Xp, want to know the grid value of the position
int gridValue(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp){
/*
MapMetaData
time map_load_time
float32 resolution
uint32 width
uint32 height
geometry_msgs/Pose origin*/
float resolution=mapData.info.resolution;
float Xstartx=mapData.info.origin.position.x;
float Xstarty=mapData.info.origin.position.y;//Xstaty is the origin of the map, not the begining point we chose

float width=mapData.info.width;
std::vector<signed char> Data=mapData.data;//the probabilities of each cell

//returns grid value at "Xp" location
//map data:  100 occupied      -1 unknown       0 free 
float indx=(  floor((Xp[1]-Xstarty)/resolution)*width)+( floor((Xp[0]-Xstartx)/resolution) );//floor:Rounds x downward, returning the largest integral value that is not greater than x.
//ATTENTION,Xp[0]-Xstartx)/resolution,no nedd to multiply height?
int out;
out=Data[int(indx)];
return out;
}




// ObstacleFree function-------------------------------------

char ObstacleFree(std::vector<float> xnear, std::vector<float> &xnew, nav_msgs::OccupancyGrid mapsub){
  //if xnear=xnearest, to check whether there are obstacles on the trajectory
float rez=float(mapsub.info.resolution)*.2;//why??choosing a steplength maybe
float stepz=int(ceil(Norm(xnew,xnear))/rez); //why??number of steps
std::vector<float> xi=xnear;//attention
char  obs=0; char unk=0;
 
geometry_msgs::Point p;
for (int c=0;c<stepz;c++){//std::vector<float> Steer(  std::vector<float> x_nearest , std::vector<float> x_rand, float eta)
  xi=Steer(xi,xnew,rez);//at begining, xi=xnear=x_nearest,x_rand=xnew,eat=rez,the steplength
  		

   if (gridValue(mapsub,xi) ==100){     obs=1; }//int gridValue(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp)
   
   if (gridValue(mapsub,xi) ==-1){      unk=1;	break;}//can be deleted,because no unknown
  }
char out=0;
 xnew=xi;
 if (unk==1){  out=-1;}
 	
 if (obs==1){  out=0;}
 		
 if (obs!=1 && unk!=1){   out=1;}

 
 return out;
 

 }
 


     
   


  
 
 
 
 





























