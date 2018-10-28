#include <iostream>
#include <fstream>
#include <math.h>
#define EXOSPHERE 30000000.0
#define MARS_RADIUS 3386000.0
#define EARTH_RADIUS 6.371e6
#define MARS_MASS 6.42e23
#define MAX_THRUST 1110
#define MAX_FUEL_R 0.5
#define FUEL_CAP 100
#define FUEL_RHO 1
//For optimal descent time use Kh = 0.157332, Kp = 0.001, del = 0.99, useParachute = false


using namespace std;

double atmospheric_density (double pos)
{

	double alt = pos - MARS_RADIUS;
  
	if ((alt > EXOSPHERE) || (alt < 0.0)) return 0.0;
  
	else return (0.017 * exp(-alt/11000.0));

}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


int main(){
	//file opening
	ofstream myfile;
	ofstream improve;
	myfile.open("data2.txt");
	improve.open("improve.txt");

	//initialisation
	double Kh = 0.04484938;
	double Kp = 0.7;
	double del = 0.7;
	
	//INTEGRATION PARAMETERS
	float dt = 10;
	int t_max = 200000;

	float A = M_PI;
	float As = 20;
	float Cd = 1;
	float Cds = 2;

	//MASS OF THE PLANET
	double M = 5.972e24;//5.972e24

	double MPl = 7.5e26;//8e25
	double G = 6.67e-11;
	double planet[2] = {1e8, 0};//1e8
	double pos[2];//6.371e6 + 2.02e7, 7e7
	double nex[2];//MARS_RADIUS + 1e4
	double las[2];
	double accD[2];
	double vel[2];//3861, 8000
	bool useParachute = false;
	bool autopilotEnabled = false;
	bool train = false;
	
	double m;
	double thr[2] = {0, 0};
	double pow[2];
	double final;
	double accT[2];
	double fuel;
	double rsq;
	double vsq;
	double a;
	double accE[2];
	double posPl[2];
	double rsqPl;
	double aPl;
	double accPl[2];
	double F[2];
	double Fs[2];
	double err[2];
	double P[2];
	double acc[2];
	double step = 0.01;
	
	if (!train) goto goo;
	tra:
		//if (Kp + del < 0.5 || Kp + del > 0.75) goto loop;
		//if (Kh + del > 0.8 || Kh + del < 0.5) goto loop;
		//if (Kh + Kp > 0.1) goto loop;
		//if (Kp + Kh + del > 0.8 || Kp + del + Kh < 0.5) goto loop;

	goo:
		final = 0;
		fuel = FUEL_CAP;
		for (int i=0; i<=2; i++){	
			accT[i] = 0;
			pos[i] = 0;
			nex[i] = 0;
			accD[i] = 0;
			vel[i] = 0;
		}
		//THIS IS THE INITIAL POSITION
		pos[1] = EARTH_RADIUS + 7e7;
		nex[1] = pos[1];
		//THIS IS THE INITIAL VELOCITY (x, y) COMPONENTS
		vel[0] = 8000;

		//integration
		for(float t=0; t<t_max; t=t+dt){

			m = 100 + (fuel * FUEL_RHO);

			//calculate acceleration due to origin planet
			rsq = (nex[0]*nex[0]) + (nex[1]*nex[1]);
			vsq = (vel[0]*vel[0]) + (vel[1]*vel[1]);
			a = -(G*M)/rsq;
			accE[0] = (a*nex[0])/sqrt(rsq);
			accE[1] = (a*nex[1])/sqrt(rsq);

			//calculate acceleration due to second planet
			posPl[0] = nex[0] - planet[0];
			posPl[1] = nex[1] - planet[1];
			rsqPl = (posPl[0]*posPl[0]) + (posPl[1]*posPl[1]);
			aPl = -(G*MPl)/rsqPl;
			accPl[0] = (aPl*posPl[0])/sqrt(rsqPl);
			accPl[1] = (aPl*posPl[1])/sqrt(rsqPl);


			//drag
			F[0] = -0.5*atmospheric_density(sqrt(rsq))*Cd*A*vel[0]*vel[0]*sgn(vel[0]);
			F[1] = -0.5*atmospheric_density(sqrt(rsq))*Cd*A*vel[1]*vel[1]*sgn(vel[1]);			       	
			Fs[0] = -0.5*atmospheric_density(sqrt(rsq))*Cds*As*vel[0]*vel[0]*sgn(vel[0]);
			Fs[1] = -0.5*atmospheric_density(sqrt(rsq))*Cds*As*vel[1]*vel[1]*sgn(vel[1]);
			if (useParachute && vsq < 500 && sqrt((Fs[0]*Fs[0])+(Fs[1]*Fs[1]))<20000){
				accD[0] = (F[0]+Fs[0])/m;
				accD[1] = (F[1]+Fs[1])/m;
			}
			else{
				accD[0] = F[0]/m; 
				accD[1] = F[1]/m;
			}

			//orbital autopilot
			err[0] = -(0.5 + Kh*(sqrt(rsq)-MARS_RADIUS) + vel[0]);
			err[1] = -(0.5 + Kh*(sqrt(rsq)-MARS_RADIUS) + vel[1]);			 	 
			P[0] = Kp * err[0]; 
			P[1] = Kp * err[1];
			for (int i = 0; i<2; i++){
				if (P[i] <= -del) thr[i] = 0;
				else if (P[i] >= (1-del)) thr[i] = 1;
				else thr[i] = del + P[i];
				pow[i] = MAX_THRUST * thr[i];
				if (autopilotEnabled && fuel > (MAX_FUEL_R * dt)){
					accT[i] = pow[i]/m;
					fuel -= (MAX_FUEL_R * sqrt(thr[0] * thr[0] + thr[1] * thr[1])) * dt;
				}
			}
			//combine accelerations
			acc[0] = accE[0]+accPl[0]+accD[0]+accT[0];
			acc[1] = accE[1]+accPl[1]+accD[1]+accT[1];		


			//set second boundary condition
			if (t==0){
				for(int i = 0; i<2; i++){ 
					vel[i] += acc[i]*dt;
					nex[i] = pos[i] + vel[i]*dt;
				}
			}

			//Verlet integrator
			else{
				for (int i = 0; i<2; i++){					
					las[i] = pos[i];
					pos[i] = nex[i];
					nex[i] = 2*pos[i] - las[i] + (dt*dt)*acc[i];
					vel[i] = (nex[i] - pos[i])/dt;
				}
			}

			//write results
			if(sqrt(rsq)<EARTH_RADIUS || sqrt(rsqPl)<EARTH_RADIUS) {final = t; break;} 
			if (!train) myfile << nex[0] <<':'<< nex[1]<<endl;
		}
	loop:
	if (-vel[1]< 1 && vel[1] < 0 && final != 0 && fuel > 10) improve << final <<':'<< vel[1] <<':'<< fuel <<':'<< Kh <<':'<< Kp <<':'<< del <<endl;
	if (train && Kp<1) {Kp+=0.01; goto tra;}
	if (train && del<1) {Kp=0; del+=step; goto tra;}
	if (train && Kh<0.055) {Kp=0; del=0; Kh+=step; goto tra;}
	if (!train) cout<<"Time:"<<final<<" Landing velocity:"<<vel[1]<<" Fuel left:" <<fuel;
	myfile.close();
	improve.close();
	return 0;

}