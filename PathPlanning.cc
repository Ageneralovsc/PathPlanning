#include "PathPlanning.h"
#include "constgps.h"
#include <math.h>
#include <stdio.h>
#include <vector>
#include <array>
#include <iostream>

PATHPLAN::PATHPLAN()
{
// nothing should be done before vertices are not set
}

void PATHPLAN::init()
{
//
// Init before start
//
    Rmin = 2.5;
    Rgoaway = 3*Rmin;
    Nspl = 0;
    
    startflag = true;
    goawayflag = false;
    isStartingManeuver = false;    
    
    //target start of trejectory
    spline(1, 0.0, Pspl, dPspl, ddPspl);
    x0 = Pspl[0];
    y0 = Pspl[1];
    double e[2] = {1,0};
    tau0 = AngSign(e, dPspl); 
}

PATHPLAN::~PATHPLAN()
{
//
}

bool PATHPLAN::Run(double x, double y, double theta){
    
  bool ret = true;
  double orient[2] = {cos(theta), sin(theta)};
  double X[2] = {x,y};
    
  if (startflag){
    xs = x;
    ys = y;
    thetas = theta;
    startflag = false;
  }
    
  if ((pow((xs-x0),2)+pow((ys-y0),2) <= pow((Rgoaway),2)) && (x == xs) && (y == ys))
    goawayflag = true;

  if (pow((x-x0),2)+pow((y-y0),2) > pow((Rgoaway),2))
    goawayflag = false;

  // Calculation DKPsi
  double XsPspl[2] = {xs-Pspl[0], ys-Pspl[1]};
  double XPspl[2] =  {x-Pspl[0],  y-Pspl[1]};
  
  if (dot(XsPspl,dPspl) < 0)
    isStartingManeuver = true;
  
  if (dot(XPspl, dPspl) < 0){ //We are moving behind of spline starting point - construct the line and follow it
    // Distance from point to line signed with according to NormVec(dR)
    double UN[2];
    UnityNormVec(dPspl, UN); 
    
    DKPsi[0] = dot(XPspl, UN);
    DKPsi[1] = 0;
    DKPsi[2] = AngSign(dPspl,orient);
  } else { // We are moving on the spline
    isStartingManeuver = false;
    
    double Xc[2];
    double phi, PHI;
    double r0[2],dr0[2],ddr0[2],r1[2],dr1[2],ddr1[2];

    // Nspl = 1..sw.size()-3
    while( Nspl<=(sw.size()-3) ) {
      if (Nspl < 1)
        Nspl = 1;

      spline(Nspl, 0.0, r0, dr0, ddr0);
      spline(Nspl, 1.0, r1, dr1, ddr1);
      NorCrossPnt(r0,dr0,r1,dr1, Xc);

      if ((isinf(Xc[0])||isinf(Xc[1])) ) { // Xc=inf seems like following the line
        double Ndr0[2], XL[2], XLr0[2], r1r0[2];
        NormVec(dr0,Ndr0);
        CrossPnt(X,Ndr0,r0,dr0, XL);
          
        if (isinf(XL[0])||isinf(XL[1])) {
            // WARNING ("XL is inf \n");
            ret = false;
          } else {
            XLr0[0] = XL[0]-r0[0];
            XLr0[1] = XL[1]-r0[1];
            r1r0[0] = r1[0]-r0[0];
            r1r0[1] = r1[1]-r0[1];
            phi = sqrt(dot(XLr0,XLr0)); //*cos(AngbwVec(XLr0,r0)); // norm(XL-r0) * cos(AngbwVec(XL-r(Nspl,0),r(Nspl,0)))
            PHI = sqrt(dot(r1r0,r1r0)); // norm(r1-r0);
            Xc[0] = INFINITY;
            Xc[1] = INFINITY;
          }
        }
        else { // Xc!=inf => following the spline
          double r0Xc[2], r1Xc[2], XXc[2];
          r0Xc[0] = r0[0]-Xc[0];
          r0Xc[1] = r0[1]-Xc[1];
          r1Xc[0] = r1[0]-Xc[0];
          r1Xc[1] = r1[1]-Xc[1];
          XXc[0] = X[0]-Xc[0];
          XXc[1] = X[1]-Xc[1];

          PHI = AngSign(r0Xc, r1Xc)*180/PI;
          phi = AngSign(r0Xc, XXc)*180/PI;
          phi = phi*copysign(1, PHI);
          PHI = PHI*copysign(1, PHI);
        }

        // Checking is the Nspl number right?
        if (phi>PHI) {
          Nspl = Nspl+1;
        }
        else if ( (phi<= PHI) && (phi>=0) ) {
          break;
        }
        else {//if (phi< 0) {
          // WARNING("phi < 0 exit?\n");
          Nspl = Nspl - 1;
          break;
        }
      }

      if (Nspl < 1) {
        // WARNING('Behind of spline start');
        ret = false;
      }
      else if (Nspl > (sw.size()-3)) {
        // WARNING('End of spline'); 
        ret = false;
      }
      else {
        DistKPsi(X,orient,Nspl,phi,PHI,Xc,DKPsi);
      }
  }  
  
// Starting Maneuver
  if(isStartingManeuver) {
    int param, params, R;
    double tangent[2];

    if ((AngSign(dPspl, XsPspl)) < 0)
      params = +1;
    else
      params = -1;

    if ((AngSign(dPspl, XPspl)) < 0)
      param = +1;
    else
      param = -1;

// target circle radius and tangent vector
    
    if (AngSign(dPspl, XsPspl) == 0) {
      tangent[0] = dPspl[0];
      tangent[1] = dPspl[1];
      R = INFINITY;
    } else {
        double Xcir[2];
        BuildCircle(x,y,x0,y0,tau0,Xcir);
        R = sqrt(pow((x0-Xcir[0]),2) + pow((y0-Xcir[1]),2));
        double dxdy[2] = {Xcir[0]-x, Xcir[1]-y};

        if ( R < Rmin ) {
          tangent[0] = dPspl[0];
          tangent[1] = dPspl[1];
        } else  {
          UnityNormVec(dxdy, tangent);
          tangent[0] = param*tangent[0];
          tangent[1] = param*tangent[1];
        }
    }
    
    // target heur vector
    heur[0] = AngSign(tangent,orient); //dalpha
    heur[1] = PI + DKPsi[2] - params*PI/2.5; // dalpha2 // [x0;y0] - 2*Rgoaway*[cos(tau + params*PI/2.5);sin(tau + params*PI/2.5)]
    heur[2] = dot(tangent, orient);  //comp
    heur[3] = dot(dPspl, orient);  //comp2
    heur[4] = params;
  }
  else {
    heur[0] = 0; //dalpha
    heur[1] = 0; // dalpha2 // [x0;y0] - 2*Rgoaway*[cos(tau + params*PI/2.5);sin(tau + params*PI/2.5)]
    heur[2] = 0;  //comp
    heur[3] = 0;  //comp2
    heur[4] = 0;
  }
  
  return ret;
}

void PATHPLAN::set_spline_vertices(const std::vector<std::array<double, 2>>& spline_vector)
{
// Uniform cubic B-spline vertices.
// Adding phantom vertices according to: Bartels, Introduction to spline, 1987, p.43    
    
    sw = spline_vector;
    
    std::array<double, 2> startrow = {2*sw[0][0]-sw[1][0], 2*sw[0][1]-sw[1][1]};    
    std::array<double, 2> endrow = {2*sw[sw.size()-1][0]-sw[sw.size()-2][0], 2*sw[sw.size()-1][1]-sw[sw.size()-2][1]};
    
    sw.insert(sw.begin(), startrow);
    sw.emplace_back(endrow);           
    //        
    
#ifdef DEBUG_VERTICES
    std::cerr << "Combined waypoints:" << std::endl;

    for(unsigned int i = 0; i < sw.size(); i++)
        std::cerr << i << "x=" << sw[i][0] << " y=" << sw[i][1] << std::endl;
#endif   
        
}

void PATHPLAN::spline(int i, double t, double *r, double *dr, double *ddr)
{
//
// Uniform cubic B-spline r(t).
// Output: r[2], dr[2], ddr[2].
//
  double x1, x2, x3, x4, y1, y2, y3, y4;
  double x0 = 0; //here GK E
  double y0 = 0; //      N   

  x1 = sw[i-1][0]  +x0;
  y1 = sw[i-1][1]  +y0;
  x2 = sw[i][0]    +x0;
  y2 = sw[i][1]    +y0;
  x3 = sw[i+1][0]  +x0;
  y3 = sw[i+1][1]  +y0;
  x4 = sw[i+2][0]  +x0;
  y4 = sw[i+2][1]  +y0;

  r[0] = (x2/2 - x1/6 - x3/2 + x4/6)*pow(t,3) + (x1/2 - x2 + x3/2)*pow(t,2) + (x3/2 - x1/2)*t + x1/6 + (2*x2)/3 + x3/6; //x
  r[1] = (y2/2 - y1/6 - y3/2 + y4/6)*pow(t,3) + (y1/2 - y2 + y3/2)*pow(t,2) + (y3/2 - y1/2)*t + y1/6 + (2*y2)/3 + y3/6; //y

  dr[0] = x3/2 - x1/2 + 2*t*(x1/2 - x2 + x3/2) - 3*pow(t,2)*(x1/6 - x2/2 + x3/2 - x4/6); //dx
  dr[1] = y3/2 - y1/2 + 2*t*(y1/2 - y2 + y3/2) - 3*pow(t,2)*(y1/6 - y2/2 + y3/2 - y4/6); //dy

  ddr[0] = x1 - 2*x2 + x3 - 6*t*(x1/6 - x2/2 + x3/2 - x4/6); //ddx
  ddr[1] = y1 - 2*y2 + y3 - 6*t*(y1/6 - y2/2 + y3/2 - y4/6); //ddy
}


void PATHPLAN::NormVec(double *A, double *N) {
  // Rotation of Vec A to +90 degrees
  N[0] = -A[1];
  N[1] =  A[0];
}

void PATHPLAN::UnityNormVec(double *A, double *N) {
  // Normal Vector with unity magnitude
  NormVec(A,N);
  double normN = sqrt(dot(N,N));
  N[0] = N[0]/normN;
  N[1] = N[1]/normN;
}

double PATHPLAN::AngbwVec(double *R1,double *R2) {
  // Calculate angle magnitude b/w two vectors [0..PI]
  double cos_ang = (R1[0]*R2[0]+R1[1]*R2[1])/(sqrt(R1[0]*R1[0]+R1[1]*R1[1])*sqrt(R2[0]*R2[0]+R2[1]*R2[1]));

  if (cos_ang>1)
    cos_ang=1;
  else if (cos_ang<-1)
    cos_ang=-1;

  //  double ANG = fmin(acos(cos_ang),PI-acos(cos_ang)); // another realisation (should not be used here)
  double ANG = acos(cos_ang);

  return ANG;
}

double PATHPLAN::AngSign(double *R1,double *R2) {
  // Calculate angle between two vectors with sign (-PI..PI)
  double sin_ang = (R1[0]*R2[1]-R1[1]*R2[0])/(sqrt(R1[0]*R1[0]+R1[1]*R1[1])*sqrt(R2[0]*R2[0]+R2[1]*R2[1]));
  if (sin_ang>1)
    sin_ang=1.;
  if (sin_ang<-1)
    sin_ang=-1.;


  double cos_ang = (R1[0]*R2[0]+R1[1]*R2[1])/(sqrt(R1[0]*R1[0]+R1[1]*R1[1])*sqrt(R2[0]*R2[0]+R2[1]*R2[1]));
  if (cos_ang>1)
    cos_ang=1.;
  if (cos_ang<-1)
    cos_ang=-1.;

  double ANG = copysign(acos(cos_ang), sin_ang);

  return ANG;
}

void PATHPLAN::CrossPnt(double *R1,double *A1,double *R2,double *A2,double *Mn) {
  // Crossing point of two lines of A1 and A2 with starting points of R1 and R2
  if ((A1[0]*A2[1] - A1[1]*A2[0]) != 0) {
    Mn[0] = (A1[0]*A2[0]*R1[1] - A1[1]*A2[0]*R1[0] - A1[0]*A2[0]*R2[1] + A1[0]*A2[1]*R2[0])/(A1[0]*A2[1] - A1[1]*A2[0]);
    Mn[1] = (A1[0]*A2[1]*R1[1] - A1[1]*A2[1]*R1[0] - A1[1]*A2[0]*R2[1] + A1[1]*A2[1]*R2[0])/(A1[0]*A2[1] - A1[1]*A2[0]);
  }
  else {
    Mn[0] = INFINITY;
    Mn[1] = INFINITY;
  }
}


void PATHPLAN::NorCrossPnt(double *R1,double *A1,double *R2,double *A2,double *Mn) {
  // %Crossing point of two normal to A1 and A2 vectors with starting points of R1 and R2
  if ((A1[0]*A2[1] - A1[1]*A2[0]) != 0) {
    Mn[0] =  (A1[0]*A2[1]*R1[0] + A1[1]*A2[1]*R1[1] - A1[1]*A2[0]*R2[0] - A1[1]*A2[1]*R2[1])/(A1[0]*A2[1] - A1[1]*A2[0]);
    Mn[1] = -(A1[0]*A2[0]*R1[0] + A1[1]*A2[0]*R1[1] - A1[0]*A2[0]*R2[0] - A1[0]*A2[1]*R2[1])/(A1[0]*A2[1] - A1[1]*A2[0]);
  }
  else {
    Mn[0] = INFINITY;
    Mn[1] = INFINITY;
  }
}


bool PATHPLAN::DistKPsi(double *X,double *orient,int N_spl,double phi,double PHI,double *Xc, double *DKPsi)
{
//    
// Calculation of Distance D, Curvature K and angle Psi - DKPsi[3]
// double orient[2] = {cos(theta), sin(theta)}; // orientation vector
//  
  bool ret = true;
  double UN[2], R[2], dR[2], ddR[2];

  if ( (isinf(Xc[0])) || (isinf(Xc[1])) ) { // looks like a line following
    spline(N_spl, 0.0, R, dR, ddR);
    UnityNormVec(dR,UN);

    DKPsi[0] = (X[0]-R[0])*UN[0] + (X[1]-R[1])*UN[1]; // (X-R(0.0))'*UN
    DKPsi[1] = 0;
    DKPsi[2] = AngSign(dR,orient);
  }
  else if ( ( (isfinite(Xc[0])) && (isfinite(Xc[1])) ) && ( sqrt((X[0]-Xc[0])*(X[0]-Xc[0])+(X[1]-Xc[1])*(X[1]-Xc[1])) < 1e-4 ) ) {
    // X=Xc singular point => take D,K from start of spline
    spline(N_spl, 0.0, R, dR, ddR);
    UnityNormVec(dR,UN);

    DKPsi[0] = (X[0]-R[0])*UN[0] + (X[1]-R[1])*UN[1]; // (X-R(0.0))'*UN
    DKPsi[1] = Cur(dR,ddR);
    DKPsi[2] = AngSign(dR,orient);
  }
  else if ( (isfinite(Xc[0])) && (isfinite(Xc[1])) ) { // regular spline with Xc
    double tx0 = phi/PHI;
    spline(N_spl, tx0, R, dR, ddR);
    double dXR[2], dXXc[2], NdXXc[2], UN[2]; //X-R, X-Xc, NormVec(X-Xc), UnityNormVec
    dXR[0] = X[0]-R[0];
    dXXc[0] = X[0]-Xc[0];
    dXR[1] = X[1]-R[1];
    dXXc[1] = X[1]-Xc[1];
    NormVec(dXXc, NdXXc); //NormVec(X-Xc)
    double dtx = dot(dXR,NdXXc)/dot(dR,NdXXc); //(X - R(tx0))'*NormVec(X-Xc)/((dR(tx0))'*NormVec(X-Xc))

    spline(N_spl, tx0+dtx, R, dR, ddR);
    UnityNormVec(dR, UN); //NormVec(dR(tx0+dtx))/norm(NormVec(dR(tx0+dtx)))
    dXR[0] = X[0]-R[0];
    dXR[1] = X[1]-R[1];

    DKPsi[0] = dot(dXR,UN); // (X - R(tx0+dtx))'*UnityNormVec
    DKPsi[1] = Cur(dR,ddR); // Cur(dR(tx0+dtx),ddR(tx0+dtx))
    DKPsi[2] = AngSign(dR,orient);
  }
  else {
    //WARNING: Xc may be Inf or NaN - isfinite(Xc) is false
    DKPsi[0] = 0;
    DKPsi[1] = 0;
    DKPsi[2] = 0;
    ret = false;
  }
  
  return ret;
}

double PATHPLAN::dot(double *A, double *B) {
//
// Dot product of a two vectors A[2] and B[2]
//
  double DotP = A[0]*B[0]+A[1]*B[1];

  return DotP;
}

double PATHPLAN::Cur(double *dR, double *ddR) {
//
// The curvature of the spline in the point with existing first and second derivatives of R(s).
// Input: dR[2], ddR[2].
//
  double K;
  if (dot(dR,dR) != 0) {
      K = (dR[0]*ddR[1]-dR[1]*ddR[0])/pow(sqrt(dot(dR,dR)),3);      
  } else {
      K = INFINITY;
  }

  return K;
}

void PATHPLAN::BuildCircle(double x, double y, double x0, double y0, double tau, double *Xc) {
//    
// Build circle touching the vector <{x0;y0} + {cos(tau);sin(tau)}> in the starting point {x0;y0} passing the point {x;y}.
// Output: coordinates of the circle center Xc[2].
//    
  double det = (-sin(tau)*(x-x0)+cos(tau)*(y-y0));
  if (det == 0) {
    Xc[0] = INFINITY;
    Xc[1] = INFINITY;
  }else {
    double bb[2] = {x*x+y*y-x0*x0-y0*y0, -cos(tau)*x0-sin(tau)*y0};
    double AA[4] = {-sin(tau)/2/det, -(y-y0)/det, cos(tau)/2/det,  (x-x0)/det};

    Xc[0] = AA[0]*bb[0]+AA[1]*bb[1];
    Xc[1] = AA[2]*bb[0]+AA[3]*bb[1];
  }
}
