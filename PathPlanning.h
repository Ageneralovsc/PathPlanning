#ifndef __PATHPLAN_H__
#define __PATHPLAN_H__

#include <vector>
#include <array>

class PATHPLAN {
public:
    PATHPLAN();
    ~PATHPLAN();
    
    bool goawayflag;
    bool isStartingManeuver;    

    double Rmin;
    double Rgoaway;
    
    double DKPsi[3];
    double heur[5];
    
    bool Run(double x, double y, double theta);
    void set_spline_vertices(const std::vector<std::array<double, 2>>& spline_vector);
    void init();
    
    double xs, ys;
    double thetas;
    double Nspl;
    bool startflag;
    
private:


    double Pspl[2], dPspl[2], ddPspl[2];
    double x0, y0, tau0;
    std::vector<std::array<double, 2>> sw; ///> spline waypoints

    
    void NormVec(double *A, double *N);
    void UnityNormVec(double *A, double *N);
    double AngbwVec(double *R1,double *R2);
    double AngSign(double *R1,double *R2);
    void CrossPnt(double *R1,double *A1,double *R2,double *A2,double *Mn);
    void NorCrossPnt(double *R1,double *A1,double *R2,double *A2,double *Mn) ;
    bool DistKPsi(double *X,double *orient,int N_spl,double phi,double PHI,double *Xc, double *DKPsi);
    void BuildCircle(double x, double y, double x0, double y0, double tau, double *Xc);
    double Cur(double *dR, double *ddR);
    double dot(double *A, double *B);
    void spline(int i, double t, double *r, double *dr, double *ddr);
};


#endif
