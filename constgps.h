#ifndef __constGPS_H
#define __constGPS_H

#define PI 3.14159265358979323846
#define TWOPI 6.28318530717958648
#define FOURPI 12.566370614359173
#define PI_DIV_2 1.57079632679489661923
#define PI_DIV_4 0.78539816339744831
#define PI_DIV_3 1.04719755119659775
#define PI_DIV_6 0.523598775598298873

#define RO (180*M_1_PI)
#define RO_D RO0
#define RO_M RO/60.
#define RO_S RO/3600.

//WGS-72
#define mu_WGS72 398600.8e9

// WGS-84
#define mu_WGS84 398600.5e9 // ¨**3/·**2
#define omega_earth_WGS84 7.2921151467e-5 // p†§/·
#define c_WGS84 299792458. // ¨/·
#define PI_WGS84 3.1415926535898 // Á®·´Æ è®
#define a_WGS84 6378137.0
#define _1_alfa_WGS84 298.257223563
#define e2_WGS84 0.00669438006998
#define e_WGS84 0.0818191913305 //0.081819190842622 - 3dUtils
#define C20_NORM_WGS84 -484.16685e-6

// _85
#define omega_earth_85    0.7292115e-4 // p†§/·
#define mu_85    398600.44e9 // ¨**3/·**2
#define a_85    6378136. // ¨
#define _1_alfa_85    298.257
#define e2_85    0.00669438499959
#define e_85    0.0818192214552
#define J2_85    -1082627.e-9

//	Constants: Earth Parameters-90
#define omega_earth_90    0.72921151467e-4 // rad/s
#define mu_90    398600.44e9 // m**3/s**2
#define a_90    6378136.0 // m
#define C20_90      -1082625.7e-9//-484164.953e-9//
#define _1_alfa_90    298.257839303 //298.257839
#define U0_90    62636861.074 // m**2/s**2

//#define x84_90      0.0 // m
//#define y84_90      0.0 // m
//#define z84_90      1.5 // m
#define x84_90      0.0 // m
#define y84_90      2.5 // m
#define z84_90      0.0 // m
//#define om84_90      0.076*PI_WGS84/180./3600. //rad
#define om84_90      1.9e-6

#define f0_GPS    1.57542e9
#define f2_GPS    1.2276e9
#define f0_GLN    1.602e9
#define step_GLN    0.5625e6 // ÉÊ
#define step_f0    0.5625e6 // ÉÊ
#define f0_f2_GLN    7./9. // ÉÊ

#define bFLAT84             0.00335281066474
#define bFLATINV84        298.257223563
#define bDTR                0.01745329251994330
#define bRTD               57.2957795130823229
#define bGM84               3.986005e8
#define bOMEGAE84           7.2921151467e-5
#define bCBAR2084        -484.16685e-6
#define bAJ284              0.00108263
#define bCSPEED             2.99792458e8
#define bESQ84              0.006694379990141316
#define bFREQCA             1.023000e6
#define bFREQP              1.023000e7
#define bFREQL1             1.57542e9
#define bFREQL2             1.22760e9
#define bWEEKSEC       604800
#define bWEEKSEC2      302400
#define bWEEKSEC_ms    604800000
#define bWEEKSEC2_ms   302400000
#define bMJDSUN         45336
#define bLITE001            2.99792458e5

#endif

