#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: dllh2enu.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "dllh2enu.h"

/* Function Definitions */

/*
 * function [de,dn,du]=dllh2enu(lon0,lat0,h0,lon,lat,h)
 * Arguments    : int b_lon0
 *                int b_lat0
 *                float b_h0
 *                int lon
 *                int lat
 *                float h
 *                float *de
 *                float *dn
 *                float *du
 * Return Type  : void
 */
void dllh2enu(int b_lon0, int b_lat0, float b_h0, int lon, int lat, float h, float *de,
              float *dn, float *du)
{
    float phi;
    float dphi;
    float dlam;
    float dh;
    float x;
    float b_x;

    /*  dllh2enu - convert latitude, longitude, and altitude to */
    /*             ENU cartesian */
    /*  Converting GPS Coordinates to Navigation Coordinates (ENU) */
    /*  USAGE: */
    /*  [de,dn,du]=dllh2enu(lon0,lat0,h0,lon,lat,h) */
    /*   */
    /*  de = EAST X-coordinate (m) */
    /*  dn = NORTH Y-coordinate (m) */
    /*  du = HEIGHT Z-coordinate (m) */
    /*  lat = geodetic latitude (radians) */
    /*  lon = longitude (radians) */
    /*  alt = height above WGS84 ellipsoid (m) */
    /*   */
    /*  Notes: This function assumes the WGS84 model. */
    /*         Latitude is customary geodetic (not geocentric). */
    /*   */
    /*   */
    /*  Modified by Zhongyuan Liu , Feb 2018 */
    /*  Modified by Zhongyuan Liu , April 2018 */
    /* 'dllh2enu:22' coder.inline('never'); */
    /*  % WGS84 ellipsoid constants: */
    /*  lon0 = lon0/pi*180; */
    /*  lat0 = lat0/pi*180; */
    /*  lon = lon/pi*180; */
    /*  lat = lat/pi*180; */
    /*  a = 6378137; */
    /*  e = 8.1819190842622e-2; */
    /*  dlat = lat-lat0; */
    /*  dlon = lon - lon0; */
    /*  dh = h - h0; */
    /*  % intermediate calculation */
    /*  % (prime vertical radius of curvature) */
    /*  % N = a ../ sqrt(1 - e.^2 ..* sin(lat)..^2); */
    /*  % lon lambda, lat phi */
    /*  X = sqrt(1 - e^2 * sin(lat0)^2); */
    /*  % results: */
    /*  de = (a/X+h0)*cos(lat0)*dlon -
     * (a*(1-e^2)/X^3+h0)*sin(lat0).*dlat.*dlon+cos(lat0)*dlon.*dh; */
    /*  % de = (a/tmp1+h0)*cp*dlam - (a*(1-e2)/(tmp1^3)+h0)*sp.*dphi.*dlam +cp.*dlam.*dh;
     */
    /*  dn = (a*(1-e^2)/X^3+h0).*dlat
     * + 1.5*a*cos(lat0)*sin(lat0)*e^2*dlat.^2+sin(lat0)^2*dh.*dlat... */
    /*      +0.5*sin(lat0)*cos(lat0)*(a./X+h0)*dlon.^2; */
    /*  % dn = (a*(1-e2)/tmp1^3 + h0)*dphi + 1.5*cp*sp*a*e2*dphi.^2 + sp^2.*dh.*dphi + ...
     */
    /*  % 0.5*sp*cp*(a/tmp1 +h0)*dlam.^2; */
    /*  du = dh - 0.5*a*(1-1.5*e^2*cos(lat0)+0.5*e.^2+h0/a)*dlat.^2-0.5*(a*cos(lat0)^2/X -
     * h0*cos(lat0)^2)*dlon.^2; */
    /*  */
    /* %%%%%%%%%CONSTANTS */
    /* 'dllh2enu:50' a = 6378137; */
    /*  b = 6356752.3142; */
    /*  e2 = 1 - (b/a)^2; */
    /* 'dllh2enu:53' e2 = 0.00669438; */
    /* %%%%%%%%%Location of reference point in radians */
    /*  llh0 = [lat0,lon0,h0]; */
    /*  llh = [lat,lon,h]; */
    /* 'dllh2enu:57' phi = single(lat0)*pi/180*1e-7; */
    phi = (float)b_lat0 * 3.14159274F / 180.0F * 1.0E-7F;

    /* !!!!precision loss, can be optimized */
    /*  lam = lon0*pi/180; */
    /* %%%%%%%%%Location of data points in radians */
    /* 'dllh2enu:61' dphi= single(lat-lat0)*1e-7/180*pi; */
    dphi = (float)(lat - b_lat0) * 1.0E-7F / 180.0F * 3.14159274F;

    /* 'dllh2enu:62' dlam= single(lon-lon0)*1e-7/180*pi; */
    dlam = (float)(lon - b_lon0) * 1.0E-7F / 180.0F * 3.14159274F;

    /* 'dllh2enu:63' dh = h - h0; */
    dh = h - b_h0;

    /* %%%%%%%%%Some useful definitions */
    /* 'dllh2enu:65' tmp1 = sqrt(1-e2*sin(phi)^2); */
    x = sinf(phi);
    x = sqrtf(1.0F - 0.00669438F * (x * x));

    /*  cl = cos(lam); */
    /*  sl = sin(lam); */
    /* 'dllh2enu:68' cp = cos(phi); */
    b_x = cosf(phi);

    /* 'dllh2enu:69' sp = sin(phi); */
    phi = sinf(phi);

    /* %%%%%%%%%Transformations */
    /* 'dllh2enu:71' de = (a/tmp1+h)*cp*dlam - (a*(1-e2)/(tmp1^3)+h)*sp.*dphi.*dlam
     * +cp.*dlam.*dh; */
    *de = ((6.378137E+6F / x + h) * b_x * dlam -
           (6.3354395E+6F / powf(x, 3.0F) + h) * phi * dphi * dlam) +
          b_x * dlam * dh;

    /* 'dllh2enu:72' dn = (a*(1-e2)/tmp1^3 + h)*dphi + 1.5*cp*sp*a*e2*dphi.^2 +
     * sp^2*dh.*dphi + ... */
    /* 'dllh2enu:73' 0.5*sp*cp*(a/tmp1 +h)*dlam.^2; */
    *dn = (((6.3354395E+6F / powf(x, 3.0F) + h) * dphi +
            1.5F * b_x * phi * 6.378137E+6F * 0.00669438F * (dphi * dphi)) +
           phi * phi * dh * dphi) +
          0.5F * phi * b_x * (6.378137E+6F / x + h) * (dlam * dlam);

    /* 'dllh2enu:74' du = dh - 0.5*(a-1.5*a*e2*cp^2+0.5*a*e2+h)*dphi.^2 - ... */
    /* 'dllh2enu:75' 0.5*cp^2*(a/tmp1 -h)*dlam.^2; */
    *du = (dh - 0.5F * (((6.378137E+6F - 64046.5156F * (b_x * b_x)) + 21348.8379F) + h) *
                    (dphi * dphi)) -
          0.5F * (b_x * b_x) * (6.378137E+6F / x - h) * (dlam * dlam);
}

/*
 * File trailer for dllh2enu.c
 *
 * [EOF]
 */
