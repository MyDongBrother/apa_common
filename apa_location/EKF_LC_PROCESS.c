#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: EKF_LC_PROCESS.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 23-Nov-2018 22:21:28
 */

/* Include Files */

#include "EKF_LC_PROCESS.h"
#include "norm.h"
#include "sum.h"
#include "SUKF.h"
#include "EKF_LC.h"
#include "LinearDemingRegression.h"
#include "dllh2enu.h"

/* Variable Definitions */
static float buffer_z[60];
static boolean_T buffer_z_not_empty;
static unsigned short deming_window_count;
static boolean_T deming_window_count_not_empty;
static unsigned char window_count;
static unsigned char heading_estimated;
static int lon0;
static int lat0;
static float h0;
static float EPROM_EKF_distance;
static float EPROM_odometer_distance;
static float EPROM_motor_distance;
static float location[4];
static float PK_xyS_offset[3];
static float moving_avg_pitch;
static float moving_avg_roll;
static int lon_prev;
static int lat_prev;
static unsigned char set_origin_heading_once;
static unsigned char reset_frame_origin;
static unsigned char reset_all_diverge;
static unsigned char b_enu_set;

extern void RTE_PK_Location_Get_CurPos(float pa[4]);
/* Function Declarations */
void b_EKF_LC_PROCESS(EKF_LC_PROCESS_arg_Type *EKF_LC_PROCESS_arg);

/* Function Definitions */

/*
 * function EKF_LC_PROCESS_arg = EKF_LC_PROCESS(EKF_LC_PROCESS_arg)
 * Arguments    : EKF_LC_PROCESS_arg_Type *EKF_LC_PROCESS_arg
 * Return Type  : void
 */
void b_EKF_LC_PROCESS(EKF_LC_PROCESS_arg_Type *EKF_LC_PROCESS_arg)
{
    unsigned char set_origin_heading;
    unsigned char c_reset_origin_heading_process_;
    float psi0;
    int i;
    float z[2];
    unsigned char GPS_OK;
    unsigned char measurement_valid;
    ResetType rEQ0;
    unsigned char collect_start;
    float q;
    float dn;
    float du;
    int i6;
    float fv8[58];
    float fv9[30];
    float fv10[30];
    float r[2];
    float b_z[2];
    float b_EKF_LC_PROCESS_arg[2];
    static const signed char iv4[3] = {0, 1, 3};

    /*  EKF_LC_PROCESS - Relative localization using EKF */
    /*  [X,Pdiag,z] =
     * EKF_LC_PROCESS(time,lon,lat,h,angRatez,v_Skf,dS_wheel,GPS_accuracy,dt,flag,reset,set,set_value)
     */
    /*  de = EAST X-coordinate (m) */
    /*  dn = NORTH Y-coordinate (m) */
    /*  du = HEIGHT Z-coordinate (m) */
    /*  lat = geodetic latitude (radians) */
    /*  lon = longitude (radians) */
    /*  alt = height above WGS84 ellipsoid (m) */
    /*  */
    /*  Notes: This function assumes the WGS84 model. */
    /*         Latitude is customary geodetic (not geocentric). */
    /*  */
    /*  */
    /*  Created by Zhongyuan Liu */
    /*  Modified by Zhongyuan Liu ,18 April 2018: add dS_wheel */
    /*  v_filtered should be replaced with v_motor or filtered speed */
    /* { */
    /* 	-------------------------------------------------------------------------------------
     */
    /* 	@author     Zhongyuan Liu */
    /* 	@date   Sep 25, 2018 */
    /* 	@version        V1.0.45 */
    /* 	@since        add slotnumber, merge pk and rd reset, pk and rd use the */
    /*                   same result */
    /* 	-------------------------------------------------------------------------------------
     */
    /* } */
    /* 'EKF_LC_PROCESS:27' coder.inline('never'); */
    /* 'EKF_LC_PROCESS:33' MAX_CAL_XY=single(20000); */
    /* Origin of the local frame will be reset to [0;0] when vehicle position exceeds
     * MAX_CAL_XY */
    /* 'EKF_LC_PROCESS:34' MAX_CAL_RANGE = single(80000); */
    /* Origin of the local frame will be reset to [0;0] when vehicle position exceeds
     * MAX_CAL_RANGE todo determine the shreshold */
    /* to do: experiment the value: 1km, 5km, 10km, .... by driving 100km */
    /* 'EKF_LC_PROCESS:36' range_interval_4scale_factor = single(100); */
    /* Calculate wheel and motor scale factor very RANGE_FACTOR */
    /* 'EKF_LC_PROCESS:37' window_odometer_distance4scale_factor = single(5000); */
    /* the window size of odometer distance record for scale_factor. */
    /* 'EKF_LC_PROCESS:38' GPS_EKF_MAX_DIFF = single(200); */
    /* max difference of gps and ekf should not exceed 500m otherwise reset origin and
     * heading of ekf */
    /* 'EKF_LC_PROCESS:39' rear_wheel_scale_factor = single(1); */
    /* 'EKF_LC_PROCESS:40' motor_scale_factor = single(1); */
    /* 'EKF_LC_PROCESS:41' deming_window_size = uint16(30); */
    /* 'EKF_LC_PROCESS:42' set_origin_heading = uint8(0); */
    set_origin_heading = 0;

    /* set local frame paralleled with enu frame, origin of local frame is set to GPS enu
     * coordinates */
    /* 'EKF_LC_PROCESS:43' reset_origin_heading_process_restart = uint8(0); */
    c_reset_origin_heading_process_ = 0;

    /* 'EKF_LC_PROCESS:44' EKF_LC_PROCESS_arg.state = uint8(0); */
    EKF_LC_PROCESS_arg->state = 0;

    /* EKF_LC_PROCESS_arg.state of EKF_LC_PROCESS, 0 normal, 1
     * EKF_LC_PROCESS_arg.reset_all, 2 no measurement, 2 is not used,... */
    /*  3 reset_frame_origin,4 in deming,5 deming success,set_origin_heading, 6 KF
     * diverge, */
    /*  7 EPROM_EKF_distance cal, 8 GPS_EKF_MAX_DIFF reached */
    /*  reset_PK_origin = uint8(0);%reset origin of pk local frame to [0,0]; */
    /* 'EKF_LC_PROCESS:48' psi0 = single(0); */
    psi0 = 0.0F;

    /* 'EKF_LC_PROCESS:49' z = single([0;0]); */
    for (i = 0; i < 2; i++)
    {
        z[i] = 0.0F;
    }

    /* 'EKF_LC_PROCESS:50' GPS_accuracy_max = uint8(3); */
    /* 'EKF_LC_PROCESS:51' pk_max_speed = single(10); */
    /*  */
    /* measurement_valid = uint8(1);%use measurement */
    /* 'EKF_LC_PROCESS:54' if EKF_LC_PROCESS_arg.GPS_Accuracy < GPS_accuracy_max &&
     * EKF_LC_PROCESS_arg.GPS_Accuracy >= 1 */
    if ((EKF_LC_PROCESS_arg->GPS_Accuracy < 3U) &&
        (EKF_LC_PROCESS_arg->GPS_Accuracy >= 1U))
    {
        /* 'EKF_LC_PROCESS:55' GPS_OK = uint8(1); */
        GPS_OK = 1;
    }
    else
    {
        /* 'EKF_LC_PROCESS:56' else */
        /* 'EKF_LC_PROCESS:57' GPS_OK = uint8(0); */
        GPS_OK = 0;
    }

    /* 'EKF_LC_PROCESS:60' measurement_valid = uint8(1); */
    measurement_valid = 1;

    /* determined at EKF_LC_PROCESS instead of Location.c */
    /* 'EKF_LC_PROCESS:61' if isempty(Rear_wheel_fac_ds) */
    /* 'EKF_LC_PROCESS:65' if isempty(reset_all_diverge) */
    /* 'EKF_LC_PROCESS:68' EKF_LC_PROCESS_arg.reset_all =
     * uint32(EKF_LC_PROCESS_arg.reset_all||reset_all_diverge); */
    if ((EKF_LC_PROCESS_arg->reset_all != 0U) || (reset_all_diverge != 0))
    {
        rEQ0 = RESET;
    }
    else
    {
        rEQ0 = NOT_RESET;
    }

    EKF_LC_PROCESS_arg->reset_all = rEQ0;

    /* 'EKF_LC_PROCESS:69' if isempty(buffer_z) || EKF_LC_PROCESS_arg.reset_all */
    if ((!buffer_z_not_empty) || (EKF_LC_PROCESS_arg->reset_all != 0U))
    {
        /* 'EKF_LC_PROCESS:70' buffer_z = single(zeros(2,deming_window_size)); */
        memset(&buffer_z[0], 0, 60U * sizeof(float));
        buffer_z_not_empty = true;

        /* 'EKF_LC_PROCESS:71' GPS_signal_received = uint8(0); */
        /* 'EKF_LC_PROCESS:72' run_ekf = uint8(0); */
        /* 'EKF_LC_PROCESS:73' heading_estimated = uint8(0); */
        heading_estimated = 0;

        /* 'EKF_LC_PROCESS:74' GPS_signal_received_once = uint8(0); */
        /* 'EKF_LC_PROCESS:75' set_origin_heading_once = uint8(0); */
        set_origin_heading_once = 0;

        /* 'EKF_LC_PROCESS:76' lon0 = int32(0); */
        lon0 = 0;

        /* 'EKF_LC_PROCESS:77' lat0 =int32(0); */
        lat0 = 0;

        /* 'EKF_LC_PROCESS:78' h0 = single(0); */
        h0 = 0.0F;

        /* 'EKF_LC_PROCESS:79' lon_prev = int32(0); */
        lon_prev = 0;

        /* 'EKF_LC_PROCESS:80' lat_prev =int32(0); */
        lat_prev = 0;

        /* 'EKF_LC_PROCESS:81' EPROM_EKF_distance = single(0); */
        EPROM_EKF_distance = 0.0F;

        /* 'EKF_LC_PROCESS:82' EPROM_odometer_distance = single(0); */
        EPROM_odometer_distance = 0.0F;

        /* 'EKF_LC_PROCESS:83' EPROM_motor_distance = single(0); */
        EPROM_motor_distance = 0.0F;

        /* 'EKF_LC_PROCESS:84' location = single([0;0;0;0]); */
        RTE_PK_Location_Get_CurPos(location);

        /* 'EKF_LC_PROCESS:85' PK_xyS_offset = single([0;0;0]); */
        for (i = 0; i < 3; i++)
        {
            PK_xyS_offset[i] = 0.0F;
        }

        /* 'EKF_LC_PROCESS:86' moving_avg_pitch = single(0); */
        moving_avg_pitch = 0.0F;

        /* 'EKF_LC_PROCESS:87' moving_avg_roll = single(0); */
        moving_avg_roll = 0.0F;

        /* 'EKF_LC_PROCESS:88' reset_frame_origin = uint8(0); */
        reset_frame_origin = 0;

        /* reset origin of local frame to [0,0] */
        /* 'EKF_LC_PROCESS:89' b_enu_set = uint8(0); */
        b_enu_set = 0;
    }

    if ((EKF_LC_PROCESS_arg->reset_all != 0U))
    {
        for (i = 0; i < 4; i++)
        {
            location[i] = 0.0F;
        }
    }

    /* 'EKF_LC_PROCESS:92' if EKF_LC_PROCESS_arg.reset_all == 1 */
    if (EKF_LC_PROCESS_arg->reset_all == 1U)
    {
        /* 'EKF_LC_PROCESS:93' EKF_LC_PROCESS_arg.reset_KF = uint32(1); */
        EKF_LC_PROCESS_arg->reset_KF = (ResetType)1U;

        /* 'EKF_LC_PROCESS:94' EKF_LC_PROCESS_arg.state = uint8(1); */
        EKF_LC_PROCESS_arg->state = 1;

        /* 'EKF_LC_PROCESS:95' reset_frame_origin = uint8(1); */
        reset_frame_origin = 1;

        /* reset EKF_LC origin */
    }

    /* 'EKF_LC_PROCESS:98' if isempty(deming_window_count)|| EKF_LC_PROCESS_arg.reset_all
     */
    if ((!deming_window_count_not_empty) || (EKF_LC_PROCESS_arg->reset_all != 0U))
    {
        /* 'EKF_LC_PROCESS:99' deming_window_count = uint16(0); */
        deming_window_count           = 0;
        deming_window_count_not_empty = true;

        /* 'EKF_LC_PROCESS:100' window_count = uint8(0); */
        window_count = 0;
    }

    /* 'EKF_LC_PROCESS:102' if (lon_prev ~=EKF_LC_PROCESS_arg.GPS_Longitude ||
     * lat_prev~=EKF_LC_PROCESS_arg.GPS_Latitude) && GPS_OK */
    if (((lon_prev != EKF_LC_PROCESS_arg->GPS_Longitude) ||
         (lat_prev != EKF_LC_PROCESS_arg->GPS_Latitude)) &&
        (GPS_OK != 0))
    {
        /*      measurement_valid = bitand(measurement_valid,uint8(1)); */
        /* 'EKF_LC_PROCESS:104' collect_start = uint8(1); */
        collect_start = 1;

        /* 'EKF_LC_PROCESS:105' window_count = uint8(0); */
        window_count = 0;
    }
    else
    {
        /* 'EKF_LC_PROCESS:106' else */
        /* 'EKF_LC_PROCESS:107' collect_start = uint8(0); */
        collect_start = 0;

        /*      measurement_valid = uint8(0); */
    }

    /* 'EKF_LC_PROCESS:111' if GPS_OK */
    if (GPS_OK != 0)
    {
        /* 'EKF_LC_PROCESS:112' lon_prev = EKF_LC_PROCESS_arg.GPS_Longitude; */
        lon_prev = EKF_LC_PROCESS_arg->GPS_Longitude;

        /* 'EKF_LC_PROCESS:113' lat_prev = EKF_LC_PROCESS_arg.GPS_Latitude; */
        lat_prev = EKF_LC_PROCESS_arg->GPS_Latitude;
    }

    /* 'EKF_LC_PROCESS:116' GPS_signal_received_once0 = GPS_signal_received_once; */
    /* 'EKF_LC_PROCESS:118' if ((abs(location(1)) > MAX_CAL_XY ... */
    /* 'EKF_LC_PROCESS:119'         || abs(location(2)) > MAX_CAL_XY ) ... */
    /* 'EKF_LC_PROCESS:120'         ||
     * location(4)>MAX_CAL_RANGE)&&(EKF_LC_PROCESS_arg.SpeedForward>pk_max_speed)&&(EKF_LC_PROCESS_arg.slotnumber==0)
     */
    if (((fabsf(location[0]) > 20000.0F) || (fabsf(location[1]) > 20000.0F) ||
         (location[3] > 80000.0F)) &&
        (EKF_LC_PROCESS_arg->SpeedForward > 10.0F) &&
        (EKF_LC_PROCESS_arg->slotnumber == 0))
    {
        /* && (EKF_LC_PROCESS_arg.GPS_Accuracy <GPS_accuracy_max) %added 2018-5-12 */
        /* 'EKF_LC_PROCESS:122' reset_frame_origin = uint8(1); */
        reset_frame_origin = 1;

        /* reset EKF_LC origin */
    }

    /* 'EKF_LC_PROCESS:125' if reset_frame_origin == 1 */
    if (reset_frame_origin == 1)
    {
        /* todo 需要优化完全没有GPS的情况 */
        /* 'EKF_LC_PROCESS:126' location(4) = single(0); */
        location[3] = 0.0F;

        /* added 2018-5-12 */
        /* 'EKF_LC_PROCESS:127' if b_enu_set == 1 && GPS_OK */
        if ((b_enu_set == 1) && (GPS_OK != 0))
        {
            /* 'EKF_LC_PROCESS:128' lon0 = EKF_LC_PROCESS_arg.GPS_Longitude; */
            lon0 = EKF_LC_PROCESS_arg->GPS_Longitude;

            /* Set_llh_origin */
            /* 'EKF_LC_PROCESS:129' lat0 = EKF_LC_PROCESS_arg.GPS_Latitude; */
            lat0 = EKF_LC_PROCESS_arg->GPS_Latitude;

            /* 'EKF_LC_PROCESS:130' h0 = EKF_LC_PROCESS_arg.GPS_Altitude; */
            h0 = EKF_LC_PROCESS_arg->GPS_Altitude;
        }
        else
        {
            if ((b_enu_set == 1) && (GPS_OK == 0))
            {
                /* 'EKF_LC_PROCESS:131' elseif b_enu_set == 1 && GPS_OK == 0 */
                /* 'EKF_LC_PROCESS:132' b_enu_set = uint8(0); */
                b_enu_set = 0;

                /*      else */
            }
        }

        /* 'EKF_LC_PROCESS:135' z = [0;0]; */
        /* 'EKF_LC_PROCESS:136' deming_window_count = uint16(0); */
        deming_window_count = 0;

        /* if LinearDemingRegression is running, deming_window_count must reset when
         * reset_frame_origin */
        /* 'EKF_LC_PROCESS:138' EKF_LC_PROCESS_arg.state = uint8(3); */
        EKF_LC_PROCESS_arg->state = 3;

        /*   */
    }

    /* 'EKF_LC_PROCESS:141' if GPS_OK && b_enu_set == 0 */
    if ((GPS_OK != 0) && (b_enu_set == 0))
    {
        /* GPS_accuracy_OK */
        /*      GPS_signal_received_once = uint8(1); */
        /*      if GPS_signal_received_once0 == 0 %Set_llh_origin */
        /* 'EKF_LC_PROCESS:144' lon0 = EKF_LC_PROCESS_arg.GPS_Longitude; */
        lon0 = EKF_LC_PROCESS_arg->GPS_Longitude;

        /* 'EKF_LC_PROCESS:145' lat0 = EKF_LC_PROCESS_arg.GPS_Latitude; */
        lat0 = EKF_LC_PROCESS_arg->GPS_Latitude;

        /* 'EKF_LC_PROCESS:146' h0 = EKF_LC_PROCESS_arg.GPS_Altitude; */
        h0 = EKF_LC_PROCESS_arg->GPS_Altitude;

        /* 'EKF_LC_PROCESS:147' b_enu_set = uint8(1); */
        b_enu_set = 1;

        /* 'EKF_LC_PROCESS:148' heading_estimated = uint8(0); */
        heading_estimated = 0;

        /*          reset_frame_origin = uint8(1);%reset EKF_LC origin */
        /*      end */
    }

    /* 'EKF_LC_PROCESS:153' if b_enu_set == 1 && GPS_OK */
    if ((b_enu_set == 1) && (GPS_OK != 0))
    {
        /* /$run dllh2enu$/ */
        /* 'EKF_LC_PROCESS:154'
         * [de,dn,du]=dllh2enu(lon0,lat0,h0,EKF_LC_PROCESS_arg.GPS_Longitude,EKF_LC_PROCESS_arg.GPS_Latitude,EKF_LC_PROCESS_arg.GPS_Altitude);
         */
        dllh2enu(lon0, lat0, h0, EKF_LC_PROCESS_arg->GPS_Longitude,
                 EKF_LC_PROCESS_arg->GPS_Latitude, EKF_LC_PROCESS_arg->GPS_Altitude, &q,
                 &dn, &du);

        /* 'EKF_LC_PROCESS:155' z = [de;dn]; */
        z[0] = q;
        z[1] = dn;
    }
    else
    {
        /* 'EKF_LC_PROCESS:156' else */
        /* 'EKF_LC_PROCESS:157' measurement_valid = uint8(0); */
        measurement_valid = 0;

        /* not use measurement */
        /*      EKF_LC_PROCESS_arg.state = uint8(2);% */
    }

    /* 'EKF_LC_PROCESS:161' if EKF_LC_PROCESS_arg.SpeedForward <= pk_max_speed ||
     * GPS_OK==0 */
    if ((EKF_LC_PROCESS_arg->SpeedForward <= 10.0F) || (GPS_OK == 0))
    {
        /* EKF_LC_PROCESS_arg.GPS_Accuracy min is 1, when speed is 0, do not use gps
         * update */
        /* 'EKF_LC_PROCESS:162' measurement_valid = uint8(0); */
        measurement_valid = 0;

        /*      EKF_LC_PROCESS_arg.state = uint8(2); */
    }

    /*  %speed not too low */
    /* 'EKF_LC_PROCESS:166' if heading_estimated == 0 ... */
    /* 'EKF_LC_PROCESS:167'         && (EKF_LC_PROCESS_arg.SpeedForward)>pk_max_speed ...
     * %speed not too low */
    /* 'EKF_LC_PROCESS:168'         && GPS_OK ... */
    /* 'EKF_LC_PROCESS:169'         &&collect_start==1 */
    if ((heading_estimated == 0) && (EKF_LC_PROCESS_arg->SpeedForward > 10.0F) &&
        (GPS_OK != 0) && (collect_start == 1))
    {
        /* && abs(time-floor(time)) < EKF_LC_PROCESS_arg.dt/2 %when t is integer%before
         * ekf runs */
        /* 'EKF_LC_PROCESS:170' EKF_LC_PROCESS_arg.state = uint8(4); */
        EKF_LC_PROCESS_arg->state = 4;

        /*  */
        /* 'EKF_LC_PROCESS:171' if deming_window_count <deming_window_size */
        if (deming_window_count < 30)
        {
            /* 'EKF_LC_PROCESS:172' deming_window_count = deming_window_count+1; */
            deming_window_count++;
        }

        /* 'EKF_LC_PROCESS:174' buffer_z(:,1:end-1) = buffer_z(:,2:end); */
        for (i = 0; i < 29; i++)
        {
            for (i6 = 0; i6 < 2; i6++)
            {
                fv8[i6 + (i << 1)]      = buffer_z[i6 + ((1 + i) << 1)];
                buffer_z[i6 + (i << 1)] = fv8[i6 + (i << 1)];
            }
        }

        /* 'EKF_LC_PROCESS:175' buffer_z(:,end) = z; */
        for (i = 0; i < 2; i++)
        {
            buffer_z[58 + i] = z[i];
        }

        /* 'EKF_LC_PROCESS:177' if deming_window_count == deming_window_size */
        if (deming_window_count == 30)
        {
            /* 'EKF_LC_PROCESS:178' [~,~,psi0,deming_cost] ... */
            /* 'EKF_LC_PROCESS:179'
             * =LinearDemingRegression(buffer_z(1,:),buffer_z(2,:),single(deming_window_size));
             */
            for (i = 0; i < 30; i++)
            {
                fv9[i]  = buffer_z[i << 1];
                fv10[i] = buffer_z[1 + (i << 1)];
            }

            LinearDemingRegression(fv9, fv10, 30.0F, &q, &dn, &psi0, &du);

            /* 'EKF_LC_PROCESS:178' ~ */
            /* 'EKF_LC_PROCESS:178' ~ */
            /* 'EKF_LC_PROCESS:180' if deming_cost < 1 &&
             * deming_cost>0.001&&EKF_LC_PROCESS_arg.EPS_angle<20 */
            if ((du < 1.0F) && (du > 0.001F) && (EKF_LC_PROCESS_arg->EPS_angle < 20.0F))
            {
                /* degree!!!!!%    % && measurement_valid == 1 */
                /*              run_ekf = uint8(1); */
                /* 'EKF_LC_PROCESS:182' EKF_LC_PROCESS_arg.state = uint8(5); */
                EKF_LC_PROCESS_arg->state = 5;

                /*  */
                /* 'EKF_LC_PROCESS:183' heading_estimated = uint8(1); */
                heading_estimated = 1;

                /* 'EKF_LC_PROCESS:184' set_origin_heading = uint8(1); */
                set_origin_heading = 1;

                /* 'EKF_LC_PROCESS:185' location(4) = single(0); */
                location[3] = 0.0F;

                /* added 2018-5-12 */
                /* 'EKF_LC_PROCESS:186' set_origin_heading_once = uint8(1); */
                set_origin_heading_once = 1;
            }
        }
    }

    /* if window_count == max_window_count, it is 1 second */
    /* 'EKF_LC_PROCESS:191' max_window_count = uint8(1/EKF_LC_PROCESS_arg.dt); */
    /* for  1 second!!!!!to do check if ok */
    /* 'EKF_LC_PROCESS:192' window_count = window_count + 1; */
    window_count++;

    /* 'EKF_LC_PROCESS:193' if window_count == max_window_count */
    if (window_count == (unsigned char)(1.0F / EKF_LC_PROCESS_arg->dt))
    {
        /* 'EKF_LC_PROCESS:194' window_count = uint8(0); */
        window_count = 0;
    }

    /* 'EKF_LC_PROCESS:196' if heading_estimated == 0 */
    if (heading_estimated == 0)
    {
        /* 'EKF_LC_PROCESS:197' measurement_valid = uint8(0); */
        measurement_valid = 0;
    }

    /* 'EKF_LC_PROCESS:200' EKF_LC_PROCESS_arg.X = single([0 0 0 0]'); */
    /* 'EKF_LC_PROCESS:201' EKF_LC_PROCESS_arg.Pdiag = single([0 0 0 0]'); */
    for (i = 0; i < 4; i++)
    {
        EKF_LC_PROCESS_arg->X[i]     = 0.0F;
        EKF_LC_PROCESS_arg->Pdiag[i] = 0.0F;
    }

    /* 'EKF_LC_PROCESS:203' q = EKF_LC_PROCESS_arg.q; */
    /*      r = [ 4 4]*2000*1000* EKF_LC_PROCESS_arg.GPS_Accuracy^5; */
    /*      r = single([ 4 4]*2000*100.0*
     * single(EKF_LC_PROCESS_arg.GPS_Accuracy)^5/(abs(v_Skf)+1)^2)*100; */
    /*      r = r * 0.00000001;%good */
    /*      r = r*0.00000001;%UKF */
    /*  r = r*0.00000001;%UKF */
    /* 'EKF_LC_PROCESS:211' moving_avg_pitch = 0.99*moving_avg_pitch +
     * 0.01*EKF_LC_PROCESS_arg.pitch; */
    moving_avg_pitch = 0.99F * moving_avg_pitch + 0.01F * EKF_LC_PROCESS_arg->pitch;

    /* 'EKF_LC_PROCESS:212' moving_avg_roll = 0.99*moving_avg_roll +
     * 0.01*EKF_LC_PROCESS_arg.roll; */
    moving_avg_roll = 0.99F * moving_avg_roll + 0.01F * EKF_LC_PROCESS_arg->roll;

    /* 'EKF_LC_PROCESS:213' r = single(EKF_LC_PROCESS_arg.r*2*100.0*
     * single(EKF_LC_PROCESS_arg.GPS_Accuracy)^5*100*(1+moving_avg_pitch^5)); */
    dn = powf((float)EKF_LC_PROCESS_arg->GPS_Accuracy, 5.0F);
    q  = powf(moving_avg_pitch, 5.0F);
    for (i = 0; i < 2; i++)
    {
        r[i] = EKF_LC_PROCESS_arg->r[i] * 2.0F * 100.0F * dn * 100.0F * (1.0F + q);
    }

    /* 'EKF_LC_PROCESS:215' if  abs(EKF_LC_PROCESS_arg.SpeedForward) < 10 */
    if (fabsf(EKF_LC_PROCESS_arg->SpeedForward) < 10.0F)
    {
        /* 'EKF_LC_PROCESS:216' EKF_LC_PROCESS_arg.SpeedForward =
         * EKF_LC_PROCESS_arg.dS_wheel / EKF_LC_PROCESS_arg.dt; */
        EKF_LC_PROCESS_arg->SpeedForward =
            EKF_LC_PROCESS_arg->dS_wheel / EKF_LC_PROCESS_arg->dt;
    }

    /* 'EKF_LC_PROCESS:219' switch EKF_LC_PROCESS_arg.KF_selection */
    switch (EKF_LC_PROCESS_arg->KF_selection)
    {
        case 1:
            /* 'EKF_LC_PROCESS:220' case 1 */
            /* 'EKF_LC_PROCESS:221' r = r*0.1; */
            /*          r = r * 0.001;%to be tuned */
            /*      [EKF_LC_PROCESS_arg.EKF_LC_PROCESS_arg.EKF_LC_PROCESS_arg.X,EKF_LC_PROCESS_arg.Pdiag]=EKF_LC0(z,EKF_LC_PROCESS_arg.IMU_Data.AngRate_Z_rad,EKF_LC_PROCESS_arg.LinearMotionData.SpeedForward,EKF_LC_PROCESS_arg.dt,q,r,measurement_valid,reset,set_origin_heading,psi0);%GPS
             * update rate 1Hz. The computation rate is 50Hz. */
            /* 'EKF_LC_PROCESS:224'
             * [EKF_LC_PROCESS_arg.X,EKF_LC_PROCESS_arg.Pdiag]=EKF_LC(z,EKF_LC_PROCESS_arg.AngRate_Z_rad,EKF_LC_PROCESS_arg.SpeedForward,EKF_LC_PROCESS_arg.dS_wheel,EKF_LC_PROCESS_arg.dt,q,r,measurement_valid,EKF_LC_PROCESS_arg.reset_KF,set_origin_heading,psi0,reset_frame_origin,
             * EKF_LC_PROCESS_arg.car_moving); */
            for (i = 0; i < 2; i++)
            {
                b_z[i]                  = z[i];
                b_EKF_LC_PROCESS_arg[i] = EKF_LC_PROCESS_arg->q[i];
                r[i] *= 0.1F;
            }

            EKF_LC(b_z, EKF_LC_PROCESS_arg->AngRate_Z_rad,
                   EKF_LC_PROCESS_arg->SpeedForward, EKF_LC_PROCESS_arg->dt,
                   b_EKF_LC_PROCESS_arg, r, measurement_valid,
                   EKF_LC_PROCESS_arg->reset_KF, set_origin_heading, psi0,
                   reset_frame_origin, EKF_LC_PROCESS_arg->car_moving,
                   EKF_LC_PROCESS_arg->X, EKF_LC_PROCESS_arg->Pdiag);

            /* GPS update rate 1Hz. The computation rate is 50Hz. */
            /*  [EKF_LC_PROCESS_arg.X,EKF_LC_PROCESS_arg.Pdiag]=EKF_LC_OLD(z,EKF_LC_PROCESS_arg.AngRate_Z_rad,EKF_LC_PROCESS_arg.SpeedForward,EKF_LC_PROCESS_arg.dt,q,r,measurement_valid,reset,set_origin_heading,psi0);%GPS
             * update rate 1Hz. The computation rate is 50Hz. */
            /*      [EKF_LC_PROCESS_arg.EKF_LC_PROCESS_arg.EKF_LC_PROCESS_arg.X,EKF_LC_PROCESS_arg.Pdiag]=ErEKF_LC(z,EKF_LC_PROCESS_arg.IMU_Data.AngRate_Z_rad,EKF_LC_PROCESS_arg.LinearMotionData.SpeedForward,EKF_LC_PROCESS_arg.LinearMotionData.dS_wheel,EKF_LC_PROCESS_arg.dt,[0.1
             * 0.001
             * 0.0001],r,measurement_valid,EKF_LC_PROCESS_arg.reset_KF,set_origin_heading,psi0,reset_frame_origin);%GPS
             * update rate 1Hz. The computation rate is 50Hz. */
            break;

        case 2:
            /* 'EKF_LC_PROCESS:227' case 2 */
            /* 'EKF_LC_PROCESS:228' r = r * 1; */
            /* to be tuned */
            /*          r = single([ 1 1]*
             * single(EKF_LC_PROCESS_arg.GPS_Data.GPS_Accuracy)^5*10); */
            /* 'EKF_LC_PROCESS:230'
             * [EKF_LC_PROCESS_arg.X,EKF_LC_PROCESS_arg.Pdiag]=SUKF(z,EKF_LC_PROCESS_arg.AngRate_Z_rad,EKF_LC_PROCESS_arg.SpeedForward,EKF_LC_PROCESS_arg.dS_wheel,EKF_LC_PROCESS_arg.dt,q,r,measurement_valid,EKF_LC_PROCESS_arg.reset_KF,set_origin_heading,psi0,reset_frame_origin,
             * EKF_LC_PROCESS_arg.car_moving); */
            for (i = 0; i < 2; i++)
            {
                b_z[i]                  = z[i];
                b_EKF_LC_PROCESS_arg[i] = EKF_LC_PROCESS_arg->q[i];
            }

            SUKF(b_z, EKF_LC_PROCESS_arg->AngRate_Z_rad, EKF_LC_PROCESS_arg->SpeedForward,
                 EKF_LC_PROCESS_arg->dt, b_EKF_LC_PROCESS_arg, r, measurement_valid,
                 EKF_LC_PROCESS_arg->reset_KF, set_origin_heading, psi0,
                 reset_frame_origin, EKF_LC_PROCESS_arg->car_moving,
                 EKF_LC_PROCESS_arg->X, EKF_LC_PROCESS_arg->Pdiag);
            break;
    }

    /* 'EKF_LC_PROCESS:232' if sum(EKF_LC_PROCESS_arg.Pdiag)>1e10 */
    if (sum(EKF_LC_PROCESS_arg->Pdiag) > 1.0E+10F)
    {
        /* diverge happens */
        /* 'EKF_LC_PROCESS:233' reset_all_diverge = uint8(1); */
        reset_all_diverge = 1;

        /* 'EKF_LC_PROCESS:234' EKF_LC_PROCESS_arg.state = uint8(6); */
        EKF_LC_PROCESS_arg->state = 6;
    }
    else
    {
        /* 'EKF_LC_PROCESS:235' else */
        /* 'EKF_LC_PROCESS:236' reset_all_diverge = uint8(0); */
        reset_all_diverge = 0;
    }

    /* 'EKF_LC_PROCESS:238' MAX_SPEED = uint8(50); */
    /* m/s */
    /*  && measurement_valid == 1 ... */
    /*  %slope smaller than 2 degrees */
    /* 'EKF_LC_PROCESS:239' if reset_all_diverge == 0 && reset_frame_origin == 0 ... */
    /* 'EKF_LC_PROCESS:240'         &&EKF_LC_PROCESS_arg.reset_all == 0 ... */
    /* 'EKF_LC_PROCESS:241'         &&EKF_LC_PROCESS_arg.reset_KF == 0 ... */
    /* 'EKF_LC_PROCESS:242'         && set_origin_heading == 0 ... */
    /* 'EKF_LC_PROCESS:243'         ... && measurement_valid == 1 ... */
    /* 'EKF_LC_PROCESS:244'         && GPS_OK ... */
    /* 'EKF_LC_PROCESS:245'         && EKF_LC_PROCESS_arg.SpeedForward > pk_max_speed ...
     */
    /* 'EKF_LC_PROCESS:246'         && abs(moving_avg_pitch) < 3 ... %slope smaller than 2
     * degrees */
    /* 'EKF_LC_PROCESS:247'         && norm(EKF_LC_PROCESS_arg.X(1:2) -
     * location(1:2))<single(MAX_SPEED)*EKF_LC_PROCESS_arg.dt */
    if ((reset_all_diverge == 0) && (reset_frame_origin == 0) &&
        (EKF_LC_PROCESS_arg->reset_all == 0U) && (EKF_LC_PROCESS_arg->reset_KF == 0U) &&
        (set_origin_heading == 0) && (GPS_OK != 0) &&
        (EKF_LC_PROCESS_arg->SpeedForward > 10.0F) && (fabsf(moving_avg_pitch) < 3.0F))
    {
        for (i = 0; i < 2; i++)
        {
            b_EKF_LC_PROCESS_arg[i] = EKF_LC_PROCESS_arg->X[i] - location[i];
        }

        if (norm(b_EKF_LC_PROCESS_arg) < 50.0F * EKF_LC_PROCESS_arg->dt)
        {
            /* Distance travelled between two loops must be smaller than max possible
             * speed * EKF_LC_PROCESS_arg.dt */
            /* 'EKF_LC_PROCESS:248' EPROM_EKF_distance = EPROM_EKF_distance +
             * norm(EKF_LC_PROCESS_arg.X(1:2) - location(1:2)); */
            for (i = 0; i < 2; i++)
            {
                b_EKF_LC_PROCESS_arg[i] = EKF_LC_PROCESS_arg->X[i] - location[i];
            }

            EPROM_EKF_distance += norm(b_EKF_LC_PROCESS_arg);

            /* 'EKF_LC_PROCESS:249' EPROM_odometer_distance = EPROM_odometer_distance +
             * EKF_LC_PROCESS_arg.dS_wheel; */
            EPROM_odometer_distance += EKF_LC_PROCESS_arg->dS_wheel;

            /* 'EKF_LC_PROCESS:250' EPROM_motor_distance = EPROM_motor_distance +
             * (EKF_LC_PROCESS_arg.v_motor) * EKF_LC_PROCESS_arg.dt; */
            EPROM_motor_distance += EKF_LC_PROCESS_arg->v_motor * EKF_LC_PROCESS_arg->dt;

            /* 'EKF_LC_PROCESS:251' EKF_LC_PROCESS_arg.state = uint8(7); */
            EKF_LC_PROCESS_arg->state = 7;
        }
    }

    /* 'EKF_LC_PROCESS:255' if EPROM_odometer_distance >=
     * window_odometer_distance4scale_factor */
    if (EPROM_odometer_distance >= 5000.0F)
    {
        /*      rear_wheel_scale_factor = EPROM_EKF_distance / EPROM_odometer_distance; */
        /*      motor_scale_factor = EPROM_EKF_distance / EPROM_motor_distance; */
        /* 'EKF_LC_PROCESS:258' Rear_wheel_fac_ds_max     = single(0.010892); */
        /* 'EKF_LC_PROCESS:259' Motorspd_fac_vspd_max     = single(0.00427); */
        /* 'EKF_LC_PROCESS:260' Rear_wheel_fac_ds_min     = single(0.010465); */
        /* 'EKF_LC_PROCESS:261' Motorspd_fac_vspd_min     = single(0.004103); */
        /* 'EKF_LC_PROCESS:262' Motorspd_fac_vspd =  Motorspd_fac_vspd *
         * EPROM_EKF_distance / EPROM_motor_distance; */

        /* 'EKF_LC_PROCESS:263' Rear_wheel_fac_ds = Rear_wheel_fac_ds * EPROM_EKF_distance
         * / EPROM_odometer_distance; */

        /* 'EKF_LC_PROCESS:264' Motorspd_fac_vspd = limit(Motorspd_fac_vspd,
         * Motorspd_fac_vspd_max, Motorspd_fac_vspd_min); */
        /*   */
        /* 'EKF_LC_PROCESS:321' coder.inline('always'); */
        /* 'EKF_LC_PROCESS:322' if value> upper */

        /* 'EKF_LC_PROCESS:331' y = value; */
        /* 'EKF_LC_PROCESS:265' Rear_wheel_fac_ds = limit(Rear_wheel_fac_ds,
         * Rear_wheel_fac_ds_max, Rear_wheel_fac_ds_min); */
        /*   */
        /* 'EKF_LC_PROCESS:321' coder.inline('always'); */
        /* 'EKF_LC_PROCESS:322' if value> upper */

        /* 'EKF_LC_PROCESS:331' y = value; */
        /* 'EKF_LC_PROCESS:266' EPROM_odometer_distance = 0; */
        EPROM_odometer_distance = 0.0F;

        /* 'EKF_LC_PROCESS:267' EPROM_EKF_distance = 0; */
        EPROM_EKF_distance = 0.0F;

        /* 'EKF_LC_PROCESS:268' EPROM_motor_distance = 0; */
        EPROM_motor_distance = 0.0F;

        /*      temp_factor = EPROM_odometer_distance - range_interval_4scale_factor; */
        /*      EPROM_odometer_distance = EPROM_odometer_distance*temp_factor; */
        /*      EPROM_EKF_distance = EPROM_EKF_distance*temp_factor; */
        /*      EPROM_motor_distance = EPROM_motor_distance*temp_factor; */
        /* to do in Location.c: Motorspd_fac_vspd =  Motorspd_fac_vspd *
         * EPROM_EKF_distance / EPROM_motor_distance and */
        /* Rear_wheel_fac_ds = Rear_wheel_fac_ds * EPROM_EKF_distance /
         * EPROM_odometer_distance */
    }

    /* 'EKF_LC_PROCESS:276' EKF_LC_PROCESS_arg.X(3) = Pi_toPi(EKF_LC_PROCESS_arg.X(3)); */
    /*  change angle to the range -pi to pi */
    /* 'EKF_LC_PROCESS:310' coder.inline('always'); */
    /* 'EKF_LC_PROCESS:311' phi = single(mod(phi,2 * pi)); */
    if (EKF_LC_PROCESS_arg->X[2] == 0.0F)
    {
        dn = 0.0F;
    }
    else
    {
        dn   = fmodf(EKF_LC_PROCESS_arg->X[2], 6.28318548F);
        rEQ0 = (dn == 0.0F) ? RESET : NOT_RESET;
        if (rEQ0 == RESET)
        {
            q    = fabsf(EKF_LC_PROCESS_arg->X[2] / 6.28318548F);
            rEQ0 = (fabsf(q - floorf(q + 0.5F)) <= 1.1920929E-7F * q) ? RESET : NOT_RESET;
        }

        if ((ValidType)rEQ0 == Valid)
        {
            dn = 0.0F;
        }
        else
        {
            if (EKF_LC_PROCESS_arg->X[2] < 0.0F)
            {
                dn += 6.28318548F;
            }
        }
    }

    /* 'EKF_LC_PROCESS:312' if phi > pi */
    if (dn > 3.14159274F)
    {
        /* 'EKF_LC_PROCESS:313' y = -2 * pi + phi; */
        dn += -6.28318548F;
    }
    else
    {
        /* 'EKF_LC_PROCESS:314' else */
        /* 'EKF_LC_PROCESS:315' y = phi; */
    }

    EKF_LC_PROCESS_arg->X[2] = dn;

    /* added 2018-5-12 */
    /* 'EKF_LC_PROCESS:277' if reset_frame_origin == 0 && set_origin_heading ==0 */
    if ((reset_frame_origin == 0) && (set_origin_heading == 0))
    {
        /* added 2018-5-12 */
        /* 'EKF_LC_PROCESS:278' location(4) = location(4) + norm(EKF_LC_PROCESS_arg.X(1:2)
         * - location(1:2)); */
        for (i = 0; i < 2; i++)
        {
            b_EKF_LC_PROCESS_arg[i] = EKF_LC_PROCESS_arg->X[i] - location[i];
        }

        location[3] += norm(b_EKF_LC_PROCESS_arg);
    }

    /* 'EKF_LC_PROCESS:280' location(1:3) = EKF_LC_PROCESS_arg.X(1:3); */
    for (i = 0; i < 3; i++)
    {
        location[i] = EKF_LC_PROCESS_arg->X[i];
    }

    /* added 2018-5-12 */
    /* 'EKF_LC_PROCESS:281' EKF_LC_PROCESS_arg.PK_locat = location; */
    for (i = 0; i < 4; i++)
    {
        EKF_LC_PROCESS_arg->PK_locat[i] = location[i];
    }

    /* 'EKF_LC_PROCESS:283' if ((EKF_LC_PROCESS_arg.SpeedForward) >
     * pk_max_speed)||(abs(location(1)-PK_xyS_offset(1))>=20000)||(abs(location(2)-PK_xyS_offset(2))>=20000)
     */
    if ((EKF_LC_PROCESS_arg->SpeedForward > 10.0F) ||
        (fabsf(location[0] - PK_xyS_offset[0]) >= 20000.0F) ||
        (fabsf(location[1] - PK_xyS_offset[1]) >= 20000.0F))
    {
        /* ||reset_PK_origin == uint8(1)%打滑的话整个模块重置 */
        /* 'EKF_LC_PROCESS:284' PK_xyS_offset = location([1,2,4]); */
        for (i = 0; i < 3; i++)
        {
            PK_xyS_offset[i] = location[iv4[i]];
        }
    }

    /*  EKF_LC_PROCESS_arg.PK_locat([1,2,4]) = EKF_LC_PROCESS_arg.PK_locat([1,2,4]) -
     * PK_xyS_offset; */
    /* 'EKF_LC_PROCESS:287' EKF_LC_PROCESS_arg.PK_locat([1,2]) =
     * EKF_LC_PROCESS_arg.PK_locat([1,2]) - PK_xyS_offset([1,2]); */
    for (i = 0; i < 2; i++)
    {
        EKF_LC_PROCESS_arg->PK_locat[i] = location[i] - PK_xyS_offset[i];
    }

    /*  to do: send info reset_frame_origin set_origin_heading
     * EKF_LC_PROCESS_arg.reset_all to */
    /*  RD/PK */
    /*  if difference between gps and ekf is too much, origin and heading of ekf/ukf
     * should */
    /*  be reset */
    /* 'EKF_LC_PROCESS:293' if reset_frame_origin == 1 */
    if (reset_frame_origin == 1)
    {
        /* 'EKF_LC_PROCESS:294' reset_frame_origin = uint8(0); */
        reset_frame_origin = 0;
    }

    /* 'EKF_LC_PROCESS:296' if EKF_LC_PROCESS_arg.GPS_Accuracy<=2 &&
     * norm(z-EKF_LC_PROCESS_arg.X(1:2)) > GPS_EKF_MAX_DIFF && heading_estimated == 1 &&
     * set_origin_heading_once == 1 && EKF_LC_PROCESS_arg.SpeedForward > pk_max_speed &&
     * (EKF_LC_PROCESS_arg.slotnumber==0) */
    if (EKF_LC_PROCESS_arg->GPS_Accuracy <= 2U)
    {
        for (i = 0; i < 2; i++)
        {
            b_z[i] = z[i] - EKF_LC_PROCESS_arg->X[i];
        }

        if ((norm(b_z) > 200.0F) && (heading_estimated == 1) &&
            (set_origin_heading_once == 1) &&
            (EKF_LC_PROCESS_arg->SpeedForward > 10.0F) &&
            (EKF_LC_PROCESS_arg->slotnumber == 0))
        {
            /* 'EKF_LC_PROCESS:297' reset_origin_heading_process_restart = uint8(1); */
            c_reset_origin_heading_process_ = 1;

            /* 'EKF_LC_PROCESS:298' EKF_LC_PROCESS_arg.state = uint8(8); */
            EKF_LC_PROCESS_arg->state = 8;
        }
    }

    /* 'EKF_LC_PROCESS:300' if reset_origin_heading_process_restart ==uint8(1) */
    if (c_reset_origin_heading_process_ == 1)
    {
        /* 'EKF_LC_PROCESS:301' heading_estimated = uint8(0); */
        heading_estimated = 0;

        /* 'EKF_LC_PROCESS:302' set_origin_heading_once = uint8(0); */
        set_origin_heading_once = 0;

        /* 'EKF_LC_PROCESS:303' deming_window_count = uint16(0); */
        deming_window_count = 0;

        /* 'EKF_LC_PROCESS:304' buffer_z = single(zeros(2,deming_window_size)); */
        memset(&buffer_z[0], 0, 60U * sizeof(float));
    }
}

/*
 * function EKF_LC_PROCESS_arg = EKF_LC_PROCESS(EKF_LC_PROCESS_arg)
 * Arguments    : const EKF_LC_PROCESS_arg_Type *EKF_LC_PROCESS_arg
 *                EKF_LC_PROCESS_arg_Type *b_EKF_LC_PROCESS_arg
 * Return Type  : void
 */
void EKF_LC_PROCESS(const EKF_LC_PROCESS_arg_Type *EKF_LC_PROCESS_arg,
                    EKF_LC_PROCESS_arg_Type *b_EKF_LC_PROCESS_arg)
{
    *b_EKF_LC_PROCESS_arg = *EKF_LC_PROCESS_arg;
    b_EKF_LC_PROCESS(b_EKF_LC_PROCESS_arg);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void EKF_LC_PROCESS_init(void)
{
    /* to to remove in c */
    /* 'EKF_LC_PROCESS:62' Rear_wheel_fac_ds =   single(0.01067861); */
    // Rear_wheel_fac_ds = 0.0106786098F;

    /* 'EKF_LC_PROCESS:63' Motorspd_fac_vspd = single(0.004186883); */
    // Motorspd_fac_vspd = 0.0041868831F;

    /* 'EKF_LC_PROCESS:66' reset_all_diverge = uint8(0); */
    reset_all_diverge = 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void buffer_z_not_empty_init(void) { buffer_z_not_empty = false; }

/*
 * Arguments    : void
 * Return Type  : void
 */
void c_deming_window_count_not_empty(void) { deming_window_count_not_empty = false; }

/*
 * File trailer for EKF_LC_PROCESS.c
 *
 * [EOF]
 */
