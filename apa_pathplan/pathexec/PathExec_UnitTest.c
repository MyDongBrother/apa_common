#include "PK_PathExecute.h"
#include "PK_PathExecuteP.h"
#include "PathExec_Debug.h"
#include "PathExec_Func.h"
#include "PathExec_Obj.h"
#include "PK_Utility.h"

#ifdef SIMULATE
#define UINT_TEST
#endif

#ifdef UINT_TEST
#include "json/json.h"
#include <fstream>
extern bool PathExec_UpdateTarget(PK_Cur_AutodrPara curSt[1], float enviObj[5][4],
                                  float newTarget[3], bool update, int flag = 0,
                                  const float swell = 0.02);
extern void Calc_Target(const int slottype, const uint8_t confirence,
                        const SlotObj_T &slotobj, const Avm_Pot_T &avmPot,
                        VehPos_T &targpos);

void PathExec_WaitClearObjs_Test()
{
#if 0
    float curpos[4] = {10.287700, -2.561394, 1.532209, 23.824194};
    float finpos[3] = {10.194578, -5.960462, 1.548497};
    float rx_cur_to_fin = 3.400299;
    float ry_cur_to_fin = -0.017309;
    float rtheta_cur_to_fin = -0.016288;
    uint8_t B_Danger_filter = 02;
    uint16_t dangerType = 128;
#endif

    float curpos[4]         = {7.764730, -1.577718, 0.807809, 15.704644};
    float finpos[3]         = {7.681460, -6.324198, 1.547812};
    float rx_cur_to_fin     = 4.747139;
    float ry_cur_to_fin     = 0.025838;
    float rtheta_cur_to_fin = -0.740003;
    uint8_t B_Danger_filter = 02;
    uint16_t dangerType     = 64;

    PK_Cur_AutodrPara curSt;
    memcpy(curSt.curpos, curpos, sizeof(curpos));
    memcpy(curSt.finpos, finpos, sizeof(finpos));
    curSt.rx_cur_to_fin     = rx_cur_to_fin;
    curSt.ry_cur_to_fin     = ry_cur_to_fin;
    curSt.rtheta_cur_to_fin = rtheta_cur_to_fin;
    curSt.B_Danger_filter   = B_Danger_filter;
    RTE_PK_ObjAvoid_Set_DangerSt(dangerType);
}

void IsVehInSlot_Test()
{
    float target[3] = {6.934818, -6.150357, 1.593064};
    float curPos[4] = {6.996563, -1.390605, 1.314276, 21.405325};
    float objs[12]  = {3.470569, -2.164547, 5.385095, -2.121907, 5.516198,  -8.008405,
                       9.020115, -7.930367, 8.901367, -2.598558, 10.815891, -2.555919};
    PK_SlotShapeType slotShap = PK_SLOT_RIGHT_VERT;

    SlotObj_T slotObj;
    memcpy(&slotObj, objs, sizeof(objs));

    RTE_PK_SlotDetect_Set_TargPos(target);
    RTE_PK_SlotDetect_Set_SlotShape(slotShap);
    RTE_PK_SlotDetect_Set_SlotObj(&slotObj);

    extern bool IsVehInSlot(const float curPos[4]);
    IsVehInSlot(curPos);
}

void PathExec_B_MoveOut_VertRePlan_UnitTest()
{
    float plannedPath[20][TRAJITEM_LEN];
#if 0
    float finpoint[3] = {-0.165465,2.782911,-0.041379};
    //float curpos[4] = {-0.249453, 2.688247, -0.495758, 12.831689};
    float curpos[4] = {-0.395032, 2.780207, -0.589606, 11.968675};
    float slotpoints[12] = {-4.119875,2.298859,-1.623014,2.195481,-1.531810,4.398316,5.376274,4.112300,5.285069,1.909466,7.781932,1.806087};
    int slotshap = 1;

    float finpoint[3] = {-0.381727, -2.928566, -0.023007};
    float curpos[4] = {-0.390725, -2.844639, 0.253866, 13.513005};
    float slotpoints[12] = {-4.644342,-2.080279,-2.146001,-2.137769,-2.196857,-4.347807,6.764771,-4.554029,6.815627,-2.343990,9.313964,-2.401481};
    int slotshap = 2;

    float finpoint[3] = {9.534602, 7.011106, -1.586934};
    float curpos[4] = {9.413082, 1.463448, -1.573817, 22.200359};
    float slotpoints[12] = {4.723480,4.656436,6.638229,4.625534,6.704358,8.723000,13.018535,8.621097,12.952407,4.523630,14.867158,4.492728};
    float avmSlot[8] = {8.271241,2.838949,8.352867,7.896677,10.744300,7.858082,10.662674,2.80035};

    int slotshap = 3;
    int avmType = 1;
    uint16_t dangerType = 16;

    float finpoint[3] = {7.728129, 6.539309, -1.591593};
    float curpos[4] = {9.620872, 0.759569, -0.698516, 28.343971};
    float slotpoints[12] = {3.905872,4.186284,5.820458,4.146461,5.905675,8.243576,10.219741,8.153846,10.092824,2.051903,12.049110,4.016909};
    float avmSlot[8] = {6.451038,2.633021,6.557051,7.729918,8.947701,7.680195,8.841688,2.58329};
    int slotshap = 3;
    int avmType = 1;
    uint16_t dangerType = 16;


    float finpoint[3] = {24.152531, -3.418921, -0.842251};
    float curpos[4] = {21.926439, -1.067610, -0.617597, 46.751789};
    float slotpoints[12] = {21.626652,-6.254420,23.055517,-4.979442,25.783905,-8.037142,28.860882,-5.291552,26.132496,-2.233855,27.561363,-0.958878};
    float avmSlot[8] = {22.511179,-3.454821,25.905241,-7.258539,27.768490,-5.595961,24.374428,-1.79224};
    int slotshap = 4;
    int avmType = 1;
    uint16_t dangerType = 4;

    float finpoint[3] = {9.216811, -8.455169, 1.495426};
    float curpos[4] = {9.641022, -3.972313, 1.207590, 17.646488};
    float slotpoints[12] = {6.204809,-4.314790,8.114371,-4.458986,7.695122,-10.011004,11.541887,-10.301485,11.850462,-6.215119,13.760024,-6.359316};
    float avmSlot[8] = {8.365354,-4.447678,7.989011,-9.431489,10.284075,-9.604796,10.660418,-4.62098};
    int slotshap = 4;
    int avmType = 1;
    uint16_t dangerType = 16;

    float finpoint[3] = {13.152860, -5.907486, 1.480332};
    float curpos[4] = {14.463231, -0.376852, 0.687394, 30.172029};
    float slotpoints[12] = {9.615496,-3.144619,11.522665,-3.317621,11.152447,-7.398865,15.245390,-7.770144,15.794372,-1.718218,17.701544,-1.891222};
    float avmSlot[8] = {12.290848,-1.881146,11.839323,-6.858709,14.273788,-7.079545,14.725313,-2.10198};
    int slotshap = 4;
    int avmType = 1;
    uint16_t dangerType = 32;

    float finpoint[3] = {6.163183, -6.382302, 1.516881};
    float curpos[4] = {6.393963, -2.606401, 1.441324, 26.561029};
    float slotpoints[12] = {2.966348,-3.774238,4.878565,-3.877435,4.649105,-8.129248,7.229632,-8.268513,7.502513,-3.212116,9.702272,-3.330831};
    float avmSlot[8] = {5.302543,-2.658512,5.033206,-7.649250,7.150247,-7.763501,7.419585,-2.77276};
    int slotshap = 4;
    int avmType = 1;
    uint16_t dangerType = 32;

    float curpos[4] = {7.613707,-3.562164,1.470468, 16.136154};
    float finpoint[3] = {7.681303,-6.418919,1.615257};
    float slotpoints[12] = {3.795599,-3.127668,6.213862,-3.020081,6.448833,-8.301557,9.797947,-8.152556,9.608698,-3.898764,11.521807,-3.813650};
    float avmSlot[8] = {5.794526,-2.827228,6.021109,-7.920190,8.629085,-7.804163,8.402501,-2.71120};
    int slotshap = 4;
    int dangerType = 4112;
    int avmType = 1;

    float curpos[4] = {6.054049,2.112322,-1.374789,12.309343};
    float finpoint[3] = {6.171346,6.207977,-1.640743};
    float slotpoints[12] = {1.489933,4.098000,3.150862,3.981633,3.451247,8.269124,7.873761,7.959279,7.489507,2.474723,9.150436,2.358357};
    float avmSlot[8] = {4.594896,2.677522,4.951193,7.763056,7.596416,7.577729,7.240119,2.49219};
    int slotshap = 3;
    int dangerType = 4112;
    int avmType = 1;
#endif

    float curpos[4]      = {12.542224, 1.274379, -1.305762, 18.214281};
    float finpoint[3]    = {12.711355, 5.932324, -1.666011};
    float slotpoints[12] = {7.978128,  3.941297, 9.635588,  3.783005,
                            10.044201, 8.061537, 14.452893, 7.640491,
                            13.930194, 2.167395, 15.587652, 2.009103};
    float avmSlot[8]     = {11.050792, 2.442388, 11.535462, 7.517297,
                            14.165997, 7.266072, 13.681326, 2.19116};
    int slotshap         = 3;
    int dangerType       = 4112;
    int avmType          = 1;

    Avm_Pot_T avmPot;
    memcpy((float *)&avmPot.near_rear, &avmSlot[0], sizeof(avmPot.near_rear));
    memcpy((float *)&avmPot.far_rear, &avmSlot[2], sizeof(avmPot.far_rear));
    memcpy((float *)&avmPot.far_front, &avmSlot[4], sizeof(avmPot.far_front));
    memcpy((float *)&avmPot.near_front, &avmSlot[6], sizeof(avmPot.near_front));
    RTE_PK_DataConvt_Set_IsAVM_Slot(avmType);
    RTE_PK_DataConvt_Set_TargAVM_SlotInfo(&avmPot);
    RTE_PK_ObjAvoid_Set_DangerSt(dangerType);

    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));
    float slotPoints[5][4];
    SlotObj_Convert_float(slotObj, slotPoints);

    float newTarget[3];
    PK_Cur_AutodrPara curSt[1];
    curSt[0].slotshape = slotshap;
    memcpy(curSt[0].curpos, curpos, sizeof(curpos));
    memcpy(curSt[0].finpos, finpoint, sizeof(curpos));
    PathExec_UpdateTarget(curSt, slotPoints, newTarget, false);
    int trajNum =
        PathExec_B_MoveOutFront(plannedPath, newTarget, slotPoints, curpos, slotshap);
    for (int index = 0; index < trajNum; index++)
    {
        float endPos[3];
        PK_Get_Path_EndPos(plannedPath[index], endPos);
        printf("path %d: %2.4f %2.4f %2.4f %2.4f %2.4f end: %2.4f %2.4f %2.4f\n", index,
               plannedPath[index][0], plannedPath[index][1], plannedPath[index][2],
               plannedPath[index][3], plannedPath[index][4], endPos[0], endPos[1],
               endPos[2]);
    }
}

void PathExec_UpdateTarget_UnitTest()
{

#if 0
    float curpos[4] = {14.041228, 6.179450, -1.601045, 30.580935};
    float finpoint[3] = {14.049195, 6.172668, -1.601393};
    float slotpoints[12] = {9.646909,3.896543,11.310846,3.837039,11.464449,8.132295,15.676724,7.981659,15.463739,2.025880,17.355309,1.958236};
    float avmSlot[8] = {12.744484,2.996786,12.858799,7.993479,15.321580,7.937136,15.207266,2.94044};
    float left_object[] = {16.519299,1.738719,16.254560,2.104692,15.860139,1.990756,15.620975,1.999674,15.564648,1.788154,15.703944,1.872966,15.698849,1.898726,15.494823,2.002841,15.489392,2.042876,15.449625,2.141337,15.323523,2.381658,15.415805,3.264195};
    float right_object[] = {13.157955,-6.175131,11.240582,-4.592729,10.990741,-4.478681,8.568865,-0.519559,9.487463,-2.805646,7.830590,1.670696,7.934606,3.962136,7.997727,6.852118};

    int avmType = 1;
    int dangerType = 0;
    int slotshap = 3;

    float curpos[4] = {9.069670, -0.188419, -0.225297, 9.107129};
    float finpoint[3] = {9.559866, 6.072683, -1.620743};
    float slotpoints[12] = {5.567078,3.917020,7.231537,3.882201,7.321417,8.178805,10.965668,8.049161,10.841373,2.107422,13.260953,2.056807};
    float avmSlot[8] = {8.865603,2.703419,8.861057,7.701417,11.309330,7.703644,11.313876,2.70564};
    float left_object[] = {10.845562,2.234938,12.424723,2.074300,11.533001,1.980334,11.335141,2.141197,11.619197,1.741267,11.427002,1.875280,11.258510,2.201383,11.186029,2.398587,11.191193,2.426040,11.269732,3.227852};
    float right_object[] = {6.434402,-4.295672,3.828413,-1.699932,5.098983,-3.300568,3.387166,0.972892,3.432338,3.849148,3.487481,6.731322,6.980265,6.565601,7.020162,6.766102};

    int avmType = 1;
    int dangerType = 4112;
    int slotshap = 3;
    float curpos[4] = {8.849965,1.222631,-1.257443, 9.107129};
    float finpoint[3] = {8.748332,5.787743,-1.630688};
    float slotpoints[12] = {4.104567,5.699947,6.014292,5.557904,6.192162,7.949299,10.320020,7.642273,9.893930,1.913638,12.976155,1.684385};
    float avmSlot[8] = {7.360128,2.488248,7.551829,7.482570,10.071041,7.385873,9.879339,2.39155};
    float left_object[] = {9.762662,1.923401,10.917620,1.860263,10.089496,1.711753,9.974640,1.802147};
    float right_object[] = {};

    int avmType = 1;
    int dangerType = 0;
    int slotshap = 3;
    float curpos[4] = {11.797080, 4.597175, -1.531030, 29.865469};
    float finpoint[3] = {11.748215,6.483469,-1.547866};
    float slotpoints[12] = {7.276890,5.981446,9.191499,6.018216,9.145454,8.415728,13.079720,8.466716,13.193162,2.559843,15.267213,2.599675};
    float avmSlot[8] = {10.858797,2.896024,10.716269,7.891992,13.237954,7.963932,13.380482,2.96796};
    float left_object[] = {12.993882,2.774498,13.105155,2.696361,13.217732,2.620277,14.521935,2.626862,13.682279,2.569236,13.464967,2.649501,13.350595,2.647974,13.312999,3.742717};
    float right_object[] = {};


    int avmType = 1;
    int dangerType = 0;
    int slotshap = 3;
    float curpos[4] = {16.128586, 3.567074, -1.593346, 30.512810};
    float finpoint[3] = {16.199894, 5.888574, -1.597079};
    float slotpoints[12] = {12.064756,5.595187,13.978848,5.537494,14.051092,7.934356,17.573282,7.805435,17.391552,1.776109,19.913380,1.700098};
    float avmSlot[8] = {15.143641,2.353756,15.246135,7.350705,17.752682,7.299293,17.650188,2.30234};
    float left_object[] = {};
    float right_object[] = {17.153605,1.936284,17.517384,1.994920,17.538424,1.865110,18.876669,1.798880,20.818501,1.843252,20.902441,1.995584,20.975803,1.923433,22.836473,1.578438,18.355425,1.806174,18.256538,1.885226,18.236059,1.886738,17.965584,1.758807,17.614233,1.953184,17.646580,2.983711};

    int avmType = 0;
    int dangerType = 0;
    int slotshap = 3;
    float curpos[4] = {7.715500, 5.952093, -1.671031, 27.378649};
    float finpoint[3] = {7.793126, 6.386401, -1.672745};
    float slotpoints[12] = {3.623949,2.687388,5.890839,2.628490,6.460564,8.197423,9.675457,7.868526,9.149182,2.724305,10.962902,2.398155};
    float avmSlot[8] = {-1.517185,0.000000,0.000000,0.000000,0.000000,-0.000000,0.000000,-0.000000};
    float left_object[] = {11.250426,2.358232,10.880802,2.438997,10.858997,2.422838,10.541815,2.504435,10.316573,2.572949,9.652526,2.591190,9.809316,2.502092,9.296021,2.754737,9.283824,2.795555,9.146231,2.935442,9.276095,3.139079,9.122160,3.284533,9.453635,2.478526,9.206973,2.810339,9.217736,2.823318,9.177185,3.208919,9.097798,3.842495,9.127353,6.256322};
    float right_object[] = {5.908800,3.154079,5.864016,6.607420};

    int avmType = 0;
    int dangerType = 0;
    int slotshap = 3;
    float curpos[4] = {8.980567, 4.422261, -1.650898, 23.120176};
    float finpoint[3] = {9.162008, 6.070549, -1.670524};
    float slotpoints[12] = {3.495032,2.544959,7.275712,2.307656,7.833061,7.877841,10.822693,7.578701,10.294542,2.300324,11.795205,2.093789};
    float avmSlot[8] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.00000};
    float left_object[] = {10.714912,2.225578,10.657698,2.416644,10.466462,2.425550,10.292973,2.728219,10.313486,2.745118,10.273326,4.760960};
    float right_object[] = {7.479218,2.644494,7.583822,3.230936,7.552928,3.261300,7.553744,5.010040};

    int avmType = 1;
    int dangerType = 0X8224;
    int slotshap = 3;

    float curpos[4] = {8.562171, 14.114932, 2.924007, 29.212055};
    float finpoint[3] = {12.589131,13.804774,3.098856};
    float slotpoints[12] = {10.462080,10.603350,10.542740,12.489573,13.967942,12.343102,14.082823,15.029572,8.747699,15.257718,8.828360,17.143940};
    float avmSlot[8] = {8.871918,12.662097,13.422758,12.467490,13.533878,15.066010,8.983038,15.26061};

    float left_object[] = {};
    float right_object[] = {10.645039,14.543468,10.547581,14.572336,9.450210,14.919394,9.266787,15.079441,9.115918,15.308620,9.142995,16.563046,9.224841,16.584259,9.333570,16.773920,9.443656,16.790268,9.555687,16.804527,10.092810,16.943653,10.106595,17.075113,11.192552,19.794310,11.189310,20.037603,9.142414,16.359348,9.127830,15.375948,9.169807,15.365077,9.269192,15.140131,9.236540,15.371393,9.449252,15.072946,9.054918,15.862768,9.189385,15.850109};

    int avmType = 1;
    int dangerType = 0X8224;
    int slotshap = 3;

    float curpos[4] = {8.562171, 14.114932, 2.924007, 29.212055};
    float finpoint[3] = {7.497380,7.954088,2.75013};
    float slotpoints[12] = {3.717005,1.867677,5.428306,1.851966,5.483629,7.878029,8.310988,7.852073,8.265930,2.944197,9.977230,2.928486};
    float avmSlot[8] = {5.706438,2.770769,5.750021,7.518016,7.997672,7.497380,7.954088,2.75013};

    float left_object[] = {};
    float right_object[] = {10.645039,14.543468,10.547581,14.572336,9.450210,14.919394,9.266787,15.079441,9.115918,15.308620,9.142995,16.563046,9.224841,16.584259,9.333570,16.773920,9.443656,16.790268,9.555687,16.804527,10.092810,16.943653,10.106595,17.075113,11.192552,19.794310,11.189310,20.037603,9.142414,16.359348,9.127830,15.375948,9.169807,15.365077,9.269192,15.140131,9.236540,15.371393,9.449252,15.072946,9.054918,15.862768,9.189385,15.850109};
#endif

    float curpos[4]      = {44.873253, 3.691591, 2.693015, 65.548286};
    float finpoint[3]    = {44.902084, 3.695610, 2.690691};
    float slotpoints[12] = {42.581448, -0.594376, 43.415962, 1.129229,
                            45.304283, 0.214966,  47.298901, 4.334650,
                            42.418068, 6.697788,  43.252583, 8.421392};
    float avmSlot[8]     = {41.030922, 4.073163, 46.517685, 1.416649,
                            47.691795, 3.841658, 42.205032, 6.49817};
    int slotshap         = 4;
    int dangerType       = 64;
    int avmType          = 1;
    float left_object[]  = {35.801678, 7.572368,  38.114742, 12.131098,
                            37.387890, 12.799011, 37.631413, 13.430583};
    float right_object[] = {};

    FusionObj_T fusion;
    fusion.num = sizeof(left_object) / sizeof(LineSeg_T);
    for (int i = 0; i < fusion.num; i++)
    {
        memcpy((float *)&fusion.obj[i], &left_object[i * 4], sizeof(LineSeg_T));
        fusion.attr[i] = 10000;
    }
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Left(&fusion);

    fusion.num = sizeof(right_object) / sizeof(LineSeg_T);
    for (int i = 0; i < fusion.num; i++)
    {
        memcpy((float *)&fusion.obj[i], &right_object[i * 4], sizeof(LineSeg_T));
        fusion.attr[i] = 30000;
    }
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Right(&fusion);

    Avm_Pot_T avmPot;
    memcpy((float *)&avmPot.near_rear, &avmSlot[0], sizeof(avmPot.near_rear));
    memcpy((float *)&avmPot.far_rear, &avmSlot[2], sizeof(avmPot.far_rear));
    memcpy((float *)&avmPot.far_front, &avmSlot[4], sizeof(avmPot.far_front));
    memcpy((float *)&avmPot.near_front, &avmSlot[6], sizeof(avmPot.near_front));
    RTE_PK_DataConvt_Set_IsAVM_Slot(avmType);
    RTE_PK_DataConvt_Set_TargAVM_SlotInfo(&avmPot);
    RTE_PK_ObjAvoid_Set_DangerSt(dangerType);

    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));
    float slotPoints[5][4];
    SlotObj_Convert_float(slotObj, slotPoints);

    float newTarget[3];
    PK_Cur_AutodrPara curSt[1];
    curSt[0].slotshape = slotshap;
    memcpy(curSt[0].curpos, curpos, sizeof(curpos));
    memcpy(curSt[0].finpos, finpoint, sizeof(curpos));
    PathExec_UpdateTarget(curSt, slotPoints, newTarget, false);
}

void PathExec_UpdateSlotByObjNearSlot_UnitTest()
{
#if 0
    int avmType = 1;
    int dangerType = 0;
    // int slotshap = 3;

    float curpos[4] = {7.331587, 1.680782, -1.336460, 13.285930};
    float finpoint[3] = {7.251686,6.186475,-1.591841};
    float slotpoints[12] = {2.929070,3.149544,4.843275,3.094425,4.990819,8.218391,8.679464,8.112178,8.509930,2.224497,10.607563,2.164096};
    float avmSlot[8] = {6.007048,3.039492,6.054191,8.037271,8.633788,8.012938,8.586645,3.01516};
    float left_object[] = {8.509930,2.224497,9.811673,2.071354,8.568752,2.273441,8.551634,2.390254};
    float right_object[] = {4.722789,-5.679368,1.729810,-1.352916,1.890742,-1.296027,1.283762,0.774477};

    float curpos[4] = {13.213869,-0.271821,-0.228748, 13.285930};
    float finpoint[3] = {9.111889,6.210642,-1.537047};
    float slotpoints[12] = {4.356684,5.617851,6.270593,5.682468,6.189678,8.079103,11.501652,8.258446,11.582567,5.861810,13.496477,5.926428};
    float avmSlot[8] = {7.861701,2.668490,7.660151,7.662425,10.213192,7.765463,10.414742,2.77152};
    float left_object[] = {5.961794,-4.635635,6.313796,-4.578503};
    float right_object[] = {10.225883,2.369970,11.656224,2.369415};
    //float curpos[4] = {10.878321, -0.042327, 0.053602, 10.878587};
    //float finpoint[3] = {7.590241,-6.590642,1.519721};
    //float slotpoints[12] = {3.816540,-2.751619,5.729043,-2.849385,5.442771,-8.449431,8.762680,-8.619143,8.941263,-5.125705,10.853765,-5.223471};
    //float avmSlot[8] = {6.071024,-2.699131,5.828687,-7.691252,8.138876,-7.803398,8.381212,-2.81127};
    //float left_object[] = {};
    //float right_object[] = {8.814228,-2.703964,8.845047,-2.594195,8.935491,-2.568172,9.891224,-2.612460,12.176176,-2.657933,12.278801,-2.657833,12.393038,-3.036304,13.118861,-2.892362,13.141021,-2.925112,13.784940,-2.983639};

    //float curpos[4] = {10.878321, -0.042327, 0.053602, 10.878587};
    //float finpoint[3] = {7.276149,6.321159,-1.5606681};
    //float slotpoints[12] = {2.672683,2.903614,4.587584,2.923009,4.533535,8.259481,8.468640,8.299337,8.504068,4.801517,10.418970,4.820912};
    //float avmSlot[8] = {6.128392,2.899745,6.144625,7.897719,8.559311,7.889877,8.543078,2.89190};
    //float left_object[] = {};
    //float right_object[] = {4.862051,2.945788,5.749928,3.051652,8.534706,2.640262,8.592742,2.531262,8.626026,2.510667,9.806665,2.531737};

    // E
    float curpos[4] = {9.184211, 0.012715, -0.128918, 9.186634};
    float finpoint[3] = {7.052014,6.349863,-1.568361};
    float slotpoints[12] = {3.477496,2.560726,5.392489,2.565392,5.378492,8.311793,8.123598,8.318480,8.132120,4.820490,10.047115,4.825154};
    float avmSlot[8] = {6.005777,2.784454,6.102386,7.822463,8.424660,7.777931,8.328051,2.73992};
    float left_object[] = {};
    float right_object[] = {8.356149,2.494549,9.625727,2.484500,9.636386,2.513950,9.735096,2.645130,10.085959,2.686795,11.683496,2.608549};
    // error: 3.477496,2.560727,5.392489,2.565392,5.378492,8.311793,8.123600,8.318480,8.137787,2.494017,10.052782,2.498681,6.806925,6.349266,-1.570803,5.922806,2.931445,5.941149,7.929412,8.374262,7.920482,8.355920,2.92251
#endif

    int dangerType       = 4112;
    int slotshap         = 3;
    int avmType          = 1;
    float curpos[4]      = {5.529337, 2.350852, -1.460654, 14.462945};
    float finpoint[3]    = {5.595460, 6.446042, -1.563917};
    float slotpoints[12] = {2.551479, 1.836646, 4.262838, 1.842982, 4.240453, 7.890119,
                            6.897830, 7.899956, 6.918225, 2.390994, 8.629584, 2.397330};
    float avmSlot[8]     = {4.505380, 2.786063, 4.487808, 7.532929,
                            6.735537, 7.541250, 6.753109, 2.79438};
    float left_object[]  = {4.822582, -4.958961, 5.498782, -4.883987, 5.949112, -4.924573,
                            6.200565, -5.038180, 6.210288, -3.627624, 6.227985, -3.307198,
                            6.277308, -3.002137, 6.351053, -3.306336, 6.883304, -4.580660,
                            7.116064, -4.541142, 7.401750, -4.479195, 7.646678, -4.471973,
                            7.786353, -4.439187, 8.126275, -4.349574};
    float right_object[] = {8.891694, 3.013314,  9.005739, 2.949739, 9.047251,
                            1.887404, 10.505512, 1.849999, 7.160328, 2.427189,
                            6.985630, 2.590186,  7.408273, 2.331655, 6.997767,
                            2.515953, 6.861908,  2.612869, 6.664094, 2.871156};

    FusionObj_T fusion;
    fusion.num = sizeof(left_object) / sizeof(LineSeg_T);
    for (int i = 0; i < fusion.num; i++)
    {
        memcpy((float *)&fusion.obj[i], &left_object[i * 4], sizeof(LineSeg_T));
        fusion.attr[i] = 10000;
    }
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Left(&fusion);

    fusion.num = sizeof(right_object) / sizeof(LineSeg_T);
    for (int i = 0; i < fusion.num; i++)
    {
        memcpy((float *)&fusion.obj[i], &right_object[i * 4], sizeof(LineSeg_T));
        fusion.attr[i] = 30000;
    }
    RTE_PK_SensorFusion_Set_Fusion_ObsInfo_B_Right(&fusion);

    Avm_Pot_T avmPot;
    memcpy((float *)&avmPot.near_rear, &avmSlot[0], sizeof(avmPot.near_rear));
    memcpy((float *)&avmPot.far_rear, &avmSlot[2], sizeof(avmPot.far_rear));
    memcpy((float *)&avmPot.far_front, &avmSlot[4], sizeof(avmPot.far_front));
    memcpy((float *)&avmPot.near_front, &avmSlot[6], sizeof(avmPot.near_front));
    RTE_PK_DataConvt_Set_IsAVM_Slot(avmType);
    RTE_PK_DataConvt_Set_TargAVM_SlotInfo(&avmPot);

    RTE_PK_ObjAvoid_Set_DangerSt(dangerType);

    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));

    PathExec_UpdateSlotByObjNearSlot(curpos, finpoint, avmPot, slotObj);
    printf("A:%f %f B:%f %f C:%f %f D:%f %f E:%f %f F:%f %f\n", slotObj.ptA.x,
           slotObj.ptA.y, slotObj.ptB.x, slotObj.ptB.y, slotObj.ptC.x, slotObj.ptC.y,
           slotObj.ptD.x, slotObj.ptD.y, slotObj.ptE.x, slotObj.ptE.y, slotObj.ptF.x,
           slotObj.ptF.y);
}

void PathExec_B_MoveOutRear_UnitTest()
{
#if 0
    float curpos[4] = {13.598312, -1.201796, -0.582263, 13.842353};
    float finpoint[3] = {10.802480,6.166512,-1.602892};
    float slotpoints[12] = {6.777051,3.864961,8.441160,3.810494,8.581758,8.106194,12.227821,7.986860,12.032879,2.030784,14.473444,1.950905};

    float curpos[4] = {12.030493,-0.719894,-0.509266, 13.842353};
    float finpoint[3] = {9.149180,6.627431,-1.575981};
    float slotpoints[12] = {4.827950,4.263966,6.492739,4.237443,6.561203,8.534898,10.537722,8.471546,10.442914,2.520592,12.576421,2.486602};

    float curpos[4] = {14.260373, 0.047304, -0.101815, 14.262486};
    float finpoint[3] = {9.312228,6.188707,-1.580144};
    float slotpoints[12] = {6.040975,3.091294,7.955865,3.070952,8.010017,8.168652,10.660599,8.140495,10.597436,2.194573,13.949951,2.158960};

    float curpos[4] = {11.375306,-0.086097,-0.140739, 14.262486};
    float finpoint[3] = {6.853374,5.974190,-1.613149};
    float slotpoints[12] = {3.391381,2.585732,5.263242,2.506406,5.496027,7.999476,8.377199,7.877378,8.129336,2.028496,11.057329,1.904414};

    float curpos[4] = {11.275688,2.515349,-1.601045, 14.262486};
    float finpoint[3] = {11.415277,6.138351,-1.609451};
    float slotpoints[12] = {6.509850,2.793421,8.423419,2.719417,8.635887,8.213310,12.901365,8.048348,12.688897,2.554456,14.598945,2.480587};

    float curpos[4] = {8.044133,-3.118243,1.565573, 14.262486};
    float finpoint[3] = {8.024882,-6.020906,1.562734};
    float slotpoints[12] = {3.046675,-2.503055,4.961610,-2.509913,4.941953,-7.998657,10.256910,-7.978001,10.276426,-2.528947,12.191360,-2.535805};

    float curpos[4] = {16.465366, 0.071878, 0.006136, 14.262486};
    float finpoint[3] = {11.957427,-6.273531,1.598635};
    float slotpoints[12] = {7.195022,-3.979288,9.109209,-3.923503,9.228585,-8.019765,13.413459,-7.897806,13.276268,-3.190261,16.318325,-3.101607};

    float curpos[4] = {6.086051,1.284788,-1.068928, 14.262486};
    float finpoint[3] = {5.604629,6.097624,-1.589271};
    float slotpoints[12] = {2.217845,4.324862,4.132518,4.289484,4.202739,8.089863,8.098031,8.017889,8.053730,5.620297,9.968403,5.584919};

    float curpos[4] = {13.213869,-0.271821,-0.228748, 14.262486};
    float finpoint[3] = {8.997500,6.206780,-1.534412};
    float slotpoints[12] = {4.356684,5.617851,6.270593,5.682468,6.189678,8.079103,10.380857,8.220606,10.577984,2.381858,13.612689,2.484315};

    float curpos[4] = {9.946882,-0.329441,-0.303719, 14.262486};
    float finpoint[3] = {6.293038,6.099671,-1.545071};
    float slotpoints[12] = {1.506691,5.571455,3.421305,5.609908,3.373153,8.007424,8.687082,8.114150,8.735233,5.716632,10.649845,5.755085};
#endif
    // float curpos[4] = {9.810924,1.398292,-0.928222, 14.262486};  //OK
    // float finpoint[3] = {9.695939,6.307230,-1.549460};
    // float slotpoints[12] =
    // {4.933743,5.773510,6.848307,5.814365,6.797147,8.211820,11.067513,8.302946,11.193956,2.377565,13.108521,2.418421};

    // float curpos[4] = {19.423887,-0.826217,-0.413750, 10.935361};  // OK
    // float finpoint[3] = {15.821032,5.945745,-1.592994};
    // float slotpoints[12] =
    // {11.385725,5.627271,13.300097,5.578209,13.361533,7.975422,17.243439,7.875937,17.094978,2.083048,19.229507,2.028344};

    // float curpos[4] = {11.120831,-0.316002,-0.255362, 10.935361}; // OK
    // float finpoint[3] = {8.566640,6.132995,-1.616574};
    // float slotpoints[12] =
    // {4.306783,5.893363,6.219825,5.806775,6.328253,8.202322,9.962379,8.037833,9.696482,2.163221,12.119907,2.053532};

    // float curpos[4] = {19.878904, -0.393403, -0.215537, 10.935361};   // OK
    // float finpoint[3] = {15.835651,6.293700,-1.557716};
    // float slotpoints[12] =
    // {11.502905,5.808687,13.417762,5.832102,13.388441,8.229922,17.124247,8.275604,17.196674,2.352543,19.496805,2.380669};

    // float curpos[4] = {9.457324,0.001474,-0.119714, 10.878587}; // OK
    // float finpoint[3] = {7.276149,6.321159,-1.5606681};
    // float slotpoints[12] =
    // {2.672683,2.903614,4.587584,2.923009,4.533535,8.259481,8.468640,8.299337,8.504068,4.801517,10.418970,4.820912};

    // float curpos[4] = {9.832036,4.157954,-1.535248,1.297982}; // OK
    // float finpoint[3] = {9.932306,6.471995,-1.537336};
    // float slotpoints[12] =
    // {5.301638,5.860734,7.215374,5.929794,7.128896,8.326212,11.263723,8.492452,11.470675,2.757508,13.356662,2.825565};

    // float curpos[4] = {7.365407, 2.984052, -1.625779, 1.297982}; // OK
    // float finpoint[3] = {7.710213,5.769024,-1.632588};
    // float slotpoints[12] =
    // {3.148729,5.690820,5.059329,5.564054,5.218068,7.956544,9.158310,7.619186,8.775328,1.846946,10.773865,1.714345};

    // float curpos[4] = {12.365458, -0.227176, -0.328473, 1.297982}; // OK
    // float finpoint[3] = {8.656410,6.262147,-1.534691};
    // float slotpoints[12] =
    // {4.595271,2.384827,6.509264,2.446886,6.324231,8.153562,9.912809,8.269918,10.098985,2.528024,12.012980,2.590083};

    float curpos[4]      = {10.962542, -3.651288, 1.529717, 1.297982};
    float finpoint[3]    = {11.021290, -6.458023, 1.540066};
    float slotpoints[12] = {6.341831,  -5.893314, 8.255992,  -5.949823,
                            8.185230,  -8.346775, 12.430120, -8.464052,
                            12.604982, -2.540855, 14.335852, -2.591953};

    float updatePath[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN];

    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));
    float enviObj[5][4];
    SlotObj_Convert_float(slotObj, enviObj);

    int trajNum = PathExec_B_MoveOutRear(updatePath, finpoint, enviObj, curpos);
    for (int index = 0; index < trajNum; index++)
    {
        float endPos[3];
        PK_Get_Path_EndPos(updatePath[index], endPos);
        printf("path %d: %2.4f %2.4f %2.4f %2.4f %2.4f end: %2.4f %2.4f %2.4f\n", index,
               updatePath[index][0], updatePath[index][1], updatePath[index][2],
               updatePath[index][3], updatePath[index][4], endPos[0], endPos[1],
               endPos[2]);
    }
}

void PathExec_B_MoveOut_ParaRePlan_UnitTest()
{
    float plannedPath[20][TRAJITEM_LEN];
#if 0
    float finpoint[3] = {-0.165465,2.782911,-0.041379};
    //float curpos[4] = {-0.249453, 2.688247, -0.495758, 12.831689};
    float curpos[4] = {-0.395032, 2.780207, -0.589606, 11.968675};
    float slotpoints[12] = {-4.119875,2.298859,-1.623014,2.195481,-1.531810,4.398316,5.376274,4.112300,5.285069,1.909466,7.781932,1.806087};
    int slotshap = 1;

    float finpoint[3] = {-0.381727, -2.928566, -0.023007};
    float curpos[4] = {-0.390725, -2.844639, 0.253866, 13.513005};
    float slotpoints[12] = {-4.644342,-2.080279,-2.146001,-2.137769,-2.196857,-4.347807,6.764771,-4.554029,6.815627,-2.343990,9.313964,-2.401481};
    int slotshap = 2;

    float finpoint[3] = {-0.013310, -2.916773, -0.046168};
    float curpos[4] = {-0.023638, -2.850743, -0.046402, 15.829508};
    float slotpoints[12] = {-4.055349,-1.979227,-1.559011,-2.094560,-1.660890,-4.299707,6.893984,-4.694950,6.995864,-2.489803,9.492204,-2.605135};
    int slotshap = 2;
#endif

    float finpoint[3]    = {-64.042244, -21.570183, -1.214177};
    float curpos[4]      = {-64.414078, -21.703947, -1.217940, 254.507156};
    float slotpoints[12] = {-66.042534, -17.919632, -65.170113, -20.261402,
                            -63.418972, -19.609018, -60.402550, -27.705750,
                            -62.153687, -28.358135, -61.281265, -30.699900};
    int slotshap         = 1;

    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));
    float slotPoints[5][4];
    SlotObj_Convert_float(slotObj, slotPoints);

    int trajNum = PathExec_B_MoveOut_ParaRePlan(plannedPath, finpoint, slotPoints, curpos,
                                                slotshap);
    for (int index = 0; index < trajNum; index++)
    {
        float endPos[3];
        PK_Get_Path_EndPos(plannedPath[index], endPos);
        printf("path %d: %2.4f %2.4f %2.4f %2.4f %2.4f end: %2.4f %2.4f %2.4f\n", index,
               plannedPath[index][0], plannedPath[index][1], plannedPath[index][2],
               plannedPath[index][3], plannedPath[index][4], endPos[0], endPos[1],
               endPos[2]);
    }
}

void PathExec_B_MoveOut_ParaRePlan_UnitTest1()
{
    float plannedPath[10][TRAJITEM_LEN];
    float tempTra[5] = {3.9167063305072816, -0.5368136701379513, -0.8505737847995576, 1.0,
                        0};
    float curpos[4]  = {-0.249453, 2.688247, -0.495758, 0};
    PK_PathPlan_TraExplorer_Con(tempTra, curpos, 0, NULL, plannedPath, 0, 1, 0, 0);
}

void PathExec_B_MoveOut_ParaRePlan_UnitTest2()
{
    float plannedPath[10][TRAJITEM_LEN];

    float curpos[4]      = {7.433434, -0.700565, 1.026778, 16.136154};
    float finpoint[3]    = {6.603241, -6.061104, 1.584209};
    float slotpoints[12] = {1.114924, -2.053652, 4.652505,  -2.154917,
                            4.727588, -7.752413, 8.523584,  -7.701495,
                            8.450065, -2.220533, 11.375722, -2.058159};
    int slotshap         = 4;
    int dangerType       = 0x20;
    int isAvmSlot        = 0;

    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));
    float slotPoints[5][4];
    SlotObj_Convert_float(slotObj, slotPoints);
    RTE_PK_DataConvt_Set_IsAVM_Slot(isAvmSlot);

    RTE_PK_ObjAvoid_Set_DangerSt(dangerType);
    PathExec_B_MoveOutFront(plannedPath, finpoint, slotPoints, curpos, slotshap);
}

void PathExec_B_MoveOut_ParaRePlan_UnitTest4()
{
    float plannedPath[10][TRAJITEM_LEN];

    float curpos[4]      = {7.613707, -3.562164, 1.470468, 16.136154};
    float finpoint[3]    = {7.681303, -6.418919, 1.615257};
    float slotpoints[12] = {3.795599, -3.127668, 6.213862,  -3.020081,
                            6.448833, -8.301557, 9.797947,  -8.152556,
                            9.608698, -3.898764, 11.521807, -3.813650};
    int slotshap         = 4;
    int dangerType       = 4112;
    int isAvmSlot        = 1;

    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));
    float slotPoints[5][4];
    SlotObj_Convert_float(slotObj, slotPoints);
    RTE_PK_DataConvt_Set_IsAVM_Slot(isAvmSlot);

    RTE_PK_ObjAvoid_Set_DangerSt(dangerType);
    PathExec_B_MoveOutFront(plannedPath, finpoint, slotPoints, curpos, slotshap);
}

void PathExec_SpaceExplorer_RePlanInSlot_UnitTest2()
{
    float plannedPath[10][TRAJITEM_LEN];
    int isAvmSlot = 0;

#if 0
    float curpos[4] = {8.165653,-6.054410,1.671985,1.0};
    float finpoint[3] = {7.507660, -6.519097, 1.579524};
    float slotpoints[12] = {2.744116,-4.128842,4.659043,-4.112129,4.696205,-8.369967,10.011003,-8.323580,9.973841,-4.065741,11.888767,-4.049028};

    float curpos[4] = {9.069670, -0.188419, -0.225297, 9.107129};
    float finpoint[3] = {9.559866, 6.072683, -1.620743};
    float slotpoints[12] = {4.921587,3.869505,6.584511,3.786378,6.799090,8.079018,11.115636,7.863243,10.841146,2.372098,12.504070,2.288973};
    int slotshap = 3;
#endif

    float curpos[4]      = {9.771319, 5.991717, -1.514569, 38.223686};
    float finpoint[3]    = {9.624549, 6.010133, -1.500721};
    float slotpoints[12] = {6.150984,  1.997458, 8.421519,  2.156827,
                            8.032440,  7.700030, 10.935622, 7.546561,
                            11.296940, 2.398875, 12.518297, 2.484602};
    int slotshap         = 3;

    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));
    float slotPoints[5][4];
    SlotObj_Convert_float(slotObj, slotPoints);
    RTE_PK_DataConvt_Set_IsAVM_Slot(isAvmSlot);

    int trajNum =
        PathExec_SpaceExplorer_RePlanInSlot(plannedPath, finpoint, slotPoints, curpos);
    if (trajNum <= 0)
    {
        trajNum =
            PathExec_B_MoveOutFront(plannedPath, finpoint, slotPoints, curpos, slotshap);
    }

    if (trajNum <= 0)
    {
        trajNum = PathExec_B_MoveOutRear(plannedPath, finpoint, slotPoints, curpos);
    }

    if (trajNum > 0)
    {
        float end[3];
        PK_Get_Path_EndPos(plannedPath[trajNum - 1], end);
        for (int j = 0; j < trajNum; j++)
        {
            PK_Get_Path_EndPos(plannedPath[j], end);
            printf(
                "------- path i %d %f %f %f %f %f end: %f %f %f---------------------\n",
                j, plannedPath[j][0], plannedPath[j][1], plannedPath[j][2],
                plannedPath[j][3], plannedPath[j][4], end[0], end[1], end[2]);
        }
    }
}

void PathExec_SpaceExplorer_RePlanInSlot_UnitTest3()
{
    float plannedPath[10][TRAJITEM_LEN];

    float curpos[][4] = {
        {7.689346, -6.507673, 1.473242, 24.335026},
        {7.689346, -6.507673, 1.673242, 24.335026},
        {7.309346, -6.507673, 1.473242, 24.335026},
        {7.309346, -6.507673, 1.673242, 24.335026},
        {7.309346, -6.507673, 1.573242, 24.335026},
        {7.689346, -6.507673, 1.573242, 24.335026},
        {7.309346, -6.507673, 1.573242, 24.335026},
        {7.589346, -6.507673, 1.573242, 24.335026},
        {7.409346, -6.507673, 1.573242, 24.335026},
        {7.559346, -6.507673, 1.573242, 24.335026},
        {7.459346, -6.507673, 1.573242, 24.335026},
        {7.539346, -6.507673, 1.573242, 24.335026},
        {7.459346, -6.507673, 1.573242, 24.335026},
        {7.469346, -6.507673, 1.573242, 24.335026},
        {7.489346, -6.507673, 1.503242, 24.335026},
        {7.489346, -6.507673, 1.633242, 24.335026},
        {7.56346, -6.507673, 1.503242, 24.335026},
        {7.56346, -6.507673, 1.633242, 24.335026},
        {7.40346, -6.507673, 1.503242, 24.335026},
        {7.40346, -6.507673, 1.633242, 24.335026},
    };

    float finpoint[3]    = {7.507660, -6.519097, 1.579524};
    float slotpoints[12] = {2.744116, -4.128842, 4.659043,  -4.112129,
                            4.696205, -8.369967, 10.011003, -8.323580,
                            9.973841, -4.065741, 11.888767, -4.049028};
    int isAvmSlot        = 1;

    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));
    float slotPoints[5][4];
    SlotObj_Convert_float(slotObj, slotPoints);
    RTE_PK_DataConvt_Set_IsAVM_Slot(isAvmSlot);

    for (size_t i = 0; i < sizeof(curpos) / sizeof(curpos[0]); i++)
    {
        int trajNum = PathExec_SpaceExplorer_RePlanInSlot(plannedPath, finpoint,
                                                          slotPoints, curpos[i]);
        if (trajNum > 0)
        {
            float end[3];
            PK_Get_Path_EndPos(plannedPath[trajNum - 1], end);
            printf(
                "----------------------test %ld start = %f %f %f end = %f %f "
                "%f---------------------\n",
                i, curpos[i][0], curpos[i][1], curpos[i][2], finpoint[0], finpoint[1],
                finpoint[2]);

            for (int j = 0; j < trajNum; j++)
            {
                PK_Get_Path_EndPos(plannedPath[j], end);
                printf(
                    "------- path i %d %f %f %f %f %f end: %f %f "
                    "%f---------------------\n",
                    j, plannedPath[j][0], plannedPath[j][1], plannedPath[j][2],
                    plannedPath[j][3], plannedPath[j][4], end[0], end[1], end[2]);
            }
        }
        else
        {
            printf(
                "----------------------failed %ld start = %f %f %f end = %f %f "
                "%f---------------------\n",
                i, curpos[i][0], curpos[i][1], curpos[i][2], finpoint[0], finpoint[1],
                finpoint[2]);
        }
    }
}

static void UpdateSlotBE_UnitTestStub(const char *msg)
{
    Json::Reader jsonReader;
    Json::Value jsonRoot;
    if (!jsonReader.parse(msg, jsonRoot))
    {
        return;
    }

    const char *key1 = "stamp";
    if (jsonRoot.isMember(key1))
    {
        std::string stamp = jsonRoot[key1].asString();
        printf("current stamp is %s\n", stamp.c_str());
    }

    const char *key7 = "target";
    if (jsonRoot.isMember(key7) && jsonRoot[key7].isArray() && jsonRoot[key7].size() == 3)
    {
        int offset = 0;
        float target[3];
        target[0] = jsonRoot[key7][offset++].asDouble();
        target[1] = jsonRoot[key7][offset++].asDouble();
        target[2] = jsonRoot[key7][offset++].asDouble();
        RTE_PK_SlotDetect_Set_TargPos(target);
    }

    const char *key8 = "slotobj";
    if (jsonRoot.isMember(key8) && jsonRoot[key8].isArray() &&
        jsonRoot[key8].size() == 12)
    {
        int offset = 0;
        SlotObj_T slotObj;
        slotObj.ptA.x = jsonRoot[key8][offset++].asDouble();
        slotObj.ptA.y = jsonRoot[key8][offset++].asDouble();
        slotObj.ptB.x = jsonRoot[key8][offset++].asDouble();
        slotObj.ptB.y = jsonRoot[key8][offset++].asDouble();
        slotObj.ptC.x = jsonRoot[key8][offset++].asDouble();
        slotObj.ptC.y = jsonRoot[key8][offset++].asDouble();
        slotObj.ptD.x = jsonRoot[key8][offset++].asDouble();
        slotObj.ptD.y = jsonRoot[key8][offset++].asDouble();
        slotObj.ptE.x = jsonRoot[key8][offset++].asDouble();
        slotObj.ptE.y = jsonRoot[key8][offset++].asDouble();
        slotObj.ptF.x = jsonRoot[key8][offset++].asDouble();
        slotObj.ptF.y = jsonRoot[key8][offset++].asDouble();
        RTE_PK_SlotDetect_Set_SlotObj(&slotObj);
    }

    const char *key9 = "radar";
    float radarDist[10];
    if (jsonRoot.isMember(key9) && jsonRoot[key9].isArray() &&
        jsonRoot[key9].size() == 10)
    {
        int offset   = 0;
        radarDist[0] = jsonRoot[key9][offset++].asInt();
        radarDist[1] = jsonRoot[key9][offset++].asInt();
        radarDist[2] = jsonRoot[key9][offset++].asInt();
        radarDist[3] = jsonRoot[key9][offset++].asInt();
        radarDist[4] = jsonRoot[key9][offset++].asInt();
        radarDist[5] = jsonRoot[key9][offset++].asInt();
        radarDist[6] = jsonRoot[key9][offset++].asInt();
        radarDist[7] = jsonRoot[key9][offset++].asInt();
        radarDist[8] = jsonRoot[key9][offset++].asInt();
        radarDist[9] = jsonRoot[key9][offset++].asInt();
    }

    const char *key10 = "curpos";
    float curpos[3];
    if (jsonRoot.isMember(key10) && jsonRoot[key10].isArray() &&
        jsonRoot[key10].size() == 4)
    {
        int offset = 0;
        curpos[0]  = jsonRoot[key10][offset++].asDouble();
        curpos[1]  = jsonRoot[key10][offset++].asDouble();
        curpos[2]  = jsonRoot[key10][offset++].asDouble();
        curpos[3]  = jsonRoot[key10][offset++].asDouble();
        RTE_PK_Location_Set_CurPos(curpos);
    }

    float ultrasMotorPos[USS_ECHO_INDEX_MAX][4];
    for (int i = 0; i < USS_ECHO_INDEX_MAX; i++)
    {
        memcpy(ultrasMotorPos[i], curpos, sizeof(curpos));
    }

    ultrasMotorPos[17][3] = radarDist[6];
    ultrasMotorPos[16][3] = radarDist[6];
    ultrasMotorPos[18][3] = radarDist[4];
    ultrasMotorPos[19][3] = radarDist[5];
    ultrasMotorPos[0][3]  = radarDist[6];
    ultrasMotorPos[1][3]  = radarDist[6];
    ultrasMotorPos[2][3]  = radarDist[6];
    ultrasMotorPos[3][3]  = radarDist[6];
    ultrasMotorPos[4][3]  = radarDist[0];
    ultrasMotorPos[5][3]  = radarDist[1];
    ultrasMotorPos[6][3]  = radarDist[2];
    ultrasMotorPos[7][3]  = radarDist[3];

    extern void PK_SensorT2S_SetCurrUltrasMotorPos(
        float ultrasMotorPos[USS_ECHO_INDEX_MAX][4]);
    PK_SensorT2S_SetCurrUltrasMotorPos(ultrasMotorPos);
}

void PathExec_B_MoveOut_ParaRePlan_UnitTest13()
{
    // extern void UpdateSlotBE(PK_Cur_AutodrPara curSt[1], float enviObj[5][4]);

    std::string query;
    std::ifstream in("pkb.json");

    RTE_PK_DataConvt_Set_IsAVM_Slot(0);
    PK_Cur_AutodrPara curSt;
    curSt.slotshape = 4;

    while (std::getline(in, query))
    {
        UpdateSlotBE_UnitTestStub(query.c_str());
        RTE_PK_Location_Get_CurPos(curSt.curpos);
        RTE_PK_SlotDetect_Get_TargPos(curSt.finpos);

        float enviObj[5][4];
        SlotObj_T slotObj;
        RTE_PK_SlotDetect_Get_SlotObj(&slotObj);
        SlotObj_Convert_float(slotObj, enviObj);
        // UpdateSlotBE(&curSt, enviObj);

        usleep(10 * 1000);
    }
}

void PathExec_Buff_UnitTest()
{
    PRINT_SWITCH_INFO("UnitTest Start");
    PathExec_PrintPathStart();

    PK_Cur_AutodrPara CurState;
    PathExec_PrintCurState(CurState);
    PathExec_PrintPathSwitch("11111111111111111111111111111", 123);
    PathExec_PrintPathEnd();
}

inline void PathExec_PointToAvmSlot(Avm_Pot_T &avmPot, float points[8])
{
    avmPot.near_rear.x  = points[0];
    avmPot.near_rear.y  = points[1];
    avmPot.far_rear.x   = points[2];
    avmPot.far_rear.y   = points[3];
    avmPot.far_front.x  = points[4];
    avmPot.far_front.y  = points[5];
    avmPot.near_front.x = points[6];
    avmPot.near_front.y = points[7];
}

void PathExec_Calc_Target_UnitTest()
{
#if 0
    PK_SlotShapeType slotShape = PK_SLOT_LEFT_VERT;
    float finpoint[3] = {8.074812, 6.247015, -1.577682};
    float slotpoints[12] = {3.535673,3.846214,5.200633,3.834749,5.230228,8.132647,9.669412,8.102079,9.631554,2.604210,11.296514,2.592745};
    float avmpoints[8] = {6.678672,2.748930,6.676277,7.746929,9.021625,7.748053,9.024019,2.75005};

    float finpoint[3] = {11.603178, -6.812764, 1.576861};
    float slotpoints[12] = {7.066011,-4.408237,8.730981,-4.398139,8.757048,-8.696060,13.158628,-8.669365,13.125282,-3.171467,14.790253,-3.161367};
    float avmpoints[8] = {10.603573,-3.474969,10.649569,-8.472757,13.009141,-8.451042,12.963145,-3.453255};

    PK_SlotShapeType slotShape = PK_SLOT_LEFT_VERT;
    float slotpoints[12] = {5.92496586, -2.18151045, 8.72652054, -2.02278376, 9.05254555, -7.77718067, 9.05254555, -7.77718067, 8.7449789, -2.3485868, 12.3990202, -2.14156079};
    float avmpoints[8] = {7.85171604, -2.33839846, 8.13443279, -7.32839632, 10.6903477, -7.1835866, 10.4076309, -2.19358873};
    float finpoint[3]= {9.41932011, -5.78724766, 1.62739253};
    int isAvmSlot = 1;

    int isAvmSlot = 1;
    uint8_t confidence = 64;
    PK_SlotShapeType slotShape = PK_SLOT_LEFT_VERT;
    float slotpoints[12] = {-1.237697,5.817802,0.694619,5.865690,0.638181,8.142990,6.551365,8.289534,6.607802,6.012232,8.540119,6.060120};
    float avmpoints[8] = {2.400679,2.810316,2.327258,7.807776,5.000200,7.847046,5.073621,2.84958};
    float finpoint[3]= {3.628077,6.872443,-1.546019};

    // 3?茅霉2篓3渭??渭梅??
    int isAvmSlot = 0;
    uint8_t confidence = 1;
    PK_SlotShapeType slotShape = PK_SLOT_RIGHT_VERT;
    float slotpoints[12] = {5.704358,-2.642478,7.464187,-2.576087,7.892266,-8.430545,11.117757,-8.194696,10.729362,-2.882965,11.758172,-2.771136};
    float avmpoints[8] = {0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.00000};
    float finpoint[3]= {9.383517,-6.651057,1.643786};
#endif

    int isAvmSlot              = 1;
    uint8_t confidence         = 68;
    PK_SlotShapeType slotShape = PK_SLOT_LEFT_VERT;
    float slotpoints[12]       = {8.487202,  -11.336191, 8.454517,  -13.318434,
                                  14.126205, -13.411956, 14.073603, -16.602091,
                                  9.844959,  -16.532364, 9.795265,  -19.546152};
    float avmpoints[8]         = {8.983527,  -13.376141, 13.980848, -13.458542,
                                  13.938534, -16.024775, 8.941213,  -15.94237};
    float finpoint[3]          = {12.288357, -14.714099, 3.125105};

    Avm_Pot_T avmSlot;
    VehPos_T targpos;
    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));
    float slotPoints[5][4];
    SlotObj_Convert_float(slotObj, slotPoints);
    RTE_PK_DataConvt_Set_IsAVM_Slot(isAvmSlot);

    PathExec_PointToAvmSlot(avmSlot, avmpoints);
    memcpy((float *)&targpos, finpoint, sizeof(targpos));
    Calc_Target(isAvmSlot, confidence, slotObj, avmSlot, targpos);
    printf("1 old is %f %f %f, new target is %f %f %f\n", finpoint[0], finpoint[1],
           finpoint[2], targpos.x, targpos.y, targpos.theta);

    PK_CopyPos(finpoint, (float *)&targpos);
    Calc_Target(slotShape, confidence, slotObj, avmSlot, targpos);
    printf("2 old is %f %f %f, new target is %f %f %f\n", finpoint[0], finpoint[1],
           finpoint[2], targpos.x, targpos.y, targpos.theta);

    PK_CopyPos(finpoint, (float *)&targpos);
    Calc_Target(isAvmSlot, confidence, slotObj, avmSlot, targpos);
    printf("3 old is %f %f %f, new target is %f %f %f\n", finpoint[0], finpoint[1],
           finpoint[2], targpos.x, targpos.y, targpos.theta);
}

void PathExec_MovePath_dyna_UnitTest()
{
    extern void PathExec_MovePath_dyna(
        float Planned_Path[MAX_SINGLE_TRAJ_NUM][TRAJITEM_LEN], float newTarget[3],
        PK_Cur_AutodrPara curSt[1]);
    float Planned_Path[4][TRAJITEM_LEN] = {
        {1.597285, 0.001748, 0.001518, 1.647816, 0.000000},
        {3.245100, 0.004249, 0.001518, 2.988458, -5.890265},
        {6.108111, -0.733387, -0.505837, -6.000000, 5.800000},
        {3.121137, 4.163538, -1.540320, -2.100000, 0.000000}};

    float newTarget[3] = {3.057147, 6.262563, -1.540320};
    float finPos[3]    = {3.057147, 6.262563, -1.540320};
    PK_Cur_AutodrPara curSt;
    PK_CopyPos(curSt.finpos, finPos);
    curSt.slotshape   = PK_SLOT_RIGHT_VERT;
    curSt.cur_tra_nth = 0;
    curSt.tra_num     = 4;
    PathExec_MovePath_dyna(Planned_Path, newTarget, &curSt);

    newTarget[0] = 3.157147;
    PathExec_MovePath_dyna(Planned_Path, newTarget, &curSt);

    newTarget[0] = 3.087147;
    newTarget[1] = 6.302563;
    PathExec_MovePath_dyna(Planned_Path, newTarget, &curSt);

    newTarget[0] = 3.087147;
    newTarget[1] = 6.302563;
    newTarget[2] = -1.560320;
    PathExec_MovePath_dyna(Planned_Path, newTarget, &curSt);
}

void UpdateSlotByObjNearSlot_Unit()
{
#if 0
    int avmType = 1;
    int slottype = 3;
    float finpoint[3] = {6.863909,6.425297,-1.579977};
    float slotpoints[12] = {3.717005,1.867677, 5.428306,1.851966, 5.483629,7.878029, 8.310988,7.852073, 8.265930,2.944197, 9.977230,2.928486};
    float avmSlot[8] = {5.706438,2.770769,5.750021,7.518016,7.997672,7.497380,7.954088,2.75013};
    float nearobject[] = {8.051346,3.067718,7.932731,3.449727,8.950190,2.587551,8.152008,2.944899,8.629012,2.610713,8.184828,2.897875,5.508213,1.688298,5.359802,2.382244};
#endif

    int avmType          = 1;
    int slottype         = 3;
    float finpoint[3]    = {6.380719, 6.224401, -1.569540};
    float slotpoints[12] = {3.034357, 2.645060, 4.909356, 2.647415, 4.903055, 7.662545,
                            7.782611, 7.666162, 7.789368, 2.288537, 9.664367, 2.290893};
    float avmSlot[8]     = {5.083424, 2.629937, 5.076795, 7.905491,
                            7.680415, 7.908763, 7.687044, 2.63320};
    float nearobject[]   = {8.661360, 2.359368, 7.714924, 2.657473, 8.153090,
                            2.305959, 7.704790, 2.590918, 7.688099, 2.717610,
                            7.465953, 3.132439, 7.554195, 2.730383, 7.470170,
                            3.672388, 4.742038, 2.689552, 4.889044, 3.505216};

    FusionObj_T fusion;
    memset(&fusion, 0, sizeof(fusion));
    fusion.num = sizeof(nearobject) / sizeof(LineSeg_T);
    for (int i = 0; i < fusion.num; i++)
    {
        memcpy((float *)&fusion.obj[i], &nearobject[i * 4], sizeof(LineSeg_T));
        fusion.attr[i] = 10000;
    }

    Avm_Pot_T avmPot;
    memcpy((float *)&avmPot.near_rear, &avmSlot[0], sizeof(avmPot.near_rear));
    memcpy((float *)&avmPot.far_rear, &avmSlot[2], sizeof(avmPot.far_rear));
    memcpy((float *)&avmPot.far_front, &avmSlot[4], sizeof(avmPot.far_front));
    memcpy((float *)&avmPot.near_front, &avmSlot[6], sizeof(avmPot.near_front));
    RTE_PK_DataConvt_Set_IsAVM_Slot(avmType);

    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));

    float newTarget[3];
    extern uint8_t UpdateSlotByObjNearSlot(const float finpoint[3],
                                           const Avm_Pot_T &avmSlot,
                                           FusionObj_T &nearSlot, SlotObj_T &slotObj);
    auto confirence = UpdateSlotByObjNearSlot(finpoint, avmPot, fusion, slotObj);

    VehPos_T targpos;
    memcpy(&targpos, finpoint, sizeof(finpoint));
    Calc_Target(slottype, confirence, slotObj, avmPot, targpos);
}

void PathExec_ParaPostTraj_UnitTest()
{
    extern int PathExec_ParaPostTraj1(PK_Cur_AutodrPara curSt[1], float enviObj[5][4]);
    float slotpoints[12] = {4.619415,  1.558415,  7.117755,  1.615798,
                            7.190844,  -1.566369, 15.792381, -1.368808,
                            15.728001, 1.434234,  18.226343, 1.491617};

    PK_Cur_AutodrPara curSt;
    curSt.cur_tra_nth         = 05;
    curSt.tra_num             = 05;
    curSt.cur_tra_state       = 0;
    curSt.slotshape           = 2;
    curSt.step_gone           = 0000;
    curSt.B_BrakeState        = 0;
    curSt.B_DynaPlanCounter   = 0;
    curSt.ry_RePathSt         = 0.000000;
    curSt.B_Danger_filter     = 8;
    curSt.B_replan            = 2;
    curSt.brakeSt             = 1;
    curSt.B_Stop_Counter      = 0;
    curSt.AfterTra_nth        = 00;
    curSt.stillCount          = 18;
    curSt.B_ReplanResult      = 0;
    curSt.curpos[0]           = 9.083779;
    curSt.curpos[1]           = 0.091623;
    curSt.curpos[2]           = 0.197316;
    curSt.curpos[3]           = 27.317629;
    curSt.finpos[0]           = 9.085079;
    curSt.finpos[1]           = 0.035048;
    curSt.finpos[2]           = 0.022964;
    curSt.cur_eps             = 415.899994;
    curSt.cur_vel             = -0.000000;
    curSt.ds_gone_dir         = 0.000000;
    curSt.ds_start_dir        = 27.317629;
    curSt.ceps                = 418.172424;
    curSt.cvel                = 0.000000;
    curSt.cvel_last           = 0.000000;
    curSt.de                  = -0.057358;
    curSt.de_last             = -0.057358;
    curSt.sum_de              = 0.000000;
    curSt.rx_cur_to_fin       = -0.000000;
    curSt.ry_cur_to_fin       = 0.056589;
    curSt.rtheta_cur_to_fin   = 0.174352;
    curSt.deps                = 0.000000;
    curSt.Rou_req             = 0.176984;
    curSt.EPS_req             = 418.172394;
    curSt.drx_brake           = 0.000000;
    curSt.drx_crou            = 0.000000;
    curSt.rx_RePathSt         = 0.000000;
    curSt.rx_brake            = 0.000000;
    curSt.rxf                 = 0.090641;
    curSt.rxs                 = 0.524757;
    curSt.leftDist            = 0.107669;
    curSt.rthetaf             = -0.174352;
    curSt.rthetas             = 0.093960;
    curSt.min_arround_dist    = 0.000000;
    curSt.min_arround_dist_dt = 0.000000;
    curSt.de_dt               = -0.057358;
    curSt.bUpdate             = 00;
    curSt.dangerCounter       = 000;
    curSt.percent             = 0.988497;
    curSt.B_FirstPlanFlag     = 0;
    curSt.updatePathNum       = 0;
    curSt.stillCount          = 1;
    curSt.pathDist            = 9.36;
    curSt.gear                = 3;
    curSt.replanSt            = 0;
    curSt.targState           = 0;

    SlotObj_T slotObj;
    memcpy(&slotObj, slotpoints, sizeof(slotpoints));

    float enviObj[5][4];
    SlotObj_Convert_float(slotObj, enviObj);

    PathExec_ParaPostTraj1(&curSt, enviObj);
}
#endif
