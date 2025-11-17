#include "PK_Calibration.h"
#include "MathPara.h"
#include "json/json.h"

//  Calibration Para
const volatile float Rear_wheel_fac_ds_max =
    0.01106951f; // 0.010725761;  //PT6 0.01079362f;  //PT5//0.01068398f; // ET9 DATA
const volatile float Motorspd_fac_vspd_max =
    0.0042867f; // PT5//0.004111746;  //0.0043063372f;  // et data
const volatile float Rear_wheel_fac_ds_min = 0.01063541f;
const volatile float Motorspd_fac_vspd_min =
    0.0041186f; // PT5//0.004111746;  //0.0043063372f;  // et data
const volatile float MIN_S =
    3.0f; // 2.7f                  //  鍓嶅悗杞﹁締娌胯溅鍓嶈繘鏂瑰悜鐨勬渶灏忛暱
const volatile float MAX_VEHICLE_LEN =
    7.0f; //  鏋勬垚杞︿綅鐨勮溅杈嗘渶澶х殑鍙?鑳介暱搴?:m
const volatile float MAX_S =
    6.0f; //  鍓嶅悗杞﹁締娌胯溅鍓嶈繘鏂瑰悜鐨勬渶澶ч暱搴︼紝鎺掗櫎璺?娌裤€佽繛缁?澧欓潰鐨勬儏鍐?
const volatile float MAX_PARK_LEN_PARALLEL =
    10.0f; // 12.0f//   骞宠?岃溅浣嶇殑鏈€澶ц溅浣嶉暱
const volatile float MIN_PARK_LEN_PARALLEL =
    5.1f; //(VEHICLE_LEN+1.0f);         //  骞宠?岃溅浣嶇殑鏈€灏忚溅浣嶉暱
const volatile float MAX_PARK_LEN_VERTICAL =
    5.1f; // MIN_PARK_LEN_PARALLEL;  //VEHICLE_LEN                    //
const volatile float MIN_PARK_LEN_VERTICAL =
    2.46f; //(VEHICLE_WID+0.6f);            //  鍨傜洿杞︿綅鐨勬渶灏忚溅浣嶉暱
const volatile float MID_S = 5.3f; // MIN_PARK_LEN_PARALLEL;                        //
                                   // 鍓嶅悗涓よ溅涓?闂存部杞﹀墠杩涙柟鍚戠殑鏈€灏忛暱
const volatile uint16 MAX_VALID_DIST_ULTRASONIC =
    2000; //  渚у悜瓒呭０娉㈤浄杈炬?€娴嬬殑鏈夋晥璺濈?绘渶澶?mm
const volatile uint16 MABEY_JUMP_ULTRA_DIST =
    500; //  杞︿綅妫€娴嬬枒浼艰烦鍙樼殑鏈€灏忚窛?mm
const volatile uint16 REAL_JUMP_ULTRA_DIST =
    1500; //  纭?璁ゅ彂鐢熻烦鍙樼殑鏈€灏忚窛绂伙細mm
const volatile float JUMP_DETCT_DELTA_PASS_PATH =
    0.75f; //  杩涜?岃烦鍙樻部妫€娴嬫椂锛屾帓闄よ繛缁?璺冲彉骞叉壈鐨勬?€娴嬭窛绂伙細m
const volatile float MIN_PASS_PATH_START_DETECT =
    5.0f; //  鍓嶉潰鍙戠敓杩囨硦杞︿腑鏂?鎴栨硦鍏ュ畬鎴愮殑鎯呭喌锛屽?昏溅浣嶈Е鍙戦渶瑕佽溅浣嶇Щ鍔ㄧ殑鏈€灏忚窛
const volatile float FRONT_SAFE_RATIO =
    0.5f; //  鐩?鏍囧仠杞︿綅濮胯溅澶磋窛绂诲墠鏂瑰崰鍏ㄩ儴杞︿綅绌洪棿鐨勬瘮渚?
const volatile float BIG_VERT_SLOT_LEN_DIFF =
    2.5f; //  忙炉鈥澝?陆娄氓庐陆茅鈥⒙棵ヂ郝γヂぢ?莽拧鈥灻ヅ锯€毭р€郝疵?陆娄盲陆聧盲戮搂猫戮鹿盲录拧茅聺聽猫驴鈥樏?卢卢盲潞艗猫戮鈥犆?陆??
const volatile float BIG_VERT_SEC_VEH_DIST =
    1.0f; //  氓陇搂莽拧鈥灻ヅ锯€毭р€郝疵?陆娄盲陆聧氓聛艙茅聺聽盲戮搂猫戮鹿忙沤楼猫驴鈥樏?卢卢盲潞艗猫戮鈥犆?陆娄莽拧鈥灻?路聺莽???
const volatile float BIG_PARA_SLOT_LEN_DIFF = 3.0f; // ???????????????????
const volatile float BIG_PARA_SEC_VEH_DIST  = 1.5f; //  ?????????????????
const volatile float SlotMidObjLen_min =
    0.2f; //  杞︽Ы鍐呴儴闈犺繎杞﹁締鐨勭粏灏忕嚎娈靛彧鏈夐暱搴﹁秴杩囪?ュ€兼墠鑰冭檻锛屽惁鍒欏拷鐣?
const volatile float AVM_TarPos_CD_sidedist =
    0.4f; //  if no curb detected,set this value for targpos side dist to CD
const volatile float SlotTransDistMax_ccp =
    0.9f; //  B鹿媒鲁脤鲁碌脦禄脝陆脪脝赂眉脨脗脳卯露脿脭脢脨铆脫毛A鹿媒鲁脤录矛虏芒碌脛鲁碌脦禄碌脛脝芦虏卯
const volatile float Para_SafeDist_Curb_ccp =
    0.35f; //   虏麓脠毛鲁碌脦禄潞贸拢卢鲁碌脕戮虏脿卤脽戮脿脌毛脗路脩脴碌脛掳虏脠芦戮脿脌毛:m
const volatile float AVM_Slot_Trans_SafeDist_ccp =
    0.4f; //  脢脫戮玫鲁碌脦禄脛驴卤锚脦禄脳脣戮脿脌毛脮脧掳颅脦茂鹿媒陆眉陆酶脨脨脝陆脪脝拢卢脝陆脪脝碌陆麓脣戮脿脌毛
const volatile float Vert_AVM_Targpos_bias_max =
    0.6f; //  the AVM slot is cut by sensorfusion A,the max allowed targpos bias.
const volatile float Para_AVM_Targpos_bias_max =
    1.2f; //  the AVM slot is cut by sensorfusion A,the max allowed targpos bias.
const volatile float AVM_Para_SlotUpdate_Safe_dist_ccp =
    0.6f; // for AVM parallel slot update, the safe dist bewteen vehicle and obstacles
const volatile float Update_Rotate_Start_Dist_ccp =
    0.8f; // vehicle move in park distance start rotate park.
const volatile float MAX_AVM_PARK_LEN_PARALLEL = 7.1f; // 2018/08/08
const volatile float MIN_AVM_PARK_LEN_PARALLEL = 4.6f; // 2018/08/08
const volatile float MAX_AVM_PARK_LEN_VERTICAL = 3.7f; // 2018/08/08
volatile float MIN_AVM_PARK_LEN_VERTICAL       = 1.9f; // 2018/08/08
const volatile float CurbParaSlotDepth_min =
    1.2f; //  the min depth for parallel slots with curbs
const volatile float Vertical_Target_Max_mov_ccp =
    0.8f; // Vertical Target Max move distance
const volatile float Parallel_Target_Max_mov_ccp =
    0.9f; // Parallel Target Max move distance
const volatile float Para_VehEdge2CD_Min_Dist =
    0.4f; // AVM_Parallel slot,safe distance from CD to Vehicle body
const volatile float Para_CD_Search_Region_Offset = 0.4f;
const volatile float WIDE_OFFSET = 0.5f; //  娉婂叆杞︿綅鍚庤溅杈嗚窛绂昏矾娌跨殑璺濈??
const volatile float LEN_OFFSET  = 0.4f; //  娉婂叆杞︿綅鍚庤溅杈嗚窛绂昏矾娌跨殑璺濈??
const volatile float PEPS_CALL_RUN_VEL = 0.25f;  //  鍙?鍞よ溅閫燂細m/s
const volatile float LocatingPeriod_dt = 0.02f;  //
const volatile float EPS_SPD_CCP       = 650.0f; // D2
const volatile float V_FAC_PARKING_CCP = 0.5f;   //
const volatile float K1_MDB_CCP        = 330.0f; // model based control para1,
const volatile float K2_MDB_CCP        = 12.0f;  // model based control para2,
const volatile float MAXEPS_MDB_CCP    = 300.0f; // the max EPS_angle PI module can tune.
const volatile float DE_GAP_CCP        = 0.04f;
const volatile float g_local_swell     = 0.2;

volatile float ESC_VehSpd_factor        = 1.014f; // factor to correct ESC_VehSpd
volatile float EPS_Offset               = 0.0f;   // factor to correct ESC_VehSpd
volatile float AVM_Para_TargPos_off_dy  = 0.0;
volatile float AVM_Para_TargPos_off_ldx = 0.01;
volatile float AVM_Para_TargPos_off_rdx = 0.01;
volatile float AVM_Vert_TargPos_off_ldy = 0.00;
volatile float AVM_Vert_TargPos_off_rdy = 0.00;
volatile float AVM_Vert_TargPos_off_ldx = 0.00;
volatile float AVM_Vert_TargPos_off_rdx = 0.00;

float VEHICLE_WID     = 1.890;  // 杞﹁締瀹藉害 (m)
float VEHICLE_LEN     = 4.707;  // 杞﹁締闀垮害 (m)
float REAR_SUSPENSION = 0.999f; // 鍚庢偓 (m)
float WHEEL_BASE      = 2.740;  // 杞磋窛 (m)
float WHELL_RADIUS    = 0.39;   // 0.34961;      // rear 0.3496077449435367887983283141038
                           // front 0.34619693767579496646858852567352 杞?鑳庡崐寰? (m)
float FRONT_WHELL_RADIUS = 0.347;
float WHELL_PLUAS        = 96;     // 杞?鑴夊啿鏁? (娆?鍦?
float PK_EPS_ANGLE_MAX   = 480.0;  // PK鏂瑰悜鐩樻渶澶ц浆瑙?(deg)
float REAR_VIEW_MIRROR_X = 1.818f; // x distance relative to rear suspension center point
float Rrmin              = 4.75;   // 鏈€灏忚浆寮?鍗婂??
float PLUS_MOTOR_RPM     = 0.150163784f;

volatile float Motorspd_fac_vspd =
    PI2 * FRONT_WHELL_RADIUS * PLUS_MOTOR_RPM / WHELL_PLUAS; // 0.0043063372f;
volatile float Rear_wheel_fac_ds = PI * WHELL_RADIUS / WHELL_PLUAS;

float FSR_DELTAX = 3.19062f; // 定义探头1，FSR：右侧探头位置,朝向     对应宋pro-6号FSR
float FSR_DELTAY = -0.9188f;  // 2022.7.31 超声探头位置调整by lyx,update 探头位置和角度
float FSR_AERFA  = -1.57080f; // 2022.7.31 超声探头位置调整by lyx,update 探头位置和角度
float FSL_DELTAX = 3.19062f; // 定义探头2，FSL：左侧探头位置,朝向      对应宋pro-1号FSL
float FSL_DELTAY = 0.9188f;  // 2022.7.31 超声探头位置调整by lyx,update 探头位置和角度
float FSL_AERFA  = 1.57080f; // 2022.7.31 超声探头位置调整by lyx,update 探头位置和角度
float RSL_DELTAX = -0.45174f; // 定义探头3，RSL：左侧探头位置，朝向      对应宋pro-12号RLS
float RSL_DELTAY = 0.92545f;
float RSL_AERFA  = 1.605703f;
float RSR_DELTAX = -0.45174f; // 定义探头4，RSR：右侧探头位置，朝向     对应宋pro-7号RRS
float RSR_DELTAY = -0.92545f;
float RSR_AERFA  = -1.605703f;
float FOL_DELTAX = 3.44664f; // 定义探头5，FOL：前方探头位置，朝向
float FOL_DELTAY = 0.76067f;
float FOL_AERFA  = 0.806866f;
float FCL_DELTAX = 3.61424f; // 定义探头6，FCL：前方探头位置，朝向
float FCL_DELTAY = 0.30188f;
float FCL_AERFA  = 0.10472f;
float FCR_DELTAX = 3.61424f; // 定义探头7，FCR：前方探头位置，朝向
float FCR_DELTAY = -0.30188f;
float FCR_AERFA  = -0.10472f;
float FOR_DELTAX = 3.44664f; // 定义探头8，FOR：前方探头位置，朝向
float FOR_DELTAY = -0.76067f;
float FOR_AERFA  = -0.806866f;
float ROL_DELTAX = -0.84185f; // 定义探头9，ROL：后方探头位置，朝向
float ROL_DELTAY = 0.75065f;
float ROL_AERFA  = 2.626207f;
float RCL_DELTAX = -0.9453f; // 定义探头10，RCL：前方探头位置，朝向
float RCL_DELTAY = 0.30076f;
float RCL_AERFA  = 3.07283f;
float RCR_DELTAX = -0.9453f; // 定义探头11，RCR：前方探头位置，朝向
float RCR_DELTAY = -0.30076f;
float RCR_AERFA  = -3.07283f;
float ROR_DELTAX = -0.84185f; // 定义探头12，ROR：前方探头位置，朝向
float ROR_DELTAY = -0.75065f;
float ROR_AERFA  = -2.626207f;
