#include "PK_Calibration.h"
#include "PK_Utility.h"
#include "MathPara.h"
#include "MathFunc.h"
#include "Record_Log.h"
#include "json/json.h"

#define list_number 36
static float angle_list[4][list_number];
static float radius_list[4][list_number];
static int angle_number[4]  = {0, 0, 0, 0};
static int radius_number[4] = {0, 0, 0, 0};
static int vehicle_type     = -1;

#define READJSONDOUBLE(data)                   \
    {                                          \
        if (jsonRoot.isMember(#data))          \
            data = jsonRoot[#data].asDouble(); \
    }
#define READJSONINT(data)                   \
    {                                       \
        if (jsonRoot.isMember(#data))       \
            data = jsonRoot[#data].asInt(); \
    }
#define READJSONSTRING(data)                                  \
    {                                                         \
        if (jsonRoot.isMember(#data))                         \
            strcpy(data, jsonRoot[#data].asString().c_str()); \
    }

static int ReadList(Json::Value &jsons, const char *key, float *values,
                    const uint32_t valueLen)
{
    int count = 0;
    if (jsons.isMember(key) && jsons[key].isArray() && jsons[key].size() <= valueLen)
    {
        count = jsons[key].size();
        for (int i = 0; i < count; i++)
        {
            values[i] = jsons[key][i].asDouble();
        }
    }

    return count;
}

static void ParseVehParas(std::string &jsonfile)
{
    Json::Reader jsonReader;
    Json::Value jsonRoot;
    std::ifstream ifs;

    jsonRoot.clear();
    ifs.open(jsonfile.c_str());
    if (!ifs)
    {
        log_err("open json file %s failed!\n", jsonfile.c_str());
        return;
    }

    if (!jsonReader.parse(ifs, jsonRoot))
    {
        log_err("read json file %s failed\n", jsonfile.c_str());
        ifs.close();
        return;
    }

    READJSONDOUBLE(VEHICLE_WID)
    READJSONDOUBLE(VEHICLE_LEN)
    READJSONDOUBLE(REAR_SUSPENSION)
    READJSONDOUBLE(WHEEL_BASE)
    READJSONDOUBLE(WHELL_RADIUS)
    READJSONDOUBLE(WHELL_PLUAS)
    READJSONDOUBLE(PK_EPS_ANGLE_MAX)
    READJSONDOUBLE(REAR_VIEW_MIRROR_X)
    READJSONDOUBLE(Rrmin)

    const char *angleKeys[]  = {"angleLD", "angleLR", "angleRD", "angleRR"};
    const char *radiusKeys[] = {"radiusLD", "radiusLR", "radiusRD", "radiusRR"};
    float factor[4]          = {1.0f, 1.0f, 1.0f, 1.0f}; // LD LR RD RR
    for (int i = 0; i < 4; i++)
    {
        angle_number[i]  = ReadList(jsonRoot, angleKeys[i], angle_list[i], list_number);
        radius_number[i] = ReadList(jsonRoot, radiusKeys[i], radius_list[i], list_number);
        if (angle_number[i] != radius_number[i])
        {
            memset(radius_number, 0, sizeof(radius_number));
            memset(angle_number, 0, sizeof(angle_number));
            break;
        }

        for (int j = 0; j < radius_number[i]; j++)
        {
            radius_list[i][j] = radius_list[i][j] * factor[i];
        }
    }

#define READRADARVALUE(key, x, y, dir)             \
    {                                              \
        float para[3];                             \
        if (ReadList(jsonRoot, key, para, 3) == 3) \
        {                                          \
            x   = para[0];                         \
            y   = para[1];                         \
            dir = para[2];                         \
        }                                          \
    }

    READRADARVALUE("FSR_DELTA", FSR_DELTAX, FSR_DELTAY, FSR_AERFA)
    READRADARVALUE("FSL_DELTA", FSL_DELTAX, FSL_DELTAY, FSL_AERFA)
    READRADARVALUE("RSL_DELTA", RSL_DELTAX, RSL_DELTAY, RSL_AERFA)
    READRADARVALUE("RSR_DELTA", RSR_DELTAX, RSR_DELTAY, RSR_AERFA)
    READRADARVALUE("FOL_DELTA", FOL_DELTAX, FOL_DELTAY, FOL_AERFA)
    READRADARVALUE("FCL_DELTA", FCL_DELTAX, FCL_DELTAY, FCL_AERFA)
    READRADARVALUE("FCR_DELTA", FCR_DELTAX, FCR_DELTAY, FCR_AERFA)
    READRADARVALUE("FOR_DELTA", FOR_DELTAX, FOR_DELTAY, FOR_AERFA)
    READRADARVALUE("ROL_DELTA", ROL_DELTAX, ROL_DELTAY, ROL_AERFA)
    READRADARVALUE("RCL_DELTA", RCL_DELTAX, RCL_DELTAY, RCL_AERFA)
    READRADARVALUE("RCR_DELTA", RCR_DELTAX, RCR_DELTAY, RCR_AERFA)
    READRADARVALUE("ROR_DELTA", ROR_DELTAX, ROR_DELTAY, ROR_AERFA)

    Motorspd_fac_vspd =
        PI2 * FRONT_WHELL_RADIUS * PLUS_MOTOR_RPM / WHELL_PLUAS; // 0.0043063372f;
    Rear_wheel_fac_ds = PI * WHELL_RADIUS / WHELL_PLUAS;
}

static void ParseCCPParas(std::string &jsonfile)
{
    Json::Reader jsonReader;
    Json::Value jsonRoot;
    std::ifstream ifs;

    jsonRoot.clear();
    ifs.open(jsonfile.c_str());
    if (!ifs)
    {
        log_err("open json file %s failed!\n", jsonfile.c_str());
        return;
    }

    if (!jsonReader.parse(ifs, jsonRoot))
    {
        log_err("read json file %s failed\n", jsonfile.c_str());
        ifs.close();
        return;
    }

    READJSONDOUBLE(AVM_Para_TargPos_off_dy)
    READJSONDOUBLE(AVM_Para_TargPos_off_ldx)
    READJSONDOUBLE(AVM_Para_TargPos_off_rdx)
    READJSONDOUBLE(AVM_Vert_TargPos_off_ldy)
    READJSONDOUBLE(AVM_Vert_TargPos_off_rdy)
    READJSONDOUBLE(AVM_Vert_TargPos_off_ldx)
    READJSONDOUBLE(AVM_Vert_TargPos_off_rdx)
    READJSONDOUBLE(AVM_MaxProjRx_SlotCal)
    READJSONDOUBLE(expand_slot_width)
    READJSONDOUBLE(eps_delaydt_ccp)
    READJSONDOUBLE(eps_vel_factor)
    READJSONDOUBLE(Radar_Vert_TargPos_off_dx)
    READJSONDOUBLE(Parkout_off_dx)
    READJSONDOUBLE(MIN_AVM_PARK_LEN_VERTICAL)
    READJSONDOUBLE(min_object_length)
    READJSONDOUBLE(avm_vert_slot_min_dx)
    READJSONDOUBLE(avm_vert_slot_max_dx)
    READJSONDOUBLE(big_eps_angle_gap)
    READJSONDOUBLE(avm_slotinfo_b_factor)
    READJSONDOUBLE(slot_ang_reverse_off_dy)
    READJSONDOUBLE(Avm_Min_Delay_Ms)
    READJSONDOUBLE(searching_vert_slot_off_ldy)
    READJSONDOUBLE(searching_vert_slot_off_rdy)
    READJSONDOUBLE(vehicle_travel_dist_min)
    READJSONDOUBLE(offset_curb)
    READJSONDOUBLE(offset_suspension)
    READJSONDOUBLE(max_esp_spd)
    // bool
    READJSONINT(stop_vision_slot)
    READJSONINT(Avm_CheckStop)
    READJSONINT(avm_slot_stopbar)
    READJSONINT(avm_slot_update_on_parking)
    READJSONINT(obj_of_para_slot_bottom)
    READJSONINT(is_save_log)
    READJSONINT(ultrasonic_parking_space)

    // int
    READJSONINT(min_frame_num)
    READJSONINT(a_frame_num)
    READJSONINT(b_frame_num)
    // string
    READJSONSTRING(local_ip_address)
    READJSONDOUBLE(searching_vert_slot_off_ltheta)
    READJSONDOUBLE(searching_vert_slot_off_rtheta)
    READJSONSTRING(cfg_log_path)

    ifs.close();
}

void PK_Calibration()
{
    Json::Reader jsonReader;
    Json::Value jsonRoot;
    std::ifstream ifs;

    jsonRoot.clear();
    ifs.open("vehicle.json");
    if (!ifs)
    {
        log_err("open json file failed!\n");
        return;
    }

    if (!jsonReader.parse(ifs, jsonRoot))
    {
        log_err("read json file failed\n");
        ifs.close();
        return;
    }

    if (jsonRoot.isMember("vehicle_type"))
    {
        vehicle_type = jsonRoot["vehicle_type"].asInt();
        printf("\n***********\n %s:%d --vehicle_type: %d\n*********\n ", __FILE__,
               __LINE__, vehicle_type);
    }
    else
    {
        log_err("read vehicle_type failed!\n");
        ifs.close();
        return;
    }

    if (jsonRoot.isMember("vehicle_params"))
    {
        std::string configFile = jsonRoot["vehicle_params"].asString();
        ParseVehParas(configFile);
    }
    else
    {
        log_err("read vehicle_paras failed!\n");
        ifs.close();
        return;
    }

    if (jsonRoot.isMember("ccp_params"))
    {
        std::string configFile = jsonRoot["ccp_params"].asString();
        ParseCCPParas(configFile);
    }

    ifs.close();
}

int PK_CalibrationPrint(const int index, char buff[128])
{
#define PRINTJSONVALUE(line, data)                                             \
    {                                                                          \
        if (index == line)                                                     \
        {                                                                      \
            sprintf(buff, "index %03d: %s=%06f\n", index, #data, (float)data); \
            return index + 1;                                                  \
        }                                                                      \
    }
#define PRINTJSONINT(line, data)                                      \
    {                                                                 \
        if (index == line)                                            \
        {                                                             \
            sprintf(buff, "index %03d: %s=%d\n", index, #data, data); \
            return index + 1;                                         \
        }                                                             \
    }
#define PRINTJSONSTRING(line, data)                                   \
    {                                                                 \
        if (index == line)                                            \
        {                                                             \
            sprintf(buff, "index %03d: %s=%s\n", index, #data, data); \
            return index + 1;                                         \
        }                                                             \
    }

    PRINTJSONVALUE(0, AVM_Para_TargPos_off_dy)
    PRINTJSONVALUE(1, AVM_Vert_TargPos_off_ldy)
    PRINTJSONVALUE(2, AVM_Vert_TargPos_off_rdy)
    PRINTJSONVALUE(3, VEHICLE_WID)
    PRINTJSONVALUE(4, VEHICLE_LEN)
    PRINTJSONVALUE(5, REAR_SUSPENSION)
    PRINTJSONVALUE(6, WHEEL_BASE)
    PRINTJSONVALUE(7, WHELL_RADIUS)
    PRINTJSONVALUE(8, WHELL_PLUAS)
    PRINTJSONVALUE(9, PK_EPS_ANGLE_MAX)
    PRINTJSONVALUE(10, REAR_VIEW_MIRROR_X)
    PRINTJSONVALUE(11, Rrmin)
    PRINTJSONVALUE(12, angle_number[0])
    PRINTJSONVALUE(13, radius_number[0])
    PRINTJSONVALUE(14, angle_number[1])
    PRINTJSONVALUE(15, radius_number[1])
    PRINTJSONVALUE(16, angle_number[2])
    PRINTJSONVALUE(17, radius_number[2])
    PRINTJSONVALUE(18, angle_number[3])
    PRINTJSONVALUE(19, radius_number[3])
    PRINTJSONVALUE(20, FSR_DELTAX)
    PRINTJSONVALUE(21, FSL_DELTAX)
    PRINTJSONVALUE(22, RSL_DELTAX)
    PRINTJSONVALUE(23, RSR_DELTAX)
    PRINTJSONVALUE(24, FOL_DELTAX)
    PRINTJSONVALUE(25, FCL_DELTAX)
    PRINTJSONVALUE(26, FCR_DELTAX)
    PRINTJSONVALUE(27, FOR_DELTAX)
    PRINTJSONVALUE(28, ROL_DELTAX)
    PRINTJSONVALUE(29, RCL_DELTAX)
    PRINTJSONVALUE(30, RCR_DELTAX)
    PRINTJSONVALUE(31, ROR_DELTAX)
    PRINTJSONVALUE(32, AVM_Vert_TargPos_off_ldx)
    PRINTJSONVALUE(33, AVM_Vert_TargPos_off_rdx)
    PRINTJSONVALUE(34, Avm_Min_Delay_Ms)
    PRINTJSONVALUE(35, AVM_MaxProjRx_SlotCal)
    PRINTJSONVALUE(36, AVM_Para_TargPos_off_ldx)
    PRINTJSONVALUE(37, AVM_Para_TargPos_off_rdx)
    PRINTJSONVALUE(38, expand_slot_width)
    PRINTJSONINT(39, Avm_CheckStop)
    PRINTJSONVALUE(40, eps_delaydt_ccp)
    PRINTJSONVALUE(41, eps_vel_factor)
    PRINTJSONINT(42, stop_vision_slot)
    PRINTJSONVALUE(43, Radar_Vert_TargPos_off_dx)
    PRINTJSONVALUE(44, Parkout_off_dx)
    PRINTJSONVALUE(45, MIN_AVM_PARK_LEN_VERTICAL)
    PRINTJSONVALUE(46, min_object_length)
    PRINTJSONVALUE(47, searching_vert_slot_off_ltheta)
    PRINTJSONVALUE(48, searching_vert_slot_off_rtheta)
    PRINTJSONSTRING(49, cfg_log_path)

    return 0;
}

static float EpsToRad(const float epsAngle, const float dir)
{
    int index = -1;
    if (epsAngle > 0.5)
    {
        index = (dir > 0) ? 0 : 1;
    }
    else if (epsAngle < -0.5)
    {
        index = (dir > 0) ? 2 : 3;
    }
    else
    {
        return 0.0;
    }

    int count = angle_number[index] - 1;
    if (count < 0)
    {
        return 0.0;
    }

    float angle = fabs(epsAngle);
    if (angle >= angle_list[index][count])
    {
        return radius_list[index][count];
    }
    if (angle < angle_list[index][0])
    {
        return 0.0;
    }

    if (index >= 0)
    {
        return InterPolationOneDim(angle_list[index], radius_list[index], angle,
                                   angle_number[index]);
    }

    return 0.0;
}

static float RadToEps(const float radius, const float dir)
{
    int index = -1;
    if (sign(radius) > 0.5)
    {
        index = (dir > 0) ? 0 : 1;
    }
    else if (sign(radius) < -0.5)
    {
        index = (dir > 0) ? 2 : 3;
    }
    else
    {
        return 0.0;
    }

    int count = radius_number[index] - 1;
    if (count < 0)
    {
        return 0.0;
    }

    float temp = fabs(radius);
    if (temp > radius_list[index][0])
    {
        return 0.0;
    }
    if (temp < radius_list[index][count])
    {
        return PK_EPS_ANGLE_MAX;
    }

    if (index >= 0)
    {
        return InterPolationOneDim(radius_list[index], angle_list[index], temp,
                                   radius_number[index]);
    }

    return 0.0;
}

float PK_Rr_to_EPS(const float Rr, const float spd)
{
    if (fabs(Rr) < 0.01)
    {
        return 0.0;
    }
    float angle = RadToEps(Rr, sign(spd));
    return sign(Rr) * angle;
}

float PK_Rou_to_EPS(const float rou, const float spd)
{
    if (fabs(rou) < ZERO_FLOAT)
    {
        return 0;
    }

    float radius = 1.0 / rou;
    float angle  = RadToEps(radius, sign(spd));
    return sign(rou) * angle;
}

float PK_EPS_to_Rou(const float angle, const float spd)
{
    if (fabs(angle) < 1)
    {
        return 0;
    }

    float radius = EpsToRad(angle, sign(spd));
    if (radius < ZERO_FLOAT)
    {
        return 0;
    }
    return sign(angle) / radius;
}

int PK_VehConfigType() { return vehicle_type; }

int PK_Calibration_GetDebugValue(const char *key, char buff[64])
{
    Json::Reader jsonReader;
    Json::Value jsonRoot;
    std::ifstream ifs;

    jsonRoot.clear();
    ifs.open("vehicle.json");
    if (!ifs)
    {
        log_err("open json file failed!\n");
        return 0;
    }

    if (!jsonReader.parse(ifs, jsonRoot))
    {
        log_err("read json file failed\n");
        ifs.close();
        return 0;
    }

    std::string configFile;

    if (jsonRoot.isMember("ccp_params"))
    {
        configFile = jsonRoot["ccp_params"].asString();
    }
    ifs.close();

    jsonRoot.clear();
    ifs.open(configFile.c_str());
    if (!ifs)
    {
        log_err("open json file %s failed!\n", configFile.c_str());
        return 0;
    }

    if (!jsonReader.parse(ifs, jsonRoot))
    {
        log_err("read json file %s failed\n", configFile.c_str());
        ifs.close();
        return 0;
    }

    int result = 0;
    if (jsonRoot.isMember(key))
    {
        if (jsonRoot[key].isString())
        {
            std::string value = jsonRoot[key].asString();
            result            = snprintf(buff, 63, "%s", value.c_str());
        }
        else if (jsonRoot[key].isInt())
        {
            int value = jsonRoot[key].asInt();
            result    = snprintf(buff, 63, "%d", value);
        }
        else if (jsonRoot[key].isDouble())
        {
            double value = jsonRoot[key].asDouble();
            result       = snprintf(buff, 63, "%f", value);
        }
        else
        {
            ;
        }
    }
    return result;
}
