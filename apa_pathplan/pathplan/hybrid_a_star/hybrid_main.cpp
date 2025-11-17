#include "GridMapGenerator.h" // 引入 GridMapGenerator 类
#include "hybrid_a_star_flow.h"
#include "hybrid_data.h"
#include "Rte_Func.h"
#include "MathFunc.h"
#include "json/json.h"
#include "Pathplan_Hybrid.h"

// 车辆参数
double cfg_vehicle_length         = 4.998; // 4.998
double cfg_vehicle_width          = 1.90;  // 1.915
double cfg_vehicle_rear_axle_dist = 1.066; // 1.066
double cfg_wheel_base             = 2.982;

double cfg_map_resolution              = 0.1;
double cfg_grid_resolution             = 1.0;
double cfg_steering_angle              = 26.00; // 最小转弯半径按照5.6计算, 30偏大
double cfg_steering_angle_discrete_num = 4.0;
double cfg_segment_length              = 1.0;
double cfg_segment_length_discrete_num = 3.0; // 6;
double cfg_steering_penalty            = 1.0;
double cfg_steering_change_penalty     = 1.5;
double cfg_reversing_penalty           = 1.8;
double cfg_shot_distance               = 0.3;
double cfg_grid_size_phi               = 72;

class Hybrid_Plan_Data
{
    int curSlot;
    std::string input_json_str;
    int init_replan;
    VectorVec5d *resplan_result;
    std::vector<Traject_Node> old_trajs;
    std::string namePrex;

    enum
    {
        replan_start,
        replan_done,
        replan_end
    };

  public:
    bool Start(const std::string jsonstr, const std::vector<Traject_Node> &trajs,
               int slotId)
    {
        if (init_replan != replan_end)
        {
            // printf("hybrid busy cur slot is %d state %d\n", curSlot, init_replan);
            return false;
        }

        printf("hybrid start cur slot is %d\n", slotId);
        resplan_result = nullptr;
        init_replan    = replan_start;
        input_json_str.assign(jsonstr);
        old_trajs = trajs;
        curSlot   = slotId;

        struct tm absStamp;
        RTE_BSW_Get_AbsStamp(&absStamp);
        struct timeval stamp;
        gettimeofday(&stamp, NULL);
        namePrex = "./hybridlog/";
        namePrex = namePrex + std::to_string(absStamp.tm_year) + "_";
        namePrex = namePrex + std::to_string(absStamp.tm_mon) + "_";
        namePrex = namePrex + std::to_string(absStamp.tm_mday) + "_";
        namePrex = namePrex + std::to_string(absStamp.tm_hour) + "_";
        namePrex = namePrex + std::to_string(absStamp.tm_min) + "_";
        namePrex = namePrex + std::to_string(stamp.tv_sec) + "_";
        namePrex = namePrex + std::to_string(stamp.tv_usec / 1000);
        return true;
    }

    int DoneSlot()
    {
        if (init_replan == replan_done)
        {
            return curSlot;
        }
        return 0;
    }

    bool IsEnd(const int slot) { return (init_replan == replan_end); }

    void Init()
    {
        // printf("hybrid slot %d init!\n", curSlot);
        input_json_str.clear();
        resplan_result = nullptr;
        init_replan    = replan_end;
    }

    void End(VectorVec5d *result)
    {
        input_json_str.clear();
        init_replan    = replan_done;
        resplan_result = result;
        printf("hybrid slot %d done! %p data len %ld info :%s\n", curSlot, resplan_result,
               resplan_result->size(), namePrex.c_str());
    }

    const std::string &GetInputJsonStr() { return input_json_str; }

    const std::string &GetNamePrex() { return namePrex; }

    int GetResult(const int slot, const float target[3],
                  std::vector<Traject_Node> &result, std::vector<Traject_Node> &trajs);

    static Hybrid_Plan_Data *GetHybridDataRef();

  private:
    ~Hybrid_Plan_Data() {};

    Hybrid_Plan_Data()
    {
        init_replan = replan_end;
        input_json_str.clear();
        resplan_result = nullptr;
        curSlot        = 0;
    };

    Hybrid_Plan_Data(const Hybrid_Plan_Data &single) = delete;

    const Hybrid_Plan_Data &operator=(const Hybrid_Plan_Data &single) = delete;
};

int Hybrid_Plan_Data::GetResult(const int slotId, const float target[3],
                                std::vector<Traject_Node> &result,
                                std::vector<Traject_Node> &trajs)
{
    if (slotId != curSlot)
    {
        return -1;
    }

    if (init_replan == replan_start)
    {
        // printf("hybrid result running! cur slot is %d!\n", curSlot);
        result.clear();
    }
    else if (init_replan == replan_done)
    {
        Traject_Node lastTra;
        auto number = old_trajs.size();
        lastTra     = old_trajs.at(number - 1);

        float trajEnd[3];
        PK_Get_Path_EndPos((float *)&lastTra, trajEnd);
        float dist = Cal_Dis_Pt2Pt(target, trajEnd);
        if (dist < 0.15)
        {
            for (int i = 0; i < resplan_result->size(); i++)
            {
                auto item       = resplan_result->at(i);
                lastTra.traj[0] = item(0);
                lastTra.traj[1] = item(1);
                lastTra.traj[2] = item(2);
                lastTra.traj[3] = item(3);
                lastTra.traj[4] = item(4);
                result.push_back(lastTra);
            }
            trajs = old_trajs;
        }

        resplan_result = nullptr;
        init_replan    = replan_end;
        printf("hybrid result end! cur slot is %d!\n", curSlot);
    }

    return curSlot;
}

Hybrid_Plan_Data *Hybrid_Plan_Data::GetHybridDataRef()
{
    static Hybrid_Plan_Data s_refData;
    return &s_refData;
}

static void ParseHybridCfg()
{
    const char *finename = "hybrid.json";
    Json::Reader jsonReader;
    Json::Value jsonRoot;
    std::ifstream ifs;

    jsonRoot.clear();
    ifs.open(finename);
    if (!ifs)
    {
        std::cout << "open json file failed! " << finename << std::endl;
        return;
    }

    if (!jsonReader.parse(ifs, jsonRoot))
    {
        std::cout << "read json file failed! " << finename << std::endl;
        ifs.close();
        return;
    }

#define read_json_doubleitem(data)                            \
    {                                                         \
        if (jsonRoot.isMember(#data))                         \
        {                                                     \
            data = jsonRoot[#data].asDouble();                \
            std::cout << #data << " = " << data << std::endl; \
        }                                                     \
    }

    read_json_doubleitem(cfg_map_resolution);              //= 0.1;
    read_json_doubleitem(cfg_grid_resolution);             // 1.0;
    read_json_doubleitem(cfg_steering_angle);              // 26.00;
    read_json_doubleitem(cfg_steering_angle_discrete_num); // 4.0;
    read_json_doubleitem(cfg_segment_length);              // 1.0;
    read_json_doubleitem(cfg_segment_length_discrete_num); // 3.0; // 6;
    read_json_doubleitem(cfg_steering_penalty);            // 1.0;
    read_json_doubleitem(cfg_steering_change_penalty);     // 1.5;
    read_json_doubleitem(cfg_reversing_penalty);           // 1.8;
    read_json_doubleitem(cfg_shot_distance);               // 0.3;
    read_json_doubleitem(cfg_grid_size_phi);
    read_json_doubleitem(cfg_vehicle_length);
    read_json_doubleitem(cfg_vehicle_width);
    read_json_doubleitem(cfg_vehicle_rear_axle_dist);
    read_json_doubleitem(cfg_wheel_base);
}

void *RunFunction(void *args)
{
    static bool init_cfg = false;
    if (!init_cfg)
    {
        ParseHybridCfg();
        init_cfg = true;
    }

    // 创建 GridMapGenerator 对象并生成地图

    double x_min = -10.0, x_max = 10.0, y_min = -10.0, y_max = 10.0;

    auto data = Hybrid_Plan_Data::GetHybridDataRef();
    GridMapGenerator gridMapGenerator(cfg_map_resolution, x_min, x_max, y_min, y_max,
                                      data->GetInputJsonStr(), data->GetNamePrex());
    OccupancyGrid grid; // 假设 OccupancyGrid 是用于存储生成地图的结构体

    gridMapGenerator.generate(grid);

    const auto &start = gridMapGenerator.getStart();
    const auto &end   = gridMapGenerator.getEnd();

    // 创建 HybridAStarFlow 对象
    HybridAStarFlow hybridAStarFlow;
    std::string nameIdPrex(data->GetNamePrex());
    nameIdPrex = nameIdPrex + "_" + std::to_string(gridMapGenerator.getGridId());

    struct timeval stamp1, stamp2;
    gettimeofday(&stamp1, NULL);
    VectorVec5d *result = hybridAStarFlow.Run(
        grid, cfg_map_resolution, cfg_grid_resolution,
        Vec3d(start[0], start[1], start[2]), Vec3d(end[0], end[1], end[2]), nameIdPrex);
    gettimeofday(&stamp2, NULL);

    std::cout << "HybridAStarFlow Run time2: " << stamp2.tv_sec - stamp1.tv_sec << "s "
              << stamp2.tv_usec - stamp1.tv_usec << "us" << std::endl;

    data->End(result);
    return nullptr;
}

int HybridAStarRun(const char *jsonstr, const std::vector<Traject_Node> &trajs, int slot)
{
    // auto data = Hybrid_Plan_Data::GetHybridDataRef();
    // if (data->Start(jsonstr, trajs, slot))
    // {
    //     pthread_t thread;
    //     pthread_create(&thread, nullptr, RunFunction, (void *)nullptr);
    //     pthread_detach(thread);
    //     return 0;
    // }
    // else
    // {
    //     return -1;
    // }
}

static void FilterRefPoint(std::vector<Traject_Node> &result)
{
    std::vector<Traject_Node> temp;
    temp = result;
    result.clear();
    result.push_back(temp[0]);

    // 删除双边点
    while (1)
    {
        for (size_t i = 1; i < temp.size(); i++)
        {
            if (i < temp.size() - 1)
            {
                float dist1 = Cal_Dis_Pt2Pt((float *)&temp[i], (float *)&temp[i - 1]);
                float dist2 = Cal_Dis_Pt2Pt((float *)&temp[i], (float *)&temp[i + 1]);
                if (dist1 < 0.02f && dist2 < 0.02f)
                {
                    result.pop_back();
                    result.push_back(temp[i]);
                    i = i + 1;
                    continue;
                }
            }

            result.push_back(temp[i]);
        }

        // 如果没有删除掉的点，则退出
        if (result.size() == temp.size())
        {
            break;
        }

        // 重新检查
        temp = result;
        result.clear();
        result.push_back(temp[0]);
    }

    // 删除单边点
    temp = result;
    result.clear();
    result.push_back(temp[0]);
    while (1)
    {
        for (size_t i = 1; i < temp.size(); i++)
        {
            float dist1 = Cal_Dis_Pt2Pt((float *)&temp[i], (float *)&result.back());
            if (dist1 > 0.03f || i == temp.size() - 1)
            {
                result.push_back(temp[i]);
            }
            else
            {
                ;
            }
        }

        if (result.size() == temp.size())
        {
            break;
        }
        temp = result;
        result.clear();
        result.push_back(temp[0]);
    }
}

static void NormalVector(const float pt1[2], const float pt2[2], float result[2])
{
    float x   = pt2[0] - pt1[0];
    float y   = pt2[1] - pt1[1];
    float len = sqrtf(x * x + y * y);
    result[0] = x / len;
    result[1] = y / len;
}

static void RefPointToTraj(std::vector<Traject_Node> &result,
                           std::vector<int> &topPointIdx)
{
    topPointIdx.clear();
    FilterRefPoint(result);

    topPointIdx.push_back(0);
    for (size_t i = 1; i < result.size() - 1; i++)
    {
        float v1[2], v2[2];
        NormalVector((float *)&result[i], (float *)&result[i - 1], v1);
        NormalVector((float *)&result[i], (float *)&result[i + 1], v2);
        float crossv = v1[0] * v2[0] + v1[1] * v2[1];
        if (crossv > 0.5f)
        {
            topPointIdx.push_back(i);
        }
    }
    topPointIdx.push_back(result.size() - 1);
}

void HybridAStarResult(const int slot, const float target[3],
                       std::vector<Traject_Node> &result,
                       std::vector<Traject_Node> &trajs)
{
    auto data = Hybrid_Plan_Data::GetHybridDataRef();
    std::vector<Traject_Node> refPoint;
    auto curSlot = data->GetResult(slot, target, refPoint, trajs);
    if (curSlot != slot || refPoint.size() <= 0)
    {
        return;
    }

    // printf("HybridAStarResult %d Point num %ld\n", slot, refPoint.size());
    std::vector<int> topPoint;
    RefPointToTraj(refPoint, topPoint);

#define Is_0(value)           ((value) > -0.5f && (value) < 0.5f)
#define Is_1(value)           ((value) > 0.5f && (value) < 1.5f)
#define Is_3(value)           ((value) > 2.5f && (value) < 3.5f)
#define Is_NE(value1, value2) (fabs(value1 - value2) > 0.1f)

    float dir = 0.0f;
    for (int i = 0; i < (int)refPoint.size() - 1; i++)
    {
        Traject_Node temp;
        PK_CopyTra(temp.traj, refPoint[i].traj);
        float radius = HybridGetredius(refPoint[i + 1].traj[3]);
        float theta  = fabs(Round_PI(refPoint[i + 1].traj[2] - refPoint[i].traj[2]));
        float length = fabs(theta * radius);
        if (fabs(radius) < 1)
        {
            length = Cal_Dis_Pt2Pt(refPoint[i].traj, refPoint[i + 1].traj);
        }

        temp.traj[4] = radius;

        // FORWARD = 0, BACKWARD = 1, NO = 3
        // 3 -- > 1
        if (Is_NE(refPoint[i].traj[4], refPoint[i + 1].traj[4]))
        {
            if (Is_0(refPoint[i + 1].traj[4]))
            {
                dir = 1.0f;
            }
            if (Is_1(refPoint[i + 1].traj[4]))
            {
                dir = -1.0f;
            }
            if (Is_3(refPoint[i + 1].traj[4]))
            {
                float rx = Project_PosTo1st_rx(refPoint[i].traj, refPoint[i + 1].traj);
                dir      = sign(rx);
            }
        }
        else
        {
            if (Is_3(refPoint[i + 1].traj[4]))
            {
                float rx = Project_PosTo1st_rx(refPoint[i].traj, refPoint[i + 1].traj);
                dir      = sign(rx);
            }
        }

        temp.traj[3] = length * dir;
        result.push_back(temp);
    }

    std::ofstream outfile2("out2.txt");
    for (int i = 0; i < (int)result.size(); i++)
    {
        float endPos[3];
        PK_Get_Path_EndPos(result[i].traj, endPos);

        outfile2 << result[i].traj[0] << ",";
        outfile2 << result[i].traj[1] << ",";
        outfile2 << result[i].traj[2] << ",";
        outfile2 << result[i].traj[3] << ",";
        outfile2 << result[i].traj[4] << std::endl;
        outfile2 << endPos[0] << ",";
        outfile2 << endPos[1] << ",";
        outfile2 << endPos[2] << std::endl;
    }
    outfile2.close();
}

void HybridAClearResult(const int *slot, int slotNum)
{
    auto data   = Hybrid_Plan_Data::GetHybridDataRef();
    auto slotId = data->DoneSlot();
    if (slotId <= 0)
    {
        return;
    }

    for (int i = 0; i < slotNum; i++)
    {
        if (slotId == slot[i])
        {
            return;
        }
    }

    data->Init();
}

bool HybridASlot(const int slotId)
{
    auto data = Hybrid_Plan_Data::GetHybridDataRef();
    return (data->IsEnd(slotId) == false);
}

void HybridAStar_UnitTest()
{
    // std::ifstream ifs("1694042620_15_36326_input.json");
    // std::string content((std::istreambuf_iterator<char>(ifs) ),
    // (std::istreambuf_iterator<char>() ) ); std::string content("{\n\t\"curpos\" :
    // \n\t[\n\t\t-36.603176116943359,\n\t\t5.2494382858276367,\n\t\t4.3014316558837891,\n\t\t191.4520263671875\n\t],\n\t\"objs\"
    // :
    // \n\t[\n\t\t[\n\t\t\t-33.967258453369141,\n\t\t\t6.4346628189086914,\n\t\t\t-33.948566436767578,\n\t\t\t5.185910701751709\n\t\t],\n\t\t[\n\t\t\t-34.248603820800781,\n\t\t\t6.5308904647827148,\n\t\t\t-34.184383392333984,\n\t\t\t5.2324771881103516\n\t\t],\n\t\t[\n\t\t\t-33.773563385009766,\n\t\t\t23.158424377441406,\n\t\t\t-34.366813659667969,\n\t\t\t20.510488510131836\n\t\t],\n\t\t[\n\t\t\t-34.523754119873047,\n\t\t\t21.166376113891602,\n\t\t\t-34.162639617919922,\n\t\t\t19.439844131469727\n\t\t],\n\t\t[\n\t\t\t-34.239963531494141,\n\t\t\t14.747525215148926,\n\t\t\t-34.158496856689453,\n\t\t\t13.362213134765625\n\t\t],\n\t\t[\n\t\t\t-40.659595489501953,\n\t\t\t4.8344974517822266,\n\t\t\t-40.449047088623047,\n\t\t\t3.686237096786499\n\t\t],\n\t\t[\n\t\t\t-40.448722839355469,\n\t\t\t4.0402979850769043,\n\t\t\t-40.361106872558594,\n\t\t\t2.7565891742706299\n\t\t],\n\t\t[\n\t\t\t-41.781528472900391,\n\t\t\t14.522336006164551,\n\t\t\t-41.678829193115234,\n\t\t\t13.504001617431641\n\t\t],\n\t\t[\n\t\t\t-41.8350830078125,\n\t\t\t14.027338981628418,\n\t\t\t-41.940990447998047,\n\t\t\t12.753484725952148\n\t\t],\n\t\t[\n\t\t\t-41.573066711425781,\n\t\t\t5.9023947715759277,\n\t\t\t-41.596290588378906,\n\t\t\t4.812136173248291\n\t\t],\n\t\t[\n\t\t\t-41.041183471679688,\n\t\t\t5.5490117073059082,\n\t\t\t-40.549240112304688,\n\t\t\t4.5534467697143555\n\t\t],\n\t\t[\n\t\t\t-36.371288299560547,\n\t\t\t-1.4865670204162598,\n\t\t\t-35.549831390380859,\n\t\t\t-1.410214900970459\n\t\t],\n\t\t[\n\t\t\t-36.968715667724609,\n\t\t\t-1.5420956611633301,\n\t\t\t-37.790172576904297,\n\t\t\t-1.6184477806091309\n\t\t],\n\t\t[\n\t\t\t-37.067100524902344,\n\t\t\t-1.4254899024963379,\n\t\t\t-37.890037536621094,\n\t\t\t-1.4838032722473145\n\t\t],\n\t\t[\n\t\t\t-36.53863525390625,\n\t\t\t-1.4236392974853516,\n\t\t\t-35.714797973632812,\n\t\t\t-1.3798370361328125\n\t\t],\n\t\t[\n\t\t\t-37.231147766113281,\n\t\t\t-1.440650463104248,\n\t\t\t-38.055747985839844,\n\t\t\t-1.4663763046264648\n\t\t],\n\t\t[\n\t\t\t-36.711399078369141,\n\t\t\t-1.3900651931762695,\n\t\t\t-35.886466979980469,\n\t\t\t-1.3794913291931152\n\t\t],\n\t\t[\n\t\t\t-38.929046630859375,\n\t\t\t2.9322457313537598,\n\t\t\t-38.533851623535156,\n\t\t\t-5.0579872131347656\n\t\t],\n\t\t[\n\t\t\t-30.800434112548828,\n\t\t\t13.434658050537109,\n\t\t\t-30.65223503112793,\n\t\t\t10.438320159912109\n\t\t],\n\t\t[\n\t\t\t-30.65223503112793,\n\t\t\t10.438320159912109,\n\t\t\t-28.999258041381836,\n\t\t\t10.520074844360352\n\t\t],\n\t\t[\n\t\t\t-28.999258041381836,\n\t\t\t10.520074844360352,\n\t\t\t-28.805364608764648,\n\t\t\t6.5998687744140625\n\t\t],\n\t\t[\n\t\t\t-28.805364608764648,\n\t\t\t6.5998687744140625,\n\t\t\t-34.238719940185547,\n\t\t\t6.3311347961425781\n\t\t],\n\t\t[\n\t\t\t-34.238719940185547,\n\t\t\t6.3311347961425781,\n\t\t\t-34.090526580810547,\n\t\t\t3.3347978591918945\n\t\t]\n\t],\n\t\"slotId\"
    // : 20731,\n\t\"slottarg\" :
    // \n\t[\n\t\t-30.102512359619141,\n\t\t7.7246627807617188,\n\t\t-3.0921730995178223\n\t]\n}\n");
    std::string content(
        "{\n\t\"curpos\" : "
        "\n\t[\n\t\t8.4156122207641602,\n\t\t0.0020409207791090012,\n\t\t0."
        "0013156011700630188,\n\t\t13.835155487060547\n\t],\n\t\"objs\" : "
        "\n\t[\n\t\t[\n\t\t\t1.2479910850524902,\n\t\t\t4.2887396812438965,\n\t\t\t2."
        "8835077285766602,\n\t\t\t4.2783041000366211\n\t\t],\n\t\t[\n\t\t\t9."
        "8331165313720703,\n\t\t\t4.2461438179016113,\n\t\t\t11.329957962036133,"
        "\n\t\t\t4.2090363502502441\n\t\t],\n\t\t[\n\t\t\t0.83327674865722656,\n\t\t\t-3."
        "3924522399902344,\n\t\t\t3.0273358821868896,\n\t\t\t-3.2276248931884766\n\t\t],"
        "\n\t\t[\n\t\t\t10.167728424072266,\n\t\t\t-3.3387694358825684,\n\t\t\t11."
        "576462745666504,\n\t\t\t-3.3845610618591309\n\t\t],\n\t\t[\n\t\t\t11."
        "294988632202148,\n\t\t\t3.7249760627746582,\n\t\t\t19.294187545776367,\n\t\t\t3."
        "8382449150085449\n\t\t],\n\t\t[\n\t\t\t9.2225074768066406,\n\t\t\t-5."
        "8903312683105469,\n\t\t\t12.222207069396973,\n\t\t\t-5.8478550910949707\n\t\t],"
        "\n\t\t[\n\t\t\t12.222207069396973,\n\t\t\t-5.8478550910949707,\n\t\t\t12."
        "245637893676758,\n\t\t\t-7.5026884078979492\n\t\t],\n\t\t[\n\t\t\t12."
        "245637893676758,\n\t\t\t-7.5026884078979492,\n\t\t\t16.170242309570312,\n\t\t\t-"
        "7.4471158981323242\n\t\t],\n\t\t[\n\t\t\t16.170242309570312,\n\t\t\t-7."
        "4471158981323242,\n\t\t\t16.093221664428711,\n\t\t\t-2.0076627731323242\n\t\t],"
        "\n\t\t[\n\t\t\t16.093221664428711,\n\t\t\t-2.0076627731323242,\n\t\t\t19."
        "092920303344727,\n\t\t\t-1.9651875495910645\n\t\t]\n\t],\n\t\"slotId\" : "
        "45616,\n\t\"slottarg\" : "
        "\n\t[\n\t\t14.965307235717773,\n\t\t-6.2240548133850098,\n\t\t1."
        "5849554538726807\n\t]\n}\n");
    // std::string content("{\n\t\"curpos\" :
    // \n\t[\n\t\t15.634418487548828,\n\t\t1.1364567279815674,\n\t\t0.07393038272857666,\n\t\t32.340316772460938\n\t],\n\t\"objs\"
    // :
    // \n\t[\n\t\t[\n\t\t\t2.9107818603515625,\n\t\t\t3.2731583118438721,\n\t\t\t3.9086768627166748,\n\t\t\t3.3000710010528564\n\t\t],\n\t\t[\n\t\t\t6.1915421485900879,\n\t\t\t3.7892498970031738,\n\t\t\t7.6257338523864746,\n\t\t\t3.4907157421112061\n\t\t],\n\t\t[\n\t\t\t10.605117797851562,\n\t\t\t3.4818475246429443,\n\t\t\t11.892152786254883,\n\t\t\t3.4586682319641113\n\t\t],\n\t\t[\n\t\t\t17.831607818603516,\n\t\t\t4.4131793975830078,\n\t\t\t18.680088043212891,\n\t\t\t3.9427919387817383\n\t\t],\n\t\t[\n\t\t\t2.9763152599334717,\n\t\t\t-4.316159725189209,\n\t\t\t4.0675544738769531,\n\t\t\t-4.4299988746643066\n\t\t],\n\t\t[\n\t\t\t19.207265853881836,\n\t\t\t1.8142718076705933,\n\t\t\t19.123163223266602,\n\t\t\t2.6349737644195557\n\t\t],\n\t\t[\n\t\t\t19.2684326171875,\n\t\t\t1.2173976898193359,\n\t\t\t19.352535247802734,\n\t\t\t0.39669567346572876\n\t\t],\n\t\t[\n\t\t\t19.153825759887695,\n\t\t\t1.1102170944213867,\n\t\t\t19.217937469482422,\n\t\t\t0.2877119779586792\n\t\t],\n\t\t[\n\t\t\t19.089616775512695,\n\t\t\t1.7049367427825928,\n\t\t\t19.025974273681641,\n\t\t\t2.5274782180786133\n\t\t],\n\t\t[\n\t\t\t18.422098159790039,\n\t\t\t3.6856684684753418,\n\t\t\t26.415897369384766,\n\t\t\t4.0005850791931152\n\t\t],\n\t\t[\n\t\t\t19.257625579833984,\n\t\t\t-0.60066723823547363,\n\t\t\t18.962173461914062,\n\t\t\t3.3884062767028809\n\t\t],\n\t\t[\n\t\t\t9.177515983581543,\n\t\t\t-4.1901617050170898,\n\t\t\t12.175189971923828,\n\t\t\t-4.0720672607421875\n\t\t],\n\t\t[\n\t\t\t12.175189971923828,\n\t\t\t-4.0720672607421875,\n\t\t\t12.337944030761719,\n\t\t\t-8.2034120559692383\n\t\t],\n\t\t[\n\t\t\t12.337944030761719,\n\t\t\t-8.2034120559692383,\n\t\t\t16.486263275146484,\n\t\t\t-8.0399894714355469\n\t\t],\n\t\t[\n\t\t\t16.486263275146484,\n\t\t\t-8.0399894714355469,\n\t\t\t16.421110153198242,\n\t\t\t-6.3862709999084473\n\t\t],\n\t\t[\n\t\t\t16.421110153198242,\n\t\t\t-6.3862709999084473,\n\t\t\t19.418785095214844,\n\t\t\t-6.2681770324707031\n\t\t]\n\t],\n\t\"slotId\"
    // : 43615,\n\t\"slottarg\" :
    // \n\t[\n\t\t13.65211009979248,\n\t\t-6.9106783866882324,\n\t\t1.6101710796356201\n\t]\n}\n");
    auto data = Hybrid_Plan_Data::GetHybridDataRef();
    std::vector<Traject_Node> trajs;
    data->Start(content, trajs, 1);
    RunFunction(nullptr);
}

void HybridAStar_UnitTest1()
{
    std::ifstream ifs("2025_05_16_15_47_1747476625_738_17609_input.json");
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        (std::istreambuf_iterator<char>()));
    auto data = Hybrid_Plan_Data::GetHybridDataRef();
    std::vector<Traject_Node> trajs;
    data->Start(content, trajs, 1);
    RunFunction(nullptr);
}
