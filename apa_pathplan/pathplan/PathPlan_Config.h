#ifndef _PATHPLAN_CONFIG_H_
#define _PATHPLAN_CONFIG_H_
#include "Rte_Types.h"

class PathPlanCfg
{
  private:
    uint8_t startMagic[4];

  public:
    uint8_t amvobj_check;
    uint8_t plan_front_slot;

  private:
    uint8_t endMagic[4];

  public:
    // 获取单例实例的静态方法
    static PathPlanCfg *getInstance()
    {
        static PathPlanCfg instance;
        return &instance;
    }

    bool CheckCfgData()
    {
        return (startMagic[0] == 'P' && startMagic[1] == 'P' && startMagic[2] == 'C' &&
                startMagic[3] == 'D' && endMagic[0] == 'D' && endMagic[1] == 'D' &&
                endMagic[2] == 'C' && endMagic[3] == 'D');
    }

    // 删除拷贝构造函数和赋值操作符以防止拷贝
    PathPlanCfg(const PathPlanCfg &)            = delete;
    PathPlanCfg &operator=(const PathPlanCfg &) = delete;

  private:
    // 私有构造函数以防止外部实例化
    PathPlanCfg()
    {
        startMagic[0]   = 'P';
        startMagic[1]   = 'P';
        startMagic[2]   = 'C';
        startMagic[3]   = 'D';
        amvobj_check    = 0;
        plan_front_slot = 1;
        endMagic[0]     = 'D';
        endMagic[1]     = 'D';
        endMagic[2]     = 'C';
        endMagic[3]     = 'D';
    }
    // 私有析构函数以防止外部删除实例
    ~PathPlanCfg() { ; }
};
class PathPlanConstCfg
{
  public:
    // 规划路径延长长度（m），用于保证轨迹末端有余量
    inline const static float para_plan_extlen = 0.6f;
    // 平行泊车车位：CD（车尾到车位中心）最小距离（m）
    inline const static float para_cd_minswell = 0.25f;
    // 有效障碍物长度（m），小于该长度的障碍物可忽略
    inline const static float valid_obj_len = 0.35f;
    // 路径规划周期（ms），300ms 即约 3Hz
    inline const static uint32_t pathplan_period = 300;
    // 路径分段误差（m），用于删除小于该误差的轨迹段
    inline const static float div_err = 0.12f;
    // 终点转向精度误差（m），用于精确到终点的轨迹分段
    inline const static float div_err_ry_fin = 0.06f;
    // 终点角度精度误差（rad），终点需要更高精度
    inline const static float div_err_theta_fin = PI_RAD * 1.2f;
    // 起始倒车距离（m），-1.0 表示未使用
    inline const static float start_revert_dist = -1.0f;
    // 短路径长度阈值（m），小于该值的路径可以认为很短
    inline const static float short_path_len = 0.30f;
};

#define PATH_ITEM_LEN (sizeof(float) * TRAJITEM_LEN)

#define FS_OBJ_ARR_NUM (SF_OBJ_NUM * 2) //  the defined obj array num

#define EXPAND_SLOTOBJ_CB_VERT 2.0

#define EXPAND_SLOTOBJ_DE_VERT 2.0

#define SAFE_OBJ_SWELL 0.35

#define OBJ_SWELL 0.30

#define EXPLORE_DIST_MAX 3.5

#define EXPLORE_DIST_MIN 0.26

#define PATH_LAST_STRAIGHT_MIN 0.5

#define PATH_LENGTH_MAX 12.0f

#endif
