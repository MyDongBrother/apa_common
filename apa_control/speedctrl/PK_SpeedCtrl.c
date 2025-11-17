#include "PK_SpeedCtrl.h"
#include "Rte.h"
#include "PK_Calibration.h"

// 2016年3月16日：修改了起步准备的扭矩， 完全copy控制组的算法，
// 这里也没有考虑是不是在坡道上， 如果有坡道信息， 可能要考虑如何使用了
// 另外修改了关电机的情况， 这个只是低速泊车， 我想正常情况下扭矩不大， 或者说扭矩大时，
// 变化大些， 小时， 我就变化小些， 让关机有个过程即可
#define IsZeroSpeed(x) (fabsf(x) < 0.01f)
#define IsPosSpeed(x)  ((x) >= 0.01f)
#define IsNegSpeed(x)  ((x) <= -0.01f)

static PK_SpeedCtl_StartUpType m_startup       = {0, 0, 0, 0}; //
static PK_SpeedCtl_StoppingType m_stopping     = {0, 0, 0, 0, 0, 0};
static PK_SpeedCtl_SemiManualType m_semimanual = {0};
static PK_SpeedCtl_SpdControlType m_spdcontrol; // 2016/12/13   这里不要给值吗?
static PK_SpeedCtl_ControlParamType m_controlParam = {0, 0};
static PK_SpeedCtl_MODELPARAM m_modelParam = {640,   7.9f, 0.32f, 0.5f,   1.29f, 2.84f,
                                              0.33f, 1550, 9.8f,  0.014f, 0.02f, 0};
static int SpdControl_SpeedTask(float m_targetSpeed, float realSpeed, float measuredangle,
                                int drivemode, boolean BrakeSt, boolean epb_state,
                                int epb_waiting_time, float *torque, float *brake,
                                uint8 *EpbCtl, int *abnormal);
float LdFilterCal(float input, float *num, float *den, float *x, float *y);
void SemiManualInit(float pre_torque);
void SemiManualAction(float v, float *torque, float *pressure, uint8 *EpbCtl);
void StartUpInit(float targetSpeed);
float StartUpOutput(int *flag, float targetSpeed);
void StoppingInit(float pre_torque);
void StoppingAction(float v, float *torque, float *pressure, uint8 *EpbCtl);
void FullManualAction(float v, float *torque, float *pressure, uint8 *EpbCtl);
void ControllerInit(float speed_filtered, float pre_tor);
void GetCtrlParameters(float targetSpeed, float speed, float measuredangle);
void longitudinalControl(float speed_filtered);
float slidingModeControlForSpeed_nomotordyn(float speed);
float sat(float in, float factor);
void estimator(float speed);

// float today_realSpeed = 0;
// float today_Loc_speed = 0;
PK_ModuleStateType ModuleState_SpeedCtrl;
float d_count              = 0; // 这是一个危险的计数器
int danger                 = 0; // 0表示不是地锁停车， 1表示是地锁停车
float global_measuredangle = 0;
float zero_pause_time      = 0; // 新增加， 用于计数
int speed_sign             = 0; // 这个表示车速的符号, 这个用于车速的控制,
                    // 因为车速控制中有一个滚动阻力, 在v=0附近是变号的, 当然王丹的程序中
// 就直接忽略这么一个力的抵消了, 我这里是有的. 但是车速=0附近时, 车速可能有小变化,
// 导致我有时就变化扭矩了, 想了一下, 这个和
// 方法不太好, 可能在低速\车速变号时 采用这么模型+控制逻辑并不太好. 这里先用一个方法,
// 就是加一个小信号的磁滞环, 例如当车速为正,
// 然后到0, 然后有点小误差的, 我们就不变号了, 这个量1表示v>0,-1表示v<0, 0表示车辆静止吧.
// 先这样, 再每次进入parked之后, 把这个量设为0? 有问题吗?

void PK_SpeedCtrl(void)
{
    // 内部数据定义——————————————————————————————————————————————————————————————————————————————————————————————————
    PK_ModuleComType ModuleCom_SpeedCtrl;
    // PK_ModuleStateType ModuleState_SpeedCtrl;
    float torque = 0, brake = 0;
    uint8 EpbCtl        = 0;
    int B_state         = 0;
    int abnormal        = 0;
    float m_targetSpeed = RTE_PK_ObjAvoid_Get_ObjAvoid_Velspd();
    // float realSpeed =  0.0043f*RTE_BSW_Get_Motorspd();
    float realSpeed = RTE_PK_Location_Get_Vel_Spd(); // 我比较一下两个速度信号如何区别,
                                                     // 都有值， 问题排除
    // today_realSpeed =  0.0042f*RTE_BSW_Get_Motorspd();
    // today_Loc_speed = RTE_PK_Location_Get_Vel_Spd(); // 我比较一下两个速度信号如何区别
    float measuredangle  = RTE_BSW_Get_EPS_Angle();
    global_measuredangle = measuredangle;
    int drivemode        = 0; // 弃用?
    uint8 epb_state      = 0;
    int epb_waiting_time =
        RTE_PK_StateManage_Get_EPB_WaitingTime(); // 编译有问题， 应该是RTE文件不全吧，
                                                  // 现在没问题了
    boolean BrakeSt =
        RTE_BSW_Get_BrakeSt(); // 2016/12/12日资讯闫雪新增加, 表示现在是否有压强输出，
                               // 这个信号要配合drivemode使用，

    ModuleCom_SpeedCtrl =
        RTE_PK_StateManage_Get_ModuleCom_SpeedCtrl(); //		1.Initial
                                                      //: 初始化 2.Calcu_ON
                                                      //;打开计算 3.Calcu_OFF;关闭计算
    realSpeed_ccp     = realSpeed;
    m_targetSpeed_ccp = m_targetSpeed;
    // 内部逻辑，响应状态管理命令——————————————————————————————————————————————————————————————————————————————————————————————————
    if (ModuleCom_SpeedCtrl == MCOM_OFF) // DO NOT run Locating Condition;
    {
        ModuleState_SpeedCtrl = MSTAT_OFF;
        drivemode             = 0;
        B_state = SpdControl_SpeedTask(m_targetSpeed, realSpeed, measuredangle, drivemode,
                                       BrakeSt, epb_state, epb_waiting_time, &torque,
                                       &brake, &EpbCtl, &abnormal);
        if ((B_state <= 8) && (abnormal == 0)) // 这个放 这干 啥 , 2017年2月18日
            ModuleState_SpeedCtrl = MSTAT_NORM;
        else
            ModuleState_SpeedCtrl = MSTAT_ABNORM;
    }
    else if (ModuleCom_SpeedCtrl == MCOM_ON)
    {

        B_state = SpdControl_SpeedTask(m_targetSpeed, realSpeed, measuredangle, drivemode,
                                       BrakeSt, epb_state, epb_waiting_time, &torque,
                                       &brake, &EpbCtl, &abnormal);
        if ((B_state <= 8) && (abnormal == 0))
            ModuleState_SpeedCtrl = MSTAT_NORM;
        else
            ModuleState_SpeedCtrl = MSTAT_ABNORM;
    }
    // 内部数据接口函数输出——————————————————————————————————————————————————————————————————————————————————————————————————
    RTE_PK_SpeedCtrl_Set_ModuleState_SpeedCtrl(ModuleState_SpeedCtrl);
    RTE_PK_SpeedCtrl_Set_SpeedCtrl_Torque(torque);
    RTE_PK_SpeedCtrl_Set_SpeedCtrl_Pressure(brake);
    RTE_PK_SpeedCtrl_Set_SpeedCtrl_EpbCtl(EpbCtl);
    abnormal_ccp = abnormal;
    danger_ccp   = danger;
    //		const volatile float SpeedCtrl_k_cpp = 60;
    //      RootSig sint32 SpeedCtrl_State_cpp;  //
    // RootSig sint32 SpeedCtrl_Brake_State_cpp;
    // MidSig float SpeedCtrl_Torque_cpp;
}

float targetSpeed    = 0;
float speed_filtered = 0;
boolean manual = 0, semi_manual = 0;

static int SpdControl_SpeedTask(float m_targetSpeed, float realSpeed, float measuredangle,
                                int drivemode, boolean BrakeSt, boolean epb_state,
                                int epb_waiting_time, float *out_torque, float *out_brake,
                                uint8 *out_EpbCtl, int *abnormal)
{
    // 以下为变量声明////////////////////////////////////////////
    static PK_SpeedCtl_VehicleStateMachine _state           = VSM_MANUAL;
    static PK_SpeedCtl_ParkedStateMachineType _state_parked = wait;
    static int _count                                       = 0;
    static float pre_T =
        0; // 2016/12/12 这里是不是不对呀?这些值static的变量好像应该是每次都应该更新的，
           // 比方说这个应该是前一个时刻电机的值， 如果我们不运行这个函数了，
           // 那就麻烦了。没有更新， 等再次进入时， 用的是很早之前的值了。
    static int _release_count = 0;
    float torque = 0, brake = 0;
    uint8 EpbCtl = 0; // 增加了一个0

    // int v_filter_num_order = 3, v_filter_den_order = 3; // order of numerator, order of
    // denominator， 2016121注释掉
    static float v_filter_num[5] = {0.00002925024085431231, 0.00008775073259853326,
                                    0.00008775073259853326, 0.00002925024085431231,
                                    0}; // numerator coefficients
    static float v_filter_den[5] = {1, -2.874356892677485, 2.756483195225695,
                                    -0.881893130592486, 0}; // denominator coefficients
    static float v_filter_y[4]   = {0, 0, 0, 0};            // output state, past output
    static float v_filter_x[5]   = {0, 0, 0, 0, 0}; // input state, current + past input
    // float speed_filtered = LdFilterCal(realSpeed, v_filter_num, v_filter_den,
    // v_filter_x, v_filter_y);
    int flag = 0; // 用来表征是不是要转换状态

    // 以下为程序逻辑///////////////////////////////////////////////

    targetSpeed = m_targetSpeed;

    // float targetSpeed = m_targetSpeed;

    // boolean manual = 0, semi_manual = 0;

    if (drivemode ==
        0) // 2016/12/12 这里根据drivemode and brakest决定汽车工作在哪个模式，
           // 如果不在人工或者半人工， 我们就在自动模式
        manual = 1;
    else
        manual = 0;
    if ((BrakeSt == 1) && (drivemode != 0))
        semi_manual = 1;
    else
        semi_manual = 0;

    speed_filtered =
        LdFilterCal(realSpeed, v_filter_num, v_filter_den, v_filter_x, v_filter_y);

    speed_filtered_ccp = speed_filtered;

    epb_waiting_time =
        rte_min(30000, rte_max(epb_waiting_time,
                               500)); // 我们做了一个限制， 也就是说 30s〉=这个量〉=0.5s，
    switch (_state)
    {
        case VSM_PARKED:
            speed_sign = 0; //
            _count     = _count + 10;

            if (manual == 1) // manual == true
            {
                _state         = VSM_MANUAL;
                _count         = 0;
                _release_count = 0;
                d_count        = 0;
                danger         = 0;
                *abnormal      = 0;
            }
            else if (semi_manual == 1) // semi_manual == true
            {
                _state = VSM_SEMI_MANUAL;
                SemiManualInit(0);
                _count         = 0;
                _release_count = 0;
                danger         = 0;
                *abnormal      = 0;
            }
            else
            {
                if (danger == 1)
                    *abnormal = 1;
                // 我找到问题了
                if (*abnormal == 1)
                {
                    // 在地锁工况下， 逻辑是就不再响应速度请求了
                    targetSpeed = 0;
                }

                switch (_state_parked)
                {
                    case wait:
                        if (epb_state == 0)
                        {
                            if (_count > epb_waiting_time)
                            {
                                _state_parked = pull;
                                _count        = 0;
                            }
                            else if (!IsZeroSpeed(targetSpeed))
                            {
                                if (IsPosSpeed(targetSpeed))
                                {
                                    _state = VSM_STARTUP_FORWARD; // 前进
                                    StartUpInit(targetSpeed);
                                    _count = 0;
                                }
                                else if (IsNegSpeed(targetSpeed))
                                {
                                    _state = VSM_STARTUP_BACKWARD; // 后退
                                    StartUpInit(targetSpeed);
                                    _count = 0;
                                }
                            }
                        }
                        else // 手刹信号显示已经有手刹了
                        {
                            if (!IsZeroSpeed(targetSpeed))
                            {
                                _state_parked = push;
                                _count        = 0;
                            }
                            else
                            {
                                _state_parked = pull;
                                _count        = 0;
                            }
                        }

                        break;
                    case pull:
                        if (epb_state == 1)
                            _state_parked = hold;
                        break;
                    case hold:
                        if (!IsZeroSpeed(targetSpeed))
                            _state_parked = push;
                        break;
                    case push:
                        if ((epb_state == 0) && !IsZeroSpeed(targetSpeed))
                        {
                            if (IsPosSpeed(targetSpeed))
                            {
                                _state = VSM_STARTUP_FORWARD; // 前进
                                StartUpInit(targetSpeed);
                                _count = 0;
                            }
                            else if (IsNegSpeed(targetSpeed))
                            {
                                _state = VSM_STARTUP_BACKWARD; // 后退
                                StartUpInit(targetSpeed);
                                _count = 0;
                            }
                        }
                        if (IsZeroSpeed(targetSpeed))
                        {
                            _state_parked = wait;
                            _count        = 0;
                        }
                        break;
                    default:
                        break;
                }
            }
            if (_count >= 33000)
                _count = 32000;

            switch (_state_parked)
            {
                case wait:
                    torque = 0;
                    brake  = 1;
                    EpbCtl = 0;
                    break;
                case pull:
                    torque = 0;
                    brake  = 1;
                    EpbCtl = 2;
                    break;
                case hold:
                    torque = 0;
                    brake  = 0;
                    EpbCtl = 0;
                    break;
                case push:
                    torque = 0;
                    brake  = 0;
                    EpbCtl = 1;
                    break;
                default:
                    break;
            }
            break;
        case VSM_STARTUP_FORWARD:
            if (m_startup.current_time == 0)
                EpbCtl = 1;
            else
                EpbCtl = 0;
            torque = StartUpOutput(&flag, targetSpeed); // 这里判断flag， 对吗？
            brake  = 0;

            if (flag == 1)
            {
                _state = VSM_SPEEDCTRL_FORWARD;
                ControllerInit(speed_filtered, torque);
                if (speed_filtered < -0.05)
                    speed_sign = -1;
                else
                    speed_sign = 1;

                zero_pause_time = 0; // 添加一个初始化
            }
            else if (flag == 2)
            {
                _state = VSM_PARKING;
                StoppingInit(torque);
                _count = 0;
            }

            if (manual == 1)
            {
                _state = VSM_MANUAL;
                _count = 0;
            }
            else if (semi_manual == 1)
            {
                _state = VSM_SEMI_MANUAL;
                SemiManualInit(torque);
                _count = 0;
            }
            break;
        case VSM_STARTUP_BACKWARD:
            if (m_startup.current_time == 0)
                EpbCtl = 1;
            else
                EpbCtl = 0;
            torque = StartUpOutput(&flag, targetSpeed);
            brake  = 0;

            if (flag == 1)
            {
                _state = VSM_SPEEDCTRL_BACKWARD;
                ControllerInit(speed_filtered, torque);
                zero_pause_time = 0;
                if (speed_filtered > 0.05)
                    speed_sign = 1;
                else
                    speed_sign = -1;
            }
            else if (flag == 2)
            {
                _state = VSM_PARKING;
                StoppingInit(torque);
                _count = 0;
            }
            if (manual == 1)
            {
                _state = VSM_MANUAL;
                _count = 0;
            }
            else if (semi_manual == 1)
            {
                _state = VSM_SEMI_MANUAL;
                SemiManualInit(torque);
                _count = 0;
            }
            break;
        case VSM_SPEEDCTRL_FORWARD:
            GetCtrlParameters(targetSpeed, speed_filtered, measuredangle);
            torque = m_controlParam.driveTorque;
            brake  = m_controlParam.brake;
            EpbCtl = 0;
            if (d_count > 400)
            {
                _state = VSM_PARKING;
                StoppingInit(torque);
                _count  = 0;
                d_count = 0;
                danger  = 1;
            }
            if (!IsPosSpeed(targetSpeed))
            {
                _state = VSM_PARKING;
                StoppingInit(torque);
                _count  = 0;
                d_count = 0;
                danger  = 0;
            }
            if (manual == 1) // manual == true
            {
                _state  = VSM_MANUAL;
                _count  = 0;
                d_count = 0;
                danger  = 0;
            }
            else if (semi_manual == 1) // semi_manual == true
            {
                _state = VSM_SEMI_MANUAL;
                SemiManualInit(torque);
                _count  = 0;
                d_count = 0;
                danger  = 0;
            }
            break;
        case VSM_SPEEDCTRL_BACKWARD:
            GetCtrlParameters(targetSpeed, speed_filtered, measuredangle);
            torque = m_controlParam.driveTorque;
            brake  = m_controlParam.brake;
            EpbCtl = 0;
            if (d_count > 400)
            {
                _state = VSM_PARKING;
                StoppingInit(torque);
                _count  = 0;
                d_count = 0;
                danger  = 1;
            }
            if (!IsNegSpeed(targetSpeed))
            {
                _state = VSM_PARKING;
                StoppingInit(torque);
                _count  = 0;
                d_count = 0;
                danger  = 0;
            }
            if (manual == 1)
            {
                _state  = VSM_MANUAL;
                _count  = 0;
                d_count = 0;
                danger  = 0;
            }
            else if (semi_manual == 1)
            {
                _state = VSM_SEMI_MANUAL;
                SemiManualInit(torque);
                _count  = 0;
                d_count = 0;
                danger  = 0;
            }
            break;
        case VSM_PARKING:
            StoppingAction(realSpeed, &torque, &brake, &EpbCtl);
            if (IsZeroSpeed(realSpeed))
            {
                if (_count < 500)
                {
                    _count = _count + 10;
                }
                else
                {
                    _state         = VSM_PARKED;
                    _state_parked  = wait;
                    EpbCtl         = 0;
                    _count         = 0;
                    _release_count = 0;
                }
            }
            else
            {
                _count = 0;
            }
            if (manual == 1)
            {
                _state = VSM_MANUAL;
                _count = 0;
            }
            else if (semi_manual == 1)
            {
                _state = VSM_SEMI_MANUAL;
                SemiManualInit(torque);
                _count = 0;
            }
            break;
        case VSM_SEMI_MANUAL:
            SemiManualAction(realSpeed, &torque, &brake, &EpbCtl);
            if (manual == 1)
            {
                _state = VSM_MANUAL;
                _count = 0;
            }
            else if (semi_manual == 1)
            {
                if ((!IsZeroSpeed(targetSpeed)) && (epb_state == 0))
                {
                    if (IsPosSpeed(targetSpeed))
                    {
                        _state = VSM_SPEEDCTRL_FORWARD;
                        ControllerInit(speed_filtered, torque);
                        zero_pause_time = 0;
                        EpbCtl          = 1;
                        if (speed_filtered < -0.05)
                            speed_sign = -1;
                        else
                            speed_sign = 1;
                    }
                    else
                    {
                        _state = VSM_SPEEDCTRL_BACKWARD;
                        ControllerInit(speed_filtered, torque);
                        zero_pause_time = 0;
                        EpbCtl          = 1;
                        if (speed_filtered > 0.05)
                            speed_sign = 1;
                        else
                            speed_sign = -1;
                    }
                }

                if ((!IsZeroSpeed(targetSpeed)) && (epb_state == 1))
                {
                    _state = VSM_PARKING;
                    StoppingInit(torque);
                    _count = 0;
                }

                if (IsZeroSpeed(targetSpeed))
                {
                    _state = VSM_PARKING;
                    StoppingInit(torque);
                    _count = 0;
                }
            }
            break;
        case VSM_MANUAL:
            FullManualAction(realSpeed, &torque, &brake, &EpbCtl);
            if (manual != 1)
            {
                if (semi_manual == 1)
                {
                    _state = VSM_SEMI_MANUAL;
                    SemiManualInit(torque);
                    _count = 0;
                }
                else
                {
                    if ((!IsZeroSpeed(targetSpeed)) && (epb_state == 0))
                    {
                        if (IsPosSpeed(targetSpeed))
                        {
                            _state = VSM_SPEEDCTRL_FORWARD;
                            ControllerInit(speed_filtered, torque);
                            zero_pause_time = 0;
                            EpbCtl          = 1;
                            if (speed_filtered < -0.05)
                                speed_sign = -1;
                            else
                                speed_sign = 1;
                        }
                        else
                        {
                            _state = VSM_SPEEDCTRL_BACKWARD;
                            ControllerInit(speed_filtered, torque);
                            zero_pause_time = 0;
                            EpbCtl          = 1;
                            if (speed_filtered > 0.05)
                                speed_sign = 1;
                            else
                                speed_sign = -1;
                        }
                    }
                    else if ((!IsZeroSpeed(targetSpeed)) && (epb_state == 1) &&
                             (IsZeroSpeed(realSpeed))) // epb显示有刹，目标速度不为0，
                    {
                        _state        = VSM_PARKED; // 我们进入parking 模式
                        _state_parked = hold;
                        _count        = 0;
                    }
                    else if ((!IsZeroSpeed(targetSpeed)) && (epb_state == 1) &&
                             (!IsZeroSpeed(realSpeed)))
                    {
                        _state = VSM_PARKING;
                        StoppingInit(torque);
                        _count = 0;
                    }
                    else if ((IsZeroSpeed(targetSpeed)) && (epb_state == 1) &&
                             (IsZeroSpeed(realSpeed)))
                    {
                        _state        = VSM_PARKED; // 我们进入parking 模式
                        _state_parked = hold;
                        _count        = 0;
                    }
                    else if ((IsZeroSpeed(targetSpeed)) && (epb_state == 1) &&
                             (!IsZeroSpeed(realSpeed)))
                    {
                        _state = VSM_PARKING;
                        StoppingInit(torque);
                        _count = 0;
                    }
                    else if ((IsZeroSpeed(targetSpeed)) && (epb_state == 0))
                    {
                        _state = VSM_PARKING;
                        StoppingInit(torque);
                        _count = 0;
                    }
                }
            }
            else
            {
                _state = VSM_MANUAL;
                _count = 0;
            }

            break;
        default:
            break;
    }
    pre_T                     = torque;
    pre_T_ccp                 = pre_T;
    *out_brake                = brake;
    *out_torque               = torque;
    *out_EpbCtl               = EpbCtl;
    SpeedCtrl_Brake_State_cpp = (int)_state_parked;
    return (int)_state;
}

float LdFilterCal(float input, float *num, float *den, float *x, float *y)
{
    float output = 0;

    x[0] = input;
    if (fabs(x[0]) < 0.000000000001)
        x[0] = 0;
    output =
        (num[0] * x[0] + num[1] * x[1] + num[2] * x[2] + num[3] * x[3] + num[4] * x[4] -
         den[1] * y[0] - den[2] * y[1] - den[3] * y[2] - den[4] * y[3]) /
        den[0];
    if (fabs(output) < 0.000000000001)
        output = 0;
    x[4] = x[3];
    x[3] = x[2];
    x[2] = x[1];
    x[1] = x[0];
    x[0] = 0;
    y[3] = y[2];
    y[2] = y[1];
    y[1] = y[0];
    y[0] = output;

    return output;

    // int v_filter_num_order = 3, v_filter_den_order = 3; // order of numerator, order of
    // denominator， 2016121注释掉
    //	static double v_filter_num[5] = {0.00002914649446565665, 0.00008743948339696994,
    // 0.00008743948339696994, 0.00002914649446565665, 0}; // numerator coefficients
    // static double v_filter_den[5] = {1, -2.874356892677485, 2.756483195225695,
    //-0.881893130592486, 0}; // denominator coefficients 	static double v_filter_y[4] =
    //{0, 0, 0, 0}; // output state, past output 	static double v_filter_x[5] = {0, 0,
    // 0, 0, 0}; // input state, current + past input
    // speed_filtered = LdFilterCal(realSpeed, v_filter_num, v_filter_den, v_filter_x,
    // v_filter_y);
}

void SemiManualInit(float pre_torque) { m_semimanual.pre_Tor = pre_torque; }
void SemiManualAction(float v, float *torque, float *pressure, uint8 *EpbCtl)
{
    if (m_semimanual.pre_Tor > 0)
        *torque = rte_max(m_semimanual.pre_Tor - 5, 0);
    else
        *torque = rte_min(m_semimanual.pre_Tor + 5, 0);
    m_semimanual.pre_Tor = *torque;

    *pressure = 0;
    *EpbCtl   = 0;
}

void StartUpInit(float targetSpeed)
{
    m_startup.time_limit   = 200;
    m_startup.last_output  = 0;
    m_startup.direction    = targetSpeed;
    m_startup.current_time = 0;
}

float StartUpOutput(int *flag, float targetSpeed)
{
    float output = 0;

    /* 	if (m_startup.direction>0)
        {
            if (m_startup.current_time<=50)
                output = 1.5f;
            else if (m_startup.current_time<=100)
                output = 2.0f;
            else if (m_startup.current_time<=150)
                output = 2.5f;
            else if (m_startup.current_time<=200)
                output = 3.0f;
        }
        else
        {
            if (m_startup.current_time<=50)
                output = -1.5f;
            else if (m_startup.current_time<=100)
                output = -2.0f;
            else if (m_startup.current_time<=150)
                output = -2.5f;
            else if (m_startup.current_time<=200)
                output = -3.0f;
        } */ // 注释部分为我之前写的起步准备, 下面使用王敏\qq等人设计的一个起步准备程序

    if (m_startup.direction > 0)
    {
        if (m_startup.current_time <= 100)
            output = 0.5f;
        else if (m_startup.current_time <= 200)
            output = 1.0f;
        else if (m_startup.current_time <= 300)
            output = 1.5f;
        else if (m_startup.current_time <= 400)
            output = 2.0f;
        else if (m_startup.current_time <= 500)
            output = 2.5f;
        else
            output = 3.0f;
    }
    else
    {
        if (m_startup.current_time <= 100)
            output = -0.5f;
        else if (m_startup.current_time <= 200)
            output = -1.0f;
        else if (m_startup.current_time <= 300)
            output = -1.5f;
        else if (m_startup.current_time <= 400)
            output = -2.0f;
        else if (m_startup.current_time <= 500)
            output = -2.5f;
        else
            output = -3.0f;
    }

    m_startup.current_time = m_startup.current_time + 10;
    m_startup.last_output  = output;

    if (m_startup.current_time == 520)
        *flag = 1;
    if ((fabs(targetSpeed) < 0.01f) || (targetSpeed * m_startup.direction <= 0))
        *flag = 2;
    return output;
}

void StoppingInit(float pre_torque)
{
    m_stopping.last_pressure =
        0.7f; // 2017年3月10日左右, 经与朝鑫等人测试, 感觉0.7比较合适
    m_stopping.current_time     = 0;
    m_stopping.stopping_time    = 0;
    m_stopping.pressure_on_time = 0;
    m_stopping.stopped          = 0;
    m_stopping.pre_Tor          = pre_torque;
}

void StoppingAction(float v, float *torque, float *pressure, uint8 *EpbCtl)
{
    if (m_stopping.current_time == 0)
    {
        *torque = m_stopping.pre_Tor;
    }
    if (m_stopping.pre_Tor > 0)
        *torque = rte_max(m_stopping.pre_Tor - 5, 0);
    else
        *torque = rte_min(m_stopping.pre_Tor + 5, 0);
    m_stopping.pre_Tor = *torque;

    if (m_stopping.stopped == 0)
    {
        if (m_stopping.current_time - m_stopping.pressure_on_time >= 1500)
        {
            *pressure                   = m_stopping.last_pressure + 0.5f;
            m_stopping.pressure_on_time = m_stopping.current_time;
        }
        else
        {
            *pressure = m_stopping.last_pressure;
        }
    }

    *EpbCtl = 0;
    if (*pressure >= 5)
        *pressure = 5;

    m_stopping.last_pressure = *pressure;
    m_stopping.current_time  = m_stopping.current_time + 10;
    // *torque = 0; 2017年3月16日， 把这里屏蔽掉， 我想这个可能有点帮助，
    // 如果这个感觉还是不好的话， 我就要设计比较细致一点的关机了，
    m_stopping.pre_Tor = *torque;
}

void FullManualAction(float v, float *torque, float *pressure, uint8 *EpbCtl)
{
    *torque   = 0; // 2016/12/13日修改原来忘记加邋*号了， 所以才会有assert什么的错误。
    *pressure = 0;
    *EpbCtl   = 0;
}

void ControllerInit(float speed_filtered, float pre_tor)
{
    m_spdcontrol.m_uDesire   = 0;
    m_spdcontrol.v_est       = speed_filtered;
    m_spdcontrol.d_est       = 0;
    m_spdcontrol.est_gain[0] = 0.6197f;
    m_spdcontrol.est_gain[1] = 8.7331f;

    m_spdcontrol.tilde_Treq = 0;
    m_spdcontrol.initial    = 1;

    m_spdcontrol.pre_torque = pre_tor;
}

// 计算控制量, speed：车速
void GetCtrlParameters(float targetSpeed, float speed, float measuredangle)
{
    float steerAngle       = 0;
    float motor_speed      = speed;
    m_spdcontrol.m_uDesire = targetSpeed;
    longitudinalControl(speed);
}

void longitudinalControl(float speed_filtered)
{
    m_controlParam.driveTorque = slidingModeControlForSpeed_nomotordyn(speed_filtered);
    m_controlParam.brake =
        0; // 这里都有问题， 如果电机减速不能使用， 我们这里也完了， 这里也要做逻辑。
    estimator(speed_filtered);
}

float global_bar_torque = 0;
float global_v_est      = 0;
float global_d_est      = 0;

float slidingModeControlForSpeed_nomotordyn(float speed)
{
    float torque;
    float F_z  = m_modelParam.m * m_modelParam.g;
    float tau  = m_modelParam.tau;
    float R    = m_modelParam.R;
    float rho  = m_modelParam.rho;
    float r    = m_modelParam.r;
    float A    = m_modelParam.A;
    float C_x  = m_modelParam.C_x;
    float J    = m_modelParam.J;
    float f_R0 = m_modelParam.f_R0;
    float f_R1 = m_modelParam.f_R1;
    float f_R2 = m_modelParam.f_R2;
    float F_R, F_L, F_s;
    float FR_filter_num[5]      = {0.1, 0, 0, 0, 0};
    float FR_filter_den[5]      = {1, -0.9, 0, 0, 0};
    static float FR_filter_y[4] = {0, 0, 0, 0};    // output state, past output
    static float FR_filter_x[5] = {0, 0, 0, 0, 0}; // input state, current + past input

    float e                  = m_spdcontrol.v_est - m_spdcontrol.m_uDesire;
    float k                  = -60;
    float T_max              = 100;
    float delta_u            = 3;
    float bar_torque         = T_max * sat(k * e, 1 / T_max);
    m_spdcontrol_v_est_ccp   = m_spdcontrol.v_est;
    m_spdcontrol_d_est_ccp   = m_spdcontrol.d_est;
    m_spdcontrol_m_uDesi_ccp = m_spdcontrol.m_uDesire;
    global_bar_torque        = bar_torque; // 增加

    F_R = F_z * (f_R0 + f_R1 * fabs(speed) / 30 +
                 f_R2 * speed * speed * speed * speed / 810000);
    // F_R = F_R * sign(speed);
    if ((speed_sign < 0.1) && (speed > 0.05))
        speed_sign = 1;
    if ((speed_sign > -0.1) && (speed < -0.05))
        speed_sign = -1;
    F_R = F_R * speed_sign;

    F_R = LdFilterCal(F_R, FR_filter_num, FR_filter_den, FR_filter_x, FR_filter_y);

    F_L = 0.5 * rho * A * C_x * speed * speed;

    F_L = F_L * sign(speed);
    F_s = 0;

    m_spdcontrol.tilde_Treq = bar_torque - m_spdcontrol.d_est;
    torque                  = m_spdcontrol.tilde_Treq + r / R * (F_R + F_L + F_s);
    // 以下为2017年3月17日, 按照王丹设计修改
    float zero_pause_period = 50; // 在扭矩=0处停留zero_pause_period ms
    float rate_v[6]         = {3, 1, 0.2f, 0.2f, 1, 3};
    float rate_bp[5]        = {
        -10, -4, 0, 4,
        10}; // 这个就是在不同的扭矩范围， 限制不同的扭矩梯度， 减少冲击感寄希望于此，

    float rate_now = 0;

    if (m_spdcontrol.pre_torque < rate_bp[0])
        rate_now = rate_v[0];
    else if (m_spdcontrol.pre_torque < rate_bp[1])
        rate_now = rate_v[1];
    else if (m_spdcontrol.pre_torque < rate_bp[2])
        rate_now = rate_v[2];
    else if (m_spdcontrol.pre_torque < rate_bp[3])
        rate_now = rate_v[3];
    else if (m_spdcontrol.pre_torque < rate_bp[4])
        rate_now = rate_v[4];
    else
        rate_now = rate_v[5];

    if (torque - m_spdcontrol.pre_torque > rate_now)
        torque = m_spdcontrol.pre_torque + rate_now;
    else if (torque - m_spdcontrol.pre_torque < -rate_now)
        torque = m_spdcontrol.pre_torque - rate_now;

    if ((m_spdcontrol.pre_torque * torque < 0) ||
        ((fabs(torque) < 0.1f) && (zero_pause_time < 9)) ||
        ((zero_pause_time > 1) && (zero_pause_time < zero_pause_period)))
    {
        torque          = 0;
        zero_pause_time = zero_pause_time + 10;
    }
    else
        zero_pause_time = 0;

    // 以上为2017年3月17日, 按照王丹设计修改

    m_spdcontrol.tilde_Treq = torque - r / R * (F_R + F_L + F_s);
    m_spdcontrol.pre_torque = torque;

    // 以下为2017年4月14日, 按照一定的条件退出, 还是按照刹停算吧, 条件就是扭矩持续大于20,
    // 速度持续低于0.1, 持续0.5s, 我们就推出. 大于30加5， 15 加1 其他的加1
    if ((fabs(speed) < 0.05) &&
        (fabsf(e) >
         0.1)) // 当车速比0.1低， 误差也比较大时， 我们开始计数， 否则则计数器清0
    {
        if (fabs(torque) > 45)
            d_count = d_count + 5;
        else if (fabs(torque) > 25)
            d_count = d_count + 2;
        else
            d_count = d_count + 1;
    }
    else
    {
        d_count = 0;
    }
    d_count_xx_ccp = d_count;
    return torque;
}

// 饱和函数，输入乘以factor以后计算饱和，饱和输出-1或1，不饱和输出in*factor
float sat(float in, float factor)
{
    if (in * factor > 1)
    {
        return 1;
    }
    else if (in * factor < -1)
    {
        return -1;
    }
    else
    {
        return in * factor;
    }
}

float error_a = 0;
void estimator(float speed)
{
    float v_est_01;
    v_est_01 = m_spdcontrol.v_est +
               0.01f * m_modelParam.R / m_modelParam.J * m_spdcontrol.tilde_Treq +
               0.01f * m_modelParam.R / m_modelParam.J * m_spdcontrol.d_est +
               m_spdcontrol.est_gain[0] * (speed - m_spdcontrol.v_est);
    m_spdcontrol.d_est =
        m_spdcontrol.d_est + m_spdcontrol.est_gain[1] * (speed - m_spdcontrol.v_est);
    error_a            = speed - m_spdcontrol.v_est;
    m_spdcontrol.v_est = v_est_01;

    global_v_est = m_spdcontrol.v_est; // 2016年1216增加
    global_d_est = m_spdcontrol.d_est;
}
