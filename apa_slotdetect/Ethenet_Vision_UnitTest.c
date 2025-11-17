#ifdef UNIT_TEST
#include "PK_Calibration.h"
#include "Record_Log.h"
#include "Rte.h"
#include "json/json.h"
#include <fstream>
#include <string>
#include <iostream>

extern void ParseVisionPark(const char *str);

void ParseVisionPark_UnitTest()
{
    // test parking process
    // std::ifstream in("2023_08_10_11_41_43_jsonavm.txt");
    std::ifstream in("2023_08_23_09_57_20_jsonavm.txt");
    std::string lineslot;

    float target[3]    = {6.648129, -6.238125, 1.603864};
    float avmpoints[8] = {5.195075, -2.612184, 5.363625, -7.707397,
                          8.026927, -7.619294, 7.858377, -2.52408};

    RTE_PK_SlotDetect_Set_TargPos(target);
    // RTE_PK_StateManage_Set_ModuleCom_PathExecute(MCOM_ON);
    RTE_PK_StateManage_Set_ModuleCom_PathExecute(MCOM_ON_PARKING);
    RTE_PK_StateManage_Set_ModuleCom_Location(MCOM_ON);
    RTE_PK_DataConvt_Set_IsAVM_Slot(1);
    Avm_Pot_T avmPot;
    memcpy(&avmPot.near_rear, &avmpoints[0], sizeof(Point_T));
    memcpy(&avmPot.far_rear, &avmpoints[2], sizeof(Point_T));
    memcpy(&avmPot.far_front, &avmpoints[4], sizeof(Point_T));
    memcpy(&avmPot.near_front, &avmpoints[6], sizeof(Point_T));
    RTE_PK_DataConvt_Set_TargAVM_SlotInfo(&avmPot);

    if (in)
    {
        int count = 0;
        while (getline(in, lineslot))
        {
            Json::Reader jsonReader;
            Json::Value jsonRoot;

            if (!jsonReader.parse(lineslot, jsonRoot))
            {
                continue;
            }

            const char *key1   = "curpos";
            std::string curpos = jsonRoot[key1].asString();
            RD_PosStamp curPosStamp;

            sscanf(curpos.c_str(), "%f,%f,%f", &curPosStamp.pos[0], &curPosStamp.pos[1],
                   &curPosStamp.pos[2]);
            curPosStamp.pos[3] = 1.0;
            RTE_PK_Location_Set_CurPos(curPosStamp.pos);
            RTE_PK_Location_Set_CurPos_With_Timestamp(&curPosStamp);

            const char *key2 = "recv";
            std::string data = jsonRoot[key2].toStyledString();

            ParseVisionPark(data.c_str());
        }
    }
}

#endif
