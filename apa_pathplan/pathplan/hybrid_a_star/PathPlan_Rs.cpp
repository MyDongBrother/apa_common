#include "hybrid_a_star.h"
#include "hybrid_type.h"

static void trajEnd(const float Path[5], float EndPos[3])
{
    float x0 = Path[0], y0 = Path[1], theta0 = Path[2], ds = Path[3], Rr = Path[4];
    float ftheta = 0;
    if (fabsf(Rr) < 1)
    {
        EndPos[0] = cosf(theta0) * ds + Path[0];
        EndPos[1] = sinf(theta0) * ds + y0;
        EndPos[2] = theta0;
    }
    else
    {
        ftheta    = -sign(Rr) * M_PI / 2 + theta0;
        EndPos[0] = cosf(ds / Rr + ftheta) * fabsf(Rr) + x0 - fabsf(Rr) * cosf(ftheta);
        EndPos[1] = sinf(ds / Rr + ftheta) * fabsf(Rr) + y0 - fabsf(Rr) * sinf(ftheta);
        EndPos[2] = theta0 + ds / Rr;
    }
}

int PathPlan_Rs(const float start[3], const float end[3], float trajs[][5])
{
    const float min_radius = 5.8;
    static RSPath rs_path(min_radius);
    RSPath::RSPathData result =
        rs_path.GetRSPath(start[0], start[1], start[2], end[0], end[1], end[2]);

    float trajStart[3];
    memcpy(trajStart, start, sizeof(trajStart));
    int count = 0;
    for (int i = 0; i < 5; i++)
    {
        if (result.type_[i] == RSPath::N)
        {
            return count;
        }

        if (fabs(result.length_[i]) > 0.0001)
        {
            if (result.type_[i] == RSPath::L)
            {
                memcpy(trajs[count], trajStart, sizeof(trajStart));
                trajs[count][3] = min_radius * result.length_[i];
                trajs[count][4] = min_radius;
                trajEnd(trajs[count], trajStart);
                count++;
                continue;
            }

            if (result.type_[i] == RSPath::R)
            {
                memcpy(trajs[count], trajStart, sizeof(trajStart));
                trajs[count][3] = min_radius * result.length_[i];
                trajs[count][4] = -min_radius;
                trajEnd(trajs[count], trajStart);
                count++;
                continue;
            }

            if (result.type_[i] == RSPath::S)
            {
                memcpy(trajs[count], trajStart, sizeof(trajStart));
                trajs[count][3] = min_radius * result.length_[i];
                trajs[count][4] = 0;
                trajEnd(trajs[count], trajStart);
                count++;
                continue;
            }
        }
        else
        {
            continue;
        }
    }

    return count;
}
