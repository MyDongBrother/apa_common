#pragma section all "CPU1.Private"
/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: LinearDemingRegression.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 11-Oct-2018 23:25:44
 */

/* Include Files */

#include "LinearDemingRegression.h"
#include "combineVectorElements.h"

/* Function Definitions */

/*
 * function [a,b,psi,cost]=LinearDemingRegression(x,y,window_size)
 * x_mean = mean(x);
 *  y_mean = mean(y);
 * Arguments    : float x[30]
 *                float y[30]
 *                float window_size
 *                float *a
 *                float *b
 *                float *psi
 *                float *cost
 * Return Type  : void
 */
void LinearDemingRegression(float x[30], float y[30], float window_size, float *a,
                            float *b, float *psi, float *cost)
{
    float cost1;
    float X1;
    int k;
    float b_x[30];
    float Xn;
    float b_y;
    float a1;
    float c_y[30];

    /* 'LinearDemingRegression:4' x = x - mean(x); */
    cost1 = combineVectorElements(x) / 30.0F;

    /* 'LinearDemingRegression:5' y = y - mean(y); */
    X1 = combineVectorElements(y) / 30.0F;

    /* 'LinearDemingRegression:6' x_mean = 0; */
    /* 'LinearDemingRegression:7' y_mean = 0; */
    /* 'LinearDemingRegression:8' Lxy = sum((x-x_mean).*(y-y_mean)); */
    for (k = 0; k < 30; k++)
    {
        Xn     = x[k] - cost1;
        b_y    = y[k] - X1;
        b_x[k] = Xn * b_y;
        x[k]   = Xn;
        y[k]   = b_y;
    }

    cost1 = combineVectorElements(b_x);

    /* 'LinearDemingRegression:9' Lxx = sum((x-x_mean).^2); */
    for (k = 0; k < 30; k++)
    {
        b_x[k] = x[k] * x[k];
    }

    X1 = combineVectorElements(b_x);

    /* 'LinearDemingRegression:10' Lyy = sum((y-y_mean).^2); */
    for (k = 0; k < 30; k++)
    {
        b_x[k] = y[k] * y[k];
    }

    Xn = combineVectorElements(b_x);

    /* 'LinearDemingRegression:11' if abs(Lxy)>single(1e-7) */
    if (fabsf(cost1) > 1.0E-7F)
    {
        /* 'LinearDemingRegression:12' a1 = ((Lyy-Lxx)+sqrt((Lyy-Lxx)^2+4*Lxy^2))/2/Lxy;
         */
        b_y = Xn - X1;
        a1  = ((Xn - X1) + sqrtf(b_y * b_y + 4.0F * (cost1 * cost1))) / 2.0F / cost1;

        /* 'LinearDemingRegression:13' a2 = ((Lyy-Lxx)-sqrt((Lyy-Lxx)^2+4*Lxy^2))/2/Lxy;
         */
        b_y = Xn - X1;
        *a  = ((Xn - X1) - sqrtf(b_y * b_y + 4.0F * (cost1 * cost1))) / 2.0F / cost1;

        /* 'LinearDemingRegression:15' b1 = y_mean - a1*x_mean; */
        /* 'LinearDemingRegression:16' b2 = y_mean - a2*x_mean; */
        /* 'LinearDemingRegression:17' cost1 = sum(abs(y -
         * a1*x-b1))/sqrt(1+a1^2)/window_size; */
        for (k = 0; k < 30; k++)
        {
            c_y[k] = fabsf(y[k] - a1 * x[k]);
            b_x[k] = y[k] - *a * x[k];
        }

        cost1 = combineVectorElements(c_y) / sqrtf(1.0F + a1 * a1) / window_size;

        /* 'LinearDemingRegression:18' cost2 = sum(abs(y -
         * a2*x-b2))/sqrt(1+a2^2)/window_size; */
        for (k = 0; k < 30; k++)
        {
            c_y[k] = fabsf(b_x[k]);
        }

        *cost = combineVectorElements(c_y) / sqrtf(1.0F + *a * *a) / window_size;

        /* 'LinearDemingRegression:19' if cost1 <= cost2 */
        if (cost1 <= *cost)
        {
            /* 'LinearDemingRegression:20' a = a1; */
            *a = a1;

            /* 'LinearDemingRegression:21' b = b1; */
            *b = 0.0F;

            /* 'LinearDemingRegression:22' cost = cost1; */
            *cost = cost1;
        }
        else
        {
            /* 'LinearDemingRegression:23' else */
            /* 'LinearDemingRegression:24' a = a2; */
            /* 'LinearDemingRegression:25' b = b2; */
            *b = 0.0F;

            /* 'LinearDemingRegression:26' cost = cost2; */
        }

        /* 'LinearDemingRegression:29' X1=(a*(y(1)-b)+x(1))/(a*a+1); */
        X1 = (*a * y[0] + x[0]) / (*a * *a + 1.0F);

        /* 'LinearDemingRegression:30' Y1=a*X1+b; */
        /* 'LinearDemingRegression:31' Xn=(a*(y(end)-b)+x(end))/(a*a+1); */
        Xn = (*a * y[29] + x[29]) / (*a * *a + 1.0F);

        /* 'LinearDemingRegression:32' Yn=a*Xn+b; */
        /* 'LinearDemingRegression:33' psi = atan2(Yn-Y1,Xn-X1); */
        *psi = atan2f(*a * Xn - *a * X1, Xn - X1);
    }
    else
    {
        /* 'LinearDemingRegression:34' else */
        /* 'LinearDemingRegression:35' a = single(0); */
        *a = 0.0F;

        /* 'LinearDemingRegression:36' b = single(0); */
        *b = 0.0F;

        /* 'LinearDemingRegression:37' cost = single(99999); */
        *cost = 99999.0F;

        /* 'LinearDemingRegression:38' psi = single(0); */
        *psi = 0.0F;
    }

    /*  if abs(x(end) - x(1)) >= abs(y(end) - y(1)) */
    /*      x10 = x(end) - x(1); */
    /*      y10 = a*x10; */
    /*  else */
    /*      y10 = y(end) - y(1); */
    /*      x10 = y10/a; */
    /*  end */
    /*  figure('Name','Deming') */
    /*  plot(x,x*a+b,x,y) */
    /*  legend('fit','data') */
    /*  title([num2str(psi/pi*180) 'degree']) */
    /*  axis equal */
}

/*
 * File trailer for LinearDemingRegression.c
 *
 * [EOF]
 */
