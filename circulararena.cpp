#include "circulararena.h"
#include <math.h>

/******************************************************************************/
/******************************************************************************/

CCircularArena::CCircularArena(const char* pch_name, double f_radius, unsigned int un_res_x, unsigned int un_res_y) : 
    CArena(pch_name, f_radius * 2.0, f_radius * 2.0, un_res_x, un_res_y),
    m_fRadius(f_radius)    
{
}

/******************************************************************************/
/******************************************************************************/

CCircularArena::~CCircularArena()
{
}

/******************************************************************************/
/******************************************************************************/

bool CCircularArena::IsObstacle(TVector2d* t_position)
{
    double fDistanceFromCenter = sqrt(
        t_position->x * t_position->x +
        t_position->y * t_position->y
        );

    if (fDistanceFromCenter <= m_fRadius)
        return false;
    else
        return true;
}

/******************************************************************************/
/******************************************************************************/
