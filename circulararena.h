#ifndef CIRCULARARENA_H_
#define CIRCULARARENA_H_

/******************************************************************************/
/******************************************************************************/

#include "arena.h"

/******************************************************************************/
/******************************************************************************/

class CCircularArena : public CArena
{
public:
    CCircularArena(const char* pch_name, double f_radius, unsigned int un_res_x, unsigned int un_res_y);
    virtual ~CCircularArena();

    virtual bool IsObstacle(TVector2d* t_position);

protected:
    double m_fRadius;
};

/******************************************************************************/
/******************************************************************************/

#endif
