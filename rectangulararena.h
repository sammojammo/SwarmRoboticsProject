#ifndef RECTANGULARARENA_H_
#define RECTANGULARARENA_H_

/******************************************************************************/
/******************************************************************************/

#include "arena.h"

/******************************************************************************/
/******************************************************************************/

class CRectangularArena : public CArena
{
public:
    CRectangularArena(
        const char* pch_name, 
        double f_size_x,       
        double f_size_y, 
        unsigned int un_res_x, 
        unsigned int un_res_y);

    virtual ~CRectangularArena();

    virtual bool IsObstacle(TVector2d* t_position);
};

/******************************************************************************/
/******************************************************************************/

#endif
