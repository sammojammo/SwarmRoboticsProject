#ifndef RANDOMWALKBEHAVIOR_H_
#define RANDOMWALKBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

class CRandomWalkBehavior : public CBehavior 
{
public:
    CRandomWalkBehavior(double f_change_direction_probability);
    
    virtual bool TakeControl();
    virtual void Action();

protected:
    double m_fChangeDirectionProbability;
};

/******************************************************************************/
/******************************************************************************/

#endif 
