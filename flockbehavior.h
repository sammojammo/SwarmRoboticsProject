#ifndef FLOCKBEHAVIOR_H_
#define FLOCKBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

class CFlockBehavior : public CBehavior 
{
public:
    CFlockBehavior(double f_sensory_radius);

    virtual void SimulationStep();    
    virtual bool TakeControl();
    virtual void Action();

protected:
    double     m_fSensoryRadius;
    TVector2d  m_tVelocity;
};


/******************************************************************************/
/******************************************************************************/

#endif 
