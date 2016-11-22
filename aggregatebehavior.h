#ifndef AGGREGATEBEHAVIOR_H_
#define AGGREGATEBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

class CAggregateBehavior : public CBehavior 
{
public:
    CAggregateBehavior(double f_sensory_radius);
    
    virtual void SimulationStep();
    virtual bool TakeControl();
    virtual void Action();

protected:
    double     m_fSensoryRadius;
    TVector2d  m_tCenterOfMass;
};


/******************************************************************************/
/******************************************************************************/

#endif 
