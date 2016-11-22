#ifndef DISPERSEBEHAVIOR_H_
#define DISPERSEBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

class CDisperseBehavior : public CBehavior 
{
public:
    CDisperseBehavior(double f_sensory_radius);

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
