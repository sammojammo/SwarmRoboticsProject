#ifndef HOMINGBEHAVIOR_H_
#define HOMINGBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

class CHomingBehavior : public CBehavior 
{
public:
    CHomingBehavior(double f_sensory_radius, CAgent* pc_agent_to_follow);
    
    virtual void SimulationStep();
    virtual bool TakeControl();
    virtual void Action();

protected:
    double     m_fSensoryRadius;
    TVector2d  m_tCenterOfMass;
    CAgent*    m_pcAgentToFollow;
};


/******************************************************************************/
/******************************************************************************/

#endif 
