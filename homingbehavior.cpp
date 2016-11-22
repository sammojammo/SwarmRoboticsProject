#include "homingbehavior.h"

/******************************************************************************/
/******************************************************************************/

CHomingBehavior::CHomingBehavior(double f_sensory_radius, CAgent* pc_agent_to_follow) : 
    m_fSensoryRadius(f_sensory_radius), m_pcAgentToFollow(pc_agent_to_follow)
{
}

/******************************************************************************/
/******************************************************************************/
    
bool CHomingBehavior::TakeControl() 
{
    if (m_pcAgentToFollow == NULL)
        return false;

    //return GetDistanceBetweenPositions(m_pcAgentToFollow->GetPosition(), m_pcAgent->GetPosition());

    return GetDistanceBetweenPositions(m_pcAgentToFollow->GetPosition(), m_pcAgent->GetPosition()) <=     m_fSensoryRadius;
}

/******************************************************************************/
/******************************************************************************/

void CHomingBehavior::SimulationStep() 
{
}

/******************************************************************************/
/******************************************************************************/

void CHomingBehavior::Action()
{
    m_pcAgent->MoveTowards((*m_pcAgentToFollow->GetPosition()), m_pcAgent->GetMaximumSpeed());
}

/******************************************************************************/
/******************************************************************************/
