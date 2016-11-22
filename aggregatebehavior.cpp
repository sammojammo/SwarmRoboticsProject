#include "aggregatebehavior.h"

/******************************************************************************/
/******************************************************************************/

CAggregateBehavior::CAggregateBehavior(double f_sensory_radius) : 
    m_fSensoryRadius(f_sensory_radius) 
{
}

/******************************************************************************/
/******************************************************************************/
    
bool CAggregateBehavior::TakeControl() 
{
    return m_pcAgent->CountAgents(m_fSensoryRadius, ROBOT) > 0;
}

/******************************************************************************/
/******************************************************************************/

void CAggregateBehavior::SimulationStep() 
{
    m_tCenterOfMass = m_pcAgent->GetCenterOfMassOfSurroundingAgents(m_fSensoryRadius, ANY);
}

/******************************************************************************/
/******************************************************************************/

void CAggregateBehavior::Action()
{
//    if (m_tCenterOfMass.x != 0.0 && m_tCenterOfMass.y != 0.0) 
//    {
        double fDist  = GetDistanceBetweenPositions(&m_tCenterOfMass, m_pcAgent->GetPosition());
        double fSpeed = min(fDist, m_pcAgent->GetMaximumSpeed());

        m_pcAgent->MoveTowards(m_tCenterOfMass, fSpeed);
//    }
}

/******************************************************************************/
/******************************************************************************/
