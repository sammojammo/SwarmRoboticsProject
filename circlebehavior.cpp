#include "circlebehavior.h"


/******************************************************************************/
/******************************************************************************/

CCircleBehavior::CCircleBehavior()
{
    m_ftheta = 0.0;
}

/******************************************************************************/
/******************************************************************************/

bool CCircleBehavior::TakeControl()
{
    return true;
}

/******************************************************************************/
/******************************************************************************/

void CCircleBehavior::Action()
{
    // circle of diameter 1 unit (with 32 tim-steps for a rotation)
    m_ftheta += M_PI*2.0 / 32.0;//10.0;//200.0;

    if (m_ftheta > M_PI*2.0)
        m_ftheta -= M_PI*2.0;

    TVector2d newvelocity;
    newvelocity.x = m_pcAgent->GetMaximumSpeed() * cos(m_ftheta);
    newvelocity.y = m_pcAgent->GetMaximumSpeed() * sin(m_ftheta);

    m_pcAgent->SetVelocity(&newvelocity);
}

/******************************************************************************/
/******************************************************************************/

void CCircleBehavior::SetAgent(CAgent* pc_agent)
{
    CBehavior::SetAgent(pc_agent);
}

/******************************************************************************/
/******************************************************************************/
