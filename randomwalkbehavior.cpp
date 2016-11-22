#include "randomwalkbehavior.h"
#include "random.h"


/******************************************************************************/
/******************************************************************************/

CRandomWalkBehavior::CRandomWalkBehavior(double f_change_direction_probability) :
    m_fChangeDirectionProbability(f_change_direction_probability) 
{    
}

/******************************************************************************/
/******************************************************************************/
    
bool CRandomWalkBehavior::TakeControl() 
{
    return true;
}

/******************************************************************************/
/******************************************************************************/

void CRandomWalkBehavior::Action()
{
    if (m_fChangeDirectionProbability >= Random::nextDouble())
        m_pcAgent->SetRandomVelocity();
}

/******************************************************************************/
/******************************************************************************/
