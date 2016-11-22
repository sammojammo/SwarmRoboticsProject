#ifndef STOPWHENCLOSETOOTHERAGENTBEHAVIOR_H
#define STOPWHENCLOSETOOTHERAGENTBEHAVIOR_H

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

class CStopWhenCloseToOtherAgentBehavior : public CBehavior
{
public:
    CStopWhenCloseToOtherAgentBehavior(double f_distance);

    virtual bool TakeControl();
    virtual void Action();
    virtual void SetAgent(CAgent* pc_agent);


protected:
    double   m_fDistance;
};

/******************************************************************************/
/******************************************************************************/

#endif // STOPWHENCLOSETOOTHERAGENTBEHAVIOR_H
