#ifndef CIRCLEBEHAVIOR_H
#define CIRCLEBEHAVIOR_H

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

class CCircleBehavior : public CBehavior
{
public:
    CCircleBehavior();

    virtual bool TakeControl();
    virtual void Action();
    virtual void SetAgent(CAgent* pc_agent);


protected:
    double m_ftheta;
};

/******************************************************************************/
/******************************************************************************/


#endif // CIRCLEBEHAVIOR_H
