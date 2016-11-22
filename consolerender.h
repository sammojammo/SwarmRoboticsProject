#ifndef CONSOLRENDER_H_
#define CONSOLRENDER_H_

/******************************************************************************/
/******************************************************************************/

#include "common.h"
#include "render.h"

/******************************************************************************/
/******************************************************************************/

class CConsolRender : public CRender
{
public:
    CConsolRender(const char* pc_name);
    virtual ~CConsolRender(const char* pc_name);

    virtual void SimulationStep(unsigned int un_step_number);

protected:

}

/******************************************************************************/
/******************************************************************************/

#endif
