#ifndef POPULATIONANALYZER_H_
#define POPULATIONANALYZER_H_

/******************************************************************************/
/******************************************************************************/

#include "simobject.h"

/******************************************************************************/
/******************************************************************************/


class CPopulationAnalyzer : public CSimObject 
{
public:
    CPopulationAnalyzer(CArguments* pc_arguments);
    
    virtual void SimulationStep(unsigned int un_step_number);

    
protected:
    CArguments*     m_pcArguments;
    unsigned int    m_unFullUpdatePeriod;
    unsigned int    m_unBriefUpdatePeriod;
    unsigned int    m_unStepsSinceLastFullUpdate;
    unsigned int    m_unStepsSinceLastBriefUpdate;
    unsigned int    m_unMacroModeImmuneThreshold;

    bool            m_bAnalyzeSingleAgent;

};


/******************************************************************************/
/******************************************************************************/

#endif
