#include "populationanalyzer.h"
#include "simulator.h"
#include "agent.h"

#include <vector>

/******************************************************************************/
/******************************************************************************/

CPopulationAnalyzer::CPopulationAnalyzer(CArguments* pc_arguments) : 
    m_pcArguments(pc_arguments),
    CSimObject("PopulationAnalyzer")
{

    m_unFullUpdatePeriod  = m_pcArguments->GetArgumentAsIntOr("updateperiod", 100);
    m_unBriefUpdatePeriod = m_pcArguments->GetArgumentAsIntOr("updateperiod", 100);

    m_unFullUpdatePeriod  = m_pcArguments->GetArgumentAsIntOr("fullupdateperiod",  m_unFullUpdatePeriod);
    m_unBriefUpdatePeriod = m_pcArguments->GetArgumentAsIntOr("briefupdateperiod", m_unBriefUpdatePeriod);

    m_unMacroModeImmuneThreshold = m_pcArguments->GetArgumentAsIntOr("macromodeimmunethreshold", 17);//Threshold conc. = 2.15, and range of robot is 1.6. Consequently, macromodeimmunethreshold=2.15*pi*1.6*1.6

    m_bAnalyzeSingleAgent = m_pcArguments->GetArgumentIsDefined("singleagent");

    if (m_pcArguments->GetArgumentIsDefined("help"))
    {
        printf("Population analyzer setup:\n"
               " fullupdateperiod=#              Set the number simulation steps between full output (%d)\n"
               " briefupdateperiod=#             Set the number simulation steps between brief output (%d)\n"
               " singleagent                     Output only stats for a single agent\n"
               " macromodeimmunethreshold=#      Set the threshold for the maximum number the number of APCs\n"
               "                                 detected by an agent that should lead to an immune response [%d]\n",
               m_unFullUpdatePeriod,
               m_unBriefUpdatePeriod,
               m_unMacroModeImmuneThreshold);
    }

    m_unStepsSinceLastFullUpdate  = m_unFullUpdatePeriod;
    m_unStepsSinceLastBriefUpdate = m_unBriefUpdatePeriod;
}

/******************************************************************************/
/******************************************************************************/

void CPopulationAnalyzer::SimulationStep(unsigned int un_step_number) 
{
    if (m_unStepsSinceLastBriefUpdate >= m_unBriefUpdatePeriod && m_unBriefUpdatePeriod > 0) 
    {       
        m_unStepsSinceLastBriefUpdate = 1;
    } else {
        m_unStepsSinceLastBriefUpdate++;        
    }

    if (m_unStepsSinceLastFullUpdate >= m_unFullUpdatePeriod && m_unFullUpdatePeriod > 0) 
    {
        m_unStepsSinceLastFullUpdate = 1;
    } else {
        m_unStepsSinceLastFullUpdate++;
    }
}

/******************************************************************************/
/******************************************************************************/


