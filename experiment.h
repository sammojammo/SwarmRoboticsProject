#ifndef EXPERIMENT_H_
#define EXPERIMENT_H_

/******************************************************************************/
/******************************************************************************/

class CExperiment;

#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

class CExperiment : public CSimObject 
{
public:
    CExperiment(CArguments* pc_experiment_arguments,
                CArguments* pc_arena_arguments,
                CArguments* pc_agent_arguments,
                CArguments* pc_model_arguments);
    virtual ~CExperiment();

    // The following methods creates the various parts of the
    // simulation environment. The CreateSimulator method calls the
    // others.
    virtual CSimulator* CreateSimulator(unsigned int un_number_of_cycles);

protected:
    virtual CArena*     CreateArena();
    virtual CAgent*     CreateAgent();
    virtual void        CreateAndAddAgents(CSimulator* pc_simulator);
    
    virtual void        PlaceAgentRandomly(CAgent* pc_agent);
    virtual void        PlaceAgentRandomly(CAgent* pc_agent, double f_origin_x, double f_origin_y, double f_radius);
    virtual void        PlaceAgentRandomly(CAgent* pc_agent, double f_origin_x, double f_origin_y, double f_inner_radius, double f_outer_radius);


protected:
    CArguments*         m_pcExperimentArguments;
    CArguments*         m_pcArenaArguments;
    CArguments*         m_pcAgentArguments;
    CArguments*         m_pcModelArguments;

    CArena*             m_pcArena;
    CSimulator*         m_pcSimulator;

    unsigned int        m_unNumberOfAgents;    
    unsigned int        m_unNumberOfColors;
};

/******************************************************************************/
/******************************************************************************/

#endif
