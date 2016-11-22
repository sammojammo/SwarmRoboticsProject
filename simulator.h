#ifndef SIMULATOR_H_
#define SIMULATOR_H_

/******************************************************************************/
/******************************************************************************/

class CSimulator;

#include "arena.h"
#include "agent.h"
#include "experiment.h"

/******************************************************************************/
/******************************************************************************/

#define RED       0
#define GREEN     1
#define BLUE      2
#define HALFRED   3
#define HALFGREEN 4
#define HALFBLUE  5
#define GREY      6
#define YELLOW    7

/******************************************************************************/
/******************************************************************************/

class CSimulator : public CSimObject
{
public:
    CSimulator(unsigned int un_number_of_cycles);
    virtual ~CSimulator();       

    virtual void Run();
    virtual void SetArena(CArena* pc_arena);
    virtual CArena*      GetArena();

    virtual void SetExperiment(CExperiment* pc_experiment);
    virtual CExperiment* GetExperiment();
    
    static CSimulator* GetInstance();

    virtual void AddAgent(CAgent* pc_new_agent);
    virtual void RemoveAgent(CAgent* pc_agent);
    virtual TAgentVector* GetAllAgents();

    virtual void AddAgentToDeleteList(CAgent* pc_agent);
    virtual void EndSimulation();
    virtual void SimulationStep(unsigned int un_step_number);

    virtual unsigned int GetSimulationStepNumber() const;
    virtual inline unsigned int GetNumberOfCycles() {return m_unNumberOfCycles;}


protected:    
    TAgentList         m_tDeleteList;

    CArena*            m_pcArena;
    CExperiment*       m_pcExperiment;
    unsigned int       m_unNumberOfCycles;
    bool               m_bEndSimulation;

    static CSimulator* m_pcSimulator;

    TAgentVector       m_tAllAgents;

    unsigned int       m_unCurrentSimulationStep;
};

/******************************************************************************/
/******************************************************************************/

#endif
