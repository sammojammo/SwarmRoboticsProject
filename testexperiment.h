#ifndef TESTEXPERIMENT_H_
#define TESTEXPERIMENT_H_

/******************************************************************************/
/******************************************************************************/

#include "robotagent.h"
#include "robotagent_optimised.h"
#include "experiment.h"

/******************************************************************************/
/******************************************************************************/

#define OPTIMISED

/******************************************************************************/
/******************************************************************************/

class CTestExperiment : public CExperiment
{
public:
    CTestExperiment(CArguments* pc_experiment_arguments,
                    CArguments* pc_arena_arguments,
                    CArguments* pc_agent_arguments,
                    CArguments* pc_model_arguments);
    ~CTestExperiment();
    
    virtual CAgent*     CreateAgent();
    virtual void SimulationStep(unsigned int un_step_number);
   
protected:
    virtual void PrintStatsForAgent(CAgent* pc_agent);
    virtual void PrintVelocityDifference(CAgent* pc_agent, double f_range);
    vector<CBehavior*> GetAgentBehavior(ESwarmBehavType swarmbehavType, CAgent*  previousAgent);

    virtual void ChaseAndCaptureAgent(CAgent* pc_agent_to_chase, unsigned int un_agents_to_assign);
    virtual void SpreadBehavior(ESwarmBehavType e_behavior, double f_probability);

    ESwarmBehavType m_eswarmbehavType, m_eerrorbehavType;
    //EErrorBehavType m_eerrorbehavType;

    unsigned int    m_unMisbehaveStep;
    unsigned int    m_unNormalAgentToTrack;
    unsigned int    m_unAbnormalAgentToTrack;
    unsigned int    m_unNumAbnormalAgents;
    int             m_iSwitchNormalBehavior;
    unsigned int    m_unDurationofSwitch;

    unsigned int    m_unChaseAbnormalAgents;
    double          m_fSpreadProbability;     
    unsigned int    m_unSpreadPeriod;         

#ifdef OPTIMISED
    CRobotAgentOptimised*    m_pcMisbehaveAgent[20];
    CRobotAgentOptimised*    m_pcNormalAgentToTrack;
#else
    CRobotAgent*    m_pcMisbehaveAgent[20];
    CRobotAgent*    m_pcNormalAgentToTrack;
#endif

    CAgent**        m_ppcListAgentsCreated;

    CAgent*         pcHomeToAgent;


};

/******************************************************************************/
/******************************************************************************/

#endif
