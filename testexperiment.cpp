//bioinstsim2 -a sizex=100,sizey=100,resx=50,resy=50,help -e name=TEST,swarmbehav=AGGREGATION,help -T maxspeed=0.1,count=50,fvsenserange=10,featuresenserange=6,bitflipprob=0.0,help -M,numberoffeatures=8,exchangeprob=0.0,cross-affinity=0.4,help -s 111,help -n 10000,help

//!TODO replace the many preprocessor OPTIMISE switches used in typecasting with some smart typedef

#include <vector>
#include "testexperiment.h"

#include "aggregatebehavior.h"
#include "dispersebehavior.h"
#include "flockbehavior.h"
#include "homingbehavior.h"
#include "randomwalkbehavior.h"
#include "circlebehavior.h"
#include "stopbehavior.h"
#include "stopwhenclosetootheragentbehavior.h"

#ifdef OPTIMISED
#  define CROBOTAGENT CRobotAgentOptimised
#else
#  define CROBOTAGENT CRobotAgent
#endif                            


/******************************************************************************/
/******************************************************************************/

CTestExperiment::CTestExperiment(CArguments* pc_experiment_arguments,
                                 CArguments* pc_arena_arguments,
                                 CArguments* pc_agent_arguments,
                                 CArguments* pc_model_arguments) :
    CExperiment(pc_experiment_arguments, pc_arena_arguments, pc_agent_arguments, pc_model_arguments)
{    
    static bool bHelpDisplayed = false;

    const char* swarmbehav = pc_experiment_arguments->GetArgumentAsStringOr("swarmbehav", "FLOCKING");

    if (strcmp(swarmbehav, "AGGREGATION") == 0)
    {
        m_eswarmbehavType = AGGREGATION;
    }
    else if (strcmp(swarmbehav, "DISPERSION") == 0)
    {
        m_eswarmbehavType = DISPERSION;
    }
    else if (strcmp(swarmbehav, "FLOCKING") == 0)
    {
        m_eswarmbehavType = FLOCKING;
    }
    else if (strcmp(swarmbehav, "HOMING1") == 0)
    {
        m_eswarmbehavType = HOMING1;
    }
    else if (strcmp(swarmbehav, "HOMING2") == 0)
    {
        m_eswarmbehavType = HOMING2;
    }
    else if (strcmp(swarmbehav, "STRLN") == 0)
    {
        m_eswarmbehavType = STRAIGHTLINE;
    }
    else if (strcmp(swarmbehav, "RNDWK") == 0)
    {
        m_eswarmbehavType = RANDOMWK;
    }
    else if (strcmp(swarmbehav, "CIRCLE") == 0)
    {
        m_eswarmbehavType = CIRCLE;
    }
    else if (strcmp(swarmbehav, "STOP") == 0)
    {
        m_eswarmbehavType = STOP;
    }

    const char* errorbehav = pc_experiment_arguments->GetArgumentAsStringOr("errorbehav", "");
    if (strcmp(errorbehav, "STRLN") == 0)
    {
        m_eerrorbehavType = STRAIGHTLINE;
    }
    else if (strcmp(errorbehav, "RNDWK") == 0)
    {
        m_eerrorbehavType = RANDOMWK;
    }
    else if (strcmp(errorbehav, "CIRCLE") == 0)
    {
        m_eerrorbehavType = CIRCLE;
    }
    else if (strcmp(errorbehav, "STOP") == 0)
    {
        m_eerrorbehavType = STOP;
    }
    else if (strcmp(errorbehav, "AGGREGATION") == 0)
    {
        m_eerrorbehavType = AGGREGATION;
    }
    else if (strcmp(errorbehav, "DISPERSION") == 0)
    {
        m_eerrorbehavType = DISPERSION;
    }
    else if (strcmp(errorbehav, "FLOCKING") == 0)
    {
        m_eerrorbehavType = FLOCKING;
    }
    else if (strcmp(errorbehav, "HOMING1") == 0)
    {
        m_eerrorbehavType = HOMING1;
    }
    else if (strcmp(errorbehav, "HOMING2") == 0)
    {
        m_eerrorbehavType = HOMING2;
    }
    else
    {
        m_eerrorbehavType = NOERR;
    }

    m_unMisbehaveStep = pc_experiment_arguments->GetArgumentAsIntOr("misbehavestep", 0);

    m_unNormalAgentToTrack   = pc_experiment_arguments->GetArgumentAsIntOr("tracknormalagent", 0);
    m_unAbnormalAgentToTrack = pc_experiment_arguments->GetArgumentAsIntOr("trackabnormalagent", 15);
    m_unNumAbnormalAgents    = pc_experiment_arguments->GetArgumentAsIntOr("numabnormalagents", 1);
    m_iSwitchNormalBehavior  = pc_experiment_arguments->GetArgumentAsIntOr("switchnormalbehav", 0);
    m_unDurationofSwitch     = pc_experiment_arguments->GetArgumentAsIntOr("durationofswitch", 0);
    m_unChaseAbnormalAgents  = pc_experiment_arguments->GetArgumentAsIntOr("chaseabnormalagents", 0);
    m_fSpreadProbability     = pc_experiment_arguments->GetArgumentAsDoubleOr("spreadprobability", 0.0);
    m_unSpreadPeriod         = pc_experiment_arguments->GetArgumentAsIntOr("spreadperiod", 0);


    if (pc_experiment_arguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        char* pchSwarmBehavior = "INVALID";
        switch (m_eswarmbehavType) {
        case AGGREGATION   : pchSwarmBehavior = "AGGREGATION"; break;
        case DISPERSION    : pchSwarmBehavior = "DISPERSION"; break;
        case FLOCKING      : pchSwarmBehavior = "FLOCKING"; break;
        case HOMING1       : pchSwarmBehavior = "HOMING1"; break;
        case HOMING2       : pchSwarmBehavior = "HOMING2"; break;

        case STRAIGHTLINE  : pchSwarmBehavior = "STRAIGHTLINE"; break;
        case RANDOMWK      : pchSwarmBehavior = "RANDOMWK"; break;
        case CIRCLE        : pchSwarmBehavior = "CIRCLE"; break;
        case STOP          : pchSwarmBehavior = "STOP"; break;

        default:
            pchSwarmBehavior = "UNKNOWN";
        }

        char* pchErrorBehavior = "INVALID";
        switch (m_eerrorbehavType) {
        case STRAIGHTLINE  : pchErrorBehavior = "STRAIGHTLINE"; break;
        case RANDOMWK      : pchErrorBehavior = "RANDOMWK"; break;
        case CIRCLE        : pchErrorBehavior = "CIRCLE"; break;
        case STOP          : pchErrorBehavior = "STOP"; break;
        case NOERR         : pchErrorBehavior = "NOERR"; break;

        case AGGREGATION   : pchErrorBehavior = "AGGREGATION"; break;
        case DISPERSION    : pchErrorBehavior = "DISPERSION"; break;
        case FLOCKING      : pchErrorBehavior = "FLOCKING"; break;
        case HOMING1       : pchErrorBehavior = "HOMING1"; break;
        case HOMING2       : pchErrorBehavior = "HOMING2"; break;

        default:
            pchErrorBehavior = "UNKNOWN";
        }

        printf("swarmbehav=[AGGREGATION,DISPERSION,FLOCKING,HOMING1,HOMING2,STRLN,RNDWK,CIRCLE,STOP] -- behavior selected: %s\n", pchSwarmBehavior);
        printf("errorbehav=[STRLN,RNDWK,CIRCLE,STOP,AGGREGATION,DISPERSION,FLOCKING,HOMING1,HOMING2]                        -- behavior selected: %s\n", pchErrorBehavior);
        printf("misbehavestep=#       Step when agent starts misbehaving [%d]\n",m_unMisbehaveStep);
        printf("tracknormalagent=#    Id of normal agent to track [%d]\n",  m_unNormalAgentToTrack);
        printf("trackabnormalagent=#  Id of abnormal agent to track [%d]\n",m_unAbnormalAgentToTrack);
        printf("numabnormalagents=#   Number of abnormal agents [%d]\n",m_unNumAbnormalAgents);

        printf("switchnormalbehav=#   Set to 1 if normal behavior is to be switched during simulation [%d]\n", m_iSwitchNormalBehavior);
        printf("durationofswitch=#   Set to number of steps the behavior is switched for during simulation [%d]\n", m_unDurationofSwitch);

        printf("chaseabnormalagents=# Set to 1 misbehaving agents should be chased and boxed in [%d]\n", m_unChaseAbnormalAgents);
        printf("spreadprobability=#   Probability of an errorbehav'ing agent spreads its behavior to the nearest swarmbehav'ing neighbor [%1.4f]\n", m_fSpreadProbability);
        printf("spreadperiod=#        How often in simulation cycles that a potential spread of errorbehav should spread (0: never) [%d]\n", m_unSpreadPeriod);

        bHelpDisplayed = true;
    }

    for(int i = 0; i < 20; i++)
    {
        m_pcMisbehaveAgent[i]     = NULL;
    }
    m_pcNormalAgentToTrack = NULL;


    m_ppcListAgentsCreated = new CAgent*[m_unNumberOfAgents];
    for(int i = 0; i < m_unNumberOfAgents; i++)
    {
        m_ppcListAgentsCreated[i]     = NULL;
    }
}

/******************************************************************************/
/******************************************************************************/

CTestExperiment::~CTestExperiment()
{
    delete m_ppcListAgentsCreated;
}

/******************************************************************************/
/******************************************************************************/

CAgent* CTestExperiment::CreateAgent() 
{
    static unsigned int id = 0;
    static CAgent* pcPreviousAgent = NULL;
    
    vector<CBehavior*> vecBehaviors;
    vecBehaviors = GetAgentBehavior(m_eswarmbehavType, pcPreviousAgent);

#ifdef OPTIMISED
    CAgent* pcAgent = new CRobotAgentOptimised("robot", id++, m_pcAgentArguments, m_pcModelArguments, vecBehaviors);
    pcAgent->SetBehavior(m_eswarmbehavType);

    for(int i = 0; i < m_unNumAbnormalAgents; i++)
    {
        if ((id - 1 + i) == m_unAbnormalAgentToTrack)
        {
            m_pcMisbehaveAgent[i] = (CRobotAgentOptimised*) pcAgent;
            pcAgent->SetBehavIdentification(-1); //abnormal agent
        }
    }

    if ((id - 1) == m_unNormalAgentToTrack)
    {
        m_pcNormalAgentToTrack = (CRobotAgentOptimised*) pcAgent;
        pcAgent->SetBehavIdentification(1); // normal agent
    }
#else
    CAgent* pcAgent = new CRobotAgent("robot", id++, m_pcAgentArguments, m_pcModelArguments, vecBehaviors);
    pcAgent->SetBehavior(m_eswarmbehavType);

    for(int i = 0; i < m_unNumAbnormalAgents; i++)
    {
        if ((id - 1 + i) == m_unAbnormalAgentToTrack)
        {
            m_pcMisbehaveAgent[i] = (CRobotAgent*) pcAgent;
            pcAgent->SetBehavIdentification(-1); //abnormal agent
        }
    }

    if ((id - 1) == m_unNormalAgentToTrack)
    {
        m_pcNormalAgentToTrack = (CRobotAgent*) pcAgent;
        pcAgent->SetBehavIdentification(1); // normal agent
    }
#endif

    if(m_eswarmbehavType == HOMING1)
    {
        if (pcPreviousAgent == NULL)
        {
            pcPreviousAgent = pcAgent;
            pcHomeToAgent   = pcAgent;
        }
    }
    else
    {
        pcPreviousAgent = pcAgent;
    }

    m_ppcListAgentsCreated[id-1] = pcAgent;
    return pcAgent;
}

/******************************************************************************/
/******************************************************************************/

void CTestExperiment::PrintVelocityDifference(CAgent* pc_agent, double f_range) 
{
    // Velocity magnitude and direction wrt. surrounding agents:
    TVector2d tTemp    = pc_agent->GetAverageVelocityOfSurroundingAgents(f_range, ROBOT);
    
    float dir_relativeagentvelocity = 0;
    float tmp_agentvelocity         = Vec2dLength((*pc_agent->GetVelocity()));
    float mag_relativeagentvelocity = tmp_agentvelocity - Vec2dLength((tTemp));

    if (Vec2dLength((*pc_agent->GetVelocity())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
        dir_relativeagentvelocity = Vec2dAngle((*pc_agent->GetVelocity()), tTemp);
    else
        dir_relativeagentvelocity = 0.0;


    if(isnan(dir_relativeagentvelocity))
    {
        double cosinevalue = Vec2dCosAngle(tTemp, (*pc_agent->GetVelocity()));

        if(fabs(cosinevalue - 1.0) < 1.0e-3)
            dir_relativeagentvelocity = 0.0;
        else if(fabs(cosinevalue - -1.0) < 1.0e-3)
            dir_relativeagentvelocity = M_PI;
        else
        {
            dir_relativeagentvelocity = acos(cosinevalue);
        }
    }


    printf("%f:[ mag: %f, dir: %f ]", f_range, mag_relativeagentvelocity, dir_relativeagentvelocity);
}

/******************************************************************************/
/******************************************************************************/


void CTestExperiment::PrintStatsForAgent(CAgent* pc_agent)
{
    unsigned int unAgentsWithIn2            = pc_agent->CountAgents(2.0, ROBOT);
    unsigned int unAgentsWithIn4            = pc_agent->CountAgents(4.0, ROBOT);
    unsigned int unAgentsWithIn6            = pc_agent->CountAgents(6.0, ROBOT);
    unsigned int unAgentsWithIn8            = pc_agent->CountAgents(8.0, ROBOT);
    unsigned int unAgentsWithIn10           = pc_agent->CountAgents(10.0, ROBOT);
    
    printf("AgentsWithin - 2: %d, 4: %d, 6: %d, 8: %d, 10: %d, ", unAgentsWithIn2, unAgentsWithIn4, unAgentsWithIn6, unAgentsWithIn8, unAgentsWithIn10);

    double fAverageDistanceToAgentsWithin2  = pc_agent->GetAverageDistanceToSurroundingAgents(2.0, ROBOT);
    double fAverageDistanceToAgentsWithin4  = pc_agent->GetAverageDistanceToSurroundingAgents(4.0, ROBOT);
    double fAverageDistanceToAgentsWithin6  = pc_agent->GetAverageDistanceToSurroundingAgents(6.0, ROBOT);
    double fAverageDistanceToAgentsWithin8  = pc_agent->GetAverageDistanceToSurroundingAgents(8.0, ROBOT);
    double fAverageDistanceToAgentsWithin10 = pc_agent->GetAverageDistanceToSurroundingAgents(10.0, ROBOT);

    printf("AverageDistanceToAgentsWithin - 2: %f, 4: %f, 6: %f, 8: %f, 10: %f, ", fAverageDistanceToAgentsWithin2, fAverageDistanceToAgentsWithin4,
           fAverageDistanceToAgentsWithin6, fAverageDistanceToAgentsWithin8, fAverageDistanceToAgentsWithin10);

    printf("VelocityDifference: ");
    PrintVelocityDifference(pc_agent, 2.0);
    printf(", ");
    PrintVelocityDifference(pc_agent, 4.0);
    printf(", ");
    PrintVelocityDifference(pc_agent, 6.0);
    printf(", ");
    PrintVelocityDifference(pc_agent, 8.0);
    printf(", ");
    PrintVelocityDifference(pc_agent, 10.0);
    printf(", ");

    printf("\n");

}


/******************************************************************************/
/******************************************************************************/


void CTestExperiment::SimulationStep(unsigned int un_step_number)
{
    //    if(un_step_number > MODELSTARTTIME)
    //        for(int agentindex = 0; agentindex < m_unNumberOfAgents; agentindex++)
    //        {
    //            printf("\nStepforfv: %d, Agent_id: %d, Agent_fv: %d",un_step_number,m_ppcListAgentsCreated[agentindex]->GetIdentification(),((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->GetFeatureVector()->GetValue());
    //        }

    if(m_iSwitchNormalBehavior)  // Switching normal behavior during simulation run
    {
        if(un_step_number == 5000U)
            for(int agentindex = 0; agentindex < m_unNumberOfAgents; agentindex++)
            {
                // All agents switch behaviors since this was meant to see tolerance to normal behav and its transitions. No abnormal behavs
                //    #ifdef OPTIMISED
                //                TBehaviorVector vec_behaviors = ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->GetBehaviors();
                //    #else
                //                TBehaviorVector vec_behaviors = ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->GetBehaviors();
                //    #endif
                //                for (TBehaviorVectorIterator i = vec_behaviors.begin(); i != vec_behaviors.end(); i++)
                //                {
                //                    (*i)->SetAgent(NULL);
                //                    //vec_behaviors.pop_back(); //Doesnot seem to work, the pop back function. loop hangs
                //                }
                //                vec_behaviors.clear();

                //                TBehaviorVector vec_newbehaviors = GetAgentBehavior(m_eerrorbehavType, pcHomeToAgent);
                //                //TBehaviorVector vec_newbehaviors = GetAgentBehavior(FLOCKING, pcHomeToAgent);
                //                //TBehaviorVector vec_newbehaviors = GetAgentBehavior(DISPERSION, pcHomeToAgent);

                //    #ifdef OPTIMISED
                //                ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->SetBehavior(m_eerrorbehavType);
                //                ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->SetBehaviors(vec_newbehaviors);
                //    #else
                //                ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->SetBehavior(m_eerrorbehavType);
                //                ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->SetBehaviors(vec_newbehaviors);
                //    #endif
#ifdef OPTIMISED
                TBehaviorVector vec_newbehaviors = GetAgentBehavior(m_eerrorbehavType, pcHomeToAgent);
                ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->ClearBehaviors();

                ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->SetBehavior(m_eerrorbehavType);
                ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->SetBehaviors(vec_newbehaviors);
#else
                TBehaviorVector vec_newbehaviors = GetAgentBehavior(m_eerrorbehavType, pcHomeToAgent);
                ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->ClearBehaviors();

                ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->SetBehavior(m_eerrorbehavType);
                ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->SetBehaviors(vec_newbehaviors);
#endif
            }

        if(un_step_number == 5000U + m_unDurationofSwitch)
            for(int agentindex = 0; agentindex < m_unNumberOfAgents; agentindex++)
            {
                // All agents switch back to aggregation behaviors
                //    #ifdef OPTIMISED
                //                TBehaviorVector vec_behaviors = ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->GetBehaviors();
                //    #else
                //                TBehaviorVector vec_behaviors = ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->GetBehaviors();
                //    #endif
                //                for (TBehaviorVectorIterator i = vec_behaviors.begin(); i != vec_behaviors.end(); i++)
                //                {
                //                    (*i)->SetAgent(NULL);
                //                }
                //                vec_behaviors.clear();




#ifdef OPTIMISED
                TBehaviorVector vec_newbehaviors = GetAgentBehavior(AGGREGATION, pcHomeToAgent);
                ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->ClearBehaviors();

                ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->SetBehavior(AGGREGATION);
                ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->SetBehaviors(vec_newbehaviors);
#else
                TBehaviorVector vec_newbehaviors = GetAgentBehavior(AGGREGATION, pcHomeToAgent);
                ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->ClearBehaviors();

                ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->SetBehavior(AGGREGATION);
                ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->SetBehaviors(vec_newbehaviors);
#endif
            }
    }

    //TODO: DANESH - {DONE}
    // The code below is an example of agents attacking the misbehaving agent
    // The code should be made more general so that any agent detected as
    // behaving abnormally is attacked.
    if (un_step_number > MODELSTARTTIME && m_unChaseAbnormalAgents != 0)
    {
        for(int agentindex = 0; agentindex < m_unNumberOfAgents; agentindex++)
        {
#ifdef OPTIMISED
            if(((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->GetRobotDeactivationState())
                ChaseAndCaptureAgent(((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex]), 5);
#else
            if(((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->GetRobotDeactivationState())
                ChaseAndCaptureAgent(((CRobotAgent*)m_ppcListAgentsCreated[agentindex]), 5);
#endif
        }
    }

    //TODO: DANESH
    // The code below is spreading the errorbehavior
    if (un_step_number > MODELSTARTTIME && un_step_number >= m_unMisbehaveStep){
        if (m_unSpreadPeriod > 0) {
            if ((un_step_number - 1) % m_unSpreadPeriod == 0) {
                SpreadBehavior(m_eerrorbehavType, m_fSpreadProbability); }}}


    if (un_step_number == m_unMisbehaveStep)
    {
        for(int i = 0; i < m_unNumAbnormalAgents; i++)
        {
            vector<CBehavior*> vecBehaviors;
            vecBehaviors = GetAgentBehavior(m_eerrorbehavType, pcHomeToAgent);
            m_pcMisbehaveAgent[i]->SetBehavior(m_eerrorbehavType);
            m_pcMisbehaveAgent[i]->SetBehaviors(vecBehaviors);
        }
    }

#ifndef DISABLEMODEL_RETAINRNDCALLS

    // detailed log of the responses to abnormal agent
    if (m_pcMisbehaveAgent[0] && un_step_number > MODELSTARTTIME)
    {
        unsigned int unToleraters  = 0;
        unsigned int unAttackers   = 0;
        unsigned int unNbrsInSensoryRange = 0;

#ifdef DEBUGCROSSREGULATIONMODELFLAG
        bool dbgflag = true;
#else
        bool dbgflag = false;
#endif

        m_pcMisbehaveAgent[0]->CheckNeighborsResponseToMyFV(&unToleraters, &unAttackers, &unNbrsInSensoryRange, dbgflag);//true
        printf("\nStep: %d, MisbehavingAgentResponse: tol: %d, att: %d, neighboursinsensoryrange: %d", un_step_number, unToleraters, unAttackers, unNbrsInSensoryRange);

        printf("\nMisbehavingAgentFeatureVector: %d\n\n", m_pcMisbehaveAgent[0]->GetFeatureVector()->GetValue());
        //printf("\nMisbehavingAgentStats: ");
        //PrintStatsForAgent(m_pcMisbehaveAgent[0]);
    }

    if(FDMODELTYPE != LINEQ) // lineq - low expected run time; can come back and log more details if needed
    {
        // detailed log of the responses (minimal log) to a single normal agent being tracked
        if (m_pcNormalAgentToTrack && un_step_number > MODELSTARTTIME)
        {
            unsigned int unToleraters = 0;
            unsigned int unAttackers  = 0;
            unsigned int unNbrsInSensoryRange = 0;

#ifdef DEBUGCROSSREGULATIONMODELFLAG
            bool dbgflag = true;
#else
            bool dbgflag = false;
#endif

            m_pcNormalAgentToTrack->CheckNeighborsResponseToMyFV(&unToleraters, &unAttackers, &unNbrsInSensoryRange, dbgflag);
            printf("\nStep: %d, NormalAgentResponse: tol: %d, att: %d, neighboursinsensoryrange: %d", un_step_number, unToleraters, unAttackers, unNbrsInSensoryRange);
            printf("\nNormalAgentFeatureVector: %d\n\n", m_pcNormalAgentToTrack->GetFeatureVector()->GetValue());
            //printf("\nNormalAgentStats: ");
            //PrintStatsForAgent(m_pcNormalAgentToTrack);
        }

        TAgentVector* allagents = this->m_pcSimulator->GetAllAgents();
        TAgentVectorIterator i = allagents->begin();
        printf("\nStep: %d, AgentsFeatureVectors: ", un_step_number);
        while (i != allagents->end())
        {
#ifdef OPTIMISED
            CRobotAgentOptimised* tmp_robotagent  = (CRobotAgentOptimised*) (*i);
#else
            CRobotAgent*          tmp_robotagent  = (CRobotAgent*) (*i);
#endif
            const CFeatureVector* tmp_fv = tmp_robotagent->GetFeatureVector();
            printf("%d %d   ",tmp_robotagent->GetIdentification(), tmp_fv->GetValue());
            i++;
        }
    }
    printf("\n");


    // logging the responses (minimal log) to all the agents
    if(un_step_number > MODELSTARTTIME)
    {
        TAgentVector* allagents = this->m_pcSimulator->GetAllAgents();
        TAgentVectorIterator i = allagents->begin();
        while (i != allagents->end())
        {
#ifdef OPTIMISED
            CRobotAgentOptimised* tmp_robotagent  = (CRobotAgentOptimised*) (*i);
#else
            CRobotAgent*          tmp_robotagent  = (CRobotAgent*) (*i);
#endif
            const CFeatureVector* tmp_fv = tmp_robotagent->GetFeatureVector();

            unsigned int unToleraters  = 0;
            unsigned int unAttackers   = 0;
            unsigned int unNbrsInSensoryRange = 0;

            bool dbgflag = false;

            tmp_robotagent->CheckNeighborsResponseToMyFV(&unToleraters, &unAttackers, &unNbrsInSensoryRange, dbgflag);
            printf("\nResponsestoAllAgents: Step: %d, Id: %d, FV: %d, tol: %d, att: %d, neighboursinsensoryrange: %d", un_step_number, tmp_robotagent->GetIdentification(), tmp_fv->GetValue(), unToleraters, unAttackers, unNbrsInSensoryRange);
            i++;
        }
        printf("\n");
    }
#endif //DISABLEMODEL_RETAINRNDCALLS

#ifdef FLOATINGPOINTOPERATIONS
    //if(un_step_number + 1 == CSimulator::GetInstance()->GetNumberOfCycles())
    {
        TAgentVector* allagents = m_pcSimulator->GetAllAgents();
        TAgentVectorIterator i = allagents->begin();
        printf("\n\nTimeLogFpOp: %u\t", un_step_number);
        while (i != allagents->end())
        {
#ifdef OPTIMISED
            CRobotAgentOptimised* tmp_robotagent  = (CRobotAgentOptimised*) (*i);
#else
            CRobotAgent*          tmp_robotagent  = (CRobotAgent*) (*i);
#endif
            printf("%llu  ",tmp_robotagent->GetNumberFloatingPtOperations());
            i++;
        }
        printf("\n");
    }
#endif

    if(m_unChaseAbnormalAgents != 0 && (un_step_number + 1 == CSimulator::GetInstance()->GetNumberOfCycles()))
    {
        for(int agentindex = 0; agentindex < m_unNumberOfAgents; agentindex++)
        {
#ifdef OPTIMISED
            if(((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->GetRobotDeactivationState())
                printf("\nChaselogs. Agent: %d DeactivationState: 1 Deactivationtime: %d DeactivationCount: %d",agentindex, ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->GetRobotDeactivationTime(),
                       ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->GetConsequentFaultCount());
            else
                printf("\nChaselogs. Agent: %d DeactivationState: 0 Deactivationtime: %d DeactivationCount: %d",agentindex, ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->GetRobotDeactivationTime(),
                       ((CRobotAgentOptimised*)m_ppcListAgentsCreated[agentindex])->GetConsequentFaultCount());
#else
            if(((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->GetRobotDeactivationState())
                printf("\nChaselogs. Agent: %d DeactivationState: 1 Deactivationtime: %d DeactivationCount: %d",agentindex, ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->GetRobotDeactivationTime(),
                       ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->GetConsequentFaultCount());
            else
                printf("\nChaselogs. Agent: %d DeactivationState: 0 Deactivationtime: %d DeactivationCount: %d",agentindex, ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->GetRobotDeactivationTime(),
                       ((CRobotAgent*)m_ppcListAgentsCreated[agentindex])->GetConsequentFaultCount());
#endif
        }
    }
}

/******************************************************************************/
/******************************************************************************/


void CTestExperiment::ChaseAndCaptureAgent(CAgent* pc_agent_to_chase, unsigned int un_agents_to_assign)
{
    /* At the moment we only mark agents that are to be chased*/
    return;

    /*
    TAgentVector tSortedAgents;
    pc_agent_to_chase->SortAllAgentsAccordingToDistance(&tSortedAgents);
    for (int i = 1; i <= un_agents_to_assign; i++) {
        TBehaviorVector vec_behaviors;
        double fArenaSizeX;
        double fArenaSizeY;
        CSimulator::GetInstance()->GetArena()->GetSize(&fArenaSizeX, &fArenaSizeY);
        vec_behaviors.push_back(new CHomingBehavior(max(fArenaSizeX, fArenaSizeY), pc_agent_to_chase));
        
#ifdef OPTIMISED
        ((CRobotAgentOptimised*)tSortedAgents[i])->SetBehaviors(vec_behaviors);
#else
        ((CRobotAgent*)tSortedAgents[i])->SetBehaviors(vec_behaviors);
#endif
    }*/
}
/******************************************************************************/
/******************************************************************************/

void CTestExperiment::SpreadBehavior(ESwarmBehavType e_behavior, double f_probability)
{
    TAgentList listInfectedAgents;
    
    TAgentVector* vecAllAgents = CSimulator::GetInstance()->GetAllAgents();
    for (TAgentVectorIterator i = vecAllAgents->begin(); i != vecAllAgents->end(); i++)
    {
        if ((*i)->GetType() == ROBOT)
        {
            if (((CROBOTAGENT*) (*i))->GetBehavior() == e_behavior)
            {
                listInfectedAgents.push_back(*i);
            }
        }
    }

    for (TAgentListIterator j = listInfectedAgents.begin(); j != listInfectedAgents.end(); j++)
    {
        if (Random::nextDouble() < f_probability)
        {
            
            TAgentVector tSortedAgents;
            (*j)->SortAllAgentsAccordingToDistance(&tSortedAgents);
            
            unsigned index = 1;
            bool     found = false;
            do {
                if (((CROBOTAGENT*) tSortedAgents[index])->GetBehavior() != e_behavior)
                {
                    ((CROBOTAGENT*)tSortedAgents[index])->SetBehavior(e_behavior);

                    ((CROBOTAGENT*)tSortedAgents[index])->SetBehaviors(GetAgentBehavior(e_behavior, NULL));
                    found = true;
                } else {
                    index++;
                }
            } while (index < tSortedAgents.size() && !found);
        }
    }
}

/******************************************************************************/
/******************************************************************************/

vector<CBehavior*> CTestExperiment::GetAgentBehavior(ESwarmBehavType swarmbehavType, CAgent*  previousAgent)
{
    vector<CBehavior*> vecBehaviors;

    //    vecBehaviors.push_back(new CStopWhenCloseToOtherAgentBehavior(CAgent::RADIUS * 3));

    //behav. pushed is decreasing order of priority to take control of the agent

    // Dispersion behavior
    // Parameters: Dispersion range (d)
    // Robots disperse with the distance between them decided by dispersion range d in CDisperseBehavior(d)
    // Note that for large value of d (e.g. 50), the robots collapse into each other!
    if(swarmbehavType == DISPERSION)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(3.0);
        vecBehaviors.push_back(pcDisperseBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    // Aggregation behavior: formation of clusters of robots
    // Parameters: Dispersion range (d), Aggregation range (a)
    // The number of robots in each cluster seems to be proportional to d/a
    else if(swarmbehavType == AGGREGATION)
    {
        CDisperseBehavior* pcDisperseBehavior2 = new CDisperseBehavior(3.0); //1
        vecBehaviors.push_back(pcDisperseBehavior2);
        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(10);
        vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    // Homing behavior 2: Ind. robots follow the robot that was placed immediately before them or the first robot
    // Parameters: Dispersion range (d), Homing range (h)
    // The dispersion range d > 0, else robots collapse into their individual leaders
    // If dispersion range d is too high (e.g. 5), other robots besides the leader disrupt the homing behavior - resulting in slowly moving clusters of robots
    else if(swarmbehavType == HOMING2)
    {
        printf("\nHOMING2 behav. not defined any more\n"); exit(-1);

        /*CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(3);
        vecBehaviors.push_back(pcDisperseBehavior);
        CHomingBehavior* pcHomingBehavior = new CHomingBehavior(10000, previousAgent);
        vecBehaviors.push_back(pcHomingBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);*/
    }

    // Homing behavior 1: All other robots follow a single leader bot
    // The robots form a single cluster around the leader-bot. The bahavior is quite similar to the aggregation behav.
    // The dispersion range d > 0 else robots collapse into leader.
    // Increasing d increases the area occupied by the cluster
    else if(swarmbehavType == HOMING1)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(3.0);
        vecBehaviors.push_back(pcDisperseBehavior);
        CHomingBehavior* pcHomingBehavior = new CHomingBehavior(100.0, previousAgent);
        vecBehaviors.push_back(pcHomingBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    // Flocking behavior:
    // Parameters: Dispersion range (d), Flocking range (f)
    // Dispersion range d is proportional to the size of individual flock of robots
    // Flocking range f influences how fast a flock is formed. Also f > d to initiate flocking
    else if(swarmbehavType == FLOCKING)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(1.0); //5
        vecBehaviors.push_back(pcDisperseBehavior);
        CFlockBehavior* pcFlockBehavior = new CFlockBehavior(3.0); //10
        vecBehaviors.push_back(pcFlockBehavior);
        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(5.0);
        vecBehaviors.push_back(pcAggregateBehavior);
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }
    else if(swarmbehavType == STRAIGHTLINE)
    {
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }
    else if(swarmbehavType == RANDOMWK)
    {
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.01);
        vecBehaviors.push_back(pcRandomWalkBehavior);
    }
    else if(swarmbehavType == CIRCLE)
    {
        CCircleBehavior* pcCircleBehavior = new CCircleBehavior();
        vecBehaviors.push_back(pcCircleBehavior);
    }
    else if(swarmbehavType == STOP)
    {
        CStopBehavior* pcStopBehavior = new CStopBehavior();
        vecBehaviors.push_back(pcStopBehavior);
    }
    else
    {
        printf("\n No behavior selected");
        exit(-1);
    }

    return vecBehaviors;
}

/******************************************************************************/
/******************************************************************************/
