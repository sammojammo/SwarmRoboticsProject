#include "robotagent_optimised.h"
#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

CRobotAgentOptimised::CRobotAgentOptimised(const char* pch_name, unsigned int un_identification, CArguments* pc_arguments, CArguments* pc_model_arguments, TBehaviorVector vec_behaviors) :
        CAgent(pch_name, un_identification, pc_arguments), m_vecBehaviors(vec_behaviors)
{
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
        (*i)->SetAgent(this);

    crminAgent = NULL;
    if(FDMODELTYPE == CRM || FDMODELTYPE == CRM_TCELLSINEXCESS)
        crminAgent = new CRMinRobotAgentOptimised(this, pc_model_arguments);
    else if(FDMODELTYPE == THRESHOLDONFVDIST)
        thresholdinAgent = new ThresholdinRobotAgentOptimised(this, pc_model_arguments);
    else { printf("\nUnknown model type"); exit(-1);}

    m_pcFeatureVector   = new CFeatureVector(this);

    m_fWeight = 0.0;

    static bool bHelpDisplayed = false;

    //control at what distances agents can sense one another when FVs have to be communicated
    // now made redundant with selectnumnearestnbrs
    m_fFVSenseRange               = pc_arguments->GetArgumentAsDoubleOr("fvsenserange", 10.0);

    //at what distances agents are considered neighbors when the individual features are computed
    CFeatureVector::FEATURE_RANGE = pc_arguments->GetArgumentAsDoubleOr("featuresenserange", 6.0);

    m_fResponseRange              = pc_arguments->GetArgumentAsDoubleOr("responserange", m_fFVSenseRange);
    m_uSelectedNumNearestNbrs     = pc_arguments->GetArgumentAsIntOr("selectnumnearestnbrs", 10);
    m_uNumVotingNbrs              = pc_arguments->GetArgumentAsIntOr("numvotingnbrs", 10);


    if(pc_arguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
        printf("fvsenserange=#.#              Range at which other agents' FVs are sensed [%f]\n"
               "featuresenserange=#.#         Range based on which features are computed  [%f]\n"
               "responserange=#.#             Range at which a robot \"reponds\" to other features [%f]\n"
               "selectnumnearestnbrs=#        The number of nearest neighbours for FV sensing and T-cell diffusion (makes fvsenserange redundant) [%d]\n"
               "numvotingnbrs=#               The number of nearest neighbours for voting an agent abnormal) [%d]\n"
               ,
               m_fFVSenseRange,
               CFeatureVector::FEATURE_RANGE,
               m_fResponseRange,
               m_uSelectedNumNearestNbrs,
               m_uNumVotingNbrs
               );


//    m_pfFeaturesSensed  = new float[CFeatureVector::NUMBER_OF_FEATURE_VECTORS];
//    m_pbMostWantedList = new unsigned int[CFeatureVector::NUMBER_OF_FEATURE_VECTORS];
//    for (unsigned int i = 0; i < CFeatureVector::NUMBER_OF_FEATURE_VECTORS; i++)
//    {
//        m_pbMostWantedList[i] = 0;
//    }

    m_bRobotDeactivated = false;
    m_iDEactivationTime = -1;
    m_unConseqDetectedFaulty = 0;

    m_uNumberFloatingPtOperations = 0;
}

/******************************************************************************/
/******************************************************************************/

CRobotAgentOptimised::~CRobotAgentOptimised()
{
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
        delete (*i);

    delete m_pcFeatureVector;
    listFVsSensed.clear();
//    delete m_pfFeaturesSensed;
//    delete m_pbMostWantedList;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::SimulationStepUpdatePosition()
{
    bool bControlTaken = false;
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        (*i)->SimulationStep();
        if (!bControlTaken)
        {
            bControlTaken = (*i)->TakeControl();
            if (bControlTaken)
                (*i)->Action();
        } else
            (*i)->Suppress();
    }

    // Update the model (T-cells of the CRM instance for this robot), CTRNN neuron activations, lineq on fvs
    m_pcFeatureVector->SimulationStep();
    unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();

#ifdef DEBUGFEATUREVECTORFLAG
    if (m_iBehavIdentification == 1)
        printf("\nStep: %d, FV for normal agent %d: %s\n", CurrentStepNumber, m_unIdentification, m_pcFeatureVector->ToString().c_str());

    if (m_iBehavIdentification == -1)
        printf("\nStep: %d, FV for abnormal agent %d: %s\n", CurrentStepNumber, m_unIdentification, m_pcFeatureVector->ToString().c_str());
#endif

    Sense(GetSelectedNumNearestNbrs());

    if(CurrentStepNumber > MODELSTARTTIME)
        if(FDMODELTYPE == CRM || FDMODELTYPE == CRM_TCELLSINEXCESS)
            crminAgent->SimulationStepUpdatePosition();
        else if(FDMODELTYPE == THRESHOLDONFVDIST)
            thresholdinAgent->SimulationStepUpdatePosition();

    CAgent::SimulationStepUpdatePosition();
}

/******************************************************************************/
/******************************************************************************/

CRobotAgentOptimised* CRobotAgentOptimised::GetRandomRobotWithWeights(double f_range)
{
    TAgentListList tAgentListList;
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);

    if (tAgentListList.size() == 0)
    {
        ERROR2("This should never happen - the agent list-list is empty - maybe the position of the agent is wrong (%f,%f)",
               m_tPosition.x,
               m_tPosition.y);
    }

    double fWeightSum = CountWeightsInAgentListList(&tAgentListList, f_range);
    if (fWeightSum < 1e-10)
        return NULL;

    double fSelectedWeight  = Random::nextDouble() * fWeightSum;

    TAgentListListIterator i = tAgentListList.begin();
    CAgent* pcAgentSelected  = NULL;

    do
    {
        while ((*i)->size() == 0)
        {
            i++;
        }
        TAgentListIterator j = (*i)->begin();

        while (j != (*i)->end() && fSelectedWeight > 0.0)
        {
            if ((*j)->m_bTempWithInRange)
                fSelectedWeight -= ((CRobotAgentOptimised*) (*j))->GetWeight();
            if (fSelectedWeight > 0.0)
                j++;
        }

        if (fSelectedWeight > 0.0)
            i++;
        else
        {
            if ((*j)->m_bTempWithInRange)
            {
                pcAgentSelected = (*j);
            } else {
                while (j != (*i)->end() && pcAgentSelected == NULL)
                {
                    if ((*j)->m_bTempWithInRange)
                    {
                        pcAgentSelected = (*j);
                    }
                    else
                    {
                        j++;
                    }
                }

                if (pcAgentSelected == NULL)
                    i++;
            }

        }
    } while (pcAgentSelected == NULL && i != tAgentListList.end());

    if (i == tAgentListList.end())
        ERROR("The random generator seems to be wrong");

    return (CRobotAgentOptimised*) pcAgentSelected;
}

/******************************************************************************/
/******************************************************************************/

double CRobotAgentOptimised::CountWeightsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range)
{
    double fReturn = 0.0;
    TAgentListListIterator i;

    double fSquareRange = f_range * f_range;

    for (i = ptlist_agent_list_list->begin(); i != ptlist_agent_list_list->end(); i++)
    {
        TAgentListIterator j;
        for (j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if ((*j)->GetType() == ROBOT && (*j) != this)
            {
                (*j)->m_bTempWithInRange = (GetSquaredDistanceBetweenPositions(&m_tPosition, (*j)->GetPosition()) <= fSquareRange);
                if ((*j)->m_bTempWithInRange)
                    fReturn += ((CRobotAgentOptimised*) (*j))->GetWeight();
            } else {
                (*j)->m_bTempWithInRange = false;
            }
        }
    }

    return fReturn;
}

/******************************************************************************/
/******************************************************************************/

CRobotAgentOptimised* CRobotAgentOptimised::GetRandomRobotWithWeights(unsigned int u_nearestnbrs)
{
    TAgentVector tSortedAgents;

    SortAllAgentsAccordingToDistance(&tSortedAgents);

    double fWeightSum = 0.0;
    // 1-11 if u_nearestnbrs is 10, because agent at index 0 is ourselves:
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgentOptimised* pcRobot = (CRobotAgentOptimised*) tSortedAgents[i];
        fWeightSum += pcRobot->GetWeight();
    }

    if (fWeightSum < 1e-10)
        return NULL;

    double fSelectedWeight  = Random::nextDouble() * fWeightSum;


    CAgent* pcAgentSelected  = NULL;
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgentOptimised* pcRobot = (CRobotAgentOptimised*) tSortedAgents[i];
        fSelectedWeight -= pcRobot->GetWeight();

        if(fSelectedWeight <= 0.0)
        {
            pcAgentSelected = pcRobot;
            break;
        }
    }
    return (CRobotAgentOptimised*) pcAgentSelected;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::SetWeight(double f_weight)
{
    m_fWeight = f_weight;
}

/******************************************************************************/
/******************************************************************************/

double CRobotAgentOptimised::GetWeight() const
{
    return m_fWeight;
}

/******************************************************************************/
/******************************************************************************/

double CRobotAgentOptimised::GetFVSenseRange() const
{
    return m_fFVSenseRange;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::Sense(unsigned int u_nearestnbrs)
{
    listFVsSensed.clear();
    TAgentVector tSortedAgents;
    SortAllAgentsAccordingToDistance(&tSortedAgents);

    // 1-11 because agent at index 0 is ourselves:
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgentOptimised* pcRobot = (CRobotAgentOptimised*) tSortedAgents[i];
        UpdateFeatureVectorDistribution(pcRobot->GetFeatureVector()->GetValue(), 1.0);
    }
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::UpdateFeatureVectorDistribution(unsigned int fv, double increment)
{
    list<structFVsSensed>::iterator it;

    // check if fv is in listFVsSensed
    // if so, update the value it holds by increment
    // if not insert it (while keeping list sorted based on fv) and initialize its value by increment
    for (it = listFVsSensed.begin(); it != listFVsSensed.end(); ++it)
    {
        if((*it).uFV == fv)
        {
            // if fv is already present
            (*it).fRobots += increment;
            return;
        }

        if((*it).uFV > fv)
        {   // we assume the list is kept sorted.
            // if fv is absent
            listFVsSensed.insert(it, structFVsSensed(fv, increment));
            return;
        }
    }

    // when the list is empty
    listFVsSensed.push_back(structFVsSensed(fv, increment));
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::PrintFeatureVectorDistribution(unsigned int id)
{
    if(!(GetIdentification() == id))
        return;

    list<structFVsSensed>::iterator it;
    printf("\n====R%d Feature Vector Distribution=====\n",GetIdentification());
    for (it = listFVsSensed.begin(); it != listFVsSensed.end(); ++it)
        printf("FV:%d,Robots:%f, ",(*it).uFV, (*it).fRobots);
}

/******************************************************************************/
/******************************************************************************/

unsigned int CRobotAgentOptimised::GetColor()
{
    unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();
    if(CurrentStepNumber > MODELSTARTTIME)
    {
        unsigned int unToleraters  = 0;
        unsigned int unAttackers   = 0;
        unsigned int unNbrsInSensoryRange = 0;

        CheckNeighborsResponseToMyFV(&unToleraters, &unAttackers, &unNbrsInSensoryRange, false);
        if(unToleraters > unAttackers)
            return GREEN;
        else
            return RED;
    }
    //changing initial colours so tracked normal agents are dark green, abnormal agent is dark red
    if(GetIdentification() == 1)
        return HALFGREEN;
    else if(GetIdentification() == 3)
        return HALFGREEN;
    if(GetIdentification() == 15)
        return HALFRED;

    return BLUE;

}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::SetBehaviors(TBehaviorVector vec_behaviors)
{
    m_vecBehaviors = vec_behaviors;

    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        (*i)->SetAgent(this);
    }
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::ClearBehaviors()
{
    m_vecBehaviors.clear();
}

/******************************************************************************/
/******************************************************************************/

inline TBehaviorVector CRobotAgentOptimised::GetBehaviors()
{
    return m_vecBehaviors;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::SetMostWantedList(list<structFVsSensed>::iterator* it, unsigned int state)
{
    list<structFVsSensed>::iterator it_fvsensed = (*it);

    (*it_fvsensed).uMostWantedState = state;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CRobotAgentOptimised::GetMostWantedState(unsigned int fv)
{
    list<structFVsSensed>::iterator it_fvsensed;

    for(it_fvsensed = listFVsSensed.begin(); it_fvsensed != listFVsSensed.end(); ++it_fvsensed)
        if((*it_fvsensed).uFV == fv)
            return (*it_fvsensed).uMostWantedState;

    return 3; // if fv is not in the list (not sensed)
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::CheckNeighborsResponseToMyFV(unsigned int* pun_number_of_toleraters, unsigned int* pun_number_of_attackers, unsigned int* pun_number_of_neighborsinsensoryrange, bool b_logs)
{
    (*pun_number_of_toleraters)  = 0;
    (*pun_number_of_attackers)   = 0;
    (*pun_number_of_neighborsinsensoryrange) = 0;


    TAgentVector tSortedAgents;

    SortAllAgentsAccordingToDistance(&tSortedAgents);

    // 1-11 because agent at index 0 is ourselves:
    bool m_battackeragentlog=true, m_btolerateragentlog=true;
    for (unsigned int nbrs = 1; nbrs < m_uNumVotingNbrs+1; nbrs++)
    {
        CRobotAgentOptimised* pcRobot         = (CRobotAgentOptimised*) tSortedAgents[nbrs];

        unsigned int fv_status   = pcRobot->Attack(m_pcFeatureVector);
        if (fv_status == 1)
        {
            (*pun_number_of_attackers)++;

            if(m_battackeragentlog && b_logs)
            {
                printf("\nAn attacker agent.");
                //float* FeatureVectorsSensed;
                //FeatureVectorsSensed = pcRobot->GetFeaturesSensed();

                PrintDecidingAgentDetails(m_pcFeatureVector, pcRobot);
                m_battackeragentlog = false;
            }
        }
        else if(fv_status == 2)
        {
            (*pun_number_of_toleraters)++;

            if(m_btolerateragentlog && b_logs)
            {
                printf("\nA tolerator agent.");
                //float* FeatureVectorsSensed;
                //FeatureVectorsSensed = pcRobot->GetFeaturesSensed();
                PrintDecidingAgentDetails(m_pcFeatureVector, pcRobot);
                m_btolerateragentlog = false;
            }
        }

        //float* FeatureVectorsSensed;
        //FeatureVectorsSensed = pcRobot->GetFeaturesSensed();
//        if(FeatureVectorsSensed[m_pcFeatureVector->GetValue()] > 0.0)
        if(fv_status != 3)
            (*pun_number_of_neighborsinsensoryrange)++;
        // change to if m_pcFeatureVector->GetValue() is a member of (pcRobot->crminAgent->vecAPCs), then increment (*pun_number_of_neighborsinsensoryrange) by 1

        if(b_logs)
        {
            printf("\nAn agent.");
            //float* FeatureVectorsSensed;
            //FeatureVectorsSensed = pcRobot->GetFeaturesSensed();
            PrintDecidingAgentDetails(m_pcFeatureVector, pcRobot);
        }
    }

    if(!m_bRobotDeactivated)
    {
        if((*pun_number_of_attackers) > (*pun_number_of_toleraters))
            m_unConseqDetectedFaulty++; // incremented faulted counter
        else if((*pun_number_of_attackers) <= (*pun_number_of_toleraters))
            m_unConseqDetectedFaulty=0; // reset the counter

        if(m_unConseqDetectedFaulty > DEACTIVATIONTHRESHOLD)
        {
            m_bRobotDeactivated = true;
            m_iDEactivationTime = (int) CSimulator::GetInstance()->GetSimulationStepNumber();
        }
    }
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgentOptimised::PrintDecidingAgentDetails(CFeatureVector* m_pcFV,
                                                     CRobotAgentOptimised* decidingrobot)
{
    if(FDMODELTYPE == THRESHOLDONFVDIST) {
        decidingrobot->PrintFeatureVectorDistribution(decidingrobot->GetIdentification());
        return; }


    CRMinRobotAgentOptimised* model_crminagent     = decidingrobot->GetCRMinRobotAgent();

    printf("  Convg. error %f (%fperc)    ",model_crminagent->GetConvergenceError(), model_crminagent->GetConvergenceError_Perc());
    decidingrobot->PrintFeatureVectorDistribution(decidingrobot->GetIdentification());
    model_crminagent->PrintAPCList(decidingrobot->GetIdentification());
    model_crminagent->PrintTcellList(decidingrobot->GetIdentification());
    model_crminagent->PrintTcellResponseToAPCList(decidingrobot->GetIdentification());
}

/******************************************************************************/
/******************************************************************************/
