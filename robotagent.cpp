#include "robotagent.h"
#include "simulator.h"

/******************************************************************************/
/******************************************************************************/

CRobotAgent::CRobotAgent(const char* pch_name, unsigned int un_identification, CArguments* pc_arguments, CArguments* pc_model_arguments, TBehaviorVector vec_behaviors) :
        CAgent(pch_name, un_identification, pc_arguments), m_vecBehaviors(vec_behaviors)
{
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        (*i)->SetAgent(this);
    }

    crminAgent = NULL; ctrnninAgent = NULL; lineqinAgent = NULL;
    if(FDMODELTYPE == CRM || FDMODELTYPE == CRM_TCELLSINEXCESS)
        crminAgent          = new CRMinRobotAgent(this, pc_model_arguments);
    else if(FDMODELTYPE == CTRNN)
        ctrnninAgent        = new CTRNNinRobotAgent(this, pc_model_arguments);
    else if(FDMODELTYPE == LINEQ)
        lineqinAgent        = new LINEQinRobotAgent(this, pc_model_arguments);
    else {
        printf("\nUnknown model type"); exit(-1);}

    m_pcFeatureVector   = new CFeatureVector(this);
    m_pfFeaturesSensed  = new float[CFeatureVector::NUMBER_OF_FEATURE_VECTORS];

    m_fWeight = 0.0;

    static bool bHelpDisplayed = false;

    //control at what distances agents can sense one another when FVs have to be communicated
    // now made redundant with selectnumnearestnbrs
    m_fFVSenseRange               = pc_arguments->GetArgumentAsDoubleOr("fvsenserange", 10.0);

    //at what distances agents are considered neighbors when the individual features are computed
    CFeatureVector::FEATURE_RANGE = pc_arguments->GetArgumentAsDoubleOr("featuresenserange", 6.0);

    m_fResponseRange              = pc_arguments->GetArgumentAsDoubleOr("responserange", m_fFVSenseRange);
    m_uSelectedNumNearestNbrs     = pc_arguments->GetArgumentAsIntOr("selectnumnearestnbrs", 10);


    if (pc_arguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("fvsenserange=#.#              Range at which other agents' FVs are sensed [%f]\n"
               "featuresenserange=#.#         Range based on which features are computed  [%f]\n"
               "responserange=#.#             Range at which a robot \"reponds\" to other features [%f]\n"
               "selectnumnearestnbrs=#        The number of nearest neighbours for FV sensing and T-cell diffusion (makes fvsenserange redundant) [%d]\n"
               ,
               m_fFVSenseRange,
               CFeatureVector::FEATURE_RANGE,
               m_fResponseRange,
               m_uSelectedNumNearestNbrs
               );
    }

    m_pbMostWantedList = new unsigned int[CFeatureVector::NUMBER_OF_FEATURE_VECTORS];

    for (unsigned int i = 0; i < CFeatureVector::NUMBER_OF_FEATURE_VECTORS; i++)
        m_pbMostWantedList[i] = 0;
}

/******************************************************************************/
/******************************************************************************/

CRobotAgent::~CRobotAgent()
{
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        delete (*i);
    }
    delete m_pcFeatureVector;
    delete m_pfFeaturesSensed;
    delete m_pbMostWantedList;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::SimulationStepUpdatePosition()
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
        } else {
            (*i)->Suppress();
        }
    }

    // Update the model (T-cells of the CRM instance for this robot), CTRNN neuron activations, lineq on fvs
    m_pcFeatureVector->SimulationStep();
    unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();

    if(FDMODELTYPE != LINEQ) // lineq - low expected run time; can come back and log more details if needed
    {
        if (m_iBehavIdentification == 1)
            printf("\nStep: %d, FV for normal agent %d: %s\n", CurrentStepNumber, m_unIdentification, m_pcFeatureVector->ToString().c_str());

        if (m_iBehavIdentification == -1)
            printf("\nStep: %d, FV for abnormal agent %d: %s\n", CurrentStepNumber, m_unIdentification, m_pcFeatureVector->ToString().c_str());
    }

    Sense(GetSelectedNumNearestNbrs());

    if(CurrentStepNumber > MODELSTARTTIME)
    {
        if(FDMODELTYPE == CRM || FDMODELTYPE == CRM_TCELLSINEXCESS)
            crminAgent->SimulationStepUpdatePosition();
        else if (FDMODELTYPE == CTRNN)
            ctrnninAgent->SimulationStepUpdatePosition();
        else
            lineqinAgent->SimulationStepUpdatePosition();
    }
    CAgent::SimulationStepUpdatePosition();
}

/******************************************************************************/
/******************************************************************************/

const CFeatureVector* CRobotAgent::GetFeatureVector() const
{
    return m_pcFeatureVector;
}

/******************************************************************************/
/******************************************************************************/

EAgentType CRobotAgent::GetType()
{
    return ROBOT;
}

/******************************************************************************/
/******************************************************************************/

float* CRobotAgent::GetFeaturesSensed() const
{
    return m_pfFeaturesSensed;
}

/******************************************************************************/
/******************************************************************************/

CRobotAgent* CRobotAgent::GetRandomRobotWithWeights(double f_range)
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
    {
        return NULL;
    }
    double fSelectedWeight  = Random::nextDouble() * fWeightSum;

    TAgentList* ptAgentList  = NULL;
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
                fSelectedWeight -= ((CRobotAgent*) (*j))->GetWeight();
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
    {
        ERROR("The random generator seems to be wrong");
    }

    return (CRobotAgent*) pcAgentSelected;
}

/******************************************************************************/
/******************************************************************************/

double CRobotAgent::CountWeightsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range)
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
                    fReturn += ((CRobotAgent*) (*j))->GetWeight();
            } else {
                (*j)->m_bTempWithInRange = false;
            }

        }
    }

    return fReturn;
}

/******************************************************************************/
/******************************************************************************/

CRobotAgent* CRobotAgent::GetRandomRobotWithWeights(unsigned int u_nearestnbrs)
{
    TAgentVector tSortedAgents;

    SortAllAgentsAccordingToDistance(&tSortedAgents);

    double fWeightSum = 0.0;
    // 1-11 if u_nearestnbrs is 10, because agent at index 0 is ourselves:
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgent* pcRobot = (CRobotAgent*) tSortedAgents[i];
        fWeightSum += pcRobot->GetWeight();
    }

    if (fWeightSum < 1e-10)
    {
        return NULL;
    }

    double fSelectedWeight  = Random::nextDouble() * fWeightSum;


    CAgent* pcAgentSelected  = NULL;
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgent* pcRobot = (CRobotAgent*) tSortedAgents[i];
        fSelectedWeight -= pcRobot->GetWeight();

        if(fSelectedWeight <= 0.0)
        {
            pcAgentSelected = pcRobot;
            break;
        }
    }
    return (CRobotAgent*) pcAgentSelected;
}

/******************************************************************************/
/******************************************************************************/

CRMinRobotAgent* CRobotAgent::GetCRMinRobotAgent()
{
    return crminAgent;
}

/******************************************************************************/
/******************************************************************************/

CTRNNinRobotAgent* CRobotAgent::GetCTRNNinRobotAgent()
{
    return ctrnninAgent;
}

/******************************************************************************/
/******************************************************************************/

LINEQinRobotAgent* CRobotAgent::GetLINEQinRobotAgent()
{
    return lineqinAgent;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::SetWeight(double f_weight)
{
    m_fWeight = f_weight;
}

/******************************************************************************/
/******************************************************************************/

double CRobotAgent::GetWeight() const
{
    return m_fWeight;
}

/******************************************************************************/
/******************************************************************************/

double CRobotAgent::GetFVSenseRange() const
{
    return m_fFVSenseRange;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CRobotAgent::GetSelectedNumNearestNbrs()
{
    return m_uSelectedNumNearestNbrs;
}

/******************************************************************************/
/******************************************************************************/
void CRobotAgent::FVsOfWcFeature(const CFeatureVector* pc_feature_vector, unsigned int *fv1, unsigned int *fv2)
{
    if(CFeatureVector::NUMBER_OF_FEATURES == 6)
    {
#ifdef WILDCARDINFV

    assert(pc_feature_vector->m_iWildCardBit != -1);
    (*fv1) = pc_feature_vector->GetValue();
    (*fv2) = pc_feature_vector->GetValue() +
             (1 << pc_feature_vector->m_iWildCardBit);

    return;

#endif
    }
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::Sense(unsigned int u_nearestnbrs)
{
    for (int i = 0; i < CFeatureVector::NUMBER_OF_FEATURE_VECTORS; i++)
    {
        m_pfFeaturesSensed[i] = 0.0;
    }

    TAgentVector tSortedAgents;

    SortAllAgentsAccordingToDistance(&tSortedAgents);

    // 1-11 because agent at index 0 is ourselves:
    for (int i = 1; i < u_nearestnbrs+1; i++)
    {
        CRobotAgent* pcRobot = (CRobotAgent*) tSortedAgents[i];

#ifdef WILDCARDINFV
        if((CFeatureVector::NUMBER_OF_FEATURES == 6) &&
           (pcRobot->GetFeatureVector()->m_iWildCardBit != -1))
        {
            unsigned int fv1, fv2;
            FVsOfWcFeature(pcRobot->GetFeatureVector(), &fv1, &fv2);

            if(WILDCARDINFV == 0.5){
                m_pfFeaturesSensed[fv1] += 0.5;
                m_pfFeaturesSensed[fv2] += 0.5;}
            else if(WILDCARDINFV == 1.0){
                m_pfFeaturesSensed[fv1] += 1.0;
                m_pfFeaturesSensed[fv2] += 1.0;}
            else{
                printf("\n WILDCARDINFV is to take values at 0.5 or 1.0 and not %f",WILDCARDINFV);
                exit(-1);}
        }
        else
#endif
        {
            unsigned int unFeatureVector = pcRobot->GetFeatureVector()->GetValue();
            m_pfFeaturesSensed[unFeatureVector] += 1.0;
        }
    }
}


/******************************************************************************/
/******************************************************************************/

unsigned int CRobotAgent::GetColor()
{
    //return m_unIdentification == TRACKAGENT ? GREEN : RED;
    /*if(m_unIdentification == 1)
        return GREEN;
    else if (m_unIdentification == 15)
        return RED;
    else  if (m_unIdentification == 5)
        return YELLOW;
     else
        return BLUE;*/

    /*if(m_iBehavIdentification  == 1)
        return GREEN;
    else if (m_iBehavIdentification  == -1)
        return RED;
    else  if (m_unIdentification == 5) // a supposedly normal agent that seems to take long to join the aggregate
        return YELLOW;
    else*/

    #ifdef TCELLCLONEEXCHANGEANALYSIS
            if(GetIdentification() <= 9) {
                return GREEN;
            }
            else
                return RED;
    #else
        return BLUE;
    #endif
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::SetBehaviors(TBehaviorVector vec_behaviors)
{
    m_vecBehaviors = vec_behaviors;

    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        (*i)->SetAgent(this);
    }
}

/******************************************************************************/
/******************************************************************************/

TBehaviorVector CRobotAgent::GetBehaviors()
{
    return m_vecBehaviors;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::SetMostWantedList(unsigned unFeatureVector, unsigned int state)
{
    m_pbMostWantedList[unFeatureVector] = state;
}

/******************************************************************************/
/******************************************************************************/

unsigned int* CRobotAgent::GetMostWantedList()
{
    return m_pbMostWantedList;
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::CheckNeighborsResponseToMyFV(unsigned int* pun_number_of_toleraters, unsigned int* pun_number_of_attackers, unsigned int* pun_number_of_neighborsinsensoryrange, bool b_logs)
{
    (*pun_number_of_toleraters)  = 0;
    (*pun_number_of_attackers)   = 0;
    (*pun_number_of_neighborsinsensoryrange) = 0;


    TAgentVector tSortedAgents;

    SortAllAgentsAccordingToDistance(&tSortedAgents);

    // 1-11 because agent at index 0 is ourselves:
    bool m_battackeragentlog=true, m_btolerateragentlog=true;
    for (unsigned int nbrs = 1; nbrs < m_uSelectedNumNearestNbrs+1; nbrs++)
    {
        CRobotAgent* pcRobot     = (CRobotAgent*) tSortedAgents[nbrs];
        CRMinRobotAgent*   tmp_crm   = pcRobot->GetCRMinRobotAgent();
        CTRNNinRobotAgent* tmp_ctrnn = pcRobot->GetCTRNNinRobotAgent();
        LINEQinRobotAgent* tmp_lineq = pcRobot->GetLINEQinRobotAgent();

        unsigned int fv_status   = pcRobot->Attack(m_pcFeatureVector);
        if (fv_status == 1)
        {
            (*pun_number_of_attackers)++;

            if(m_battackeragentlog && b_logs)
            {
                printf("\nAn attacker agent.");
                float* FeatureVectorsSensed;
                FeatureVectorsSensed = pcRobot->GetFeaturesSensed();

                PrintDecidingAgentDetails(m_pcFeatureVector, tmp_crm, tmp_ctrnn, tmp_lineq,
                                          FeatureVectorsSensed);
                m_battackeragentlog = false;
            }
        }
        else if(fv_status == 2)
        {
            (*pun_number_of_toleraters)++;

            if(m_btolerateragentlog && b_logs)
            {
                printf("\nA tolerator agent.");
                float* FeatureVectorsSensed;
                FeatureVectorsSensed = pcRobot->GetFeaturesSensed();
                PrintDecidingAgentDetails(m_pcFeatureVector, tmp_crm, tmp_ctrnn, tmp_lineq,
                                          FeatureVectorsSensed);
                m_btolerateragentlog = false;
            }
        }

        float* FeatureVectorsSensed;
        FeatureVectorsSensed = pcRobot->GetFeaturesSensed();
        if(FeatureVectorsSensed[m_pcFeatureVector->GetValue()] > 0.0)
            (*pun_number_of_neighborsinsensoryrange)++;


        if(b_logs)
        {
            printf("\nAn agent.");
            float* FeatureVectorsSensed;
            FeatureVectorsSensed = pcRobot->GetFeaturesSensed();
            PrintDecidingAgentDetails(m_pcFeatureVector, tmp_crm, tmp_ctrnn, tmp_lineq,
                                      FeatureVectorsSensed);
        }
    }
}

/******************************************************************************/
/******************************************************************************/

void CRobotAgent::PrintDecidingAgentDetails(CFeatureVector* m_pcFV, CRMinRobotAgent* model_crminagent, CTRNNinRobotAgent* model_ctrnninagent, LINEQinRobotAgent* model_lineqinagent, float* FeatureVectorsSensed)
{
    if(FDMODELTYPE == CRM || FDMODELTYPE == CRM_TCELLSINEXCESS)
    {
        printf("  Convg. error %f (%fperc)    ",model_crminagent->GetConvergenceError(), model_crminagent->GetConvergenceError_Perc());
        for (int fv = 0; fv < CFeatureVector::NUMBER_OF_FEATURE_VECTORS; fv++)
        {
            if(FeatureVectorsSensed[fv] > 0.0)
            {
                printf("FV:%d, [APC]:%f, [E%d]:%f, [R%d]:%f,   [wtsumE]:%f, [wtsumR]:%f   ",
                       fv,
                       model_crminagent->GetAPC(fv),
                       fv,
                       model_crminagent->GetCurrE(fv),
                       fv,
                       model_crminagent->GetCurrR(fv),
                       model_crminagent->m_pfSumEffectorsWeightedbyAffinity[fv],
                       model_crminagent->m_pfSumRegulatorsWeightedbyAffinity[fv]);
            }
        }

        // if the neighbour doesnot have your fv
        if(FeatureVectorsSensed[m_pcFV->GetValue()] == 0.0)
        {
            printf(",for the evaluated agent's FV that is not in the deciding agent's repertoire: FV:%d, [APC]:%f, [E%d]:%f, [R%d]:%f,   [wtsumE]:%f, [wtsumR]:%f   ",
                   m_pcFV->GetValue(),
                   model_crminagent->GetAPC(m_pcFV->GetValue()),
                   m_pcFV->GetValue(),
                   model_crminagent->GetCurrE(m_pcFV->GetValue()),
                   m_pcFV->GetValue(),
                   model_crminagent->GetCurrR(m_pcFV->GetValue()),
                   model_crminagent->m_pfSumEffectorsWeightedbyAffinity[m_pcFV->GetValue()],
                   model_crminagent->m_pfSumRegulatorsWeightedbyAffinity[m_pcFV->GetValue()]);
        }
    }
    else if(FDMODELTYPE == CTRNN) //model_ctrnninagent
    {
        printf("  HN-Convg.error %f (%f%%)      ON-Convg. error %f (%f%%)    ",model_ctrnninagent->GetConvergenceError(HiddenLayer), model_ctrnninagent->GetConvergenceError_Perc(HiddenLayer),model_ctrnninagent->GetConvergenceError(OutputLayer), model_ctrnninagent->GetConvergenceError_Perc(OutputLayer));
        for (int fv = 0; fv < CFeatureVector::NUMBER_OF_FEATURE_VECTORS; fv++)
        {
            if(FeatureVectorsSensed[fv] > 0.0)
            {
                printf("FV:%d, [APC]:%f, [HN%d]:%f,  [ON%d]:Sg%f (%f,%f)   ",
                       fv,
                       model_ctrnninagent->GetAPC(fv),
                       fv,
                       model_ctrnninagent->GetHN(fv),
                       fv,
                       model_ctrnninagent->Sigmoid(model_ctrnninagent->GetON(fv),model_ctrnninagent->GetSigmoidSaturation()),
                       model_ctrnninagent->GetON(fv),
                       model_ctrnninagent->GetInputsToON(fv));
            }
        }

        // if the evaluating neighbour doesnot have your fv
        if(FeatureVectorsSensed[m_pcFV->GetValue()] == 0.0)
        {
            printf(",for the evaluated agent's FV that is not in the deciding agent's repertoire: FV:%d, [APC]:%f, [HN%d]:%f, [ON%d]:Sg%f (%f,%f)   ",
                   m_pcFV->GetValue(),
                   model_ctrnninagent->GetAPC(m_pcFV->GetValue()),
                   m_pcFV->GetValue(),
                   model_ctrnninagent->GetHN(m_pcFV->GetValue()),
                   m_pcFV->GetValue(),
                   model_ctrnninagent->Sigmoid(model_ctrnninagent->GetON(m_pcFV->GetValue()),model_ctrnninagent->GetSigmoidSaturation()),
                   model_ctrnninagent->GetON(m_pcFV->GetValue()),
                   model_ctrnninagent->GetInputsToON(m_pcFV->GetValue()));
        }
    }
    else
    {
        for (int fv = 0; fv < CFeatureVector::NUMBER_OF_FEATURE_VECTORS; fv++)
        {
            if(FeatureVectorsSensed[fv] > 0.0)
            {
                printf("FV:%d, [APC]:%f, [LineqFV%d]:%f   ",
                       fv,
                       model_lineqinagent->GetAPC(fv),
                       fv,
                       model_lineqinagent->GetLineqFV(fv));
            }
        }

        // if the evaluating neighbour doesnot have your fv
        if(FeatureVectorsSensed[m_pcFV->GetValue()] == 0.0)
        {
            printf(",for the evaluated agent's FV that is not in the deciding agent's repertoire: FV:%d, [APC]:%f, [LineqFV%d]:%f   ",
                   m_pcFV->GetValue(),
                   model_lineqinagent->GetAPC(m_pcFV->GetValue()),
                   m_pcFV->GetValue(),
                   model_lineqinagent->GetLineqFV(m_pcFV->GetValue()));
        }
    }
}

/******************************************************************************/
/******************************************************************************/

unsigned int CRobotAgent::Attack(CFeatureVector* pc_feature_vector)
{
#if defined(WILDCARDINFV) && (FDMODELTYPE==CRM || FDMODELTYPE==CRM_TCELLSINEXCESS)

    if(CFeatureVector::NUMBER_OF_FEATURES == 6 &&
       pc_feature_vector->m_iWildCardBit != -1)
        {
            CRMinRobotAgent* tmp_crm = GetCRMinRobotAgent(); // Get the CRM of the evaluating robot

            unsigned int fv1, fv2;
            FVsOfWcFeature(pc_feature_vector, &fv1, &fv2);

            double E  = tmp_crm->m_pfSumEffectorsWeightedbyAffinity[fv1] +
                        tmp_crm->m_pfSumEffectorsWeightedbyAffinity[fv2];

            double R = tmp_crm->m_pfSumRegulatorsWeightedbyAffinity[fv1] +
                       tmp_crm->m_pfSumRegulatorsWeightedbyAffinity[fv2];

            printf("\nWC: WCBit=%d, fv1=%u, fv2=%u, E=%f, R=%f", pc_feature_vector->m_iWildCardBit,
                   fv1, fv2, E, R);

            if (((E + R) <= CELLLOWERBOUND) || fabs(E - R) <= CELLLOWERBOUND)
            {
                return 0; // Dont know - no T-cells to make decision or E approx. equal to R
            }
            else if (E > R)
            {
                // Attack
                return 1;
            }
            else if(R > E)
            {
                // Tolerate
                return 2;
            }
        }
        else
            return m_pbMostWantedList[pc_feature_vector->GetValue()];
#else
    return m_pbMostWantedList[pc_feature_vector->GetValue()];
#endif

}

/******************************************************************************/
/******************************************************************************/
