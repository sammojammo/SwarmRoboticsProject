#include "agent.h"
#include "simulator.h"
#include "random.h"
#include <math.h>
#include <algorithm>

/******************************************************************************/
/******************************************************************************/

unsigned int CAgent::g_unGlobalNumberOfAgentsCreated = 0;

// E-puck radius * 10:
double CAgent::RADIUS = 0.375;


/******************************************************************************/
/******************************************************************************/


CAgent::CAgent(const char* pch_name, unsigned un_identification, CArguments* pc_arguments) :
        CSimObject(pch_name)
{
    m_pcArguments = pc_arguments;


    const char* pchControllerType = pc_arguments->GetArgumentAsStringOr("controller", "RANDOMWALK");

    if (strcmp(pchControllerType, "RANDOMWALK") == 0)
    {
        m_eControllerType = RANDOMWALK;
    }
    else if (strcmp(pchControllerType, "RANDOMBOUNCE") == 0)
    {
        m_eControllerType = RANDOMBOUNCE;
    }
    else if (strcmp(pchControllerType, "REGULARBOUNCE") == 0)
    {
        m_eControllerType = REGULARBOUNCE;
    }

    m_fMaximumSpeed                         = pc_arguments->GetArgumentAsDoubleOr("maxspeed",             0.01);

    static bool bHelpDisplayed = false;

    //newMag = currMag + currSpeed / maxSpeed * Gaussian * magnoise
    //newDir    = currDir + currSpeed / maxSpeed * Gaussian * magdir * 360 / (PI * 2)

    m_fProportionalDirectionNoise = pc_arguments->GetArgumentAsDoubleOr("dirnoise", 0.0);
    m_fProportionalMagnitudeNoise = pc_arguments->GetArgumentAsDoubleOr("magnoise", 0.0);

    if (pc_arguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("Agent help:\n"
               "  controller=[RANDOMWALK,RANDOMBOUNCE,REGULARBOUNCE]\n"
               "  maxspeed=#.#             Max speed of the agents per time-step [%f]\n"
               "  recruitment_range=#.#    Max physical distance for recruitment only [%f]\n"
               "  dirnoise=#.#             Proportional direction noise (in degrees) on velocity [%f]\n"
               "  magnoise=#.#             Proportional magnitude noise on velocity [%f]\n",
               m_fMaximumSpeed,
               m_fMaximumPhysicalRange_Recruitment,
               m_fProportionalDirectionNoise,
               m_fProportionalMagnitudeNoise);
        bHelpDisplayed = true;
    }

    g_unGlobalNumberOfAgentsCreated++;

#ifdef TCELLCLONEEXCHANGEANALYSIS
    if(un_identification <= 9) {
        m_tPosition.x = -10.0;
        m_tPosition.y = -10.0;}
    else {
        m_tPosition.x = 10.0;
        m_tPosition.y = 10.0;}
#else
    m_tPosition.x = 0.0;
    m_tPosition.y = 0.0;
#endif


    m_unIdentification = un_identification;
    m_bInteractable    = false;
    m_unColor          = 0;

    SetRandomVelocity();
}

/******************************************************************************/
/******************************************************************************/

CAgent::~CAgent()
{
}

/******************************************************************************/
/******************************************************************************/

const TVector2d* CAgent::GetPosition() const
{
    return (const TVector2d*) &m_tPosition;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetPosition(TVector2d* pt_new_position)
{
    m_tPosition = (*pt_new_position);
}

/******************************************************************************/
/******************************************************************************/

const TVector2d* CAgent::GetVelocity() const
{
    return (const TVector2d*) &m_tVelocity;
}

/******************************************************************************/
/******************************************************************************/

const TVector2d* CAgent::GetAcceleration() const
{
    return (const TVector2d*) &m_tAcceleration;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetVelocity(TVector2d* pt_new_velocity)
{
    m_tVelocity = (*pt_new_velocity);
}

/******************************************************************************/
/******************************************************************************/

double CAgent::GetAngularVelocity()
{
    return m_tAngularVelocity;
}

/******************************************************************************/
/******************************************************************************/

double CAgent::GetAngularAcceleration()
{
    return m_tAngularAcceleration;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::GetRelativeVelocity(double* mag_relvelocity, double* dir_relvelocity, double feature_range)
{
    // Velocity magnitude and direction wrt. surrounding agents:
    TVector2d tTemp    = GetAverageVelocityOfSurroundingAgents(feature_range, ROBOT);

    float tmp_agentvelocity = Vec2dLength(m_tVelocity);
    (*mag_relvelocity)      = tmp_agentvelocity - Vec2dLength(tTemp);
    (*dir_relvelocity)      = GetVectorAngle(m_tVelocity, tTemp);
}

/******************************************************************************/
/******************************************************************************/

void CAgent::GetRelativeAcceleration(double *mag_relacceleration, double *dir_relacceleration, double feature_range)
{

    TVector2d tTemp    = GetAverageAccelerationOfSurroundingAgents(feature_range, ROBOT);

    float tmp_agentacceleration      = Vec2dLength(m_tAcceleration);
    (*mag_relacceleration)           = tmp_agentacceleration - Vec2dLength(tTemp);
    (*dir_relacceleration) = GetVectorAngle(m_tAcceleration, tTemp);
}

/******************************************************************************/
/******************************************************************************/

double CAgent::GetVectorAngle(TVector2d vector1, TVector2d vector2)
{
    double vectorangle = 0.0;

    if (Vec2dLength(vector1) > EPSILON && Vec2dLength(vector2) > EPSILON)
    {
        vectorangle = acos(Vec2dCosAngle(vector1,vector2));
    }
    else
    {
        vectorangle = 0.0;
    }

    if(std::isnan(vectorangle))
    {
        double cosinevalue = Vec2dCosAngle(vector2,vector1);

        if(fabs(cosinevalue - 1.0) < 1.0e-3)
            vectorangle = 0.0;
        else if(fabs(cosinevalue - -1.0) < 1.0e-3)
            vectorangle = M_PI;
        else
        {
            vectorangle = acos(cosinevalue);

            if(std::isnan(vectorangle))
            {
                printf("Error in computing acos - getting nan");
                exit(-1);
            }
        }
    }

    return vectorangle;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SimulationStep(unsigned int un_step_number)
{
    TVector2d tTemp = m_tVelocity;
    double tTempAngVelocity = m_tAngularVelocity;

    SimulationStepUpdatePosition();
    m_tAcceleration = m_tVelocity;
    m_tAcceleration.x -= tTemp.x;
    m_tAcceleration.y -= tTemp.y;

    m_tAngularVelocity     = GetVectorAngle(m_tVelocity,tTemp); // [0 pi]
    m_tAngularAcceleration = m_tAngularVelocity - tTempAngVelocity; // [-pi pi]
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SimulationStepUpdatePosition()
{
    double fSpeed      = Vec2dLength(m_tVelocity);
    double fSpeedRatio = fSpeed / m_fMaximumSpeed;

    if (fSpeedRatio > EPSILON && m_fProportionalDirectionNoise > EPSILON)
    {
        double fAngle = m_fProportionalDirectionNoise / 360.0 * (M_PI * 2.0) * fSpeedRatio * Random::nextNormGaussian();

        Vec2dRotate(fAngle, m_tVelocity);
    }

    if (fSpeedRatio > EPSILON && m_fProportionalMagnitudeNoise > EPSILON)
    {
        double fMagnitude = 1.0 + m_fProportionalMagnitudeNoise * fSpeedRatio * Random::nextNormGaussian();
        if (fSpeed * fMagnitude > m_fMaximumSpeed)
        {
            fMagnitude = 1.0;
        }

        Vec2dMultiplyScalar(m_tVelocity, fMagnitude);
    }


    TVector2d tOldPosition = m_tPosition;
    TVector2d tNewPosition = { m_tPosition.x + m_tVelocity.x,
                               m_tPosition.y + m_tVelocity.y  };

    CSimulator::GetInstance()->GetArena()->MoveAgent(this, &tNewPosition);

    CAgent* pcCollidingAgent = GetClosestAgent(RADIUS * 2.0, ROBOT);


    if (pcCollidingAgent)
    {
        // TVector2d vecCollidingAgentPos = *(pcCollidingAgent->GetPosition());
        // TVector2d vecTemp = vecCollidingAgentPos;

        // vecTemp.x = vecTemp.x - GetPosition()->x;
        // vecTemp.y = vecTemp.y - GetPosition()->y;

        // Vec2dNormalize(vecTemp);

        // vecTemp.x = (-vecTemp.x * RADIUS * 2.0 + vecCollidingAgentPos.x);
        // vecTemp.y = (-vecTemp.y * RADIUS * 2.0 + vecCollidingAgentPos.y);

        // CSimulator::GetInstance()->GetArena()->MoveAgent(this, &vecTemp);

        // m_tVelocity.x = 0;
        // m_tVelocity.y = 0;

        CSimulator::GetInstance()->GetArena()->MoveAgent(this, &tOldPosition);
    }
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetMaximumSpeed(double f_max_speed)
{
    m_fMaximumSpeed = f_max_speed;
}

/******************************************************************************/
/******************************************************************************/

double CAgent::GetMaximumSpeed() const
{
    return m_fMaximumSpeed;
}

/******************************************************************************/
/******************************************************************************/

double CAgent::GetMaximumAngularVelocity() const
{
    return M_PI;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CAgent::CountAgentsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type)
{
    unsigned int unReturn = 0;
    TAgentListListIterator i;

    double fSquareRange = f_range * f_range;

    for (i = ptlist_agent_list_list->begin(); i != ptlist_agent_list_list->end(); i++)
    {
        TAgentListIterator j;
        for (j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if (((*j)->GetType() == e_type || e_type == ANY) && (*j) != this)
            {
                (*j)->m_bTempWithInRange = (GetSquaredDistanceBetweenPositions(&m_tPosition, (*j)->GetPosition()) <= fSquareRange);
                if ((*j)->m_bTempWithInRange)
                    unReturn++;
            } else {
                (*j)->m_bTempWithInRange = false;
            }
        }
    }

    return unReturn;
}

/******************************************************************************/
/******************************************************************************/

CAgent* CAgent::GetRandomAgentWithinRange(TAgentListList* pt_agents_list_list, double f_range, EAgentType e_type)
{
    unsigned int unNumberOfAgents = CountAgentsInAgentListList(pt_agents_list_list, f_range, e_type);
    if (unNumberOfAgents == 0)
    {
        return NULL;
    }
    unsigned int unSelectedAgent  = Random::nextInt(0, unNumberOfAgents);

    TAgentList* ptAgentList  = NULL;
    TAgentListListIterator i = pt_agents_list_list->begin();
    CAgent* pcAgentSelected  = NULL;

    do
    {
        while ((*i)->size() == 0)
        {
            i++;
        }
        TAgentListIterator j = (*i)->begin();

        while (j != (*i)->end() && unSelectedAgent > 0)
        {
            if ((*j)->m_bTempWithInRange)
                unSelectedAgent--;
            if (unSelectedAgent > 0)
                j++;
        }

        if (unSelectedAgent > 0)
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
    } while (pcAgentSelected == NULL && i != pt_agents_list_list->end());

    if (i == pt_agents_list_list->end())
    {
        ERROR("The random generator seems to be wrong");
    }

    return pcAgentSelected;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetRandomVelocity()
{
    double fSpeed = m_fMaximumSpeed;
    double fAngle = Random::nextDouble() * 2.0 * 3.141592;

    m_tVelocity.x = cos(fAngle) * fSpeed;
    m_tVelocity.y = sin(fAngle) * fSpeed;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CAgent::GetColor()
{
    return m_unColor;
}

/******************************************************************************/
/******************************************************************************/

double   CAgent::GetSize()
{
    return 2.0;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetBehavIdentification(int i_behavidentification)
{
    m_iBehavIdentification = i_behavidentification;
}

/******************************************************************************/
/******************************************************************************/

int CAgent::GetBehavIdentification()
{
    // +1 normal agent
    // -1 abnormal agent
    return m_iBehavIdentification;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::MoveTowards(TVector2d t_position, double f_max_speed)
{
    CArena* pcArena = CSimulator::GetInstance()->GetArena();
    double fArenaWidth;
    double fArenaHeight;
    pcArena->GetSize(&fArenaWidth, &fArenaHeight);

    if (CArena::g_bIsBoundless)
    {

        if (fabs(t_position.x - m_tPosition.x) > fArenaWidth / 2.0)
        {
            if (t_position.x < m_tPosition.x)
            {
                t_position.x += fArenaWidth;
            } else {
                t_position.x -= fArenaWidth;
            }
        }

        if (fabs(t_position.y - m_tPosition.y) > fArenaHeight / 2.0)
        {
            if (t_position.y < m_tPosition.y)
            {
                t_position.y += fArenaHeight;
            } else {
                t_position.y -= fArenaHeight;
            }
        }
    }

    m_tVelocity.x = (t_position.x - m_tPosition.x);
    m_tVelocity.y = (t_position.y - m_tPosition.y);

    double fSpeed = sqrt(m_tVelocity.x * m_tVelocity.x + m_tVelocity.y * m_tVelocity.y);

    if (fSpeed > f_max_speed)
    {
        double fModifier = fSpeed / f_max_speed;
        m_tVelocity.x /= fModifier;
        m_tVelocity.y /= fModifier;
    }

    // TVector2d tNewPosition = { m_tPosition.x + m_tVelocity.x,
    //                            m_tPosition.y + m_tVelocity.y };

    // if (tNewPosition.x >= fArenaWidth / 2.0)
    //     tNewPosition.x -= fArenaWidth;

    // if (tNewPosition.x <= -fArenaWidth / 2.0)
    //     tNewPosition.x += fArenaWidth;

    // if (tNewPosition.y >= fArenaHeight / 2.0)
    //     tNewPosition.y -= fArenaHeight;

    // if (tNewPosition.x <= -fArenaHeight / 2.0)
    //     tNewPosition.x += fArenaHeight;

    // if (!CSimulator::GetInstance()->GetArena()->IsObstacle(&tNewPosition)) {
    //     CSimulator::GetInstance()->GetArena()->MoveAgent(this, &tNewPosition);
    // }
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetColor(unsigned int un_color)
{
    m_unColor = un_color;
}

/******************************************************************************/
/******************************************************************************/

void CAgent::MarkAgentsWithinRange(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type)
{
    CountAgentsInAgentListList(ptlist_agent_list_list, f_range, e_type);
}

/******************************************************************************/
/******************************************************************************/

unsigned int CAgent::CountAgents(double f_range, EAgentType e_type)
{
    TAgentListList tAgentListList;
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);
    return CountAgentsInAgentListList(&tAgentListList, f_range, e_type);
}

/******************************************************************************/
/******************************************************************************/

CAgent* CAgent::GetClosestAgent(double f_range, EAgentType e_type)
{
    TAgentListList tAgentListList;
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);

    double fShortestDistanceSquared = f_range * f_range;
    CAgent* pcAgent  = NULL;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if ((*j) != this)
            {
                double fDistanceSqaured = GetSquaredDistanceBetweenPositions((*j)->GetPosition(), GetPosition());
                if (fDistanceSqaured < fShortestDistanceSquared)
                {
                    fShortestDistanceSquared = fDistanceSqaured;
                    pcAgent = (*j);
                    //                    printf("Closest agent found --- dist: %f, range: %f \n", sqrt(fDistanceSqaured), f_range);
                }
            }
        }
    }

    return pcAgent;
}

/******************************************************************************/
/******************************************************************************/


TVector2d CAgent::GetCenterOfMassOfSurroundingAgents(double f_range, EAgentType e_type)
{
    double fArenaWidth;
    double fArenaHeight;
    CArena* pcArena = CSimulator::GetInstance()->GetArena();
    pcArena->GetSize(&fArenaWidth, &fArenaHeight);

    TAgentListList tAgentListList;
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);
    MarkAgentsWithinRange(&tAgentListList, f_range, e_type);
    TVector2d tCenter = { 0.0, 0.0 };

    TAgentList* ptAgentList  = NULL;
    CAgent* pcAgentSelected  = NULL;
    unsigned int unCount     = 0;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if ((*j)->m_bTempWithInRange)
            {
                TVector2d posAgent = { (*j)->GetPosition()->x, (*j)->GetPosition()->y} ;

                if (fabs(posAgent.x - m_tPosition.x) > fArenaWidth / 2.0)
                {
                    if (posAgent.x < m_tPosition.x)
                    {
                        posAgent.x += fArenaWidth;
                    } else {
                        posAgent.x -= fArenaWidth;
                    }
                }

                if (fabs(posAgent.y - m_tPosition.y) > fArenaHeight / 2.0)
                {
                    if (posAgent.y < m_tPosition.y)
                    {
                        posAgent.y += fArenaHeight;
                    } else {
                        posAgent.y -= fArenaHeight;
                    }
                }

                tCenter.x += posAgent.x - m_tPosition.x;
                tCenter.y += posAgent.y - m_tPosition.y;
                unCount++;
            }
        }
    }

    if (unCount > 0)
    {
        tCenter.x /= (double) unCount;
        tCenter.y /= (double) unCount;

        tCenter.x += m_tPosition.x;
        tCenter.y += m_tPosition.y;
    }

    return tCenter;

}

/******************************************************************************/
/******************************************************************************/

TVector2d CAgent::GetAverageVelocityOfSurroundingAgents(double f_range, EAgentType e_type)
{
    TAgentListList tAgentListList;
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);

    MarkAgentsWithinRange(&tAgentListList, f_range, e_type);
    TVector2d tVelocity = { 0.0, 0.0 };

    TAgentList* ptAgentList  = NULL;
    CAgent* pcAgentSelected  = NULL;
    unsigned int unCount     = 0;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if ((*j)->m_bTempWithInRange)
            {
                tVelocity.x += (*j)->GetVelocity()->x;
                tVelocity.y += (*j)->GetVelocity()->y;
                unCount++;
            }
        }
    }

    if (unCount > 0)
    {
        tVelocity.x /= (double) unCount;
        tVelocity.y /= (double) unCount;
    }

    return tVelocity;

}

/******************************************************************************/
/******************************************************************************/

double CAgent::GetAverageDistanceToSurroundingAgents(double f_range, EAgentType e_type)
{
    TAgentListList tAgentListList;
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);

    MarkAgentsWithinRange(&tAgentListList, f_range, e_type);
    double distance = 0.0;

    TAgentList* ptAgentList  = NULL;
    CAgent* pcAgentSelected  = NULL;
    unsigned int unCount     = 0;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if ((*j)->m_bTempWithInRange)
            {
                distance += GetDistanceBetweenPositions(&m_tPosition, (*j)->GetPosition());
                unCount++;
            }
        }
    }

    if (unCount > 0.0)
    {
        distance /= (double) unCount;
    } else {
        distance = f_range; // if no neighbours in range, returns the maximum range
    }

    return distance;
}

/******************************************************************************/
/******************************************************************************/

TVector2d CAgent::GetAverageAccelerationOfSurroundingAgents(double f_range, EAgentType e_type)
{
    TAgentListList tAgentListList;
    CSimulator::GetInstance()->GetArena()->GetAgentsCloseTo(&tAgentListList, GetPosition(), f_range);
    MarkAgentsWithinRange(&tAgentListList, f_range, e_type);
    TVector2d tAcceleration = { 0.0, 0.0 };

    TAgentList* ptAgentList  = NULL;
    CAgent* pcAgentSelected  = NULL;
    unsigned int unCount     = 0;
    for (TAgentListListIterator i = tAgentListList.begin(); i != tAgentListList.end(); i++)
    {
        for (TAgentListIterator j = (*i)->begin(); j != (*i)->end(); j++)
        {
            if ((*j)->m_bTempWithInRange)
            {
                tAcceleration.x += (*j)->GetAcceleration()->x;
                tAcceleration.y += (*j)->GetAcceleration()->y;
                unCount++;
            }
        }
    }

    if (unCount > 0)
    {
        tAcceleration.x /= (double) unCount;
        tAcceleration.y /= (double) unCount;
    }

    return tAcceleration;
}

/******************************************************************************/
/******************************************************************************/

bool CompareDistances (CAgent* a, CAgent* b) { return (a->m_fTempDistance < b->m_fTempDistance); }


void CAgent::SortAllAgentsAccordingToDistance(TAgentVector* pt_result)
{
    TAgentVector* ptAllAgents = CSimulator::GetInstance()->GetAllAgents();
    pt_result->resize(ptAllAgents->size());
    copy(ptAllAgents->begin(), ptAllAgents->end(), pt_result->begin());

    for (TAgentVectorIterator i = pt_result->begin(); i != pt_result->end(); i++)
    {
        if ((*i) == this)
            (*i)->m_fTempDistance = 0.0;
        else
            (*i)->m_fTempDistance = GetDistanceBetweenPositions(&m_tPosition, &((*i)->m_tPosition));
    }

    sort(pt_result->begin(), pt_result->end(), CompareDistances);
}

/******************************************************************************/
/******************************************************************************/

void CAgent::SetBehavior(ESwarmBehavType e_behavior)
{
    m_eBehavior = e_behavior;
}

/******************************************************************************/
/******************************************************************************/

ESwarmBehavType CAgent::GetBehavior()
{
    return m_eBehavior;
}

/******************************************************************************/
/******************************************************************************/
