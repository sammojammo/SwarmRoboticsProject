#ifndef AGENT_H_
#define AGENT_H_

/******************************************************************************/
/******************************************************************************/
// FAULT DETECTION MODEL TYPE
enum faultdetectionmodeltype {CRM, CRM_TCELLSINEXCESS, CTRNN, LINEQ, THRESHOLDONFVDIST};

#define FDMODELTYPE  LINEQ

/******************************************************************************/
/******************************************************************************/

enum ESwarmBehavType
{
    AGGREGATION,
    DISPERSION,
    FLOCKING,
    HOMING1,
    HOMING2,

    STRAIGHTLINE,
    RANDOMWK,
    CIRCLE,
    STOP,

    NOERR
};

/******************************************************************************/
/******************************************************************************/

#define MODELSTARTTIME 450

/******************************************************************************/
/******************************************************************************/

//#define DEBUGFEATUREVECTORFLAG // verbose - prints stmts on cell densities, fvs etc... (currently only implemented for crm optimised model)
//#define DEBUGCROSSREGULATIONMODELFLAG
#define FLOATINGPOINTOPERATIONS

/******************************************************************************/
/******************************************************************************/

//#define DISABLEMODEL_RETAINRNDCALLS

/******************************************************************************/
/******************************************************************************/

//#define TCELLCLONEEXCHANGEANALYSIS

/******************************************************************************/
/******************************************************************************/

#define DEACTIVATIONTHRESHOLD 50 //120 // the number of consequtive time-steps an agent is faulted, before it is surrounded

/******************************************************************************/
/******************************************************************************/

class CAgent;

/******************************************************************************/
/******************************************************************************/

#include "common.h"

/******************************************************************************/
/******************************************************************************/


typedef struct {
    float fRed;
    float fGreen;
    float fBlue;
} TColor3f;



enum EControllerType
{
    RANDOMWALK,
    REGULARBOUNCE,
    RANDOMBOUNCE
};


enum EAgentType
{
    ANY,
    ROBOT,
    LIGHT
};

/******************************************************************************/
/******************************************************************************/

typedef list<CAgent*>               TAgentList;
typedef list<CAgent*>::iterator     TAgentListIterator;
typedef list<TAgentList*>           TAgentListList;
typedef list<TAgentList*>::iterator TAgentListListIterator;

typedef vector<CAgent*>             TAgentVector;
typedef vector<CAgent*>::iterator   TAgentVectorIterator;


/******************************************************************************/
/******************************************************************************/

class CAgent : public CSimObject
{
public:
    CAgent(const char* pch_name, unsigned int un_identification, CArguments* pc_arguments);
    virtual ~CAgent();

    // Get the current position of the agent:
    virtual const TVector2d* GetPosition() const;

    // Set the current position of the agent:
    virtual void SetPosition(TVector2d* pt_new_position);

    // Get the current velocity of the agent:
    virtual const TVector2d* GetVelocity() const;

    // Get the current velocity of the agent:
    virtual const TVector2d* GetAcceleration() const;

    // Get the change in velocity direction of the agent
    virtual double GetAngularVelocity();

    // Get the angular acceleration of the agent
    virtual double GetAngularAcceleration();

    // Get the current velocity of the agent:
    virtual void SetVelocity(TVector2d* pt_velocity_position);

    // Get the magnitude and direction of agent's relative velocity
    virtual void GetRelativeVelocity(double* mag_relvelocity, double* dir_relvelocity, double feature_range);

    // Get the magnitude and direction of agent's relative acceleration
    virtual void GetRelativeAcceleration(double *mag_relacceleration, double *dir_relacceleration, double feature_range);


    // Get the angle between the two vectors (acos)
    virtual double GetVectorAngle(TVector2d vector1, TVector2d vector2);

    // This method is called if the agent moves to a new arena square.
    // Useful to calculate distances to other agents, update physical
    // links etc.
    virtual void SimulationStep(unsigned int n_step_number);
    virtual void SimulationStepUpdatePosition();

    virtual void   SetMaximumSpeed(double f_max_speed);
    virtual double GetMaximumSpeed() const;

    virtual double GetMaximumAngularVelocity() const;

    static unsigned int g_unGlobalNumberOfAgentsCreated;

    virtual unsigned int GetColor();
    virtual void         SetColor(unsigned int un_index);

    virtual double       GetSize();

    virtual EAgentType   GetType() = 0;
//    virtual bool         AcceptConnections() = 0;

    virtual inline unsigned int GetIdentification() {return m_unIdentification;}

    virtual void SetBehavIdentification(int i_behavidentification);
    virtual int  GetBehavIdentification();

    bool      m_bTempWithInRange;
    double    m_fTempDistance;

    static double RADIUS;

    virtual CAgent* GetClosestAgent(double f_range, EAgentType e_type);

    virtual TVector2d    GetCenterOfMassOfSurroundingAgents(double f_range, EAgentType e_type);
    virtual double       GetAverageDistanceToSurroundingAgents(double f_range, EAgentType e_type);
    virtual TVector2d    GetAverageVelocityOfSurroundingAgents(double f_range, EAgentType e_type);
    virtual TVector2d    GetAverageAccelerationOfSurroundingAgents(double f_range, EAgentType e_type);
    virtual void         MarkAgentsWithinRange(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type);
    virtual CAgent*      GetRandomAgentWithinRange(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type);
    virtual unsigned int CountAgentsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type);
    virtual unsigned int CountAgents(double f_range, EAgentType e_type);

    virtual void SetRandomVelocity();
    virtual void MoveTowards(TVector2d t_position, double f_max_speed);

    virtual void SortAllAgentsAccordingToDistance(TAgentVector* pt_result);
    virtual void SetBehavior(ESwarmBehavType e_behavior);
    virtual ESwarmBehavType GetBehavior();


protected:
    ESwarmBehavType m_eBehavior;

    TVector2d    m_tPosition;
    TVector2d    m_tVelocity;
    TVector2d    m_tAcceleration;

    double       m_fMaximumSpeed;
    double       m_fMaximumPhysicalRange_Recruitment;

    double       m_tAngularVelocity; // tracks changes in velocity (direction) of the agent
    double       m_tAngularAcceleration; // tracks changes in velocity (direction) of the agent

    EControllerType    m_eControllerType;

    bool               m_bInteractable;

    unsigned int       m_unIdentification;

    unsigned int       m_iBehavIdentification; // -1: abnormal, +1: normal

    unsigned int       m_unColor;

    double             m_fProportionalDirectionNoise;
    double             m_fProportionalMagnitudeNoise;

    CArguments*        m_pcArguments;
};

/******************************************************************************/
/******************************************************************************/

#endif
