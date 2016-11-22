#ifndef ROBOTAGENT_H_
#define ROBOTAGENT_H_

/******************************************************************************/
/******************************************************************************/

class CRobotAgent;

/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgent;

/******************************************************************************/
/******************************************************************************/

class CTRNNinRobotAgent;

/******************************************************************************/
/******************************************************************************/

class LINEQinRobotAgent;

/******************************************************************************/
/******************************************************************************/


#include "common.h"
#include "agent.h"
#include "behavior.h"
#include "crminrobotagent.h"
#include "ctrnninrobotagent.h"
#include "lineqinrobotagent.h"

/******************************************************************************/
/******************************************************************************/

class CRobotAgent : public CAgent
{
public: 
    CRobotAgent(const char* pch_name, unsigned int un_identification, CArguments* pc_agent_arguments, CArguments* pc_model_arguments, TBehaviorVector vec_behaviors);
    virtual ~CRobotAgent();
    
    // This method is called if the agent moves to a new arena square.
    // Useful to calculate distances to other agents, update physical
    // links etc.
    virtual void SimulationStepUpdatePosition();    
    virtual void SetBehaviors(TBehaviorVector vec_behaviors);
    virtual TBehaviorVector GetBehaviors();
    
    virtual EAgentType   GetType();   

    // Gets the number of feature vectors of different types
    // into m_pfFeaturesSensed.
    virtual float* GetFeaturesSensed() const;

    virtual CRobotAgent* GetRandomRobotWithWeights(double f_range);
    virtual CRobotAgent* GetRandomRobotWithWeights(unsigned int u_nearestnbrs);

    virtual CRMinRobotAgent*   GetCRMinRobotAgent();
    virtual CTRNNinRobotAgent* GetCTRNNinRobotAgent();
    virtual LINEQinRobotAgent* GetLINEQinRobotAgent();

    virtual void   SetWeight(double f_weight);
    virtual double GetWeight() const; 
    virtual const CFeatureVector* GetFeatureVector() const;
    virtual void Sense(unsigned int u_nearestnbrs);
    virtual void FVsOfWcFeature(const CFeatureVector* pc_feature_vector, unsigned int *fv1, unsigned int *fv2);
    virtual double GetFVSenseRange() const;
    virtual unsigned int GetColor();
    virtual unsigned int GetSelectedNumNearestNbrs();

    virtual void SetMostWantedList(unsigned unFeatureVector, unsigned int state);
    virtual unsigned int*  GetMostWantedList();
    
    virtual void CheckNeighborsResponseToMyFV(unsigned int* pun_number_of_toleraters, unsigned int* pun_number_of_attackers, unsigned int* pun_number_of_unconverged, bool b_logs);

    virtual void PrintDecidingAgentDetails(CFeatureVector* m_pcFV, CRMinRobotAgent* model_crminagent, CTRNNinRobotAgent* model_ctrnninagent, LINEQinRobotAgent* model_lineqinagent, float* FeatureVectorsSensed);

    virtual unsigned int Attack(CFeatureVector* pc_feature_vector);

protected:
    virtual double CountWeightsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range);

    double              m_fFVSenseRange;
    CFeatureVector*     m_pcFeatureVector;
    TBehaviorVector     m_vecBehaviors;
    CRMinRobotAgent*    crminAgent;
    CTRNNinRobotAgent*  ctrnninAgent;
    LINEQinRobotAgent*  lineqinAgent;

    double              m_fWeight;
    double              m_fBitflipProbabililty;

    float*              m_pfFeaturesSensed; //2^n
    unsigned int*       m_pbMostWantedList; //2^n

    double              m_fResponseRange;
    unsigned int        m_uSelectedNumNearestNbrs;

};

/******************************************************************************/
/******************************************************************************/

#endif // ROBOTAGENT_H_

/******************************************************************************/
/******************************************************************************/
