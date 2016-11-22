#ifndef CRMINROBOTAGENT_H
#define CRMINROBOTAGENT_H

/******************************************************************************/
/******************************************************************************/
#include <assert.h>
#include <limits.h>
#include <math.h>
#include "arguments.h"
#include "featurevector.h"
#include "random.h"
#include "robotagent.h"

/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgent;

/******************************************************************************/
/******************************************************************************/

class CRobotAgent;

/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgent
{
public:
    CRMinRobotAgent(CRobotAgent* ptr_robotAgent, CArguments* m_crmArguments);

    virtual ~CRMinRobotAgent();

    virtual double GetCurrE(unsigned thtype);
    virtual double GetCurrR(unsigned thtype);

    virtual void SetCurrE(unsigned thtype, double f_currE);
    virtual void SetCurrR(unsigned thtype, double f_currR);

    virtual double GetAPC(unsigned apctype);

    virtual double FreeThCells(double* E, double* R, double** C, unsigned int thtype);
    virtual double AvailableBindingSites(double** C, unsigned int apctype);

    virtual double Factorial(double val);
    virtual double Combination(double N, double R);
    virtual double Hyp(double N, double No, double M, double L);

    virtual void ConjugatesQSS(double* E, double* R, double** C);
    virtual void Derivative(double* E, double* R, double** C, double* deltaE, double* deltaR);

    virtual double GetFVtoApcScaling();

    virtual bool GetConvergenceFlag();

    virtual double GetConvergenceError();
    virtual double GetConvergenceError_Perc();

    virtual void PrintCRMDetails(unsigned id);

    virtual void SimulationStepUpdatePosition();

    void ScaleDownConjugates(double** f_Conjugates);

    double* m_pfSumEffectorsWeightedbyAffinity;
    double* m_pfSumRegulatorsWeightedbyAffinity;


    static unsigned int GetNumberOfSetBits(unsigned int x);

protected:

    CRobotAgent* robotAgent;

    static double NormalizedAffinity(unsigned int v1, unsigned int v2);
    static double NegExpDistAffinity(unsigned int v1, unsigned int v2, double k);

    virtual void UpdateState();

    virtual void Sense();

    //virtual bool TryToConnectToRandomAgentWithWeights(EAgentType e_type, double f_range);
    //virtual CAgent* GetRandomAgentWithinRangeWithWeights(TAgentListList* pt_agents_list_list, double f_range, EAgentType e_type);
    //virtual double CountWeightsInAgentListList(TAgentListList* ptlist_agent_list_list, double f_range, EAgentType e_type);

    virtual double GetWeight();


    double step_h; double conjstep_h; // internal step count of the CRM instance
    double currE; // : Density of effector cells
    double currR; // : Density of regulatory cells
    double kon;   // : Conjugation rate
    double koff;  // : Dissociation rate
    double kpe;   // : Proliferation rate for effector cells
    double kde;   // : Death rate for effector cells
    double kpr;   // : Proliferation rate for regulatory cells
    double kdr;   //  Death rate for regulatory cells
    double se;   // Rate of generation of new effector cells
    double sr;   // Rate of generation of new regulatory cells
    unsigned int sites; // Number of binding sites on each APC

    // For communication of cells between robots
    double m_fTryExchangeProbability; // Probability of trying to exchange cells with other robots
    //double m_fExchangeRange;

    double*        m_pfEffectors;
    double*        m_pfRegulators;
    double*        m_pfEffectors_prev;
    double*        m_pfRegulators_prev;

    double*        m_pfAPCs;


    // predicted number of cells at time t+step with Euler method
    double* m_pfEffectors_Eu;
    double* m_pfRegulators_Eu;
    // predicted number of cells at time t+step with Huen method
    double* m_pfEffectors_Hu;
    double* m_pfRegulators_Hu;
    // the slopes at time = t and time = t+step
    double* m_pfDeltaEffectors_k0;
    double* m_pfDeltaRegulators_k0;
    double* m_pfDeltaEffectors_k1;
    double* m_pfDeltaRegulators_k1;

    // for the computation of conjugates in QSS
    double** m_pfDeltaConjugates_k0;
    double** m_pfDeltaConjugates_k1;
    double** m_pfConj_tmp_Eu;
    double** m_pfConj_tmp_Hu;


    unsigned int   m_unNumberOfReceptors;

    double**        m_pfConjugates;
    double**        m_pfConjugates_tmp;
    double**        m_pfConjugates_Eu; //  the number of conjugates for cells at time t+step, predicted with Eulers method

    double**        m_pfEffectorConjugates;
    double**        m_pfRegulatorConjugates;
    double*         m_pfEffectorConjugatesPerAPC;
    double*         m_pfRegulatorConjugatesPerAPC;

    double**        m_pfAffinities;

    double*         m_pfEff_h;
    double*         m_pfReg_h;

    double          m_fAttackProbability;

    double          m_fcross_affinity; /* the level of cross affinity*/

    //int*            m_pbAttack; // 0: No state 1: Attack 2: Tolerate

    double          m_fWeight;

    double          m_fFVtoApcscaling;

    bool            m_bConvergenceFlag;
    double          m_dconvergence_error;
    double          m_dpercconvergence_error;

};

/******************************************************************************/
/******************************************************************************/

#endif // CRMINROBOTAGENT_H
