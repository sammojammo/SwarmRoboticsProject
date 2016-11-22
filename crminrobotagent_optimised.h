#ifndef CRMINROBOTAGENTOPTIMISED_H
#define CRMINROBOTAGENTOPTIMISED_H

/******************************************************************************/
/******************************************************************************/
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <list>
#include "arguments.h"
#include "featurevector.h"
#include "random.h"
#include "robotagent_optimised.h"
#include "celldatacontainers.h"

/******************************************************************************/
/******************************************************************************/

using namespace std;

/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgentOptimised;

/******************************************************************************/
/******************************************************************************/

class CRobotAgentOptimised;

/******************************************************************************/
/******************************************************************************/

struct structTcell; struct structAPC;
enum ConjugationIntegrationPhase : unsigned {CONJ_K0, CONJ_K1, CONJ};
enum TcellIntegrationPhase : unsigned {K0, K1};

/******************************************************************************/
/******************************************************************************/

#define DISABLE_PERSISTENCE_HACK // If defined then: persistence threshold is at 0

//#define SELECTIVE_TCELL_INFLUX_RATE // If defined then: Influx RATE (not density) of new T-cells is provided only to T-cell clonaltypes with associated APCs (affinity = 1) present

//#define SELECTIVE_TCELL_INFLUX_DENSITY // If defined then: Influx density of new T-cells is provided at each simulation time-step only to newly added T-cell clonaltypes i.e., t-cell clones clones with associated APCs (affinity = 1) present
/******************************************************************************/
/******************************************************************************/

class CRMinRobotAgentOptimised
{
public:
    CRMinRobotAgentOptimised(CRobotAgentOptimised* ptr_robotAgent, CArguments* m_crmArguments);

    virtual ~CRMinRobotAgentOptimised();

    virtual double GetCurrE(unsigned int thtype);
    virtual double GetCurrR(unsigned int thtype);

    virtual void SetCurrE(unsigned int thtype, double f_currE);
    virtual void SetCurrR(unsigned int thtype, double f_currR);

    virtual double GetAPC(unsigned int apctype);

//    virtual inline double FreeThCells(double* E, double* R, double** C, unsigned int thtype);
//    virtual inline double AvailableBindingSites(double** C, unsigned int apctype);

    virtual void FreeTcellsAndAvailableAPCSites(TcellIntegrationPhase TK, ConjugationIntegrationPhase CONJK);

    virtual void ConjugatesQSS(bool bResetConjugates, TcellIntegrationPhase TK); //double* E, double* R, double** C);
    virtual void Derivative(TcellIntegrationPhase TK); //double* E, double* R, double** C, double* deltaE, double* deltaR);

    virtual void ConjugatesQSS_ExcessTcells(bool bClearDeadConjugates, TcellIntegrationPhase TK); //double* E, double* R, double** C);
    virtual void Derivative_ExcessTcells(TcellIntegrationPhase TK); //double* E, double* R, double** C, double* deltaE, double* deltaR);
    virtual void ComputeNewDerivative(TcellIntegrationPhase TK);

    virtual inline double GetFVtoApcScaling() {return m_fFVtoApcscaling;}

    virtual inline double GetConvergenceError() {return m_dconvergence_error;}
    virtual inline double GetConvergenceError_Perc() {return m_dpercconvergence_error;}

    virtual void PrintCRMDetails(unsigned int id);
    virtual void PrintAPCList(unsigned int id);
    virtual void PrintTcellResponseToAPCList(unsigned int id);
    virtual void PrintTcellList(unsigned int id);
    virtual void PrintConjugatestoAPCList(unsigned int id, ConjugationIntegrationPhase CONJK);
    virtual void PrintConjugatestoTcellList(unsigned int id, ConjugationIntegrationPhase CONJK);

    virtual double GetIntegrationTime_StepFunction();
    virtual void TcellNumericalIntegration_RK2();
    virtual void SimulationStepUpdatePosition();
    virtual void DiffuseTcells();

    void ScaleDownConjugates(ConjugationIntegrationPhase CONJK);

//    double* m_pfSumEffectorsWeightedbyAffinity;
//    double* m_pfSumRegulatorsWeightedbyAffinity;

    inline list<structAPC>*    GetListAPCs() {return &listAPCs;}
    inline list<structTcell>*  GetListTcells() {return &listTcells;}

    static double NegExpDistAffinity(unsigned int v1, unsigned int v2, double k);
    static unsigned int GetNumberOfSetBits(unsigned int x);

protected:

    CRobotAgentOptimised* robotAgent;

    virtual void UpdateState();

    virtual void UpdateAPCList(); //Sense()
    virtual void UpdateTcellList(unsigned int hammingdistance); //unsigned hammingdistance
    virtual void UpdateConjugatesToAPCList();
    virtual void UpdateConjugatesToTcellList();
    virtual void MarkConjugatesOfDeadTcell(list<structTcell>::iterator* ptrit_tcells);

    virtual inline double GetWeight() {return m_fWeight;}


    double step_h; double conjstep_h; // internal step count of the CRM instance
    double seedE; // : Density of effector cells at the start
    double seedR; // : Density of regulatory cells at the start
    double kon;   // : Conjugation rate
    double koff;  // : Dissociation rate
    double kpe;   // : Proliferation rate for effector cells
    double kde;   // : Death rate for effector cells
    double kpr;   // : Proliferation rate for regulatory cells
    double kdr;   //  Death rate for regulatory cells
    double se;    // Density of new effector cells added at each simulation step
    double sr;    // Density of new regulatory cells added at each simulation step
    double se_rate, sr_rate; // Rate of influx of new T-cells
    unsigned int sites; // Number of binding sites on each APC
    unsigned m_uPersistenceThreshold; double m_fIntegrationTime, m_fStartExpIntegrationTime;

    double m_fTCELL_UPPERLIMIT_STEPSIZE, m_fTCELL_LOWERLIMIT_STEPSIZE;
    double m_fERRORALLOWED_TCELL_STEPSIZE, m_fERRORALLOWED_CONJ_STEPSIZE;
    double m_fTCELL_CONVERGENCE, m_fCONJ_CONVERGENCE; //-3//todo: set as percentage instead of absolute value

    // For communication of cells between robots
    double m_fTryExchangeProbability; // Probability of trying to exchange cells with other robots
    //double m_fExchangeRange;

    unsigned m_uSeedfvHdRange; // diversity of seed t-cell population


//    double*        m_pfEffectors;
//    double*        m_pfRegulators;
//    double*        m_pfEffectors_prev;
//    double*        m_pfRegulators_prev;

//    double*        m_pfAPCs;


//    // predicted number of cells at time t+step with Euler method
//    double* m_pfEffectors_Eu;
//    double* m_pfRegulators_Eu;
//    // predicted number of cells at time t+step with Huen method
//    double* m_pfEffectors_Hu;
//    double* m_pfRegulators_Hu;
//    // the slopes at time = t and time = t+step
//    double* m_pfDeltaEffectors_k0;
//    double* m_pfDeltaRegulators_k0;
//    double* m_pfDeltaEffectors_k1;
//    double* m_pfDeltaRegulators_k1;

//    // for the computation of conjugates in QSS
//    double** m_pfDeltaConjugates_k0;
//    double** m_pfDeltaConjugates_k1;
//    double** m_pfConj_tmp_Eu;
//    double** m_pfConj_tmp_Hu;

    list<structTcell> listTcells, listTcells_cpy; // list of non-zero t-cell clonaltypes and a copy in case integration has to be run again
    list<structAPC>   listAPCs;
    virtual inline void IncIt(list<structTcell>::iterator *it_tcell, list<structTcell>* list)
    { (*it_tcell) == list->end() ? (*it_tcell):++(*it_tcell); }
    virtual inline void IncIt(list<structAPC>::iterator *it_apc, list<structAPC>* list)
    { (*it_apc)   == list->end() ? (*it_apc):++(*it_apc); }

    unsigned int   m_unNumberOfReceptors;

//    double**        m_pfConjugates;
//    double**        m_pfConjugates_tmp;
//    double**        m_pfConjugates_Eu; //  the number of conjugates for cells at time t+step, predicted with Eulers method

//    double**        m_pfEffectorConjugates;
//    double**        m_pfRegulatorConjugates;
//    double*         m_pfEffectorConjugatesPerAPC;
//    double*         m_pfRegulatorConjugatesPerAPC;

//    double**        m_pfAffinities;

    double          m_fcross_affinity; /* the level of cross affinity*/


    double          m_fWeight;
    int             m_fApcscalingtype; //0: linear, 1: exp
    double          m_fFVtoApcscaling; //linear scaling - multiplicative factor
    double          m_fFVtoApcscaling_exprate, m_fFVtoApcscaling_expbase; //exp scaling - rate, and base value

    bool            m_bConvergenceFlag;
    double          m_dconvergence_error;
    double          m_dpercconvergence_error;

};


/******************************************************************************/
/******************************************************************************/

#endif // CRMINROBOTAGENTOPTIMISED_H
