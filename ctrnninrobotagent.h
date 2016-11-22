#ifndef CTRNNINROBOTAGENT_H
#define CTRNNINROBOTAGENT_H

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
enum NNLAYER
{
    HiddenLayer,
    OutputLayer
};

/******************************************************************************/
/******************************************************************************/

#define HIDDENNEURONACTIVATIONLOWERBOUND 1.0e-6 //todo: set as percentage instead of absolute value

/******************************************************************************/
/******************************************************************************/

class CTRNNinRobotAgent;

/******************************************************************************/
/******************************************************************************/

class CRobotAgent;

/******************************************************************************/
/******************************************************************************/

class CTRNNinRobotAgent
{
public:
    CTRNNinRobotAgent(CRobotAgent* ptr_robotAgent, CArguments* m_ctrnnArguments);

    virtual ~CTRNNinRobotAgent();

    virtual double GetHN(unsigned hnindex);
    virtual void SetHN(unsigned hnindex, double f_hn);

    virtual double GetON(unsigned hnindex);
    virtual double GetInputsToON(unsigned hnindex);

    virtual double GetAPC(unsigned apctype);

    virtual double GetFVtoApcScaling();

    virtual bool   GetConvergenceFlag(NNLAYER layer);
    virtual double GetConvergenceError(NNLAYER layer);
    virtual double GetConvergenceError_Perc(NNLAYER layer);

    virtual void PrintCTRNNDetails(unsigned id);

    virtual void SimulationStepUpdatePosition();

    static unsigned int GetNumberOfSetBits(unsigned int x);
    static double Sigmoid(double f_currActv, double f_Saturation);

    virtual double GetSigmoidSaturation();

protected:

    CRobotAgent* robotAgent;

    static double NormalizedAffinity(unsigned int v1, unsigned int v2);
    static double NegExpDistAffinity(unsigned int v1, unsigned int v2, double k);
    double GetAf(unsigned int v1, unsigned int v2);


    virtual void Derivative(double *hn, double *deltaHN, NNLAYER layer);
    virtual void NumericalIntegration(double totalintegration_t, double* neurons, double* neurons_prev, double* convg_error, double* percconvg_error, bool* convg_flag, NNLAYER layer);

    virtual void UpdateState();

    virtual void Sense();

    virtual double GetWeight();



    double step_h; // internal step count of the CTRNN instance

    // For communication of hidden neuron activation levels between robots
    double m_fTryExchangeProbability; // Probability of trying to exchange cells with other robots
    //double m_fExchangeRange;

    // hidden neurons
    double         m_fCOMPETITION_FACTOR;
    double         m_fACTIVATION_FACTOR;

    // output neurons
    double         m_fOUTPUTGAIN_FACTOR;
    // PLASTIC BIAS
    double         m_fINHIBTIONSCALINGFACTOR;
    double         m_fBIASWEIGHT;
    double         m_fEXTERNALBIASWEIGHT;
    double         m_fSIGMOIDSATURATION;

    // or FIXED (NONPLASTIC) BIAS
    double         m_fOUTPUTNEURONBIAS;


    double*        m_pfOutputNeurons;
    double*        m_pfOutputNeurons_prev;
    double*        m_pfInputToOutputNeurons;

    double*        m_pfHiddenNeurons;
    double*        m_pfHiddenNeurons_prev;
    double         m_pfNeuronTimeConstant;


    // predicted activation level of neurons at time t+step with Euler method
    double* m_pfNeurons_Eu;
    // predicted activation level of neurons at time t+step with Heun method
    double* m_pfNeurons_Hu;
    // the slopes at time = t and time = t+step
    double* m_pfDeltaNeurons_k0;
    double* m_pfDeltaNeurons_k1;


    double*        m_pfAPCs;
    //double**       m_pfAffinities;
    unsigned int   m_unNumberOfReceptors;


    //double          m_fAttackProbability;
    double          m_fcross_affinity; /* the level of cross affinity*/
    double          m_fWeight;
    double          m_fFVtoApcscaling;

    bool            m_bhlconvergence_flag;
    double          m_fhlconvergence_error;
    double          m_fhlpercconvergence_error;

    bool            m_bolconvergence_flag;
    double          m_folconvergence_error;
    double          m_folpercconvergence_error;
};

/******************************************************************************/
/******************************************************************************/

#endif // CTRNNINROBOTAGENT_H
