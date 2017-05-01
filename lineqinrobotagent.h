#ifndef LINEQINROBOTAGENT_H
#define LINEQINROBOTAGENT_H

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

#define ACTIVATIONLOWERBOUND 1.0e-6 //todo: set as percentage instead of absolute value

/******************************************************************************/
/******************************************************************************/

class LINEQinRobotAgent;

/******************************************************************************/
/******************************************************************************/

class CRobotAgent;

/******************************************************************************/
/******************************************************************************/

class LINEQinRobotAgent
{
public:
    LINEQinRobotAgent(CRobotAgent* ptr_robotAgent, CArguments* m_lineqArguments);

    virtual ~LINEQinRobotAgent();

    virtual double GetLineqFV(unsigned index);
    virtual void   SetLineqFV(unsigned index, double f_hn);

    virtual double GetAPC(unsigned apctype);

    virtual double GetFVtoApcScaling();

    virtual void PrintLINEQDetails(unsigned id);

    virtual void SimulationStepUpdatePosition();

    static unsigned int GetNumberOfSetBits(unsigned int x);

protected:

    CRobotAgent* robotAgent;

    static double NormalizedAffinity(unsigned int v1, unsigned int v2);
    static double NegExpDistAffinity(unsigned int v1, unsigned int v2, double k);
    static double GetAfEuclidean(unsigned int v1, unsigned int v2, double k);
    double GetAf(unsigned int v1, unsigned int v2);

    virtual void UpdateState();

    virtual void Sense();

    virtual double GetWeight();



    double step_h; // internal step count of the LINEQ instance

    // For communication of hidden neuron activation levels between robots
    double m_fTryExchangeProbability; // Probability of trying to exchange cells with other robots
    //double m_fExchangeRange;

    double         m_fMemory;
    double         m_fThreshold;

    double*        m_pfLineqFV;
    double*        m_pfLineqFV_prev;



    double*        m_pfAPCs;
    //double**       m_pfAffinities;
    unsigned int   m_unNumberOfReceptors;

    double          m_fcross_affinity; /* the level of cross affinity*/
    double          m_fWeight;
    double          m_fFVtoApcscaling;
};

/******************************************************************************/
/******************************************************************************/

#endif // LINEQINROBOTAGENT_H
