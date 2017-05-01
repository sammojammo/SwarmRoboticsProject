#ifndef EUCLIDEANDISTINROBOTAGENTOPTIMISED_H
#define EUCLIDEANDISTINROBOTAGENTOPTIMISED_H

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

/******************************************************************************/
/******************************************************************************/

using namespace std;

/******************************************************************************/
/******************************************************************************/

class EuclideanDistinRobotAgentOptimised;

/******************************************************************************/
/******************************************************************************/

class CRobotAgentOptimised;

/******************************************************************************/
/******************************************************************************/

class EuclideanDistinRobotAgentOptimised
{
public:
    EuclideanDistinRobotAgentOptimised(CRobotAgentOptimised* ptr_robotAgent, CArguments* m_euclideanArguments);

    virtual ~EuclideanDistinRobotAgentOptimised();

    virtual void SimulationStepUpdatePosition();

protected:
    CRobotAgentOptimised* robotAgent;

    static double GetAfEuclidean(unsigned int v1, unsigned int v2, double k);
    double GetAf(unsigned int v1, unsigned int v2);

    virtual void UpdateState();


    double         m_fThreshold;

    double*        m_pfYi;

    double          m_fcross_affinity;
    double          m_fWeight;
    double          m_fFVtoApcscaling;

};


/******************************************************************************/
/******************************************************************************/

#endif  //EUCLIDEANDISTINROBOTAGENTOPTIMISED_H
