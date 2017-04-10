#ifndef FEATUREVECTOR_H_
#define FEATUREVECTOR_H_

/******************************************************************************/
/******************************************************************************/

class CTestExperiment;

/******************************************************************************/
/******************************************************************************/

#include "common.h"
#include "agent.h"
#include "simulator.h"

//#include "testexperiment.h"

#include <string>



/******************************************************************************/
/******************************************************************************/

class CFeatureVector
{
public:
    CFeatureVector(CAgent* pc_agent);
    virtual ~CFeatureVector();

    static unsigned int NUMBER_OF_FEATURES;
    static unsigned int NUMBER_OF_FEATURE_VECTORS;
    static double       FEATURE_RANGE;
    static double       FEATURE_DEPTH;

    virtual unsigned int GetValue() const;
    virtual unsigned int GetLength() const;

    virtual float GetFeatureValue(int featureNum) const;

    void PrintFeatureDetails();

    virtual unsigned int SimulationStep();

    virtual std::string ToString();


protected:
    virtual void ComputeFeatureValues();

    CAgent*      m_pcAgent;
    unsigned int m_unValue;
    unsigned int m_unLength;
    unsigned int m_unCorrectResponse;

    float*         m_pfFeatureValues;
    int*           m_piLastOccuranceEvent;
    int*           m_piLastOccuranceNegEvent;

    int          m_iEventSelectionTimeWindow;
    int          m_iCorrectResponseTimeWindow;

    double          m_fRobotHeading;
    double          m_fPreviousRobotHeading;


    double       m_tAngularAccelerationThreshold;

 /*************bands replacing thresholds*************/
    float* m_pfVelocityBands;
    float* m_pfAccelerationBands;

    float* m_pfSensoryMotorBands;

    float* m_pfSquaredDistBands;

    float* m_pfCorrectResponseBands;
 /*************************/




    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    unsigned int m_unNbrsCurrQueueIndex;

    unsigned int m_unSumTimeStepsNbrsRange0to3;
    unsigned int m_unSumTimeStepsNbrsRange3to6;

    unsigned int* m_punNbrsRange0to3AtTimeStep;
    unsigned int* m_punNbrsRange3to6AtTimeStep;


    //Sensory motor vars
    unsigned int  m_unTurnCurrQueueIndex;

    unsigned int  m_unSensoryMotorFarCount;
    unsigned int  m_unSensoryMotorNearCount;

    unsigned int* m_punTurnedWithNbrsAtTimeStep;
    unsigned int* m_punTurnedWithoutNbrsAtTimeStep;




    // keeping track of distance travelled by bot in last 100 time-steps
    int              m_iDistTravelledTimeWindow;

    unsigned int     m_unCoordCurrQueueIndex;

    double           m_fSquaredDistTravelled;


    TVector2d*       m_pvecCoordAtTimeStep;


};

/******************************************************************************/
/******************************************************************************/


#endif
