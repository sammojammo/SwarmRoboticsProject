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
    static double       FEATURE_DEPTH;                      /***bit depth of features**/

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

    float*         m_pfFeatureValues;
    int*           m_piLastOccuranceEvent;
    int*           m_piLastOccuranceNegEvent;

    int          m_iEventSelectionTimeWindow;

/*    double       m_fVelocityThreshold;
    double       m_fAccelerationThreshold;

    double       m_tAngularVelocityThreshold;
    double       m_tAngularAccelerationThreshold;

    double       m_fRelativeVelocityMagThreshold;
    double       m_fRelativeVelocityDirThreshold;
*/

 /*************bands replacing thresholds*************/
    float* m_pfVelocityBands;
    float* m_pfAccelerationBands;

    float* m_pfAngularVelocityBands;
    float* m_pfAngularAccelerationBands;

    float* m_pfRelativeVelocityMagBands;
    float* m_pfRelativeVelocityDirBands;

    float* m_pfSquaredDistBands;


 /*************************/




    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    unsigned int m_unNbrsCurrQueueIndex;

    unsigned int m_unSumTimeStepsNbrsRange0to3;
    unsigned int m_unSumTimeStepsNbrsRange3to6;

    unsigned int* m_punNbrsRange0to3AtTimeStep;
    unsigned int* m_punNbrsRange3to6AtTimeStep;



    // keeping track of distance travelled by bot in last 100 time-steps
    int              m_iDistTravelledTimeWindow;

    unsigned int     m_unCoordCurrQueueIndex;

    double           m_fSquaredDistTravelled;
//    double           m_fSquaredDistThreshold;

    TVector2d*       m_pvecCoordAtTimeStep;


};

/******************************************************************************/
/******************************************************************************/


#endif
