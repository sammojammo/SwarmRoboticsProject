#include "euclideandistinrobotagent_optimised.h"

/******************************************************************************/
/******************************************************************************/

EuclideanDistinRobotAgentOptimised::EuclideanDistinRobotAgentOptimised(CRobotAgentOptimised* ptr_robotAgent,
                                                   CArguments* m_euclideanArguments)
{
    robotAgent = ptr_robotAgent;

    static bool bHelpDisplayed = false;

    CFeatureVector::NUMBER_OF_FEATURES = m_euclideanArguments->GetArgumentAsIntOr("numberoffeatures", 6);
    m_fcross_affinity                  = m_euclideanArguments->GetArgumentAsDoubleOr("cross-affinity", 0.15);
    m_fThreshold                       = m_euclideanArguments->GetArgumentAsDoubleOr("euclidean_threshold",  0.004);

    if (m_euclideanArguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("numberoffeatures=#            Number of features in a single FV [%d]\n"
               "cross-affinity=#.#            Level of cross-affinity (>0)     [%2.5f]\n"
               "euclidean_threshold=#.#                          Abnormal agent detection threshold [%e]\n",
               CFeatureVector::NUMBER_OF_FEATURES,
               m_fcross_affinity,
               m_fThreshold);
        bHelpDisplayed = true;
    }

    m_pfYi       = new double[20]; //number of agents in swarm
}

/******************************************************************************/
/******************************************************************************/

EuclideanDistinRobotAgentOptimised::~EuclideanDistinRobotAgentOptimised()
{
    delete m_pfYi;
}

/******************************************************************************/
/******************************************************************************/

void EuclideanDistinRobotAgentOptimised::SimulationStepUpdatePosition()
{
#ifndef DISABLEMODEL_RETAINRNDCALLS // DISABLEMODEL_RETAINRNDCALLS is defined so as to be closer to the same sequence of random numbers generated with the normal working of the LINEQ, so that the same agent behaviors may be obtained when LINEQ is disabled.
    list<structFVsSensed>* fvsensed = robotAgent->GetFeatureVectorsSensed();
    list<structFVsSensed>::iterator it_fvsensed = fvsensed->begin();

    while(it_fvsensed != fvsensed->end())
    {
        double inputs = 0.0;

        int it_fvsensedinner = 0;

        while(it_fvsensedinner <=it_fvsensed->fRobots)
        {
            inputs += GetAf(distance(fvsensed->begin(), it_fvsensed),it_fvsensedinner);
            ++it_fvsensedinner;
        }
        m_pfYi[distance(fvsensed->begin(), it_fvsensed)]  =  inputs;

        ++it_fvsensed;
    }

#endif
    UpdateState();
}

/******************************************************************************/
/******************************************************************************/

void EuclideanDistinRobotAgentOptimised::UpdateState()
{
    list<structFVsSensed>* fvsensed = robotAgent->GetFeatureVectorsSensed();
    list<structFVsSensed>::iterator it_fvsensed = fvsensed->begin();

    while(it_fvsensed != fvsensed->end())
    {
        if(m_pfYi[distance(fvsensed->begin(), it_fvsensed)] <= m_fThreshold)
            robotAgent->SetMostWantedList(&it_fvsensed, 1); //Attack
        else
            robotAgent->SetMostWantedList(&it_fvsensed, 2); //Tolerate

         ++it_fvsensed;
    }
}

/******************************************************************************/
/******************************************************************************/

double EuclideanDistinRobotAgentOptimised::GetAf(unsigned int i, unsigned int j)
{
    /*New affinity function based on euclidean distance*/
    return GetAfEuclidean(i,j, m_fcross_affinity);
}

/******************************************************************************/
/******************************************************************************/

double EuclideanDistinRobotAgentOptimised::GetAfEuclidean(unsigned int fv1, unsigned int fv2, double k)
{
    /*Create mask using bit depth of each feature*/
    unsigned int featureMask = 0;

    for(int i = 0; i < CFeatureVector::FEATURE_DEPTH; i++)
        featureMask += (1 << i);

    unsigned int value1 = 0, value2 = 0, currentSum = 0;

    /*Looping through the feature vectors extracting each feature value*/
    /*Feature 2 subtracted from feature 1. result is squared and added to current running total*/
    for(int i = 0; i < CFeatureVector::NUMBER_OF_FEATURES; i++)
    {
        /*shift FV right so the & operation gives the binary value of the feature without 0 padding to the right of it*/
        value1 = ((fv1 >> (i * (unsigned int)CFeatureVector::FEATURE_DEPTH)) & featureMask);
        value2 = ((fv2 >> (i * (unsigned int)CFeatureVector::FEATURE_DEPTH)) & featureMask);

        currentSum += ((value1 - value2) * (value1 - value2));
    }

    /*Square root running total to give euclidean distance*/
    double euclideanDistance = sqrt(currentSum);

    double maxEuclideanDistance = sqrt(CFeatureVector::NUMBER_OF_FEATURES * (CFeatureVector::FEATURE_DEPTH * CFeatureVector::FEATURE_DEPTH));

    //return euclideanDistance;
    return 1.0 * exp( ((-1.0/k) * euclideanDistance) / maxEuclideanDistance);
}

/******************************************************************************/
/******************************************************************************/
