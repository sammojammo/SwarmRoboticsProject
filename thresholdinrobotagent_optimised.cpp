#include "thresholdinrobotagent_optimised.h"

/******************************************************************************/
/******************************************************************************/

ThresholdinRobotAgentOptimised::ThresholdinRobotAgentOptimised(CRobotAgentOptimised* ptr_robotAgent,
                                                   CArguments* m_crmArguments)
{
    robotAgent = ptr_robotAgent;

    static bool bHelpDisplayed = false;

    CFeatureVector::NUMBER_OF_FEATURES = m_crmArguments->GetArgumentAsIntOr("numberoffeatures", 6);

    m_uThreshold   = m_crmArguments->GetArgumentAsIntOr("th", 1);

    if (m_crmArguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("numberoffeatures=#            Number of features in a single FV [%d]\n"
               "th=#                          Abnormal agent detection threshold [%d]\n",
               CFeatureVector::NUMBER_OF_FEATURES,
               m_uThreshold);
        bHelpDisplayed = true;
    }
}

/******************************************************************************/
/******************************************************************************/

ThresholdinRobotAgentOptimised::~ThresholdinRobotAgentOptimised()
{
}

/******************************************************************************/
/******************************************************************************/

void ThresholdinRobotAgentOptimised::SimulationStepUpdatePosition()
{    
    Random::nextDouble(); /* We want the same sequence of random numbers generated as with the CRM */

    UpdateState();
}

/******************************************************************************/
/******************************************************************************/

void ThresholdinRobotAgentOptimised::UpdateState()
{
    list<structFVsSensed>* fvsensed = robotAgent->GetFeatureVectorsSensed();
    list<structFVsSensed>::iterator it_fvsensed = fvsensed->begin();

    while(it_fvsensed != fvsensed->end())
    {
        if((*it_fvsensed).fRobots <= m_uThreshold)
            robotAgent->SetMostWantedList(&it_fvsensed, 1); //Attack
        else
            robotAgent->SetMostWantedList(&it_fvsensed, 2); //Tolerate

         ++it_fvsensed;
    }
}

/******************************************************************************/
/******************************************************************************/
