#include "lineqinrobotagent.h"
#include <math.h>

/******************************************************************************/
/******************************************************************************/

LINEQinRobotAgent::LINEQinRobotAgent(CRobotAgent* ptr_robotAgent, CArguments* m_lineqArguments)
{
    robotAgent = ptr_robotAgent;

    m_fWeight         = 1.0;
    static bool bHelpDisplayed = false;

    CFeatureVector::NUMBER_OF_FEATURES = m_lineqArguments->GetArgumentAsIntOr("numberoffeatures", 6);
    m_fcross_affinity                  = m_lineqArguments->GetArgumentAsDoubleOr("cross_affinity", 0.8);
    m_fTryExchangeProbability          = m_lineqArguments->GetArgumentAsDoubleOr("exchangeprob", 0.5);
    m_fFVtoApcscaling                  = m_lineqArguments->GetArgumentAsDoubleOr("fvapcscaling", 1.0); //2.0E-3

    /******************************************************************************/
    //LINEQ - FV
    m_fMemory                          = m_lineqArguments->GetArgumentAsDoubleOr("lineqfv_memory", 0.0);
    m_fThreshold                       = m_lineqArguments->GetArgumentAsDoubleOr("lineqfv_threshold",  0.004);
    /******************************************************************************/

    if (m_lineqArguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("numberoffeatures=#            Number of features in a single FV [%d]\n"
               "exchangeprob=#.#              Probability of trying to exchange cells with other robots [%f]\n"
               "cross_affinity=#.#            Level of cross-affinity (>0)     [%2.5f]\n"
               "fvapcscaling=#.#              Scaling parameter of [FV] to [APC] [%e]\n"


               "lineqfv_memory=#.#            Memory (crm death rate) term used in lineq [%e]\n"
               "lineqfv_threshold=#.#         Threshold deciding factor (crm saddle point) used on lineq [%e]\n",
               CFeatureVector::NUMBER_OF_FEATURES,
               m_fTryExchangeProbability,
               m_fcross_affinity,
               m_fFVtoApcscaling,

               m_fMemory,
               m_fThreshold
               );
        bHelpDisplayed = true;
    }

    m_unNumberOfReceptors = 1 << (CFeatureVector::NUMBER_OF_FEATURES);

    m_pfAPCs		        = new double[m_unNumberOfReceptors]; // In this implementation, each type of APC presents one FV.


    //    m_pfAffinities = new double* [m_unNumberOfReceptors];
    //    m_pfAffinities[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    //    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    //    {
    //        m_pfAffinities[i] = m_pfAffinities[i-1] + m_unNumberOfReceptors;
    //    }


    m_pfLineqFV       = new double[m_unNumberOfReceptors];
    m_pfLineqFV_prev  = new double[m_unNumberOfReceptors];


    for (unsigned int i = 0; i < m_unNumberOfReceptors; i++)
    {
        m_pfLineqFV[i]          = 0.0;
        m_pfAPCs[i]             = 0.0;

        //        for (unsigned int j = 0; j < m_unNumberOfReceptors; j++)
        //        {
        //            //m_pfAffinities[i][j]      = NegExpDistAffinity(i,j,m_fcross_affinity);
        //            //printf("Af(%d,%d)=%f\n",i,j,GetAf(i,j));
        //        }
    }

    step_h = 1.0;

    assert(m_fcross_affinity > 0.0);
}

/******************************************************************************/
/******************************************************************************/

LINEQinRobotAgent::~LINEQinRobotAgent()
{
    delete m_pfLineqFV;
    delete m_pfLineqFV_prev;

    delete m_pfAPCs;

    //    delete [] m_pfAffinities[0];
    //    delete [] m_pfAffinities;
}

/******************************************************************************/
/******************************************************************************/

void LINEQinRobotAgent::SimulationStepUpdatePosition()
{
    // Convert the number of feature vectors from robot agents in the vicinity to APCs for the LINEQ
    Sense();

#ifndef DISABLEMODEL_RETAINRNDCALLS // DISABLEMODEL_RETAINRNDCALLS is defined so as to be closer to the same sequence of random numbers generated with the normal working of the LINEQ, so that the same agent behaviors may be obtained when LINEQ is disabled.

    for(unsigned index=0; index < m_unNumberOfReceptors; index++)
    {
        m_pfLineqFV_prev[index]  = m_pfLineqFV[index];

        double inputs = 0.0;
        for(unsigned inputindex=0; inputindex < m_unNumberOfReceptors; inputindex++)
            inputs += GetAf(index,inputindex) * m_pfAPCs[inputindex];

//As m_fMemory is 0, m_pfLineqFV = inputs (=Yi)
        m_pfLineqFV[index]  =  inputs;// * (1.0 - m_fMemory) +
                                    //m_fMemory * m_pfLineqFV[index];

        if(m_pfLineqFV[index] < ACTIVATIONLOWERBOUND)
            m_pfLineqFV[index] = 0.0;
    }

#endif

    m_fWeight = 0.0;
    for(unsigned index=0; index < m_unNumberOfReceptors; index++)
    {
        m_fWeight += m_pfLineqFV[index];
    }
    m_fWeight = m_fWeight * m_fWeight;
    robotAgent->SetWeight(m_fWeight);


    //We set the communication range to be twice that of the FV sensory range
    //CRobotAgent* pcRemoteRobotAgent = robotAgent->GetRandomRobotWithWeights(2.0*robotAgent->GetFVSenseRange());

    // select the robot from one of the 10 nearest neighbours - but in these expts. comm does not seem to be needed
    CRobotAgent* pcRemoteRobotAgent = robotAgent->GetRandomRobotWithWeights((unsigned int)((double)robotAgent->GetSelectedNumNearestNbrs()*1.0));

    if (pcRemoteRobotAgent != NULL)
    {
        LINEQinRobotAgent* lineqinRemoteRobotAgent = pcRemoteRobotAgent->GetLINEQinRobotAgent();

        if (lineqinRemoteRobotAgent)
        {
            for(unsigned index=0; index < m_unNumberOfReceptors; index++)
            {
                double currActivtoSend = m_pfLineqFV[index]  * m_fTryExchangeProbability;

                double remoteActiv     = lineqinRemoteRobotAgent->GetLineqFV(index);

                double currActivtoReceive = remoteActiv * m_fTryExchangeProbability;

                lineqinRemoteRobotAgent->SetLineqFV(index, remoteActiv + currActivtoSend - currActivtoReceive);

                m_pfLineqFV[index] += currActivtoReceive - currActivtoSend;
            }
        }
    }

    UpdateState();
}

/******************************************************************************/
/******************************************************************************/

void LINEQinRobotAgent::PrintLINEQDetails(unsigned id)
{
    //unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();
    if(!(robotAgent->GetIdentification() == id))
        return;

    // Print the lineq details
    printf("\n");
    for (unsigned i = 0; i < m_unNumberOfReceptors; i++)
    {
        if(m_pfAPCs[i] > 0.0)
            printf("LineqFV[%d]=%f [APC:%f] [Status:%d]\n",i,m_pfLineqFV[i],m_pfAPCs[i],robotAgent->GetMostWantedList()[i]);
    }
}

/******************************************************************************/
/******************************************************************************/

void LINEQinRobotAgent::UpdateState()
{
#ifndef DISABLEMODEL_RETAINRNDCALLS // DISABLEMODEL_RETAINRNDCALLS is defined so as to be closer to the same sequence of random numbers generated with the normal working of the LINEQ, so that the same agent behaviors may be obtained when LINEQ is disabled.
    for(unsigned index=0; index < m_unNumberOfReceptors; index++)
    {
        if(m_pfAPCs[index] == 0.0)
            // Dont know - no Hidden Neuron Activ to make decision
            robotAgent->SetMostWantedList(index, 0);
        else if (m_pfLineqFV[index] < m_fThreshold)
            // Attack
            robotAgent->SetMostWantedList(index, 1);
        else
            // Tolerate
            robotAgent->SetMostWantedList(index, 2);
    }
#endif
}

/******************************************************************************/
/******************************************************************************/

void LINEQinRobotAgent::Sense()
{
    // Ask the robot you belong to for the number of feature vectors of different types
    // returns in m_pfFeaturesSensed
    float* m_pfFeaturesSensed  = robotAgent->GetFeaturesSensed();

    for (int i = 0; i < m_unNumberOfReceptors; i++)
    {
        m_pfAPCs[i] = (double) m_pfFeaturesSensed[i];///(M_PI * robotAgent->GetFVSenseRange()  * robotAgent->GetFVSenseRange());

        m_pfAPCs[i] = m_fFVtoApcscaling * m_pfAPCs[i];
    }
}

/******************************************************************************/
/******************************************************************************/

double LINEQinRobotAgent::NormalizedAffinity(unsigned int v1, unsigned int v2)
{
    /* XOr operation between the 2 feature vectors */
    unsigned int unXoredString = (v1 ^ v2);

    /* Number of 1's from the result of the previous XOR operation,  at positions preset by mask */
    /* how do we decide whose mask should be used */
    unsigned int unMatching  = LINEQinRobotAgent::GetNumberOfSetBits(unXoredString);

    //TODO: Have to change affinity computation
    return (double) (CFeatureVector::NUMBER_OF_FEATURES - unMatching) / (double)
            CFeatureVector::NUMBER_OF_FEATURES;
}

/******************************************************************************/
/******************************************************************************/

double LINEQinRobotAgent::GetAf(unsigned int i, unsigned int j)
{
    //return NegExpDistAffinity(i,j,m_fcross_affinity);

    /*New affinity function based on euclidean distance*/
    return GetAfEuclidean(i,j, m_fcross_affinity);
}

/******************************************************************************/
/******************************************************************************/

double LINEQinRobotAgent::NegExpDistAffinity(unsigned int v1, unsigned int v2, double k)
{
    /* k is proportional to the level of cross affinity*/
    /* k=0.01 affinity of 1 when HD is 0, else 0  */
    /* k=inf  affinity of 1 for all HD */

    /* XOr operation between the 2 feature vectors */
    unsigned int unXoredString = (v1 ^ v2);

    /* Number of 1's from the result of the previous XOR operation,  at positions preset by mask */
    /* how do we decide whose mask should be used */
    unsigned int hammingdistance  = LINEQinRobotAgent::GetNumberOfSetBits(unXoredString);

    //return 1.0 * exp(-(1.0/k) * (double)hammingdistance);
    // Should we normalize the hammingdistance when input to the exp function, or as above?

    return 1.0 * exp(-(1.0/k) * (double)hammingdistance / (double) CFeatureVector::NUMBER_OF_FEATURES);

}

double LINEQinRobotAgent::GetAfEuclidean(unsigned int fv1, unsigned int fv2, double k)
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

double LINEQinRobotAgent::GetLineqFV(unsigned index)
{
    return m_pfLineqFV[index];
}

/******************************************************************************/
/******************************************************************************/

void LINEQinRobotAgent::SetLineqFV(unsigned index, double f_currActv)
{
    m_pfLineqFV[index] = f_currActv;
}

/******************************************************************************/
/******************************************************************************/

double LINEQinRobotAgent::GetAPC(unsigned apctype)
{
    return m_pfAPCs[apctype];
}

/******************************************************************************/
/******************************************************************************/

double LINEQinRobotAgent::GetWeight()
{
    return m_fWeight;
}

/******************************************************************************/
/******************************************************************************/

double LINEQinRobotAgent::GetFVtoApcScaling()
{
    return m_fFVtoApcscaling;
}

/******************************************************************************/
/******************************************************************************/

// a shortcut, the function GetNumberOfSetBits(...) is same for crminrobotagent and lineqinrobotagent.
//putting it in each class to prevent: multiple definition of `GetNumberOfSetBits(unsigned int)' crminrobotagent.o:/home/danesh/Work/postdocwork/robotsim/bioinstsim2/crminrobotagent.cpp:1295: first defined here
unsigned int LINEQinRobotAgent::GetNumberOfSetBits(unsigned int x)
{
    // from http://stackoverflow.com/questions/1639723/ruby-count-the-number-of-1s-in-a-binary-number
    unsigned int m1 = 0x55555555;
    unsigned int m2 = 0x33333333;
    unsigned int m4 = 0x0f0f0f0f;
    x -= (x >> 1) & m1;
    x = (x & m2) + ((x >> 2) & m2);
    x = (x + (x >> 4)) & m4;
    x += x >> 8;
    return (x + (x >> 16)) & 0x3f;
}

/******************************************************************************/
/******************************************************************************/
