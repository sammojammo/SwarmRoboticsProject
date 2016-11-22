#include "ctrnninrobotagent.h"

/******************************************************************************/
/******************************************************************************/

//#define DEBUG // Verbose output to help debugging - relevant to CTRNN, and system seemingly stalling

#define NN_UPPERLIMIT_STEPSIZE 500000.0 //todo: could be set as a propotion of the INTEGRATION_TIME
#define NN_LOWERLIMIT_STEPSIZE 1.0e-6
#define NN_TIMECONSTANT NN_UPPERLIMIT_STEPSIZE
#define ERRORALLOWED_NN_STEPSIZE 1.0e-6 //todo: set as percentage instead of absolute value


#define INTEGRATION_TIME_SIMSTART 5.0e+7


#define NN_CONVERGENCE  ERRORALLOWED_NN_STEPSIZE

// bias on output neurons
//#define FIXEDBIAS
#define AFFINITYPLASTICBIAS

//#ifdef FIXEDBIAS
//#define m_fOUTPUTNEURONBIAS 0.003
//#elif defined AFFINITYPLASTICBIAS // affinity specific bias
//#define m_fBIASWEIGHT               0.0 // doesnot do well when inputs are scaled large (e.g. 10 fold or .1 fold change to all) // what it does is translate the output values
//#define m_fINHIBTIONSCALINGFACTOR   1.0 //0.25 // what it does is scaling the output values
//#define EXTERNALm_fBIASWEIGHT       0.0 // in case of only one type of object in agents local env. Not needed for fault detection, where they would always be a reference normal repertoire of agents
//#endif
/******************************************************************************/
/******************************************************************************/


CTRNNinRobotAgent::CTRNNinRobotAgent(CRobotAgent* ptr_robotAgent, CArguments* m_ctrnnArguments)
{
    robotAgent = ptr_robotAgent;

    m_fWeight         = 1.0;
    static bool bHelpDisplayed = false;

    CFeatureVector::NUMBER_OF_FEATURES = m_ctrnnArguments->GetArgumentAsIntOr("numberoffeatures", 6);
    m_fcross_affinity                  = m_ctrnnArguments->GetArgumentAsDoubleOr("cross-affinity", 0.15);
    m_fTryExchangeProbability          = m_ctrnnArguments->GetArgumentAsDoubleOr("exchangeprob", 0.5);
    m_fFVtoApcscaling                  = m_ctrnnArguments->GetArgumentAsDoubleOr("fvapcscaling", 1.0); //2.0E-3

    /******************************************************************************/
    //Hidden neurons
    m_fCOMPETITION_FACTOR              = m_ctrnnArguments->GetArgumentAsDoubleOr("hn_competition", 1.0);
    m_fACTIVATION_FACTOR               = m_ctrnnArguments->GetArgumentAsDoubleOr("hn_activation", 1.0);
    /******************************************************************************/
    ////Output neurons
    m_fOUTPUTGAIN_FACTOR               = m_ctrnnArguments->GetArgumentAsDoubleOr("on_gain", 1.0);//10.0

    // AFFINITYPLASTICBIAS
    // Scales the output values. The inhibition factor is reduced so as to treat more than one agent with the same FV, as normal.
    m_fINHIBTIONSCALINGFACTOR          = m_ctrnnArguments->GetArgumentAsDoubleOr("on_inhibitionscaling", 0.4); //20.0
    m_fBIASWEIGHT                      = m_ctrnnArguments->GetArgumentAsDoubleOr("on_biaswt", 0.0);
    m_fEXTERNALBIASWEIGHT              = m_ctrnnArguments->GetArgumentAsDoubleOr("on_extbiaswt", 0.0);

    //Reducing this rate shifts or translates the output values up (uniformly increases the values).
    m_fSIGMOIDSATURATION               = m_ctrnnArguments->GetArgumentAsDoubleOr("on_sigmoidsaturation", 1.0); //0.05
    // or FIXED (NONPLASTIC) BIAS
    m_fOUTPUTNEURONBIAS                = m_ctrnnArguments->GetArgumentAsDoubleOr("on_nonplasticbiaswt", 0.004); //0.004
    /******************************************************************************/

    if (m_ctrnnArguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("numberoffeatures=#            Number of features in a single FV [%d]\n"
               "exchangeprob=#.#              Probability of trying to exchange cells with other robots [%f]\n"
               "cross-affinity=#.#            Level of cross-affinity (>0)     [%2.5f]\n"
               "fvapcscaling=#.#              Scaling parameter of [FV] to [APC] [%e]\n"


               "hn_competition=#.#            Scaling parameter of competition between hidden neurons [%e]\n"
               "hn_activation=#.#             Scaling parameter of activation of hidden neurons [%e]\n"


               "on_gain=#.#                   Multiplicative gain applied to inputs of output neurons [%e]\n"
#ifdef FIXEDBIAS
               "on_nonplasticbiaswt=#.#       Fixed non-plastic bias weight on output neurons (bias activation: -1) [%e]\n",
#elif defined AFFINITYPLASTICBIAS
               "on_inhibitionscaling=#.#      Scaling parameter on inhibitions between output neurons [%e]\n"
               "on_biaswt=#.#                 Bias weight on output neurons [%e]\n"
               "on_extbiaswt=#.#              External bias weight applied on final activations of output neurons (bias activation: -1) [%e]\n"
               "on_sigmoidsaturation=#.#      Saturation parameter of output neuron sigmoid function [%e]\n",
#endif
               CFeatureVector::NUMBER_OF_FEATURES,
               m_fTryExchangeProbability,
               m_fcross_affinity,
               m_fFVtoApcscaling,

               m_fCOMPETITION_FACTOR,
               m_fACTIVATION_FACTOR,
               m_fOUTPUTGAIN_FACTOR,

#ifdef FIXEDBIAS
               m_fOUTPUTNEURONBIAS
#elif defined AFFINITYPLASTICBIAS
               m_fINHIBTIONSCALINGFACTOR,
               m_fBIASWEIGHT,
               m_fEXTERNALBIASWEIGHT,
               m_fSIGMOIDSATURATION
#endif
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


    m_pfHiddenNeurons       = new double[m_unNumberOfReceptors];
    m_pfHiddenNeurons_prev  = new double[m_unNumberOfReceptors];

    // predicted activation level of (hidden/output) neurons at time t+step with Euler method
    m_pfNeurons_Eu    = new double[m_unNumberOfReceptors];
    // predicted activation level of (hidden/output) neurons at time t+step with Huen method
    m_pfNeurons_Hu    = new double[m_unNumberOfReceptors];

    // the slopes at time = t and time = t+step
    m_pfDeltaNeurons_k0  = new double[m_unNumberOfReceptors];
    m_pfDeltaNeurons_k1  = new double[m_unNumberOfReceptors];

    // the activation level at the output neurons used to make decision on FVs (was the total number of effectors and regulators weighted by affinity)
    m_pfOutputNeurons  = new double[m_unNumberOfReceptors];
    m_pfOutputNeurons_prev  = new double[m_unNumberOfReceptors];
    m_pfInputToOutputNeurons  = new double[m_unNumberOfReceptors];

    // in case of RK2, response speed of the hidden neurons of the CTRNN is adaptive (range [0,1]), and approaches 1 as step_h approaches m_pfNeuronTimeConstant
    // in case of euler, response speed fixed
    m_pfNeuronTimeConstant = NN_TIMECONSTANT;


    for (unsigned int i = 0; i < m_unNumberOfReceptors; i++)
    {
        m_pfHiddenNeurons[i]          = 0.0;
        m_pfOutputNeurons[i]          = 0.0;
        m_pfInputToOutputNeurons[i]   = 0.0;

        m_pfAPCs[i]                   = 0.0;

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

CTRNNinRobotAgent::~CTRNNinRobotAgent()
{
    delete m_pfHiddenNeurons;
    delete m_pfHiddenNeurons_prev;

    delete m_pfNeurons_Eu;
    delete m_pfNeurons_Hu;

    delete m_pfDeltaNeurons_k0;
    delete m_pfDeltaNeurons_k1;

    delete m_pfOutputNeurons;
    delete m_pfOutputNeurons_prev;
    delete m_pfInputToOutputNeurons;

    delete m_pfAPCs;

//    delete [] m_pfAffinities[0];
//    delete [] m_pfAffinities;
}


/******************************************************************************/
/******************************************************************************/

void CTRNNinRobotAgent::SimulationStepUpdatePosition()
{
    // Convert the number of feature vectors from robot agents in the vicinity to APCs for the CTRNN
    Sense();

#ifndef DISABLEMODEL_RETAINRNDCALLS // DISABLEMODEL_RETAINRNDCALLS is defined so as to be closer to the same sequence of random numbers generated with the normal working of the CTRNN, so that the same agent behaviors may be obtained when CTRNN is disabled.

    NumericalIntegration(INTEGRATION_TIME_SIMSTART, m_pfHiddenNeurons, m_pfHiddenNeurons_prev, &m_fhlconvergence_error, &m_fhlpercconvergence_error, &m_bhlconvergence_flag, HiddenLayer);

#endif

    m_fWeight = 0.0;
    for(unsigned hnindex=0; hnindex < m_unNumberOfReceptors; hnindex++)
    {
        m_fWeight += m_pfHiddenNeurons[hnindex];
    }
    m_fWeight = m_fWeight * m_fWeight;
    robotAgent->SetWeight(m_fWeight);


    //We set the communication range to be twice that of the FV sensory range
    //CRobotAgent* pcRemoteRobotAgent = robotAgent->GetRandomRobotWithWeights(2.0*robotAgent->GetFVSenseRange());

    // select the robot from one of the 10 nearest neighbours - but in these expts. comm does not seem to be needed
    CRobotAgent* pcRemoteRobotAgent = robotAgent->GetRandomRobotWithWeights((unsigned int)((double)robotAgent->GetSelectedNumNearestNbrs()*1.0));

    if (pcRemoteRobotAgent != NULL)
    {
        CTRNNinRobotAgent* ctrnninRemoteRobotAgent = pcRemoteRobotAgent->GetCTRNNinRobotAgent();

        if (ctrnninRemoteRobotAgent)
        {
            for(unsigned hnindex=0; hnindex < m_unNumberOfReceptors; hnindex++)
            {
                double currActivtoSend = m_pfHiddenNeurons[hnindex]  * m_fTryExchangeProbability;

                double remoteActiv     = ctrnninRemoteRobotAgent->GetHN(hnindex);

                double currActivtoReceive = remoteActiv * m_fTryExchangeProbability;

                ctrnninRemoteRobotAgent->SetHN(hnindex, remoteActiv + currActivtoSend - currActivtoReceive);

                m_pfHiddenNeurons[hnindex] += currActivtoReceive - currActivtoSend;
            }
        }
    }

    UpdateState();
}

/******************************************************************************/
/******************************************************************************/

void CTRNNinRobotAgent::Derivative(double *n, double *deltaN, NNLAYER layer)
{
    // More accurately, the function returns gradient * m_pfNeuronTimeConstant
    // hn'(t + deltaT) = (inputstohn(t) - hn(t)) / m_pfNeuronTimeConstant
    for(unsigned index=0; index < m_unNumberOfReceptors; index++)
    {
        deltaN[index] = 0.0;

        if(layer == HiddenLayer)
        {
            for(unsigned inindex=0; inindex < m_unNumberOfReceptors; inindex++) // from layer below
                deltaN[index] += m_fACTIVATION_FACTOR * GetAf(index, inindex) * m_pfAPCs[inindex];

            for(unsigned index1=0; index1 < m_unNumberOfReceptors; index1++) // from same layer
            {
                if(index != index1)
                    deltaN[index] -=m_fCOMPETITION_FACTOR * GetAf(index,index1) * n[index1];
            }
            deltaN[index] -= n[index];
        }
#if defined(AFFINITYPLASTICBIAS)
        else
        {
            if(m_pfAPCs[index] > 0.0) // the output node at index exists
            {
                deltaN[index] += m_pfInputToOutputNeurons[index];  // from layer below

                for(unsigned index1=0; index1 < m_unNumberOfReceptors; index1++) //from same layer
                {
                    if(index != index1)
                        if(m_pfAPCs[index1] > 0.0)  // if the output node at index1 exists
                            deltaN[index] -=   m_fINHIBTIONSCALINGFACTOR *
                                               (1.0 - GetAf(index,index1)) *
                                               Sigmoid(n[index1] - m_fBIASWEIGHT, m_fSIGMOIDSATURATION);
                }

                deltaN[index] -= n[index];
            }
        }
#endif
    }
}

/******************************************************************************/
/******************************************************************************/

void CTRNNinRobotAgent::PrintCTRNNDetails(unsigned id)
{
    //unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();
    if(!(robotAgent->GetIdentification() == id))
        return;

    // Print the nn
    printf("\n");
    for (unsigned i = 0; i < m_unNumberOfReceptors; i++)
    {
        if(m_pfAPCs[i] > 0.0)
            printf("Sig(on[%d])=%f (%f,%f) [APC:%f] [Status:%d]\n",i, Sigmoid(m_pfOutputNeurons[i], m_fSIGMOIDSATURATION),m_pfOutputNeurons[i],m_pfInputToOutputNeurons[i],m_pfAPCs[i],robotAgent->GetMostWantedList()[i]);
    }
}

/******************************************************************************/
/******************************************************************************/

void CTRNNinRobotAgent::NumericalIntegration(double totalintegration_t, double* neurons, double* neurons_prev, double* convg_error, double* percconvg_error, bool* convg_flag, NNLAYER layer)
{
    (*convg_error) = -1.0;
    (*percconvg_error) = -1.0;
    double integration_t = 0.0;
    bool b_prevdiff0occurance = false;

    step_h = 1.0;

    double step_h = 1.0;
    while(integration_t < totalintegration_t)
    {
        Derivative(neurons, m_pfDeltaNeurons_k0, layer);
        for(unsigned index=0; index < m_unNumberOfReceptors; index++)
        {
            if((layer == OutputLayer && m_pfAPCs[index] > 0.0) || (layer == HiddenLayer))
                m_pfNeurons_Eu[index]  = neurons[index]  +
                                         step_h *
                                         m_pfDeltaNeurons_k0[index]/m_pfNeuronTimeConstant;
            else
                m_pfNeurons_Eu[index]  = 0.0;

            if(layer == HiddenLayer)
                if(m_pfNeurons_Eu[index] < 0.0)
                    m_pfNeurons_Eu[index] = 0.0;
        }


        Derivative(m_pfNeurons_Eu, m_pfDeltaNeurons_k1, layer);
        double absDiffHuenEuler = -1.0;
        for(unsigned index=0; index < m_unNumberOfReceptors; index++)
        {
            if((layer == OutputLayer && m_pfAPCs[index] > 0.0) || (layer == HiddenLayer))
            {
                m_pfNeurons_Hu[index]  = neurons[index]  +
                                           step_h * 0.5 *
                                           (m_pfDeltaNeurons_k0[index] +
                                            m_pfDeltaNeurons_k1[index])/m_pfNeuronTimeConstant;

                if(layer == HiddenLayer)
                    if(m_pfNeurons_Hu[index] < 0.0)
                        m_pfNeurons_Hu[index] = 0.0;


                if(fabs(m_pfNeurons_Hu[index] - m_pfNeurons_Eu[index]) > absDiffHuenEuler)
                {
                    absDiffHuenEuler = fabs(m_pfNeurons_Hu[index] - m_pfNeurons_Eu[index]);
                }
            }
            else
                m_pfNeurons_Hu[index]  = 0.0;
        }


        if(!(absDiffHuenEuler > 0.0))
        {
#ifdef DEBUG
            printf("\n NN layer %d", layer);

            for(unsigned index=0; index < m_unNumberOfReceptors; index++)
                printf("\n N_Hu %e, N_Eu %e", m_pfNeurons_Hu[onindex],
                       m_pfNeurons_Eu[onindex]);

            printf("\n Stepsize %e, m_foutputconvergence_error %e, m_foutputpercconvergence_error %e  \n",
                   step_h,(*convg_error),(*percconvg_error));
#endif

            if((*convg_error) <= NN_CONVERGENCE)
                break;
        }

        if(absDiffHuenEuler == 0.0)
        {
            if(b_prevdiff0occurance && step_h == NN_LOWERLIMIT_STEPSIZE)
            {
#ifdef DEBUG
                printf("\n NN layer %d.", layer);
                printf(" The neurons solution is stalled, or solution at start may already be at steady state\n");
#endif

                break; // exit(-1);
            }
            step_h = step_h / 2.0;

            if(step_h < NN_LOWERLIMIT_STEPSIZE) {
                step_h = NN_LOWERLIMIT_STEPSIZE;}

#ifdef DEBUG
            printf("\nNN layer %d.", layer);
            printf(" New stepsize %f - integration time %e",step_h, integration_t);
#endif

            b_prevdiff0occurance = true;
            continue;
        }

        b_prevdiff0occurance = false;


        assert(absDiffHuenEuler >= 0.0);
        step_h *= sqrt(ERRORALLOWED_NN_STEPSIZE/absDiffHuenEuler);
        if(step_h > NN_UPPERLIMIT_STEPSIZE) {
            step_h = NN_UPPERLIMIT_STEPSIZE;}
        else if(step_h < NN_LOWERLIMIT_STEPSIZE) {
            step_h = NN_LOWERLIMIT_STEPSIZE;}


        (*convg_error) = -1.0;
        for(unsigned index=0; index < m_unNumberOfReceptors; index++)
        {
            if((layer == OutputLayer && m_pfAPCs[index] > 0.0) || (layer == HiddenLayer))
            {
                neurons_prev[index]  = neurons[index];
                neurons[index]       = neurons[index] + step_h *
                                       m_pfDeltaNeurons_k0[index]/m_pfNeuronTimeConstant;

                if((layer==HiddenLayer) && (neurons[index] < HIDDENNEURONACTIVATIONLOWERBOUND))
                    m_pfHiddenNeurons[index] = 0.0;
                else
                {
                    double diffneuronactiv = fabs(neurons[index] -
                                                  neurons_prev[index]);
                    if(diffneuronactiv > (*convg_error))
                    {
                        (*convg_error)      = diffneuronactiv;
                        (*percconvg_error)  = ((*convg_error)/
                                               fabs(neurons_prev[index])) * 100.0;
                    }
                }
            }
            else
            {
                neurons[index] = 0.0; neurons_prev[index] = 0.0;
            }
        }

        //if((*convg_error) <= NN_CONVERGENCE){
        if((*percconvg_error) <= NN_CONVERGENCE){
            (*convg_flag) = true; break;}

        integration_t += step_h;
    }
}

/******************************************************************************/
/******************************************************************************/

void CTRNNinRobotAgent::UpdateState()
{
#ifndef DISABLEMODEL_RETAINRNDCALLS // DISABLEMODEL_RETAINRNDCALLS is defined so as to be closer to the same sequence of random numbers generated with the normal working of the CTRNN, so that the same agent behaviors may be obtained when CTRNN is disabled.


#ifdef FIXEDBIAS
    for(unsigned onindex=0; onindex < m_unNumberOfReceptors; onindex++)
    {
        m_pfOutputNeurons[onindex] = 0.0;

        // if no apc's of specific type, how can i make a decision
        if(m_pfAPCs[onindex] > 0.0)
        {
            for(unsigned hnindex=0; hnindex < m_unNumberOfReceptors; hnindex++)
                m_pfOutputNeurons[onindex] += GetAf(hnindex,onindex) *
                                              m_pfHiddenNeurons[hnindex];
        }
        m_pfOutputNeurons[onindex] *= m_fOUTPUTGAIN_FACTOR;

        if((m_pfOutputNeurons[onindex] <= HIDDENNEURONACTIVATIONLOWERBOUND) || (m_pfAPCs[onindex] == 0.0))
            // Dont know - no Hidden Neuron Activ to make decision
            robotAgent->SetMostWantedList(onindex, 0);
        else if (m_pfOutputNeurons[onindex] < m_fOUTPUTNEURONBIAS)
            // Attack
            robotAgent->SetMostWantedList(onindex, 1);
        else
            // Tolerate
            robotAgent->SetMostWantedList(onindex, 2);
    }
#elif defined AFFINITYPLASTICBIAS

    for(unsigned onindex=0; onindex < m_unNumberOfReceptors; onindex++)
    {
        m_pfInputToOutputNeurons[onindex] = 0.0;

        // if no apc's of specific type, how can i make a decision
        // output layer same as input layer
        if(m_pfAPCs[onindex] > 0.0)
        {
            for(unsigned hnindex=0; hnindex < m_unNumberOfReceptors; hnindex++)
            {
                m_pfInputToOutputNeurons[onindex] += GetAf(hnindex,onindex) *
                                                     m_pfHiddenNeurons[hnindex];
            }
            m_pfInputToOutputNeurons[onindex] *= m_fOUTPUTGAIN_FACTOR;
        }
        m_pfOutputNeurons[onindex] = 0.0; // lets try initialization to 0, first. I.E. no preservation of state of output neurons since the output layer is restructured at each timestep, and consequently new output nodes may suddenly appear with outdated values - that would have to be integrated to a more updated values. To be safe, lets always reset seed value to 0.
    }

    NumericalIntegration(INTEGRATION_TIME_SIMSTART, m_pfOutputNeurons, m_pfOutputNeurons_prev, &m_folconvergence_error, &m_folpercconvergence_error, &m_bolconvergence_flag, OutputLayer);


    for(unsigned onindex=0; onindex < m_unNumberOfReceptors; onindex++)
    {
        if (m_pfAPCs[onindex] == 0.0)
            // Dont know - no Hidden Neuron Activ to make decision
            robotAgent->SetMostWantedList(onindex, 0);
        else if (Sigmoid(m_pfOutputNeurons[onindex], m_fSIGMOIDSATURATION) <= (0.0+m_fEXTERNALBIASWEIGHT))
            // Attack
            robotAgent->SetMostWantedList(onindex, 1);
        else
            // Tolerate
            robotAgent->SetMostWantedList(onindex, 2);
    }
#endif
#endif // DISABLEMODEL_RETAINRNDCALLS
}

/******************************************************************************/
/******************************************************************************/

void CTRNNinRobotAgent::Sense()
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

double CTRNNinRobotAgent::NormalizedAffinity(unsigned int v1, unsigned int v2)
{
    /* XOr operation between the 2 feature vectors */
    unsigned int unXoredString = (v1 ^ v2);

    /* Number of 1's from the result of the previous XOR operation,  at positions preset by mask */
    /* how do we decide whose mask should be used */
    unsigned int unMatching  = CTRNNinRobotAgent::GetNumberOfSetBits(unXoredString);

    //TODO: Have to change affinity computation
    return (double) (CFeatureVector::NUMBER_OF_FEATURES - unMatching) / (double)
            CFeatureVector::NUMBER_OF_FEATURES;
}

/******************************************************************************/
/******************************************************************************/

double CTRNNinRobotAgent::GetAf(unsigned int i, unsigned int j)
{
    return NegExpDistAffinity(i,j,m_fcross_affinity);
}

/******************************************************************************/
/******************************************************************************/

double CTRNNinRobotAgent::NegExpDistAffinity(unsigned int v1, unsigned int v2, double k)
{
    /* k is proportional to the level of cross affinity*/
    /* k=0.01 affinity of 1 when HD is 0, else 0  */
    /* k=inf  affinity of 1 for all HD */

    /* XOr operation between the 2 feature vectors */
    unsigned int unXoredString = (v1 ^ v2);

    /* Number of 1's from the result of the previous XOR operation,  at positions preset by mask */
    /* how do we decide whose mask should be used */
    unsigned int hammingdistance  = CTRNNinRobotAgent::GetNumberOfSetBits(unXoredString);

    //return 1.0 * exp(-(1.0/k) * (double)hammingdistance);
    // Should we normalize the hammingdistance when input to the exp function, or as above?

    return 1.0 * exp(-(1.0/k) * (double)hammingdistance / (double) CFeatureVector::NUMBER_OF_FEATURES);
}

/******************************************************************************/
/******************************************************************************/

double CTRNNinRobotAgent::GetHN(unsigned nindex)
{
    return m_pfHiddenNeurons[nindex];
}

/******************************************************************************/
/******************************************************************************/

void CTRNNinRobotAgent::SetHN(unsigned nindex, double f_currActv)
{
    m_pfHiddenNeurons[nindex] = f_currActv;
}

/******************************************************************************/
/******************************************************************************/

double CTRNNinRobotAgent::GetON(unsigned nindex)
{
    return m_pfOutputNeurons[nindex];
}

/******************************************************************************/
/******************************************************************************/

double CTRNNinRobotAgent::GetInputsToON(unsigned nindex)
{
    return m_pfInputToOutputNeurons[nindex];
}

/******************************************************************************/
/******************************************************************************/

double CTRNNinRobotAgent::GetAPC(unsigned apctype)
{
    return m_pfAPCs[apctype];
}

/******************************************************************************/
/******************************************************************************/

double CTRNNinRobotAgent::GetWeight()
{
    return m_fWeight;
}

/******************************************************************************/
/******************************************************************************/

double CTRNNinRobotAgent::GetFVtoApcScaling()
{
    return m_fFVtoApcscaling;
}

/******************************************************************************/
/******************************************************************************/

bool CTRNNinRobotAgent::GetConvergenceFlag(NNLAYER layer)
{
    if(layer == HiddenLayer)
        return m_bhlconvergence_flag;
    else
        return m_bolconvergence_flag;
}

/******************************************************************************/
/******************************************************************************/

double CTRNNinRobotAgent::GetConvergenceError(NNLAYER layer)
{
    if(layer == HiddenLayer)
        return m_fhlconvergence_error;
    else
        return m_folconvergence_error;
}

/******************************************************************************/
/******************************************************************************/

double CTRNNinRobotAgent::GetConvergenceError_Perc(NNLAYER layer)
{
    if(layer == HiddenLayer)
        return m_fhlpercconvergence_error;
    else
        return m_folpercconvergence_error;
}

/******************************************************************************/
/******************************************************************************/
// a shortcut, the function GetNumberOfSetBits(...) is same for crminrobotagent and ctrnninrobotagent.
//putting it in each class to prevent: multiple definition of `GetNumberOfSetBits(unsigned int)' crminrobotagent.o:/home/danesh/Work/postdocwork/robotsim/bioinstsim2/crminrobotagent.cpp:1295: first defined here
unsigned int CTRNNinRobotAgent::GetNumberOfSetBits(unsigned int x)
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

double CTRNNinRobotAgent::Sigmoid(double f_currActv, double f_Saturation)
{
    double maxcapacity = 1.0;
    double tmp = exp(f_Saturation * -f_currActv);

    return maxcapacity*(1.0 - tmp) / (1.0 + tmp);
}

/******************************************************************************/
/******************************************************************************/

double CTRNNinRobotAgent::GetSigmoidSaturation()
{
    return m_fSIGMOIDSATURATION;
}

/******************************************************************************/
/******************************************************************************/
