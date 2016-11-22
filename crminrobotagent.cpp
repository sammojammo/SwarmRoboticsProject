#include "crminrobotagent.h"

/******************************************************************************/
/******************************************************************************/
// rv. 708 - bioinstsim2 with unoptimized CRM
/******************************************************************************/
/******************************************************************************/

#define CELLLOWERBOUND 1e-3 //todo: set as percentage instead of absolute value
// note: could result in euler-huen diff at 0, for high error thresholds. In that case, lower this value

#define CONJUGATION_OVERFLOW_LIMIT 1.0e-15  //todo: set as percentage instead of absolute value

/******************************************************************************/
/******************************************************************************/

#define TCELL_UPPERLIMIT_STEPSIZE 500000.0 //todo: could be set as a propotion of the INTEGRATION_TIME
#define TCELL_LOWERLIMIT_STEPSIZE 1.0e-6

#define CONJ_UPPERLIMIT_STEPSIZE 10.0
#define CONJ_LOWERLIMIT_STEPSIZE 1.0e-6

#define ERRORALLOWED_TCELL_STEPSIZE 1.0e-2 //todo: set as percentage instead of absolute value
#define ERRORALLOWED_CONJ_STEPSIZE  1.0e-3 //todo: set as percentage instead of absolute value; else will introduce problems when m_fFVtoApcscaling is reduced, and dealing with density of conjugates in order of 1e-6


#define INTEGRATION_TIME  5.0e+7 // was 1.5e+7 on elephant01a  earlier 1.0e+7
#define FAILSAFE_CONJ_INTEGRATION_TIME  5.0e+5 // a failsafe to prevent endless integrations of a stiff system.
//TODO: Could instead use the differences in the error values (between time-steps), being same over a period of time as a requirement to reduce time-step
#define REDUCESTEPSIZE_CONJ_INTEGRATION_TIME 1.0e+5 // lowers the step size when the conjugation integration has passed this limit, and the error allowed is high (>1e-3).



#define TCELL_CONVERGENCE  1.0e-2 //todo: set as percentage instead of absolute value. Already using the percentage values to break out of integration loop
#define CONJ_CONVERGENCE   1.0e-3 //todo: set as percentage instead of absolute value


/******************************************************************************/
/******************************************************************************/

CRMinRobotAgent::CRMinRobotAgent(CRobotAgent* ptr_robotAgent, CArguments* m_crmArguments)
{
    robotAgent = ptr_robotAgent;

    m_fWeight         = 1.0;
    static bool bHelpDisplayed = false;

    CFeatureVector::NUMBER_OF_FEATURES = m_crmArguments->GetArgumentAsIntOr("numberoffeatures", 4);
    currE = m_crmArguments->GetArgumentAsDoubleOr("currE", 10.0);  // : Density of effector cells
    currR = m_crmArguments->GetArgumentAsDoubleOr("currR", 10.0);  // : Density of regulatory cells
    kon   = m_crmArguments->GetArgumentAsDoubleOr("kon", .1);   // : Conjugation rate
    koff  = m_crmArguments->GetArgumentAsDoubleOr("koff", .1);  // : Dissociation rate
    kpe   = m_crmArguments->GetArgumentAsDoubleOr("kpe", 1e-3);   // : Proliferation rate for effector cells
    kde   = m_crmArguments->GetArgumentAsDoubleOr("kde", 1e-6);   // : Death rate for effector cells
    kpr   = m_crmArguments->GetArgumentAsDoubleOr("kpr", 0.5e-3);//0.6e-2   // : Proliferation rate for regulatory cells
    kdr   = m_crmArguments->GetArgumentAsDoubleOr("kdr", 1e-6);   // : Death rate for regulatory cells

    m_fTryExchangeProbability = m_crmArguments->GetArgumentAsDoubleOr("exchangeprob", 0.0);
    // is now set based on characteristics of robot
    //m_fExchangeRange          = m_crmArguments->GetArgumentAsDoubleOr("exchangerange", 2.0);

    se                        = m_crmArguments->GetArgumentAsDoubleOr("sourcerateE", 0.0); //1.1e-3  // Source rate of E cell generation
    sr                        = m_crmArguments->GetArgumentAsDoubleOr("sourcerateR", 0.0); //0.6e-3 // Source rate of R cell generation

    m_fcross_affinity         = m_crmArguments->GetArgumentAsDoubleOr("cross-affinity", 0.4);

    m_fFVtoApcscaling         = m_crmArguments->GetArgumentAsDoubleOr("fvapcscaling", 1.0e-3);


    if(FDMODELTYPE == CRM_TCELLSINEXCESS)
        assert(kon == koff);

    if (m_crmArguments->GetArgumentIsDefined("help") && !bHelpDisplayed)
    {
        printf("numberoffeatures=#             Number of features in a single FV [%d]\n"
               "currE=#.#                     Density of effector cells [%f]\n"
               "currR=#.#                     Density of regulatory cells [%f]\n"
               "kon=#.#                       Conjugation rate [%f]\n"
               "koff=#.#                      Dissociation rate [%f]\n"
               "kpe=#.#                       Proliferation rate for effector cells [%f]\n"
               "kde=#.#                       Death rate for effector cells [%f]\n"
               "kpr=#.#                       Proliferation rate for regulatory cells [%f]\n"
               "kdr=#.#                       Death rate for regulatory cells [%f]\n"
               "exchangeprob=#.#              Probability of trying to exchange cells with other robots [%f]\n"
               "Source_E=#.#                  Source rate of E cell generation [%f]\n"
               "Source_R=#.#                  Source rate of R cell generation [%f]\n"
               "cross-affinity=#.#            Level of cross-affinity (>0)     [%2.5f]\n"
               "fvapcscaling=#.#              Scaling parameter of [FV] to [APC] [%e]\n",
               CFeatureVector::NUMBER_OF_FEATURES,
               currE,
               currR,
               kon,
               koff,
               kpe,
               kde,
               kpr,
               kdr,
               m_fTryExchangeProbability,
               se,
               sr,
               m_fcross_affinity,
               m_fFVtoApcscaling);
        bHelpDisplayed = true;
    }



    m_unNumberOfReceptors = 1 << (CFeatureVector::NUMBER_OF_FEATURES);

    m_pfEffectors       = new double[m_unNumberOfReceptors];
    m_pfRegulators      = new double[m_unNumberOfReceptors];

    m_pfEffectors_prev  = new double[m_unNumberOfReceptors];
    m_pfRegulators_prev = new double[m_unNumberOfReceptors];

    // predicted number of cells at time t+step with Euler method
    m_pfEffectors_Eu    = new double[m_unNumberOfReceptors];
    m_pfRegulators_Eu   = new double[m_unNumberOfReceptors];
    // predicted number of cells at time t+step with Huen method
    m_pfEffectors_Hu    = new double[m_unNumberOfReceptors];
    m_pfRegulators_Hu   = new double[m_unNumberOfReceptors];

    // the slopes at time = t and time = t+step
    m_pfDeltaEffectors_k0  = new double[m_unNumberOfReceptors];
    m_pfDeltaRegulators_k0 = new double[m_unNumberOfReceptors];
    m_pfDeltaEffectors_k1  = new double[m_unNumberOfReceptors];
    m_pfDeltaRegulators_k1 = new double[m_unNumberOfReceptors];

    // the total number of effectors and regulators weighted by affinity
    // used to make decision on FVs
    m_pfSumEffectorsWeightedbyAffinity  = new double[m_unNumberOfReceptors];
    m_pfSumRegulatorsWeightedbyAffinity = new double[m_unNumberOfReceptors];


    m_pfAPCs		      = new double[m_unNumberOfReceptors]; // In this implementation, each type of APC presents one FV.


    m_pfEffectorConjugatesPerAPC  = new double[m_unNumberOfReceptors];
    m_pfRegulatorConjugatesPerAPC = new double[m_unNumberOfReceptors];

    m_pfConjugates = new double* [m_unNumberOfReceptors];
    m_pfConjugates[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfConjugates[i] = m_pfConjugates[i-1] + m_unNumberOfReceptors;
    }

    m_pfConjugates_tmp = new double* [m_unNumberOfReceptors];
    m_pfConjugates_tmp[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfConjugates_tmp[i] = m_pfConjugates_tmp[i-1] + m_unNumberOfReceptors;
    }

    m_pfConjugates_Eu = new double* [m_unNumberOfReceptors];
    m_pfConjugates_Eu[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfConjugates_Eu[i] = m_pfConjugates_Eu[i-1] + m_unNumberOfReceptors;
    }

    m_pfAffinities = new double* [m_unNumberOfReceptors];
    m_pfAffinities[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfAffinities[i] = m_pfAffinities[i-1] + m_unNumberOfReceptors;
    }


    m_pfEffectorConjugates = new double* [m_unNumberOfReceptors];
    m_pfEffectorConjugates[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfEffectorConjugates[i] = m_pfEffectorConjugates[i-1] + m_unNumberOfReceptors;
    }


    m_pfRegulatorConjugates = new double* [m_unNumberOfReceptors];
    m_pfRegulatorConjugates[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfRegulatorConjugates[i] = m_pfRegulatorConjugates[i-1] + m_unNumberOfReceptors;
    }



    m_pfDeltaConjugates_k0    = new double* [m_unNumberOfReceptors];
    m_pfDeltaConjugates_k0[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfDeltaConjugates_k0[i] = m_pfDeltaConjugates_k0[i-1] + m_unNumberOfReceptors;
    }

    m_pfDeltaConjugates_k1 = new double* [m_unNumberOfReceptors];
    m_pfDeltaConjugates_k1[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfDeltaConjugates_k1[i] = m_pfDeltaConjugates_k1[i-1] + m_unNumberOfReceptors;
    }

    m_pfConj_tmp_Eu = new double* [m_unNumberOfReceptors];
    m_pfConj_tmp_Eu[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfConj_tmp_Eu[i] = m_pfConj_tmp_Eu[i-1] + m_unNumberOfReceptors;
    }

    m_pfConj_tmp_Hu = new double* [m_unNumberOfReceptors];
    m_pfConj_tmp_Hu[0] = new double [m_unNumberOfReceptors * m_unNumberOfReceptors];
    for (unsigned int i = 1; i < m_unNumberOfReceptors; ++i)
    {
        m_pfConj_tmp_Hu[i] = m_pfConj_tmp_Hu[i-1] + m_unNumberOfReceptors;
    }




    for (unsigned int i = 0; i < m_unNumberOfReceptors; i++)
    {
#ifdef TCELLCLONEEXCHANGEANALYSIS
        if(this->robotAgent->GetIdentification() <= 9) {
        m_pfEffectors[i]       = currE;
        m_pfRegulators[i]      = currR;}
        else {
            m_pfEffectors[i]       = currE*10.0;
            m_pfRegulators[i]      = 0.0;}
#else
        m_pfEffectors[i]       = currE;
        m_pfRegulators[i]      = currR;
#endif

        m_pfAPCs[i]            = 0.0;
        m_pfEffectorConjugatesPerAPC[i]  = 0.0;
        m_pfRegulatorConjugatesPerAPC[i] = 0.0;

        m_pfSumEffectorsWeightedbyAffinity[i]  = 0.0;
        m_pfSumRegulatorsWeightedbyAffinity[i] = 0.0;

        for (unsigned int j = 0; j < m_unNumberOfReceptors; j++)
        {
            m_pfAffinities[i][j]              = NegExpDistAffinity(i,j,m_fcross_affinity);
            //printf("Af(%d,%d)=%f\n",i,j,m_pfAffinities[i][j]);

            m_pfConjugates[i][j]              = 0.0;
            m_pfConjugates_tmp[i][j]          = 0.0;
            m_pfConjugates_Eu[i][j]           = 0.0;

            m_pfEffectorConjugates[i][j]      = 0.0;
            m_pfRegulatorConjugates[i][j]     = 0.0;
        }
    }

    sites = 3U;
    step_h = 1.0;

    assert(sites == 3U);
    assert(m_fcross_affinity > 0.0);
}

/******************************************************************************/
/******************************************************************************/

CRMinRobotAgent::~CRMinRobotAgent()
{
    delete m_pfEffectors;
    delete m_pfRegulators;
    delete m_pfEffectors_prev;
    delete m_pfRegulators_prev;



    delete m_pfAPCs;
    delete m_pfEffectorConjugatesPerAPC;
    delete m_pfRegulatorConjugatesPerAPC;

    delete m_pfEffectors_Eu;
    delete m_pfRegulators_Eu;
    delete m_pfEffectors_Hu;
    delete m_pfRegulators_Hu;

    delete m_pfDeltaEffectors_k0;
    delete m_pfDeltaRegulators_k0;
    delete m_pfDeltaEffectors_k1;
    delete m_pfDeltaRegulators_k1;

    delete m_pfSumEffectorsWeightedbyAffinity;
    delete m_pfSumRegulatorsWeightedbyAffinity;


    delete [] m_pfDeltaConjugates_k0[0];
    delete [] m_pfDeltaConjugates_k0;

    delete [] m_pfDeltaConjugates_k1[0];
    delete [] m_pfDeltaConjugates_k1;

    delete [] m_pfConj_tmp_Eu[0];
    delete [] m_pfConj_tmp_Eu;

    delete [] m_pfConj_tmp_Hu[0];
    delete [] m_pfConj_tmp_Hu;


    delete [] m_pfConjugates[0];
    delete [] m_pfConjugates;

    delete [] m_pfConjugates_tmp[0];
    delete [] m_pfConjugates_tmp;

    delete [] m_pfConjugates_Eu[0];
    delete [] m_pfConjugates_Eu;

    delete [] m_pfEffectorConjugates[0];
    delete [] m_pfEffectorConjugates;

    delete [] m_pfRegulatorConjugates[0];
    delete [] m_pfRegulatorConjugates;

    delete [] m_pfAffinities[0];
    delete [] m_pfAffinities;
}


/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::SimulationStepUpdatePosition()
{
    // Convert the number of feature vectors from robot agents in the vicinity to APCs for the CRM
    Sense();

#ifndef DISABLEMODEL_RETAINRNDCALLS // DISABLEMODEL_RETAINRNDCALLS is defined so as to be closer to the same sequence of random numbers generated with the normal working of the CRM, so that the same agent behaviors may be obtained when CRM is disabled.


    int selectedclonaltype;// = Random::nextInt(m_unNumberOfReceptors);

    for(selectedclonaltype=0;selectedclonaltype<m_unNumberOfReceptors;selectedclonaltype++)
   {
        if(m_pfAPCs[selectedclonaltype] > 0.0)
        {
            m_pfEffectors[selectedclonaltype]  += 10.0;
            m_pfRegulators[selectedclonaltype] += 10.0;
        }
   }

    // --- Numerical integration to compute m_pfEffectors[] and m_pfRegulators[] to reflect m_pfAPCs[]
    m_bConvergenceFlag = false;
    m_dconvergence_error = 10.0;
    double convergence_errormax = -1.0, perc_convergence_errormax;
    double integration_t = 0.0;
    double step_h = 1.0;
    bool b_prevdiff0occurance = false;
    while(integration_t < INTEGRATION_TIME)
    {

        // Compute number of conjugates for T cells m_pfEffectors[..] + m_pfRegulators[..];
        // Stored in m_pfConjugates[i][j], the conjugates of Ti to APCj
        ConjugatesQSS(m_pfEffectors, m_pfRegulators, m_pfConjugates);
        Derivative(m_pfEffectors, m_pfRegulators, m_pfConjugates, m_pfDeltaEffectors_k0, m_pfDeltaRegulators_k0);

        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            m_pfEffectors_Eu[thtype]  = m_pfEffectors[thtype]  +
                                        step_h * m_pfDeltaEffectors_k0[thtype];
            m_pfRegulators_Eu[thtype] = m_pfRegulators[thtype] +
                                        step_h * m_pfDeltaRegulators_k0[thtype];

            if(m_pfEffectors_Eu[thtype] < 0.0) {
                m_pfEffectors_Eu[thtype] = 0.0;}

            if(m_pfRegulators_Eu[thtype] < 0.0) {
                m_pfRegulators_Eu[thtype] = 0.0;}
        }

        ConjugatesQSS(m_pfEffectors_Eu, m_pfRegulators_Eu, m_pfConjugates_Eu);
        Derivative(m_pfEffectors_Eu, m_pfRegulators_Eu, m_pfConjugates_Eu, m_pfDeltaEffectors_k1, m_pfDeltaRegulators_k1);

        double absDiffHuenEuler = -1.0;
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            m_pfEffectors_Hu[thtype]  = m_pfEffectors[thtype]  +
                                        0.5 * step_h *
                                        (m_pfDeltaEffectors_k0[thtype] + m_pfDeltaEffectors_k1[thtype]);

            if(m_pfEffectors_Hu[thtype] < 0.0) {
                m_pfEffectors_Hu[thtype] = 0.0;}


            if(fabs(m_pfEffectors_Hu[thtype] - m_pfEffectors_Eu[thtype]) > absDiffHuenEuler)
            {
                absDiffHuenEuler = fabs(m_pfEffectors_Hu[thtype] - m_pfEffectors_Eu[thtype]);
            }


            m_pfRegulators_Hu[thtype] = m_pfRegulators[thtype] +
                                        0.5 * step_h *
                                        (m_pfDeltaRegulators_k0[thtype] +
                                         m_pfDeltaRegulators_k1[thtype]);
            if(m_pfRegulators_Hu[thtype] < 0.0) {
                m_pfRegulators_Hu[thtype] = 0.0;}


            if(fabs(m_pfRegulators_Hu[thtype] - m_pfRegulators_Eu[thtype]) > absDiffHuenEuler)
            {
                absDiffHuenEuler = fabs(m_pfRegulators_Hu[thtype] - m_pfRegulators_Eu[thtype]);
            }
        }


        if(!(absDiffHuenEuler > 0.0))
        {
            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
            {
                printf("\n E_Hu %f, E_Eu %f        R_Hu %f, R_Eu %f",
                       m_pfEffectors_Hu[thtype],m_pfEffectors_Eu[thtype],
                       m_pfRegulators_Hu[thtype],m_pfRegulators_Eu[thtype]);
            }

            printf("\n Stepsize %e",step_h);
        }

        if(absDiffHuenEuler == 0.0)
        {
            if(b_prevdiff0occurance && step_h == TCELL_LOWERLIMIT_STEPSIZE)
            {
                printf("\n The T-cell population solution is stalled");
                exit(-1);
            }
            step_h = step_h / 2.0;

            if(step_h < TCELL_LOWERLIMIT_STEPSIZE) {
                        step_h = TCELL_LOWERLIMIT_STEPSIZE;}

            printf("\n New stepsize %f - integration time %e",step_h,integration_t);

            b_prevdiff0occurance = true;

            continue;
        }

        b_prevdiff0occurance = false;


        assert(absDiffHuenEuler >= 0.0);
        step_h *= sqrt(ERRORALLOWED_TCELL_STEPSIZE/absDiffHuenEuler);
        if(step_h > TCELL_UPPERLIMIT_STEPSIZE) {
            step_h = TCELL_UPPERLIMIT_STEPSIZE;}
        else if(step_h < TCELL_LOWERLIMIT_STEPSIZE) {
            step_h = TCELL_LOWERLIMIT_STEPSIZE;}


        convergence_errormax = -1.0;
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            m_pfEffectors_prev[thtype]  = m_pfEffectors[thtype];
            m_pfRegulators_prev[thtype] = m_pfRegulators[thtype];

            m_pfEffectors[thtype]  = m_pfEffectors[thtype]  + step_h * m_pfDeltaEffectors_k0[thtype];
            m_pfRegulators[thtype] = m_pfRegulators[thtype] + step_h * m_pfDeltaRegulators_k0[thtype];


            if(m_pfEffectors[thtype] < 0.0){
                m_pfEffectors[thtype] = 0.0;}
            else
            {
                if(fabs(step_h * m_pfDeltaEffectors_k0[thtype]) > convergence_errormax)
                {
                    convergence_errormax      = fabs(step_h * m_pfDeltaEffectors_k0[thtype]);
                    perc_convergence_errormax = (convergence_errormax / m_pfEffectors_prev[thtype]) *
                                                100.0;
                }
            }


            if(m_pfRegulators[thtype] < 0.0){
                m_pfRegulators[thtype] = 0.0;}
            else
            {
                if(fabs(step_h * m_pfDeltaRegulators_k0[thtype]) > convergence_errormax)
                {
                    convergence_errormax      = fabs(step_h * m_pfDeltaRegulators_k0[thtype]);
                    perc_convergence_errormax = (convergence_errormax / m_pfRegulators_prev[thtype]) *
                                                100.0;
                }
            }
        }

        m_dconvergence_error     = convergence_errormax;
        m_dpercconvergence_error = perc_convergence_errormax;

        if(m_dpercconvergence_error <= 0.001)
        {
            m_bConvergenceFlag = true;
            break;
        }

        integration_t += step_h;
    }

#endif

    m_fWeight = 0.0;
    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    {
        m_fWeight += m_pfEffectors[thtype] + m_pfRegulators[thtype];
    }
    m_fWeight = m_fWeight * m_fWeight;
    robotAgent->SetWeight(m_fWeight);


    //We set the communication range to be twice that of the FV sensory range
    //CRobotAgent* pcRemoteRobotAgent = robotAgent->GetRandomRobotWithWeights(2.0*robotAgent->GetFVSenseRange());

    // select the robot from one of the 10 nearest neighbours - but in these expts. comm does not seem to be needed
    CRobotAgent* pcRemoteRobotAgent = robotAgent->GetRandomRobotWithWeights((unsigned int)((double)robotAgent->GetSelectedNumNearestNbrs()*1.0));


    if (pcRemoteRobotAgent != NULL)
    {
        CRMinRobotAgent* crminRemoteRobotAgent = pcRemoteRobotAgent->GetCRMinRobotAgent();

        if (crminRemoteRobotAgent)
        {
            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
            {
                double currEtoSend = m_pfEffectors[thtype]  * m_fTryExchangeProbability;
                double currRtoSend = m_pfRegulators[thtype] * m_fTryExchangeProbability;

                double remoteCurrE = crminRemoteRobotAgent->GetCurrE(thtype);
                double remoteCurrR = crminRemoteRobotAgent->GetCurrR(thtype);

                double currEtoReceive = remoteCurrE * m_fTryExchangeProbability;
                double currRtoReceive = remoteCurrR * m_fTryExchangeProbability;

                crminRemoteRobotAgent->SetCurrR(thtype, remoteCurrR + currRtoSend - currRtoReceive);
                crminRemoteRobotAgent->SetCurrE(thtype, remoteCurrE + currEtoSend - currEtoReceive);

                m_pfRegulators[thtype] += currRtoReceive - currRtoSend;
                m_pfEffectors[thtype]  += currEtoReceive - currEtoSend;

            }
        }

        printf("\nCommTime: %d, RobotId1: %d, RobotId2: %d\n",
               CSimulator::GetInstance()->GetSimulationStepNumber(),
               this->robotAgent->GetIdentification(), pcRemoteRobotAgent->GetIdentification());
    }
    else
        printf("\nCommTime: %d, RobotId1: %d, RobotId2: %d\n",
               CSimulator::GetInstance()->GetSimulationStepNumber(),
               this->robotAgent->GetIdentification(), -1);


    if(this->robotAgent->GetIdentification()==1 || this->robotAgent->GetIdentification()==15) {
    printf("\nAllTcellClonesTime: %d, RobotId: %d, ",
           CSimulator::GetInstance()->GetSimulationStepNumber(),
           this->robotAgent->GetIdentification());
    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    {
        //if(!(m_pfEffectors[thtype] + m_pfRegulators[thtype] <= CELLLOWERBOUND))
            printf("Clone: %d, A: %f, E: %f, R: %f ", thtype, m_pfAPCs[thtype], m_pfEffectors[thtype], m_pfRegulators[thtype]);
    }
    printf("\n");}

    UpdateState();
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::ConjugatesQSS(double *E, double *R, double **C)
{
    if(FDMODELTYPE == CRM_TCELLSINEXCESS)
    {
        for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
        {
            if(m_pfAPCs[apctype])
            {
                double tcellsweightedaffinity = 0.0;
                for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
                    tcellsweightedaffinity += m_pfAffinities[thtype][apctype] * (E[thtype] + R[thtype]);

                // storing all the conjuagtes for APC of subpopulation apctype at the position of the first T-cell clonaltype. will factorize it into effector and regulatory conjuagtes at the Derivative function
                C[0][apctype] = (m_pfAPCs[apctype]*((double)sites)*tcellsweightedaffinity) /
                                (tcellsweightedaffinity + 1.0);
            }
            else
                C[0][apctype] = 0.0;

        }
        return;
    }

    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    {
        for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
        {
            // since we are now deleting pathogens and the rate of deletion mayb be faster than the conjugate unbindingrate, we always start with 0 conjugates to compute QSS values
            C[thtype][apctype] = 0.0;
        }
    }

    
    conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;
    double error = 1.0;
    double conjintegration_t = 0.0;
    unsigned n_iteration = 0;
    bool b_prevdiff0occurance=false;

    //while(((error - CONJ_CONVERGENCE) > 0.00001))
    while(error > CONJ_CONVERGENCE)
    {
        n_iteration++;

        // a failsafe to prevent endless integrations, because of oscillations in the density of conjugates.If the value of oscillating conjugates is above the error threshold, we will break out of this loop after 100 iterations
        //if (n_iteration > 100U)
        //    break;
        //        if (n_iteration >= UINT_MAX-1)
        //        {
        //            // a failsafe to prevent endless integrations, because of oscillations in the density of conjugates.If the value of oscillating conjugates is above the error threshold, we will break out of this loop after 100 iterations

        //            printf("\n The numerical integration of conjugates has undergone %d iterations. And yet the error is %f (CONJ_CONVERGENCE=%f)\n",UINT_MAX-1,error,CONJ_CONVERGENCE);
        //            exit(-1);
        //        }
        if(conjintegration_t > FAILSAFE_CONJ_INTEGRATION_TIME)
        {
            printf("\nAttention. The numerical integration of conjugates has undergone %d iterations. And yet the error is %f (CONJ_CONVERGENCE=%f). Breaking off now\n",n_iteration,error,CONJ_CONVERGENCE);
            break;
        }



        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    if(m_pfAPCs[apctype] > 0.0)
                    {
                        m_pfDeltaConjugates_k0[thtype][apctype] =
                                ((kon * m_pfAffinities[thtype][apctype] *
                                  FreeThCells(E, R, C, thtype) *
                                  AvailableBindingSites(C, apctype)) -
                                 koff*C[thtype][apctype]);
                    }
                    else
                        m_pfDeltaConjugates_k0[thtype][apctype] = 0.0;
                }
            }
            else
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    m_pfDeltaConjugates_k0[thtype][apctype] = 0.0;
                }
            }
        }


        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    if(m_pfAPCs[apctype] > 0.0)
                    {
                        m_pfConj_tmp_Eu[thtype][apctype] = C[thtype][apctype] + conjstep_h *
                                                           m_pfDeltaConjugates_k0[thtype][apctype];

                        if(m_pfConj_tmp_Eu[thtype][apctype] < 0.0)
                            m_pfConj_tmp_Eu[thtype][apctype] = 0.0;
                    }
                    else
                        m_pfConj_tmp_Eu[thtype][apctype] = 0.0;
                }
            }
            else
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    m_pfConj_tmp_Eu[thtype][apctype] = 0.0;
                }
            }
        }


        //-----Scaling down conjugates, that may have overflowed because of nuemercal errors in integration. Particularly relevant at relatively high error thresholds of 1e-3
        ScaleDownConjugates(m_pfConj_tmp_Eu);


        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    if(m_pfAPCs[apctype] > 0.0)
                    {
                        m_pfDeltaConjugates_k1[thtype][apctype] =
                                ((kon * m_pfAffinities[thtype][apctype] *
                                  FreeThCells(E, R, m_pfConj_tmp_Eu, thtype) *
                                  AvailableBindingSites(m_pfConj_tmp_Eu, apctype)) -
                                 koff*m_pfConj_tmp_Eu[thtype][apctype]);
                    }
                    else
                        m_pfDeltaConjugates_k1[thtype][apctype] = 0.0;
                }
            }
            else
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    m_pfDeltaConjugates_k1[thtype][apctype] = 0.0;
                }
            }
        }


        double absDiffHuenEuler = -1.0;
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            if(!(E[thtype] + R[thtype] <= CELLLOWERBOUND))
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    if(m_pfAPCs[apctype] > 0.0)
                    {
                        m_pfConj_tmp_Hu[thtype][apctype] = C[thtype][apctype] + 0.5 * conjstep_h * (m_pfDeltaConjugates_k0[thtype][apctype] + m_pfDeltaConjugates_k1[thtype][apctype]);

                        if(m_pfConj_tmp_Hu[thtype][apctype] < 0.0)
                            m_pfConj_tmp_Hu[thtype][apctype] = 0.0;


                        if(fabs(m_pfConj_tmp_Hu[thtype][apctype] - m_pfConj_tmp_Eu[thtype][apctype]) > absDiffHuenEuler)
                        {
                            absDiffHuenEuler = fabs(m_pfConj_tmp_Hu[thtype][apctype] - m_pfConj_tmp_Eu[thtype][apctype]);
                        }
                    }
                    else
                        m_pfConj_tmp_Hu[thtype][apctype] = 0.0;
                }
            }
            else
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    m_pfConj_tmp_Hu[thtype][apctype] = 0.0;
                }
            }
        }


        //-----Scaling down conjugates, that may have overflowed because of nuemercal errors in integration. Particularly relevant at relatively high error thresholds of 1e-3
        ScaleDownConjugates(m_pfConj_tmp_Hu);


        if(absDiffHuenEuler == 0.0)
        {
            if(b_prevdiff0occurance && conjstep_h == CONJ_LOWERLIMIT_STEPSIZE)
            {
                printf("\n The T-cell population solution is stalled");
                exit(-1);
            }

            conjstep_h = conjstep_h / 2.0;

            if(conjstep_h < CONJ_LOWERLIMIT_STEPSIZE) {
                        conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;}

            printf("\n new conjugation step %f, n_iteration %u",conjstep_h,n_iteration);

            b_prevdiff0occurance = true;

            continue;
        }

        b_prevdiff0occurance = false;

        assert(absDiffHuenEuler >= 0.0);


        if(conjintegration_t > REDUCESTEPSIZE_CONJ_INTEGRATION_TIME && ERRORALLOWED_CONJ_STEPSIZE >= 1.0e-3)
        {
            /*The system is most likely stiff and oscillating around the "true" value, as the slope approaches 0*/
            /*we reduce the step size to reduce the difference between the oscillating values*/

            /*If the error allowed was lower, the corresponding step sizes would alreadz be lower,
            and so would be the differences between oscillating values*/
            conjstep_h /= 2.0;
            if(conjstep_h < CONJ_LOWERLIMIT_STEPSIZE) {
                            conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;}
        }
        else
        {
            conjstep_h *= sqrt(ERRORALLOWED_CONJ_STEPSIZE/absDiffHuenEuler);
            if(conjstep_h > CONJ_UPPERLIMIT_STEPSIZE) {
                conjstep_h = CONJ_UPPERLIMIT_STEPSIZE;}
            else if(conjstep_h < CONJ_LOWERLIMIT_STEPSIZE) {
                conjstep_h = CONJ_LOWERLIMIT_STEPSIZE;}
        }



        double error_max = -1.0;
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            if(E[thtype] + R[thtype] <= CELLLOWERBOUND)
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    C[thtype][apctype] = 0.0;
                }
            }
            else
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    if(m_pfAPCs[apctype] > 0.0)
                    {
                        C[thtype][apctype] = C[thtype][apctype] + conjstep_h * m_pfDeltaConjugates_k0[thtype][apctype];
                        if(C[thtype][apctype] < 0.0)
                        {
                            C[thtype][apctype] = 0.0;
                        }
                        else
                        {
                            if(fabs(conjstep_h * m_pfDeltaConjugates_k0[thtype][apctype]) > error_max)
                            {
                                error_max = fabs(conjstep_h * m_pfDeltaConjugates_k0[thtype][apctype]);
                            }
                        }
                    }
                    else
                        C[thtype][apctype] = 0.0;
                }
            }
        }

        //-----Scaling down conjugates, that may have overflowed because of nuemercal errors in integration. Particularly relevant at relatively high error thresholds of 1e-3
        ScaleDownConjugates(C);


        conjintegration_t = conjintegration_t + conjstep_h;
        error = error_max;
    }



    //Iterative procedure to compute the number of conjugates for current number of T cells
    //TODO eulers or repair the fast iterative process - need to clarify points with Jorge, use Eulers method for now
    //   double error = 1.0;
    //   while(error >= 0.01)
    //   {
    //       for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    //       {
    // 	  for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
    // 	  {
    // 	      m_pfConjugates_tmp[thtype][apctype] =
    // 		  C[thtype][apctype] +
    // 		  ((kon * m_pfAffinities[thtype][apctype] *
    // 		  FreeThCells(E, R, C, thtype) * AvailableBindingSites(C, apctype)) -
    // 		  koff*C[thtype][apctype]) * conjstep_h;
    // 	  }
    //       }
    //
    //       double error_max = -1.0;
    //       for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    //       {
    // 	  for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
    // 	  {
    // 	      if (fabs(C[thtype][apctype] - m_pfConjugates_tmp[thtype][apctype]) > error_max)
    // 	      {
    // 		  error_max = fabs(C[thtype][apctype] - m_pfConjugates_tmp[thtype][apctype]);
    // 	      }
    //
    // 	      C[thtype][apctype] = m_pfConjugates_tmp[thtype][apctype];
    // 	  }
    //       }
    //
    //       error = error_max;
    //TODO Add a fail safe counter to exit from this while loop
    //   }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::Derivative(double *E, double *R, double **C, double *deltaE, double *deltaR)
{
    if(FDMODELTYPE == CRM)
    {
        // Dividing the conjugates into Effector and Regulator type
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            if((E[thtype] + R[thtype]) <= CELLLOWERBOUND)
            {
                //TODO: check I think we are doing this initialization to 0, twice. done before in computation of conjugates (ConjugateQSS(...))
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    m_pfEffectorConjugates[thtype][apctype]  =  0.0;
                    m_pfRegulatorConjugates[thtype][apctype] =  0.0;
                }
            }
            else
            {
                for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
                {
                    m_pfEffectorConjugates[thtype][apctype] =
                                (E[thtype]/(E[thtype] + R[thtype])) * C[thtype][apctype];

                    m_pfRegulatorConjugates[thtype][apctype] =
                                (R[thtype]/(E[thtype] + R[thtype])) * C[thtype][apctype];
                }
            }
        }

        // Computing the total number of effector and regulator conjugates per APC
        for(unsigned apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
        {
            m_pfEffectorConjugatesPerAPC[apctype]  = 0.0;
            m_pfRegulatorConjugatesPerAPC[apctype] = 0.0;

            for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
            {
                m_pfEffectorConjugatesPerAPC[apctype]  += m_pfEffectorConjugates[thtype][apctype];
                m_pfRegulatorConjugatesPerAPC[apctype] += m_pfRegulatorConjugates[thtype][apctype];
            }

            assert((m_pfEffectorConjugatesPerAPC[apctype]+m_pfRegulatorConjugatesPerAPC[apctype]) -
                   m_pfAPCs[apctype]*((double)sites) <= CONJUGATION_OVERFLOW_LIMIT);
        }
    }
    else
    {
        //CRM_TCELLSINEXCESS
        for(unsigned apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
        {
            if(m_pfAPCs[apctype] == 0.0)
            {
                m_pfEffectorConjugatesPerAPC[apctype] = 0.0;
                m_pfRegulatorConjugatesPerAPC[apctype] = 0.0;

                for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
                {
                    m_pfEffectorConjugates[thtype][apctype]  = 0.0;
                    m_pfRegulatorConjugates[thtype][apctype] = 0.0;
                }
            }
            else
            {
                double tcellsweightedaffinity = 0.0, ecellsweightedaffinity = 0.0,
                rcellsweightedaffinity = 0.0;

                for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
                {
                    tcellsweightedaffinity += m_pfAffinities[thtype][apctype] * (E[thtype] + R[thtype]);
                    ecellsweightedaffinity += m_pfAffinities[thtype][apctype] * E[thtype];
                    rcellsweightedaffinity += m_pfAffinities[thtype][apctype] * R[thtype];
                }

            // using the conjuagtes for APC of subpopulation apctype that was stored at the position of the first T-cell clonaltype. will factorize it into effector and regulatory conjuagtes at the Derivative function
                m_pfEffectorConjugatesPerAPC[apctype]  = C[0][apctype] *
                                                         ecellsweightedaffinity/tcellsweightedaffinity;
                m_pfRegulatorConjugatesPerAPC[apctype] = C[0][apctype] *
                                                          rcellsweightedaffinity/tcellsweightedaffinity;

                assert((m_pfEffectorConjugatesPerAPC[apctype]+m_pfRegulatorConjugatesPerAPC[apctype]) -
                       m_pfAPCs[apctype]*((double)sites) <= CONJUGATION_OVERFLOW_LIMIT);


                double totalconjugatesonapc = C[0][apctype];
                for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
                {
                    if((E[thtype] + R[thtype]) <= CELLLOWERBOUND)
                    {
                        m_pfEffectorConjugates[thtype][apctype]  =  0.0;
                        m_pfRegulatorConjugates[thtype][apctype] =  0.0;
                    }
                    else
                    {
                        C[thtype][apctype] = totalconjugatesonapc *
                                             m_pfAffinities[thtype][apctype] * (E[thtype] + R[thtype]) /
                                             tcellsweightedaffinity;


                        m_pfEffectorConjugates[thtype][apctype] =
                                (E[thtype]/(E[thtype] + R[thtype])) * C[thtype][apctype];

                        m_pfRegulatorConjugates[thtype][apctype] =
                                (R[thtype]/(E[thtype] + R[thtype])) * C[thtype][apctype];
                    }
                }
            }
        }
    }


    // We now compute the new values of m_pfEffectors[..] and m_pfRegulators[..], based on their old values, proliferation and cell death, and thymic generation of new cells
    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    {
        double effector_incr = 0.0, regulator_incr = 0.0;

        for(unsigned apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
        {
            if(m_pfAPCs[apctype])
            {
                double exp_tmp1 = (9.0*m_pfAPCs[apctype]*m_pfAPCs[apctype]);
                double exp_tmp2 = (m_pfRegulatorConjugatesPerAPC[apctype] - 3.0*m_pfAPCs[apctype])*
                                  (m_pfRegulatorConjugatesPerAPC[apctype] - 3.0*m_pfAPCs[apctype]);

                double Pe = exp_tmp2 / exp_tmp1;
                effector_incr  += kpe*Pe*m_pfEffectorConjugates[thtype][apctype];

                // 		effector_incr  += kpe*Hyp(0, m_pfRegulatorConjugatesPerAPC[apctype],
                // 				      	          m_pfAPCs[apctype]*(double)sites, (double)sites) *
                // 			      ((m_pfAPCs[apctype]*(double)sites)/((m_pfAPCs[apctype]*(double)sites) - m_pfRegulatorConjugatesPerAPC[apctype])) *
                // 			      m_pfEffectorConjugates[thtype][apctype];

                double Pr = (6.0*m_pfAPCs[apctype] -m_pfEffectorConjugatesPerAPC[apctype])*
                            m_pfEffectorConjugatesPerAPC[apctype] / exp_tmp1;
                regulator_incr += kpr*Pr*m_pfRegulatorConjugates[thtype][apctype];

                // 		regulator_incr += kpr*((double)(sites-1) / (double)sites) *
                // 			      (m_pfEffectorConjugatesPerAPC[apctype]/m_pfAPCs[apctype]) *
                // 			      m_pfRegulatorConjugates[thtype][apctype];
            }
        }

        effector_incr  += se - kde * E[thtype];
        regulator_incr += sr - kdr * R[thtype];

        deltaE[thtype] = effector_incr;
        deltaR[thtype] = regulator_incr;
    }
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::FreeThCells(double* E, double* R, double** C, unsigned int thtype)
{
    double conjugatedcells = 0.0;
    for(unsigned apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
    {
        conjugatedcells += C[thtype][apctype];
    }

    return E[thtype] + R[thtype] - conjugatedcells;
}
/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::AvailableBindingSites(double** C, unsigned int apctype)
{
    double conjugatedcells = 0.0;
    for(unsigned thtype = 0; thtype < m_unNumberOfReceptors; thtype++)
    {
        conjugatedcells += C[thtype][apctype];
    }

    assert(conjugatedcells - (m_pfAPCs[apctype]*((double)sites)) <= CONJUGATION_OVERFLOW_LIMIT);

    //    if(conjugatedcells > (m_pfAPCs[apctype]*((double)sites)))
    //{

    //    printf("\nThe errant APC type: %d\n\n",apctype);

    //    // Print NonZero APCs
    //    printf("\n");
    //    for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
    //    {
    //        if(m_pfAPCs[apctype]>0.0)
    //            printf("APC[%d]=%f  ",apctype,m_pfAPCs[apctype]);
    //    }

    //    // Print table of conjugates
    //    printf("\n=================================\n");
    //    for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
    //    {
    //        printf("\nAPC:%d\t",apctype);
    //        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    //        {
    //            printf("%e  ",C[thtype][apctype]);
    //        }
    //    }


    //    // Print Effector and regulatory clonaltypes
    //    printf("\n=================================\n");
    //    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    //    {
    //   if(!(m_pfEffectors[thtype] + m_pfRegulators[thtype] <= CELLLOWERBOUND))
    //           printf("E[%d]=%e,R[%d]=%e  ",thtype,m_pfEffectors[thtype],thtype,m_pfRegulators[thtype]);
    //    }

    //	exit(-1);
    //}


    return m_pfAPCs[apctype]*sites - conjugatedcells;
}
/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::Factorial(double val)
{
    assert(val >= 0);

    // tgamma applies the gamma function to x. The gamma function is defined as
    // gamma (x) = integral from 0 to inf of t^(x-1) e^-t dt
    // http://www.delorie.com/gnu/docs/glibc/libc_393.html

    return tgamma(val+1.0);
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::Combination(double N, double R)
{
    if((N-R) < 0)
        return 0.0;

    return (Factorial(N) / (Factorial(N-R) * Factorial(R)));
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::Hyp(double N, double No, double M, double L)
{
    if(M < 3) // If the number of APCs*sites is < 3, return 0 - prevents 0 in denominator
    {
        return 0.0;
    }

    return (Combination(No, N) * Combination(M - No, L - N))/Combination(M, L);
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::PrintCRMDetails(unsigned id)
{
    //unsigned int CurrentStepNumber = CSimulator::GetInstance()->GetSimulationStepNumber();
    if(!(robotAgent->GetIdentification() == id))
        return;

    // Print NonZero APCs
    printf("\n");
    for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
    {
        if(m_pfAPCs[apctype]>0.0)
            printf("APC[%d]=%f  ",apctype,m_pfAPCs[apctype]);
    }

    // Print table of conjugates
    printf("\n=================================\n");
    for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
    {
        printf("\nAPC:%d\t",apctype);
        for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
        {
            printf("%e  ",m_pfConjugates[thtype][apctype]);
        }
    }


    // Print Effector and regulatory clonaltypes
    printf("\n=================================\n");
    for(unsigned thtype=0; thtype < m_unNumberOfReceptors; thtype++)
    {
        printf("E[%d]=%e,R[%d]=%e  ",thtype,m_pfEffectors[thtype],thtype,m_pfRegulators[thtype]);
    }
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::UpdateState()
{
    double E, R;

    //if(m_bConvergenceFlag)
        for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
        {
            E = 0.0; R = 0.0;

            // if no apc's of specific type, how can i make a decision
            if(m_pfAPCs[apctype] > 0.0)
            {
                for(unsigned thtype = 0; thtype < m_unNumberOfReceptors; thtype++)
                {
                    // Brute force approach to cell generation
                    //cells which are not reacting to APCs would have stable states at se/kde and sr/kdr.
                    //we ignore these clonaltypes in the tolerance decision
                    /*if ((fabs(m_pfEffectors[thtype] - se/kde)  < 0.00001  &&
                         fabs(m_pfRegulators[thtype] - sr/kdr) < 0.00001) ||
                        (m_pfAPCs[apctype] == 0.0))
                    {
                        continue;
                    }*/

                    E += m_pfAffinities[thtype][apctype] * m_pfEffectors[thtype];
                    R += m_pfAffinities[thtype][apctype] * m_pfRegulators[thtype];
                }
            }
            m_pfSumEffectorsWeightedbyAffinity[apctype]  = E;
            m_pfSumRegulatorsWeightedbyAffinity[apctype] = R;


            if (((E + R) <= CELLLOWERBOUND) || fabs(E - R) <= CELLLOWERBOUND)
            {
                // Dont know - no T-cells to make decision or E approx. equal to R
                robotAgent->SetMostWantedList(apctype, 0);
            }
            else if (E > R)
            {
                // Attack
                robotAgent->SetMostWantedList(apctype, 1);
            }
            else if(R > E)
            {
                // Tolerate
                robotAgent->SetMostWantedList(apctype, 2);
            }
        }
//    else
//        for(unsigned apctype=0; apctype < m_unNumberOfReceptors; apctype++)
//        {
//            E = 0.0; R = 0.0;
//            for(unsigned thtype = 0; thtype < m_unNumberOfReceptors; thtype++)
//            {
//                E += m_pfAffinities[thtype][apctype] *
//                     (m_pfEffectors[thtype] - m_pfEffectors_prev[thtype]);
//                R += m_pfAffinities[thtype][apctype] *
//                     (m_pfRegulators[thtype] - m_pfRegulators_prev[thtype]);
//            }

//            if (E > R)
//            {
//                m_pbAttack[apctype] = 1;
//                robotAgent->SetMostWantedList(apctype, true);
//            }
//            else if (R > E)
//            {
//                m_pbAttack[apctype] = 2;
//                robotAgent->SetMostWantedList(apctype, false);
//            }

//            //(E==R) case ingnored does not give any information. If we dont know, tolerate!
//        }

}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::Sense()
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

double CRMinRobotAgent::NormalizedAffinity(unsigned int v1, unsigned int v2)
{
    /* XOr operation between the 2 feature vectors */
    unsigned int unXoredString = (v1 ^ v2);

    /* Number of 1's from the result of the previous XOR operation,  at positions preset by mask */
    /* how do we decide whose mask should be used */
    unsigned int unMatching  = CRMinRobotAgent::GetNumberOfSetBits(unXoredString);

    //TODO: Have to change affinity computation
    return (double) (CFeatureVector::NUMBER_OF_FEATURES - unMatching) / (double)
            CFeatureVector::NUMBER_OF_FEATURES;
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::NegExpDistAffinity(unsigned int v1, unsigned int v2, double k)
{
    /* k is proportional to the level of cross affinity*/
    /* k=0.01 affinity of 1 when HD is 0, else 0  */
    /* k=inf  affinity of 1 for all HD */

    /* XOr operation between the 2 feature vectors */
    unsigned int unXoredString = (v1 ^ v2);

    /* Number of 1's from the result of the previous XOR operation,  at positions preset by mask */
    /* how do we decide whose mask should be used */
    unsigned int hammingdistance  = CRMinRobotAgent::GetNumberOfSetBits(unXoredString);

    //return 1.0 * exp(-(1.0/k) * (double)hammingdistance);
    // Should we normalize the hammingdistance when input to the exp function, or as above?

    return 1.0 * exp(-(1.0/k) * (double)hammingdistance / (double) CFeatureVector::NUMBER_OF_FEATURES);
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetCurrE(unsigned thtype)
{
    return m_pfEffectors[thtype];
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetCurrR(unsigned thtype)
{
    return m_pfRegulators[thtype];
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetAPC(unsigned apctype)
{
    return m_pfAPCs[apctype];
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::SetCurrE(unsigned thtype, double f_currE)
{
    m_pfEffectors[thtype] = f_currE;
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::SetCurrR(unsigned thtype, double f_currR)
{
    m_pfRegulators[thtype] = f_currR;
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetWeight()
{
    return m_fWeight;
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetFVtoApcScaling()
{
    return m_fFVtoApcscaling;
}

/******************************************************************************/
/******************************************************************************/

bool CRMinRobotAgent::GetConvergenceFlag()
{
    return m_bConvergenceFlag;
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetConvergenceError()
{
    return m_dconvergence_error;
}

/******************************************************************************/
/******************************************************************************/

double CRMinRobotAgent::GetConvergenceError_Perc()
{
    return m_dpercconvergence_error;
}

/******************************************************************************/
/******************************************************************************/

void CRMinRobotAgent::ScaleDownConjugates(double** pf_Conjugates)
{
    for(unsigned int apctype = 0; apctype < m_unNumberOfReceptors; apctype++)
    {
        if(m_pfAPCs[apctype] > 0.0)
        {

            double f_ConjugatesOnAPC = 0.0;
            for(unsigned int thtype = 0; thtype < m_unNumberOfReceptors; thtype++)
            {
                f_ConjugatesOnAPC += pf_Conjugates[thtype][apctype];
            }

            if(f_ConjugatesOnAPC > (m_pfAPCs[apctype]*(double)sites))
            {
                double scaledownfactor = f_ConjugatesOnAPC/(m_pfAPCs[apctype]*(double)sites);

                for(unsigned int thtype = 0; thtype < m_unNumberOfReceptors; thtype++)
                {
                    pf_Conjugates[thtype][apctype] /= scaledownfactor;
                }
            }

        }
    }
}

/******************************************************************************/
/******************************************************************************/

unsigned int CRMinRobotAgent::GetNumberOfSetBits(unsigned int x)
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
