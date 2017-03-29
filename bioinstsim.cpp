/******************************************************************************/
/******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/******************************************************************************/
/******************************************************************************/

#include "arguments.h"
#include "bioinstsim.h"
#include "random.h"
#include "openglrender.h"
#include "populationanalyzer.h"


// Experiments:
#include "testexperiment.h"
//#include "proliferationvsinternalcounterexperiment.h"
//#include "proliferationvsmacromodelexperiment.h"
//#include "multimacromodelexperiment.h"


#define OPTIONSFILEBUFFERSIZE (1024 * 10)

/******************************************************************************/
/******************************************************************************/

struct option CBioInstSim::m_tLongOptions[] = {
    {"help",             0, 0,              'h'},
    {"verbose",          0, 0,              'v'},
    {"random-seed",      1, 0,              's'},
    {"arena",            1, 0,              'a'},
    {"experiment",       1, 0,              'e'},
    {"agent",            1, 0,              'T'},
    {"model",            1, 0,              'M'},
    {"apcagent",         1, 0,              'A'},
    {"analyzer",         1, 0,              'Z'},
    {"no-rendering",     0, 0,              'z'},
    {"number-of-cycles", 1, 0,              'n'},
    {"enable-analysis",  1, 0,              'Z'},
    {"silent",           0, 0,              'S'},
    {"color-file",       1, 0,              'c'},
    {0, 0, 0, 0}
};


/******************************************************************************/
/******************************************************************************/

CBioInstSim::CBioInstSim(int argc, char** argv) :
    m_bVerbose(true),
    m_unRandomSeed(0),
    m_argv(argv),
    m_argc(argc),
    m_bRendering(true),
    m_unNumberOfCycles(10000),
    m_pcExperiment(NULL),
    m_pcExperimentArguments(NULL),
    m_pcAgentArguments(NULL),
    m_pcArenaArguments(NULL),
    m_pcModelArguments(NULL),
    m_pcPopulationAnalyzerArguments(NULL),
    m_bOutputStatistics(true),
    m_pchColorFilename(NULL)

{
    for (int i = 0; i < argc; i++)
    {
        printf("%s ", argv[i]);
    }
    printf("\n");

}

/******************************************************************************/
/******************************************************************************/

CBioInstSim::~CBioInstSim()
{
}

/******************************************************************************/
/******************************************************************************/

void CBioInstSim::Run()
{
    ParseArguments();

    m_pcExperiment = CreateExperiment();
    m_pcSimulator  = m_pcExperiment->CreateSimulator(m_unNumberOfCycles);
    m_pcSimulator->SetExperiment(m_pcExperiment);

    if (m_pcPopulationAnalyzerArguments != NULL)
    {
        CPopulationAnalyzer* pcAnalyzer = new CPopulationAnalyzer(m_pcPopulationAnalyzerArguments);
        m_pcSimulator->AddChild(pcAnalyzer);
    }

    if (m_bRendering)
    {
        COpenGLRender* pcRender = new COpenGLRender(m_pchColorFilename, m_pcSimulator->GetAllAgents()->size(), m_unNumberOfCycles);
        m_pcSimulator->AddChild(pcRender);
        pcRender->SetOutputStatistics(m_bOutputStatistics);
    }

    m_pcSimulator->Run();

    delete m_pcSimulator;
}

/******************************************************************************/
/******************************************************************************/

void CBioInstSim::ParseArguments()
{

    m_unRandomSeed = Random::set_seed_usec();
    DEBUGOUT1("Setting random-seed to %d, from current CPU clock (usec)", m_unRandomSeed);
    Random::nextInt(1);

    int cOption;
    int nOptionIndex = 0;
    int unNumAssigned;

    char* pchTemp;
    char  pchOptionFileOutput[OPTIONSFILEBUFFERSIZE];
    pchOptionFileOutput[0] = '\0';

    // First parse the options:
    while ((cOption = getopt_long(m_argc, m_argv,
                                  "hvs:a:e:T:M:A:zn:SZ:c:d:",
                                  m_tLongOptions, &nOptionIndex)) != -1)
    {
        if (nOptionIndex <= 0)
        {
            char pchTempOption[16];
            sprintf(pchTempOption, "-%c", cOption);
            strcat(pchOptionFileOutput, pchTempOption);
        } else {
            strcat(pchOptionFileOutput, "--");
            strcat(pchOptionFileOutput, m_tLongOptions[nOptionIndex].name);
        }

        if (optarg != 0)
        {
            strcat(pchOptionFileOutput, " ");
            strcat(pchOptionFileOutput, optarg);
        }

        strcat(pchOptionFileOutput, "\n");

        switch (cOption)
        {
        case 'h' :
            PrintUsage();
            exit(-1);
            break;

        case 'v' :
            m_bVerbose = true;
            break;


        case 's' :
    	    m_unRandomSeed = atoi(optarg);
            Random::set_seed(m_unRandomSeed);
            DEBUGOUT1("Setting random-seed to %d, from command-line", m_unRandomSeed);
            break;


        case 'z':
            m_bRendering = false;
            break;

        case 'e':
            m_pcExperimentArguments = new CArguments(optarg);
            break;

        case 'a':
            m_pcArenaArguments      = new CArguments(optarg);
            break;

        case 'T':
            m_pcAgentArguments      = new CArguments(optarg);
            break;

        case 'M':
            m_pcModelArguments      = new CArguments(optarg);
            break;

        case 'Z':
            m_pcPopulationAnalyzerArguments = new CArguments(optarg);
//            printf("optarg = %d\n", optarg);

            break;

        case 'n':
            m_unNumberOfCycles = atoi(optarg);
            break;

        case 'c':
            m_pchColorFilename = new char[strlen(optarg) + 2];
            strcpy(m_pchColorFilename, optarg);
            break;

        case 'S':
            m_bOutputStatistics = false;
            break;
        /*New parameter added - bit depth of features*/
        case 'd':
            CFeatureVector::FEATURE_DEPTH = atof(optarg);
            break;

        default:
            ERROR("Unrecognized option or missing parameter");
            exit(-1);
        }
    }

    if (m_pcExperimentArguments == NULL)
    {
        m_pcExperimentArguments = new CArguments("");
    }

    if (m_pcAgentArguments == NULL)
    {
        m_pcAgentArguments = new CArguments("");
    }

    if (m_pcArenaArguments == NULL)
    {
        m_pcArenaArguments = new CArguments("");
    }

    if (m_pcModelArguments == NULL)
    {
        m_pcModelArguments = new CArguments("");
    }

    printf("Setting random-seed to %d\n", m_unRandomSeed);
    //Random::set_seed(m_unRandomSeed);
}

/******************************************************************************/
/******************************************************************************/

void CBioInstSim::PrintUsage()
{
    printf("Usage: combotsim [options...]\n"
           "-h, --help                              Display help           \n"
           "-v, --verbose                           Produce verbose output \n"
           "-s, --random-seed #                     Set the random seed [%d]\n"
           "-a, --arena                             Set the arena arguments (-a help)\n"
           "-e, --experiment                        Set the experiment arguments (-a help)\n"
           "-T, --agent                             Set the th-agent arguments (-T help)\n"
           "-M, --model                             Set the fault-detection model arguments (-M help)\n"
           "-n, --number-of-cycles                  Number of simulation cycles [%d]\n"
           "-z, --no-rendering                      Disable rendering\n"
           "-Z, --enable-analysis                   Enable agent distribution analysis and set the arguments (e.g. -Z updateperiod=100)\n"
           "-S, --silent                            Disable output\n"
           "-c, --color-file #.#                    Name of file with agent colors [%s]\n"
           ,
           m_unRandomSeed,
           m_unNumberOfCycles,
           m_pchColorFilename == NULL ? "#none#" : m_pchColorFilename);
}

/******************************************************************************/
/******************************************************************************/

CExperiment* CBioInstSim::CreateExperiment()
{

    CExperiment* pcExperiment = NULL;

    if (m_pcExperimentArguments->GetArgumentIsDefined("help"))
    {
        printf("Experiment help:\n"
               "  name = [PROLIFERATIONVSRECRUITMENT(P1)]\n"
               "  If no name is given, the default experiment (CExperiment) will be used)\n");

    }

    if (!m_pcExperimentArguments->GetArgumentIsDefined("name"))
    {
        pcExperiment = new CExperiment(m_pcExperimentArguments,
                                       m_pcArenaArguments,
                                       m_pcAgentArguments,
                                       m_pcModelArguments);

    } else {
        const char* pchExperimentName = m_pcExperimentArguments->GetArgumentAsString("name");
        if (strcmp(pchExperimentName, "TEST") == 0)
        {
            pcExperiment = new CTestExperiment(m_pcExperimentArguments,
                                               m_pcArenaArguments,
                                               m_pcAgentArguments,
                                               m_pcModelArguments);

        }
    }

    return pcExperiment;
}

/******************************************************************************/
/******************************************************************************/


