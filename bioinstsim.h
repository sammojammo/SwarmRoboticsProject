#ifndef BIOINSTSIM_H_
#define BIOINSTSIM_H_

/******************************************************************************/
/******************************************************************************/

#include <stdio.h>
#include <getopt.h>

/******************************************************************************/
/******************************************************************************/

#include "arguments.h"
#include "common.h"
#include "simulator.h"
#include "arena.h"
#include "agent.h"

/******************************************************************************/
/******************************************************************************/

enum EArenaType 
{
    BOUNDLESS,
    CIRCULAR,
    RECTANGULAR
};

/******************************************************************************/
/******************************************************************************/

enum ECytokineSetupType 
{
    NONE,
    SIMPLEIL2,
};

/******************************************************************************/
/******************************************************************************/

enum EThAgentType 
{
    REAL,
    REALIL2,
    RECRUITMENT,
    RECRUITMENTIL2,
    REALWITHFEATURE
};

/******************************************************************************/
/******************************************************************************/

class CBioInstSim
{
public:
    CBioInstSim(int argc, char** argv);
    virtual ~CBioInstSim();

    void Run();

protected:
    void ParseArguments();
    void PrintUsage();

    CExperiment* CreateExperiment();
   
protected:
    unsigned int        m_argc;
    char**              m_argv;

    static struct option m_tLongOptions[];

    bool                m_bVerbose;
    bool                m_bOutputStatistics;
    unsigned int        m_unRandomSeed;
    unsigned int        m_unNumberOfCycles;

    char*               m_pchColorFilename;

    CSimulator*         m_pcSimulator;
    bool                m_bRendering;
    CExperiment*        m_pcExperiment;
    CArguments*         m_pcExperimentArguments;
    CArguments*         m_pcAgentArguments;
    CArguments*         m_pcArenaArguments;
    CArguments*         m_pcPopulationAnalyzerArguments;

    CArguments*         m_pcModelArguments; //m_pcCRMArguments;
};

/******************************************************************************/
/******************************************************************************/

#endif
