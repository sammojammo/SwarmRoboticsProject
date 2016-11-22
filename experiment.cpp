#include <math.h>

#include "experiment.h"
#include "random.h"

// Arenas:
#include "circulararena.h"
#include "rectangulararena.h"
#include "boundlessarena.h"

// Agents:
#include "agent.h"



/******************************************************************************/
/******************************************************************************/

CExperiment::CExperiment(CArguments* pc_experiment_arguments,
                         CArguments* pc_arena_arguments,
                         CArguments* pc_agent_arguments,
                         CArguments* pc_model_arguments) :
    m_pcExperimentArguments(pc_experiment_arguments),
    m_pcArenaArguments(pc_arena_arguments),
    m_pcAgentArguments(pc_agent_arguments),
    m_pcModelArguments(pc_model_arguments),
    m_unNumberOfColors(5),
    CSimObject("")
{
    m_unNumberOfAgents   = pc_agent_arguments->GetArgumentAsIntOr("count", 10);
}

/******************************************************************************/
/******************************************************************************/

CExperiment::~CExperiment() 
{
}

/******************************************************************************/
/******************************************************************************/

CSimulator* CExperiment::CreateSimulator(unsigned int un_number_of_cycles)
{
    m_pcSimulator = new CSimulator(un_number_of_cycles);

    m_pcArena = CreateArena();
    m_pcSimulator->SetArena(m_pcArena);    

    CreateAndAddAgents(m_pcSimulator);

    return m_pcSimulator;
}

/******************************************************************************/
/******************************************************************************/

CArena* CExperiment::CreateArena()
{
    CArena* pcArena = NULL;
    double fRadius   = m_pcArenaArguments->GetArgumentAsDoubleOr("radius", 50.0f);
    double fSizeX    = m_pcArenaArguments->GetArgumentAsDoubleOr("sizex", 100.0f);
    double fSizeY    = m_pcArenaArguments->GetArgumentAsDoubleOr("sizey", 100.0f);
    int    unResX    = m_pcArenaArguments->GetArgumentAsIntOr("resy", 50);
    int    unResY    = m_pcArenaArguments->GetArgumentAsIntOr("resy", 50);
    

    if (m_pcArenaArguments->GetArgumentIsDefined("help")) 
    {
        printf("Arena help:\n"
               "  name=[CIRCULAR, RECTANGULAR, BOUNDLESS]\n"
               "  radius=#.#          Arena radius, (only applicable if a circular arena is used [%f])\n"
               "  sizex=#.#           Arena dimension x, only applicable if a rectangular arena is used [%f]\n"
               "  sizey=#.#           Arena dimension y, only applicable if a rectangular arena is used [%f]\n"
               "  resx=#              Arena resolution x (number of rectangles in the X dimension) [%d]\n"
               "  resy=#              Arena resolution y (number of rectangles in the Y dimension) [%d]\n", 
               fRadius,
               fSizeX,
               fSizeY,
               unResX,
               unResY);
    }
    
    
    if (m_pcArenaArguments->GetArgumentIsDefined("name")) 
    {
        CArena* pchArenaSetup = NULL;
        
        const char* pchName = m_pcArenaArguments->GetArgumentAsString("name");
        if (strcmp(pchName, "CIRCULAR") == 0)
        {
            pcArena = new CCircularArena("CircularArena", fRadius, unResX, unResY);
        } else if (strcmp(pchName, "RECTANGULAR") == 0) {
            pcArena = new CRectangularArena("RectangularArena", fSizeX, fSizeY, unResX, unResY);                          
        } else if (strcmp(pchName, "BOUNDLESS") == 0) {
            pcArena = new CBoundlessArena("BoundlessArena", fSizeX, fSizeY, unResX, unResY);                          
        } else {
            ERROR1("Unknown arena: %s", pchName); 
        } 
    } else {
        pcArena = new CBoundlessArena("BoundlessArena", fSizeX, fSizeY, unResX, unResY);
    }

    return pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::CreateAndAddAgents(CSimulator* pc_simulator)
{
    if (m_pcAgentArguments->GetArgumentIsDefined("help")) 
    {
        printf("ThAgent setup:\n"
               "  count=#                Set the number of ThAgents (%d)\n", m_unNumberOfAgents);                  
    }

    for (int i = 0; i < m_unNumberOfAgents; i++)
    {
       
        CAgent* pcAgent = CreateAgent();

        #ifndef TCELLCLONEEXCHANGEANALYSIS
            PlaceAgentRandomly(pcAgent);
        #endif
        pc_simulator->AddAgent(pcAgent);
    }
}

/******************************************************************************/
/******************************************************************************/

CAgent* CExperiment::CreateAgent()
{
    const char* pchName = m_pcAgentArguments->GetArgumentAsStringOr("name", "notset");
    static unsigned int unAgentNumber = 0;
    char pchTemp[128];
    sprintf(pchTemp, "Agent%d", unAgentNumber);   
    CAgent* pchAgent = NULL;       
    
    unAgentNumber++;
    return pchAgent;
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::PlaceAgentRandomly(CAgent* pc_agent) 
{
    double fArenaSizeX;
    double fArenaSizeY;
    m_pcArena->GetSize(&fArenaSizeX, &fArenaSizeY);

    TVector2d tPosition;
    
    // Find the initial position for the agent:
    bool bFreeSpotFound = false;
    do 
    {
        tPosition.x = Random::nextDouble() * fArenaSizeX - (fArenaSizeX / 2.0f);
        tPosition.y = Random::nextDouble() * fArenaSizeY - (fArenaSizeY / 2.0f);
        
        bFreeSpotFound = !m_pcArena->IsObstacle(&tPosition);
    } while (!bFreeSpotFound);
    
    pc_agent->SetPosition(&tPosition);
    if (pc_agent->GetClosestAgent(CAgent::RADIUS * 2.0, ANY) > 0)
        PlaceAgentRandomly(pc_agent);

}

/******************************************************************************/
/******************************************************************************/

void CExperiment::PlaceAgentRandomly(CAgent* pc_agent, 
                                     double f_origin_x, 
                                     double f_origin_y, 
                                     double f_radius)
{
    PlaceAgentRandomly(pc_agent, f_origin_x, f_origin_y, 0.0, f_radius);
}

/******************************************************************************/
/******************************************************************************/

void CExperiment::PlaceAgentRandomly(CAgent* pc_agent, double f_origin_x, double f_origin_y, double f_inner_radius, double f_outer_radius)
{
    TVector2d tPosition;

    double fArenaSizeX;
    double fArenaSizeY;
    m_pcArena->GetSize(&fArenaSizeX, &fArenaSizeY);
    
    // Find the initial position for the agent:
    bool bFreeSpotFound = false;
    do 
    {        
        double fX = (1.0 - Random::nextDouble() * 2.0) * f_outer_radius + f_origin_x;
        double fY = (1.0 - Random::nextDouble() * 2.0) * f_outer_radius + f_origin_y;

        if (fX >= -fArenaSizeX / 2.0 && fX <= fArenaSizeX / 2.0 && fY >= -fArenaSizeY / 2.0 && fY <= fArenaSizeY / 2.0)
        {
            double fDistance = sqrt((fX - f_origin_x) * (fX - f_origin_x) + (fY - f_origin_y) * (fY - f_origin_y));
            
            if (!m_pcArena->IsObstacle(&tPosition) && !(fDistance > f_outer_radius || fDistance < f_inner_radius))
            {
                bFreeSpotFound = true;
                tPosition.x = fX;
                tPosition.y = fY;
            }

        }

    } while (!bFreeSpotFound);
    
    pc_agent->SetPosition(&tPosition);

}

/******************************************************************************/
/******************************************************************************/
