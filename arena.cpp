#include "arena.h"
#include <math.h>

bool CArena::g_bIsBoundless = false;

/******************************************************************************/
/******************************************************************************/

CArena::CArena(const char* pch_name,
               double f_size_x,       
               double f_size_y, 
               unsigned int un_res_x,
               unsigned int un_res_y) : 
    CSimObject(pch_name),
    m_fSizeX(f_size_x),
    m_fSizeY(f_size_y),
    m_unResX(un_res_x),
    m_unResY(un_res_y)
{
    m_plistAgents                      = new TAgentList[un_res_x * un_res_y];
    m_unNumberOfCytokineConcentrations = 0;
}

/******************************************************************************/
/******************************************************************************/

CArena::~CArena()
{
    delete[] m_plistAgents;
}

/******************************************************************************/
/******************************************************************************/

void CArena::GetSize(double* pf_size_x, double* pf_size_y) const
{
    (*pf_size_x) = m_fSizeX;
    (*pf_size_y) = m_fSizeY;
}

/******************************************************************************/
/******************************************************************************/

void CArena::GetResolution(unsigned int* pun_res_x, unsigned int* pun_res_y) const
{
    (*pun_res_x) = m_unResX;
    (*pun_res_y) = m_unResY;
}

/******************************************************************************/
/******************************************************************************/

TAgentListIterator CArena::FindAgent(TAgentList* plist_agents, CAgent* pc_agent) const
{
    TAgentListIterator i = plist_agents->begin();

    while (i != plist_agents->end() && (*i) != pc_agent)
        i++;

    return i;
}

/******************************************************************************/
/******************************************************************************/

void CArena::AddAgent(CAgent* pc_agent, TVector2d* pt_new_position)
{
    m_plistAgents[XYToArrayPosition(pc_agent->GetPosition())].push_front(pc_agent);
}

/******************************************************************************/
/******************************************************************************/

void CArena::AddAgent(CAgent* pc_agent, unsigned int un_array_position)
{
    m_plistAgents[un_array_position].push_front(pc_agent);    
}

/******************************************************************************/
/******************************************************************************/

void CArena::RemoveAgent(CAgent* pc_agent)
{
    TAgentList* plistAgents = &m_plistAgents[XYToArrayPosition(pc_agent->GetPosition())];
    plistAgents->erase(FindAgent(plistAgents, pc_agent));
}

/******************************************************************************/
/******************************************************************************/

void CArena::RemoveAgent(CAgent* pc_agent, unsigned int un_array_position)
{
    TAgentList* plistAgents = &m_plistAgents[un_array_position];
    plistAgents->erase(FindAgent(plistAgents, pc_agent));
}

/******************************************************************************/
/******************************************************************************/

void CArena::MoveAgent(CAgent* pc_agent, TVector2d* pt_new_position)
{
    unsigned int unOldArrayPosition = XYToArrayPosition(pc_agent->GetPosition());
    unsigned int unNewArrayPosition = XYToArrayPosition(pt_new_position); 

    pc_agent->SetPosition(pt_new_position);

    if (unOldArrayPosition != unNewArrayPosition)
    {
        RemoveAgent(pc_agent, unOldArrayPosition);
        AddAgent(pc_agent, unNewArrayPosition);
    }
}

/******************************************************************************/
/******************************************************************************/

void CArena::GetAgentsCloseTo(TAgentListList* pt_output_list, 
                              const TVector2d* pt_position,
                              double f_radius)
{
    pt_output_list->clear();

    if (f_radius > m_fSizeX / 2.0 || f_radius > m_fSizeY / 2.0)
    {
        for (int i = 0; i < m_unResX * m_unResY; i++)
        { 
            pt_output_list->push_back(&m_plistAgents[i]);
        }
    } else {        
        TVector2d tTranslatedPosition;

        double fCellSizeX = m_fSizeX / (double) m_unResX; 
        double fCellSizeY = m_fSizeY / (double) m_unResY; 

        f_radius += max(fCellSizeX, fCellSizeY);

        // We translate all coordinates to the first quadrant:
        tTranslatedPosition.x = pt_position->x + (m_fSizeX / 2.0);
        tTranslatedPosition.y = pt_position->y + (m_fSizeY / 2.0);

        double fStartCellX = (tTranslatedPosition.x - f_radius) / fCellSizeX;
        double fStartCellY = (tTranslatedPosition.y - f_radius) / fCellSizeY;

        if (fStartCellX < 0.0)
        {
            fStartCellX = 0.0;
        }

        if (fStartCellY < 0.0)
        {
            fStartCellY = 0.0;
        }

        // Go to the center of the start square:
        double fStartX = (fStartCellX + (double) 0.49) * (double) fCellSizeX;
        double fStartY = (fStartCellY + (double) 0.49) * (double) fCellSizeY;

        double fEndX   = (tTranslatedPosition.x + f_radius) + fCellSizeX / 2.0;
        double fEndY   = (tTranslatedPosition.y + f_radius) + fCellSizeY / 2.0;

        if (fEndX > m_fSizeX)
        {
            fEndX = m_fSizeX;
        }

        if (fEndY > m_fSizeY)
        {
            fEndY = m_fSizeY;
        }

        double fCellY = fStartCellY;
        for (double fY = fStartY; fY < fEndY; fY += fCellSizeY, fCellY += 1.0)
        {
            int nCellY = (int) floor(fCellY);

            if (nCellY >= 0 && nCellY < m_unResY)
            {
                double fCellX = fStartCellX;

                for (double fX = fStartX; fX < (tTranslatedPosition.x + f_radius) + fCellSizeX / 2.0 && fX < m_fSizeX; fX += fCellSizeX, fCellX += 1.0)
                {
                    double fRelativeX = fX - tTranslatedPosition.x;
                    double fRelativeY = fY - tTranslatedPosition.y;
                    int nCellX = (int) floor(fCellX);
                
                    if (nCellX >= 0 && nCellX < m_unResX && (sqrt(fRelativeX * fRelativeX + fRelativeY * fRelativeY) <= f_radius))
                    {
                        unsigned int unArrayPosition = nCellX + nCellY * m_unResX;
//                    printf("Array position: %d\n", unArrayPosition);
                        pt_output_list->push_back(&m_plistAgents[unArrayPosition]);
                    }
                }
            }
        }
    }
}

/******************************************************************************/
/******************************************************************************/

unsigned int CArena::XYToArrayPosition(const TVector2d* pt_position) const
{
    return XYToArrayPosition(pt_position->x, pt_position->y);
}

/******************************************************************************/
/******************************************************************************/

unsigned int CArena::XYToArrayPosition(double f_x, double f_y) const
{
    double fX = (((f_x + m_fSizeX / 2.0) * (double)m_unResX) / m_fSizeX);
    double fY = (((f_y + m_fSizeY / 2.0) * (double)m_unResY) / m_fSizeY);
    
    unsigned int unX = (unsigned int) fX;
    unsigned int unY = (unsigned int) fY;
    
    if (unX >= m_unResX) 
        printf("\n X -- x: %d (%d), y: %d (%d)\n", unX, m_unResX, unY, m_unResY); 

    if (unY >= m_unResY) 
        printf("\n Y -- x: %d (%d), y: %d (%d)\n", unX, m_unResX, unY, m_unResY); 


    return unX + unY * m_unResX;
}

/******************************************************************************/
/******************************************************************************/

void CArena::XYToArrayXY(double f_x, 
                         double f_y, 
                         unsigned int* pun_x, 
                         unsigned int* pun_y) const
{
    double fX = (((f_x + m_fSizeX / 2.0) * (double)m_unResX) / m_fSizeX);
    double fY = (((f_y + m_fSizeY / 2.0) * (double)m_unResY) / m_fSizeY);

    (*pun_x) = (unsigned int) fX;
    (*pun_y) = (unsigned int) fY;    
}

/******************************************************************************/
/******************************************************************************/

