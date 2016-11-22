#include "boundlessarena.h"
#include <math.h>


/******************************************************************************/
/******************************************************************************/

CBoundlessArena::CBoundlessArena(const char* pch_name, 
                                 double f_size_x,       
                                 double f_size_y, 
                                 unsigned int un_res_x, 
                                 unsigned int un_res_y) : CArena(pch_name, f_size_x, f_size_y, un_res_x, un_res_y)
{
    g_bIsBoundless = true;
}

/******************************************************************************/
/******************************************************************************/



void CBoundlessArena::MoveAgent(CAgent* pc_agent, TVector2d* pt_new_position)
{
    if (pt_new_position->x > m_fSizeX / 2.0)
    {
        pt_new_position->x -= m_fSizeX;
    } 
    else if (pt_new_position->x < -m_fSizeX / 2.0)
    {
        pt_new_position->x += m_fSizeX;
    }

    if (pt_new_position->y > m_fSizeX / 2.0)
    {
        pt_new_position->y -= m_fSizeY;
    } 
    else if (pt_new_position->y < -m_fSizeY / 2.0)
    {
        pt_new_position->y += m_fSizeY;
    }

    CArena::MoveAgent(pc_agent, pt_new_position);
}

/******************************************************************************/
/******************************************************************************/

bool CBoundlessArena::IsObstacle(TVector2d* t_position)
{
    if (t_position->x > m_fSizeX / 2.0 || 
        t_position->x < -m_fSizeX / 2.0 ||
                        t_position->y > m_fSizeY / 2.0 || 
        t_position->y < -m_fSizeY / 2.0)
        return true;
    else
        return false;
}

/******************************************************************************/
/******************************************************************************/

void CBoundlessArena::GetAgentsCloseTo(TAgentListList* pt_output_list, 
                                       const TVector2d* pt_position,
                                       double f_radius)
{
    pt_output_list->clear();
    
    for (int i = 0; i < m_unResX * m_unResY; i++)
    { 
        pt_output_list->push_back(&m_plistAgents[i]);
    }
    
    
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

        double fStartCellX = (tTranslatedPosition.x - f_radius) / fCellSizeX - 1.0;
        double fStartCellY = (tTranslatedPosition.y - f_radius) / fCellSizeY - 1.0;

        // Go to the center of the start square:
        double fStartX = (fStartCellX + (double) 0.49) * (double) fCellSizeX;
        double fStartY = (fStartCellY + (double) 0.49) * (double) fCellSizeY;

        double fEndX   = (tTranslatedPosition.x + f_radius) + fCellSizeX / 2.0 + fCellSizeX;
        double fEndY   = (tTranslatedPosition.y + f_radius) + fCellSizeY / 2.0 + fCellSizeY;

        double fCellY = fStartCellY;
        for (double fY = fStartY; fY < fEndY; fY += fCellSizeY, fCellY += 1.0)
        {
            int nCellY = (int) floor(fCellY);
            if (nCellY < 0)         nCellY += m_unResY;
            if (nCellY >= m_unResY) nCellY -= m_unResY;
        
            double fCellX = fStartCellX;

            for (double fX = fStartX; fX < (tTranslatedPosition.x + f_radius) + fCellSizeX / 2.0 ; fX += fCellSizeX, fCellX += 1.0)
            {                        
                double fRelativeX = fabs(fX - tTranslatedPosition.x);
                if (fRelativeX > m_fSizeX / 2.0) fRelativeX -= m_fSizeX / 2.0;
                
                double fRelativeY = fabs(fY - tTranslatedPosition.y);
                if (fRelativeY > m_fSizeY / 2.0) fRelativeY -= m_fSizeY / 2.0;


                if (sqrt(fRelativeX * fRelativeX + fRelativeY * fRelativeY) <= f_radius)
                {
                    int nCellX = (int) floor(fCellX);
                    if (nCellX < 0)         nCellX += m_unResX;
                    if (nCellX >= m_unResX) nCellX -= m_unResX;
                
                    unsigned int unArrayPosition = nCellX + nCellY * m_unResX;
//                    printf("Array position: %d\n", unArrayPosition);
                    pt_output_list->push_back(&m_plistAgents[unArrayPosition]);
                }
            }
        }
    }
}

/******************************************************************************/
/******************************************************************************/
