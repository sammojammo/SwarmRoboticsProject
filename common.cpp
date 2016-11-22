#include "common.h"
#include <math.h>
#include "arena.h"
#include "simulator.h"

double GetDistanceBetweenPositions(const TVector2d* pt_pos1, const TVector2d* pt_pos2)
{
    if (CArena::g_bIsBoundless)
    {
        CArena* pcArena = CSimulator::GetInstance()->GetArena();
        double fSizeX;
        double fSizeY;
        pcArena->GetSize(&fSizeX, &fSizeY);

        double fX = fabs(pt_pos1->x - pt_pos2->x);
        if (fX > fSizeX / 2.0)
        {
            fX = fSizeX - fX;
        }
        double fY = fabs(pt_pos1->y - pt_pos2->y);
        if (fY > fSizeY / 2.0)
        {
            fY = fSizeY - fY;
        }

        return sqrt(fX * fX + fY * fY);
    }
    {
        double fX = pt_pos1->x - pt_pos2->x;
        double fY = pt_pos1->y - pt_pos2->y;

        return sqrt(fX * fX + fY * fY);
    }
}


double GetSquaredDistanceBetweenPositions(const TVector2d* pt_pos1, const TVector2d* pt_pos2)
{
    if (CArena::g_bIsBoundless)
    {
        CArena* pcArena = CSimulator::GetInstance()->GetArena();
        double fSizeX;
        double fSizeY;
        pcArena->GetSize(&fSizeX, &fSizeY);

        double fX = fabs(pt_pos1->x - pt_pos2->x);
        if (fX > fSizeX / 2.0)
        {
            fX = fSizeX - fX;
        }
        double fY = fabs(pt_pos1->y - pt_pos2->y);
        if (fY > fSizeY / 2.0)
        {
            fY = fSizeY - fY;
        }

        return fX * fX + fY * fY;
    }
    {
        double fX = pt_pos1->x - pt_pos2->x;
        double fY = pt_pos1->y - pt_pos2->y;

        return fX * fX + fY * fY;
    }
}
