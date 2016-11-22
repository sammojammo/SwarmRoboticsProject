#ifndef ARENA_H_ 
#define ARENA_H_ 

/******************************************************************************/
/******************************************************************************/

class CArena;

/******************************************************************************/
/******************************************************************************/

#include "common.h"
#include "agent.h"

/******************************************************************************/
/******************************************************************************/

class CArena : public CSimObject
{
public:
    CArena(const char* pch_name, 
           double f_size_x,       
           double f_size_y, 
           unsigned int un_res_x, 
           unsigned int un_res_y);
    virtual ~CArena();

    virtual void GetSize(double* pf_size_x, double* pf_size_y) const;
    virtual void GetResolution(unsigned int* pun_res_x, unsigned int* pun_res_y) const;

    virtual void GetAgentsCloseTo(TAgentListList* pt_output_list, const TVector2d* pt_position, double f_radius);

    virtual void AddAgent(CAgent* pc_agent, TVector2d* pt_new_position);
    virtual void RemoveAgent(CAgent* pc_agent);
    virtual void MoveAgent(CAgent* pc_agent, TVector2d* pt_new_position);

    virtual bool IsObstacle(TVector2d* t_position) = 0;
    
    static bool g_bIsBoundless;

protected:
    unsigned int XYToArrayPosition(const TVector2d* pt_position) const;
    unsigned int XYToArrayPosition(double f_x, double f_y) const;
    void         XYToArrayXY(double f_x, double f_y, unsigned int* pun_x, unsigned int* pun_y) const;

    TAgentListIterator FindAgent(TAgentList* plist_agents, CAgent* pc_agent) const;

    virtual void RemoveAgent(CAgent* pc_agent, unsigned int un_array_position);
    virtual void AddAgent(CAgent* pc_agent, unsigned int un_array_position);


protected:
    double       m_fSizeX;
    double       m_fSizeY;

    unsigned int m_unResX;
    unsigned int m_unResY;

    // A m_unResX * m_unResY array of agents in the corresponding squares.
    TAgentList*   m_plistAgents;

    unsigned int             m_unNumberOfCytokineConcentrations;    
};

/******************************************************************************/
/******************************************************************************/

#endif
