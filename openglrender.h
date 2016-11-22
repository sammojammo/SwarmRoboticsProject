#ifndef OPENGLRENDER_H_
#define OPENGLRENDER_H_

/******************************************************************************/
/******************************************************************************/

#include "common.h"
#include "simobject.h"
#include "render.h"
#include "agent.h"

/******************************************************************************/
/******************************************************************************/

#include <X11/Xlib.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>
#include <GL/glx.h>

#ifdef WITHSBOTTRACE
#include "sbottrace.h" 
#endif

/******************************************************************************/
/******************************************************************************/

class COpenGLRender : public CRender
{
public:
    COpenGLRender(const char* pch_agent_color, unsigned int un_number_of_agents, unsigned int un_number_of_simulation_steps);
    virtual ~COpenGLRender();

    virtual void SimulationStep(unsigned int un_step_number);
    virtual void SetOutputStatistics(bool on_off);

protected:
    virtual void StartGraphics();
    virtual void StopGraphics();
    virtual void DrawFrame();

    virtual void HandleEvent (XEvent& event);

    virtual void CaptureFrame (int num);

    virtual void DrawAllAgents();
    virtual void DrawAgent(CAgent* pc_agent, unsigned int un_agent_number);

    virtual void DrawCircle(double f_center_x, double f_center_y, double f_radius);
    virtual void DrawSolidCircle(double f_center_x, double f_center_y, double f_radius);

    virtual void OutputStatistics(unsigned int un_step_number);

    virtual TColor3f GetColorFromIndex(unsigned int index);

protected:
    virtual void GenerateColors();

    double    m_fCurrentFrame;
    int       m_nWindowWidth;
    int       m_nWindowHeight;

    double    m_fDetailLevel;

    int       m_nCurrentFileFrame;

    unsigned int m_unNumberOfAgents;
    unsigned int m_unNumberOfPhysicalLinks;
    unsigned int m_unMaximumNumberOfPhysicalLinks;

    bool m_bOutputStatistics;

    unsigned int m_unNumberOfRobotAgents;
    unsigned int m_unNumberOfLightAgents;
    
    unsigned int        m_unNumberOfColors;
    TColor3f*           m_ptColors;

    bool            m_bSetAgentColorsFromFile;
    unsigned int**  m_ppunColors;

#ifdef WITHSBOTTRACE
    CSbotTrace*     m_pcSbotTrace;
#endif
};

/******************************************************************************/
/******************************************************************************/

#endif
