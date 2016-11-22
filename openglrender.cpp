#include "openglrender.h"
#include "simulator.h"
#include "random.h"


//#include <config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include <fstream>  
#include <iostream>  

using namespace std;

#define NEUTRAL   0
#define TOLERATED 1
#define ATTACKED  2



#define Vec2dRotate(angle, vec)                         \
  {                                                     \
     double xt_ = vec.x;                                 \
     vec.x = cos(angle) * vec.x - sin(angle) * vec.y;   \
     vec.y = cos(angle) * vec.y + sin(angle) * xt_;     \
  }


// Find the angle of one vector
#define Vec2dOwnAngle(vec) \
     (atan2(vec.y, vec.x))


// X11 display info
static Display *display=0;
static int screen=0;
static XVisualInfo *visual=0;		// best visual for openGL
static Colormap colormap=0;		// window's colormap
static Atom wm_protocols_atom = 0;
static Atom wm_delete_window_atom = 0;

// window and openGL
static Window win=0;			// X11 window, 0 if not initialized
static GLXContext glx_context=0;	// openGL rendering context
static int last_key_pressed=0;		// last key pressed in the window
static int run=1;			// 1 if simulation running
static int singlestep=0;		// 1 if single step key pressed
static int writeframes=0;		// 1 if frame files to be written
static GC  gc;

/******************************************************************************/
/******************************************************************************/

COpenGLRender::COpenGLRender(const char* pch_agent_color_filename, unsigned int un_number_of_agents, unsigned int un_number_of_simulation_steps) : CRender("OpenGLRender")
{
    if (pch_agent_color_filename)
    {
        m_bSetAgentColorsFromFile = true;
        m_ppunColors = new unsigned int*[un_number_of_simulation_steps];
        for (unsigned int i = 0; i < un_number_of_simulation_steps; i++)
        {
            m_ppunColors[i] = new unsigned int[un_number_of_agents];
            for (unsigned int j = 0; j < un_number_of_agents; j++)
            {
                m_ppunColors[i][j] = NEUTRAL;
            }
        }

        unsigned int step;
        unsigned int id;
        unsigned int fv;
        unsigned int tol;
        unsigned int attck;
        unsigned int nbrswithfv;

        ifstream myfile (pch_agent_color_filename);
        
        if (!myfile.is_open())
        {
            ERROR1("Unable to open %s for reading", pch_agent_color_filename);
            exit(0);

        }

        while (! myfile.eof() )
        {
            myfile >> step >> id >> fv >> tol >> attck >> nbrswithfv;            

            if (tol > attck)
                m_ppunColors[step][id] = TOLERATED;
            else if (tol < attck)                
                m_ppunColors[step][id] = ATTACKED;
            else 
                m_ppunColors[step][id] = NEUTRAL;
        }
    } else {
        m_bSetAgentColorsFromFile = false;
    }
    
    m_nWindowWidth  = 640;
    m_nWindowHeight  = 480;

    m_fDetailLevel   = 20.0;

    m_nCurrentFileFrame = 0;
    m_bOutputStatistics = true;
    
    //TODO:
    m_unNumberOfColors = 100; 
    GenerateColors();

    StartGraphics();
}

/******************************************************************************/
/******************************************************************************/

COpenGLRender::~COpenGLRender() 
{
    StopGraphics();
    delete[] m_ptColors;
}

/******************************************************************************/
/******************************************************************************/

void COpenGLRender::StartGraphics()
{
//    pause = initial_pause;
    // create X11 display connection
    display = XOpenDisplay (NULL);
    if (!display) ERROR("Can not open X11 display");
    screen = DefaultScreen(display);

    // get GL visual
    static int attribList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE,16,
                               GLX_RED_SIZE,4, GLX_GREEN_SIZE,4,
                               GLX_BLUE_SIZE,4, None};
    visual = glXChooseVisual (display,screen,attribList);
    if (!visual) 
        ERROR ("no good X11 visual found for OpenGL");

    // create colormap
    colormap = XCreateColormap (display,RootWindow(display, screen),
                                visual->visual, AllocNone);

    // initialize variables
    win = 0;
    glx_context = 0;
    last_key_pressed = 0;

    if (m_nWindowWidth < 1 || m_nWindowHeight < 1) 
        ERROR("Bad window width or height");

    // create the window
    XSetWindowAttributes attributes;
    attributes.background_pixel = BlackPixel(display,screen);
    attributes.colormap = colormap;
    attributes.event_mask = ButtonPressMask | ButtonReleaseMask |
        KeyPressMask | KeyReleaseMask | ButtonMotionMask | PointerMotionHintMask |
        StructureNotifyMask;
    win = XCreateWindow (display,
                         RootWindow(display,screen), 
                         50, 
                         50, 
                         m_nWindowWidth, 
                         m_nWindowHeight,
                         0,visual->depth, 
                         InputOutput,
                         visual->visual,
                         CWBackPixel | CWColormap | CWEventMask,&attributes);

    // associate a GLX context with the window
    glx_context = glXCreateContext (display,visual,0,GL_TRUE);
    if (!glx_context) 
        ERROR ("can't make an OpenGL context");

    // set the window title
    XTextProperty window_name;
    window_name.value = (unsigned char *) "Simulation";
    window_name.encoding = XA_STRING;
    window_name.format = 8;
    window_name.nitems = strlen((char *) window_name.value);
    XSetWMName (display,win,&window_name);

    // participate in the window manager 'delete yourself' protocol
    wm_protocols_atom = XInternAtom (display,"WM_PROTOCOLS",False);
    wm_delete_window_atom = XInternAtom (display,"WM_DELETE_WINDOW",False);
    if (XSetWMProtocols (display,win,&wm_delete_window_atom,1)==0)
        ERROR ("XSetWMProtocols() call failed");

    // pop up the window
    XMapWindow (display,win);
    XSync (display,win);
    glXMakeCurrent (display, win, glx_context);
  
    m_fCurrentFrame = 1;
        
    int frame = 1;        
    gc = XCreateGC(display, win, 0,0);

#ifdef WITHSBOTTRACE
    m_pcSbotTrace = new CSbotTrace();
#endif
}

/******************************************************************************/
/******************************************************************************/

void COpenGLRender::StopGraphics()
{
    glXDestroyContext (display,glx_context);
    XDestroyWindow (display,win);
    XSync (display,0);
    display = 0;
    win = 0;
    glx_context = 0;

#ifdef WITHSBOTTRACE
    m_pcSbotTrace->Save("sbottrace.xml");
#endif // WITHSBOTTRACE

}


/******************************************************************************/
/******************************************************************************/

void COpenGLRender::SimulationStep(unsigned int un_step_number)
{
    XEvent event;
    while (XPending (display)) 
    {
        XNextEvent (display,&event);
        HandleEvent (event);
    }

    m_fCurrentFrame -= 1;

    while (m_fCurrentFrame <= 0)
    {    
#ifdef WITHSBOTTRACE
        m_pcSbotTrace->StartNewFrame(un_step_number);
#endif // WITHSBOTTRACE
        DrawFrame();

        if (m_bOutputStatistics)
        {
            OutputStatistics(un_step_number);
        }
        
        if (writeframes) 
        {
            m_nCurrentFileFrame++;
            CaptureFrame(m_nCurrentFileFrame);
        }        
        
        glFlush();
        glXSwapBuffers (display,win);
        XSync (display,0);

        m_fCurrentFrame += m_fFrameRate;

    }
}

/******************************************************************************/
/******************************************************************************/

void COpenGLRender::DrawFrame()
{	
    glViewport (0, 0, m_nWindowWidth, m_nWindowHeight);

    glClear(GL_COLOR_BUFFER_BIT);

    // Set the drawing color
	glColor3f(1.0, 1.0, 1.0);
    
	// Specify which primitive type is to be drawn
    DrawAllAgents();
//  DrawConcentrations();
}

/******************************************************************************/
/******************************************************************************/

void COpenGLRender::HandleEvent (XEvent &event)
{
    static int mx=0, my=0; 	// mouse position
    static int mode = 0;		// mouse button bits

    switch (event.type) {

    case ButtonPress: {
        if (event.xbutton.button == Button1) mode |= 1;
        if (event.xbutton.button == Button2) mode |= 2;
        if (event.xbutton.button == Button3) mode |= 4;
        mx = event.xbutton.x;
        my = event.xbutton.y;
    }
        return;

    case ButtonRelease: {
        if (event.xbutton.button == Button1) mode &= (~1);
        if (event.xbutton.button == Button2) mode &= (~2);
        if (event.xbutton.button == Button3) mode &= (~4);
        mx = event.xbutton.x;
        my = event.xbutton.x;
    }
        return;

    case MotionNotify: {
        if (event.xmotion.is_hint) {
            Window root,child;
            unsigned int mask;
            XQueryPointer (display,win,&root,&child,&event.xbutton.x_root,
                           &event.xbutton.y_root,&event.xbutton.x,&event.xbutton.y,
                           &mask);
        }
//        dsMotion (mode, event.xmotion.x - mx, event.xmotion.y - my);
        mx = event.xmotion.x;
        my = event.xmotion.y;
    }
        return;

    case KeyPress: {
        KeySym key;
        XLookupString (&event.xkey,NULL,0,&key,0);
        if ((event.xkey.state & ControlMask) == 0) {
            if (key >= ' ' && key <= 126) 
            {
                switch (key) 
                {
                case '+': m_fFrameRate *= 1.25; break;
                case '-': m_fFrameRate /= 1.20; break;
                case 'i': m_bOutputStatistics = !m_bOutputStatistics; break;
                }
            }
                ;
        }
        else if (event.xkey.state & ControlMask) {
            switch (key) {
            case 't': case 'T':
//                dsSetTextures (dsGetTextures() ^ 1);
                break;
            case 's': case 'S':
//                dsSetShadows (dsGetShadows() ^ 1);
                break;
            case 'x': case 'X':
                CSimulator::GetInstance()->EndSimulation();
#ifdef WITHSBOTTRACE
                m_pcSbotTrace->Save("sbottrace.xml");
#endif // WITHSBOTTRACE
                break;
            case 'p': case 'P':
//                pause ^= 1;
                singlestep = 0;
                break;
            case 'o': case 'O':
//                if (pause) singlestep = 1;
                break;
            case 'v': case 'V': {
                float xyz[3],hpr[3];
//                dsGetViewpoint (xyz,hpr);
                printf ("Viewpoint = (%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)\n",
                        xyz[0],xyz[1],xyz[2],hpr[0],hpr[1],hpr[2]);
                break;
            }
            case 'w': case 'W':
                writeframes ^= 1;
                if (writeframes) printf ("Now writing frames to PPM files\n");
                break;
            }
        }
        last_key_pressed = key;		// a kludgy place to put this...
    }
        return;

    case KeyRelease: {
        // hmmmm...
    }
        return;

    case ClientMessage:
        if (event.xclient.message_type == wm_protocols_atom &&
            event.xclient.format == 32 &&
            Atom(event.xclient.data.l[0]) == wm_delete_window_atom) {
            run = 0;
            return;
        }
        return;

    case ConfigureNotify:
        m_nWindowWidth  = event.xconfigure.width;
        m_nWindowHeight = event.xconfigure.height;
        return;
    }
}


// shift x left by i, where i can be positive or negative
#define SHIFTL(x,i) (((i) >= 0) ? ((x) << (i)) : ((x) >> (-i)))

/******************************************************************************/
/******************************************************************************/

// return the index of the highest bit
static int getHighBitIndex (unsigned int x)
{
  int i = 0;
  while (x) {
    i++;
    x >>= 1;
  }
  return i-1;
}

/******************************************************************************/
/******************************************************************************/

void COpenGLRender::CaptureFrame (int num)
{
  fprintf (stderr,"capturing frame %04d\n", num);

  char s[100];
  sprintf (s,"frame/frame%04d.ppm", num);
  FILE *f = fopen (s,"wb");
  if (!f) 
      ERROR1("Can't open \"%s\" for writing",s);
  fprintf (f,"P6\n%d %d\n255\n", m_nWindowWidth, m_nWindowHeight);
  XImage *image = XGetImage (display, win, 0, 0, m_nWindowWidth, m_nWindowHeight, ~0, ZPixmap);

  int rshift = 7 - getHighBitIndex (image->red_mask);
  int gshift = 7 - getHighBitIndex (image->green_mask);
  int bshift = 7 - getHighBitIndex (image->blue_mask);

  for (int y = 0; y < m_nWindowHeight; y++) 
  {
    for (int x = 0; x < m_nWindowWidth; x++) 
    {
      unsigned long pixel = XGetPixel (image, x, y);
      unsigned char b[3];
      b[0] = SHIFTL(pixel & image->red_mask, rshift);
      b[1] = SHIFTL(pixel & image->green_mask, gshift);
      b[2] = SHIFTL(pixel & image->blue_mask, bshift);
      fwrite (b, 3, 1, f);
    }
  }
  fclose (f);
  XDestroyImage (image);
}

/******************************************************************************/
/******************************************************************************/

void COpenGLRender::DrawAllAgents()
{
    TAgentVector* ptAllAgents = CSimulator::GetInstance()->GetAllAgents();
    TAgentVectorIterator i;
    m_unNumberOfRobotAgents    = 0;
    m_unNumberOfLightAgents    = 0;
            
    m_unNumberOfAgents = ptAllAgents->size();
    
    for (i = ptAllAgents->begin(); i != ptAllAgents->end(); i++)
    {
        if ((*i)->GetType() == LIGHT) {
            DrawAgent(*i, m_unNumberOfLightAgents);
            m_unNumberOfLightAgents++;
        }            
    }

    for (i = ptAllAgents->begin(); i != ptAllAgents->end(); i++)
    {
        if ((*i)->GetType() == ROBOT) 
        {
            DrawAgent(*i, m_unNumberOfRobotAgents);
            m_unNumberOfRobotAgents++;            
        } 
    }
}

/******************************************************************************/
/******************************************************************************/

// void RenderString(float x, float y, void *font, const char* string, double red, double green, double blue)
// {  
//   char *c;

//   glColor3f(red, green, blue); 
//   glRasterPos2f(x, y);

//   glutBitmapString(font, (const unsigned char*) string);
// }


/******************************************************************************/
/******************************************************************************/

/*void COpenGLRender::DrawAgent(CAgent* pc_agent)
{

    const TVector2d* ptPosition = pc_agent->GetPosition();

    double fArenaSizeX;
    double fArenaSizeY;

    CSimulator::GetInstance()->GetArena()->GetSize(&fArenaSizeX, &fArenaSizeY);

    if (pc_agent->GetType() == ROBOT) 
    {

        glPointSize((GLfloat) pc_agent->GetSize());
        glBegin(GL_POINTS);
        
        unsigned int unColor = pc_agent->GetColor();
        TColor3f tColor      = GetColorFromIndex(unColor);
        
        glColor3f(tColor.fRed, tColor.fGreen, tColor.fBlue);
        
        glVertex2d(2 * ptPosition->x / fArenaSizeX, 2 * ptPosition->y / fArenaSizeY);
        glEnd();
    } else if (pc_agent->GetType() == LIGHT) {
        glBegin(GL_TRIANGLE_FAN);
        double fCenterX = 2.0 * ptPosition->x / fArenaSizeX;
        double fCenterY = 2.0 * ptPosition->y / fArenaSizeY;
        double fRadius  = 0.0015 *  pc_agent->GetSize();

        unsigned int unColor = pc_agent->GetColor();
        TColor3f tColor      = GetColorFromIndex(unColor);
        glColor3f(tColor.fRed, tColor.fGreen, tColor.fBlue);

        glVertex2f(fCenterX, fCenterY);
        for (double fAngle = PI / 4.0; fAngle <= PI * 3.0 + 0.1; fAngle += PI / 2.0) 
        {
            glVertex2f(fCenterX + sin(fAngle) * fRadius, fCenterY + cos(fAngle) * fRadius);
        }

        glEnd();
    }    
}*/


void COpenGLRender::DrawAgent(CAgent* pc_agent, unsigned int un_agent_number)
{

    const TVector2d* ptPosition = pc_agent->GetPosition();

    double fArenaSizeX;
    double fArenaSizeY;


    CSimulator::GetInstance()->GetArena()->GetSize(&fArenaSizeX, &fArenaSizeY);

    if (pc_agent->GetType() == ROBOT)
    {
        unsigned int unColor = pc_agent->GetColor();
        TColor3f tColor      = GetColorFromIndex(unColor);
        glColor3f(tColor.fRed, tColor.fGreen, tColor.fBlue);

        double fCenterX = 2.0 * ptPosition->x / fArenaSizeX;
        double fCenterY = 2.0 * ptPosition->y / fArenaSizeY;

        glColor3f(0.005, 0.005, 0.005);
        DrawSolidCircle(fCenterX + 0.005, fCenterY - 0.005, CAgent::RADIUS/10.0 * .6); //0.02

        glColor3f(tColor.fRed, tColor.fGreen, tColor.fBlue);
        DrawSolidCircle(fCenterX, fCenterY, CAgent::RADIUS/10.0 * .6); //0.02
        
        if (m_bSetAgentColorsFromFile) {
            unsigned int unStep = CSimulator::GetInstance()->GetSimulationStepNumber();
            if (m_ppunColors[unStep][un_agent_number] == NEUTRAL)
            {
                tColor.fRed   = 1.0;
                tColor.fGreen = 1.0;
                tColor.fBlue  = 1.0;
                glColor3f(1.0, 1.0, 1.0);
            } 
            else if (m_ppunColors[unStep][un_agent_number] == ATTACKED)
            {
                tColor.fRed   = 1.0;
                tColor.fGreen = 0.0;
                tColor.fBlue  = 0.0;
                glColor3f(1.0, 0.0, 0.0);
            }
            else if (m_ppunColors[unStep][un_agent_number] == TOLERATED)
            {
                tColor.fRed   = 0.0;
                tColor.fGreen = 1.0;
                tColor.fBlue  = 0.0;
                glColor3f(0.0, 1.0, 0.0);
            }
        } else {
            glColor3f(1.0, 1.0, 1.0);
        }
        DrawCircle(fCenterX, fCenterY, CAgent::RADIUS/10.0 * .6); //0.02

        double fAngle = Vec2dOwnAngle((*pc_agent->GetVelocity()));
        TVector2d vDot = { 0.0244 * .6, 0.0 }; //x was 0.013
        Vec2dRotate(fAngle, vDot);
        DrawSolidCircle(fCenterX + vDot.x, fCenterY + vDot.y, 0.0131 * .6); //0.007

        glEnd();

#ifdef WITHSBOTTRACE
        char pchTemp[128];
        sprintf(pchTemp, "%x", pc_agent);

        TSbotStateDescription* ptState = m_pcSbotTrace->GetNewEmptySbotStateDescription(pchTemp);
        
        ptState->fPositionX = ptPosition->x;
        ptState->fPositionY = ptPosition->y;
        ptState->fPositionZ = 0;
        
        ptState->fRotationX      = 0;
        ptState->fRotationY      = 0;
        ptState->fRotationZ      = Vec2dOwnAngle((*pc_agent->GetVelocity()));
        ptState->fTurretRotation = Vec2dOwnAngle((*pc_agent->GetVelocity()));
        
        for (int i = 0; i < 8; i++)
        {
            ptState->pchLedsRGB[i][0] = (tColor.fRed * 255.0);
            ptState->pchLedsRGB[i][1] = (tColor.fGreen * 255.0);
            ptState->pchLedsRGB[i][2] = (tColor.fBlue * 255.0);

        }

        ptState->unGripperAperture = 0;
        double fLeft, fRight;
        
        m_pcSbotTrace->AddSbotStateDescription(ptState);  
#endif //WITHSBOTTRACE


    }

    else if (pc_agent->GetType() == LIGHT) {
           glBegin(GL_TRIANGLE_FAN);
           double fCenterX = 2.0 * ptPosition->x / fArenaSizeX;
           double fCenterY = 2.0 * ptPosition->y / fArenaSizeY;
           double fRadius  = 0.0015 *  pc_agent->GetSize();

           unsigned int unColor = pc_agent->GetColor();
           TColor3f tColor      = GetColorFromIndex(unColor);
           glColor3f(tColor.fRed, tColor.fGreen, tColor.fBlue);

           glVertex2f(fCenterX, fCenterY);
           for (double fAngle = PI / 4.0; fAngle <= PI * 3.0 + 0.1; fAngle += PI / 2.0)
           {
               glVertex2f(fCenterX + sin(fAngle) * fRadius, fCenterY + cos(fAngle) * fRadius);
           }

           glEnd();
       }
}

/******************************************************************************/
/******************************************************************************/

void COpenGLRender::DrawCircle(double f_center_x, double f_center_y, double f_radius)
{
    double vectorY1 = f_center_y + f_radius;
    double vectorX1 = f_center_x;
    glBegin(GL_LINE_STRIP);
    for(double angle =0.0f; angle <= (2.0f * 3.14159f); angle += 2.0f*3.14159f / (double)m_fDetailLevel)
    {
        double vectorX1 = f_center_x + (f_radius * (float) sin ((double) angle));
        double vectorY1 = f_center_y + (f_radius * (float) cos ((double) angle));
        glVertex2d(vectorX1, vectorY1);
//        vectorY1 = vectorY;
//        vectorX1 = vectorX;
    }
    glEnd();
}

/******************************************************************************/
/******************************************************************************/

void COpenGLRender::DrawSolidCircle(double f_center_x, double f_center_y, double f_radius)
{
    double vectorY1 = f_center_y + f_radius;
    double vectorX1 = f_center_x;
    glBegin(GL_TRIANGLES);
    for(double angle =0.0f; angle <= (2.0f * 3.14159f); angle += 2.0f*3.14159f / (double) m_fDetailLevel)
    {
        double vectorX = f_center_x + (f_radius * (float) sin ((double) angle));
        double vectorY = f_center_y + (f_radius * (float) cos ((double) angle));
        glVertex2d(f_center_x, f_center_y);
        glVertex2d(vectorX1, vectorY1);
        glVertex2d(vectorX, vectorY);
        vectorY1 = vectorY;
        vectorX1 = vectorX;
    }
    glEnd();
}

/******************************************************************************/
/******************************************************************************/

void COpenGLRender::OutputStatistics(unsigned int un_step_number)
{
    printf("\rStep number: %4d, Frame-rate: %2.2f, Agents: (Robots: %d, lights: %d), links: %d, average degree: %2.2f, max. degree: %d   ", 
           un_step_number, 
           m_fFrameRate, 
           m_unNumberOfRobotAgents, 
           m_unNumberOfLightAgents, 
           m_unNumberOfPhysicalLinks / 2,  
           (double) m_unNumberOfPhysicalLinks / (double) m_unNumberOfAgents,          
           m_unMaximumNumberOfPhysicalLinks);
   

    CArena* pcArena = CSimulator::GetInstance()->GetArena();
}

/******************************************************************************/
/******************************************************************************/

void COpenGLRender::GenerateColors()
{
    long int tempSeed = Random::seed;

    m_ptColors = new TColor3f[m_unNumberOfColors];

    m_ptColors[RED].fRed   = 1;
    m_ptColors[RED].fGreen = 0;
    m_ptColors[RED].fBlue  = 0;

    m_ptColors[GREEN].fRed   = 0;
    m_ptColors[GREEN].fGreen = 1;
    m_ptColors[GREEN].fBlue  = 0;

    m_ptColors[BLUE].fRed   = 0;
    m_ptColors[BLUE].fGreen = 0;
    m_ptColors[BLUE].fBlue  = 1;


    m_ptColors[HALFRED].fRed   = 0.3;
    m_ptColors[HALFRED].fGreen = 0;
    m_ptColors[HALFRED].fBlue  = 0;

    m_ptColors[HALFGREEN].fRed   = 0;
    m_ptColors[HALFGREEN].fGreen = 0.3;
    m_ptColors[HALFGREEN].fBlue  = 0;

    m_ptColors[HALFBLUE].fRed   = 0;
    m_ptColors[HALFBLUE].fGreen = 0;
    m_ptColors[HALFBLUE].fBlue  = 0.3;

    m_ptColors[GREY].fRed   = 0.7;
    m_ptColors[GREY].fGreen = 0.7;
    m_ptColors[GREY].fBlue  = 0.7;

    m_ptColors[YELLOW].fRed   = 1.0;
    m_ptColors[YELLOW].fGreen = 1.0;
    m_ptColors[YELLOW].fBlue  = 0.0;

    for (int i = YELLOW + 1; i < m_unNumberOfColors; i++) 
    {
        m_ptColors[i].fRed   = Random::nextDouble(0.1, 1);
        m_ptColors[i].fGreen = m_ptColors[i].fRed;
        m_ptColors[i].fBlue  = Random::nextDouble(0.1, 1);
    }

    Random::seed = tempSeed;

}

/******************************************************************************/
/******************************************************************************/

TColor3f COpenGLRender::GetColorFromIndex(unsigned int un_index) 
{   
    return m_ptColors[un_index % m_unNumberOfColors];
}

/******************************************************************************/
/******************************************************************************/

void COpenGLRender::SetOutputStatistics(bool on_off) 
{
    m_bOutputStatistics = on_off;
}

/******************************************************************************/
/******************************************************************************/
