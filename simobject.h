/******************************************************************************

This is the super-class for almost all simulator related classes. It
is quite simple. It simply contains a couple of public accessible
methods. Its main feature is that it uses recursive parent-child
relationships. This allows for events such as keypresses, a new
simulation cycle, and rendering to be done recursively.

It also allows all objects to have distinct names - and invaluable
tool for debugging.

*******************************************************************************/

#ifndef SIMOBJECT_H_
#define SIMOBJECT_H_

/******************************************************************************/
/******************************************************************************/

#include "common.h"

/******************************************************************************/
/******************************************************************************/

class CSimObject;
class CRender;

typedef vector<CSimObject*>           TSimObjectVector;
typedef vector<CSimObject*>::iterator TSimObjectVectorIterator;


/******************************************************************************/
/******************************************************************************/

class CSimObject 
{
public:
    CSimObject(const char* pch_name);
    virtual ~CSimObject();
    const char* GetName() const;

    virtual void Draw(CRender* pc_render);
    virtual void SimulationStep(unsigned int un_step_number);
    virtual void Keypressed(int keycode);

    virtual void AddChild(CSimObject* pc_child);
    virtual void RemoveChild(CSimObject* pc_child);

    virtual void PrintfChildren(unsigned indent);

    virtual TSimObjectVector* GetChildren();

    static bool g_bShuffleChildren;

protected:
    char*             m_pchName;    
    TSimObjectVector  m_vecSimObjectChildren;
};

/******************************************************************************/
/******************************************************************************/


#endif
