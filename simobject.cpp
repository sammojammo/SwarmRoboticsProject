#include "simobject.h"


/******************************************************************************/
/******************************************************************************/

bool CSimObject::g_bShuffleChildren = false;

/******************************************************************************/
/******************************************************************************/

CSimObject::CSimObject(const char* pch_name) 
{
    if (pch_name != NULL)
    {
        m_pchName = (char*) malloc(strlen(pch_name) + 1);
        strcpy(m_pchName, pch_name);
    } else {
        m_pchName = NULL;
    }
}

/******************************************************************************/
/******************************************************************************/

CSimObject::~CSimObject() 
{
    // Delete children:
    TSimObjectVectorIterator i = m_vecSimObjectChildren.begin();
    for (i = m_vecSimObjectChildren.begin(); i != m_vecSimObjectChildren.end(); i++)
    {        
        delete (*i);
    }


    if (m_pchName) 
        free(m_pchName);
}

/******************************************************************************/
/******************************************************************************/

const char* CSimObject::GetName() const
{
    return m_pchName;
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::Draw(CRender* pc_render)
{
    TSimObjectVectorIterator i = m_vecSimObjectChildren.begin();

    for (i = m_vecSimObjectChildren.begin(); i != m_vecSimObjectChildren.end(); i++)
    {
        (*i)->Draw(pc_render);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::SimulationStep(unsigned int un_step_number)
{
    TSimObjectVectorIterator i = m_vecSimObjectChildren.begin();



    for (i = m_vecSimObjectChildren.begin(); i != m_vecSimObjectChildren.end(); i++)
    {
        (*i)->SimulationStep(un_step_number);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::Keypressed(int keycode)
{
    TSimObjectVectorIterator i = m_vecSimObjectChildren.begin();

    for (i = m_vecSimObjectChildren.begin(); i != m_vecSimObjectChildren.end(); i++)
    {
        (*i)->Keypressed(keycode);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::AddChild(CSimObject* pc_child)
{
    m_vecSimObjectChildren.push_back(pc_child);
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::RemoveChild(CSimObject* pc_child)
{
    TSimObjectVectorIterator i = m_vecSimObjectChildren.begin();

    while (i != m_vecSimObjectChildren.end() && (*i) != pc_child)
        i++;

    if (i == m_vecSimObjectChildren.end())
    {
        ERROR2("%s tried to remove a non-existing child %s", GetName(), pc_child->GetName());
    } else {
        m_vecSimObjectChildren.erase(i);
    }
}

/******************************************************************************/
/******************************************************************************/

void CSimObject::PrintfChildren(unsigned indent)
{
    for (int j = 0; j < indent; j++)
        printf(" ");

    if (m_pchName)
        printf("%s\n", GetName());
    else
        printf("NULL\n", GetName());
        
    TSimObjectVectorIterator i = m_vecSimObjectChildren.begin();
    for (i = m_vecSimObjectChildren.begin(); i != m_vecSimObjectChildren.end(); i++)
    {
        (*i)->PrintfChildren(indent + 2);
    }
}

/******************************************************************************/
/******************************************************************************/

TSimObjectVector* CSimObject::GetChildren()
{
    return &m_vecSimObjectChildren;
}

/******************************************************************************/
/******************************************************************************/
