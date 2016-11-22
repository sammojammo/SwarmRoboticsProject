/******************************************************************************

SbotTrace is a simple class to write read and write sbottrace files

*******************************************************************************/

#ifndef SBOTTRACE_H_
#define SBOTTRACE_H_

/*******************************************************************************/
/*******************************************************************************/

#include <vector>
#include <stdio.h>
#include <stdlib.h>

/*******************************************************************************/
/*******************************************************************************/

using namespace std;

/*******************************************************************************/
/*******************************************************************************/

typedef struct {

    char    pchSbotName[128];

    float   fPositionX, fPositionY, fPositionZ;
    float   fRotationX, fRotationY, fRotationZ;

    float   fTurretRotation;

    unsigned int unGripperAperture;

    float   fLeftTreelSpeed, fRightTreelSpeed;

    unsigned char pchLedsRGB[8][3];

    unsigned int  unSoundFrequency;

    float  fUserSpecific1;
    int    nUserSpecific2;

    float   fTime;
} TSbotStateDescription;

/*******************************************************************************/
/*******************************************************************************/

typedef struct {

    char    pchStoyName[128];
    
    float   fRadius;
    unsigned int unNumberOfLeds;

    float   fPositionX, fPositionY, fPositionZ;
    float   fRotationX, fRotationY, fRotationZ;

    unsigned char pchLedsRGB[3];
    float   fTime;

} TStoyStateDescription;


/*******************************************************************************/
/*******************************************************************************/

class CSbotTrace 
{
public:
    CSbotTrace();
    virtual ~CSbotTrace();
   
    virtual TSbotStateDescription* GetNewEmptySbotStateDescription(const char* pch_sbot_name);
    virtual TStoyStateDescription* GetNewEmptyStoyStateDescription(const char* pch_stoy_name);
      
    virtual float GetTime();
    
    virtual unsigned int GetNumberOfFrames();
    virtual bool GotoNextFrame();
    virtual bool GotoFrame(unsigned int un_frame_number);
    virtual void StartNewFrame(float f_time);
    virtual bool GotoTime(float f_time);
    
    virtual TSbotStateDescription* GetNextSbotStateDescription(); 
    virtual TStoyStateDescription* GetNextStoyStateDescription(); 
       
    virtual void AddSbotStateDescription(TSbotStateDescription* pt_sbot_state_description);    
    virtual void AddStoyStateDescription(TStoyStateDescription* pt_stoy_state_description);    

    virtual bool Load(const char* pch_filename);
    virtual void Save(const char* pch_filename);
    virtual void Append(const char* pch_filename);
    virtual void Clear();

protected:
    void WriteStatesToFile(FILE* file);

    char* GetAttribute(const char* pch_tag, char* pch_buffer, int n_buffer_size);
    unsigned int FindNextLine(const char* pch_buffer, int n_size);


protected:
    vector<vector<TSbotStateDescription*> >  m_vecSbotFrames;
    vector<vector<TStoyStateDescription*> >  m_vecStoyFrames;
    
    unsigned int m_unCurrentSbotState;
    unsigned int m_unCurrentStoyState;
    unsigned int m_unCurrentFrame;
    float        m_fCurrentTime;
};

/*******************************************************************************/
/*******************************************************************************/

#endif
