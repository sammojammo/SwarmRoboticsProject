#include <string>
#include <string.h>
#include <errno.h>

#include "common.h"
#include "sbottrace.h"

/*******************************************************************************/
/*******************************************************************************/

CSbotTrace::CSbotTrace() :
    m_unCurrentSbotState(0), m_unCurrentStoyState(0), m_unCurrentFrame(0), m_fCurrentTime(0)

{
    
}
 
/*******************************************************************************/
/*******************************************************************************/

CSbotTrace::~CSbotTrace()
{
    Clear();
}
 
/*******************************************************************************/
/*******************************************************************************/

TSbotStateDescription* CSbotTrace::GetNewEmptySbotStateDescription(const char* pch_sbot_name)
{
    TSbotStateDescription* ptState = new TSbotStateDescription;
    
    strcpy(ptState->pchSbotName, pch_sbot_name);

    ptState->fPositionX = ptState->fPositionY = ptState->fPositionZ = 
        ptState->fRotationX = ptState->fRotationY = ptState->fRotationZ = 
        ptState->fTurretRotation  = ptState->fUserSpecific1 = 
        ptState->fLeftTreelSpeed = ptState->fRightTreelSpeed = 0;

    ptState->unSoundFrequency = ptState->nUserSpecific2 = ptState->unGripperAperture = 0;

    ptState->fTime = m_fCurrentTime;

    for (int i = 0; i < 8; i++)
        for (int j = 0; j < 3; j++)           
            ptState->pchLedsRGB[i][j] = 0;

    return ptState;
}
      
/*******************************************************************************/
/*******************************************************************************/


TStoyStateDescription* CSbotTrace::GetNewEmptyStoyStateDescription(const char* pch_stoy_name)
{
    TStoyStateDescription* ptState = new TStoyStateDescription;
    
    strcpy(ptState->pchStoyName, pch_stoy_name);

    ptState->fPositionX = ptState->fPositionY = ptState->fPositionZ = 
        ptState->fRotationX = ptState->fRotationY = ptState->fRotationZ = 0;

    ptState->unNumberOfLeds = 16;
    ptState->fRadius        = 0.15;
    
    ptState->fTime = m_fCurrentTime;

    ptState->pchLedsRGB[0] = 1;
    ptState->pchLedsRGB[1] = 0;
    ptState->pchLedsRGB[2] = 0;

    return ptState;
}
      
/*******************************************************************************/
/*******************************************************************************/

bool  CSbotTrace::GotoTime(float f_time)
{
    m_fCurrentTime = f_time;
}

/*******************************************************************************/
/*******************************************************************************/

float CSbotTrace::GetTime()
{
    return m_fCurrentTime;
}

/*******************************************************************************/
/*******************************************************************************/
    
unsigned int CSbotTrace::GetNumberOfFrames()
{
    return m_vecSbotFrames.size();
}

/*******************************************************************************/
/*******************************************************************************/

bool CSbotTrace::GotoNextFrame()
{
    if (m_vecSbotFrames.size() < m_unCurrentFrame + 1)
    {
        m_unCurrentFrame++;
        m_unCurrentSbotState = 0;    
        m_unCurrentStoyState = 0;    
        return true;
    }        

    return false;
}

/*******************************************************************************/
/*******************************************************************************/

bool CSbotTrace::GotoFrame(unsigned int un_frame_number)
{
    if (un_frame_number >= m_vecSbotFrames.size())
    {
        return false;
    } else {
        m_unCurrentFrame     = un_frame_number;
        m_unCurrentSbotState = 0;
        m_unCurrentStoyState = 0;
        return true;
    }
}

/*******************************************************************************/
/*******************************************************************************/

void CSbotTrace::StartNewFrame(float f_time)
{
    m_fCurrentTime = f_time;

    vector<TSbotStateDescription*> vecNewSbot;
    m_vecSbotFrames.push_back(vecNewSbot);

    vector<TStoyStateDescription*> vecNewStoy;
    m_vecStoyFrames.push_back(vecNewStoy);

    m_unCurrentFrame = m_vecSbotFrames.size() - 1;
    m_unCurrentSbotState = 0;
    m_unCurrentStoyState = 0;
}

/*******************************************************************************/
/*******************************************************************************/

TSbotStateDescription* CSbotTrace::GetNextSbotStateDescription()
{
    if (m_unCurrentSbotState < m_vecSbotFrames[m_unCurrentFrame].size())
    {
        return m_vecSbotFrames[m_unCurrentFrame][m_unCurrentSbotState++];
    } else {
        return NULL;
    }
}
       
/*******************************************************************************/
/*******************************************************************************/

TStoyStateDescription* CSbotTrace::GetNextStoyStateDescription()
{
    if (m_unCurrentStoyState < m_vecStoyFrames[m_unCurrentFrame].size())
    {
        return m_vecStoyFrames[m_unCurrentFrame][m_unCurrentStoyState++];
    } else {
        return NULL;
    }
}
       
/*******************************************************************************/
/*******************************************************************************/

void CSbotTrace::AddSbotStateDescription(TSbotStateDescription* pt_sbot_state_description)
{
    m_vecSbotFrames[m_unCurrentFrame].push_back(pt_sbot_state_description);
}

/*******************************************************************************/
/*******************************************************************************/

void CSbotTrace::AddStoyStateDescription(TStoyStateDescription* pt_stoy_state_description)
{
    m_vecStoyFrames[m_unCurrentFrame].push_back(pt_stoy_state_description);
}

/*******************************************************************************/
/*******************************************************************************/


bool CSbotTrace::Load(const char* pch_filename)
{    
    int nFileSize;
    FILE* file = fopen(pch_filename, "rb");
   
    if (!file)
    {
        fprintf(stderr, "ERROR: Cannot create/open %s, error code %d (%s)\n", pch_filename, errno, strerror(errno));
        return false;
    }

    fseek(file, 0, SEEK_END);
    nFileSize = ftell(file);    
    fseek(file, 0, SEEK_SET);

    if (nFileSize == 0)
    {
        fprintf(stderr, "ERROR: File %s is empty\n", pch_filename);
        return false;
    }
        
    Clear();

    char* pchBuffer = new char[nFileSize];
    int nBytesRead = fread(pchBuffer, 1, nFileSize, file);
    if (nBytesRead != nFileSize)
    {
        fprintf(stderr, "ERROR: Read %d bytes, should have been %d\n", nBytesRead, nFileSize);
        delete pchBuffer; 
        return false;        
    }

    int    nBytesLeft    = nBytesRead;
    int    nCurrentIndex = 0;
    bool   bAllLoaded    = false;
    int    nFrame        = 0;
    do {
        // Load frame:
        int nNumberOfSbotStates = atoi(GetAttribute("sbotstates", &pchBuffer[nCurrentIndex], nBytesLeft));
        int nNumberOfStoyStates = atoi(GetAttribute("stoystates", &pchBuffer[nCurrentIndex], nBytesLeft));

        fprintf(stderr, "\rFrame: %5d, s-bots: %d, s-toys: %d", ++nFrame, nNumberOfSbotStates, nNumberOfStoyStates);
        fflush(stderr);

        float fTime             = atof(GetAttribute("time", &pchBuffer[nCurrentIndex], nBytesLeft));

        int nNextLineOffset  = FindNextLine(&pchBuffer[nCurrentIndex], nBytesLeft);
        nCurrentIndex += nNextLineOffset;
        nBytesLeft    -= nNextLineOffset;

        StartNewFrame(fTime);
        
        for (int i = 0; i < nNumberOfSbotStates; i++)
        {
            TSbotStateDescription* ptState = GetNewEmptySbotStateDescription(GetAttribute("name", &pchBuffer[nCurrentIndex], nBytesLeft));
            
            ptState->fPositionX = atof(GetAttribute("x", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->fPositionY = atof(GetAttribute("y", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->fPositionZ = atof(GetAttribute("z", &pchBuffer[nCurrentIndex], nBytesLeft));
            
            ptState->fRotationX = atof(GetAttribute("rotx", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->fRotationY = atof(GetAttribute("roty", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->fRotationZ = atof(GetAttribute("rotz", &pchBuffer[nCurrentIndex], nBytesLeft));

            ptState->fTurretRotation   = atof(GetAttribute("turretrot", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->unGripperAperture = atoi(GetAttribute("gripperaperture", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->fLeftTreelSpeed   = atof(GetAttribute("lefttweel", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->fRightTreelSpeed  = atof(GetAttribute("righttweel", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->unSoundFrequency  = atoi(GetAttribute("soundfrequency", &pchBuffer[nCurrentIndex], nBytesLeft));

            ptState->fUserSpecific1   = atof(GetAttribute("user1", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->nUserSpecific2   = atoi(GetAttribute("user2", &pchBuffer[nCurrentIndex], nBytesLeft));
            
            // Ohh the lazyness and copy-paste!
            ptState->pchLedsRGB[0][0] = atoi(GetAttribute("led0r", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[0][1] = atoi(GetAttribute("led0g", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[0][2] = atoi(GetAttribute("led0b", &pchBuffer[nCurrentIndex], nBytesLeft));
            
            ptState->pchLedsRGB[1][0] = atoi(GetAttribute("led1r", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[1][1] = atoi(GetAttribute("led1g", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[1][2] = atoi(GetAttribute("led1b", &pchBuffer[nCurrentIndex], nBytesLeft));
            
            ptState->pchLedsRGB[2][0] = atoi(GetAttribute("led2r", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[2][1] = atoi(GetAttribute("led2g", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[2][2] = atoi(GetAttribute("led2b", &pchBuffer[nCurrentIndex], nBytesLeft));
            
            ptState->pchLedsRGB[3][0] = atoi(GetAttribute("led3r", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[3][1] = atoi(GetAttribute("led3g", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[3][2] = atoi(GetAttribute("led3b", &pchBuffer[nCurrentIndex], nBytesLeft));

            ptState->pchLedsRGB[4][0] = atoi(GetAttribute("led4r", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[4][1] = atoi(GetAttribute("led4g", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[4][2] = atoi(GetAttribute("led4b", &pchBuffer[nCurrentIndex], nBytesLeft));

            ptState->pchLedsRGB[5][0] = atoi(GetAttribute("led5r", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[5][1] = atoi(GetAttribute("led5g", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[5][2] = atoi(GetAttribute("led5b", &pchBuffer[nCurrentIndex], nBytesLeft));

            ptState->pchLedsRGB[6][0] = atoi(GetAttribute("led6r", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[6][1] = atoi(GetAttribute("led6g", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[6][2] = atoi(GetAttribute("led6b", &pchBuffer[nCurrentIndex], nBytesLeft));

            ptState->pchLedsRGB[7][0] = atoi(GetAttribute("led7r", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[7][1] = atoi(GetAttribute("led7g", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[7][2] = atoi(GetAttribute("led7b", &pchBuffer[nCurrentIndex], nBytesLeft));

            AddSbotStateDescription(ptState);
            
            nNextLineOffset  = FindNextLine(&pchBuffer[nCurrentIndex], nBytesLeft);
            nCurrentIndex += nNextLineOffset;
            nBytesLeft    -= nNextLineOffset;        
        }



        for (int i = 0; i < nNumberOfStoyStates; i++)
        {
            TStoyStateDescription* ptState = GetNewEmptyStoyStateDescription(GetAttribute("name", &pchBuffer[nCurrentIndex], nBytesLeft));
            
            ptState->fPositionX = atof(GetAttribute("x", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->fPositionY = atof(GetAttribute("y", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->fPositionZ = atof(GetAttribute("z", &pchBuffer[nCurrentIndex], nBytesLeft));
            
            ptState->fRotationX = atof(GetAttribute("rotx", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->fRotationY = atof(GetAttribute("roty", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->fRotationZ = atof(GetAttribute("rotz", &pchBuffer[nCurrentIndex], nBytesLeft));

            ptState->fRadius          = atof(GetAttribute("radius", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->unNumberOfLeds   = atoi(GetAttribute("numberofleds", &pchBuffer[nCurrentIndex], nBytesLeft));

            // Ohh the lazyness and copy-paste!
            ptState->pchLedsRGB[0] = atoi(GetAttribute("ledr", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[1] = atoi(GetAttribute("ledg", &pchBuffer[nCurrentIndex], nBytesLeft));
            ptState->pchLedsRGB[2] = atoi(GetAttribute("ledb", &pchBuffer[nCurrentIndex], nBytesLeft));

            AddStoyStateDescription(ptState);
            
            nNextLineOffset  = FindNextLine(&pchBuffer[nCurrentIndex], nBytesLeft);
            nCurrentIndex += nNextLineOffset;
            nBytesLeft    -= nNextLineOffset;        
        }

        nNextLineOffset  = FindNextLine(&pchBuffer[nCurrentIndex], nBytesLeft);
        nCurrentIndex += nNextLineOffset;
        nBytesLeft    -= nNextLineOffset;

        if (nNextLineOffset == 0 || nBytesLeft < 10)
        {
            bAllLoaded = true;
        }
        
    } while (!bAllLoaded);
   
    delete pchBuffer; 
}

/*******************************************************************************/
/*******************************************************************************/

void CSbotTrace::Save(const char* pch_filename)
{
    FILE* file = fopen(pch_filename, "w");
    if (!file)
        fprintf(stderr, "ERROR: Cannot create/open %s, error code %d (%s)", pch_filename, errno, strerror(errno));
    else
    {
        WriteStatesToFile(file);
        fclose(file);
    }
}

/*******************************************************************************/
/*******************************************************************************/

void CSbotTrace::Append(const char* pch_filename)
{
    FILE* file = fopen(pch_filename, "a");
    if (!file)
        fprintf(stderr, "ERROR: Cannot create/open %s, error code %d (%s)", pch_filename, errno, strerror(errno));
    else
    {
        WriteStatesToFile(file);
        fclose(file);
    }
}

/*******************************************************************************/
/*******************************************************************************/

void CSbotTrace::Clear()
{
    for (int i = 0; i < m_vecSbotFrames.size(); i++)
    {
        for (int j = 0; j < m_vecSbotFrames[i].size(); j++)
            delete m_vecSbotFrames[i][j];
        
        m_vecSbotFrames[i].clear();
    }

    m_vecSbotFrames.clear();    

    for (int i = 0; i < m_vecStoyFrames.size(); i++)
    {
        for (int j = 0; j < m_vecStoyFrames[i].size(); j++)
            delete m_vecStoyFrames[i][j];
        
        m_vecStoyFrames[i].clear();
    }

    m_vecStoyFrames.clear();    


    m_unCurrentSbotState = 0;
    m_unCurrentStoyState = 0;
    m_unCurrentFrame = 0;
    m_fCurrentTime   = 0;   
}

/*******************************************************************************/
/*******************************************************************************/

void CSbotTrace::WriteStatesToFile(FILE* file)
{
    for (int i = 0; i < m_vecSbotFrames.size(); i++)
    {
        if (m_vecSbotFrames[i].size() > 0)
        {
            fprintf(file, "<frame sbotstates=\"%d\" stoystates=\"%d\" time=\"%2.5f\">\n", m_vecSbotFrames[i].size(), m_vecStoyFrames[i].size(), m_vecSbotFrames[i][0]->fTime);
            
            for (int j = 0; j < m_vecSbotFrames[i].size(); j++)
            {
                TSbotStateDescription* ptState = m_vecSbotFrames[i][j];
                
                fprintf(file, "  <sbot name=\"%s\" x=\"%2.5f\" y=\"%2.5f\" z=\"%2.5f\" " 
                        "rotx=\"%2.5f\" roty=\"%2.5f\" rotz=\"%2.5f\" "
                    "turretrot=\"%2.5f\" gripperaperture=\"%d\" "
                        "lefttweel=\"%f\" righttweel=\"%f\" " 
                        "soundfrequency=\"%d\" " 
                    "user1=\"%2.5f\" " 
                        "user2=\"%d\" ",
                        ptState->pchSbotName,
                        ptState->fPositionX, ptState->fPositionY, ptState->fPositionZ,
                        ptState->fRotationX, ptState->fRotationY, ptState->fRotationZ , 
                        ptState->fTurretRotation, ptState->unGripperAperture, 
                        ptState->fLeftTreelSpeed, ptState->fRightTreelSpeed,
                        ptState->unSoundFrequency, ptState->fUserSpecific1, 
                        ptState->nUserSpecific2);
                for (int k = 0; k < 8; k++)
                { 
                    fprintf(file, "led%dr=\"%d\" led%dg=\"%d\" led%db=\"%d\" ", 
                            k, ptState->pchLedsRGB[k][0], 
                            k, ptState->pchLedsRGB[k][1], 
                            k, ptState->pchLedsRGB[k][2]);
                }
                fprintf(file, "/>\n");
                            
            }

            for (int j = 0; j < m_vecStoyFrames[i].size(); j++)
            {
                TStoyStateDescription* ptState = m_vecStoyFrames[i][j];
                
                fprintf(file, "  <stoy name=\"%s\" x=\"%2.5f\" y=\"%2.5f\" z=\"%2.5f\" " 
                        "rotx=\"%2.5f\" roty=\"%2.5f\" rotz=\"%2.5f\" "
                        "radius=\"%2.5f\" numberofleds=\"%d\" ledr=\"%d\" ledg=\"%d\" ledb=\"%d\"/>\n",
                        ptState->pchStoyName,
                        ptState->fPositionX, ptState->fPositionY, ptState->fPositionZ,
                        ptState->fRotationX, ptState->fRotationY, ptState->fRotationZ, 
                        ptState->fRadius, ptState->unNumberOfLeds, ptState->pchLedsRGB[0], 
                        ptState->pchLedsRGB[1], ptState->pchLedsRGB[2]);                             
            }

            fprintf(file, "</frame>\n");
        }
    }
}

/*******************************************************************************/
/*******************************************************************************/

char* CSbotTrace::GetAttribute(const char* pch_tag, char* pch_buffer, int n_buffer_size)
{
    static char pchTagValue[1024];
    char pchTempTag[1024];
    strcpy(pchTempTag, pch_tag);
    strcat(pchTempTag, "=\"");

    int nCurrentIndex = 0;
    int nTagLength    = strlen(pchTempTag);
   
    bool bTagFound = false;

    while (nCurrentIndex + nTagLength < n_buffer_size && !bTagFound)
    {
        if (strncmp(&pch_buffer[nCurrentIndex], pchTempTag, nTagLength) == 0)
        {
            bTagFound = true;
        } else {
            nCurrentIndex++;
        }
    }

    if (!bTagFound)
    {
        fprintf(stderr, "ERROR: Tag \"%s\" not found while loading s-bot trace - corrupt, incompatible or wrong file?\n", pch_tag);
    }
    else 
    {
        
        nCurrentIndex += nTagLength;
        int nValueIndex = 0;
        while (pch_buffer[nCurrentIndex] != '"' && nCurrentIndex < n_buffer_size)
        {
            pchTagValue[nValueIndex++] = pch_buffer[nCurrentIndex++];
        }

        pchTagValue[nValueIndex] = '\0';
    }
       
    return pchTagValue;
}

/*******************************************************************************/
/*******************************************************************************/

unsigned int CSbotTrace::FindNextLine(const char* pch_buffer, int n_size)
{
    unsigned int nCurrentIndex = 0;
    bool bLineFound = false;

    while (nCurrentIndex < n_size && !bLineFound)
    {
        if (pch_buffer[nCurrentIndex++] == 10)
            bLineFound = true;
    }

    return nCurrentIndex;
}

/*******************************************************************************/
/*******************************************************************************/
