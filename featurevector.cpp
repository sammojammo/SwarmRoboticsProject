#include "featurevector.h"
#include "assert.h"

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::NUMBER_OF_FEATURES        = 7;
unsigned int CFeatureVector::NUMBER_OF_FEATURE_VECTORS = 0;
double       CFeatureVector::FEATURE_RANGE             = 7.0;
//new feature depth variable - number of possible values each feature can have
double       CFeatureVector::FEATURE_DEPTH             = 4.0;

/******************************************************************************/
/******************************************************************************/


CFeatureVector::CFeatureVector(CAgent* pc_agent) : m_pcAgent(pc_agent)
{
    m_unValue  = 0;
    m_unLength = NUMBER_OF_FEATURES;
    m_unCorrectResponse = 0;

    NUMBER_OF_FEATURE_VECTORS = 1 << NUMBER_OF_FEATURES*(int)log2(FEATURE_DEPTH);

    m_pfFeatureValues         = new float[m_unLength];
    m_piLastOccuranceEvent    = new int[m_unLength];
    m_piLastOccuranceNegEvent = new int[m_unLength];

    m_iEventSelectionTimeWindow = MODELSTARTTIME; //1500;
    m_iCorrectResponseTimeWindow = 150;

    for(unsigned int i = 0; i < NUMBER_OF_FEATURES; i++)
    {
        m_piLastOccuranceEvent[i]    = 0;
        m_piLastOccuranceNegEvent[i] = 0;

        m_pfFeatureValues[i]         = 0.0;
    }

//Range of feature values split into bands - feature depth is number of possible values, so must be
    m_pfVelocityBands = new float[(int)FEATURE_DEPTH];
    m_pfAccelerationBands = new float[(int)FEATURE_DEPTH];

    m_pfAngularVelocityBands = new float[(int)FEATURE_DEPTH];
    m_pfAngularAccelerationBands = new float[(int)FEATURE_DEPTH];

    m_pfRelativeVelocityMagBands = new float[(int)FEATURE_DEPTH];
    m_pfRelativeVelocityDirBands = new float[(int)FEATURE_DEPTH];

    m_pfSquaredDistBands = new float[(int)FEATURE_DEPTH];

    m_pfCorrectResponseBands = new float[(int)FEATURE_DEPTH];

    for(int i = 0; i < FEATURE_DEPTH; i++)
    {
        m_pfVelocityBands[i] = (i * (1/FEATURE_DEPTH) * (m_pcAgent->GetMaximumSpeed()));
        m_pfAccelerationBands[i] = i * (1/FEATURE_DEPTH) * (m_pcAgent->GetMaximumSpeed());

        m_pfAngularVelocityBands[i] = i * (1/FEATURE_DEPTH) * (m_pcAgent->GetMaximumAngularVelocity());
        m_pfAngularAccelerationBands[i] = i * (1/FEATURE_DEPTH) * (m_pcAgent->GetMaximumAngularVelocity());

        m_pfRelativeVelocityMagBands[i] = i * (1/FEATURE_DEPTH) * (m_pcAgent->GetMaximumSpeed());
        m_pfRelativeVelocityDirBands[i] = i * (1/FEATURE_DEPTH) * (m_pcAgent->GetMaximumAngularVelocity());

        m_pfCorrectResponseBands[i] = i * (1/FEATURE_DEPTH) * m_iEventSelectionTimeWindow;
    }

    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    m_unNbrsCurrQueueIndex = 0;

    m_unSumTimeStepsNbrsRange0to3 = 0;
    m_unSumTimeStepsNbrsRange3to6 = 0;

    m_punNbrsRange0to3AtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];
    m_punNbrsRange3to6AtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];


    // keeping track of distance travelled by bot in last 100 time-steps
    m_iDistTravelledTimeWindow = 100;/******/
    m_unCoordCurrQueueIndex    = 0;

    m_fSquaredDistTravelled = 0.0;

    for(int i = 0; i < FEATURE_DEPTH; i++)
    {
        m_pfSquaredDistBands[i] = i * (1/FEATURE_DEPTH) * ((m_pcAgent->GetMaximumSpeed() * (double)m_iDistTravelledTimeWindow) *
                                                                (m_pcAgent->GetMaximumSpeed() * (double)m_iDistTravelledTimeWindow));
    }

    m_pvecCoordAtTimeStep = new TVector2d[m_iDistTravelledTimeWindow];
}

/******************************************************************************/
/******************************************************************************/

CFeatureVector::~CFeatureVector()
{
    delete m_pfFeatureValues;

    delete m_piLastOccuranceEvent;
    delete m_piLastOccuranceNegEvent;

    delete m_punNbrsRange0to3AtTimeStep;
    delete m_punNbrsRange3to6AtTimeStep;

    delete m_pvecCoordAtTimeStep;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::GetValue() const
{
    return m_unValue;
}
/******************************************************************************/
/*new function to return value of specific feature, specified by parameter featureNum*/
float CFeatureVector::GetFeatureValue(int featureNum) const
{
    return m_pfFeatureValues[featureNum];
}

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::GetLength() const
{
    return m_unLength;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::SimulationStep()
{
    ComputeFeatureValues();
    m_unValue = 0;

    for(unsigned int i = 0; i < m_unLength; i++)
    {
        m_unValue += (unsigned int)m_pfFeatureValues[i] * (unsigned int)pow(FEATURE_DEPTH,i); // << (FEATURE_DEPTH * i));
    }

}

/******************************************************************************/
/******************************************************************************/

void CFeatureVector::ComputeFeatureValues()
{
    double dist_nbrsagents, angle_acceleration, angle_velocity, mag_velocity, mag_acceleration;
    double mag_relvelocity, dir_relvelocity;
    unsigned int unCloseRangeNbrCount, unFarRangeNbrCount;

    dist_nbrsagents    = m_pcAgent->GetAverageDistanceToSurroundingAgents(FEATURE_RANGE, ROBOT);
    angle_acceleration = m_pcAgent->GetAngularAcceleration();
    angle_velocity     = m_pcAgent->GetAngularVelocity();
    mag_velocity       = Vec2dLength((*m_pcAgent->GetVelocity()));
    mag_acceleration   = Vec2dLength((*m_pcAgent->GetAcceleration()));

    m_pcAgent->GetRelativeVelocity(&mag_relvelocity, &dir_relvelocity, FEATURE_RANGE);

    int CurrentStepNumber = (int) CSimulator::GetInstance()->GetSimulationStepNumber();


    // 6 bit or more feature-vectors
    // Feature (from LS to MS bits in FV)
    // Sensors
    //1st: How long is there at least one neighbor in range 0-3 in past X time steps
    //2nd: How long is there at least one neighbor in range 0-6 in past X time-steps
    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
    {
        // decision based on the last X time-steps

        bool feature0Set = false, feature1Set = false; //Flags to set to the last band if neccessary as the loop won't reach it

        for(int i = 0; i < FEATURE_DEPTH-1; i++)
        {
            if( (m_unSumTimeStepsNbrsRange0to3 >= (i * (1/(FEATURE_DEPTH)) * (double)m_iEventSelectionTimeWindow))
                && (m_unSumTimeStepsNbrsRange0to3 < (i+1) * (1/(FEATURE_DEPTH)) * (double)m_iEventSelectionTimeWindow) )
            {
                m_pfFeatureValues[0] = i;
                feature0Set = true;
                break;
            }

            if ( (m_unSumTimeStepsNbrsRange3to6 >= (i * (1/(FEATURE_DEPTH)) * (double)m_iEventSelectionTimeWindow))
                && (m_unSumTimeStepsNbrsRange3to6 < ((i+1) * (1/(FEATURE_DEPTH)) * (double)m_iEventSelectionTimeWindow)) )
            {
                m_pfFeatureValues[1] = i;
                feature1Set = true;
                break;
            }
        }

        if (feature0Set == false)
            m_pfFeatureValues[0] = FEATURE_DEPTH-1;

        if (feature1Set == false)
            m_pfFeatureValues[1] = FEATURE_DEPTH-1;

        // removing the first entry of the moving time window  from the sum
        m_unSumTimeStepsNbrsRange0to3 -=  m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex];
        m_unSumTimeStepsNbrsRange3to6 -=  m_punNbrsRange3to6AtTimeStep[m_unNbrsCurrQueueIndex];
    }

    // adding new values into the queue
    unCloseRangeNbrCount = m_pcAgent->CountAgents(FEATURE_RANGE/2.0, ROBOT);
    if (unCloseRangeNbrCount > 0)
    {
        m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange0to3++;
    }
    else
        m_punNbrsRange0to3AtTimeStep[m_unNbrsCurrQueueIndex] = 0;

    unFarRangeNbrCount = (m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT) - unCloseRangeNbrCount);
    if (unFarRangeNbrCount > 0)
    {
        m_punNbrsRange3to6AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange3to6++;
    }
    else
        m_punNbrsRange3to6AtTimeStep[m_unNbrsCurrQueueIndex] = 0;


    m_unNbrsCurrQueueIndex = (m_unNbrsCurrQueueIndex + 1) % m_iEventSelectionTimeWindow;



    // Sensors-motor interactions
    // Set if the occurance of the following event, atleast once in time window X
    // 3rd: distance to nbrs 0-6 && change in angular acceleration
    // 4th: no neighbors detected  && change in angular acceleration
/**could change to larger time window**/
/*  old feature 3 and 4 code

    if(dist_nbrsagents < 6.0 &&
            (angle_acceleration > m_tAngularAccelerationThreshold ||
             angle_acceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[2] = CurrentStepNumber;
    }

    if(dist_nbrsagents == 6.0 &&
            (angle_acceleration > m_tAngularAccelerationThreshold ||
             angle_acceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[3] = CurrentStepNumber;
    }

    for(unsigned int featureindex = 2; featureindex <=3; featureindex++)
    {
        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfFeatureValues[featureindex] = 1.0;
        }
        else
        {
            m_pfFeatureValues[featureindex] = 0.0;
        }
    }
*/

    int accelerationBand = 0;
    bool feature2Set = false, feature3Set = false;

    for(int i = 0; i < FEATURE_DEPTH-1; i++)
    {
        if(dist_nbrsagents < 6.0 &&
          ( (angle_acceleration >= m_pfAngularAccelerationBands[i] && angle_acceleration < m_pfAngularAccelerationBands[i+1]) ||
            (angle_acceleration <= -m_pfAngularAccelerationBands[i] && angle_acceleration > -m_pfAngularAccelerationBands[i+1]) ))
        {
            m_piLastOccuranceEvent[2] = CurrentStepNumber;
            accelerationBand = i;
            feature2Set = true;
            break;
        }

        if(dist_nbrsagents == 6.0 &&
          ( (angle_acceleration >= m_pfAngularAccelerationBands[i] && angle_acceleration < m_pfAngularAccelerationBands[i+1]) ||
            (angle_acceleration <= -m_pfAngularAccelerationBands[i] && angle_acceleration > -m_pfAngularAccelerationBands[i+1]) ))
        {
            m_piLastOccuranceEvent[3] = CurrentStepNumber;
            accelerationBand = i;
            feature3Set = true;
            break;
        }
    }

    if(feature2Set == false && feature3Set == false)
        accelerationBand = FEATURE_DEPTH-1;

    for(unsigned int featureindex = 2; featureindex <= 3; featureindex++)
    {
        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfFeatureValues[featureindex] = accelerationBand;
        }
    }

/////////////////////////////////FEATURE 7
//The feature is supposed to test if a robot disperses correctly, ie turning away from the centre of mass of its surrounding agents

    bool feature6set = false;
    double heading = 0;

//get agent velocity
    TVector2d velocityVec;
    velocityVec.x = m_pcAgent->GetVelocity()->x;
    velocityVec.y = m_pcAgent->GetVelocity()->y;

    heading = Vec2dOwnAngle(velocityVec);
    if (heading < 0)
        heading = -heading;

    double m_fAngleToCentreOfMass = m_pcAgent->GetVectorAngle(*m_pcAgent->GetPosition(),m_pcAgent->GetCenterOfMassOfSurroundingAgents(FEATURE_RANGE,ROBOT));

//Following commented code was using a heading based on an angle in radians
    //set opposite direction to 180 degrees from the angle to Centre of mass (+Pi)
       double m_fOppositeAngleToCOM = m_fAngleToCentreOfMass + M_PI;

    //if angle is over 2*Pi, subtract 2*Pi
    if (m_fOppositeAngleToCOM >= 2 * M_PI)
        m_fOppositeAngleToCOM -= 2 * M_PI;

//Opposite angle here is just the negative of the angle to centre of mass
//double m_fOppositeAngleToCOM = -m_fAngleToCentreOfMass;

    //tracking agent 0 (normal) and agent 15 (faulty) for testing
    if (m_pcAgent->GetIdentification() == 0 || m_pcAgent->GetIdentification() == 15)
    {
        printf("\n\nBot: %d; Heading: %f;\n", m_pcAgent->GetIdentification(), heading);
        printf("AngletoCOM: %f; OppositeAngleToCOM: %f\n",m_fAngleToCentreOfMass, m_fOppositeAngleToCOM);
    }

//comparison here should test if the robot turns 180 degrees away from the centre of mass of surrounding agents, with 5% tolerance (hence 0.95 and 1.05)
    if (heading >= 0.95 * m_fOppositeAngleToCOM && heading <= 1.05 * m_fOppositeAngleToCOM)
    {
        m_piLastOccuranceEvent[6] = CurrentStepNumber;
    }

//if the robot has turned correctly, a counter is incremented, if not it is decremented (limits of 0 to m_iEventSelectionWindow)
    if (m_piLastOccuranceEvent[6] == CurrentStepNumber)
    {
        if (m_unCorrectResponse < m_iCorrectResponseTimeWindow)
            m_unCorrectResponse++;
    }
    else
    {
        if(m_unCorrectResponse > 0)
            m_unCorrectResponse--;
    }

//Tracking count variable to see if the feature is working
    if(m_pcAgent->GetIdentification() == 0)
        printf("CorrectResponseCount: %d\n",m_unCorrectResponse);

//Set feature band based on count variable value
    for(int i = 0; i < FEATURE_DEPTH-1; i++)
    {
        if (m_unCorrectResponse >= m_pfCorrectResponseBands[i] && m_unCorrectResponse < m_pfCorrectResponseBands[i+1])
        {
            m_pfFeatureValues[6] = i;
            feature6set = true;
        }
    }

    if(feature6set == false)
    {
        m_pfFeatureValues[6] = FEATURE_DEPTH-1;
    }

///////////////////////////////


    // Motors
    //5th: distance travelled by bot in past Y time-steps. Higher than 5% of max-possible distance travelled is accepted as feature=1.
    TVector2d vecAgentPos = *(m_pcAgent->GetPosition());

    if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
    {
        // distance travelled in last 100 time-steps
        m_fSquaredDistTravelled =
                GetSquaredDistanceBetweenPositions(&vecAgentPos,
                                                   &(m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex]));


        // decision based on distance travelled in the last 100 time-steps
/*       if(m_fSquaredDistTravelled >= m_fSquaredDistThreshold)
            m_pfFeatureValues[4] = 1.0;
        else
            m_pfFeatureValues[4] = 0.0;
*/
        bool feature4Set = false;

        for(int i = 0; i < FEATURE_DEPTH-1; i++)
        {
            if((m_fSquaredDistTravelled >= m_pfSquaredDistBands[i]) && (m_fSquaredDistTravelled < m_pfSquaredDistBands[i+1]))
            {
                m_pfFeatureValues[4] = i;
                feature4Set = true;
                break;
            }
        }

        if(feature4Set == false)
            m_pfFeatureValues[4] = FEATURE_DEPTH-1;
    }

    // adding new coordinate values into the queue
    m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex] = vecAgentPos;
    m_unCoordCurrQueueIndex = (m_unCoordCurrQueueIndex + 1) % m_iDistTravelledTimeWindow;


    //6th: velocity, higher than 5% of speed is accepted as feature=1
//    m_pfFeatureValues[5] = (mag_velocity >= m_fVelocityThreshold) ? 1.0:0.0;
    bool feature5Set = false;

    for(int i = 0; i < FEATURE_DEPTH-1; i++)
    {
        if(mag_velocity >= m_pfVelocityBands[i] && mag_velocity < m_pfVelocityBands[i+1])
        {
           m_pfFeatureValues[5] = i;
           feature5Set = true;
           break;
        }
    }

    if(feature5Set == false)
        m_pfFeatureValues[5] = FEATURE_DEPTH-1;

/**NEW FEATURE - Tests if the robot disperses correctly, often, as it should**/
  //  m_pcAgent->GetVelocity();
 //   if (m_pcAgent->GetVectorAngle(m_pcAgent->GetVelocity(),m_pcAgent->GetCentreOfMassOfSurroundingAgents()) >= 175 && <= 185)
// m_pcAgent->GetCenterOfMassOfSurroundingAgents();


#ifdef DEBUGFEATUREVECTORFLAG
if(FDMODELTYPE != LINEQ) // lineq - low expected run time; can come back and log more details if needed
PrintFeatureDetails();
#endif
}

/******************************************************************************/
/******************************************************************************/

void CFeatureVector::PrintFeatureDetails()
{
    int CurrentStepNumber = (int) CSimulator::GetInstance()->GetSimulationStepNumber();

    double dist_nbrsagents, angle_acceleration, angle_velocity;
    double mag_relativeagentvelocity, dir_relativeagentvelocity,
            mag_relativeagentacceleration, dir_relativeagentacceleration;

    // Velocity magnitude and direction wrt. surrounding agents:
    m_pcAgent->GetRelativeVelocity(&mag_relativeagentvelocity, &dir_relativeagentvelocity, FEATURE_RANGE);
    //    TVector2d tTemp    = m_pcAgent->GetAverageVelocityOfSurroundingAgents(FEATURE_RANGE, ROBOT);

    //    float tmp_agentvelocity = Vec2dLength((*m_pcAgent->GetVelocity()));
    //    mag_relativeagentvelocity = tmp_agentvelocity - Vec2dLength((tTemp));

    //    if (Vec2dLength((*m_pcAgent->GetVelocity())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
    //        dir_relativeagentvelocity = Vec2dAngle((*m_pcAgent->GetVelocity()), tTemp);
    //    else
    //        dir_relativeagentvelocity = 0.0;


    // Acceleration magnitude and direction wrt. surrounding agents:
    m_pcAgent->GetRelativeAcceleration(&mag_relativeagentacceleration, &dir_relativeagentacceleration, FEATURE_RANGE);
    //    tTemp                = m_pcAgent->GetAverageAccelerationOfSurroundingAgents(FEATURE_RANGE, ROBOT);

    //    float tmp_agentacceleration = Vec2dLength((*m_pcAgent->GetAcceleration()));
    //    mag_relativeagentacceleration = tmp_agentacceleration - Vec2dLength((tTemp));
    //    if (Vec2dLength((*m_pcAgent->GetAcceleration())) > EPSILON && Vec2dLength(tTemp) > EPSILON)
    //        dir_relativeagentacceleration = Vec2dAngle((*m_pcAgent->GetAcceleration()), tTemp);
    //    else
    //        dir_relativeagentacceleration = 0.0;


    if (m_pcAgent->GetBehavIdentification() == 1)
    {
        //#ifdef DISABLEMODEL_RETAINRNDCALLS // additional behav stats data
        //        double f_MaxSquareDistTravelled =
        //                (m_pcAgent->GetMaximumSpeed()*(double)m_iDistTravelledTimeWindow)*
        //                (m_pcAgent->GetMaximumSpeed()*(double)m_iDistTravelledTimeWindow);

        //        int sensorymotorinteract    = (m_piLastOccuranceEvent[0]==CurrentStepNumber)?1:0;
        //        int negsensorymotorinteract = (m_piLastOccuranceEvent[1]==CurrentStepNumber)?1:0;

        //        printf("AdditionalNormBehavData: Step: %d, NbrsInRange0to3: %d, NbrsInRange3to6: %d, Sensor-Motor interaction: %d, ~Sensor-Motor interaction: %d, SquaredDistTravelled: %f, MaxSquaredDist: %f\n",
        //               CurrentStepNumber,
        //               m_pcAgent->CountAgents(FEATURE_RANGE/2.0, ROBOT),
        //               m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT)-m_pcAgent->CountAgents(FEATURE_RANGE/2.0, ROBOT),
        //               sensorymotorinteract, negsensorymotorinteract,
        //               m_fSquaredDistTravelled, f_MaxSquareDistTravelled);
        //#endif //DISABLEMODEL_RETAINRNDCALLS

        printf("\nStep: %d. FV for normal agent %d: #NBRS %d, AvgDistSurroundAgents %f, AngAcc %f, AngVel %f, RelVel_mag %f, RelVel_dir %f, RelAcc_mag %f, RelAcc_dir %f, Abs_vel [%f, %f], Abs_acel [%f, %f]\n", CurrentStepNumber, m_pcAgent->GetIdentification(), m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT),dist_nbrsagents,angle_acceleration,angle_velocity,
               mag_relativeagentvelocity,dir_relativeagentvelocity,
               mag_relativeagentacceleration,dir_relativeagentacceleration,
               m_pcAgent->GetVelocity()->x, m_pcAgent->GetVelocity()->y,
               m_pcAgent->GetAcceleration()->x , m_pcAgent->GetAcceleration()->y);

#ifdef WILDCARDINFV
        printf("Step: %d, Alternate normal FV info (with WC on S-M features), TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f, WildCard: %d\n", CurrentStepNumber, m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_fSquaredDistTravelled, m_fSquaredDistThreshold, m_iWildCardBit);

#else
        printf("Step: %d, Alternate normal FV info, TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f"/*, SquaredDistThreshold: %f\n"*/, CurrentStepNumber, m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_fSquaredDistTravelled/*, m_fSquaredDistThreshold*/);
#endif //WILDCARDINFV
    }

    if (m_pcAgent->GetBehavIdentification() == -1) //&& CurrentStepNumber > MODELSTARTTIME)
    {
        printf("\nStep: %d, FV for abnormal agent %d: #NBRS %d, AvgDistSurroundAgents %f, AngAcc %f, AngVel %f, RelVel_mag %f, RelVel_dir %f, RelAcc_mag %f, RelAcc_dir %f, Abs_vel [%f, %f], Abs_acel [%f, %f]\n", CurrentStepNumber, m_pcAgent->GetIdentification(), m_pcAgent->CountAgents(FEATURE_RANGE, ROBOT),dist_nbrsagents,angle_acceleration,angle_velocity,
               mag_relativeagentvelocity,dir_relativeagentvelocity,
               mag_relativeagentacceleration,dir_relativeagentacceleration,
               m_pcAgent->GetVelocity()->x, m_pcAgent->GetVelocity()->y,
               m_pcAgent->GetAcceleration()->x , m_pcAgent->GetAcceleration()->y);


#ifdef WILDCARDINFV
        printf("Step: %d, Alternate abnormal FV info (with WC on S-M features), TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f, SquaredDistThreshold: %f, WildCard: %d\n", CurrentStepNumber, m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_fSquaredDistTravelled, m_fSquaredDistThreshold, m_iWildCardBit);

#else
        printf("Step: %d, Alternate abnormal FV info, TimeSteps_NbrsInRange0to3: %d, TimeSteps_NbrsInRange3to6: %d, SquaredDistTravelled: %f"/*, SquaredDistThreshold: %f\n"*/, CurrentStepNumber, m_unSumTimeStepsNbrsRange0to3, m_unSumTimeStepsNbrsRange3to6, m_fSquaredDistTravelled/*, m_fSquaredDistThreshold*/);
#endif //WILDCARDINFV
    }
}

/******************************************************************************/
/******************************************************************************/

std::string CFeatureVector::ToString()
{
    char pchTemp[4096];

    if(NUMBER_OF_FEATURES == 3U)
        sprintf(pchTemp, "Values - "
                "TS_nbrs:0to3: %f - "
                "TW450_dist0to6_angacc: %1.1f - "
                "DistTW100: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_unValue);


    if(NUMBER_OF_FEATURES == 6U)
        sprintf(pchTemp, "Values - "
                "TS_nbrs:0to3: %f - "
                "TS_nbrs:3to6: %f - "
                "TW450_dist0to6_angacc: %1.1f - "
                "TW450_dist6_angacc: %1.1f - "
                "DistTW100: %1.1f - "
                "speed: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_pfFeatureValues[3],
                m_pfFeatureValues[4],
                m_pfFeatureValues[5],
                m_unValue);

    if(NUMBER_OF_FEATURES == 9U)
        sprintf(pchTemp, "Values - "
                "TS_nbrs:0to3: %f - "
                "TS_nbrs:3to6: %f - "
                "TW450_dist0to6_angacc: %1.1f - "
                "TW450_dist6_angacc: %1.1f - "
                "DistTW100: %1.1f - "
                "speed: %1.1f - "

                "nbrs:0to3: %1.1f - "
                "nbrs:3to6: %1.1f - "
                "TW450_dist0to6_angvel: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_pfFeatureValues[3],
                m_pfFeatureValues[4],
                m_pfFeatureValues[5],

                m_pfFeatureValues[6],
                m_pfFeatureValues[7],
                m_pfFeatureValues[8],
                m_unValue);


    if(NUMBER_OF_FEATURES == 12U)
        sprintf(pchTemp, "Values - "
                "TS_nbrs:0to3: %f - "
                "TS_nbrs:3to6: %f - "
                "TW450_dist0to6_angacc: %1.1f - "
                "TW450_dist6_angacc: %1.1f - "
                "DistTW100: %1.1f - "
                "speed: %1.1f - "

                "nbrs:0to3: %1.1f - "
                "nbrs:3to6: %1.1f - "
                "TW450_dist0to6_angvel: %1.1f - "
                "TW450_dist6_angvel: %1.1f - "

                "acceleration: %1.1f - "
                "angvelocity: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_pfFeatureValues[3],
                m_pfFeatureValues[4],
                m_pfFeatureValues[5],

                m_pfFeatureValues[6],
                m_pfFeatureValues[7],
                m_pfFeatureValues[8],
                m_pfFeatureValues[9],
                m_pfFeatureValues[10],
                m_pfFeatureValues[11],
                m_unValue);



    if(NUMBER_OF_FEATURES == 15U)
        sprintf(pchTemp, "Values - "
                "TS_nbrs:0to3: %f - "
                "TS_nbrs:3to6: %f - "
                "TW450_dist0to6_angacc: %1.1f - "
                "TW450_dist6_angacc: %1.1f - "
                "DistTW100: %1.1f - "
                "speed: %1.1f - "

                "nbrs:0to3: %1.1f - "
                "nbrs:3to6: %1.1f - "
                "TW450_dist0to6_angvel: %1.1f - "
                "TW450_dist6_angvel: %1.1f - "

                "acceleration: %1.1f - "
                "angvelocity: %1.1f - "
                "angacceleration: %1.1f - "
                "relvelocity_mag: %1.1f - "
                "relvelocity_dir: %1.1f - fv: %u",

                m_pfFeatureValues[0],
                m_pfFeatureValues[1],
                m_pfFeatureValues[2],
                m_pfFeatureValues[3],
                m_pfFeatureValues[4],
                m_pfFeatureValues[5],

                m_pfFeatureValues[6],
                m_pfFeatureValues[7],
                m_pfFeatureValues[8],
                m_pfFeatureValues[9],
                m_pfFeatureValues[10],
                m_pfFeatureValues[11],

                m_pfFeatureValues[12],
                m_pfFeatureValues[13],
                m_pfFeatureValues[14],
                m_unValue);

    return string(pchTemp);
}

/******************************************************************************/
/******************************************************************************/

