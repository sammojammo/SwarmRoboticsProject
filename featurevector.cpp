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
    m_tAngularAccelerationThreshold = 0.032 * m_pcAgent->GetMaximumAngularVelocity();

    for(unsigned int i = 0; i < NUMBER_OF_FEATURES; i++)
    {
        m_piLastOccuranceEvent[i]    = 0;
        m_piLastOccuranceNegEvent[i] = 0;

        m_pfFeatureValues[i]         = 0.0;
    }

//Range of feature values split into bands - feature depth is number of possible values, so must be
    m_pfVelocityBands = new float[(int)FEATURE_DEPTH];
    m_pfAccelerationBands = new float[(int)FEATURE_DEPTH];

    m_pfSquaredDistBands = new float[(int)FEATURE_DEPTH];

    m_pfCorrectResponseBands = new float[(int)FEATURE_DEPTH];

    m_pfSensoryMotorBands = new float[(int)FEATURE_DEPTH];

    for(int i = 0; i < FEATURE_DEPTH; i++)
    {
        m_pfVelocityBands[i] = (i * (1.0/FEATURE_DEPTH) * (m_pcAgent->GetMaximumSpeed()));
        m_pfAccelerationBands[i] = i * (1.0/FEATURE_DEPTH) * (m_pcAgent->GetMaximumSpeed());

        m_pfCorrectResponseBands[i] = i * (1.0/FEATURE_DEPTH) * m_iCorrectResponseTimeWindow;

        m_pfSensoryMotorBands[i] = i * (1.0/FEATURE_DEPTH) * m_iEventSelectionTimeWindow;
    }


    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    m_unNbrsCurrQueueIndex = 0;

    m_unSumTimeStepsNbrsRange0to3 = 0;
    m_unSumTimeStepsNbrsRange3to6 = 0;

    m_punNbrsRange0to3AtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];
    m_punNbrsRange3to6AtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];


    //Sensory motor variables F2,F3
    m_unTurnCurrQueueIndex = 0;

    m_unSensoryMotorNearCount = 0;
    m_unSensoryMotorFarCount = 0;

    m_punTurnedWithNbrsAtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];
    m_punTurnedWithoutNbrsAtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];

    m_fRobotHeading = 0.0;
    m_fPreviousRobotHeading = 0.0;

    // keeping track of distance travelled by bot in last 100 time-steps
    m_iDistTravelledTimeWindow = 100;/******/
    m_unCoordCurrQueueIndex    = 0;

    m_fSquaredDistTravelled = 0.0;

    for(int i = 0; i < FEATURE_DEPTH; i++)
    {
        m_pfSquaredDistBands[i] = i * (1.0/FEATURE_DEPTH) * ((m_pcAgent->GetMaximumSpeed() * (double)m_iDistTravelledTimeWindow) *
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

    delete m_punTurnedWithNbrsAtTimeStep;
    delete m_punTurnedWithoutNbrsAtTimeStep;

    delete m_pvecCoordAtTimeStep;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CFeatureVector::GetValue() const
{
    return m_unValue;
}
/******************************************************************************/
/*function to return value of specific feature, specified by parameter featureNum*/
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
        }

        for(int i = 0; i < FEATURE_DEPTH-1; i++)
        {
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
    bool feature2Set = false, feature3Set = false;

    if(dist_nbrsagents < 6.0 &&
        (angle_acceleration > m_tAngularAccelerationThreshold || angle_acceleration < -m_tAngularAccelerationThreshold))
    {
        m_unSensoryMotorNearCount++;
        m_punTurnedWithNbrsAtTimeStep[m_unTurnCurrQueueIndex] = 1;
    }
    else
        m_punTurnedWithNbrsAtTimeStep[m_unTurnCurrQueueIndex] = 0;


    if(dist_nbrsagents == 6.0 &&
        (angle_acceleration > m_tAngularAccelerationThreshold || angle_acceleration < -m_tAngularAccelerationThreshold))
    {
        m_unSensoryMotorFarCount++;
        m_punTurnedWithoutNbrsAtTimeStep[m_unTurnCurrQueueIndex] = 1;
    }
    else
        m_punTurnedWithoutNbrsAtTimeStep[m_unTurnCurrQueueIndex] = 0;

    //Increment counter - counts to limit X then resets to 0
    m_unTurnCurrQueueIndex = (m_unTurnCurrQueueIndex + 1) % m_iEventSelectionTimeWindow;

    //set feature band
    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
    {
        for(int i = 0; i < FEATURE_DEPTH-1; i++)
        {
            if( (m_unSensoryMotorNearCount >= m_pfSensoryMotorBands[i]) && (m_unSensoryMotorNearCount < m_pfSensoryMotorBands[i+1]))
            {
                m_pfFeatureValues[2] = i;
                feature2Set = true;
                break;
            }
        }

        for(int i = 0; i < FEATURE_DEPTH-1; i++)
        {
            if( (m_unSensoryMotorFarCount >= m_pfSensoryMotorBands[i]) && (m_unSensoryMotorFarCount < m_pfSensoryMotorBands[i+1]))
            {
                m_pfFeatureValues[3] = i;
                feature3Set = true;
                break;
            }
        }

        if(feature2Set == false)
            m_pfFeatureValues[2] = FEATURE_DEPTH-1;

        if(feature3Set == false)
            m_pfFeatureValues[3] = FEATURE_DEPTH-1;

        m_unSensoryMotorNearCount -= m_punTurnedWithNbrsAtTimeStep[m_unTurnCurrQueueIndex];
        m_unSensoryMotorFarCount -= m_punTurnedWithoutNbrsAtTimeStep[m_unTurnCurrQueueIndex];

    }

    // Motors
    //5th: distance travelled by bot in past Y time-steps. Higher than 5% of max-possible distance travelled is accepted as feature=1.
    TVector2d vecAgentPos = *(m_pcAgent->GetPosition());

    if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
    {
        // distance travelled in last 100 time-steps
        m_fSquaredDistTravelled = GetSquaredDistanceBetweenPositions(&vecAgentPos,
                                                        &(m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex]));


        // decision based on distance travelled in the last 100 time-steps
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

/////////////////////////////////FEATURE 7
//The feature is supposed to test if a robot disperses correctly, ie turning away from the centre of mass of its surrounding agents
//The agent should turn away from the centre of mass of surrounding agents if it gets too close
//This feature tests the distance to centre of mass, and if the robot heading changes correctly

    //Get distance from robot to centre of mass of surrounding agents
    double distToCentreOfMass = sqrt( (m_pcAgent->GetCenterOfMassOfSurroundingAgents(FEATURE_RANGE,ROBOT).x - m_pcAgent->GetPosition()->x)
                                        * (m_pcAgent->GetCenterOfMassOfSurroundingAgents(FEATURE_RANGE,ROBOT).x - m_pcAgent->GetPosition()->x)
                                     + ((m_pcAgent->GetCenterOfMassOfSurroundingAgents(FEATURE_RANGE,ROBOT).y - m_pcAgent->GetPosition()->y)
                                        * (m_pcAgent->GetCenterOfMassOfSurroundingAgents(FEATURE_RANGE,ROBOT).y - m_pcAgent->GetPosition()->y)) );

/*        //Centre of mass wrt robot
        TVector2d COMwrtRobot;
        COMwrtRobot.x = m_pcAgent->GetCenterOfMassOfSurroundingAgents(FEATURE_RANGE,ROBOT).x - m_pcAgent->GetPosition()->x;
        COMwrtRobot.y = m_pcAgent->GetCenterOfMassOfSurroundingAgents(FEATURE_RANGE,ROBOT).y - m_pcAgent->GetPosition()->y;

        //opposite direction to centre of mass wrt robot
        COMwrtRobot.x = -COMwrtRobot.x + m_pcAgent->GetPosition()->x;
        COMwrtRobot.y = -COMwrtRobot.y + m_pcAgent->GetPosition()->y;
*/
        //Get agent velocity
        TVector2d velocityVec;
        velocityVec.x = m_pcAgent->GetVelocity()->x;
        velocityVec.y = m_pcAgent->GetVelocity()->y;

        //Get robot heading and normalise
        m_fRobotHeading = Vec2dOwnAngle(velocityVec);
        m_fRobotHeading = NormalizeAngle(m_fRobotHeading);

    //print marker for each step - for debugging in console
    if(m_pcAgent->GetIdentification() == 0)
        printf("\n===========================Step: %d", CurrentStepNumber);

    printf("\nBot: %d, DisttoCOM: %f, AngVel: %f, Heading: %f",m_pcAgent->GetIdentification(),distToCentreOfMass,m_pcAgent->GetAngularVelocity(),m_fRobotHeading);

    //if centre of mass of surrounding agents is within range, and heading has changed
    if(distToCentreOfMass <= (FEATURE_RANGE/2.0) && ((m_fRobotHeading <= 0.95 * m_fPreviousRobotHeading) || (m_fRobotHeading >= 1.05 * m_fPreviousRobotHeading) ))//m_pcAgent->GetAngularVelocity() > 1.0)
    {

        bool feature6set = false;

        //Get angle to centre of mass and normalise
        double m_fAngleToCentreOfMass = m_pcAgent->GetVectorAngle(m_pcAgent->GetCenterOfMassOfSurroundingAgents(FEATURE_RANGE,ROBOT),*m_pcAgent->GetPosition());
        m_fAngleToCentreOfMass = NormalizeAngle(m_fAngleToCentreOfMass);

    //set opposite direction to 180 degrees from the angle to Centre of mass (+Pi)
        double m_fOppositeAngleToCOM = m_fAngleToCentreOfMass + M_PI;
        //double m_fOppositeAngleToCOM = Vec2dOwnAngle(COMwrtRobot);
        m_fOppositeAngleToCOM = NormalizeAngle(m_fOppositeAngleToCOM);

        printf("\nTesting Bot: %d; Heading: %f;\n", m_pcAgent->GetIdentification(), m_fRobotHeading);
        printf("AngletoCOM: %f; OppositeAngleToCOM: %f\n",m_fAngleToCentreOfMass, m_fOppositeAngleToCOM);

        //Test if the robot turns 180 degrees away from the centre of mass of surrounding agents, with 5% tolerance (hence 0.95 and 1.05)
        //if the robot has turned correctly, a counter is incremented, if not it is decremented (limits of 0 to m_iCorrectResponseTimeWindow)

        if (m_fRobotHeading >= 0.95 * m_fOppositeAngleToCOM && m_fRobotHeading <= 1.05 * m_fOppositeAngleToCOM)
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

    }
    m_fPreviousRobotHeading = m_fRobotHeading;


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
