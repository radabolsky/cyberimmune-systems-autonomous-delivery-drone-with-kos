#include "../include/mission.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>

#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

const int32_t FLIGHT_ALTITUDE = 250;
const int32_t MAX_SPEED = 2;
const double DISTANCE_TO_POINT_FOR_ACCEPT = 2;

CommandWaypoint *missionWaypoints = NULL;
uint32_t releaseServoAfterWaypoint = -1;



int sendSignedMessage(char *method, char *response, char *errorMessage, uint8_t delay)
{
    char message[512] = {0};
    char signature[257] = {0};
    char request[1024] = {0};
    snprintf(message, 512, "%s?%s", method, BOARD_ID);

    while (!signMessage(message, signature))
    {
        fprintf(stderr, "[%s] Warning: Failed to sign %s message at Credential Manager. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }
    snprintf(request, 1024, "%s&sig=0x%s", message, signature);

    while (!sendRequest(request, response))
    {
        fprintf(stderr, "[%s] Warning: Failed to send %s request through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    uint8_t authenticity = 0;
    while (!checkSignature(response, authenticity) || !authenticity)
    {
        fprintf(stderr, "[%s] Warning: Failed to check signature of %s response received through Server Connector. Trying again in %ds\n", ENTITY_NAME, errorMessage, delay);
        sleep(delay);
    }

    return 1;
}


double getDistanceBetweenGPSCoordinatesInMeters(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
    const float EARTH_RADIUS = 6371000.0;

    // cast int coordinates to float by dividing by 10^7
    float lat1Rad = lat1 / 10000000.0 * M_PI / 180.0;
    float lon1Rad = lon1 / 10000000.0 * M_PI / 180.0;
    float lat2Rad = lat2 / 10000000.0 * M_PI / 180.0;
    float lon2Rad = lon2 / 10000000.0 * M_PI / 180.0;

    float dLat = lat2Rad - lat1Rad;
    float dLon = lon2Rad - lon1Rad;

    float a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1Rad) * cos(lat2Rad) * sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return EARTH_RADIUS * c;

    // const double degrees_to_meters = 0.011119;

    // double dLat = (lat2 - lat1) * degrees_to_meters;
    // double dLon = (lon2 - lon1) * degrees_to_meters;

    // return std::sqrt(dLat * dLat + dLon * dLon);
}


void parseMissionToPoints()
{

    uint32_t commandNum = getCommandNum();

    MissionCommand* commands = getCommands();

    int waypointsNum = 0;
    for (int i = 0; i < commandNum; i++)
    {
        switch (commands[i].type)
        {
        case CommandType::WAYPOINT:
            waypointsNum++;
            break;

        case CommandType::SET_SERVO:
            if (releaseServoAfterWaypoint == -1)
                releaseServoAfterWaypoint = waypointsNum;
            break;

        default:
            break;
        }
    }

    fprintf(stderr, "waypointsNum %d\n", waypointsNum);

    missionWaypoints = (CommandWaypoint *)malloc(waypointsNum * sizeof(CommandWaypoint));

    int waypointIndex = 0;
    for (int i = 0; i < commandNum; i++)
    {
        switch (commands[i].type)
        {
        case CommandType::WAYPOINT:
            missionWaypoints[waypointIndex] = CommandWaypoint(
                commands[i].content.waypoint.latitude,
                commands[i].content.waypoint.longitude,
                commands[i].content.waypoint.altitude
            );
            waypointIndex++;

            break;

        default:
            break;
        }
    }
}

bool getFlyAccept()
{
    char flyAcceptResponse[1024] = {0};
    sendSignedMessage("/api/fly_accept", flyAcceptResponse, "fly_accept", RETRY_DELAY_SEC);

    if (strstr(flyAcceptResponse, "$Arm: 0#") != NULL)
        return true;

    return false;
}

double getCurrentTime()
{
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;
    using std::chrono::milliseconds;

    auto now = high_resolution_clock::now();
    double nowSeconds = duration_cast<milliseconds>(now.time_since_epoch()).count() / 1000.0;

    return nowSeconds;
}

int main(void)
{
    // Before do anything, we need to ensure, that other modules are ready to work
    while (!waitForInit("periphery_controller_connection", "PeripheryController"))
    {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector"))
    {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem"))
    {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Navigation System. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector"))
    {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Server Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager"))
    {
        fprintf(stderr, "[%s] Warning: Failed to receive initialization notification from Credential Manager. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
        sleep(RETRY_DELAY_SEC);
    }
    fprintf(stderr, "[%s] Info: Initialization is finished\n", ENTITY_NAME);

    // Enable buzzer to indicate, that all modules has been initialized
    if (!enableBuzzer())
        fprintf(stderr, "[%s] Warning: Failed to enable buzzer at Periphery Controller\n", ENTITY_NAME);

    // Copter need to be registered at ORVD
    char authResponse[1024] = {0};
    sendSignedMessage("/api/auth", authResponse, "authentication", RETRY_DELAY_SEC);
    fprintf(stderr, "[%s] Info: Successfully authenticated on the server\n", ENTITY_NAME);

    // Constantly ask server, if mission for the drone is available. Parse it and ensure, that mission is correct
    while (true)
    {
        char missionResponse[1024] = {0};
        if (sendSignedMessage("/api/fmission_kos", missionResponse, "mission", RETRY_DELAY_SEC) && parseMission(missionResponse))
        {
            fprintf(stderr, "[%s] Info: Successfully received mission from the server\n", ENTITY_NAME);
            printMission();
            break;
        }
        sleep(RETRY_REQUEST_DELAY_SEC);
    }

    // The drone is ready to arm
    fprintf(stderr, "[%s] Info: Ready to arm\n", ENTITY_NAME);
    while (true)
    {
        // Wait, until autopilot wants to arm (and fails so, as motors are disabled by default)
        while (!waitForArmRequest())
        {
            fprintf(stderr, "[%s] Warning: Failed to receive an arm request from Autopilot Connector. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
            sleep(RETRY_DELAY_SEC);
        }
        fprintf(stderr, "[%s] Info: Received arm request. Notifying the server\n", ENTITY_NAME);

        // When autopilot asked for arm, we need to receive permission from ORVD
        char armRespone[1024] = {0};
        sendSignedMessage("/api/arm", armRespone, "arm", RETRY_DELAY_SEC);

        if (strstr(armRespone, "$Arm: 0#") != NULL)
        {
            // If arm was permitted, we enable motors
            fprintf(stderr, "[%s] Info: Arm is permitted\n", ENTITY_NAME);
            while (!setKillSwitch(true))
            {
                fprintf(stderr, "[%s] Warning: Failed to permit motor usage at Periphery Controller. Trying again in %ds\n", ENTITY_NAME, RETRY_DELAY_SEC);
                sleep(RETRY_DELAY_SEC);
            }
            if (!permitArm())
                fprintf(stderr, "[%s] Warning: Failed to permit arm through Autopilot Connector\n", ENTITY_NAME);
            break;
        }
        else if (strstr(armRespone, "$Arm: 1#") != NULL)
        {
            fprintf(stderr, "[%s] Info: Arm is forbidden\n", ENTITY_NAME);
            if (!forbidArm())
                fprintf(stderr, "[%s] Warning: Failed to forbid arm through Autopilot Connector\n", ENTITY_NAME);
        }
        else
            fprintf(stderr, "[%s] Warning: Failed to parse server response\n", ENTITY_NAME);
        fprintf(stderr, "[%s] Warning: Arm was not allowed. Waiting for another arm request from autopilot\n", ENTITY_NAME);
    };

    // If we get here, the drone is able to arm and start the mission
    // The flight is need to be controlled from now on
    // Also we need to check on ORVD, whether the flight is still allowed or it is need to be paused

    parseMissionToPoints();

    bool isFlyAccept = false;
    char flyAcceptResponse[1024] = {0};
    bool isCargoReleased = false;

    int32_t curLatitude = 0;
    int32_t curLongitude = 0;
    int32_t curAltitude = 0;
    double curTime = 0;

    int32_t prevLatitude = 0;
    int32_t prevLongitude = 0;
    int32_t prevAltitude = 0;
    double prevTime = 0;

    getCoords(curLatitude, curLongitude, curAltitude);
    curTime = getCurrentTime();

    getCoords(prevLatitude, prevLongitude, prevAltitude);
    prevTime = 0.0;

    double distanceToMissionPoint = getDistanceBetweenGPSCoordinatesInMeters(curLatitude, curLongitude, missionWaypoints[0].latitude, missionWaypoints[0].longitude);
    double distanceToMissionPointPrev = distanceToMissionPoint;
    double removingFromPointStart = -1;

    int32_t nextWaypointNum = 0;

    setCargoLock(0);

    while (true)
    {
        if (getFlyAccept())
        {
            if (!isFlyAccept)
            {
                resumeFlight();
                isFlyAccept = true;
            }
        }
        else
        {
            if (isFlyAccept)
            {
                pauseFlight();
                isFlyAccept = false;
            }
        }

        if (!isFlyAccept)
        {
            curTime = getCurrentTime();
            prevTime = curTime;

            getCoords(curLatitude, curLongitude, curAltitude);
            prevLatitude = curLatitude;
            prevLongitude = curLongitude;
            prevAltitude = curAltitude;

            continue;
        }

        getCoords(curLatitude, curLongitude, curAltitude);
        curTime = getCurrentTime();

        distanceToMissionPoint = getDistanceBetweenGPSCoordinatesInMeters(curLatitude, curLongitude, missionWaypoints[nextWaypointNum].latitude, missionWaypoints[nextWaypointNum].longitude);

        if (distanceToMissionPointPrev == -1)
            distanceToMissionPointPrev = distanceToMissionPoint;

        if (distanceToMissionPoint < DISTANCE_TO_POINT_FOR_ACCEPT)
        {
            fprintf(stderr, "[%s] Reached point\n", ENTITY_NAME);
            nextWaypointNum++;
            distanceToMissionPointPrev = -1;
            distanceToMissionPoint = getDistanceBetweenGPSCoordinatesInMeters(curLatitude, curLongitude, missionWaypoints[nextWaypointNum].latitude, missionWaypoints[nextWaypointNum].longitude);
        }

        if (!isCargoReleased && nextWaypointNum+1 == releaseServoAfterWaypoint && distanceToMissionPoint < 3)
        {
            fprintf(stderr, "[%s] Release cargo\n", ENTITY_NAME);
            isCargoReleased = true;
            setCargoLock(1);
        }

        if (curAltitude < FLIGHT_ALTITUDE * 0.8 || curAltitude > FLIGHT_ALTITUDE * 1.2)
        {
            fprintf(stderr, "[%s] change altitude %d\n", ENTITY_NAME, curAltitude);
            changeAltitude(FLIGHT_ALTITUDE);
        }

        if (curTime - prevTime < 0.2)
            continue;

        if (distanceToMissionPointPrev - distanceToMissionPoint < -2)
        {
            if (removingFromPointStart == -1)
            {
                fprintf(stderr, "[%s] start moving in wrong direction\n", ENTITY_NAME);
                removingFromPointStart = curTime;
            }
            else if (curTime - removingFromPointStart > 1)
            {
                fprintf(stderr, "[%s] to many move in wrong direction kill\n", ENTITY_NAME);
                enableBuzzer();
                setKillSwitch(0);
            }
        }
        else
        {
            removingFromPointStart = -1;
        }
        if (distanceToMissionPoint < distanceToMissionPointPrev)
            distanceToMissionPointPrev = distanceToMissionPoint;

        double distanceBetweenPositions = getDistanceBetweenGPSCoordinatesInMeters(prevLatitude, prevLongitude, curLatitude, curLongitude);

        double curSpeed = (distanceBetweenPositions) / (curTime - prevTime);

        if (curSpeed > MAX_SPEED)
        {
            fprintf(stderr, "[%s] change speed %f\n", ENTITY_NAME, curSpeed);
            changeSpeed(MAX_SPEED * 0.8);
        }
        prevTime = curTime;
        prevLatitude = curLatitude;
        prevLongitude = curLongitude;
        prevAltitude = curAltitude;
    }

    return EXIT_SUCCESS;
}