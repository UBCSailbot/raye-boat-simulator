#!/usr/bin/env python

if __name__ == '__main':
    # Given globalPath from start. Assume doesn't change for now
    globalPath = [[1,0], [10,11],[24,10]]

    # Create sailbot ROS object that subscribes to relevant topics
    sailbot = Sailbot()

    # Set current global waypoint
    globalPathIndex = 0
    sailbot.globalWaypoint = globalPath[globalPathIndex]

    # Create first path and track time of updates
    currentState = sailbot.getCurrentState()
    currentLandAndBorderData = getCurrentLandAndBorderData(currentState)
    currentPath = createNewPath(currentState, currentLandAndBorderData)
    lastTimePathCreated = time.now()

    while true:
        currentState = sailbot.getCurrentState()

        # If next global waypoint reached, update land and border data
        # Update global waypoint in sailbot object
        if nextGlobalWaypointReached(currentState):
            # Update global waypoint and corresponding land+border data
            globalPathIndex = globalPathIndex + 1
            sailbot.globalWaypoint = globalPath[globalPathIndex]
            currentLandAndBorderData = getCurrentLandAndBorderData(currentState)

            # Update local path
            currentPath = createNewPath(currentState, currentLandAndBorderData)
            lastTimePathCreated = time.now()

            # publish 2nd point of new path (not the current position)


        else if isBad(currentPath) or nextLocalWaypointReached(currentState) or timeLimitExceeded(lastTimePathCreated):
            # Update local path
            currentPath = createNewPath(currentState, currentLandAndBorderData)
            lastTimePathCreated = time.now()

            # publish 2nd point of new path (not the current position)

