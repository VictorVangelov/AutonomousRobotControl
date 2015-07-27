import sys
sys.path.append('/usr/local/Aria/python')
from AriaPy import *
import fuzzy.storage.fcl.Reader
from math import degrees, atan, sqrt

workVars = {
            "robotRadius": 180,
            "robot": ArRobot(),
            "sonar": ArSonarDevice(),
            "argparser": ArArgumentParser(sys.argv),
            "achievedTolerance": 50,
            "conn": ArTcpConnection(),
            "leftVel": 0,
            "rightVel": 0,
            "leftDistance": 0,
            "rightDistance": 0,
            "frontDistance": 0,
            "frontLeftDistance": 0,
            "frontRightDistance": 0,
            "distanceToGoal": 0,
            "obstaclesUpperLimit": 549,
            "obstaclesLowerLimit": 110,
            "sonarDeadZone": 5000,
            "leftWasInDeadzone": True,
            "frontWasInDeadzone": True,
            "rightWasInDeadzone": True,
            "frontRightWasInDeadzone": True,
            "frontLeftWasInDeadzone": True,
            "trackingUpperLimit": 699,
            "trackingLowerLimit": 0,
            "xGoal": 3000,
            "yGoal": -1100,
            "sonarBugCounter": 0,
            "cyclesOfEmergency": 5,
            "smoothControlTimeDelay": 100,
            "hasObstacle": False,
            "hasGoalAchieved": False,
            "sonarData": None,
            "myGoal": ArPose(3000, -1100),
            "DzDistPair": [["rightWasInDeadzone", "rightDistance"], ["frontWasInDeadzone",
                           "frontDistance"], ["leftWasInDeadzone", "leftDistance"]],
           "DzDistPair2": [["rightWasInDeadzone", "rightDistance"], ["frontRightWasInDeadzone",
                           "frontRightDistance"], ["leftWasInDeadzone", "leftDistance"], ["frontLeftWasInDeadzone", "frontLeftDistance"]],
            "trackingSystem": fuzzy.storage.fcl.Reader.Reader().load_from_file("TFLC.fcl"),
            "avoidanceSystem": fuzzy.storage.fcl.Reader.Reader().load_from_file("OAFLC.fcl"),
            "system_output": {"LeftVelocity": 0.0, "RightVelocity": 0.0},
            "tracking_input": {"ErrorAngle": 0.0, "Distance": 0.0}
            }


def establishTCPConnection(IP, Port):
    Aria.init()
    workVars["conn"].setPort(IP, Port)
    workVars["robot"].setDeviceConnection(workVars["conn"])
    if (workVars["robot"].blockingConnect() != 1):
        print "Could not connect to robot, exiting"
        Aria.shutdown()
        sys.exit(1)


def establishLocalConnection():
    Aria.init()
    workVars["conn"] = ArRobotConnector(workVars["argparser"],
                                        workVars["robot"]),
    if (not workVars["conn"].connectRobot(workVars["robot"])):
        print 'Error connecting to robot'
        Aria.logOptions()
        print 'Could not connect to robot, exiting.'
        Aria.exit(1)
    print 'Connected to robot'


def addSonarToRobot():
    workVars["robot"].addRangeDevice(workVars["sonar"])


def startRobotMotors():
    workVars["robot"].runAsync(1)
    workVars["robot"].enableMotors()
    if not Aria_parseArgs():
        Aria.logOptions()
        Aria.exit(1)


def calculateErrorAngle(currX, currY, currTh, deltaX, deltaY):
    if currTh < 0:
        currTh += 360
    if currX == workVars["xGoal"]:
        if workVars["yGoal"] > currY:
            alfa = 90
        else:
            alfa = 180
    else:
        alfa = degrees(atan(float(deltaY)/float(deltaX)))
        print "alfa: ", alfa
    if workVars["xGoal"] < currX:
            alfa += 180
    elif alfa < currTh:
            alfa += 360
    errorAngle = currTh - alfa
    if int(errorAngle) in range(180, 361):
        errorAngle = errorAngle - 360
    elif int(errorAngle) in range(-360, -180):
        errorAngle = errorAngle + 360
    return errorAngle





def calcuclateDistanceToGoal(deltaX, deltaY ):
    return sqrt((deltaX*deltaX) + (deltaY*deltaY))


def updateToGoalParams():
    workVars["robot"].lock()
    currX = workVars["robot"].getX()
    currY = workVars["robot"].getY()
    currTh = workVars["robot"].getTh()

    deltaY = workVars["yGoal"] - currY
    deltaX = workVars["xGoal"] - currX
    workVars["errorAngle"] = calculateErrorAngle(currX, currY, currTh, deltaX, deltaY)
    workVars["distanceToGoal"] = calcuclateDistanceToGoal(deltaX, deltaY)
    workVars["robot"].unlock()


def isGoalAchieved():
    achievedTolerance = int(workVars["achievedTolerance"])
    return int(workVars["distanceToGoal"]) in range(0, achievedTolerance)


def isGoalNear():
    if workVars["distanceToGoal"] < 200:
        while isGoalAchieved():
            updateToGoalParams()
            calculateVelocityForTracking()
            executeSmoothVelocityControl()


def stopProgram():
    workVars["robot"].stop()
    print "goodbye."
    Aria.exit(0)


def updateLFRDistances():
    workVars["robot"].lock()
    workVars["leftDistance"] = int(min(workVars["robot"].getSonarRange(0),
                                       workVars["robot"].getSonarRange(1)))
    workVars["frontDistance"] = int(min(workVars["robot"].getSonarRange(2),
                                    workVars["robot"].getSonarRange(3)))
    workVars["rightDistance"] = int(min(workVars["robot"].getSonarRange(5),
                                        workVars["robot"].getSonarRange(4)))
    workVars["robot"].unlock()


def isInRange(distanceFromSonar):
    return workVars["leftDistance"] in range(workVars["obstaclesLowerLimit"],
                                             workVars["obstaclesUpperLimit"])


def hasObstacle():
    workVars["hasObstacle"] = False
    for DzSidePair in workVars["DzDistPair"]:
        if workVars[DzSidePair[1]] in range(workVars["obstaclesLowerLimit"],
                                            workVars["obstaclesUpperLimit"]):
            workVars["hasObstacle"] = True
    return workVars["hasObstacle"]


def roundToLimit(upperLimit):
    for DzDistPair in workVars["DzDistPair"]:
        if workVars[DzDistPair[1]] > workVars[upperLimit]:
            workVars[DzDistPair[1]] = workVars[upperLimit]


def roundToLowerObstacleLimit(side):
        workVars[side] = workVars["obstaclesLowerLimit"]


def updateDeadzone(sideWasInDeadzone, boolean):
    workVars[sideWasInDeadzone] = boolean


def checkForDeadzone(side):
    return workVars[side] == 5000


def checkForEmergency():
    workVars["robot"].lock()
    for DzDistPair in workVars["DzDistPair"]:
        isInDeadzone = checkForDeadzone(DzDistPair[1])
        if DzDistPair[0] and isInDeadzone:
            roundToLowerObstacleLimit(DzDistPair[1])
        updateDeadzone(DzDistPair[0], isInDeadzone)
    workVars["robot"].unlock()


def updateSonarBugReport():
    if workVars["hasObstacle"]:
        workVars["sonarBugCounter"] += 1


def hasSonarBug():
    return (workVars["sonarBugCounter"]) == workVars["cyclesOfEmergency"]


def handleSonarBug():
    workVars["robot"].setVel2(100, 0)
    ArUtil.sleep(500)
    workVars["robot"].setVel2(0, 100)
    ArUtil.sleep(500)


def executeSmoothVelocityControl():
    workVars["robot"].lock()
    print "leftVel: ", workVars["leftVel"], "rightVel: ", workVars["rightVel"]
    workVars["robot"].setVel2(workVars["leftVel"], workVars["rightVel"])
    ArUtil.sleep(100)
    workVars["robot"].unlock()


def calculateVelocityForObstacleAvoidance():
    workVars["avoidance_input"] = {
        "LeftDistance": workVars["leftDistance"],
        "RightDistance": workVars["rightDistance"],
        "FrontDistance": workVars["frontDistance"]}
    # print "\nleftDistance  :", workVars["leftDistance"]
    # print "frontDistance :", workVars["frontDistance"]
    # print "rightDistance :", workVars["rightDistance"]
    workVars["avoidanceSystem"].calculate(workVars["avoidance_input"],
                                          workVars["system_output"])
    workVars["leftVel"] = workVars["system_output"]["LeftVelocity"]
    workVars["rightVel"] = workVars["system_output"]["RightVelocity"]
    #print "inCalcObstacles", workVars["leftVel"], "   ", workVars["rightVel"]


def calculateVelocityForTracking():
    workVars["tracking_input"] = {"ErrorAngle": workVars["errorAngle"],
                                  "Distance": workVars["distanceToGoal"]}
    workVars["trackingSystem"].calculate(workVars["tracking_input"],
                                         workVars["system_output"])
    workVars["leftVel"] = workVars["system_output"]["LeftVelocity"]
    workVars["rightVel"] = workVars["system_output"]["RightVelocity"]
    #print "inCalcTrac", workVars["leftVel"], "   ", workVars["rightVel"]


def saveDataInFile(fileName, robotPoseHistory):
    newFile = open(fileName, "w+")
    newFile.write(robotPoseHistory )


def main():
    robotPoseHistory = ""
    establishTCPConnection('192.168.45.32', 8101)
    addSonarToRobot()
    startRobotMotors()
    updateLFRDistances()
    checkForEmergency()
    updateToGoalParams()
    while not isGoalAchieved():
        isGoalNear()
        updateLFRDistances()
        checkForEmergency()
        updateToGoalParams()
        ArUtil.sleep(300)
        if hasObstacle():
            if hasSonarBug():
                print "hasSonarBug"
                handleSonarBug()
            else:
                print "\nleftDistance  :", workVars["leftDistance"]
                print "frontDistance :", workVars["frontDistance"]
                print "rightDistance :", workVars["rightDistance"]
                roundToLimit("obstaclesUpperLimit")
                calculateVelocityForObstacleAvoidance()
        else:
            print "distanceToGoal: ", workVars["distanceToGoal"]
            print "errorAngle: ", workVars["errorAngle"]
            roundToLimit("trackingUpperLimit")
            calculateVelocityForTracking()
        robotPoseHistory += str(workVars["robot"].getPose())
        robotPoseHistory += "\n"
        executeSmoothVelocityControl()
        print workVars["robot"].getPose()
        saveDataInFile("dynamic.txt", robotPoseHistory)
    stopProgram()

# def main():
#     robotPoseHistory = ""
#     establishTCPConnection('192.168.45.32', 8101)
#     addSonarToRobot()
#     startRobotMotors()
#     updateLFRDistances()
#     checkForEmergency()
#     workVars["myGoal"] = ArPose(2650, -20)
#     updateToGoalParams()
#     while not isGoalAchieved():
#         isGoalNear()
#         updateLFRDistances()
#         checkForEmergency()
#         updateToGoalParams()
#         ArUtil.sleep(300)
#         if hasObstacle():
#             if hasSonarBug():
#                 print "hasSonarBug"
#                 handleSonarBug()
#             else:
#                 print "\nleftDistance  :", workVars["leftDistance"]
#                 print "frontDistance :", workVars["frontDistance"]
#                 print "rightDistance :", workVars["rightDistance"]
#                 roundToLimit("obstaclesUpperLimit")
#                 calculateVelocityForObstacleAvoidance()
#         else:
#             print "distanceToGoal: ", workVars["distanceToGoal"]
#             print "errorAngle: ", workVars["errorAngle"]
#             roundToLimit("trackingUpperLimit")
#             calculateVelocityForTracking()
#         robotPoseHistory += str(workVars["robot"].getPose())
#         robotPoseHistory += "\n"
#         executeSmoothVelocityControl()
#         print workVars["robot"].getPose()
#         saveDataInFile("dynamic.txt", robotPoseHistory)
#     workVars["myGoal"]= ArPose(3000, 700)
#     workVars["xGoal"] = 3000
#     workVars["yGoal"] = 700
#     while not isGoalAchieved():
#         isGoalNear()
#         updateToGoalParams()
#         calculateVelocityForTracking()
#         executeSmoothVelocityControl()
#         ArUtil.sleep(200)
#     stopProgram()

def main2():
    robotPoseHistory = ""
    establishTCPConnection('192.168.45.32', 8101)
    addSonarToRobot()
    startRobotMotors()
    updateToGoalParams()
    while not isGoalAchieved():
        isGoalNear()
        updateLFRDistances()
        checkForEmergency()
        updateToGoalParams()
        ArUtil.sleep(500)
        roundToLimit("trackingUpperLimit")
        calculateVelocityForTracking()
        executeSmoothVelocityControl()
        robotPoseHistory += str(workVars["robot"].getPose())
        robotPoseHistory += "\n"
        saveDataInFile("tfc.txt", robotPoseHistory)
        print "robot pose :", workVars["robot"].getPose()
        print "myGoal: ", workVars["myGoal"]
        print "distanceToGoal: ", workVars["distanceToGoal"]
        print "errorAngle: ", workVars["errorAngle"], "\n"
        print
#        print workVars["robot"].getPose( ),
    stopProgram()


if __name__ == '__main__':
    main()
