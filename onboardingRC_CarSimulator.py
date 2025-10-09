import pybullet as p
import pybullet_data
import time
import math
import os

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

startOrientation = p.getQuaternionFromEuler([0, 0, math.pi / 2])
carID = p.loadURDF("racecar/racecar.urdf", [4, 0, 0], startOrientation)
planeId = p.loadURDF("plane.urdf")  # add plane so there is a ground


# Make Duck

# duck visual
visualIdDuck1 = p.createVisualShape(
    p.GEOM_MESH,
    fileName="duck.obj",
    rgbaColor=[1, 1, 1, 1],  # opaque white
    specularColor=[0.4, 0.4, 0],  # yellowish shine
    meshScale=[0.5, 0.5, 0.5],  # size down to half its size
)

# duck collision
collisionIdDuck1 = p.createCollisionShape(
    p.GEOM_MESH,
    fileName="duck_vhacd.obj",
    meshScale=[0.5, 0.5, 0.5],
)

# duck create multibody
duckID1 = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=collisionIdDuck1,
    baseVisualShapeIndex=visualIdDuck1,
    basePosition=[4, 1, 1],
    baseOrientation=startOrientation,
)

#put steering wheel joint nubmers and wheels into array to use for driving later 
steeringJoints = [4, 6]
wheels = [2, 3, 5, 7]

# initial Variables
targetVelocity = 0.0
steeringAngle = 0.0
dt = 1.0 / 240.0
trajectory = []
collisionCheck = [duckID1]
turnCounter=0

commands = [
    "forward 55",
    "left 90",
    "brake",
    "forward 13",
    "straighten",
    "forward 30",
    "backward 10",
    "right 90",
    "forward 13",
    "straighten",
    "forward 30",
    "stop"
]

#Create track
def makeWall(orientationX, orientationY, length, width, height):
    wallVisualId = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=[length, width, height],
        rgbaColor=[1.0, 0.4, 0.7, 1.0],  # PINK!
        specularColor=[0.8, 0.3, 0.6],
    )

    wallCollisionId = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[length+0.1, width+0.1, height+0.1] #bit bigger so make sure detect
    )

    wallID = p.createMultiBody(
        baseMass=0,  # does not fall when collide with other objects
        baseCollisionShapeIndex=wallCollisionId,
        baseVisualShapeIndex=wallVisualId,
        basePosition=[orientationX, orientationY, 0],
    )

    collisionCheck.append(wallID)


# OUTER walls
makeWall(0, 5, 5, 0.1, 0.5)
makeWall(0, -5, 5, 0.1, 0.5)
makeWall(5, 0, 0.1, 5, 0.5)
makeWall(-5, 0, 0.1, 5, 0.5)

# INNER walls
makeWall(0, 3, 3, 0.1, 0.5)
makeWall(0, -3, 3, 0.1, 0.5)
makeWall(3, 0, 0.1, 3, 0.5)
makeWall(-3, 0, 0.1, 3, 0.5)



# for i in collisionCheck:
#     print(i)

hitStates = {t: {"lastHit": False, "totalT": 0.0} for t in collisionCheck}

# Used to find out what are steering and wheel joints
# numJoints = p.getNumJoints(carID)
# for joint in range(numJoints):
#     print(p.getJointInfo(carID, joint)[1])


# directions to drive
def setDrive(velocity, steeringDegree):
    for i in wheels:
        p.setJointMotorControl2(
            bodyUniqueId=carID,
            jointIndex=i,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=velocity,
            force=200)
    for i in steeringJoints:
        p.setJointMotorControl2(
            bodyUniqueId=carID,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=math.radians(steeringDegree))


def getCarState():
    # orientation is roll pitch yaw (roll = left-right tilt, pitch = up-down tilt, yaw = car’s heading)
    position, orientation = p.getBasePositionAndOrientation(carID)
    heading = p.getEulerFromQuaternion(orientation)[2]
    return position, heading


def checkContact(carID, collisionObjects, hitStates):
    num_links = p.getNumJoints(carID)
    
    #go through all possible collision objects
    for collisionObject in collisionObjects:
        contactFound = False
        
        #Go through all links in an object, inlcude -1 for base link
        for link in range(-1, num_links):
            contacts = p.getContactPoints(bodyA=carID, linkIndexA=link, bodyB=collisionObject)
            if contacts:
                contactFound = True
                break

        state = hitStates[collisionObject]
        
        if contactFound:
            state["lastHit"] = True
            state["totalT"] += dt
            if state["totalT"] == dt: 
                print(f"COLLISION START with object {collisionObject}")
        else:
            if state["lastHit"]:
                print(f"COLLIDED with object {collisionObject} for {state['totalT']:.2f} seconds")
            state["lastHit"] = False
            state["totalT"] = 0.0

def printBeforeExecute(command, position, heading):
    global turnCounter
    print(f"🦆 {turnCounter}. Executing: {command}...")
    print(f"→ Inital Position: ({position[0]:.2f}, {position[1]:.2f}), Heading: {math.degrees(heading):.1f}°")
    print("-----------------------------")
    turnCounter+=1

def printAfterExecute(command, position, heading):
    print(f"Completed: {command}")
    print(f"→ Final Position: ({position[0]:.2f}, {position[1]:.2f}), Heading: {math.degrees(heading):.1f}°")
    print("-----------------------------")


for command in commands:
    currCommand = command.split()
    action = currCommand[0].lower()

    position, heading = getCarState()
    printBeforeExecute(command, position, heading)

    if action in ["forward", "backward"]:
        distance = float(currCommand[1])
        targetVelocity = 10
        if action == "backward":
            targetVelocity = -10 
        
        movingTime = distance/abs(targetVelocity)
        steps = int(movingTime / dt)
        
        for j in range(steps):
            setDrive(targetVelocity, steeringAngle)
            p.stepSimulation()
            position, heading = getCarState()
            trajectory.append((position[0], position[1]))
            checkContact(carID, collisionCheck, hitStates)
            time.sleep(dt)

    elif action in ["left", "right"]:
        degrees = float(currCommand[1])
        if action == "left":
            steeringAngle = degrees
        else: 
            steeringAngle = -degrees
        
        #do short time for wheel to adjust
        for _ in range(60): 
            setDrive(0, steeringAngle)
            p.stepSimulation()
            checkContact(carID, collisionCheck, hitStates)
            time.sleep(dt)

    elif action == "brake":
        targetVelocity = 0.0
        for _ in range(240):
            setDrive(0, steeringAngle)
            p.stepSimulation()
            checkContact(carID, collisionCheck, hitStates)
            time.sleep(dt)
            
        
    elif action == "straighten":
        steeringAngle = 0
         #do short time for wheel to adjust
        for _ in range(60): 
            setDrive(0, steeringAngle)
            p.stepSimulation()
            checkContact(carID, collisionCheck, hitStates)
            time.sleep(dt)

    elif action == "stop":
        position, heading = getCarState()
        print("Stopping simulation.")
        break
        
    printAfterExecute(command, position, heading)

#DONE YAYAYYA 


p.disconnect()
