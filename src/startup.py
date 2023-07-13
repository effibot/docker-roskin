#! ~/venvs/py27/bin/python
import qi
import sys
import time
import argparse

def connect(ip, port = 9559):
    """Connect to the Naoqi session on the provided ip and port.

    Args:
        ip: The ip of the robot.
        port (int, optional): Open port on the robot. Defaults to 9559.
    """
    session = qi.Session()
    try:
        session.connect("tcp://" + ip + ":" + str(port))
        return(session)
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) +".\nPlease check your script arguments. Run with -h option for help.")
        sys.exit(1)

def rotateHead(motion_service):
        JointNamesH = ["HeadPitch", "HeadYaw"] # range ([-1,1],[-0.5,0.5]) // HeadPitch :{(-)up,(+)down} , HeadYaw :{(-)left,(+)right}

        #amntY =  input("Enter amount to Move Up(-) And Down(+) [-1,1] : ")
        #amntX =  input("Enter amount to Move Left(-) And Right(+) [-0.5,0.5] : ")

        ###############################
        # Set to "look down" position #
        ###############################
        amntY = 0.5
        amntX = 0

        pFractionMaxSpeed = 0.2

        HeadA = [float(amntY),float(amntX)]

        motion_service.angleInterpolationWithSpeed(JointNamesH, HeadA, pFractionMaxSpeed)
        time.sleep(1)

        return


def mapping_pose(session):
    # Get the services ALMotion & ALRobotPosture.
    motion_service  = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    # Wake up robot
    print("Waking up robot...")
    motion_service.wakeUp()
    print("--- Wake Up Complete ---")
    # Send robot to Stand
    print("Sending robot to Stand...")
    posture_service.goToPosture("StandInit", 0.5)
    print("--- Stand Complete ---")
    #####################
    ## Disable arms control by Motion algorithm
    #####################
    motion_service.setMoveArmsEnabled(False, False)
    try:
        print("Moving Head to look down...")
        rotateHead(motion_service= motion_service)
        motion_service.stopMove()
        print("Done, exiting")
    except KeyboardInterrupt:
        print("KeyBoard Interrupt initiated")
        motion_service.stopMove()
        exit()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="10.1.1.3")
    parser.add_argument("--port", type=int, default=9559)
    args = parser.parse_args()
    session = connect(args.ip, args.port)
    mapping_pose(session)

if __name__ == "__main__":
    main()