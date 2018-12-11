#!/usr/bin/env python3

import wpilib
import wpilib.drive
from robotpy_ext.common_drivers.navx import AHRS

class MyRobot(wpilib.SampleRobot):
    """This is a demo program showing the use of the navX MXP to implement
    a "rotate to angle" feature. This demo works in the pyfrc simulator.
    
    This example will automatically rotate the robot to one of four
    angles (0, 90, 180 and 270 degrees).
    
    This rotation can occur when the robot is still, but can also occur
    when the robot is driving.  When using field-oriented control, this
    will cause the robot to drive in a straight line, in whatever direction
    is selected.
    
    This example also includes a feature allowing the driver to "reset"
    the "yaw" angle.  When the reset occurs, the new gyro angle will be
    0 degrees.  This can be useful in cases when the gyro drifts, which
    doesn't typically happen during a FRC match, but can occur during
    long practice sessions.
    
    Note that the PID Controller coefficients defined below will need to
    be tuned for your drive system.
    """

    # The following PID Controller coefficients will need to be tuned */
    # to match the dynamics of your drive system.  Note that the      */
    # SmartDashboard in Test mode has support for helping you tune    */
    # controllers by displaying a form where you can enter new P, I,  */
    # and D constants and test the mechanism.                         */

    # Often, you will find it useful to have different parameters in
    # simulation than what you use on the real robot

    if wpilib.RobotBase.isSimulation():
        # These PID parameters are used in simulation
        kP = 0.03
        kI = 0.001
        kD = 0.01
        kF = 0.00
    else:
        # These PID parameters are used on a real robot
        kP = 0.073
        kI = 0.0
        kD = 0.01725
        kF = 0.00

    kToleranceDegrees = 2.0

    def robotInit(self):
        # # Channels for the wheels
        # frontLeftChannel = 2
        # rearLeftChannel = 3
        # frontRightChannel = 1
        # rearRightChannel = 0

        # self.myRobot = wpilib.RobotDrive(
        #     frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel
        # )

        # Define Driving Motors
        self.rightDrive = wpilib.VictorSP(0)
        self.leftDrive = wpilib.VictorSP(1)

        # Create Robot Drive
        self.myRobot = wpilib.drive.DifferentialDrive(self.rightDrive, self.leftDrive)

        self.myRobot.setExpiration(0.1)
        self.stick = wpilib.Joystick(0)

        # Communicate w/navX MXP via the MXP SPI Bus.
        self.ahrs = AHRS.create_spi()

        turnController = wpilib.PIDController(
            self.kP, self.kI, self.kD, self.kF, self.ahrs, output=self
        )
        turnController.setInputRange(-180.0, 180.0)
        turnController.setOutputRange(-0.5, 0.5)
        turnController.setAbsoluteTolerance(self.kToleranceDegrees)
        # turnController.setPercentTolerance(25)
        turnController.setContinuous(True)

        self.turnController = turnController
        self.rotateToAngleRate = 0

        # Add the PID Controller to the Test-mode dashboard, allowing manual  */
        # tuning of the Turn Controller's P, I and D coefficients.            */
        # Typically, only the P value needs to be modified.                   */
        wpilib.LiveWindow.addActuator("DriveSystem", "RotateController", turnController)
        wpilib.LiveWindow.addSensor("DriveSystem", "NavX", self.ahrs)

    def operatorControl(self):

        tm = wpilib.Timer()
        tm.start()


        self.myRobot.setSafetyEnabled(True)

        self.ahrs.reset()
        while self.isOperatorControl() and self.isEnabled():

            if tm.hasPeriodPassed(1.0):
                print("NavX Gyro", self.ahrs.getYaw(), self.ahrs.getAngle())

            self.turnController.setSetpoint(90.0)

            while(self.turnController.onTarget() == False):
                print("NOT ON TARGET:", self.kToleranceDegrees)
                self.turnController.enable()
                self.myRobot.arcadeDrive(self.stick.getY(), self.rotateToAngleRate)

            print("ON TARGET:", self.kToleranceDegrees)
            self.turnController.disable()

            #wpilib.Timer.delay(0.05)  # wait for a motor update time

    def pidWrite(self, output):
        """This function is invoked periodically by the PID Controller,
        based upon navX MXP yaw angle input and PID Coefficients.
        """
        self.rotateToAngleRate = output

if __name__ == "__main__":
    wpilib.run(MyRobot)
