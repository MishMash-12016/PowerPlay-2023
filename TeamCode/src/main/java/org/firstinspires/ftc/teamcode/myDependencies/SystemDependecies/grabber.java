package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;

public class grabber {
    // region SERVOS
    private static Servo grabberServo ;
    private static Servo rightServo   ;
    private static Servo leftServo    ;
    // endregion

    // region SENSORS
    private static DistanceSensor distanceFromConeSensor;
    private static DigitalChannel isOutSensor;
    // endregion

    // region CONSTANTS
    private static final double middlePosition = 0.6;
    private static final double inPosition     = 0.8;
    //                                             low -  -  -  -  -  -  - high
    private static final double[] pilePositions = {0.1, 0.72, 0.69, 0.65, 0.61};


    private static final double grabPosition = 0.45;
    private static final double midReleasePosition = 0.19;
    private static final double releasePosition = 0.16;

    private static final double coneDistanceCatchTrigger = 8.5;
    private static final double coneInDistance = 3.5;
    // endregion

    // region INITIALIZATION
    public static void initialize(){
        // region SERVOS
        grabberServo = RobotSystem.hardwareMap.servo.get("grabber");

        midRelease();

        rightServo = RobotSystem.hardwareMap.servo.get("grabberRight");
        leftServo  = RobotSystem.hardwareMap.servo.get("grabberLeft" );

        rightServo.setDirection(Servo.Direction.REVERSE);

        goToIn();
        // endregion

        // region SENSOR
        isOutSensor = RobotSystem.hardwareMap.get(DigitalChannel.class, "grabberIsOutSensor");
        isOutSensor.setMode(DigitalChannel.Mode.INPUT);

        distanceFromConeSensor = RobotSystem.hardwareMap.get(DistanceSensor.class, "grabberDistanceToConeSensor");
        // endregion
    }
    // endregion

    // region FUNCTIONALITY
    public static void fullRelease() { grabberServo.setPosition(releasePosition   ); }
    public static void midRelease()  { grabberServo.setPosition(midReleasePosition); }
    public static void grab()        { grabberServo.setPosition(grabPosition      ); }

    public static void goToCone(int coneHeight){ setPosition(pilePositions[coneHeight]); }
    public static void goToMid()            { setPosition(middlePosition); }
    public static void goToIn()             { setPosition(inPosition    ); }

    public static boolean coneIsInRange(){
        return coneDistanceCatchTrigger > distanceFromConeSensor.getDistance(DistanceUnit.CM);
    }
    public static boolean hasCone(){
        return coneInDistance > distanceReading();
    }
    public static boolean isOut() {
        return isOutSensor.getState();
    }
    public static boolean isGrabbing() {
        return grabberServo.getPosition() == grabPosition;
    }

    public static double distanceReading(){
        return distanceFromConeSensor.getDistance(DistanceUnit.CM);
    }

    public static String deBug(){
        return "left servo position : " + leftServo.getPosition()
                + "\nright servo position : " + rightServo.getPosition()
                + "\nsensor reading : " + distanceReading();
    }
    // endregion

    // region PRIVATE FUNCTIONALITY
    private static void setPosition(double position){
        rightServo.setPosition(position);
        leftServo .setPosition(position);
    }
    // endregion
}
