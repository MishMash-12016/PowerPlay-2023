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
    private static final double middlePosition = 0.61;
    private static final double inPosition     = 0.79;
    //                                             low -  -  -  -  -  -  - high
    private static final double[] pilePositions = {0.13, 0.14, 0.17, 0.2, 0.24};


    private static final double grabPosition = 0.42;
    private static final double releasePosition = 0.18;

    private static final double coneDistanceCatchTrigger = 12;
    private static final double coneInDistance = 5;

    private static final int slowIncrements = 25;
    // endregion

    // region VARIABLES
    public static double offset = 0;
    // endregion

    // region INITIALIZATION
    public static void initialize(){
        // region SERVOS
        grabberServo = RobotSystem.hardwareMap.servo.get("grabber");

        release();

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

        reset();
    }

    public static void reset(){
        offset = 0;
    }
    // endregion

    // region FUNCTIONALITY
    public static void release() { grabberServo.setPosition(releasePosition   ); }
    public static void grab()        { grabberServo.setPosition(grabPosition      ); }

    public static void goToCone(int coneHeight){ setPosition(pilePositions[coneHeight]); }
    public static void goToConeSlow(int newConeHeight){
        coneHeight = newConeHeight;
        asyncGoToConeSlow.start();
    }
    private static int coneHeight = 0;
    private static final Thread asyncGoToConeSlow = new Thread(() -> {
        for (double i = 0; i <= 1; i += 1.0 / slowIncrements) {
            setPosition(inPosition + (pilePositions[coneHeight] - inPosition) * i);
            try {
                Thread.sleep(5);
            } catch (InterruptedException e){}
        }
    });
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
    public static boolean isMoving() {
        return asyncGoToConeSlow.isAlive();
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
        rightServo.setPosition(position + offset);
        leftServo .setPosition(position + offset);
    }
    // endregion
}
