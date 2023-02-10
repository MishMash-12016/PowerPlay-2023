package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;

public class elevator {
    // region MOTORS
    private static DcMotorEx motorRight;
    private static DcMotorEx motorMiddle;
    private static DcMotorEx motorLeft ;
    // endregion

    // region CONSTANTS
    public static final int highPosition   = 17500;
    public static final int middlePosition = 11300;
    public static final int lowPosition    = 4300 ;
    public static final int bottomPosition = 0    ;

    private static final double marginOfError = 100;
    private static final double smoothness    = 800;

    private static final double goingUpPower   = 1;
    private static final double holdingPower   = 0.3;
    private static final double goingDownPower = -0.2;
    // endregion

    // region SENSOR
    private static DigitalChannel isUpSensor;
    // endregion

    // region VARIABLES
    public static double wantedPosition;
    // endregion

    // region INITIALIZATION
    public static void initialize() {
        // region MOTORS
        motorLeft   = (DcMotorEx) RobotSystem.hardwareMap.dcMotor.get("elevatorLeft"  );
        motorMiddle = (DcMotorEx) RobotSystem.hardwareMap.dcMotor.get("elevatorMiddle");
        motorRight  = (DcMotorEx) RobotSystem.hardwareMap.dcMotor.get("elevatorRight" );

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // endregion

        // region SENSOR
        isUpSensor = RobotSystem.hardwareMap.get(DigitalChannel.class, "elevatorIsUpSensor");
        isUpSensor.setMode(DigitalChannel.Mode.INPUT);
        // endregion

        // region VARIABLES
        wantedPosition = 0;
        // endregion
    }

    public static void reset(){
        wantedPosition = 0;

        if (motorLeft != null){
            setMotorPower(0);
        }
    }
    // endregion

    // region FUNCTIONALITY
    public static Thread controller = new Thread(() -> {
        while (!RobotSystem.isStopRequested && !elevator.controller.isInterrupted()){
            setMotorPower(calculatePower(motorLeft.getCurrentPosition() - wantedPosition));
        }
    });

    public static void setWantedPosition(int newWantedPosition){
        if (newWantedPosition == elevator.bottomPosition){
            if (!RobotSystem.manual.asyncCollect.isAlive() && !RobotSystem.manual.asyncHighCollect.isAlive()){
                driveTrain.fastMode();
            }
            RobotSystem.manual.asyncScore.interrupt();
        } else {
            puffer.grab();
            driveTrain.slowMode();
            if (wantedPosition == elevator.bottomPosition) {
                puffer.goToMid();
            } else {
                puffer.goToOut();
            }
        }

        wantedPosition = newWantedPosition;
    }

    public static boolean isUp(){
        return isUpSensor.getState();
    }
    public static boolean almostReachedWantedPosition(){
        return 6000 > Math.abs(wantedPosition - getCurrentPosition());
    }

    public static String deBug(){
        return  "left motor power : "    + motorLeft.getPower() +
                "\nright motor power : " + motorRight.getPower() +
                "\nheight : "            + motorLeft.getCurrentPosition();
    }
    // endregion

    // region PRIVATE FUNCTIONALITY
    private static void setMotorPower(double motorPower){
        motorLeft  .setPower(motorPower);
        motorMiddle.setPower(motorPower);
        motorRight .setPower(motorPower);
    }

    private static double calculatePower(double distanceToWantedPosition){
        if (!isUp() && wantedPosition == 0){
            led.green();
            return 0;
        }
        led.red();

        if (Math.abs(distanceToWantedPosition) < marginOfError){
            return holdingPower;
        } else if (distanceToWantedPosition > smoothness * 4 + marginOfError){
            return goingDownPower;
        } else if (-distanceToWantedPosition > smoothness + marginOfError){
            return goingUpPower;
        }

        if (distanceToWantedPosition > 0){
            distanceToWantedPosition -= marginOfError;
        } else {
            distanceToWantedPosition += marginOfError;
        }


        if (distanceToWantedPosition > 0){
            return ((Math.cos((distanceToWantedPosition) * Math.PI / smoothness / 4) - 1) / 2) * (holdingPower - goingDownPower) + holdingPower;
        } else {
            return ((1 - Math.cos(distanceToWantedPosition * Math.PI / smoothness)) / 2) * (goingUpPower - holdingPower) + holdingPower;
        }
    }

    public static double getCurrentPosition() {
        return motorLeft.getCurrentPosition();
    }
    // endregion

}
