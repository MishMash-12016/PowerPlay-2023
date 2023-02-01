package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;

public class elevator {
    // region MOTORS
    private static DcMotorEx motorRight;
    private static DcMotorEx motorLeft ;
    // endregion

// region CONSTANTS
    public static final int highPosition   = 17500;
    public static final int middlePosition = 11000;
    public static final int lowPosition    = 4000 ;
    public static final int bottomPosition = 0    ;

    private static final double marginOfError = 100;
    private static final double smoothness    = 250;
    private static final double holdingPower  = 0.3;
    // endregion

    // region SENSOR
    public static DigitalChannel isUpSensor;
    // endregion

    // region VARIABLES
    public static double wantedPosition;

    private static double motorPower;
    // endregion

    // region INITIALIZATION
    public static void initialize() {
        // region MOTORS
        motorLeft  = (DcMotorEx) RobotSystem.hardwareMap.dcMotor.get("elevatorLeft" );
        motorRight = (DcMotorEx) RobotSystem.hardwareMap.dcMotor.get("elevatorRight");

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // endregion

        // region SENSOR
        isUpSensor = RobotSystem.hardwareMap.get(DigitalChannel.class, "elevatorIsUpSensor");
        isUpSensor.setMode(DigitalChannel.Mode.INPUT);
        // endregion

        // region VARIABLES
        motorPower = 0;
        wantedPosition = 0;
        // endregion
    }

    public static void reset(){
        motorPower = 0;
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
        wantedPosition = newWantedPosition;

        if (wantedPosition == elevator.bottomPosition){
            driveTrain.fastMode();
            RobotSystem.manual.asyncScore.interrupt();
        } else {
            driveTrain.slowMode();
        }
    }

    public static boolean isUp(){
        return isUpSensor.getState();
    }
    public static boolean almostReachedWantedPosition(){
        return 1000 > Math.abs(wantedPosition - getCurrentPosition());
    }

    public static String deBug(){
        return "left motor power : " + motorLeft.getPower()
                + "\nright motor power : " + motorRight.getPower()
                + "\nheight : " + motorLeft.getCurrentPosition();
    }
    // endregion

    // region PRIVATE FUNCTIONALITY
    private static void setMotorPower(double motorPower){
        motorLeft .setPower(motorPower);
        motorRight.setPower(motorPower);
    }

    private static double calculatePower(double x){
        if (!isUp() && wantedPosition == 0){
            return 0;
        }

        if (Math.abs(x) < marginOfError){
            return holdingPower;
        } else if (x > smoothness * 4 + marginOfError){
            return 0;
        } else if (-x > smoothness + marginOfError){
            return 1;
        }

        if (x > 0){
            x -= marginOfError;
        } else {
            x += marginOfError;
        }


        if (x > 0){
            return ((Math.cos((x) * Math.PI / smoothness / 4) - 1) / 2) * holdingPower + holdingPower;
        } else {
            return ((1 - Math.cos(x * Math.PI / smoothness)) / 2) * (1 - holdingPower) + holdingPower;
        }
    }

    private static double getCurrentPosition() {
        return motorLeft.getCurrentPosition();
    }
    // endregion

}
