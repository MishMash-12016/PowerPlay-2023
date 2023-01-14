package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveController driveController = new DriveController(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();

        while (opModeIsActive()) {
            if (Y_pressed()) driveController.setElevatorPosition(20500);
            if (X_pressed()) driveController.setElevatorPosition(12000);
            if (A_pressed()) driveController.setElevatorPosition(5000 );

            driveController.update();
        }
    }
    private boolean a = true;
    public boolean A_pressed(){
        if (gamepad1.a){
            if(a){
                a = false;
                return true;
            }
        } else a = true;
        return false;
    }

    private boolean x = true;
    public boolean X_pressed(){
        if (gamepad1.x){
            if(x){
                x = false;
                return true;
            }
        } else x = true;
        return false;
    }

    private boolean y = true;
    public boolean Y_pressed(){
        if (gamepad1.y){
            if(y){
                y = false;
                return true;
            }
        } else y = true;
        return false;
    }


    private boolean right_trigger1 = false;
    public boolean right_trigger_pressed(){
        if (gamepad1.right_trigger > 0){
            if(!right_trigger1){
                right_trigger1 = true;
                return true;
            }
        } else right_trigger1 = false;
        return false;
    }

    private boolean right_trigger_was_pressed = false;
    public boolean right_trigger_released(){
        if (gamepad1.right_trigger == 0){
            if(right_trigger_was_pressed){
                right_trigger_was_pressed = false;
                return true;
            }
        } else right_trigger_was_pressed = true;
        return false;
    }
}