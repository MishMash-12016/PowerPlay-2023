package org.firstinspires.ftc.teamcode.opModes.autonomousOpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.driveTrain;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.opencv.core.Mat;


@Autonomous(group = "competition op modes")
public class safeAutonomousLeft extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotSystem.safeAuto.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);
        RobotSystem.safeAuto.initializeTrajectories(true);

        waitForStart();
        RobotSystem.safeAuto.checkParkingPosition();

        RobotSystem.safeAuto.startAllAutonomousControllers(this::opModeIsActive);
        puffer.grab();
        puffer.goToMid();

        try {
            RobotSystem.safeAuto.follow(RobotSystem.safeAuto.trajectories.get("startToScore"));

            RobotSystem.safeAuto.cycle(1);
            RobotSystem.safeAuto.park();
        } catch (Exception e){
            RobotSystem.safeAuto.terminate();
        }

        while (opModeIsActive() && !RobotSystem.isStopRequested){}

        RobotSystem.safeAuto.terminate();
    }
}
