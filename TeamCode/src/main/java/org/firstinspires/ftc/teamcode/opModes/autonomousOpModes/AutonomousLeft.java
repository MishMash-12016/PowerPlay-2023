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
public class AutonomousLeft extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotSystem.regularAuto.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);
        RobotSystem.regularAuto.initializeTrajectories(true);

        waitForStart();
        RobotSystem.regularAuto.checkParkingPosition();

        RobotSystem.regularAuto.startAllAutonomousControllers(this::opModeIsActive);
        puffer.grab();
        puffer.goToMid();

        try {
            RobotSystem.regularAuto.follow(RobotSystem.regularAuto.trajectories.get("startToScore"));
            RobotSystem.regularAuto.cycle(5);
            RobotSystem.regularAuto.park();
        } catch (Exception e){
            RobotSystem.regularAuto.terminate();
        }

        while (opModeIsActive() && !RobotSystem.isStopRequested){}

        RobotSystem.regularAuto.terminate();
    }
}
