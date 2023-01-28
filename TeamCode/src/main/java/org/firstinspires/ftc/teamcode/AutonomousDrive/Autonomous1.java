package org.firstinspires.ftc.teamcode.AutonomousDrive;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunnerUtility.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunnerUtility.trajectorysequence.TrajectorySequence;

@Autonomous
public class Autonomous1 extends LinearOpMode {
    SampleMecanumDrive drive;
    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d scorePose = new Pose2d(36.0, 12.0, Math.toRadians(Math.toRadians(-140.0)));
        Pose2d startPose = new Pose2d(30.7, 61.4, Math.toRadians(Math.toRadians(-90.0)));

        // region score_to_parking1
        Pose2d score_to_parking1_tempPose_1 = new Pose2d(36.0, 24.0, Math.toRadians(90.0));
        Pose2d parking1Pose = new Pose2d(48.0, 36.0, Math.toRadians(0.0));

        TrajectorySequence score_to_parking1_trajectory = drive.trajectorySequenceBuilder(scorePose).
                setTangent(Math.toRadians(90.0)).
                splineToLinearHeading(score_to_parking1_tempPose_1, Math.toRadians(-90.0)).
                splineToLinearHeading(parking1Pose, Math.toRadians(0.0)).build();
        // endregion

        // region score_to_parking2
        Pose2d parking2Pose = new Pose2d(36.0, 24.0, Math.toRadians(90.0));

        TrajectorySequence score_to_parking2_trajectory = drive.trajectorySequenceBuilder(scorePose).
                setTangent(Math.toRadians(90.0)).
                splineToLinearHeading(parking2Pose, Math.toRadians(-90.0)).build();
        // endregion

        // region score_to_parking3
        Pose2d score_to_parking3_tempPose_1 = new Pose2d(36.0, 24.0, Math.toRadians(90.0));
        Pose2d parking3Pose = new Pose2d(24.0, 36.0, Math.toRadians(180.0));

        TrajectorySequence score_to_parking3_trajectory = drive.trajectorySequenceBuilder(scorePose).
                setTangent(Math.toRadians(90.0)).
                splineToLinearHeading(score_to_parking3_tempPose_1, Math.toRadians(-90.0)).
                splineToLinearHeading(parking3Pose, Math.toRadians(0.0)).build();
        // endregion

        // region start_to_score
        Pose2d start_to_score_tempPose_1 = new Pose2d(36.0, 46.0, Math.toRadians(-90.0));

        TrajectorySequence start_to_score_trajectory = drive.trajectorySequenceBuilder(startPose).
                setTangent(Math.toRadians(-60.0)).
                splineToLinearHeading(start_to_score_tempPose_1, Math.toRadians(0.0)).
                splineToLinearHeading(scorePose, Math.toRadians(0.0)).build();
        // endregion

        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();

        drive.followTrajectorySequence(start_to_score_trajectory);

        sleep(700);

        if (gamepad1.x){
            drive.followTrajectorySequence(score_to_parking1_trajectory);
        } else if (gamepad1.b){
            drive.followTrajectorySequence(score_to_parking3_trajectory);
        } else {
            drive.followTrajectorySequence(score_to_parking2_trajectory);
        }
    }
}