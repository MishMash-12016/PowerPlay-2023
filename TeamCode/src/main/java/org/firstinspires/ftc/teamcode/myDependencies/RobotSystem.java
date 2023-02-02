package org.firstinspires.ftc.teamcode.myDependencies;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.*;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.trajectorysequence.TrajectorySequence;

import java.util.concurrent.Callable;

public class RobotSystem {
    // region SYSTEM CONSTANTS
    public static final InterruptedException hardStopRequest = new InterruptedException("stop everything !");
    public static final InterruptedException softStopRequest = new InterruptedException("stop running safely");

    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;
    // endregion

    // region SYSTEM VARIABLES
    public static boolean isStopRequested;
    public static int currentConeHeight;
    // endregion

    // region INITIALIZATION
    public static void initializeAll(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        RobotSystem.initialize(hardwareMap, telemetry, gamepad1, gamepad2);

        arm.initialize();
        puffer.initialize();
        grabber.initialize();
        elevator.initialize();
        driveTrain.initialize();
    }
    public static void resetAll(){
        RobotSystem.reset();

        led.reset();
        elevator.reset();
        driveTrain.reset();
    }

    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        RobotSystem.reset();

        RobotSystem.hardwareMap = hardwareMap;
        RobotSystem.telemetry   = telemetry  ;
        RobotSystem.gamepad1    = gamepad1   ;
        RobotSystem.gamepad2    = gamepad2   ;
    }
    public static void reset(){
        isStopRequested = false;
        currentConeHeight = 0;
    }
    // endregion

    // region SYSTEM FUNCTIONALITY
    public static void startAllTeleOpControllers(){
        RobotSystem.cycleController.start();
        driveTrain.controller.start();
        elevator.controller.start();
    }
    public static void startAllAutonomousControllers(){
        elevator.controller.start();

    }

    public static void terminate(){
        resetAll();
        isStopRequested = true;

        telemetry.update();
    }
    // endregion

    // region GENERAL FUNCTIONALITY
    private static final ElapsedTime timePassed = new ElapsedTime();
    public static void sleep(int wantedSleepTime) throws InterruptedException{
        timePassed.reset();
        while(wantedSleepTime > timePassed.milliseconds()){
            if (isStopRequested) {
                throw hardStopRequest;
            }
            Thread.sleep(20);
        }
    }
    public static void await(Callable<Boolean> requirement) throws InterruptedException{
        try{
            while (!requirement.call() && !isStopRequested) {
                Thread.sleep(20);
            }
        }catch (Exception e){
            throw softStopRequest;
        }
        if (isStopRequested) {
            throw hardStopRequest;
        }
    }

    public static class FirstPress{
        public static boolean a = true;
        public static boolean A(){
            if (gamepad1.a){
                if(a){
                    a = false;
                    return true;
                }
            } else a = true;
            return false;
        }

        public static boolean x = true;
        public static boolean X(){
            if (gamepad1.x){
                if(x){
                    x = false;
                    return true;
                }
            } else x = true;
            return false;
        }

        public static boolean y = true;
        public static boolean Y(){
            if (gamepad1.y){
                if(y){
                    y = false;
                    return true;
                }
            } else y = true;
            return false;
        }

        public static boolean b = true;
        public static boolean B(){
            if (gamepad1.b){
                if(b){
                    b = false;
                    return true;
                }
            } else b = true;
            return false;
        }
    }
    // endregion

    // region DRIVER CONTROLLED FUNCTIONS
    private static final Thread cycleController = new Thread(() -> {
        while (!isStopRequested && !RobotSystem.cycleController.isInterrupted()) {
            if (!manual.asyncCollect.isAlive() && (gamepad1.left_trigger > 0 || gamepad1.left_bumper)) {
                manual.collect();
            }

            if      (FirstPress.A()) { manual.score(elevator.highPosition); }
            else if (FirstPress.X()) { manual.score(elevator.middlePosition); }
            else if (FirstPress.Y()) { manual.score(elevator.lowPosition); }
            else if (FirstPress.B()) { elevator.setWantedPosition(elevator.bottomPosition); }
        }
    });

    public static class manual{
        public static void collect() {
            asyncCollect.start();
        }
        public static void score(int height) {
            elevator.setWantedPosition(height);
            if (!manual.asyncScore.isAlive()){
                asyncScore.start();
            }
        }

        public static final Thread asyncCollect = new Thread(() -> {
            try {
                driveTrain.slowMode();
                grabber.goToCone(currentConeHeight);
                grabber.fullRelease();
                while (!RobotSystem.manual.asyncCollect.isInterrupted() && !isStopRequested && (gamepad1.left_trigger > 0 || gamepad1.left_bumper)) {
                    arm.goToRelativePosition(gamepad1.left_trigger);

                    if (grabber.coneIsInRange()) {
                        if (!grabber.isGrabbing()) {
                            grabber.grab();
                        }
                    } else {
                        if (grabber.isGrabbing()) {
                            grabber.fullRelease();
                        }
                    }
                }

                driveTrain.fastMode();

                if (isStopRequested){
                    throw hardStopRequest;
                }

                if (RobotSystem.manual.asyncCollect.isInterrupted()){
                    throw softStopRequest;
                }

                grabber.goToMid();

                arm.goToRelativePosition(0);

                await(() -> !arm.isOut() && !elevator.isUp());

                grabber.goToIn();

                await(() -> !grabber.isOut());

                grabber.midRelease();

                if (grabber.hasCone()) {
                    puffer.grab();
                    puffer.goToMid();
                }

            }catch (Exception e){
                if (e == softStopRequest){
                    arm.goToRelativePosition(0);
                    grabber.midRelease();
                    grabber.goToIn();
                } else if (e != hardStopRequest){
                    telemetry.addData("stopped manual collect completely", e);
                    telemetry.update();
                }
            }
        });
        public static final Thread asyncScore = new Thread(()-> {
            try {
                await(elevator::almostReachedWantedPosition);

                puffer.goToOut();

                await(() -> (gamepad1.right_trigger > 0 || RobotSystem.manual.asyncScore.isInterrupted()));

                if (manual.asyncScore.isInterrupted()){
                    throw softStopRequest;
                }

                puffer.release();

                await(() -> gamepad1.right_trigger == 0 || RobotSystem.manual.asyncScore.isInterrupted());

                throw softStopRequest;
            }catch (InterruptedException e1){
                if (e1.equals(softStopRequest)){
                    try {
                        puffer.goToIn();

                        sleep(200);

                        elevator.setWantedPosition(elevator.bottomPosition);

                        await(() -> !elevator.isUp());

                        puffer.release();
                    } catch (Exception e2){
                        telemetry.addData("stopped manual score from safely stopping", e2);
                        telemetry.update();
                    }
                }
            } catch (Exception e){
                telemetry.addData("stopped manual score completely", e);
                telemetry.update();
            }
        });
    }
    // endregion

    // region AUTONOMOUS FUNCTIONS
    /*
        Pose2d scorePose = new Pose2d(36.0, 12.0, Math.toRadians(Math.toRadians(-140.0)));
        Pose2d startPose  = new Pose2d(30.7, 61.4, Math.toRadians(Math.toRadians(-90.0)));

        Pose2d score_to_parking1_tempPose_1 = new Pose2d(36.0, 24.0, Math.toRadians(90.0));
        Pose2d parking1Pose = new Pose2d(48.0, 36.0, Math.toRadians(0.0));

        TrajectorySequence score_to_parking1_trajectory = drive.trajectorySequenceBuilder(scorePose).
                setTangent(Math.toRadians(90.0)).
                splineToLinearHeading(score_to_parking1_tempPose_1, Math.toRadians(-90.0)).
                splineToLinearHeading(parking1Pose, Math.toRadians(0.0)).build();

        Pose2d parking2Pose = new Pose2d(36.0, 24.0, Math.toRadians(90.0));

        TrajectorySequence score_to_parking2_trajectory = drive.trajectorySequenceBuilder(scorePose).
                setTangent(Math.toRadians(90.0)).
                splineToLinearHeading(parking2Pose, Math.toRadians(-90.0)).build();

        Pose2d score_to_parking3_tempPose_1 = new Pose2d(36.0, 24.0, Math.toRadians(90.0));
        Pose2d parking3Pose = new Pose2d(24.0, 36.0, Math.toRadians(180.0));

        TrajectorySequence score_to_parking3_trajectory = drive.trajectorySequenceBuilder(scorePose).
                setTangent(Math.toRadians(90.0)).
                splineToLinearHeading(score_to_parking3_tempPose_1, Math.toRadians(-90.0)).
                splineToLinearHeading(parking3Pose, Math.toRadians(0.0)).build();

        Pose2d start_to_score_tempPose_1 = new Pose2d(36.0, 46.0, Math.toRadians(-90.0));

        TrajectorySequence start_to_score_trajectory = drive.trajectorySequenceBuilder(startPose).
                setTangent(Math.toRadians(-60.0)).
                splineToLinearHeading(start_to_score_tempPose_1, Math.toRadians(0.0)).
                splineToLinearHeading(scorePose, Math.toRadians(0.0)).build();

    */
    public static class auto{
        // region INITIALIZATION
        public static void initialize(){
            // region INITIALIZE DRIVE
            drive = new SampleMecanumDrive(hardwareMap);
            drive.setPoseEstimate(positions.start);
            // endregion

            // region CREATE TRAJECTORIES
            // region START TO SCORE
            trajectories.startToScore = drive.trajectorySequenceBuilder(positions.start).build();
            // endregion

            // region SCORE TO COLLECT
            trajectories.scoreToCollect = drive.trajectorySequenceBuilder(positions.score).build();
            // endregion

            // region COLLECT TO SCORE
            trajectories.collectToScore = drive.trajectorySequenceBuilder(positions.collect).build();
            // endregion

            // region SCORE TO PARK1
            trajectories.scoreToPark1 = drive.trajectorySequenceBuilder(positions.score).build();
            // endregion

            // region SCORE TO PARK2
            trajectories.scoreToPark2 = drive.trajectorySequenceBuilder(positions.score).build();
            // endregion

            // region SCORE TO PARK3
            trajectories.scoreToPark3 = drive.trajectorySequenceBuilder(positions.score).build();
            // endregion
            // endregion
        }

        public static void terminate(){

        }
        // endregion

        // region MOVEMENT
        private static SampleMecanumDrive drive;
        public static class trajectories{
            public static TrajectorySequence startToScore;
            public static TrajectorySequence scoreToCollect;
            public static TrajectorySequence collectToScore;
            public static TrajectorySequence scoreToPark1;
            public static TrajectorySequence scoreToPark2;
            public static TrajectorySequence scoreToPark3;
        }
        private static class positions{
            public static final Pose2d start   = new Pose2d();
            public static final Pose2d score   = new Pose2d();
            public static final Pose2d collect = new Pose2d();
            public static final Pose2d park1   = new Pose2d();
            public static final Pose2d park2   = new Pose2d();
            public static final Pose2d park3   = new Pose2d();
        }

        public static void follow(TrajectorySequence sequence) throws InterruptedException{
            drive.followTrajectorySequenceAsync(sequence);
            while (drive.isBusy()){
                drive.update();
                if (isStopRequested) {
                    throw hardStopRequest;
                }
            }
        }
        // endregion

        // region CYCLE
        private static final Thread cycle = new Thread(() -> {

        });

        public static void score(boolean prepareForNext, int nextConeNum){}
        public static void collect(int coneNum){}
        // endregion
    }
    // endregion
}
