package org.firstinspires.ftc.teamcode.myDependencies;

public class System {
    /** SYSTEM FUNCTIONALITY */
    static class General{
        public Thread elevatorController;
        public Thread driveController;
        public Thread gamepadListener;
    }

    /** DRIVER CONTROLLED FUNCTIONS */
    static class manual{
        private Thread score;
        private Thread collect;
    }

    /** AUTONOMOUS FUNCTIONS */
    static class auto{
        public Thread autoCycle;
    }


}
