package pedroPathing.CommandPackage;

public class RobotConstants {
    // * Everything that we want to store globally, for example positions of servos, motors, etc. goes in here.
    // * Variables are positions for the claw servos.

    // * Claw positions
    public static double CloseOutputClawPosition = 0.0;
    public static double OpenOutputClawPosition = 0.15;

    // * Arm positions
        // * Basket positions
        public static double BasketArmCommand_ArmPosition;
        public static double BasketArmCommand_CenterPosition;
        public static boolean BasketArmCommand_ClawPosition;
        // * Specimen positions
        public static double SpecimenArmCommand_ArmPosition;
        public static double SpecimenArmCommand_CenterPosition;
        public static boolean SpecimenArmCommand_ClawPosition;
        // * Transfer positions
        public static double TransferArmCommand_ArmPosition;
        public static double TransferArmCommand_CenterPosition;
        public static boolean TransferArmCommand_ClawPosition;

    // * Slide positions
    public static int BasketSlideTargetPosition;
    public static int SpecimenSlideMinimumTargetPosition;
    public static int DefaultSlidePosition = 0;
    public static double SlidePower = 0.5;
    public static double SlideStopPower = 0.00001;

    // * Intake positions
    public static double IntakeSubsystem_OpenClawPosition;
    public static double IntakeSubsystem_CloseClawPosition;
    public static double IntakeSubsystem_LeftArmPosition;
    public static double IntakeSubsystem_RightArmPosition;






    // * TransferSampleOutputCommand_Position
    public static double TransferSampleOutputCommand_RotatePosition;
    public static double TransferSampleOutputCommand_CenterPosition;
    public static boolean TransferSampleOutputCommand_ClawPosition;


}