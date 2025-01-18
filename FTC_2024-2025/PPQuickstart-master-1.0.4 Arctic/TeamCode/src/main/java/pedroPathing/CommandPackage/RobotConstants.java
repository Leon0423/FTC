package pedroPathing.CommandPackage;

public class RobotConstants {
    // * Everything that we want to store globally, for example positions of servos, motors, etc. goes in here.
    // * Variables are positions for the claw servos.

    // * Claw positions
    public static double CloseOutputClawPosition = 0.0;
    public static double OpenOutputClawPosition = 0.15;
    public static double BasketArmPosition;
    public static double BasketCenterPosition;
    public static double SpecimenArmPosition;
    public static double SpecimenCenterPosition;

    // * Slide positions
    public static int BasketSlideTargetPosition;
    public static int SpecimenSlideMinimumTargetPosition;
    public static int DefaultSlidePosition = 0;
    public static double SlidePower = 0.5;
    public static double SlideStopPower = 0.00001;
}