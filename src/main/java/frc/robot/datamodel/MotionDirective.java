package frc.robot.datamodel;

import edu.wpi.first.math.util.Units;

public class MotionDirective {
    public static enum MotionType {
        DRIVE,
        TURN,
        STRAFE,
        DROP_CORAL,
        WAIT,
        STOP,
        GET_CORAL,
        APRILTAG_DRIVE,
        CAMERA_DRIVE,
        CAMERA_STRAFE,
        CAMERA_ROTATE,
        CAMERA_ALL
    }
    
    private final MotionType type;

    private final double amount;

    private MotionDirective(MotionType type, double amount) {
        this.type = type;
        this.amount = amount; 
    }

    public static MotionDirective turn(double amount) {
        return new MotionDirective(MotionType.TURN, amount);
    }
    public static MotionDirective drive(double amount) {
        return new MotionDirective(MotionType.DRIVE, Units.inchesToMeters(amount));
    }
    public static MotionDirective strafe(double amount) {
        return new MotionDirective(MotionType.STRAFE, Units.inchesToMeters(amount));
    }
    public static MotionDirective dropCoral() {
        return new MotionDirective(MotionType.DROP_CORAL, 0);
    }
    public static MotionDirective wait2(double amount) {
        return new MotionDirective(MotionType.WAIT, amount);
    }
    public static MotionDirective stop() {
        return new MotionDirective(MotionType.STOP, 0);
    }
    public static MotionDirective GetCoral() {
        return new MotionDirective(MotionType.GET_CORAL, 0);
    }
    public static MotionDirective apriltag() {
        return new MotionDirective(MotionType.CAMERA_ALL, 0);
    }

    public MotionType getType() {
        return type;
    }

    public double getAmount() {
        return amount;
    }

    @Override
    public String toString() {
        return "[type=" + type.name() + ", amount=" + amount + "]";
    }

    
}
