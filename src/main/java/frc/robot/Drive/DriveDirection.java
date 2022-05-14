package frc.robot.Drive;

public enum DriveDirection {
    BACKWARD(-1),
    FORWARD(1);

    private final double direction;

    private DriveDirection(double direction) {
        this.direction = direction;
    }

    protected double get() {
        return this.direction;
    }
}