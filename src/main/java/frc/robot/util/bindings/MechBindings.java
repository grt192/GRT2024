package frc.robot.util.bindings;

/** A set of control bindings for the robot mechanisms. */
public interface MechBindings {

    /** Sets the mech controls entirely to this set of bindings. */
    public void setAllBindings();

    /** Sets the binds for the intake. */
    public void bindIntaking();

    /** Sets the binds for the elevator. */
    public void bindAmp();

    /** Sets the binds for the trap sequence. */
    public void bindTrap();

    /** Sets the binds for shooting. */
    public void bindShooting();

}
