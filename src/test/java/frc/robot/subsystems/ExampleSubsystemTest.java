package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Unit test examples. See
 * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/unit-testing.html
 * for more examples
 */
public class ExampleSubsystemTest {
  private ExampleSubsystem mExampleSubsystem;

  /**
   * Insert code that runs before each test here.
   */
  @BeforeEach
  public void setup() {
    this.mExampleSubsystem = new ExampleSubsystem();
  }

  /**
   * Basic example unit test that ensures that exampleCondition is false.
   */
  @Test
  public void exampleConditionTest() {
    // You can run all sorts of stuff here and then set expectations on what should happen

    assertEquals(false, mExampleSubsystem.exampleCondition());
  }

}
