
/** 
 * drive by camera controller **
 *
  The drive by camera controller is a simple controller that gets
information from the camera, and attempts to drive toward the thing that is
being tracked.  It would have four states: Disabled, CameraWait, Pivot, and
Drive.

  Disabled is the initial state.  In the disabled state, the controller does
nothing.  It stays in the disabled state until someone calls dbc_Drive(
velocity ), at which point, it moves to the CameraWait state.

  The dbc controller remains in the CameraWait state until the camera i/o
driver is in the Tracking state.  If dbc_Disable() has been called since the
last cycle, the controller transitions back to the Disabled state.  if
dbc_Disable has not been called, and the camera i/o driver has entered the
Tracking state, the dbc controller transitions to the Pivot state.

  In the Pivot state, it simply pivots the robot, without driving, until the
camera is lined up to 0 radians (ie. pointing straight ahead).  When the
camera pan is 0 radians, it transitions to the Drive state.  If,
dbc_Disable() has been called since the last cycle, the controller returns
to the disabled state.  if, at any point, the camera leaves the Tracking
state, and enters the Scanning state, the dbc controller returns to the
CameraWait state.  if the camera enters the Idle state, the dbc controller
must return to the Disabled state.

  In the drive state, the controller moves the robot forward at the velocity
that was specified in the call to dbc_Drive( velocity ).  each cycle, if the
pan of the camera is not 0, the controller provides the wheel controller
with a desiredTurningSpeed in the correct direction, so as to bring the
camera back to 0.  If the pan of the camera is 0, then the controller
provides a 0 turning speed to the wheel controller, and simply drives the
bot in a straight line.  if dbc_Disable() has been called since the last
cycle, the controller will return to the Disabled state. if, at any point,
the camera leaves the Tracking state, and enters the Scanning state, the dbc
controller returns the CameraWait state.  if the camera enters the Idle
state, the dbc controller must return to the Disabled state.

This controller assumes the following things:
  1) when in Pivot or Drive mode, this controller is the only thing sending
commands to the wheelController.
  2) in order to get actual drive action, someone outside the dbc controller
must put the camera i/o driver into Scanning/Tracking modes.
 *
 **/

  typedef enum {
    disabled,
    camerawait,
    pivot,
    drive

  } dbc_Mode;

  /**
   * set the controller to Disabled mode.  Causes the controller to do nothing
   */
  void dbc_Disable();

  /**
   * tell the controller to start the drive cycle.  begins in CameraWait mode, then moves
   * to Pivot mode once the camera is Tracking, and finally to drive mode,
   * once we're in line with the camera.  The specified velocity will be used to
   * drive the robot.  velocity is specified in mm/s.
   *
   * while in the Drive state, dbc_Drive( velocity ) may be called to change the
   * current drive velocity.  If 0 is specified as a drive velocity, then the controller
   * will transition back to Disabled state.
   */
  void dbc_Drive( int velocity );

  /**
   * returns the current mode of the dbc controller
   */
  dbc_Mode dbc_getMode();

  /**
   * returns the current driving velocity of the dbc controller.
   * this value is only valid when not in Mode Disabled.
   */
  int dbc_getVelocity();

  /**
   * main loop of the dbc controller.
   * called by the framework every 26.6 ms
   */
  void dbc_doMain();


  /*

  Internal state of the dbc controller would look like this:

  static dbc_Mode currentState = dbc_Mode.disabled;

  static char shouldDrive = 0;      // flag. set in dbc_Drive(...)
                                                  // tells the dbc to
transition to camerawait

  static char shouldDisable = 0;   // flag. set in dbc_Disable()
                                                  // tells the dbc to
transition to disabled

  static int velocity = 0;               // set in dbc_Drive(...).
                                                 // remembers the velocity
we should drive at.

inputs to the state machine are:
  shouldDisable
  shouldDrive
  camera_getMode()
  camera_getPan()
  velocity

outputs of the state machine are:
  wheels_setSpeed(...)
  wheels_setTurningSpeed(...)
  currentState

  */
