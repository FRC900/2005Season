/** 
 * camera i/o driver 
 *
 * The camera i/o driver is a wrapper around the basic camera functions.
 **/

  typedef enum {
    idle,                  // the camera is not actively doing anything
    scanning,              // the camera is currently scanning for a color
    tracking               // the camera is currently tracking a color

  } camera_Mode;

  typedef enum {
    yellow,
    green,
	white,
    red,
    blue

  } camera_Color;

  typedef enum {
    left,
    right
  
  }camera_Direction;

  /**
   * set the pan of the camera in radians
   */
  void camera_setPan(int radians);

  /**
   * returns the current pan of the camera in radians
   */
  int camera_getPan()

  /**
   * tell the camera to enter scanning mode.  When this mode is entered,
   * the camera begins scanning for the color specified in whatColor.
   * the scan would start in the direction specified by startDirection, proceed
   * to the maximum pan in the specified direction, then begin scanning in the
   * other pan direction.  the camera stays in scanning mode until either a color
   * is found, or some other function call changes the camera's mode.
   *
   * if a color is found while scanning, then the camera transitions to tracking mode.
   * In tracking mode, the camera makes whatever adjustments are necessary to
   * follow the area of color that it has found.  If it should "loose" the color (because
   * the color moved too quickly to keep up with, or any other reason), it should transition
   * back to scanning mode, starting in the direction of the last camera movemvent.
   */
  void camera_scanForColor(camera_Color whatColor, camera_Direction startDirection);

  /**
   * tell the camera to enter idle mode.  Stops all scanning or tracking that be ongoing
   */
  void camera_idle();

  /**
   * returns the current mode of the camera.
   */
  camera_Mode camera_getMode();

  /**
   * returns the color that the camera is currently scanning or tracking to
   */
  camera_Color camera_getColor();


  /**
   * main loop of camera i/o driver.  called by framework every .0001s
   *
   *  JLC: is camera on user processor or or master processor??
   *          if on master processor, then we'll move this call to the 26.6ms loop
   *          if on user processor, we'll leave it at .0001s
   */
  void camera_doMain();


/*
There would be two sub-systems that make up the camera i/o driver:

  1) a camera movement subsystem, that would, support three functions:
     a) if input, scanLeft, is set, it would adjust targetPan slightly to
the left.
     b) if input, scanRight, is set, it would adjust targetPan slighty to
right.
     c) if actual camera pan != targetPan, it would turn the camera to match
targetPan.
    It would generate two outputs:
      currentPan            // the actual current pan of the camera
      boundryReached  // =1 when camera is at max right or max left pan

  2) a state machine that would output scanLeft and scanRight based on
inputs shouldScan, shouldIdle, initialScanDirection, boundryReached, info
coming back from camera.

Internal state of the camera i/o driver would look something like this
(lives in the .c file):

//*** movement subsystem ***
/static int targetPan = 0;            // set by camera_setPan(...).
                                               // remembers what location we
should pan camera to

static char scanLeft = 0;        // set as output of state machine
static char scanRight = 0;      // set as output of state machine

// *** State machine state & inputs ***
static camera_Mode currentMode = camera_Mode.idle;    // current state of
machine
  // may need additional state information other than mode, to keep track of
current
  // scanning direction, and other stuff...

static int initialScanDirection;      // set in camera_scanForColor(...).
                                                 // remembers the direction
we should start scanning in.

static char shouldScan = 0;        // input flag. set by
camera_scanForColor(...).
                                                 // tells state machine to
transition on next cycle.
                                                 // should be cleared when
transition is complete

static char shouldIdle = 0;          // input flag. set by camera_Idle().
                                                 // tells state machine to
transition on next cycle.
                                                 // should be cleared when
transition is complete

static char boundryReached =0;  // output from movement subsystem.
                                                  // indicates that camera
pan is at a left or right boundry.

inputs to the state machine are:
  scanDirection
  shouldScan
  shouldIdle
  initialScanDirection
  boundryReached
  [from the camera]
    {found the color}
    {color moving left}
    {color moving right}

outputs of the state machine are:
  scanLeft
  scanRight
  currentMode
*/
