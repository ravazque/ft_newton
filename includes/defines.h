
#ifndef DEFINES_H
# define DEFINES_H

/* ---- Glad / GLFW ---- */
# define GLFW_INCLUDE_NONE

/* ---- Window ---- */
# define WIN_WIDTH   1280
# define WIN_HEIGHT  720
# define WIN_TITLE   "ft_newton"

/* ---- Resolution bounds for the [width height] args (clamped to this range) - */
# define WIN_WIDTH_MIN   720
# define WIN_HEIGHT_MIN  480
# define WIN_WIDTH_MAX   2560
# define WIN_HEIGHT_MAX  1440

/* ---- Shader source paths (relative to the run directory) ---- */
# define SHADER_VERT "shaders/basic.vert"
# define SHADER_FRAG "shaders/basic.frag"

/* ---- Object definition files (relative to the run directory) ---- */
# define ASSET_DIR        "assets/"
# define ASSET_APPLE      ASSET_DIR "apples.csv"
# define ASSET_BLOCK      ASSET_DIR "block.csv"
# define ASSET_GROUND     ASSET_DIR "ground.csv"
# define ASSET_TREBUCHET  ASSET_DIR "trebuchet.csv"

/* ---- CSV object file reader ---- */
# define CSV_HEADER       "# property,value1,value2,value3"
# define CSV_MAX_ENTRIES  32
# define CSV_KEY_LEN      32
# define CSV_LINE_LEN     256
# define CSV_PATH_LEN     128

/* ---- Bitmap font (srcs/render/font.c): 8x8 pixels, ASCII 32..126 ---- */
# define FONT_FIRST_CHAR   32
# define FONT_GLYPH_COUNT  95
# define FONT_GLYPH_SIZE   8    /* rows and columns of one glyph */
# define FONT_ADVANCE      9.0f /* font pixels from one character to the next */

/* ---- 2D overlay (srcs/render/ui.c) ---- */
# define UI_VERTEX_FLOATS     6      /* x y z + r g b (the color rides in the normal slot) */
# define UI_MAX_VERTICES      196608 /* 32768 quads: the whole menu fits in one draw call */
# define UI_FLAT_COLOR_MODE   2      /* uLit value that makes the shader use the vertex color */

/* ---- Menu ---- */
# define MENU_MAX_ROWS      64
# define MENU_REPEAT_DELAY  0.35f  /* s holding a +/- button before it repeats */
# define MENU_REPEAT_PERIOD 0.06f  /* s between repeats while held */
# define MENU_WIDTH_RATIO   0.80f  /* panel never covers more than this much of the window */
# define MENU_HEIGHT_RATIO  0.85f
# define MENU_LINE_PIXELS   (FONT_GLYPH_SIZE + 3)  /* font pixels of one menu row */
# define MENU_BUTTON_CHARS  3.5f   /* width of a [-] / [+] button, in font advances */
# define MENU_BUTTON_INSET  1.0f   /* font pixels trimmed off a button's top and bottom */
# define MENU_BUTTON_GAP    3.0f   /* font pixels between the [-] and the [+] */
# define MENU_VALUE_CHARS   7.0f   /* room a value row keeps for its number */

/* ---- Background clear color (RGB, 0..1) ---- */
# define CLEAR_R 0.10f
# define CLEAR_G 0.11f
# define CLEAR_B 0.13f

/* ---- Math ---- */
# define FTN_PI     3.14159265358979323846f
# define DEG2RAD(d) ((d) * (FTN_PI / 180.0f))
# define RAD2DEG(r) ((r) * (180.0f / FTN_PI))

/* ---- Frame rate ---- */
# define FPS_CAP  30   /* frames per second the main loop is limited to (the physics step is independent) */

/* ---- Physics defaults ---- */
# define GRAVITY_Y          (-9.81f)        /* m/s^2, tweakable at runtime  */
# define FIXED_DT           (1.0f / 120.0f) /* fixed physics step: 120 Hz */
# define MAX_STEPS_PER_FRAME 32             /* fixed steps one frame may run; extra simulated time is dropped */
# define SOLVER_ITERATIONS  16              /* contact solver passes / step */
# define ANGULAR_DAMPING    0.98f           /* angular velocity kept per second */

/* ---- Contact solver ---- */
# define ELASTICITY_THRESHOLD   1.0f   /* m/s: slower impacts do not bounce    */
# define PENETRATION_SLOP       0.005f /* m: overlap tolerated before pushing  */
# define PENETRATION_PERCENT    0.2f   /* fraction of the overlap fixed per pass */
# define POSITION_ITERATIONS    3      /* positional correction passes per step */
# define MAX_CORRECTION         0.2f   /* m: cap on one contact's push per pass  */
# define WARM_START_RADIUS      0.05f  /* m: a contact this close to last step's reuses its impulse */

/* ---- Sleeping and world bounds ---- */
# define SLEEP_LINEAR_EPS    0.05f   /* m/s                                      */
# define SLEEP_ANGULAR_EPS   0.05f   /* rad/s                                    */
# define SLEEP_TIME          0.5f    /* s below both thresholds before sleeping  */
# define WORLD_CULL_DISTANCE 200.0f  /* m: dynamic bodies beyond this are freed  */

/* ---- Collision detection ---- */
# define MAX_CONTACTS_PER_PAIR  4           /* manifold cap per body pair   */

/* ---- Free-flying camera ---- */
# define CAM_LOOK_SPEED    DEG2RAD(70.0f)  /* rad/s of yaw or pitch while an arrow is held */
# define CAM_MOVE_SPEED    14.0f           /* m/s the camera flies at, by default         */
# define CAM_SPEED_MIN     2.0f            /* m/s                                         */
# define CAM_SPEED_MAX     60.0f           /* m/s                                         */
# define CAM_SPEED_RATE    12.0f           /* m/s of fly speed gained per second on + / - */
# define CAM_MIN_PITCH     DEG2RAD(-89.0f) /* straight down would make the view degenerate */
# define CAM_MAX_PITCH     DEG2RAD(89.0f)

/* ---- Live controls (change per second while a key is held) ---- */
# define CTRL_SPEED_RATE     10.0f  /* launch speed, m/s per s */
# define CTRL_ANGLE_RATE     60.0f  /* launch angle, deg per s */
# define CTRL_MASS_RATE      1.0f   /* projectile mass, kg per s */
# define CTRL_GRAVITY_RATE   5.0f   /* m/s^2 per s */
# define CTRL_TIME_RATE      1.0f   /* time scale per s */
# define TIME_SCALE_MAX      4.0f
# define LAUNCH_SPEED_MIN    1.0f   /* m/s */
# define LAUNCH_SPEED_MAX    60.0f  /* m/s: 0.5 m per fixed step, well inside the 2 m contact window of a 1 m block */
# define LAUNCH_ANGLE_MAX    90.0f  /* deg above +X: straight up */
# define LAUNCH_ANGLE_MIN    (-70.0f) /* deg: aiming lower would fire through the trebuchet's own base */
# define ARM_ANGLE_MAX       90.0f  /* deg: the trebuchet arm stays above its base */
# define PROJECTILE_MASS_MIN 0.1f   /* kg */
# define PROJECTILE_MASS_MAX 50.0f  /* kg, the menu's upper bound */
# define GRAVITY_MIN         (-40.0f) /* m/s^2, the menu's bounds for gravity */
# define GRAVITY_MAX         20.0f
# define FRICTION_MAX        20.0f  /* the menu's upper bound (>1 is legal, just very grippy) */
# define LAUNCH_CLEARANCE    0.1f   /* m between the arm tip and a new apple, along the launch direction */

/* ---- Trebuchet frame: what the file does not spell out, as proportions ---- */
# define TREB_PARTS            5      /* sill, two A-frame legs, arm, counterweight */
# define TREB_PIVOT_RATIO      0.72f  /* share of the arm on the throwing side of the pivot */
# define TREB_LEG_SPREAD_RATIO 0.45f  /* foot of each A-frame leg, as a share of base_size.x */
# define TREB_LEG_THICK_RATIO  0.07f  /* leg thickness, as a share of frame_height */
# define TREB_ANGLE_STEP       1.0f   /* deg between the launch angles sampled for the spawn distance */

/* ---- Scene layout ---- */
# define STRUCTURE_X         8.0f   /* where 1/2/3 spawn structures  */
# define STRESS_X            20.0f  /* where 4 spawns the large wall */
# define AIM_LENGTH_PER_MPS  0.12f  /* m of the thin aim tail per m/s of launch speed */
# define AIM_BAR_THICKNESS   0.10f  /* m: the barrel, from the arm tip to the spawn point */
# define AIM_TAIL_THICKNESS  0.04f  /* m: the thin line that grows with the launch speed */

#endif
