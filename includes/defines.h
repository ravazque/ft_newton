#ifndef DEFINES_H
# define DEFINES_H

// Every tuning value of the engine, grouped by subsystem, in SI units (m, kg, s, rad) unless a comment says otherwise.

# define GLFW_INCLUDE_NONE                           // GLFW without its own OpenGL header: GLAD provides the functions

// Window: default size and the bounds accepted for the [width height] arguments, in pixels.
# define WIN_WIDTH       1280
# define WIN_HEIGHT      720
# define WIN_TITLE       "ft_newton"
# define WIN_WIDTH_MIN   720
# define WIN_HEIGHT_MIN  480
# define WIN_WIDTH_MAX   2560
# define WIN_HEIGHT_MAX  1440

// Shader sources, relative to the directory the program runs from.
# define SHADER_VERT  "shaders/basic.vert"
# define SHADER_FRAG  "shaders/basic.frag"

// Object files: one CSV per kind of object, and every MODEL_EXTENSION file of MODEL_DIR loaded as a model.
# define ASSET_DIR        "assets/"
# define ASSET_APPLE      ASSET_DIR "apples.csv"
# define ASSET_BLOCK      ASSET_DIR "block.csv"
# define ASSET_GROUND     ASSET_DIR "ground.csv"
# define ASSET_TREBUCHET  ASSET_DIR "trebuchet.csv"
# define ASSET_RAGDOLL    ASSET_DIR "ragdoll.csv"
# define ASSET_MODELS     ASSET_DIR "models.csv"      // size, density and material shared by every model
# define MODEL_DIR        ASSET_DIR "models/"
# define MODEL_EXTENSION  ".obj"
# define MODEL_MAX        16                          // models loaded at most, the first ones in name order
# define MODEL_NAME_LEN   32                          // longest model name (file name without extension) + 1

// CSV reader: the required first line and the size limits of a file.
# define CSV_HEADER       "# property,value1,value2,value3"
# define CSV_MAX_ENTRIES  32                          // property rows per file
# define CSV_KEY_LEN      32                          // longest property name or word value + 1
# define CSV_LINE_LEN     256
# define CSV_PATH_LEN     128

// Bitmap font: 8x8 glyphs for ASCII 32 to 126.
# define FONT_FIRST_CHAR   32
# define FONT_GLYPH_COUNT  95
# define FONT_GLYPH_SIZE   8
# define FONT_ADVANCE      9.0f                       // font pixels from one character to the next

// 2D overlay buffer.
# define UI_VERTEX_FLOATS    6                        // position + colour
# define UI_MAX_VERTICES     196608                   // vertex budget of one frame
# define UI_FLAT_COLOR_MODE  2                        // shader mode that takes the colour from each vertex

// Menu texts and limits.
# define MENU_TITLE          "ft_newton"
# define MENU_MODELS_TITLE   "ft_newton  models"
# define MENU_NOTE_BACK      "ESC or RESUME to go back"
# define MENU_NOTE_DIRTY     "APPLY to use the edits"
# define MENU_NOTE_MODELS    "pick a model to place it"
# define MENU_MAX_ROWS       80
# define MENU_COLUMNS        3
# define MENU_LABEL_LEN      48                       // longest generated label (model name and triangle count) + 1

// Menu behaviour.
# define MENU_REPEAT_DELAY   0.35f                    // s a [-] or [+] is held before it repeats
# define MENU_REPEAT_PERIOD  0.06f                    // s between two repeats
# define MENU_PANEL_RATIO    0.75f                    // share of the window the panel covers
# define MENU_MIN_SCALE      1.0f                     // smallest drawing scale of the font
# define MENU_MAX_STRETCH    1.5f                     // how much taller than a text line a row may grow

// Menu geometry, in font pixels (the layout scales them to the window).
# define MENU_LINE_PIXELS    (FONT_GLYPH_SIZE + 3)
# define MENU_PADDING        6.0f
# define MENU_BUTTON_CHARS   3.5f                     // width of a [-] or [+] button, in characters
# define MENU_BUTTON_INSET   1.0f
# define MENU_BUTTON_GAP     3.0f
# define MENU_VALUE_CHARS    7.0f                     // room for a number, in characters

# define HUD_REFRESH_PERIOD  0.2f                     // s between two rewrites of the window title readout

// Background colour (RGB, 0 to 1).
# define CLEAR_R  0.10f
# define CLEAR_G  0.11f
# define CLEAR_B  0.13f

// Meshes and the debug display (F1).
# define SPHERE_SEGMENTS      32                      // rings and slices of the sphere mesh
# define DEBUG_WIRE_INFLATE   1.01f                   // collider wireframes drawn 1 % larger, so they show over the bodies
# define DEBUG_PLANE_LIFT     0.01f                   // m the plane wireframe is raised
# define DEBUG_CONTACT_SIZE   0.05f                   // m, side of a contact marker
# define DEBUG_NORMAL_LENGTH  0.25f                   // m, length of a contact normal

// Maths helpers.
# define FTN_PI                    3.14159265358979323846f
# define DEG2RAD(degrees)          ((degrees) * (FTN_PI / 180.0f))
# define RAD2DEG(radians)          ((radians) * (180.0f / FTN_PI))
# define MAT3_AT(matrix, row, col) ((matrix).cells[(col) * 3 + (row)])   // column-major 3x3 read by row and column

// Frame and step.
# define FPS_CAP              30                      // frames per second of the main loop
# define GRAVITY_Y            (-9.81f)                // m/s^2
# define FIXED_DT             (1.0f / 120.0f)         // s, the physics step
# define MAX_STEPS_PER_FRAME  32                      // a heavier frame drops time instead of piling it up
# define SOLVER_ITERATIONS    16                      // velocity passes per step
# define ANGULAR_DAMPING      0.98f                   // share of the angular velocity kept per second

// Contact solver.
# define ELASTICITY_THRESHOLD  1.0f                   // m/s: slower impacts do not bounce
# define PENETRATION_SLOP      0.005f                 // m of overlap left alone
# define PENETRATION_PERCENT   0.2f                   // share of the remaining overlap removed per positional pass
# define POSITION_ITERATIONS   3                      // positional passes per step
# define MAX_CORRECTION        0.2f                   // m, most one contact is pushed per pass
# define WARM_START_RADIUS     0.05f                  // m: a contact this close to last step's reuses its impulse
# define MASS_RATIO_LIMIT      10.0f                  // a lighter body holding up a heavier one counts as at most this many times lighter
# define MASS_SCALE_FLOOR      0.1f                   // and at most this many times heavier than it is (1 / 0.1 = 10)
# define SUPPORT_COSINE        0.1f                   // a normal this far from the horizontal (cosine to the vertical) is one body resting on the other

// Sleeping and world bounds.
# define SLEEP_LINEAR_EPS     0.05f                   // m/s
# define SLEEP_ANGULAR_EPS    0.05f                   // rad/s
# define SLEEP_TIME           0.5f                    // s an island stays under both speeds before it sleeps
# define SLEEP_JITTER_RATIO   3.0f                    // a path this many times longer than the net move is shaking in place, not moving
# define SLEEP_CREEP_TIME     3.0f                    // s an island may creep before it sleeps anyway
# define SLEEP_CREEP_FACTOR   4.0f                    // creeping: moving on average no faster than this many times the sleep speeds
# define WORLD_CULL_DISTANCE  200.0f                  // m from the origin where dynamic bodies are removed
# define FALL_CULL_DEPTH      20.0f                   // m under a plane where dynamic bodies are removed

// Collision detection.
# define MAX_CONTACTS_PER_PAIR  4
# define MANIFOLD_DEPTH_TOLERANCE  0.001f            // m: depths this close count as equal, so the kept points do not swap every step
# define PLANE_THICKNESS        2.0f                  // m of solid under a plane
# define SAT_EDGE_BIAS_REL      0.95f                 // a box edge axis beats a face only when better by this ratio
# define SAT_EDGE_BIAS_ABS      0.005f                // and by this distance (m)

// Mesh colliders.
# define OBJ_DEGENERATE_EPSILON  1e-10f               // smallest doubled face area, share of the squared model diagonal
# define MESH_WELD_EPSILON       1e-6f                // vertices closer than this share of the model size are one
# define MESH_MIN_VOLUME         1e-6f                // smallest volume, share of the cubed model size
# define BVH_LEAF_TRIANGLES      4                    // triangles per tree leaf
# define BVH_STACK               64                   // depth of the tree traversal stack
# define MESH_CANDIDATES         64                   // contact candidates kept per mesh pair before the reduction to 4
# define MESH_MAX_DEPTH          0.25f                // m, deepest vertex looked for inside another mesh
# define MESH_NORMAL_MERGE       0.98f                // two sphere contact normals this alike (cosine) count as one
# define MESH_EDGE_CROSSINGS     16                   // surface crossings kept per edge of a mesh tested against another

// Joints.
# define JOINT_ROWS            3                      // angular rows per joint (locks and limits)
# define JOINT_PERCENT         0.5f                   // share of the drift closed per positional pass
# define JOINT_SLOP            0.001f                 // m or rad of drift tolerated
# define JOINT_MAX_CORRECTION  0.2f                   // m or rad, most one joint is corrected per pass
# define JOINT_SPIN_DAMPING    0.01f                  // share of the relative spin removed per step (joint friction)
# define JOINT_LIMIT_MARGIN    0.05f                  // rad before a limit where it starts acting
# define JOINT_PROJECTION_DISTANCE  0.01f             // m an anchor may open after the solver before its body is moved back
# define JOINT_PROJECTION_ANGLE     DEG2RAD(5.0f)     // rad a limit may be passed after the solver before its body is turned back
# define JOINT_PROJECTION_SWEEPS    2                 // passes over the joints, so a parent moved late carries its children

// Ragdoll: parts, joints, the height its layout table is written for, and the joint limits in degrees.
# define RAGDOLL_PARTS           11
# define RAGDOLL_JOINTS          10
# define RAGDOLL_LAYOUT_HEIGHT   1.8f                 // m
# define RAGDOLL_NECK_SWING      35.0f                // half-angle of the cone
# define RAGDOLL_NECK_TWIST      50.0f                // either way
# define RAGDOLL_WAIST_SWING     35.0f
# define RAGDOLL_WAIST_TWIST     30.0f
# define RAGDOLL_WAIST_TILT      15.0f                // how far the cone leans forward from the rest pose
# define RAGDOLL_SHOULDER_SWING  80.0f
# define RAGDOLL_SHOULDER_TWIST  60.0f
# define RAGDOLL_SHOULDER_TILT   35.0f                // outward
# define RAGDOLL_HIP_SWING       65.0f
# define RAGDOLL_HIP_TWIST       30.0f
# define RAGDOLL_HIP_TILT        30.0f                // forward
# define RAGDOLL_ELBOW_MAX       145.0f               // fold of the hinge
# define RAGDOLL_KNEE_MAX        140.0f
# define RAGDOLL_HEAD_ROLLING    0.3f                 // rolling resistance of the head, its only sphere

// Free camera.
# define CAM_LOOK_SPEED        DEG2RAD(70.0f)         // rad/s
# define CAM_MOVE_SPEED        14.0f                  // m/s
# define CAM_SPEED_MIN         2.0f
# define CAM_SPEED_MAX         60.0f
# define CAM_SPEED_RATE        12.0f                  // fly speed change per second
# define CAM_MIN_PITCH         DEG2RAD(-89.0f)
# define CAM_MAX_PITCH         DEG2RAD(89.0f)
# define CAM_RADIUS            0.3f                   // m: the camera is a sphere that bodies push out and cannot be flown through
# define CAM_PUSH_PASSES       3                      // passes over the bodies to push the camera out of all of them
# define CAM_STEP              0.1f                   // m: longest single move, a third of the radius, so the centre never crosses a thin body
# define CAM_ESCAPE_MAX        300                    // CAM_STEP distances tried at most to free a pinned camera
# define CAM_ESCAPE_DIRECTIONS 5                      // up, toward +z, toward -z, toward -x, toward +x (never down)

// Held keys: change per second of each live value.
# define CTRL_SPEED_RATE    10.0f                     // launch speed, m/s
# define CTRL_ANGLE_RATE    60.0f                     // launch angle, degrees
# define CTRL_MASS_RATE     1.0f                      // apple mass, kg
# define CTRL_GRAVITY_RATE  5.0f                      // gravity, m/s^2
# define CTRL_TIME_RATE     1.0f                      // time scale

// Ranges of the live values and of the object files (m/s, degrees, kg, m/s^2, m, kg/m^3).
# define TIME_SCALE_MAX          4.0f
# define LAUNCH_SPEED_MIN        1.0f
# define LAUNCH_SPEED_MAX        60.0f
# define LAUNCH_ANGLE_MIN        (-70.0f)
# define LAUNCH_ANGLE_MAX        90.0f
# define ARM_ANGLE_MAX           90.0f
# define PROJECTILE_MASS_MIN     0.1f
# define PROJECTILE_MASS_MAX     50.0f
# define PROJECTILE_RADIUS_MIN   (LAUNCH_SPEED_MAX * FIXED_DT * 0.5f)   // 0.25 m: the fastest apple moves at most its diameter per step
# define GRAVITY_MIN             (-40.0f)
# define GRAVITY_MAX             20.0f
# define FRICTION_MAX            20.0f
# define BODY_SIZE_MIN           0.1f                                 // m, a block side
# define BODY_SIZE_MAX           6.0f                                 // m, a block side or an apple diameter
# define ROLLING_RESISTANCE_MAX  1.0f
# define RAGDOLL_MASS_MIN        1.0f
# define RAGDOLL_MASS_MAX        200.0f
# define RAGDOLL_HEIGHT_MIN      0.5f
# define RAGDOLL_HEIGHT_MAX      3.0f
# define MODEL_SIZE_MIN          0.2f
# define MODEL_SIZE_MAX          6.0f
# define MODEL_DENSITY_MIN       10.0f
# define MODEL_DENSITY_MAX       1000.0f

// Trebuchet proportions the file does not give.
# define TREB_PARTS             5                     // sill, two legs, arm, counterweight
# define TREB_PIVOT_RATIO       0.72f                 // share of the arm on the throwing side
# define TREB_LEG_SPREAD_RATIO  0.45f                 // share of the sill between the feet of the legs
# define TREB_LEG_THICK_RATIO   0.07f                 // share of the frame height
# define TREB_ANGLE_STEP        1.0f                  // degrees between two angles of the spawn clearance search
# define LAUNCH_CLEARANCE       0.1f                  // m between the machine and a new apple

// Scene layout (m) and the aim bar.
# define STRUCTURE_X         8.0f                     // where keys 1 2 3 build, and the starting pyramid
# define STRESS_X            20.0f                    // where key 4 builds the large wall
# define TARGET_X            2.0f                     // where keys 5 to 9 place the ragdoll, the models and the rain
# define START_PYRAMID_ROWS  5
# define BLOCK_GAP           1.002f                   // spacing of stacked blocks, share of their size
# define SPAWN_TOLERANCE     0.001f                   // m of overlap still read as touching
# define SPAWN_CLEARANCE     0.01f                    // m left under a body lifted onto another
# define AIM_LENGTH_PER_MPS  0.12f                    // m of aim tail per m/s of launch speed
# define AIM_BAR_THICKNESS   0.10f
# define AIM_TAIL_THICKNESS  0.04f

// Test rain (key 9): a RAIN_GRID x RAIN_GRID square of small balls.
# define RAIN_GRID     14
# define RAIN_SPACING  0.21f                          // m between two balls
# define RAIN_RADIUS   0.07f
# define RAIN_MASS     0.05f
# define RAIN_DROP     1.0f                           // m above the highest body under the square
# define RAIN_R        0.95f
# define RAIN_G        0.95f
# define RAIN_B        0.95f

#endif
