
#include "newton.h"

/*
 * Scene, input and main loop. Object properties come from the assets/objects
 * CSV files (loaded at start and again on R), the physics runs at a fixed step
 * and the render at whatever rate the display allows.
*/

/* Loads the four CSV files into temporaries so a broken file leaves the
 * running definitions untouched (and the reason on stderr). */
static int	load_assets(Game *g)
{
	ObjectDef	bird;
	ObjectDef	block;
	ObjectDef	ground;
	Catapult	catapult;

	if (!objectdef_load(&bird, ASSET_BIRD, KIND_BIRD) || !objectdef_load(&block, ASSET_BLOCK, KIND_BLOCK) || !objectdef_load(&ground, ASSET_GROUND, KIND_GROUND))
		return (0);
	if (!catapult_load(&catapult, ASSET_CATAPULT, &bird))
		return (0);
	if (ground.shape != SHAPE_PLANE || bird.shape != SHAPE_SPHERE || block.shape != SHAPE_BOX)
		return (fprintf(stderr, "%s must be a plane, %s a sphere and %s a box\n", ASSET_GROUND, ASSET_BIRD, ASSET_BLOCK), 0);
	g->birdDef = bird;
	g->blockDef = block;
	g->groundDef = ground;
	g->catapultDef = catapult;
	return (1);
}

/* The starting scene: ground, catapult with its file launch settings, and
 * one pyramid. Everything spawned or fired before is gone. */
static void	build_scene(Game *g)
{
	world_clear(&g->world);
	g->catapult = g->catapultDef;
	world_add_body(&g->world, objectdef_make_body(&g->groundDef, vec3(0.0f, 0.0f, 0.0f)));
	catapult_build(&g->catapult, &g->world);
	structure_spawn_pyramid(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 5);
	g->paused = 0;
	g->hud.refresh = 1;
}

/* Stamps a definition's material onto every existing body of its kind. The
 * mass goes through rb_set_mass so the inertia follows the current collider. */
static void	restamp_bodies(World *w, ObjectKind kind, float mass, float friction, float restitution, Vec3 color)
{
	RigidBody	*b;
	int			i;

	i = 0;
	while (i < w->bodyCount)
	{
		b = &w->bodies[i];
		if (b->kind == kind)
		{
			b->friction = friction;
			b->restitution = restitution;
			b->color = color;
			if (b->invMass > 0.0f)
				rb_set_mass(b, mass);
		}
		i++;
	}
}

/* R: re-read the CSV files and apply what can change on the fly. Existing
 * bodies take the new mass, friction, restitution and color, the launch
 * settings return to the file values, and everything is woken so a lighter
 * or slipperier pile reacts at once. Geometry (shape, size, radius, plane,
 * catapult layout) only reaches bodies spawned from now on, or the whole
 * scene after a reset (BACKSPACE). */
static void	reload_definitions(Game *g)
{
	const ObjectDef	*bird = &g->birdDef;
	const ObjectDef	*block = &g->blockDef;
	const ObjectDef	*ground = &g->groundDef;
	const Catapult	*cat = &g->catapultDef;

	if (!load_assets(g))
	{
		fprintf(stderr, "Reload aborted: fix the object files and press R again\n");
		return ;
	}
	restamp_bodies(&g->world, KIND_BIRD, bird->mass, bird->friction, bird->restitution, bird->color);
	restamp_bodies(&g->world, KIND_BLOCK, block->mass, block->friction, block->restitution, block->color);
	restamp_bodies(&g->world, KIND_GROUND, 0.0f, ground->friction, ground->restitution, ground->color);
	restamp_bodies(&g->world, KIND_CATAPULT, 0.0f, cat->friction, cat->restitution, cat->color);
	g->catapult.launchSpeed = cat->launchSpeed;
	g->catapult.launchAngle = cat->launchAngle;
	g->catapult.projectileMass = cat->projectileMass;
	g->catapult.friction = cat->friction;
	g->catapult.restitution = cat->restitution;
	g->catapult.color = cat->color;
	mesh_release(&g->planeMesh);
	g->planeMesh = mesh_plane(ground->extent);
	world_wake_all(&g->world);
	g->hud.refresh = 1;
	printf("Object files reloaded: existing objects updated\n");
}

/* Every simulation parameter back to its starting value (gravity, time
 * scale, launch settings) and the scene back to its starting state. */
static void	reset_simulation(Game *g)
{
	g->world.gravity = vec3(0.0f, GRAVITY_Y, 0.0f);
	g->timeScale = 1.0f;
	build_scene(g);
	printf("Simulation reset\n");
}

/* Writes the four definitions back to their files, so values tuned in the
 * menu survive a restart. Overwrites the files in assets/. */
static void	save_definitions(Game *g)
{
	int	ok;

	ok = objectdef_save(&g->birdDef, ASSET_BIRD);
	ok = objectdef_save(&g->blockDef, ASSET_BLOCK) && ok;
	ok = objectdef_save(&g->groundDef, ASSET_GROUND) && ok;
	ok = catapult_save(&g->catapult, ASSET_CATAPULT) && ok;
	if (ok)
	{
		g->catapultDef = g->catapult;
		printf("Object files written\n");
	}
}

/* Applies the definitions the menu has just edited to the bodies already in
 * the world, exactly like a reload does. */
static void	apply_definitions(Game *g)
{
	restamp_bodies(&g->world, KIND_BIRD, g->birdDef.mass, g->birdDef.friction, g->birdDef.restitution, g->birdDef.color);
	restamp_bodies(&g->world, KIND_BLOCK, g->blockDef.mass, g->blockDef.friction, g->blockDef.restitution, g->blockDef.color);
	restamp_bodies(&g->world, KIND_GROUND, 0.0f, g->groundDef.friction, g->groundDef.restitution, g->groundDef.color);
	restamp_bodies(&g->world, KIND_CATAPULT, 0.0f, g->catapult.friction, g->catapult.restitution, g->catapult.color);
	g->catapult.projectileMass = g->birdDef.mass;
	world_wake_all(&g->world);
	g->hud.refresh = 1;
}

/* The menu's rows: the left column edits values straight inside the game's
 * own definitions (the pointers stay valid, a reload only rewrites what they
 * point at), the right column lists the controls and the actions. */
static void	build_menu(Game *g)
{
	Menu	*m = &g->menu;

	m->rowCount = 0;
	m->selected = 1;
	m->hoverRow = -1;
	m->heldRow = -1;
	menu_add_section(m, "SIMULATION", 0);
	menu_add_value(m, "Gravity", &g->world.gravity.y, GRAVITY_MIN, GRAVITY_MAX, 0.5f, 2);
	menu_add_value(m, "Time scale", &g->timeScale, 0.0f, TIME_SCALE_MAX, 0.1f, 2);
	menu_add_section(m, "LAUNCH", 0);
	menu_add_value(m, "Speed", &g->catapult.launchSpeed, LAUNCH_SPEED_MIN, LAUNCH_SPEED_MAX, 1.0f, 1);
	menu_add_value(m, "Angle", &g->catapult.launchAngle, LAUNCH_ANGLE_MIN, LAUNCH_ANGLE_MAX, 5.0f, 0);
	menu_add_section(m, "BIRD", 0);
	menu_add_value(m, "Mass", &g->birdDef.mass, PROJECTILE_MASS_MIN, PROJECTILE_MASS_MAX, 0.25f, 2);
	menu_add_value(m, "Friction", &g->birdDef.friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(m, "Restitution", &g->birdDef.restitution, 0.0f, 1.0f, 0.05f, 2);
	menu_add_section(m, "BLOCK", 0);
	menu_add_value(m, "Mass", &g->blockDef.mass, PROJECTILE_MASS_MIN, PROJECTILE_MASS_MAX, 0.25f, 2);
	menu_add_value(m, "Friction", &g->blockDef.friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(m, "Restitution", &g->blockDef.restitution, 0.0f, 1.0f, 0.05f, 2);
	menu_add_section(m, "GROUND", 0);
	menu_add_value(m, "Friction", &g->groundDef.friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(m, "Restitution", &g->groundDef.restitution, 0.0f, 1.0f, 0.05f, 2);
	menu_add_section(m, "CATAPULT", 0);
	menu_add_value(m, "Friction", &g->catapult.friction, 0.0f, FRICTION_MAX, 0.05f, 2);
	menu_add_value(m, "Restitution", &g->catapult.restitution, 0.0f, 1.0f, 0.05f, 2);
	menu_add_section(m, "CONTROLS", 1);
	menu_add_hint(m, "ESC     open / close this menu");
	menu_add_hint(m, "SPACE   fire a bird");
	menu_add_hint(m, "1 2 3   wall / pyramid / tower");
	menu_add_hint(m, "4       big wall (150 blocks)");
	menu_add_hint(m, "P       pause / resume");
	menu_add_hint(m, "F1      collider wireframe");
	menu_add_hint(m, "H       title readout on / off");
	menu_add_hint(m, "G B     gravity + / - (live)");
	menu_add_hint(m, "W S     launch speed + / -");
	menu_add_hint(m, "A D     launch angle + / -");
	menu_add_hint(m, "Q E     bird mass - / +");
	menu_add_hint(m, "T Y     time scale - / +");
	menu_add_hint(m, "arrows  orbit camera");
	menu_add_hint(m, "+ -     zoom camera");
	menu_add_section(m, "ACTIONS", 1);
	menu_add_action(m, "Reload from files", ACTION_RELOAD, 1);
	menu_add_action(m, "Save to files", ACTION_SAVE, 1);
	menu_add_action(m, "Reset simulation", ACTION_RESET, 1);
	menu_add_action(m, "Spawn wall", ACTION_SPAWN_WALL, 1);
	menu_add_action(m, "Spawn pyramid", ACTION_SPAWN_PYRAMID, 1);
	menu_add_action(m, "Spawn tower", ACTION_SPAWN_TOWER, 1);
	menu_add_action(m, "Spawn big wall", ACTION_SPAWN_BIG_WALL, 1);
	menu_add_action(m, "Resume", ACTION_RESUME, 1);
	menu_add_action(m, "Quit", ACTION_QUIT, 1);
}

static void	close_menu(Game *g)
{
	g->menu.open = 0;
	g->paused = 0;
	g->hud.refresh = 1;
}

/* Runs what the menu asked for. Spawning and resetting while the menu is
 * open is on purpose: it is where a scene gets set up before resuming. */
static void	run_menu_action(Game *g, MenuAction action)
{
	if (action == ACTION_RELOAD)
		reload_definitions(g);
	else if (action == ACTION_SAVE)
		save_definitions(g);
	else if (action == ACTION_RESET)
		reset_simulation(g);
	else if (action == ACTION_SPAWN_WALL)
		structure_spawn_wall(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 6, 6);
	else if (action == ACTION_SPAWN_PYRAMID)
		structure_spawn_pyramid(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 7);
	else if (action == ACTION_SPAWN_TOWER)
		structure_spawn_tower(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 8);
	else if (action == ACTION_SPAWN_BIG_WALL)
		structure_spawn_wall(&g->world, &g->blockDef, vec3(STRESS_X, 0.0f, 0.0f), 15, 10);
	else if (action == ACTION_RESUME)
		close_menu(g);
	else if (action == ACTION_QUIT)
		g->quit = 1;
}

int	game_init(Game *g, int argc, char **argv)
{
	memset(g, 0, sizeof(*g));
	g->camera = camera_default();
	check_input(argc, argv, &g->camera);
	if (!load_assets(g))
		return (0);
	if (!window_init(&g->window, g->camera.win_width, g->camera.win_height, WIN_TITLE))
		return (0);
	if (!renderer_init(&g->renderer))
		return (window_destroy(&g->window), 0);
	if (!ui_init(&g->ui))
		return (window_destroy(&g->window), 0);
	world_init(&g->world);
	build_menu(g);
	g->hud = hud_default();
	g->debug.enabled = 0;
	g->fixedDt = FIXED_DT;
	g->timeScale = 1.0f;
	g->planeMesh = mesh_plane(g->groundDef.extent);
	g->sphereMesh = mesh_sphere(32);
	g->cubeMesh = mesh_cube();
	build_scene(g);
	return (1);
}

void	game_shutdown(Game *g)
{
	ui_destroy(&g->ui);
	mesh_release(&g->planeMesh);
	mesh_release(&g->sphereMesh);
	mesh_release(&g->cubeMesh);
	world_destroy(&g->world);
	window_destroy(&g->window);
}

/* One-shot key detection (edge, not hold). */
static int	key_pressed(Game *g, int key)
{
	static char	prev[GLFW_KEY_LAST + 1];
	int			down;

	down = (glfwGetKey(window_handle(&g->window), key) == GLFW_PRESS);
	if (down && !prev[key])
	{
		prev[key] = 1;
		return (1);
	}
	prev[key] = (char)down;
	return (0);
}

static int	key_down(Game *g, int key)
{
	return (glfwGetKey(window_handle(&g->window), key) == GLFW_PRESS);
}

static void	handle_actions(Game *g)
{
	if (key_pressed(g, GLFW_KEY_SPACE))
		catapult_fire(&g->catapult, &g->birdDef, &g->world);
	if (key_pressed(g, GLFW_KEY_1))
		structure_spawn_wall(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 6, 6);
	if (key_pressed(g, GLFW_KEY_2))
		structure_spawn_pyramid(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 7);
	if (key_pressed(g, GLFW_KEY_3))
		structure_spawn_tower(&g->world, &g->blockDef, vec3(STRUCTURE_X, 0.0f, 0.0f), 8);
	if (key_pressed(g, GLFW_KEY_4))
		structure_spawn_wall(&g->world, &g->blockDef, vec3(STRESS_X, 0.0f, 0.0f), 15, 10);
	if (key_pressed(g, GLFW_KEY_P))
	{
		g->paused = !g->paused;
		g->hud.refresh = 1;
	}
	if (key_pressed(g, GLFW_KEY_F1))
		g->debug.enabled = !g->debug.enabled;
	if (key_pressed(g, GLFW_KEY_H))
	{
		g->hud.visible = !g->hud.visible;
		g->hud.refresh = 1;
	}
}

/* Held keys change a value continuously, scaled by the frame time so the
 * rate is the same at any FPS. */
static void	handle_tuning(Game *g, float frame_time)
{
	if (key_down(g, GLFW_KEY_W))
		g->catapult.launchSpeed = fminf(LAUNCH_SPEED_MAX, g->catapult.launchSpeed + CTRL_SPEED_RATE * frame_time);
	if (key_down(g, GLFW_KEY_S))
		g->catapult.launchSpeed = fmaxf(LAUNCH_SPEED_MIN, g->catapult.launchSpeed - CTRL_SPEED_RATE * frame_time);
	if (key_down(g, GLFW_KEY_A))
		g->catapult.launchAngle = fminf(LAUNCH_ANGLE_MAX, g->catapult.launchAngle + CTRL_ANGLE_RATE * frame_time);
	if (key_down(g, GLFW_KEY_D))
		g->catapult.launchAngle = fmaxf(LAUNCH_ANGLE_MIN, g->catapult.launchAngle - CTRL_ANGLE_RATE * frame_time);
	if (key_down(g, GLFW_KEY_E))
		g->birdDef.mass = fminf(PROJECTILE_MASS_MAX, g->birdDef.mass + CTRL_MASS_RATE * frame_time);
	if (key_down(g, GLFW_KEY_Q))
		g->birdDef.mass = fmaxf(PROJECTILE_MASS_MIN, g->birdDef.mass - CTRL_MASS_RATE * frame_time);
	g->catapult.projectileMass = g->birdDef.mass;
	if (key_down(g, GLFW_KEY_G))
	{
		g->world.gravity.y = fminf(GRAVITY_MAX, g->world.gravity.y + CTRL_GRAVITY_RATE * frame_time);
		world_wake_all(&g->world);
	}
	if (key_down(g, GLFW_KEY_B))
	{
		g->world.gravity.y = fmaxf(GRAVITY_MIN, g->world.gravity.y - CTRL_GRAVITY_RATE * frame_time);
		world_wake_all(&g->world);
	}
	if (key_down(g, GLFW_KEY_Y))
		g->timeScale = fminf(TIME_SCALE_MAX, g->timeScale + CTRL_TIME_RATE * frame_time);
	if (key_down(g, GLFW_KEY_T))
		g->timeScale = fmaxf(0.0f, g->timeScale - CTRL_TIME_RATE * frame_time);
	if (key_down(g, GLFW_KEY_LEFT))
		camera_orbit(&g->camera, -CAM_ORBIT_SPEED * frame_time, 0.0f);
	if (key_down(g, GLFW_KEY_RIGHT))
		camera_orbit(&g->camera, CAM_ORBIT_SPEED * frame_time, 0.0f);
	if (key_down(g, GLFW_KEY_UP))
		camera_orbit(&g->camera, 0.0f, CAM_ORBIT_SPEED * frame_time);
	if (key_down(g, GLFW_KEY_DOWN))
		camera_orbit(&g->camera, 0.0f, -CAM_ORBIT_SPEED * frame_time);
	if (key_down(g, GLFW_KEY_EQUAL) || key_down(g, GLFW_KEY_KP_ADD))
		camera_zoom(&g->camera, -CAM_ZOOM_SPEED * frame_time);
	if (key_down(g, GLFW_KEY_MINUS) || key_down(g, GLFW_KEY_KP_SUBTRACT))
		camera_zoom(&g->camera, CAM_ZOOM_SPEED * frame_time);
}

/*
 * Live controls (the menu lists the same set on screen):
 *   ESC          open / close the menu (which pauses)
 *   SPACE        fire a bird
 *   1 / 2 / 3    spawn wall / pyramid / tower
 *   4            spawn a large wall (broad-phase stress test)
 *   P            pause / resume
 *   F1           debug wireframe on/off
 *   H            title readout on/off
 *   W / S        launch speed +/-
 *   A / D        launch angle +/-
 *   Q / E        bird mass -/+
 *   G / B        gravity +/-
 *   T / Y        time scale -/+
 *   arrows       orbit the camera, + / - zoom
 */
static void	handle_input(Game *g, float frame_time)
{
	handle_actions(g);
	handle_tuning(g, frame_time);
	apply_definitions(g);
}

/* While the menu is open it owns the keyboard and the mouse: the game's own
 * keys are not read, so arrow keys move the cursor instead of the camera. */
static void	handle_menu(Game *g, float frame_time)
{
	float	width;
	float	height;

	window_size(&g->window, &width, &height);
	menu_layout(&g->menu, width, height);
	menu_update(&g->menu, &g->window, frame_time);
	if (g->menu.valueChanged)
		apply_definitions(g);
	if (g->menu.pending != ACTION_NONE)
		run_menu_action(g, g->menu.pending);
}

/* Each body is drawn with the mesh of its collider shape and its own color;
 * the plane's transform comes from its equation, not its body position. */
static void	draw_bodies(Game *g)
{
	const RigidBody	*b;
	Mat4			model;
	float			s;
	int				i;

	i = 0;
	while (i < g->world.bodyCount)
	{
		b = &g->world.bodies[i];
		if (b->collider.type == SHAPE_SPHERE)
		{
			s = b->collider.radius * 2.0f;
			model = mat4_transform(b->position, b->orientation, vec3(s, s, s));
			renderer_draw(&g->renderer, &g->sphereMesh, model, b->color);
		}
		else if (b->collider.type == SHAPE_PLANE)
		{
			model = mat4_transform(collider_plane_origin(&b->collider), collider_plane_rotation(&b->collider), vec3(1.0f, 1.0f, 1.0f));
			renderer_draw(&g->renderer, &g->planeMesh, model, b->color);
		}
		else
		{
			model = mat4_transform(b->position, b->orientation, vec3_scale(b->collider.halfExtents, 2.0f));
			renderer_draw(&g->renderer, &g->cubeMesh, model, b->color);
		}
		i++;
	}
}

/* A thin wireframe bar from the launch point: its direction is the launch
 * angle and its length grows with the launch speed. */
static void	draw_aim(Game *g)
{
	Quat	rot = quat_from_axis_angle(vec3(0.0f, 0.0f, 1.0f), DEG2RAD(g->catapult.launchAngle));
	float	len = g->catapult.launchSpeed * AIM_LENGTH_PER_MPS;
	Vec3	center = vec3_add(g->catapult.launchPoint, vec3_scale(catapult_direction(&g->catapult), len * 0.5f));

	renderer_set_wireframe(&g->renderer, 1);
	renderer_draw_flat(&g->renderer, &g->cubeMesh, mat4_transform(center, rot, vec3(len, 0.08f, 0.08f)), vec3(1.0f, 0.9f, 0.2f));
	renderer_set_wireframe(&g->renderer, 0);
}

/* The time scale stretches the simulated time of a frame, never the step:
 * at 4x the world takes four times as many 1/120 s steps, so a fast bird
 * still cannot skip through a block. When a heavy scene cannot keep up, the
 * leftover time is dropped (the game slows down) instead of piling up. */
static void	step_world(Game *g, float *accumulator, float frame_time)
{
	int	steps;

	if (!g->paused)
		*accumulator += frame_time * g->timeScale;
	steps = 0;
	while (*accumulator >= g->fixedDt && steps < MAX_STEPS_PER_FRAME)
	{
		world_step(&g->world, g->fixedDt);
		*accumulator -= g->fixedDt;
		steps++;
	}
	if (steps == MAX_STEPS_PER_FRAME)
		*accumulator = 0.0f;
}

/* ESC: the menu takes over input and pauses the simulation; closing it
 * resumes, which is why the menu is also where Resume and Quit live. */
static void	toggle_menu(Game *g)
{
	g->menu.open = !g->menu.open;
	g->paused = g->menu.open;
	g->hud.refresh = 1;
}

static void	draw_overlay(Game *g)
{
	float	width;
	float	height;

	if (!g->menu.open)
		return ;
	window_size(&g->window, &width, &height);
	ui_begin(&g->ui, width, height);
	menu_draw(&g->menu, &g->ui);
	ui_end(&g->ui, &g->renderer);
}

/* Sleeps away the rest of the frame so the loop runs at FPS_CAP: the physics
 * keeps its fixed step whatever the display does, and no work goes into
 * frames nobody would see. */
static void	limit_frame_rate(double frame_start)
{
	struct timespec	pause;
	double			remaining;

	remaining = frame_start + 1.0 / FPS_CAP - glfwGetTime();
	if (remaining <= 0.0)
		return ;
	pause.tv_sec = 0;
	pause.tv_nsec = (long)(remaining * 1e9);
	nanosleep(&pause, NULL);
}

void	game_run(Game *g)
{
	double	prev = glfwGetTime();
	double	now;
	float	accumulator = 0.0f;
	float	frame_time;

	while (!window_should_close(&g->window))
	{
		window_poll_events(&g->window);
		now = glfwGetTime();
		frame_time = (float)(now - prev);
		prev = now;
		if (frame_time > 0.25f)
			frame_time = 0.25f;
		if (key_pressed(g, GLFW_KEY_ESCAPE))
			toggle_menu(g);
		if (g->menu.open)
			handle_menu(g, frame_time);
		else
			handle_input(g, frame_time);
		if (g->quit)
			break ;
		step_world(g, &accumulator, frame_time);
		hud_update(&g->hud, frame_time);
		renderer_begin_frame(&g->renderer, &g->camera, window_aspect(&g->window));
		draw_bodies(g);
		draw_aim(g);
		debugdraw_draw_colliders(&g->debug, &g->world, &g->renderer, &g->cubeMesh, &g->sphereMesh, &g->planeMesh);
		draw_overlay(g);
		hud_draw(&g->hud, &g->world, &g->catapult, g->timeScale, &g->window, g->paused);
		window_swap_buffers(&g->window);
		limit_frame_rate(now);
	}
}
