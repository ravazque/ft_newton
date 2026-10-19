NAME     = newton
OBJDIR   = objects
SRCDIR   = srcs

CC       = cc

# O sets the optimisation level (make O=0, 1, 2, 3, s or g); objects are rebuilt whenever the compile flags change.
O        = 3

CFLAGS   = -Wall -Wextra -Werror -g3 -O$(O)
CPPFLAGS = -Iincludes -Iglad/include
LDLIBS   = -lglfw -lGL -ldl -lpthread -lm

GLADFLAGS   = -g3 -O$(O)
FLAGS_STAMP = $(OBJDIR)/.flags

ARGS     =

VALGRIND      = valgrind
SUPP          = docs/valgrind.supp
SUPP_RECENT   = docs/valgrind_recent.supp
# The recent suppression file is added only when this valgrind accepts it.
SUPPFLAGS     = --suppressions=$(SUPP) $(shell $(VALGRIND) --suppressions=$(SUPP_RECENT) true >/dev/null 2>&1 && echo --suppressions=$(SUPP_RECENT))
VALGRINDFLAGS = --leak-check=full --show-leak-kinds=all --track-origins=yes --keep-debuginfo=yes $(SUPPFLAGS)

SRCS = main.c \
       math/vec3.c math/scalar.c math/mat3.c math/mat4.c math/quat.c \
       physics/rigidbody.c physics/inertia.c physics/mesh_inertia.c physics/integrator.c physics/world.c physics/joint_list.c physics/sleep.c physics/cull.c \
       collision/collider.c collision/broadphase.c collision/narrowphase.c collision/overlap.c collision/obb.c collision/sat.c collision/manifold.c \
       collision/contact_sphere.c collision/contact_box.c collision/contact_mesh.c collision/contact_box_mesh.c \
       collision/mesh_shape.c collision/mesh_repair.c collision/mesh_topology.c collision/bvh.c collision/closest.c collision/mesh_query.c \
       response/resolver.c response/impulse.c response/correction.c response/joint_point.c response/joint_limit.c \
       render/window.c render/shader.c render/mesh.c render/camera.c render/renderer.c render/debugdraw.c render/font.c render/ui.c \
       menu/menu.c menu/menu_layout.c menu/menu_input.c menu/menu_draw.c \
       data/csv_file.c data/csv_values.c data/object_def.c data/trebuchet_file.c data/ragdoll_file.c data/obj_load.c data/obj_face.c data/obj_triangulate.c \
       game/game.c game/draw.c game/input.c game/hud.c game/scene.c game/scene_check.c game/actions.c game/menu_rows.c game/model_library.c game/model.c \
       game/placement.c game/camera_collision.c game/structure.c game/rain.c game/ragdoll.c game/trebuchet.c game/trebuchet_clearance.c game/projectile.c \
       utils/arguments.c utils/array.c utils/file.c

GLAD = glad/src/gl.c

OBJS = $(SRCS:%.c=$(OBJDIR)/%.o) $(GLAD:%.c=$(OBJDIR)/%.o)
DEPS = $(OBJS:.o=.d)


all: $(NAME)

$(NAME): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDLIBS)
	@echo "==> built $@"

$(OBJDIR)/%.o: $(SRCDIR)/%.c $(FLAGS_STAMP)
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) -MMD -MP -c $< -o $@

$(OBJDIR)/%.o: %.c $(FLAGS_STAMP)
	mkdir -p $(dir $@)
	$(CC) $(GLADFLAGS) $(CPPFLAGS) -MMD -MP -c $< -o $@

COMPILE_FLAGS = $(CC) $(CFLAGS) $(GLADFLAGS) $(CPPFLAGS)
$(shell [ ! -f $(FLAGS_STAMP) ] || echo '$(COMPILE_FLAGS)' | cmp -s - $(FLAGS_STAMP) || echo '$(COMPILE_FLAGS)' > $(FLAGS_STAMP))

$(FLAGS_STAMP):
	@mkdir -p $(OBJDIR)
	@echo '$(COMPILE_FLAGS)' > $@

install:
	@command -v $(CC) >/dev/null || { echo "$(CC) not found: install gcc or clang" >&2; exit 1; }
	@command -v pkg-config >/dev/null || { echo "pkg-config not found: install pkg-config" >&2; exit 1; }
	@pkg-config --exists glfw3 || { echo "GLFW 3 not found: install libglfw3-dev (Debian/Ubuntu) or glfw (Arch)" >&2; exit 1; }
	@pkg-config --exists gl || { echo "OpenGL not found: install libgl1-mesa-dev (Debian/Ubuntu) or mesa (Arch)" >&2; exit 1; }
	@echo "dependencies ready: make run"

run: all
	./$(NAME) $(ARGS)

valgrind: all
	$(VALGRIND) $(VALGRINDFLAGS) ./$(NAME) $(ARGS)

clean:
	rm -rf $(OBJDIR)

fclean: clean
	rm -rf $(NAME)

re: fclean all

help:
	@echo "make           build ./newton (O=0|1|2|3|s|g sets the optimisation level, 3 by default)"
	@echo "make install   check the compiler, pkg-config, GLFW and OpenGL"
	@echo "make run       build and run ./newton $(ARGS)"
	@echo "make valgrind  build and run under valgrind with docs/*.supp"
	@echo "make clean     remove the objects"
	@echo "make fclean    remove the objects and the binary"
	@echo "make re        fclean, then build"

.PHONY: all help install run valgrind clean fclean re

-include $(DEPS)
