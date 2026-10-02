NAME     = newton
OBJDIR   = objects
SRCDIR   = srcs

CC       = cc
CFLAGS   = -Wall -Wextra -Werror -g3 -O3
CPPFLAGS = -Iincludes -Iglad/include
LDLIBS   = -lglfw -lGL -ldl -lpthread -lm

GLADFLAGS = -g3 -O3

ARGS     =

VALGRIND      = valgrind
SUPP          = docs/valgrind.supp
SUPP_RECENT   = docs/valgrind_recent.supp
# Expanded only by `make valgrind`: the recent file is added when this valgrind accepts it.
SUPPFLAGS     = --suppressions=$(SUPP) \
                $(shell $(VALGRIND) --suppressions=$(SUPP_RECENT) true >/dev/null 2>&1 \
                    && echo --suppressions=$(SUPP_RECENT))
VALGRINDFLAGS = --leak-check=full --show-leak-kinds=all --track-origins=yes \
                --keep-debuginfo=yes $(SUPPFLAGS)

SRCS = main.c \
       math/vec3.c math/scalar.c math/mat3.c math/mat4.c math/quat.c \
       physics/rigidbody.c physics/inertia.c physics/integrator.c physics/world.c physics/sleep.c physics/cull.c \
       collision/collider.c collision/broadphase.c collision/narrowphase.c collision/contact_sphere.c \
       collision/contact_box.c collision/obb.c collision/sat.c collision/manifold.c collision/query.c \
       response/resolver.c response/impulse.c response/correction.c \
       render/window.c render/shader.c render/mesh.c render/camera.c render/renderer.c render/debugdraw.c \
       render/font.c render/ui.c \
       menu/menu.c menu/menu_layout.c menu/menu_input.c menu/menu_draw.c \
       data/csvfile.c data/csv_values.c data/objectdef.c data/trebuchet_file.c \
       game/game.c game/draw.c game/input.c game/hud.c game/scene.c game/actions.c game/menu_rows.c \
       game/trebuchet.c game/trebuchet_clearance.c game/projectile.c game/structure.c \
       utils/start_check.c

GLAD = glad/src/gl.c

OBJS = $(SRCS:%.c=$(OBJDIR)/%.o) $(GLAD:%.c=$(OBJDIR)/%.o)
DEPS = $(OBJS:.o=.d)


all: $(NAME)

$(NAME): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDLIBS)
	@echo "==> built $@"

# Project sources (strict warnings).
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) -MMD -MP -c $< -o $@

# GLAD (relaxed warnings).
$(OBJDIR)/%.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(GLADFLAGS) $(CPPFLAGS) -MMD -MP -c $< -o $@

run: all
	./$(NAME) $(ARGS)

# docs/valgrind.supp, plus docs/valgrind_recent.supp on valgrind >= 3.22.
valgrind: all
	$(VALGRIND) $(VALGRINDFLAGS) ./$(NAME) $(ARGS)

clean:
	rm -rf $(OBJDIR)

fclean: clean
	rm -rf $(NAME)

re: fclean all

.PHONY: all run valgrind clean fclean re

-include $(DEPS)
