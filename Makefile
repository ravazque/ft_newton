NAME     = newton
OBJDIR   = objects
SRCDIR   = srcs

CC       = cc
CFLAGS   = -Wall -Wextra -Werror -g3 -O3
CPPFLAGS = -Iincludes -Iglad/include
LDLIBS   = -lglfw -lGL -ldl -lpthread -lm

GLADFLAGS = -g3 -O3

ARGS     =

SRCS = main.c \
       utils/start_check.c utils/csvfile.c \
       math/vec3.c math/mat4.c math/quat.c math/mat3.c \
       render/window.c render/shader.c render/mesh.c render/camera.c \
       render/renderer.c render/debugdraw.c render/font.c render/ui.c \
       physics/rigidbody.c physics/integrator.c physics/world.c \
       collision/collider.c collision/broadphase.c collision/narrowphase.c \
       collision/narrowphase_box.c \
       collision/resolver.c \
       game/game.c game/scene.c game/input.c \
       game/trebuchet.c game/projectile.c game/structure.c \
       game/hud.c game/objectdef.c game/menu.c

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

clean:
	rm -rf $(OBJDIR)

fclean: clean
	rm -rf $(NAME)

re: fclean all

.PHONY: all run clean fclean re

-include $(DEPS)
