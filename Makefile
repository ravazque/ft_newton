NAME     = newton
BINDIR   = bin
OBJDIR   = bin/objects
SRCDIR   = srcs

CC       = cc
CFLAGS   = -Wall -Wextra -Werror -g3 -O3
CPPFLAGS = -Iincludes -Iglad/include
LDLIBS   = -lglfw -lGL -ldl -lpthread -lm

GLADFLAGS = -g3 -O3

# Sources, relative to srcs/. Add each new .c 00:00:00 by hand (no wildcard): you decide
# exactly what enters the build.
#   [DONE]  math/vec3 mat4 quat, render/*, main
#   [TODO]  math/mat3, physics/*, collision/*, render/debugdraw, game/*
SRCS = main.c \
       math/vec3.c math/mat4.c math/quat.c math/mat3.c \
       render/window.c render/shader.c render/mesh.c render/camera.c \
       render/renderer.c render/debugdraw.c \
       physics/rigidbody.c physics/integrator.c physics/world.c \
       collision/collider.c collision/broadphase.c collision/narrowphase.c \
       collision/resolver.c \
       game/game.c game/catapult.c game/projectile.c game/structure.c \
       game/hud.c

GLAD = glad/src/gl.c

OBJS = $(SRCS:%.c=$(OBJDIR)/%.o) $(GLAD:%.c=$(OBJDIR)/%.o)
DEPS = $(OBJS:.o=.d)


all: $(BINDIR)/$(NAME)

$(BINDIR)/$(NAME): $(OBJS)
	mkdir -p $(BINDIR)
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
	./$(BINDIR)/$(NAME)

clean:
	rm -rf $(OBJDIR)

fclean: clean
	rm -rf $(BINDIR)

re: fclean all

.PHONY: all run clean fclean re

-include $(DEPS)
