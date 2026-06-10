NAME     = newton
OBJDIR   = objects
SRCDIR   = srcs

CC       = cc
CFLAGS   = -Wall -Wextra -Werror -g3 -O3
CPPFLAGS = -Iincludes -Iglad/include
LDLIBS   = -lglfw -lGL -ldl -lpthread -lm

GLADFLAGS = -g3 -O3

VALGRIND = valgrind
# VALFLAGS = --leak-check=full --show-leak-kinds=all --track-origins=yes 
VALFLAGS = --leak-check=full --track-origins=yes 
VALSUPP  = development/valgrind.supp
ARGS     =

# [TODO]  math/mat3, math/quat (quat_integrate), physics/*, collision/resolver
SRCS = main.c \
       utils/start_check.c \
       math/vec3.c math/mat4.c math/quat.c math/mat3.c \
       render/window.c render/shader.c render/mesh.c render/camera.c \
       render/renderer.c render/debugdraw.c \
       physics/rigidbody.c physics/integrator.c physics/world.c \
       collision/collider.c collision/broadphase.c collision/narrowphase.c \
       collision/narrowphase_box.c \
       collision/resolver.c \
       game/game.c game/catapult.c game/projectile.c game/structure.c \
       game/hud.c

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

valgrind: all
	$(VALGRIND) $(VALFLAGS) --suppressions=$(VALSUPP) ./$(NAME) $(ARGS)

clean:
	rm -rf $(OBJDIR)

fclean: clean
	rm -rf $(NAME)

re: fclean all

.PHONY: all run valgrind clean fclean re

-include $(DEPS)
