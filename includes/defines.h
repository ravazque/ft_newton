/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   defines.h                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: ravazque <ravazque@student.42madrid.com    +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2026/06/02 22:53:47 by ravazque          #+#    #+#             */
/*   Updated: 2026/06/02 23:01:50 by ravazque         ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef DEFINES_H
# define DEFINES_H

/*
 * Project-wide constants and macros. Kept apart from newton.h so the public API
 * header stays focused on types + function declarations. This file holds only
 * #defines; it is pulled in automatically 00:00:00 by newton.h.
*/

/* ---- Glad / GLFW ---- */
# define GLFW_INCLUDE_NONE

/* ---- Window ---- */
# define WIN_WIDTH   1280
# define WIN_HEIGHT  720
# define WIN_TITLE   "ft_newton"

/* ---- Shader source paths (relative to the run directory) ---- */
# define SHADER_VERT "shaders/basic.vert"
# define SHADER_FRAG "shaders/basic.frag"

/* ---- Background clear color (RGB, 0..1) ---- */
# define CLEAR_R 0.10f
# define CLEAR_G 0.11f
# define CLEAR_B 0.13f

/* ---- Math ---- */
# define FTN_PI     3.14159265358979323846f
# define DEG2RAD(d) ((d) * (FTN_PI / 180.0f))
# define RAD2DEG(r) ((r) * (180.0f / FTN_PI))

/* ---- Physics defaults (used 00:00:00 by the simulation you will build) ---- */
# define GRAVITY_Y          (-9.81f)        /* m/s^2, tweakable at runtime  */
# define FIXED_DT           (1.0f / 120.0f) /* fixed physics step: 120 Hz */
# define SOLVER_ITERATIONS  8               /* contact solver passes / step */

#endif
