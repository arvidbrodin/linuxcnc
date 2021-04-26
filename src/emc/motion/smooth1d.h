/*
 * Copyright (c) 2021 Arvid Brodin
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Description: smooth1d.h
 *   A jerk-limited single axis trajectory planner.
 */

#ifndef SMOOTH1D_H
#define SMOOTH1D_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct smooth1d smooth1d_t;

extern smooth1d_t *smooth1d_new(double max_jerk);
extern void smooth1d_free(smooth1d_t *);
extern int smooth1d_reset_pos(smooth1d_t *, double new_pos);
extern void smooth1d_replan(smooth1d_t *, double s_target, double v_limit, double a_limit);
extern double smooth1d_get_pos_cmd(const smooth1d_t *);
extern double smooth1d_get_vel_limit(const smooth1d_t *);
extern void smooth1d_stop(smooth1d_t *);
extern void smooth1d_update(smooth1d_t *, double dt);
extern double smooth1d_get_pos(const smooth1d_t *);
extern double smooth1d_get_vel(const smooth1d_t *);
extern double smooth1d_get_acc(const smooth1d_t *);
extern int smooth1d_is_active(const smooth1d_t *);

#ifdef __cplusplus
}
#endif

#endif /* SMOOTH1D_H */
