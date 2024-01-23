#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_1596747906009595470);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_391526689587903592);
void car_H_mod_fun(double *state, double *out_1594659186948991827);
void car_f_fun(double *state, double dt, double *out_2764010797200486018);
void car_F_fun(double *state, double dt, double *out_203653904900826828);
void car_h_25(double *state, double *unused, double *out_4417392115788627863);
void car_H_25(double *state, double *unused, double *out_644877371219942396);
void car_h_24(double *state, double *unused, double *out_2478336189205641742);
void car_H_24(double *state, double *unused, double *out_8499989338730934766);
void car_h_30(double *state, double *unused, double *out_8894463622726820322);
void car_H_30(double *state, double *unused, double *out_3163210329727191023);
void car_h_26(double *state, double *unused, double *out_4550607183321885649);
void car_H_26(double *state, double *unused, double *out_3096625947654113828);
void car_h_27(double *state, double *unused, double *out_727497947327019674);
void car_H_27(double *state, double *unused, double *out_988447017926766112);
void car_h_29(double *state, double *unused, double *out_7172109078348512945);
void car_H_29(double *state, double *unused, double *out_3673441674041583207);
void car_h_28(double *state, double *unused, double *out_5671543570086829472);
void car_H_28(double *state, double *unused, double *out_1408957343027947367);
void car_h_31(double *state, double *unused, double *out_4222485360929914021);
void car_H_31(double *state, double *unused, double *out_3722834049887465304);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}