#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_7115639500879288649);
void live_err_fun(double *nom_x, double *delta_x, double *out_3754448067035541963);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8136815140443433684);
void live_H_mod_fun(double *state, double *out_1372702547662345762);
void live_f_fun(double *state, double dt, double *out_1307160822434448296);
void live_F_fun(double *state, double dt, double *out_2215617637110578166);
void live_h_4(double *state, double *unused, double *out_7120546279518817651);
void live_H_4(double *state, double *unused, double *out_5135387758362380378);
void live_h_9(double *state, double *unused, double *out_5842699186238266074);
void live_H_9(double *state, double *unused, double *out_2151831176902067092);
void live_h_10(double *state, double *unused, double *out_9032588997426191900);
void live_H_10(double *state, double *unused, double *out_5605273353248030079);
void live_h_12(double *state, double *unused, double *out_6945368471624154697);
void live_H_12(double *state, double *unused, double *out_6930097938304438242);
void live_h_31(double *state, double *unused, double *out_2725955837001841262);
void live_H_31(double *state, double *unused, double *out_1768725700989773002);
void live_h_32(double *state, double *unused, double *out_4975149596805211682);
void live_H_32(double *state, double *unused, double *out_5987424938661682072);
void live_h_13(double *state, double *unused, double *out_8111205192185790561);
void live_H_13(double *state, double *unused, double *out_6585823007046766775);
void live_h_14(double *state, double *unused, double *out_5842699186238266074);
void live_H_14(double *state, double *unused, double *out_2151831176902067092);
void live_h_33(double *state, double *unused, double *out_3422159033991878612);
void live_H_33(double *state, double *unused, double *out_8427860592283941427);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}