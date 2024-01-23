#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1596747906009595470) {
   out_1596747906009595470[0] = delta_x[0] + nom_x[0];
   out_1596747906009595470[1] = delta_x[1] + nom_x[1];
   out_1596747906009595470[2] = delta_x[2] + nom_x[2];
   out_1596747906009595470[3] = delta_x[3] + nom_x[3];
   out_1596747906009595470[4] = delta_x[4] + nom_x[4];
   out_1596747906009595470[5] = delta_x[5] + nom_x[5];
   out_1596747906009595470[6] = delta_x[6] + nom_x[6];
   out_1596747906009595470[7] = delta_x[7] + nom_x[7];
   out_1596747906009595470[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_391526689587903592) {
   out_391526689587903592[0] = -nom_x[0] + true_x[0];
   out_391526689587903592[1] = -nom_x[1] + true_x[1];
   out_391526689587903592[2] = -nom_x[2] + true_x[2];
   out_391526689587903592[3] = -nom_x[3] + true_x[3];
   out_391526689587903592[4] = -nom_x[4] + true_x[4];
   out_391526689587903592[5] = -nom_x[5] + true_x[5];
   out_391526689587903592[6] = -nom_x[6] + true_x[6];
   out_391526689587903592[7] = -nom_x[7] + true_x[7];
   out_391526689587903592[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1594659186948991827) {
   out_1594659186948991827[0] = 1.0;
   out_1594659186948991827[1] = 0;
   out_1594659186948991827[2] = 0;
   out_1594659186948991827[3] = 0;
   out_1594659186948991827[4] = 0;
   out_1594659186948991827[5] = 0;
   out_1594659186948991827[6] = 0;
   out_1594659186948991827[7] = 0;
   out_1594659186948991827[8] = 0;
   out_1594659186948991827[9] = 0;
   out_1594659186948991827[10] = 1.0;
   out_1594659186948991827[11] = 0;
   out_1594659186948991827[12] = 0;
   out_1594659186948991827[13] = 0;
   out_1594659186948991827[14] = 0;
   out_1594659186948991827[15] = 0;
   out_1594659186948991827[16] = 0;
   out_1594659186948991827[17] = 0;
   out_1594659186948991827[18] = 0;
   out_1594659186948991827[19] = 0;
   out_1594659186948991827[20] = 1.0;
   out_1594659186948991827[21] = 0;
   out_1594659186948991827[22] = 0;
   out_1594659186948991827[23] = 0;
   out_1594659186948991827[24] = 0;
   out_1594659186948991827[25] = 0;
   out_1594659186948991827[26] = 0;
   out_1594659186948991827[27] = 0;
   out_1594659186948991827[28] = 0;
   out_1594659186948991827[29] = 0;
   out_1594659186948991827[30] = 1.0;
   out_1594659186948991827[31] = 0;
   out_1594659186948991827[32] = 0;
   out_1594659186948991827[33] = 0;
   out_1594659186948991827[34] = 0;
   out_1594659186948991827[35] = 0;
   out_1594659186948991827[36] = 0;
   out_1594659186948991827[37] = 0;
   out_1594659186948991827[38] = 0;
   out_1594659186948991827[39] = 0;
   out_1594659186948991827[40] = 1.0;
   out_1594659186948991827[41] = 0;
   out_1594659186948991827[42] = 0;
   out_1594659186948991827[43] = 0;
   out_1594659186948991827[44] = 0;
   out_1594659186948991827[45] = 0;
   out_1594659186948991827[46] = 0;
   out_1594659186948991827[47] = 0;
   out_1594659186948991827[48] = 0;
   out_1594659186948991827[49] = 0;
   out_1594659186948991827[50] = 1.0;
   out_1594659186948991827[51] = 0;
   out_1594659186948991827[52] = 0;
   out_1594659186948991827[53] = 0;
   out_1594659186948991827[54] = 0;
   out_1594659186948991827[55] = 0;
   out_1594659186948991827[56] = 0;
   out_1594659186948991827[57] = 0;
   out_1594659186948991827[58] = 0;
   out_1594659186948991827[59] = 0;
   out_1594659186948991827[60] = 1.0;
   out_1594659186948991827[61] = 0;
   out_1594659186948991827[62] = 0;
   out_1594659186948991827[63] = 0;
   out_1594659186948991827[64] = 0;
   out_1594659186948991827[65] = 0;
   out_1594659186948991827[66] = 0;
   out_1594659186948991827[67] = 0;
   out_1594659186948991827[68] = 0;
   out_1594659186948991827[69] = 0;
   out_1594659186948991827[70] = 1.0;
   out_1594659186948991827[71] = 0;
   out_1594659186948991827[72] = 0;
   out_1594659186948991827[73] = 0;
   out_1594659186948991827[74] = 0;
   out_1594659186948991827[75] = 0;
   out_1594659186948991827[76] = 0;
   out_1594659186948991827[77] = 0;
   out_1594659186948991827[78] = 0;
   out_1594659186948991827[79] = 0;
   out_1594659186948991827[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2764010797200486018) {
   out_2764010797200486018[0] = state[0];
   out_2764010797200486018[1] = state[1];
   out_2764010797200486018[2] = state[2];
   out_2764010797200486018[3] = state[3];
   out_2764010797200486018[4] = state[4];
   out_2764010797200486018[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2764010797200486018[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2764010797200486018[7] = state[7];
   out_2764010797200486018[8] = state[8];
}
void F_fun(double *state, double dt, double *out_203653904900826828) {
   out_203653904900826828[0] = 1;
   out_203653904900826828[1] = 0;
   out_203653904900826828[2] = 0;
   out_203653904900826828[3] = 0;
   out_203653904900826828[4] = 0;
   out_203653904900826828[5] = 0;
   out_203653904900826828[6] = 0;
   out_203653904900826828[7] = 0;
   out_203653904900826828[8] = 0;
   out_203653904900826828[9] = 0;
   out_203653904900826828[10] = 1;
   out_203653904900826828[11] = 0;
   out_203653904900826828[12] = 0;
   out_203653904900826828[13] = 0;
   out_203653904900826828[14] = 0;
   out_203653904900826828[15] = 0;
   out_203653904900826828[16] = 0;
   out_203653904900826828[17] = 0;
   out_203653904900826828[18] = 0;
   out_203653904900826828[19] = 0;
   out_203653904900826828[20] = 1;
   out_203653904900826828[21] = 0;
   out_203653904900826828[22] = 0;
   out_203653904900826828[23] = 0;
   out_203653904900826828[24] = 0;
   out_203653904900826828[25] = 0;
   out_203653904900826828[26] = 0;
   out_203653904900826828[27] = 0;
   out_203653904900826828[28] = 0;
   out_203653904900826828[29] = 0;
   out_203653904900826828[30] = 1;
   out_203653904900826828[31] = 0;
   out_203653904900826828[32] = 0;
   out_203653904900826828[33] = 0;
   out_203653904900826828[34] = 0;
   out_203653904900826828[35] = 0;
   out_203653904900826828[36] = 0;
   out_203653904900826828[37] = 0;
   out_203653904900826828[38] = 0;
   out_203653904900826828[39] = 0;
   out_203653904900826828[40] = 1;
   out_203653904900826828[41] = 0;
   out_203653904900826828[42] = 0;
   out_203653904900826828[43] = 0;
   out_203653904900826828[44] = 0;
   out_203653904900826828[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_203653904900826828[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_203653904900826828[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_203653904900826828[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_203653904900826828[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_203653904900826828[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_203653904900826828[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_203653904900826828[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_203653904900826828[53] = -9.8000000000000007*dt;
   out_203653904900826828[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_203653904900826828[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_203653904900826828[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_203653904900826828[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_203653904900826828[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_203653904900826828[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_203653904900826828[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_203653904900826828[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_203653904900826828[62] = 0;
   out_203653904900826828[63] = 0;
   out_203653904900826828[64] = 0;
   out_203653904900826828[65] = 0;
   out_203653904900826828[66] = 0;
   out_203653904900826828[67] = 0;
   out_203653904900826828[68] = 0;
   out_203653904900826828[69] = 0;
   out_203653904900826828[70] = 1;
   out_203653904900826828[71] = 0;
   out_203653904900826828[72] = 0;
   out_203653904900826828[73] = 0;
   out_203653904900826828[74] = 0;
   out_203653904900826828[75] = 0;
   out_203653904900826828[76] = 0;
   out_203653904900826828[77] = 0;
   out_203653904900826828[78] = 0;
   out_203653904900826828[79] = 0;
   out_203653904900826828[80] = 1;
}
void h_25(double *state, double *unused, double *out_4417392115788627863) {
   out_4417392115788627863[0] = state[6];
}
void H_25(double *state, double *unused, double *out_644877371219942396) {
   out_644877371219942396[0] = 0;
   out_644877371219942396[1] = 0;
   out_644877371219942396[2] = 0;
   out_644877371219942396[3] = 0;
   out_644877371219942396[4] = 0;
   out_644877371219942396[5] = 0;
   out_644877371219942396[6] = 1;
   out_644877371219942396[7] = 0;
   out_644877371219942396[8] = 0;
}
void h_24(double *state, double *unused, double *out_2478336189205641742) {
   out_2478336189205641742[0] = state[4];
   out_2478336189205641742[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8499989338730934766) {
   out_8499989338730934766[0] = 0;
   out_8499989338730934766[1] = 0;
   out_8499989338730934766[2] = 0;
   out_8499989338730934766[3] = 0;
   out_8499989338730934766[4] = 1;
   out_8499989338730934766[5] = 0;
   out_8499989338730934766[6] = 0;
   out_8499989338730934766[7] = 0;
   out_8499989338730934766[8] = 0;
   out_8499989338730934766[9] = 0;
   out_8499989338730934766[10] = 0;
   out_8499989338730934766[11] = 0;
   out_8499989338730934766[12] = 0;
   out_8499989338730934766[13] = 0;
   out_8499989338730934766[14] = 1;
   out_8499989338730934766[15] = 0;
   out_8499989338730934766[16] = 0;
   out_8499989338730934766[17] = 0;
}
void h_30(double *state, double *unused, double *out_8894463622726820322) {
   out_8894463622726820322[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3163210329727191023) {
   out_3163210329727191023[0] = 0;
   out_3163210329727191023[1] = 0;
   out_3163210329727191023[2] = 0;
   out_3163210329727191023[3] = 0;
   out_3163210329727191023[4] = 1;
   out_3163210329727191023[5] = 0;
   out_3163210329727191023[6] = 0;
   out_3163210329727191023[7] = 0;
   out_3163210329727191023[8] = 0;
}
void h_26(double *state, double *unused, double *out_4550607183321885649) {
   out_4550607183321885649[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3096625947654113828) {
   out_3096625947654113828[0] = 0;
   out_3096625947654113828[1] = 0;
   out_3096625947654113828[2] = 0;
   out_3096625947654113828[3] = 0;
   out_3096625947654113828[4] = 0;
   out_3096625947654113828[5] = 0;
   out_3096625947654113828[6] = 0;
   out_3096625947654113828[7] = 1;
   out_3096625947654113828[8] = 0;
}
void h_27(double *state, double *unused, double *out_727497947327019674) {
   out_727497947327019674[0] = state[3];
}
void H_27(double *state, double *unused, double *out_988447017926766112) {
   out_988447017926766112[0] = 0;
   out_988447017926766112[1] = 0;
   out_988447017926766112[2] = 0;
   out_988447017926766112[3] = 1;
   out_988447017926766112[4] = 0;
   out_988447017926766112[5] = 0;
   out_988447017926766112[6] = 0;
   out_988447017926766112[7] = 0;
   out_988447017926766112[8] = 0;
}
void h_29(double *state, double *unused, double *out_7172109078348512945) {
   out_7172109078348512945[0] = state[1];
}
void H_29(double *state, double *unused, double *out_3673441674041583207) {
   out_3673441674041583207[0] = 0;
   out_3673441674041583207[1] = 1;
   out_3673441674041583207[2] = 0;
   out_3673441674041583207[3] = 0;
   out_3673441674041583207[4] = 0;
   out_3673441674041583207[5] = 0;
   out_3673441674041583207[6] = 0;
   out_3673441674041583207[7] = 0;
   out_3673441674041583207[8] = 0;
}
void h_28(double *state, double *unused, double *out_5671543570086829472) {
   out_5671543570086829472[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1408957343027947367) {
   out_1408957343027947367[0] = 1;
   out_1408957343027947367[1] = 0;
   out_1408957343027947367[2] = 0;
   out_1408957343027947367[3] = 0;
   out_1408957343027947367[4] = 0;
   out_1408957343027947367[5] = 0;
   out_1408957343027947367[6] = 0;
   out_1408957343027947367[7] = 0;
   out_1408957343027947367[8] = 0;
}
void h_31(double *state, double *unused, double *out_4222485360929914021) {
   out_4222485360929914021[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3722834049887465304) {
   out_3722834049887465304[0] = 0;
   out_3722834049887465304[1] = 0;
   out_3722834049887465304[2] = 0;
   out_3722834049887465304[3] = 0;
   out_3722834049887465304[4] = 0;
   out_3722834049887465304[5] = 0;
   out_3722834049887465304[6] = 0;
   out_3722834049887465304[7] = 0;
   out_3722834049887465304[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_1596747906009595470) {
  err_fun(nom_x, delta_x, out_1596747906009595470);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_391526689587903592) {
  inv_err_fun(nom_x, true_x, out_391526689587903592);
}
void car_H_mod_fun(double *state, double *out_1594659186948991827) {
  H_mod_fun(state, out_1594659186948991827);
}
void car_f_fun(double *state, double dt, double *out_2764010797200486018) {
  f_fun(state,  dt, out_2764010797200486018);
}
void car_F_fun(double *state, double dt, double *out_203653904900826828) {
  F_fun(state,  dt, out_203653904900826828);
}
void car_h_25(double *state, double *unused, double *out_4417392115788627863) {
  h_25(state, unused, out_4417392115788627863);
}
void car_H_25(double *state, double *unused, double *out_644877371219942396) {
  H_25(state, unused, out_644877371219942396);
}
void car_h_24(double *state, double *unused, double *out_2478336189205641742) {
  h_24(state, unused, out_2478336189205641742);
}
void car_H_24(double *state, double *unused, double *out_8499989338730934766) {
  H_24(state, unused, out_8499989338730934766);
}
void car_h_30(double *state, double *unused, double *out_8894463622726820322) {
  h_30(state, unused, out_8894463622726820322);
}
void car_H_30(double *state, double *unused, double *out_3163210329727191023) {
  H_30(state, unused, out_3163210329727191023);
}
void car_h_26(double *state, double *unused, double *out_4550607183321885649) {
  h_26(state, unused, out_4550607183321885649);
}
void car_H_26(double *state, double *unused, double *out_3096625947654113828) {
  H_26(state, unused, out_3096625947654113828);
}
void car_h_27(double *state, double *unused, double *out_727497947327019674) {
  h_27(state, unused, out_727497947327019674);
}
void car_H_27(double *state, double *unused, double *out_988447017926766112) {
  H_27(state, unused, out_988447017926766112);
}
void car_h_29(double *state, double *unused, double *out_7172109078348512945) {
  h_29(state, unused, out_7172109078348512945);
}
void car_H_29(double *state, double *unused, double *out_3673441674041583207) {
  H_29(state, unused, out_3673441674041583207);
}
void car_h_28(double *state, double *unused, double *out_5671543570086829472) {
  h_28(state, unused, out_5671543570086829472);
}
void car_H_28(double *state, double *unused, double *out_1408957343027947367) {
  H_28(state, unused, out_1408957343027947367);
}
void car_h_31(double *state, double *unused, double *out_4222485360929914021) {
  h_31(state, unused, out_4222485360929914021);
}
void car_H_31(double *state, double *unused, double *out_3722834049887465304) {
  H_31(state, unused, out_3722834049887465304);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
