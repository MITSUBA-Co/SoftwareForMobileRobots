#include "fisheye.hpp"

using namespace cv;
using namespace std;

int fisheye_proj_plane(const Mat &f_im, Mat &p_im, const double *f_prm,
                       const double *p_prm, bool nn)
/* --------------------------------------------------------------------------------- */
/*  fisheye_proj_plane: projects a fisheye image to a plane.                         */
/*    f_im:      input fisheye image                                                 */
/*    p_im:      output plane image                                                  */
/*    f_prm[8]:  fisheye camera parameters (k1, k2, k3, k4, fx, fy, cx, cy)          */
/*    p_prm[11]: plane parameters (X0, Y0, Z0, e1X, e1Y, e1Z, e2X, e2Y, e2Z, i0, j0) */
/*    nn:        interpolation. true:nearest neighbor, false:bi-linear               */
/* --------------------------------------------------------------------------------- */
{
  if((f_im.type() != CV_8UC1 && f_im.type() != CV_8UC3) ||
     (f_im.type() != p_im.type())) {
    return 1; }

  int ch;
  if(f_im.type() == CV_8UC3) {
    ch = 3; }
  else {
    ch = 1; }

  for(int j=0; j<(p_im.rows); j++) {
    for(int i=0; i<(p_im.cols); i++) {

      double px, py, X, Y, Z, Rxy, theta, r, a, b, x, y;

      px = (double)i - p_prm[9];   //  i - i0
      py = (double)j - p_prm[10];  //  j - j0

      X = px*p_prm[3] + py*p_prm[6] + p_prm[0]; // P =px*e1 + py*e2 + P0
      Y = px*p_prm[4] + py*p_prm[7] + p_prm[1];
      Z = px*p_prm[5] + py*p_prm[8] + p_prm[2];

      Rxy = sqrt(X*X + Y*Y);
      theta = atan2(Rxy,Z);

      double t2, t4, t6, t8;
      t2 = theta*theta;
      t4 = t2*t2;
      t6 = t4*t2;
      t8 = t4*t4;
      r = theta*(1.0 + f_prm[0]*t2 + f_prm[1]*t4 + f_prm[2]*t6 + f_prm[3]*t8);

      a = (Rxy == 0.0) ? 1.0 : (X/Rxy);
      b = (Rxy == 0.0) ? 0.0 : (Y/Rxy);
      x = f_prm[4]*r*a;
      y = f_prm[5]*r*b;

      if(nn) { // nearest neighbor

	int k, l, m;
	k = (int)round(x + f_prm[6]);
	l = (int)round(y + f_prm[7]);

	if((k >= 0) && (k < f_im.cols) && (l >= 0) && (l < f_im.rows)) {
	  for(m=0; m<ch; m++) {
	    p_im.at<unsigned char>(j,ch*i+m) = f_im.at<unsigned char>(l,ch*k+m); } }
	else {
	  for(m=0; m<ch; m++) {
	    p_im.at<unsigned char>(j,ch*i+m) = 0; } }

      }
      else { // bi-linear

	int k, l, m;
	double xk, yl, fk, fl;
	xk = floor(x + f_prm[6]);
	yl = floor(y + f_prm[7]);
	fk = (x + f_prm[6]) - xk;
	fl = (y + f_prm[7]) - yl;
	k  = (int)xk;
	l  = (int)yl;

	if(fk == 0.0) { k = k-1; fk = 1.0; }
	if(fl == 0.0) { l = l-1; fl = 1.0; }

	if((k >= 0) && (k < f_im.cols-1) && (l >= 0) && (l < f_im.rows-1)) {
	  for(m=0; m<ch; m++) {
	    double p00, p01, p10, p11, px;
	    p00 = (double)f_im.at<unsigned char>(l,ch*k+m);
	    p01 = (double)f_im.at<unsigned char>(l,ch*(k+1)+m);
	    p10 = (double)f_im.at<unsigned char>(l+1,ch*k+m);
	    p11 = (double)f_im.at<unsigned char>(l+1,ch*(k+1)+m);
	    px  = ((1.0-fl)*((1.0-fk)*p00 + fk*p01) + fl*((1.0-fk)*p10 + fk*p11));
	    p_im.at<unsigned char>(j,ch*i+m) = (unsigned char)round(px); } }
	else {
	  for(m=0; m<ch; m++) {
	    p_im.at<unsigned char>(j,ch*i+m) = 0; } }
      }
    }
  }
  return 0;
}

int fisheye_proj_cylinder(const Mat &f_im, Mat &p_im, const double *f_prm,
			  const double *p_prm, bool nn)
/* --------------------------------------------------------------------------------- */
/*  fisheye_proj_cylinder: projects a fisheye image to a cylindrical plane.          */
/*    f_im:      input fisheye image                                                 */
/*    p_im:      output plane image                                                  */
/*    f_prm[8]:  fisheye camera parameters (k1, k2, k3, k4, fx, fy, cx, cy)          */
/*    p_prm[10]: plane parameters (e1X, e1Y, e1Z, e2X, e2Y, e2Z, as, ae, bs, be)     */
/*    nn:        interpolation. true:nearest neighbor, false:bi-linear               */
/* --------------------------------------------------------------------------------- */
{
  if((f_im.empty()) || (p_im.empty()) ||
     (f_im.type() != CV_8UC1 && f_im.type() != CV_8UC3) ||
     (f_im.type() != p_im.type())) {
    return 1; }

  if(p_prm[8] <= -M_PI/2.0 || p_prm[8] >= M_PI/2.0) return 2;

  int ch;
  if(f_im.type() == CV_8UC3) {
    ch = 3; }
  else {
    ch = 1; }

  double e1[3], e2[3], e3[3];

  double len1 = sqrt(p_prm[0]*p_prm[0] + p_prm[1]*p_prm[1] + p_prm[2]*p_prm[2]);
  if(len1 == 0.0) return 3;

  e1[0] = p_prm[0]/len1;
  e1[1] = p_prm[1]/len1;
  e1[2] = p_prm[2]/len1;

  double cor = e1[0]*p_prm[3] + e1[1]*p_prm[4] + e1[2]*p_prm[5];

  /*  make the angle between e1 and e2 orthogonal */
  e2[0] = p_prm[3] - cor*e1[0];
  e2[1] = p_prm[4] - cor*e1[1];
  e2[2] = p_prm[5] - cor*e1[2];

  double len2 = sqrt(e2[0]*e2[0] + e2[1]*e2[1] + e2[2]*e2[2]);
  if(len2 == 0.0) return 4;

  e2[0] = e2[0]/len2;
  e2[1] = e2[1]/len2;
  e2[2] = e2[2]/len2;

  e3[0] = e1[1]*e2[2] - e1[2]*e2[1];
  e3[1] = e1[2]*e2[0] - e1[0]*e2[2];
  e3[2] = e1[0]*e2[1] - e1[1]*e2[0];

  // double a_step = (p_prm[7]-p_prm[6])/((double)p_im.cols);
  double a_step = (p_prm[7]-p_prm[6])/((double)(p_im.cols-1));

  double c3s, c3e, c3_step;
  c3s = tan(p_prm[8]);
  c3e = tan(p_prm[9]);
  c3_step = (c3e - c3s)/((double)(p_im.rows-1));

  for(int j=0; j<(p_im.rows); j++) {

    double X1, Y1, Z1, c3;
    // c3 = c3_step*(double)j + c3_step/2.0 + c3s;
    c3 = c3_step*(double)j + c3s;
    X1 = c3*e1[0];
    Y1 = c3*e1[1];
    Z1 = c3*e1[2];

    for(int i=0; i<(p_im.cols); i++) {

      double X, Y, Z, alpha, cos_alpha, sin_alpha;
      // alpha = a_step*(double)i + a_step/2.0 + p_prm[6];
      alpha = a_step*(double)i + p_prm[6];
      cos_alpha = cos(alpha);
      sin_alpha = sin(alpha);
      X = X1 + cos_alpha*e2[0] + sin_alpha*e3[0];
      Y = Y1 + cos_alpha*e2[1] + sin_alpha*e3[1];
      Z = Z1 + cos_alpha*e2[2] + sin_alpha*e3[2];

      double Rxy, theta;
      Rxy = sqrt(X*X + Y*Y);
      theta = atan2(Rxy,Z);

      double t2, t4, t6, t8, r;
      t2 = theta*theta;
      t4 = t2*t2;
      t6 = t4*t2;
      t8 = t4*t4;
      r = theta*(1.0 + f_prm[0]*t2 + f_prm[1]*t4 + f_prm[2]*t6 + f_prm[3]*t8);

      double a, b, x, y;
      a = (Rxy == 0.0) ? 1.0 : (X/Rxy);
      b = (Rxy == 0.0) ? 0.0 : (Y/Rxy);
      x = f_prm[4]*r*a;
      y = f_prm[5]*r*b;

      if(nn) { // nearest neighbor

	int k, l, m;
	k = (int)round(x + f_prm[6]);
	l = (int)round(y + f_prm[7]);

	if((k >= 0) && (k < f_im.cols) && (l >= 0) && (l < f_im.rows)) {
	  for(m=0; m<ch; m++) {
	    p_im.at<unsigned char>(j,ch*i+m) = f_im.at<unsigned char>(l,ch*k+m); } }
	else {
	  for(m=0; m<ch; m++) {
	    p_im.at<unsigned char>(j,ch*i+m) = 0; } }

      }
      else { // bi-linear

	int k, l, m;
	double xk, yl, fk, fl;
	xk = floor(x + f_prm[6]);
	yl = floor(y + f_prm[7]);
	fk = (x + f_prm[6]) - xk;
	fl = (y + f_prm[7]) - yl;
	k  = (int)xk;
	l  = (int)yl;

	if(fk == 0.0) { k = k-1; fk = 1.0; }
	if(fl == 0.0) { l = l-1; fl = 1.0; }

	if((k >= 0) && (k < f_im.cols-1) && (l >= 0) && (l < f_im.rows-1)) {
	  for(m=0; m<ch; m++) {
	    double p00, p01, p10, p11, px;
	    p00 = (double)f_im.at<unsigned char>(l,ch*k+m);
	    p01 = (double)f_im.at<unsigned char>(l,ch*(k+1)+m);
	    p10 = (double)f_im.at<unsigned char>(l+1,ch*k+m);
	    p11 = (double)f_im.at<unsigned char>(l+1,ch*(k+1)+m);
	    px  = ((1.0-fl)*((1.0-fk)*p00 + fk*p01) + fl*((1.0-fk)*p10 + fk*p11));
	    p_im.at<unsigned char>(j,ch*i+m) = (unsigned char)round(px); } }
	else {
	  for(m=0; m<ch; m++) {
	    p_im.at<unsigned char>(j,ch*i+m) = 0; } }
      }
    } // for(int i=0; i<(p_im.cols); i++) {
  }   // for(int j=0; j<(p_im.rows); j++) {

  return 0;
}

int fisheye_proj_sphere(const Mat &f_im, Mat &s_im, const double *f_prm,
		        const double *s_prm, bool nn)
/* --------------------------------------------------------------------------------- */
/*  fisheye_proj_sphere: projects a fisheye image to a sphere surface.               */
/*    f_im:      input fisheye image                                                 */
/*    s_im:      output sphere surface image                                         */
/*    f_prm[8]:  fisheye camera parameters (k1, k2, k3, k4, fx, fy, cx, cy)          */
/*    s_prm[10]: surface parameters (e1X, e1Y, e1Z, e2X, e2Y, e2Z, as, ae, bs, be)   */
/*    nn:        interpolation. true:nearest neighbor, false:bi-linear               */
/* --------------------------------------------------------------------------------- */
{
  if((f_im.empty()) || (s_im.empty()) ||
     (f_im.type() != CV_8UC1 && f_im.type() != CV_8UC3) ||
     (f_im.type() != s_im.type())) {
    return 1; }

  int ch;
  if(f_im.type() == CV_8UC3) {
    ch = 3; }
  else {
    ch = 1; }

  double e1[3], e2[3], e3[3];

  double len1 = sqrt(s_prm[0]*s_prm[0] + s_prm[1]*s_prm[1] + s_prm[2]*s_prm[2]);
  if(len1 == 0.0) return 1;

  e1[0] = s_prm[0]/len1;
  e1[1] = s_prm[1]/len1;
  e1[2] = s_prm[2]/len1;

  double cor = e1[0]*s_prm[3] + e1[1]*s_prm[4] + e1[2]*s_prm[5];

  /*  make the angle between e1 and e2 orthogonal */
  e2[0] = s_prm[3] - cor*e1[0];
  e2[1] = s_prm[4] - cor*e1[1];
  e2[2] = s_prm[5] - cor*e1[2];

  double len2 = sqrt(e2[0]*e2[0] + e2[1]*e2[1] + e2[2]*e2[2]);
  if(len2 == 0.0) return 2;

  e2[0] = e2[0]/len2;
  e2[1] = e2[1]/len2;
  e2[2] = e2[2]/len2;

  e3[0] = e1[1]*e2[2] - e1[2]*e2[1];
  e3[1] = e1[2]*e2[0] - e1[0]*e2[2];
  e3[2] = e1[0]*e2[1] - e1[1]*e2[0];

  // double a_step = (s_prm[7]-s_prm[6])/((double)s_im.cols);
  // double b_step = (s_prm[9]-s_prm[8])/((double)s_im.rows);
  double a_step = (s_prm[7]-s_prm[6])/((double)(s_im.cols-1));
  double b_step = (s_prm[9]-s_prm[8])/((double)(s_im.rows-1));

  for(int j=0; j<(s_im.rows); j++) {

    double beta, sin_beta, cos_beta;
    // beta = b_step*(double)j + b_step/2.0 + s_prm[8];
    beta = b_step*(double)j + s_prm[8];
    sin_beta = sin(beta);
    cos_beta = cos(beta);

    for(int i=0; i<(s_im.cols); i++) {

      double alpha, cos_alpha, sin_alpha, c1, c2, c3;
      // alpha = a_step*(double)i + a_step/2.0 + s_prm[6];
      alpha = a_step*(double)i + s_prm[6];
      cos_alpha = cos(alpha);
      sin_alpha = sin(alpha);
      c1 = sin_beta;
      c2 = cos_beta*cos_alpha;
      c3 = cos_beta*sin_alpha;

      double X, Y, Z;
      X = c1*e1[0] + c2*e2[0] + c3*e3[0];
      Y = c1*e1[1] + c2*e2[1] + c3*e3[1];
      Z = c1*e1[2] + c2*e2[2] + c3*e3[2];

      double Rxy, theta;
      Rxy = sqrt(X*X + Y*Y);
      theta = atan2(Rxy,Z);

      double t2, t4, t6, t8, r;
      t2 = theta*theta;
      t4 = t2*t2;
      t6 = t4*t2;
      t8 = t4*t4;
      r = theta*(1.0 + f_prm[0]*t2 + f_prm[1]*t4 + f_prm[2]*t6 + f_prm[3]*t8);

      double a, b, x, y;
      a = (Rxy == 0.0) ? 1.0 : (X/Rxy);
      b = (Rxy == 0.0) ? 0.0 : (Y/Rxy);
      x = f_prm[4]*r*a;
      y = f_prm[5]*r*b;

      if(nn) { // nearest neighbor

	int k, l, m;
	k = (int)round(x + f_prm[6]);
	l = (int)round(y + f_prm[7]);

	if((k >= 0) && (k < f_im.cols) && (l >= 0) && (l < f_im.rows)) {
	  for(m=0; m<ch; m++) {
	    s_im.at<unsigned char>(j,ch*i+m) = f_im.at<unsigned char>(l,ch*k+m); } }
	else {
	  for(m=0; m<ch; m++) {
	    s_im.at<unsigned char>(j,ch*i+m) = 0; } }

      }
      else { // bi-linear

	int k, l, m;
	double xk, yl, fk, fl;
	xk = floor(x + f_prm[6]);
	yl = floor(y + f_prm[7]);
	fk = (x + f_prm[6]) - xk;
	fl = (y + f_prm[7]) - yl;
	k  = (int)xk;
	l  = (int)yl;

	if(fk == 0.0) { k = k-1; fk = 1.0; }
	if(fl == 0.0) { l = l-1; fl = 1.0; }

	if((k >= 0) && (k < f_im.cols-1) && (l >= 0) && (l < f_im.rows-1)) {
	  for(m=0; m<ch; m++) {
	    double p00, p01, p10, p11, px;
	    p00 = (double)f_im.at<unsigned char>(l,ch*k+m);
	    p01 = (double)f_im.at<unsigned char>(l,ch*(k+1)+m);
	    p10 = (double)f_im.at<unsigned char>(l+1,ch*k+m);
	    p11 = (double)f_im.at<unsigned char>(l+1,ch*(k+1)+m);
	    px  = ((1.0-fl)*((1.0-fk)*p00 + fk*p01) + fl*((1.0-fk)*p10 + fk*p11));
	    s_im.at<unsigned char>(j,ch*i+m) = (unsigned char)round(px); } }
	else {
	  for(m=0; m<ch; m++) {
	    s_im.at<unsigned char>(j,ch*i+m) = 0; } }
      }
    } // for(int i=0; i<(s_im.cols); i++) {
  }   // for(int j=0; j<(s_im.rows); j++) {

  return 0;
}
