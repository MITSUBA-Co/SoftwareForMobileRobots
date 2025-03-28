#include <opencv2/opencv.hpp>

int fisheye_proj_plane(const cv::Mat &f_im, cv::Mat &p_im, const double *f_prm,
                       const double *p_prm, bool nn=false);
/* --------------------------------------------------------------------------------- */
/*  fisheye_proj_plane: projects a fisheye image to a plane.                         */
/*    f_im:      input fisheye image                                                 */
/*    p_im:      output plane image                                                  */
/*    f_prm[8]:  fisheye camera parameters (k1, k2, k3, k4, fx, fy, cx, cy)          */
/*    p_prm[11]: plane parameters (X0, Y0, Z0, e1X, e1Y, e1Z, e2X, e2Y, e2Z, i0, j0) */
/*    nn:        interpolation. true:nearest neighbor, false:bi-linear               */
/* --------------------------------------------------------------------------------- */

int fisheye_proj_cylinder(const cv::Mat &f_im, cv::Mat &p_im, const double *f_prm,
			  const double *p_prm, bool nn=false);
/* --------------------------------------------------------------------------------- */
/*  fisheye_proj_cylinder: projects a fisheye image to a cylindrical plane.          */
/*    f_im:      input fisheye image                                                 */
/*    p_im:      output plane image                                                  */
/*    f_prm[8]:  fisheye camera parameters (k1, k2, k3, k4, fx, fy, cx, cy)          */
/*    p_prm[10]: plane parameters (e1X, e1Y, e1Z, e2X, e2Y, e2Z, as, ae, bs, be)     */
/*    nn:        interpolation. true:nearest neighbor, false:bi-linear               */
/* --------------------------------------------------------------------------------- */

int fisheye_proj_sphere(const cv::Mat &f_im, cv::Mat &s_im, const double *f_prm,
		        const double *s_prm, bool nn=false);
/* --------------------------------------------------------------------------------- */
/*  fisheye_proj_sphere: projects a fisheye image to a sphere surface.               */
/*    f_im:      input fisheye image                                                 */
/*    s_im:      output sphere surface image                                         */
/*    f_prm[8]:  fisheye camera parameters (k1, k2, k3, k4, fx, fy, cx, cy)          */
/*    s_prm[10]: surface parameters (e1X, e1Y, e1Z, e2X, e2Y, e2Z, as, ae, bs, be)   */
/*    nn:        interpolation. true:nearest neighbor, false:bi-linear               */
/* --------------------------------------------------------------------------------- */
