void compute_dx(double *dx, double *x, double *u, double **p){
    /*Retrieve model parameters*/
    double *m, *a, *b, *Cx, *Cy, *CA;
    m  = p[0];   /* Vehicle mass.                    */
      a  = p[1];   /* Distance from front axle to COG. */
      b  = p[2];   /* Distance from rear axle to COG.  */
      Cx = p[3];   /* Longitudinal tire stiffness.     */
      Cy = p[4];   /* Lateral tire stiffness.          */
      CA = p[5];   /* Air resistance coefficient.      */
      /* x[0]: Longitudinal vehicle velocity. */
      /* x[1]: Lateral vehicle velocity. */
      /* x[2]: Yaw rate. */
      dx[0] = x[1]*x[2]+1/m[0]*(Cx[0]*(u[0]+u[1])*cos(u[4]-2*Cy[0]*(u[4]-(x[1]+a[0]*x[2])/x[0])*sin(u[4])+Cx[0]*(u[2]+u[3])-CA[0]*pow(x[0],2));
      dx[1] = -x[0]*x[2]+1/m[0]*(Cx[0]*(u[0]+u[1])*sin(u[4])+2*Cy[0]*(u[4]-(x[1]+a[0]*x[2])/x[0])*cos(u[4])+2*Cy[0]*(b[0]*x[2]-x[1])/x[0]);
      dx[2] = 1/(pow(((a[0]+b[0])/2),2)*m[0])*(a[0]*(Cx[0]*(u[0]+u[1])*sin(u[4])+2*Cy[0]*(u[4]-(x[1]+a[0]*x[2])/x[0])*cos(u[4]))-2*b[0]*Cy[0]*(b[0]*x[2]-x[1])/x[0]);
}

/* Output equations. */
void compute_y(double *y, double *x, double *u, double **p){
      /* Retrieve model parameters. */
      double *m  = p[0];   /* Vehicle mass.                    */
      double *a  = p[1];   /* Distance from front axle to COG. */
      double *b  = p[2];   /* Distance from rear axle to COG.  */
      double *Cx = p[3];   /* Longitudinal tire stiffness.     */
      double *Cy = p[4];   /* Lateral tire stiffness.          */
      /* y[0]: Longitudinal vehicle velocity. */
      /* y[1]: Lateral vehicle acceleration. */
      /* y[2]: Yaw rate. */
      y[0] = x[0];
      y[1] = 1/m[0]*(Cx[0]*(u[0]+u[1])*sin(u[4])
             +2*Cy[0]*(u[4]-(x[1]+a[0]*x[2])/x[0])*cos(u[4])
             +2*Cy[0]*(b[0]*x[2]-x[1])/x[0]);
      y[2] = x[2];
  }