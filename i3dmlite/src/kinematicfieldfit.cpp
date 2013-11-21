
#include <fstream>
#include <string>
#include <sstream>


#include "kinematicfieldfit.h"



#include "f2c.h"
#include "blaswrap.h"
#include "clapack.h"

#include "float.h"

using namespace std;


namespace i3dm {


// default work size for lapack to use when solving eigenvalue problems
#define KINFITWORKSIZE 200




void getParamsHelper(vector<double> &tofill, vector<double> &M, vector<double> &N, int numparams, bool recomputeErrors) {
    double work[KINFITWORKSIZE];
    double *Mtemp = new double[M.size()], *Ntemp = new double[N.size()];
    for (size_t i = 0; i < M.size(); i++) { Mtemp[i] = M[i]; Ntemp[i] = N[i]; }
    integer itype = 1;
    char jobvl = 'N'; // skip the left eigenvectors
    char jobvr = 'V'; // compute the right eigenvectors
    integer n = numparams;
    // A = Mtemp, will be overwritten
    integer lda = n;
    // B = Ntemp, will be overwritten
    integer ldb = n;
    double *alphar = new double[n], *alphai = new double[n], *beta = new double[n]; // real, imaginary and denom of eigs
    double VL[1]; // dummy var, we don't want left eigvecs
    integer ldvl = 1; // must be >= 1 even though we don't use VL
    double *VR = new double[n*n]; // right eigvecs
    integer ldvr = n;
    integer lwork = KINFITWORKSIZE;
    integer info;
    
    dggev_(&jobvl, &jobvr, &n, Mtemp, &lda, Ntemp, &ldb,
            alphar, alphai, beta, VL, &ldvl, VR, &ldvr, 
            work, &lwork, &info);

    tofill.resize(n);

    
    int mincol = -1;



    if (recomputeErrors) {
        double bestErr = -1;
        for (int col = 0; col < n; col++) {
            double merr = 0, nerr = 0;
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    merr += M[i+n*j]*VR[i+n*col]*VR[j+n*col];
                    nerr += N[i+n*j]*VR[i+n*col]*VR[j+n*col];
                }
            }
            double err = fabs(merr/nerr);
            if (col == 0 || err < bestErr) {
                bestErr = err;
                mincol = col;
            }
        }
    } else {
        double mineig = -1;
        for (int i = 0; i < n; i++) {
            if (fabs(alphai[i]) < 0.00001 && beta[i] > 0) {
                double eigval = fabs(alphar[i])/beta[i];
                if (mincol == -1 || eigval < mineig) {
                    mincol = i;
                    mineig = eigval;
                }
            }
        }
    }


    for (int i = 0; i < n; i++) {
        tofill[i] = VR[i+n*mincol]; 
    }


    delete [] alphar;
    delete [] alphai;
    delete [] beta;
    delete [] VR;
    delete [] Mtemp;
    delete [] Ntemp;
}








} // namespace i3dm

