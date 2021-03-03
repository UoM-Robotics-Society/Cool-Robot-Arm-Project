#define ARMA_DONT_USE_STD_MUTEX
#include <armadillo>
#include <iostream>
#include <cmath>

double linesearch (arma::dmat p){
    return 7;
}

double Cost(arma::vec::fixed<5> w) {
  return w[0] * w[0] + 1 - w[1] + w[2] * 0 + w[3] * 0 + w[4] * 0;
}


arma::dmat BFGS (){
arma::dmat S(1,5,arma::fill::zeros), P(1,5,arma::fill::zeros), Y(1,5,arma::fill::zeros), Bi(5,5,arma::fill::eye), X(1,5,arma::fill::zeros) ,Xp (1,5,arma::fill::zeros);
    
    double A;

    while ((Cost(X))<0.001){

        X=Xp;
        P= -(Bi)*(Cost(X));
    
        A = linesearch(P);

        S=A*P;
        Xp+=S;

        Y = Cost(Xp) - Cost(X);

        
        Bi += (((S.t()*Y.t() + Y.t()*Bi*Y)*(S*S.t()))/(arma::powmat((S.t()*Y.t()),2)))-(((Bi*Y*S.t())+(S*Y.t()*Bi))/(S.t()*Y));
    }
    return X;
}


int main (){
    BFGS();
    return 0;
}