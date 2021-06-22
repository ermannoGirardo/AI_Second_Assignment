    /*
     <one line to give the program's name and a brief idea of what it does.>
     Copyright (C) 2015  <copyright holder> <email>
     
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
     */


#include "VisitSolver.h"
#include "ExternalSolver.h"
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>

#include "armadillo"
#include <initializer_list>
#include "cstdlib"
#include "kalman_filter.h"



using namespace std;
using namespace arma;
using Eigen::MatrixXd;
using Eigen::VectorXd;


map <string, vector<double> > region_mapping;

extern "C" ExternalSolver* create_object(){
  return new VisitSolver();
}

extern "C" void destroy_object(ExternalSolver *externalSolver){
  delete externalSolver;
}

VisitSolver::VisitSolver(){

}

VisitSolver::~VisitSolver(){

}

void VisitSolver::loadSolver(string *parameters, int n){
  starting_position = "r0";
  string Paramers = parameters[0];

  char const *x[]={"dummy"};
  char const *y[]={"act-cost","triggered"};
  parseParameters(Paramers);
  affected = list<string>(x,x+1);
  dependencies = list<string>(y,y+2);

  string waypoint_file = "/home/alessio/AI_Second_Assignment/visits/visits_domain/waypoint.txt";
  parseWaypoint(waypoint_file);

  string landmark_file = "/home/alessio/AI_Second_Assignment/visits/visits_domain/landmark.txt";
  parseLandmark(landmark_file);


 

}

map<string,double> VisitSolver::callExternalSolver(map<string,double> initialState,bool isHeuristic){

  map<string, double> toReturn;
  map<string, double>::iterator iSIt = initialState.begin();
  map<string, double>::iterator isEnd = initialState.end();
  double dummy;
  double act_cost;

  map<string, double> trigger;

  for(;iSIt!=isEnd;++iSIt){

    string parameter = iSIt->first;
    string function = iSIt->first;
    double value = iSIt->second;

    function.erase(0,1);
    function.erase(function.length()-1,function.length());
    int n=function.find(" ");

    if(n!=-1){
      string arg=function;
      string tmp = function.substr(n+1,5);

      function.erase(n,function.length()-1);
      arg.erase(0,n+1);
      if(function=="triggered"){
        trigger[arg] = value>0?1:0;
        if (value>0){

       from = tmp.substr(0,2);   // from and to are regions, need to extract wps (poses)
       to = tmp.substr(3,2);


      localize(from, to);

      VisitSolver::startEKF(from, to);

    }
  }
}else{
  if(function=="dummy"){
    dummy = value;

  }else if(function=="act-cost"){
    act_cost = value;
                 } else if(function=="dummy1"){
                    dummy = value;              
                    cout << parameter << " " << value << endl;
                 }
                 }
               }


               double results = calculateExtern(dummy, act_cost);
               if (ExternalSolver::verbose){
                cout << "(dummy) " << results << endl;
              }

              toReturn["(dummy)"] = results;


              return toReturn;
            }

            list<string> VisitSolver::getParameters(){

              return affected;
            }

            list<string> VisitSolver::getDependencies(){

              return dependencies;
            }


            void VisitSolver::parseParameters(string parameters){

              int curr, next;
              string line;
              ifstream parametersFile(parameters.c_str());
              if (parametersFile.is_open()){
                while (getline(parametersFile,line)){
                 curr=line.find(" ");
                 string region_name = line.substr(0,curr).c_str();
                 curr=curr+1;
                 while(true ){
                  next=line.find(" ",curr);
                  region_mapping[region_name].push_back(line.substr(curr,next-curr).c_str());
                  if (next ==-1)
                   break;
                 curr=next+1;

               }                
             }

           }

         }

         double VisitSolver::calculateExtern(double external, double total_cost){
       //float random1 = static_cast <float> (rand())/static_cast <float>(RAND_MAX);
     //  double cost = 20;//random1;
      double cost = distance;
      cost += cov_cost;
      return cost;
     }

     void VisitSolver::parseWaypoint(string waypoint_file){

       int curr, next;
       string line;
       double pose1, pose2, pose3;
       ifstream parametersFile(waypoint_file);
       if (parametersFile.is_open()){
        while (getline(parametersFile,line)){
         curr=line.find("[");
         string waypoint_name = line.substr(0,curr).c_str();

         curr=curr+1;
         next=line.find(",",curr);

         pose1 = (double)atof(line.substr(curr,next-curr).c_str());
         curr=next+1; next=line.find(",",curr);

         pose2 = (double)atof(line.substr(curr,next-curr).c_str());
         curr=next+1; next=line.find("]",curr);

         pose3 = (double)atof(line.substr(curr,next-curr).c_str());

         waypoint[waypoint_name] = vector<double> {pose1, pose2, pose3};
       }
     }

   }

   void VisitSolver::parseLandmark(string landmark_file){

     int curr, next;
     string line;
     double pose1, pose2, pose3;
     ifstream parametersFile(landmark_file);
     if (parametersFile.is_open()){
      while (getline(parametersFile,line)){
       curr=line.find("[");
       string landmark_name = line.substr(0,curr).c_str();
       
       curr=curr+1;
       next=line.find(",",curr);

       pose1 = (double)atof(line.substr(curr,next-curr).c_str());
       curr=next+1; next=line.find(",",curr);

       pose2 = (double)atof(line.substr(curr,next-curr).c_str());
       curr=next+1; next=line.find("]",curr);

       pose3 = (double)atof(line.substr(curr,next-curr).c_str());

       landmark[landmark_name] = vector<double> {pose1, pose2, pose3};
     }
   }
   
 }


  void VisitSolver::localize( string from, string to){

    vector<string> index_from = region_mapping.at(from);
    vector<string> index_to = region_mapping.at(to);

    vector<double> from_co = waypoint.at(index_from[0]);
    vector<double> to_co = waypoint.at(index_to[0]);

    distance = sqrt(pow(from_co[0] - to_co[0], 2) + pow(from_co[1] - to_co[1], 2));

  }
/*
  void VisitSolver::startEKF()
  {
    VectorXd x(3);
    VectorXd alpha1(4);
    alpha1(0) = 0.05 * 0.05;
    alpha1(1) = 0.005 * 0.005;
    alpha1(2) = 0.01 * 0.01;
    alpha1(3) = 0.01 * 0.01;
     // alpha = alpha1;

    MatrixXd P = MatrixXd(3, 3);
    MatrixXd F = MatrixXd(3, 3);
    MatrixXd Q = MatrixXd(3, 3);

    MatrixXd V = MatrixXd(3, 3);
    MatrixXd M = MatrixXd(3, 3);

    x(0) = waypoint[starting_position].at(0);
    x(1) = waypoint[starting_position].at(1);
    x(2) = waypoint[starting_position].at(2);

    P << 0.02, 0, 0,
        0, 0.02, 0,
        0, 0, 0.02; //covariance matrix Q on slides

   double cost_function = P(0,0) + P(1,1) + P(2,2);

    V<< -sin(x(2)), cos(x(2)), 0, 
         cos(x(2)), sin(x(2)), 0,
         1, 0, 1; //jacobian matrix A of slidessssssssssssssssss

    M << alpha1(1), 0, 0,
        0, alpha1(2), 0,
        0, 0, alpha1(1); // errore di misura

    F << 1, 0, -sin(x(2)),
        0, 1, cos(x(2)),
        0, 0, 1;

    Q << V * M * V.transpose();
    //kalman
  }
  */
  
/*
 void VisitSolver::startEKF()
 {
   KalmanFilter kalman;

    kalman.x_(0) = waypoint[starting_position].at(0);
    kalman.x_(1) = waypoint[starting_position].at(1);
    kalman.x_(2) = waypoint[starting_position].at(2);

    VectorXd alpha1(4);
    alpha1(0) = 0.05 * 0.05;
    alpha1(1) = 0.005 * 0.005;
    alpha1(2) = 0.01 * 0.01;
    alpha1(3) = 0.01 * 0.01;

   kalman.F_<<-sin(kalman.x_(2)), cos(kalman.x_(2)), 0, 
         cos(kalman.x_(2)), sin(kalman.x_(2)), 0,
         1, 0, 1;
   
    kalman.P_ << 0.02, 0, 0,
        0, 0.02, 0,
        0, 0, 0.02; //covariance matrix Q on slides

    kalman.H_<<alpha1(1), 0, 0,
        0, alpha1(2), 0,
        0, 0, alpha1(1); // errore di misura

    kalman.Q_ << kalman.F_ * kalman.H_ * kalman.F_.transpose();
   

   kalman.x_ = kalman.F_ * kalman.x_;
   //Update the covariance matrix using the process noise and state transition matrix
   MatrixXd Ft = kalman.F_.transpose();
   kalman.P_ = kalman.F_ * kalman.P_ * Ft + kalman.Q_;

   //Update cost
   cov_cost = kalman.P_(0, 0) + kalman.P_(1, 1) + kalman.P_(2, 2);

   MatrixXd Ht = kalman.H_.transpose();
   MatrixXd PHt = kalman.P_ * Ht;

   VectorXd y = kalman.x_ - kalman.H_ * kalman.x_;

   MatrixXd S = kalman.H_ * PHt + kalman.H_;
   MatrixXd K = PHt * S.inverse();

    //Update state vector
   kalman.x_ = kalman.x_ + (K * y);

   //Update covariance matrix
   long x_size = kalman.x_.size();
   MatrixXd I = MatrixXd::Identity(x_size, x_size);

   kalman.P_ = (I - K * kalman.H_) * kalman.P_;

 }
 */

 void VisitSolver::startEKF(string from, string to)
 {

   MatrixXd F_(3, 3);
   MatrixXd P_(3, 3);
   MatrixXd H_(3, 3);
   MatrixXd Q_(3, 3);
   MatrixXd R_(3, 3);

   VectorXd x_(3);

   vector<string> index_from = region_mapping.at(from);
   vector<double> from_co = waypoint.at(index_from[0]);

   //Initial state vector
    x_(0) = from_co[0];
    x_(1) = from_co[1];
    x_(2) = from_co[2];

   VectorXd alpha1(4);
    alpha1(0) = 0.5 * 0.5;
    alpha1(1) = 0.05 * 0.05;
    alpha1(2) = 0.1 * 0.1;
    alpha1(3) = 0.1 * 0.1;

   KalmanFilter kalman;

//Jacobian matrix 
    H_<<-sin(x_(2)), cos(x_(2)), 0, 
            cos(x_(2)), sin(x_(2)), 0,
            1, 0, 1;

//Initial covariance matrix
    P_ << 0.02, 0, 0,
            0, 0.02, 0,
            0, 0, 0.02; //covariance matrix Q on slides

//Process noise covariance matrix
    R_<<alpha1(1), 0, 0,
            0, alpha1(2), 0,
            0, 0, alpha1(1); // errore di misura

//Transition matrix
    F_<< 1, 0, -sin(x_(2)),
        0, 1, cos(x_(2)),
        0, 0, 1;

  Q_ << H_ * R_ * H_.transpose();

    kalman.Init(x_, P_, F_, H_, R_, Q_);

//Update x_ and P_
    kalman.Predict();

    vector<string> index_to = region_mapping.at(to);
    vector<double> to_co = waypoint.at(index_to[0]);

    VectorXd z(3);

    z(0) = to_co[0];
    z(1) = to_co[1];
    z(2) = to_co[2];

    kalman.Update(z);
    kalman.UpdateEKF(z);

    cov_cost = kalman.P_(0,0) + kalman.P_(1,1) + kalman.P_(2,2);
 }
