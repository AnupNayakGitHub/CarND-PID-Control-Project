#include "twiddler.h"
#include <assert.h>
#include <limits>
#include <iostream>

using namespace std;

Twiddler::Twiddler() {
  reset();
}
bool Twiddler::isStable() {
  double sum_kds = 0.0;
  for (auto kd : kds) sum_kds += kd;
  //cout << "tolerance " << tolerance << " sum kds " << sum_kds << endl;
  return (tolerance > sum_kds);
}
Twiddler::~Twiddler() {}

void Twiddler::twiddle(double err) {
  ++step_cnt;
  sq_sum_err += err*err;
  cout << "Iteration is " << iterations << " Step count " << step_cnt << endl;
  if (step_cnt%N == 0) {
    step_cnt = 0;
    ++iterations;
    //evaluate adjustments
    if ((adjusted_pos == false) && (adjusted_neg == false)){
      //Make a positive adjustment
      ks[cur_state] += kds[cur_state];
      adjusted_pos = true;
      sq_sum_err = 0.0;
    }
    else if (adjusted_pos == true){
      auto avg_sq_err = sq_sum_err / N;
      sq_sum_err = 0;
      if (avg_sq_err < best_err) {
        //improved
        best_err = avg_sq_err;
        kds[cur_state] *= 1.1;
        cur_state = (++cur_state)%ks.size();
      }
      else {
        cout << "NEGATIVE ADJUSTMENTS!" << endl;
        //Try negative adjustment
        ks[cur_state] -= 2*kds[cur_state];
        adjusted_neg = true;
      }
      adjusted_pos = false;
    }
    else if (adjusted_neg == true){
      auto avg_sq_err = sq_sum_err / N;
      sq_sum_err = 0;
      if (avg_sq_err < best_err) {
        //improved
        best_err = avg_sq_err;
        kds[cur_state] *= 1.1;
      }
      else {
        ks[cur_state] += kds[cur_state];
        kds[cur_state] *= 0.9;
      }
      adjusted_neg = false;
      cur_state = (++cur_state)%ks.size();
    }
  }
}

std::ostream& operator <<(std::ostream& os, const Twiddler& twdlr){
  os << "Best Error : " << twdlr.best_err << endl;

  os << "ks [ ";
  for (auto a : twdlr.ks) os << a << " ";
  os << "]" << endl;
  
  os << "kds[ ";
  for (auto a : twdlr.kds) os << a << " ";
  os << "] tolerance " << twdlr.tolerance;

  return os;
}

void Twiddler::insert(double k, double dk) {
  ks.push_back(k);
  kds.push_back(dk);
}

void Twiddler::reset() {
  N = 1;
  iterations = 0;
  best_err = numeric_limits<double>::max();
  sq_sum_err = 0;
  step_cnt = 0;
  cur_state = 0;
  ks.clear();
  kds.clear();
  adjusted_pos = false;
  adjusted_neg = false;
  cur_state = 0;
  tolerance = 0;
}

void Twiddler::set_steps_per_iteration(unsigned int cnt){
  assert (cnt > 0);
  N = cnt;
}
