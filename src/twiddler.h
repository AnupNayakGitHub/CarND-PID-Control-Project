#ifndef TWIDDLER_H
#define TWIDDLER_H
#include <vector>

class Twiddler {
private:
  std::vector<double> ks;
  std::vector<double> kds;

  unsigned int N;
  double best_err; 
  double sq_sum_err; 
  unsigned int step_cnt;
  unsigned int cur_state;
  bool adjusted_pos;
  bool adjusted_neg;

public:

  unsigned int iterations;
  double tolerance;
  /*
  * Constructor
  */
  Twiddler();

  /*
  * Destructor.
  */
  virtual ~Twiddler();

  bool isStable();
  void insert(double k, double dk);
  void set_steps_per_iteration(unsigned int cnt);
  void reset();
  void twiddle(double err);
  std::vector<double> get_gains() {
    return ks; 
  }
  friend std::ostream& operator <<(std::ostream& os, const Twiddler& twdlr);
};

std::ostream& operator <<(std::ostream& os, const Twiddler& twdlr);

#endif /* TWIDDLER_H */
