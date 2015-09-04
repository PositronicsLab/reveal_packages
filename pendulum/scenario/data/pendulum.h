/*-----------------------------------------------------------------------------

-----------------------------------------------------------------------------*/

#ifndef _PENDULUM_H_
#define _PENDULUM_H_

//-----------------------------------------------------------------------------

#include <vector>
#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>
//-----------------------------------------------------------------------------

class pendulum_c;
typedef boost::shared_ptr<pendulum_c> pendulum_ptr;
typedef std::vector< double > state_t;

#ifdef ODEINT_RUNGEKUTTA_STEPPER
#ifdef ODEINT_V1
typedef boost::numeric::odeint::stepper_rk78_fehlberg< state_t, double > stepper_t;
#elif defined( ODEINT_V2 )
typedef boost::numeric::odeint::controlled_runge_kutta< boost::numeric::odeint::runge_kutta_cash_karp54< state_t > > stepper_t;
#endif // ODEINT_VERSION

#elif defined( ODEINT_EULER_STEPPER)
#ifdef ODEINT_V1
typedef boost::numeric::odeint::stepper_euler< state_t, double > stepper_t;
#elif defined( ODEINT_V2 )
typedef boost::numeric::odeint::euler< state_t, double, state_t, double > stepper_t;
#endif // ODEINT_VERSION
#endif // ODEINT_STEPPER

//-----------------------------------------------------------------------------
class pendulum_c {
private:
  double _l;
public:
  double q;
  double dq;

  pendulum_c( double l, double q_0, double dq_0 ) {
    _l = l;
    q = q_0;
    dq = dq_0;
  }

  void operator()( const state_t& x, state_t& dx, const double t ) {
    dx[0] = x[1];
    dx[1] = -9.8 / _l * sin( x[0] );
  }
};

//-----------------------------------------------------------------------------

#endif // _PENDULUM_H_
