/*-----------------------------------------------------------------------------

-----------------------------------------------------------------------------*/

#ifndef _PENDULUM_H_
#define _PENDULUM_H_

//-----------------------------------------------------------------------------

#include <vector>
#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>

//----------------------------------------------------------------------------
// Type definitons

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

//----------------------------------------------------------------------------
class pendulum_c {
private:
  double _l;    //< the length of the pendulum arm
public:
  double q;     //< to store the joint angle of the pivot joint
  double dq;    //< to store the joint velocity of the pivot joint

  /// Parameterized constructor
  /// @param l the length of the pendulum arm
  /// @param q_0 the initial joint angle of the pivot joint
  /// @param dq_0 the initial joint velocity of the pivot joint
  pendulum_c( double l, double q_0, double dq_0 ) {
    _l = l;
    q = q_0;
    dq = dq_0;
  }

  /// The pendulum's ode computation.  Interface for odeint integrators
  /// @param[in] x_0 the state before integration where x_0[0] is position and
  ///            x_0[1] is velocity
  /// @param[out] x_1 the state after integration where x_1[0] is position and 
  ///            x_1[1] is velocity
  /// @param[in] t the virtual time of the integration step
  void operator()( const state_t& x_0, state_t& x_1, const double t ) {
    x_1[0] = x_0[1];
    x_1[1] = -9.8 / _l * sin( x_0[0] );
  }
};

//----------------------------------------------------------------------------

#endif // _PENDULUM_H_
