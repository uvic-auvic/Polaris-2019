#ifndef PROCEDURES_HPP
#define PROCEDURES_HPP

/*
  This class provides definitions of the different
  functors that are responsible for the various movement
  and feedback patterns that polaris must execute.
 */

namespace procedures {
  class Procedure {
  public:

    // This is used to  
    enum class ReturnCode : int {
      FATAL = -2,
      ERROR = -1,
      CONTINUE = 0,
      NEXT = 1
    };

    // Initialize class members and load the movement
    // configuration.
    Procedure() = default;

		// Functors require access to sensors.
		// And mechanism to control submarine.   
 
    // Invoking base operator() will result in FATAL
    // return code.
    virtual ProcedureReturnCode operator()() {
			// Sensors to measure actual
			// Compare actual to goal.
			// HMMM this sounds like a PID controller...
			// Could this be provided for convenience in the base 
			// class? or should it be left up to the derived?

			// Adjust movement based on previous computation.

			// Based on computation return code to tell StateMachine
			// what the state should be doing.

			return ProcedureReturnCode::FATAL;
    };

		// This function is responsible for preparing
    // the sub and functor for use.
		virtual void prep() {
			return;
		}

		// This function is repsonsible for unpreparing
    // the sub and functor for use.
		virtual void unprep() {
			return;
		}
  };

  class DiveProcedure : public Procedure {
  public:
    DiveProcedure() = default;

    Procedure::ProcedureReturnCode operator()()
    {
      // Monitor depth of submarine.
      return Procedure::ProcedureReturnCode::FATAL;
    }
  };

}

#endif
