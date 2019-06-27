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
    enum class ProcedureReturnCode : int {
                                          FATAL = -2,
                                          ERROR = -1,
                                          CONTINUE = 0,
                                          NEXT = 1
    };

    // Initialize class members and load the movement
    // configuration.
    Procedure()
    {
    }

    // Add arguments around different input sources.
    // Might end up passing a reference to some struct
    // that contains all necessary information.
    // Might want to also pass a time duration, however
    // that might end up causing more implimentation problems.
    virtual int operator()();
  };

  class DiveProcedure : public Procedure {
  public:
    DiveProcedure();

    int operator()()
    {
      // Monitor depth of submarine.
      return -1;
    }
  };

}

#endif
