#ifndef PROCEDURES_HPP
#define PROCEDURES_HPP

/*
  This class provides definitions of the different
  functors that are responsible for the various movement
  and feedback patterns that polaris must execute.
 */

namespace proc {
  class ProcedureBase {
  public:
    // Initialize class members and load the movement
    // configuration.
    ProcedureBase()
    {
    }

    // Add arguments around different input sources.
    // Might end up passing a reference to some struct
    // that contains all necessary information.
    // Might want to also pass a time duration, however
    // that might end up causing more implimentation problems.
    virtual int operator()();
  };

  class DiveProcedure : public ProcedureBase {
  public:
    DiveProcedure()
    {
    }

    int operator()()
    {
      return -1;
    }
  };

}

#endif
